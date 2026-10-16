"""M4 native Cartesian adapter. Python publishes references, never servo torques."""
from __future__ import annotations
import importlib.metadata
import json
import hashlib
import math
import os
import threading
from contextlib import contextmanager
from pathlib import Path
import time
import numpy as np
from .feedback import EncoderForwardKinematics, _snapshot, _matrix
from ..motion.legacy import _approval, require, vector, sha256
from ..motion.config import _load, TrajectorySpec
from ..motion.trajectories import validate_trajectory
from ..motion.kinematics import UrdfKinematicsProvider, DampedLeastSquaresIK, validate_path_ik, pose_error_base

LIMIT_VECTORS = {'joint_lower': 7, 'joint_upper': 7, 'joint_velocity': 7,
                 'external_torque': 7, 'command_torque': 7, 'workspace_lower': 3, 'workspace_upper': 3}
LIMIT_SCALARS = ('reference_timeout_s', 'callback_gap_s', 'position_error_m', 'orientation_error_rad',
                'linear_speed_m_s', 'angular_speed_rad_s', 'minimum_jacobian_singular_value',
                'minimum_command_success_rate')
SAMPLE_FIELDS = ('tick', 'reference_sequence', 'monotonic_ns', 'robot_time_s', 'dt_s', 'reference_age_s',
                 'min_singular_value', 'q', 'dq', 'torque_measured', 'torque_previous', 'external_torque',
                 'twist', 'reference_twist', 'posture', 'raw', 'task', 'nullspace', 'limit', 'coriolis',
                 'slew', 'filtered_prediction', 'command_success_rate', 'fault', 'stopping',
                 'stop_finished', 'stop_at_rest')
FAULTS = {1:'reference_timeout', 2:'reference_sequence', 3:'callback_gap', 4:'nonfinite', 5:'joint_bounds',
          6:'joint_velocity', 7:'workspace', 8:'pose_error', 9:'cartesian_speed', 10:'external_torque',
          11:'contact_collision', 12:'singularity', 13:'robot_error', 14:'command_success',
          15:'command_torque', 16:'logger_overflow', 17:'controller_projection', 18:'stop_timeout', 19:'joint_reference_limit'}


def scheduling_snapshot():
    return {'tid': threading.get_native_id(),
            'policy': os.sched_getscheduler(0), 'priority': os.sched_getparam(0).sched_priority}


def command_success_threshold(value):
    # rbk_types.h sends this field as float, widened to double by libfranka.
    # Match its representation so a nominal 95% meets a configured 95% minimum.
    return float(np.float32(value))


@contextmanager
def preserve_caller_scheduler(*, realtime=False):
    """Keep libfranka's FIFO promotion out of the Python publisher/logger.

    std::thread inherits its creator's scheduler: promote briefly while spawning
    the native control thread, then restore only the calling Python thread.
    """
    policy, parameter = os.sched_getscheduler(0), os.sched_getparam(0)
    try:
        if realtime:
            os.sched_setscheduler(0, os.SCHED_FIFO,
                                  os.sched_param(os.sched_get_priority_max(os.SCHED_FIFO)))
        yield
    finally:
        os.sched_setscheduler(0, policy, parameter)


def serialize_runtime_sample(sample):
    record = {name: getattr(sample, name) for name in SAMPLE_FIELDS}
    record['stop_fault'] = getattr(sample, 'stop_fault', 0)
    for name in ('joint_reference','joint_velocity_reference','secondary_velocity','outer_correction',
                 'dls_active','dls_rank','dls_velocity_scale'):
        if hasattr(sample,name): record[name]=getattr(sample,name)
    feedback_valid = sample.min_singular_value > 0 and sample.fault not in (4, 17)
    for name in ('pose', 'reference_pose'):
        value = getattr(sample, name).matrix
        record[name] = np.asarray(value() if callable(value) else value).tolist()
    nonfinite = []
    def clean(value):
        if isinstance(value, np.ndarray): value = value.tolist()
        if isinstance(value, (list, tuple)): return [clean(v) for v in value]
        if isinstance(value, float) and not math.isfinite(value): return None
        return value
    for name, value in list(record.items()):
        converted = clean(value)
        # Missing scalar entries preserve evidence of nonfinite samples without invalid JSON.
        if 'null' in json.dumps(converted, allow_nan=False): nonfinite.append(name)
        record[name] = converted
    record['nonfinite_fields'] = nonfinite
    record['ee_feedback_valid'] = feedback_valid
    if not feedback_valid: record['pose'] = None; record['twist'] = None
    record['ee_feedback_source'] = 'measured_joint_encoder_libfranka_fk'
    record['ee_feedback_frame'] = 'fr3_flange'
    record['filtered_prediction_role'] = 'predicted_libfranka_100hz_output; torque_previous is actual prior tau_J_d'
    return record


def _legacy_hardware_config(path, *, execute=False):
    c = _load(Path(path))
    require(c.get('runtime') == 'franky_m4_cartesian_v1', 'unsupported M4 runtime')
    require(c.get('execution_context') == 'hardware', 'M4 requires hardware context')
    _approval(c, execute)
    route=c['controller'].get('kind','cartesian_impedance')
    require(route in ('cartesian_impedance','dls_joint_pd'),'invalid controller kind')
    if route=='dls_joint_pd':
        require(c['task']['kind'] in ('hold','tracking','posture'),'DLS supports hold/tracking/posture only')
        require(c['controller']['projector']=='kinematic','DLS requires kinematic velocity projector')
        d=c['controller']['dls']
        for name,size in (('stiffness',7),('damping',7),('reference_error_limit',7),('pose_gain',6),('task_scale',6)):
            require(all(v>0 for v in vector(d[name],size,name)),f'invalid DLS {name}')
        for name in ('lambda_value','rank_tolerance'):
            require(type(d[name]) in (int,float) and math.isfinite(d[name]) and d[name]>0,f'invalid DLS {name}')
        require(d['rank_tolerance']<1,'invalid DLS rank tolerance')
        require(type(d['posture_gain']) in (int,float) and math.isfinite(d['posture_gain']) and d['posture_gain']>=0,'invalid DLS posture gain')
    for name,default in (('repeats',3),):
        require(type(c.get('experiment',{}).get(name,default)) is int and 1<=c.get('experiment',{}).get(name,default)<=10,'invalid repeats')
    interval=c.get('experiment',{}).get('inter_trial_s',5.)
    require(type(interval) in (int,float) and math.isfinite(interval) and interval>=0,'invalid trial interval')
    require(c['task']['kind'] in ('hold', 'tracking', 'posture', 'z_line', 'yz_circle'), 'unsupported task')
    for name, size in LIMIT_VECTORS.items():
        values = vector(c['limits'][name], size, name)
        if name in ('joint_velocity', 'external_torque', 'command_torque'):
            require(all(v > 0 for v in values), f'invalid {name}')
    for name in LIMIT_SCALARS:
        value = c['limits'][name]
        require(type(value) in (float, int) and math.isfinite(value) and value > 0, f'invalid {name}')
    for name in ('joint', 'workspace'):
        require(np.all(np.array(c['limits'][name+'_lower']) < c['limits'][name+'_upper']), 'unordered bounds')
    require(type(c['limits']['abort_on_contact']) is bool, 'abort_on_contact must be bool')
    require(c['limits']['minimum_command_success_rate'] <= 1, 'invalid success rate')
    if c['task']['kind'] in ('z_line', 'yz_circle'):
        require(type(c['task'].get('fixture_anchor_to_start', False)) is bool,
                'fixture_anchor_to_start must be bool')
        if c['task']['kind']=='yz_circle':
            radius=c['fixture']['radius']
            require(type(radius) in (float,int) and math.isfinite(radius) and radius>0,
                    'circle radius must be positive and finite')
    for name in ('stiffness', 'damping'):
        require(all(v >= 0 for v in vector(c['controller'][name], 6, name)), 'negative gains')
    require(('stop_stiffness' in c['controller'])==('stop_damping' in c['controller']),'both stop gain vectors required')
    for name in ('stop_stiffness','stop_damping'):
        if name in c['controller']:
            require(all(v>0 for v in vector(c['controller'][name],6,name)),'invalid stop gains')
    for name in ('repeat_start_tolerance_rad','repeat_position_tolerance_m','repeat_orientation_tolerance_rad'):
        value=c.get('experiment',{}).get(name,.1)
        require(type(value) in (float,int) and math.isfinite(value) and value>0,'invalid repeat-start tolerance')
    for name in ('posture_stiffness', 'posture_damping'):
        require(math.isfinite(c['controller'][name]) and c['controller'][name] >= 0, 'invalid posture gains')
    require(math.isfinite(c['task']['duration_s']) and 0 < c['task']['duration_s'] <= 600, 'invalid duration')
    require(math.isfinite(c['robot']['max_start_velocity_rad_s']) and c['robot']['max_start_velocity_rad_s'] > 0, 'invalid start velocity')
    require(c['controller']['projector'] in ('kinematic', 'dynamic'), 'invalid projector')
    # Posture comparisons may explicitly choose the original kinematic or M4
    # dynamically consistent torque projector. Other tasks retain their contract.
    if c['task']['kind'] != 'posture' and route=='cartesian_impedance':
        require(c['controller']['projector'] == 'dynamic', 'wrong task projector')
    require(c['controller']['cutoff_hz'] == 100 and c['controller']['max_delta_tau'] == 1, 'preserve tested torque pipeline')
    require(c['reference_hz'] == 50, 'M4 reference rate must be 50 Hz')
    require(c['limits']['reference_timeout_s'] >= 2 / c['reference_hz'], 'watchdog below two publication periods')
    require(type(c['log_capacity']) is int and c['log_capacity'] >= 1024, 'invalid log capacity')
    require(all(type(c['stop'][name]) in (float, int) and math.isfinite(c['stop'][name])
                for name in ('max_duration', 'ramp_duration')), 'nonfinite stop duration')
    require(c['stop']['max_duration'] > c['stop']['ramp_duration'] >= 0, 'invalid stop duration')
    require(all(v > 0 for v in vector(c['stop']['damping'], 7, 'stop damping')), 'invalid stop damping')
    for name in ('velocity_epsilon', 'max_delta_tau'):
        require(math.isfinite(c['stop'][name]) and c['stop'][name] > 0, 'invalid stop threshold')
    if c['task']['kind']=='tracking':
        require(isinstance(c['task']['trajectory'], str), 'trajectory required')
        require(math.isfinite(c['task']['angular_speed_rad_s']) and c['task']['angular_speed_rad_s'] > 0, 'invalid angular speed')
        for name in ('approach_duration_s', 'exit_duration_s', 'settle_duration_s'):
            if name in c['task']:
                value = c['task'][name]
                require(type(value) in (float, int) and math.isfinite(value)
                        and (value >= 0 if name == 'settle_duration_s' else value > 0),
                        f'invalid {name}')
    if c['task']['kind']=='posture':
        task = c['task']
        require(('posture_target_rad' in task) != ('posture_offset_rad' in task),
                'choose absolute posture target or measured-start posture sequence')
        if 'posture_target_rad' in task:
            target = vector(task['posture_target_rad'], 7, 'posture target')
            require(np.all(np.array(target)>=c['limits']['joint_lower']) and
                    np.all(np.array(target)<=c['limits']['joint_upper']), 'posture target outside bounds')
        else:
            # Local hardware excursion is reviewed independently of the bundle's
            # 0.6 rad simulation seed; live IK/bounds/speed checks remain mandatory.
            require(math.isfinite(task['posture_offset_rad']) and task['posture_offset_rad'] > 0,
                    'invalid posture null seed')
            for name in ('posture_hold_s', 'posture_settle_s'):
                require(math.isfinite(task[name]) and task[name]>=0, f'invalid {name}')
            require(type(task['posture_return_to_start']) is bool, 'invalid posture return')
        require(math.isfinite(task['posture_transition_s']) and task['posture_transition_s'] > 0,
                'invalid posture transition')
        if 'posture_offset_rad' in task:
            expected = task['posture_hold_s'] + (3 if task['posture_return_to_start'] else 2)*(
                task['posture_transition_s']+task['posture_settle_s'])
            require(math.isclose(task['duration_s'],expected), 'posture sequence duration mismatch')
    return c


def load_hardware_config(path, *, execute=False):
    c = _legacy_hardware_config(path, execute=execute)
    # Admission hardening only: do not reinterpret booleans as numeric gains,
    # accept nonfinite metadata or defer malformed artifact identities to FCI.
    def inspect(value, key=''):
        if isinstance(value, dict):
            for k, v in value.items(): inspect(v, k)
        elif isinstance(value, list):
            for v in value: inspect(v, key)
        elif type(value) is bool:
            require(key in {'executable', 'motion_authorized', 'abort_on_contact',
                'compensate_coriolis', 'fixture_anchor_to_start', 'posture_return_to_start'},
                f'Boolean is not numeric configuration: {key}')
        elif type(value) is float:
            require(math.isfinite(value), f'Nonfinite configuration: {key}')
    inspect(c)
    digest = c.get('bundle_manifest_sha256')
    require(isinstance(digest, str) and len(digest) == 64 and
            all(ch in '0123456789abcdef' for ch in digest), 'Expected bundle manifest SHA256')
    require(isinstance(c.get('dependencies'), dict) and bool(c['dependencies']), 'Native dependency hashes required')
    for filename, digest in c['dependencies'].items():
        require(isinstance(filename, str) and Path(filename).name == filename and
            isinstance(digest, str) and len(digest) == 64 and all(ch in '0123456789abcdef' for ch in digest),
            'Malformed native dependency identity')
    for name in ('acceleration_m_s2', 'jerk_m_s3'):
        value = c['reference_limits'][name]
        require(type(value) in (int, float) and math.isfinite(value) and value > 0, f'Invalid reference {name}')
    return c


def curve_fixture_values(c, snapshot):
    values=dict(c['fixture'])
    if c['task'].get('fixture_anchor_to_start', False):
        origin=np.array(snapshot.flange_position_m,dtype=float)
        if c['task']['kind']=='yz_circle':
            # Start at the top of a base-frame YZ circle; the circle extends down.
            origin=origin-np.array([0.,0.,values['radius']])
        values['origin']=origin.tolist()
        values['orientation']=snapshot.flange_orientation_xyzw
    return values


def make_curve_fixture(c, f, snapshot):
    if c['task']['kind'] not in ('z_line', 'yz_circle'):
        return None
    fixture = f.CurveFixtureParams()
    fixture.type = f.CurveFixtureType.ZLine if c['task']['kind']=='z_line' else f.CurveFixtureType.YZCircle
    for name, value in curve_fixture_values(c,snapshot).items():
        setattr(fixture, name, value)
    return fixture


def verify_runtime_dependencies(c, f):
    package = Path(f.__file__).parent
    for filename, digest in c['dependencies'].items():
        require(Path(filename).name == filename and sha256(package.parent/'franky_control.libs'/filename)==digest,
                f'dependency mismatch: {filename}')


def connection_protocol_evidence(robot):
    """Call only after successful connection with the checksum-pinned libfranka.

    This Franky Robot binding lacks server_version. libfranka connects using
    protocol 10 and rejects incompatible versions in its native handshake.
    Preserve the distinction between compatible connection and an exposed
    measured server-version value.
    """
    exposed = getattr(robot, 'server_version', None)
    if exposed is not None:
        require(int(exposed) == 10, 'FCI server must be 10')
    return {'client_protocol_version': 10,
            'server_version_from_python_api': int(exposed) if exposed is not None else None,
            'compatibility_verified': True,
            'evidence': 'successful checksum-pinned libfranka 0.21.2 connection handshake',
            'python_server_version_available': exposed is not None}


def verify_native_capabilities(config, module):
    require(importlib.metadata.version('franky-control') == config['robot']['expected_franky_version'],
            'Franky version mismatch')
    verify_runtime_dependencies(config, module)
    pin = json.loads(Path(__file__).with_name('franky-server10.json').read_text())
    require(config['robot']['expected_franky_version'] == pin['franky_version'], 'Unpinned Franky artifact')
    root = Path(module.__file__).parent.parent
    for filename, digest in pin['files'].items():
        require(sha256(root/filename) == digest, f'Native artifact mismatch: {filename}')
    require(hasattr(module, 'M4RuntimeMotion'), 'wheel lacks M4 native runtime')
    require(getattr(module.M4RuntimeMotion, 'stop_behavior_version', 1) >=
            config.get('required_stop_behavior_version', 1), 'wheel lacks required M4 pose-hold stop')
    if config['controller'].get('kind') == 'dls_joint_pd':
        require(hasattr(module, 'M4DlsParams'), 'wheel lacks native DLS comparison route')
    require(getattr(module.M4RuntimeMotion, 'comparison_runtime_version', 0) >=
            config.get('required_comparison_runtime_version', 0), 'wheel lacks comparison runtime version')


class FrankyMotionBackend:
    """One native motion, native watchdog/guards/stop, native bounded 1 kHz log."""
    def __init__(self, config, *, robot=None, franky_module=None):
        # Phase 2 accepts an explicitly supplied backend double/connection only.
        # No vendor imports, Robot constructor, scheduler change or FCI here.
        import copy
        self.config = copy.deepcopy(config)
        if (robot is None) != (franky_module is None):
            raise ValueError('Supply both robot and franky_module, or neither')
        if franky_module is not None:
            verify_native_capabilities(self.config, franky_module)
        self.robot, self.f = robot, franky_module
        self.encoder = EncoderForwardKinematics(robot, franky_module) if robot is not None else None
        self.motion = None
        self.sequence = 0
        self._last_sample = None
        self.scheduling_evidence = {}

    def connect(self):
        raise RuntimeError('Extracted runtime is NOT QUALIFIED; FCI connection is not exposed in phase 2')

    def _require_attached(self):
        require(self.robot is not None and self.f is not None,
                'No explicit Franky connection attached; offline runtime cannot connect')

    def drain_to_authority(self, authority, stream):
        """Non-RT bridge from bounded native telemetry to application receipts."""
        samples = self.drain(stream)
        if self.motion is None:
            return samples
        authority.poll(native_fault=self.motion.fault, dropped_samples=self.motion.dropped_samples)
        for sample in samples:
            if sample.fault:
                authority.poll(native_fault=sample.fault)
            elif not sample.stopping:
                authority.acknowledge(lease_id=authority.lease.lease_id,
                    native_sequence=int(sample.reference_sequence), native_tick=int(sample.tick),
                    timestamp_ns=int(sample.monotonic_ns), clock_id=authority.clock_id)
        return samples

    def finish_authority_stop(self, authority, stream):
        """Join the unchanged native stop, then report evidence; never clear faults."""
        if not authority.stopping:
            raise RuntimeError('Request release/stop through authority before joining')
        smooth = self.stop(stream)
        last = self._last_sample
        evidence = dict(self.stop_evidence,
            joined=bool(self.stop_evidence.get('joined')),
            stop_finished=bool(last is not None and last.stop_finished),
            stop_at_rest=bool(last is not None and last.stop_at_rest),
            dropped_samples=self.motion.dropped_samples if self.motion is not None else 0)
        if self.motion is not None:
            authority.poll(native_fault=self.motion.fault, dropped_samples=self.motion.dropped_samples)
        confirmed = authority.finish_stop(evidence)
        return bool(smooth and confirmed)

    def request_stop(self, reason='operator_stop'):
        if self.motion is not None:
            self.motion.request_stop()

    def send_reference(self, payload):
        self.publish(payload['position_m'], payload['quaternion_xyzw'],
                     payload['twist_linear_angular'], payload.get('posture_rad'))
        return self.sequence

    def preflight(self):
        self._require_attached()
        state = self.robot.state
        s = _snapshot(self.robot, state, self.encoder)
        self.preflight_snapshot = s
        c = self.config
        self.connection_evidence = connection_protocol_evidence(self.robot)
        expected_model = Path(c['bundle'])/'model/robot_returned_model.urdf'
        require(hashlib.sha256(self.robot.model_urdf.encode()).hexdigest() == sha256(expected_model), 'live robot model differs from transfer')
        require(not s.has_errors and not s.is_in_control, 'robot has errors or active controller')
        require(np.allclose(s.f_t_ee, np.eye(4), atol=1e-8, rtol=0), 'requires bare flange F_T_EE identity')
        require(abs(s.load_mass_kg) < 1e-8 and abs(float(state.m_ee)) < 1e-8, 'requires zero configured payload')
        require(np.max(np.abs(s.dq)) <= c['robot']['max_start_velocity_rad_s'], 'robot must be at rest')
        require(np.all(np.abs(state.tau_ext_hat_filtered) <= c['limits']['external_torque']), 'external torque at start exceeds limits')
        require(not np.any(state.joint_collision) and not np.any(state.cartesian_collision), 'collision at start')
        require(s.encoder_fk_reported_position_error_m < 1e-5, 'encoder-model feedback frame mismatch')
        q = np.array(s.q)
        require(np.all(q >= c['limits']['joint_lower']) and np.all(q <= c['limits']['joint_upper']), 'start joints outside bounds')
        p = np.array(s.flange_position_m)
        require(np.all(p >= c['limits']['workspace_lower']) and np.all(p <= c['limits']['workspace_upper']), 'start pose outside workspace')
        j = np.asarray(self.robot.model.zero_jacobian(self.f.Frame.Flange, state))
        require(np.linalg.svd(j, compute_uv=False).min() >= c['limits']['minimum_jacobian_singular_value'], 'start Jacobian singular')
        return s

    def plan(self, snapshot):
        """Construct tracking entry from the measured start; fixtures never auto-enter."""
        c = self.config
        if c['task']['kind']=='yz_circle':
            values=curve_fixture_values(c,snapshot)
            center=np.array(values['origin']);radius=values['radius']
            extent=np.array([0.,radius,radius])
            margin=c['limits']['position_error_m']
            require(np.all(center-extent-margin>=c['limits']['workspace_lower']) and
                    np.all(center+extent+margin<=c['limits']['workspace_upper']),
                    'circle plus normal-error margin outside workspace; reposition or reduce radius')
            model=UrdfKinematicsProvider(Path(c['bundle'])/'model/robot_returned_model.urdf')
            solver=DampedLeastSquaresIK(model,c['limits']['joint_lower'],c['limits']['joint_upper'])
            q=np.asarray(snapshot.q);minimum_sigma=float('inf')
            for angle in np.linspace(0.,2*np.pi,65):
                point=center+radius*np.array([0.,np.sin(angle),np.cos(angle)])
                q=solver.solve(point,values.get('orientation',snapshot.flange_orientation_xyzw),q)
                minimum_sigma=min(minimum_sigma,float(np.linalg.svd(model.evaluate(q)[2],compute_uv=False).min()))
            require(minimum_sigma>=c['limits']['minimum_jacobian_singular_value'], 'circle IK Jacobian singular')
            self.fixture_plan={'type':'base_frame_yz_circle','origin_m':center.tolist(),
                               'radius_m':radius,'orientation_xyzw':list(values.get('orientation',snapshot.flange_orientation_xyzw)),
                               'start_position_m':list(snapshot.flange_position_m),
                               'workspace_margin_m':margin,'ik_sample_count':65,
                               'minimum_jacobian_singular_value':minimum_sigma,
                               'motion_role':'hand guidance only; no automatic trajectory',
                               'collision_clearance_role':'workspace and sampled IK checks do not certify swept-link clearance'}
            return None
        if c['task']['kind'] == 'posture' and 'posture_offset_rad' in c['task']:
            model = UrdfKinematicsProvider(Path(c['bundle'])/'model/robot_returned_model.urdf')
            solver = DampedLeastSquaresIK(model, c['limits']['joint_lower'], c['limits']['joint_upper'])
            q = np.asarray(snapshot.q)
            shared=c.get('experiment',{}).get('posture_plan_path')
            if shared:
                plan=json.loads(Path(shared).read_text())
                require(len(plan['targets_rad'])==2,'shared posture plan requires A/B')
                anchor=np.asarray(vector(plan['anchor_q_rad'],7,'shared posture anchor'))
                goals=[anchor]+[np.asarray(vector(t,7,'shared posture target')) for t in plan['targets_rad']]
                require(np.max(np.abs(q-anchor))<=.15,'shared posture start must be within0.15rad of tested anchor')
                for goal in goals:
                    require(np.all(goal>=c['limits']['joint_lower']) and np.all(goal<=c['limits']['joint_upper']),
                            'shared posture joints outside bounds')
                    position,rotation,j=model.evaluate(goal)
                    error=pose_error_base(position,rotation,plan['fixed_position_m'],plan['fixed_orientation_xyzw'])
                    require(np.linalg.norm(error[:3])<=.001 and np.linalg.norm(error[3:])<=.005,'shared posture FK mismatch')
                    require(np.linalg.svd(j,compute_uv=False).min()>=c['limits']['minimum_jacobian_singular_value'],'shared posture target singular')
                initial_error=pose_error_base(*model.evaluate(q)[:2],plan['fixed_position_m'],plan['fixed_orientation_xyzw'])
                require(np.linalg.norm(initial_error[:3])<c['limits']['position_error_m'] and
                        np.linalg.norm(initial_error[3:])<c['limits']['orientation_error_rad'],'shared posture pose differs from live start')
                sequence=[q,anchor,*goals[1:]]+([anchor] if c['task']['posture_return_to_start'] else [])
                for previous,target in zip(sequence,sequence[1:]):
                    require(np.all(1.875*np.abs(target-previous)/c['task']['posture_transition_s']<=c['limits']['joint_velocity']),
                            'shared posture reference velocity exceeds limits')
                self.posture_plan=dict(plan,source_path=str(Path(shared).resolve()),source_sha256=sha256(Path(shared)))
                return None
            j = model.evaluate(q)[2]
            _, _, vh = np.linalg.svd(j, full_matrices=True)
            targets = [solver.solve(snapshot.flange_position_m, snapshot.flange_orientation_xyzw,
                                    q+sign*c['task']['posture_offset_rad']*vh[-1]) for sign in (-1,1)]
            goals = [q, *targets, *([q] if c['task']['posture_return_to_start'] else [])]
            for previous, target in zip(goals, goals[1:]):
                require(np.all(1.875*np.abs(target-previous)/c['task']['posture_transition_s']
                               <= c['limits']['joint_velocity']), 'posture reference velocity exceeds limits')
                require(np.linalg.svd(model.evaluate(target)[2],compute_uv=False).min()
                        >= c['limits']['minimum_jacobian_singular_value'], 'posture target Jacobian singular')
            self.posture_plan = {'anchor_q_rad':q.tolist(), 'targets_rad':[t.tolist() for t in targets],
                                 'fixed_position_m':list(snapshot.flange_position_m),
                                 'fixed_orientation_xyzw':list(snapshot.flange_orientation_xyzw),
                                 'sequence':'hold -> A -> settle -> B -> settle -> return -> settle'
                                     if c['task']['posture_return_to_start'] else 'hold -> A -> settle -> B -> settle'}
            return None
        if c['task']['kind'] != 'tracking': return None
        bundle = Path(c['bundle'])
        catalog = _load(bundle/'config/trajectories.yaml')['trajectories']
        # Local commissioning timing overrides leave the verified bundle immutable.
        specification = dict(catalog[c['task']['trajectory']])
        for name in ('approach_duration_s', 'exit_duration_s', 'settle_duration_s'):
            if name in c['task']:
                specification[name] = c['task'][name]
        path = TrajectorySpec(c['task']['trajectory'], specification).build(
            c['task']['angular_speed_rad_s'], snapshot.flange_position_m, snapshot.flange_orientation_xyzw)
        validate_trajectory(path, max_speed_m_s=c['limits']['linear_speed_m_s'], max_acceleration_m_s2=c['reference_limits']['acceleration_m_s2'],
                            max_jerk_m_s3=c['reference_limits']['jerk_m_s3'], lower_m=c['limits']['workspace_lower'], upper_m=c['limits']['workspace_upper'])
        model = UrdfKinematicsProvider(bundle/'model/robot_returned_model.urdf')
        solver = DampedLeastSquaresIK(model, c['limits']['joint_lower'], c['limits']['joint_upper'])
        report, _ = validate_path_ik(path, solver, np.array(snapshot.q), sample_hz=5)
        require(report['minimum_jacobian_singular_value_mixed_units'] >= c['limits']['minimum_jacobian_singular_value'], 'planned Jacobian singular')
        return path

    def posture_target(self, elapsed, snapshot):
        task = self.config['task']
        if task['kind'] != 'posture':
            return None
        if 'posture_target_rad' in task:
            return task['posture_target_rad']
        if elapsed < task['posture_hold_s']:
            return self.posture_plan['anchor_q_rad']
        index = int((elapsed-task['posture_hold_s'])/(
            task['posture_transition_s']+task['posture_settle_s']))
        goals = self.posture_plan['targets_rad']
        if task['posture_return_to_start']:
            goals = goals+[self.posture_plan['anchor_q_rad']]
        return goals[min(index,len(goals)-1)]

    def start(self, snapshot):
        self._require_attached()
        _approval(self.config, True)
        f, c = self.f, self.config
        params = f.CartesianImpedanceParams()
        params.stiffness = np.diag(c['controller']['stiffness'])
        params.damping = np.diag(c['controller']['damping'])
        params.nullspace_projector = (f.NullspaceProjectorType.DynamicallyConsistent if c['controller']['projector']=='dynamic'
                                      else f.NullspaceProjectorType.Kinematic)
        params.nullspace_tasks = [f.PostureTask(snapshot.q, c['controller']['posture_stiffness'], c['controller']['posture_damping'])]
        safety = params.safety
        safety.lower_joint_limits = c['limits']['joint_lower']; safety.upper_joint_limits = c['limits']['joint_upper']
        safety.max_delta_tau = 1.; params.safety = safety
        limits = f.M4RuntimeLimits()
        for name in (*LIMIT_VECTORS, *LIMIT_SCALARS, 'abort_on_contact'): setattr(limits, name, c['limits'][name])
        limits.minimum_command_success_rate = command_success_threshold(c['limits']['minimum_command_success_rate'])
        self.effective_limit_evidence = {
            'configured_minimum_command_success_rate': c['limits']['minimum_command_success_rate'],
            'native_minimum_command_success_rate': limits.minimum_command_success_rate,
            'reason': 'Match robot wire float32 representation; equality passes, lower values fault.'}
        stop = f.TorqueStopParams()
        for name, value in c['stop'].items(): setattr(stop, name, value)
        # Native initialization validation runs on the actual first servo state.
        fixture = make_curve_fixture(c, f, snapshot)
        dls=None
        if c['controller'].get('kind')=='dls_joint_pd':
            dls=f.M4DlsParams()
            for name,value in c['controller']['dls'].items(): setattr(dls,name,value)
            dls.validate()
        if 'stop_stiffness' in c['controller']:
            self.motion=f.M4RuntimeMotion(params,limits,stop,fixture,c['log_capacity'],dls,
                np.diag(c['controller']['stop_stiffness']),np.diag(c['controller']['stop_damping']))
        elif dls is None:
            self.motion = f.M4RuntimeMotion(params, limits, stop, fixture, c['log_capacity'])
        else:
            self.motion = f.M4RuntimeMotion(params, limits, stop, fixture, c['log_capacity'],dls)
        self._last_sample=None
        self.stop_evidence={}

        self.publish(snapshot.flange_position_m, snapshot.flange_orientation_xyzw, np.zeros(6))
        previous_threads = {int(p.name) for p in Path('/proc/self/task').iterdir()}
        with preserve_caller_scheduler(realtime=True):
            self.scheduling_evidence['control_thread_creator'] = scheduling_snapshot()
            self.robot.move(self.motion, asynchronous=True, limit_rate=False, cutoff_frequency=100.)
        self.scheduling_evidence['python_after_native_start'] = scheduling_snapshot()
        self.scheduling_evidence['new_native_threads'] = []
        for entry in Path('/proc/self/task').iterdir():
            tid = int(entry.name)
            if tid not in previous_threads:
                try:
                    self.scheduling_evidence['new_native_threads'].append(
                        {'tid':tid, 'policy':os.sched_getscheduler(tid),
                         'priority':os.sched_getparam(tid).sched_priority})
                except ProcessLookupError:
                    pass

    def publish(self, p, q, twist, posture=None):
        self.sequence += 1
        ref = self.f.M4Reference()
        ref.sequence = self.sequence
        ref.cartesian = self.f.CartesianReference(self.f.Affine(p, q), self.f.Twist(twist[:3], twist[3:]))
        if posture is not None:
            ref.posture = posture; ref.has_posture = True
            ref.posture_transition_s = self.config['task']['posture_transition_s']
        self.motion.publish(ref)

    def drain(self, stream):
        result = [] if self.motion is None else self.motion.drain()
        for s in result:
            self._last_sample = s
            record = serialize_runtime_sample(s)
            if s.stopping and getattr(self.f.M4RuntimeMotion, 'stop_behavior_version', 1) >= 2:
                record['stop_joint_damping'] = (-np.asarray(self.config['stop']['damping'])*np.asarray(s.dq)).tolist()
                record['stop_raw_role'] = 'pose_hold_plus_joint_damping_before_transition_blend_and_slew'
            stream.write(json.dumps(record, allow_nan=False)+'\n')
        return result

    def stop(self, stream):
        if self.motion is None: return True
        self.stop_evidence = {'behavior_version': getattr(self.f.M4RuntimeMotion, 'stop_behavior_version', 1)}
        self.motion.request_stop()
        end = time.monotonic() + self.config['stop']['max_duration'] + 2
        joined = False
        try:
            while self.robot.is_in_control and time.monotonic() < end:
                self.drain(stream); time.sleep(.01)
            joined = self.robot.join_motion(.1)
        except Exception as error:
            self.stop_evidence['exception'] = str(error)
        finally:
            self.drain(stream)
        if not joined and self.robot.is_in_control:
            self.stop_evidence['fallback_abort_requested'] = True
            try:
                self.robot.stop() # Explicit abort, not a successful smooth stop.
                self.robot.join_motion(.5)
            except Exception as error:
                self.stop_evidence['fallback_exception'] = str(error)
        self.stop_evidence['stop_fault'] = getattr(self.motion, 'stop_fault', 0)
        self.stop_evidence['joined'] = bool(joined)
        return (bool(joined) and not self.stop_evidence['stop_fault'] and
                self._last_sample is not None and self._last_sample.stop_finished and self._last_sample.stop_at_rest)
