"""Explicit physical-device assembly; importing this module has no device effects.

Not a service entry point. Identity and collision verification implementations
must be supplied by commissioning; no IP-based identity or point-only collision
fallback. Device setup authorization includes Robot constructor collision writes.
"""
import math
from .concurrent_hand import ConcurrentHandWorker
from .physical_config import PhysicalDroidConfig
from .native import FrankyArmDriver, NativeDroidBackend, verify_native_artifact
from .workspace import from_settings
from .checks import check_rows
from contracts.sessions import payload_sha256


def build_motion(sdk, config):
    """Construct only native parameters/motion, never Robot/Gripper or a thread."""
    d=config.validated();c=d['controller'];g=d['guards'];b=d['joint_bounds']
    p=sdk.JointImpedanceParams()
    p.stiffness=c['joint_stiffness'];p.damping=c['joint_damping'];p.error_clip=c['error_clip_rad']
    p.constant_torque_offset=[0.]*7;p.compensate_coriolis=c['compensate_coriolis']
    p.cartesian_gains=sdk.CartesianImpedanceGains.diagonal(c['cartesian_stiffness'],c['cartesian_damping'])
    # No added friction compensation or limit-repulsion controller in this law.
    safety=p.safety;safety.max_delta_tau=c['max_delta_nm_per_callback'];p.safety=safety
    limits=sdk.DroidJointLimits()
    limits.lower=[q+m for q,m in zip(b['lower_rad'],b['margin_rad'])]
    limits.upper=[q-m for q,m in zip(b['upper_rad'],b['margin_rad'])]
    limits.velocity=g['max_measured_joint_velocity_rad_s'];limits.command_torque=g['max_command_torque_nm']
    limits.publisher_timeout_ns=g['publisher_timeout_ns'];limits.callback_gap_ns=g['max_callback_gap_ns']
    stop=sdk.TorqueStopParams();s=d['stop']
    for field,key in [('damping','damping'),('ramp_duration','ramp_duration_s'),('velocity_epsilon','rest_velocity_rad_s'),
                      ('max_duration','max_duration_s'),('compensate_coriolis','compensate_coriolis'),('max_delta_tau','max_delta_nm_per_callback')]:
        setattr(stop,field,s[key])
    return sdk.DroidJointImpedanceMotion(p,limits,stop,d['timing']['native_log_capacity'],c['gain_time_constant_s'])


class PhysicalDeviceFactory:
    def __init__(self,config,*,clock,clock_id,identity_reader,collision_checker,artifact_verifier=verify_native_artifact):
        self.config,self.clock,self.clock_id=config,clock,clock_id
        self.identity_reader,self.collision_checker=identity_reader,collision_checker
        self.artifact_verifier=artifact_verifier

    def _identity(self,d):
        r=d['robot'];v=self.identity_reader(r['address']);now=self.clock()
        for key in ('address','model','firmware_version','fci_version','arm_serial','control_serial'):
            if v.get(key)!=r[key]:raise ValueError('Physical robot identity mismatch: '+key)
        if v.get('clock_id')!=self.clock_id or type(v.get('observed_ns')) is not int or not 0<=now-v['observed_ns']<=r['identity_max_age_ns']:
            raise ValueError('Physical identity evidence is stale or uses a different clock')
        return dict(v)

    def connect(self,executor,*,spec,authorize_device_setup=False,recorder=None):
        if authorize_device_setup is not True:
            raise ValueError('Explicit device setup authorization required (constructor sets collision thresholds)')
        d=self.config.require_approved()
        admitted=PhysicalDroidConfig(d,self.config.source,self.config.config_dir)
        if (payload_sha256(spec) not in d['contract']['allowed_sha256']
                or spec.robot.robot_id != d['robot']['robot_id']
                or list(spec.robot.joint_names) != d['workspace']['joint_names']
                or spec.gripper.open_width_m != d['gripper']['open_width_m']):
            raise ValueError('Commissioned contract, unit, joint order or hand mapping mismatch')
        checker=self.collision_checker
        if checker is None or checker.artifact_sha256!=d['artifacts']['collision_model_sha256']:
            raise ValueError('Commissioned collision checker/model required')
        workspace=from_settings(d['workspace'],self.config.config_dir)
        if list(workspace.chain.joint_names)!=d['workspace']['joint_names']:
            raise ValueError('URDF physical joint ordering mismatch')
        def record(kind,payload):
            if recorder is not None and not recorder.emit(kind,payload,timestamp_ns=self.clock()):
                raise RuntimeError('Device evidence recording unavailable')
        sdk=self.artifact_verifier(self.config.path(d['artifacts']['native_pin']))
        record('device_identity',self._identity(d))
        r=d['robot']
        robot=sdk.Robot(r['address'],default_torque_threshold=r['collision_torque_nm'],
                        default_force_threshold=r['collision_force_n'],realtime_config=sdk.RealtimeConfig.Enforce)

        def read_arm():
            record('device_identity',self._identity(d))
            state=robot.state
            if state.robot_mode!=sdk.RobotMode.Idle or any(getattr(state.current_errors,name) is True for name in dir(state.current_errors) if not name.startswith('_')):
                raise ValueError('Idle, error-free robot required; no automatic recovery')
            q,dq=tuple(state.q),tuple(state.dq)
            if len(q)!=7 or len(dq)!=7 or not all(math.isfinite(x) for x in (*q,*dq)):
                raise ValueError('Invalid encoder snapshot')
            b=d['joint_bounds']
            if any(not lo+m<=x<=hi-m for x,lo,hi,m in zip(q,b['lower_rad'],b['upper_rad'],b['margin_rad'])):
                raise ValueError('Measured joints outside commissioned bounds')
            if any(abs(v)>limit for v,limit in zip(dq,d['guards']['max_measured_joint_velocity_rad_s'])):
                raise ValueError('Measured joint velocity exceeds guard')
            t=d['tool']
            for key in ('m_ee','m_load','I_ee','I_load','F_x_Cee','F_x_Cload'):
                actual=getattr(state,key);expected=t[key]
                a=[actual] if isinstance(expected,(int,float)) else list(actual)
                e=[expected] if isinstance(expected,(int,float)) else expected
                if len(a)!=len(e) or any(not math.isfinite(x) or abs(x-y)>t['scalar_tolerance'] for x,y in zip(a,e)):
                    raise ValueError('Configured tool/load mismatch: '+key)
            actual=state.F_T_EE.matrix
            if any(not math.isfinite(float(actual[i][j])) or abs(float(actual[i][j])-t['F_T_EE'][i][j])>t['transform_tolerance'] for i in range(4) for j in range(4)):
                raise ValueError('Configured TCP mismatch')
            workspace.check_configuration(q,'measured state');checker.check_configuration(q)
            stamp=self.clock()
            record('device_preflight',dict(q=list(q),dq=list(dq),measured_ns=stamp,tool_verified=True,
                                         workspace_verified=True,collision_checked=True))
            return q,dq,stamp

        q,dq,stamp=read_arm();gripper=sdk.Gripper(r['address']);g=d['gripper']
        def read_hand():
            s=gripper.state
            return dict(width=float(s.width),moving=False,grasped=bool(s.is_grasped),message_id=int(s.time.to_msec()))
        def perform(action):
            if action=='open':return gripper.move(g['open_width_m'],g['speed_m_s'])
            return gripper.grasp(g['close_width_m'],g['speed_m_s'],g['grasp_force_n'],g['epsilon_inner_m'],g['epsilon_outer_m'])
        hand=ConcurrentHandWorker(perform,read_hand,gripper.stop,initial_state=read_hand(),clock=self.clock,
            poll_interval_s=1/g['poll_hz'],max_state_age_ns=g['max_state_age_ns'],moving_epsilon_m_s=g['moving_epsilon_m_s'],
            cancel_interval_s=g['cancel_interval_ms']/1000,cancel_attempts=g['cancel_attempts'],history_capacity=g['history_capacity'],
            cancelled_exception=lambda e:isinstance(e,sdk.CommandException) and 'Command aborted!' in str(e))
        driver=FrankyArmDriver(robot,lambda:build_motion(sdk,admitted),q,initial_dq=dq,initial_timestamp_ns=stamp,
            refresh=read_arm,limit_rate=d['controller']['libfranka_limit_rate'],cutoff_hz=d['controller']['lowpass_cutoff_hz'])
        backend=NativeDroidBackend(executor,driver,hand,clock=self.clock,
            **{key:d['timing'][key] for key in ('startup_timeout_ns','consumption_timeout_ns','stop_timeout_ns','state_timeout_ns')})
        def validate_targets(rows):
            check_rows(rows,backend.arm.q,dict(lower=d['joint_bounds']['lower_rad'],upper=d['joint_bounds']['upper_rad'],
                margin=d['joint_bounds']['margin_rad'],max_velocity=d['acceptance']['max_consecutive_row_speed_rad_s'],
                max_target_lead=d['acceptance']['max_target_lead_rad']),round(1e9/spec.action.policy_hz))
            workspace.check_rows(rows,backend.arm.q)
            previous=backend.arm.q
            for row in rows:
                target=tuple(row[:7])
                for i in range(1,d['workspace']['interpolation_steps']+1):
                    checker.check_configuration(tuple(a+(b-a)*i/d['workspace']['interpolation_steps'] for a,b in zip(previous,target)))
                previous=target
        backend.validate_targets=validate_targets
        backend.configuration_sha256=admitted.sha256
        return backend
