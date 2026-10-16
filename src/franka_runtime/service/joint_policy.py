"""Local joint-policy admission/reference state; no inference or learning imports."""
from copy import deepcopy
from dataclasses import asdict
import hashlib
import math
from pathlib import Path
import queue
import threading
import numpy as np
from contracts.franka_policy import (validate_legacy_fr3, action_spec_from_legacy,
                                    JointPolicySessionSpec, VELOCITY_ID)
from contracts.sessions import payload_sha256
from .authority import SessionAuthority
from ..common.motion.legacy import action_to_target,action_to_velocity,assemble_policy_observation
from ..common.motion.kinematics import UrdfKinematicsProvider
from ..common.motion.reference_types import finite_vector


def admit_bundle(root,manifest_sha256):
    import yaml
    from ..common.artifacts.bundle import verify_motion_bundle
    root=Path(root).resolve(strict=True)
    verify_motion_bundle(root,manifest_sha256,allow_legacy_cache=True)
    c=yaml.safe_load((root/'policy_contract.yaml').read_text())
    validate_legacy_fr3(c)
    return c


class JointPolicyBackend:
    """Reference state plus optional attached native adapter; static offline state.

    Native JointReference has no sequence acknowledgement. Native publications
    therefore never fabricate application receipts; mock ticks are labelled.
    """
    def __init__(self,contract,profile,state,model_path,model_sha256,*,mode='mock',attached=None,clock):
        validate_legacy_fr3(contract)
        if mode not in ('mock','shadow','attached'):raise ValueError('Unsupported local joint mode')
        if (mode=='attached')!=(attached is not None):raise ValueError('Attached mode requires an explicit pre-authorized adapter')
        self.contract=deepcopy(contract);self.config=deepcopy(profile)
        expected='franky_joint_velocity_impedance_tracking_v1' if contract['contract_id']==VELOCITY_ID else 'franky_joint_impedance_tracking_v1'
        if profile['runtime']!=expected:raise ValueError('Policy/native route mismatch')
        if profile['timing']['command_hz']!=50:raise ValueError('PPO requires 50 Hz')
        if (profile['safety']['soft_lower_rad']!=contract['action']['soft_lower_rad'] or
            profile['safety']['soft_upper_rad']!=contract['action']['soft_upper_rad']):raise ValueError('Policy/profile reference bounds differ')
        imp=profile['robot']['impedance_controller']
        if (imp['stiffness_nm_rad']!=[100.]*7 or imp['damping_nms_rad']!=[20.]*7):
            raise ValueError('Phase 3 requires the nominal policy controller; overrides need separate qualification')
        if (imp['expected_error_clip_rad'] != [.5]*7 or imp['max_delta_tau_nm_per_ms'] != 1. or
                imp['command_filter_cutoff_hz'] != 100. or imp['compensate_coriolis'] is not True):
            raise ValueError('Profile/controller processing differs from policy contract')
        for name in ('state_timeout_ms','callback_gap_ms','command_watchdog_ms','inference_timeout_ms'):
            if not math.isfinite(float(profile['timing'][name])) or profile['timing'][name] <= 0:
                raise ValueError('Positive finite timing limits required')
        for name in ('max_start_velocity_rad_s','max_tracking_error_rad','start_tolerance_rad'):
            if any(v <= 0 for v in finite_vector(profile['safety'][name],7,name)):
                raise ValueError('Positive finite joint guards required')
        self.mode,self.attached,self.clock=mode,attached,clock
        self._callback_fault=None;self._last_callback_ns=None;self._first_command_ns=None
        self._last_callback_robot_s=None;self._fractional_ns=0
        path=Path(model_path).resolve(strict=True)
        if hashlib.sha256(path.read_bytes()).hexdigest()!=model_sha256:raise ValueError('Encoder FK model hash mismatch')
        self.model=UrdfKinematicsProvider(path);self.state=deepcopy(state)
        self.reference=None;self.previous_action=[0.]*6;self.velocity=[0.]*7
        self.sequence=0;self.tick=0;self.active=False;self._last_time=None
        self.telemetry=queue.Queue(maxsize=4096);self.telemetry_dropped=0
        self._stop_thread=None;self._stop_joined=False;self._stop_error=None
        self.mapping={'action_mapping':{'type':contract['action']['type'],'policy_joint_count':6,
                      **{k:v for k,v in contract['action'].items() if k in ('max_position_increment_rad','max_reference_velocity_rad_s')}},
                      'safety':{'soft_lower_rad':contract['action']['soft_lower_rad'],'soft_upper_rad':contract['action']['soft_upper_rad']}}
        self.last_publication=None

    def preflight(self):
        if self.attached is not None:
            s=self._read_attached();q=s.q;dq=s.dq
            if s.has_errors or s.is_in_control or s.mode not in ('Idle','Move'):raise ValueError('Attached robot not ready')
            if not np.allclose(s.f_t_ee,np.eye(4),rtol=0,atol=1e-8) or abs(s.load_mass_kg)>1e-8:raise ValueError('Bare flange/zero load required')
            if s.encoder_fk_reported_position_error_m>=1e-5:raise ValueError('Encoder FK mapping mismatch')
        else:
            q=finite_vector(self.state['q'],7,'q');dq=finite_vector(self.state.get('dq',[0.]*7),7,'dq')
        if any(not lo<=v<=hi for v,lo,hi in zip(q,self.contract['action']['soft_lower_rad'],self.contract['action']['soft_upper_rad'])):raise ValueError('Start outside joint bounds')
        if any(abs(v)>limit for v,limit in zip(dq,self.config['safety']['max_start_velocity_rad_s'])):raise ValueError('Start velocity exceeds reviewed limits')
        start=finite_vector(self.config['policy_start_position_rad'],7,'policy start position')
        if any(abs(v-r)>limit for v,r,limit in zip(q,start,self.config['safety']['start_tolerance_rad'])):
            raise ValueError('Start joint position exceeds reviewed tolerance; no automatic homing')
        self.reference=list(q);self.held_joint7=float(q[6]);self.previous_action=[0.]*6;self.velocity=[0.]*7;self._last_time=self.clock()
        return self.observe()

    def plan(self,snapshot):pass

    def start(self,snapshot):
        if self.active:raise RuntimeError('Joint session already active')
        self.active=True
        if self.attached is not None and self.contract['contract_id']==VELOCITY_ID:self.attached.synchronize_reference(self.reference)

    def observe(self):
        now=self.clock()
        if self.reference is not None and self.attached is None and self.mode=='mock' and self._last_time is not None:
            elapsed_ns=max(0,now-self._last_time)+self._fractional_ns
            steps,self._fractional_ns=divmod(elapsed_ns,1_000_000)
            # Logical integration only; reproduces native reference timing, not dynamics.
            if self.contract['contract_id']==VELOCITY_ID:
                for _ in range(steps):
                    self.reference=[min(hi,max(lo,q+v*.001)) for q,v,lo,hi in zip(self.reference,self.velocity,self.contract['action']['soft_lower_rad'],self.contract['action']['soft_upper_rad'])]
        self._last_time=now
        if self.attached is not None:
            s=self._read_attached();q=list(s.q);dq=list(s.dq);p=list(s.flange_position_m);orientation=list(s.flange_orientation_xyzw)
            if s.has_errors:raise RuntimeError('robot_error')
            if self.contract['contract_id']==VELOCITY_ID and self.reference is not None:self.reference=list(self.attached.current_reference())
            source=s.ee_feedback_source
        else:
            q=list(self.state['q']);dq=list(self.state.get('dq',[0.]*7));p,orientation,_=self.model.evaluate(q);p=p.tolist();orientation=orientation.tolist();source='offline_joint_state_urdf_fk'
        return {'timestamp_ns':now,'q':q,'dq':dq,'position_m':p,'quaternion_xyzw':orientation,
                'q_reference':list(self.reference) if self.reference is not None else q[:],
                'previous_action':self.previous_action[:],'feedback_source':source,
                'base_frame':'fr3_link0','tcp_frame':'fr3_flange','hardware_executed':self.mode=='attached'}

    def observation(self,target):
        s=self.observe()
        values=assemble_policy_observation(s['q'],s['dq'],target,s['position_m'],s['q_reference'],self.previous_action,
              self.contract['default_joint_position_rad'],self.contract['action']['soft_lower_rad'],self.contract['action']['soft_upper_rad'],6)
        return s,values

    def validate_payload(self,payload):
        if set(payload)!={'action','contract_id'} or payload['contract_id']!=self.contract['contract_id']:raise ValueError('Wrong joint capability payload')
        action_spec_from_legacy(self.contract).validate_values(payload['action'])

    def _read_attached(self):
        started=self.clock();snapshot=self.attached.read_state()
        if self.clock()-started > self.config['timing']['state_timeout_ms']*1e6:
            raise RuntimeError('state_timeout')
        if self._last_callback_robot_s is not None and snapshot.robot_time_s-self._last_callback_robot_s > self.config['timing']['callback_gap_ms']/1000:
            self._callback_fault='callback_queue_lag'
        return snapshot

    def poll_health(self,authority):
        if self.telemetry_dropped:authority.poll(dropped_samples=self.telemetry_dropped)
        reference=self._last_callback_ns or self._first_command_ns
        if self.attached is not None and self.active and reference is not None:
            if self.clock()-reference > self.config['timing']['callback_gap_ms']*1e6:
                self._callback_fault='callback_gap'
        if self._callback_fault:authority.poll(native_fault=self._callback_fault)
        if authority.fault:raise RuntimeError(authority.fault.code)

    def drain_telemetry(self):
        result=[]
        for _ in range(self.telemetry.maxsize):
            try:result.append(asdict(self.telemetry.get_nowait()))
            except queue.Empty:break
        return result

    def _callback(self,record):
        # Bounded work only: guards plus enqueue. Serialization/disk stay off callback.
        self._last_callback_ns=record.host_monotonic_ns
        self._last_callback_robot_s=record.robot_time_s
        if any(abs(q-r)>limit for q,r,limit in zip(record.q,record.q_command,self.config['safety']['max_tracking_error_rad'])):
            self._callback_fault='tracking_error'
        try:self.telemetry.put_nowait(record)
        except queue.Full:self.telemetry_dropped+=1

    def send_reference(self,payload):
        self.validate_payload(payload)
        if not self.active:raise RuntimeError('Joint session not active')
        action=[float(v) for v in payload['action']]
        if self._first_command_ns is None:self._first_command_ns=self.clock()
        if self.contract['contract_id']==VELOCITY_ID:
            velocity=action_to_velocity(self.mapping,action)
            if velocity[6]!=0.:raise RuntimeError('Held joint received nonzero velocity')
            if self.attached is not None:self.attached.send_velocity(velocity,self._callback)
            elif self.mode=='shadow':
                self.reference=[min(hi,max(lo,q+v*.02)) for q,v,lo,hi in zip(self.reference,velocity,self.contract['action']['soft_lower_rad'],self.contract['action']['soft_upper_rad'])]
            self.velocity=velocity
        else:
            target=action_to_target(self.mapping,action,self.reference)
            if self.attached is not None:
                if target==self.reference and self.sequence:self.attached.keepalive()
                else:self.attached.send_target(target,self._callback)
            self.reference=target
        if abs(self.reference[6]-self.held_joint7)>1e-10:raise RuntimeError('Held joint reference drift')
        self.sequence+=1;self.previous_action=action
        self.last_publication={'sequence':self.sequence,'reference':self.reference[:],'velocity':self.velocity[:],'raw_action':action,'timestamp_ns':self.clock()}
        return self.sequence

    def consume(self):
        if self.mode!='mock' or self.last_publication is None:return None
        self.tick+=1
        return {'native_sequence':self.sequence,'native_tick':self.tick,'timestamp_ns':self.clock()}

    def request_stop(self,reason='stop'):
        self.active=False;self.velocity=[0.]*7
        if self.attached is not None and self._stop_thread is None:
            def stop():
                try:self._stop_joined=bool(self.attached.smooth_stop())
                except Exception as error:self._stop_error=str(error)
            self._stop_thread=threading.Thread(target=stop,name='joint-policy-stop',daemon=True);self._stop_thread.start()

    def stop_evidence(self):
        if self._stop_thread is not None:self._stop_thread.join(timeout=6.)
        if self.attached is None:return dict(joined=True,stop_finished=True,stop_at_rest=True,stop_fault=0,dropped_samples=self.telemetry_dropped,evidence_source='offline_logical_only')
        rest=False
        if self._stop_joined and not self._stop_error:
            try:rest=max(abs(float(v)) for v in self.attached.robot.state.dq)<=self.config['robot']['torque_stop']['velocity_epsilon_rad_s']
            except Exception:pass
        return dict(joined=self._stop_joined,stop_finished=self._stop_joined,stop_at_rest=rest,
                    stop_fault=0 if self._stop_joined and not self._stop_error else 1,dropped_samples=self.telemetry_dropped,
                    evidence_source='legacy_stop_join_and_post_stop_encoder_speed')


def create_joint_session(backend,manifest_sha256,*,robot_id,clock_id,clock=None):
    spec=JointPolicySessionSpec(robot_id,backend.contract['contract_id'],manifest_sha256,payload_sha256(backend.config),action_spec_from_legacy(backend.contract))
    return SessionAuthority(backend,spec,backend.validate_payload,clock_id=clock_id,clock=clock,
           command_timeout_ns=int(backend.config['timing']['command_watchdog_ms']*1e6),
           expect_application=backend.mode=='mock',capability_prefix='joint-ppo-v1')
