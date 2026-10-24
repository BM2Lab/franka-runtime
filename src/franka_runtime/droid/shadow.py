"""D7 read-only backend. Virtual rows never become physical execution receipts."""
from dataclasses import asdict
import math
from types import SimpleNamespace
from contracts.droid import DroidActionChunk
from .backend import BackendUpdate

# A homed Franka Hand reports a maximum width slightly below the nominal 0.08 m contract
# opening (0.07998 m measured 2026-10-24); an unhomed or unconfigured Hand reports ~0.
HAND_OPEN_WIDTH_TOLERANCE_M=0.001


class ShadowHand:
    commands=()
    def __init__(self):self.width=0.;self.grasped=self.moving=False;self.timestamp_ns=0
    def command(self,*args):raise RuntimeError('Shadow cannot send Hand commands')
    def snapshot(self):return dict(width=self.width,grasped=self.grasped,moving=self.moving,
        measured_timestamp_ns=self.timestamp_ns,moving_source='measured_width_delta',commands_enabled=False)


class ShadowDroidBackend:
    name='shadow_libfranka_readonly_v1'
    shadow=True
    expect_application=False
    hardware_executed=False
    hardware_accessed=True
    single_session=True
    execution=(None,None) # Never claim physical/native execution of virtual targets.

    def __init__(self,executor,source,*,clock,settings,recorder):
        self.executor,self.source,self.clock,self.settings,self.recorder=executor,source,clock,settings,recorder
        self.arm=SimpleNamespace(q=(),dq=());self.hand=ShadowHand()
        self.estimate=None;self._sequence=1;self._session_started=False;self._stopping=False
        self._reported=False;self._fault=0;self._hand_pair=None;self.feedback_info={}
        self.refresh()

    def refresh(self):
        s=self.source.snapshot()
        q,dq=tuple(s['q']),tuple(s['dq'])
        if len(q)!=7 or len(dq)!=7 or any(isinstance(x,bool) or not isinstance(x,(float,int)) or not math.isfinite(x) for x in (*q,*dq)):
            raise ValueError('Invalid measured joints; retain last finite snapshot as invalid')
        if any(isinstance(s[k],bool) or not isinstance(s[k],(int,float)) or not math.isfinite(s[k]) for k in ('width','max_width')):
            raise ValueError('Invalid measured Hand state')
        self.arm.q,self.arm.dq=q,dq
        pair=(s['hand_ns'],s['width'])
        if self._hand_pair is not None and pair[0]>self._hand_pair[0]:
            self.hand.moving=abs(pair[1]-self._hand_pair[1])/((pair[0]-self._hand_pair[0])/1e9)>self.settings['moving_epsilon_m_s']
        self._hand_pair=pair
        self.hand.width,self.hand.grasped,self.hand.timestamp_ns=s['width'],s['grasped'],s['hand_ns']
        self._sample=s
        self.feedback_info=dict(arm_ns=s['arm_ns'],hand_ns=s['hand_ns'],arm_counter_ms=s['arm_counter_ms'],
            hand_counter_ms=s['hand_counter_ms'],geometry_scope='point_workspace_only_no_self_collision')

    def validate_state(self):
        try:
            self.refresh();s=self._sample;now=self.clock()
            if self.source.fault:raise ValueError('Read-only state source failed')
            if s.get('schema_version')!=1 or s['server_version']!=10:raise ValueError('Unexpected FCI state schema/family')
            if any(not 0<=now-s[k]<=limit for k,limit in [('arm_ns',self.settings['state_timeout_ns']),('hand_ns',self.settings['hand_state_timeout_ns'])]):
                raise ValueError('Stale/future read-only encoder or Hand feedback')
            if len(self.arm.q)!=7 or len(self.arm.dq)!=7 or any(isinstance(x,bool) or not isinstance(x,(float,int)) or not math.isfinite(x) for x in (*self.arm.q,*self.arm.dq)):
                raise ValueError('Invalid measured joints')
            if type(s['robot_mode']) is not int or s['robot_mode']!=1 or s['errors'] is not False or s['contact'] is not False:
                raise ValueError('Shadow requires Idle, error-free, contact-free robot feedback')
            if any(not lo+m<=q<=hi-m for q,lo,hi,m in zip(self.arm.q,self.settings['lower'],self.settings['upper'],self.settings['margin'])):
                raise ValueError('Measured joints outside candidate soft bounds')
            if any(abs(v)>limit for v,limit in zip(self.arm.dq,self.settings['measured_velocity'])):
                raise ValueError('Measured joint-speed guard exceeded')
            if (isinstance(s['width'],bool) or not math.isfinite(s['width']) or not 0<=s['width']<=s['max_width']
                    or not math.isfinite(s['max_width']) or s['max_width']<self.settings['open_width_m']-HAND_OPEN_WIDTH_TOLERANCE_M
                    or type(s['grasped']) is not bool):raise ValueError('Invalid/unconfigured measured Hand state')
        except (KeyError,TypeError,ValueError) as error:
            self.feedback_info['validation_error']=str(error)
            if self._session_started and not self._stopping:self._fault=self._fault or 207
            raise ValueError(str(error)) from error

    def preflight(self):
        if self.estimate is None:raise RuntimeError('Clock registration required')
        self.validate_state();return dict(q=self.arm.q)
    def plan(self,snapshot):pass
    def start(self,snapshot):self.validate_state();self._session_started=True
    def send_reference(self,payload):
        chunk=DroidActionChunk.from_dict(payload);self._sequence+=1
        self.executor.submit(self._sequence,chunk.values,self.estimate.deadline_to_server(chunk.valid_until_ns))
        return self._sequence
    def check_session_admission(self):
        if self._session_started or self._stopping or self._fault:raise ValueError('Shadow backend is single-session')
    def reset_session(self,executor):self.check_session_admission();self.executor=executor;self.estimate=None

    @property
    def workers_joined(self):return self.source.joined

    def request_stop(self,reason):
        self.executor.stop(reason);self._stopping=True
        self.source.stop() # Process/thread teardown only. No device stop command exists.

    def update(self,now_ns,dt_s,step,service_tick):
        if not self._stopping:
            try:self.validate_state()
            except ValueError:self._fault=self._fault or 207
        if self.recorder is not None:
            if not self.recorder.emit('shadow_virtual_step',dict(step=asdict(step),measured_q=list(self.arm.q),
                    measured_dq=list(self.arm.dq),feedback=self.feedback_info,source_sample=self._sample,
                    commands_sent=False),timestamp_ns=self.clock()):self._fault=self._fault or 209
        evidence=None
        if self._stopping and not self._reported:
            evidence=dict(mode='shadow_readonly',source_joined=self.source.joined,no_commands_sent=True)
            self._reported=True
        return BackendUpdate(fault=self._fault,stop_evidence=evidence)
