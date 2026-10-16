"""Offline joint reference, authority and callback extraction regression checks."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import pytest
import yaml
from contracts.sessions import LeaseRequest
from franka_runtime.service.joint_policy import JointPolicyBackend,create_joint_session
from franka_runtime.common.motion.legacy import assemble_policy_observation
ROOT=Path(__file__).parents[1]
FIX=ROOT/'tests/parity/fixtures'


def make(velocity=False,mode='mock'):
    contract=yaml.safe_load((FIX/('ppo.velocity.yaml' if velocity else 'ppo.position.yaml')).read_text())
    profile=yaml.safe_load((ROOT/'configs/fr3'/('ppo.velocity-impedance.pending.yaml' if velocity else 'ppo.position.pending.yaml')).read_text())
    state=json.loads((ROOT/'configs/fr3/m4.mock-start.json').read_text())
    clock=[1_000_000_000]
    model=FIX/'robot_returned_model.urdf'
    backend=JointPolicyBackend(contract,profile,state,model,hashlib.sha256(model.read_bytes()).hexdigest(),mode=mode,clock=lambda:clock[0])
    backend.preflight();backend.start(None)
    return backend,clock


def test_position_history_and_reset():
    b,c=make();q=b.reference[:]
    _,initial=b.observation([.45,0,.35])
    b.send_reference({'contract_id':b.contract['contract_id'],'action':[1.]*6})
    assert b.reference==pytest.approx([q[i]+([.0087]*4+[.01044]*2+[0])[i] for i in range(7)])
    s,obs=b.observation([.45,0,.35])
    assert obs==assemble_policy_observation(s['q'],s['dq'],[.45,0,.35],s['position_m'],b.reference,[1.]*6,b.contract['default_joint_position_rad'],b.contract['action']['soft_lower_rad'],b.contract['action']['soft_upper_rad'],6)
    assert b.reference[6]==q[6] and obs[-6:]==[1.]*6
    other,_=make();assert other.reference==q and other.previous_action==[0.]*6


def test_velocity_controller_ticks_preserve_fractional_elapsed_time():
    b,c=make(True);q=b.reference[:]
    b.send_reference({'contract_id':b.contract['contract_id'],'action':[1.]*6})
    for _ in range(100):c[0]+=200_000;b.observe()
    assert b.reference==pytest.approx([q[i]+([.435]*4+[.522]*2+[0])[i]*.02 for i in range(7)])
    b.request_stop();old=b.reference[:];c[0]+=20_000_000;b.observe();assert b.reference==old


def test_shadow_velocity_consumption_is_one_policy_step_not_tick_integrator():
    b,c=make(True,'shadow');q=b.reference[:]
    b.send_reference({'contract_id':b.contract['contract_id'],'action':[1.]*6})
    ref=b.reference[:];c[0]+=100_000_000;b.observe();assert b.reference==ref
    assert ref[0]-q[0]==pytest.approx(.435*.02)
    assert b.consume() is None


def test_guard_and_drop_are_latched_by_authority():
    b,c=make();a=create_joint_session(b,'a'*64,robot_id='mock-fr3',clock_id='test',clock=lambda:c[0])
    lease=a.acquire(LeaseRequest('req','mock-fr3','test','test',c[0],1_000_000_000),episode_id='episode')
    # authority preflight/start owns state; no physical adapter.
    b.active=False;a.start(lease)
    record=SimpleNamespace(host_monotonic_ns=c[0],robot_time_s=1.,q=[2.]*7,q_command=[0.]*7)
    b._callback(record)
    with pytest.raises(RuntimeError,match='tracking_error'):b.poll_health(a)
    assert a.stopping
    b.telemetry_dropped=1
    assert a.finish_stop(b.stop_evidence()) is False


@pytest.mark.parametrize('action',[[0.]*7,[float('nan')]*6,[2.]*6])
def test_bad_actions_never_update_reference(action):
    b,c=make();q=b.reference[:]
    with pytest.raises(ValueError):b.send_reference({'contract_id':b.contract['contract_id'],'action':action})
    assert b.reference==q and b.sequence==0


def test_native_constructors_never_create_robot_connection():
    from franka_runtime.common.backends.franky_joint import FrankyBackend,FrankyVelocityImpedanceBackend
    for cls in (FrankyBackend,FrankyVelocityImpedanceBackend):
        with pytest.raises(RuntimeError,match='explicitly attached'):cls({})


def test_terminal_disk_failure_occurs_after_stop_and_worker_cleanup(tmp_path,monkeypatch):
    client=pytest.importorskip('learning.integrations.franka.ppo_client')
    from argparse import Namespace
    from franka_runtime.service import joint_policy
    contract=yaml.safe_load((FIX/'ppo.position.yaml').read_text())
    monkeypatch.setattr(joint_policy,'admit_bundle',lambda *a:deepcopy(contract))
    cleanup=[];original_stop=joint_policy.JointPolicyBackend.request_stop
    def stop(self,*args):
        cleanup.append('stop');return original_stop(self,*args)
    monkeypatch.setattr(joint_policy.JointPolicyBackend,'request_stop',stop)
    class Worker:
        ready={'type':'ready','provider':'TEST_DOUBLE_ONLY'}
        inflight=None;sent_ns=0
        def __init__(self,*args):pass
        def submit(self,r):self.inflight=r;self.sent_ns=r['observation_monotonic_ns']
        def poll(self):
            if self.inflight is None:return None
            r=self.inflight;self.inflight=None
            return {**r,'action':[0.]*6,'completed_monotonic_ns':r['observation_monotonic_ns']}
        def close(self):cleanup.append('worker_closed')
    original_write=client.write
    def write(path,value):
        if path.name=='terminal-feedback.json':
            assert 'stop' in cleanup and 'worker_closed' in cleanup
            raise OSError('injected disk failure')
        return original_write(path,value)
    monkeypatch.setattr(client,'write',write)
    args=Namespace(output=tmp_path/'run',duration_s=.001,smoke=True,bundle=tmp_path,
        manifest_sha256='a'*64,profile=ROOT/'configs/fr3/ppo.position.pending.yaml',
        state=ROOT/'configs/fr3/m4.mock-start.json',model=FIX/'robot_returned_model.urdf',
        model_sha256=hashlib.sha256((FIX/'robot_returned_model.urdf').read_bytes()).hexdigest(),
        mode='shadow',catalog=None,target=[.45,0,.35],worker_python=Path('/explicit/test/python'),robot_id='offline-fr3')
    with pytest.raises(OSError,match='injected disk failure'):client.run(args,worker_factory=Worker)
    assert cleanup==['stop','worker_closed']
