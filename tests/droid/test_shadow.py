"""D7 integration with fake read-only packets. No device/socket imports from SDKs."""
import json
from pathlib import Path
import runpy
import pytest
from franka_runtime.droid.shadow_config import ShadowRuntimeConfig
from franka_runtime.droid.shadow_cli import prepare_shadow
from franka_runtime.droid.shadow_source import ReadOnlyStateSource
from franka_runtime.droid.recording import inspect_recording

ROOT=Path(__file__).resolve().parents[2]
N=runpy.run_path(str(Path(__file__).with_name('test_native_backend.py')))
SPEC=N['SPEC']


class Source:
    fault=None;joined=False
    def __init__(self,*args,**kwargs):self.started=False;self.stopped=False
    def start(self):self.started=True
    def wait_initial(self,timeout):return self.snapshot()
    def snapshot(self):return dict(self.sample)
    def stop(self):self.stopped=True;self.joined=True


def make(tmp_path,realtime=False):
    import time
    c=time.monotonic_ns if realtime else N['Clock']();source=Source()
    q=json.loads((ROOT/'configs/fr3/droid.jointpos.mock-link.json').read_text())['mock_start_q_rad']
    source.sample=dict(schema_version=1,server_version=10,arm_ns=c(),arm_counter_ms=1000,hand_ns=c(),hand_counter_ms=1000,
        q=q,dq=[0.]*7,robot_mode=1,errors=False,contact=False,width=.08,max_width=.08,grasped=False)
    cfg=ShadowRuntimeConfig(dict(schema_version=1,runtime='droid_shadow_v1',backend='shadow_libfranka',executable=True,
        motion_authorized=False,read_only=True,physical_candidate=str(ROOT/'configs/fr3/droid-source-review/physical.review.pending.json'),
        source=dict(address='test.invalid',helper='/not-executed',helper_sha256='a'*64,poll_hz=50,server_version=10,initial_timeout_s=1.),
        recording_directory=str(tmp_path/'recording')),ROOT)
    if realtime:
        def snapshot():
            now=c();source.sample.update(arm_ns=now,hand_ns=now,arm_counter_ms=now//1_000_000,hand_counter_ms=now//1_000_000)
            return dict(source.sample)
        source.snapshot=snapshot
    source.config=cfg
    svc=prepare_shadow(cfg,SPEC,clock=c,clock_id='test:monotonic',token='test',source_factory=lambda *a,**kw:source)
    return svc,source,c


def start(svc,c):
    b=svc.backend
    # Existing protocol helper expects driver callback; shadow deliberately has no driver.
    return N['start'](svc,b,c,consume_startup=False)


def test_shadow_name_receipts_and_measured_feedback_are_separate_from_virtual_rows(tmp_path):
    svc,src,c=make(tmp_path);estimate=start(svc,c)
    assert svc._hello({})['versions']['backend'].startswith('shadow')
    assert not svc.authority.expect_application
    before=tuple(src.sample['q']);N['submit'](svc,svc.backend,c,estimate)
    c.advance();svc.control_step(c(),.001)
    assert [r.status for r in svc.authority.drain_receipts()]==['accepted']
    assert svc.state_sample().q_rad==before and svc.state_sample().executing_sequence is None
    assert svc.backend.execution==(None,None) and svc.hand.commands==()
    assert not src.stopped
    report=svc.finish_shutdown(timeout_s=.1)
    assert report['ok'] and report['stop_evidence']['mode']=='shadow_readonly' and src.stopped
    audit=inspect_recording(tmp_path/'recording')
    assert audit['kinds']['shadow_virtual_step']>=1 and audit['kinds']['shadow_would_gripper_command']==1
    events=[json.loads(line) for line in (tmp_path/'recording/events.jsonl').read_text().splitlines()]
    assert len([e for e in events if e['kind']=='shadow_acceptance_check' and e['payload']['passed']])==2
    assert all(e['payload']['status']!='applied' for e in events if e['kind']=='receipt')


@pytest.mark.parametrize('change',[{'arm_ns':0},{'hand_ns':0},{'dq':[2.]*7},{'q':[float('nan')]*7},
                                  {'robot_mode':4},{'errors':True},{'contact':True},{'max_width':0.}])
def test_native_style_feedback_guards_invalidate_shadow(tmp_path,change):
    svc,src,c=make(tmp_path);start(svc,c);src.sample.update(change)
    assert svc.state_sample().valid is False
    svc.control_step(c(),.001)
    assert svc.authority.fault and svc.hand.commands==()
    assert svc.finish_shutdown(timeout_s=.1)['workers_joined']


def test_acceptance_uses_new_measured_q_and_records_failed_workspace_check(tmp_path):
    svc,src,c=make(tmp_path);estimate=start(svc,c)
    original=svc.workspace.check_rows
    seen=[]
    def check(rows,q):seen.append(tuple(q));raise ValueError('synthetic workspace rejection')
    svc.workspace.check_rows=check
    q=list(src.sample['q']);q[0]+=.01;src.sample['q']=q
    # helper asserts accepted, so construct its command without that assertion.
    from contracts.droid import DroidActionChunk
    from contracts.droid_session import chunk_envelope
    from contracts.episodes import SampleContext
    chunk=DroidActionChunk(SampleContext('e',0,c(),'monotonic'),SPEC.action,(*q,0.),tuple((*q,.9) for _ in range(8)),c()+533333333)
    env=chunk_envelope(svc._session['lease'],1,chunk,estimate,client_now_ns=c(),capability=svc.authority.capability_id)
    receipt=svc._submit({'envelope':env.to_dict(),'chunk':chunk.to_dict()})
    assert receipt['status']=='rejected' and seen==[tuple(q)] and svc.arm.q==tuple(q)
    svc.finish_shutdown(timeout_s=.1)
    events=[json.loads(line) for line in (tmp_path/'recording/events.jsonl').read_text().splitlines()]
    checks=[e['payload'] for e in events if e['kind']=='shadow_acceptance_check']
    assert checks[-1]['stage']=='workspace' and checks[-1]['passed'] is False and checks[-1]['measured_q']==q


def test_virtual_hold_and_underrun_are_logged_without_commands(tmp_path):
    svc,src,c=make(tmp_path);estimate=start(svc,c);N['submit'](svc,svc.backend,c,estimate)
    c.advance();svc.control_step(c(),.001)
    for delta in (534_000_000,201_000_000):
        c.advance(delta);src.sample.update(arm_ns=c(),hand_ns=c());svc.control_step(c(),delta/1e9)
    assert svc.authority.fault.code=='native_fault:101' and src.stopped and svc.hand.commands==()
    svc.finish_shutdown(timeout_s=.1)
    events=[json.loads(line) for line in (tmp_path/'recording/events.jsonl').read_text().splitlines()]
    steps=[e['payload']['step'] for e in events if e['kind']=='shadow_virtual_step']
    assert any(s['holding'] for s in steps) and any(s['stop_reason']=='underrun_hold_timeout' for s in steps)


def test_reader_rejects_packet_restamping_without_counter_progress(tmp_path):
    r=ReadOnlyStateSource('/unused','a'*64,'test.invalid',stderr_path=tmp_path/'stderr')
    v=dict(arm_counter_ms=1,arm_ns=100,hand_counter_ms=1,hand_ns=100)
    r._accept(v);r._accept(v)
    with pytest.raises(ValueError,match='restamped'):r._accept({**v,'arm_ns':200})


def test_readonly_config_refuses_motion_and_pending_execution(tmp_path):
    svc,src,c=make(tmp_path);svc.finish_shutdown(timeout_s=.1)
    for field,value in [("executable",False),("motion_authorized",True),("read_only",False)]:
        data=dict(src.config.data);data[field]=value
        with pytest.raises(ValueError):ShadowRuntimeConfig(data,ROOT).require_runnable()
    cfg=svc.settings # No mock acceleration or start-pose data is used.
    assert 'max_acceleration' not in cfg and 'mock_start_q' not in cfg


def test_native_reader_source_has_no_command_api_calls():
    import re
    source=(ROOT/'native/droid_readonly_state/droid_readonly_state.cpp').read_text()
    assert not re.search(r'(?:robot|gripper)\.(?:control|move|grasp|stop|automaticErrorRecovery|set\w+)\s*\(',source)


def test_shadow_protocol_over_localhost_sends_no_applied_receipts(tmp_path):
    import threading,time
    Wire=runpy.run_path(str(Path(__file__).with_name('test_service.py')))['Client']
    svc,src,c=make(tmp_path,realtime=True)
    svc.settings.update(bind_host='127.0.0.1',port=0,allowed_peers=('127.0.0.1',))
    svc._token='test-token'
    port=svc.bind();thread=threading.Thread(target=svc.serve,daemon=True);thread.start()
    client=Wire(port)
    try:
        assert client.call('hello')['versions']['backend'].startswith('shadow')
        client.session()
        receipt=client.submit(*client.chunk(1,0,.001))
        assert receipt['status']=='accepted'
        time.sleep(.02)
        assert all(r['status']!='applied' for r in client.call('receipts')['receipts'])
        assert client.state().q_rad==tuple(src.sample['q']) and svc.hand.commands==()
        client.call('release',{'reason':'shadow test complete'})
    finally:
        client.sock.close();svc.shutdown();thread.join(3.)
    assert not thread.is_alive() and src.joined and svc.shutdown_report['ok']


def test_readonly_source_process_lifecycle_without_fci(tmp_path):
    import hashlib
    helper=tmp_path/'fake-reader'
    helper.write_text('''#!/usr/bin/python3
import json,time
while True:
 n=time.monotonic_ns()
 print(json.dumps(dict(arm_ns=n,hand_ns=n,arm_counter_ms=n//1000000,hand_counter_ms=n//1000000)),flush=True)
 time.sleep(.01)
''')
    helper.chmod(0o700)
    source=ReadOnlyStateSource(helper,hashlib.sha256(helper.read_bytes()).hexdigest(),'not-used',stderr_path=tmp_path/'stderr')
    try:
        source.start();sample=source.wait_initial(1.)
        assert sample['arm_ns']>0 and source.fault is None
    finally:source.stop()
    assert source.joined


def test_readonly_source_hash_failure_never_spawns(tmp_path):
    helper=tmp_path/'reader';helper.write_text('not executable')
    source=ReadOnlyStateSource(helper,'0'*64,'not-used',stderr_path=tmp_path/'stderr')
    with pytest.raises(ValueError,match='hash mismatch'):source.start()
    assert source.process is None and not (tmp_path/'stderr').exists()


def test_relative_config_paths_resolve_the_workspace_model_once(tmp_path,monkeypatch):
    """The handoff launches from the repo root with relative --config; the URDF must not be joined twice."""
    monkeypatch.chdir(ROOT)
    data=json.loads(Path('configs/fr3/droid.jointpos.shadow-link.json').read_text())
    data.update(recording_directory=str(tmp_path/'recording'))
    settings=ShadowRuntimeConfig(data,Path('configs/fr3')).require_runnable()
    assert settings['robot_id']


REAL_READER = ROOT/'tests/fixtures/droid/readonly-reader-2026-10-24.jsonl'  # 80 packets, FR3 + Franka Hand, read-only


def real_source():
    from franka_runtime.droid.shadow_source import ReadOnlyStateSource
    return ReadOnlyStateSource('/bin/true','0'*64,'172.16.0.6',stderr_path=Path('/dev/null'))


def test_real_hand_counter_stream_is_accepted_and_integrity_checks_remain():
    rows=[json.loads(line) for line in REAL_READER.read_text().splitlines()]
    assert len({r['hand_counter_ms'] for r in rows})>5  # the Hand counter is not milliseconds (about 20/s)
    source=real_source()
    for row in rows:source._accept(row)
    assert source.snapshot()['arm_counter_ms']==rows[-1]['arm_counter_ms']
    stalled=dict(rows[-1],hand_counter_ms=rows[-1]['hand_counter_ms']+1)
    with pytest.raises(ValueError,match='Hand counter advanced'):source._accept(stalled)
    lagging=real_source()
    for row in rows[:2]:lagging._accept(row)
    late=dict(rows[2],arm_ns=rows[2]['arm_ns']+200_000_000)
    with pytest.raises(ValueError,match='lags host clock'):lagging._accept(late)


@pytest.mark.parametrize('max_width,valid',[(0.0799846276640892,True),(0.0791,True),(0.0789,False),(0.,False)])
def test_homed_hand_width_tolerance(tmp_path,max_width,valid):
    svc,src,c=make(tmp_path);start(svc,c);src.sample.update(max_width=max_width,width=min(0.0793,max_width))
    assert svc.state_sample().valid is valid
