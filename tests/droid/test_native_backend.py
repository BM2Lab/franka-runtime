"""Real service/authority/executor with a fake native device. Never imports Franky."""
import json
from pathlib import Path
from types import SimpleNamespace as NS
import pytest
from contracts.clock import ClockProbe, ClockOffsetEstimate
from contracts.droid import DroidIOSpec, DroidActionChunk
from contracts.droid_session import DroidSessionOpen, chunk_envelope
from contracts.episodes import SampleContext
from contracts.sessions import LeaseRequest, payload_sha256
from franka_runtime.droid.config import DroidRuntimeConfig
from franka_runtime.droid.native import NativeDroidBackend
from franka_runtime.droid.service import DroidRuntimeService

ROOT=Path(__file__).resolve().parents[2]/'configs/fr3'
SPEC=DroidIOSpec.from_json((ROOT/'droid_fr3_jointpos_v1.contract.json').read_text())


class Clock:
    now=1_000_000_000
    def __call__(self):return self.now
    def advance(self,ns=1_000_000):self.now+=ns


class Motion:
    def __init__(self,clock,q):
        self.clock,self.q=clock,q;self.sent=[];self.samples=[];self.stop=False
        self.last=NS(tick=0,monotonic_ns=0,q=q,dq=(0.,)*7,fault=0,stop_fault=0,dropped=0,
                     consumed=NS(tick=0,row_sequence=0,publication_sequence=0,monotonic_ns=0),
                     stop_finished=False,stop_at_rest=False)
    def publish(self,q,row,expiry):self.sent.append((tuple(q),row,expiry))
    def request_stop(self):self.stop=True
    def drain(self):r,self.samples=self.samples,[];return r
    def status(self):return self.last
    def callback(self,*,fault=0,finished=False,rest=False,consume=True,dropped=0):
        receipt=self.last.consumed
        if consume and self.sent and self.sent[-1][1]!=receipt.row_sequence:
            receipt=NS(tick=self.last.tick+1,row_sequence=self.sent[-1][1],publication_sequence=len(self.sent),monotonic_ns=self.clock())
        self.last=NS(tick=self.last.tick+1,monotonic_ns=self.clock(),q=self.q,dq=(0.,)*7,
                     fault=fault,stop_fault=0,dropped=dropped,consumed=receipt,
                     stop_finished=finished,stop_at_rest=rest)
        self.samples.append(self.last)


class Hand:
    width=.08;moving=False;grasped=False;fault=None;quiescent=False
    def __init__(self):self.commands=[]
    def start(self):pass
    def command(self,a):self.commands.append(a)
    def request_stop(self):self.quiescent=True


class Driver:
    error=None;joined=False
    def __init__(self,clock,q):
        self.initial_q=q;self.initial_timestamp_ns=clock();self.motion=Motion(clock,q)
    def prepare(self):pass
    def start(self):pass


class FakeNativeBackend(NativeDroidBackend):
    hardware_executed=False # Explicit fake device; a real native backend cannot run a mock config.


def make_service(recorder=None):
    clock=Clock();created=[]
    def factory(executor,settings):
        driver=Driver(clock,settings['mock_start_q']);hand=Hand()
        b=FakeNativeBackend(executor,driver,hand,clock=clock,startup_timeout_ns=1_000_000_000,
                           consumption_timeout_ns=20_000_000,stop_timeout_ns=200_000_000,state_timeout_ns=30_000_000)
        created.append(b);return b
    config=DroidRuntimeConfig(json.loads((ROOT/'droid.jointpos.mock.json').read_text()),config_dir=ROOT)
    svc=DroidRuntimeService(config,SPEC,clock=clock,clock_id='rt:boot:CLOCK_MONOTONIC',token='test',backend_factory=factory,recorder=recorder)
    return svc,created[0],clock


def start(svc,b,c,consume_startup=True):
    client='client:boot:CLOCK_MONOTONIC'
    probes=[]
    for _ in range(svc.settings['min_probes']):
        reply=svc._clock_probe({});probes.append(ClockProbe(client,svc.clock_id,c()-1000,reply['server_receive_ns'],reply['server_send_ns'],c()+1000))
    estimate=ClockOffsetEstimate.from_probes(probes,max_uncertainty_ns=svc.settings['max_uncertainty_ns'])
    req=DroidSessionOpen(SPEC,payload_sha256(SPEC),LeaseRequest('r','fr3_01','test',svc.clock_id,c(),30_000_000_000),client)
    svc._open({'token':'test','session':req.to_dict()});svc._clock_register({'probes':[p.to_dict() for p in probes]});svc._start({})
    if consume_startup:
        c.advance();b.driver.motion.callback();svc.control_step(c(),.001)
    return estimate


def submit(svc,b,c,estimate,seq=1):
    q=svc.arm.q;rows=tuple((*q,.9) for _ in range(8))
    chunk=DroidActionChunk(SampleContext('e',0,c(),'monotonic'),SPEC.action,(*q,0.),rows,c()+533333333)
    envelope=chunk_envelope(svc._session['lease'],seq,chunk,estimate,client_now_ns=c(),capability=svc.authority.capability_id)
    reply=svc._submit({'envelope':envelope.to_dict(),'chunk':chunk.to_dict()});assert reply['status']=='accepted'


def test_service_requires_native_consumption_before_applied():
    svc,b,c=make_service();estimate=start(svc,b,c);submit(svc,b,c,estimate)
    c.advance();svc.control_step(c(),.001)
    receipts=svc.authority.drain_receipts();assert [r.status for r in receipts]==['accepted']
    assert b.driver.motion.sent[-1][1]==2
    assert svc.state_sample().executing_sequence is None
    c.advance();b.driver.motion.callback();svc.control_step(c(),.001)
    receipts=svc.authority.drain_receipts();assert [r.status for r in receipts]==['applied']
    assert receipts[0].native_tick==2
    assert svc.state_sample().executing_sequence==2
    assert svc.hand.commands==['close']
    assert svc._versions()['backend']=='franky-droid-candidate'


def test_callback_observed_after_executor_timestamp_is_not_falsely_future():
    svc,b,c=make_service();estimate=start(svc,b,c);submit(svc,b,c,estimate)
    c.advance();svc.control_step(c(),.001);svc.authority.drain_receipts()
    executor_now=c();c.advance();b.driver.motion.callback();svc.control_step(executor_now,.001)
    assert [r.status for r in svc.authority.drain_receipts()]==['applied']
    assert not svc.authority.fault


def test_no_native_receipt_times_out_and_latches_stop():
    svc,b,c=make_service();estimate=start(svc,b,c);submit(svc,b,c,estimate)
    c.advance();svc.control_step(c(),.001)
    c.advance(21_000_000);svc.control_step(c(),.021)
    assert svc.authority.fault and b.driver.motion.stop
    assert not any(r.status=='applied' for r in svc.authority.drain_receipts())


@pytest.mark.parametrize('rest',[True,False])
def test_stop_requires_join_and_native_rest_evidence(rest):
    svc,b,c=make_service();start(svc,b,c)
    svc._release({'reason':'test'})
    c.advance();b.driver.motion.callback(finished=True,rest=rest,consume=False);svc.control_step(c(),.001)
    assert svc.authority.stopping # Native completion is insufficient without join.
    b.driver.joined=True;c.advance();svc.control_step(c(),.001)
    assert svc.authority.state==('complete' if rest else 'fault')
    if not rest:assert svc.authority.fault.code=='stop_failed'


def test_overflow_or_driver_failure_never_gets_mock_stop_evidence():
    svc,b,c=make_service();start(svc,b,c)
    b.driver.error='control failed';c.advance();svc.control_step(c(),.001)
    assert svc.authority.fault and svc.authority.stopping
    c.advance(201_000_000);svc.control_step(c(),.201)
    assert svc.authority.state=='fault' and svc.authority.fault.stop_state=='timeout'


def test_encoder_staleness_rejects_payload_and_requests_native_stop():
    svc,b,c=make_service();start(svc,b,c);c.advance(31_000_000)
    with pytest.raises(ValueError):b.validate_state()
    svc.control_step(c(),.031)
    assert svc.authority.fault and b.driver.motion.stop


def test_real_backend_cannot_be_injected_through_a_mock_config():
    data=json.loads((ROOT/'droid.jointpos.mock.json').read_text())
    with pytest.raises(ValueError,match='Physical DROID'):
        DroidRuntimeService(DroidRuntimeConfig(data,config_dir=ROOT),SPEC,token='test',
                            backend_factory=lambda e,s:NS(hardware_executed=True))


def test_unknown_native_row_identity_latches_fault():
    svc,b,c=make_service();start(svc,b,c)
    c.advance();b.driver.motion.callback()
    b.driver.motion.last.consumed=NS(tick=999,row_sequence=999,publication_sequence=999,monotonic_ns=c())
    svc.control_step(c(),.001)
    assert svc.authority.fault and b.driver.motion.stop


def test_startup_consumption_can_arrive_after_first_executor_tick():
    svc,b,c=make_service();estimate=start(svc,b,c,consume_startup=False)
    c.advance();svc.control_step(c(),.001)
    assert not svc.authority.fault
    assert b.driver.motion.sent[-1][1]==1
    c.advance();b.driver.motion.callback();svc.control_step(c(),.001)
    submit(svc,b,c,estimate);c.advance();svc.control_step(c(),.001)
    assert not svc.authority.fault and b.driver.motion.sent[-1][1]==2


def test_ring_samples_older_than_latest_status_are_not_reordered_faults():
    svc,b,c=make_service();start(svc,b,c)
    c.advance();b.driver.motion.callback()
    sample=b.driver.motion.samples.pop() # Ring commit delayed, latest status already visible.
    svc.control_step(c(),.001)
    b.driver.motion.samples.append(sample);c.advance();svc.control_step(c(),.001)
    assert not svc.authority.fault


def test_loader_overrides_reject_before_vendor_import(monkeypatch):
    from franka_runtime.droid.native import verify_native_artifact
    monkeypatch.setenv('LD_LIBRARY_PATH','/unreviewed')
    with pytest.raises(ValueError,match='loader overrides'):
        verify_native_artifact('/not-needed.json')


def test_initial_application_deadline_is_not_extended_until_consumption():
    svc,b,c=make_service();estimate=start(svc,b,c);submit(svc,b,c,estimate)
    c.advance();svc.control_step(c(),.001)
    deadline=svc.executor.current.deadline_ns
    assert b.driver.motion.sent[-1][2]==deadline
    c.advance();b.driver.motion.callback();svc.control_step(c(),.001)
    # Same q, new native identity, only after first application was confirmed.
    assert b.driver.motion.sent[-1][1]==3
    assert b.driver.motion.sent[-1][2]==deadline+svc.executor.hold_timeout


def test_completed_native_session_does_not_latch_a_new_fault_from_old_state():
    svc,b,c=make_service();start(svc,b,c);svc._release({'reason':'test'})
    c.advance();b.driver.motion.callback(finished=True,rest=True,consume=False)
    b.driver.joined=True;svc.control_step(c(),.001)
    assert svc.authority.state=='complete'
    c.advance(31_000_000)
    assert not svc.state_sample().valid
    svc.control_step(c(),.031)
    assert svc.authority.state=='complete' and svc.authority.fault is None


def test_stop_before_native_start_fails_closed_without_hanging_or_fake_evidence():
    svc,b,c=make_service();b.request_stop('not started')
    c.advance(201_000_000)
    update=b.update(c(),.201,svc.executor.tick(c()),1)
    assert update.stop_evidence['joined'] is False
    assert update.stop_evidence['stop_finished'] is False
    assert update.stop_evidence['stop_fault']!=0


def test_startup_uses_startup_deadline_without_valid_feedback_claim():
    svc,b,c=make_service();start(svc,b,c,consume_startup=False)
    c.advance(40_000_000);svc.control_step(c(),.04)
    assert b._fault==0
    assert not svc.state_sample().valid
    b.driver.motion.callback();svc.control_step(c(),.001)
    assert b._fault==0 and svc.state_sample().valid


def test_native_recording_preserves_receipts_samples_and_stop(tmp_path):
    from franka_runtime.droid.recording import SessionRecorder
    recorder=SessionRecorder(tmp_path/'native',metadata={'test':True},queue_capacity=128,
        max_record_bytes=16384,max_event_bytes=1_000_000,min_free_bytes=1,fsync_interval_s=.01)
    recorder.start();svc,b,c=make_service(recorder);estimate=start(svc,b,c);submit(svc,b,c,estimate)
    c.advance();svc.control_step(c(),.001);c.advance();b.driver.motion.callback();svc.control_step(c(),.001)
    svc.shutdown();c.advance();b.driver.motion.callback(finished=True,rest=True,consume=False)
    b.driver.joined=True
    result=svc.finish_shutdown(timeout_s=.1)
    assert result['ok'] and result['stop_evidence']['stop_at_rest']
    events=[json.loads(line) for line in (tmp_path/'native/events.jsonl').read_text().splitlines()]
    assert {'native_sample','native_status','receipt','command','stop_evidence','shutdown'}<={e['kind'] for e in events}
    assert [e['payload']['status'] for e in events if e['kind']=='receipt']==['accepted','applied']
    assert 'test-token' not in (tmp_path/'native/events.jsonl').read_text()


def test_recording_failure_stops_before_command_publication(tmp_path):
    from franka_runtime.droid.recording import SessionRecorder
    recorder=SessionRecorder(tmp_path/'native',metadata={},queue_capacity=128,
        max_record_bytes=16384,max_event_bytes=1_000_000,min_free_bytes=1,fsync_interval_s=.01)
    recorder.start();svc,b,c=make_service(recorder);estimate=start(svc,b,c)
    recorder._fail('simulated_disk_failure');c.advance();svc.control_step(c(),.001)
    assert svc.authority.fault.code=='native_fault:209' and b.driver.motion.stop
    assert not any(row==2 for q,row,expiry in b.driver.motion.sent)
    b.driver.joined=True;c.advance();b.driver.motion.callback(finished=True,rest=True,consume=False)
    result=svc.finish_shutdown(timeout_s=.1)
    assert not result['ok'] and not result['recording']['sealed']


def test_shutdown_keeps_supervising_until_actual_stop_evidence():
    svc,b,c=make_service();start(svc,b,c);ticks=[]
    def sleep(seconds):
        c.advance(1_000_000);ticks.append(c())
        if len(ticks)==3:
            b.driver.motion.callback(finished=True,rest=True,consume=False);b.driver.joined=True
    result=svc.finish_shutdown(timeout_s=.2,sleep=sleep)
    assert len(ticks)>=3 and result['ok'] and result['workers_joined']
    assert svc.finish_shutdown() is result


def test_shutdown_timeout_never_claims_native_join_or_rest():
    svc,b,c=make_service();start(svc,b,c)
    result=svc.finish_shutdown(timeout_s=.01)
    assert not result['ok'] and not result['workers_joined'] and not result['stop_confirmed']
    assert result['stop_evidence'] is None and b.driver.motion.stop


def test_receipt_recording_failure_prevents_dispatch():
    svc,b,c=make_service();estimate=start(svc,b,c)
    svc.authority.receipt_sink=lambda receipt:False
    q=svc.arm.q;rows=tuple((*q,.9) for _ in range(8))
    chunk=DroidActionChunk(SampleContext('e',0,c(),'monotonic'),SPEC.action,(*q,0.),rows,c()+533333333)
    envelope=chunk_envelope(svc._session['lease'],1,chunk,estimate,client_now_ns=c(),capability=svc.authority.capability_id)
    response=svc._submit({'envelope':envelope.to_dict(),'chunk':chunk.to_dict()})
    assert response['status']=='rejected' and svc.authority.fault.code=='receipt_recording_failed'
    assert b.driver.motion.stop and not any(row==2 for q,row,expiry in b.driver.motion.sent)


def test_shutdown_before_native_start_does_not_fabricate_rest():
    svc,b,c=make_service()
    # Mimic an admitted ready lease but no successful native startup.
    start(svc,b,c);b._started=False;b.driver.joined=False
    def sleep(seconds):c.advance(100_000_000)
    result=svc.finish_shutdown(timeout_s=.1,sleep=sleep)
    assert not result['ok'] and result['stop_evidence']['stop_finished'] is False


def test_shutdown_reports_supervision_exception_without_claiming_success():
    svc,b,c=make_service();start(svc,b,c)
    def failed_step(*a):raise RuntimeError('simulated_supervision_failure')
    svc.control_step=failed_step
    result=svc.finish_shutdown(timeout_s=.1)
    assert not result['ok'] and 'supervision_failure' in result['shutdown_error']
    assert b.driver.motion.stop and not result['workers_joined']
