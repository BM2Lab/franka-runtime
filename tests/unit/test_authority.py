from dataclasses import replace
import itertools
import pytest
from dataclasses import dataclass
from contracts.sessions import LeaseRequest, LeaseRelease, CommandEnvelope, payload_sha256
from franka_runtime.service.authority import SessionAuthority, AuthorityError


@dataclass(frozen=True)
class MotionSessionSpec:
    """Stand-in for contracts.motion.MotionSessionSpec: the authority needs only a frozen spec with robot_id."""
    robot_id: str
    mode: str
    digest: str
    base: str
    tcp: str
    profile: str
    tool: str


class Clock:
    now=1000
    def __call__(self):return self.now


class Backend:
    def __init__(self):self.sent=[];self.stops=[]
    def preflight(self):return 'state'
    def plan(self,s):pass
    def start(self,s):pass
    def send_reference(self,p):self.sent.append(p);return len(self.sent)+1 # native startup has sequence 1
    def request_stop(self,reason):self.stops.append(reason)


def setup(capacity=16):
    clock=Clock();backend=Backend();ids=iter(map(str,itertools.count()))
    def validate(p):
        if set(p)!={'value'} or not isinstance(p['value'],int):raise ValueError('Invalid reference')
    a=SessionAuthority(backend,MotionSessionSpec('unit','m4','a'*64,'base','tcp','dynamic','none'),validate,
        clock_id='host:boot:raw',command_timeout_ns=100,clock=clock,receipt_capacity=capacity,id_factory=lambda:next(ids))
    lease=a.acquire(LeaseRequest('r','unit','owner',a.clock_id,clock(),1000),episode_id='e1')
    a.start(lease)
    return a,backend,clock,lease


def command(a,lease,clock,seq=1,value=1):
    p={'value':value}
    return CommandEnvelope(lease,seq,seq-1,clock(),clock(),clock()+100,a.capability_id,payload_sha256(p)),p


def finish(a):return a.finish_stop({'joined':True,'stop_finished':True,'stop_at_rest':True,'stop_fault':0,'dropped_samples':0})


def test_acceptance_dispatch_and_application_are_distinct_and_payload_copied():
    a,b,c,l=setup();e,p=command(a,l,c)
    assert a.submit(e,p).status=='accepted';assert b.sent==[]
    p['value']=9
    seq=a.dispatch();assert b.sent==[{'value':1}]
    assert [r.status for r in a.drain_receipts()]==['accepted']
    c.now+=1
    r=a.acknowledge(lease_id=l.lease_id,native_sequence=seq,native_tick=1,timestamp_ns=c(),clock_id=a.clock_id)
    assert r.status=='applied' and r.sequence==1 and r.native_tick==1


def test_queue_rejects_overwrite_wrong_owner_and_stale_sequence():
    a,b,c,l=setup();e,p=command(a,l,c);a.submit(e,p)
    assert a.submit(*command(a,l,c,seq=2)).status=='rejected'
    assert a.submit(e,p).status=='rejected'
    assert a.submit(replace(e,lease=replace(l,owner_id='other')),p).status=='rejected'
    assert a.fault is None
    a.dispatch();assert len(b.sent)==1


def test_dispatch_rechecks_expiry_and_requests_existing_stop():
    a,b,c,l=setup();e,p=command(a,l,c);a.submit(e,p);c.now=e.deadline_ns
    assert a.dispatch().status=='rejected' and not b.sent
    assert a.fault.code=='dispatch_failed' and b.stops
    assert finish(a) and a.fault.stop_state=='complete'
    with pytest.raises(AuthorityError):a.acquire(LeaseRequest('r','unit','new',a.clock_id,c(),1000),episode_id='e2')


def test_leases_and_native_faults_are_sticky_even_after_stop():
    a,b,c,l=setup();c.now=l.expires_ns;a.poll()
    assert a.fault.code=='lease_expired';a.poll(native_fault=8)
    assert a.fault.code=='lease_expired' and len(b.stops)==1
    assert finish(a);assert a.state=='fault'


def test_native_mail_supersession_and_old_episode_ack_rejection():
    a,b,c,l=setup()
    a.submit(*command(a,l,c));one=a.dispatch()
    a.submit(*command(a,l,c,seq=2));two=a.dispatch();c.now+=1
    r=a.acknowledge(lease_id=l.lease_id,native_sequence=two,native_tick=2,timestamp_ns=c(),clock_id=a.clock_id)
    assert r.status=='applied'
    receipts=a.drain_receipts();assert any('Superseded' in r.reason and r.sequence==1 for r in receipts)
    a.release(LeaseRelease(l,c(),'complete'));assert finish(a)
    new=a.acquire(LeaseRequest('r2','unit','owner',a.clock_id,c(),1000),episode_id='e2')
    assert new.session_id!=l.session_id and new.lease_id!=l.lease_id
    with pytest.raises(AuthorityError):a.acknowledge(lease_id=l.lease_id,native_sequence=two,native_tick=3,timestamp_ns=c(),clock_id=a.clock_id)
    a.start(new);assert a.submit(*command(a,new,c)).status=='accepted'


def test_application_requires_timely_matching_native_evidence():
    a,b,c,l=setup();e,p=command(a,l,c);a.submit(e,p);seq=a.dispatch();c.now=e.deadline_ns
    r=a.acknowledge(lease_id=l.lease_id,native_sequence=seq,native_tick=1,timestamp_ns=c(),clock_id=a.clock_id)
    assert r.status=='rejected' and a.fault.code=='application_deadline'


def test_bounded_receipts_and_failed_stop_prevent_restart():
    a,b,c,l=setup(capacity=1);e,p=command(a,l,c);a.submit(e,p);a.submit(e,p)
    assert a.fault.code=='receipt_overflow' and a.dropped_receipts==1
    assert not a.finish_stop({'joined':False,'stop_finished':False,'stop_at_rest':False,'stop_fault':18,'dropped_samples':0})
    assert a.fault.stop_state=='timeout'


def test_release_invalidates_queued_mail_and_reset_is_per_instance():
    a,b,c,l=setup();other,_,_,_=setup();a.submit(*command(a,l,c));a.release(LeaseRelease(l,c(),'stop'))
    assert any(r.status=='rejected' for r in a.drain_receipts())
    assert finish(a);assert a.state=='complete' and other.state=='running'
    assert b.sent==[]


def test_wrong_clock_hash_or_payload_never_reaches_backend():
    a,b,c,l=setup();e,p=command(a,l,c)
    assert a.submit(e,{'value':2}).status=='rejected'
    assert a.submit(replace(e,lease=replace(l,clock_id='other')),p).status=='rejected'
    bad,p=command(a,l,c,value='bad')
    assert a.submit(bad,p).status=='rejected' and b.sent==[]


def test_native_telemetry_bridge_and_logging_overflow():
    from types import SimpleNamespace
    from franka_runtime.common.backends.franky_m4 import FrankyMotionBackend
    a,_,c,l=setup();b=FrankyMotionBackend({});a.backend=b
    b.motion=SimpleNamespace(fault=0,dropped_samples=0,request_stop=lambda:None)
    b.send_reference=lambda payload:2
    a.submit(*command(a,l,c));a.dispatch();c.now+=1
    sample=SimpleNamespace(fault=0,stopping=False,reference_sequence=2,tick=1,monotonic_ns=c())
    b.drain=lambda stream:[sample]
    b.drain_to_authority(a,None)
    assert [r.status for r in a.drain_receipts()]==['accepted','applied']
    b.motion.dropped_samples=1;b.drain=lambda stream:[]
    b.drain_to_authority(a,None)
    assert a.fault.code=='native_logger_overflow'


def test_lease_is_rechecked_after_offline_planning_before_start():
    a,b,c,l=setup();a.state='ready';called=[]
    def plan(state):c.now=l.expires_ns
    b.plan=plan;b.start=lambda state:called.append('started')
    with pytest.raises(AuthorityError):a.start(l)
    assert called==[] and a.fault.code=='lease_expired'
