import threading
import time
from franka_runtime.droid.concurrent_hand import ConcurrentHandWorker


def wait(predicate,timeout=1.):
    end=time.monotonic()+timeout
    while not predicate() and time.monotonic()<end:time.sleep(.002)
    assert predicate()


def worker(perform,cancel,*,stalled=False,attempts=20):
    count=[0]
    def read():
        if not stalled:count[0]+=1
        return dict(width=.08,moving=False,grasped=False,message_id=count[0])
    h=ConcurrentHandWorker(perform,read,cancel,initial_state=dict(width=.08,moving=False,grasped=False,message_id=0),
        clock=time.monotonic_ns,poll_interval_s=.005,max_state_age_ns=50_000_000,moving_epsilon_m_s=.001,
        cancel_interval_s=.01,cancel_attempts=attempts,history_capacity=20,cancelled_exception=lambda e:str(e)=='aborted')
    return h


def test_reads_continue_during_blocking_command_and_stop_cancels():
    entered=threading.Event();released=threading.Event()
    def perform(a):
        entered.set();released.wait(1.);raise RuntimeError('aborted')
    h=worker(perform,lambda:(released.set() or True))
    assert h._thread is None
    h.prime();h.start();h.command('close');assert entered.wait(1.)
    first=h.snapshot()['message_id'];wait(lambda:h.snapshot()['message_id']>=first+3)
    assert h.fresh and h.moving and not h.quiescent
    h.request_stop();wait(lambda:h.quiescent)
    assert h.fault is None and h.snapshot()['cancel_success']


def test_stalled_udp_counter_faults():
    h=worker(lambda a:True,lambda:True,stalled=True);h.start()
    wait(lambda:h.fault is not None);wait(lambda:h.quiescent)
    assert not h.fresh and 'counter' in h.fault


def test_failed_cancel_does_not_claim_quiescence():
    entered=threading.Event();released=threading.Event()
    def perform(a):entered.set();released.wait(1.);return True
    h=worker(perform,lambda:False);h.start();h.command('open');assert entered.wait(1.)
    h.request_stop();wait(lambda:h.fault is not None)
    assert not h.quiescent and not h.snapshot()['cancel_success']
    released.set();wait(lambda:h.quiescent)


def test_stale_host_measurement():
    h=worker(lambda a:True,lambda:True);h._stamp-=100_000_000
    assert not h.fresh
    h.request_stop();wait(lambda:h.quiescent)
