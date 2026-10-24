import threading
import time
from franka_runtime.droid.hand import SerializedHandWorker

STATE=dict(width=.08,moving=False,grasped=False)


def wait(predicate):
    deadline=time.monotonic()+1
    while not predicate():
        assert time.monotonic()<deadline
        time.sleep(.001)


def test_blocking_hand_does_not_block_arm_thread_and_stop_discards_pending():
    active=threading.Event();release=threading.Event();calls=[]
    def perform(action):
        calls.append(action);active.set();assert release.wait(1)
    hand=SerializedHandWorker(perform,lambda:STATE,initial_state=STATE)
    hand.start();hand.command('close');assert active.wait(1)
    hand.command('open');hand.command('close')
    assert hand.replaced==1 and hand.moving
    hand.request_stop();assert not hand.quiescent
    release.set();wait(lambda:hand.quiescent)
    assert calls==['close'] and hand.fault is None


def test_grasp_result_and_measured_flags_are_preserved():
    hand=SerializedHandWorker(lambda _:True,lambda:dict(STATE,grasped=True),initial_state=STATE)
    hand.start();hand.command('close');wait(lambda:hand.grasped)
    hand.request_stop();wait(lambda:hand.quiescent)
    assert hand.snapshot()['measured_timestamp_ns']>0


def test_hand_exception_and_false_result_are_latched():
    hand=SerializedHandWorker(lambda _:False,lambda:STATE,initial_state=STATE)
    hand.start();hand.command('close');wait(lambda:hand.fault is not None)
    hand.request_stop();wait(lambda:hand.quiescent)
    assert 'failure' in hand.fault


def test_command_history_overflow_latches_and_stops_accepting_intents():
    import pytest
    active=threading.Event();release=threading.Event();calls=[]
    def perform(action):
        calls.append(action);active.set();assert release.wait(1)
    hand=SerializedHandWorker(perform,lambda:STATE,initial_state=STATE,history_capacity=1)
    hand.start();hand.command('close');assert active.wait(1)
    with pytest.raises(RuntimeError,match='history_overflow'):
        hand.command('open')
    assert hand.fault=='hand_command_history_overflow'
    hand.request_stop();release.set();wait(lambda:hand.quiescent)
    assert calls==['close']
