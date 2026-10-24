"""Serialized non-RT hand worker; imports and construction start no threads.

One active blocking operation plus one replaceable pending intent. Stop clears
pending work and waits for the active SDK call; it never claims instant preemption.
A backend deadline faults if that worker cannot become quiescent.
"""
from collections import deque
import math
import threading
import time


class SerializedHandWorker:
    def __init__(self, perform, read_state, *, initial_state, clock=time.monotonic_ns,
                 poll_interval_s=.02, history_capacity=1024):
        if poll_interval_s <= 0 or type(history_capacity) is not int or history_capacity <= 0:
            raise ValueError('Invalid hand poll/history bounds')
        self.perform, self.read_state, self.clock = perform, read_state, clock
        self.poll_interval = poll_interval_s
        self._cv = threading.Condition()
        self._state = dict(initial_state)
        self._validate(self._state)
        self._stamp = self.clock()
        self._pending = None
        self._busy = self._closing = False
        self._fault = None
        self._thread = None
        self._commands = deque(maxlen=history_capacity)
        self.replaced = 0

    @staticmethod
    def _validate(state):
        if not math.isfinite(state['width']) or state['width'] < 0:
            raise ValueError('Invalid measured gripper width')
        if type(state['moving']) is not bool or type(state['grasped']) is not bool:
            raise ValueError('Explicit hand moving/grasped flags required')

    def start(self):
        with self._cv:
            if self._closing or self._fault:
                raise RuntimeError('Hand worker is terminal')
            if self._thread is None:
                self._thread = threading.Thread(target=self._run, name='droid-hand', daemon=True)
                self._thread.start()

    def command(self, action):
        if action not in ('open','close'):
            raise ValueError('Invalid hand intent')
        with self._cv:
            if self._closing or self._fault:
                raise RuntimeError('Hand worker is stopping/faulted')
            if len(self._commands) == self._commands.maxlen:
                self._fault = 'hand_command_history_overflow'
                raise RuntimeError(self._fault)
            self._commands.append(action)
            if self._pending is not None:
                self.replaced += 1
            self._pending = action
            self._cv.notify()

    def _run(self):
        try:
            while True:
                with self._cv:
                    if self._closing or self._fault:
                        self._pending = None
                        self._busy = False
                        return
                    action, self._pending = self._pending, None
                    self._busy = action is not None
                if action is not None:
                    result = self.perform(action)  # ONLY this worker invokes blocking SDK operations.
                    if result is False:
                        raise RuntimeError('Hand operation returned failure')
                state = dict(self.read_state())
                self._validate(state)
                with self._cv:
                    self._state, self._stamp, self._busy = state, self.clock(), False
                    if self._pending is None and not self._closing:
                        self._cv.wait(self.poll_interval)
        except Exception as error:
            with self._cv:
                self._fault = self._fault or repr(error)
                self._busy = False
                self._pending = None

    def request_stop(self):
        with self._cv:
            self._closing = True
            self._pending = None
            self._cv.notify_all()

    @property
    def quiescent(self):
        with self._cv:
            if not self._closing:
                return False
            thread = self._thread
            return not self._busy and (thread is None or not thread.is_alive())

    @property
    def fault(self):
        with self._cv:return self._fault

    @property
    def commands(self):
        with self._cv:return tuple(self._commands)

    def snapshot(self):
        with self._cv:
            return dict(self._state, operation_pending=self._busy or self._pending is not None,
                        measured_timestamp_ns=self._stamp, fault=self._fault)

    @property
    def width(self):return self.snapshot()['width']
    @property
    def moving(self):
        s=self.snapshot();return s['moving'] or s['operation_pending']
    @property
    def grasped(self):return self.snapshot()['grasped']
