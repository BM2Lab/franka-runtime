"""Concurrent libfranka-compatible hand worker; no thread starts at import/construction.

One UDP state reader, one serialized command worker, and bounded stop attempts on
an independent cancellation worker. Requires an SDK with documented thread safety.
No assumption that a successful stop response instantly joins an active command.
"""
import math
import threading
from .hand import SerializedHandWorker


class ConcurrentHandWorker(SerializedHandWorker):
    def __init__(self,perform,read_state,cancel,*,initial_state,clock,poll_interval_s,
                 max_state_age_ns,moving_epsilon_m_s,cancel_interval_s,cancel_attempts,
                 history_capacity,cancelled_exception):
        for v in (poll_interval_s,moving_epsilon_m_s,cancel_interval_s):
            if isinstance(v,bool) or not math.isfinite(v) or v<=0:raise ValueError('Invalid hand cadence/threshold')
        if type(max_state_age_ns) is not int or max_state_age_ns<=0 or type(cancel_attempts) is not int or cancel_attempts<1:
            raise ValueError('Explicit positive hand freshness/cancellation bounds required')
        super().__init__(perform,read_state,initial_state=initial_state,clock=clock,
                         poll_interval_s=poll_interval_s,history_capacity=history_capacity)
        self.cancel,self.cancelled_exception=cancel,cancelled_exception
        self.max_state_age=max_state_age_ns;self.moving_epsilon=moving_epsilon_m_s
        self.cancel_interval,self.cancel_attempts=cancel_interval_s,cancel_attempts
        self._reader=self._canceller=None
        self._cancel_done=False;self._cancel_success=False
        self._message_id=initial_state['message_id']
        if type(self._message_id) is not int or self._message_id<0:raise ValueError('Explicit SDK hand message counter required')

    def _failure(self,error):
        with self._cv:self._fault=self._fault or repr(error)
        self.request_stop()

    def prime(self):
        with self._cv:
            if self._thread is not None or self._closing:raise RuntimeError('Hand cannot be primed after startup/stop')
        self._read_once()

    def _read_once(self):
        state=dict(self.read_state());self._validate(state);now=self.clock()
        message=state['message_id']
        with self._cv:
            if type(message) is not int or message<=self._message_id:
                raise ValueError('Hand UDP message counter stalled or regressed')
            dt=(now-self._stamp)/1e9
            if dt<=0:raise ValueError('Hand host clock did not advance')
            state['moving']=abs(state['width']-self._state['width'])/dt>self.moving_epsilon
            self._state,self._stamp,self._message_id=state,now,message

    def command(self, action):
        with self._cv:
            super().command(action)
            self._cv.notify_all()  # Both command and reader wait on this condition.

    def start(self):
        with self._cv:
            if self._closing or self._fault:raise RuntimeError('Hand worker is terminal')
            if self._thread is not None:return
            self._thread=threading.Thread(target=self._commands_run,name='droid-hand-command',daemon=True)
            self._reader=threading.Thread(target=self._read_run,name='droid-hand-state',daemon=True)
            self._reader.start();self._thread.start()

    def _commands_run(self):
        try:
            while True:
                with self._cv:
                    while self._pending is None and not self._closing and not self._fault:self._cv.wait()
                    if self._closing or self._fault:return
                    action,self._pending=self._pending,None;self._busy=True
                try:
                    if self.perform(action) is False:raise RuntimeError('Hand operation returned failure')
                except Exception as error:
                    with self._cv:expected=self._closing and self.cancelled_exception(error)
                    if not expected:self._failure(error)
                finally:
                    with self._cv:self._busy=False;self._cv.notify_all()
        except Exception as error:self._failure(error)

    def _read_run(self):
        try:
            while True:
                with self._cv:
                    if self._closing and not self._busy and self._cancel_done:return
                self._read_once()
                with self._cv:self._cv.wait(self.poll_interval)
        except Exception as error:self._failure(error)

    def request_stop(self):
        with self._cv:
            self._closing=True;self._pending=None;self._cv.notify_all()
            if self._canceller is None:
                self._canceller=threading.Thread(target=self._cancel_run,name='droid-hand-cancel',daemon=True)
                self._canceller.start()

    def _cancel_run(self):
        success=True
        try:
            for _ in range(self.cancel_attempts):
                with self._cv:
                    if not self._busy:break
                if self.cancel() is not True:raise RuntimeError('Hand stop command failed')
                with self._cv:
                    if self._busy:self._cv.wait(self.cancel_interval)
            with self._cv:
                if self._busy:raise RuntimeError('Hand cancellation unconfirmed: active command has not joined')
        except Exception as error:
            success=False
            with self._cv:self._fault=self._fault or repr(error)
        finally:
            with self._cv:
                self._cancel_success=success;self._cancel_done=True;self._cv.notify_all()

    @property
    def quiescent(self):
        with self._cv:
            threads=(self._thread,self._reader,self._canceller)
            return self._closing and self._cancel_done and not self._busy and all(t is None or not t.is_alive() for t in threads)

    @property
    def fresh(self):
        with self._cv:return self._fault is None and 0<=self.clock()-self._stamp<=self.max_state_age

    def snapshot(self):
        state=super().snapshot()
        with self._cv:
            state.update(message_id=self._message_id,state_age_ns=self.clock()-self._stamp,
                         fresh=self.fresh,cancel_done=self._cancel_done,cancel_success=self._cancel_success,
                         moving_source='measured_width_delta_or_operation_pending')
        return state
