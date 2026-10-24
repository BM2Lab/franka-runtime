"""DROID native adapter. Dependencies/connected devices are explicitly injected.

This module never imports Franky or constructs a Robot/Gripper. The physical CLI
remains disabled until commissioning/schema/factory work is complete. Fake drivers
exercise the same adapter. All clocks must be this host's CLOCK_MONOTONIC.
"""
from dataclasses import dataclass
import math
import threading

from contracts.droid import DroidActionChunk
from .backend import BackendUpdate, Consumption


@dataclass
class EncoderState:
    q: tuple
    dq: tuple


class FrankyArmDriver:
    """Nonblocking adapter around an explicitly supplied, pinned Franky robot.

    motion_factory creates a fresh DroidJointImpedanceMotion. robot.move executes
    on a dedicated worker; no automatic recovery, robot.stop or fallback motion.
    """
    def __init__(self, robot, motion_factory, initial_q, *, initial_timestamp_ns, limit_rate, cutoff_hz):
        if (type(limit_rate) is not bool or isinstance(cutoff_hz, bool)
                or not math.isfinite(cutoff_hz) or cutoff_hz <= 0
                or type(initial_timestamp_ns) is not int or initial_timestamp_ns <= 0):
            raise ValueError('Explicit libfranka filter/rate settings required')
        self.robot, self.motion_factory = robot, motion_factory
        self.initial_timestamp_ns = initial_timestamp_ns
        self.initial_q = tuple(initial_q)
        if len(self.initial_q) != 7 or not all(math.isfinite(x) for x in self.initial_q):
            raise ValueError('Seven measured initial positions required')
        self.limit_rate, self.cutoff_hz = limit_rate, cutoff_hz
        self.motion = self.thread = None
        self.error = None

    def prepare(self):
        if self.thread is not None and self.thread.is_alive():
            raise RuntimeError('Native control still running')
        if self.error is not None:
            raise RuntimeError('Native driver fault is latched')
        self.motion = self.motion_factory()
        return self.initial_q

    def start(self):
        def run():
            try:
                self.robot.move(self.motion, asynchronous=False, limit_rate=self.limit_rate,
                                cutoff_frequency=self.cutoff_hz)
            except Exception as error:
                self.error = repr(error)
        self.thread = threading.Thread(target=run, name='droid-native-control', daemon=True)
        self.thread.start()

    @property
    def joined(self):
        if self.thread is None:
            return False
        if self.thread.is_alive():
            return False
        self.thread.join(0)
        return True


class NativeDroidBackend:
    single_session = True
    hardware_executed = True
    name = 'franky-droid-candidate'

    def __init__(self, executor, driver, hand, *, clock, startup_timeout_ns,
                 consumption_timeout_ns, stop_timeout_ns, state_timeout_ns):
        if any(type(v) is not int or v <= 0 for v in
               (startup_timeout_ns, consumption_timeout_ns, stop_timeout_ns, state_timeout_ns)):
            raise ValueError('Explicit native startup/consumption/stop timeouts required')
        self.executor, self.driver, self.hand, self.clock = executor, driver, hand, clock
        self.state_timeout = state_timeout_ns
        self.startup_timeout = startup_timeout_ns
        self.consumption_timeout, self.stop_timeout = consumption_timeout_ns, stop_timeout_ns
        self.arm = EncoderState(tuple(driver.initial_q), (0.,)*7)
        self.estimate = None
        self._native = 1
        self.execution = (None, None)
        self._started = self._stopping = False
        self._fault = 0
        self._row_id = 0
        self._row_key = None
        self._row_target = None
        self._expiry = None
        self._pending = None
        self._last_receipt_tick = 0
        self._last_sample_tick = 0
        self._last_ring_tick = 0
        self._row_map = {}
        self._deadlines = {}
        self._last_status = None
        self._stop_since = None
        self._stop_reported = False

    def preflight(self):
        if self.estimate is None or self._fault:
            raise RuntimeError('Registered clock and fault-free backend required')
        self.validate_state()
        return {'q': self.arm.q}

    def validate_state(self):
        stamp = self._last_status.monotonic_ns if self._last_status else self.driver.initial_timestamp_ns
        now = self.clock()
        if not 0 <= now-stamp <= self.state_timeout:
            if not self._stopping:
                self._fault = self._fault or 207
            raise ValueError('Native encoder state is stale or from a future clock')

    def plan(self, snapshot):
        # Fresh-session use requires fresh measured state from an external qualified
        # factory. This candidate is single-session; never reuse stale initial_q.
        if self._started:
            raise RuntimeError('Construct a fresh native backend for a new session')
        self.driver.prepare()

    def start(self, snapshot):
        now = self.clock()
        self._row_id = 1
        self._row_target = tuple(snapshot['q'])
        self._expiry = now+self.startup_timeout
        self.driver.motion.publish(self._row_target, self._row_id, self._expiry)
        self._row_map[1] = (None, None)
        self._pending = (1, now)
        self.driver.start()
        self.hand.start()
        self._started = True

    def send_reference(self, payload):
        chunk = DroidActionChunk.from_dict(payload)
        self._native += 1
        deadline = self.estimate.deadline_to_server(chunk.valid_until_ns)
        self.executor.submit(self._native, chunk.values, deadline)
        # Only current/pending chunks are retained, never an unbounded command log.
        keep = {c.sequence for c in (self.executor.current, self.executor.pending) if c}
        if self._row_key is not None:
            keep.add(self._row_key[0])
        self._deadlines = {k:v for k,v in self._deadlines.items() if k in keep}
        self._deadlines[self._native] = deadline
        return self._native

    def request_stop(self, reason):
        self.executor.stop(reason)
        self.execution = (None, None)
        if not self._stopping:
            self._stopping = True
            self._stop_since = self.clock()
            if self.driver.motion is not None:
                self.driver.motion.request_stop()
            try:
                self.hand.request_stop()
            except Exception:
                self._fault = self._fault or 205

    def check_session_admission(self):
        if self._started or self._fault:
            raise ValueError('Native backend is single-session; no fault reset or stale-state reuse')

    def reset_session(self, executor):
        self.check_session_admission()
        self.executor = executor
        self.estimate = None

    def update(self, now_ns, dt_s, step, service_tick):
        if not self._started:
            if (self._stopping and not self._stop_reported
                    and now_ns-self._stop_since >= self.stop_timeout):
                self._stop_reported = True
                return BackendUpdate(fault=206, stop_evidence=dict(joined=False, stop_finished=False,
                    stop_at_rest=False, stop_fault=206, dropped_samples=0))
            return BackendUpdate()
        acknowledgements = []
        motion = self.driver.motion
        samples = motion.drain()
        status = motion.status()
        # status preserves terminal evidence when the bounded ring overflows.
        observed_ns = self.clock()  # Drain can observe callbacks newer than executor's now_ns.
        for sample in samples:
            if sample.tick <= self._last_ring_tick:
                self._fault = self._fault or 202
            self._last_ring_tick = max(self._last_ring_tick, sample.tick)
        for sample in [*samples, status]:
            if sample.tick >= self._last_sample_tick and sample.tick:
                self._last_status = sample
                self._last_sample_tick = sample.tick
                self.arm.q, self.arm.dq = tuple(sample.q), tuple(sample.dq)
            self._fault = self._fault or sample.fault or sample.stop_fault
            receipt = sample.consumed
            if receipt.tick > self._last_receipt_tick:
                if (receipt.row_sequence not in self._row_map or receipt.monotonic_ns > observed_ns
                        or receipt.publication_sequence <= 0 or receipt.tick > sample.tick
                        or receipt.monotonic_ns > sample.monotonic_ns):
                    self._fault = self._fault or 202
                    continue
                sequence, row = self._row_map.pop(receipt.row_sequence)
                self._last_receipt_tick = receipt.tick
                if self._pending and self._pending[0] == receipt.row_sequence:
                    if receipt.monotonic_ns < self._pending[1]:
                        self._fault = self._fault or 202
                        continue
                    self._pending = None
                if sequence is not None and not self._stopping and not self._fault:
                    self.execution = (sequence, row)
                    if row == 0:
                        acknowledgements.append(Consumption(sequence, receipt.tick, receipt.monotonic_ns))
        if self.driver.error is not None:
            self._fault = self._fault or 201
        if self.hand.fault:
            self._fault = self._fault or 205
        if not self._stopping:
            try:
                self.validate_state()
            except ValueError:
                pass
        if not self._stopping and self._pending and now_ns-self._pending[1] >= self.consumption_timeout:
            self._fault = self._fault or 203
        if not self._stopping and not self._fault and not step.stop_reason:
            if step.sequence is not None:
                key = (step.sequence, step.row)
                if key != self._row_key:
                    if (self._row_key is not None and
                            (step.row != (self._row_key[1]+1 if step.sequence == self._row_key[0] else 0))):
                        self._fault = self._fault or 203
                    # Never silently skip a row whose native consumption was unconfirmed.
                    if self._pending:
                        if self._row_key is not None:
                            self._fault = 203
                    elif self._row_key is None and step.row != 0:
                        self._fault = 203
                    elif not self._fault:
                        self._row_id += 1
                        self._row_key = key
                        self._row_target = tuple(step.target)
                        self._expiry = self._deadlines[step.sequence]
                        self._row_map[self._row_id] = key
                        self._pending = (self._row_id, now_ns)
            elif step.target is not None and tuple(step.target) != self._row_target:
                self._fault = 203  # Sparse tick would change an unannounced held row.
            if (not self._fault and not self._pending and self._row_key is not None
                    and self._expiry == self._deadlines[self._row_key[0]]):
                # Only an already-consumed target may receive bounded hold time.
                # Use a new native identity: the original row's immutable expiry
                # must prevent its FIRST application after the chunk deadline.
                if self.clock() >= self._expiry:
                    self._fault = 204
                else:
                    self._row_id += 1
                    self._expiry += self.executor.hold_timeout
                    self._row_map[self._row_id] = (None, None)
                    self._pending = (self._row_id, now_ns)
            if not self._fault:
                # Producer liveness is refreshed only after the executor tick and
                # successful telemetry checks. Same-row expiry is immutable.
                try:
                    motion.publish(self._row_target, self._row_id, self._expiry)
                except Exception:
                    self._fault = self._fault or 204
        if self._fault and not self._stopping:
            self.request_stop('native_fault')
        evidence = None
        if self._stopping and not self._stop_reported:
            terminal = self.driver.joined and self.hand.quiescent
            timed_out = now_ns-self._stop_since >= self.stop_timeout
            if terminal or timed_out:
                s = self._last_status
                evidence = dict(joined=self.driver.joined, stop_finished=bool(s and s.stop_finished),
                                stop_at_rest=bool(s and s.stop_at_rest),
                                stop_fault=(s.stop_fault if s else 206) or (206 if self.driver.error or not self.hand.quiescent or self.hand.fault else 0),
                                dropped_samples=0 if s is None else s.dropped)
                self._stop_reported = True
        return BackendUpdate(tuple(acknowledgements), self._fault,
                             0 if self._last_status is None else self._last_status.dropped, evidence)


def verify_native_artifact(pin_path):
    """Lazy, read-only artifact check. Never constructs/contacts a device.

    Pin approval and per-unit physical qualification are separate requirements.
    Use the isolated launcher; loader overrides are not permitted here.
    """
    import hashlib
    from importlib import import_module
    from importlib.metadata import version
    import json
    import os
    from pathlib import Path
    import sys
    if os.environ.get('LD_LIBRARY_PATH') or os.environ.get('LD_PRELOAD'):
        raise ValueError('Clear native loader overrides before loading the DROID candidate')
    pin = json.loads(Path(pin_path).read_text())
    if version('franky-control') != pin['franky_version']:
        raise ValueError('Franky candidate version mismatch')
    module = import_module('franky')
    site = Path(module.__file__).resolve().parent.parent
    extension = Path(sys.modules['franky._franky'].__file__).resolve()
    if extension.parent != site/'franky':
        raise ValueError('Franky extension loaded from a different installation')
    for name, expected in pin['files'].items():
        path = (site/name).resolve()
        path.relative_to(site)
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError(f'Native artifact hash mismatch: {name}')
    if not hasattr(module, 'DroidJointImpedanceMotion'):
        raise ValueError('DROID native API missing')
    return module
