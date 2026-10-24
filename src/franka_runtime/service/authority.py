"""Non-RT local motion command authority; native guards/watchdogs remain final.

All mutation is serialized. This is an in-process API, not network authentication.
No imports or connections to vendor software occur here.
"""
from collections import deque
from copy import deepcopy
from dataclasses import is_dataclass, replace
import threading
import time
import uuid
from contracts.sessions import (SessionLease, LeaseRequest, LeaseRelease, CommandEnvelope,
                                CommandReceipt, FaultReport, payload_sha256)


class AuthorityError(RuntimeError):
    pass


class SessionAuthority:
    def __init__(self, backend, specification, validate_payload, *, clock_id,
                 command_timeout_ns, clock=None, max_lease_ns=600_000_000_000,
                 receipt_capacity=4096, id_factory=None, expect_application=True, capability_prefix="m4-v1",
                 robot_id=None, receipt_sink=None):
        # Any frozen contract dataclass (M4, PPO, DROID, ...); its hash forms the capability.
        if (not is_dataclass(specification) or isinstance(specification,type)
                or not specification.__dataclass_params__.frozen or not callable(validate_payload)):
            raise ValueError('Typed session specification and payload validator required')
        robot_id=getattr(specification,'robot_id',None) if robot_id is None else robot_id
        if not isinstance(robot_id,str) or not robot_id.strip():
            raise ValueError('Explicit robot identity required')
        self.robot_id=robot_id
        if not isinstance(clock_id,str) or not clock_id:
            raise ValueError('Explicit host/boot/clock identity required')
        for name,value in [('command_timeout_ns',command_timeout_ns),('max_lease_ns',max_lease_ns),
                           ('receipt_capacity',receipt_capacity)]:
            if type(value) is not int or value <= 0:
                raise ValueError(f'{name} must be a positive integer')
        if type(expect_application) is not bool:
            raise ValueError('expect_application must be boolean')
        if not isinstance(capability_prefix,str) or not capability_prefix.strip():
            raise ValueError('Capability prefix required')
        self.receipt_sink = receipt_sink
        self.expect_application = expect_application
        self.backend, self.specification, self.validate_payload = backend, specification, validate_payload
        self.capability_id = capability_prefix + ':' + payload_sha256(specification)
        self.clock_id, self.command_timeout_ns, self.max_lease_ns = clock_id,command_timeout_ns,max_lease_ns
        self._clock = clock or (lambda:time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW))
        self._id = id_factory or (lambda:uuid.uuid4().hex)
        self._lock = threading.RLock()
        self._receipts = deque(maxlen=receipt_capacity)
        self.dropped_receipts = 0
        self.state = 'idle'
        self.lease = None
        self.fault = None
        self._next_sequence = 1
        self._pending = None
        self._delivered = {}
        self._last_native_tick = -1
        self._last_native_timestamp = -1
        self._stopping = False

    @property
    def stopping(self):
        return self._stopping

    def _now(self):
        value=self._clock()
        if type(value) is not int or value < 0:
            raise AuthorityError('Invalid local clock')
        return value

    def _receipt(self, envelope, status, reason='', *, native_tick=None, timestamp_ns=None):
        receipt=CommandReceipt(envelope.lease.session_id,envelope.lease.episode_id,
                               envelope.lease.lease_id,envelope.sequence,status,self.clock_id,
                               self._now() if timestamp_ns is None else timestamp_ns,native_tick,reason)
        if len(self._receipts)==self._receipts.maxlen:
            self.dropped_receipts += 1
            self._latch('receipt_overflow')
        self._receipts.append(receipt)
        if self.receipt_sink is not None:
            try:
                recorded = self.receipt_sink(receipt)
            except Exception:
                recorded = False
            if recorded is not True:
                self._latch("receipt_recording_failed")
        return receipt

    def drain_receipts(self):
        with self._lock:
            result=list(self._receipts);self._receipts.clear();return result

    def _latch(self, code):
        if self.fault is not None:
            return
        self.state='fault'; self._stopping=True
        if self.lease is not None:
            self.fault=FaultReport(self.lease.session_id,self.lease.episode_id,self.clock_id,self._now(),code,'requested')
        try:
            self.backend.request_stop(code)
        except Exception:
            if self.fault is not None:
                self.fault=replace(self.fault,stop_state='unavailable')

    def acquire(self, request, *, episode_id):
        with self._lock:
            if not isinstance(request, LeaseRequest):
                raise AuthorityError('Typed lease request required')
            now=self._now()
            if (request.robot_id != self.robot_id or request.clock_id != self.clock_id or
                    not request.requested_ns <= now <= request.requested_ns+self.command_timeout_ns):
                raise AuthorityError('Lease request identity/clock/time mismatch')
            if self.fault is not None or self._stopping or self.state not in ('idle','complete'):
                raise AuthorityError('Previous owner/fault/stop still active')
            self.lease=SessionLease(request.robot_id,self._id(),episode_id,request.owner_id,self._id(),
                                    self.clock_id,now,now+min(request.duration_ns,self.max_lease_ns))
            self.state='ready';self._next_sequence=1;self._pending=None;self._delivered.clear()
            self._last_native_tick=-1;self._last_native_timestamp=-1
            return self.lease

    def _active(self, lease):
        if lease != self.lease:
            raise AuthorityError('Wrong lease/owner/session/episode')
        if self.fault is not None or self._stopping:
            raise AuthorityError('Fault or stop is latched')
        try:
            lease.validate_at(self._now(),self.clock_id)
        except ValueError as error:
            self._latch('lease_expired');raise AuthorityError(str(error)) from error

    def start(self, lease):
        with self._lock:
            self._active(lease)
            if self.state!='ready':raise AuthorityError('Session is not ready')
            try:
                snapshot=self.backend.preflight()
                self.backend.plan(snapshot)
                self._active(lease)
                self.backend.start(snapshot)
            except Exception:
                self._latch('startup_failed');raise
            self.state='running'
            return snapshot

    def submit(self, envelope, payload):
        with self._lock:
            if not isinstance(envelope,CommandEnvelope):
                raise AuthorityError('Typed command envelope required')
            try:
                payload=deepcopy(payload)
                self._active(envelope.lease)
                if self.state!='running':raise AuthorityError('Session is not running')
                envelope.validate_for(self.lease,now_ns=self._now(),clock_id=self.clock_id,
                    next_sequence=self._next_sequence,capability_id=self.capability_id,payload=payload)
                if self._now()-envelope.source_observation_ns > self.command_timeout_ns:
                    raise ValueError('Source observation is stale')
                if envelope.deadline_ns-envelope.source_observation_ns > self.command_timeout_ns:
                    raise ValueError('Command window exceeds configured watchdog interval')
                self.validate_payload(payload)
                if self._pending is not None:
                    raise AuthorityError('One queued command permitted; no overwrite')
                self._pending=(envelope,deepcopy(payload))
                self._next_sequence+=1
                return self._receipt(envelope,'accepted')
            except (ValueError,AuthorityError) as error:
                return self._receipt(envelope,'rejected',str(error))

    def dispatch(self):
        """Recheck immediately before publishing to the native mailbox."""
        with self._lock:
            self.poll()
            if self._pending is None:return None
            envelope,payload=self._pending;self._pending=None
            try:
                payload=deepcopy(payload)
                self._active(envelope.lease)
                now=self._now()
                if not envelope.created_ns <= now < envelope.deadline_ns:
                    raise AuthorityError('Queued command expired')
                self.validate_payload(payload)
                native_sequence=self.backend.send_reference(payload)
                if type(native_sequence) is not int or native_sequence < 1 or native_sequence in self._delivered:
                    raise AuthorityError('Invalid native sequence')
                if self.expect_application:
                    self._delivered[native_sequence]=(envelope,now)
                if self._now() >= envelope.deadline_ns:
                    raise AuthorityError('Publication deadline elapsed; native application unconfirmed')
                # Bound unacknowledged publications as well as the input mailbox.
                if len(self._delivered)>self._receipts.maxlen:
                    raise AuthorityError('Native acknowledgement overflow')
                return native_sequence
            except Exception as error:
                self._latch('dispatch_failed')
                return self._receipt(envelope,'rejected',str(error))

    def acknowledge(self, *, lease_id, native_sequence, native_tick, timestamp_ns, clock_id):
        """Use actual native telemetry, never the fact that publish() returned.

        Producer must supply samples from this attached run. The adapter drains
        all native samples in order; a later sequence rejects superseded mail.
        """
        with self._lock:
            if not self.expect_application:
                raise AuthorityError('Offline shadow never confirms application')
            if self.lease is None or lease_id!=self.lease.lease_id or clock_id!=self.clock_id:
                raise AuthorityError('Wrong telemetry run/clock')
            if any(type(v) is not int or v<0 for v in (native_sequence,native_tick,timestamp_ns)):
                raise AuthorityError('Invalid native telemetry identity/time')
            if (native_tick<=self._last_native_tick or timestamp_ns<self._last_native_timestamp or
                    timestamp_ns>self._now()):
                self._latch('telemetry_order');raise AuthorityError('Nonmonotonic/future telemetry')
            self._last_native_tick=native_tick;self._last_native_timestamp=timestamp_ns
            record=self._delivered.pop(native_sequence,None)
            if record is None:return None
            envelope,published_ns=record
            if (self.fault is not None or self._stopping or
                    not published_ns<=timestamp_ns<envelope.deadline_ns):
                self._latch('application_deadline')
                return self._receipt(envelope,'rejected','Native application not confirmed within active deadline')
            # Native references are monotonic; older unobserved mail was superseded.
            for key in list(self._delivered):
                if key<native_sequence:
                    old,_=self._delivered.pop(key)
                    self._receipt(old,'rejected','Superseded before native application receipt')
            return self._receipt(envelope,'applied',native_tick=native_tick,timestamp_ns=timestamp_ns)

    def poll(self, *, native_fault=0, dropped_samples=0):
        with self._lock:
            if native_fault:
                self._latch(f'native_fault:{native_fault}')
            if dropped_samples:
                self._latch('native_logger_overflow')
            if self.lease is not None and not self._stopping and self.state in ('ready','running'):
                if self._now()>=self.lease.expires_ns:
                    self._latch('lease_expired')
            return self.fault

    def release(self, request):
        with self._lock:
            if not isinstance(request,LeaseRelease) or request.lease!=self.lease:
                raise AuthorityError('Release must match current lease')
            self._active(request.lease)
            self._stopping=True;self.state='stopping'
            try:self.backend.request_stop(request.reason)
            except Exception:
                self._stopping=False;self._latch('stop_unavailable');raise
            self._reject_outstanding('Session released; application unconfirmed')

    def _reject_outstanding(self, reason):
        if self._pending is not None:
            e,_=self._pending;self._pending=None;self._receipt(e,'rejected',reason)
        for e,_ in list(self._delivered.values()):self._receipt(e,'rejected',reason)
        self._delivered.clear()

    def finish_stop(self, evidence):
        with self._lock:
            if not self._stopping:raise AuthorityError('No stop requested')
            self._reject_outstanding('Stop completed or failed before application confirmation')
            ok=(evidence.get('joined') is True and evidence.get('stop_finished') is True and
                evidence.get('stop_at_rest') is True and evidence.get('stop_fault')==0 and
                evidence.get('dropped_samples')==0)
            if not ok:self._latch('stop_failed')
            if self.fault is not None:
                self.fault=replace(self.fault,stop_state='complete' if ok else 'timeout')
                self.state='fault'
            else:self.state='complete'
            self._stopping=False
            return ok
