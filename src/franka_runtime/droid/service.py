"""DROID chunk session service over TCP (contracts.wire framing), mock backend only in D4.

One client at a time, from an allow-listed peer that presents the configured session
token. SessionAuthority owns leases, sequencing, receipts and latched faults; the
executor owns row timing; the backend owns motion. Faults cannot be reset through
this API. Nothing starts until ``serve()`` is called; importing opens nothing.

Protocol (request ``{"protocol", "id", "op", "payload"}`` → ``{"protocol", "id", "ok", "result"|"error"}``):
hello, clock_probe, open, clock_register, start, submit, state, receipts, release, close.
"""
from dataclasses import asdict
import hmac
import os
from pathlib import Path
import socket
import threading
import time
import uuid

import contracts
from contracts.clock import ClockOffsetEstimate, ClockProbe
from contracts.droid import DroidActionChunk
from contracts.droid_session import DroidSessionOpen, DroidSessionReady, capability_id, validate_chunk_command
from contracts.franka_state import FrankaStateSample
from contracts.sessions import CommandEnvelope, LeaseRelease, payload_sha256
from contracts.wire import PROTOCOL_VERSION, read_message, write_message

import franka_runtime
from ..service.authority import AuthorityError, SessionAuthority
from .checks import check_rows
from .executor import UNDERRUN, ChunkExecutor
from .gripper import GripperIntent
from .mock import MockArm, MockHand
from .workspace import from_settings as workspace_from_settings

# Integer native fault codes reported through SessionAuthority.poll(native_fault=...).
FAULT_CODES = {UNDERRUN: 101, "heartbeat_timeout": 102, "client_disconnected": 103}
MOCK_STOP_EVIDENCE = {"joined": True, "stop_finished": True, "stop_at_rest": True, "stop_fault": 0, "dropped_samples": 0}


def host_clock_id():
    try:
        boot = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    except OSError:
        boot = "unknown-boot"
    return f"{socket.gethostname()}:{boot}:CLOCK_MONOTONIC"


class MockDroidBackend:
    """SessionAuthority backend: publishes accepted chunks to the executor; stops the mock arm."""
    hardware_executed = False

    def __init__(self, executor, arm):
        self.executor, self.arm = executor, arm
        self.estimate = None
        self._native = 1  # Native startup consumes sequence 1, as in the existing backends.
        self.stop_reasons = []

    def preflight(self):
        if self.estimate is None:
            raise RuntimeError("No registered clock estimate")
        return {"q": tuple(self.arm.q)}

    def plan(self, snapshot):
        pass

    def start(self, snapshot):
        pass

    def send_reference(self, payload):
        chunk = DroidActionChunk.from_dict(payload)
        self._native += 1
        self.executor.submit(self._native, chunk.values, self.estimate.deadline_to_server(chunk.valid_until_ns))
        return self._native

    def request_stop(self, reason):
        self.stop_reasons.append(reason)
        self.executor.stop(reason)
        self.arm.stopping = True


class DroidRuntimeService:
    def __init__(self, config, spec, *, clock=time.monotonic_ns, clock_id=None, token=None):
        settings = config.require_runnable()
        sha = payload_sha256(spec)
        if sha not in settings["allowed_contracts"] or spec.robot.robot_id != settings["robot_id"]:
            raise ValueError("Contract is not allow-listed for this robot")
        if capability_id(spec).split(":")[0] != settings["capability_prefix"]:
            raise ValueError("Capability prefix mismatch")
        token = os.environ.get(settings["token_env"]) if token is None else token
        if not token:
            raise ValueError(f"Session token missing; set {settings['token_env']}")
        if settings["open_width_m"] != spec.gripper.open_width_m:
            raise ValueError("Configured gripper width differs from the contract's open_width_m")
        self.settings, self.spec, self.contract_sha256, self._token = settings, spec, sha, token
        self.clock, self.clock_id = clock, clock_id or host_clock_id()
        self.row_period_ns = round(1e9/spec.action.policy_hz)
        self.executor = ChunkExecutor(self.row_period_ns, settings["hold_timeout_ns"])
        self.arm = MockArm(settings["mock_start_q"], settings["max_velocity"], settings["max_acceleration"])
        self.hand = MockHand(settings["open_width_m"], settings["gripper_speed_m_s"])
        self.backend = MockDroidBackend(self.executor, self.arm)
        self.workspace = workspace_from_settings(settings["workspace"], settings["config_dir"])
        self.workspace.check_configuration(self.arm.q, "start pose")
        self.authority = SessionAuthority(self.backend, spec, self._validate_payload, clock_id=self.clock_id,
            command_timeout_ns=settings["command_timeout_ns"], clock=clock, max_lease_ns=settings["max_lease_ns"],
            capability_prefix=settings["capability_prefix"], robot_id=settings["robot_id"])
        self.intent = GripperIntent(spec.gripper, self.hand.width)
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._session = None
        self._probes = set()
        self._tick = 0
        self._sample_sequence = 0
        self._last_heartbeat = None
        self._fault_reported = False
        self.peak_speed = [0.]*7
        self.listener = None

    # ---- payload validation (called by SessionAuthority at submit and again at dispatch)
    def _validate_payload(self, payload):
        chunk = DroidActionChunk.from_dict(payload)
        if chunk.action_spec != self.spec.action or len(chunk.values) > self.spec.open_loop_horizon:
            raise ValueError("Chunk incompatible with the contract")
        peak = check_rows(chunk.values, self.arm.q, self.settings, self.row_period_ns)
        self.workspace.check_rows(chunk.values, self.arm.q)
        self.peak_speed = [max(a, b) for a, b in zip(self.peak_speed, peak)]

    # ---- control loop
    def control_step(self, now_ns, dt_s):
        with self._lock:
            self._tick += 1
            if self._session and self._session.get("started") and self.authority.fault is None \
                    and not self.authority.stopping and now_ns-self._last_heartbeat > self.settings["heartbeat_timeout_ns"]:
                self._fault("heartbeat_timeout")
            step = self.executor.tick(now_ns)
            if step.started is not None and self._session:
                try:
                    self.authority.acknowledge(lease_id=self._session["lease"].lease_id, native_sequence=step.started,
                                               native_tick=self._tick, timestamp_ns=now_ns, clock_id=self.clock_id)
                except AuthorityError:
                    pass  # The authority has already latched the fault (telemetry order/identity).
                if self.authority.fault is not None:
                    self.executor.stop(self.authority.fault.code)
            if step.row_started and step.closure is not None:
                action = self.intent.update(step.closure)
                if action:
                    self.hand.command(action)
            if step.stop_reason == UNDERRUN and not self._fault_reported:
                self._fault(UNDERRUN)
            self.arm.advance(dt_s, None if step.stop_reason else step.target)
            self.hand.advance(dt_s)
            self.authority.poll()
            if self.authority.stopping and self.arm.at_rest:
                self.authority.finish_stop(MOCK_STOP_EVIDENCE)
            self._last_step = step

    def _fault(self, reason):
        self._fault_reported = True
        self.executor.stop(reason)
        self.authority.poll(native_fault=FAULT_CODES[reason])

    def state_sample(self):
        with self._lock:
            step = getattr(self, "_last_step", None)
            fault = self.authority.fault
            lease = self._session["lease"].lease_id if self._session else None
            executing = step is not None and step.sequence is not None and lease is not None
            self._sample_sequence += 1
            return FrankaStateSample(self.settings["robot_id"], self.clock_id, self.clock(), self._sample_sequence,
                tuple(self.arm.q), tuple(self.arm.dq), self.hand.width, self.hand.moving, False,
                fault is None and not self.authority.stopping, lease,
                step.sequence if executing else None, step.row if executing else None,
                None if fault is None else fault.code)

    # ---- request handlers
    def _hello(self, payload):
        return {"server_clock_id": self.clock_id, "robot_id": self.settings["robot_id"],
                "contract_sha256": self.contract_sha256, "versions": self._versions()}

    def _versions(self):
        return {"franka-runtime": franka_runtime.__version__, "contracts": contracts.__version__, "backend": "mock"}

    def _clock_probe(self, payload):
        received = self.clock()
        with self._lock:
            if self._session and self._session.get("started"):
                self._last_heartbeat = received
        sent = self.clock()
        with self._lock:
            self._probes.add((received, sent))
        return {"server_clock_id": self.clock_id, "server_receive_ns": received, "server_send_ns": sent}

    def _open(self, payload):
        if self._session is not None:
            raise AuthorityError("A session is already open on this connection")
        if not hmac.compare_digest(str(payload.get("token", "")), self._token):
            raise AuthorityError("Invalid session token")
        request = DroidSessionOpen.from_dict(payload["session"])
        if request.contract_sha256 != self.contract_sha256:
            raise AuthorityError("Contract is not allow-listed")
        with self._lock:
            lease = self.authority.acquire(request.lease_request, episode_id=f"droid-{uuid.uuid4().hex}")
            # The authority admits a new lease only after the previous stop completed without a latched
            # fault, so per-session execution state starts fresh here.
            self.executor = self.backend.executor = ChunkExecutor(self.row_period_ns, self.settings["hold_timeout_ns"])
            self.backend.estimate = None
            self.arm.stopping = False
            self.intent = GripperIntent(self.spec.gripper, self.hand.width)
            self._last_step = None
            self._session = {"request": request, "lease": lease, "estimate": None, "started": False}
            self._fault_reported = False
        ready = DroidSessionReady(lease, self.contract_sha256, self.authority.capability_id,
                                  self.settings["open_width_m"], self.settings["hold_timeout_ns"],
                                  self.settings["max_uncertainty_ns"], self._versions())
        return ready.to_dict()

    def _session_or_fail(self):
        if self._session is None:
            raise AuthorityError("No open session")
        return self._session

    def _clock_register(self, payload):
        session = self._session_or_fail()
        probes = tuple(ClockProbe.from_dict(p) for p in payload["probes"])
        if len(probes) < self.settings["min_probes"]:
            raise ValueError("Too few clock probes")
        if any((p.server_receive_ns, p.server_send_ns) not in self._probes for p in probes):
            raise ValueError("Probe server stamps were not issued by this runtime")
        if any(p.client_clock_id != session["request"].client_clock_id or p.server_clock_id != self.clock_id
               for p in probes):
            raise ValueError("Probe clock identities differ from the session")
        estimate = ClockOffsetEstimate.from_probes(probes, max_uncertainty_ns=self.settings["max_uncertainty_ns"])
        with self._lock:
            session["estimate"] = self.backend.estimate = estimate
        return estimate.to_dict()

    def _start(self, payload):
        session = self._session_or_fail()
        with self._lock:
            self.authority.start(session["lease"])
            session["started"] = True
            self._last_heartbeat = self.clock()
        return {"state": self.authority.state}

    def _submit(self, payload):
        session = self._session_or_fail()
        envelope = CommandEnvelope.from_dict(payload["envelope"])
        chunk = DroidActionChunk.from_dict(payload["chunk"])
        with self._lock:
            validate_chunk_command(self.spec, envelope, chunk, lease=session["lease"], estimate=session["estimate"],
                                   now_ns=self.clock(), next_sequence=envelope.sequence,
                                   capability=self.authority.capability_id)
            receipt = self.authority.submit(envelope, chunk.to_dict())
            if receipt.status == "accepted":
                dispatched = self.authority.dispatch()
                if not isinstance(dispatched, int):
                    receipt = dispatched
        return asdict(receipt)

    def _state(self, payload):
        return self.state_sample().to_dict()

    def _receipts(self, payload):
        with self._lock:
            fault = self.authority.fault
            return {"receipts": [asdict(r) for r in self.authority.drain_receipts()],
                    "fault": None if fault is None else asdict(fault), "authority_state": self.authority.state,
                    "gripper_commands": list(self.hand.commands), "peak_implied_speed_rad_s": list(self.peak_speed)}

    def _release(self, payload):
        session = self._session_or_fail()
        with self._lock:
            self.authority.release(LeaseRelease(session["lease"], self.clock(), str(payload.get("reason", "release"))))
        return {"authority_state": self.authority.state}

    HANDLERS = {"hello": "_hello", "clock_probe": "_clock_probe", "open": "_open", "clock_register": "_clock_register",
                "start": "_start", "submit": "_submit", "state": "_state", "receipts": "_receipts",
                "release": "_release"}

    # ---- network and threads
    def bind(self):
        settings = self.settings
        self.listener = socket.create_server((settings["bind_host"], settings["port"]))
        self.listener.settimeout(0.2)
        return self.listener.getsockname()[1]

    def serve(self):
        """Run the control loop and accept clients until ``shutdown``. Call ``bind`` first."""
        control = threading.Thread(target=self._control_loop, name="droid-control", daemon=True)
        control.start()
        try:
            while not self._stop.is_set():
                try:
                    connection, peer = self.listener.accept()
                except socket.timeout:
                    continue
                if peer[0] not in self.settings["allowed_peers"] or self._session is not None:
                    connection.close()
                    continue
                self._serve_client(connection)
        finally:
            self._stop.set()
            control.join(2)
            self.listener.close()

    def _control_loop(self):
        period = self.settings["control_period_ns"]
        previous = self.clock()
        while not self._stop.is_set():
            time.sleep(period/1e9)
            now = self.clock()
            self.control_step(now, (now-previous)/1e9)
            previous = now

    def _serve_client(self, connection):
        connection.settimeout(1.0)
        stream = connection.makefile("rwb", buffering=0)
        try:
            while not self._stop.is_set():
                try:
                    request = read_message(stream)
                except socket.timeout:
                    continue
                except (ConnectionError, ValueError, OSError):
                    break
                response = {"protocol": PROTOCOL_VERSION, "id": request.get("id")}
                op = request.get("op")
                if request.get("protocol") != PROTOCOL_VERSION or op not in (*self.HANDLERS, "close"):
                    response.update(ok=False, error="Unsupported protocol or operation")
                elif op == "close":
                    response.update(ok=True, result={})
                    write_message(stream, response)
                    break
                else:
                    try:
                        result = getattr(self, self.HANDLERS[op])(request.get("payload") or {})
                        response.update(ok=True, result=result)
                    except (ValueError, KeyError, TypeError, AuthorityError, RuntimeError) as error:
                        response.update(ok=False, error=f"{type(error).__name__}: {error}")
                write_message(stream, response)
        finally:
            self._end_session()
            stream.close()
            connection.close()

    def _end_session(self):
        with self._lock:
            if self._session is not None and self._session.get("started") \
                    and self.authority.state in ("running", "ready") and not self.authority.stopping:
                self._fault("client_disconnected")
            self._session = None
            self._probes.clear()

    def shutdown(self):
        self._stop.set()
