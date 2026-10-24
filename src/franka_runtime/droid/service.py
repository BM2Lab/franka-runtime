"""DROID chunk session service with an injected backend evidence interface.

The public CLI remains mock-only; native integration is an offline candidate.

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
from .backend import BackendUpdate, Consumption
from .admission import _AdmittedPhysicalConfig
from .shadow_config import ShadowRuntimeConfig
from .checks import check_rows
from .executor import UNDERRUN, ChunkExecutor
from .gripper import GripperIntent
from .mock import MockArm, MockHand
from .workspace import from_settings as workspace_from_settings

# Integer native fault codes reported through SessionAuthority.poll(native_fault=...).
FAULT_CODES = {UNDERRUN: 101, "heartbeat_timeout": 102, "client_disconnected": 103}
# No row starts this close to its chunk deadline: the consumed row must be extended into the hold
# before its native expiry, across a supervisor pause (13.7 ms measured, D8 2026-10-24) plus a tick.
ROW_START_MARGIN_NS = 50_000_000
# Periodic state samples are recorded at this rate; every authority/fault change is always recorded.
TELEMETRY_PERIOD_NS = 100_000_000
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
    name = "mock"

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

    def reset_session(self, executor):
        self.executor = executor
        self.estimate = None
        self.arm.stopping = False

    def update(self, now_ns, dt_s, step, service_tick):
        self.arm.advance(dt_s, None if step.stop_reason else step.target)
        self.hand.advance(dt_s)
        consumed = () if step.started is None else (Consumption(step.started, service_tick, now_ns),)
        evidence = MOCK_STOP_EVIDENCE if self.arm.stopping and self.arm.at_rest else None
        return BackendUpdate(consumed, stop_evidence=evidence)


class DroidRuntimeService:
    def __init__(self, config, spec, *, clock=time.monotonic_ns, clock_id=None, token=None, backend_factory=None, recorder=None):
        physical = isinstance(config,_AdmittedPhysicalConfig)
        shadow = isinstance(config,ShadowRuntimeConfig)
        settings = config.require_runnable()
        if (physical or shadow) and (backend_factory is None or recorder is None or recorder.fault):
            raise ValueError("Physical composition requires backend and healthy started recording")
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
        self.recorder = recorder
        self.settings, self.spec, self.contract_sha256, self._token = settings, spec, sha, token
        self.clock, self.clock_id = clock, clock_id or host_clock_id()
        self.row_period_ns = round(1e9/spec.action.policy_hz)
        self.executor = ChunkExecutor(self.row_period_ns, settings["hold_timeout_ns"], ROW_START_MARGIN_NS)
        if backend_factory is None:
            self.arm = MockArm(settings["mock_start_q"], settings["max_velocity"], settings["max_acceleration"])
            self.hand = MockHand(settings["open_width_m"], settings["gripper_speed_m_s"])
            self.backend = MockDroidBackend(self.executor, self.arm)
            self.backend.hand = self.hand
        else:
            self.backend = backend_factory(self.executor, settings)
            # A mock config must never be a route to real hardware. The production
            # physical selector/schema/commissioning path is deliberately still closed.
            if self.backend.hardware_executed and not physical:
                raise ValueError("Physical DROID selection requires a commissioned physical config/factory")
            if physical and not self.backend.hardware_executed:
                raise ValueError("Physical composition cannot substitute a mock backend")
            if shadow and (not getattr(self.backend,"shadow",False) or self.backend.hardware_executed
                    or getattr(self.backend,"expect_application",True) is not False):
                raise ValueError("Shadow must use read-only backend without application receipts")
            if getattr(self.backend,"shadow",False) and not shadow:
                raise ValueError("Shadow backend requires explicit shadow configuration")
            self.arm, self.hand = self.backend.arm, self.backend.hand
        if hasattr(self.backend, "recorder"):
            self.backend.recorder = recorder
        self.workspace = workspace_from_settings(settings["workspace"], settings["config_dir"])
        self.workspace.check_configuration(self.arm.q, "start pose")
        self.authority = SessionAuthority(self.backend, spec, self._validate_payload, clock_id=self.clock_id,
            command_timeout_ns=settings["command_timeout_ns"], clock=clock, max_lease_ns=settings["max_lease_ns"],
            capability_prefix=settings["capability_prefix"], robot_id=settings["robot_id"],expect_application=not shadow,
            receipt_sink=None if recorder is None else lambda receipt:self._record("receipt",asdict(receipt)))
        self.intent = GripperIntent(spec.gripper, self.hand.width)
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._shutdown = threading.Event()
        self.shutdown_report = None
        self._shutdown_error = None
        self._last_stop_evidence = None
        self._session = None
        self._probes = set()
        self._tick = 0
        self._telemetry_key = self._telemetry_ns = None
        self._sample_sequence = 0
        self._last_heartbeat = None
        self._fault_reported = False
        self.peak_speed = [0.]*7
        self.listener = None
        if not self._record("service_created",dict(clock_id=self.clock_id,contract=spec.to_dict(),
                contract_sha256=sha,backend=self.backend.name,configuration=config.data,versions=self._versions())):
            raise ValueError("Recorder must be started and healthy before service construction")

    def _record(self, kind, payload):
        if self.recorder is None:
            return True
        return self.recorder.emit(kind,payload,timestamp_ns=self.clock())

    # ---- payload validation (called by SessionAuthority at submit and again at dispatch)
    def _validate_payload(self, payload):
        stage="state"
        try:
            if hasattr(self.backend, "validate_state"):
                self.backend.validate_state()
            stage="contract"
            chunk = DroidActionChunk.from_dict(payload)
            if chunk.action_spec != self.spec.action or len(chunk.values) > self.spec.open_loop_horizon:
                raise ValueError("Chunk incompatible with the contract")
            stage="row_limits"
            peak = check_rows(chunk.values, self.arm.q, self.settings, self.row_period_ns)
            stage="workspace"
            self.workspace.check_rows(chunk.values, self.arm.q)
            if hasattr(self.backend, "validate_targets"):
                stage="backend_geometry"
                self.backend.validate_targets(chunk.values)
            self.peak_speed = [max(a, b) for a, b in zip(self.peak_speed, peak)]
        except Exception as error:
            if getattr(self.backend,"shadow",False):
                self._record("shadow_acceptance_check",dict(passed=False,stage=stage,reason=str(error),
                    measured_q=list(self.arm.q),chunk_sha256=payload_sha256(payload),feedback=self.backend.feedback_info))
            raise
        if getattr(self.backend,"shadow",False):
            if not self._record("shadow_acceptance_check",dict(passed=True,stage="complete",workspace_passed=True,
                    measured_q=list(self.arm.q),chunk_sha256=payload_sha256(payload),feedback=self.backend.feedback_info)):
                self.authority.poll(native_fault=209)
                raise ValueError("Shadow acceptance recording unavailable")

    # ---- control loop
    def control_step(self, now_ns, dt_s):
        with self._lock:
            self._tick += 1
            if self.recorder is not None and self.recorder.fault:
                self.authority.poll(native_fault=209)
            if self._session and self._session.get("started") and self.authority.fault is None \
                    and not self.authority.stopping and now_ns-self._last_heartbeat > self.settings["heartbeat_timeout_ns"]:
                self._fault("heartbeat_timeout")
            step = self.executor.tick(now_ns)
            try:
                if step.row_started and step.closure is not None:
                    action = self.intent.update(step.closure)
                    if action:
                        if getattr(self.backend,"shadow",False):
                            if not self._record("shadow_would_gripper_command",dict(action=action,commands_sent=False)):
                                raise RuntimeError("Shadow intent recording failed")
                        else:
                            self.hand.command(action)
                if step.stop_reason == UNDERRUN and not self._fault_reported:
                    self._fault(UNDERRUN)
                update = self.backend.update(now_ns, dt_s, step, self._tick)
                self.authority.poll(native_fault=update.fault, dropped_samples=update.dropped)
                for consumed in update.consumed:
                    if self._session:
                        self.authority.acknowledge(lease_id=self._session["lease"].lease_id,
                            native_sequence=consumed.sequence, native_tick=consumed.tick,
                            timestamp_ns=consumed.timestamp_ns, clock_id=self.clock_id)
                if self.authority.stopping and update.stop_evidence is not None:
                    self._last_stop_evidence = dict(update.stop_evidence)
                    self._record("stop_evidence",self._last_stop_evidence)
                    self.authority.finish_stop(update.stop_evidence)
            except Exception:
                # Exceptions must latch authority and request stop, not kill the
                # supervisor silently. Native watchdog remains independent.
                self.authority.poll(native_fault=208)
                self.executor.stop("backend_exception")
            self._last_step = step
            fault = self.authority.fault
            key = (self.authority.state, None if fault is None else fault.code, self.authority.stopping)
            changed = key != self._telemetry_key
            if changed or self._telemetry_ns is None or now_ns-self._telemetry_ns >= TELEMETRY_PERIOD_NS:
                payload = dict(tick=self._tick, authority_state=self.authority.state, q=list(self.arm.q), dq=list(self.arm.dq),
                    hand=self.hand.snapshot() if hasattr(self.hand,"snapshot") else dict(width=self.hand.width,moving=self.hand.moving))
                if changed:
                    payload.update(change=True, stopping=self.authority.stopping,
                                   fault=None if fault is None else asdict(fault))
                if not self._record("supervisor", payload):
                    self.authority.poll(native_fault=209)
                self._telemetry_key, self._telemetry_ns = key, now_ns

    def _fault(self, reason):
        self._fault_reported = True
        self.executor.stop(reason)
        self.authority.poll(native_fault=FAULT_CODES[reason])

    def state_sample(self):
        with self._lock:
            step = getattr(self, "_last_step", None)
            feedback_valid = True
            if hasattr(self.backend, "validate_state"):
                try:
                    self.backend.validate_state()
                except ValueError:
                    feedback_valid = False
                    if (self.authority.state == "running" and not self.authority.stopping
                            and not getattr(self.backend, "awaiting_feedback", False)):
                        self.authority.poll(native_fault=207)
            fault = self.authority.fault
            lease = self._session["lease"].lease_id if self._session else None
            sequence, row = getattr(self.backend, "execution", (
                None if step is None else step.sequence, None if step is None else step.row))
            executing = sequence is not None and lease is not None
            self._sample_sequence += 1
            return FrankaStateSample(self.settings["robot_id"], self.clock_id, self.clock(), self._sample_sequence,
                tuple(self.arm.q), tuple(self.arm.dq), self.hand.width, self.hand.moving, getattr(self.hand, "grasped", False),
                fault is None and not self.authority.stopping and feedback_valid, lease,
                sequence if executing else None, row if executing else None,
                None if fault is None else fault.code)

    # ---- request handlers
    def _hello(self, payload):
        return {"server_clock_id": self.clock_id, "robot_id": self.settings["robot_id"],
                "contract_sha256": self.contract_sha256, "versions": self._versions()}

    def _versions(self):
        return {"franka-runtime": franka_runtime.__version__, "contracts": contracts.__version__, "backend": self.backend.name}

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
        if self._shutdown.is_set():
            raise RuntimeError("Service shutdown requested")
        if hasattr(self.backend, "check_session_admission"):
            self.backend.check_session_admission()
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
            self.executor = ChunkExecutor(self.row_period_ns, self.settings["hold_timeout_ns"], ROW_START_MARGIN_NS)
            self.backend.reset_session(self.executor)
            self.intent = GripperIntent(self.spec.gripper, self.hand.width)
            self._last_step = None
            self._session = {"request": request, "lease": lease, "estimate": None, "started": False}
            self._fault_reported = False
        ready = DroidSessionReady(lease, self.contract_sha256, self.authority.capability_id,
                                  self.settings["open_width_m"], self.settings["hold_timeout_ns"],
                                  self.settings["max_uncertainty_ns"], self._versions())
        if not self._record("session_opened",ready.to_dict()):
            self.authority.poll(native_fault=209)
            raise RuntimeError("Session recording unavailable")
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
        if not self._record("clock_registered",dict(estimate=estimate.to_dict(),probes=[p.to_dict() for p in probes])):
            self.authority.poll(native_fault=209)
            raise RuntimeError("Clock recording unavailable")
        return estimate.to_dict()

    def _start(self, payload):
        session = self._session_or_fail()
        with self._lock:
            if self._shutdown.is_set():
                raise RuntimeError("Service shutdown requested")
            if not self._record("start_requested",dict(lease=asdict(session["lease"]))):
                raise RuntimeError("Recording unavailable before start")
            self.authority.start(session["lease"])
            self.intent = GripperIntent(self.spec.gripper, self.hand.width)
            session["started"] = True
            self._last_heartbeat = self.clock()
            if not self._record("started",dict(q=list(self.arm.q),dq=list(self.arm.dq))):
                self.authority.poll(native_fault=209)
        return {"state": self.authority.state}

    def _submit(self, payload):
        if self._shutdown.is_set():
            raise RuntimeError("Service shutdown requested")
        session = self._session_or_fail()
        envelope = CommandEnvelope.from_dict(payload["envelope"])
        chunk = DroidActionChunk.from_dict(payload["chunk"])
        with self._lock:
            validate_chunk_command(self.spec, envelope, chunk, lease=session["lease"], estimate=session["estimate"],
                                   now_ns=self.clock(), next_sequence=envelope.sequence,
                                   capability=self.authority.capability_id)
            if not self._record("command",dict(envelope=envelope.to_dict(),chunk=chunk.to_dict())):
                self.authority.poll(native_fault=209)
                raise RuntimeError("Command recording unavailable")
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
                    "gripper_commands": list(self.hand.commands), "peak_implied_speed_rad_s": list(self.peak_speed),
                    "hand_status": self.hand.snapshot() if hasattr(self.hand, "snapshot") else None}

    def _release(self, payload):
        session = self._session_or_fail()
        with self._lock:
            self.authority.release(LeaseRelease(session["lease"], self.clock(), str(payload.get("reason", "release"))))
        if not self._record("released",dict(authority_state=self.authority.state,
                cancellation_evidence=getattr(self.backend,"cancellation_evidence",None))):
            self.authority.poll(native_fault=209)
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
            while not self._shutdown.is_set():
                try:
                    connection, peer = self.listener.accept()
                except socket.timeout:
                    continue
                if peer[0] not in self.settings["allowed_peers"] or self._session is not None:
                    connection.close()
                    continue
                self._serve_client(connection)
        finally:
            self.finish_shutdown(control=control)
            self.listener.close()

    def _control_loop(self):
        period = self.settings["control_period_ns"]
        previous = self.clock()
        while not self._stop.is_set():
            time.sleep(period/1e9)
            # Submit/dispatch uses the same lock. Stamp after acquiring it so
            # a row cannot be acknowledged with a pre-publication timestamp
            # when a concurrent submission delayed this control iteration.
            with self._lock:
                now = self.clock()
                self.control_step(now, (now-previous)/1e9)
                previous = now

    def _serve_client(self, connection):
        connection.settimeout(1.0)
        stream = connection.makefile("rwb", buffering=0)
        try:
            while not self._shutdown.is_set():
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
            self._record("client_disconnected",dict(authority_state=self.authority.state))
            self._session = None
            self._probes.clear()

    def shutdown(self):
        """Request shutdown; signal handlers never join control/device/disk threads."""
        self._shutdown.set()
        with self._lock:
            if self.authority.state in ("running","ready") and not self.authority.stopping:
                try:
                    self.authority.release(LeaseRelease(self.authority.lease,self.clock(),"service_shutdown"))
                except Exception as error:
                    self._shutdown_error = repr(error)

    def finish_shutdown(self, *, control=None, timeout_s=None, recorder_timeout_s=None,
                        wall_clock=time.monotonic, sleep=time.sleep):
        """Keep supervision alive through stop; report unjoined workers honestly.

        If serve() owns the control thread, it keeps ticking until terminal evidence
        or the wall deadline. Without it, this method pumps the same control_step.
        Native watchdog/stop remain independent of Python progress.
        """
        if self.shutdown_report is not None:
            return self.shutdown_report
        budget = (getattr(self.backend,"stop_timeout",0)/1e9+2.) if timeout_s is None else timeout_s
        if isinstance(budget,bool) or not isinstance(budget,(int,float)) or not 0<budget<60:
            raise ValueError("Positive shutdown deadline below 60 seconds required")
        self.shutdown()
        deadline=wall_clock()+budget
        previous=self.clock()
        try:
            while self.authority.stopping and wall_clock()<deadline:
                if control is None or not control.is_alive():
                    now=self.clock();self.control_step(now,max(0.,(now-previous)/1e9));previous=now
                sleep(min(.01,max(0.,deadline-wall_clock())))
        except Exception as error:
            self._shutdown_error = repr(error)
            try:
                self.backend.request_stop('supervisor_failure')
            except Exception:
                pass  # No alternate controller or invented successful stop.
        self._stop.set()
        if control is not None:
            control.join(max(0.,deadline-wall_clock()))
        driver=getattr(self.backend,"driver",None)
        native_started=getattr(self.backend,"_started",False)
        if driver is None:
            workers_joined=bool(getattr(self.backend,"workers_joined",True)) # Shadow also joins read-only workers.
        else:
            arm_joined=driver.joined if native_started else getattr(driver,"thread",None) is None
            hand_passive=(not native_started and not getattr(self.backend,"_stopping",False))
            workers_joined=arm_joined and (hand_passive or self.hand.quiescent)
        evidence=self._last_stop_evidence
        stop_ok=(not self.authority.stopping and (evidence is None or
            (evidence.get("joined") is True and evidence.get("stop_finished") is True and
             evidence.get("stop_at_rest") is True and evidence.get("stop_fault")==0 and
             evidence.get("dropped_samples")==0)))
        if getattr(self.backend,"shadow",False):
            stop_ok=(not self.authority.stopping and (evidence is None or
                (evidence.get("mode")=="shadow_readonly" and evidence.get("source_joined") is True
                 and evidence.get("no_commands_sent") is True)))
        stop_ok=stop_ok and (not native_started or evidence is not None)
        report=dict(shutdown_error=self._shutdown_error,native_control_started=bool(native_started),supervisor_joined=control is None or not control.is_alive(),workers_joined=bool(workers_joined),
            stop_confirmed=bool(stop_ok),stop_evidence=evidence,cancellation_evidence=getattr(self.backend,"cancellation_evidence",None),authority_state=self.authority.state,
            fault=None if self.authority.fault is None else asdict(self.authority.fault))
        report["ok"]=self._shutdown_error is None and report["supervisor_joined"] and report["workers_joined"] and report["stop_confirmed"]
        if self.recorder is not None:
            self._record("shutdown",report)
            report["recording"]=self.recorder.close(report,timeout_s=(self.settings.get("recorder_close_timeout_s",2.)
                if recorder_timeout_s is None else recorder_timeout_s))
            report["ok"]=report["ok"] and report["recording"]["sealed"]
        self.shutdown_report=report
        return report
