"""Loopback tests over a real TCP socket against the mock backend; no hardware."""
import json
from pathlib import Path
import socket
import threading
import time

import pytest

from contracts.clock import ClockOffsetEstimate, ClockProbe
from contracts.droid import DroidActionChunk, DroidIOSpec
from contracts.droid_session import DroidSessionOpen, DroidSessionReady, chunk_envelope
from contracts.episodes import SampleContext
from contracts.franka_state import FrankaStateSample
from contracts.sessions import LeaseRequest, payload_sha256
from contracts.wire import PROTOCOL_VERSION, read_message, write_message
from franka_runtime.droid.config import DroidRuntimeConfig
from franka_runtime.droid.service import DroidRuntimeService

ROOT = Path(__file__).resolve().parents[2]
CONFIGS = ROOT / "configs/fr3"
SPEC = DroidIOSpec.from_json((CONFIGS / "droid_fr3_jointpos_v1.contract.json").read_text())
TOKEN = "test-token"
CLIENT_CLOCK = "client:test:CLOCK_MONOTONIC"


@pytest.fixture
def service(request):
    data = json.loads((CONFIGS / "droid.jointpos.mock.json").read_text())
    for section, updates in getattr(request, "param", {}).items():
        data[section].update(updates)
    svc = DroidRuntimeService(DroidRuntimeConfig(data, config_dir=CONFIGS), SPEC, token=TOKEN,
                              clock_id="rt:test:CLOCK_MONOTONIC")
    port = svc.bind()
    thread = threading.Thread(target=svc.serve, daemon=True)
    thread.start()
    yield svc, port
    svc.shutdown()
    thread.join(3)


class Client:
    def __init__(self, port):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=5)
        self.stream = self.sock.makefile("rwb", buffering=0)
        self.n = 0

    def call(self, op, payload=None, ok=True):
        self.n += 1
        write_message(self.stream, {"protocol": PROTOCOL_VERSION, "id": str(self.n), "op": op, "payload": payload or {}})
        response = read_message(self.stream)
        assert response["id"] == str(self.n)
        if ok:
            assert response["ok"], response.get("error")
            return response["result"]
        assert not response["ok"]
        return response["error"]

    def probes(self, server_clock, count=6):
        result = []
        for _ in range(count):
            sent = time.monotonic_ns()
            reply = self.call("clock_probe")
            result.append(ClockProbe(CLIENT_CLOCK, server_clock, sent, reply["server_receive_ns"],
                                     reply["server_send_ns"], time.monotonic_ns()))
        return result

    def session(self, token=TOKEN, spec=SPEC):
        server_clock = self.call("hello")["server_clock_id"]
        probes = self.probes(server_clock)
        estimate = ClockOffsetEstimate.from_probes(probes, max_uncertainty_ns=2_000_000)
        request = DroidSessionOpen(spec, payload_sha256(spec), LeaseRequest(
            "req", "fr3_01", "test-runner", server_clock, estimate.to_server(time.monotonic_ns()), 30*10**9),
            CLIENT_CLOCK)
        reply = self.call("open", {"session": request.to_dict(), "token": token}, ok=token == TOKEN and spec == SPEC)
        if not isinstance(reply, dict):
            return reply
        ready = DroidSessionReady.from_dict(reply)
        ready.validate_for(request)
        self.call("clock_register", {"probes": [p.to_dict() for p in probes]})
        self.call("start")
        self.ready, self.estimate = ready, estimate
        return ready

    def state(self):
        return FrankaStateSample.from_dict(self.call("state"))

    def chunk(self, sequence, step, delta, closure=0., n=8, q=None):
        q = self.state().q_rad if q is None else q
        now = time.monotonic_ns()
        context = SampleContext("episode-1", step, now, "monotonic")
        values = tuple((*[v+delta*(i+1) for v in q], closure) for i in range(n))
        value = DroidActionChunk(context, SPEC.action, (*q, 0.), values, now+533_333_333)
        envelope = chunk_envelope(self.ready.lease, sequence, value, self.estimate,
                                  client_now_ns=time.monotonic_ns(), capability=self.ready.capability_id)
        return envelope, value

    def submit(self, envelope, value):
        return self.call("submit", {"envelope": envelope.to_dict(), "chunk": value.to_dict()})

    def wait(self, predicate, timeout=3.):
        deadline, seen = time.monotonic()+timeout, []
        while time.monotonic() < deadline:
            result = self.call("receipts")
            seen += result["receipts"]
            if predicate(result, seen):
                return result, seen
            time.sleep(0.01)
        raise AssertionError(f"timed out; receipts {seen}")

    def close(self):
        try:
            self.call("close")
        finally:
            self.sock.close()


def test_chunk_is_applied_tracked_and_underrun_latches_stop(service):
    svc, port = service
    client = Client(port)
    client.session()
    start_q = client.state().q_rad
    envelope, value = client.chunk(1, 0, 0.01, closure=0.9)
    receipt = client.submit(envelope, value)
    assert receipt["status"] == "accepted"
    assert client.submit(envelope, value)["status"] == "rejected"  # replayed sequence
    _, seen = client.wait(lambda r, s: any(x["status"] == "applied" for x in s))
    applied = next(x for x in seen if x["status"] == "applied")
    assert applied["sequence"] == 1 and applied["native_tick"] > 0
    time.sleep(0.15)
    mid = client.state()
    assert mid.executing_sequence is not None and mid.valid
    assert mid.q_rad[0] > start_q[0]
    result, _ = client.wait(lambda r, s: r["fault"] is not None and r["fault"]["stop_state"] == "complete")
    assert result["fault"]["code"] == "native_fault:101"  # underrun hold timeout
    assert result["gripper_commands"] == ["close"] and result["authority_state"] == "fault"
    final = client.state()
    assert not final.valid and final.fault_code == "native_fault:101"
    assert final.q_rad[0] == pytest.approx(start_q[0]+0.08, abs=1e-6)  # last row reached, then held
    late = client.submit(*client.chunk(2, 1, 0.))
    assert late["status"] == "rejected" and "latched" in late["reason"]
    client.close()


def test_physical_and_binding_rejections(service):
    svc, port = service
    client = Client(port)
    client.session()
    too_fast = client.submit(*client.chunk(1, 0, 0.1, n=2))  # row 0→1 is 1.5 rad/s > mock 1.0
    assert too_fast["status"] == "rejected" and "rad/s" in too_fast["reason"]
    q = list(client.state().q_rad); q[3] = -0.2
    # Rejections do not consume a sequence number: the next command is still sequence 1.
    envelope, value = client.chunk(1, 1, 0., q=tuple(q))
    # Contract binding fails before the authority if the payload is swapped.
    other = client.chunk(1, 1, 0.001)[1]
    assert "Payload" in client.call("submit", {"envelope": envelope.to_dict(), "chunk": other.to_dict()}, ok=False)
    out_of_range = client.submit(envelope, value)
    assert out_of_range["status"] == "rejected" and "joint 4" in out_of_range["reason"]
    assert svc.authority.fault is None
    client.call("release", {"reason": "test complete"})
    client.wait(lambda r, s: r["authority_state"] == "complete")
    client.close()


def test_authentication_and_contract_allow_list(service):
    _, port = service
    client = Client(port)
    assert "token" in client.session(token="wrong")
    client.close()
    other_spec = DroidIOSpec.from_dict({**SPEC.to_dict(), "open_loop_horizon": 4})
    client = Client(port)
    assert "allow-listed" in client.session(spec=other_spec)
    client.close()


def test_release_allows_a_fresh_session(service):
    svc, port = service
    client = Client(port)
    client.session()
    client.submit(*client.chunk(1, 0, 0.005))
    client.call("release", {"reason": "operator"})
    client.wait(lambda r, s: r["authority_state"] == "complete")
    client.close()
    time.sleep(0.05)
    again = Client(port)
    again.session()
    assert again.submit(*again.chunk(1, 0, 0.005))["status"] == "accepted"
    again.wait(lambda r, s: any(x["status"] == "applied" for x in s))
    again.call("release", {"reason": "done"})
    again.close()


@pytest.mark.parametrize("service", [{"timing": {"heartbeat_timeout_ms": 300}}], indirect=True)
def test_missing_heartbeat_latches_stop(service):
    svc, port = service
    client = Client(port)
    client.session()
    time.sleep(0.5)
    result = client.call("receipts")
    assert result["fault"]["code"] == "native_fault:102"
    client.close()


def test_disconnect_latches_stop(service):
    svc, port = service
    client = Client(port)
    client.session()
    client.sock.close()
    deadline = time.monotonic()+3
    while svc.authority.fault is None and time.monotonic() < deadline:
        time.sleep(0.01)
    assert svc.authority.fault.code == "native_fault:103"


@pytest.mark.parametrize("service", [{"workspace": {"box_max_m": [0.85, 0.6, 0.5]}}], indirect=True)
def test_workspace_rejects_rows_leaving_the_box(service):
    svc, port = service
    client = Client(port)
    client.session()  # home tool points sit at z ~0.487 m, just inside the lowered box
    receipt = client.submit(*client.chunk(1, 0, 0.0, n=1, q=tuple(v+d for v, d in zip(client.state().q_rad,
                                                                                       (0, -0.2, 0, 0, 0, 0, 0)))))
    assert receipt["status"] == "rejected" and "workspace box" in receipt["reason"]
    client.call("release", {"reason": "done"})
    client.close()
