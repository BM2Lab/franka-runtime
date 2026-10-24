"""A delayed control iteration cannot report consumption before publication."""
from types import SimpleNamespace
import threading

from franka_runtime.droid.service import DroidRuntimeService


def test_control_timestamp_includes_time_waiting_for_submission(monkeypatch):
    clock = SimpleNamespace(now=1_000_000_000)
    published_ns = clock.now + 20_000_000
    observed = []

    class SubmissionLock:
        def __enter__(self):
            # A submitter owns the lock until its publication has completed.
            clock.now = published_ns + 1
            return self

        def __exit__(self, *args):
            pass

    service = DroidRuntimeService.__new__(DroidRuntimeService)
    service.settings = {"control_period_ns": 4_000_000}
    service.clock = lambda: clock.now
    service._lock = SubmissionLock()
    service._stop = threading.Event()

    def consume(now_ns, dt_s):
        observed.append((now_ns, dt_s))
        service._stop.set()

    service.control_step = consume
    monkeypatch.setattr("franka_runtime.droid.service.time.sleep", lambda _: None)
    service._control_loop()
    stamp, elapsed = observed[0]
    assert stamp > published_ns
    assert elapsed == (stamp-1_000_000_000)/1e9
