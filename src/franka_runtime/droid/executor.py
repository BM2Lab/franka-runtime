"""Deterministic DROID chunk executor. Pure logic: time is passed in, nothing runs on its own.

Rows of the active chunk start on a fixed grid (``row_period_ns``). An accepted chunk
waits as the single pending chunk and replaces the current chunk's not-yet-started
rows at the next row boundary (immediately when idle or holding). No row starts at or
after its chunk's runtime deadline. When rows run out, the last target is held for at
most ``hold_timeout_ns``; then the executor requests a stop. The backend owns native
execution; the hybrid impedance candidate does not interpolate row targets.
"""
from dataclasses import dataclass
from typing import Optional

UNDERRUN = "underrun_hold_timeout"


@dataclass
class _Chunk:
    sequence: int
    rows: tuple
    deadline_ns: int
    start_ns: Optional[int] = None


@dataclass(frozen=True)
class Step:
    target: Optional[tuple]
    closure: Optional[float]
    sequence: Optional[int]
    row: Optional[int]
    row_started: bool
    started: Optional[int]
    dropped: tuple
    holding: bool
    stop_reason: Optional[str]


class ChunkExecutor:
    def __init__(self, row_period_ns, hold_timeout_ns):
        for value, name in ((row_period_ns, "row_period_ns"), (hold_timeout_ns, "hold_timeout_ns")):
            if type(value) is not int or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        self.period, self.hold_timeout = row_period_ns, hold_timeout_ns
        self.current = self.pending = None
        self.active_row = None
        self.last_target = self.last_closure = None
        self.hold_since = None
        self.stopped = None
        self._dropped = []

    def submit(self, sequence, rows, deadline_ns):
        if self.stopped is not None:
            raise RuntimeError(f"Executor stopped: {self.stopped}")
        rows = tuple(tuple(r) for r in rows)
        if not rows or any(len(r) != 8 for r in rows):
            raise ValueError("Rows must be seven joint targets plus closure")
        if self.pending is not None:
            self._dropped.append(self.pending.sequence)  # Replaced before it started.
        self.pending = _Chunk(sequence, rows, deadline_ns)

    def stop(self, reason):
        if self.stopped is None:
            self.stopped = reason
        self.current = self.pending = None

    def _row(self, chunk, now_ns):
        index = (now_ns-chunk.start_ns)//self.period
        start = chunk.start_ns+index*self.period
        return index if index < len(chunk.rows) and start < chunk.deadline_ns else None

    def tick(self, now_ns):
        dropped, self._dropped = tuple(self._dropped), []
        if self.stopped is not None:
            return Step(self.last_target, self.last_closure, None, None, False, None, dropped, False, self.stopped)
        row = None if self.current is None else self._row(self.current, now_ns)
        row_due = self.current is None or row is None or row != self.active_row
        started = None
        if self.pending is not None and now_ns >= self.pending.deadline_ns:
            dropped += (self.pending.sequence,)  # Expired before it could start.
            self.pending = None
        if row_due and self.pending is not None:
            self.current, self.pending = self.pending, None
            self.current.start_ns, row, started = now_ns, 0, self.current.sequence
        if self.current is not None and row is not None:
            row_started = row != self.active_row or started is not None
            self.active_row = row
            target = self.current.rows[row]
            self.last_target, self.last_closure, self.hold_since = target[:7], target[7], None
            return Step(self.last_target, self.last_closure, self.current.sequence, row, row_started, started,
                        dropped, False, None)
        if self.current is not None:
            # Rows whose start time passed between sparse ticks still count: hold the last due row.
            due = min(len(self.current.rows), -(-(self.current.deadline_ns-self.current.start_ns)//self.period))-1
            if due >= 0 and (self.active_row is None or due > self.active_row):
                self.last_target, self.last_closure = self.current.rows[due][:7], self.current.rows[due][7]
        self.current, self.active_row = None, None
        if self.last_target is None:
            return Step(None, None, None, None, False, None, dropped, False, None)  # Armed, never commanded.
        if self.hold_since is None:
            self.hold_since = now_ns
        if now_ns-self.hold_since > self.hold_timeout:
            self.stop(UNDERRUN)
            return Step(self.last_target, self.last_closure, None, None, False, None, dropped, False, UNDERRUN)
        return Step(self.last_target, self.last_closure, None, None, False, None, dropped, True, None)
