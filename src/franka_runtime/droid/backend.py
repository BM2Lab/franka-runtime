"""Backend-neutral control evidence. No vendor imports or hardware side effects."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Consumption:
    sequence: int
    tick: int
    timestamp_ns: int


@dataclass(frozen=True)
class BackendUpdate:
    consumed: tuple = ()
    fault: int = 0
    dropped: int = 0
    stop_evidence: object = None
