"""Extracted hardware motion helpers; provenance: deployment/migration/phase2-extraction.json."""
from __future__ import annotations
import math
from dataclasses import dataclass
import numpy as np

def finite_vector(value, length: int, name: str) -> tuple[float, ...]:
    array = np.asarray(value, dtype=float)
    if array.shape != (length,) or not np.isfinite(array).all():
        raise ValueError(f"{name} must contain {length} finite scalars")
    return tuple(float(v) for v in array)


def positive(value: float, name: str, *, allow_zero: bool = False) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a number, not a boolean")
    result = float(value)
    if not math.isfinite(result) or result < 0 or (result == 0 and not allow_zero):
        raise ValueError(f"{name} must be finite and {'nonnegative' if allow_zero else 'positive'}")
    return result


def unit_quaternion_xyzw(value) -> tuple[float, ...]:
    quaternion = finite_vector(value, 4, "quaternion_xyzw")
    if not math.isclose(float(np.linalg.norm(quaternion)), 1.0, rel_tol=0, abs_tol=1e-8):
        raise ValueError("quaternion_xyzw must have unit norm; normalization is not implicit")
    return quaternion


@dataclass(frozen=True)
class CartesianReference:
    position_m: tuple[float, ...]
    quaternion_xyzw: tuple[float, ...]
    linear_velocity_m_s: tuple[float, ...] = (0.0, 0.0, 0.0)
    angular_velocity_rad_s: tuple[float, ...] = (0.0, 0.0, 0.0)
    time_s: float = 0.0
    base_frame: str = "fr3_link0"
    tcp_frame: str = "fr3_flange"

    def __post_init__(self):
        for name in ("position_m", "linear_velocity_m_s", "angular_velocity_rad_s"):
            object.__setattr__(self, name, finite_vector(getattr(self, name), 3, name))
        object.__setattr__(self, "quaternion_xyzw", unit_quaternion_xyzw(self.quaternion_xyzw))
        object.__setattr__(self, "time_s", positive(self.time_s, "time_s", allow_zero=True))
        if any(not isinstance(value, str) or not value for value in (self.base_frame, self.tcp_frame)):
            raise ValueError("Explicit base_frame and tcp_frame are required")

    def as_action(self, *, twist_feedforward: bool = True) -> np.ndarray:
        twist = self.linear_velocity_m_s + self.angular_velocity_rad_s if twist_feedforward else (0.0,) * 6
        return np.asarray(self.position_m + self.quaternion_xyzw + twist)


@dataclass(frozen=True)
class TrajectorySample:
    reference: CartesianReference
    acceleration_m_s2: tuple[float, ...]
    jerk_m_s3: tuple[float, ...]
    phase: str = "track"

    def __post_init__(self):
        for name in ("acceleration_m_s2", "jerk_m_s3"):
            object.__setattr__(self, name, finite_vector(getattr(self, name), 3, name))
