"""Analytic time laws. Bounds are checked, never achieved by derivative clipping."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.polynomial import polynomial as poly

from .reference_types import positive


@dataclass(frozen=True)
class TimeLawSample:
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    jerk: np.ndarray


def _time(t: float, duration: float) -> float:
    value = float(t)
    if not math.isfinite(value) or value < 0 or value > duration:
        raise ValueError(f"time must lie in [0, {duration}]")
    return value


class QuinticSegment:
    """Position/velocity/acceleration boundary values, in normalized time s=t/T."""

    def __init__(self, start, end, duration_s: float, *, start_velocity=None, end_velocity=None,
                 start_acceleration=None, end_acceleration=None):
        self.duration_s = positive(duration_s, "duration_s")
        p0, p1 = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
        if p0.shape != p1.shape or p0.ndim > 1:
            raise ValueError("Endpoint shapes must match and be scalar or vector")
        v0 = np.zeros_like(p0) if start_velocity is None else np.asarray(start_velocity, dtype=float)
        v1 = np.zeros_like(p0) if end_velocity is None else np.asarray(end_velocity, dtype=float)
        a0 = np.zeros_like(p0) if start_acceleration is None else np.asarray(start_acceleration, dtype=float)
        a1 = np.zeros_like(p0) if end_acceleration is None else np.asarray(end_acceleration, dtype=float)
        if any(v.shape != p0.shape or not np.isfinite(v).all() for v in (p0, p1, v0, v1, a0, a1)):
            raise ValueError("Boundary values must have matching shapes and be finite")
        T = self.duration_s
        c0, c1, c2 = p0, v0 * T, 0.5 * a0 * T**2
        rhs = np.stack((p1 - c0 - c1 - c2, v1 * T - c1 - 2 * c2, a1 * T**2 - 2 * c2))
        tail = np.linalg.solve(np.array([[1., 1., 1.], [3., 4., 5.], [6., 12., 20.]]), rhs)
        self.coefficients = np.concatenate((np.stack((c0, c1, c2)), tail))
        self.coefficients.setflags(write=False)

    def evaluate(self, time_s: float) -> TimeLawSample:
        s = _time(time_s, self.duration_s) / self.duration_s
        values = [poly.polyval(s, poly.polyder(self.coefficients, m=d, axis=0)) / self.duration_s**d
                  for d in range(4)]
        return TimeLawSample(*values)

    def derivative_peaks(self) -> dict[str, list[float]]:
        result = {}
        for d, name in enumerate(("velocity", "acceleration", "jerk"), 1):
            coefficients = poly.polyder(self.coefficients, m=d, axis=0).reshape(6 - d, -1)
            peaks = []
            for column in coefficients.T:
                roots = poly.polyroots(poly.polyder(column))
                critical = [float(root.real) for root in roots if abs(root.imag) < 1e-9 and 0 < root.real < 1]
                peaks.append(float(np.max(np.abs(poly.polyval([0., 1., *critical], column)))) / self.duration_s**d)
            result[name] = peaks
        return result


@dataclass(frozen=True)
class SmoothPhaseSchedule:
    """Quintic SPEED ramps give continuous phase derivatives through jerk at joins.

    The total angular displacement includes both ramps. The cruise duration is
    derived from the requested turns, not appended to an already complete circle.
    """

    angular_speed_rad_s: float
    ramp_duration_s: float
    turns: float = 1.0
    initial_phase_rad: float = 0.0

    def __post_init__(self):
        for name in ("angular_speed_rad_s", "ramp_duration_s", "turns"):
            object.__setattr__(self, name, positive(getattr(self, name), name))
        if not math.isfinite(self.initial_phase_rad):
            raise ValueError("initial_phase_rad must be finite")
        if self.cruise_duration_s < 0:
            raise ValueError("Requested turns are too few for the selected speed/ramp; reduce speed or ramp duration")

    @property
    def cruise_duration_s(self) -> float:
        return 2 * math.pi * self.turns / self.angular_speed_rad_s - self.ramp_duration_s

    @property
    def duration_s(self) -> float:
        return self.cruise_duration_s + 2 * self.ramp_duration_s

    def evaluate(self, time_s: float) -> TimeLawSample:
        t = _time(time_s, self.duration_s)
        w, T, C = self.angular_speed_rad_s, self.ramp_duration_s, self.cruise_duration_s

        def ramp(s):
            h = s**3 * (10 - 15 * s + 6 * s**2)
            integral = s**4 * (2.5 - 3 * s + s**2)
            first = 30 * s**2 * (1 - s)**2
            second = 60 * s * (1 - s) * (1 - 2 * s)
            return h, integral, first, second

        if t < T:
            h, integral, first, second = ramp(t / T)
            theta, speed, acceleration, jerk = w * T * integral, w * h, w / T * first, w / T**2 * second
        elif t <= T + C:
            theta, speed, acceleration, jerk = w * (0.5 * T + t - T), w, 0., 0.
        else:
            s = (t - T - C) / T
            h, integral, first, second = ramp(s)
            theta = w * (0.5 * T + C + T * (s - integral))
            speed, acceleration, jerk = w * (1 - h), -w / T * first, -w / T**2 * second
        return TimeLawSample(np.asarray(theta + self.initial_phase_rad), np.asarray(speed),
                             np.asarray(acceleration), np.asarray(jerk))

    def derivative_peaks(self) -> dict[str, list[float]]:
        return {"velocity": [self.angular_speed_rad_s],
                "acceleration": [1.875 * self.angular_speed_rad_s / self.ramp_duration_s],
                "jerk": [10 / math.sqrt(3) * self.angular_speed_rad_s / self.ramp_duration_s**2]}


def evaluate_quintic(segment: QuinticSegment, time_s: float) -> TimeLawSample:
    return segment.evaluate(time_s)


def validate_time_law(law, *, max_velocity, max_acceleration, max_jerk) -> dict:
    peaks = law.derivative_peaks()
    for name, limit in (("velocity", max_velocity), ("acceleration", max_acceleration), ("jerk", max_jerk)):
        bound = np.asarray(limit, dtype=float)
        values = np.asarray(peaks[name])
        if bound.ndim > 1 or (bound.ndim == 1 and bound.shape != values.shape):
            raise ValueError(f"{name} bound shape does not match time law")
        if not np.isfinite(bound).all() or np.any(bound <= 0):
            raise ValueError(f"{name} bound must be positive and finite")
        if np.any(values > bound * (1 + 1e-12)):
            raise ValueError(f"Time law exceeds {name} bound: {values} > {bound}")
    return {"passed": True, "method": "analytic extrema", "derivative_peaks": peaks}
