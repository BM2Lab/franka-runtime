"""Continuous base-frame paths with analytic derivatives and fixed tool orientation."""

from __future__ import annotations

import math
from dataclasses import replace

import numpy as np

from .reference_types import CartesianReference, TrajectorySample, finite_vector, positive, unit_quaternion_xyzw
from .time_scaling import QuinticSegment, SmoothPhaseSchedule
from .kinematics import quat_multiply, rotation_vector, quat_exp


def rotation_matrix_rpy_deg(rpy_deg) -> np.ndarray:
    roll, pitch, yaw = np.radians(finite_vector(rpy_deg, 3, "plane_rpy_deg"))
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


class _PlanarTrajectory:
    def __init__(self, center_m, plane_rpy_deg, quaternion_xyzw, schedule: SmoothPhaseSchedule,
                 *, base_frame="fr3_link0", tcp_frame="fr3_flange"):
        self.center_m = np.asarray(finite_vector(center_m, 3, "center_m"))
        self.rotation = rotation_matrix_rpy_deg(plane_rpy_deg)
        self.quaternion_xyzw = unit_quaternion_xyzw(quaternion_xyzw)
        self.schedule = schedule
        self.base_frame, self.tcp_frame = base_frame, tcp_frame

    @property
    def duration_s(self):
        return self.schedule.duration_s

    def evaluate(self, time_s: float) -> TrajectorySample:
        phase = self.schedule.evaluate(time_s)
        p, first, second, third = self.geometry(float(phase.position))
        w, a, j = float(phase.velocity), float(phase.acceleration), float(phase.jerk)
        reference = CartesianReference(
            tuple(self.center_m + self.rotation @ p), self.quaternion_xyzw,
            tuple(self.rotation @ (first * w)), time_s=time_s,
            base_frame=self.base_frame, tcp_frame=self.tcp_frame,
        )
        return TrajectorySample(reference, tuple(self.rotation @ (second * w**2 + first * a)),
                                tuple(self.rotation @ (third * w**3 + 3 * second * w * a + first * j)))


class CircleTrajectory(_PlanarTrajectory):
    def __init__(self, *, center_m, radius_m, schedule, plane_rpy_deg=(0., 0., 0.),
                 quaternion_xyzw=(0., 1., 0., 0.), **kwargs):
        super().__init__(center_m, plane_rpy_deg, quaternion_xyzw, schedule, **kwargs)
        self.radius_m = positive(radius_m, "radius_m")

    def geometry(self, theta):
        p = self.radius_m * np.array([math.cos(theta), math.sin(theta), 0.])
        first = self.radius_m * np.array([-math.sin(theta), math.cos(theta), 0.])
        return p, first, -p, -first


class LissajousTrajectory(_PlanarTrajectory):
    def __init__(self, *, center_m, amplitudes_m, harmonics, schedule, phase_offsets_rad=(0., 0.),
                 plane_rpy_deg=(0., 0., 0.), quaternion_xyzw=(0., 1., 0., 0.), **kwargs):
        super().__init__(center_m, plane_rpy_deg, quaternion_xyzw, schedule, **kwargs)
        self.amplitudes = np.asarray(finite_vector(amplitudes_m, 2, "amplitudes_m"))
        self.harmonics = np.asarray(finite_vector(harmonics, 2, "harmonics"))
        self.offsets = np.asarray(finite_vector(phase_offsets_rad, 2, "phase_offsets_rad"))
        if np.any(self.amplitudes <= 0) or np.any(self.harmonics < 1) or np.any(self.harmonics % 1):
            raise ValueError("Amplitudes must be positive and harmonics positive integers")

    def geometry(self, theta):
        phase = self.harmonics * theta + self.offsets
        m, A = self.harmonics, self.amplitudes
        return tuple(np.r_[value, 0.] for value in (A * np.sin(phase), A * m * np.cos(phase),
                     -A * m**2 * np.sin(phase), -A * m**3 * np.cos(phase)))


class ApproachAndTrackTrajectory:
    """Approach/settle/ramped track/exit; initial position is explicit feedback or a preview input.

    Optional measured entry orientation is joined by a quintic SO(3) geodesic.
    Orientation is constant on the track, never jumped instantaneously at entry.
    Quintic position approach/exit has continuous position/velocity/acceleration;
    its jerk has finite boundary steps, which are explicitly included in limits.
    """

    def __init__(self, path, initial_position_m, *, approach_duration_s=4., settle_duration_s=1., exit_duration_s=4.,
                 return_to_start=True, initial_quaternion_xyzw=None):
        self.path = path
        self.initial_position = finite_vector(initial_position_m, 3, "initial_position_m")
        self.approach_duration_s = positive(approach_duration_s, "approach_duration_s")
        self.settle_duration_s = positive(settle_duration_s, "settle_duration_s", allow_zero=True)
        self.exit_duration_s = positive(exit_duration_s, "exit_duration_s")
        start = path.evaluate(0).reference.position_m
        end = path.evaluate(path.duration_s).reference.position_m
        self.approach = QuinticSegment(self.initial_position, start, self.approach_duration_s)
        self.exit = QuinticSegment(end, self.initial_position if return_to_start else end, self.exit_duration_s)
        self.initial_quaternion = np.array(unit_quaternion_xyzw(
            path.quaternion_xyzw if initial_quaternion_xyzw is None else initial_quaternion_xyzw))
        self.rotation_delta = rotation_vector(quat_multiply(path.quaternion_xyzw,
            np.r_[-self.initial_quaternion[:3], self.initial_quaternion[3]]))
        self.orientation_approach = QuinticSegment(0., 1., self.approach_duration_s)
        self.orientation_exit = QuinticSegment(1., 0. if return_to_start else 1., self.exit_duration_s)

    @property
    def track_start_s(self):
        return self.approach_duration_s + self.settle_duration_s

    @property
    def duration_s(self):
        return self.track_start_s + self.path.duration_s + self.exit_duration_s

    def evaluate(self, time_s):
        t = positive(time_s, "time_s", allow_zero=True)
        if t > self.duration_s:
            raise ValueError("time exceeds trajectory duration")
        if t < self.approach_duration_s:
            law, phase = self.approach.evaluate(t), "approach"
            orientation = self.orientation_approach.evaluate(t)
        elif t < self.track_start_s:
            reference = replace(self.path.evaluate(0).reference, time_s=t)
            return TrajectorySample(reference, (0.,) * 3, (0.,) * 3, "settle")
        elif t < self.track_start_s + self.path.duration_s:
            sample = self.path.evaluate(t - self.track_start_s)
            return replace(sample, reference=replace(sample.reference, time_s=t))
        else:
            # Subtraction at the exact global endpoint can round beyond the local endpoint.
            local_time = min(self.exit_duration_s, max(0., t - self.track_start_s - self.path.duration_s))
            law, phase = self.exit.evaluate(local_time), "exit"
            orientation = self.orientation_exit.evaluate(local_time)
        quaternion = quat_multiply(quat_exp(self.rotation_delta*float(orientation.position)), self.initial_quaternion)
        reference = CartesianReference(tuple(law.position), tuple(quaternion), tuple(law.velocity),
                                       tuple(self.rotation_delta*float(orientation.velocity)),
                                       time_s=t, base_frame=self.path.base_frame, tcp_frame=self.path.tcp_frame)
        return TrajectorySample(reference, tuple(law.acceleration), tuple(law.jerk), phase)


def sample_trajectory(trajectory, *, sample_hz=200.) -> dict[str, np.ndarray]:
    rate = positive(sample_hz, "sample_hz")
    times = np.linspace(0., trajectory.duration_s, math.ceil(trajectory.duration_s * rate) + 1)
    samples = [trajectory.evaluate(float(t)) for t in times]
    return {"time_s": times, "position_m": np.array([s.reference.position_m for s in samples]),
            "quaternion_xyzw": np.array([s.reference.quaternion_xyzw for s in samples]),
            "linear_velocity_m_s": np.array([s.reference.linear_velocity_m_s for s in samples]),
            "angular_velocity_rad_s": np.array([s.reference.angular_velocity_rad_s for s in samples]),
            "acceleration_m_s2": np.array([s.acceleration_m_s2 for s in samples]),
            "jerk_m_s3": np.array([s.jerk_m_s3 for s in samples]), "phase": np.array([s.phase for s in samples])}


def validate_trajectory(trajectory, *, max_speed_m_s, max_acceleration_m_s2, max_jerk_m_s3,
                        lower_m=None, upper_m=None, sample_hz=200.) -> dict:
    samples = sample_trajectory(trajectory, sample_hz=sample_hz)
    peaks = {}
    for field, bound in (("linear_velocity_m_s", max_speed_m_s), ("acceleration_m_s2", max_acceleration_m_s2),
                         ("jerk_m_s3", max_jerk_m_s3)):
        positive(bound, field)
        peaks[field] = float(np.linalg.norm(samples[field], axis=1).max())
        if peaks[field] > bound * (1 + 1e-9):
            raise ValueError(f"Trajectory exceeds {field}: {peaks[field]} > {bound}")
    if (lower_m is None) != (upper_m is None):
        raise ValueError("Both Cartesian workspace bounds are required")
    if lower_m is not None:
        lower, upper = np.array(finite_vector(lower_m, 3, "lower_m")), np.array(finite_vector(upper_m, 3, "upper_m"))
        if np.any(lower >= upper) or np.any(samples["position_m"] < lower) or np.any(samples["position_m"] > upper):
            raise ValueError("Preview trajectory leaves the configured Cartesian box")
    return {"passed": True, "method": "sampled analytic derivatives (not collision/IK validation)",
            "sample_count": len(samples["time_s"]), "sample_hz_requested": sample_hz, "peaks": peaks}
