"""Extracted hardware motion helpers; provenance: deployment/migration/phase2-extraction.json."""
from __future__ import annotations
import math
import time
from dataclasses import dataclass
from typing import Any
import numpy as np
EE_FEEDBACK_SOURCE="measured_joint_encoder_libfranka_fk"
EE_FEEDBACK_BASE_FRAME="fr3_link0"
EE_FEEDBACK_FRAME="fr3_flange"

@dataclass(frozen=True)
class Snapshot:
    host_monotonic_ns: int
    robot_time_s: float
    q: tuple[float, ...]
    dq: tuple[float, ...]
    q_d: tuple[float, ...]
    dq_d: tuple[float, ...]
    ddq_d: tuple[float, ...]
    flange_position_m: tuple[float, ...]
    flange_orientation_xyzw: tuple[float, ...]
    ee_feedback_source: str
    ee_feedback_base_frame: str
    ee_feedback_frame: str
    reported_flange_position_m: tuple[float, ...]
    encoder_fk_reported_position_error_m: float
    encoder_fk_reported_orientation_error_rad: float
    encoder_fk_compute_ns: int
    f_t_ee: tuple[tuple[float, ...], ...]
    load_mass_kg: float
    mode: str
    has_errors: bool
    current_errors: str
    last_motion_errors: str
    is_in_control: bool
    command_success_rate: float


def _tuple(value: Any) -> tuple[float, ...]:
    return tuple(float(v) for v in np.asarray(value).reshape(-1))


def _matrix(value: Any) -> np.ndarray:
    matrix = getattr(value, "matrix", value)
    result = np.asarray(matrix() if callable(matrix) else matrix, dtype=float)
    if result.shape != (4, 4) or not np.isfinite(result).all():
        raise RuntimeError("pose is not a finite 4x4 matrix")
    return result


@dataclass(frozen=True)
class EncoderPoseFeedback:
    position_m: tuple[float, ...]
    orientation_xyzw: tuple[float, ...]
    reported_position_m: tuple[float, ...]
    reported_position_error_m: float
    reported_orientation_error_rad: float
    compute_ns: int


class EncoderForwardKinematics:
    """Compute O_T_F from measured q through the pinned libfranka model."""

    def __init__(self, robot: Any, franky_module: Any):
        self.model = robot.model
        self.frame = franky_module.Frame.Flange
        self.identity = franky_module.Affine()

    def calculate(self, q: Any) -> tuple[np.ndarray, tuple[float, ...]]:
        joint_position = np.asarray(q, dtype=float).reshape(7)
        if not np.isfinite(joint_position).all():
            raise RuntimeError("joint encoder position is non-finite")
        pose = self.model.pose(self.frame, joint_position, self.identity, self.identity)
        matrix = _matrix(pose)
        quaternion = _tuple(pose.quaternion)
        if len(quaternion) != 4 or not all(math.isfinite(value) for value in quaternion):
            raise RuntimeError("encoder FK quaternion is invalid")
        return matrix, quaternion


def _encoder_pose_feedback(state: Any, encoder_fk: Any) -> EncoderPoseFeedback:
    started_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
    calculated, quaternion = encoder_fk.calculate(state.q)
    compute_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW) - started_ns
    # O_T_EE is only an audit comparison. Convert it to the same flange frame.
    reported = _matrix(state.O_T_EE) @ np.linalg.inv(_matrix(state.F_T_EE))
    position = calculated[:3, 3]
    reported_position = reported[:3, 3]
    rotation_delta = calculated[:3, :3].T @ reported[:3, :3]
    cosine = float(np.clip((np.trace(rotation_delta) - 1.0) / 2.0, -1.0, 1.0))
    return EncoderPoseFeedback(
        position_m=tuple(float(value) for value in position),
        orientation_xyzw=quaternion,
        reported_position_m=tuple(float(value) for value in reported_position),
        reported_position_error_m=float(np.linalg.norm(position - reported_position)),
        reported_orientation_error_rad=math.acos(cosine),
        compute_ns=compute_ns,
    )


def _snapshot(robot: Any, state: Any, encoder_fk: Any) -> Snapshot:
    feedback = _encoder_pose_feedback(state, encoder_fk)
    return Snapshot(
        host_monotonic_ns=time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW),
        robot_time_s=float(state.time.to_sec()),
        q=_tuple(state.q), dq=_tuple(state.dq), q_d=_tuple(state.q_d),
        dq_d=_tuple(state.dq_d), ddq_d=_tuple(state.ddq_d),
        flange_position_m=feedback.position_m,
        flange_orientation_xyzw=feedback.orientation_xyzw,
        ee_feedback_source=EE_FEEDBACK_SOURCE,
        ee_feedback_base_frame=EE_FEEDBACK_BASE_FRAME,
        ee_feedback_frame=EE_FEEDBACK_FRAME,
        reported_flange_position_m=feedback.reported_position_m,
        encoder_fk_reported_position_error_m=feedback.reported_position_error_m,
        encoder_fk_reported_orientation_error_rad=feedback.reported_orientation_error_rad,
        encoder_fk_compute_ns=feedback.compute_ns,
        f_t_ee=tuple(tuple(float(v) for v in row) for row in _matrix(state.F_T_EE)),
        load_mass_kg=float(state.m_load), mode=state.robot_mode.name,
        has_errors=bool(robot.has_errors), current_errors=repr(state.current_errors),
        last_motion_errors=repr(state.last_motion_errors), is_in_control=bool(robot.is_in_control),
        command_success_rate=float(state.control_command_success_rate),
    )
