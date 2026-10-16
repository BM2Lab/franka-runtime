"""Extracted native PPO routes. No vendor import or implicit connection."""
from __future__ import annotations
import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable
from .feedback import (EncoderForwardKinematics, _snapshot, _tuple, _encoder_pose_feedback,
                      EE_FEEDBACK_SOURCE,EE_FEEDBACK_BASE_FRAME,EE_FEEDBACK_FRAME,Snapshot)
@dataclass(frozen=True)
class CallbackRecord:
    host_monotonic_ns: int
    robot_time_s: float
    time_step_s: float
    relative_time_s: float
    absolute_time_s: float
    q_command: tuple[float, ...]
    dq_command: tuple[float, ...]
    tau_command: tuple[float, ...]
    q: tuple[float, ...]
    dq: tuple[float, ...]
    q_d: tuple[float, ...]
    dq_d: tuple[float, ...]
    ddq_d: tuple[float, ...]
    tau_joint: tuple[float, ...]
    tau_joint_desired: tuple[float, ...]
    tau_external: tuple[float, ...]
    flange_position_m: tuple[float, ...]
    flange_orientation_xyzw: tuple[float, ...]
    ee_feedback_source: str
    ee_feedback_base_frame: str
    ee_feedback_frame: str
    reported_flange_position_m: tuple[float, ...]
    encoder_fk_reported_position_error_m: float
    encoder_fk_reported_orientation_error_rad: float
    encoder_fk_compute_ns: int
    mode: str
    current_errors: str
    last_motion_errors: str
    command_success_rate: float

class FrankyBackend:
    """One long-lived Franky torque motion with atomically replaced q/dq references."""
    def __init__(self, config, *, robot=None, franky_module=None):
        from copy import deepcopy
        self.config=deepcopy(config)
        if robot is None or franky_module is None:
            raise RuntimeError('PPO native backend requires an explicitly attached connection; no FCI connection in phase 3')
        from ..motion.legacy import _approval
        _approval(config, True)
        if config.get('execution_context') == 'shadow' or config.get('motion_authorized') is not True:
            raise ValueError('Native attachment requires explicit motion approval')
        from .franky_m4 import verify_native_capabilities
        verify_native_capabilities(config,franky_module)
        self._franky=franky_module
        self.robot=robot
        self._encoder_fk=EncoderForwardKinematics(robot,franky_module)
        self.impedance = config["robot"]["impedance_controller"]
        self.command_filter_cutoff_hz = float(self.impedance.get("command_filter_cutoff_hz", 100.0))
        self.stop_config = config["robot"]["torque_stop"]
        self.soft_lower = tuple(config["safety"]["soft_lower_rad"])
        self.soft_upper = tuple(config["safety"]["soft_upper_rad"])
        self.watchdog_ms = int(math.ceil(float(config["timing"]["command_watchdog_ms"])))
        self._motion_lock = threading.Lock()
        self._motion = None
        self._active_callback: Callable[[CallbackRecord], None] | None = None
        self._reference = _tuple(self.robot.state.q)
        self._velocity_reference = (0.0,) * 7
        self._last_target_ns = 0
        self._watchdog_tripped = False
        self._watchdog_error = ""
        self._watchdog_stop = threading.Event()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop, name="franky-command-watchdog", daemon=True)
        self._watchdog_thread.start()

    def _torque_stop_motion(self):
        s = self.stop_config
        return self._franky.TorqueStopMotion(
            damping=s["damping_nms_rad"], ramp_duration=float(s["ramp_duration_s"]),
            velocity_epsilon=float(s["velocity_epsilon_rad_s"]),
            max_duration=float(s["max_duration_s"]),
            compensate_coriolis=bool(s["compensate_coriolis"]),
            max_delta_tau=float(s["max_delta_tau_nm_per_ms"]),
        )

    def _watchdog_loop(self) -> None:
        interval_s = min(0.005, self.watchdog_ms / 4000.0)
        while not self._watchdog_stop.wait(interval_s):
            try:
                with self._motion_lock:
                    if (self._last_target_ns == 0 or
                            time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW) - self._last_target_ns
                            <= self.watchdog_ms * 1_000_000):
                        continue
                    self._watchdog_tripped = True
                    if self.robot.is_in_control:
                        self.robot.move(self._torque_stop_motion(), asynchronous=True,
                                        limit_rate=False, cutoff_frequency=100.0)
                    return
            except Exception as exc:
                self._watchdog_error = repr(exc)
                self._watchdog_tripped = True
                return

    def _check_watchdog(self) -> None:
        if self._watchdog_error:
            raise RuntimeError(f"command_watchdog_stop_error:{self._watchdog_error}")
        if self._watchdog_tripped:
            raise RuntimeError("command_watchdog")

    def read_state(self) -> Snapshot:
        self._check_watchdog()
        return _snapshot(self.robot, self.robot.state, self._encoder_fk)

    def keepalive(self) -> None:
        with self._motion_lock:
            self._check_watchdog()
            if not self.robot.is_in_control:
                raise RuntimeError("control ended while holding impedance reference")
            self._last_target_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)

    def _create_motion(self):
        f, c = self._franky, self.impedance
        friction = f.FrictionCompensationParams(
            coulomb=c["friction"]["coulomb_nm"],
            viscous=c["friction"]["viscous_nms_rad"],
            max_torque=c["friction"]["max_torque_nm"],
            velocity_epsilon=float(c["friction"]["velocity_epsilon_rad_s"]),
        )
        motion = f.JointImpedanceTrackingMotion(
            stiffness=c["stiffness_nm_rad"], damping=c["damping_nms_rad"],
            constant_torque_offset=c["constant_torque_offset_nm"],
            lower_joint_limits=self.soft_lower, upper_joint_limits=self.soft_upper,
            compensate_coriolis=bool(c["compensate_coriolis"]),
            max_delta_tau=float(c["max_delta_tau_nm_per_ms"]),
            joint_limit_activation_distance=float(c["joint_limit_activation_distance_rad"]),
            joint_limit_stiffness=float(c["joint_limit_stiffness_nm"]),
            joint_limit_damping=float(c["joint_limit_damping_nms_rad"]),
            joint_limit_max_torque=float(c["joint_limit_max_torque_nm"]),
            friction=friction, gains_time_constant=float(c["gains_time_constant_s"]),
        )
        resolved_error_clip = _tuple(motion.params.error_clip)
        expected_error_clip = tuple(float(v) for v in c["expected_error_clip_rad"])
        if resolved_error_clip != expected_error_clip:
            raise RuntimeError(
                f"Franky error clip mismatch: expected {expected_error_clip}, got {resolved_error_clip}")
        return motion

    def _on_update(self, state, time_step, relative_time, absolute_time, command) -> None:
        callback = self._active_callback
        if callback is None:
            return
        if self._motion is not None and hasattr(self._motion, "get_applied_reference"):
            applied = self._motion.get_applied_reference()
            if applied is not None:
                self._reference = _tuple(applied.q)
                self._velocity_reference = _tuple(applied.dq)
        feedback = _encoder_pose_feedback(state, self._encoder_fk)
        callback(CallbackRecord(
            host_monotonic_ns=time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW),
            robot_time_s=float(state.time.to_sec()), time_step_s=float(time_step.to_sec()),
            relative_time_s=float(relative_time.to_sec()), absolute_time_s=float(absolute_time.to_sec()),
            q_command=self._reference, dq_command=self._velocity_reference,
            tau_command=_tuple(command.tau_J), q=_tuple(state.q), dq=_tuple(state.dq),
            q_d=_tuple(state.q_d), dq_d=_tuple(state.dq_d), ddq_d=_tuple(state.ddq_d),
            tau_joint=_tuple(state.tau_J), tau_joint_desired=_tuple(state.tau_J_d),
            tau_external=_tuple(state.tau_ext_hat_filtered),
            flange_position_m=feedback.position_m,
            flange_orientation_xyzw=feedback.orientation_xyzw,
            ee_feedback_source=EE_FEEDBACK_SOURCE,
            ee_feedback_base_frame=EE_FEEDBACK_BASE_FRAME,
            ee_feedback_frame=EE_FEEDBACK_FRAME,
            reported_flange_position_m=feedback.reported_position_m,
            encoder_fk_reported_position_error_m=feedback.reported_position_error_m,
            encoder_fk_reported_orientation_error_rad=feedback.reported_orientation_error_rad,
            encoder_fk_compute_ns=feedback.compute_ns,
            mode=state.robot_mode.name, current_errors=repr(state.current_errors),
            last_motion_errors=repr(state.last_motion_errors),
            command_success_rate=float(state.control_command_success_rate),
        ))

    def send_target(self, target: list[float], callback: Callable[[CallbackRecord], None]) -> None:
        f = self._franky
        q = tuple(float(v) for v in target)
        dq = (0.0,) * 7
        with self._motion_lock:
            self._check_watchdog()
            self._active_callback = callback
            if self._motion is None:
                self._motion = self._create_motion()
                seed = _tuple(self.robot.state.q)
                self._reference, self._velocity_reference = seed, dq
                self._motion.set_reference(f.JointReference(q=seed, dq=dq))
                self._motion.register_callback(self._on_update)
                self._last_target_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
                self.robot.move(self._motion, asynchronous=True,
                                limit_rate=False, cutoff_frequency=self.command_filter_cutoff_hz)
            self._reference, self._velocity_reference = q, dq
            self._motion.set_reference(f.JointReference(q=q, dq=dq))
            self._last_target_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)

    def smooth_stop(self, timeout_s: float = 5.0) -> bool:
        self._watchdog_stop.set()
        self._watchdog_thread.join(timeout=0.1)
        with self._motion_lock:
            if not self.robot.is_in_control:
                return True
            if not self._watchdog_tripped:
                self.robot.move(self._torque_stop_motion(), asynchronous=True,
                                limit_rate=False, cutoff_frequency=100.0)
        return bool(self.robot.join_motion(timeout_s))

    def close(self) -> None:
        self._watchdog_stop.set()
        self._watchdog_thread.join(timeout=0.1)
        self._motion = None
        self.robot = None

class FrankyVelocityImpedanceBackend(FrankyBackend):
    """Explicit velocity-reference impedance control with 1 kHz C++ integration."""

    def _create_motion(self):
        f, c = self._franky, self.impedance
        if not hasattr(f, "JointVelocityImpedanceTrackingMotion"):
            raise RuntimeError(
                "installed Franky lacks JointVelocityImpedanceTrackingMotion; "
                "install the pinned franky velocity-impedance extension")
        friction = f.FrictionCompensationParams(
            coulomb=c["friction"]["coulomb_nm"],
            viscous=c["friction"]["viscous_nms_rad"],
            max_torque=c["friction"]["max_torque_nm"],
            velocity_epsilon=float(c["friction"]["velocity_epsilon_rad_s"]),
        )
        motion = f.JointVelocityImpedanceTrackingMotion(
            stiffness=c["stiffness_nm_rad"], damping=c["damping_nms_rad"],
            constant_torque_offset=c["constant_torque_offset_nm"],
            lower_joint_limits=self.soft_lower, upper_joint_limits=self.soft_upper,
            compensate_coriolis=bool(c["compensate_coriolis"]),
            max_delta_tau=float(c["max_delta_tau_nm_per_ms"]),
            joint_limit_activation_distance=float(c["joint_limit_activation_distance_rad"]),
            joint_limit_stiffness=float(c["joint_limit_stiffness_nm"]),
            joint_limit_damping=float(c["joint_limit_damping_nms_rad"]),
            joint_limit_max_torque=float(c["joint_limit_max_torque_nm"]),
            friction=friction, gains_time_constant=float(c["gains_time_constant_s"]),
        )
        resolved_error_clip = _tuple(motion.params.error_clip)
        expected_error_clip = tuple(float(v) for v in c["expected_error_clip_rad"])
        if resolved_error_clip != expected_error_clip:
            raise RuntimeError(
                f"Franky error clip mismatch: expected {expected_error_clip}, got {resolved_error_clip}")
        return motion

    def synchronize_reference(self, reference: list[float] | tuple[float, ...]) -> None:
        q = _tuple(reference)
        if len(q) != 7 or not all(math.isfinite(value) for value in q):
            raise RuntimeError("velocity-impedance reference seed must contain seven finite values")
        with self._motion_lock:
            if self._last_target_ns != 0 or self.robot.is_in_control:
                raise RuntimeError("velocity-impedance reference can only be seeded before motion")
            self._reference = q
            self._velocity_reference = (0.0,) * 7

    def current_reference(self) -> tuple[float, ...]:
        with self._motion_lock:
            if self._motion is not None:
                applied = self._motion.get_applied_reference()
                if applied is not None:
                    self._reference = _tuple(applied.q)
                    self._velocity_reference = _tuple(applied.dq)
            return self._reference

    def send_velocity(self, velocity: list[float], callback: Callable[[CallbackRecord], None]) -> None:
        dq = _tuple(velocity)
        if len(dq) != 7 or not all(math.isfinite(value) for value in dq):
            raise RuntimeError("velocity command must contain seven finite values")
        with self._motion_lock:
            self._check_watchdog()
            self._active_callback = callback
            if self._motion is None:
                self._motion = self._create_motion()
                self._motion.set_initial_reference(self._reference)
                self._motion.set_velocity(dq)
                self._motion.register_callback(self._on_update)
                self._last_target_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
                self.robot.move(self._motion, asynchronous=True,
                                limit_rate=False, cutoff_frequency=self.command_filter_cutoff_hz)
            else:
                self._motion.set_velocity(dq)
            self._velocity_reference = dq
            self._last_target_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)

    def send_target(self, target: list[float], callback: Callable[[CallbackRecord], None]) -> None:
        del target, callback
        raise RuntimeError("velocity-impedance backend requires send_velocity")
