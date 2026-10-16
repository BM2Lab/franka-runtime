"""Extracted hardware motion helpers; provenance: deployment/migration/phase2-extraction.json."""
from __future__ import annotations
import math
from dataclasses import dataclass
from pathlib import Path
import numpy as np
import yaml
from .reference_types import finite_vector, positive, unit_quaternion_xyzw
from .time_scaling import SmoothPhaseSchedule
from .trajectories import ApproachAndTrackTrajectory, CircleTrajectory, LissajousTrajectory

class _UniqueLoader(yaml.SafeLoader):
    pass


def _unique_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise ValueError(f"Duplicate YAML key: {key}")
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _unique_mapping)


def _load(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        document = yaml.load(stream, Loader=_UniqueLoader)
    if not isinstance(document, dict) or type(document.get("schema_version")) is not int or document["schema_version"] != 1:
        raise ValueError(f"Expected schema_version: 1 mapping: {path}")
    return document


def _keys(document: dict, allowed: set[str], required: set[str], name: str):
    if not isinstance(document, dict) or set(document) - allowed or required - set(document):
        raise ValueError(f"Invalid fields for {name}; allowed={sorted(allowed)}, required={sorted(required)}")


def _boolean(value, name):
    if type(value) is not bool:
        raise ValueError(f"{name} must be a YAML boolean")
    return value


@dataclass(frozen=True)
class TrajectorySpec:
    name: str
    data: dict

    def __post_init__(self):
        required = {"type", "center_m", "plane_rpy_deg", "quaternion_xyzw", "phase_rad", "turns",
                    "ramp_duration_s", "approach_duration_s", "settle_duration_s", "exit_duration_s", "return_to_start"}
        kind = self.data.get("type")
        if kind in ("joint_hold", "joint_ptp"):
            required = {"type", "target_position_rad", "hold_duration_s", "settle_duration_s",
                        "max_velocity_rad_s", "max_acceleration_rad_s2", "max_jerk_rad_s3"}
            _keys(self.data, required, required, self.name)
            finite_vector(self.data["target_position_rad"], 7, "target_position_rad")
            positive(self.data["hold_duration_s"], "hold_duration_s")
            positive(self.data["settle_duration_s"], "settle_duration_s")
            for key in ("max_velocity_rad_s", "max_acceleration_rad_s2", "max_jerk_rad_s3"):
                if np.any(np.array(finite_vector(self.data[key], 7, key)) <= 0):
                    raise ValueError(f"{key} must be positive")
            return
        extra = {"radius_m"} if kind == "circle" else {"amplitudes_m", "harmonics", "phase_offsets_rad"}
        _keys(self.data, required | extra, required | extra, self.name)
        if kind not in ("circle", "lissajous"):
            raise ValueError("M0 trajectory catalog supports circle and lissajous")
        for name in ("center_m", "plane_rpy_deg"):
            finite_vector(self.data[name], 3, name)
        unit_quaternion_xyzw(self.data["quaternion_xyzw"])
        for name in ("turns", "ramp_duration_s", "approach_duration_s", "exit_duration_s"):
            positive(self.data[name], name)
        positive(self.data["settle_duration_s"], "settle_duration_s", allow_zero=True)
        if not math.isfinite(self.data["phase_rad"]):
            raise ValueError("phase_rad must be finite")
        _boolean(self.data["return_to_start"], "return_to_start")
        if kind == "circle":
            positive(self.data["radius_m"], "radius_m")
        else:
            for name in ("amplitudes_m", "harmonics", "phase_offsets_rad"):
                values = np.array(finite_vector(self.data[name], 2, name))
                if name != "phase_offsets_rad" and np.any(values <= 0):
                    raise ValueError(f"{name} must be positive")
                if name == "harmonics" and np.any(values % 1):
                    raise ValueError("harmonics must be integers")

    def build(self, angular_speed_rad_s: float, initial_position_m, initial_quaternion_xyzw=None):
        d = self.data
        schedule = SmoothPhaseSchedule(angular_speed_rad_s, d["ramp_duration_s"], d["turns"], d["phase_rad"])
        common = dict(center_m=d["center_m"], plane_rpy_deg=d["plane_rpy_deg"],
                      quaternion_xyzw=d["quaternion_xyzw"], schedule=schedule)
        if d["type"] == "circle":
            path = CircleTrajectory(radius_m=d["radius_m"], **common)
        else:
            path = LissajousTrajectory(amplitudes_m=d["amplitudes_m"], harmonics=d["harmonics"],
                                       phase_offsets_rad=d["phase_offsets_rad"], **common)
        return ApproachAndTrackTrajectory(path, initial_position_m,
            initial_quaternion_xyzw=initial_quaternion_xyzw,
            **{key: d[key] for key in ("approach_duration_s", "settle_duration_s", "exit_duration_s", "return_to_start")})
