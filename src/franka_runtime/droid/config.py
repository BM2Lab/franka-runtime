"""Strict loader for droid.jointpos runtime configs. Unknown keys and silent defaults are rejected."""
import json
import math
from pathlib import Path

SECTIONS = {
    "schema_version", "runtime", "status", "backend", "executable", "motion_authorized", "approval",
    "provenance", "robot", "contract", "limits", "timing", "clock", "gripper", "network", "workspace",
    "mock_start_q_rad"}
REQUIRED = {
    "limits": ("joint_lower_rad", "joint_upper_rad", "position_margin_rad", "max_joint_velocity_rad_s",
               "max_joint_acceleration_rad_s2", "max_target_lead_rad"),
    "timing": ("control_hz", "hold_timeout_ms", "heartbeat_timeout_ms", "command_timeout_ms", "max_lease_s"),
    "clock": ("max_uncertainty_us", "min_probes"),
    "gripper": ("open_width_m", "grasp_force_n", "speed_m_s"),
    "network": ("bind_host", "port", "allowed_peers", "token_env"),
    "workspace": ("model_urdf", "base_link", "flange_link", "link_points", "tool_points_m", "table_z_m",
                  "clearance_m", "box_min_m", "box_max_m", "interpolation_steps"),
}


def _vector(value, field, positive=False):
    if not isinstance(value, list) or len(value) != 7 or not all(
            isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in value):
        raise ValueError(f"{field} must be seven finite numbers")
    if positive and any(v <= 0 for v in value):
        raise ValueError(f"{field} must be positive")
    return tuple(float(v) for v in value)


class DroidRuntimeConfig:
    def __init__(self, data, source="<memory>", config_dir=None):
        if not isinstance(data, dict) or not set(data) <= SECTIONS:
            raise ValueError(f"Unknown config sections: {sorted(set(data) - SECTIONS) if isinstance(data, dict) else data}")
        if data.get("schema_version") != 1 or data.get("runtime") != "droid_jointpos_v1":
            raise ValueError("Unsupported droid runtime config")
        self.data, self.source, self.config_dir = data, source, config_dir

    @classmethod
    def load(cls, path):
        path = Path(path)
        return cls(json.loads(path.read_text()), str(path), path.parent)

    def missing(self):
        """Fields that commissioning must still fill (null or absent)."""
        result = []
        for section, keys in REQUIRED.items():
            values = self.data.get(section) or {}
            result += [f"{section}.{k}" for k in keys if values.get(k) in (None, [])]
        return result

    def require_runnable(self):
        """D4 runs only the mock backend; the Franky backend and motion authorization are D6+ gated work."""
        missing = self.missing()
        if missing:
            raise ValueError(f"{self.source} is not runnable; unset: {', '.join(missing)}")
        if self.data.get("executable") is not True:
            raise ValueError(f"{self.source} is not executable")
        if self.data.get("backend") != "mock":
            raise ValueError("Only the mock backend exists in D4; the Franky backend requires D6 and authorization")
        if self.data.get("motion_authorized") is not False:
            raise ValueError("A mock config must not claim motion authorization")
        return self.validated()

    def validated(self):
        d = self.data
        limits, timing, clock, gripper, network = d["limits"], d["timing"], d["clock"], d["gripper"], d["network"]
        lower = _vector(limits["joint_lower_rad"], "joint_lower_rad")
        upper = _vector(limits["joint_upper_rad"], "joint_upper_rad")
        margin = _vector(limits["position_margin_rad"], "position_margin_rad", positive=True)
        if any(lo+m >= hi-m for lo, hi, m in zip(lower, upper, margin)):
            raise ValueError("Joint margins leave no usable range")
        for key in ("control_hz", "hold_timeout_ms", "heartbeat_timeout_ms", "command_timeout_ms", "max_lease_s",
                    "max_uncertainty_us", "min_probes"):
            value = (timing | clock)[key]
            if type(value) is not int or value <= 0:
                raise ValueError(f"{key} must be a positive integer")
        for key in ("open_width_m", "grasp_force_n", "speed_m_s"):
            if not isinstance(gripper[key], (int, float)) or isinstance(gripper[key], bool) or gripper[key] <= 0:
                raise ValueError(f"gripper.{key} must be positive")
        if (not isinstance(network["allowed_peers"], list) or not network["allowed_peers"]
                or not all(isinstance(p, str) and p for p in network["allowed_peers"])
                or type(network["port"]) is not int or not 0 <= network["port"] < 65536):
            raise ValueError("Network section needs explicit peers and port")
        if network["bind_host"] in ("0.0.0.0", "::", ""):
            raise ValueError("Bind the dedicated link interface explicitly, never all interfaces")
        workspace = d["workspace"]
        box_min, box_max = workspace["box_min_m"], workspace["box_max_m"]
        if (len(box_min) != 3 or len(box_max) != 3 or any(lo >= hi for lo, hi in zip(box_min, box_max))
                or workspace["clearance_m"] < 0 or type(workspace["interpolation_steps"]) is not int
                or workspace["interpolation_steps"] < 1 or not workspace["tool_points_m"]
                or any(len(point) != 3 for point in workspace["tool_points_m"])):
            raise ValueError("Workspace needs a valid box, clearance, tool points and interpolation steps")
        if not Path(workspace["model_urdf"]).is_absolute() and self.config_dir is None:
            raise ValueError("A relative model_urdf needs the config's directory")
        return {
            "robot_id": d["robot"]["robot_id"], "allowed_contracts": tuple(d["contract"]["allowed_sha256"]),
            "capability_prefix": d["contract"]["capability_prefix"],
            "lower": lower, "upper": upper, "margin": margin,
            "max_velocity": _vector(limits["max_joint_velocity_rad_s"], "max_joint_velocity_rad_s", positive=True),
            "max_acceleration": _vector(limits["max_joint_acceleration_rad_s2"], "max_joint_acceleration_rad_s2",
                                        positive=True),
            "max_target_lead": _vector(limits["max_target_lead_rad"], "max_target_lead_rad", positive=True),
            "control_period_ns": 1_000_000_000//timing["control_hz"], "hold_timeout_ns": timing["hold_timeout_ms"]*1_000_000,
            "heartbeat_timeout_ns": timing["heartbeat_timeout_ms"]*1_000_000,
            "command_timeout_ns": timing["command_timeout_ms"]*1_000_000, "max_lease_ns": timing["max_lease_s"]*1_000_000_000,
            "max_uncertainty_ns": clock["max_uncertainty_us"]*1_000, "min_probes": clock["min_probes"],
            "open_width_m": float(gripper["open_width_m"]), "grasp_force_n": float(gripper["grasp_force_n"]),
            "gripper_speed_m_s": float(gripper["speed_m_s"]),
            "bind_host": network["bind_host"], "port": network["port"], "allowed_peers": tuple(network["allowed_peers"]),
            "token_env": network["token_env"],
            "workspace": workspace, "config_dir": self.config_dir,
            "mock_start_q": _vector(d["mock_start_q_rad"], "mock_start_q_rad") if "mock_start_q_rad" in d else None,
        }
