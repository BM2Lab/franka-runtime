"""Extracted hardware motion helpers; provenance: deployment/migration/phase2-extraction.json."""
from __future__ import annotations
import hashlib
import math
from pathlib import Path
from typing import Any
JOINTS=7
TERMINAL={"complete","fault","stopped"}

class ConfigError(ValueError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ConfigError(message)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def vector(value: Any, size: int, name: str) -> list[float]:
    require(isinstance(value, list) and len(value) == size, f"{name} must have {size} values")
    result = [float(v) for v in value]
    require(all(math.isfinite(v) for v in result), f"{name} must be finite")
    return result


def _approval(data: dict[str, Any], require_approved: bool) -> None:
    if not require_approved:
        return
    require(data.get("status") == "approved", "status must be approved")
    require(data.get("executable") is True, "executable must be true")
    if data.get("execution_context") == "shadow":
        require(data.get("motion_authorized") is False,
                "shadow execution must explicitly set motion_authorized false")
    else:
        require(data.get("motion_authorized") is True, "motion_authorized must be true")
    approval = data.get("approval", {})
    for key in ("approved_by", "approved_at", "physical_stop_procedure", "workspace_review"):
        require(isinstance(approval.get(key), str) and approval[key].strip(), f"approval.{key} required")


def check_target(config: dict[str, Any], target: list[float] | tuple[float, ...], name: str = "target") -> list[float]:
    q = vector(list(target), JOINTS, name)
    lower, upper = config["safety"]["soft_lower_rad"], config["safety"]["soft_upper_rad"]
    require(all(lower[i] <= q[i] <= upper[i] for i in range(JOINTS)), f"{name} exceeds soft joint bounds")
    return q


def action_to_target(
    config: dict[str, Any], action: list[float], reference: list[float] | tuple[float, ...] | None = None
) -> list[float]:
    mapping = config["action_mapping"]
    action_size = int(mapping.get("policy_joint_count", JOINTS))
    raw = vector(list(action), action_size, "policy action")
    if mapping.get("type") != "normalized_position_increment":
        return check_target(config, [mapping["default_position_rad"][i] + mapping["scale_rad"][i] * raw[i]
                                     for i in range(JOINTS)], "mapped policy target")
    require(reference is not None, "incremental action requires the previous position reference")
    prior = vector(list(reference), JOINTS, "previous position reference")
    require(all(-1.0 <= value <= 1.0 for value in raw), "incremental policy action exceeds [-1, 1]")
    lower, upper = config["safety"]["soft_lower_rad"], config["safety"]["soft_upper_rad"]
    increment = mapping["max_position_increment_rad"]
    target = prior.copy()
    for i in range(action_size):
        target[i] = min(upper[i], max(lower[i], prior[i] + increment[i] * raw[i]))
    return target


def action_to_velocity(config: dict[str, Any], action: list[float]) -> list[float]:
    """Map a normalized velocity-policy action to a seven-joint velocity command."""
    mapping = config["action_mapping"]
    require(mapping.get("type") == "normalized_joint_velocity",
            "velocity action requires normalized_joint_velocity mapping")
    action_size = int(mapping["policy_joint_count"])
    raw = vector(list(action), action_size, "policy action")
    require(all(-1.0 <= value <= 1.0 for value in raw),
            "velocity policy action exceeds [-1, 1]")
    velocity = [0.0] * JOINTS
    for i, value in enumerate(raw):
        velocity[i] = value * mapping["max_reference_velocity_rad_s"][i]
    return velocity


def assemble_policy_observation(
    q, dq, target, flange, reference, previous, default, lower, upper, controlled_joint_count
) -> list[float]:
    q = vector(list(q), 7, "q"); dq = vector(list(dq), 7, "dq")
    target = vector(list(target), 3, "target"); flange = vector(list(flange), 3, "flange")
    reference = vector(list(reference), 7, "reference")
    previous = vector(list(previous), controlled_joint_count, "previous")
    default = vector(list(default), 7, "default")
    lower = vector(list(lower), 7, "lower"); upper = vector(list(upper), 7, "upper")
    reference_normalized = [2.0 * (reference[i] - lower[i]) / (upper[i] - lower[i]) - 1.0
                            for i in range(controlled_joint_count)]
    return ([q[i] - default[i] for i in range(7)] + dq +
            [target[i] - flange[i] for i in range(3)] + reference_normalized + previous)


