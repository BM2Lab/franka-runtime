"""Self-collision engine on the bundled franka_description FR3 v2.1 + Franka Hand model."""
import json
from pathlib import Path
import shutil

import pytest

pytest.importorskip("pinocchio")
from franka_runtime.droid.collision import SelfCollisionEngine  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
ASSETS = ROOT/"configs/fr3/droid-source-review/assets"
STATE = json.loads((ROOT/"tests/fixtures/droid/robot-state-with-hand-2026-10-24.json").read_text())  # read-only capture
JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
HOME = (0.005415, -0.783251, -0.000904, -2.353230, -0.013001, 1.562240, -0.001859)
HAND_INTO_BASE = (2.531, 0.617, 0.21, -2.715, -0.034, 1.943, 1.315)  # found by search; finger inside link0


def tool(**changes):
    c = STATE["F_T_EE"]
    value = dict(F_T_EE=[[c[col*4+row] for col in range(4)] for row in range(4)], m_ee=STATE["m_ee"], I_ee=STATE["I_ee"],
                 F_x_Cee=STATE["F_x_Cee"], m_load=STATE["m_load"], I_load=STATE["I_load"], F_x_Cload=STATE["F_x_Cload"],
                 scalar_tolerance=1e-6, transform_tolerance=1e-6)
    value.update(changes)
    return value


def engine(manifest=ASSETS/"collision/manifest.json", **kw):
    return SelfCollisionEngine(manifest, workspace_urdf=kw.get("urdf", ASSETS/"arm-contract-names.reference.urdf"),
                               joint_names=kw.get("joints", JOINTS), tool=kw.get("tool", tool()))


def test_bound_model_accepts_home_and_rejects_real_self_collision():
    e = engine()
    assert e.check_configuration(HOME) is True
    distance, first, second = e.closest_pair(HOME)
    assert 0.03 < distance < 0.05 and {first, second} == {"fr3v2.1_link5_0", "fr3v2.1_hand_0"}
    assert e.check_configuration(HAND_INTO_BASE) is False
    assert e.closest_pair(HAND_INTO_BASE)[0] < 0
    assert e.joint_names == tuple(JOINTS) and e.min_clearance_m == 0.02 and "franka_description 1.3.0" in e.provenance


def test_clearance_margin_rejects_near_contact():
    e = engine()
    near = e.closest_pair(HOME)[0]
    for request in e.geometry_data.collisionRequests:
        request.security_margin = near+0.001  # tighter than the measured home clearance
    assert e.check_configuration(HOME) is False


def test_binding_refuses_changed_files_tool_urdf_or_joint_names(tmp_path):
    bundle = tmp_path/"collision"
    shutil.copytree(ASSETS/"collision", bundle)
    mesh = bundle/"franka_description/meshes/robot_ee/franka_hand_white/collision/hand.stl"
    mesh.write_bytes(mesh.read_bytes()+b"\0")
    with pytest.raises(ValueError, match="changed"):
        engine(bundle/"manifest.json")
    bare_flange = tool(F_T_EE=[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    with pytest.raises(ValueError, match="Hand mount"):
        engine(tool=bare_flange)
    with pytest.raises(ValueError, match="workspace URDF"):
        engine(urdf=ROOT/"tests/parity/fixtures/robot_returned_model.urdf")  # same geometry, unrenamed joints
    with pytest.raises(ValueError, match="joint names"):
        engine(joints=[f"fr3_joint{i}" for i in range(1, 8)])
    with pytest.raises(ValueError):
        engine().check_configuration(HOME[:6])
