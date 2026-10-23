import json
from pathlib import Path

import pytest

from contracts.droid import DroidIOSpec
from franka_runtime.droid.checks import check_rows
from franka_runtime.droid.config import DroidRuntimeConfig
from franka_runtime.droid.executor import UNDERRUN, ChunkExecutor
from franka_runtime.droid.gripper import GripperIntent

ROOT = Path(__file__).resolve().parents[2]
CONFIGS = ROOT / "configs/fr3"
Q = (0.0, 0.27, 0.0, -2.03, 0.0, 2.35, 0.08)
P = 66_666_667


def mock_config(**changes):
    data = json.loads((CONFIGS / "droid.jointpos.mock.json").read_text())
    for dotted, value in changes.items():
        section, key = dotted.split(".")
        data[section][key] = value
    return DroidRuntimeConfig(data, config_dir=CONFIGS)


def test_pending_config_is_not_runnable_and_lists_commissioning_gaps():
    pending = DroidRuntimeConfig.load(CONFIGS / "droid.jointpos.pending.json")
    missing = pending.missing()
    assert "limits.max_joint_velocity_rad_s" in missing and "gripper.open_width_m" in missing
    assert "limits.max_target_lead_rad" in missing and "limits.max_joint_acceleration_rad_s2" in missing
    assert "network.allowed_peers" in missing
    with pytest.raises(ValueError, match="not runnable"):
        pending.require_runnable()


def test_mock_config_runs_only_as_mock():
    settings = mock_config().require_runnable()
    assert settings["control_period_ns"] == 4_000_000 and settings["max_velocity"] == (1.0,)*7
    with pytest.raises(ValueError, match="Franky"):
        DroidRuntimeConfig({**mock_config().data, "backend": "franky"}, config_dir=CONFIGS).require_runnable()
    with pytest.raises(ValueError, match="all interfaces"):
        mock_config(**{"network.bind_host": "0.0.0.0"}).require_runnable()
    with pytest.raises(ValueError, match="Unknown"):
        DroidRuntimeConfig({**mock_config().data, "safety_override": True}, config_dir=CONFIGS)
    with pytest.raises(ValueError, match="authorization"):
        DroidRuntimeConfig({**mock_config().data, "motion_authorized": True}, config_dir=CONFIGS).require_runnable()


def test_setpoint_lead_checks_reject_range_lead_and_row_speed_without_clipping():
    limits = mock_config().require_runnable()
    rows = [(*[q+0.01*(i+1) for q in Q], 0.) for i in range(8)]
    peak = check_rows(rows, Q, limits, P)
    assert max(peak) == pytest.approx(0.15, rel=1e-3)
    beyond = list(rows[0]); beyond[3] = -0.2  # joint 4 upper -0.117 minus margin 0.148
    with pytest.raises(ValueError, match="joint 4"):
        check_rows([tuple(beyond)], Q, limits, P)
    leading = [(*[q+0.25 for q in Q], 0.)]  # 3.75 rad/s "from measured" under the old strict check
    assert check_rows(leading, Q, limits, P) == [0.]*7
    with pytest.raises(ValueError, match="leads measured"):
        check_rows([(*[q+0.35 for q in Q], 0.)], Q, limits, P)
    with pytest.raises(ValueError, match="rad/s"):
        check_rows([rows[0], (*[q+0.09 for q in Q], 0.)], Q, limits, P)  # 1.2 rad/s between rows


def rows(n, closure=0.):
    return tuple((*[q+0.01*i for q in Q], closure) for i in range(n))


def test_executor_row_grid_expiry_and_underrun():
    ex = ChunkExecutor(P, 200_000_000)
    assert ex.tick(0).target is None  # armed, never commanded: no hold timer
    ex.submit(2, rows(8), deadline_ns=10*P)
    first = ex.tick(5)
    assert first.started == 2 and first.row == 0 and first.row_started
    assert ex.tick(5+P-1).row == 0 and not ex.tick(5+P-1).row_started
    assert ex.tick(5+3*P).row == 3
    assert ex.tick(5+8*P).holding and ex.tick(5+8*P).target == rows(8)[7][:7]
    assert ex.tick(5+8*P+200_000_001).stop_reason == UNDERRUN
    with pytest.raises(RuntimeError):
        ex.submit(3, rows(8), deadline_ns=10**12)


def test_executor_deadline_truncates_rows_and_expires_pending():
    ex = ChunkExecutor(P, 200_000_000)
    ex.submit(2, rows(8), deadline_ns=3*P)
    ex.tick(0)
    assert ex.tick(2*P).row == 2 and ex.tick(3*P).holding  # row 3 would start at the deadline
    assert ex.tick(3*P+1).target == rows(8)[2][:7]


def test_sparse_ticks_hold_the_last_due_row():
    ex = ChunkExecutor(P, 200_000_000)
    ex.submit(2, rows(8), deadline_ns=10**12)
    ex.tick(0)
    ex.tick(P)
    assert ex.tick(20*P).target == rows(8)[7][:7]
    ex.submit(3, rows(8), deadline_ns=3*P+10)
    assert 3 in ex.tick(3*P+10).dropped


def test_executor_replaces_unstarted_rows_at_next_boundary():
    ex = ChunkExecutor(P, 200_000_000)
    ex.submit(2, rows(8), deadline_ns=10**12)
    ex.tick(0)
    ex.submit(3, rows(8, closure=1.), deadline_ns=10**12)
    ex.submit(4, rows(8, closure=1.), deadline_ns=10**12)  # replaces 3 before it starts
    mid = ex.tick(P//2)
    assert mid.sequence == 2 and mid.row == 0 and 3 in mid.dropped
    boundary = ex.tick(P)
    assert boundary.sequence == 4 and boundary.row == 0 and boundary.started == 4 and boundary.closure == 1.


def test_gripper_commands_only_on_transitions():
    spec = DroidIOSpec.from_json((CONFIGS / "droid_fr3_jointpos_v1.contract.json").read_text())
    intent = GripperIntent(spec.gripper, 0.08)
    assert intent.update(0.2) is None and intent.update(0.51) == "close" and intent.update(0.9) is None
    assert intent.update(0.5) == "open"
    assert GripperIntent(spec.gripper, 0.0).state == "close"


def test_mock_arm_approaches_leading_setpoint_within_limits():
    from franka_runtime.droid.mock import MockArm
    arm = MockArm(Q, (1.0,)*7, (4.0,)*7)
    target = tuple(q+0.25 for q in Q)
    speeds = []
    for _ in range(500):
        before = list(arm.dq)
        arm.advance(0.004, target)
        speeds.append(max(abs(v) for v in arm.dq))
        assert all(abs(a-b) <= 4.0*0.004+1e-9 for a, b in zip(arm.dq, before))
    assert max(speeds) <= 1.0+1e-9
    assert arm.q == pytest.approx(target, abs=1e-6) and arm.at_rest


HOME = (0.005415, -0.783251, -0.000904, -2.353230, -0.013001, 1.562240, -0.001859)


def workspace(**changes):
    from franka_runtime.droid.workspace import from_settings
    settings = dict(mock_config().require_runnable()["workspace"])
    settings.update(changes)
    return from_settings(settings, CONFIGS)


def test_workspace_fk_matches_robot_reported_home_flange():
    links, tool = workspace().points(HOME)
    assert links["link8"] == pytest.approx((0.306601, -0.000101, 0.590281), abs=1e-5)  # echo_robot_state EE
    assert all(p[2] == pytest.approx(0.590281-0.1034, abs=2e-3) for p in tool)  # hand points below the flange


def test_workspace_rejects_table_box_and_dipping_paths():
    check = workspace()
    check.check_configuration(HOME, "home")
    with pytest.raises(ValueError, match="below table"):
        workspace(table_z_m=0.45).check_configuration(HOME, "home")
    with pytest.raises(ValueError, match="outside workspace box"):
        workspace(box_min_m=[0.4, -0.6, 0.0]).check_configuration(HOME, "home")
    low = tuple(v+d for v, d in zip(HOME, (0, 1.6, 0, 0.5, 0, 0.8, 0)))  # FK: flange at z=0.056 m, hand below
    with pytest.raises(ValueError, match="row 0"):
        check.check_rows([(*low, 0.)], HOME)
    # Both ends are valid (and inside the joint margins); the joint-space path between them
    # leaves the box. Found by FK search. The measured state itself is never rejected:
    # moving out of a bad pose must stay possible.
    start = (0.082, 0.056, -0.001, -2.493, -0.013, 0.858, -0.002)
    end = (0.455, -0.466, -0.001, -2.911, -0.013, 1.924, -0.002)
    check.check_configuration(start, "start")
    check.check_configuration(end, "end")
    with pytest.raises(ValueError, match="before row 0"):
        check.check_rows([(*end, 0.)], start)
    check.check_rows([(*HOME, 0.)], low)  # leaving a bad measured pose toward home is allowed


def test_workspace_requires_connected_chain(tmp_path):
    from franka_runtime.droid.workspace import SerialChain
    with pytest.raises(ValueError, match="not connected"):
        SerialChain(ROOT / "tests/parity/fixtures/robot_returned_model.urdf", "link0", "missing_link")


def test_cross_host_mock_config_binds_only_the_direct_link():
    settings = DroidRuntimeConfig.load(CONFIGS / "droid.jointpos.mock-link.json").require_runnable()
    assert (settings["bind_host"], settings["port"], settings["allowed_peers"]) == ("10.77.0.2", 47700, ("10.77.0.1",))
