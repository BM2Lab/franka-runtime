# D6 — DROID runtime on the RT host: handoff checklist

For whoever continues on the RT host (`BMECHEN-URT`). Read
[droid-runtime.md](droid-runtime.md) first. The roadmap and the policy side are in
robot-learning's `docs/droid_handoff.md`. Nothing here authorizes motion. Every step
marked **gate** needs explicit operator authorization and
[hardware-qualification.md](hardware-qualification.md).

## Where things stand (2026-10-23)

| Component | State |
|---|---|
| robot-learning (inference host) | contracts 0.0.2; D0–D5 committed (`ec04c2d`); pinned pi05 server and runner verified in loopback against the mock |
| franka-runtime | 0.0.4 on branch `d4-droid-runtime`. **Uncommitted, not pushed.** DROID mock runtime: setpoint-lead checks, workspace check, executor, gripper intent, TCP service. 46 tests pass, 2 skipped (M4/PPO). |
| sensors | RealSense drivers (`2361570`); both cameras on the inference host |
| Franky backend for DROID | **not implemented** (this step) |
| Self-collision checks | **not implemented**; needs a vetted collision model |

## Hardware and network (set up 2026-10-23)

| Host | Interface | Use |
|---|---|---|
| RT host | `enp0s31f6` (built-in) | FCI to the FR3 (`Franka_Research_3_FCI` profile) |
| RT host | `enx00606e00f1bd` (USB) | direct cable to the inference host, `inference-link` profile, 10.77.0.2/30 |
| RT host | `wlp0s20f3` | eduroam, internet only |
| Inference host | `enp130s0` | direct cable, `rt-link` profile, 10.77.0.1/30 |
| Inference host | `wlp129s0f0` | campus Wi-Fi, internet |

Measured from the inference host:
- **Small packets:** 0.38 ms average round trip, 0.71 ms maximum, 0.08 ms jitter, 0% loss over 200.
- **1400-byte packets:** about 3 ms.
- **Link speed:** 100 Mb/s (USB 2 adapter or cable).

The `Defei_FR3_FCI` profile on the USB adapter belongs to a labmate; `inference-link`
outranks it through autoconnect priority. Coordinate before re-plugging that adapter.

Robot (read-only `echo_robot_state`):
- system version 5.9.2, Idle, no errors
- home pose `[0.0054, -0.7833, -0.0009, -2.3532, -0.0130, 1.5622, -0.0019]` rad
- reported EE position equals the bare flange, so **Desk has no end effector configured
  even though a Franka Hand is mounted.**

## 1. Get the code onto the RT host

franka-runtime's DROID work is uncommitted on the inference host. Choose one:

- **Commit and push** `d4-droid-runtime`, then clone it on the RT host. Simplest, and
  needs the maintainers' agreement to publish the branch.
- **Without publishing:** after committing on the inference host, run
  `git bundle create droid.bundle d4-droid-runtime` there, copy the bundle over the
  direct link, and clone from it on the RT host.

Environment on the RT host: an isolated Python 3.12 venv. Never reuse or copy the
inference host's venvs.

```bash
uv venv --python python3.12 "$RUNTIME_VENV"
uv pip install --python "$RUNTIME_VENV/bin/python" --no-deps \
  -e "$ROBOT_LEARNING/packages/contracts" -e "$FRANKA_RUNTIME"   # contracts 0.0.2 is stdlib-only
uv pip install --python "$RUNTIME_VENV/bin/python" "pytest>=8,<9"
"$RUNTIME_VENV/bin/python" -m pytest "$FRANKA_RUNTIME/tests" --import-mode=importlib -rs
```

`$ROBOT_LEARNING` must be checked out at `ec04c2d` or later; only `packages/contracts`
is needed. The pinned Franky fork wheel (`common/backends/franky-server10.json`) is
installed separately as the existing backends require. A Franky source tree exists at
`~/yifan/franky-source-f88f0e9b`; confirm it is the pinned build before using it.

## 2. Host hygiene (no robot motion)

- `sysctl net.ipv4.ip_forward` must be 0, so the RT host never routes between campus,
  the inference link and the robot.
- **Firewall (nftables):**
  - allow the runtime port (47700 in the mock-link config) only on `enx00606e00f1bd`
    from 10.77.0.1;
  - allow no inbound traffic on `wlp0s20f3` except what you choose (SSH);
  - leave FCI on `enp0s31f6` untouched.
- **Tailscale** is running on the RT host; decide whether it should be.
- **Wi-Fi:** consider turning it off while the robot runs. Check its effect on
  real-time latency with `cyclictest`.

## 3. Cross-host mock loopback (no robot)

Run the D4 mock on the RT host and the D5 runner on the inference host. This measures
clock uncertainty and latency over the real link with nothing connected to FCI.

```bash
# RT host. The token comes from a file and is also copied to the inference host as a chmod-600 file.
FRANKA_DROID_TOKEN="$(cat ~/.config/droid/token)" "$RUNTIME_VENV/bin/franka-droid-mock" \
  --config configs/fr3/droid.jointpos.mock-link.json --contract configs/fr3/droid_fr3_jointpos_v1.contract.json
```

On the inference host: start the pinned policy server (robot-learning
`docs/openpi_server.md`), then run `scripts/run_droid_runtime.py` with
`--runtime-host 10.77.0.2 --runtime-port 47700 --token-file <file>` (see robot-learning
`docs/droid_runner.md`).

Record: clock-estimate uncertainty, policy round trip, capture → chunk latency,
applied-receipt delay, and any underrun or heartbeat faults. Use these numbers, not the
mock's guesses, to set `timing.*` and `clock.max_uncertainty_us`.

## 4. Implement the Franky DROID backend (code, offline first)

Mirror `MockDroidBackend` and `MockArm` against the pinned Franky fork:

- **State:** measured q and dq, Franka Hand width, moving and grasped flags, and faults,
  produced as `FrankaStateSample`s.
- **Setpoint tracking:** the executor's current row target is the setpoint. A native
  1 kHz motion generator approaches it within `max_joint_velocity_rad_s`,
  `max_joint_acceleration_rad_s2` and the generator's jerk limit. Evaluate whether
  the existing `common/backends/franky_joint.py` position route can be reused for a
  7-joint target (the PPO route holds J7) before writing a new one. Preserve the native
  watchdog and the tracking-error reflexes.
- **Gripper:** asynchronous Franka Hand `move` for open and `grasp` for close, using
  the configured force and speed. It must never block the arm loop.
- **Stop:** real stop evidence (`joined`, `stop_finished`, `stop_at_rest`, `stop_fault`,
  `dropped_samples`) for `SessionAuthority.finish_stop`. Do not reuse the mock's
  immediate evidence.
- **Config:** a new `backend: franky` path in `droid.config`. It must refuse to start
  until every commissioning value is set and the approval fields are filled.
- **Tests:** offline tests with a fake Franky module, the way the existing M4/PPO
  backends are tested.

## 5. Commissioning values (fill `droid.jointpos.pending.json`)

| Field | How |
|---|---|
| `robot.robot_id`, serial, model | Desk system info; replace the `fr3_01` placeholder in config and contract consistently |
| End effector | **configure the Franka Hand in Desk** (mass, centre of mass, TCP), then re-read the EE pose |
| Joint limits and margins | verify the PPO-copied values for this unit and the hand |
| `max_joint_velocity_rad_s`, `max_joint_acceleration_rad_s2` | lab decision; start conservative (the PPO config used 0.435 rad/s) |
| `max_target_lead_rad` | from shadow-mode lead statistics (the loopback median was 0.11 rad, maximum 0.30) |
| `timing.*`, `clock.*` | from step 3 measurements |
| `gripper.open_width_m` | homed width after `homing()` (**gate**: gripper motion); must also update the robot-learning contract's `open_width_m` |
| `gripper.grasp_force_n`, `speed_m_s` | lab decision |
| `workspace.model_urdf` | export this unit's URDF including the hand |
| `workspace.tool_points_m`, `table_z_m`, `box_*`, `clearance_m` | measure in the robot base frame |
| `network.*` | 10.77.0.2, the chosen port, peer 10.77.0.1 |
| `approval.*` | operator, date, physical stop procedure, workspace review |

## 6. Gates

1. **Robot health (gate):** libfranka `communication_test` (it runs a control loop) and
   `cyclictest` under inference traffic.
2. **D7 shadow mode (gate):** read-only robot state, the full pipeline, and every
   acceptance, workspace and executor decision logged, with **no command sent**. Agree
   the pass criteria before running: lead and speed distributions, rejection causes,
   clock uncertainty, latency.
3. **D8 commissioning (gate):** hold and stop qualification, and gripper homing, per
   `hardware-qualification.md`.
4. **D9 trials (gate):** slow, inside a reduced workspace box, an operator on the
   e-stop, every episode recorded. Self-collision checks are required before this.
