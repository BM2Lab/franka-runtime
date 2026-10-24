# D6 — DROID runtime on the RT host: handoff checklist

For whoever continues on the RT host (`BMECHEN-URT`). Read
[droid-runtime.md](droid-runtime.md) first. The roadmap and the policy side are in
robot-learning's `docs/droid_handoff.md`. Nothing here authorizes motion. Every step
marked **gate** needs explicit operator authorization and
[hardware-qualification.md](hardware-qualification.md).

This revision selects **continuous DROID-style hybrid joint impedance in Franky**
as the first physical-controller baseline. It supersedes this handoff's earlier
Ruckig/motion-generator proposal and the corresponding future-backend descriptions
in `droid-runtime.md` and robot-learning's interface proposal. Those documents still
describe the current mock correctly; no physical backend is implemented by this plan.

## Where things stand (2026-10-23)

| Component | State |
|---|---|
| robot-learning (inference host) | upstream `08d64c6` includes the D0–D5 work (`ec04c2d`) and RT-host handoff; contracts 0.0.2; pinned pi05 server and runner verified in loopback against the mock |
| franka-runtime | 0.0.4; branch `d4-droid-runtime` is already on this RT host, tracking origin at `fc4e9fa`. DROID mock runtime: setpoint-lead checks, workspace check, executor, gripper intent, TCP service. Inference-host handoff reports 46 tests passing, 2 skipped (M4/PPO); rerun in the new isolated RT environment. |
| sensors | RealSense drivers (`2361570`); both cameras on the inference host |
| Franky backend for DROID | Native wheel + injected adapter implemented and tested offline; physical config/device selector remains disabled |
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

The DROID branch is already checked out here at `fc4e9fa`. Record the exact runtime,
contracts, policy and native artifact revisions before building the D6 candidate.
Use a separate upstream robot-learning checkout/worktree for DROID: the existing
`~/robot-learning` working tree contains uncommitted phase-1/3 M4/PPO contracts
and learning extraction work (candidate 0.0.3). Preserve it; do not reset it or
silently downgrade its environment to satisfy DROID's 0.0.2 pin.

Environment on the RT host: an isolated Python 3.12 venv. Never reuse or copy the
inference host's venvs.

```bash
uv venv --python python3.12 "$RUNTIME_VENV"
uv pip install --python "$RUNTIME_VENV/bin/python" --no-deps \
  -e "$ROBOT_LEARNING/packages/contracts" -e "$FRANKA_RUNTIME"   # contracts 0.0.2 is stdlib-only
uv pip install --python "$RUNTIME_VENV/bin/python" "pytest>=8,<9"
"$RUNTIME_VENV/bin/python" -m pytest "$FRANKA_RUNTIME/tests" --import-mode=importlib -rs
```

`$ROBOT_LEARNING` must point to the separate, pinned upstream DROID checkout
(`08d64c6` reviewed here, containing `ec04c2d`); only `packages/contracts` is needed.
The upstream 0.0.2 distribution does not contain our local `contracts.motion` and
`contracts.franka_policy` additions. Keep M4/PPO preservation and reconciliation
separate; do not advertise their skipped tests as passing or change package
versions merely to make a dependency resolver accept incompatible contents.

The pinned Franky fork wheel (`common/backends/franky-server10.json`) is installed
separately as the existing backends require. A Franky source tree exists at
`~/yifan/franky-source-f88f0e9b`; confirm its source revision, installed wheel hashes
and shaping API exposure before using it. Read the canonical host policy at
`~/franka-system-inventory/AGENT_POLICY.md` before any install/build/path change.
Preserve both frozen originals. Any needed Franky extension/build stays in the
owning project with a separate candidate wheel/venv; do not upgrade libfranka or
Ruckig as a side effect.

### Step 1 completed on the RT host (2026-10-23)

The isolated setup is under `~/yifan/droid-deployment/`; its local README and
`validation/step1.json` record the exact wheel hashes and reconstruction inputs.

- Clean detached source pins: robot-learning `08d64c6` and runtime base `fc4e9fa`.
- Python 3.12 environment: `~/yifan/venvs/franka-droid-d6-py312`.
- Installed distributions: upstream DROID contracts 0.0.2 and runtime 0.0.4, from
  wheels rather than editable links. NumPy/PyYAML are installed only for existing
  cross-slice numerical tests; neither learning nor a native Franky wheel is installed.
- Validation: **114 passed, 2 skipped** across contracts/runtime. The skips are
  M4/PPO tests requiring the local contracts modules absent from upstream 0.0.2.
  Imports were checked with optional/native modules and socket/thread creation
  blocked, and the pending physical config correctly refuses to run.
- A mock-service clock race was exposed: a control timestamp taken before waiting
  for the submit lock could precede publication, falsely rejecting an application
  receipt. A minimal local patch now samples time under that lock. Its deterministic
  regression fails against the original wheel and passes against the candidate.
  Base and patched wheels are saved separately; the candidate is `fc4e9fa` plus
  `artifacts/d6-step1-clock-fix.patch`, not an unmodified upstream release.
- The original dirty `~/robot-learning` M4/PPO work, its environments, both frozen
  compatibility stacks and all commissioning/operating values were preserved.
  No FCI read, motion, gripper operation, network reconfiguration or native build
  was performed for this step. The robot can remain powered off for the next
  shaping-API audit and offline torque-fixture work.

### Controller audit completed (implementation-order step 2, offline)

See [DROID controller audit](droid-controller-audit.md) for the pinned upstream
sources, API mapping, numerical fixtures, processing differences and proposed
physical config schema. The existing modified wheel supports explicit joint and
Cartesian gains plus all seven targets; 23 native hashes match its pin. Across
32 supplied-J fixtures the unclipped feedback equations match to 2.85e-14 Nm;
clipping is recorded as an intentional difference. These are source-expression
checks, not execution/qualification of the wheel torque callback or robot model.

The wheel has **published-reference readback only**, with no consumed sequence,
tick or timestamp. It also lacks the DROID publisher watchdog and dedicated native
measured-speed/command-torque guards in this tracking motion. A minimal
project-owned native extension remains necessary before backend integration.
No native rebuild/install, hardware access, operating-limit change or host-network
change was performed. The physical schema is specified for implementation; no
pending physical config was made runnable.

### Native candidate implementation (offline, not deployed)

A separate `DroidJointImpedanceMotion` candidate now lives in the project-owned
Franky fork, alongside the unchanged legacy tracking API. It reuses the joint
impedance/shaping law and adds native publication/row identities, first-consumption
sequence/tick/CLOCK_MONOTONIC evidence, independent producer freshness/deadline
checks, callback/state/raw-torque guards, bounded telemetry and latched stop
status. Same-row heartbeat publications cannot change its target or extend its
expiry. No physical thresholds or pending configs were approved.

Implementation and stop limitations are documented at
`~/yifan/franky-source-f88f0e9b/docs/droid-joint-runtime.md`. The project build and
validation receipts live under `~/yifan/droid-deployment/`; original and dev66
installed environments remain unchanged. A tested source/build candidate is not
a deployable pinned wheel or a qualified physical backend. Next: package/pin it in
a separate DROID native environment, implement backend injection and fake-backed
service/hand integration, and introduce the reviewed physical config schema.
Do not emit physical `applied` receipts using the still-mock-only service.

Validation receipt: `~/yifan/droid-deployment/validation/step3-native.json`.
11 DROID C++ cases/47 assertions and 23 existing M4 cases/82 assertions pass;
the new native binding constructor/mailbox checks pass. Runtime/contracts tests:
148 passed, 2 documented M4/PPO-contract skips. All 23 installed dev66 native
hashes remain unchanged. The raw candidate links read-only server10 prefixes;
it is not a repaired wheel and makes no binary-dependency-equivalence claim for
dev66's bundles. Explicit per-process dependency paths and resolved hashes are
recorded. No global environment or installed native library was changed.


### Packaging and service integration completed (offline)

See [DROID backend integration](droid-backend-integration.md). The candidate is
installed in `~/yifan/venvs/franka-droid-native-server10-py312`; launch isolated
Python with `~/yifan/droid-deployment/run-native-python`. The native wheel is
`2.0.1.dev67+gdroidd6.libfranka.0.21.2`, repacked from the verified dev66 wheel with
only Franky core/extension replaced. All **21 bundled dependency payloads**,
including libfranka/Ruckig, remain byte-identical. Installed loading no longer
requires ROS or frozen-prefix paths. Existing wheels/environments and originals
remain unchanged; no hardware or host-network operation was performed.

The runtime now injects backend state/consumption/fault/stop evidence and has a
native row/receipt adapter plus serialized blocking hand worker. It confirms first
application before extending an already-applied target's bounded hold, records
actual native execution identity, and requires join/rest evidence for stop.
Physical backends are rejected by the mock config/CLI; fake-device integration is
not commissioning or a runnable physical configuration.

Validation against installed wheels: **167 passed, 2 skipped** (the same upstream
contracts M4/PPO omissions), plus native API/hash checks and **11 DROID / 23 M4**
C++ cases against the packaged dependencies. Passive imports and pending-config
refusal pass. `pip check` passes. Receipt and reconstruction inputs are at
`~/yifan/droid-deployment/validation/step4.json`, `native-requirements.lock` and
`artifacts/native-wheelhouse/`.

### Physical schema and device assembly implemented (offline, step 5)

See [physical configuration and factory](droid-physical-configuration.md). Physical
schema v2 separates target admission from measured native guards and leaves all
commissioning values null. The explicit factory checks contract/artifacts,
identity, tool/load/TCP and geometry, requires an injected collision checker,
and refreshes measured q/dq/Hand state at preflight. Device construction is
separately authorized because Franky's Robot constructor sets collision thresholds.
A concurrent Hand reader/command/cancellation worker replaces the serialized
feedback limitation for this candidate; measured freshness and actual worker exit
remain mandatory. Hardware feedback/cancellation timing is not yet qualified.

The physical selector remains disabled. Next: bounded durable recording and
supervised physical lifecycle, concrete identity/collision adapters, cross-host
mock timing, then D7/D8/D9 commissioning. No native wheel/dependency or frozen
baseline changed in step 5; the robot can remain off.

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

### 4.1 Controller baseline and upstream reference

Reuse the existing native `JointImpedanceTrackingMotion` / `JointImpedanceBase`
in the Franky fork with Cartesian shaping enabled. Do not adopt another Polymetis
installation on this machine. Use DROID's published Polymetis implementation as
an offline numerical reference, not as another physical robot driver.

The intended feedback law, before native safety/torque processing, is:

$$
K_{eff}(q)=K_q+J(q)^T K_x J(q),\qquad
D_{eff}(q)=D_q+J(q)^T D_x J(q),
$$

$$
\tau_{raw}=K_{eff}(q)(q_d-q)+D_{eff}(q)(0-\dot q)+c(q,\dot q).
$$

Here `J` is the measured-configuration end-effector geometric Jacobian, with its
frame, TCP and linear/angular ordering explicitly matched to the reference.
`K_q`, `D_q` are joint gains; `K_x`, `D_x` shape the joint-space gains through `J`.
This is not a Cartesian pose-error controller, a nullspace projector or an
operational-space inertia controller. Franky computes Coriolis when enabled;
robot firmware supplies gravity compensation, so do not add gravity twice.

DROID's normal nonblocking path starts a continuous
`HybridJointImpedanceControl` and updates `joint_pos_desired`. Its desired joint
velocity remains zero. That supplies damping; it does **not** require the arm to
reach rest at every 15 Hz row. The position setpoint can lead the measured arm.

Published DROID FR3 reference gains inspected on 2026-10-23 are:

| Gain | Reference values (diagonal) |
|---|---|
| `K_q` (Nm/rad) | `[40, 30, 50, 25, 35, 25, 10]` |
| `D_q` (Nm s/rad) | `[4, 6, 5, 5, 3, 2, 1]` |
| `K_x` (translation N/m; rotation Nm/rad) | `[400, 400, 400, 15, 15, 15]` |
| `D_x` (translation N s/m; rotation Nm s/rad) | `[37, 37, 37, 2, 2, 2]` |

These are provenance/reference values, **not commissioned settings for this unit**.
Use explicit damping, not Franky's automatically selected damping. Do not carry
forward the old PPO's uniform K=100/D=20 and call it DROID-equivalent.

Sources to pin before generating fixtures:

- [DROID command path](https://github.com/droid-dataset/droid/blob/main/droid/franka/robot.py)
  and [FR3 configuration](https://github.com/droid-dataset/droid/blob/main/config/fr3/franka_hardware.yaml).
  Record the DROID commit; the links track `main`.
- DROID's Polymetis submodule revision: `0a01a7fa7a7c65b2f9a3aebf5e79040940daf9d2`.
  [Controller selection and setpoint updates](https://github.com/facebookresearch/fairo/blob/0a01a7fa7a7c65b2f9a3aebf5e79040940daf9d2/polymetis/polymetis/python/polymetis/robot_interface.py),
  [hybrid feedback law](https://github.com/facebookresearch/fairo/blob/0a01a7fa7a7c65b2f9a3aebf5e79040940daf9d2/polymetis/polymetis/python/torchcontrol/modules/feedback.py),
  [impedance policy](https://github.com/facebookresearch/fairo/blob/0a01a7fa7a7c65b2f9a3aebf5e79040940daf9d2/polymetis/polymetis/python/torchcontrol/policies/impedance.py),
  [C++ torque client](https://github.com/facebookresearch/fairo/blob/0a01a7fa7a7c65b2f9a3aebf5e79040940daf9d2/polymetis/polymetis/src/clients/franka_panda_client/franka_panda_client.cpp).

### 4.2 Native execution and service integration

Implement a DROID backend interface first, then inject either the mock or Franky
implementation into `DroidRuntimeService`. Remove mock-specific assumptions from
state sampling, stop completion, version reporting and hand commands. Keep
`ChunkExecutor` pure/time-injected and retain the existing wire/session contracts.

- **State:** measured encoder q/dq and real Franka Hand width, moving/grasped flags
  and faults, produced as `FrankaStateSample`s at least 50 Hz. Validate the mapping
  from policy names `panda_joint1..7` to this FR3's physical joint order. DROID uses
  `CLOCK_MONOTONIC`; explicitly map any native RAW timestamps instead of relabelling them.
- **References:** stream all seven absolute targets from the active executor row
  into one long-lived native 1 kHz impedance motion. Seed its startup reference
  with measured q; set desired velocity to zero for this baseline. Preserve row
  replacement, expiry and bounded hold semantics. Reuse the low-level position
  adapter where appropriate, but not the six-action PPO mapping or its held J7.
- **No Ruckig in the first baseline:** no separate `JointMotion` per row, no
  point-to-point stop at each target, and no inferred velocity feedforward or
  lookahead trajectory blending. Those would be separately named controller
  variants requiring their own tests and qualification.
- **Watchdogs:** supervise the local reference publisher/executor and native
  control health independently of the 15 Hz row changes and chunk arrivals.
  Do not copy the PPO's 50 ms action-arrival timeout onto 66.7 ms DROID rows.
  A watchdog refresh must prove producer/executor liveness; a native loop must
  not refresh its own stale reference indefinitely. Keep network heartbeat,
  chunk deadline, hold timeout and native reference freshness distinct.
- **Applied receipts:** the mock's Python row-start acknowledgement is not native
  evidence. Require native-consumed reference sequence/tick/time before issuing a
  physical `applied` receipt for the chunk's first row. The old joint tracking API
  exposes last-published q/dq, not consumption evidence, so implement a minimal native
  mailbox/telemetry extension. Never fabricate an acknowledgement from a successful
  `set_reference()` call. Keep row scheduling versus native consumption visible.
- **Gripper:** dedicate a serialized worker to blocking Franka Hand `move`/`grasp`
  operations on closure transitions. Arm control must never wait for them. Define
  pending-command replacement, error handling and stop/cancel behavior explicitly;
  do not assume an API called `move_async` returns promptly or preempts instantly.
- **Stop:** real stop evidence (`joined`, `stop_finished`, `stop_at_rest`,
  `stop_fault`, `dropped_samples`) for `SessionAuthority.finish_stop`. Preserve a
  reviewed native stop path, latched faults and explicit fresh-session admission.
  Do not reuse mock evidence or upstream automatic error recovery.
- **Recording:** bounded native/off-callback telemetry for commanded and consumed
  rows, q/dq, torque stages, timing, guards and stop evidence. Serialize/write off
  the native control callback; report overflow/backpressure and fault as specified.
- **Config:** implement `backend: franky` only after its schema, artifact checks and
  backend exist. Require all commissioning/approval values and the exact native
  artifact pin. Keep the pending JSON nonexecutable; this document does not fill
  or approve it.

### 4.3 Motion limits and parity: explicit distinctions

Direct impedance setpoints do not guarantee a reference trajectory whose velocity,
acceleration and jerk are bounded. A 15 Hz setpoint jump is allowed by this design;
`dq_desired=0` does not change that fact. Replace the previous planned generator
promise with distinct, configured checks:

| Quantity | Physical baseline meaning |
|---|---|
| Target range/margin | Reject the entire chunk outside commissioned joint bounds; never clip policy targets |
| Row-0 lead | Bound `abs(q_target[0] - q_measured)` at acceptance and dispatch |
| Consecutive-row speed | Bound `abs(q_target[k+1] - q_target[k]) / row_period`; this is an acceptance statistic, not measured velocity |
| Measured joint speed | Native guard on encoder dq with a reviewed stop response |
| Measured acceleration | If required, define the estimator/filter, thresholds and stop behavior; do not claim a hard acceleration bound from PD or torque filtering |
| Reference jerk | No trajectory-generator jerk guarantee in this baseline; a bounded-trajectory variant would be separate work |
| Torque processing | Explicit torque magnitude/slew/filter behavior and tracking/limit guards, independently qualified |

The existing `max_joint_velocity_rad_s` and `max_joint_acceleration_rad_s2` currently
also parameterize `MockArm`'s kinematic approach. Plan a schema revision with separate
names/units for target acceptance and measured-state guards; document migration
and runner compatibility before implementation. Do not silently repurpose the
acceleration field or remove a guard to get a trial running. The mock's braking
model remains useful for protocol tests, but is not a model of physical impedance.

Before connecting a robot, produce offline fixtures and a difference report:

1. Match feedback torques for identical q, dq, q targets, gains and Jacobians;
   include seven-joint commands, coupled Jacobians, changing configurations and
   both translational/rotational shaping. Compare with the pinned upstream law.
2. Separate algebraic feedback parity from complete controller parity. Record
   differences in model/TCP, clipping, gain startup/smoothing, friction, offsets,
   joint-limit repulsion, torque magnitude bounds, slew and low-pass filtering.
   DROID's published config enables libfranka rate limiting and a 100 Hz filter;
   preserve the existing Franky processing order unless a separately reviewed
   change is needed. Matching the equation alone is not behavioral equivalence.
3. Test chunk replacement, deadline/hold/heartbeat expiry, producer death, callback
   gaps, native sequence acknowledgements, queue overflow and terminal state.
   Verify reference transitions, native faults and stop completion/timeout using
   fake native/hand backends and injected clocks. Import/tests must never connect.
4. Audit the existing pinned wheel's shaping API first. If it cannot supply the
   required parameter mapping, receipt evidence or native guards, make the minimum
   project-owned Franky extension and pin/test a new candidate artifact. Do not
   replace an original baseline or quietly weaken the requirements.

D6's offline exit is a tested backend candidate, dependency/native receipts and an
explicit parity/difference report, plus cross-host mock timing measurements. It
is not physical qualification and does not establish zero-shot task success.

## 5. Commissioning values (fill `droid.jointpos.pending.json`)

| Field | How |
|---|---|
| `robot.robot_id`, serial, model | Desk system info; replace the `fr3_01` placeholder in config and contract consistently |
| End effector | **configure the Franka Hand in Desk** (mass, centre of mass, TCP), then re-read the EE pose |
| Joint limits and margins | verify the PPO-copied values for this unit and the hand |
| Controller gains and processing (planned schema additions) | explicit `K_q`, `D_q`, `K_x`, `D_x`, Jacobian/TCP convention, clipping, gain smoothing, friction/offsets, torque bounds/slew/filter and guard configuration; upstream gains are reference values only |
| Motion-limit schema (replace ambiguous mock generator semantics) | separate target row-speed acceptance from native measured-speed guards; define acceleration monitoring if required; no Ruckig acceleration/jerk guarantee in this baseline |
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
   e-stop, every episode recorded. Vetted self-collision checks and commissioned
   hand geometry are required
   before physical hold/tracking commissioning or trials; point-based workspace
   checks alone do not qualify them.

## 7. Implementation order and ownership

1. Pin an isolated DROID contracts/runtime environment without altering the local
   M4/PPO extraction or either original compatibility stack.
2. Verify the existing Franky shaping API and publish offline torque fixtures plus
   the processing-difference inventory; settle the revised limit/config schema.
3. Implement backend injection, seven-joint native tracking/receipt/guard plumbing,
   asynchronous hand state/commands and real stop evidence. Test with fakes first.
4. Run the cross-host mock loopback and choose timing margins from measurements.
5. Fill and review unit/tool/workspace commissioning data, then separately authorize
   D7 shadow, D8 hold/stop/gripper work and D9 trials in that order.

`franka-runtime` owns the backend/service/guards; the separate Franky fork owns any
native additions. `robot-learning` owns policy inference, cameras, chunk submission
and episode recording. Exchange pinned commits/artifacts and update the upstream
handoff/interface descriptions with the selected controller and limit semantics.
During physical control use a pinned deployment and keep development/IDE/agent
work off the controlling host. This plan authorizes neither host-network changes
nor hardware motion, recovery, gripper homing or modified operating limits.


### Recording and supervised shutdown implemented (offline, step 6)

See [recording and shutdown](droid-recording-shutdown.md). Bounded non-RT JSONL
recording now retains native telemetry, commands/receipts, clocks, Hand state and
stop evidence with fsync and digest validation. Logging failure requests stop and
prevents unrecorded dispatch. Shutdown keeps supervision running until terminal
evidence or a deadline, without fabricating rest/join. Physical schema recording
settings remain null; the physical CLI is still disabled. Native packages and
frozen baselines were unchanged.

Next: reviewed identity/collision adapters and physical admission composition,
cross-host mock timing, then authorized per-unit D7/D8/D9 qualification.


## Step 7 update: physical admission composition (offline)

[Physical admission](droid-physical-admission.md) now ties reviewed physical
settings, device checks, recording and the existing service/protocol to supervised
shutdown. Ready-lease cancellation records that control never started instead of
fabricating native rest evidence. Explicit recorded-identity and collision-engine
binding adapters are available; neither supplies missing per-unit data nor
qualifies an engine or live identity procedure. The inspection CLI has no execution
option; the mock CLI cannot select physical devices. Real providers/commissioning
values and cross-host/D7/D8/D9 qualification remain outstanding.


## Commissioning review preparation (offline, 2026-10-24)

[Commissioning review instructions](droid-commissioning-review.md) describe the
source-bound, non-executable draft package at
`~/yifan/droid-deployment/commissioning/2026-10-24-review/`. Known contract/native
bindings are prefilled; audited gains remain separate suggestions. Real unit/tool
evidence, reviewed geometry and operator decisions remain required for physical
operation. Physical timing margins can follow cross-host mock measurements;
physical commissioning does not block the separate mock timing test. No robot
access, fresh identity capture, approval or cross-host test occurred here.
