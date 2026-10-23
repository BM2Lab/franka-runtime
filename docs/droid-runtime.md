# D4 — DROID joint-position chunk runtime (mock only)

`franka_runtime.droid` accepts native DROID chunks (`contracts.droid`, 8 rows of
absolute FR3 joint targets plus gripper closure at 15 Hz) from a learning runner
over TCP and executes them against a **mock** arm and Franka Hand. It implements
the runtime side of robot-learning's `docs/droid_runtime_interface.md`. **No
Franky backend, no FCI connection and no motion authorization exist.** The Franky
integration (native 1 kHz interpolation, timing) is D6 on the RT host and requires
separate authorization under [hardware-qualification.md](hardware-qualification.md).

## Dependencies and versions

franka-runtime 0.0.4 pins `contracts==0.0.2` (robot-learning). The DROID path uses
only the standard library and contracts. `SessionAuthority` no longer imports
`contracts.motion`/`contracts.franka_policy`: it accepts any frozen contract
dataclass as its session specification, with an optional explicit `robot_id`
(DROID keeps it in `spec.robot.robot_id`). Its original 11 unit tests run
unchanged except that they use a local frozen stand-in for `MotionSessionSpec`.

The M4 and PPO slices still import `contracts.motion` and `contracts.franka_policy`,
which no released contracts version provides. Their tests are skipped with that
reason (`pytest -rs` shows it), and their CLIs cannot run until those modules
exist, either in a later contracts release or moved into this repository.

## Modules

| Module | Role |
|---|---|
| `droid.config` | strict loader; unknown keys rejected; `require_runnable()` lists every unset commissioning value; only `backend: mock` runs; refuses all-interface binds and any mock that claims motion authorization |
| `droid.checks` | setpoint-lead acceptance: per-row joint range with margins, row 0 lead over measured q (`max_target_lead_rad`), row-to-row speed; rejects, never clips |
| `droid.executor` | pure, time-injected row scheduler (below) |
| `droid.workspace` | stdlib URDF forward kinematics (matches the numpy `UrdfKinematicsProvider` to 1e-15 m) and a point-based workspace check: link origins and tool points above table + clearance, tool points inside a box, at every row and at interpolated points from measured q onward. The measured state itself is never rejected, so moving out of a bad pose stays possible. |
| `droid.gripper` | Franka Hand intent: commands only on closure transitions across 0.5 (contract `FrankaHandClosureMap`) |
| `droid.mock` | kinematic arm approaching the setpoint under velocity and acceleration limits with braking (stands in for the motion generator), constant-speed hand; not a dynamics model |
| `droid.service` | TCP service: `SessionAuthority` + executor + mock backend + control loop |
| `droid.cli` | `franka-droid-mock --config … --contract …` |

## Execution semantics

**Setpoint lead (chosen 2026-10-23).** DROID rows are joint-impedance setpoints that may
run ahead of the measured state, as in DROID's own data. Acceptance bounds the lead of
row 0 over the measured q per joint (`limits.max_target_lead_rad`) and the speed between
consecutive rows (`limits.max_joint_velocity_rad_s`). How fast the arm actually
approaches a leading setpoint is limited by the motion generator using
`max_joint_velocity_rad_s` and `max_joint_acceleration_rad_s2`: the mock now, the Franky
motion generator (with its own jerk limit and tracking-error reflexes) in D6. The
earlier strict rule (measured q → row 0 within one row at the velocity limit) rejected
most chunks in the robot-learning D5 loopback, because policy row 0 led the measured
state by up to 0.25 rad.

- **Acceptance.** First the contract binding (`contracts.droid_session.validate_chunk_command`):
  lease, clock, capability, payload hash, source step and time, and a deadline no
  later than the chunk's conservatively converted one. Then `SessionAuthority.submit`
  (sequence, staleness, command window) and the physical checks, which are repeated
  at dispatch. A rejected command does **not** consume its sequence number.
- **Rows.** The executor starts rows on a fixed 66.7 ms grid from the moment a
  chunk starts. A newly accepted chunk is the single pending chunk. It replaces a
  pending chunk that has not started yet (the replaced one is reported as superseded),
  and it replaces the current chunk's rows that have not started yet at the next row
  boundary, or immediately when the executor is idle or holding.
- **Deadline.** No row starts at or after the chunk's runtime-clock deadline. A
  pending chunk that expires before starting is dropped.
- **Underrun.** When rows run out, the last due row's target is held for at most
  `hold_timeout_ms`, then the executor stops. That latches fault `native_fault:101`.
- **Applied receipts.** These are per chunk: the authority marks a chunk `applied`
  when its first row starts within its deadline (native tick and runtime time).
  Per-row progress is in the state stream (`executing_sequence`, `executing_row`).
- **Faults are latched and never reset through the API.** Fault codes:

  | Code | Cause |
  |---|---|
  | 101 | underrun hold timeout |
  | 102 | heartbeat timeout |
  | 103 | client disconnected while running |
  | (named) | the authority's own codes: `lease_expired`, `dispatch_failed`, `application_deadline`, and so on |

  The mock completes stops immediately with clean evidence. A real backend must
  supply real stop evidence.
- **New session.** A new lease (after a clean release and stop) starts with a fresh
  executor and gripper intent. A latched fault blocks new leases.

## Protocol

Framing is `contracts.wire`: 4-byte length followed by JSON, at most 2 MiB, with
NaN and duplicate keys rejected. Requests are `{"protocol": 1, "id", "op", "payload"}`.
Responses are `{"protocol", "id", "ok", "result"|"error"}`. One client at a time,
from an allow-listed peer address.

| Op | Payload → result |
|---|---|
| `hello` | → runtime clock ID, robot ID, allow-listed contract SHA-256, versions |
| `clock_probe` | → runtime receive/send stamps. Also the heartbeat once the session has started. |
| `open` | `{session: DroidSessionOpen, token}` → `DroidSessionReady`. The token is compared in constant time against the env var named by `network.token_env`; it is never stored in a file. |
| `clock_register` | `{probes: [ClockProbe]}` → `ClockOffsetEstimate`. At least `min_probes`, server stamps issued by this runtime, matching clock IDs, uncertainty within bound. |
| `start` | → authority starts the backend (requires a registered estimate) |
| `submit` | `{envelope: CommandEnvelope, chunk: DroidActionChunk}` → `CommandReceipt` |
| `state` | → `FrankaStateSample` |
| `receipts` | → drained receipts, latched fault, authority state, gripper commands, peak implied speeds |
| `release` | `{reason}` → authority stops; the state becomes `complete` once the stop finishes |
| `close` | ends the connection. If the session was running, that is fault 103. |

Runner order: `hello` → probes → `open` (lease `requested_ns` in runtime time via the
runner's own estimate) → `clock_register` (same probes) → `start` → submit chunks
while probing ≥ 1/s → `release` → `close`.

## Configs

| File | Status |
|---|---|
| `configs/fr3/droid.jointpos.pending.json` | `backend: franky`, `executable: false`. Velocity limits, timing, clock bound, gripper force/speed/homed width and network are `null` until commissioning. Joint limits and margins are copied from the PPO pending config (fr3v2.1, not re-verified for this unit or the Franka Hand). |
| `configs/fr3/droid.jointpos.mock.json` | `backend: mock`; complete but every non-limit value is a **mock test value** (1.0 rad/s velocity limit, 250 Hz control, 200 ms hold, 1.5 s heartbeat, 700 ms command window, 2 ms clock bound) |
| `configs/fr3/droid_fr3_jointpos_v1.contract.json` | DROID contract copied from robot-learning (contracts 0.0.2), allow-listed by SHA-256 |

Run the mock for loopback work (set a token in the environment, never in a file):

```bash
FRANKA_DROID_TOKEN=<secret> franka-droid-mock \
  --config configs/fr3/droid.jointpos.mock.json --contract configs/fr3/droid_fr3_jointpos_v1.contract.json
```

## Verification (2026-10-23, offline)

`python -m pytest tests --import-mode=importlib -rs`: 45 passed, 2 skipped (M4/PPO,
missing contracts modules), stable across repeated runs. This includes real-socket
loopback sessions:

- the handshake, with clock registration
- accepted, then applied, receipts while the mock arm tracks the targets
- a gripper command on the closure transition
- replay, swapped-payload, bad-token, unlisted-contract, joint-range and speed rejections
- underrun, heartbeat and disconnect faults
- release followed by a fresh session

Importing the package loads no numpy, YAML or vendor module.

## Observed unit facts

From a read-only `echo_robot_state` on the RT host (2026-10-23), recorded under
`robot.observed` in both DROID configs: system version 5.9.2, mode Idle, no current or
last-motion errors, and home pose
`[0.0054, -0.7833, -0.0009, -2.3532, -0.0130, 1.5622, -0.0019]` rad (the mock's start
pose). Serial and model were not reported and must be recorded during commissioning.

## Mock loopback in setpoint mode (2026-10-23)

robot-learning D5 runner, live cameras, pinned pi05 server, this mock: 30 steps, 29
chunks applied, one rejection (joint 6 lead 0.316 rad > mock 0.3), clean release, no
fault. Row-0 lead had a median of 0.11 rad and a maximum of 0.30. The mock arm drifted far,
by +1.9 rad on joint 2 and +2.0 rad on joint 4 (joint 4 ended near its margin), because
the cameras cannot see the mock arm and the scene had no task objects. On real hardware,
motion like that heads toward the table: **workspace and self-collision checks are
required before shadow-mode evaluation of motion and before any motion**, not optional.

## Workspace check (2026-10-23)

The `workspace` config section is required:

| Field | Meaning |
|---|---|
| `model_urdf` | relative to the config file |
| `base_link`, `flange_link` | the chain to evaluate |
| `link_points` | link origins that must stay above the table |
| `tool_points_m` | points in the flange frame (Franka Hand) |
| `table_z_m`, `clearance_m` | the table plane and margin |
| `box_min_m`, `box_max_m` | the workspace box for tool points |
| `interpolation_steps` | points checked between rows |

- **Pending config:** `null` until commissioning.
- **Mock config:** the parity-fixture URDF (robot-returned FR3, bare flange), nominal hand
  fingertip points 0.1034 m beyond the flange, the table at base z = 0, a 5 cm clearance,
  and a test box.

Findings:
- **FK matches the robot.** FK at the observed home pose gives the flange at
  (0.3066, −0.0001, 0.5903) m, which equals the EE position the robot reported. So the
  URDF agrees with this unit at that pose, and **the robot's configured end effector is
  currently the bare flange even though a Franka Hand is mounted**. Check the Desk
  end-effector settings (hand mass, centre of mass, TCP) before any motion.
- **The check stops the drift.** In the D5 loopback with the check enabled, 14 chunks were
  applied. Then the policy kept reaching toward the table and 6 submissions were rejected
  with the hand 4.4–4.9 cm above the mock table plane (5 cm clearance). The runtime held
  and latched `native_fault:101`. Without the check, the same policy drove joint 4 to its
  margin.

Tests: 45 passed, 2 skipped (M4/PPO).

## Not implemented (D6 and later)

- **Franky backend:** native interpolation between row targets at 1 kHz, measured
  state, real stop evidence and Franka Hand grasp control.
- **Timing and clocks on the real link:** measurement on the RT host and the
  dedicated ethernet link (probe rate, uncertainty bound, watchdog margins).
- **Commissioning values** for every `null` in the pending config.
- **Self-collision checks (required before D8).** These need a vetted collision model of
  the FR3 links and the Franka Hand (e.g. from franka_description). None is invented here.
  The workspace check above is point-based with clearances, not geometric collision checking.
- **Transport security beyond the minimum:** TLS, and token rotation.
