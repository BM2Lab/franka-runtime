# RT host → inference workstation: DROID handoff, 2026-10-24

## Read this first

The RT implementation is an **offline-tested candidate**, with a populated
non-executable physical draft. No new robot access, motion, recovery, Hand homing,
network change or cross-host timing experiment occurred during this handoff.

The sourced draft now contains **86 of 104 fields**. The remaining 18 are real
unit/tool/model/approval inputs, not missing boilerplate. Do not replace these
with test fixtures or treat old M4 approval as approval for a mounted Hand.

Portable candidate/evidence: `configs/fr3/droid-source-review/` in franka-runtime.
Host copy: `~/yifan/droid-deployment/commissioning/2026-10-24-sourced/`.
Start with `physical.review.pending.json`, `provenance.json`, `REVIEW.md` and
`observations-and-alternatives.json`. `source-evidence/` retains hash-verified
copies of the referenced inputs, including the original completed-run receipt.

## Software pins and controller contract

- RT repository base: `5e8e32687b53026913f39b5ea55472f75a93e4ac`, branch
  `d4-droid-runtime`, plus the uncommitted candidate changes in this handoff.
  The base commit alone does not contain all candidate code. Use the transfer
  manifest/patch or a subsequently published commit, not an assumed remote branch.
- Separate pinned robot-learning checkout: `08d64c65ea06639dda55b3cdcb2459a7784e4c85`;
  `contracts==0.0.2`, franka-runtime source-patched `0.0.4` (wheel hash identifies it).
  Preserve the inference workstation's policy/camera environments; do not install
  Franky or ROS there to use this protocol.
- RT-only native wheel: `2.0.1.dev67+gdroidd6.libfranka.0.21.2`, server10, with
  libfranka 0.21.2. `native-pin.json` records the wheel and bundled binary hashes.
  Original server6/server10 baselines and the old M4 environment remain unchanged.
- DROID source snapshot: `33ae6a67274f36d2e29525b86f23a56616ef43a7`, Polymetis
  submodule `0a01a7fa7a7c65b2f9a3aebf5e79040940daf9d2`.
- Exact canonical contract SHA256:
  `46b22083d8cf18e669ff06e35a4f759806140ae4e074712dd480ba85a3f239fc`.
  Do not change joint names, frame labels or logical robot ID on one host alone.

Physical DROID control uses continuous native 1 kHz hybrid joint impedance:

$$
\tau_{fb}=(K_q+J^T K_x J)(q_d-q)-(D_q+J^T D_x J)\dot q.
$$

The native candidate adds Coriolis; firmware supplies gravity. Desired joint
velocity is zero, but no planner requires a stop at each row. Franky slew and
libfranka filtering/rate limiting remain explicit. This route has no Ruckig
trajectory generator and promises no acceleration/jerk-limited interpolation.
The old robot-learning interface descriptions of D6 as a future Franky motion
generator should be updated accordingly; they still describe the mock separately.

## Selected candidate values and why

| Component | Candidate | Source/status |
|---|---|---|
| Joint K | 40, 30, 50, 25, 35, 25, 10 | Pinned DROID FR3 defaults |
| Joint D | 4, 6, 5, 5, 3, 2, 1 | Pinned DROID FR3 defaults |
| Cartesian shaping K | 400, 400, 400, 15, 15, 15 | Pinned DROID FR3 defaults |
| Cartesian shaping D | 37, 37, 37, 2, 2, 2 | Pinned DROID FR3 defaults |
| Clip / gain smoothing / Franky slew | 0.5 rad / 0.1 s / 1 Nm per callback | Archived Franky pipeline |
| libfranka | 100 Hz low-pass, rate limiting on | Pinned DROID client config |
| Constructor collision thresholds | 20 Nm / 30 N | Historical completed M4 run, not DROID's 40/40 |
| Measured joint speed guard | 0.435 rad/s J1–4, 0.522 J5–7 | Historical completed M4 run |
| Raw command torque guard | 40 Nm J1–4, 8 Nm J5–7 | Historical M4 guard; **not** DROID's 86/11.5 Nm clamp |
| Target row-speed limit | 2.075 rad/s J1–4, 2.51 J5–7 | Pinned DROID ceilings; target admission only; updated after cross-host replay |
| Row-0 target lead | 0.3 rad per joint | Existing D5/mock baseline |
| Stop D | 2, 2, 2, 1, 1, 1, 0.5 | Completed archived smooth stop |
| Stop ramp/rest/max | 0.2 s / 0.02 rad/s / 2 s | Same historical stop |
| Stop evidence deadline | 2.5 s | Proposal: native maximum + 0.5 s join allowance |
| Native callback gap / publisher timeout | 10 ms / 100 ms | Historical M4 settings |
| Servo / Python supervisor | 1000 Hz / 250 Hz | Native route / existing mock baseline |
| State age / consumption / startup | 40 ms / 50 ms / 5 s | PPO candidate / proposal / proposal |
| Hold / heartbeat / command window | 200 ms / 1500 ms / 700 ms | Existing mock baseline; measure before physical use |
| Clock admission | 4 probes, max uncertainty 2000 µs | Existing mock baseline |
| Hand open/close/speed | 0.08 m / 0 m / 0.05 m/s | Contract endpoints / pinned DROID speed; qualify actual Hand |
| Hand force | 20 N | Direct-libfranka candidate from existing mock config; not hardware-tested |

The historical guard/stop source is the **complete** October 12 M4 run
`tracking.20261012T142902Z`, whose saved config hash matches its successful report.
This proves which settings were used then, not that a new controller/tool is
qualified. The native raw-torque guard faults/stops; it is not a magnitude clamp.
DROID's larger guards and 40/40 collision behavior are retained as alternatives,
not silently applied to this arm.

DROID's wrapper passes `goto(force=0.1)` to Polymetis. Franky's `grasp` force is in
Newtons. The transport/client conversion was not audited, so 0.1 was **not copied**
into the native command. Hand cadence, freshness, cancellation and logging budgets
are explicitly labeled proposals in the provenance, not measured results.

The archived URDF's joint labels `joint1..7` were renamed in a copied reference
asset to contract labels `panda_joint1..7`, preserving order and all transforms.
FK equality is checked at archived/measured and synthetic configurations. The
original URDF is untouched. This arm-only asset is not a Hand collision model.
The workspace box is the old circle-task box; it may reject DROID manipulation
positions and needs task-specific review, not automatic widening.

## What the inference agent should implement/check

1. Keep the current wire format and canonical contract. Actions are **seven
   absolute joint positions**, radians, plus closure in [0,1], 15 Hz, up to eight
   open-loop rows. No deltas, velocity reinterpretation, six-joint PPO mapping,
   held J7 or added q-reference observation.
2. Observe measured q/dq/Hand width. Honor `valid`, clock conversion and freshness.
   Native startup feedback is invalid until the first callback. Do not submit
   based on an old startup snapshot or infer readiness from `start` alone.
3. Treat `accepted` as admission and `applied` as **confirmed native consumption of
   the first row**, not a successful publish call. Native execution identity is
   confirmed feedback, not the next executor row. Preserve sequence handling on
   rejection and the existing lease/clock/heartbeat protocol.
4. Keep cameras, pi0.5-DROID inference and episode recording on the inference host.
   Keep native servo/guards/watchdog/stopping on RT. Do not add Polymetis as a
   second controller process on the RT host or copy venvs between machines.
5. For the next cross-host test, use **mock-link config only**. Record probe
   uncertainty, state age/camera skew, inference and full capture-to-receipt
   latency, accepted→applied delay, row-0 lead and row-speed rejections, underrun,
   faults and clean release. Distinguish physical-candidate bounds from the
   current mock's 1 rad/s target-speed settings when interpreting rejection rates.
6. Return the episode directory, summary and exact runtime/contracts/policy/camera
   revisions. Suggest physical timing margins from measurements rather than
   claiming the proposed values are qualified. No motion authorization follows
   from a passing mock test.

RT mock command, after both agents coordinate the session token privately:

```bash
cd /home/chen-lab/franka-runtime
/home/chen-lab/yifan/droid-deployment/run-native-python -m franka_runtime.droid.cli \
  --config configs/fr3/droid.jointpos.mock-link.json \
  --contract configs/fr3/droid_fr3_jointpos_v1.contract.json
```

`FRANKA_DROID_TOKEN` must already be set securely; do not write its value in Git
or the handoff. RT endpoint is 10.77.0.2:47700, allow-listed inference peer
10.77.0.1. This handoff does not change interface addresses or profiles.
The inference host uses its existing `scripts/run_droid_runtime.py` with
`--runtime-host 10.77.0.2 --runtime-port 47700`, its current policy pin, policy
port and camera configs, a private `--token-file`, a new `--output-dir`, and
initially a small `--max-steps`. Exact local inference paths stay workstation-owned.

## Physical items still unresolved

- Current arm/control serials and identity freshness policy. Firmware/model/IP in
  the draft are archived/reference candidates, not a fresh identity observation.
- Actual `F_T_EE`, EE mass/inertia/COM and external load mass/inertia/COM. The
  October 23 snapshot reported identity `F_T_EE` and **zero EE mass/inertia**,
  despite the documented mounted Hand. Those seven values were deliberately not
  promoted to usable tool settings. Correct Desk configuration requires a separate
  operator action and a fresh read; neither happened here.
- A vetted Hand-inclusive self-collision model/engine and its binding hashes.
  Point/table checks and a contract-named arm URDF do not replace it.
- New unit/tool/workspace/Hand/stop approvals. Old approval names/timestamps are
  retained only inside archived source evidence, never copied into approval fields.

The physical candidate remains `executable: false`, `motion_authorized: false`.
Cross-host mock timing does not require these physical approvals. D7 shadow,
D8 hold/stop/Hand qualification and D9 trials remain separately authorized work.

## Validation and transfer

Offline regression: **163 runtime tests + 67 external contracts tests = 230 passed, 2 runtime skips**. The skips are the pinned upstream
contracts' missing M4/PPO modules. Population tests verify archived successful-run
config hashes, source hashes, effective joint bounds, unchanged FK after joint
aliasing, copied artifact hashes, the 18 deliberate omissions, and schema validity
of populated numeric fields with test-only stand-ins for missing unit/tool data.
Those stand-ins were never written into the candidate. No native package was
installed/rebuilt or changed for this population task.

Transfer archive on the RT host:
`~/yifan/droid-deployment/transfers/2026-10-24-inference-handoff.tar.gz`.
It includes this handoff, the sourced candidate/evidence, the runtime Git base
revision and a binary-capable source patch, plus a per-file SHA256 manifest.
It includes no venv, native wheel, token, camera images or hardware recording.
Apply the source patch only to a clean checkout of the recorded base (check with
`git apply --check runtime.patch` first). Do not overwrite unrelated workstation
changes or install the RT native environment on the inference host. The patch
has not been pushed to GitHub by this task.


## Cross-host follow-up, 2026-10-24

[Cross-host results and test-count reconciliation](droid-crosshost-mock-results.md)
record the successful 10/30-step episodes and stopped mock. The candidate target
row-speed bound now uses DROID 2.075/2.51 rad/s; measured guards remain unchanged.
Offline replay reduces target-speed rejections from 13/37 accepted chunks to 0/37.
This is not physical-speed qualification. The previous transfer archive is a
historical pre-update snapshot; use the updated repository candidate for new
workstation comparisons. The policy's position contract defines no speed ceiling.
The earlier test total included external contracts tests, not missing runtime tests.
