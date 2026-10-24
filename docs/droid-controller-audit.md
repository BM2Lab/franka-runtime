# DROID hybrid-impedance audit — offline, 2026-10-23

The existing modified Franky wheel can express DROID's **feedback law**. It cannot
alone supply the native receipts/watchdogs/guards required for the physical DROID
backend. No physical backend or motion approval was added by this audit. The
robot remained unused; no native library, baseline, venv or host setting changed.

## Pins and evidence

- DROID: `33ae6a67274f36d2e29525b86f23a56616ef43a7`.
  Its `droid/fairo` submodule was verified at
  `0a01a7fa7a7c65b2f9a3aebf5e79040940daf9d2`.
- Franky source: clean `ea855c9ce3f48ece2c7a0bc4e1d7d033083e1bf9`.
- Installed modified wheel: `2.0.1.dev66+gm4comparefix.libfranka.0.21.2`;
  all **23 native file hashes** match
  [`franky-server10.json`](../src/franka_runtime/common/backends/franky-server10.json).
  Wheel SHA256: `06f96b3104e75f664869d80ee3107926ce8119a26c74dbeb72c38ce92fb93c76`.
- Host receipts: `~/yifan/droid-deployment/validation/franky-shaping-api.json`;
  downloaded, pinned sources and their hashes:
  `~/yifan/droid-deployment/artifacts/controller-audit/sources.json`.
- Portable numerical inputs/outputs and source hashes:
  [`hybrid-feedback.json`](../tests/fixtures/droid/hybrid-feedback.json).

The wheel audit constructs motion/reference objects only, with `Robot` and
`Gripper` constructors forbidden. No installed package is changed. Source revision
and binary identity are recorded separately; we did not execute the wheel's
protected torque callback with a fake model.

## Controller and API mapping

For measured q, dq and a supplied geometric Jacobian J (6 by 7), DROID uses

$$
\tau_{fb}=(K_q+J^T K_xJ)(q_d-q)+(D_q+J^TD_xJ)(-\dot q).
$$

The pinned Polymetis `HybridJointSpacePD.forward` implements this equation;
`HybridJointImpedanceControl` keeps desired joint velocity zero and adds inverse
dynamics with zero desired acceleration. With its default `ignore_gravity=True`,
that feedforward subtracts gravity, leaving the model's velocity-dependent bias.
Franky adds `model->coriolis`; firmware supplies gravity. Their model outputs
have not been compared, so matching this algebra does not prove equal feedforward.

| DROID quantity | Existing installed Franky API | Audit result |
|---|---|---|
| Kq, Dq | `JointImpedanceTrackingMotion(stiffness=…, damping=…)` | Exact seven-vector readback |
| Kx, Dx | `cartesian_stiffness=…, cartesian_damping=…` | Exact diagonal matrices via `get_cartesian_gains()` |
| Absolute q target, zero desired dq | `set_reference(JointReference(q=…, dq=zeros(7), tau_ff=zeros(7)))` | All seven values publish/read back, including J7 |
| Gain updates | `set_gains`, `set_cartesian_gains` | Exposed; source applies exponential smoothing |
| Published target | `get_reference()` | Last written mailbox value only; **not consumption evidence** |
| Consumed sequence/tick/time | No such tracking-motion API | Native extension required |
| Error clip | C++ default 0.5 rad/joint | Constructor does not expose a clip argument; `params` is a copy |

Setting `motion.params.error_clip` edits a returned copy and has no effect on
motion. A nondefault clip needs an explicit constructor/binding extension, not
Python mutation. Cartesian shaping must be enabled at construction; the source's
optional shaping state is created there. Do not try to enable it by gain updates
on a motion constructed without shaping.

The reference FR3 gains are Kq `[40,30,50,25,35,25,10]`,
Dq `[4,6,5,5,3,2,1]`, Kx `[400,400,400,15,15,15]`,
Dx `[37,37,37,2,2,2]`. These are audited upstream values, **not commissioned
settings** for this arm/hand.

## Numerical result and limits of the evidence

32 deterministic fixtures cover zero error, J7-only commands, translational and
rotational shaping separately, coupled/changing supplied Jacobians, rank-deficient
and zero Jacobians, damping, the clipping boundary and an out-of-clip command.

The audit extracts upstream's unchanged `HybridJointSpacePD.forward` body and
executes its matrix arithmetic with NumPy arrays (annotations removed, no Torch
imports or TorchScript execution). It compiles the **unchanged feedback expression**
from Franky's C++ source into a standalone Eigen executable. Supplied inputs,
gains and Jacobians bypass the robot/model. Nothing in Franky/libfranka is rebuilt.

- Maximum unclipped difference: **2.842170943040401e-14 Nm**.
- A deliberately out-of-clip input differs by **64.408878257653 Nm**, demonstrating
  that Franky's 0.5-rad clipping prevents unrestricted equation parity. This is
  a synthetic numerical case, not a recommended hardware command.
- Actual wheel callback, live gains, model/Jacobian, Coriolis, runtime startup,
  filter/slew behavior, scheduling and stop behavior are **not qualified** by these
  fixtures. Nor do they compare Torch float32 precision to C++ double precision.
- J is externally supplied. Real q-to-J equivalence still needs the same tool/TCP,
  base frame, linear/angular ordering and kinematic model. The upstream docstring
  calls J `(N,6)`, but its multiplications require `(6,N)`; use the actual equation.

## Torque-processing differences

| Stage | Pinned DROID/Polymetis | Pinned Franky path | Decision for D6 |
|---|---|---|---|
| Reference scheduling | Continuous impedance; absolute q updates; desired dq zero | Long-lived tracking motion/mailbox; desired dq explicitly zero | Reuse; no Ruckig or per-row motion |
| Position error | No clipping in hybrid feedback | Clip each q error to ±0.5 before joint and shaped proportional terms | Record explicit clip; bound lead; do not claim unrestricted parity |
| Initial gains | Explicit policy parameters | Constructor seeds current gains at configured values | No automatic ramp from zero; initialize with explicit damping |
| Gain changes | Policy parameter changes | Exponential interpolation, default time constant 0.1 s | Fix gains per first session; log any later changes |
| Model/TCP | Polymetis robot model Jacobian and inverse dynamics | libfranka end-effector `zeroJacobian` and Coriolis | Still requires model/TCP comparison; mounted hand was not configured in Desk |
| Additional feedforward | Zero desired acceleration; gravity removed if configured | Constant offset + per-reference tau_ff + optional friction | Set explicit zero for equivalence candidate; not a friction calibration |
| Soft-limit response | Cartesian position, joint position/velocity safety reflexes; hard violations throw | Optional exponential joint-limit repulsion; different activation/gains/clamp | Preserve native guards; do not copy upstream reflex thresholds as approvals |
| Total magnitude clamp | Client clamps sum of policy + safety torques (FR3 config: 86 Nm first four; 11.5 Nm last three) | `JointImpedanceBase` has **no total-command magnitude clamp** (repulsion/friction have individual caps) | Native command guard required; preserve processing order; do not confuse a stop-on-limit guard with a clamp |
| Franky slew | None at this stage in DROID client | Delta clip against `tau_J_d`, default 1 Nm **per callback**, not scaled by dt | Preserve explicitly; callback-gap guard still needed |
| libfranka processing | Published config requests filter 100 Hz + rate limiting | `Robot.move` defaults rate limiting false, default cutoff; caller can override | Explicit caller settings required; do not rely on defaults |
| libfranka 0.21.2 order | Historical DROID binary/version not audited here | Low-pass vs previous `tau_J_d`, then optional libfranka rate limit, then finite check | Record exact version/options; an extra Franky slew precedes this filter |
| Recovery | Client contains automatic recovery attempts | D6 must latch faults, stop, require fresh admission | Do not copy automatic recovery |

Thus the intended native sequence is feedback → offsets/friction/limit repulsion
→ Coriolis → Franky slew → libfranka low-pass → optional libfranka rate limit →
firmware torque control/gravity. A D6 total-command guard must inspect the intended
stage explicitly; no processing change or increased guard was made here.

## Revised config boundary for implementation

Keep existing schema-v1 mock configs and their braking model unchanged. Implement
an explicitly versioned physical schema with a backend-specific controller block;
never reinterpret v1's mock acceleration as a physical impedance guarantee.
Proposed field semantics for the next implementation:

| Block/field | Meaning |
|---|---|
| `controller.kind` | Named `droid_hybrid_joint_impedance`; absolute seven-joint targets; desired dq zero |
| `controller.joint_stiffness_nm_rad`, `joint_damping_nm_s_rad` | Explicit seven-vectors |
| `controller.cartesian_stiffness`, `cartesian_damping` | Explicit six-vectors, translation first; units N/m, N s/m then Nm/rad, Nm s/rad |
| `controller.jacobian_frame`, `tcp`, `model_provenance` | Base-expressed end-effector Jacobian, tool transform/model identity; no silent flange/hand substitution |
| `controller.error_clip_rad`, `gain_time_constant_s` | Explicit native clip and gain-update semantics; unsupported values reject |
| `controller.coriolis`, `friction`, `constant_torque_offset_nm` | Explicit model compensation/extra torque; no gravity double addition |
| `torque.max_delta_nm_per_callback`, `libfranka_limit_rate`, `lowpass_cutoff_hz` | Native slew/filter processing options and exact ordering |
| `guards.max_command_torque_nm`, `max_measured_joint_velocity_rad_s`, `max_callback_gap_ns` | Separate native fault/stop guards; not trajectory generation |
| `acceptance.max_consecutive_row_speed_rad_s`, `max_target_lead_rad` | Whole-chunk/dispatch acceptance checks; reject, do not clip |
| `watchdog.publisher_timeout_ns` | Independent executor/publisher liveness; holding a row is not proof of freshness |
| `native_pin`, `approval` | Exact candidate artifact and per-unit commissioning; remain required |

If acceleration monitoring is added, specify estimator/filter, units, threshold
and stop response separately. There is no reference acceleration/jerk guarantee.
Wire contracts remain `droid_jointpos_v1`; changing host config schema does not
silently change runner admission semantics. Any revised capabilities/acceptance
must be reflected in runner compatibility tests before rollout.

## Next implementation

Add a minimal project-owned native candidate for seven-joint tracking with a
sequence-bearing mailbox, native consumption telemetry (sequence/tick/monotonic
time), publisher-death/callback-gap and measured-speed/command-torque guards,
and reviewed stop telemetry. Expose immutable clip/processing settings accurately.
Keep libfranka/Ruckig and both original families unchanged. Then implement backend
injection, fake-backed integration tests and a serialized hand worker. Cross-host
mock timing and per-unit tool/workspace commissioning remain separate work.

Reproduce source-expression fixtures with
`scripts/droid/audit_controller.py --sources <saved-source-dir> --franky-source
<project-source> --work-dir <project-output> --output <fixture.json>` using the D6
NumPy environment and g++/Eigen. Run `scripts/droid/check_franky_api.py --pin
src/franka_runtime/common/backends/franky-server10.json --output <receipt.json>`
with the **existing modified** wheel environment, never a frozen original venv.

## Primary source locations

- [DROID command dispatch](https://github.com/droid-dataset/droid/blob/33ae6a67274f36d2e29525b86f23a56616ef43a7/droid/franka/robot.py)
  and [FR3 gains/client configuration](https://github.com/droid-dataset/droid/blob/33ae6a67274f36d2e29525b86f23a56616ef43a7/config/fr3/franka_hardware.yaml).
- [Hybrid feedback equation](https://github.com/facebookresearch/fairo/blob/0a01a7fa7a7c65b2f9a3aebf5e79040940daf9d2/polymetis/polymetis/python/torchcontrol/modules/feedback.py),
  [policy/reference/feedforward](https://github.com/facebookresearch/fairo/blob/0a01a7fa7a7c65b2f9a3aebf5e79040940daf9d2/polymetis/polymetis/python/torchcontrol/policies/impedance.py),
  [inverse dynamics](https://github.com/facebookresearch/fairo/blob/0a01a7fa7a7c65b2f9a3aebf5e79040940daf9d2/polymetis/polymetis/python/torchcontrol/modules/feedforward.py),
  and [client safety/clamp/filter options](https://github.com/facebookresearch/fairo/blob/0a01a7fa7a7c65b2f9a3aebf5e79040940daf9d2/polymetis/polymetis/src/clients/franka_panda_client/franka_panda_client.cpp).
- Local pinned Franky source: `src/motion/joint_impedance_base.cpp`,
  `src/motion/joint_impedance_tracking_motion.cpp`, `python/bind_motion_torque.cpp`,
  `include/franky/motion/torque_control_utils.hpp`, `include/franky/robot.hpp`.
  Read-only libfranka 0.21.2 source: `src/control_loop.cpp::createControlCommand`.
