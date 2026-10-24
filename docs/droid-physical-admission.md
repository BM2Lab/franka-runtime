# DROID physical admission composition (offline candidate)

Step 7 connects physical schema v2, the existing device factory, shared DROID
service/authority/executor, recording, and supervised shutdown. Tests use fake
devices. No robot access, motion, recovery, native rebuild or network setup was
performed. The public mock CLI remains mock-only. The new CLI only inspects files;
it has no execute option. No complete per-unit configuration is shipped.

## Admission order

`PhysicalAdmission` construction and `inspect()` do not load Franky, call an
identity provider, start a thread or open a socket. Inspection requires reviewed,
complete settings and validates artifact hashes, contract/unit/joint/Hand mapping,
capability prefix, URDF joint order, collision-model binding, and factory config/
host-clock identity. Configuration is copied, not silently modified.

The separate `prepare()` API requires both explicit device-setup and motion
authorization. This is code for a future separately authorized run; it is not
permission to operate hardware during development. It then:

1. Validates admission and the session token before device setup.
2. Starts the bounded recorder and records admission before device construction.
3. Uses the explicit factory to verify native artifacts, identity and measured
   tool/workspace/collision state; actual identity and preflight evidence are logged.
4. Creates the shared service with physical settings and the connected backend.
5. Returns a prepared service without binding a network socket or starting control.

The existing open/clock-register/start/chunk/receipt/release protocol remains in
use. Native consumption drives applied receipts; actual join/rest evidence drives
active-control shutdown. Physical settings contain no mock start position or mock
acceleration/braking model. Consecutive-row target speed and measured native speed
remain distinct. A mock config still cannot accept a physical backend; an admitted
physical config cannot substitute a mock backend. A started, healthy recorder is
required. Recorded config hashes bind the connected backend and controller
parameter snapshot; later edits do not silently alter that session's gains.

Failures before control starts close the recording with `admission_failed` and
`control_started: false`. This does not undo authorized constructor collision-
threshold writes. No automatic motion, recovery, load/TCP setter or fallback
controller is added.

## Cancellation before control starts

Releasing a ready lease now uses `cancel_before_start()` when the backend provides
it. The native adapter verifies that arm/Hand workers and Hand commands have not
started, cancels the executor, and prevents reuse of that backend. Authority
completion records `control_never_started` and `workers_never_started` evidence.
This is cancellation, not a claim that a native stop/rest callback occurred.

Once control may have started, normal native stop/worker-join evidence is still
required. Direct pre-start stop requests and startup failures do not gain fabricated
rest evidence from this change. The existing failure/timeout path remains intact.
The recorder close timeout comes from physical configuration.

## Identity provider

`RecordedIdentityReader` accepts an explicitly selected, SHA256-bound JSON evidence
file. It checks its schema, size, address and required fields, and preserves the
original capture timestamp/host boot clock. Factory admission checks those clocks,
age and every identity field against commissioning.

Evidence fields are `schema_version: 1`, `source`, `address`, `model`,
`firmware_version`, `fci_version`, `arm_serial`, `control_serial`, `clock_id`, and
`observed_ns`. These must come from a reviewed capture; do not infer the model from
firmware or an IP address, or stamp an old inventory as a new observation.

This adapter is **recorded operator evidence, not a live Desk query**. It cannot
detect a robot substitution during its freshness window. A qualified live provider
or the approved physical identity procedure is still needed for commissioning.
Expired evidence requires a fresh capture and a new hash-bound run. No endpoint,
credentials, serial number, model or timestamp was invented for the current arm.

## Collision provider

`BoundCollisionChecker` adapts an externally supplied vetted engine. It verifies
that the engine declares the configured collision-model hash, URDF hash, physical
joint order, tool/payload/TCP description hash, and provenance. A query must return
exactly `True` for clearance; False, None, errors and malformed q are rejected.

The engine interface is `model_sha256`, `urdf_sha256`, `joint_names`,
`tool_sha256` (contracts `payload_sha256(config.data['tool'])`), `provenance`, and
`check_configuration(q)`. Hash declarations are binding checks, not proof of
geometric coverage or correctness. The engine/model must still be reviewed for
this robot and Hand, including relevant self-collision pairs and geometry.

There is no newly invented capsule geometry, dynamic plugin loading or point-only
self-collision fallback. Existing point/table/box checks remain additional
approximations. The actual vetted collision engine/model is still a commissioning
input; this change does not qualify one.

## Offline command and next steps

```bash
~/yifan/droid-deployment/run-native-python -m franka_runtime.droid.admission \
  --config /home/chen-lab/franka-runtime/configs/fr3/droid.jointpos.physical.pending.json \
  --contract /home/chen-lab/franka-runtime/configs/fr3/droid_fr3_jointpos_v1.contract.json
```

This prints commissioning gaps and hashes only. It does not instantiate providers,
load a vendor SDK, contact a device, change settings, open a listener or start motion.

Remaining work is to supply/review the real identity process, collision engine/
model and unit/tool/workspace/stop values; run the cross-host mock timing test;
then separately authorize D7 shadow and D8/D9 commissioning/trials in order. The
robot can remain off for missing code/config work. Inspection and successful fake
sessions are not hardware qualification.

Project-local validation and pinned reconstruction are recorded under
`~/yifan/droid-deployment/validation/step7.json`, `step7-requirements.lock`,
`artifacts/runtime-step7/` and `artifacts/step7-wheelhouse/`. Native Franky dev67,
libfranka, Ruckig, earlier candidates and both frozen baselines remain unchanged.

Installed-wheel validation: **215 passed, 2 skipped** (pinned upstream contracts
lack the local M4/PPO modules). Tests include a complete admitted service session
with fake native consumption/receipts and supervised stop, constructor-failure
recording, cancellation before startup and provider binding/rejection. Passive
imports, pending-config refusal, offline inspection, pinned native artifact/API
checks, durable recording audit and `pip check` pass.
