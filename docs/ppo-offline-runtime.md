# Phase 3: installed PPO adapters and learning client

Phase 3 is implemented offline in candidate `contracts`, `franka-runtime` and
`learning` **0.0.3**. The old deployment checkout, historical bundles and raw logs
remain intact. This is not a hardware cutover or motion approval. No FCI
connection, firmware change, native build or frozen-stack modification occurred.

## Ownership and supported routes

| Component | Owner |
| --- | --- |
| FR3 29D metadata validation and typed joint-policy session | `contracts.franka_policy` |
| Position/explicit velocity-impedance native adapters | `franka_runtime.common.backends.franky_joint` |
| Reference state, payload admission, guards, bounded callback queue | `franka_runtime.service.joint_policy` |
| Leases, source timestamps/deadlines, receipts and latched stop/fault | `franka_runtime.service.authority` |
| CPU ONNX verifier/exporter and subprocess worker | `learning.artifacts.franka_bundle`, `learning.integrations.onnx` |
| Waypoints, stale-result scheduling, repetitions and off-callback recording | `learning.experiments` |
| Standalone client and historical replay/report tools | `learning.integrations.franka`, `learning.evaluation.franka` |

Runtime has no dependency on `learning`, Torch, ONNX, Isaac, ROS or the old
`franka-motion` package. The optional PPO numerical extra uses NumPy/PyYAML.
Native controller code remains in the separate, checksum-pinned Franky fork.
The existing dev66 M4 comparison artifact is reused; no replacement native wheel
or libfranka/Ruckig build is required by this extraction.

Only the existing FR3 bare-flange **position-increment** and **explicit velocity
impedance** contracts are translated. Native Ruckig velocity preemption,
Panda/server6 modified control and grippers are not silently substituted.

## Preserved semantics

- Policy rate 50 Hz; six actions; 29 observations: measured relative q (7),
  measured dq (7), base-frame target minus encoder-FK flange position (3),
  normalized consumed reference (6), previous consumed raw action (6).
- Position commands increment the last consumed reference and project to its
  declared bounds. Desired velocity remains zero in this route.
- Explicit velocity commands publish six scaled velocities and zero J7 velocity.
  Native C++ integrates its reference at 1 kHz. Actor feedback reads that native
  reference, rather than projecting the next action before inference.
- Session start seeds reference from measured q and clears previous actions.
  J7 holds its session-start reference. Waypoint changes carry reference/history
  forward; results for an earlier waypoint are discarded. A new session has a
  fresh lease, episode identity, worker and history.
- Consume prior inference once, then submit the next observation. A fast result
  is never consumed immediately after submission. The legacy 15 ms pending
  inference timeout and 50 ms command window remain explicit; a result already
  ready at the next policy tick is consumed before checking pending timeout.
- Local worker process, single-thread CPU ORT settings and bounded one-request
  IPC are preserved. No network worker or remote clock conversion is added.
- Native adapter motion/stop/watchdog kernels retain their source behavior.
  Callback recording only enqueues bounded records and updates guard state;
  serialization and disk writes happen outside the callback. Loss is reported
  and faults the session. Stop/queue failures cannot produce a complete result.

The joint native API exposes applied q/dq but **no sequence acknowledgement**.
Native publications therefore do not fabricate `applied` receipts. The mock
labels its logical application evidence; shadow produces accepted receipts only.
A returned publication call is not evidence of physical application.

## Installation and offline use

Use isolated project environments and the three compatible candidate wheels.
Install `franka-runtime[ppo]==0.0.3` for the runtime integration and
`learning[franka-ppo]==0.0.3` for the client/worker. Do not install project code
into either frozen original stack. The tested numerical/inference interpreter
is CPython 3.12; native artifact qualification remains specific to its ABI.

The installed worker is launched with `python -I -m ...` and never imports
Python files from a transfer bundle. New exports contain model/config/vectors,
schemas and archival source snapshots, with `artifact_format: franka_data_only_v1`.
Historical bundles may retain hashed legacy verifier files as inert evidence.
Only the narrowly named legacy `__pycache__` files are ignored and reported;
new-format bundles require an exact file inventory. Supply the manifest SHA256
independently: a computed hash alone does not authorize a model or motion.

With explicit paths to an existing bundle, reviewed profile, measured-state
fixture and SHA-anchored bare-flange URDF:

```bash
learning-franka-verify "$BUNDLE" --manifest-sha256 "$MANIFEST_SHA256"
learning-franka-ppo \
  --bundle "$BUNDLE" --manifest-sha256 "$MANIFEST_SHA256" \
  --profile "$PROFILE" --state "$STATE_JSON" \
  --model "$MODEL_URDF" --model-sha256 "$MODEL_SHA256" \
  --worker-python "$WORKER_PYTHON" \
  --mode shadow --target 0.45 0.0 0.35 --duration-s 1 \
  --output /tmp/franka-ppo-smoke-unique --smoke
```

This `shadow` mode uses a supplied static state and URDF FK; it opens **no FCI
connection**. It is not a live encoder shadow run. `mock` additionally supplies
logical servo reference ticks/receipts; neither mode simulates robot dynamics.
For velocity shadow, retain the legacy one-policy-step virtual-reference update;
for mock, integrate in 1 ms logical ticks with carried fractional elapsed time.

Templates: `configs/fr3/ppo.position.pending.yaml` and
`ppo.velocity-impedance.pending.yaml`. Their joint bounds come explicitly from
verified training contracts, and start tolerance/guards retain the old pending
profiles. They contain no robot host or transferred motion approval. Native
constructors require an explicitly attached, approved connection; the CLI exposes
no connection or hardware execution option. Full per-unit admission/stop
qualification is still required before exposing a physical run entry point.

Replace `--target` with `--catalog PATH --catalog-sha256 SHA --path-name NAME
--repetitions N` for the extracted waypoint scheduler. Path repetitions are
continuous; `learning.experiments.runner.run_repetitions` schedules separate
sessions, stops on faults/unconfirmed stops, and requires explicit repeat
admission. Neither mechanism homes the robot automatically.

Outputs include effective config, events, receipts, terminal feedback,
off-callback telemetry and final metadata. Existing output directories are never
overwritten. Persistent runs require a mounted, writable external
`LAB_DATA_ROOT`; `--smoke` permits bounded explicit `/tmp` output. Offline stop
and tracking fields are clearly labelled logical, not physical measurements.

## Validation and remaining qualification

See [phase 3 receipt](../deployment/migration/phase3-validation.json) and
[extraction provenance](../deployment/migration/phase3-extraction.json).
Both actual transferred models pass 1,024 stored sentinel/random vectors,
runtime observation/action parity and 30 live IPC sentinels. Historical position
and velocity shadow observations/actions/references replay against the same
manifest identities. Saved circle/Lissajous metrics reproduce exactly without
writing into the raw recordings. Tests cover one-step order, old-waypoint
results, held joints, resets, deadlines, callback faults and recording loss.

The exporter was ported with explicit source/artifact roots and optional pinned
training dependencies. A fresh Torch/checkpoint export was not run here: no
training checkpoint or training framework is installed in the migration venv.
Validate a fresh export on the training workstation before publishing it.
Hardware, simulator extraction and commissioning remain later phases; historical
successful motion and these offline checks do not transfer approval.
