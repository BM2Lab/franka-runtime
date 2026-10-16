# Offline M4 runtime (phase 2)

The M4 hardware integration is now extracted into `franka-runtime` 0.0.2. It has
**no dependency on `franka-motion`, `franka-rl`, learning, Isaac or ROS**. Numerical
helpers needed for hardware reference preparation live in `common.motion`; the
native controllers remain in the separately maintained, hash-pinned Franky fork.

Physical status: **NOT QUALIFIED**. No FCI connection, native rebuild, controller
retuning or original-stack change was performed. The CLI exposes offline tools
only. `FrankyMotionBackend.connect()` refuses connection; an explicitly attached
connection/test double is required for adapter APIs. Supplying a real connection
requires its own operator authorization and per-unit commissioning. Historical
profile approvals are not carried into this package.

## Components

| Module | Responsibility |
|---|---|
| `common.backends.franky_m4` | Native parameter/reference mapping, encoder/model preflight, bounded capture serialization, scheduler scope and original stop/join/abort semantics |
| `common.backends.feedback` | Measured encoder FK, base/flange transforms and audit-only reported-pose comparison |
| `common.backends.offline` | Static-state mock and offline shadow; no physical dynamics or real-time simulation |
| `common.motion` | Extracted URDF FK/IK, analytic trajectory preparation, validation and offline numerical parity oracles |
| `common.artifacts.bundle` | SHA-anchored data-only bundle verification; no bundle code imports |
| `service.authority` / `service.m4` | Serialized lease/episode/sequence admission, one pending command, bounded receipts and native application correlation |
| `service.replay` / `service.cli` | Streaming historical log checks and installed `franka-m4` offline commands |

The Python torque/controller kernels are **offline reference oracles**, never a
replacement for native servo torque computation. Native callback/control behavior
and the 50 Hz reference / 1 kHz servo contract remain unchanged. Native start
still selects `limit_rate=False`, a 100 Hz libfranka cutoff, 1 Nm/tick slew,
configured projector/fixture/gains/guards and the existing native stop behavior.

## Environment and artifacts

Base runtime needs `contracts==0.0.2` only. The `m4` extra adds NumPy/PyYAML;
`dev` adds pytest. The exercised native backend is the existing CPython 3.12
server10 modified wheel `2.0.1.dev66+gm4comparefix.libfranka.0.21.2`. Its module,
libfranky and bundled library hashes ship as `common/backends/franky-server10.json`.
Other wheels or server6 are not silently selected. Base/offline numerical support
on Python >=3.10 does not imply native wheel ABI support on those interpreters.

Use explicit local contracts/runtime wheels in a new project environment; do not
install into a frozen original or reuse a simulation/ML environment as a native
build environment. The development environment used here is
`/home/chen-lab/yifan/venvs/lab-migration-phase01-py312` (no Franky installed).
The native API check used the existing modified environment read-only.

Candidate runtime wheel and release manifest:
`/home/chen-lab/yifan/lab-runtime-artifacts/phase02/`.
These are local candidates, not published releases. Configuration files remain
external and explicitly selected; no sibling checkout or home path is assumed.

## Offline commands

Choose a new output directory whose parent already exists and has capacity.
Outputs are exclusive; nothing falls back to the source checkout. Keep outputs
outside source Git or in an explicitly ignored artifact area. In a development
shell, unset inherited ROS `PYTHONPATH` and disable user-site imports.

```bash
RUNTIME_PY=/home/chen-lab/yifan/venvs/lab-migration-phase01-py312/bin/python
RUNTIME_ROOT=/home/chen-lab/franka-runtime
BUNDLE=/home/chen-lab/yifan/2026-10-12_m4_torque_v2_verified

env -u PYTHONPATH PYTHONNOUSERSITE=1 "$RUNTIME_PY" -m franka_runtime.service.cli preflight \
  --config "$RUNTIME_ROOT/configs/fr3/m4.tracking.pending.yaml" \
  --bundle "$BUNDLE" --state "$RUNTIME_ROOT/configs/fr3/m4.mock-start.json"

env -u PYTHONPATH PYTHONNOUSERSITE=1 "$RUNTIME_PY" -m franka_runtime.service.cli mock \
  --config "$RUNTIME_ROOT/configs/fr3/m4.tracking.pending.yaml" \
  --bundle "$BUNDLE" --state "$RUNTIME_ROOT/configs/fr3/m4.mock-start.json" \
  --output /external/artifacts/new-m4-mock
```

`--shadow` performs an **offline** shadow against supplied static joint state. It
never connects to FCI and emits no application receipts. Mock mode produces
logical consumption receipts only; encoder state remains static and references
are not passed through simulated robot dynamics. A completed mock run is not a
tracking-performance result or evidence of real-time deadlines.

Six pending profiles cover hold, Cartesian tracking, DLS tracking, posture,
Z-line and YZ-circle fixtures. Numerical gains, limits, timing and geometry come
from the recorded legacy source profiles. Host paths and approvals were removed;
all native artifact selectors now explicitly pin the existing dev66 wheel and
require stop behavior version 2. These are configuration candidates, not approvals.

For YZ-circle planning use `configs/fr3/m4.circle.mock-start.json`, derived from
the historical final circle session. The idealized home fixture is approximately
0.43 micrometers outside the profile's rounded circle-normal workspace margin;
that check correctly rejects it. The historical start is interior and passes.
The guard and radius were preserved, rather than relaxed to make the mock pass.
Neither example state is a command to home the currently attached arm.

Historical log replay:

```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 "$RUNTIME_PY" -m franka_runtime.service.cli replay \
  --config "$RUNTIME_ROOT/configs/fr3/m4.tracking.pending.yaml" --bundle "$BUNDLE" \
  --input /external/recording/servo.jsonl --output /external/artifacts/new-m4-replay
```

Replay streams bounded-size JSON records and checks tick/timestamp order,
recorded encoder-FK pose, filter prediction, faults and stop evidence. It does
not replay recorded torque, force or motion. Raw historical recordings remain
at their existing paths.

## Authority and native boundary

`create_m4_session()` binds authority to the effective backend configuration and
its profile hash. One owner/lease/episode may be active. Admission validates clock,
source age, exact command sequence, capability, payload hash, pose/twist frames,
workspace/speed bounds and configured posture capability. A command is copied
into a one-entry queue; publication rechecks lease/deadline/configuration. New
sessions require prior stop completion; faults cannot be reset through this API.

An accepted receipt does not imply application. `drain_to_authority()` correlates
native `reference_sequence`/tick/monotonic-raw telemetry with publications. It
rejects superseded/unconfirmed or late application evidence. Native faults and
log loss request the original stop path; `finish_authority_stop()` reports native
join/rest/stop evidence without clearing a fault. Receipt buffers are bounded;
overflow faults rather than silently continuing.

This is a local, in-process authority API, **not network authentication**. For
native integration use the native host/boot monotonic-raw clock domain. Remote
clock conversion, transport security and network service deployment are deferred.
The unchanged native reference API has its own publication-age watchdog, not a
new lease/client-deadline field. Python checks dispatch and telemetry deadlines
but cannot guarantee a hard native cancellation instant after publication;
late/unconfirmed application faults and requests stopping. This limitation must
be resolved or explicitly qualified before exposing a physical command service.

## Validation and remaining work

Validation: **141 passed, 3 optional-integration skips** in the combined suite;
**35 runtime tests passed against the built wheel**. Six real native route
constructors and three DLS cases passed with no Robot creation. Circle mock
tracking produced 3,671 logical application receipts with no loss. Historical
Lissajous replay checked 73,719 records: maximum FK difference
`3.89e-16 m`, zero filter-prediction difference. These are offline results.

See [phase 2 receipt](../deployment/migration/phase2-validation.json) for test and
artifact hashes, and [extraction provenance](../deployment/migration/phase2-extraction.json)
for source symbols/hashes. Tests exercise numerical golden outputs, trajectory
phase/twist timing, queue ownership/deadlines, reset isolation, native telemetry,
stop failure and import isolation. No test needs the old checkout to run.

The explicit `scripts/migration/check_native_api.py` check constructs real native
motion objects using a Python robot double, forbids `Robot` creation and replaces
scheduler mutation with a null scope. It checks all six routes and DLS numerical
parity; it is not a live-control test.

Next migration work is Phase 3: PPO worker/client and learning orchestration.
Hardware commissioning remains a separate phase per physical unit/tool and
firmware family. Panda, grippers, ROS backend and server6 modified-controller
qualification remain unimplemented here.
