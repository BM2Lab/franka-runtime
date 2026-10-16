# Migration plan: franka-rl deployment to the lab infrastructure

Date: 2026-10-16. Status: phases 0–3 implemented offline; see [implementation receipt](../deployment/migration/README.md). M4 integration is extracted; no hardware accessed. See [phase 2](m4-offline-runtime.md) and [phase 3](ppo-offline-runtime.md). Simulation extraction and commissioning remain planned.

## 1. Decision and scope

Make `franka-runtime` the canonical home of physical Franka runtime implementation. Do **not** copy the whole `franka-rl/deployment` directory into it: that directory mixes hardware control, inference, simulator integration, artifact tooling, experiment analysis, and historical evidence.

Use a behavior-preserving M4 vertical slice first, followed by the PPO runtime and its inference client. Extract simulation adapters and learning tools into their respective repositories. Keep the existing implementation usable until each replacement passes offline parity and, separately, per-unit hardware commissioning. Migration is not an opportunity to retune controllers or change stopping behavior.

The initial hardware implementation target is the previously exercised **modified Franky server10 M4 stack**. This is a software extraction target, not authorization to run it against the currently attached arm. Server6 support is a separate implementation/qualification item; do not infer support from having an upstream server6 baseline installed.

### Documentation and source snapshot

Reviewed repository READMEs/contributor instructions and applicable architecture docs at these revisions:

| Repository | Revision | Role |
|---|---|---|
| [robot-learning](https://github.com/BM2Lab/robot-learning) | `0ba9994c31983a89f0141157ba6bf71aeef00528` | Separate `contracts` and `learning` distributions |
| [isaac-tasks](https://github.com/BM2Lab/isaac-tasks) | `a656b326f9688ed7faf6e5b71cfa3fb742e36e6f` | Isaac Lab extension, simulated tasks/controllers/bridges |
| [franka-runtime](https://github.com/BM2Lab/franka-runtime) | `102b3f278312da008531c74023176259a3334483` | Hardware runtime scaffold, also local checkout revision |
| [openarm-runtime](https://github.com/BM2Lab/openarm-runtime) | `a195b122194145e864594ba2859ba9fcafee627d` | Independent OpenArm hardware runtime scaffold |
| Local `yifan/franka-rl` | `0bd7f54044f8cc404a53c3223a19966edbfbf917` | Extraction source; working tree clean at inspection |
| Local `yifan/franky-source-f88f0e9b` | `ea855c9ce3f48ece2c7a0bc4e1d7d033083e1bf9` | Native modified controllers, including M4/DLS |

Important source guidance: [runtime ownership](https://github.com/BM2Lab/franka-runtime/blob/102b3f278312da008531c74023176259a3334483/AGENTS.md), [learning/contracts ownership](https://github.com/BM2Lab/robot-learning/blob/0ba9994c31983a89f0141157ba6bf71aeef00528/AGENTS.md), [Isaac boundaries](https://github.com/BM2Lab/isaac-tasks/blob/a656b326f9688ed7faf6e5b71cfa3fb742e36e6f/AGENTS.md), and [OpenArm boundaries](https://github.com/BM2Lab/openarm-runtime/blob/a195b122194145e864594ba2859ba9fcafee627d/AGENTS.md). Recheck revisions before implementation if upstream changes.

## 2. Ownership and dependency boundaries

| Destination | Owns | Must not own |
|---|---|---|
| `robot-learning/packages/contracts` | Portable versioned action/observation/session/episode semantics; units, frames, joint order, validity and command receipts | Franky imports, controller algorithms, ML inference or device connections |
| `robot-learning/packages/learning` | Policy inference/adapters, experiment orchestration, recording/export, training/evaluation and report generation | Native servo loops, physical fault authority or robot recovery |
| `franka-runtime` | Franka interfaces, encoder/model FK, native controller integration, guard/watchdog/stop enforcement, command ownership/session authority, grippers and non-RT device adapters | Training, Isaac imports, full learning dependency or unqualified automatic hardware selection |
| `isaac-tasks` | Simulated robot/controller adapters, task configuration, observations/actions/rewards/reset semantics and parity checks | Physical Franky/FCI connections or hardware deployment authority |
| `openarm-runtime` | Its own OpenArm driver, control, stop and single/bimanual authority | Franka-specific drivers, gains, torque assumptions or stop implementations |

Dependency direction: runtimes and simulation depend on lightweight `contracts`; `learning` integrates through those contracts and optional clients. Neither runtime needs the full `learning` distribution. OpenArm shares protocol semantics, not a new Franka-derived base driver.

The current `franka-runtime` is a scaffold (`franka_runtime` import, Python >=3.10, `contracts==0.0.1`), not a functioning driver. Its import must remain usable without Franky, ROS, Isaac, or a connected robot. Vendor modules load only after explicit backend selection.

## 3. Source-to-destination map

Paths below are relative to `franka-rl`, unless otherwise specified. Package/module names in the destination column are proposed, not existing APIs.

| Source | Destination and treatment |
|---|---|
| `deployment/franky_runtime/franky_experiment/backend.py` | `franka_runtime/common/backends/franky.py`: extract encoder FK, state conversion and explicit backend adapters; separate mock/shadow implementations. Preserve frame/tool transforms and encoder feedback provenance. |
| `.../franky_experiment/motion_demos/hardware.py` | `franka_runtime/common/motion/` and backend integration: M4 configuration, native parameter mapping, per-unit checks, capture and stop integration. Separate configuration parsing from connection and execution. |
| `.../franky_experiment/core.py` | Split: portable message semantics to `contracts`; Franka limits/reference mapping/config validation to runtime; reference experiment schedule generation to learning. Retain legacy contract versions. |
| `.../franky_experiment/coordinator.py` | Split: local command admission, ownership, watchdog/lifecycle/fault authority into `franka_runtime/service`; policy request scheduling and generic experiment/repeat logic into `learning`. Runtime retains a standalone commissioning CLI with no learning dependency. |
| `.../franky_experiment/motion_demos/{backend,runner}.py` | Runtime mock/native interface and local execution lifecycle stay runtime; cross-trial experiment plans and aggregate analysis go to learning. Stop/reset operations remain mediated by runtime. |
| `.../franky_experiment/path_runtime.py` | Separate reusable path specification/advancement from Franka reference conversion. Generic path-task execution belongs in learning; hardware admissibility/reference state belongs in runtime. Preserve transition/dwell semantics. |
| `.../franky_experiment/policy_worker.py` | Optional ONNX worker/client in learning. It can run locally on the robot host in a **separate inference environment**, or remotely after timing validation; no ONNX/Torch requirement for base runtime. First port keeps the existing worker placement and timing. |
| `deployment/franka_policy_bundle/{contract,bundle,suite}.py`, `deployment/schemas/` | Portable semantic types to contracts; bundle export/packaging and suite orchestration to learning; lightweight identity/hash/admissibility checks at runtime admission. Keep old bundle verification available during transition. |
| `deployment/{config,suites,target_sets,path_catalogs}` | Split portable task descriptions and experiment suites into learning; robot/controller profiles into runtime. Keep task assets with their consumer. Do not transfer host paths or historical approval flags into reusable profiles. |
| `deployment/reference_governor/` | Preserve as an **optional standalone numerical distribution**, `franka-reference-governor`, maintained with runtime sources. Separate its Isaac adapter into `isaac-tasks/controllers`. It is not the active M4 controller and must not be enabled as part of migration. |
| `deployment/franky_runtime/{summarize_*,report_*,plot_*,prepare_m4_comparisons.py}` and corresponding analysis helpers | Learning-side evaluation/report tooling; first retain input log compatibility and existing figure/metric definitions. Profile preparation separates experiment choices from runtime admissibility. |
| `deployment/franky_runtime/read_only_probe.py`, diagnostic tools | Explicit runtime inspection/diagnostic CLI. Document that the current Franky constructor sets collision behavior: a connected probe is not configuration-free. Offline inspection must not instantiate a robot. |
| `deployment/{hardware_control_audit,model_audit}`, hardware results reports, dated handoffs | Historical archive in the old repo, linked from runtime provenance/qualification docs. Carry selected small golden fixtures with source/session/hash attribution; do not import all historical reports as current instructions. |
| `deployment/FR3_*`, training plans and simulation handoffs | Current simulation/controller documentation belongs in Isaac tasks; policy/training/export documentation in learning. Keep dated originals as history and replace entry points with migration links only after cutover. |

### Dependencies outside deployment: part of the extraction

1. **`source/franka_motion`** is currently a simulator-independent distribution (`franka-motion==0.1.0`, NumPy/PyYAML, optional Ruckig). M4 hardware imports its config, transfer, kinematics and trajectory functions. **The completed runtime must be independent of this package and the old repo.** Extract hardware-required configuration, reference preparation, kinematics and trajectory helpers into `franka_runtime.common.motion`, with provenance and parity fixtures. Put generic evaluation/suite tooling in learning and simulated controller/task adapters in `isaac-tasks`. Do not move the entire package wholesale or introduce a replacement external motion dependency merely to preserve the old layout. During incremental development only, a pinned legacy artifact may bridge unported modules; it is not an acceptable final dependency. Simulation should use its own vectorized implementation checked against shared equations/golden fixtures; a shared standalone numerical kernel is an optional later decision justified by actual consumers. No simulator needs a real Franky/FCI driver.
2. **Modified Franky C++ fork** contains the real servo implementations, including `m4_runtime_motion`, impedance/DLS/fixture processing, torque utilities, and existing velocity tracking. Keep a single canonical fork with pinned revision and build recipes; runtime depends on its qualified wheel. Do not duplicate C++ classes into the Python runtime or rewrite them during extraction. Record fork URL/license/upstream base, patches, wheel hash and native dependency hashes. Moving the fork to a lab organization, if desired, is a separate repository administration action.
3. **`scripts/deployment` and `tests/deployment`**, plus motion-specific source/tests elsewhere in `franka-rl`, contain exporters, verifiers, replay and simulation checks. Include their consumers in the manifest; moving `deployment/` alone would leave imports and tests broken.
4. **Modified ROS workspace** is outside this directory. Treat ROS as an optional later backend, with separate colcon builds and dependency manifests. Do not migrate both ROS and Franky control routes in the first slice or build through frozen baseline aliases.

### Repository strategy: keep upstream forks separate

Recommended organization (repository names below are proposals, not claims that these repositories exist):

| Repository | Responsibility |
|---|---|
| `BM2Lab/franka-runtime` | Lab-facing Python API/service, physical session authority, optional backend adapters, profiles, deployment/build recipes and integration/qualification tests |
| `BM2Lab/franky` | Upstream-derived Franky source plus the existing native M4/velocity/DLS extensions and their native tests; independent wheel releases |
| `BM2Lab/franka_ros2` | Upstream-derived ROS 2 packages plus maintained lab modifications; independent ROS build/test lifecycle |

Use the Git hosting platform's fork relationship where practical. Preserve full history, upstream attribution/licenses, existing patches and release provenance. Do not combine the forks into a new monorepo or copy their source under runtime: they have different upstream histories, build systems and release cycles. Upstream merges and native wheel qualification should not require restructuring the runtime package.

`franka-runtime` is the integration entry point, not the container for every dependency's source. It pins backend revisions/artifact hashes and declares a tested compatibility matrix. Franky and ROS 2 are optional alternative backends; normal deployment does not need both installed. Retain controller extensions in the current Franky fork during migration rather than also implementing them inside runtime. A future standalone native controller library would require its own design and parity work.

Do **not** push a whole `franka_ros2_ws` directory. A workspace is a local assembly of repositories and generated outputs. Publish modified source repositories only; keep a pinned `.repos` manifest and colcon recipes in runtime's deployment tooling. Include other modified source packages as explicitly owned dependencies. Keep `build/`, `install/`, `log/`, local environments, wheels and hardware recordings outside source Git history.

Prefer immutable wheel/version/hash references for Python/native deployment and exact commits in a ROS `.repos` manifest. Do not require Git submodules for ordinary runtime installation. Fetch pinned sources into explicit project-local build directories when rebuilding; never use an unpinned default branch or modify frozen originals. Store large release artifacts in an artifact/release store, with small manifests in Git.

Moving an existing personal fork into the lab or creating a lab fork should preserve the current patched commit and upstream tracking. Neither repository creation/transfer nor remote pushes are performed by this planning change.

## 4. Contract work required before integration

The current shared `CartesianDeltaSpec` describes a 3D delta in the robot base frame relative to **measured TCP at execution**, with episode-initial orientation held. It is not the existing six-action joint PPO contract or an M4 Cartesian pose/twist command. `contracts/sessions.py` is currently a placeholder, not an implemented lease/receipt protocol.

Introduce explicitly versioned, separately named capabilities; do not reinterpret existing VLA messages:

- Joint position-increment and joint velocity actions, including action joint order, held joints, scale/clipping, reference integration, and previous-action semantics.
- M4 Cartesian pose/twist references and fixture/posture configuration. Specify base/TCP frames, quaternion convention, wrench/twist ordering, units and reference timestamps. Gains/controller selection are session configuration, not arbitrary per-tick remote mutations.
- State/feedback validity, measured versus commanded/reference quantities, encoder-FK provenance, episode/session identity, and terminal context.
- Ownership acquisition/release, bounded command sequence/deadline validation, accepted/applied/rejected receipts, and latched fault reports. Acceptance must not imply actual native application.
- Clock identity and conversion rules: same-host monotonic time versus simulation time or a remote clock. Expired commands, disconnects and ownership loss have explicit runtime-specific behavior.

Preserve the legacy PPO contract: 50 Hz policy, native 1 kHz controller, measured encoder q/dq and FK feedback, previous reference/action ordering, the qualified one-step inference delay, held J7, and distinct position versus explicit velocity-impedance mapping. A 20 Hz VLA contract is a different capability, not a new label for this policy.

The existing V2 local simulation bridge pauses physics while waiting/when its action queue is empty. That transport is useful protocol evidence, but **cannot be reused as physical servo scheduling or stop behavior**. The hardware loop continues independently of inference and must enforce wall-clock deadlines locally. Keep physical-session authority in runtime even if learning owns the client experiment loop.

## 5. Proposed target layout

Extend the existing scaffold rather than replacing its package structure:

```text
franka-runtime/
  src/franka_runtime/
    common/
      backends/             # explicit optional Franky; mock/shadow
      motion/               # reference/config/FK integration, no Isaac/ML
      artifacts/            # lightweight runtime admission checks
    fr3/                    # model-specific configuration and capability checks
    panda/                  # separate implementation/qualification status
    safety/                 # guard/fault/stop interfaces; native final authority
    service/                # local session authority, command/state API
    grippers/               # only implemented/qualified hardware variants
    lerobot_adapter/        # optional non-RT device adapter, not inference
  configs/{fr3,panda}/       # pending templates, no baked-in motion approval
  scripts/                  # inspect, preflight, shadow, explicit commissioning
  tests/{unit,parity,replay}/
  deployment/               # manifests/build recipes/commissioning instructions
  docs/                     # architecture, contracts, migration, qualification
  components/               # optional standalone numerical packages, if retained
```

Keep a minimal base dependency set and explicit extras for Franky, ROS, numerical planning and adapters. The current scaffold supports Python >=3.10, while exercised modified Franky environments use Python 3.12: publish a tested backend/Python matrix instead of assuming every extra works on every supported interpreter. Update the exact contracts pin deliberately with its API revision.

No source-tree `sys.path` insertion, implicit parent workspace imports, copied venvs, or hard-coded `/home/chen-lab` paths in reusable code. Bundle verification currently loads code from the bundle path; replace this with an installed, versioned verifier plus data-only bundles for new releases. Retain an explicit legacy verifier for existing bundles; hash verification establishes identity/integrity, not sandboxing or motion approval.

## 6. Implementation sequence and exit criteria

### Phase 0 — freeze extraction inputs and inventory consumers

Create a machine-readable migration manifest: every in-scope source path, destination, source commit/hash, dependencies, tests, config consumers and disposition (extract/split/archive/defer). Include external Franky and `franka-motion` artifacts. Record dirty changes if any. Inventory hard-coded paths and bundle-embedded code; identify old CLI/config consumers before removing anything.

Exit: every executable in the first slice has an owner, reproducible dependency receipt and tests; no source or frozen software changes. Existing hardware evidence locations remain intact.

### Phase 1 — contracts and offline parity fixtures

Add the missing portable messages and session semantics in `robot-learning/packages/contracts`, with serialization/version/rejection tests. Build lightweight reference conversion, FK/frame mapping, reset and controller/stop fixtures from the pinned sources and selected historical logs. Distinguish numerical tolerances from operating limits. Publish/pin a compatible contracts release before runtime starts relying on it.

Exit: tests run without ML, Isaac, ROS, Franky or hardware. Existing 3D VLA schemas remain unchanged. Legacy PPO/M4 adapters have explicit translation tests.

### Phase 2 — first M4 runtime slice

Port state/FK conversion, config/admission checks, Franky M4 parameter mapping, native capture/guard/stop integration, and mock/shadow backends. Pin the existing modified native wheel and extract the M4-required `franka-motion` helpers into runtime with parity tests. Keep the current native loop and non-RT update cadence; do not tune gains, filters, projectors or watchdog thresholds. Keep local commissioning possible without installing learning.

Exit: offline M4 preflight and replay run from installed packages, independent of `franka-rl` and `franka-motion`, with lazy imports and no physical connection. Hold, Cartesian tracking, DLS/joint PD, posture and fixture parameter mappings match the old implementation. Preserve native constructor side effects in documentation rather than calling connection “read-only.”

### Phase 3 — PPO client, artifacts and learning orchestration

Port the position and explicit velocity-impedance adapters, then move ONNX worker, generic experiment scheduling, report generation and policy export to learning. Initially retain local worker/process placement and observed timing. Add runtime command admission/receipts around the existing behavior before changing IPC/host topology. Keep recording off the native callback and report dropped samples/backpressure.

Exit: bundle sentinel/random/replay parity, action/reference sequence and stale-result behavior agree with the source. No runtime dependency on learning/Isaac/Torch. The installed standalone client can run mock/shadow from an external artifact root.

### Phase 4 — simulation extraction and parity

Port the appropriate task/config/MDP pieces to `isaac-tasks`, with simulated adapters for the exact declared runtime capabilities. Separate shared numerical kernels from real drivers. Include per-environment selective reset of references, previous actions, queues, clocks and recorded episode context. Preserve vectorized device-resident execution.

Exit: model/joint/TCP mapping, controller equations and torque-processing order, policy timing/delay and reset behavior are checked against fixtures. The existing simplified FR3 visual-reach controller is not advertised as an equivalent implementation of M4/PPO. Do not register placeholder tasks as deployment-ready.

### Phase 5 — per-unit commissioning and cutover

Only after separate operator authorization, verify arm identity, firmware/FCI, model, tool/payload/TCP and native hashes; proceed through explicit connection/shadow, hold/stop and task qualification. This is required independently for FR3/Panda and each compatibility family actually supported. Historical successful sessions are regression evidence, not transferred approval.

Exit: new installation has its own qualification receipt and rollback procedure. Update old entry points to point to the new releases, then remove duplicate active code only after all consumers migrate. Keep legacy source commits, original bundles and log readers available for old reports.

## 7. Verification and preservation checklist

- Import the base package without optional native dependencies: no socket/robot creation or background control thread. CI uses fake backends and cannot discover/connect to hardware.
- Check measured joint encoder FK, base/flange/TCP transforms and tool handling against fixtures; do not silently substitute reported pose or assume a bare flange.
- Preserve native controller equations/projectors, torque clipping/slew/filter processing order, Coriolis/gravity ownership, startup callback handling and stop progression. A simulator change is not permission to modify hardware processing.
- Fault/replay tests cover stale/invalid commands, disconnect/ownership loss, callback gaps, command-success faults, guard breaches, logging overflow, stop completion/timeout and terminal-state capture. Do not replace reviewed native stop paths with a generic zero-torque fallback.
- Preserve metric definitions and log provenance. Mark partial/faulted trials accurately; distinguish fixture path-normal deviation from distance to a fixed anchor and measured joint torque from commanded/model torque.
- Preserve old bundle file hashes separately from any new canonical contract fingerprint. Record both when comparing old and new runs.
- Verify explicit artifact/output roots exist and have capacity. Native buffers, wheels, builds, raw logs, videos and checkpoints stay outside Git; small receipts, fixtures and config templates may be tracked.
- Recreate environments/builds at their destination; do not move venvs/CMake caches. Do not assume wheel ABI compatibility across Python versions or native dependency families.

### Shared machine constraints

Canonical host inventory remains `/home/chen-lab/franka-system-inventory/`; read its `AGENT_POLICY.md` before any installation/build/path change. It is not migrated into a project repo.

`/home/chen-lab/franka-stack-server6` and `franka-stack-server10` remain frozen, including their aliases. Create any new modified runtime environment/build under its owning project and select dependencies explicitly. Do not rebuild/install into originals, change global loader paths, or clean up shared packages as a migration side effect.

Historical raw data remain `/home/chen-lab/franka_ros2_ws/hardware_inventory/` at their recorded paths. Export/copy selected data with manifests for analysis; retain original identifiers and checksums. Reusable runtime docs should describe configurable storage and inventory locations rather than bake in this host layout.

## 8. Recommended first implementation change

Implement **Phase 0 plus Phase 1's contract proposal/tests**, then the **mock/offline M4 slice** in Phase 2. This provides one independently installable, reviewable path before splitting every historical deployment experiment.

Keep the work in separate reviewable changes: (1) extraction manifest and parity fixtures, (2) portable contracts, (3) offline M4 runtime, (4) PPO/learning client, (5) Isaac task adapters, (6) commissioning/cutover receipts. Do not combine extraction with a native rebuild, firmware change, controller retuning or additional hardware experiments.
