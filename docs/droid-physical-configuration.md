# DROID physical configuration and device factory

Implementation-order step 5 is an **offline candidate**, not hardware qualification.
The physical CLI remains disabled. No robot connection, homing, recovery, load/TCP
write, motion, or host-network change was performed during implementation.

## Configuration

`configs/fr3/droid.jointpos.physical.pending.json` is physical schema v2. All
commissioning fields remain null. Inspect it without loading Franky:

```bash
~/yifan/droid-deployment/run-native-python -m franka_runtime.droid.physical_config \
  --config /home/chen-lab/franka-runtime/configs/fr3/droid.jointpos.physical.pending.json
```

The v1 mock schema and its mock braking model remain separate. Physical admission
has joint position/margin, first-row target lead and consecutive-row target speed
limits. Native guards separately bound **measured** velocity, raw command torque,
callback gaps and publisher freshness. These do not promise acceleration/jerk
limited impedance motion; no Ruckig planner is inserted. Gains, damping, error
clipping, Coriolis, Franky torque slew and libfranka filtering/rate limiting are
explicit. Desired joint velocity remains zero in the DROID impedance law.

Schema validation rejects unknown fields, missing values, nonfinite numbers,
malformed vectors, unusable joint intervals, inconsistent watchdogs, tool transforms
and changed artifact hashes. The candidate supports FCI10 only. Artifact identity
and a reviewed configuration do not by themselves enable the physical CLI.

## Device factory boundary

`franka_runtime.droid.physical_factory.PhysicalDeviceFactory` is passive until
`connect()` is called. That call requires explicit device-setup authorization,
a complete reviewed configuration, matching contract hash/unit/joint order/hand
mapping, and separately supplied identity-reader and collision-checker adapters.
There is no default network identity reader or collision fallback.

The identity reader must return matching address, model, firmware, FCI version,
arm and control serials, `clock_id`, and a fresh `observed_ns` in this host's
CLOCK_MONOTONIC domain. An IP address or successful FCI connection alone is
insufficient. Franky's Robot has no `server_version` property.

The collision checker must identify its reviewed model with `artifact_sha256`
and implement `check_configuration(q)` by raising on rejection. Its implementation
and geometric coverage need commissioning review; hashing alone does not prove
collision checking. Workspace point/table/box checks are additional approximations,
not self-collision checking. Target checks interpolate the measured-to-first-row
and subsequent-row segments; they do not prove the robot follows those segments.

Franky's Robot constructor writes collision thresholds. Consequently device setup
is not a read-only probe: explicit configured thresholds and Enforce realtime
settings are used. No load/TCP setters or automatic recovery are called. Measured
idle/error-free state, finite q/dq, joint bounds/speed, load/mass/inertia/COM and
F_T_EE must match commissioning. Preflight refreshes robot and Hand feedback after
operator waiting, rather than reusing a construction-time snapshot. A single
backend is a single session. Construction does not start control/Hand threads.

## Concurrent Hand feedback and cancellation

The frozen libfranka 0.21.2 `include/franka/gripper.h` explicitly documents class
members as thread-safe. `src/gripper.cpp` uses UDP `readOnce()` for state and TCP
commands for move/grasp/stop; `src/network.h` has separate transport mutexes and
command-response IDs. Franky's Python binding releases the GIL for these calls.
This supports one state reader alongside one serialized blocking command worker.
Franky's async wrapper waits for a previous future, so it is not used as an
assumed preemption mechanism.

`ConcurrentHandWorker` adds an independent bounded cancellation worker. Stop
clears pending intents and issues direct SDK stop requests while an active command
remains. A successful stop reply is not sufficient: actual command completion
and all worker exits are required for quiescence. Only the specific SDK
`CommandException` reporting an aborted command is expected during intentional
cancellation; other exceptions/false results fault. A blocked read/stop or command
can exceed the backend deadline, which reports failure without inventing a join.

The state reader checks strictly advancing SDK message counters and host timestamp
freshness. Width/grasped are measured; libfranka has no moving bit, so moving is
inferred from measured width changes and conservatively includes pending work.
The configured poll rate is a target, **not a qualified hardware measurement rate**.
Fresh feedback during blocking motion, cancellation races/latency and stop behavior
still require per-unit hardware qualification.

## Startup and remaining work

Measured initial dq is retained. Before the first native callback, state feedback
is invalid and target submission is rejected. The explicit startup deadline allows
SDK startup without prematurely applying the ordinary per-row consumption timeout.
A state query during that interval does not create a false stale-state fault.

Physical service selection stays disabled while durable recording, supervised
shutdown, concrete identity/collision adapters and commissioning remain incomplete.
A stop before native startup still has no native rest/join evidence and fails by
its deadline. Next implement bounded durable recording and supervised lifecycle,
then cross-host mock timing and the authorized D7/D8/D9 sequence. The robot may
remain powered off for the next code step.

## Offline validation receipt

The updated project-local Python runtime wheel passes **189 tests, 2 skipped**
(the pinned upstream contracts distribution lacks the M4/PPO contract modules).
Passive imports run with vendor imports, sockets and thread creation forbidden;
pending physical configuration is refused. The real pinned dev67 API constructs
motion parameters without devices; artifact hashes, zero friction and disabled
limit repulsion are checked. `pip check` passes.

Installed candidate: `~/yifan/venvs/franka-droid-native-server10-py312`.
Reconstruction/validation: `~/yifan/droid-deployment/validation/step5.json`,
`step5-requirements.lock`, `artifacts/step5-wheelhouse/` and
`artifacts/runtime-step5/`. Runtime 0.0.4 is distinguished by its wheel hash.
Historical step 4 artifacts are retained. Only the project Python runtime was
reinstalled; Franky/libfranka/Ruckig and frozen original stacks were unchanged.
