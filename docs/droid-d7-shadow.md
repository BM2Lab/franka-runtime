# D7 read-only FCI shadow

This mode runs the same session authority, chunk executor, row admission and
workspace checks as the DROID service, using measured encoders from a separate
libfranka reader. It never starts a servo controller or sends arm/Hand commands.
It is separate from mock and from motion-authorized physical deployment.

## Runner contract

- `hello.versions.backend = shadow_libfranka_readonly_v1`.
- `SessionAuthority(expect_application=False)`: only acceptance/rejection receipts;
  no applied receipts or physical executing sequence. Pace replanning from accepted.
- State is a `FrankaStateSample`: measured q/dq and Hand width, original arm/Hand
  read timestamps, monotonic SDK counters, validity, faults and real clock identity.
  Virtual reference values never replace measured state.
- Acceptance and dispatch each refresh measured q before row-0 lead, joint bounds,
  row speed and interpolated workspace checks. No clipping or simulated arm state.
- Executor targets, holds, underruns and would-be Hand intents are journaled.
  A Hand intent is logged, never passed to a command worker.

## Components and validity

`native/droid_readonly_state/` calls only libfranka Robot/Gripper constructors,
`serverVersion()` and `readOnce()`. It deliberately avoids Franky's Robot
constructor, which writes collision thresholds. The helper is built in the
DROID project and links the unchanged frozen libfranka 0.21.2 prefix. It preserves
FCI10 compatibility checks and records reported mass/TCP configuration without
changing it. `--describe` opens no device.

`shadow_source.py` pins the helper SHA256, rejects loader overrides, supervises
its process and rejects regressed/restamped/backlogged packet streams.
`shadow.py` checks finite feedback, candidate soft bounds and measured joint-speed
limits, error/contact flags, **Idle** robot mode and Hand width/configuration.
Arm freshness is 40 ms; Hand freshness is 100 ms, with 50 Hz requested sampling.
Read timestamps are not publication or controller-consumption acknowledgements.
A source fault or invalid feedback invalidates state and latches a session fault.

The physical candidate supplies checks, not motion authorization. Its workspace
is the existing URDF FK + nominal tool-point box/table/interpolation checker.
This is **not a qualified self-collision engine or verified actual Hand model**.
Unknown unit serials and tool calibration remain physical-admission blockers.
The runtime records this geometry scope; passing D7 cannot clear those blockers.

## RT launch

The host config `configs/fr3/droid.jointpos.shadow-link.json` binds to
10.77.0.2:47700, uses the existing peer allowlist/token environment, and reads the
FCI10 arm at 172.16.0.1. Confirm that endpoint belongs to the intended arm; IP
alone is not unit identity. It authorizes no actuation. Do not run alongside a
controller, homing script or another session using the same endpoint/port.

Run after the operator authorizes read-only FCI access:

```bash
cd /home/chen-lab/franka-runtime
FRANKA_DROID_TOKEN="$(cat ~/.config/droid/token)" \
  /home/chen-lab/yifan/droid-deployment/run-native-python \
  -m franka_runtime.droid.shadow_cli \
  --config configs/fr3/droid.jointpos.shadow-link.json \
  --contract configs/fr3/droid_fr3_jointpos_v1.contract.json \
  --read-only-fci
```

The recording directory is `/home/chen-lab/yifan/droid-deployment/runs/d7-shadow`.
It must be new: the recorder refuses overwriting. For another run, change
`recording_directory` in a copied config. Helper hash/path are host-specific;
rebuild only in a project-local directory, then explicitly update the hash.
Do not rebuild/install any original baseline.

On the inference host use shadow mode and `expect_application=False`. Abort a
backend mismatch. Request state before starting the episode and inspect validity.
Acceptance can reject policy chunks from the fixed real start configuration;
the robot is stationary while the virtual executor advances, so this differs
fundamentally from a closed-loop physical policy evaluation.

## Records and shutdown

`events.jsonl` contains `shadow_acceptance_check` with pass/fail, stage, chunk hash,
measured q and feedback timestamps; `shadow_virtual_step` contains the executor
step, real q/dq and source packet; `shadow_would_gripper_command` contains intents.
Existing receipt/state/session events remain. Reader stderr is in
`reader.stderr.log`; inspect the final recording seal and shutdown report.

Release or Ctrl-C/SIGTERM terminates/joins only the read-only helper and logging
workers. Stop evidence says `mode=shadow_readonly`, `source_joined` and
`no_commands_sent`; it does **not** claim a physical stop, at-rest state, Hand
cancellation or native controller callback timing. No automatic recovery/homing.

Offline tests use fake source packets and a localhost protocol client. They cover
backend identity, no applied receipts/Hand commands, real-q acceptance, invalid
feedback, virtual holds/underruns, source counter integrity and clean release.
Actual FCI sampling/freshness, cross-host timing and recorder load remain to be
measured in D7. D8 motion/Hand/stop qualification is a separate task.

Offline installed verification: **247 passed, 2 skipped** (180 runtime + 67 pinned
contracts). Receipt: `~/yifan/droid-deployment/validation/d7-shadow.json`. The
helper pins its transitive ROS model-library directory with executable RPATH;
clean-launch `--describe` and linkage checks require no sourced ROS environment.
Neither ROS packages nor the frozen libfranka prefix were changed.
