# DROID recording and supervised shutdown

Implementation-order step 6 is an offline candidate. The physical service selector
remains disabled. The robot was not accessed and native software was not rebuilt.

## Recording format and boundaries

`franka_runtime.droid.recording.SessionRecorder` is constructed passively. Explicit
`start()` checks the parent directory and free space for the configured event-file
budget plus reserve, exclusively creates a private session directory, writes an
initial `status: open` manifest, then starts one non-RT writer. Existing directories
are never reused or overwritten. The caller supplies a new directory per run.

The physical schema's `recording` section specifies directory, queue capacity,
maximum record bytes, maximum total event-file bytes, minimum free-space reserve,
fsync interval, and bounded close timeout. Its pending values are all null.
`from_physical_config()` constructs a recorder from reviewed settings without
starting it or contacting hardware. Importing any new module remains passive.

`events.jsonl` records monotonically numbered JSON events with host monotonic
admission timestamps. Native callback timestamps and ticks remain separate fields.
The service records configuration/contract/version context, admitted lease,
clock probes/offset estimate, commands, accepted/applied/rejected receipts,
supervisor/Hand feedback, native samples/latest status, stop evidence and shutdown.
Authentication requests/tokens and camera data are not recorded. Callers must not
supply secret values in custom metadata.

Native samples retain available q/dq, reference, raw/tracking torque, outgoing
Franky torque, publication/consumption identities, faults, drop counts and stop
flags. `outgoing` precedes libfranka processing and does not measure final actuator
torque. The native ring is drained by the Python adapter; file I/O never occurs in
the 1 kHz callback. Native ring overflow remains its own fault.

Queue admission never waits for disk. Queue overflow, excessive record/file size,
nonfinite/unserializable data or writer failure permanently faults the recording.
The service/backend then requests stop (recording fault 209); failed receipt
persistence latches `receipt_recording_failed` before dispatch can publish it.
Queue admission is not a durability acknowledgement. The writer periodically
flushes/fsyncs and fsyncs again on close. The interval is a target, not a guaranteed
latency during I/O failure. A process/power crash can lose the unsynced tail;
the manifest stays open and must not be treated as a completed recording.

Close drains accepted events, fsyncs them and atomically replaces/fsyncs the
manifest with counts, event-file SHA256, errors and terminal stop evidence.
`sealed` means data persistence completed; it does **not** mean motion/stop was
successful. Consult `terminal.ok` and stop evidence separately. Failed/timed-out
writers never report successful close or worker join. Free-space reservation is
an admission check, not protection against another user filling the disk later.

Inspect without loading Franky or opening a device:

```bash
~/yifan/droid-deployment/run-native-python -m franka_runtime.droid.recording /absolute/session/directory
```

The inspector streams events with a line-size bound and verifies sequence order,
counts, byte totals and digest for terminal manifests. Corrupt/truncated data is
reported as an error. Open records remain unsealed evidence.

## Supervised shutdown

`DroidRuntimeService.shutdown()` requests shutdown and stops command intake; it
does not join device/disk threads from a signal handler. `serve()` uses
`finish_shutdown()` in its finally block. The control supervisor keeps processing
native telemetry, faults and the configured stop until actual evidence arrives
or the shutdown wall deadline expires. Without a running supervisor,
`finish_shutdown()` pumps the same `control_step()` implementation.

Completion requires joined supervisor/device/Hand workers and native finished,
sustained-rest, fault-free stop evidence with no dropped samples. Timeouts or
exceptions never fabricate rest/join. Shutdown reports and the recording manifest
preserve failures. Stopping before native startup still cannot prove a native
rest/stop and fails closed. There is no added robot.stop(), recovery, homing,
gain change or alternative fallback controller.

After the bound expires, supervision exits with an unsuccessful report; a blocked
SDK or writer thread may remain alive. Python cannot forcibly cancel such a call.
Native watchdog/stop remain independent, and the operator's physical stop
procedure is still required. These tests do not qualify that procedure.

## Validation and next step

Fake-device and localhost tests cover persistence/integrity, queue/file bounds,
disk and close failures, failed receipt persistence preventing dispatch, native
sample/receipt recording, delayed stop completion, unjoined-worker timeouts and
pre-start stop failure. Existing mock wire sessions continue to pass.

The project-local Python runtime candidate is packaged separately under
`~/yifan/droid-deployment/artifacts/runtime-step6/`; validation/reconstruction are
in `validation/step6.json`, `step6-requirements.lock` and
`artifacts/step6-wheelhouse/`. Franky dev67, libfranka, Ruckig, old candidate wheels
and both frozen baseline families are unchanged.

Next: concrete reviewed identity/collision adapters and a physical admission
composition tying the recorder/lifecycle to commissioned settings; cross-host mock
timing can then precede authorized D7/D8/D9 qualification. Missing unit/tool/Hand/
workspace/stop values remain blockers. The robot can remain off for code work.

Installed-wheel validation: **205 passed, 2 skipped** (upstream contracts 0.0.2
lacks the local M4/PPO modules). Passive imports, pending-config refusal, pinned
native artifact verification/status serialization, streaming recording integrity
and `pip check` pass. The API recording smoke test contains only an unstarted
motion snapshot and explicitly marks hardware access/control as false.


## Step 7 update: physical admission composition (offline)

[Physical admission](droid-physical-admission.md) now ties reviewed physical
settings, device checks, recording and the existing service/protocol to supervised
shutdown. Ready-lease cancellation records that control never started instead of
fabricating native rest evidence. Explicit recorded-identity and collision-engine
binding adapters are available; neither supplies missing per-unit data nor
qualifies an engine or live identity procedure. The inspection CLI has no execution
option; the mock CLI cannot select physical devices. Real providers/commissioning
values and cross-host/D7/D8/D9 qualification remain outstanding.
