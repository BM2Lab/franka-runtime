# DROID backend candidate: packaging and service integration

Implemented offline on 2026-10-23. The service now accepts a backend factory and
uses backend-supplied consumption/fault/stop evidence. It no longer supplies mock
application or mock stop evidence to an injected backend. The ordinary CLI still
selects only the mock; a physical backend cannot be injected through a mock
configuration. No pending config became runnable and no robot was contacted.

## Packaged native candidate

The host artifact lives under
`~/yifan/droid-deployment/artifacts/native-wheel/`:

- `franky_control-2.0.1.dev67+gdroidd6.libfranka.0.21.2-cp312-cp312-linux_x86_64.whl`
- `droid-native-pin.json`: wheel/native payload hashes and provenance.

This is an explicitly repacked **project candidate**, not an upstream release.
`scripts/droid/package_native_candidate.py` starts from the verified dev66 wheel,
replaces only the tested Franky core/extension binaries, patches their ELF needed
names/RPATH, updates distribution version/tags and regenerates RECORD. All
**21 bundled dependency payloads remain byte-identical** to dev66, including
libfranka 0.21.2 and Ruckig. It makes no new manylinux certification claim.

A new environment, `~/yifan/venvs/franka-droid-native-server10-py312`, imports the
wheel with PYTHONPATH/LD_LIBRARY_PATH cleared. Resolved native dependencies stay
inside that wheel (apart from normal system C/C++ runtime libraries), with no ROS
or frozen-prefix dependency resolution. Patched copies of the offline C++ test
executables were run against the packaged library/dependency payload, rather than
the earlier prefix-linked raw build: 11 DROID cases/47 assertions and 23 M4
cases/82 assertions pass. Existing native environments and frozen originals were
not changed. The earlier raw build and its separate provenance remain retained.

## Runtime implementation

- `droid/backend.py`: backend-neutral consumption and update evidence.
- `droid/native.py`: seven-joint `NativeDroidBackend`, an explicitly injected
  `FrankyArmDriver`, lazy artifact verification, native-row/chunk mapping, liveness
  refresh, state-age/consumption checks and real stop evidence.
- `droid/hand.py`: serialized worker for blocking hand operations and state reads.
- `droid/service.py`: backend injection, native acknowledgement dispatch, encoder
  and grasped-state feedback, backend version and stop handling; the mock uses the
  same update interface.

Imports and constructors open no device/network connection or control thread.
`FrankyArmDriver` takes an already explicitly supplied device and motion factory;
`start()` runs its blocking `robot.move` on a worker, with explicit rate/filter
options. No recovery, homing, Robot construction or automatic fallback is added.
Initial encoder state must have an explicit fresh CLOCK_MONOTONIC timestamp.
Native sessions are single-use: a new measured-state/device factory is needed for
another physical session, rather than reusing stale initial_q.

The executor remains pure and time-injected. Native row IDs are distinct from
chunk/authority IDs. First-row `applied` is emitted only after native consumption
maps back to that chunk's first row. Every newly commanded row must be consumed
before the next logical row; skipped rows and missing acknowledgements fault.
Startup consumption may arrive after the first executor tick, within its explicit
startup/consumption deadlines. A producer refresh cannot change an active row's
q or silently extend its immutable native deadline.

### Deadline and bounded-hold distinction

An unconsumed new target is first published with the original chunk deadline.
That prevents its first native application after the chunk becomes invalid. Only
**after native consumption** is confirmed may the adapter publish a new internal
hold identity for the **same already-applied q**, bounded by the original chunk
deadline plus configured hold timeout. This is not a new policy action. Its
receipt does not emit a second chunk `applied`. A subsequent logical row still
requires its own native consumption, and the live executor must refresh the
publisher watchdog throughout holding. No reference interpolation or Ruckig is
introduced.

Native receipts are validated after draining, against a fresh host-clock sample;
a callback newer than the executor's earlier timestamp is not falsely rejected.
Ring order and latest-status order are tracked separately because latest status
may legitimately overtake a ring drain. Faults, native exceptions, stale encoder
state, telemetry overflow and worker faults latch the authority and request stop.
Stop completion requires native finish/rest/fault evidence **and actual worker
join**. A stop/hand deadline reports failed evidence, never MOCK_STOP_EVIDENCE.

## Hand worker semantics and remaining qualification

One blocking SDK operation runs at a time, with one latest pending intent. A new
pending intent replaces an older pending intent; it does not preempt the active
operation. Command history is bounded and overflow faults. Stop discards pending
work and waits for the active call; inability to become quiescent by the backend
stop deadline is a fault. No assumption about `move_async` preemption is made.

SDK false return values and exceptions are latched. Width and grasped flags come
from supplied measured-state reads. State timestamps and in-flight operation
status are exposed in `hand_status`; the moving flag conservatively includes
in-flight work. A blocking SDK call prevents this serialized worker from reading
another state simultaneously. Thus **50 Hz fresh hand measurement during motion
is not established**. Before physical selection, choose and qualify an SDK-safe
concurrent read/cancellation arrangement or revise the state-freshness contract
explicitly. Cached hand state must not be presented as a newly measured sample.

## Tests and scope

Installed-wheel validation: **167 passed, 2 skipped** (upstream contracts lack
the local M4/PPO modules), plus the C++ and native artifact checks above.
`pip check` passes. Exact pins/reconstruction inputs and receipts live in
`~/yifan/droid-deployment/native-requirements.lock`, `artifacts/native-wheelhouse/`
and `validation/step4.json`. Runtime 0.0.4 here is a source-patched candidate,
identified by wheel SHA256, not the unchanged upstream 0.0.4 artifact.

The test suite uses real SessionAuthority/ChunkExecutor/service logic with fake
native devices. It covers delayed applied receipts, callback/clock races, unknown
identity, producer/consumption loss, encoder staleness, native failure, join/rest
stop requirements, bounded hold admission and hand blocking/errors/replacement.
The existing localhost mock wire tests still pass. Hardware constructors are never
used in these tests.

Before physical operation, remaining work is the versioned physical config and
commissioned device factory/identity/tool/workspace verification, reviewed hand
feedback/cancellation behavior, bounded durable session recording, cross-host mock
timing, and the D7/D8/D9 commissioning sequence in `droid-d6-rt-host.md`. The
artifact pin verifies identity only; it is not motion authorization. The physical
selector remains fail-closed while those pieces are incomplete.

Native execution identity in state samples comes from confirmed native rows,
not the executor's projected next row. Old encoder feedback becomes invalid
without converting a completed clean session into a new tracking fault. A stop
requested before native startup has no native rest/join evidence: the adapter
reports failure by its deadline rather than hanging or fabricating a mock success.
The physical session factory must resolve that admission/stop lifecycle before use.


## Step 5 update: physical assembly and concurrent Hand (offline)

[Physical configuration and factory](droid-physical-configuration.md) supersedes
the serialized Hand limitation above for the new physical candidate. The old
serialized worker remains available for tests/adapters. Physical schema v2 and
factory checks are implemented; commissioning inputs and concrete identity/
collision adapters remain pending, and the physical CLI stays disabled.
The native dev67 wheel and all underlying dependencies remain unchanged.
Step 4 receipts are historical; step 5 has separate runtime artifacts/validation.


## Step 6 update (offline)

[Bounded recording and supervised shutdown](droid-recording-shutdown.md) are now
implemented and tested with fake devices. This supersedes the earlier code-work
blocker above, not hardware qualification. The physical selector remains disabled
until concrete identity/collision adapters, commissioned values, admission
composition and cross-host/per-unit qualification are complete.
