# franka-runtime contributor instructions

## Ownership and compatibility

This repository owns physical FR3/Panda interfaces, native controllers, grippers,
guards, watchdogs, faults/stopping, session authority and non-real-time adapters.
Depend on the lightweight `contracts` distribution, not `learning` or Isaac.
Importing the package must not import unavailable vendor libraries, connect to a
robot or start a control thread. Select optional backends explicitly.

FR3 and Panda are not interchangeable qualification targets. Inventory each
physical unit's model, firmware/FCI, Franky/libfranka/native dependency hashes,
joint limits, gripper/tool, payload/COM/TCP, workspace and stop behavior.
Do not assume one model's native build or bare-flange approval covers another
model, physical unit or tool-equipped configuration.

During extraction, record source revisions/provenance and preserve controller
equations, processing order, callback timing, native guards and stop semantics.
Do not silently change them to match a simulator. Validate reference conversion,
model/FK, controller output, selective reset and fault/stop parity with fixtures.
Historical qualification is evidence to cite, not approval for a changed runtime.

## Control and safety boundaries

Keep real-time servo/torque processing and final safety authority local to native
runtime code. Python, policy services and LeRobot adapters are non-real-time;
they do not replace native watchdogs or safety checks. Preserve explicit command
ownership/leases, timestamps/deadlines, accepted/applied/rejected receipts and
latched faults. Do not invent a universal safe-stop fallback.

Default work is offline/read-only/mock/shadow. Hardware connections, motion,
recovery, calibration or guard changes require explicit separate authorization
and reviewed per-unit commissioning. CI must never connect to or actuate a robot.
Keep physical E-stop/operator procedures separate from software test assertions.

## Development, artifacts and current status

Use an isolated, version-pinned environment on each host; never assume a parent
workspace, sibling checkout, interpreter path or copied venv. Configure deployment
paths/connectivity externally. Do not globally source ROS/CUDA paths or install
native dependencies into a simulator/ML environment as a shortcut.

```bash
python -m pytest tests --import-mode=importlib
```

Pure/offline tests must run without hardware. Keep large logs, recordings, native
builds and deployable binary artifacts outside Git; retain configuration, source,
lightweight fixtures and dependency/qualification manifests. Check explicit output
locations for availability/capacity. Buffer logging outside the native callback.
This scaffold provides no working driver, validated stop path or motion approval.
