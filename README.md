# franka-runtime

Offline-first runtime scaffold. Install with ../robot-learning/packages/contracts,
not the learning package. Importing this package never connects to hardware.

Extract FR3/Panda code only after source/dependency inventory. Preserve native 1 kHz guards, stop behavior, model/tool checks and reference timing. Qualify each model and unit independently.

No backend, controller, stop path or physical deployment is implemented or
approved. The workstation supports pure/offline tests only. Receiving control
hosts maintain separate pinned environments; do not copy this machine's venv.
