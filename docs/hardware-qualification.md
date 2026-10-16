# Hardware qualification

Status: **NOT QUALIFIED**. Phase 2 implements offline M4 integration and mock/replay
commands; it does not expose FCI connection or hardware execution. Native API tests
use a Python robot double; recorded-log replay is historical numerical evidence.

Before physical commissioning, record unit/model/serial, firmware/FCI, exact
native artifact hashes, tool/payload/COM/TCP, workspace, controller/guard/stop
semantics, authority clock/deadline behavior and separate operator authorization.
Historical FR3 approval does not approve this extraction, Panda, server6 or other
tools. See [M4 boundaries](m4-offline-runtime.md).
