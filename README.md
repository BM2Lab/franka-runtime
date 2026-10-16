# franka-runtime

Physical Franka runtime infrastructure with extracted offline M4 and PPO slices.
The base package depends on lightweight `contracts`; optional `m4` adds
NumPy/PyYAML. Importing the base package never loads a vendor driver or connects
to hardware. Native controllers remain in the separately pinned Franky fork.

[Offline M4 runtime, commands and boundaries](docs/m4-offline-runtime.md).
[Offline PPO client and learning integration](docs/ppo-offline-runtime.md).
[Migration plan](docs/franka-rl-deployment-migration.md).
[Source inventory and phase 0–1 receipts](deployment/migration/README.md).

Install local version-pinned contracts/runtime wheels in a project-isolated
environment. Use the `m4,dev` extras for the full offline test suite:

```bash
python -m pytest tests --import-mode=importlib
```

Physical status: **NOT QUALIFIED**. Phases 2–3 expose preflight/mock/offline-shadow
and historical replay only. There is no FCI connection or hardware execution CLI.
Preserve per-unit model/tool/firmware checks, native guards and stop semantics;
historical deployment approval does not approve this extracted runtime.
