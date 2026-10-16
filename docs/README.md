# docs

Reserved for franka-runtime implementation. No hardware execution or task qualification is provided by this scaffold.

## Migration planning

[Franka RL deployment migration plan](franka-rl-deployment-migration.md) maps the
existing implementation to the lab repositories, records source revisions, and
defines staged offline parity and separate hardware commissioning. It does not
change the scaffold qualification status.

[Phases 0–1 implementation and validation](../deployment/migration/README.md) records the source inventory, portable contracts, golden fixtures and offline-only scope.

[Extracted offline M4 runtime (phase 2)](m4-offline-runtime.md) — installed commands, authority/native boundary and validation.

[Installed PPO adapters/client (phase 3)](ppo-offline-runtime.md) — worker/artifact boundaries, reference semantics and offline validation.
