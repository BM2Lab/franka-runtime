# DROID commissioning review before cross-host timing

The offline review generator collects known software bindings and identifies the
remaining commissioning inputs. It does not connect to a robot, load Franky, open
a listener, choose physical limits or create approval. All output configurations
are pending, non-executable and unauthorized for motion.

## Generated package on this host

`~/yifan/droid-deployment/commissioning/2026-10-24-review/`

- `REVIEW.md`: fields grouped by the evidence or decision needed.
- `physical.review.pending.json`: candidate configuration, with seven sourced
  software fields prefilled and all unknown physical fields left null.
- `provenance.json`: input hashes, per-field sources/basis, and unresolved fields.
- `contract.json`, `native-pin.json`, `controller-audit.json`: copied inputs.
- `controller.reference.json`: audited DROID gain values for review, not applied.
- `field-evidence.pending.json`: empty template for sourced operator decisions.
- `identity.pending.json`: empty identity-capture schema, without a generated
  capture time, model, firmware, serial or approval.

Prefilled values are the contract's logical robot label, contract hash/capability,
Hand closure normalization width, implemented controller kind and copied native
pin/hash. A logical robot label is not physical identity. The contract's open
width still needs to match the real Hand; it is not a measurement or homing result.
The copied pin hash covers the actual copied JSON bytes; input provenance keeps
its original source hash separately.

The upstream gain reference is Kq=[40,30,50,25,35,25,10], Dq=[4,6,5,5,3,2,1],
Kx=[400,400,400,15,15,15], Dx=[37,37,37,2,2,2], taken from the pinned offline
feedback-audit fixture. The generator checks consistency across audit cases and
records the fixture's hash. These remain suggestions and do not become physical
settings merely because the algebraic audit passed.

## Reproduce a review

```bash
~/yifan/droid-deployment/run-native-python -m franka_runtime.droid.commissioning \
  --contract /home/chen-lab/franka-runtime/configs/fr3/droid_fr3_jointpos_v1.contract.json \
  --native-pin /home/chen-lab/franka-runtime/src/franka_runtime/droid/droid-server10.candidate.json \
  --audit-fixture /home/chen-lab/franka-runtime/tests/fixtures/droid/hybrid-feedback.json \
  --output-dir /absolute/new/review-directory
```

The parent must exist and the output directory must be new; the generator never
overwrites an existing review. It copies source inputs and records hashes, without
installing software or modifying the canonical pending template.

To incorporate reviewed measurements/decisions, pass `--evidence evidence.json`.
This is a provenance import into a **draft**, never an approval operation:

```json
{
  "schema_version": 1,
  "entries": [
    {
      "field": "controller.joint_stiffness",
      "value": [40, 30, 50, 25, 35, 25, 10],
      "source": "operator-notes.md",
      "sha256": "<actual SHA256 of operator-notes.md>",
      "basis": "operator decision recorded in the referenced note; pending commissioning"
    }
  ]
}
```

Source paths resolve relative to the evidence file. Changed source hashes,
duplicate fields, unknown fields, approval/top-level flag changes, conflicting
software bindings and another FCI family are rejected. Complete physical schema,
artifact and admission checks still apply later; partial draft generation does not
claim that its settings are validated. File hashes establish provenance, not
measurement accuracy or physical approval.

## What is needed from the lab

1. **Unit/tool identity:** a fresh, trusted record of the connected arm's model,
   firmware/FCI and arm/control serials, plus actual Hand/load/TCP configuration.
   Do not restamp archived observations or substitute the previous arm's approval.
2. **Geometry:** this unit's exported URDF, reviewed Hand geometry, measured table/
   workspace coordinates, and a vetted self-collision engine/model. Provider
   binding interfaces exist; a placeholder or hash alone does not supply geometry.
3. **Operator decisions:** explicit gains/processing, position/target/measured
   guards, collision thresholds, Hand settings, storage budget and stop procedure.
   Upstream reference values and old PPO trials are evidence, not automatic approval.
4. **Timing:** cross-host mock measurements should inform physical timing/clock
   margins. Those fields intentionally remain undecided before measurements.

Physical commissioning is **not a prerequisite for the cross-host mock timing
experiment**, which uses its separate mock configuration and cannot command the
robot. This review can be completed incrementally; it does not introduce another
approval gate for that mock test. No cross-host timing or hardware operation was
run by this preparation step.

Installed-wheel validation: **225 passed, 2 skipped** (the pinned upstream
contracts omit the local M4/PPO modules). Tests cover provenance, copied-pin
hashes, conflicting evidence, source changes, gain consistency, refusal to write
approval and refusal to overwrite a review. Passive imports and `pip check` pass.
The candidate and review hashes are retained in
`~/yifan/droid-deployment/validation/step8.json`, with reconstruction inputs in
`step8-requirements.lock` and `artifacts/step8-wheelhouse/`. Earlier runtime
artifacts and all native packages/baselines remain unchanged.


## Sourced candidate and inference-workstation handoff, 2026-10-24

The operator requested population from archived hardware pipelines and DROID.
The resulting non-executable draft has **86 populated fields and 18 unresolved
unit/tool/collision/approval fields**. See
[inference workstation handoff](droid-inference-workstation-handoff.md) and
`configs/fr3/droid-source-review/`. Per-field source hashes and retained source
copies distinguish completed M4 evidence, pinned DROID defaults, mock baselines
and engineering proposals. The prior all-null review remains historical.
No robot/network operation or new motion approval occurred.
