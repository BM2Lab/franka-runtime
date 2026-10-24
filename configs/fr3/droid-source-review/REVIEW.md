# DROID commissioning review

This is a non-executable review draft. No robot was accessed and no approval was generated.

98 fields have sourced candidate values; 6 fields remain unset.

DROID gains have been populated as unapproved candidate values from the pinned implementation; see per-field provenance and controller.reference.json.

| Section | Unset fields | Required evidence / decision |
|---|---|---|
| approval | approved_by, approved_at, physical_stop_procedure, workspace_review, hand_review, collision_review | Operator review, never auto-generated |

A partial draft is not a validated physical configuration. Unknown values stay null.
Evidence import checks source hashes and field provenance; it never writes approval flags.
Old archived robot state must not be restamped as a fresh identity observation.
The cross-host mock test can proceed independently of physical commissioning approval.
