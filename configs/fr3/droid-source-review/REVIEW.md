# DROID commissioning review

This is a non-executable review draft. No robot was accessed and no approval was generated.

86 fields have sourced candidate values; 18 fields remain unset.

DROID gains have been populated as unapproved candidate values from the pinned implementation; see per-field provenance and controller.reference.json.

| Section | Unset fields | Required evidence / decision |
|---|---|---|
| robot | arm_serial, control_serial, identity_max_age_ns | Current unit identity/Desk + trusted FCI evidence; operator collision thresholds |
| tool | F_T_EE, m_ee, I_ee, F_x_Cee, m_load, I_load, F_x_Cload | Measured/configured Hand, payload, COM, inertia and TCP |
| artifacts | collision_model, collision_model_sha256 | Vetted self-collision model/engine; software pin alone does not supply one |
| approval | approved_by, approved_at, physical_stop_procedure, workspace_review, hand_review, collision_review | Operator review, never auto-generated |

A partial draft is not a validated physical configuration. Unknown values stay null.
Evidence import checks source hashes and field provenance; it never writes approval flags.
Old archived robot state must not be restamped as a fresh identity observation.
The cross-host mock test can proceed independently of physical commissioning approval.
