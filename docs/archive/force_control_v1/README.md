# Archived: project-native force control (v1)

**Audit only. Nothing here describes the running system.**

The v1 force stack was removed from `robotics_lab` on 2026-08-26 so it could be
rebuilt against `controller-manager` as the reference, starting from sensor and
tool setup. What went with it:

- the server's F/T pipeline, contact guard, normal-admittance and 6D Cartesian
  compliance paths, and the `ExternalForceLimit` reflex
- the `force_torque:` / `force_control:` config sections and their validation
- the `eft_*` hardware read (rbpodo `sdata.eft_fx..eft_mz`)
- every force telemetry column and state-JSON block
- the rb_gui F/T monitor, overlays and payload-identification session
- the policy_runner `force_recovery` gate

These two pages are kept because they carry hardware-measured evidence that is
expensive to reproduce and that the rebuild will want to compare against:

- `force_control.md` — the v1 schema, the 2026-07-12 positive X/Y/Z frame
  capture and roll/pitch/yaw direction/recenter captures, and the 2026-07-24
  translation-only reflex profile.
- `guarded_contact_design.md` — the `surface_source: contact_force` episode
  state machine (implemented, never activated on the tracked real stack).

Numbers here were measured against the v1 sensor/tool setup. Re-derive rather
than port them: the CM-referenced rebuild starts from its own frame and tare
definitions, and a value that silently disagrees is worse than no value.
