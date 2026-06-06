# Joint Range Policy

This document defines the supported joint-angle range policy for the rbpodo-only
500 Hz control stack.

## Raw Representation

`rbpodo` state and command values are raw controller joint angles in degrees.
The server must preserve raw values for:

- `q_actual_deg`
- `q_target_deg`
- `q_ref_deg`
- `q_sent_deg`
- servo logs and state JSON

Control, safety filtering, tracking comparisons, and q-ref comparisons must not
normalize these values to `[-180, 180]`. A UI may add a display-only normalized
view, but raw values must remain visible and must remain the source of truth.

## Supported Rbpodo Range

The current controller soft-limit configuration for the supported real rbpodo
scope is represented as explicit per-joint raw limits:

```yaml
safety:
  q_min_deg: [-360, -360, -360, -360, -360, -360]
  q_max_deg: [360, 360, 360, 360, 360, 360]
```

Tracked `dual_real*.yaml` rbpodo templates must carry these arrays explicitly.
Site-owned configs under `rb_servo_server/config/local/` may override them only
after confirming the active controller soft limits and the physical setup.

`[-180, 180]` is not a supported production rbpodo default. It may appear only
in tests or diagnostic fixtures that intentionally prove range-violation
behavior.

## Wrapping

`joint_wrap_for_startup_validation` is diagnostic/read-only only. It may publish
`q_range_wrapped` and `q_actual_normalized_for_safety_deg`, but state JSON must
still preserve raw `q_actual_deg`.

`joint_wrap_for_motion_safety` remains rejected. Motion targets are not wrapped
across `+/-180` or any other period; they clamp to configured raw
`q_min_deg`/`q_max_deg` until continuous motion-safe unwrapping is implemented
and accepted.

## Kinematics Alignment

The current `rb3_730e.urdf` model has limits close to `+/-360 deg` for most
joints and approximately `+/-150 deg` for `elbow_joint`. When rbpodo configs
enable kinematics and use broader controller raw safety limits, the config
loader emits a warning that IK may still be model-limited.

This warning is intentional. It prevents silent confusion between:

- raw controller safety limits used for state, command, tracking, and logging
- URDF/Pinocchio model limits used by IK

Do not treat the warning as physical acceptance. Real motion still requires the
normal real robot gates and separate acceptance evidence.
