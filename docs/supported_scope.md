# Supported Scope

The active real-controller backend is `rbpodo` only. `mock` and `simulator`
remain hardware-free validation surfaces.

Supported robot command/control defaults are 500 Hz:

- `servo.rate_hz: 500`
- `servo_t1_sec: 0.002`
- policy-runner `command_rate_hz: 500`

Supported tracked rbpodo real templates preserve raw controller joint degrees
and use explicit per-joint safety limits for the current controller soft-limit
configuration:

- `q_min_deg: [-360, -360, -360, -360, -360, -360]`
- `q_max_deg: [360, 360, 360, 360, 360, 360]`

Do not normalize raw control, safety, tracking, q-ref, state JSON, or log values
to `[-180, 180]`. See `docs/joint_range_policy.md`.

Manual non-500 YAML overrides may remain parseable for compatibility, but they
are not supported profiles and must not be documented as runnable defaults.

Unsupported raw script TCP comparison paths were removed from the active code,
config, gate, test, and runbook surface. Do not reintroduce direct raw script
controller command paths without a new accepted safety plan.

This scope does not change unrelated rates such as state publication,
recording, camera, simulator update, or timeout settings.
