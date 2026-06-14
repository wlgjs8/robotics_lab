# Supported Scope

The active real-controller backend is `rbpodo` only. `mock` and `simulator`
remain hardware-free validation surfaces.

The repository is in **rbpodo pgmode-real physical bring-up**. Real motion is a
gated, operator-supervised lane that has carried a dual-arm physical Cartesian
circle (`docs/runbooks/rbpodo_real_physical_circle.md`); the conservative
promotion ladder is `docs/runbooks/pgmode_real_transition.md`. Real motion stays
fail-closed: env gates (`RB_ALLOW_REAL_ROBOT` / `RB_ALLOW_REAL_MOTION` /
`RB_ALLOW_REAL_CARTESIAN`, plus `RB_ALLOW_RBPODO_SUSPECT_DIAGNOSTICS_REAL_MOTION`
for the `-2001` carve-out) are still required and necessary-but-not-sufficient.
The policy-side `SafetyGate` real-Cartesian block was relaxed (PR #13), so for
real motion `rb_servo_server` is the sole safety layer. A full `flow-infer`
`real_policy` closed-loop rollout (pi0.5/openpi, `TcpTwistLocal` + real gripper via
the Pika Gripper Backend) and the async URDF-mesh `CollisionMonitor` have run on the
physical robot; the `real_policy` rollout-mode gate stays fully enforced and was
satisfied via accepted/validated config. Remaining gaps: policy task success
(model-side, not runtime) and force control (still `provider: null`,
`enable: false`). Measured hand-eye calibration is unneeded for the deployed pika
ee_local image-conditioned policy but still required for general geometry-dependent
policy.

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
