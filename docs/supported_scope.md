# Supported Scope

The active real-controller backend is `rbpodo` only. `mock` is the
hardware-free validation surface; the `rb_simulator` software-simulator
backend was removed.

The repository is in **rbpodo pgmode-real physical bring-up**. Real motion is an
operator-supervised lane that has carried a dual-arm physical Cartesian circle
(`docs/runbooks/rbpodo_real_physical_circle.md`); the promotion ladder is
`docs/runbooks/pgmode_real_transition.md`. Real-motion execution authority is
config-driven and server-owned, and is **no longer gated on env vars**: the legacy
`RB_ALLOW_REAL_*` execution gates were removed from the server runtime, and real
motion is now decided solely by site-local config + the mode-independent safety
layers (`rb_servo_server/config/local/`). The `-2001` suspect-diagnostics
acceptance is a per-arm config opt-in
(`allow_real_motion_with_suspect_diagnostics: true`, no env).
The policy-side `SafetyGate` real-Cartesian block was retired (PR #13); stale
state, fault, camera, and kinematics readiness checks remain. For real Cartesian
motion, `rb_servo_server` makes the final allow/deny decision. A full `flow-infer`
`real_policy` closed-loop rollout (pi0.5/openpi, `TcpPoseTarget` + real gripper via
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

- `q_min_deg: [-360, -360, -160, -360, -360, -360]`
- `q_max_deg: [360, 360, 160, 360, 360, 360]`

Do not normalize raw control, safety, tracking, q-ref, state JSON, or log values
to `[-180, 180]`. See `docs/joint_range_policy.md`.

Manual non-500 YAML overrides may remain parseable for compatibility, but they
are not supported profiles and must not be documented as runnable defaults.

Unsupported raw script TCP comparison paths were removed from the active code,
config, gate, test, and runbook surface. Do not reintroduce direct raw script
controller command paths without a new accepted safety plan.

This scope does not change unrelated rates such as state publication,
recording, camera, or timeout settings.
