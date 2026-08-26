# Design Review Fixes Applied in v3

> Historical implementation record. Its milestone/pending statements are not
> current runtime guidance. Use `rb_servo_server/README.md`,
> `docs/architecture.md`, `docs/control_loop.md`, and
> `docs/fail_safe_policy.md`; force-control v2 is now live.

This document records concrete fixes after the v1/v2 design reviews.

## v2 fixes retained

- `CommandServer` receives UDP JSON and writes to `CommandBuffer`.
- C++ receive timestamp is authoritative for timeout checks.
- `loadConfigFromYaml()` reads the provided simple YAML files.
- `period_ms` and `jitter_ms` are measured from actual loop timing.
- Hold uses the previous sent target instead of chasing `q_actual` every tick.
- Tracking-error safety is checked with `abs(previous_sent_q - q_actual)`.
- Force-control types/config/interfaces are present but not wired into the active joint-only path.

## New v3 fixes

### 1. Safety verdicts replace a single bool

`SafetyFilter` now returns `SafetyCheckResult` with a `SafetyVerdict`:

- `Ok`
- `JointLimitClamped`
- `TrackingError`
- `RobotStateError`
- `EmergencyStop`
- `FaultLatched`
- `InvalidCommand`
- `CartesianUnavailable`

This makes tracking error, robot-state error, invalid command, and Cartesian-not-ready cases distinguishable.

### 2. Tracking-error policy is explicit

`SafetyConfig` includes:

```yaml
tracking_error_policy: snap_to_actual  # mock/simulator default
# or
tracking_error_policy: fault_latch     # real default
```

- `snap_to_actual`: set the servo target to current actual q and continue.
  Useful for mock/simulator iteration.
- `fault_latch`: latch a fault hold pose and ignore motion commands until `ResetFault`. Safer for real hardware.

### 3. Fault latch path added

`EmergencyStop`, real tracking errors, and robot-state errors can latch a fault:

```text
fault → hold current actual pose if available, otherwise last safe sent target
```

While latched, non-reset motion commands are ignored.

Reset is available through:

```json
{"seq": 1, "mode": "ResetFault"}
```

or:

```bash
python3 tools/send_reset_fault.py
```

### 4. Missing command payloads cannot become zero targets

A packet like this is now converted to Hold:

```json
{"mode": "JointTarget", "left": {}, "right": {}}
```

The parser sets payload flags for the retained motion primitives, and the loop refuses to execute a mode whose required payload is absent.

### 5. Cartesian failure means Hold, not zero

`TcpPoseTarget` and `TcpLinearMove` now have active configured Cartesian paths.
They return `CartesianUnavailable` and hold the previous safe target when
kinematics, config, input state, or IK solving is unavailable. This prevents the
Cartesian layer from accidentally defaulting to zero joint arrays.

The rule remains:

```text
IK success → q_target
IK failure → previous safe target or fault latch, never zeros
```

### 6. Filter dt is capped

The logger still records actual `period_ms`, but trajectory/safety filters use a capped `filter_dt_ms`:

```yaml
filter_dt_min_ratio: 0.5
filter_dt_max_ratio: 1.5
```

At 500 Hz this means filter dt is constrained to 1.0-3.0 ms by default. One late OS tick no longer permits a proportionally large joint step.

### 7. Acceleration clamp no longer overshoots the velocity-limited target

The acceleration limiter now prevents overshoot past the already velocity-limited target. This avoids direction-change artifacts where `q_sent` could exceed the intended command range.

## Still pending from this v3 review

- lock-free or priority-inheritance command buffer
- optional Ruckig or jerk-limited interpolation beyond the current filters
- production JSON parser such as `nlohmann/json`
