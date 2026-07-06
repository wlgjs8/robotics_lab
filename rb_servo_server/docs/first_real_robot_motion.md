# First Real Robot Motion

Real motion is not enabled by default. This checklist is a gate for a supervised
joint-only acceptance run after read-only state publishing has been verified.

## Required Gates

- Operator present with physical E-stop reachable.
- `RB_SERVO_ENABLE_RBPODO=ON` build completed locally.
- Hardware-free CMake gate completed immediately before the run.
- Real read-only run completed first with `servo.send_servo_commands=false`.
- Real motion is config-driven, not env-gated (the legacy `RB_ALLOW_REAL_*` env
  gates were removed from the server runtime): the site-local real config keeps
  `servo.send_servo_commands: false` for any read-only real connection and is set
  to `true` only for the first-motion run.
- Command and state endpoints remain loopback unless separately reviewed.
- Cartesian/TCP and force control remain disabled.
- `RbpodoBackend::initialize()` is read-only: it must not enter operation mode
  or set speed bar during the read-only run.
- Read-only state publishing can be healthy with `servo_enabled=false` when
  `q_actual` is valid. Treat that as observation-only readiness, not permission
  to send `servo_j`.
- The real example keeps `disable_waiting_ack=false`, so rbpodo command sends
  wait for controller ACK by default. Only set it true in a site-local motion
  config after supervised acceptance of the command path.
- `RbpodoBackend::stop()` and `resetFault()` are not verified controller-level
  real-robot recovery APIs. They return `rbpodo_stop_unverified` and
  `rbpodo_reset_fault_unverified`; use the physical E-stop and operator
  procedure as the actual stop/recovery path until a verified API is added.

## Config

Start from `rb_servo_server/config/stack_real.yaml` and copy it into
`rb_servo_server/config/local/` with a site-specific name such as
`stack_real_readonly.yaml` or `stack_real_motion.yaml`. Files in `config/local/`
are gitignored.

For read-only acceptance:

```yaml
servo:
  send_servo_commands: false
```

During this run, accepted evidence is valid published joint state plus
`servo_enabled=false` or another non-motion-ready lifecycle. It must not include
motion mode entry, speed-bar setup, or `servo_j`.

For the first motion attempt, change only the local copy:

```yaml
servo:
  send_servo_commands: true
```

## First Motion Shape

Use joint-only motion. The first target must be no more than 1 degree from the
current measured joint position on one low-risk joint. Stop immediately on any
unexpected direction, tracking error, send failure, stale state, or controller
warning.

`ResetFault` must not resume motion by itself. The rbpodo backend currently
fails reset closed until a verified fault-reset API exists. After any external
operator reset, return to `ConnectedHold` and require a new `ArmMotion`.
