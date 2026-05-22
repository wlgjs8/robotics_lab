# First Real Robot Motion

Real motion is not enabled by default. This checklist is a gate for a supervised
joint-only acceptance run after read-only state publishing has been verified.

## Required Gates

- Operator present with physical E-stop reachable.
- `RB_SERVO_ENABLE_RBPODO=ON` build completed locally.
- Hardware-free CMake gate completed immediately before the run.
- Real read-only run completed first with `servo.send_servo_commands=false`.
- `RB_ALLOW_REAL_ROBOT=1` set for any real connection.
- `RB_ALLOW_REAL_MOTION=1` set only for the first-motion run.
- Command and state endpoints remain loopback unless separately reviewed.
- Cartesian/TCP and force control remain disabled.

## Config

Start from `rb_servo_server/config/dual_real.example.yaml` and copy it into
`rb_servo_server/config/local/`. Files in `config/local/` are gitignored.

For read-only acceptance:

```yaml
servo:
  send_servo_commands: false
```

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

`ResetFault` must not resume motion by itself. After any reset, return to
`ConnectedHold` and require a new `ArmMotion`.
