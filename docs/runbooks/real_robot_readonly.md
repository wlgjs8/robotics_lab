# Real Robot Read-Only Runbook

This runbook is the first real-controller stage. It allows state acquisition
only. It does not approve Servo J motion, Cartesian motion, `rt_script`, force
control, or collision-threshold changes.

**ACKON500 PASS is controller-reference lower-bound evidence, not physical TCP tracking.**

Read-only evidence is required after controller `pgmode` simulation and before
tiny physical acceptance. It must not be interpreted as motion readiness by
itself.

Transition ladder:

1. Controller pgmode simulation repeatability
2. Right arm
3. Dual arm
4. P0 diagnostics root cause
5. Real controller read-only
6. Tiny physical acceptance
7. Slow physical circle
8. Fast physical circle only after approval

## Scope

Use this before any rbpodo motion acceptance:

1. Copy a tracked real template to `rb_servo_server/config/local/`.
2. Keep `servo.send_servo_commands: false`.
3. Confirm both arms use `backend_type: rbpodo` and `run_mode: real`.
4. Start with `RB_ALLOW_REAL_ROBOT=1` only.
5. Verify valid state, low state age, no fault latch, and expected backend.

Do not set:

```bash
RB_ALLOW_REAL_MOTION=1
RB_ALLOW_REAL_CARTESIAN=1
RB_ALLOW_RBPODO_ACK_DISABLED_MOTION=1
```

Those gates belong to later supervised acceptance stages, not read-only.

## Config Rules

`rb_servo_server/config/dual_real.example.yaml` is a template, not a
ready-to-run real motion config. Local real configs live under:

```text
rb_servo_server/config/local/
```

For rbpodo Servo J parameters, use canonical fields only:

- `servo_t1_sec` -> Rainbow `move_servo_j` `t1`
- `servo_t2_sec` -> Rainbow `move_servo_j` `t2`
- `servo_gain` -> `gain`
- `servo_alpha` -> `alpha`

Do not use deprecated aliases in new configs:

- `servo_time_sec`
- `servo_lookahead_sec`
- `servo_acc`

Servo J range checks in config validation (see
`docs/servo_backend_contract.md` → "Rbpodo Servo J Parameters"):

- `servo_t1_sec >= 0.002` (refused otherwise; real motion must match `1 / servo.rate_hz`)
- `0.02 < servo_t2_sec < 0.2` (vendor-recommended; outside → WARN only)
- `servo_gain > 0`
- `0 < servo_alpha <= 10` — **script-level** units. The controller scales
  `gain`/`alpha` by `0.1` internally, so effective `0 < alpha < 1` maps to
  script-level `0 < servo_alpha <= 10`; `servo_alpha: 10.0` = effective `1.0` =
  inner LPF OFF (the server-side SMD owns smoothing).

For later motion configs, `servo_t1_sec` must match the supported command
period:

- 500 Hz -> `servo_t1_sec: 0.002`

## Read-Only Command

Example for the left arm 500 Hz ACK-on profile:

```bash
RB_ALLOW_REAL_ROBOT=1 \
python3 scripts/rbpodo_servo_acceptance.py \
  --config rb_servo_server/config/local/dual_real_500hz_ack.yaml \
  --arm left \
  --mode read_only \
  --profile 500hz_ack \
  --duration-sec 10 \
  --artifact-dir artifacts/rbpodo_acceptance/500hz_ack_read_only_left \
  --i-understand-this-connects-to-real-controller
```

## Evidence To Review

Record these fields before moving to any no-op or motion stage:

- `observed_backend`
- `state_valid_ratio`
- `state_age_us`
- `fault_latched`
- `error_code`
- `q_actual`
- M561/M568/M569/M570 if available
- absence of Servo J send attempts

ACK-off profiles may be inspected in read-only mode, but ACK-off motion is not
approved here. ACK-off success means socket/API send evidence only; controller
acceptance is not observed unless ACK waiting is enabled.
