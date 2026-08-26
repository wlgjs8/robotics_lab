# Real Robot Read-Only Runbook

This is the first physical-controller evidence stage. It connects to the real
rbpodo endpoints and publishes state, but it must not send Servo J, Cartesian,
gripper, or force-motion commands. It is not motion readiness by itself.

## Config preparation

Use `rb_servo_server/config/stack_real.yaml` directly. Do not make a
`config/local` copy.

With the stack stopped, change only:

```yaml
servo:
  send_servo_commands: false
```

Leave the tracked Cartesian and ACK settings unchanged: the server-wide send
gate is closed, so neither setting is exercised or accepted by this result.
Retain the canonical Servo J fields, and do not change force calibration/law
values. Record the config diff with the artifact.

J3 must remain exactly `[-150 deg, +150 deg]`:

```yaml
safety:
  q_min_deg: [-360, -360, -150, -360, -360, -360]
  q_max_deg: [360, 360, 150, 360, 360, 360]
```

Preflight without opening controller sockets:

```bash
rb_servo_server/build/rbpodo_real_gate/rb_servo_server \
  --check-config --config rb_servo_server/config/stack_real.yaml
```

Use the freshly built binary path when it differs.

## Operator preflight

- Both endpoint/IP assignments match the cell.
- Physical controllers and pendant state are known.
- Workspace is clear and the E-stop is immediately available even though no
  motion is expected.
- `servo.send_servo_commands` is visibly `false` in the recorded diff.
- No gripper/policy process with independent hardware authority is started.

## Collection

The supervised acceptance helper may start the server against the tracked
config after its normal real-controller confirmation:

```bash
python3 scripts/rbpodo_servo_acceptance.py \
  --config rb_servo_server/config/stack_real.yaml \
  --arm left \
  --mode read_only \
  --profile 500hz_ack \
  --duration-sec 10 \
  --artifact-dir artifacts/rbpodo_acceptance/500hz_ack_read_only_left \
  --i-understand-this-connects-to-real-controller
```

Repeat the reviewed stage for the other arm and then both arms as required by
the acceptance plan. Do not convert the same run into motion by editing the
config while the server is live.

## Pass evidence

- `observed_backend` is rbpodo and operation mode is physical real.
- `q_actual_deg`/`q_ref_deg` are finite and raw; J3 stays within ±150°.
- State age and update rate satisfy the reviewed budget.
- Startup/fault/diagnostic interpretation is explicit, including the accepted
  `-2001` unavailable-field policy where configured.
- `send_policy` is read-only and no Servo J send attempt occurred.
- No physical or gripper motion occurred.
- F/T telemetry may be observed, but an untared or read-only arm is not evidence
  of accepted force motion.

ACK-off success is never controller-ACK evidence. Do not change ACK policy in a
read-only stage.

After collection, stop the server, restore the reviewed tracked setting
explicitly, and verify the final `git diff`. Do not use a broad checkout/reset
operation that could discard unrelated user work.
