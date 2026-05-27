# rbpodo Servo J Acceptance Runbook

This runbook covers supervised rbpodo Servo J rate and ACK acceptance. It is
not a real Cartesian acceptance runbook and it does not approve unattended real
motion.

## Purpose

This runbook is for `RBPODO-ACCEPT-01`. It documents how to collect supervised
evidence for the primary vendor-library real backend. It does not authorize
real Cartesian motion, `rt_script`, collision-threshold changes, or unattended
robot motion.

The current profiles are:

| Profile | Rate | `servo_t1_sec` | ACK policy | Default motion |
| --- | ---: | ---: | --- | --- |
| `100hz_ack` | 100 Hz | 0.01 s | ACK-on | disabled |
| `200hz_ack` | 200 Hz | 0.005 s | ACK-on | disabled |
| `200hz_no_ack` | 200 Hz | 0.005 s | ACK-off | disabled |

`servo_t1_sec` must match the command period. For 100 Hz the period is
0.01 s; for 200 Hz it is 0.005 s.

## Servo J Parameters

Rbpodo sends Rainbow UI Script `move_servo_j(jnt, t1, t2, gain, alpha)`.
The config mapping is:

| Config field | Rainbow field | Meaning |
| --- | --- | --- |
| `servo_t1_sec` | `t1` | command arrival/time parameter; match streaming period |
| `servo_t2_sec` | `t2` | controller hold time, not UR-style lookahead |
| `servo_gain` | `gain` | Servo J gain |
| `servo_alpha` | `alpha` | low-pass-filter gain, not acceleration |

Official validation ranges:

- `servo_t1_sec >= 0.002`
- `0.02 < servo_t2_sec < 0.2`
- `servo_gain > 0`
- `0 < servo_alpha < 1`

Do not use `servo_acc`; use `servo_alpha`. Do not use
`servo_lookahead_sec`; use `servo_t2_sec`. Old aliases are deprecated and
exist only for migration.

For real motion configs, a rate mismatch should fail unless an explicit
acceptance task allows it:

- 100 Hz -> `servo_t1_sec: 0.01`
- 200 Hz -> `servo_t1_sec: 0.005`

## ACK Semantics

ACK-on means `sendServoJ().accepted=true` is controller ACK evidence.

ACK-off means `sendServoJ().accepted=true` is socket/API send evidence only.
It does not prove immediate controller acceptance. The state stream and servo
log must be interpreted using:

- `ack_policy`
- `ack_observed`
- `controller_acceptance_observed`
- `send_acceptance_semantics`
- `ack_wait_duration_us`

ACK-off real motion additionally requires:

```bash
RB_ALLOW_RBPODO_ACK_DISABLED_MOTION=1
```

ACK-off settings are not a real baseline until ACK-off acceptance passes.
They require stronger monitoring because immediate controller rejection is not
observed.

## Environment Gates

Read-only real connection requires:

```bash
RB_ALLOW_REAL_ROBOT=1
```

Real joint Servo J motion requires:

```bash
RB_ALLOW_REAL_MOTION=1
```

ACK-off real joint Servo J motion additionally requires:

```bash
RB_ALLOW_RBPODO_ACK_DISABLED_MOTION=1
```

Real Cartesian/TCP motion is out of scope for this runbook and must remain
unset:

```bash
RB_ALLOW_REAL_CARTESIAN=1
```

## Required Staging

1. `read_only`: start `rb_servo_server` with `send_servo_commands: false`,
   collect state, and verify valid joint feedback, state age, fault state, and
   observed backend.
2. `hold_no_motion`: send `Hold` packets only. This is a command-channel smoke
   step and must not command joint deltas.
3. `servo_j_noop`: only for explicitly confirmed controller simulation mode.
   It sends a target equal to current `q_actual_deg`.
4. 100 Hz ACK-on no-op / tiny simulation-mode evidence.
5. 200 Hz ACK-on evidence.
6. 200 Hz ACK-off evidence, with `--allow-ack-disabled`.
7. `tiny_joint_motion`: reserved for a future motion runbook. Do not run it
   from this runbook and do not treat this runbook as approval.

## Config Handling

`rb_servo_server/config/dual_real.example.yaml` is a template, not a
ready-to-run real motion config. Copy one of the tracked examples to
`rb_servo_server/config/local/`, review the local values, and keep
`send_servo_commands: false` for read-only acceptance. The `config/local`
directory is user-owned and gitignored.

## Read-Only Example

Use a local copy under `rb_servo_server/config/local/`, not a tracked template.

```bash
RB_ALLOW_REAL_ROBOT=1 \
python3 scripts/rbpodo_servo_acceptance.py \
  --config rb_servo_server/config/local/dual_real_100hz_ack.yaml \
  --arm left \
  --mode read_only \
  --profile 100hz_ack \
  --duration-sec 10 \
  --artifact-dir artifacts/rbpodo_acceptance/100hz_ack_read_only_left \
  --i-understand-this-connects-to-real-controller
```

## 200 Hz ACK-On Read-Only

```bash
RB_ALLOW_REAL_ROBOT=1 \
python3 scripts/rbpodo_servo_acceptance.py \
  --config rb_servo_server/config/local/dual_real_200hz_ack.yaml \
  --arm left \
  --mode read_only \
  --profile 200hz_ack \
  --duration-sec 10 \
  --artifact-dir artifacts/rbpodo_acceptance/200hz_ack_read_only_left \
  --i-understand-this-connects-to-real-controller
```

## 200 Hz ACK-Off Read-Only

ACK-off still requires explicit acknowledgement even in read-only acceptance,
because the run is evaluating a profile where immediate command ACK is disabled.

```bash
RB_ALLOW_REAL_ROBOT=1 \
python3 scripts/rbpodo_servo_acceptance.py \
  --config rb_servo_server/config/local/dual_real_200hz_no_ack.yaml \
  --arm left \
  --mode read_only \
  --profile 200hz_no_ack \
  --allow-ack-disabled \
  --duration-sec 10 \
  --artifact-dir artifacts/rbpodo_acceptance/200hz_no_ack_read_only_left \
  --i-understand-this-connects-to-real-controller
```

## Artifacts

Each run writes:

- `summary.json`
- `summary.csv`
- `state_stream.jsonl`
- `command_packets.jsonl`
- `rb_servo_server.log`
- `servo_log.csv`, when produced by the server
- `timing_send_duration.png`
- `timing_state_age.png`
- `ack_policy.png`
- `q_actual_time.png`
- `error_code_counts.json`

## Interpreting Results

For read-only promotion, require:

- `observed_backend == "rbpodo"`
- valid joint state ratio near 1.0
- low state age
- `fault_latched == false`
- no unexpected controller `error_code`
- no `sendServoJ` acceptance in read-only mode

For ACK-on send tests, `controller_acceptance_observed_count` is the key count.
For ACK-off send tests, `send_success_count` must be treated as socket/API send
evidence only.

When observable, count Rainbow system codes such as M561/M568/M569/M570 and
record them in the artifact notes. Any recurring code is a blocker for profile
promotion until explained.

Record and compare at least:

- `send_duration_us`
- `ack_wait_duration_us`
- `ack_observed`
- `controller_acceptance_observed`
- `state_age_us`
- `error_code`
- M561/M568/M569/M570 when available
- `q_ref` / `q_actual`

## Safety Policy

Do not run real motion unattended. Do not use `rt_script`. Do not change
collision thresholds or undocumented controller settings. Do not set
`RB_ALLOW_REAL_CARTESIAN` for this runbook.
