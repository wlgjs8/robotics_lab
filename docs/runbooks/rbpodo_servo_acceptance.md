# rbpodo Servo J Acceptance Runbook

This runbook covers supervised rbpodo Servo J rate and ACK acceptance. It is
not a real Cartesian acceptance runbook and it does not approve unattended real
motion.

## Purpose

This runbook is for `RBPODO-ACCEPT-01`. It documents how to collect supervised
evidence for the primary vendor-library real backend. It does not authorize
real Cartesian motion, `rt_script`, collision-threshold changes, or unattended
robot motion.

The supported rbpodo profile is:

| Profile | Template | Rate | `servo_t1_sec` | ACK policy | Command/state endpoints | Default motion |
| --- | --- | ---: | ---: | --- | --- | --- |
| `500hz_ack` | `rb_servo_server/config/dual_real.example.yaml` | 500 Hz | 0.002 s | ACK-on | site-local | disabled |

`servo_t1_sec` must match the supported command period: 0.002 s at 500 Hz.

## Servo J Parameters

Rbpodo sends Rainbow UI Script `move_servo_j(jnt, t1, t2, gain, alpha)`.
The config mapping is:

| Config field | Rainbow field | Meaning |
| --- | --- | --- |
| `servo_t1_sec` | `t1` | command arrival/time parameter; match streaming period |
| `servo_t2_sec` | `t2` | controller hold time, not UR-style lookahead |
| `servo_gain` | `gain` | Servo J gain (0.1-scaled inside controller, see below) |
| `servo_alpha` | `alpha` | low-pass-filter gain, not acceleration (0.1-scaled inside controller, see below) |

**0.1 internal scaling (vendor-confirmed):** Rainbow scales `gain` and `alpha`
by 0.1 INSIDE the controller, so the script-level value we send via
`move_servo_j` is 10x the effective value. Therefore `servo_alpha: 10` →
effective `1.0` = **LPF off** (Rainbow's internal low-pass disabled so the
`rb_servo_server` SMD owns all smoothing). Do NOT lower `servo_alpha` back into
`(0, 1)` thinking it is out of range — that drives the effective alpha down to
~0.1 and turns the controller LPF strongly back on. The same 10x convention
applies to `servo_gain`.

Official validation ranges (effective vendor range in parentheses; config /
script-level values use the 10x convention for gain/alpha):

- `servo_t1_sec >= 0.002`
- `0.02 < servo_t2_sec < 0.2`
- `servo_gain > 0` (script-level; effective = `servo_gain * 0.1`)
- `0 < servo_alpha <= 10` (script-level; effective `0 < alpha <= 1`, so
  `servo_alpha: 10` = effective `1.0` = LPF off)

Do not use `servo_acc`; use `servo_alpha`. Do not use
`servo_lookahead_sec`; use `servo_t2_sec`. Old aliases are deprecated and
exist only for migration.

For real/controller-simulation configs, a rate mismatch should fail unless an
explicit acceptance task allows it:

- 500 Hz -> `servo_t1_sec: 0.002`

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

Rbpodo controller `pgmode` simulation Servo J commands additionally require:

```bash
RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1
```

The acceptance tool sets `RB_RBPODO_PGMODE_SIMULATION_CONFIRMED=1` only for the
server process after the same run successfully sends or verifies controller
`pgmode` simulation. Do not set this variable by hand for ordinary runs.

The temporary diagnostics-suspect bridge for controller simulation additionally
requires both the YAML opt-in
`servo.allow_controller_simulation_diagnostics_suspect: true` and:

```bash
RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1
```

This is not real motion approval. It is limited to `operation_mode:
simulation`, known rbpodo suspicious status layouts, finite joint state, no
range violation, no explicit E-stop/SOS/soft E-stop/collision fault, and a
confirmed controller `pgmode` simulation run.

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
   It sends a target equal to current `q_actual_deg`. The same tool run must
   use `--set-pgmode-simulation` or `--verify-pgmode-simulation`, and the config
   must set `operation_mode: simulation`.
4. 500 Hz ACK-on no-op / tiny simulation-mode evidence.
5. ACK-off diagnostics, if explicitly requested, are not supported motion
   profiles and must not be treated as controller-ACK acceptance.
6. `tiny_joint_motion`: reserved for a future motion runbook. Do not run it
   from this runbook and do not treat this runbook as approval.

## Config Handling

`rb_servo_server/config/dual_real.example.yaml` is a template, not a
ready-to-run real motion config. Copy it to `rb_servo_server/config/local/`,
review the local values, and keep `send_servo_commands: false` for read-only
acceptance.
The `config/local` directory is user-owned; tracked local YAML samples are not
production configuration.

Example copy commands:

```bash
cp rb_servo_server/config/dual_real.example.yaml \
  rb_servo_server/config/local/dual_real_500hz_ack.yaml
```

For controller bring-up diagnostics, the ACK-on read-only examples enable:

```yaml
servo:
  send_servo_commands: false
  allow_readonly_faulted_startup: true
  allow_readonly_q_range_violation_startup: true
  allow_readonly_wrong_mode_startup: true
safety:
  joint_wrap_period_deg: [0, 0, 0, 360, 0, 360]
  joint_wrap_for_startup_validation: true
  joint_wrap_for_motion_safety: false
```

These flags only allow the server to publish state JSON after valid state
acquisition. They do not mark the robot as motion-ready, do not suppress fault
or range telemetry, and are refused for `send_servo_commands: true` motion
configs.

Startup joint wrapping exists only to interpret controller bring-up diagnostics.
Raw `q_actual_deg` remains the controller value. Inspect `q_range_wrapped` and
`q_actual_normalized_for_safety_deg` when a raw joint value is outside the
configured range but may be equivalent modulo a wrap period. A right-arm report
near `-317 deg` with range `[-190, 190]` and period `360` is equivalent to about
`43 deg` for startup range diagnostics. Motion target wrapping remains disabled
to avoid discontinuities.

Controller-simulation no-op acceptance is separate from the read-only template.
Use an artifact-local or site-local 500 Hz controller-simulation config only
after controller `pgmode` simulation has been verified by the acceptance tool
and the normal real-controller/motion env gates are set. The local copy must
explicitly set:

```yaml
servo:
  send_servo_commands: true
  allow_controller_simulation_motion: true
```

Set `allow_controller_simulation_diagnostics_suspect: true` only for the
temporary diagnostics-suspect bridge described above. These files intentionally
send Servo J to the controller box, but they are still not physical motion
acceptance configs and do not approve ACK-off or Cartesian motion.

`scripts/rainbow_pgmode.py` is the first-class pgmode helper. It sends only
`pgmode simulation` or reads `real_vs_simulation_mode`; it must not set
`pgmode real`, reset faults, change collision thresholds, enable servo power, or
send motion:

```bash
RB_ALLOW_REAL_ROBOT=1 \
python3 scripts/rainbow_pgmode.py \
  --ips 172.28.60.200 172.28.60.201 \
  --set-simulation \
  --summary-json artifacts/rbpodo_acceptance/pgmode_simulation.json \
  --i-understand-this-connects-to-real-controller
```

## State Dump Bring-Up Tool

Before starting the server, or after an acceptance startup timeout, capture a
read-only rbpodo dump:

```bash
python3 scripts/rbpodo_state_dump.py \
  --ips 172.28.60.200 172.28.60.201 \
  --q-min=-170,-120,-170,-190,-120,-360 \
  --q-max=170,120,170,190,120,360 \
  --wrap-period-deg=0,0,0,360,0,360 \
  --output artifacts/rbpodo_acceptance/state_dump.json \
  --pretty \
  --i-understand-this-connects-to-real-controller
```

The tool only reads `rbpodo.CobotData`. It must not be used to set `pgmode`,
reset faults, activate motion, or modify controller state. The JSON artifact
includes raw status fields, `q_actual_deg`, `q_ref_deg`, finite flags, range
violations, optional wrap diagnostics, `real_vs_simulation_mode`, controller
mode warnings, and recommended next steps.

If `rbpodo_servo_acceptance.py` times out waiting for the server state stream,
the failure message includes the server return code, the last 120 lines of
`rb_servo_server.log`, and likely causes. If the log shows
`invalid robot startup state`, run `scripts/rbpodo_state_dump.py` and compare
its diagnostics with the server startup validation.

## Read-Only Example

Use a local copy under `rb_servo_server/config/local/`, not a tracked template.

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

The local config must remain 500 Hz with `servo_t1_sec: 0.002` on both arms.

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
- inspect `rbpodo_diagnostics` for each arm; suspicious diagnostics block
  motion promotion even when read-only state acquisition succeeds
- inspect `q_range_wrapped`; it explains startup range normalization while raw
  `q_actual_deg` remains unchanged
- no `sendServoJ` acceptance in read-only mode

`rbpodo_diagnostics.raw` preserves firmware/SDK-layout-dependent fields such as
`time`, `real_vs_simulation_mode`, `init_state_info`, `init_error`,
`op_stat_sos_flag`, `op_stat_ems_flag`, `op_stat_soft_estop_occur`,
`op_stat_collision_occur`, and `op_stat_self_collision`. If a status flag that
is expected to be boolean appears as a large value, or if controller `time` is
non-finite/implausibly tiny, the state should show
`diagnostics_suspect: true`. That is useful bring-up evidence, not a
motion-ready condition.

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
