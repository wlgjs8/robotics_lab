# rbpodo pgmode Simulation To Physical Real Transition

This runbook defines the conservative ladder from rbpodo controller
`pgmode` simulation evidence to physical `operation_mode: real` evidence.
It creates artifacts and refuses unsafe promotion paths; it is not approval to
run hardware by itself.

Do not use `tcp_ref_stand` as physical tracking evidence. Physical pass
evidence must use `tcp_actual_stand`.

## Ladder

| Stage | Name | Purpose |
| --- | --- | --- |
| P0 | `controller_sim_repeatability_done` | Reviewed controller `pgmode` repeatability artifact. |
| P1 | `real_readonly_diagnostics_parity` | Read-only physical diagnostics with no Servo J sends. |
| P2 | `stop_resetFault_or_operator_stop_policy_verified` | Verified stop/resetFault behavior or unresolved operator-stop policy recorded. |
| P3 | `real_hold_no_motion` | Hold/no-motion physical run; no motion expected or detected. |
| P4 | `tiny_joint_noop_or_tiny_joint_motion` | Tiny joint no-op or explicitly tiny supervised joint motion. |
| P5 | `tiny_cartesian_delta` | Tiny Cartesian delta with actual TCP state. |
| P6 | `slow_physical_circle_5cm_10s` | First slow physical circle. |
| P7 | `stable_physical_circle_15cm_16s` | Stable 15 cm / 16 s physical circle. |
| P8 | `medium_physical_circle_15cm_8s` | Medium 15 cm / 8 s physical circle. |
| P9 | `fast_physical_circle_15cm_4s_only_after_explicit_approval` | Fast 15 cm / 4 s only after explicit approval. |

P9 is not a default or automatic promotion target.

## Tools

Dry-run commands do not require environment gates and do not start hardware:

```bash
python3 scripts/rbpodo_physical_transition_acceptance.py --help

python3 scripts/rbpodo_physical_transition_acceptance.py \
  --stage read_only \
  --dry-run \
  --artifact-dir artifacts/rbpodo_physical_transition/dry_run_read_only

python3 scripts/rbpodo_physical_transition_acceptance.py \
  --stage tiny_joint \
  --dry-run \
  --artifact-dir artifacts/rbpodo_physical_transition/dry_run_tiny_joint

python3 scripts/rbpodo_physical_transition_acceptance.py \
  --stage tiny_cartesian \
  --dry-run \
  --artifact-dir artifacts/rbpodo_physical_transition/dry_run_tiny_cartesian

python3 scripts/generate_rbpodo_physical_transition_report.py \
  --artifact-dir artifacts/rbpodo_physical_transition
```

The acceptance script defaults to dry-run. `--execute` validates live gates and
writes a supervised preflight artifact, but the script itself does not launch
the servo server or send motion commands.

## Config Policy

Tracked physical transition templates are non-runnable for motion:

```text
rb_servo_server/config/dual_real_rbpodo_physical_readonly.example.yaml
rb_servo_server/config/dual_real_rbpodo_physical_tiny_joint.example.yaml
rb_servo_server/config/dual_real_rbpodo_physical_tiny_cartesian.example.yaml
```

Every tracked example must keep:

```yaml
servo:
  send_servo_commands: false
force_control:
  provider: null
  enable: false
cartesian_control:
  allow_in_real: false
```

Non-dry-run stages require a site-local config under:

```text
rb_servo_server/config/local/
```

The operator-owned local config must explicitly opt into the relevant stage.
Tracked templates that set `servo.send_servo_commands: true`,
`force_control.enable: true`, non-null `force_control.provider`, or
`cartesian_control.allow_in_real: true` are refused by the acceptance script.

## Required Gates

Read-only physical diagnostics:

```bash
RB_ALLOW_REAL_ROBOT=1
```

Real joint motion stages:

```bash
RB_ALLOW_REAL_ROBOT=1
RB_ALLOW_REAL_MOTION=1
```

Real Cartesian stages:

```bash
RB_ALLOW_REAL_ROBOT=1
RB_ALLOW_REAL_MOTION=1
RB_ALLOW_REAL_CARTESIAN=1
```

Motion stages also require:

```text
--i-understand-this-may-move-the-physical-robot
--i-have-clear-workspace
--i-have-estop-in-hand
--i-reviewed-local-config
--i-confirm-operator-supervision
```

P9 additionally requires:

```text
--i-have-explicit-p9-approval
```

Dry-run prints the exact gates and flags that would be required, but does not
require them.

## Refusal Cases

The acceptance script refuses non-dry-run stages when:

- `--config` is missing.
- The config is not under `rb_servo_server/config/local/`.
- Any arm is not `backend_type: rbpodo`, `run_mode: real`, and
  `operation_mode: real`.
- A read-only/no-motion stage has `servo.send_servo_commands: true`.
- A motion stage has `servo.send_servo_commands: false`.
- A Cartesian motion stage does not have site-local
  `cartesian_control.allow_in_real: true`.
- Required env gates or confirmation flags are missing.
- `force_control` is enabled or has a provider.
- P4 exceeds `--max-joint-delta-deg 0.05`.
- P5 or later exceeds `--max-cartesian-step-m 0.005`.
- P9 lacks explicit approval.

These checks are necessary but not sufficient for physical acceptance.

## Measurement Semantics

Physical real reports must use actual physical state:

```yaml
physical_tracking_result:
  status: pass|fail|not_run
  tracking_source: tcp_actual_stand
controller_reference_result:
  status: informational_only
  tracking_source: tcp_ref_stand
```

If an artifact claims `physical_tracking_result.status: pass` with
`tracking_source: tcp_ref_stand`, the report generator rejects it.

Reports must carry:

- state age and jitter
- `q_actual` and `q_ref` update rates
- fault latch status
- Cartesian availability
- stop/reset behavior result or unresolved status
- actual TCP tracking RMS, p95, and max error where applicable
- physical motion expected and detected fields
- calibration status, including whether it is measured or configured estimate

## Parameter Policy

The GENE 26.5 / ACKON500 controller-simulation profile is only a seed record.
It must not be copied directly into physical motion.

Physical defaults remain conservative until accepted:

- `rate_hz`: use the accepted real Servo J rate, not automatically 500 Hz.
- `speed_bar`: use a conservative local value.
- Cartesian max step: tiny and stage-specific.
- First circle: 5 cm diameter / 10 s period.

The report may compare controller-simulation best parameters with physical
candidate parameters. It must not mark them promoted until the corresponding
physical ladder stage passes with `tcp_actual_stand`.

## Artifact Schema

The acceptance script writes `summary.json` with schema
`robotics_lab.rbpodo_physical_transition.stage.v1`.

Key fields:

```yaml
stage:
  id: P0|P1|P2|P3|P4|P5|P6|P7|P8|P9
  ladder_name: string
result:
  status: dry_run|blocked|preflight_pass|pass|fail
  dry_run: true|false
  hardware_process_started: false
  motion_command_sent: false
  blockers: []
gates:
  required_env: []
  missing_env: []
  required_confirmation_flags: []
  missing_confirmation_flags: []
physical_tracking_result:
  status: pass|fail|not_run
  tracking_source: tcp_actual_stand
  rms_error_m: null
  p95_error_m: null
  max_error_m: null
controller_reference_result:
  status: informational_only
  tracking_source: tcp_ref_stand
telemetry_requirements:
  state_age_us: {}
  state_jitter_us: {}
  q_actual_update_rate_hz: null
  q_ref_update_rate_hz: null
  fault_latch_status: not_checked|unresolved|pass|fail
  cartesian_availability: not_checked|unresolved|available|unavailable
  stop_reset_behavior_result: not_applicable|unresolved|pass|fail
  physical_motion_expected: true|false
  physical_motion_detected: true|false|null
calibration:
  status: configured_estimate|measured|unknown
  measured: true|false
```

The report generator emits schema
`robotics_lab.rbpodo_physical_transition.report.v1` and keeps
`physical_readiness.status=blocked` until P0-P8 have artifact references and
the source semantics are valid.
