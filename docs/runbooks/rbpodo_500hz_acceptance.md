# rbpodo 500 Hz Controller-Simulation Acceptance

This runbook is for staged 500 Hz rbpodo controller `pgmode` simulation only.
It connects to real Rainbow controller boxes, but every config in this track
must keep each robot `operation_mode: simulation` and
`cartesian_control.allow_in_real: false`.

It is not a default-rate change and it is not physical real-motion readiness.

## 500 Hz Mode Taxonomy

There are three distinct 500 Hz controller-simulation modes. Do not merge
their report semantics:

| Mode | Loop behavior | ACK/reference evidence | Use |
| --- | --- | --- | --- |
| synchronous ACK-on | `servo.rbpodo_async_streaming.enable: false`, `disable_waiting_ack: false`; the servo loop calls rbpodo and waits for the SDK call/ACK path before the tick can complete | Successful sends are `controller_ack_observed` when the controller ACK is observed. `q_ref` / `tcp_ref_stand` are still measurement telemetry, but ACK is the command-acceptance signal. | Baseline 500 Hz evidence and failure comparison. |
| `sdk_ack_worker` | The servo loop enqueues the latest Servo J target and does not wait for ACK; a per-arm worker calls the synchronous rbpodo SDK and records ACK timing/results | Worker ACKs are `sdk_worker_ack_observed` / `controller_ack_observed`-style evidence in the worker lane. Missing ACK supervision can fault; q_ref/tcp_ref supervision contributes health telemetry and invalid `q_ref` faults. | Tests whether moving ACK waits out of the tick removes servo-loop blocking. |
| `socket_send_supervised` | The servo loop enqueues the latest target and does not wait for ACK; the worker uses ACK-disabled socket/API send semantics | Successful sends are `socket_send_only`, never per-command controller ACK. A q_ref watchdog and tcp_ref watchdog must show fresh controller-reference movement and target convergence; stale/missing reference telemetry faults. | Tests high-rate socket-send feasibility with controller-reference supervision. |

No physical real: all three modes in this runbook are rbpodo controller
`pgmode` simulation only. They require `operation_mode: simulation`
(`operation_mode=simulation` in reports), `physical_motion_expected=false`,
and no physical robot motion.
`operation_mode: real` is out of scope and must be refused for async 500 Hz
evidence.

## Result Contract

Circle summaries and 500 Hz reports separate execution, safety, generic
benchmark thresholds, the official ACKON500 goal, and diagnostics:

- `run_result.status` says whether the run completed, faulted, was blocked, or
  errored. New artifacts mirror this in the legacy top-level `result`.
- `safety_result.status` covers fault latch, physical-motion detection, and
  Cartesian availability.
- `benchmark_threshold_result.status` covers generic report thresholds such as
  `max_orientation_drift_rad`.
- `ackon500_goal_result.status` is the official goal result for
  `gene_15cm_4s`. It uses `p95_orientation_drift_rad <= 0.02 rad`, not max
  orientation drift, unless `GOAL.md` is updated.
- `diagnostic_warnings` keeps non-fatal diagnostics visible, including
  `max_orientation_drift_spike`, timing spikes, diagnostics-suspect override
  evidence, and controller-reference lower-bound caveats.

A 500 Hz `sdk_ack_worker` candidate can therefore be official goal `pass` while
`benchmark_threshold_result.status` is `fail` because a generic
`max_orientation_drift_rad` diagnostic threshold fired. That is not a hidden
failure of the official ACKON500 goal; it is a diagnostic warning that must
remain visible. `socket_send_only_count > 0` is still an official goal failure.

## Canonical Benchmark Lanes

Every 500 Hz circle summary and report row must expose these canonical fields:

```text
benchmark_lane
control_loop_location
trajectory_generation_location
feedback_loop_location
low_level_send_mode
acceptance_semantics
tracking_source
physical_motion_expected
```

The report may also keep older comparison labels such as `500hz_ack_on` or
`500hz_async_sdk_ack_worker`, but `benchmark_lane` is the source of truth:

| Canonical lane | Required interpretation |
| --- | --- |
| `rbpodo_server_side_circle_ackon500_sync` | Server-side circle with synchronous ACK-on sends at 500 Hz. Useful comparison evidence, not the official async ACKON500 pass lane. |
| `rbpodo_server_side_circle_ackon500_sdk_worker` | Official ACKON500 pass lane when `async_mode=sdk_ack_worker`, `low_level_send_mode=sdk_ack_worker`, and `acceptance_semantics=sdk_worker_ack_observed`. |
| `rbpodo_server_side_circle_500hz_socket_send_supervised` | Socket/API send with reference supervision. It is `socket_send_only` and cannot pass the official ACKON500 goal. |
| `rbpodo_python_streaming_feedback` | Python benchmark feedback streaming. Keep separate from server-side circle even when tracking metrics are good. |

Official ACKON500 goal reports must group by `benchmark_lane` and reject any
candidate whose lane is not `rbpodo_server_side_circle_ackon500_sdk_worker`.

## Current Best Controller-Simulation Profile

The named high-performance controller-simulation profile is:

```text
rb_servo_server/config/dual_real_rbpodo_circle_15cm4s_500hz_goal.example.yaml
configs/rbpodo_circle_ablation/ackon500_gene_goal_best.yaml
tools/rbpodo_ackon500_gene_goal.sh --profile best
```

It encodes the current ACKON500 best candidate for the controller-reference
lower-bound lane. It is not physical real tracking and is not physical real
readiness. The runner prints this caveat for every run:

```text
This is controller-reference lower-bound evidence, not physical real tracking.
```

**ACKON500 PASS is controller-reference lower-bound evidence, not physical TCP tracking.**

Exact promoted parameters:

```yaml
servo:
  rate_hz: 500
  rbpodo_async_streaming:
    enable: true
    mode: sdk_ack_worker
    rate_hz: 500
    queue_policy: latest_wins
    ack_supervision.enable: true
    reference_supervision.enable: true

left_robot/right_robot:
  operation_mode: simulation
  command_timeout_sec: 0.05
  speed_bar: 0.2
  servo_t1_sec: 0.002
  servo_t2_sec: 0.08
  servo_alpha: 0.8
  disable_waiting_ack: false

cartesian_control:
  enable: true
  enable_benchmark_primitives: true
  allow_in_controller_simulation: true
  allow_in_real: false
  max_twist_linear_m_s: 0.2
  max_twist_angular_rad_s: 0.4
  path_kp_pos: 6.0
  path_kp_ori: 6.0

network:
  state_pub_rate_hz: 100
```

The server-side circle row sets `phase_advance_sec: 0.005` in the matrix. The
current evidence category is `profile=gene_15cm_4s`, `diameter_m=0.15`,
`period_sec=4.0`, `repeat>=5`, `tracking_source=tcp_ref_stand`,
`async_mode=sdk_ack_worker`, and ACK-observed semantics. The latest reviewed
best candidate was about 1.5 mm RMS with about 3.3 ms effective phase latency.

Exact pass criteria for this profile:

- `operation_mode=simulation`, `physical_motion_expected=false`, and
  `physical_motion_detected=false`.
- `benchmark_lane=rbpodo_server_side_circle_ackon500_sdk_worker`,
  `low_level_send_mode=sdk_ack_worker`, and
  `acceptance_semantics=sdk_worker_ack_observed`.
- 500 Hz official goal-window send rate within the configured tolerance band
  of 490 to 510 Hz.
- ACK-observed evidence for the official window; `socket_send_only_count=0`.
- `profile=gene_15cm_4s`, left arm unless the row explicitly says otherwise,
  15 cm diameter, 4 s period, and `repeat>=5`.
- `rms_error_m <= 0.003`, `p95_error_m <= 0.006`,
  `p95_orientation_drift_rad <= 0.02`, and effective phase latency absolute
  value <= 5 ms.
- `fault_latched=false`, `cartesian_unavailable_count=0`, and
  `feedback_saturation_count=0`.

Caveats:

- This is controller-reference lower-bound evidence from Rainbow `pgmode`
  simulation, not physical `tcp_actual_stand` tracking.
- `allow_in_real: false` must remain set, and `RB_ALLOW_REAL_CARTESIAN` must
  not be used by this workflow.
- `disable_waiting_ack: false` must remain set; ACK-off/socket-send-only rows
  cannot pass the official profile.
- `diagnostics_suspect` remains a documented controller-simulation carve-out
  until its root cause is resolved.
- The profile is left-arm single-arm evidence until the follow-on validations
  below are completed.

Required ACKON500 report boundary fields:

```yaml
physical_readiness:
  status: blocked
  blockers:
    - diagnostics_suspect_unresolved
    - physical_reference_to_actual_error_unmeasured
    - stop_resetFault_unverified
    - camera_tcp_calibration_unresolved
    - no_tiny_physical_acceptance
  next_required_acceptance:
    - read-only diagnostics parity
    - tiny joint no-op physical or approved safe mode
    - tiny physical joint move
    - tiny physical Cartesian move
    - low-speed circle
    - then speed ladder
controller_reference_result:
  status: pass|fail
  explanation: "tcp_ref_stand lower-bound evidence"
physical_tracking_result:
  status: not_measured
```

Transition ladder:

1. Controller pgmode simulation repeatability
2. Right arm
3. Dual arm
4. P0 diagnostics root cause
5. Real controller read-only
6. Tiny physical acceptance
7. Slow physical circle
8. Fast physical circle only after approval

## ACKON500 Repeatability Validation

ACKON500-REPEATABILITY-VALIDATION-01 is the official repeatability matrix for
the achieved best profile. It is reporting/configuration only and does not
change control behavior or physical real gates.

Matrix:

```text
configs/rbpodo_circle_ablation/ackon500_gene_repeatability.yaml
```

Required runs:

```text
best_left_run01
best_left_run02
best_left_run03
best_right_run01
best_right_run02
best_right_run03
```

Every required row uses the tracked best-profile config and pins the same
best-profile runtime shape:

- `profile=gene_15cm_4s`, `controller=server_circle`, `repeat=5`
- `servo.rate_hz=500`, `servo_t1_sec=0.002`
- `sdk_ack_worker`, `disable_waiting_ack=false`, ACK-observed semantics
- `servo_t2_sec=0.08`, `servo_alpha=0.8`, `speed_bar=0.2`
- `cartesian_control.path_kp_pos=6.0`, `path_kp_ori=6.0`
- `phase_advance_sec=0.005`
- `tracking_source=tcp_ref_stand`

The tracked goal config must still validate with
`cartesian_control.allow_in_controller_simulation=true` and
`cartesian_control.allow_in_real=false`; these safety fields are not matrix
overrides.

Runner:

```bash
tools/rbpodo_ackon500_gene_goal.sh --profile repeatability \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation
```

As with the best-profile runner, pass `--with-required-env` only when the
operator explicitly wants the wrapper to export the required controller
simulation gates. Do not set `RB_ALLOW_REAL_CARTESIAN` for this workflow.

Repeatability report outputs:

```text
matrix_resolved.yaml
repeatability_report.md
repeatability_summary.csv
repeatability_summary.json
```

Aggregate pass criteria:

- every required row passes the official ACKON500 goal
- the left-arm aggregate passes only if all three required left rows pass
- the right-arm aggregate passes only if all three required right rows pass
- global repeatability passes only if both left and right aggregates pass
- median RMS <= 3 mm and worst RMS <= 3.5 mm
- median p95 <= 6 mm and worst p95 <= 8 mm
- `fault_latched=false` for every required row
- `physical_motion_detected=false` for every required row
- ACK observed ratio >= 0.98 for every required row
- `socket_send_only_count=0` for every required row
- `diagnostics_suspect` remains an explicit caveat
- `physical_readiness.status=blocked` remains machine-readable in JSON and
  visible in markdown

Aggregate fields:

```text
rms_mean/std/min/max
p95_mean/std/min/max
latency_mean/std/min/max
ack_observed_ratio_min
state_age_p95_max
```

Classifications:

- `repeatable_pass`: complete left/right evidence, all required runs pass, and
  aggregate thresholds pass.
- `not_repeatable`: complete evidence where any required official row fails,
  either per-arm aggregate fails, hard safety/ACK/socket failures are present,
  or aggregate medians fail.
- `insufficient_evidence`: missing required left/right repeats or missing
  aggregate metrics.

Optional dual-arm sequential rows, if a future task adds a reviewed
controller-simulation-only runner shape, must use
`best_dual_sequential_run01..03` and remain separate from the required
left/right aggregate pass. Do not add inert disabled `arm: both` rows; the
matrix validator currently accepts only `left` and `right`.

## Servo Rate Accounting

ACKON500 reports separate high-level UDP command packets from low-level ServoJ
worker sends. Do not use `command_count` or `udp_command_count` as a low-level
ServoJ command count.

Official rate fields:

- `udp_command_count`: high-level UDP commands sent by the benchmark, such as
  one circle command plus hold.
- `server_servo_tick_count`: servo-loop ticks observed inside the official
  tracking window.
- `async_commands_enqueued_total`, `async_commands_sent_total`, and
  `async_commands_acked_total`: async worker lifetime totals, including any
  warmup, hold, or cleanup traffic.
- `official_tracking_window_sec`: start of official circle tracking to the end
  of the repeat window. For 15 cm / 4 s repeat 5 this is about 20 seconds.
- `expected_servo_ticks`: `round(servo_rate_hz * official_tracking_window_sec)`.
- `goal_window_commands_sent` and `goal_window_commands_acked`: low-level
  ServoJ sends and ACKs counted for the official tracking window only.
- `official_servo_rate_hz`: configured official target rate for the run.
- `effective_goal_command_rate_hz`:
  `goal_window_commands_sent / official_tracking_window_sec`.
- `ack_coverage_ratio`:
  `goal_window_commands_acked / goal_window_commands_sent`.
- `measured_worker_window_sec`, `worker_send_rate_hz`, and
  `worker_lifetime_send_rate_hz`: worker lifetime diagnostics. They can differ
  from the official goal rate when warmup, hold, cleanup, or measurement-window
  boundaries differ.
- `worker_sends_outside_official_window`: lifetime sends minus official-window
  sends. This must remain visible instead of being folded into the goal rate.

The official ACKON500 rate check uses `effective_goal_command_rate_hz` in the
configured tolerance band, currently 490 to 510 Hz. A candidate must not pass
the official goal from `worker_send_rate_hz` alone when official tracking-window
evidence is missing. The older ambiguous `effective_command_rate_hz` should not
be used for pass/fail.

## Why Synchronous ACK-On Is Fragile At 500 Hz

A 500 Hz servo tick is only 2 ms. In synchronous ACK-on mode, any rbpodo
`move_servo_j` call that waits close to or beyond that 2 ms budget consumes the
whole tick before the loop can finish timing, safety, state publication, and
the second arm.

This fragility shows up in three places:

- ACK wait outliers: occasional controller ACK latency spikes can create
  deadline misses even when median send time looks acceptable.
- Dual-arm sequential effects: if the loop services left and right arms in one
  thread, the second arm inherits the first arm's ACK delay before its own send
  begins.
- Worker effects: worker I/O avoids blocking the main loop only if backlog,
  overwrites, drops, and ACK supervision stay bounded; otherwise the worker may
  hide a controller-acceptance problem until the watchdog faults.

Async ACK-supervised mode exists to separate the servo tick from controller ACK
waiting. The target generation loop must keep ticking without waiting for ACK,
while a worker or supervisor records ACK and/or q_ref/tcp_ref state. Missing
ACK, missing q_ref, stale tcp_ref, target divergence, or excessive worker
overwrite/drop counts are faults or acceptance failures, not acceptable jitter.

## Stage 0 Evidence

The initial evidence is a single-arm `rbpodo_500hz_acceptance.py`
`servo_j_noop_500hz` artifact:

```text
artifacts/rbpodo_servo_j_rate_probe_left
```

The 500 Hz row completed 5000/5000 Servo J no-op sends over 10 seconds in
controller `pgmode` simulation. The measured feedback loop interval p99 was
about 2.006 ms, the max loop interval was about 2.097 ms, and the max send
duration was about 501 us. `q_actual_drift_max_deg` was 0.0 in that artifact.

This is sufficient only to begin staged 500 Hz controller-simulation
acceptance. It does not prove dual-arm server behavior, circle tracking, or
physical robot safety.

## Staged Plan

Run stages in order and stop at the first fault, physical-motion warning,
Cartesian gate rejection, timing classification problem, or missing reference
telemetry.

Synchronous 500 Hz acceptance starts from the existing no-op acceptance:

1. Single-arm 500 Hz no-op acceptance: already completed for the
   left controller in `artifacts/rbpodo_servo_j_rate_probe_left`.
2. `rb_servo_server` full-path no-op at 500 Hz using
   `scripts/rbpodo_500hz_acceptance.py --mode servo_j_noop_500hz`.
3. 5 cm / 10 s circle.
4. 15 cm / 16 s circle.
5. 15 cm / 8 s circle.
6. 15 cm / 4 s circle.

Async ACK-supervised acceptance must use this order:

1. SDK async capability probe.
2. No-op acceptance for the candidate async mode.
3. `safe_5cm_10s`.
4. `circle_15cm_16s`.
5. `circle_15cm_8s`.
6. `gene_15cm_4s` stress.

The 15 cm / 4 s profile remains stress evidence. Do not label it physical-real
ready.

## Configs

Create local 500 Hz configs explicitly:

```bash
tools/create_rbpodo_circle_local_configs.sh --include-500hz
```

The helper does not create 500 Hz local files by default. Use `--force` only
after reviewing local operator edits:

```bash
tools/create_rbpodo_circle_local_configs.sh --include-500hz --force
```

Tracked 500 Hz templates:

```text
rb_servo_server/config/dual_real_rbpodo_circle_5cm10s_500hz.example.yaml
rb_servo_server/config/dual_real_rbpodo_circle_15cm16s_500hz.example.yaml
rb_servo_server/config/dual_real_rbpodo_circle_15cm8s_500hz.example.yaml
rb_servo_server/config/dual_real_rbpodo_circle_15cm4s_500hz.example.yaml
rb_servo_server/config/dual_real_rbpodo_circle_15cm4s_500hz_goal.example.yaml
```

Required safety shape:

```yaml
servo:
  rate_hz: 500
  send_servo_commands: true
  allow_controller_simulation_motion: true
  allow_controller_simulation_diagnostics_suspect: true

left_robot:
  backend_type: rbpodo
  run_mode: real
  operation_mode: simulation
  servo_t1_sec: 0.002
  servo_t2_sec: 0.03
  disable_waiting_ack: false

cartesian_control:
  allow_in_controller_simulation: true
  allow_in_real: false
  controller_simulation_servo_state_source: reference
  controller_simulation_divergence_source: reference

safety:
  controller_simulation_tracking_error_source: reference
  controller_simulation_physical_motion_policy: fault_latch
  controller_simulation_physical_motion_threshold_deg: 0.05
```

`network.state_pub_rate_hz` remains a telemetry publication setting. Do not
tie it to the 500 Hz servo command rate until a separate measurement task
accepts that change.

## Required Gates

Every controller-simulation command run still requires explicit operator
confirmation. The 500 Hz Servo J no-op stage requires these env gates:

```bash
RB_ALLOW_REAL_ROBOT=1
RB_ALLOW_REAL_MOTION=1
RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1
RB_RBPODO_PGMODE_SIMULATION_CONFIRMED=1
```

Async 500 Hz modes additionally require:

```bash
RB_ALLOW_RBPODO_ASYNC_STREAMING=1
```

Circle stages additionally require the Cartesian controller-simulation carve-out:

```bash
RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1
```

The no-op acceptance runner does not need Cartesian primitives. By default it
writes an artifact-local `resolved_config.yaml` with
`cartesian_control.enable: false` before launching `rb_servo_server`. This does
not edit the source or local YAML. Use `--preserve-cartesian-control` only when
you intentionally want to validate a config that keeps Cartesian enabled; in
that case the runner requires `RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1` and
classifies a missing gate as `preflight_env_missing_or_config_mismatch`.

Do not set `RB_ALLOW_REAL_CARTESIAN` for this workflow. The 500 Hz templates
are still controller-simulation Cartesian only, not physical real Cartesian.

If the temporary diagnostics-suspect bridge is needed, it must be explicitly
allowed by the YAML and by:

```bash
RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1
```

ACK waiting remains enabled in the 500 Hz templates:

```yaml
disable_waiting_ack: false
```

This is synchronous ACK-on evidence, meaning accepted Servo J sends are
reported as `controller_ack_observed`. Existing ACK-off diagnostics remain
`socket_send_only` evidence and are not controller ACK acceptance.

## Async ACK-Supervised Worker Contract

The async 500 Hz worker is controller `pgmode` simulation only and remains
disabled by default. When enabled, the servo loop computes a target every tick
and enqueues a latest-wins Servo J request into one per-arm worker slot instead
of waiting for rbpodo SDK ACK in the loop thread:

```yaml
servo:
  rbpodo_async_streaming:
    enable: false
    mode: disabled
    rate_hz: 500
    queue_policy: latest_wins
    max_pending_age_ms: 10
    ack_supervision:
      enable: true
      expected_ack_timeout_ms: 50
      missing_ack_fault_after_ms: 100
      max_consecutive_missing_ack: 10
    reference_supervision:
      enable: true
      q_ref_update_timeout_ms: 50
      tcp_ref_update_timeout_ms: 50
      q_ref_target_tolerance_deg: 1.0
      q_ref_target_fault_after_ms: 100
      tcp_ref_target_tolerance_m: 0.02
      tcp_ref_target_fault_after_ms: 100
      policy: fault_latch
    diagnostics:
      publish_per_command_jsonl: false
```

Modes:

- `disabled`: current behavior.
- `sdk_ack_worker`: the worker calls the synchronous rbpodo SDK and may block
  waiting for ACK in its own thread; the servo loop does not block. Accepted
  worker results use `controller_ack_observed`, but this mode may still miss
  500 Hz if controller ACK latency is slower than the command period. Backlog,
  overwrites, drops, send duration, ACK duration, and last failure are published
  per arm. ACK remains the primary acceptance signal; q_ref/tcp_ref reference
  supervision is warning telemetry unless `q_ref` is invalid.
- `socket_send_supervised`: the SDK uses `disable_waiting_ack=true` or an
  equivalent socket-send-only path. Sends must be reported as
  `socket_send_only`, never `controller_ack_observed`. `q_ref` and/or
  `tcp_ref` watchdog supervision is required to infer controller progress:
  invalid `q_ref` faults, stale `q_ref`/`tcp_ref` beyond the configured update
  timeout faults, and target divergence becomes a warning or fault according to
  `reference_supervision.policy`. Socket send evidence alone is not controller
  ACK acceptance.

Async mode requires both arms to use `backend_type: rbpodo`, `run_mode: real`,
and `operation_mode: simulation`; `operation_mode: real` is refused. Runtime
also requires:

```bash
RB_ALLOW_RBPODO_ASYNC_STREAMING=1
RB_ALLOW_REAL_ROBOT=1
RB_ALLOW_REAL_MOTION=1
RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1
RB_RBPODO_PGMODE_SIMULATION_CONFIRMED=1
```

For `socket_send_supervised`, add one of:

```bash
RB_ALLOW_RBPODO_ACK_DISABLED_MOTION=1
RB_ALLOW_RBPODO_SOCKET_SEND_ONLY_STREAMING=1
```

Async supervision is not proof of physical real safety. It does not authorize
physical `operation_mode: real`, physical Cartesian motion, or a default 500 Hz
rate change.

State JSON publishes per-arm `async_streaming` telemetry including
`enabled`, `mode`, `commands_enqueued_total`, `commands_sent_total`,
`commands_acked_total`, `commands_socket_sent_total`,
`commands_overwritten_total`, `commands_dropped_total`, `ack_timeout_count`,
`first_worker_send_ns`, `last_worker_send_ns`,
`first_goal_command_send_ns`, `last_goal_command_send_ns`,
`goal_window_commands_sent`, `goal_window_commands_acked`,
`command_phase`,
`last_async_send_duration_us`, `last_async_ack_duration_us`,
`last_async_acceptance_semantics`, `async_worker_backlog`,
`q_ref_update_age_ms`, `tcp_ref_update_age_ms`,
`q_ref_target_error_deg_max`, `tcp_ref_target_error_m`,
`reference_supervision_state`, `reference_supervision_reason`,
`reference_supervision_fault_count`, and `async_supervision_state`. A
supervision fault latches the servo loop fault state and suppresses further
regular servo sends.

## SDK Async Capability Probe

Before changing `sdk_ack_worker` or `socket_send_supervised` assumptions,
characterize what the Python rbpodo SDK exposes. `RBPODO-ASYNC-SDK-PROBE-01`
adds a controller `pgmode` simulation-only probe:

```bash
python3 scripts/rbpodo_async_sdk_probe.py \
  --ip 172.28.60.200 \
  --duration-sec 5 \
  --rate-hz 500 \
  --mode ack_on \
  --artifact-dir artifacts/rbpodo_async_sdk_probe/left \
  --set-pgmode-simulation \
  --i-understand-this-connects-to-real-controller \
  --allow-simulation-servo-j
```

Modes:

- `ack_on`: calls `move_servo_j` with ACK waiting enabled and records per-call
  duration. Successful sends are treated as `controller_ack_observed` evidence.
- `ack_off`: requires `RB_ALLOW_RBPODO_ACK_DISABLED_MOTION=1`, configures
  `disable_waiting_ack` when the SDK exposes it, records socket/API send
  duration, and samples `q_ref` for supervision evidence. Successful sends
  remain `socket_send_only`, not controller ACK evidence.
- `concurrent_read_send`: uses separate `Cobot` and `CobotData` objects so one
  thread can send no-op Servo J while another reads state. It does not prove a
  shared `Cobot` object is thread-safe; `sdk_thread_safety_observed` remains
  `unknown` unless a future SDK-documented safe sharing path is tested.

Required gates for every mode:

```bash
RB_ALLOW_REAL_ROBOT=1
RB_ALLOW_REAL_MOTION=1
RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1
```

Do not set `RB_ALLOW_REAL_CARTESIAN`. The probe refuses `operation_mode: real`,
requires `--set-pgmode-simulation` or `--verify-pgmode-simulation`, requires
`--i-understand-this-connects-to-real-controller`, and requires
`--allow-simulation-servo-j`. Before every Servo J send, the target must still
match current `q_ref` and `q_actual` within tolerance; otherwise the probe stops
without sending.

Artifacts:

```text
summary.json
send_samples.csv
state_samples.csv
responses.jsonl
sdk_capabilities.json
report.md
```

The summary classification is one of:

- `ack_on_fast_enough`
- `ack_on_outlier_limited`
- `ack_off_state_supervision_viable`
- `concurrent_read_send_viable`
- `sdk_async_ack_not_supported`
- `insufficient_evidence`

Use the result only to choose the next implementation shape:

- `ack_on_fast_enough` or `ack_on_outlier_limited` supports investigating
  `sdk_ack_worker`, where a worker may block inside the SDK while the servo loop
  remains non-blocking.
- `ack_off_state_supervision_viable` supports investigating
  `socket_send_supervised`, where sends are socket/API evidence only and
  `q_ref` / `tcp_ref` watchdogs supervise controller progress.
- `concurrent_read_send_viable` supports using a separate data-channel reader
  while command sends are in flight.
- `sdk_async_ack_not_supported` or `insufficient_evidence` means do not
  implement async streaming from assumptions.

This probe does not prove dual-arm `rb_servo_server` 500 Hz behavior, does not
prove circle tracking, and does not authorize physical robot motion.

Use the probe before the no-op stage for any async acceptance run. A typical
socket-send supervision probe uses the common controller-simulation gates plus
an ACK-disabled/socket-send approval:

```bash
RB_ALLOW_REAL_ROBOT=1 \
RB_ALLOW_REAL_MOTION=1 \
RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1 \
RB_ALLOW_RBPODO_ACK_DISABLED_MOTION=1 \
python3 scripts/rbpodo_async_sdk_probe.py \
  --ip 172.28.60.200 \
  --duration-sec 5 \
  --rate-hz 500 \
  --mode ack_off \
  --artifact-dir artifacts/rbpodo_async_sdk_probe/left_ack_off \
  --set-pgmode-simulation \
  --i-understand-this-connects-to-real-controller \
  --allow-simulation-servo-j
```

## Build And Capabilities

Build the rbpodo-enabled server:

```bash
cmake -S rb_servo_server -B rb_servo_server/build/rbpodo_real_gate \
  -DCMAKE_BUILD_TYPE=Release \
  -DRB_SERVO_ENABLE_RBPODO=ON \
  -DBUILD_TESTING=ON
cmake --build rb_servo_server/build/rbpodo_real_gate -j
```

If realtime scheduling is required, apply capabilities to the exact binary:

```bash
sudo setcap cap_sys_nice,cap_ipc_lock+ep rb_servo_server/build/rbpodo_real_gate/rb_servo_server
getcap rb_servo_server/build/rbpodo_real_gate/rb_servo_server
```

Rebuilding or replacing the binary removes these capabilities. Re-run `setcap`
after every rebuild before a 500 Hz controller-simulation run.

## Config Checks

Before running, verify the local copies:

```bash
python3 - <<'PY'
from pathlib import Path
import yaml

for path in sorted(Path("rb_servo_server/config/local").glob("*_500hz.yaml")):
    yaml.safe_load(path.read_text())
    print(path)
PY

grep -H "servo_t1_sec: 0.002" rb_servo_server/config/local/*_500hz.yaml
grep -H "allow_in_real: false" rb_servo_server/config/local/*_500hz.yaml
grep -H "operation_mode: simulation" rb_servo_server/config/local/*_500hz.yaml
```

Treat Servo J ACKs as command-path evidence only. Circle evidence requires
Cartesian commands to be accepted and controller reference telemetry such as
`tcp_ref_stand` to move.

## Stage 2: rb_servo_server No-Op

Run the server-level no-op acceptance before any 500 Hz circle stage. The
runner starts `rb_servo_server`, sends `ArmMotion`, captures the current
selected-arm `q_ref_deg` / `q_target_deg`, and streams constant `JointTarget`
packets at 500 Hz. This is still controller `pgmode` simulation only.

Example:

```bash
python3 scripts/rbpodo_500hz_acceptance.py \
  --server rb_servo_server/build/rbpodo_real_gate/rb_servo_server \
  --config rb_servo_server/config/local/dual_real_rbpodo_circle_5cm10s_500hz.yaml \
  --arm left \
  --send-arms left \
  --command-timeout-sec 0.005 \
  --duration-sec 10 \
  --artifact-dir artifacts/rbpodo_500hz_acceptance/left_noop \
  --set-pgmode-simulation \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation
```

ACK timeout tightness can be separated from 500 Hz timing with a sequential
sweep. Each timeout gets its own subdirectory, for example
`timeout_005ms`, `timeout_010ms`, and so on:

```bash
python3 scripts/rbpodo_500hz_acceptance.py \
  --server rb_servo_server/build/rbpodo_real_gate/rb_servo_server \
  --config rb_servo_server/config/local/dual_real_rbpodo_circle_5cm10s_500hz.yaml \
  --arm left \
  --send-arms both \
  --duration-sec 10 \
  --warmup-duration-sec 2 \
  --warmup-rate-hz 100 \
  --warmup-command-timeout-sec 0.05 \
  --ack-timeout-sweep 0.005,0.01,0.02,0.05 \
  --artifact-dir artifacts/rbpodo_500hz_acceptance/ack_timeout_sweep \
  --set-pgmode-simulation \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation
```

Default pass/fail thresholds:

```text
--max-send-duration-p99-us 1000
--max-servo-jitter-p99-ms 2.5
--max-deadline-miss-count 0
--min-send-count-ratio 0.98
--min-controller-acceptance-ratio 0.98
--max-physical-motion-deg 0.05
--max-reference-drift-deg 0.05
```

Required artifacts:

```text
summary.json
summary.csv
safety_preflight.json
pgmode_summary.json
resolved_config.yaml
noop_target.json
state_stream.jsonl
command_packets.jsonl
rb_servo_server.log
servo_log.csv
timing_send_duration.png
timing_servo_jitter.png
```

Acceptance means:

- `result: pass`
- `physical_motion_expected=false`
- `physical_motion_detected=false`
- no fault latch
- no non-noop joint target; every streamed command uses the captured
  `q_ref_deg` / `q_target_deg`
- `send_count` is close to `500 * duration_sec`
- `controller_acceptance_observed_count` is close to `send_count`
- `send_duration_us.p99` is below threshold
- `servo_jitter_ms.p99` is below threshold
- `send_deadline_missed_count` is at or below threshold
- `worker_command_drops_total` is zero if worker I/O is used

Failure classification fields are part of the acceptance evidence:

- `failure_phase`: `preflight`, `startup`, `warmup`, or `measurement`
- `failure_classification`: `preflight_env_missing`,
  `preflight_env_missing_or_config_mismatch`, `startup_ack_timeout`,
  `warmup_ack_timeout`, `measurement_ack_timeout`, `deadline_limited`, or a
  generic threshold/fault class
- `ack_timeout_count_by_arm`, `warmup_timeout_count`, and
  `measurement_timeout_count`
- `first_send_failure_arm`, `first_send_failure_index`, and
  `first_send_failure_elapsed_sec`
- `send_duration_p99_us_by_arm` and `deadline_miss_count_by_arm`

If `servo_log.csv` is missing, do not treat the run as accepted. The state
stream is still useful diagnostic evidence, but per-servo-tick 500 Hz
acceptance depends on the servo log timing and ACK fields.

### Async ACK-Supervised No-Op Acceptance

`RBPODO-ASYNC-500HZ-ACCEPT-01` extends the same no-op runner with explicit
async modes:

```text
--async-mode disabled|sdk_ack_worker|socket_send_supervised
--require-reference-supervision
--max-q-ref-update-age-ms
--max-tcp-ref-update-age-ms
--max-overwrite-ratio
--max-drop-ratio
--min-q-ref-update-rate-hz
--allow-socket-send-only
```

The runner writes an artifact-local `resolved_config.yaml`; it does not edit
the source or local YAML. With `--async-mode sdk_ack_worker`, it enables
`servo.rbpodo_async_streaming.mode: sdk_ack_worker` and keeps ACK waiting
enabled. With `--async-mode socket_send_supervised`, it enables
`servo.rbpodo_async_streaming.mode: socket_send_supervised`, sets both
`disable_waiting_ack` fields in the artifact-local config, and requires
`--allow-socket-send-only`.

Additional env gates for both async no-op modes:

```bash
RB_ALLOW_RBPODO_ASYNC_STREAMING=1
```

`socket_send_supervised` additionally requires one of:

```bash
RB_ALLOW_RBPODO_ACK_DISABLED_MOTION=1
RB_ALLOW_RBPODO_SOCKET_SEND_ONLY_STREAMING=1
```

Run the ACK-worker no-op stage:

```bash
RB_ALLOW_REAL_ROBOT=1 \
RB_ALLOW_REAL_MOTION=1 \
RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1 \
RB_ALLOW_RBPODO_ASYNC_STREAMING=1 \
RB_RBPODO_PGMODE_SIMULATION_CONFIRMED=1 \
python3 scripts/rbpodo_500hz_acceptance.py \
  --server rb_servo_server/build/rbpodo_real_gate/rb_servo_server \
  --config rb_servo_server/config/local/dual_real_rbpodo_circle_5cm10s_500hz.yaml \
  --arm left \
  --send-arms both \
  --duration-sec 10 \
  --async-mode sdk_ack_worker \
  --max-overwrite-ratio 0.05 \
  --max-drop-ratio 0.0 \
  --artifact-dir artifacts/rbpodo_async_500hz/noop_sdk_ack_worker \
  --set-pgmode-simulation \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation
```

Run the socket-send supervised no-op stage:

```bash
RB_ALLOW_REAL_ROBOT=1 \
RB_ALLOW_REAL_MOTION=1 \
RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1 \
RB_ALLOW_RBPODO_ASYNC_STREAMING=1 \
RB_ALLOW_RBPODO_ACK_DISABLED_MOTION=1 \
RB_RBPODO_PGMODE_SIMULATION_CONFIRMED=1 \
python3 scripts/rbpodo_500hz_acceptance.py \
  --server rb_servo_server/build/rbpodo_real_gate/rb_servo_server \
  --config rb_servo_server/config/local/dual_real_rbpodo_circle_5cm10s_500hz.yaml \
  --arm left \
  --send-arms both \
  --duration-sec 10 \
  --async-mode socket_send_supervised \
  --allow-socket-send-only \
  --require-reference-supervision \
  --min-q-ref-update-rate-hz 20 \
  --max-q-ref-update-age-ms 50 \
  --max-tcp-ref-update-age-ms 50 \
  --max-overwrite-ratio 0.05 \
  --max-drop-ratio 0.0 \
  --artifact-dir artifacts/rbpodo_async_500hz/noop_socket_send_supervised \
  --set-pgmode-simulation \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation
```

Async summaries add these required fields:

```text
async_mode
socket_send_only_count
controller_ack_observed_count
reference_supervision_state
q_ref_update_rate_hz
q_ref_update_age_p95_ms
tcp_ref_update_age_p95_ms
commands_overwritten_total
commands_dropped_total
async_worker_backlog_max
supervision_fault_count
servo_loop_blocked_by_ack
```

Async no-op pass criteria:

- `fault_latched=false`
- `physical_motion_detected=false`
- `send_deadline_missed_count` is at or below `--max-deadline-miss-count`
- `servo_loop_blocked_by_ack=false`
- no invalid selected-arm state packets
- `sdk_ack_worker`: `controller_ack_observed_count > 0` and overwrite/drop
  ratios are below their thresholds
- `socket_send_supervised`: `socket_send_only_count > 0`,
  `reference_supervision_state=ok`, `q_ref_update_rate_hz` is above threshold,
  q_ref/tcp_ref update ages are below thresholds, and q_ref target error stays
  within `--max-reference-drift-deg`

Failure classifications distinguish ACK timeout, deadline-limited,
`async_ack_missing`, `async_overwrite_limited`, `async_drop_limited`,
`reference_supervision_failed`, and `async_servo_loop_blocked`.

## Circle Matrix Stages

After the no-op stage passes, run the circle matrices in order. These matrices
are controller `pgmode` simulation only, keep physical Cartesian blocked, and
exercise the supported 500 Hz command/control path while leaving state
publication as a separate telemetry rate.

| Stage | Matrix | Purpose |
| --- | --- | --- |
| 1 | `configs/rbpodo_circle_ablation/500hz_stage1_noop_and_safe.yaml` | safe 5 cm / 10 s, low gains, 500 Hz supported path |
| 2 | `configs/rbpodo_circle_ablation/500hz_stage2_15cm16s.yaml` | 15 cm / 16 s open-loop and closed-loop candidates |
| 3 | `configs/rbpodo_circle_ablation/500hz_stage3_8s_4s.yaml` | 15 cm / 8 s bridge before 15 cm / 4 s stress |
| async | `configs/rbpodo_circle_ablation/500hz_async_acceptance.yaml` | safe 5 cm / 10 s async socket-send supervised row; ACK-worker row is disabled until no-op ACK-worker evidence is feasible |
| async stage 1 | `configs/rbpodo_circle_ablation/async_500hz_stage1_safe.yaml` | safe 5 cm / 10 s, 500 Hz synchronous ACK-on, 500 Hz socket-send supervised, and disabled ACK-worker candidate |
| async stage 2 | `configs/rbpodo_circle_ablation/async_500hz_stage2_15cm16s.yaml` | 15 cm / 16 s, `Kp_pos/Kp_ori` candidates `0.5/0.2` and `1.0/0.5` across ACK-on and socket-send supervised modes |
| async stage 3 | `configs/rbpodo_circle_ablation/async_500hz_stage3_8s_4s.yaml` | 15 cm / 8 s and 15 cm / 4 s stress, 500 Hz ACK-on best row, socket-send supervised repeat 5, phase advance `0.00/0.02/0.04`, and `t2` `0.08/0.03` with `alpha=0.8` |

For `RBPODO-ASYNC-CIRCLE-MATRIX-01`, use this run order:

1. Complete async no-op evidence for `socket_send_supervised`; enable the
   disabled `sdk_ack_worker` rows only after no-op ACK-worker evidence is
   feasible.
2. Run `async_500hz_stage1_safe.yaml`.
3. Run `async_500hz_stage2_15cm16s.yaml`.
4. Run `async_500hz_stage3_8s_4s.yaml`, stopping before the 4 s rows if the
   8 s bridge shows timing, fault, physical-motion, or reference-supervision
   problems.

Every async circle matrix row writes an artifact-local resolved config with
explicit `servo.rate_hz`, per-arm `servo_t1_sec`, per-arm
`disable_waiting_ack`, `servo.rbpodo_async_streaming.*`, `network.state_pub_rate_hz`,
and per-arm `speed_bar` overrides. Source configs are not edited.

`socket_send_supervised` rows intentionally set `disable_waiting_ack: true`.
Their successful sends are `socket_send_only` evidence, require reference
supervision, and must not be counted as `controller_ack_observed`.

Dry-run each matrix before connecting to controllers:

```bash
RB_ALLOW_REAL_ROBOT=1 \
RB_ALLOW_REAL_MOTION=1 \
RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1 \
RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1 \
RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1 \
python3 scripts/run_rbpodo_circle_ablation.py \
  --matrix configs/rbpodo_circle_ablation/500hz_stage1_noop_and_safe.yaml \
  --artifact-root artifacts/rbpodo_circle_ablation/500hz_stage1_dry_run \
  --server rb_servo_server/build/rbpodo_real_gate/rb_servo_server \
  --dry-run \
  --verify-pgmode-simulation \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation
```

Use the same command shape for stages 2 and 3, changing `--matrix` and
`--artifact-root`. For live runs, remove `--dry-run` only after reviewing the
resolved configs and confirming the controllers are in `pgmode` simulation for
that session.

For async matrices, include the async env gate and ACK-disabled socket-send
gate for dry-runs and live runs that include enabled `socket_send_supervised`
rows:

```bash
RB_ALLOW_RBPODO_ASYNC_STREAMING=1 \
RB_ALLOW_RBPODO_ACK_DISABLED_MOTION=1 \
RB_ALLOW_REAL_ROBOT=1 \
RB_ALLOW_REAL_MOTION=1 \
RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1 \
RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1 \
RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1 \
python3 scripts/run_rbpodo_circle_ablation.py \
  --matrix configs/rbpodo_circle_ablation/async_500hz_stage1_safe.yaml \
  --artifact-root artifacts/rbpodo_circle_ablation/async_500hz_stage1_dry_run \
  --server rb_servo_server/build/rbpodo_real_gate/rb_servo_server \
  --dry-run \
  --verify-pgmode-simulation \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation
```

Stop the staged sequence at the first fault latch, physical-motion warning,
Cartesian gate rejection, missing `q_ref` / `tcp_ref_stand`, poor timing
classification, deadline miss, or measurement reliability downgrade. The 15 cm
/ 4 s rows remain stress evidence even if they complete cleanly; do not mark
500 Hz as real-ready from these matrices.

## Evidence Report

After collecting no-op and circle artifacts, summarize the 500 Hz evidence from
the generated `summary.json`, `ablation_summary.csv`, and ACKON500 report
artifacts. Keep synchronous ACK-on, `socket_send_supervised`, and async
`sdk_ack_worker` rows separated so socket-send-only evidence cannot be reported
as per-command controller ACK.

Acceptance semantics are reported separately:

- `controller_ack_observed`: synchronous ACK-on command calls observed
  per-command controller ACK.
- `sdk_worker_ack_observed`: async `sdk_ack_worker` observed controller ACK in
  the worker thread.
- `socket_send_only`: command write/socket/API send evidence only; it is not
  per-command controller ACK.
- `q_ref_supervised`: controller-reference watchdog evidence was present and OK.

Async telemetry columns include `async_mode`, `commands_enqueued_total`,
`commands_sent_total`, `commands_acked_total`,
`commands_socket_sent_total`, `commands_overwritten_total`,
`commands_dropped_total`, `async_commands_enqueued_total`,
`async_commands_sent_total`, `async_commands_acked_total`,
`official_tracking_window_sec`, `server_servo_tick_count`,
`goal_window_commands_sent`, `goal_window_commands_acked`,
`effective_goal_command_rate_hz`, `ack_coverage_ratio`,
`worker_send_rate_hz`, `worker_sends_outside_official_window`,
`reference_supervision_state`, `q_ref_update_rate_hz`, and
`q_ref_target_error_deg_max`.

Read these fields before interpreting tracking error:

- `controller_ack_observed` / `controller_acceptance_observed_count`: ACK-on
  controller acceptance evidence. This can come from synchronous ACK-on or from
  the ACK worker, depending on the row.
- `socket_send_only`: socket/API write evidence only. It must not be counted
  as per-command controller ACK.
- `q_ref_supervised` and `reference_supervision_state`: whether the q_ref
  watchdog and tcp_ref watchdog saw fresh controller-reference telemetry and no
  target-divergence fault.
- `tcp_ref_stand`: controller-reference lower bound for pgmode simulation
  tracking, not physical TCP tracking. Treat this as the tcp_ref lower bound;
  it is useful only when reference validity and update-rate fields are healthy.
- `diagnostics_suspect`: measurement caveat. Suspect diagnostics can make an
  otherwise completed row useful for debugging, but not for promoting 500 Hz
  defaults or physical-real readiness.

Classifications are deliberately conservative:

- `500hz_async_supervised_pass`: the selected 500 Hz row has supervised ACK or
  q_ref evidence and passes report-stage stability checks. Read the
  `tracking_delta_interpretation` column to distinguish real tracking
  improvement from timing-only improvement.
- `500hz_socket_send_only_promising`: socket-send-only tracking is promising
  with q_ref supervision, but it is not controller ACK evidence and cannot
  promote defaults by itself.
- `500hz_ack_on_blocking_limited`: synchronous ACK-on at 500 Hz appears limited
  by ACK blocking, ACK timeout, or deadline timing.
- `500hz_reference_watchdog_failed`: q_ref/tcp_ref reference supervision failed
  or faulted, so tracking metrics are not accepted.
- `500hz_unstable`: the 500 Hz row faulted, latched safety, detected physical
  motion, missed deadlines, had Cartesian rejection, or was otherwise
  unreliable.
- `insufficient_evidence`: the 100/500 pair, tracking metrics, or measurement
  reliability evidence is incomplete.

Required caveats remain attached to every report:

- `socket_send_only` is not per-command controller ACK
- `tcp_ref_stand` is a controller-reference lower bound, not physical TCP
  tracking
- diagnostics-suspect evidence remains unresolved and cannot promote defaults
- physical real motion is not proven
- dual-arm acceptance is required before any default-rate change

Recommended interpretation:

- Do not change the default rate automatically.
- Allow 500 Hz only as a rbpodo controller-simulation experimental profile.
- Promote only after stable 5 cm / 10 s and 15 cm / 16 s evidence passes with
  usable measurement reliability. The 15 cm / 4 s rows remain stress evidence.

## Troubleshooting

ACK timeout:

- In synchronous ACK-on, classify the row as ACK blocking limited. Widening
  `command_timeout_sec` can diagnose timeout tightness, but it does not fix the
  2 ms servo-tick budget.
- In `sdk_ack_worker`, inspect `ack_timeout_count`, last worker failure,
  `commands_acked_total`, and worker backlog before trying circle stages.

q_ref watchdog miss:

- Treat `reference_supervision_failed`, stale `q_ref_update_age_ms`, stale
  `tcp_ref_update_age_ms`, invalid `q_ref`, or high q_ref target error as a
  failed async row.
- Do not fall back to `tcp_actual_stand` for scoring; pgmode simulation uses
  `tcp_ref_stand` as controller-reference lower-bound evidence.

Worker overwrite/drop high:

- High `commands_overwritten_total`, `commands_dropped_total`, overwrite ratio,
  drop ratio, or `async_worker_backlog_max` means the async path is not keeping
  up with 500 Hz.
- Stop before circle stages if no-op exceeds the configured overwrite/drop
  thresholds.

Physical motion detected:

- Stop immediately. These runs require `operation_mode: simulation`,
  `physical_motion_expected=false`, and `physical_motion_detected=false`.
- Do not retry as a physical-real run. Capture state/preflight artifacts and
  inspect pgmode confirmation, local config, and controller state.

setcap missing:

- `realtime setup failed` or `failed to set realtime priority` usually means
  the exact server binary lacks Linux capabilities.
- Re-run after every rebuild and verify with `getcap`:

  ```bash
  sudo setcap cap_sys_nice,cap_ipc_lock+ep rb_servo_server/build/rbpodo_real_gate/rb_servo_server
  getcap rb_servo_server/build/rbpodo_real_gate/rb_servo_server
  ```
