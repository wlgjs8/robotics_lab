# rbpodo 500 Hz Controller-Simulation Acceptance

This runbook is for staged 500 Hz rbpodo controller `pgmode` simulation only.
It connects to real Rainbow controller boxes, but every config in this track
must keep each robot `operation_mode: simulation` and
`cartesian_control.allow_in_real: false`.

It is not a default-rate change and it is not physical real-motion readiness.

## Stage 0 Evidence

The initial evidence is a single-arm `rainbow_rate_probe.py`
`servo_j_simulation_only` artifact:

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

1. Single-arm no-op rate probe at 100 Hz and 500 Hz: already completed for the
   left controller in `artifacts/rbpodo_servo_j_rate_probe_left`.
2. `rb_servo_server` full-path no-op at 500 Hz using
   `scripts/rbpodo_500hz_acceptance.py --mode servo_j_noop_500hz`.
3. 5 cm / 10 s circle.
4. 15 cm / 16 s circle.
5. 15 cm / 8 s circle.
6. 15 cm / 4 s circle.

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

`network.state_pub_rate_hz` stays at 100 Hz initially. Do not set state
publication to 500 Hz until a separate measurement task accepts it.

## Required Gates

Every controller-simulation command run still requires explicit operator
confirmation. The 500 Hz Servo J no-op stage requires these env gates:

```bash
RB_ALLOW_REAL_ROBOT=1
RB_ALLOW_REAL_MOTION=1
RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1
RB_RBPODO_PGMODE_SIMULATION_CONFIRMED=1
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
      q_ref_target_tolerance_deg: 0.5
      tcp_ref_update_timeout_ms: 50
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
  per arm.
- `socket_send_supervised`: the SDK uses `disable_waiting_ack=true` or an
  equivalent socket-send-only path. Sends must be reported as
  `socket_send_only`, never `controller_ack_observed`. `q_ref` and/or
  `tcp_ref` watchdog supervision is required to infer controller progress.
  Socket send evidence alone is not controller ACK acceptance.

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
`last_async_send_duration_us`, `last_async_ack_duration_us`,
`last_async_acceptance_semantics`, `async_worker_backlog`, and
`async_supervision_state`. A supervision fault latches the servo loop fault
state and suppresses further regular servo sends.

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

## Circle Matrix Stages

After the no-op stage passes, run the circle matrices in order. These matrices
are controller `pgmode` simulation only, keep physical Cartesian blocked, and
compare 100 Hz against 500 Hz with `network.state_pub_rate_hz: 100`.

| Stage | Matrix | Purpose |
| --- | --- | --- |
| 1 | `configs/rbpodo_circle_ablation/500hz_stage1_noop_and_safe.yaml` | safe 5 cm / 10 s, low gains, 100 Hz vs 500 Hz |
| 2 | `configs/rbpodo_circle_ablation/500hz_stage2_15cm16s.yaml` | 15 cm / 16 s open-loop and closed-loop candidates |
| 3 | `configs/rbpodo_circle_ablation/500hz_stage3_8s_4s.yaml` | 15 cm / 8 s bridge before 15 cm / 4 s stress |

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

Stop the staged sequence at the first fault latch, physical-motion warning,
Cartesian gate rejection, missing `q_ref` / `tcp_ref_stand`, poor timing
classification, deadline miss, or measurement reliability downgrade. The 15 cm
/ 4 s rows remain stress evidence even if they complete cleanly; do not mark
500 Hz as real-ready from these matrices.

## Comparison Report

After collecting no-op and circle artifacts, generate the 100 Hz vs 500 Hz
controller-simulation report without rerunning benchmarks:

```bash
python3 scripts/generate_rbpodo_500hz_report.py \
  --noop-summary artifacts/rbpodo_servo_j_rate_probe_left/summary.json \
  --ablation-summary-csv artifacts/<stage1>/ablation_summary.csv \
  --ablation-summary-csv artifacts/<stage2>/ablation_summary.csv \
  --ablation-summary-csv artifacts/<stage3>/ablation_summary.csv \
  --output-md artifacts/rbpodo_500hz_report.md \
  --csv artifacts/rbpodo_500hz_report.csv \
  --json artifacts/rbpodo_500hz_report.json
```

The report compares these evidence pairs:

- 100 Hz vs 500 Hz no-op acceptance
- 100 Hz vs 500 Hz `safe_5cm_10s`
- 100 Hz vs 500 Hz 15 cm / 16 s
- 100 Hz vs 500 Hz 15 cm / 8 s
- 100 Hz vs 500 Hz 15 cm / 4 s

Classifications are deliberately conservative:

- `500hz_noop_pass`: 500 Hz no-op evidence satisfies acceptance-stage report
  checks, but remains single-arm no-op controller-simulation evidence only.
- `500hz_circle_improved`: selected 500 Hz circle evidence has lower RMS error
  without worse p95 error or p99 jitter evidence.
- `500hz_circle_no_improvement`: selected 500 Hz circle evidence is stable but
  does not improve the selected 100 Hz row.
- `500hz_unstable`: the 500 Hz row faulted, latched safety, detected physical
  motion, missed deadlines, had Cartesian rejection, or was otherwise
  unreliable.
- `insufficient_evidence`: the 100/500 pair, tracking metrics, or measurement
  reliability evidence is incomplete.

Required caveats remain attached to every report:

- no-op success does not prove physical real motion
- `tcp_ref_stand` is a controller-reference lower bound, not physical TCP
  tracking
- diagnostics-suspect evidence remains caveated and cannot promote defaults
- dual-arm acceptance is required before any default-rate change

Recommended interpretation:

- Do not change the default rate automatically.
- Allow 500 Hz only as a rbpodo controller-simulation experimental profile.
- Promote only after stable 5 cm / 10 s and 15 cm / 16 s evidence passes with
  usable measurement reliability. The 15 cm / 4 s rows remain stress evidence.
