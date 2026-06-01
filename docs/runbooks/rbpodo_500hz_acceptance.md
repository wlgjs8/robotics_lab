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
  --duration-sec 10 \
  --artifact-dir artifacts/rbpodo_500hz_acceptance/left_noop \
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
