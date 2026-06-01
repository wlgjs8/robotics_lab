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
2. `rb_servo_server` dual-arm no-op at 500 Hz.
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
confirmation and env gates:

```bash
RB_ALLOW_REAL_ROBOT=1
RB_ALLOW_REAL_MOTION=1
RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1
RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1
RB_RBPODO_PGMODE_SIMULATION_CONFIRMED=1
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
