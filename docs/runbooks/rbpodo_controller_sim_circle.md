# rbpodo Controller-Simulation Circle Runbook

This runbook covers configuration templates and commands for running circle
tracking against real Rainbow controller boxes while the controllers are in
`pgmode` simulation. It does not approve physical robot motion or real
Cartesian motion.

There are three separate benchmark environments:

| Environment | Command path | Measurement path | Physical motion |
| --- | --- | --- | --- |
| `rb_simulator` software simulation | local simulator backends | simulator TCP state | none |
| Rainbow controller `pgmode` simulation via `rbpodo` | real controller boxes, `operation_mode: simulation` | rbpodo controller reference telemetry, normally `tcp_ref_stand` | `physical_motion_expected=false` |
| future physical real robot | real controller boxes, physical motion acceptance | physical `tcp_actual_stand` / measured state | not covered by this runbook |

Do not mix the artifact categories. A good `rb_simulator` run is not evidence
that the Rainbow controller path works, and a good controller-simulation run is
not approval for physical Cartesian motion.

## Scope

Use these templates only for `rbpodo` controller-simulation bring-up:

| Purpose | Template | Rate | Command/state endpoints | Motion default |
| --- | --- | ---: | --- | --- |
| read-only diagnostic | `rb_servo_server/config/dual_real_rbpodo_readonly.example.yaml` | 100 Hz | `50031` / `50131` | disabled |
| Servo J no-op ACK-on | `rb_servo_server/config/dual_real_rbpodo_sim_noop_100hz_ack.example.yaml` | 100 Hz | `50041` / `50141` | controller pgmode simulation only |
| Servo J no-op ACK-on | `rb_servo_server/config/dual_real_rbpodo_sim_noop_200hz_ack.example.yaml` | 200 Hz | `50042` / `50142` | controller pgmode simulation only |
| Servo J no-op ACK-off | `rb_servo_server/config/dual_real_rbpodo_sim_noop_200hz_no_ack.example.yaml` | 200 Hz | `50043` / `50143` | controller pgmode simulation only, experimental |
| stable circle | `rb_servo_server/config/dual_real_rbpodo_circle_15cm16s.example.yaml` | 100 Hz | `50051` / `50151` | controller pgmode simulation only |
| GENE-style stress circle | `rb_servo_server/config/dual_real_rbpodo_circle_15cm4s.example.yaml` | 100 Hz | `50052` / `50152` | controller pgmode simulation only, stress |

Copy a template to `rb_servo_server/config/local/` and edit the copy for the
site. Treat `local/*.yaml` as operator-owned working files, not production
templates. A clean checkout may have no local YAMLs, and an older checkout may
have stale local copies; regenerate them before running the benchmark.

Create or refresh the circle local configs with:

```bash
tools/create_rbpodo_circle_local_configs.sh
```

The helper refuses to overwrite existing local files unless `--force` is
passed:

```bash
tools/create_rbpodo_circle_local_configs.sh --force
```

Verify the controller-simulation Cartesian gate and physical-real block before
running:

```bash
grep -H "allow_in_controller_simulation: true" rb_servo_server/config/local/dual_real_rbpodo_circle_15cm*.yaml
grep -H "allow_in_real: false" rb_servo_server/config/local/dual_real_rbpodo_circle_15cm*.yaml
grep -H "operation_mode: simulation" rb_servo_server/config/local/dual_real_rbpodo_circle_15cm*.yaml
```

The required shape for rbpodo controller simulation is:

```yaml
left_robot:
  backend_type: rbpodo
  run_mode: real
  operation_mode: simulation

cartesian_control:
  allow_in_controller_simulation: true
  allow_in_real: false
```

`run_mode: real` means the server is connecting to real Rainbow controller
boxes. `operation_mode: simulation` means the controller command path must be
Rainbow `pgmode` simulation, so the physical robot should not move.

## Safety Model

These configs connect to real controller IPs:

- left: `172.28.60.200`
- right: `172.28.60.201`

The controllers must be in `pgmode` simulation before any template with
`send_servo_commands: true` is used. The physical robot must not move.

`rb_simulator` software simulation and Rainbow controller `pgmode` simulation
are different systems. The configs in this runbook connect to real Rainbow
controller boxes over the rbpodo backend, then require those controller boxes to
report simulation mode. They are not hardware-free simulator configs.

Every motion-capable template is labeled:

```text
controller pgmode simulation only; physical robot must not move; requires env gates and confirmation.
```

The YAML files do not set environment variables. The acceptance scripts still
require the normal real-controller and motion environment gates plus explicit
confirmation flags. ACK-off additionally requires
`RB_ALLOW_RBPODO_ACK_DISABLED_MOTION=1` and is socket-send evidence, not
controller-acceptance evidence.

Every controller-simulation circle run requires these env gates:

```bash
RB_ALLOW_REAL_ROBOT=1
RB_ALLOW_REAL_MOTION=1
RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1
RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1
RB_RBPODO_PGMODE_SIMULATION_CONFIRMED=1
```

The benchmark runner normally sets `RB_RBPODO_PGMODE_SIMULATION_CONFIRMED=1`
only for the launched server process after the same run sets or verifies
pgmode simulation. Set it manually only when you are starting the server
outside the benchmark and have current controller-simulation evidence.

A temporary diagnostics-suspect override exists only for known suspicious
rbpodo status-field layouts while the controller is in `pgmode` simulation. It
requires all normal gates plus YAML
`servo.allow_controller_simulation_diagnostics_suspect: true` and:

```bash
RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1
```

It must not be used for non-finite joints, unresolved range violations, explicit
SOS/E-stop/soft E-stop/collision faults, stale state, lease failures, or
`operation_mode: real`.

An operator must still be present with E-stop access. `pgmode` simulation is
required so the physical robot should not move, but it is not a substitute for
supervised real-controller bring-up discipline.

Cartesian real motion remains disabled:

```yaml
cartesian_control:
  allow_in_real: false
```

The controller-simulation circle templates must opt into the narrow streaming
Cartesian carve-out:

```yaml
cartesian_control:
  enable: true
  allow_in_real: false
  allow_in_controller_simulation: true
```

This carve-out applies only when the server connects to an `rbpodo` backend with
`run_mode: real` and robot `operation_mode: simulation`, the same run has
confirmed Rainbow `pgmode` simulation, and all controller-simulation env gates
are present. It does not make `TcpPoseTarget` or physical real Cartesian
execution available. State JSON exposes the decision under per-arm
`cartesian_gate`, including `physical_motion_expected=false` when the carve-out
is active. If the gate is closed, the per-arm `cartesian_solve.reason` should
report a reason such as `cartesian_control_unavailable_controller_sim_env`,
`cartesian_control_unavailable_controller_sim_config`,
`cartesian_control_unavailable_backend`,
`cartesian_control_unavailable_operation_mode`, or
`cartesian_control_unavailable_physical_real_blocked`.

## One-Time Setup Commands

Build the rbpodo-enabled server on a machine with the rbpodo SDK available:

```bash
cmake -S rb_servo_server -B rb_servo_server/build/rbpodo_real_gate \
  -DCMAKE_BUILD_TYPE=Release \
  -DRB_SERVO_ENABLE_RBPODO=ON \
  -DBUILD_TESTING=ON
cmake --build rb_servo_server/build/rbpodo_real_gate -j
```

If the server fails with realtime scheduling errors, grant the binary the
needed Linux capabilities and verify them:

```bash
sudo setcap cap_sys_nice,cap_ipc_lock+ep rb_servo_server/build/rbpodo_real_gate/rb_servo_server
getcap rb_servo_server/build/rbpodo_real_gate/rb_servo_server
```

Create operator-local circle configs from the tracked templates:

```bash
tools/create_rbpodo_circle_local_configs.sh
```

Use `--force` only when you intentionally want to refresh stale local copies
after reviewing local edits:

```bash
tools/create_rbpodo_circle_local_configs.sh --force
```

Put both controllers into Rainbow `pgmode` simulation before a benchmark. This
wrapper sends only `pgmode simulation`; it does not set real mode, reset faults,
change collision thresholds, enable servo power, or send motion:

```bash
RB_ALLOW_REAL_ROBOT=1 \
tools/simulation_mode.sh \
  --summary-json artifacts/rbpodo_controller_sim_circle/pgmode_simulation.json \
  --i-understand-this-connects-to-real-controller
```

For a read-only check that does not send `pgmode simulation`, use:

```bash
RB_ALLOW_REAL_ROBOT=1 \
tools/simulation_mode.sh \
  --verify-only \
  --summary-json artifacts/rbpodo_controller_sim_circle/pgmode_verify.json \
  --i-understand-this-connects-to-real-controller
```

## Diagnostic State First

First set or verify controller pgmode simulation. This sends only
`pgmode simulation`; it must not set real mode, reset faults, change collision
thresholds, enable servo power, or send motion:

```bash
RB_ALLOW_REAL_ROBOT=1 \
python3 scripts/rainbow_pgmode.py \
  --ips 172.28.60.200 172.28.60.201 \
  --set-simulation \
  --summary-json artifacts/rbpodo_controller_sim_circle/pgmode_simulation.json \
  --i-understand-this-connects-to-real-controller
```

To verify without sending `pgmode simulation`, use:

```bash
RB_ALLOW_REAL_ROBOT=1 \
python3 scripts/rainbow_pgmode.py \
  --ips 172.28.60.200 172.28.60.201 \
  --verify-only \
  --summary-json artifacts/rbpodo_controller_sim_circle/pgmode_verify.json \
  --i-understand-this-connects-to-real-controller
```

Then capture a read-only state dump:

```bash
RB_ALLOW_REAL_ROBOT=1 \
python3 scripts/rbpodo_state_dump.py \
  --ips 172.28.60.200 172.28.60.201 \
  --q-min=-170,-120,-170,-190,-120,-360 \
  --q-max=170,120,170,190,120,360 \
  --wrap-period-deg=0,0,0,360,0,360 \
  --output artifacts/rbpodo_controller_sim_circle/state_dump.json \
  --pretty \
  --i-understand-this-connects-to-real-controller
```

Then run read-only acceptance with a local copy of
`dual_real_rbpodo_readonly.example.yaml`. Read-only diagnostic startup may
publish faulted or wrong-mode state after valid joint acquisition; it does not
mark the system motion-ready.

## Servo J No-Op

Servo J no-op is the first controller-simulation command check. The target must
equal current `q_actual` or `q_ref` within the acceptance script tolerance.
The acceptance tool refuses controller-simulation motion modes unless the same
tool run uses `--set-pgmode-simulation` or `--verify-pgmode-simulation` and the
config operation mode is `simulation`.

Example after copying the template to local config:

```bash
RB_ALLOW_REAL_ROBOT=1 \
RB_ALLOW_REAL_MOTION=1 \
RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1 \
python3 scripts/rbpodo_servo_acceptance.py \
  --server rb_servo_server/build/rbpodo_real_gate/rb_servo_server \
  --config rb_servo_server/config/local/dual_real_rbpodo_sim_noop_200hz_ack.yaml \
  --arm left \
  --mode servo_j_noop \
  --profile 200hz_ack \
  --allow-motion \
  --set-pgmode-simulation \
  --duration-sec 10 \
  --artifact-dir artifacts/rbpodo_controller_sim_circle/servo_j_noop_200hz_ack_left \
  --i-understand-this-connects-to-real-controller
```

If the controller reports a robot fault, servo-disabled state, or wrong
operation mode, stop and inspect the state dump. Do not clear faults or change
controller mode from these scripts.

## Circle Profiles

The stable profile is `15cm/16s`. It mirrors the current simulator baseline:

- `servo.rate_hz: 100`
- `velocity_target_integration: previous_command`
- `path_kp_pos: 6.0`
- `path_kp_ori: 6.0`
- `max_twist_linear_m_s: 0.03`
- `max_twist_angular_rad_s: 0.2`

The stress profile is `15cm/4s`. It mirrors the GENE-style simulator stress
settings and is not a pass/fail acceptance profile:

- `servo.rate_hz: 100`
- `max_twist_linear_m_s: 0.15`
- `max_twist_angular_rad_s: 0.4`
- feedback benchmark mode is recommended before interpreting drift

For controller pgmode simulation, the correct measurement target is normally
the controller reference path rather than physical TCP motion. The state stream
publishes both:

- `tcp_actual_stand`: FK from measured `q_actual_deg`
- `tcp_ref_stand`: FK from rbpodo controller reference joints (`jnt_ref`)

Legacy `tcp_stand` remains an actual-pose alias. When the per-arm
`controller_simulation_mode.recommended_tracking_pose` is `tcp_ref_stand`,
circle reports should compare the desired circle trajectory against
`tcp_ref_stand`. If `tcp_ref_valid` is false, report the benchmark as missing
reference telemetry instead of silently treating stationary physical
`tcp_actual_stand` as controller tracking failure.

Physical real circle testing is a separate future task. These templates are not
real-ready and must not be copied into a physical Cartesian acceptance run.

## Circle Benchmark Runner

`scripts/rbpodo_circle_tracking_benchmark.py` runs the controller-simulation
circle benchmark through the rbpodo path:

```text
benchmark script -> UDP CommandServer -> rb_servo_server -> RbpodoBackend
-> Rainbow controller in pgmode simulation -> rbpodo data channel -> state stream
```

It is not the hardware-free `rb_simulator` benchmark. It connects to real
controller boxes, requires pgmode simulation confirmation, and refuses
`operation_mode: real`.

Stable 15 cm / 16 s example:

```bash
RB_ALLOW_REAL_ROBOT=1 \
RB_ALLOW_REAL_MOTION=1 \
RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1 \
RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1 \
python3 scripts/rbpodo_circle_tracking_benchmark.py \
  --server rb_servo_server/build/rbpodo_real_gate/rb_servo_server \
  --server-config rb_servo_server/config/local/dual_real_rbpodo_circle_15cm16s.yaml \
  --arm left \
  --controller twist_stand \
  --profile circle_15cm_16s \
  --repeat 3 \
  --command-rate-hz 100 \
  --tracking-source tcp_ref_stand \
  --set-pgmode-simulation \
  --artifact-dir artifacts/rbpodo_circle/circle_15cm16s_twist_stand_left \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation
```

GENE-style 15 cm / 4 s stress example:

```bash
RB_ALLOW_REAL_ROBOT=1 \
RB_ALLOW_REAL_MOTION=1 \
RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1 \
RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1 \
python3 scripts/rbpodo_circle_tracking_benchmark.py \
  --server rb_servo_server/build/rbpodo_real_gate/rb_servo_server \
  --server-config rb_servo_server/config/local/dual_real_rbpodo_circle_15cm4s.yaml \
  --arm left \
  --controller twist_stand_feedback \
  --profile gene_15cm_4s \
  --repeat 5 \
  --command-rate-hz 100 \
  --feedback-kp-pos 2.0 \
  --feedback-kp-ori 2.0 \
  --feedback-max-linear-m-s 0.15 \
  --feedback-max-angular-rad-s 0.4 \
  --tracking-source tcp_ref_stand \
  --verify-pgmode-simulation \
  --artifact-dir artifacts/rbpodo_circle/gene_15cm4s_feedback_left \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation
```

If the state dump and local config show that the temporary controller-
simulation diagnostics bridge is required, prepend
`RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1` to the same command.
Do not use that env var outside this pgmode simulation workflow.

If ACK waiting is disabled in a local copy, also set:

```bash
RB_ALLOW_RBPODO_ACK_DISABLED_MOTION=1
```

If the local config explicitly opts into the temporary diagnostics-suspect
bridge, also set:

```bash
RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1
```

The runner passes `RB_RBPODO_PGMODE_SIMULATION_CONFIRMED=1` only to the
launched server process after pgmode simulation is set or verified in the same
run, or after a supplied `--pgmode-summary-json` confirms simulation for the
configured controller IPs. It does not set any `RB_ALLOW_*` variable.

If the run reports `server_rejected_cartesian: true` or
`result_reason: cartesian_commands_rejected_by_server`, do not interpret the
artifact as a tracking result. This means the server stayed in `ArmedHold` and
rejected Cartesian command generation before attempting the path. Servo J ACKs
can still be observed in that state because the server may keep sending the
previous hold target; ACKs alone are not circle motion evidence.

Check these fields first:

- `cartesian_unavailable_count`
- `cartesian_unavailable_reason_counts`
- `armed_hold_count`
- `command_accepted_but_target_static`
- `q_ref_moved`
- `tcp_ref_moved`
- `max_command_actual_error_deg_observed`

The usual fix is to regenerate or edit the local config and rerun with the
controller-simulation Cartesian gate:

```bash
tools/create_rbpodo_circle_local_configs.sh --force
grep -H "allow_in_controller_simulation: true" rb_servo_server/config/local/dual_real_rbpodo_circle_15cm*.yaml
```

Then verify the run environment includes:

```bash
RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1
```

Also confirm `operation_mode: simulation` and same-run pgmode simulation
confirmation. Do not set `RB_ALLOW_REAL_CARTESIAN` for this workflow.

## Troubleshooting

`realtime setup failed` or `failed to set realtime priority` means the server
could not acquire its requested scheduler privileges. Rebuild if needed, then
apply Linux capabilities to the exact binary you pass with `--server`:

```bash
sudo setcap cap_sys_nice,cap_ipc_lock+ep rb_servo_server/build/rbpodo_real_gate/rb_servo_server
getcap rb_servo_server/build/rbpodo_real_gate/rb_servo_server
```

If startup reports `diagnostics_suspect`, inspect controller state before
running any command benchmark:

```bash
RB_ALLOW_REAL_ROBOT=1 \
python3 scripts/rbpodo_state_dump.py \
  --ips 172.28.60.200 172.28.60.201 \
  --q-min=-170,-120,-170,-190,-120,-360 \
  --q-max=170,120,170,190,120,360 \
  --wrap-period-deg=0,0,0,360,0,360 \
  --output artifacts/rbpodo_controller_sim_circle/state_dump.json \
  --pretty \
  --i-understand-this-connects-to-real-controller
```

Use `RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1` only for this
controller-simulation workflow, only with `operation_mode: simulation`, and
only when the YAML explicitly allows the temporary diagnostics bridge. Do not
use that override for physical real motion.

`cartesian_control_unavailable` means the server rejected Cartesian command
generation before attempting the circle. Check:

```bash
grep -H "allow_in_controller_simulation: true" rb_servo_server/config/local/dual_real_rbpodo_circle_15cm*.yaml
grep -H "allow_in_real: false" rb_servo_server/config/local/dual_real_rbpodo_circle_15cm*.yaml
grep -H "operation_mode: simulation" rb_servo_server/config/local/dual_real_rbpodo_circle_15cm*.yaml
env | grep '^RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN='
```

If `circle_fit_reason` is `singular`, first determine whether the reference
trajectory moved. A singular fit with high
`cartesian_unavailable_count`, `motion_state=ArmedHold`, `q_ref_moved=false`,
or `tcp_ref_moved=false` is a blocked command path, not poor controller
tracking.

Servo J ACKs do not imply the circle executed. In a blocked Cartesian run the
server can keep sending the previous hold target and still observe ACKs. Treat
the artifact as tracking evidence only when the summary shows Cartesian
commands were accepted and `tcp_ref_stand` moved.

## Ablation Matrix Runner

`scripts/run_rbpodo_circle_ablation.py` runs a rbpodo-only matrix of
controller-simulation circle benchmarks. It is separate from the simulator-only
`scripts/run_circle_ablation.py` and refuses configs whose robot
`operation_mode` is not `simulation`.

The runner does not run by default from gates. It requires explicit
controller-simulation env gates, real-controller confirmation, and either
`--set-pgmode-simulation`, `--verify-pgmode-simulation`, or a verified
`--pgmode-summary-json`:

```bash
RB_ALLOW_REAL_ROBOT=1 \
RB_ALLOW_REAL_MOTION=1 \
RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1 \
RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1 \
python3 scripts/run_rbpodo_circle_ablation.py \
  --matrix configs/rbpodo_circle_ablation/rbpodo_circle_ablation_example.yaml \
  --artifact-root artifacts/rbpodo_circle/ablation_gene15cm4s_left \
  --server rb_servo_server/build/rbpodo_real_gate/rb_servo_server \
  --verify-pgmode-simulation \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation
```

Use `--dry-run` first to validate matrix/config shape and print the exact
benchmark commands. Dry-run still checks that the runner has been given the
same explicit safety flags and env gates required for a real matrix run.

The matrix supports the intended factor split:

- `profile`: `circle_15cm_16s` or `gene_15cm_4s`
- `controller`: `twist_stand`, `twist_local`,
  `twist_stand_feedback`, or `twist_local_feedback`
- `command_rate_hz`, `repeat`, `tracking_source`, feedback gains, feedback
  speed limits, and explicit extra env requirements
- ACK mode is derived from the referenced config's
  `disable_waiting_ack` fields

ACK-off experiments require `RB_ALLOW_RBPODO_ACK_DISABLED_MOTION=1`.
Configs that opt into the temporary diagnostics-suspect bridge require
`RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1`. The runner stops on
safety preflight failure or child benchmark `result: blocked` / `result: error`.

Each matrix run writes:

- one artifact subdirectory per enabled experiment
- `experiment_command.txt` and `ablation_command.json` per experiment
- `matrix_resolved.json`
- `ablation_summary.csv`
- `ablation_summary.json`
- `ablation_report.md`
- summary plots for RMS error, p95 error, radius gain, latency, q_ref update
  rate, and physical-motion detection when the metrics are available

## Reporting And Decision Policy

Use `scripts/generate_rbpodo_circle_report.py` for rbpodo
controller-simulation evidence. The general
`scripts/generate_circle_benchmark_report.py` can report mixed inputs, but the
rbpodo wrapper makes the pgmode-simulation intent explicit:

For a quick side-by-side table:

```bash
python3 scripts/compare_circle_benchmarks.py \
  artifacts/rbpodo_circle/circle_15cm16s_twist_stand_left/summary.json \
  artifacts/rbpodo_circle/gene_15cm4s_feedback_left/summary.json \
  --csv artifacts/rbpodo_circle/rbpodo_circle_compare.csv
```

```bash
python3 scripts/generate_rbpodo_circle_report.py \
  artifacts/rbpodo_circle/circle_15cm16s_twist_stand_left/summary.json \
  artifacts/rbpodo_circle/gene_15cm4s_feedback_left/summary.json \
  --output-md artifacts/rbpodo_circle/rbpodo_circle_report.md \
  --csv artifacts/rbpodo_circle/rbpodo_circle_report.csv
```

For an ablation run:

```bash
python3 scripts/generate_rbpodo_circle_report.py \
  --ablation-summary-csv artifacts/rbpodo_circle/ablation_gene15cm4s_left/ablation_summary.csv \
  --output-md artifacts/rbpodo_circle/ablation_gene15cm4s_left/rbpodo_circle_report.md \
  --csv artifacts/rbpodo_circle/ablation_gene15cm4s_left/rbpodo_circle_report.csv
```

The report keeps categories separate:

- `rb_simulator`: hardware-free simulator evidence from
  `scripts/circle_tracking_benchmark.py`
- `rbpodo_controller_simulation`: real Rainbow controller boxes in
  `pgmode` simulation, measured through rbpodo telemetry
- `real_physical_benchmark`: future physical-motion evidence; not run by this
  workflow

Every rbpodo controller-simulation row must state:

- `backend: rbpodo`
- `controller_mode: pgmode_simulation`
- `tracking_source`, normally `tcp_ref_stand`
- `physical_motion_expected: false`
- `physical_motion_detected`
- `q_ref_update_rate_hz` and `q_actual_update_rate_hz`
- `ack_policy`
- `controller_acceptance_observed_count`

Stable controller-simulation baseline criteria:

- profile `circle_15cm_16s`
- `tracking_source == tcp_ref_stand`
- `radius_gain` in `[0.98, 1.02]`
- `rms_error_m <= 0.005`
- `p95_error_m <= 0.007`
- `physical_motion_detected == false`
- `fault_latched == false`
- command/source drops, timeouts, and controller rejections are zero when
  reported
- ACK-on runs have `controller_acceptance_observed_count > 0`

GENE-style stress candidate criteria:

- profile `gene_15cm_4s`
- `radius_gain` in `[0.90, 1.10]`
- RMS and p95 errors are recorded
- feedback saturation is recorded for feedback controllers
- `physical_motion_detected == false`
- `fault_latched == false`

Even a good rbpodo controller-simulation report is not real physical tracking
evidence. It can guide future low-speed parameter selection, but speed, gains,
and ACK-off behavior must not be copied directly into physical motion.

## Tracking Source

The default `--tracking-source auto` selects `tcp_ref_stand`. This is the
controller reference TCP computed from rbpodo controller reference joints, and
it is the recommended tracking source in pgmode simulation. If `tcp_ref_stand`
is missing or invalid, the run fails rather than silently scoring stationary
physical `tcp_actual_stand`.

`--tracking-source tcp_actual_stand` is available only for explicit diagnostic
comparison. Reports using it must state that physical `q_actual` is not the
controller-simulation tracking source.

The runner writes both paths when available:

- `desired_reference.csv`: benchmark desired circle
- `controller_reference_actual.csv`: `tcp_ref_stand`
- `physical_actual.csv`: `tcp_actual_stand`

It also records `tcp_ref_valid_ratio`, `tcp_actual_valid_ratio`,
`q_ref_update_rate_hz`, `q_actual_update_rate_hz`, and
`physical_motion_detected`. Any physical `q_actual` drift above the configured
warning threshold is a warning in pgmode simulation, not a success criterion.

## Artifacts

Each run writes:

- `summary.json` and `summary.csv`
- `safety_preflight.json`
- `pgmode_summary.json`
- `raw_config.yaml`
- `state_stream.jsonl`
- `command_packets.jsonl`
- `desired_reference.csv`
- `controller_reference_actual.csv`
- `physical_actual.csv`, if available
- `samples.csv`
- `rb_servo_server.log`
- `circle_trajectory_controller_reference.png`
- `circle_trajectory_physical_actual.png`, if available
- `tracking_error_time.png`
- `orientation_drift_time.png`
- `phase_lag_time.png`, when reliable
- `axis_positions_time.png`

Result semantics match the simulator benchmark where possible: `completed`
means the run finished without thresholds, `pass`/`fail` require explicit
threshold flags, `blocked` means server-side gates prevented the requested
Cartesian path from being attempted, and `error` means safety preflight or
execution failed.
