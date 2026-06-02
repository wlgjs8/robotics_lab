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

## Result Contract

Every new circle summary separates these result surfaces:

- `run_result`: execution status (`completed`, `faulted`, `startup_fault`,
  `blocked`, or `error`).
- `safety_result`: fault latch, physical-motion detection, Cartesian
  availability, and pass/fail safety status.
- `benchmark_threshold_result`: generic benchmark threshold pass/fail, such as
  max error, max orientation drift, or latency thresholds.
- `ackon500_goal_result`: official ACKON500 `gene_15cm_4s` pass/fail when the
  row is applicable.
- `diagnostic_warnings`: non-fatal labels such as
  `max_orientation_drift_spike`, `diagnostics_suspect_override_active`,
  `controller_reference_lower_bound`, `max_error_spike`, and `timing_spike`.

Reports should prefer those structured fields over the legacy top-level
`result`. For new circle artifacts, the top-level `result` mirrors
`run_result.status`; a completed run with a generic threshold failure remains a
completed run. The official ACKON500 orientation criterion remains
`p95_orientation_drift_rad <= 0.02 rad`; `max_orientation_drift_rad` stays
visible as a diagnostic unless `GOAL.md` explicitly promotes it to a goal
criterion. Socket-send-only rows are never official ACK-ON passes.

Every benchmark summary, ablation row, and report row also carries canonical
lane metadata:

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

For rbpodo controller simulation, the current lanes are:

| Lane | Meaning |
| --- | --- |
| `rbpodo_python_streaming_open_loop` | Python benchmark streams open-loop twist/segment commands. |
| `rbpodo_python_streaming_feedback` | Python benchmark closes the feedback loop from state telemetry. |
| `rbpodo_server_side_circle_ackon100` | Server-side circle tracking at the 100 Hz ACK-on baseline. |
| `rbpodo_server_side_circle_ackon500_sync` | Server-side circle tracking at 500 Hz with synchronous ACK-on sends. |
| `rbpodo_server_side_circle_ackon500_sdk_worker` | Official ACKON500 pass lane: server-side circle, 500 Hz, async `sdk_ack_worker`, worker ACK observed. |
| `rbpodo_server_side_circle_500hz_socket_send_supervised` | Server-side circle with `socket_send_supervised`; send-only evidence, never an official ACKON500 pass. |
| `future_physical_real_unavailable` | Placeholder for future physical-real evidence; this runbook does not create it. |

`socket_send_supervised` must not be grouped with ACKON500 pass evidence.
Python streaming feedback remains a distinct lane even when its metrics are
good enough to guide future policy-runner or IL work.

## Scope

Use these templates only for `rbpodo` controller-simulation bring-up:

| Purpose | Template | Rate | Command/state endpoints | Motion default |
| --- | --- | ---: | --- | --- |
| read-only diagnostic | `rb_servo_server/config/dual_real_rbpodo_readonly.example.yaml` | 100 Hz | `50031` / `50131` | disabled |
| Servo J no-op ACK-on | `rb_servo_server/config/dual_real_rbpodo_sim_noop_100hz_ack.example.yaml` | 100 Hz | `50041` / `50141` | controller pgmode simulation only |
| Servo J no-op ACK-on | `rb_servo_server/config/dual_real_rbpodo_sim_noop_200hz_ack.example.yaml` | 200 Hz | `50042` / `50142` | controller pgmode simulation only |
| Servo J no-op ACK-off | `rb_servo_server/config/dual_real_rbpodo_sim_noop_200hz_no_ack.example.yaml` | 200 Hz | `50043` / `50143` | controller pgmode simulation only, experimental |
| stable circle | `rb_servo_server/config/dual_real_rbpodo_circle_15cm16s.example.yaml` | 100 Hz | command `50051`, state `50151` recorder + `50161` GUI | controller pgmode simulation only |
| GENE-style stress circle | `rb_servo_server/config/dual_real_rbpodo_circle_15cm4s.example.yaml` | 100 Hz | command `50052`, state `50152` recorder + `50162` GUI | controller pgmode simulation only, stress |
| safe 500 Hz circle | `rb_servo_server/config/dual_real_rbpodo_circle_5cm10s_500hz.example.yaml` | 500 Hz | command `50251`, state `50351` recorder + `50361` GUI | controller pgmode simulation only, staged |
| stable 500 Hz circle | `rb_servo_server/config/dual_real_rbpodo_circle_15cm16s_500hz.example.yaml` | 500 Hz | command `50252`, state `50352` recorder + `50362` GUI | controller pgmode simulation only, staged |
| middle 500 Hz circle | `rb_servo_server/config/dual_real_rbpodo_circle_15cm8s_500hz.example.yaml` | 500 Hz | command `50253`, state `50353` recorder + `50363` GUI | controller pgmode simulation only, staged |
| stress 500 Hz circle | `rb_servo_server/config/dual_real_rbpodo_circle_15cm4s_500hz.example.yaml` | 500 Hz | command `50254`, state `50354` recorder + `50364` GUI | controller pgmode simulation only, stress |

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

Create staged 500 Hz local configs only when you intend to run the 500 Hz
acceptance track:

```bash
tools/create_rbpodo_circle_local_configs.sh --include-500hz
```

The 500 Hz configs are not created by default and do not change the existing
100 Hz circle defaults.

Verify the controller-simulation Cartesian gate and physical-real block before
running:

```bash
grep -H "allow_in_controller_simulation: true" rb_servo_server/config/local/dual_real_rbpodo_circle_15cm*.yaml
grep -H "allow_in_real: false" rb_servo_server/config/local/dual_real_rbpodo_circle_15cm*.yaml
grep -H "operation_mode: simulation" rb_servo_server/config/local/dual_real_rbpodo_circle_15cm*.yaml
grep -H "controller_simulation_servo_state_source: reference" rb_servo_server/config/local/dual_real_rbpodo_circle_15cm*.yaml
grep -H "controller_simulation_divergence_source: reference" rb_servo_server/config/local/dual_real_rbpodo_circle_15cm*.yaml
```

For 500 Hz local configs, use the `_500hz` glob and also verify `servo_t1_sec`
matches the 2 ms command period:

```bash
grep -H "servo_t1_sec: 0.002" rb_servo_server/config/local/*_500hz.yaml
grep -H "allow_in_real: false" rb_servo_server/config/local/*_500hz.yaml
grep -H "operation_mode: simulation" rb_servo_server/config/local/*_500hz.yaml
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
  controller_simulation_servo_state_source: reference
  controller_simulation_divergence_source: reference
```

`run_mode: real` means the server is connecting to real Rainbow controller
boxes. `operation_mode: simulation` means the controller command path must be
Rainbow `pgmode` simulation, so the physical robot should not move.

The circle templates use server-side state fanout:

```yaml
network:
  command_bind: "udp://127.0.0.1:50051"
  state_pub_endpoints:
    - "udp://127.0.0.1:50151"  # benchmark recorder
    - "udp://127.0.0.1:50161"  # rb_gui live viewer
```

`network.state_pub_endpoints` is the canonical multi-consumer field. The
legacy single `network.state_pub_endpoint` remains accepted for one consumer,
but do not combine it with `state_pub_endpoints`. The first list entry is still
mirrored into `state_pub_endpoint` internally for older tooling.

Live visualization uses two independent UDP streams:

```text
rb_servo_server state fanout
  udp://127.0.0.1:50151 -> benchmark recorder
  udp://127.0.0.1:50161 -> rb_gui state receiver

rbpodo_circle_tracking_benchmark overlay
  udp://127.0.0.1:50261 -> rb_gui circle overlay receiver
```

State fanout carries robot/server telemetry: `tcp_actual_stand`,
`tcp_ref_stand`, safety verdicts, Cartesian gate reasons, and
`physical_motion_expected=false` when the controller-simulation carve-out is
active. The overlay carries desired circle geometry and live metrics owned by
the benchmark. Do not put overlay traffic in `state_pub_endpoints`.

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

Rebuilding or replacing `rb_servo_server/build/rbpodo_real_gate/rb_servo_server`
removes these capabilities. Re-run `setcap` after every rebuild before
starting a controller-simulation benchmark.

## Convenience Wrapper Workflow

The manual commands below remain the source-of-truth sequence, but these
wrappers cover the common live GUI plus benchmark workflow with shorter
commands. They do not weaken safety gates.

Prepare local configs, optionally build the rbpodo server, apply realtime
capabilities, check controller command/data TCP ports, and set controller
`pgmode` simulation:

```bash
RB_ALLOW_REAL_ROBOT=1 \
tools/rbpodo_circle_prepare.sh \
  --create-local-configs \
  --build \
  --setcap \
  --check-ports \
  --set-pgmode-simulation \
  --i-understand-this-connects-to-real-controller
```

Use `--force-local-configs` only when you intentionally want to overwrite
operator-local configs after reviewing local edits. Rebuilding the server
removes Linux capabilities, so rerun `tools/rbpodo_circle_prepare.sh --setcap`
after every rebuild. If you want to verify `pgmode` without sending
`pgmode simulation`, replace `--set-pgmode-simulation` with
`--verify-pgmode-simulation`.

Start the GUI for the stable profile:

```bash
tools/rbpodo_circle_gui.sh --profile stable
```

Start the GUI for the GENE-style stress profile:

```bash
tools/rbpodo_circle_gui.sh --profile gene
```

The GUI wrapper binds the server fanout state port and the benchmark overlay
port. It only receives telemetry and does not send robot commands. It refuses
to start if another local process is already listening on the selected UDP
ports unless `--force` is passed.

Run the stable 15 cm / 16 s controller-simulation benchmark:

```bash
tools/rbpodo_circle_benchmark.sh \
  --profile stable \
  --arm left \
  --with-required-env \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation
```

Run the GENE-style 15 cm / 4 s stress benchmark:

```bash
tools/rbpodo_circle_benchmark.sh \
  --profile gene \
  --arm left \
  --with-required-env \
  --feedback-kp-pos 2.0 \
  --feedback-kp-ori 2.0 \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation
```

`--with-required-env` is an explicit opt-in that exports:

```bash
RB_ALLOW_REAL_ROBOT=1
RB_ALLOW_REAL_MOTION=1
RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1
RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1
RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1
```

If `--with-required-env` is omitted, the benchmark wrapper prints any missing
env vars and exits. The wrapper still requires both confirmation flags. It
defaults to `--set-pgmode-simulation`; pass `--verify-pgmode-simulation` when
you want a verify-only pgmode check.

Before launching the benchmark, the wrapper checks the local config and refuses
stale or dangerous settings:

- `cartesian_control.allow_in_real: true`
- missing `cartesian_control.allow_in_controller_simulation: true`
- missing `network.state_pub_endpoints`
- any arm without `backend_type: rbpodo`
- any arm without `operation_mode: simulation`
- missing realtime capabilities on the server binary, unless
  `--allow-no-realtime` is passed explicitly

Artifacts are written under:

```text
artifacts/rbpodo_circle/<timestamp>_<profile>_<arm>
```

Each completed benchmark also writes offline timing evidence:

- `alignment_summary.json`
- `alignment_report.md`
- nested `summary.json` blocks `timestamp_alignment` and
  `tail_error_correlation`
- `error_decomposition.json`
- `cycle_error_decomposition.csv`

Review `timing_classification` before interpreting tracking error tails.
`clean_timing` means the audit did not find command, state publish, or ACK
spikes above its thresholds. `ack_spike_limited`,
`command_generation_limited`, `state_publish_limited`, and `jitter_limited`
mean the artifact is measurement-limited; do not use p95/max error tails from
that run as reliable gain-quality evidence.

Review `error_classification` before tuning. The benchmark decomposes tracking
error into phase lag, center drift, radius error, orientation drift, feedback
saturation, timing jitter, and tail spikes. This is diagnosis only; it does not
change control behavior or authorize physical motion.

For orientation-specific diagnosis, inspect these summary fields before raising
orientation gain:

- `desired_orientation_stand` and `measured_orientation_source`
- `orientation_error_vector_samples`
- `angular_feedback_norm` and `angular_applied_norm`
- `angular_saturation_count` and `angular_saturation_ratio`
- `orientation_p50_deg`, `orientation_p95_deg`, and `orientation_max_deg`
- `orientation_position_equiv_mm`

Run a `Kp_ori=0` row under the same `Kp_pos`, profile, tracking source, command
rate, and feedback limits before tuning `Kp_ori` upward. If a positive
`Kp_ori` row increases orientation drift relative to the matched `Kp_ori=0`
row, classify the result as `orientation_feedback_suspect` and inspect sign,
stand/local frame convention, angular saturation, desired-orientation hold, and
position/orientation coupling before trying a larger orientation gain. Do not
couple `Kp_pos=Kp_ori` by default.

Review `measurement_reliability_level` before both tuning and dataset
selection. In pgmode simulation, `tcp_ref_stand` is a controller-reference
lower bound, not physical TCP tracking. A completed run can be
`controller_reference_valid` only when reference telemetry is valid, no fault
or physical motion was detected, Cartesian commands were available, and
diagnostics are not suspect. Runs with `diagnostics_suspect_count > 0` or a
diagnostics-suspect override are `suspect`, not physical-ready. Runs with
failed Python/C++ state parity are no higher than `suspect`. Caveats such as
`tcp_ref_lower_bound_only`, `controller_reference_lower_bound`, and
`q_ref_not_directly_validated` must stay visible before tuning or dataset
selection. `physical_ready_candidate` is not assigned while
diagnostics_suspect remains unresolved.

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

If `diagnostics_suspect` persists, capture raw Rainbow data-port payloads
before interpreting status bits or circle tracking errors:

```bash
RB_ALLOW_REAL_ROBOT=1 \
python3 scripts/rainbow_data_port_capture.py \
  --ips 172.28.60.200 172.28.60.201 \
  --port 5001 \
  --duration-sec 5 \
  --rate-hz 10 \
  --artifact-dir artifacts/rbpodo_measurement/raw_data \
  --also-rbpodo-python \
  --i-understand-this-connects-to-real-controller
```

This raw capture is read-only field-layout evidence. It uses only TCP data
port 5001, does not send `pgmode`, motion, reset, or command-port traffic, and
does not parse binary payloads. Keep real raw payloads under `artifacts/` and
do not commit them unless sanitized.

For measurement reliability, compare the Python rbpodo state decode with the
C++ state stream before interpreting circle errors:

```bash
python3 scripts/rbpodo_state_parity_check.py \
  --use-running-server \
  --ips 172.28.60.200 172.28.60.201 \
  --duration-sec 5 \
  --state-endpoint udp://127.0.0.1:50151 \
  --artifact-dir artifacts/rbpodo_measurement/state_parity \
  --i-understand-this-connects-to-real-controller
```

See `docs/runbooks/rbpodo_measurement_reliability.md`. Parity passing only
means Python and C++ decode the same fields; it does not make suspicious
diagnostics semantically valid.

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

Run profiles from slowest/safest to fastest/stress:

| Profile | Diameter | Period | Required speed | Stress level | Purpose |
| --- | ---: | ---: | ---: | --- | --- |
| `safe_5cm_10s` | 0.05 m | 10 s | 0.016 m/s | `bringup` | bring-up |
| `circle_15cm_16s` | 0.15 m | 16 s | 0.029 m/s | `baseline` | stable baseline |
| `circle_15cm_8s` | 0.15 m | 8 s | 0.059 m/s | `middle` | middle speed ablation |
| `gene_15cm_4s` | 0.15 m | 4 s | 0.118 m/s | `stress` | GENE-style stress |

Recommended progression:

1. `safe_5cm_10s`
2. `circle_15cm_16s`
3. `circle_15cm_8s`
4. `gene_15cm_4s`

Use `circle_15cm_8s` before the 4 s stress case to isolate whether 4 s
failures are bandwidth, latency, saturation, or speed-limit constrained rather
than basic 15 cm tracking errors. The 4 s profile is not real-ready and remains
explicit stress evidence only.

## 500 Hz Controller-Simulation Track

The 500 Hz track is staged from a single-arm no-op rate-probe artifact, not from
a physical-motion run. Stage 0 evidence in
`artifacts/rbpodo_servo_j_rate_probe_left` shows 5000/5000 500 Hz Servo J
no-op sends succeeded over 10 seconds in controller `pgmode` simulation, with
loop interval p99 about 2.006 ms and max send duration about 501 us. This is
only enough to begin staged 500 Hz controller-simulation acceptance.

500 Hz is not the default. Existing 100 Hz templates and local-config creation
remain unchanged unless `--include-500hz` is passed.

Run order:

1. single-arm no-op 500 Hz rate probe: already done for the left controller
2. `rb_servo_server` dual-arm no-op 500 Hz
3. 5 cm / 10 s circle
4. 15 cm / 16 s circle
5. 15 cm / 8 s circle
6. 15 cm / 4 s circle

All 500 Hz templates keep `operation_mode: simulation`,
`disable_waiting_ack: false`, `cartesian_control.allow_in_real: false`,
`cartesian_control.allow_in_controller_simulation: true`, and
`network.state_pub_rate_hz: 100`. Do not raise state publication to 500 Hz by
default.

See `docs/runbooks/rbpodo_500hz_acceptance.md` for the full staged workflow.

### Async ACK-Supervised 500 Hz Workflow

`RBPODO-ASYNC-RUNBOOK-01` extends the 500 Hz track with async ACK-supervised
controller-simulation modes. The three 500 Hz modes are:

- synchronous ACK-on: `disable_waiting_ack: false` and no async worker; the
  servo loop waits for the rbpodo SDK call/ACK path. At 500 Hz, the 2 ms tick
  makes this fragile because ACK wait outliers can consume the tick, and
  dual-arm sequential sends can make the second arm inherit the first arm's
  ACK delay.
- `sdk_ack_worker`: the servo loop enqueues a latest-wins target and does not
  wait for ACK. A per-arm worker waits for SDK ACK and reports worker ACK
  telemetry, while supervision faults on missing ACK or invalid reference
  state.
- `socket_send_supervised`: the servo loop enqueues without waiting for ACK and
  the send lane is socket/API send only. Reports must label successful sends as
  `socket_send_only`; a q_ref watchdog and tcp_ref watchdog must show fresh
  controller-reference telemetry before the row can be accepted.

Async ACK-supervised means the servo loop is not ACK-blocked, but ACK or
controller-reference supervision is still mandatory. `sdk_ack_worker` preserves
controller ACK evidence in the worker lane. `socket_send_supervised` is not
controller ACK evidence; it is supervised socket-send evidence that depends on
healthy `q_ref` / `tcp_ref_stand` updates.

No physical real: async 500 Hz evidence is controller `pgmode` simulation only.
Every row must keep `operation_mode: simulation` (`operation_mode=simulation`
in reports), `allow_in_real: false`, `physical_motion_expected=false`, and
`physical_motion_detected=false`.

Required async gates are the normal real-controller/controller-simulation
gates plus:

```bash
RB_ALLOW_RBPODO_ASYNC_STREAMING=1
RB_RBPODO_PGMODE_SIMULATION_CONFIRMED=1
```

`socket_send_supervised` additionally requires one of:

```bash
RB_ALLOW_RBPODO_ACK_DISABLED_MOTION=1
RB_ALLOW_RBPODO_SOCKET_SEND_ONLY_STREAMING=1
```

Run async acceptance in this order:

1. SDK probe.
2. No-op acceptance.
3. `safe_5cm_10s`.
4. `circle_15cm_16s`.
5. `circle_15cm_8s`.
6. `gene_15cm_4s` stress.

Read async reports by separating `controller_ack_observed` from
`socket_send_only`, then checking `q_ref_supervised`,
`reference_supervision_state`, q_ref/tcp_ref update ages, and the
`tcp_ref_stand` / tcp_ref lower bound caveat. `diagnostics_suspect` rows remain
suspect measurement evidence and cannot promote 500 Hz defaults or physical
real readiness.

RBPODO-500HZ-CIRCLE-MATRIX-01 adds three staged matrix files for the circle
part of that workflow:

1. `configs/rbpodo_circle_ablation/500hz_stage1_noop_and_safe.yaml`
2. `configs/rbpodo_circle_ablation/500hz_stage2_15cm16s.yaml`
3. `configs/rbpodo_circle_ablation/500hz_stage3_8s_4s.yaml`

Run them in that order after the 500 Hz no-op stage. Stage 1 uses the safe
5 cm / 10 s profile with low gains. Stage 2 compares 15 cm / 16 s open-loop
and low-gain closed-loop rows at speed_bar 0.2 and 0.5. Stage 3 runs the
15 cm / 8 s bridge before any 15 cm / 4 s stress rows; its speed_bar 0.5 rows
are disabled optional follow-ups. All 500 Hz rows explicitly keep
`servo.rate_hz: 500`, `servo_t1_sec: 0.002`, controller
`operation_mode: simulation`, and `cartesian_control.allow_in_real: false`.

The stable baseline profile is `15cm/16s`. It mirrors the current simulator
baseline:

- `servo.rate_hz: 100`
- `velocity_target_integration: previous_command`
- `controller_simulation_servo_state_source: reference`
- `controller_simulation_divergence_source: reference`
- `path_kp_pos: 6.0`
- `path_kp_ori: 6.0`
- `max_twist_linear_m_s: 0.03`
- `max_twist_angular_rad_s: 0.2`

The middle profile is `15cm/8s`. It requires about `0.059 m/s`, so the stable
15cm/16s template speed limit is not sufficient unless a local copy raises the
controller-simulation twist limits. Keep it as a middle-speed ablation, not a
real-candidate label.

The stress profile is `15cm/4s`. It mirrors the GENE-style simulator stress
settings and is not a pass/fail acceptance profile or real-ready profile:

- `servo.rate_hz: 100`
- `max_twist_linear_m_s: 0.15`
- `max_twist_angular_rad_s: 0.4`
- feedback benchmark mode is recommended before interpreting drift

The benchmark profile metadata recommends comparing `twist_stand` and
`twist_stand_feedback` for all four profiles.

For controller pgmode simulation, the correct measurement target is normally
the controller reference path rather than physical TCP motion. The state stream
publishes both:

- `tcp_actual_stand` from physical/measured `q_actual`
- `tcp_ref_stand` from controller reference `q_ref` / `q_target`

The controller-simulation circle templates also use the controller reference
state for Cartesian servo integration and the divergence guard:

```yaml
cartesian_control:
  controller_simulation_servo_state_source: reference
  controller_simulation_divergence_source: reference
```

This source selection is only meaningful for rbpodo controller pgmode
simulation. Physical real and rb_simulator paths continue to use `q_actual`.
If `q_ref` / `tcp_ref_stand` is unavailable, the server fails closed instead of
falling back silently. Physical-motion monitoring still uses `q_actual`.

- `tcp_actual_stand`: FK from measured `q_actual_deg`
- `tcp_ref_stand`: FK from rbpodo controller reference joints (`q_ref_deg`,
  alias `q_target_deg`, source `rbpodo.sdata.jnt_ref`)

Legacy `tcp_stand` remains an actual-pose alias. When the per-arm
`controller_simulation_mode.recommended_tracking_pose` is `tcp_ref_stand`,
circle reports should compare the desired circle trajectory against
`tcp_ref_stand`. If `tcp_ref_valid` is false, report the benchmark as missing
reference telemetry instead of silently treating stationary physical
`tcp_actual_stand` as controller tracking failure.

Physical real circle testing is a separate future task. These templates are not
real-ready and must not be copied into a physical Cartesian acceptance run.

## Server-Side Circle Tracking Skeleton

`TcpCircleMove` is now the implemented benchmark-only server-side circle
primitive used by the `server_circle` benchmark controller. It starts from the
current TCP pose, chooses the center so the current pose is on the circle, holds
the initial orientation, and updates the desired circle reference in the servo
tick. In rbpodo controller simulation it remains gated by
`enable_benchmark_primitives`, `allow_in_controller_simulation: true`,
`allow_in_real: false`, controller `operation_mode: simulation`, and the same
pgmode confirmation env gates as the streaming Cartesian carve-out.

`phase_advance_sec` on `TcpCircleMove` is allowed only as visible benchmark
dead-time compensation. Reports must keep the configured phase advance,
uncompensated latency estimate, and effective residual latency separate.

`TcpCircleTrack` is the non-default server-side closed-loop circle tracking
skeleton. It is intended to replace the current Python feedback loop only after
separate implementation and acceptance work:

```text
current: benchmark state UDP -> Python feedback -> UDP command -> server -> rbpodo
target: benchmark parameters -> server servo tick desired pose/twist + feedback
```

Use server-side circle tracking as the official report term. Keep accepting
legacy names for now, but report both `TcpCircleMove` and `TcpCircleTrack` as
`command_family: server_side_circle` so historical artifacts, GUI overlays, and
future reports can group them without implying they are the same command.

The command carries trajectory and feedback parameters:

```json
{
  "schema_version": 1,
  "seq": 1,
  "mode": "TcpCircleTrack",
  "arm": "left",
  "center_stand": [0.4, -0.1, 0.2],
  "radius_m": 0.075,
  "plane": "xy",
  "period_sec": 6.0,
  "repeat": 3,
  "start_phase_rad": 0.0,
  "orientation_hold": true,
  "feedback_kp_pos": 1.5,
  "feedback_kp_ori": 0.0,
  "max_linear_m_s": 0.2,
  "max_angular_rad_s": 0.4,
  "tracking_source": "tcp_ref_stand"
}
```

Safety status for this skeleton:

- Default config leaves `cartesian_control.enable_server_side_circle_track:
  false`, so the command rejects as `tcp_circle_track_disabled`.
- If the config flag is enabled, the current implementation still rejects as
  `tcp_circle_track_not_implemented`; it does not send a generated twist.
- Physical real `operation_mode: real` rejects as
  `tcp_circle_track_physical_real_blocked`.
- Future rbpodo support is limited to controller pgmode simulation with
  `operation_mode: simulation`, `allow_in_real: false`, and the existing
  `RB_ALLOW_RBPODO_CONTROLLER_SIM_*` gates.

Implementation phases:

1. Parser/schema and structured accepted/rejected telemetry.
2. Simulator-side tick-local trajectory and feedback implementation.
3. Rbpodo controller-simulation implementation using controller-reference
   state (`tcp_ref_stand`) in pgmode simulation only.
4. Acceptance matrix that keeps `rb_simulator`, rbpodo controller-simulation,
   and any future physical-real evidence separate.

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

Use server-side UDP fanout as the primary live-visualization path. The
benchmark recorder and `rb_gui` must listen on different state ports, and
`rb_servo_server` publishes the same serialized state JSON to both destinations
from `network.state_pub_endpoints`. Avoid using a Python tee/rebroadcast as the
main path; it adds another process and more latency than direct server fanout.
The GUI command path still goes directly to `network.command_bind`; state
fanout does not change command ownership or safety gates.

Benchmark-specific desired-circle geometry and running metrics come from a
separate telemetry-only overlay stream published by
`scripts/rbpodo_circle_tracking_benchmark.py`. Use
`--overlay-pub-endpoint udp://127.0.0.1:50261` for the GUI overlay listener and
`--overlay-pub-rate-hz 20` unless a test needs a different display rate. This
overlay carries desired pose, circle radius/center/plane, current error,
running RMS/p95, estimated latency, and tracking source. It never carries robot
commands and does not replace the server state stream for pass/fail evidence.

Run the GUI against the fanout state port and overlay port as separate streams.
The current repository module entry point is `rb_servo_gui.app`:

```bash
PYTHONPATH=rb_gui \
RB_GUI_STATE_BIND=0.0.0.0 \
RB_GUI_STATE_PORT=50161 \
RB_GUI_CIRCLE_OVERLAY_BIND=udp://0.0.0.0:50261 \
python3 -m rb_servo_gui.app
```

After installing `rb_gui`, the equivalent console command is `rb-servo-gui`.
Do not use bare `python3 -m rb_servo_gui` in this checkout unless a future
change adds `rb_servo_gui/__main__.py`; that shorthand is intended to mean the
same GUI process:

```bash
PYTHONPATH=rb_gui \
RB_GUI_STATE_BIND=0.0.0.0 \
RB_GUI_STATE_PORT=50161 \
RB_GUI_CIRCLE_OVERLAY_BIND=udp://0.0.0.0:50261 \
python3 -m rb_servo_gui
```

The state stream is robot/server telemetry (`tcp_actual_stand`,
`tcp_ref_stand`, faults, gates). The overlay stream is benchmark-owned desired
geometry and live metrics, so it must be visually treated as the desired path,
not as robot state.

Dead-time compensation must stay visible. For streaming controllers,
`--phase-advance-sec SEC` advances the desired circle sample used to generate
benchmark commands by `SEC`. For `server_circle`, the same field is forwarded
as `TcpCircleMove.phase_advance_sec` and applied inside the server-side
reference generator. In both cases, `samples.csv`, error decomposition, and
summary metrics still compare measured `tcp_ref_stand` against the desired
trajectory at measurement time. The summary distinguishes
`commanded_phase_advance_ms` from measured `estimated_latency_ms`. Do not use
this option for physical-real commands; the rbpodo benchmark still requires
`operation_mode: simulation`, pgmode confirmation,
`physical_motion_expected=false`, and `allow_in_real: false`.

`policy_runner` is not in this live-visualization path. It is a separate
command source for policy/SpaceMouse workflows. In the circle live view, the
benchmark script is the explicit command source, while the benchmark recorder
and `rb_gui` are state consumers. The GUI does not route commands through
`policy_runner`.

In `rb_gui`, leave `TCP display` on `Auto` for controller pgmode simulation.
Auto shows `tcp_ref_stand` when the state stream recommends it and the
reference pose is valid. Use `Actual` only to inspect the physical
`q_actual`-based pose; it is not controller-simulation tracking evidence.

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
  --overlay-pub-endpoint udp://127.0.0.1:50261 \
  --overlay-pub-rate-hz 20 \
  --set-pgmode-simulation \
  --artifact-dir artifacts/rbpodo_circle/circle_15cm16s_twist_stand_left \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation
```

Middle 15 cm / 8 s ablation example:

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
  --profile circle_15cm_8s \
  --repeat 3 \
  --command-rate-hz 100 \
  --feedback-kp-pos 1.0 \
  --feedback-kp-ori 1.0 \
  --tracking-source tcp_ref_stand \
  --overlay-pub-endpoint udp://127.0.0.1:50261 \
  --overlay-pub-rate-hz 20 \
  --verify-pgmode-simulation \
  --artifact-dir artifacts/rbpodo_circle/circle_15cm8s_feedback_left \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation
```

This middle-speed example uses the local 15cm/4s-capable config only for its
speed envelope; it is still controller-simulation ablation evidence and is not
real-ready.

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
  --allow-fast-stress \
  --repeat 5 \
  --command-rate-hz 100 \
  --feedback-kp-pos 2.0 \
  --feedback-kp-ori 2.0 \
  --feedback-max-linear-m-s 0.15 \
  --feedback-max-angular-rad-s 0.4 \
  --tracking-source tcp_ref_stand \
  --overlay-pub-endpoint udp://127.0.0.1:50261 \
  --overlay-pub-rate-hz 20 \
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
- `q_sent_moved`
- `q_ref_moved`
- `q_ref_reason`
- `tcp_ref_moved`
- `tcp_actual_moved`
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

## Timestamp Alignment Audit

The benchmark runs `scripts/timestamp_alignment_audit.py` automatically after
writing `samples.csv`, `state_stream.jsonl`, `command_packets.jsonl`, feedback
terms, and overlay artifacts. Re-run it manually when inspecting an older
artifact or after copying artifacts between machines:

```bash
python3 scripts/timestamp_alignment_audit.py \
  --artifact-dir artifacts/rbpodo_circle/gene_15cm4s_feedback_left \
  --output-md alignment_report.md \
  --output-json alignment_summary.json
```

The audit is offline/read-only and sends no robot commands. It reports command
generation intervals, state publish intervals, state age, servo send duration,
ACK wait duration, stale feedback skips, overlay timing, and p95 error near
versus away from timing spikes. Its recommendation is deliberately
conservative: fix jitter, inspect ACK spikes, consider an explicit ACK-off
controller-simulation experiment, move feedback server-side, increase state
publish rate, or tune gains only after timing is clean.

Also confirm `operation_mode: simulation` and same-run pgmode simulation
confirmation. Do not set `RB_ALLOW_REAL_CARTESIAN` for this workflow.

## Error Decomposition

Each completed circle benchmark also writes `error_decomposition.json` and
`cycle_error_decomposition.csv`. These files are offline/read-only analysis of
the benchmark artifacts. They do not connect to controllers, send commands, or
modify robot state.

The run-level decomposition records:

- `median_error_m`, `mad_error_m`, `iqr_error_m`, `tail_ratio`, and
  `max_over_p95`
- `center_error_m`, `radius_error_m`, and `radius_gain`
- `phase_lag_rad`, `estimated_latency_ms`, and
  `phase_aligned_rms_error_m`
- `center_removed_rms_error_m` and
  `center_and_phase_removed_rms_error_m`
- orientation p50/p95/max drift, plus position-equivalent orientation error
  for configurable tool-offset guesses
- `error_classification` and `error_classifications`

The default orientation-equivalent offsets are 0.03 m, 0.05 m, and 0.10 m.
Override them when needed:

```bash
python3 scripts/rbpodo_circle_tracking_benchmark.py \
  ... \
  --tool-offset-m 0.03,0.05,0.10
```

The per-cycle CSV records `cycle_rms_error_m`, `cycle_p95_error_m`,
`cycle_fit_center_error_m`, `cycle_radius_gain`,
`cycle_orientation_p95_rad`, and `cycle_saturation_count`.

Interpret the classifications as diagnostic hints:

- `phase_lag_limited`: phase alignment strongly lowers RMS.
- `center_drift_limited`: removing fitted center drift strongly lowers RMS.
- `tail_spike_limited`: p95 is much larger than median error.
- `orientation_limited`: orientation drift has a large tool-offset equivalent.
- `saturation_limited`: feedback saturation is significant.
- `timing_jitter_limited`: timestamp audit classified the artifact as
  timing-limited.

For stage-1 style 4 s rows, expect patterns such as open-loop having good
radius but large center drift, closed-loop reducing center drift, high Kp rows
showing saturation or orientation drift, and low-median rows still being
tail-spike limited. Those observations guide the next diagnosis matrix; they
are not physical real-motion acceptance.

## Troubleshooting

If `rb_gui` shows no state, check that the local circle config uses
`network.state_pub_endpoints`, that one endpoint is the GUI port, and that the
GUI binds that same port:

```bash
grep -A4 "state_pub_endpoints" rb_servo_server/config/local/dual_real_rbpodo_circle_15cm16s.yaml
RB_GUI_STATE_BIND=0.0.0.0 RB_GUI_STATE_PORT=50161 env | grep '^RB_GUI_STATE'
```

For the 15 cm / 4 s stress template, use the matching GUI state endpoint
`50162` unless you intentionally edited the local config.

If the desired circle overlay is missing but state is live, check that the
benchmark publishes to the same endpoint the GUI bound:

```bash
grep -R "\"overlay_pub_endpoint\"" artifacts/rbpodo_circle -n 2>/dev/null | tail
env | grep '^RB_GUI_CIRCLE_OVERLAY_BIND='
```

The normal local pairing is:

```text
benchmark: --overlay-pub-endpoint udp://127.0.0.1:50261
rb_gui:    RB_GUI_CIRCLE_OVERLAY_BIND=udp://0.0.0.0:50261
```

`realtime setup failed` or `failed to set realtime priority` means the server
could not acquire its requested scheduler privileges. Rebuild if needed, then
apply Linux capabilities to the exact binary you pass with `--server`:

```bash
sudo setcap cap_sys_nice,cap_ipc_lock+ep rb_servo_server/build/rbpodo_real_gate/rb_servo_server
getcap rb_servo_server/build/rbpodo_real_gate/rb_servo_server
```

If `getcap` prints nothing after a rebuild, the binary lost its capabilities;
run `setcap` again before retrying.

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

If the benchmark reports `result: startup_fault`, the server did publish state
packets, but the latest startup packet was already fault-latched. This is
different from a state stream timeout. Inspect `summary.json` fields such as
`latched_fault_reason`, `fault_context`, `safety_tracking`, and
`q_actual_target_error_summary`. For `TrackingError` with reference tracking,
common causes are a stale controller-simulation `q_target` / `q_ref` that
differs from startup `q_actual`, a server build missing the startup-reference
initialization fix, or a controller reference that should be reset to current
joint state before retrying.

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
RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1 \
python3 scripts/run_rbpodo_circle_ablation.py \
  --matrix configs/rbpodo_circle_ablation/rbpodo_circle_ablation_example.yaml \
  --artifact-root artifacts/rbpodo_circle/ablation_gene15cm4s_left \
  --server rb_servo_server/build/rbpodo_real_gate/rb_servo_server \
  --verify-pgmode-simulation \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation
```

Use `--dry-run` first to validate matrix/config shape and print the exact
benchmark commands plus each experiment's `resolved_server_config.yaml` path.
Dry-run still checks that the runner has been given the same explicit safety
flags and env gates required for a real matrix run.

The matrix supports the intended factor split:

- `profile`: `safe_5cm_10s`, `circle_15cm_16s`,
  `circle_15cm_8s`, or `gene_15cm_4s`
- `controller`: `twist_stand`, `twist_local`,
  `twist_stand_feedback`, `twist_local_feedback`, or `server_circle`
- `command_rate_hz`, `repeat`, `tracking_source`, feedback gains, feedback
  speed limits, and explicit extra env requirements
- temporary per-experiment `config_overrides` for the allowlisted server-config
  factors: `network.state_pub_rate_hz`, `servo.rate_hz`,
  `servo.worker_read_period_sec`,
  `left_robot.speed_bar`, `right_robot.speed_bar`,
  `left_robot.servo_t1_sec`, `right_robot.servo_t1_sec`,
  `left_robot.servo_t2_sec`, `right_robot.servo_t2_sec`,
  `left_robot.servo_alpha`, `right_robot.servo_alpha`,
  `left_robot.command_timeout_sec`, `right_robot.command_timeout_sec`,
  `cartesian_control.max_twist_linear_m_s`,
  `cartesian_control.max_twist_angular_rad_s`,
  `cartesian_control.max_linear_move_speed_m_s`,
  `cartesian_control.enable_benchmark_primitives`,
  `cartesian_control.path_kp_pos`, `cartesian_control.path_kp_ori`,
  `cartesian_control.twist_angular_deadband_rad_s`,
  `cartesian_control.velocity_target_integration`, and
  `cartesian_control.velocity_target_lookahead_sec`
- `phase_advance_sec` forwards to benchmark `--phase-advance-sec`; the default
  is `0.0`, values must be explicit and non-negative, and values greater than
  `0.25 * period_sec` are rejected
- ACK mode is derived from the referenced config's
  `disable_waiting_ack` fields

ACK-off experiments require `RB_ALLOW_RBPODO_ACK_DISABLED_MOTION=1`.
Configs that opt into the temporary diagnostics-suspect bridge require
`RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1`. The runner stops on
safety preflight failure or child benchmark `result: blocked`, `faulted`,
`startup_fault`, or `error`.
Overrides are written to each experiment artifact directory as
`resolved_server_config.yaml`; the source or local config is not edited in
place. The runner rejects overrides that would change `operation_mode` to
`real`, set `cartesian_control.allow_in_real: true`, remove
`allow_in_controller_simulation: true`, remove `backend_type: rbpodo`, or
create a `servo.rate_hz` / `servo_t1_sec` mismatch unless the source config
already explicitly allows that mismatch. `network.state_pub_rate_hz` overrides
must be `> 0` and `<= 200`; `speed_bar` overrides must be `> 0` and `<= 1.0`.
Servo J parameter sweeps are limited to temporary resolved configs and are
validated on the resolved file before a benchmark command is launched:
`servo_t2_sec` must be finite and in `(0.02, 0.2)`, and `servo_alpha` must be
finite and in `(0, 1)` on both arms. These ranges are an ablation safety
envelope, not an interpretation of what Rainbow's `alpha` means; alpha
semantics must be determined from measured controller-simulation artifacts.
For rate/t1 sweeps, use `servo.rate_hz: 100` with `servo_t1_sec: 0.01`,
`servo.rate_hz: 200` with `servo_t1_sec: 0.005`, or staged 500 Hz
controller-simulation rows with `servo.rate_hz: 500` and
`servo_t1_sec: 0.002` on both arms.

`server_circle` rows send one `TcpCircleMove` packet and let the server update
the circle reference on the servo tick. This is a controller-simulation
benchmark primitive only. It requires
`cartesian_control.enable_benchmark_primitives: true`,
`cartesian_control.circle_move.allow_in_simulation: true`,
`cartesian_control.circle_move.allow_in_real: false`, the normal pgmode
simulation gates, and `physical_motion_expected=false`. It must not be copied
to physical real configs.

Example GENE-style Kp, state publish rate, and speed-bar sweep entries:

```yaml
experiments:
  - name: gene_fb_kp05_ori02_pub100_speed02
    config: rb_servo_server/config/local/dual_real_rbpodo_circle_15cm4s.yaml
    profile: gene_15cm_4s
    controller: twist_stand_feedback
    arm: left
    command_rate_hz: 100
    repeat: 5
    tracking_source: tcp_ref_stand
    feedback_kp_pos: 0.5
    feedback_kp_ori: 0.2
    feedback_max_linear_m_s: 0.15
    feedback_max_angular_rad_s: 0.4
    config_overrides:
      network.state_pub_rate_hz: 100
      left_robot.speed_bar: 0.2
      right_robot.speed_bar: 0.2

  - name: gene_fb_kp20_pub50_speed05
    config: rb_servo_server/config/local/dual_real_rbpodo_circle_15cm4s.yaml
    profile: gene_15cm_4s
    controller: twist_stand_feedback
    arm: left
    command_rate_hz: 100
    repeat: 5
    tracking_source: tcp_ref_stand
    feedback_kp_pos: 2.0
    feedback_kp_ori: 2.0
    feedback_max_linear_m_s: 0.15
    feedback_max_angular_rad_s: 0.4
    config_overrides:
      network.state_pub_rate_hz: 50
      left_robot.speed_bar: 0.5
      right_robot.speed_bar: 0.5

  - name: gene_200hz_t1_005_speed10
    config: rb_servo_server/config/local/dual_real_rbpodo_circle_15cm4s.yaml
    profile: gene_15cm_4s
    controller: twist_stand_feedback
    arm: left
    command_rate_hz: 100
    repeat: 5
    tracking_source: tcp_ref_stand
    feedback_kp_pos: 2.0
    feedback_kp_ori: 2.0
    feedback_max_linear_m_s: 0.15
    feedback_max_angular_rad_s: 0.4
    config_overrides:
      servo.rate_hz: 200
      left_robot.servo_t1_sec: 0.005
      right_robot.servo_t1_sec: 0.005
      left_robot.speed_bar: 1.0
      right_robot.speed_bar: 1.0
```

Each matrix run writes:

- one artifact subdirectory per enabled experiment
- `experiment_command.txt` and `ablation_command.json` per experiment
- `resolved_server_config.yaml` per experiment
- `matrix_resolved.json`
- `ablation_summary.csv`
- `ablation_summary.json`
- `ablation_report.md`
- summary plots for RMS error, p95 error, radius gain, latency, q_ref update
  rate, and physical-motion detection when the metrics are available

The ablation summary also carries decomposition columns:

- `servo_rate_hz`
- `servo_t1_sec`
- `servo_t2_sec`
- `servo_t2_sec_left`
- `servo_t2_sec_right`
- `servo_alpha`
- `servo_alpha_left`
- `servo_alpha_right`
- `phase_advance_sec`
- `phase_advance_fraction_of_period`
- `phase_advance_effect`
- `commanded_phase_advance_ms`
- `speed_bar`
- `send_duration_p99_us`
- `servo_jitter_p99_ms`
- `deadline_miss_count`
- `feedback_saturation_count`
- `orientation_p95_deg`
- `median_error_mm`
- `tail_ratio`
- `center_removed_rms_mm`
- `phase_aligned_rms_mm`
- `orientation_position_equiv_50mm_mm`
- `error_classification`

The runner additionally writes measurement reliability artifacts at the
ablation root:

- `measurement_reliability_report.md`
- `measurement_reliability_summary.csv`
- `measurement_reliability_summary.json`

`ablation_report.md` includes a "Measurement reliability and caveats" section
before tuning candidate tables. Read this section first; unreliable or suspect
rows should not drive gain changes.

## 500 Hz Rbpodo Circle Matrices

The 500 Hz matrices are a staged comparison, not a promotion to physical real
motion. Keep the run order:

1. `500hz_stage1_noop_and_safe.yaml`
2. `500hz_stage2_15cm16s.yaml`
3. `500hz_stage3_8s_4s.yaml`

Dry-run stage 1:

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

Use the same command shape for stage 2 and stage 3. For actual runs, remove
`--dry-run` only after reviewing `matrix_resolved.json` and each
`resolved_server_config.yaml`. Stop before the next stage if any row reports a
fault latch, physical-motion detection, Cartesian unavailable rejection,
deadline miss, timing issue, missing `tcp_ref_stand`, or suspect measurement
reliability. The 15 cm / 4 s rows remain stress evidence.

## Stage-2 Rbpodo Circle Matrices

The stage-2 matrices refine the latest GENE-style controller-simulation
evidence from
`artifacts/rbpodo_circle_ablation/gene4s_stage1_20260531_232739`. Stage-1
showed that open-loop 15 cm / 4 s has large center drift, closed-loop feedback
is structurally better, `Kp=2` is too aggressive, `Kp_pos=0.5` /
`Kp_ori=0.5` is the best current 4 s stress candidate, orientation feedback
can increase orientation drift, and `pub_rate=100` with `speed_bar=0.1`
saturates heavily. These observations guide the next matrices only; no
controller-simulation result is real-ready.

Run order:

1. `stage2_gain_split.yaml`
2. `stage2_pub_speed.yaml`
3. `stage2_8s_middle.yaml`

`stage2_gain_split.yaml` separates position and orientation feedback at
`network.state_pub_rate_hz: 50` and `speed_bar: 0.1`, centered around
`Kp_pos=0.5`. `stage2_pub_speed.yaml` then checks publish-rate and speed-bar
effects for the low-gain candidates. `stage2_8s_middle.yaml` bridges the stable
15 cm / 16 s baseline and the GENE-style 15 cm / 4 s stress profile; it uses
the 15 cm / 4 s local config only because its controller-simulation speed
limits are sufficient for 15 cm / 8 s.

Interpretation:

- Test `Kp_ori=0` before increasing orientation gain, and compare it only
  against rows with the same `Kp_pos` and timing/speed limits.
- Do not set `Kp_pos=Kp_ori` automatically; position and orientation feedback
  are separate channels with separate saturation and drift failure modes.
- Choose candidates with low or zero `feedback_saturation_count` first.
- Check `angular_saturation_count`, `angular_feedback_norm`, and
  `angular_applied_norm` before interpreting orientation drift as a gain issue.
- Reject high orientation drift even if RMS error is lower.
- Compare `p95_error_m` and `fit_center_error_m`; center drift matters, not
  only RMS.
- Compare `median_error_m`, `tail_ratio`, `center_removed_rms_error_m`,
  `phase_aligned_rms_error_m`, and `orientation_position_equiv_50mm_m` before
  changing gains.
- Use `measurement_reliability_level` and `benchmark_interpretation` before
  selecting IL data. Stress-profile rows are marked `IL_data_not_recommended`;
  use stable clean-timing profiles for any future IL candidate.
- Treat `gene_15cm_4s` rows as stress evidence. Do not mark any 4 s matrix
  result as real-ready.
- Treat the 8 s matrix as a bridge between 16 s and 4 s evidence, not as
  physical Cartesian acceptance.

Use the convenience runner for stage-2 tuning matrices:

```bash
tools/rbpodo_circle_tune.sh \
  --matrix stage2_gain_split \
  --arm left \
  --with-required-env \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation
```

The wrapper resolves the stage-2 matrix file, creates an artifact directory
under `artifacts/rbpodo_circle_ablation/<timestamp>_<matrix>_<arm>`, checks the
local controller-simulation configs, checks server realtime capabilities, runs
`scripts/run_rbpodo_circle_ablation.py`, and prints the final
`ablation_report.md` path. Use `--matrix stage2_pub_speed` and
`--matrix stage2_8s_middle` for the next two matrices, or `--matrix-file PATH`
for a custom matrix whose enabled rows already match `--arm`.

Safety behavior:

- `--i-understand-this-connects-to-real-controller` and
  `--i-confirm-controller-is-in-pgmode-simulation` are always required.
- `RB_ALLOW_*` gates are exported only when `--with-required-env` is passed;
  otherwise they must already be set by the operator.
- `RB_ALLOW_REAL_CARTESIAN=1` is refused.
- Local configs must keep `operation_mode: simulation`,
  `allow_in_real: false`, `allow_in_controller_simulation: true`,
  `controller_simulation_tracking_error_source: reference`, and
  `controller_simulation_servo_state_source: reference`.
- `getcap` and the server's `cap_sys_nice,cap_ipc_lock` capabilities are
  required unless `--allow-no-realtime` is passed.
- Existing local configs are never overwritten unless `--force-local-configs`
  is passed.
- `--check-controller` verifies controller ports 5000 and 5001 before the
  matrix run. `--set-pgmode-simulation` calls `tools/simulation_mode.sh` first
  and then passes the resulting pgmode summary to the ablation runner.

Dry-run the convenience command before running a live matrix:

```bash
tools/rbpodo_circle_tune.sh \
  --matrix stage2_gain_split \
  --arm left \
  --with-required-env \
  --dry-run \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation
```

Dry-run the matrices first:

```bash
RB_ALLOW_REAL_ROBOT=1 \
RB_ALLOW_REAL_MOTION=1 \
RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1 \
RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1 \
RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1 \
python3 scripts/run_rbpodo_circle_ablation.py \
  --matrix configs/rbpodo_circle_ablation/stage2_gain_split.yaml \
  --artifact-root artifacts/rbpodo_circle_ablation/stage2_gain_split_dry_run \
  --server rb_servo_server/build/rbpodo_real_gate/rb_servo_server \
  --dry-run \
  --verify-pgmode-simulation \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation

RB_ALLOW_REAL_ROBOT=1 \
RB_ALLOW_REAL_MOTION=1 \
RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1 \
RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1 \
RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1 \
python3 scripts/run_rbpodo_circle_ablation.py \
  --matrix configs/rbpodo_circle_ablation/stage2_pub_speed.yaml \
  --artifact-root artifacts/rbpodo_circle_ablation/stage2_pub_speed_dry_run \
  --server rb_servo_server/build/rbpodo_real_gate/rb_servo_server \
  --dry-run \
  --verify-pgmode-simulation \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation

RB_ALLOW_REAL_ROBOT=1 \
RB_ALLOW_REAL_MOTION=1 \
RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1 \
RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1 \
RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1 \
python3 scripts/run_rbpodo_circle_ablation.py \
  --matrix configs/rbpodo_circle_ablation/stage2_8s_middle.yaml \
  --artifact-root artifacts/rbpodo_circle_ablation/stage2_8s_middle_dry_run \
  --server rb_servo_server/build/rbpodo_real_gate/rb_servo_server \
  --dry-run \
  --verify-pgmode-simulation \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation
```

Run the actual matrices in the same order after confirming pgmode simulation
for the same session:

```bash
RB_ALLOW_REAL_ROBOT=1 \
RB_ALLOW_REAL_MOTION=1 \
RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1 \
RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1 \
RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1 \
python3 scripts/run_rbpodo_circle_ablation.py \
  --matrix configs/rbpodo_circle_ablation/stage2_gain_split.yaml \
  --artifact-root artifacts/rbpodo_circle_ablation/stage2_gain_split \
  --server rb_servo_server/build/rbpodo_real_gate/rb_servo_server \
  --verify-pgmode-simulation \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation

RB_ALLOW_REAL_ROBOT=1 \
RB_ALLOW_REAL_MOTION=1 \
RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1 \
RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1 \
RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1 \
python3 scripts/run_rbpodo_circle_ablation.py \
  --matrix configs/rbpodo_circle_ablation/stage2_pub_speed.yaml \
  --artifact-root artifacts/rbpodo_circle_ablation/stage2_pub_speed \
  --server rb_servo_server/build/rbpodo_real_gate/rb_servo_server \
  --verify-pgmode-simulation \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation

RB_ALLOW_REAL_ROBOT=1 \
RB_ALLOW_REAL_MOTION=1 \
RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1 \
RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1 \
RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1 \
python3 scripts/run_rbpodo_circle_ablation.py \
  --matrix configs/rbpodo_circle_ablation/stage2_8s_middle.yaml \
  --artifact-root artifacts/rbpodo_circle_ablation/stage2_8s_middle \
  --server rb_servo_server/build/rbpodo_real_gate/rb_servo_server \
  --verify-pgmode-simulation \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation
```

## P1 Rbpodo Circle Factor Matrices

P1-CIRCLE-FACTOR-MATRIX-01 adds factor-separated matrices under
`configs/rbpodo_circle_ablation/`:

1. `p1_gain_split.yaml`
2. `p1_pub_speed.yaml`
3. `p1_twist_cap.yaml`
4. `p1_servo_t2_alpha.yaml`
5. `p1_phase_advance.yaml`

The P1 matrices are not random gain searches. They isolate one factor family at
a time while keeping the GENE-style 15 cm / 4 s result category as stress-only
controller-reference lower-bound evidence:

- `p1_gain_split.yaml`: open-loop baseline plus `Kp_pos` 0.3/0.5/0.8 crossed
  with `Kp_ori` 0.0/0.2/0.5 at `state_pub_rate_hz=50`, `speed_bar=0.1`, and
  `max_twist_linear_m_s=0.15`.
- `p1_pub_speed.yaml`: low-gain candidates `0.5/0.2`, `0.5/0.5`, and
  `0.8/0.2` across `pub50 speed0.1`, `pub100 speed0.2`,
  `pub100 speed0.5`, and `pub100 speed1.0`.
- `p1_twist_cap.yaml`: placeholder best-two gain candidates across
  `max_twist_linear_m_s` 0.15/0.18/0.20/0.25, with separate 0.2 rad/s angular
  cap probes.
- `p1_servo_t2_alpha.yaml`: stable low-gain candidate `pos=0.5, ori=0.2`
  across `servo_t2_sec` 0.03/0.05/0.08 and `servo_alpha` 0.3/0.5/0.8,
  crossed with `pub50/speed0.1` and `pub100/speed0.2`.
- `p1_phase_advance.yaml`: low-gain candidate across `phase_advance_sec`
  0.00/0.02/0.04/0.06/0.08. The benchmark samples commands at
  `t_command = t_now + phase_advance_sec`; tracking metrics still compare
  against the measurement-time desired trajectory.

Suggested run order:

1. Run `p1_gain_split.yaml`; reject rows with high orientation drift, center
   drift, saturation, fault latch, physical-motion detection, or suspect
   measurement reliability.
2. Run `p1_pub_speed.yaml` only on the low-gain candidates; do not retry
   stage-1 aggressive `Kp=1.0/1.0` as a default.
3. Update the placeholder candidate comments in `p1_twist_cap.yaml` if the
   first two matrices identify different best-two candidates, then run the
   twist-cap matrix.
4. Run `p1_servo_t2_alpha.yaml` after a low-gain candidate is stable enough to
   separate controller parameter effects from feedback effects. Compare
   `servo_t2_sec_left/right`, `servo_alpha_left/right`, send-duration p95/p99,
   radius gain, orientation drift, saturation, and timing classification before
   drawing conclusions; do not assume the sign or meaning of `alpha`.
5. Run `p1_phase_advance.yaml` last. Treat `commanded_phase_advance_ms` as the
   configured feed-forward offset and `estimated_latency_ms` as the measured
   residual phase lag, not the same quantity.

Dry-run any P1 matrix before a live controller-simulation run:

```bash
RB_ALLOW_REAL_ROBOT=1 \
RB_ALLOW_REAL_MOTION=1 \
RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1 \
RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1 \
RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1 \
python3 scripts/run_rbpodo_circle_ablation.py \
  --matrix configs/rbpodo_circle_ablation/p1_gain_split.yaml \
  --artifact-root artifacts/rbpodo_circle_ablation/p1_gain_split_dry_run \
  --server rb_servo_server/build/rbpodo_real_gate/rb_servo_server \
  --dry-run \
  --verify-pgmode-simulation \
  --i-understand-this-connects-to-real-controller \
  --i-confirm-controller-is-in-pgmode-simulation
```

Use the same command shape for `p1_pub_speed.yaml`, `p1_twist_cap.yaml`,
`p1_servo_t2_alpha.yaml`, and `p1_phase_advance.yaml`, changing both
`--matrix` and `--artifact-root`. Remove `--dry-run` only after checking
`matrix_resolved.json`, `resolved_server_config.yaml`, measurement reliability,
and current pgmode confirmation. No P1 row may be promoted to physical real
Cartesian readiness or IL readiness.

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
- `diameter_m`, `period_sec`, `required_tangential_speed_m_s`, and
  `stress_level`
- `tracking_source`, normally `tcp_ref_stand`
- `physical_motion_expected: false`
- `physical_motion_detected`
- `q_ref_update_rate_hz` and `q_actual_update_rate_hz`
- `ack_policy`
- `controller_acceptance_observed_count`
- `measurement_reliability_level`
- `reliability_caveats`
- `benchmark_interpretation`
- `physical_real_blockers`

The tuning report adds structure-aware columns for `kp_pos`, `kp_ori`,
`state_pub_rate_hz`, `speed_bar_left`, `speed_bar_right`,
`servo_t2_sec_left`, `servo_t2_sec_right`, `servo_alpha_left`,
`servo_alpha_right`, `saturation_ratio`, `orientation_p95_deg`,
`center_error_mm`, `score`, and `classification`. It also reports
decomposition columns:
`median_error_mm`, `tail_ratio`, `center_removed_rms_mm`,
`phase_aligned_rms_mm`, `orientation_position_equiv_50mm_mm`, and
`error_classification`, plus measurement reliability fields:
`measurement_reliability_level`, `reliability_caveats`,
`benchmark_interpretation`, and `physical_real_blockers`. Its rbpodo tuning
classifications are:

- `open_loop_baseline`
- `closed_loop_candidate`
- `saturation_limited`
- `orientation_unstable`
- `center_drift_limited`
- `state_pub_speed_mismatch`
- `stress_only`

Read report sections in this order: tuning result, measurement reliability,
then physical-readiness blockers. A tuning candidate with `suspect` or
`unreliable` measurement reliability is not a physical-ready candidate and
should not be promoted as IL data without a clean follow-up artifact.

Interpret the stage-1 rows structurally: open-loop radius can be good while
center drift is bad, closed-loop is structurally needed for rbpodo controller
simulation, Kp=2 was aggressive in previous stage, and Kp_pos and Kp_ori
should be tuned separately. Orientation feedback may worsen orientation drift.
`pub_rate=100` with `speed_bar=0.1` can destabilize; `speed_bar>=0.2` can
restore stability but still needs radius, center, orientation, and saturation
review.

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
Physical real motion remains blocked by unresolved diagnostics_suspect,
unverified stop/resetFault behavior, unmeasured physical reference-to-actual
tracking error, unresolved camera/TCP calibration, and missing tiny physical
acceptance.

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
`q_sent_moved`, `q_sent_update_rate_hz`, `q_ref_moved`,
`q_ref_update_rate_hz`, `q_ref_reason`, `q_actual_moved`,
`q_actual_update_rate_hz`, `tcp_ref_moved`, `tcp_actual_moved`, and
`physical_motion_detected`. Current state JSON publishes `q_ref_deg` as an
explicit alias for `q_target_deg`; `q_ref_reason: q_ref_deg not published`
indicates an older or incompatible state stream and must not be interpreted as
a static controller reference. In controller pgmode simulation,
`tcp_ref_moved` is the primary motion evidence
for the controller-reference path, while physical `q_actual` and
`tcp_actual_stand` are expected to remain stationary. Any physical `q_actual`
drift above the configured warning threshold is a warning in pgmode simulation,
not a success criterion.

Integrator diagnostics are recorded as `integrator_resets_total`,
`integrator_divergence_total`, `integrator_clamps_total`, `reset_rate_hz`, and
`divergence_rate_hz`. A high divergence count with stationary `q_actual`
produces the warning that Cartesian integration may need a reference-state
source; this warning is about controller-simulation diagnostics, not physical
motion.

For rbpodo controller pgmode simulation, the circle templates also set:

```yaml
safety:
  controller_simulation_tracking_error_source: reference
  controller_simulation_physical_motion_policy: fault_latch
  controller_simulation_physical_motion_threshold_deg: 0.05
```

This makes the server-side tracking-error guard compare the previous Servo J
target against the controller reference joint state instead of physical
`q_actual`, which is expected to remain stationary in pgmode simulation. This
is not a physical-real relaxation: `operation_mode: real` still uses
`q_actual`, and any physical `q_actual` movement beyond the configured
controller-simulation threshold is latched as a fault.

When reference tracking is active, servo-loop startup also seeds
`previous_sent`, `previous_previous_sent`, and the initial fault-hold target
from the controller reference joints (`q_target` / `q_ref`) instead of
physical `q_actual`. This prevents a stale controller-simulation reference
from producing an immediate startup `TrackingError` when physical `q_actual`
is stationary. The physical-motion baseline still comes from `q_actual`. If
the startup reference is non-finite or unavailable, startup fails closed with
`controller_simulation_startup_reference_unavailable`.

## Artifacts

Each run writes:

- `summary.json` and `summary.csv`
- `safety_preflight.json`
- `pgmode_summary.json`
- `raw_config.yaml`
- `state_stream.jsonl`
- `command_packets.jsonl`
- `overlay_stream.jsonl` when overlay publishing is enabled
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
Cartesian path from being attempted, `faulted` means the server latched a
safety fault during the run, and `error` means safety preflight or execution
failed. Faulted partial runs are not scored as completed tracking evidence.
