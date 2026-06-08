# Circle Tracking Benchmark Runbook

BENCH-CIRCLE-01 is a simulator-only Cartesian tracking benchmark for circular
TCP motion. It measures tracking quality, writes CSV/JSON artifacts, and
generates plots for comparing controller behavior across runs.

This is not real robot evidence. Do not set:

```bash
RB_ALLOW_REAL_ROBOT=1
RB_ALLOW_REAL_MOTION=1
RB_ALLOW_REAL_CARTESIAN=1
```

For rbpodo controller-box pgmode simulation, use
`scripts/rbpodo_circle_tracking_benchmark.py` and
`docs/runbooks/rbpodo_controller_sim_circle.md` instead. That runner connects to
real Rainbow controller boxes, requires explicit pgmode simulation
confirmation, and scores controller-reference `tcp_ref_stand` telemetry rather
than hardware-free simulator TCP state.

Live visualization for the rbpodo controller-simulation runner uses two UDP
telemetry paths: server-side state fanout for `rb_servo_server` state JSON, and
an optional benchmark overlay stream from the benchmark script. The overlay is
published with `--overlay-pub-endpoint` / `--overlay-pub-rate-hz` and carries
desired circle geometry plus running metrics only. It is not a command channel
and does not replace captured state samples for benchmark scoring.

Keep the three evidence categories separate:

- `rb_simulator` software simulation: hardware-free, this runbook.
- Rainbow controller `pgmode` simulation through `rbpodo`: real controller
  boxes with `run_mode: real`, `backend_type: rbpodo`,
  `operation_mode: simulation`, `physical_motion_expected=false`, and
  `tcp_ref_stand` tracking.
- future physical real robot: not covered by either circle benchmark runbook.

Every new summary and report row must carry canonical lane metadata:

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

The lane separates where the trajectory and feedback loop run from how the
low-level send is accepted. Python streaming feedback remains a separate lane
from server-side circle tracking even if its tracking metrics are good.

For the rbpodo controller-simulation category, streaming Cartesian primitives
require `cartesian_control.allow_in_controller_simulation: true` and
`RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1`. Servo J ACKs alone do not imply a
circle was executed; a blocked server can ACK repeated hold targets while
`cartesian_solve.status` reports `unavailable`. A singular circle fit in that
case is a gate/configuration failure, not controller tracking performance.

The runner refuses configs containing `run_mode: real`, `backend_type: rbpodo`,
`allow_in_real: true`, or the real robot IPs `172.28.60.200` /
`172.28.60.201`.

## Profiles

Run profiles from slowest/safest to fastest/stress:

| Profile | Diameter | Period | Required speed | Expected use case |
| --- | ---: | ---: | ---: | --- |
| `safe_5cm_10s` | 0.05 m | 10 s | 0.016 m/s | bring-up |
| `circle_15cm_16s` | 0.15 m | 16 s | 0.029 m/s | stable baseline |
| `circle_15cm_8s` | 0.15 m | 8 s | 0.059 m/s | middle speed ablation |
| `gene_15cm_4s` | 0.15 m | 4 s | 0.118 m/s | GENE-style stress |

Recommended regression sequence:

1. Run `safe_5cm_10s`.
2. Run `circle_15cm_16s` with
   `rb_servo_server/config/dual_simulator_circle_baseline_15cm16s.yaml`.
3. Run `circle_15cm_8s` with a config whose simulator speed limits allow it.
4. Run `gene_15cm_4s` only with
   `rb_servo_server/config/dual_simulator_circle_stress_15cm4s.yaml` and
   `--allow-fast-stress`.

The `circle_15cm_8s` profile is the middle-speed split between the stable
16 s baseline and the GENE-style 4 s stress case. Use it to isolate whether a
4 s failure is caused by bandwidth, latency, saturation, or limits rather than
by basic 15 cm tracking geometry.

## Named Server Config Profiles

Use named simulator profiles when recording benchmark evidence so later runs
can reproduce the same parameter set.

| Config profile | Purpose | Speed envelope | Use before real? |
| --- | --- | --- | --- |
| `dual_simulator_circle_baseline_15cm16s.yaml` | `simulator_baseline` for `circle_15cm_16s` | 15 cm / 16 s, about 0.029 m/s | Yes, as simulator evidence only |
| `dual_simulator_circle_stress_15cm4s.yaml` | `simulator_stress` for `gene_15cm_4s` | 15 cm / 4 s, about 0.118 m/s | No; stress and repeatability evidence only |
| `dual_simulator_circle_real_candidate_conservative.yaml` | `real_candidate_conservative` seed profile, still simulator-only | Low-speed candidate, not faster than the baseline limit | Only as a starting guess after separate real acceptance |

All three configs use simulator backends, simulation run mode, local simulator
endpoints, and `cartesian_control.allow_in_real: false`. None of them is a
real robot config. These named circle configs also enable
`cartesian_control.enable_benchmark_primitives` so the optional `server_circle`
diagnostic can run in simulation. The general TCP acceptance config keeps
benchmark primitives disabled.

The older `dual_simulator_tcp_acceptance.yaml` remains the general Cartesian
acceptance config. The older `dual_simulator_circle_stress.yaml` remains for
compatibility, but new circle tracking evidence should prefer the named
profiles above.

## GENE-Style Context

A GENE-style circle benchmark used a 15 cm diameter circle over 4 seconds. That
requires about 0.118 m/s tangential TCP speed:

```text
radius = 0.075 m
circumference = 2 * pi * 0.075 ~= 0.471 m
speed = 0.471 / 4 ~= 0.118 m/s
```

The current simulator TCP acceptance config limits twist speed to about
0.03 m/s, so BENCH-CIRCLE-01 defaults to a slower conservative profile. The
15 cm / 4 s stress profile requires explicit opt-in and the dedicated
simulator-only stress config:

```text
rb_servo_server/config/dual_simulator_circle_stress_15cm4s.yaml
```

## Safe Baseline

```bash
python3 scripts/circle_tracking_benchmark.py \
  --root . \
  --mode start-local \
  --server rb_servo_server/build/hardware_free_gate/rb_servo_server \
  --server-config rb_servo_server/config/dual_simulator_circle_baseline_15cm16s.yaml \
  --left-config rb_simulator/config/left_rb3_730e.yaml \
  --right-config rb_simulator/config/right_rb3_730e.yaml \
  --arm left \
  --controller twist_stand \
  --plane xy \
  --profile safe_5cm_10s \
  --repeat 3 \
  --command-rate-hz 100 \
  --artifact-dir artifacts/circle_tracking/safe_5cm_10s
```

## 15 cm / 16 s Limit-Compatible Run

```bash
python3 scripts/circle_tracking_benchmark.py \
  --root . \
  --mode start-local \
  --server rb_servo_server/build/hardware_free_gate/rb_servo_server \
  --server-config rb_servo_server/config/dual_simulator_circle_baseline_15cm16s.yaml \
  --left-config rb_simulator/config/left_rb3_730e.yaml \
  --right-config rb_simulator/config/right_rb3_730e.yaml \
  --arm left \
  --controller twist_stand \
  --plane xy \
  --profile circle_15cm_16s \
  --repeat 3 \
  --command-rate-hz 100 \
  --artifact-dir artifacts/circle_tracking/circle_15cm_16s
```

Post-fix comparison-friendly artifact naming:

```bash
python3 scripts/circle_tracking_benchmark.py \
  --root . \
  --mode start-local \
  --server rb_servo_server/build/hardware_free_gate/rb_servo_server \
  --server-config rb_servo_server/config/dual_simulator_tcp_acceptance.yaml \
  --left-config rb_simulator/config/left_rb3_730e.yaml \
  --right-config rb_simulator/config/right_rb3_730e.yaml \
  --arm left \
  --controller twist_stand \
  --profile circle_15cm_16s \
  --repeat 3 \
  --artifact-dir artifacts/circle_tracking/left_twist_stand_15cm_16s
```

Equivalent explicit dimensions:

```bash
python3 scripts/circle_tracking_benchmark.py \
  --root . \
  --mode start-local \
  --server rb_servo_server/build/hardware_free_gate/rb_servo_server \
  --server-config rb_servo_server/config/dual_simulator_tcp_acceptance.yaml \
  --left-config rb_simulator/config/left_rb3_730e.yaml \
  --right-config rb_simulator/config/right_rb3_730e.yaml \
  --arm left \
  --controller twist_stand \
  --plane xy \
  --diameter-m 0.15 \
  --period-sec 16.0 \
  --repeat 3 \
  --command-rate-hz 100 \
  --artifact-dir artifacts/circle_tracking/example
```

## 15 cm / 8 s Simulator Stress

This profile requires about 0.059 m/s, above the default
`dual_simulator_tcp_acceptance.yaml` twist limit. Use a simulator-only config
whose speed limits allow it, such as `dual_simulator_circle_stress_15cm4s.yaml`:

```bash
python3 scripts/circle_tracking_benchmark.py \
  --root . \
  --mode start-local \
  --server rb_servo_server/build/hardware_free_gate/rb_servo_server \
  --server-config rb_servo_server/config/dual_simulator_circle_stress_15cm4s.yaml \
  --left-config rb_simulator/config/left_rb3_730e.yaml \
  --right-config rb_simulator/config/right_rb3_730e.yaml \
  --arm left \
  --controller twist_stand \
  --plane xy \
  --profile circle_15cm_8s \
  --repeat 3 \
  --artifact-dir artifacts/circle_tracking/left_twist_stand_15cm_8s
```

## 15 cm / 4 s Stress

GENE-style stress requires `dual_simulator_circle_stress_15cm4s.yaml`. This
config is simulator-only, keeps `allow_in_real: false`, and raises only
simulator Cartesian speed limits enough for the 15 cm / 4 s profile:

```yaml
cartesian_control:
  max_twist_linear_m_s: 0.15
  max_linear_move_speed_m_s: 0.15
  max_twist_angular_rad_s: 0.4
  exceed_limit_policy: clamp
```

```bash
python3 scripts/circle_tracking_benchmark.py \
  --root . \
  --mode start-local \
  --server rb_servo_server/build/hardware_free_gate/rb_servo_server \
  --server-config rb_servo_server/config/dual_simulator_circle_stress_15cm4s.yaml \
  --left-config rb_simulator/config/left_rb3_730e.yaml \
  --right-config rb_simulator/config/right_rb3_730e.yaml \
  --arm left \
  --controller twist_stand \
  --plane xy \
  --profile gene_15cm_4s \
  --allow-fast-stress \
  --repeat 1 \
  --command-rate-hz 100 \
  --artifact-dir artifacts/circle_tracking/gene_15cm_4s
```

The default `dual_simulator_tcp_acceptance.yaml` speed limit is not intended to
run this profile unchanged; preflight should reject that default config for
`gene_15cm_4s`.

For a post-fix GENE-style comparison run:

```bash
python3 scripts/circle_tracking_benchmark.py \
  --root . \
  --mode start-local \
  --server rb_servo_server/build/hardware_free_gate/rb_servo_server \
  --server-config rb_servo_server/config/dual_simulator_circle_stress_15cm4s.yaml \
  --left-config rb_simulator/config/left_rb3_730e.yaml \
  --right-config rb_simulator/config/right_rb3_730e.yaml \
  --arm left \
  --controller twist_stand \
  --profile gene_15cm_4s \
  --allow-fast-stress \
  --repeat 5 \
  --artifact-dir artifacts/circle_tracking/gene_15cm_4s_after_servo_fix
```

## Controllers

- `twist_stand`: streams `TcpTwistStand` circular velocity in the selected
  stand-frame plane.
- `twist_local`: streams `TcpTwistLocal` circular velocity in the initial TCP
  local plane and compares against the corresponding stand-frame reference.
- `twist_stand_feedback`: streams `TcpTwistStand` with command-source position
  feedback added to the feedforward circular velocity.
- `twist_local_feedback`: streams `TcpTwistLocal` with the same stand-frame
  feedback law, then converts the command to the current TCP local frame using
  `R_current^T`.
- `server_circle`: sends one simulation-only `TcpCircleMove` command, then
  observes the server-generated circular trajectory. This isolates the servo
  loop and Cartesian controller from Python UDP streaming jitter.
- `linear_segments`: approximates the circle with `TcpLinearMove` waypoints and
  evaluates MoveL-like polyline tracking.

Official terminology for `server_circle` is server-side circle tracking. The
implemented benchmark command is `TcpCircleMove`. `TcpCircleTrack` is a
reserved closed-loop server-side circle schema in the servo server, not the
current benchmark command. Both names are reported as
`command_family: server_side_circle` when they appear in command/state metadata.
This preserves backward compatibility while making report grouping unambiguous.

Open-loop twist modes benchmark server-side velocity tracking and plant
integration. Feedback twist modes benchmark closed-loop command-source
compensation. Feedback can reduce center drift, but it can also hide
server/controller limitations, so compare feedback runs against matching
open-loop runs instead of replacing them.

`server_circle` is a diagnostic benchmark primitive, not a robot command for
hardware. It requires `cartesian_control.enable_benchmark_primitives: true`,
`cartesian_control.circle_move.allow_in_simulation: true`, and
`cartesian_control.circle_move.allow_in_real: false`. Real mode rejects
`TcpCircleMove` even if Cartesian real-motion environment gates are set. The
primitive captures the current TCP pose, chooses the circle center so the
reference starts at that pose, holds the initial orientation, and generates the
circle reference once per servo tick.

Feedback command law:

```text
v_cmd_stand = v_ref_stand + feedback_kp_pos * (p_ref_stand - p_actual_stand)
w_cmd_stand = feedback_kp_ori * orientation_error_stand
```

The reference orientation is the initial TCP orientation. `twist_local_feedback`
computes the reference and feedback error in stand frame, then converts the
linear and angular command vectors to the current local frame. If state is stale
or invalid, the benchmark sends zero twist for that tick and increments
`stale_state_feedback_skips`.

Conservative feedback defaults:

```text
--feedback-kp-pos 2.0
--feedback-kp-ori 2.0
```

By default, feedback command clamp limits are taken from the server config
`max_twist_linear_m_s` and `max_twist_angular_rad_s`. They can be narrowed with
`--feedback-max-linear-m-s` and `--feedback-max-angular-rad-s`.

Example closed-loop baseline:

```bash
python3 scripts/circle_tracking_benchmark.py \
  --root . \
  --mode start-local \
  --server rb_servo_server/build/hardware_free_gate/rb_servo_server \
  --server-config rb_servo_server/config/dual_simulator_circle_baseline_15cm16s.yaml \
  --left-config rb_simulator/config/left_rb3_730e.yaml \
  --right-config rb_simulator/config/right_rb3_730e.yaml \
  --arm left \
  --controller twist_stand_feedback \
  --plane xy \
  --profile circle_15cm_16s \
  --repeat 3 \
  --command-rate-hz 100 \
  --artifact-dir artifacts/circle_tracking/left_twist_stand_feedback_15cm_16s
```

Example server-side circle diagnostic:

```bash
python3 scripts/circle_tracking_benchmark.py \
  --root . \
  --mode start-local \
  --server rb_servo_server/build/hardware_free_gate/rb_servo_server \
  --server-config rb_servo_server/config/dual_simulator_circle_baseline_15cm16s.yaml \
  --left-config rb_simulator/config/left_rb3_730e.yaml \
  --right-config rb_simulator/config/right_rb3_730e.yaml \
  --arm left \
  --controller server_circle \
  --plane xy \
  --profile circle_15cm_16s \
  --repeat 3 \
  --artifact-dir artifacts/circle_tracking/left_server_circle_15cm_16s
```

## Artifacts

The artifact directory contains:

- `summary.json` and `summary.csv`
- `reference.csv`, `actual.csv`, and `merged_samples.csv`
- `command_packets.jsonl` and `state_stream.jsonl`
- `feedback_terms.jsonl` and `feedback_terms.csv` for feedback controllers
- `circle_trajectory.png`
- `tracking_error_time.png`
- `orientation_drift_time.png`
- `phase_lag_time.png` when the phase estimate is reliable
- `radial_error_time.png`
- `axis_positions_time.png`
- `rb_servo_server.log`, `left_simulator.log`, and `right_simulator.log`

## Metrics

`summary.json` records position error mean/RMS/median/p95/max, quaternion angle
drift mean/p95/max, estimated phase lag and latency, least-squares fitted
circle radius/center, sample and command counts, worker drop/overwrite telemetry
when present, state age, send result age, IK timing, fault state, and threshold
pass/fail if threshold flags were supplied.

Profile metadata is serialized with each summary: `diameter_m`, `period_sec`,
`required_tangential_speed_m_s`, `angular_frequency_rad_s`,
`recommended_controller` / `recommended_controllers`, and `stress_level`.

Feedback runs also record:

- `mean_feedback_linear_norm_m_s`
- `max_feedback_linear_norm_m_s`
- `mean_total_command_linear_norm_m_s`
- `feedback_saturation_count`
- `stale_state_feedback_skips`

Each feedback tick records `feedforward_twist`, `feedback_twist`,
`applied_twist`, `position_error_vector`, and `orientation_error_vector` fields
in `feedback_terms.jsonl` / `.csv`.

Server-side circle runs publish circle debug telemetry in `state_stream.jsonl`
and summary samples when available:

- `circle_active`
- `circle_phase`
- `circle_repeat_index`
- `circle_radius_m`
- `circle_period_sec`
- `circle_position_error_m`
- `circle_orientation_error_rad`
- `circle_done`

Result semantics:

- `completed`: the simulator-only run finished, but no performance thresholds
  were supplied. This is not a tracking-quality pass.
- `pass`: thresholds were supplied and all were satisfied.
- `fail`: thresholds were supplied and at least one failed; when thresholds are
  active, a fault latch is treated as a failure.
- `error`: the benchmark could not run.

Performance warnings are emitted even without thresholds. They highlight severe
attenuation or drift, such as `radius_gain < 0.8`, `rms_error_m` greater than
half the reference radius, `p95_error_m` greater than the reference radius, or
large orientation drift.

`radius_gain = fit_radius_m / reference_radius_m` describes circle amplitude
tracking. `radius_gain ~= 1.0` means the actual fitted circle radius matched the
reference. `radius_gain ~= 0.2` means severe attenuation: the TCP moved only
about 20% of the requested circle radius.

The summary also records simulator dynamics context when it can be parsed:
`configured_max_twist_linear_m_s`, `configured_max_linear_move_speed_m_s`,
`simulator_motion_time_constant_sec`, `servo_rate_hz`, `servo_dt_sec`, and
`simulator_dt_over_tau`. A first-order simulator with servo `dt ~= 0.01 s` and
`motion_time_constant_sec ~= 0.04 s` has `dt/tau ~= 0.25`; that dynamics context
helps explain the attenuation seen with legacy measured-actual one-step target
integration.

Timing telemetry uses explicit names. `send_within_period` means the send
completed inside the loop period. `send_period_overrun` means it did not. The
older `send_deadline_hit` field is kept as a deprecated alias for
`send_within_period` for compatibility.

Quaternion distance treats `q` and `-q` as the same orientation:

```text
angle = 2 * acos(clamp(abs(dot(q0, q1)), -1, 1))
```

## Comparing Runs

Compare runs with the same profile, controller, arm, plane, command rate, and
server config. Track at least:

- `rms_error_m`
- `p95_error_m`
- `p95_orientation_drift_rad`
- `estimated_latency_ms`
- `fit_radius_m`
- `radius_error_m`
- `worker_command_drops_total`
- `fault_latched`

To separate open-loop velocity drift from command-source feedback compensation,
compare matched pairs such as:

```bash
python3 scripts/compare_circle_benchmarks.py \
  artifacts/circle_tracking/left_twist_stand_15cm_16s/summary.json \
  artifacts/circle_tracking/left_twist_stand_feedback_15cm_16s/summary.json
```

If feedback reduces center drift but requires frequent saturation, the issue is
not solved; the run is showing that command-source feedback is masking a limit
or timing problem.

To separate Python command-stream timing from server-side trajectory tracking,
compare `twist_stand` and `server_circle` with the same profile and server
config. If `server_circle` is materially better than `twist_stand`, command
interval jitter or UDP sender timing is a likely contributor. If both runs show
similar error, focus on server-side controller gains, limits, simulator dynamics,
and SafetyFilter clamp telemetry.

Use the comparison helper to summarize multiple `summary.json` artifacts:

```bash
python3 scripts/compare_circle_benchmarks.py \
  artifacts/circle_tracking/gene_15cm_4s_before_servo_fix/summary.json \
  artifacts/circle_tracking/gene_15cm_4s_after_servo_fix/summary.json
```

Sort by RMS error and write a CSV:

```bash
python3 scripts/compare_circle_benchmarks.py \
  --sort rms_error \
  --csv artifacts/circle_tracking/comparison.csv \
  artifacts/circle_tracking/*/summary.json
```

## Reporting Workflow

BENCH-REPORT-01 turns `summary.json` artifacts into a markdown report, CSV
table, and promotion classification. Use it when a run is meant to influence
controller parameters or `REVIEW.md`.

```bash
python3 scripts/generate_circle_benchmark_report.py \
  artifacts/circle_tracking/left_twist_stand_15cm_16s_after/summary.json \
  artifacts/circle_tracking/gene_15cm_4s_after/summary.json \
  --output-md artifacts/circle_tracking/circle_benchmark_report.md \
  --csv artifacts/circle_tracking/circle_benchmark_report.csv
```

The report includes controller/profile metadata, `diameter_m`, `period_sec`,
`required_tangential_speed_m_s`, `stress_level`, radius gain, position error,
orientation drift, estimated latency, worker drops, integrator clamp/divergence
counts, timing jitter when available, benchmark result, performance warnings,
and a classification:

- `stable_simulator_baseline_candidate`
- `stress_benchmark_candidate`
- `not_baseline_candidate`
- `stress_rejected_or_incomplete`
- `informational`

`completed` still means the run finished without performance thresholds. It is
not the same as `pass`. `pass` requires explicit thresholds supplied to the
benchmark and satisfied by the artifact.

## Baseline Promotion Rules

A stable simulator baseline candidate must satisfy all of:

- profile `circle_15cm_16s`
- `radius_gain >= 0.98`
- `rms_error_m <= 0.005`
- `p95_error_m <= 0.006`
- `max_orientation_drift_rad <= 0.005`
- `worker_command_drops_total == 0`
- `send_command_deadline_missed_count == 0`
- `fault_latched == false`
- repeated at least three times, either as `repeat >= 3` in one artifact or as
  matching repeated artifacts

Record baseline candidates in `REVIEW.md` with the artifact path, controller,
profile, repeat evidence, radius gain, RMS error, p95 error, result, and a
simulator-only caveat. Do not claim a benchmark passed unless the artifact
result is `pass` or the note explicitly says it is only a completed run that
meets promotion criteria.

## Stress Interpretation

The `gene_15cm_4s` profile is stress evidence: stress is not real-ready.

A stress benchmark candidate should have:

- profile `gene_15cm_4s`
- `radius_gain >= 0.90`
- `fault_latched == false`
- `worker_command_drops_total == 0`
- `integrator_divergence_total == 0`

Stress runs keep error metrics for comparison and tuning, but those metrics do
not make the parameter set real-ready. A good 15 cm / 4 s plot can show that the
simulator/controller stack is improving; it cannot authorize hardware speed,
aggressive gains, or real Cartesian motion.

## Real-Candidate Policy

Simulator parameters can seed real testing only if:

- they come from a stable simulator baseline, not the stress profile
- speeds are scaled down for real
- real read-only acceptance has passed
- real tiny joint motion acceptance has passed
- real tiny Cartesian PTP acceptance has passed

Do not copy directly to hardware:

- simulator `motion_time_constant_sec`
- aggressive stress gains
- GENE-style 4 s speed
- Python sender timing assumptions

Copy cautiously:

- frame conventions
- conservative path gains
- command-source lease/deadman requirements
- telemetry thresholds
- safety gates

## Ablation Workflow

BENCH-ABLATION-01 runs factor-separated simulator-only experiments from a YAML
matrix. Use it to understand which settings improve or degrade tracking rather
than to tune the 15 cm / 4 s stress case by accident.

The current stable simulator baseline is `circle_15cm_16s`: it uses the same
15 cm diameter as the GENE-style stress but stays within the default
0.03 m/s twist limit. The `gene_15cm_4s` profile is a stress case and requires
`dual_simulator_circle_stress_15cm4s.yaml` plus explicit
`allow_fast_stress: true`.

Example matrix:

```yaml
experiments:
  - name: baseline_15cm_16s_twist_stand
    profile: circle_15cm_16s
    controller: twist_stand
    arm: left
    command_rate_hz: 500
    repeat: 3
    server_config: rb_servo_server/config/dual_simulator_circle_baseline_15cm16s.yaml

  - name: stress_15cm_4s_twist_stand
    profile: gene_15cm_4s
    allow_fast_stress: true
    controller: twist_stand
    arm: left
    command_rate_hz: 500
    repeat: 3
    server_config: rb_servo_server/config/dual_simulator_circle_stress_15cm4s.yaml

  - name: stress_15cm_4s_twist_stand_500hz
    profile: gene_15cm_4s
    allow_fast_stress: true
    controller: twist_stand
    arm: left
    command_rate_hz: 500
    server_config: rb_servo_server/config/dual_simulator_circle_stress_15cm4s.yaml
    server_config_overrides:
      servo.rate_hz: 500
      network.state_pub_rate_hz: 50
      cartesian_control.velocity_target_integration: previous_command
```

Run the matrix:

```bash
python3 scripts/run_circle_ablation.py \
  --root . \
  --matrix configs/circle_ablation_baseline.yaml \
  --artifact-root artifacts/circle_tracking/ablation_001 \
  --max-workers 1
```

The ablation runner writes one benchmark artifact directory per experiment and
aggregate files at the artifact root:

- `ablation_summary.csv`
- `ablation_summary.json`
- `ablation_report.md`
- `rms_error_by_experiment.png`
- `radius_gain_by_experiment.png`
- `p95_error_by_experiment.png`
- `latency_by_experiment.png`
- optional jitter and center-drift plots when those fields exist

Supported matrix factors include controller/profile/arm/command rate/repeat,
temporary server config overlays for `servo.rate_hz`,
`network.state_pub_rate_hz`, Cartesian velocity integration mode, lookahead,
path gains, damping, twist limits, and command-actual error limits. Simulator
config overlays can vary `simulator.motion_time_constant_sec` when needed. The
runner saves generated configs inside each experiment artifact directory and
does not modify source configs.

Interpretation rules:

- An experiment is not a performance `pass` unless thresholds were supplied.
- `radius_gain < 0.95` is an ablation warning.
- For `gene_15cm_4s`, `rms_error_mm > 10` or `p95_error_mm > 20` is a stress
  warning.
- Any command drop, deadline miss, safety clamp, fault, or large timing jitter
  should reject that parameter set as a stable candidate.

Promoting a parameter set toward a real-candidate discussion requires more than
a good stress plot: it must pass the `circle_15cm_16s` baseline, show no drops,
faults, or command deadline misses, keep clamp counts low, and be scaled down
before any real robot plan. Passing simulator ablation does not authorize real
robot motion.

## Parameter Transfer Policy

Transferable from simulator evidence only as initial guesses:

- controller structure, command sequencing, and telemetry fields
- conservative gains from `dual_simulator_circle_real_candidate_conservative.yaml`
- speed limits after scaling down for a future real test plan
- benchmark thresholds used to decide whether a simulator run is repeatable

Not directly transferable to hardware:

- simulator `motion_time_constant_sec`
- aggressive `gene_15cm_4s` stress speed
- lookahead values tuned specifically to simulator lag
- Python sender timing artifacts, command interval jitter, or host scheduling
  quirks

Real testing still requires separate real read-only acceptance, real motion
acceptance, and a future task that explicitly opens tiny Cartesian motion.

For CART-SERVO-01 and later, `TcpTwist*` and `TcpLinearMove` velocity-level
targets should use `cartesian_control.velocity_target_integration:
previous_command` in simulator acceptance configs. This avoids the first-order
simulator plant attenuation seen when a velocity command is converted into only
a one-tick-ahead target from measured `q_actual`. The benchmark state stream
and summary should be checked for the Cartesian solve telemetry fields
`cartesian_velocity_integration_mode`, `q_integrator_valid`,
`integrator_resets_total`, `integrator_clamps_total`, and
`integrator_divergence_total` when comparing pre/post controller runs.

When a benchmark run is used as review evidence, add the artifact path and a
short result note to `REVIEW.md`. Do not mark BENCH-CIRCLE-01 passed unless the
benchmark was actually run and its artifacts were preserved.

## Codex Gate

Static/default gate:

```bash
./scripts/codex_gate.sh BENCH-CIRCLE-01
```

Run the conservative benchmark through the gate:

```bash
CODEX_RUN_CIRCLE_BENCHMARK=1 ./scripts/codex_gate.sh BENCH-CIRCLE-01
```

Run the GENE-style stress only with explicit opt-in and compatible speed limits:

```bash
CODEX_RUN_GENE_STYLE_CIRCLE=1 ./scripts/codex_gate.sh BENCH-CIRCLE-01
```

The GENE-style gate path uses `dual_simulator_circle_stress_15cm4s.yaml`.

Passing this simulator benchmark does not permit real robot motion.
