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

The runner refuses configs containing `run_mode: real`, `backend_type: rbpodo`,
`allow_in_real: true`, or the real robot IPs `172.28.60.200` /
`172.28.60.201`.

## Profiles

Run profiles from slowest/safest to fastest/stress:

| Profile | Diameter | Period | Required speed | Expected use case |
| --- | ---: | ---: | ---: | --- |
| `safe_5cm_10s` | 0.05 m | 10 s | 0.016 m/s | conservative simulator smoke/regression baseline |
| `circle_15cm_16s` | 0.15 m | 16 s | 0.029 m/s | 15 cm circle within the default 0.03 m/s twist limit |
| `circle_15cm_8s` | 0.15 m | 8 s | 0.059 m/s | 15 cm simulator stress below GENE-style speed |
| `gene_15cm_4s` | 0.15 m | 4 s | 0.118 m/s | explicit GENE-style simulator-only stress |

Recommended regression sequence:

1. Run `safe_5cm_10s`.
2. Run `circle_15cm_16s`.
3. Run `circle_15cm_8s` with a config whose simulator speed limits allow it.
4. Run `gene_15cm_4s` only with `dual_simulator_circle_stress.yaml` and
   `--allow-fast-stress`.

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
rb_servo_server/config/dual_simulator_circle_stress.yaml
```

## Safe Baseline

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
  --server-config rb_servo_server/config/dual_simulator_tcp_acceptance.yaml \
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
whose speed limits allow it, such as `dual_simulator_circle_stress.yaml`:

```bash
python3 scripts/circle_tracking_benchmark.py \
  --root . \
  --mode start-local \
  --server rb_servo_server/build/hardware_free_gate/rb_servo_server \
  --server-config rb_servo_server/config/dual_simulator_circle_stress.yaml \
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

GENE-style stress requires `dual_simulator_circle_stress.yaml`. This config is
simulator-only, keeps `allow_in_real: false`, and raises only simulator
Cartesian speed limits enough for the 15 cm / 4 s profile:

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
  --server-config rb_servo_server/config/dual_simulator_circle_stress.yaml \
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
  --server-config rb_servo_server/config/dual_simulator_circle_stress.yaml \
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
- `linear_segments`: approximates the circle with `TcpLinearMove` waypoints and
  evaluates MoveL-like polyline tracking.

## Artifacts

The artifact directory contains:

- `summary.json` and `summary.csv`
- `reference.csv`, `actual.csv`, and `merged_samples.csv`
- `command_packets.jsonl` and `state_stream.jsonl`
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

The GENE-style gate path uses `dual_simulator_circle_stress.yaml`.

Passing this simulator benchmark does not permit real robot motion.
