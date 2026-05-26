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

## GENE-Style Context

A GENE-style circle benchmark used a 15 cm diameter circle over 4 seconds. That
requires about 0.118 m/s tangential TCP speed:

```text
radius = 0.075 m
circumference = 2 * pi * 0.075 ~= 0.471 m
speed = 0.471 / 4 ~= 0.118 m/s
```

The current simulator TCP acceptance config may limit twist speed to about
0.03 m/s, so BENCH-CIRCLE-01 defaults to a slower conservative profile. The
15 cm / 4 s stress profile requires explicit opt-in and a compatible simulator
config.

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

## 15 cm / 4 s Stress

Use this only with a simulator config whose Cartesian speed limits are
compatible:

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
  --profile gene_15cm_4s \
  --allow-fast-stress \
  --repeat 1 \
  --command-rate-hz 100 \
  --artifact-dir artifacts/circle_tracking/gene_15cm_4s
```

The default `dual_simulator_tcp_acceptance.yaml` speed limit is not intended to
run this profile unchanged.

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

Passing this simulator benchmark does not permit real robot motion.
