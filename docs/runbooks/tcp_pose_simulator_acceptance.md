# Cartesian Simulator Acceptance Runbook

This runbook validates simulator-only Cartesian command behavior for the
dual-arm RB3-730 stack. It covers:

- `TcpPoseTarget`: point-to-point final pose; Cartesian path is not guaranteed.
- `TcpLinearMove`: MoveL-like Cartesian path primitive.
- `TcpTwistLocal` / `TcpTwistStand`: streaming Cartesian velocity primitives.

This is not real robot evidence. Do not set `RB_ALLOW_REAL_ROBOT`,
`RB_ALLOW_REAL_MOTION`, or `RB_ALLOW_REAL_CARTESIAN` for this runbook.

## Safety Contract

The scripted runner refuses to continue when the selected config contains:

- `run_mode: real`
- `backend_type: rbpodo`
- `cartesian_control.allow_in_real: true`
- real controller IPs `172.28.60.200` or `172.28.60.201`

The canonical config is
`rb_servo_server/config/dual_simulator_tcp_acceptance.yaml`. It enables
Pinocchio FK/IK and simulator Cartesian control while keeping
`cartesian_control.allow_in_real: false`.

## Dependencies

Build the Pinocchio-enabled server:

```bash
cmake -S rb_servo_server -B rb_servo_server/build/pinocchio_gate \
  -DCMAKE_BUILD_TYPE=Debug \
  -DRB_SERVO_ENABLE_RBPODO=OFF \
  -DRB_SERVO_ENABLE_PINOCCHIO=ON \
  -DRB_SERVO_ALLOW_FETCHCONTENT=OFF \
  -DBUILD_TESTING=ON
cmake --build rb_servo_server/build/pinocchio_gate -j
```

The script checks `scripts/check_deps.sh --profile hardware-free` and
`scripts/check_deps.sh --profile kinematics`. If Pinocchio is unavailable, the
runner fails before runtime acceptance. Use `--allow-missing-pinocchio` only for
preflight-only validation; it does not fake a pass.

## Run

Run all scenarios:

```bash
bash scripts/tcp_pose_simulator_acceptance.sh --all
```

Run a single scenario:

```bash
bash scripts/tcp_pose_simulator_acceptance.sh --run-linear \
  --artifact-dir artifacts/cartesian_acceptance/linear_debug
```

Useful flags:

```text
--run-ptp
--run-linear
--run-twist-local
--run-twist-stand
--all
--allow-missing-pinocchio
--skip-estop-reset
--artifact-dir DIR
--server-config PATH
--linear-duration-sec SEC
--orientation-tolerance-rad RAD
--line-tolerance-m M
```

Default thresholds:

- final TCP position tolerance: `0.004 m`
- quaternion angle tolerance: `0.005 rad`
- linear path line-deviation tolerance: `0.002 m`
- linear move duration: `1.0 s`
- twist stream: `30 Hz` for about `1.0 s`

## Acceptance Scenarios

### A. TcpPoseTarget PTP

The runner waits for valid FK TCP state, arms motion, sends a small absolute
left-arm `TcpPoseTarget`, and verifies:

- command verdict is `Ok`
- no fault latch
- final position error is below threshold
- final quaternion angle error is below threshold
- `q_sent_deg` is finite and inside configured joint limits

Path linearity is intentionally not checked for PTP.

### B. TcpLinearMove Constant Orientation

The runner captures the initial left TCP pose, sends a `TcpLinearMove` with the
same quaternion and a small position offset, then samples state throughout the
move. It verifies:

- `cartesian_solve.path_s` increases toward `1.0`
- `path_done` becomes true
- final position and orientation errors are below thresholds
- max distance to the start-target line is below `--line-tolerance-m`
- max quaternion angle from the start orientation is below
  `--orientation-tolerance-rad`
- no fault latch

### C. TcpLinearMove Slerp

The runner sends a small target orientation change with `orientation_mode:
slerp`. It verifies final orientation reaches the target within tolerance and
intermediate orientation progress remains finite and monotonic within a small
drop allowance.

### D. TcpTwistLocal Orientation Hold

The runner streams `TcpTwistLocal` at 30 Hz with positive local `vx` and zero
angular velocity. It verifies:

- TCP moves primarily along the initial TCP local +X direction
- orientation remains within the quaternion angle threshold
- no fault latch
- command source stays valid through repeated twist packets

### E. TcpTwistStand Frame Conversion

The runner streams `TcpTwistStand` with stand-frame +X velocity and zero angular
velocity. It verifies motion projects along stand +X and orientation remains
within threshold. This covers the lower-level stand-frame twist API path.

## Artifacts

Artifacts are written under `artifacts/cartesian_acceptance/<timestamp>/` unless
`--artifact-dir` is supplied:

```text
summary.json
tcp_pose_acceptance_summary.json
state_stream.jsonl
command_packets.jsonl
rb_servo_server.log
left_simulator.log
right_simulator.log
servo_log.csv
path_samples_left.csv
path_samples_right.csv
```

`summary.json` includes pass/fail per scenario, maximum position/orientation
errors, maximum line deviation, maximum path tracking error, IK timing, detected
faults, config path, and git commit.

Use `command_packets.jsonl` to replay or inspect what was sent. Use
`path_samples_left.csv` for plotting path shape and `state_stream.jsonl` for
raw telemetry.

## Codex Gate

The lightweight gate runs syntax/tool/docs checks:

```bash
./scripts/codex_gate.sh CART-HARDEN-05
```

The full simulator acceptance is intentionally opt-in:

```bash
CODEX_RUN_CARTESIAN_ACCEPTANCE=1 ./scripts/codex_gate.sh CART-HARDEN-05
```

## Rerun Failed Scenario

Inspect `summary.json` for the failed scenario name, then rerun only that path:

```bash
bash scripts/tcp_pose_simulator_acceptance.sh --run-twist-local \
  --artifact-dir artifacts/cartesian_acceptance/twist_local_retry
```

The retry still uses only simulator endpoints and must not be pointed at real
robot configs.
