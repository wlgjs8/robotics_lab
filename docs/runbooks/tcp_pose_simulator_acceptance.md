# Cartesian Simulator Acceptance Runbook

This runbook validates simulator-only Cartesian behavior for the dual-arm RB3-730 stack.

It covers:

- `TcpPoseTarget`: point-to-point final-pose target
- `TcpLinearMove`: MoveL-like Cartesian path primitive
- `TcpTwistLocal`: local-frame Cartesian velocity with orientation hold
- `TcpTwistStand`: stand-frame Cartesian velocity

This is not real robot evidence.

## Safety Contract

Do not set:

```bash
RB_ALLOW_REAL_ROBOT=1
RB_ALLOW_REAL_MOTION=1
RB_ALLOW_REAL_CARTESIAN=1
```

The runner must refuse configs containing:

- `run_mode: real`
- `backend_type: rbpodo`
- `cartesian_control.allow_in_real: true`
- real controller IPs `172.28.60.200` or `172.28.60.201`

Canonical simulator config:

```text
rb_servo_server/config/dual_simulator_tcp_acceptance.yaml
```

## Dependencies

Eigen3 and Pinocchio are mandatory for `rb_servo_server` Cartesian math. The
C++ math gate includes near-pi SO(3) log, quaternion convention, body-error, and
stand/local frame-conversion invariants.

Run the math rebaseline gate:

```bash
./scripts/codex_gate.sh CART-MATH-03
```

Build the server with mandatory Eigen3/Pinocchio support:

```bash
cmake -S rb_servo_server -B rb_servo_server/build/pinocchio_gate \
  -DCMAKE_BUILD_TYPE=Debug \
  -DRB_SERVO_ENABLE_RBPODO=OFF \
  -DRB_SERVO_ALLOW_FETCHCONTENT=OFF \
  -DBUILD_TESTING=ON
cmake --build rb_servo_server/build/pinocchio_gate -j
```

Run dependency preflight:

```bash
./scripts/check_deps.sh --profile hardware-free
```

## Run All Scenarios

```bash
bash scripts/tcp_pose_simulator_acceptance.sh --all
```

Through Codex gate:

```bash
CODEX_RUN_CARTESIAN_ACCEPTANCE=1 ./scripts/codex_gate.sh CART-HARDEN-05
```

## Run Selected Scenarios

```bash
bash scripts/tcp_pose_simulator_acceptance.sh --run-ptp
bash scripts/tcp_pose_simulator_acceptance.sh --run-linear
bash scripts/tcp_pose_simulator_acceptance.sh --run-twist-local
bash scripts/tcp_pose_simulator_acceptance.sh --run-twist-stand
bash scripts/tcp_pose_simulator_acceptance.sh --run-near-pi-ptp
```

Useful flags:

```text
--artifact-dir DIR
--server-config PATH
--linear-duration-sec SEC
--orientation-tolerance-rad RAD
--line-tolerance-m M
--run-near-pi-ptp          # optional near-pi TcpPoseTarget orientation scenario
--skip-estop-reset
--allow-missing-pinocchio   # preflight only; does not fake runtime acceptance
```

## Scenario A: `TcpPoseTarget` PTP

Expected behavior:

- send a small absolute TCP target
- final position error below threshold
- final quaternion angle error below threshold
- no path linearity requirement
- no fault latch

This scenario verifies final-pose targeting, not MoveL behavior.

Optional near-pi PTP acceptance can be run with `--run-near-pi-ptp`. It targets
a rotation of approximately `pi - 1e-6` radians and is simulator-only evidence.

## Scenario B: `TcpLinearMove` Constant Orientation

Expected behavior:

- TCP position follows a straight line from start to goal
- orientation remains close to start orientation
- `path_s` progresses toward `1.0`
- `path_done` becomes visible
- max line deviation below threshold
- no fault latch

This is the core MoveL-like acceptance path.

## Scenario C: `TcpLinearMove` Slerp

Expected behavior:

- TCP position follows a straight line
- orientation interpolates from start to target
- final orientation reaches target within threshold
- orientation progress remains finite and monotonic within tolerance

If slerp is not implemented in the running binary, the runner should fail or explicitly skip only when a skip flag says so.

## Scenario D: `TcpTwistLocal` Orientation Hold

Expected behavior:

- stream positive local `vx`
- zero angular velocity
- TCP moves primarily along initial local +X
- orientation remains within quaternion threshold
- no fault latch

This is the SpaceMouse-style Cartesian servo check.

## Scenario E: `TcpTwistStand` Frame Conversion

Expected behavior:

- stream stand-frame +X velocity
- TCP moves primarily along stand +X
- orientation remains within quaternion threshold
- no fault latch

## Quaternion Distance

Use:

```text
angle = 2 * acos(clamp(abs(dot(q0, q1)), -1, 1))
```

Treat `q` and `-q` as the same orientation.

## Artifacts

Artifacts are written to `artifacts/cartesian_acceptance/<timestamp>/` unless `--artifact-dir` is provided.

Expected files:

```text
summary.json
state_stream.jsonl
command_packets.jsonl
rb_servo_server.log
left_simulator.log
right_simulator.log
servo_log.csv
path_samples_left.csv
path_samples_right.csv
```

`summary.json` should contain:

- pass/fail per scenario
- max position error
- max orientation error
- max path orientation error
- max line deviation
- max twist orientation drift
- whether near-pi C++ math tests ran
- max path tracking error
- max IK / Cartesian duration
- faults, if any
- config path
- git commit, if available

## Common Failure Interpretation

`TcpPoseTarget` passes but `TcpLinearMove` fails line deviation:

- path tracker, velocity clamp, or safety filter is distorting the path

`TcpLinearMove` final pose passes but orientation drift fails:

- orientation mode, Jlog6 IK, or Cartesian velocity control requires review

`TcpTwistLocal` moves but orientation drifts:

- orientation-hold feedback or twist-to-Jacobian mapping requires review

`path_done` never observed:

- path telemetry may be cleared too quickly for the state publish rate

## Not Real Robot Evidence

Even if all scenarios pass, real Cartesian motion remains blocked until a separate real-hardware acceptance workflow exists.
