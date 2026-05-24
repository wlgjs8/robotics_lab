# TCP Pose Simulator Acceptance Runbook

This runbook validates simulator-only TCP Pose/Delta behavior for the per-arm
RB3-730 simulator stack. It is not real robot evidence and it must not be used
to enable real Cartesian motion.

Do not set `RB_ALLOW_REAL_ROBOT`, `RB_ALLOW_REAL_MOTION`, or
`RB_ALLOW_REAL_CARTESIAN` for this runbook.

## Dependency Checklist

P3-F depends on these earlier work packages:

- P1-C: per-arm simulator smoke exists and can run.
- P2-C: state stream publishes FK TCP state with `tcp_stand` and
  `has_valid_tcp_pose`.
- P3-B: `CartesianController` can route simulator TCP commands through IK.
- P3-C: TCP command protocol docs/tools exist, including
  `rb_servo_server/tools/send_tcp_delta.py` and
  `rb_servo_server/tools/send_tcp_pose_target.py`.

The canonical simulator-only server config for this runbook is
`rb_servo_server/config/dual_simulator_tcp_acceptance.yaml`. It is the default
used by `scripts/tcp_pose_simulator_acceptance.sh` and contains:

```yaml
kinematics:
  enable: true
  provider: pinocchio
  urdf: path/to/rb3_730e.urdf
  publish_tcp: true
  ik:
    enable: true

cartesian_control:
  enable: true
  allow_in_simulation: true
  allow_in_real: false
  warn_ik_duration_us: 3000
  fail_ik_duration_us: 5000

servo:
  send_servo_commands: true
```

If those dependencies or config gates are missing, the scripted runner exits
nonzero before motion commands and prints the missing item.

The server binary must be built with `RB_SERVO_ENABLE_PINOCCHIO=ON`; the
default hardware-free gate intentionally builds with Pinocchio off. The
scripted runner checks `scripts/check_deps.sh --profile kinematics` and, for
local starts, rejects a CMake build cache that shows Pinocchio was off. Its
default server path is
`rb_servo_server/build/pinocchio_gate/rb_servo_server`.

For syntax-only environments where Pinocchio is not installed, pass
`--allow-missing-pinocchio`. That mode still validates the script, command-tool
dry runs, and selected config contract, then prints that runtime FK/IK
acceptance was skipped because Pinocchio is unavailable before launching any
simulator/server processes.

## Start Per-Arm Simulator Stack

For the compose operator stack:

```bash
make sim-up
```

The default compose stack does not publish the servo UDP command/state endpoints
as a host-facing acceptance interface. Use it for operator-level simulator
startup checks, or run the host-loopback sequence below when command/state
artifacts are required.

For artifact-producing host-loopback acceptance, use the script below. It starts
one left simulator process, one right simulator process, and one
`rb_servo_server` process unless `--assume-running` is passed:

```bash
bash scripts/codex_gate.sh HARDEN-06

cmake -S rb_servo_server -B rb_servo_server/build/pinocchio_gate \
  -DCMAKE_BUILD_TYPE=Debug \
  -DRB_SERVO_ENABLE_RBPODO=OFF \
  -DRB_SERVO_ENABLE_PINOCCHIO=ON \
  -DRB_SERVO_ALLOW_FETCHCONTENT=OFF \
  -DBUILD_TESTING=ON
cmake --build rb_servo_server/build/pinocchio_gate -j

bash scripts/tcp_pose_simulator_acceptance.sh
```

Use `--assume-running` only when a simulator stack is already reachable through
the host-loopback command and state endpoints configured by the script.

## Manual Acceptance Sequence

The command examples in this section assume host-loopback command/state
endpoints. For a manual host-loopback run, start the same three processes the
script starts: left simulator, right simulator, and `rb_servo_server` with an
FK/IK-enabled simulator config. Do not use a real-mode or `rbpodo` config.

1. Start the per-arm host-loopback simulator stack with an FK/IK-enabled
   simulator config, or let the scripted runner start it:

   ```bash
   bash scripts/tcp_pose_simulator_acceptance.sh
   ```

2. Verify the state stream reports FK TCP pose for both arms:

   ```text
   left.tcp_stand != null
   right.tcp_stand != null
   left.has_valid_tcp_pose == true
   right.has_valid_tcp_pose == true
   ```

3. Send `ArmMotion` if required by the protocol:

   ```bash
   python3 rb_servo_server/tools/send_arm_motion.py --host 127.0.0.1 --port 50010
   ```

4. Send a small stand-frame TCP delta:

   ```bash
   python3 rb_servo_server/tools/send_tcp_delta.py \
     --endpoint udp://127.0.0.1:50010 \
     --frame stand \
     --left 0.005 0 0 0 0 0 \
     --right 0 0 0 0 0 0
   ```

5. Verify the normal-motion result:

   - command verdict is `Ok`
   - no fault latch is set
   - `q_sent_deg` / target q values are finite
   - q remains within configured joint limits
   - `left.cartesian_solve.ik_duration_us`,
     `left.cartesian_solve.ik_iterations`, and `left.fk_duration_us` are
     finite in the state stream
   - IK latency is below the acceptance threshold for the 200 Hz loop:
     p95 `ik_duration_us <= 3000` and max `ik_duration_us <= 5000`
   - final left FK TCP `x` moves in the positive direction within tolerance
   - right FK TCP pose remains within the no-motion tolerance

6. Run the IK failure test by sending an unreachable stand-frame target:

   ```bash
   python3 rb_servo_server/tools/send_tcp_pose_target.py \
     --endpoint udp://127.0.0.1:50010 \
     --left 10 0 10 0 0 0
   ```

   Verify:

   - verdict is `IkFailed` or an explicitly documented equivalent failure
     verdict for a Cartesian dependency that is unavailable
   - state reports `cartesian_solve.status`, `cartesian_solve.reason`,
     `ik_duration_us`, `ik_iterations`, and `ik_timed_out`
   - previous safe target is retained; no zero/default q target is sent
   - no crash occurs and the state stream continues

7. If supported safely in the simulator, validate `EmergencyStop` /
   `ResetFault`:

   - send `EmergencyStop`
   - verify the emergency/fault latch is visible in state
   - send `ResetFault`
   - verify the stack returns to a safe hold state
   - send a fresh `ArmMotion` before any later motion command

## Scripted Runner

```bash
bash scripts/tcp_pose_simulator_acceptance.sh \
  --artifact-dir artifacts/tcp_pose_acceptance/manual
```

Default behavior:

- runs `scripts/check_deps.sh --profile hardware-free`
- runs `scripts/check_deps.sh --profile kinematics` so missing Pinocchio/Eigen
  fails before processes start unless `--allow-missing-pinocchio` is provided
- verifies required TCP command tools exist and parse dry-run packets
- verifies the server binary and simulator configs exist
- verifies the selected server config declares simulator-only endpoints,
  Pinocchio FK/TCP, IK, Cartesian simulation gates, and
  `servo.send_servo_commands: true`
- verifies published TCP poses retain finite RPY fields and publish normalized
  `quaternion_xyzw: [qx, qy, qz, qw]` with scalar aliases that match the array
- starts host-loopback left/right simulator processes and `rb_servo_server`
- captures the state stream
- sends `ArmMotion`
- sends left `TcpDeltaStand` `+0.005 m x` and right zero delta
- verifies `Ok`, no fault latch, finite q, joint limits, and TCP x direction
- records FK/IK latency summary from `last_cartesian_solve` /
  per-arm `cartesian_solve` state fields, including count, min, p50, p95, max,
  timeout count, and iteration min/max
- fails simulator acceptance if p95 IK latency is above `3000 us` or max IK
  latency is above one 200 Hz loop period (`5000 us`)
- sends an unreachable `TcpPoseTarget`
- verifies `IkFailed`, previous safe q retention, and continued state stream
- attempts simulator `EmergencyStop` / `ResetFault` unless skipped

If P3-B/P3-C tools are missing, the script exits nonzero before starting the
stack.

## Artifacts

The script writes artifacts under
`artifacts/tcp_pose_acceptance/<timestamp>/` by default:

```text
tcp_pose_acceptance_summary.json
state_stream.jsonl
servo_log.csv
rb_servo_server.log
left_simulator.log
right_simulator.log
```

`tcp_pose_acceptance_summary.json` should include:

```json
{
  "ik_latency_us": {
    "count": 2,
    "min": 120,
    "p50": 130,
    "p95": 140,
    "max": 140,
    "threshold_p95": 3000,
    "threshold_max": 5000,
    "timed_out_count": 0,
    "iterations_min": 1,
    "iterations_max": 4
  }
}
```

These artifacts are for debugging only and are ignored by git.
