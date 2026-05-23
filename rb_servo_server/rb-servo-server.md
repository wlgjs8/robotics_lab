# Codex Implementation Spec: rb_servo_server

> Historical implementation spec. Some milestone text predates the current
> per-arm simulator, rbpodo backend, and optional Pinocchio FK/IK work. The
> current source of truth is the root `README.md`,
> `docs/architecture.md`, and `rb_servo_server/README.md`.

## Goal

Build a C++ dual-arm servo server for two Rainbow RB3-730 robots.

Primary near-term goal:

- mock mode runs without robots
- Python sends UDP JSON commands
- C++ server runs same-tick dual-arm `servo_j` style loop at 100–200 Hz
- logs period/jitter and joint targets

Later goals:

- Rainbow rbsim backend
- real rbpodo backend
- Cartesian TCP control
- optional force/admittance control
- Python VLA / imitation policy integration

## Non-goals for the first milestone

Do not implement these in Milestone 1:

- full Cartesian IK
- real robot rbpodo calls
- force control in the active servo path
- RealSense capture in this process
- ROS2 integration
- 500 Hz optimization

## Architecture

```text
Python policy / teleop
  UDP JSON, 10–30 Hz
        ↓
CommandServer
        ↓
CommandBuffer, lifecycle queue + latest-motion-wins
        ↓
DualArmServoLoop, 100–200 Hz
        ↓
TrajectoryFilter
        ↓
SafetyFilter
        ↓
IRobotBackend
  MockBackend / RbpodoBackend
```

Force-control future path:

```text
Tcp command + desired wrench
        ↓
CartesianController
        ↓
ForceController
        ↓
IK
        ↓
TrajectoryFilter / SafetyFilter
        ↓
servo_j
```

## Current rb_servo_server status

Already implemented:

- mock backend
- UDP command receiver
- minimal YAML config parser
- servo period/jitter logging
- Hold as previous sent target
- tracking-error safety guard with configurable snap/fault-latch policy
- latched EmergencyStop / fault state
- invalid command payload guard; malformed packets are dropped before the command buffer changes
- explicit `ArmMotion` gate before motion commands can run
- send failure policy that records only successfully sent targets
- real-mode startup guards for `RB_ALLOW_REAL_ROBOT`, realtime setup, local command bind, and conservative safety policy
- capped filter dt and acceleration-overshoot guard
- force-control types/config/interface scaffold
- thread-safe `ServoSnapshot` read surface for tests/debug/publisher integration
- send timestamp/skew/duration logging for left/right servo commands

Still pending:

- production `RbpodoBackend`
- production robot state/TCP pose publisher fields
- better command buffer for RT priority inversion
- action chunk interpolation
- Cartesian FK/IK
- force control integration into Cartesian layer


## v3 fail-safe requirements

The server must satisfy this invariant:

```text
No failure path may output [0, 0, 0, 0, 0, 0] unless that was a validated command from the user.
```

Required behavior:

- malformed JSON / unknown mode / invalid numeric payload → Drop packet.
- `JointTarget` without `q_target_deg` → Drop packet.
- `JointVelocity` without `dq_target_deg_s` → Drop packet.
- malformed 6D arrays → Drop packet.
- stale command → Hold.
- Cartesian command while IK is not implemented → Hold + `CartesianUnavailable`.
- future IK failure → explicit failure result, then Hold or fault latch.
- `EmergencyStop` → latch current actual pose if available, otherwise last safe target.
- `ResetFault` → clear fault only and return to `ConnectedHold`; it does not resume motion.
- motion commands are ignored until `ArmMotion` transitions to `ArmedHold`.
- latched fault ignores motion commands until `ResetFault`.
- failed `sendServoJ` targets are not recorded as previous sent targets.
- mock/rbsim default tracking error policy: `snap_to_actual`.
- real default tracking error policy: `fault_latch`.
- trajectory/safety math uses capped `filter_dt`, while logs keep actual `period_ms`.

Reset command:

```json
{"seq": 1, "mode": "ResetFault"}
```

## Milestone 1: mock command loop acceptance

### Required behavior

Commands:

```bash
cmake -S . -B build
cmake --build build -j
./build/rb_servo_server --config config/dual_mock.yaml
```

In another terminal:

```bash
python3 tools/send_dual_joint_sine.py --rate 20 --amp-deg 2 --freq 0.2
```

Expected:

- `logs/servo_log.csv` is created
- `command_seq` changes from incoming commands
- `left_mode/right_mode` show `JointTarget`
- `left_q_sent_0` and `right_q_sent_0` move in opposite directions
- `period_ms` is around 5 ms for 200 Hz
- `jitter_ms` is nonzero and meaningful
- send timing columns are present for later skew analysis
- after command timeout, mode falls back to `Hold`
- after `ResetFault`, motion requires a fresh `ArmMotion`

### Files involved

- `src/network/command_server.cpp`
- `src/config/config.cpp`
- `src/control/dual_arm_servo_loop.cpp`
- `src/control/trajectory_filter.cpp`
- `src/control/safety_filter.cpp`
- `src/logging/servo_logger.cpp`
- `tools/send_dual_joint_sine.py`
- `tools/plot_servo_log.py`

## Milestone 2: cleanup before rbsim

Implement or refine:

1. `StatePublisher`
   - publish latest dual robot state to Python
   - JSON is acceptable initially
   - 20–50 Hz is enough

2. `EmergencyStop`
   - make it a latched state
   - call backend `stop()`
   - ignore non-reset commands until reset is supported

3. `CommandBuffer`
   - current mutex version is acceptable for mock/100–200 Hz
   - replace with priority-inheritance mutex or seqlock before high-RT testing

4. `MockBackend`
   - move plant integration out of `readState()` if multiple readers are added
   - for now it is acceptable because only the servo loop reads it

## Milestone 3: RbpodoBackend

Implement `src/robot/rbpodo_backend.cpp` behind `RB_SERVO_ENABLE_RBPODO`.

Required behavior:

- connect by IP
- set operation mode: real vs simulation
- apply speed bar
- send `servo_j` target using config:
  - `servo_time_sec`
  - `servo_lookahead_sec`
  - `servo_gain`
  - `servo_acc`
  - `disable_waiting_ack`
- read robot state through rbpodo data channel
- populate:
  - `q_actual_deg`
  - `q_target_deg` if available
  - `dq_actual_deg_s` if available
  - error state
  - connection state

Safety:

- keep `RB_ALLOW_REAL_ROBOT=1` guard
- never default to real mode silently
- start real robot tests with `Hold` only

## Milestone 4: Cartesian controller

Implement `CartesianController` only after joint server stability is verified.

Required features:

- stand frame to left/right robot base transform
- local FK from q
- TCP target from:
  - `TcpPoseTarget`
  - `TcpDeltaStand`
  - `TcpDeltaLocal`
- damped least-squares IK
- joint velocity / acceleration safety compatibility

Do not call Rainbow FK/IK inside the high-rate loop except for debugging. Use local kinematics in the control loop.

## Milestone 5: force control integration

Force control is currently included as a design scaffold.

Use files:

- `include/rb_servo/control/force_controller.hpp`
- `src/control/force_controller.cpp`
- `include/rb_servo/sensor/i_force_torque_sensor.hpp`
- `include/rb_servo/sensor/mock_force_torque_sensor.hpp`
- `src/sensor/mock_force_torque_sensor.cpp`
- `docs/force_control.md`

Integration target:

```text
nominal TCP target
  + force/admittance TCP compensation
  → IK
  → q_target
  → servo_j
```

Do not apply force compensation directly to joint targets.

Before using real contact:

- verify force sensor sign convention
- implement tare/bias
- clamp position offset and per-tick step
- log measured wrench, target wrench, TCP compensation
- start with tiny gains

If integrating `mo_forcecontroller`:

- wrap it behind `ForceController`
- make sampling time configurable
- do not leave `Ts=0.002` hard-coded if the servo loop is 100–200 Hz

## Coding rules

- No blocking file I/O inside `DualArmServoLoop`
- No Python call inside `DualArmServoLoop`
- No camera capture inside this process
- Use C++ receive timestamp for command timeout
- Keep real/sim/mock behind `IRobotBackend`
- Keep policy process unaware of real vs sim vs mock
- Do not enable real mode without explicit user/environment guard

## Definition of done for the next Codex task

A good next Codex task is:

1. build succeeds in mock mode
2. `send_dual_joint_sine.py` changes command seq/mode in log
3. jitter columns exist and have meaningful values
4. YAML changes are reflected at runtime
5. no regression to real/rbsim placeholder build
6. force-control scaffold compiles but remains disabled by default
7. invalid `JointTarget` without `q_target_deg` holds previous target and never moves to zeros
8. `EmergencyStop` latches fault and ignores later motion commands until `ResetFault`
