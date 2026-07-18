# Codex Implementation Spec: rb_servo_server

> Historical implementation spec. Some milestone text predates the current
> per-arm simulator, rbpodo backend, and mandatory Pinocchio/Eigen FK/IK work. The
> current source of truth is the root `README.md`,
> `docs/architecture.md`, and `rb_servo_server/README.md`.

## Goal

Build a C++ dual-arm servo server for two Rainbow RB3-730 robots.

Primary near-term goal:

- mock mode runs without robots
- Python sends UDP JSON commands
- C++ server runs same-tick dual-arm `servo_j` style loop at 500 Hz
- logs period/jitter and joint targets

Later goals:

- rbpodo controller-simulation support
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
- unsupported non-500 Hz robot-control profiles

## Architecture

```text
Python policy / teleop
  UDP JSON, 10–30 Hz
        ↓
CommandServer
        ↓
CommandBuffer, lifecycle queue + latest-motion-wins
        ↓
DualArmServoLoop, 500 Hz
        ↓
TrajectoryFilter
        ↓
SafetyFilter
        ↓
IRobotBackend
  MockBackend / RbpodoBackend
```

Force-control path:

```text
TcpPoseTarget + rbpodo EFT wrench
        ↓
DeltaTwist/chunk follower
        ↓
F/T pipeline + contact supervisor + NormalForceController
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
- real-mode startup/config checks for realtime setup, local command bind, and conservative safety policy
- capped filter dt and acceleration-overshoot guard
- default-off F/T monitor, guard, and normal-admittance runtime
- thread-safe `ServoSnapshot` read surface for tests/debug/publisher integration
- send timestamp/skew/duration logging for left/right servo commands

Still pending:

- production `RbpodoBackend`
- production robot state/TCP pose publisher fields
- better command buffer for RT priority inversion
- action chunk interpolation
- Cartesian FK/IK
- physical F/T characterization and staged force acceptance


## v3 fail-safe requirements

The server must satisfy this invariant:

```text
No failure path may output [0, 0, 0, 0, 0, 0] unless that was a validated command from the user.
```

Required behavior:

- malformed JSON / unknown mode / invalid numeric payload → Drop packet.
- `JointTarget` without `q_target_deg` → Drop packet.
- malformed 6D arrays → Drop packet.
- stale command → Hold.
- Cartesian command while IK is not implemented → Hold + `CartesianUnavailable`.
- future IK failure → explicit failure result, then Hold or fault latch.
- `EmergencyStop` → latch current actual pose if available, otherwise last safe target.
- `ResetFault` → clear fault only and return to `ConnectedHold`; it does not resume motion.
- motion commands are ignored until `ArmMotion` transitions to `ArmedHold`.
- latched fault ignores motion commands until `ResetFault`.
- failed `sendServoJ` targets are not recorded as previous sent targets.
- mock/controller-simulation default tracking error policy: `snap_to_actual`.
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
./build/rb_servo_server --config config/local/<mock-config>.yaml
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
- `period_ms` is around 2 ms for 500 Hz
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

## Milestone 2: cleanup before controller simulation

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
   - current mutex version is acceptable for hardware-free 500 Hz smoke tests
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

- keep real robot motion explicit through site-local config
- never default to real mode silently
- start real robot tests with `Hold` only

## Milestone 4: Cartesian controller

Implement `CartesianController` only after joint server stability is verified.

Required features:

- stand frame to left/right robot base transform
- local FK from q
- TCP target from:
  - `TcpPoseTarget`
  - `TcpLinearMove`
- damped least-squares IK
- joint velocity / acceleration safety compatibility

Do not call Rainbow FK/IK inside the high-rate loop except for debugging. Use local kinematics in the control loop.

## Milestone 5: force control integration

Force control is integrated into `DualArmServoLoop` and disabled in tracked
configs. Site-local profiles can select monitor, guard, or guarded normal
admittance. Physical enforcement remains unaccepted until the F/T runbook
evidence is complete.

Use files:

- `include/rb_servo/control/force_controller.hpp`
- `src/control/force_controller.cpp`
- `include/rb_servo/control/normal_force_controller.hpp`
- `src/control/normal_force_controller.cpp`
- `include/rb_servo/sensor/ft_wrench_pipeline.hpp`
- `src/sensor/ft_wrench_pipeline.cpp`
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

Before physical promotion:

- supply independent sensor presence/fault/overrange/freshness signals
- verify `T_tcp_sensor`, sign, bias, tare, payload, and gravity compensation
- validate monitor-only contact supervision and telemetry on the installed sensor
- measure same-tick send suppression and flow-chunk epoch invalidation
- validate DeltaTwist tangential ownership while projecting the contact normal
- pass deterministic loop replay before supervised hardware acceptance

The project-native `NormalForceController` uses the actual loop `dt_sec`,
unilateral server hard caps, a passivity observer, and propose/commit state
updates. See `docs/force_control.md` for the active schema and promotion gates.

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
5. no regression to real/controller-simulation placeholder build
6. tracked force config stays disabled and force-control tests pass
7. invalid `JointTarget` without `q_target_deg` holds previous target and never moves to zeros
8. `EmergencyStop` latches fault and ignores later motion commands until `ResetFault`
