# Code List

## Application

- `src/main.cpp`  
  Loads config, creates robot backends, starts the logger and servo loop, then opens the command server only after the loop reaches its initial safe hold.

## Core

- `include/rb_servo/core/types.hpp` / `src/core/types.cpp`  
  Shared enums and data types: joint arrays, poses, robot state, commands, servo samples. `RobotState::has_valid_joint_state` gates startup and safe hold target selection.

- `include/rb_servo/core/clock.hpp` / `src/core/clock.cpp`  
  Steady-clock timestamp helpers.

- `include/rb_servo/core/realtime.hpp` / `src/core/realtime.cpp`  
  RT helpers: memory lock, realtime priority, CPU pinning.

- `include/rb_servo/core/thread_safe_buffer.hpp`  
  Initial mutex-based latest-value buffer. Replace with seqlock or priority-inheritance mutex for 500 Hz work.

## Config

- `include/rb_servo/config/config.hpp` / `src/config/config.cpp`  
  Config structs and minimal YAML parser. Current parser supports the scaffold's simple two-level YAML only.

## Robot Backends

- `include/rb_servo/robot/i_robot_backend.hpp`  
  rbpodo/mock backend abstraction.

- `include/rb_servo/robot/mock_backend.hpp` / `src/robot/mock_backend.cpp`  
  First-order mock plant for no-robot development.

- `include/rb_servo/robot/rbpodo_backend.hpp` / `src/robot/rbpodo_backend.cpp`  
  Guarded rbpodo backend for real RB3-730 controllers. It supports gated connect/readState and gated joint `servo_j`, keeps read-only state acquisition separate from motion readiness, and leaves stop/reset fault recovery unimplemented until verified controller APIs are accepted.

- `../../docs/archive/planning/rb_servo_server_rbpodo_backend_plan.md`
  Archived implementation plan and hardware acceptance runbook for the real RB3-730 backend. It is historical planning context, not runnable operator guidance.

- `include/rb_servo/robot/backend_factory.hpp` / `src/robot/backend_factory.cpp`  
  Creates backend from config.

## Control

- `include/rb_servo/control/command_buffer.hpp` / `src/control/command_buffer.cpp`  
  Latest-motion-wins buffer with a small lifecycle-command queue. Stale commands fall back to Hold. Invalid stored timeout values also resolve to Hold instead of a hard-coded recovery timeout.

- `include/rb_servo/control/trajectory_filter.hpp` / `src/control/trajectory_filter.cpp`  
  Joint target/velocity handling and velocity clamp. Hold returns previous sent target.

- `include/rb_servo/control/safety_filter.hpp` / `src/control/safety_filter.cpp`  
  Joint position/velocity/acceleration limits, state error checks, tracking-error checks.

- `include/rb_servo/control/cartesian_controller.hpp` / `src/control/cartesian_controller.cpp`  
  Future TCP/IK layer. Currently intentionally deferred.


- `include/rb_servo/control/dual_arm_servo_loop.hpp` / `src/control/dual_arm_servo_loop.cpp`  
  Main same-tick dual-arm servo loop.

## Network

- `include/rb_servo/network/command_server.hpp` / `src/network/command_server.cpp`  
  UDP JSON command receiver. `start()` waits for bind readiness and fails startup if command ingress cannot bind.

- `include/rb_servo/network/state_publisher.hpp` / `src/network/state_publisher.cpp`  
  Placeholder for state publishing to Python.

## Sensors



## Logging

- `include/rb_servo/logging/servo_logger.hpp` / `src/logging/servo_logger.cpp`  
  Bounded async CSV logger with period/jitter columns, queue drop count, and CSV escaping for string fields.

## Tools

- `tools/send_dual_joint_sine.py`  
  UDP JSON sine command sender.

- `tools/send_dual_hold.py`  
  UDP JSON Hold command sender.

- `tools/plot_servo_log.py`  
  Plots period, jitter, and basic joint traces.

## v3 additions

- `docs/fail_safe_policy.md`: fail-safe invariant and failure handling table.
- `tools/send_reset_fault.py`: sends a ResetFault command to clear a latched fault.
- `SafetyVerdict` and `TrackingErrorPolicy` in `include/rb_servo/core/types.hpp`.
- `SafetyCheckResult` in `include/rb_servo/control/safety_filter.hpp`.
