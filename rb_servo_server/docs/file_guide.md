# File Guide

## Start here

- `README.md`  
  Build/run instructions and current project status.

- `rb-servo-server.md`  
  Codex implementation spec and milestone plan.

- `docs/design_review_fixes.md`  
  What was changed after the first design review.

## Design docs

- `docs/architecture.md`  
  Process boundaries and backend/control architecture.

- `docs/control_loop.md`  
  Servo loop flow, timing, hold behavior, and safety behavior.

- `docs/network_protocol.md`  
  UDP JSON command schema.

- `docs/camera_sync.md`  
  RealSense sync strategy; camera stays out of the servo process.

- `docs/config_examples.md`  
  Config notes.

- `docs/code_list.md`  
  Per-file summary.

## Implementation files

- `include/rb_servo/core/types.hpp`  
  Shared structs and enums.

- `include/rb_servo/config/config.hpp` / `src/config/config.cpp`  
  Config structs and simple YAML loader.

- `include/rb_servo/robot/*` / `src/robot/*`  
  Robot backend abstraction.

- `include/rb_servo/control/*` / `src/control/*`  
  Control loop, filters, future Cartesian/force layers.

- `include/rb_servo/network/*` / `src/network/*`  
  Command receive and future state publishing.

- `include/rb_servo/sensor/*` / `src/sensor/*`  
  F/T sensor interfaces and mock sensor.

- `include/rb_servo/logging/*` / `src/logging/*`  
  Async servo logger.

## Tools

- `tools/send_dual_joint_sine.py`
- `tools/send_dual_hold.py`
- `tools/plot_servo_log.py`
