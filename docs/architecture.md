# robotics_lab Architecture

This document is the current source of truth for the system architecture. Component READMEs may contain local details, but public terminology, safety boundaries, and topology must match this document.

## Current Phase

The repository is currently in **simulator-first Cartesian acceptance hardening**.

The simulator stack should repeatedly validate:

- per-arm simulator topology
- structured backend result and fault telemetry
- `JointTarget` and `JointVelocity`
- `TcpPoseTarget`
- `TcpLinearMove`
- `TcpTwistLocal` and `TcpTwistStand`
- GUI operator controls
- policy_runner SpaceMouse command paths
- command-source lease/arbitration
- camera readiness contracts for future policy work

This is not a real robot milestone.

## Maturity Boundary

Supported for mock/simulation work:

- mock dual-arm servo control
- one local simulator endpoint per arm
- persistent simulator JSON-line transport
- simulator direct and worker I/O modes
- FK/TCP state publication with quaternion fields
- simulator-only Cartesian PTP, Linear, and Twist commands
- mock camera server
- GUI viewer/operator console for mock/simulation
- Python policy_runner with joint and Cartesian simulator action sources
- simulator-only Cartesian acceptance scripts

Not production-ready:

- real RB3-730 motion
- real Cartesian/TCP motion
- force control
- gripper control
- measured camera/robot calibration
- real camera + policy + robot closed-loop behavior

## Canonical Terms

Use only these values in public config, docs, GUI labels, and operator-facing logs:

```yaml
run_mode: mock | simulation | real
backend_type: mock | simulator | rbpodo
```

`run_mode` describes the environment. `backend_type` describes the backend implementation. Deprecated terms such as `rbsim_local`, public `rbsim`, or mixed simulator aliases must not be introduced in new public docs/configs.

## Controller Topology

The physical system has one controller endpoint per arm:

```text
rb_servo_server
  left_robot  backend_type=rbpodo -> 172.28.60.200
  right_robot backend_type=rbpodo -> 172.28.60.201
```

The simulator mirrors that controller shape:

```text
rb_servo_server
  left_robot  backend_type=simulator -> rb_simulator_left
  right_robot backend_type=simulator -> rb_simulator_right
```

The simulator topology is isomorphic to the physical topology by endpoint count and ownership, not by IP address. Simulator configs must not default to the real controller IPs.

### Docker Compose Simulator Topology

```text
rb_simulator_left container
  arm: left
  control: tcp://0.0.0.0:50200
  admin:   tcp://0.0.0.0:50201

rb_simulator_right container
  arm: right
  control: tcp://0.0.0.0:50200
  admin:   tcp://0.0.0.0:50201
```

Separate containers can reuse internal ports. Compose uses compose-specific configs and sets:

```bash
RB_SIMULATOR_ALLOW_NON_LOOPBACK=1
```

### Host-Local Simulator Topology

```text
left simulator
  control: tcp://127.0.0.1:50200
  admin:   tcp://127.0.0.1:50201

right simulator
  control: tcp://127.0.0.1:50210
  admin:   tcp://127.0.0.1:50211
```

## Safety Gates

Real robot connection is closed unless:

```bash
RB_ALLOW_REAL_ROBOT=1
```

Real joint servo motion is closed unless:

```bash
RB_ALLOW_REAL_MOTION=1
```

Real Cartesian/TCP motion is closed unless:

```bash
RB_ALLOW_REAL_CARTESIAN=1
```

These environment variables are necessary but not sufficient. Config and acceptance must also explicitly allow the operation.

Tracked real config is a template only:

```text
rb_servo_server/config/dual_real.example.yaml
```

Site-owned real configs belong under:

```text
rb_servo_server/config/local/
```

No tracked runnable real robot config should exist.

Force control is intentionally unavailable:

```yaml
force_control:
  provider: null
  enable: false
```

## Motion Primitive Contract

### `JointTarget`

Absolute joint-space target. This is a joint-space point-to-point command.

### `JointVelocity`

Streaming joint velocity command. Suitable for joint teleop/debug when safety gates allow it.

### `TcpPoseTarget`

Cartesian point-to-point final-pose target. It is MoveJ-like at the TCP level. Final TCP pose is targeted, but the intermediate TCP path is not guaranteed to be linear.

### `TcpLinearMove`

Simulator-only MoveL-like Cartesian path primitive. It plans a Cartesian path with explicit timing/speed semantics and orientation interpolation semantics. Current modes are:

- `constant`: keep start orientation along the path
- `slerp`: interpolate start orientation to target orientation

Real mode remains blocked.

### `TcpTwistLocal` / `TcpTwistStand`

Streaming Cartesian velocity primitives. `TcpTwistLocal` is intended for SpaceMouse/local-frame teleop. `TcpTwistStand` is the stand-frame low-level API. Server-side Cartesian velocity limits, stale-state checks, deadman behavior, and command-source arbitration are required.

### `TcpDeltaLocal` / `TcpDeltaStand`

Low-level one-shot/debug jog commands. They are not the default GUI target-move primitive.

## Servo Control Architecture

The control path is moving toward this structure:

```text
CommandBuffer
  -> ServoCoordinator / DualArmServoLoop
       -> Left ArmWorker  -> left IRobotBackend
       -> Right ArmWorker -> right IRobotBackend
```

`DualArmServoLoop` owns:

- command freshness
- command-source lease interpretation
- lifecycle state
- FK/IK and Cartesian target generation
- safety filtering
- fault latching
- dual-arm result aggregation
- state publication

`ArmWorker` owns blocking per-arm backend I/O in worker mode. Worker mode is simulator-only until separate real-hardware acceptance exists.

## Backend Architecture

Backends must return structured operation results:

- `BackendResult<RobotState>`
- `SendServoJResult`
- `BackendErrorKind`
- `BackendTiming`
- `FaultContext`

Bool-only backend results must not be reintroduced.

`RbsimBackend` keeps one persistent JSON-lines TCP connection per simulator backend instance during healthy operation. Transport/protocol corruption closes the socket; robot/controller-level errors such as `RobotFault` remain structured backend results.

`RbpodoBackend` separates state acquisition from motion readiness. Valid joint feedback with `servo_enabled=false` is a valid read state, not motion readiness. Real `servo_j` sends remain blocked unless real gates and controller readiness are satisfied. Real stop/reset API wiring remains conservative until verified.

## GUI And Policy Roles

`rb_gui` is a viewer/operator console for mock/simulation. It may send simulator-only TCP PTP and Linear commands when the server state, mode, backend, lease, and feature flags allow it. It must keep real motion disabled.

`policy_runner` owns Python action sources, including SpaceMouse. SpaceMouse Cartesian uses `TcpTwistLocal`, not repeated TCP deltas. Joint-only action sources do not require camera observations. Camera-dependent sources must declare camera readiness and fail closed when camera state is stale.

## Camera Role

`camera_server` owns RealSense/mock capture, shared-memory image transport, metadata, and health. Real camera acceptance is separate from robot motion acceptance.

## Frame And Calibration Contract

Shared frame names and transform directions are defined in `docs/frame_contract.md`. The active setup registry is:

```text
calibration/active_calibration.yaml
```

Current geometry is `configured_estimate`, not measured calibration. It may be used for visualization and simulation, but not for real geometry-dependent policy.

## Validation Contract

Hardware-free validation is described in `docs/hardware_free_validation.md`.

Cartesian simulator acceptance is described in `docs/runbooks/tcp_pose_simulator_acceptance.md`.

Real three-camera acceptance is described in `docs/runbooks/camera_acceptance.md`.

Passing simulator acceptance is not permission to move hardware.
