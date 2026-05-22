# Architecture

`rb_servo_server` is a low-level dual-arm command server.

It is not the VLA model, not the RealSense recorder, and not the high-level bimanual planner. Its job is to receive the latest command and push safe joint targets to two Rainbow arms at a stable servo rate.

## Main process

```text
Python policy / teleop
  10–30 Hz UDP JSON command
        ↓
CommandServer
        ↓
CommandBuffer
        ↓
DualArmServoLoop
  100–200 Hz initially
        ↓
IRobotBackend
  MockBackend / RbpodoBackend
```

## Backend abstraction

`IRobotBackend` keeps real/simulation/mock separated from the servo loop.

```text
IRobotBackend
  ├── MockBackend
  └── RbpodoBackend
       ├── Rainbow simulator
       └── real RB3-730 control box
```

The policy process should not know whether the backend is mock, rbsim, or real.

## Current active command path

```text
JointTarget / JointVelocity / Hold
        ↓
TrajectoryFilter
        ↓
SafetyFilter
        ↓
sendServoJ
```

## Future Cartesian path

```text
TcpDeltaStand / TcpDeltaLocal / TcpPoseTarget
        ↓
CartesianController
        ↓
IK
        ↓
TrajectoryFilter
        ↓
SafetyFilter
        ↓
sendServoJ
```

## Future force-control path

```text
nominal TCP target
        ↓
ForceController + F/T sensor
        ↓
TCP compensation
        ↓
IK
        ↓
sendServoJ
```

Force control is disabled by default and should only be activated after Cartesian FK/IK is stable.

## Camera architecture

RealSense capture remains outside this process.

```text
camera_recorder process
  head RealSense
  left wrist RealSense
  right wrist RealSense
        ↓
image timestamps + metadata
        ↓
dataset_builder merges with servo_log.csv
```

This prevents USB/camera blocking from affecting servo-loop jitter.
