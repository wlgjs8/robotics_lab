# Servo Backend Contract

This document defines the current backend, fault, and servo I/O contract.

## Goals

The control layer must preserve truth from each backend operation. It must not collapse all backend failures into a single `false` result or string log.

The contract should support:

- mock backend
- per-arm simulator backend
- rbpodo backend
- direct servo I/O
- worker servo I/O
- structured state publication
- future real read-only acceptance

## Backend Operations

Backends expose structured operations:

```cpp
BackendResult<RobotState> connect();
BackendResult<RobotState> initialize();
BackendResult<RobotState> readState();
SendServoJResult sendServoJ(const SendServoJRequest& request);
BackendResult<RobotState> stop();
BackendResult<RobotState> resetFault();
```

Do not reintroduce bool-only backend operations.

## Error Taxonomy

Backend failures are classified with `BackendErrorKind`, for example:

- transport/protocol: `TransportConnectFailed`, `TransportWriteFailed`, `TransportReadFailed`, `TransportTimeout`, `ProtocolError`
- topology: `WrongArm`, `WrongEndpoint`, `UnknownArm`
- robot lifecycle: `RobotDisconnected`, `RobotNotInitialized`, `ServoDisabled`, `WrongMode`, `RobotFault`, `InvalidJointState`
- command rejection: `InvalidTarget`, `ControllerRejected`, `CommandTimeout`
- software/policy: `DependencyUnavailable`, `SuppressedByPolicy`, `Unknown`

Each backend error should preserve:

- kind
- code/name/message
- retryable/recoverable flags
- robot_fault/transport_fault flags
- operation timing
- optional `state_after`

## Fault Classification

`FaultClassifier` maps structured backend and command results into the safety domain:

- robot/controller faults become robot-state safety failures
- transport failures stay transport/backend failures
- policy suppression is not a transport failure
- IK and Cartesian failures stay kinematics/control failures
- emergency and fault latch states suppress normal servo commands

The state stream should preserve both the current send policy and the original latched fault context. `last_send=SuppressedByPolicy` must not erase the first fault cause.

## Send Suppression

The servo loop must not keep sending regular `servo_j` while fault-latched, emergency-latched, or read-only. Suppression is explicit state, not a send failure.

Expected fields include:

```json
{
  "send_suppressed": true,
  "send_policy": "fault_latched",
  "fault_context": {
    "backend_error_kind": "RobotFault",
    "backend_error_name": "fault_latched"
  }
}
```

## Direct I/O And Worker I/O

### Direct I/O

The servo loop calls per-arm backend operations directly. This is simple and remains useful for baseline hardware-free validation.

### Worker I/O

Each `ArmWorker` owns one backend and one thread. The servo loop reads cached state and dispatches bounded send requests through worker interfaces. Worker mode allows left/right endpoint I/O to proceed independently.

Worker mode is simulator-only until separately accepted on hardware.

## ArmWorker Command Policy

Streaming servo requests are latest-wins. If a pending command is overwritten before dispatch, the worker must expose drop/overwrite telemetry. This is intentional for servo targets but must not be silent.

Lifecycle commands such as reset/stop should not be silently overwritten by streaming servo targets. They should use a separate lane or explicit policy.

## RbsimBackend Transport

`RbsimBackend` keeps a persistent JSON-lines TCP socket per simulator backend during healthy operation.

- Transport/protocol corruption closes the socket.
- A later request may reconnect.
- Robot/controller-level simulator errors such as `RobotFault` or `ServoDisabled` are structured backend results and do not imply TCP transport corruption.
- Transport counters should be available for diagnostics.

## RbpodoBackend Semantics

State acquisition and motion readiness are separate.

A controller can return valid joint feedback while `servo_enabled=false`. That is a valid state read, not permission to send motion.

Real `sendServoJ()` requires:

- valid state acquisition
- controller motion readiness
- config `send_servo_commands: true`
- `RB_ALLOW_REAL_ROBOT=1`
- `RB_ALLOW_REAL_MOTION=1`

Real stop/reset APIs remain conservative until verified. If no verified API is wired, return `DependencyUnavailable` and require operator intervention.

## State Publication

State JSON should expose:

- top-level observed mode/backend
- command source state
- fault context
- send policy
- per-arm read result
- per-arm send result
- worker telemetry
- TCP pose and quaternion when FK is available
- Cartesian solve/path telemetry when Cartesian modes are active

State publication is an operational truth surface. Do not hide backend or safety details behind a single `ok` field.
