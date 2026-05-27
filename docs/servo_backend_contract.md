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

Dual-arm latches preserve per-arm context as well as the top-level summary. If both
arms report different failures in the same tick, `fault_context.top_level` remains
the deterministic summary used for the existing `latched_fault_reason`, while
`fault_context.left` and `fault_context.right` preserve each arm's classified
cause. Policy suppression is still not a fault by itself and must not override a
real backend, robot-state, emergency, command, or kinematics failure as the
top-level cause.

## Send Suppression

The servo loop must not keep sending regular `servo_j` while fault-latched, emergency-latched, or read-only. Suppression is explicit state, not a send failure.

Expected fields include:

```json
{
  "send_suppressed": true,
  "send_policy": "fault_latched",
  "fault_context": {
    "backend_error_kind": "RobotFault",
    "backend_error_name": "fault_latched",
    "top_level": {
      "backend_error_kind": "RobotFault",
      "backend_error_name": "fault_latched"
    },
    "left": {
      "backend_error_kind": "RobotFault",
      "backend_error_name": "fault_latched"
    },
    "right": null
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
The internal telemetry field for overwritten commands is
`worker_command_drops_total`, mirrored in state JSON as
`command_drops_total`. Per-arm state JSON also exposes
`pending_overwrites_total`, `last_dropped_seq`, `last_enqueued_seq`,
`last_dispatched_seq`, `last_completed_seq`, `queue_policy`,
`read_period_sec`, `read_rate_hz`, and `state_age_us` under the `worker`
object.

Lifecycle commands such as reset/stop should not be silently overwritten by streaming servo targets. They should use a separate lane or explicit policy.

## RbsimBackend Transport

`RbsimBackend` keeps a persistent JSON-lines TCP socket per simulator backend during healthy operation.

- Transport/protocol corruption closes the socket.
- A later request may reconnect, but reconnect attempts are rate-limited with exponential backoff to avoid a retry storm while the simulator is restarting or down.
- Simulator reconnect backoff starts at 50 ms and caps at 1000 ms. Calls made before the next retry window do not call `getaddrinfo()`, `socket()`, or `connect()`.
- Suppressed reconnects are reported as retryable transport failures with error name `rbsim_connect_backoff`; they are not robot/controller faults and must not be treated as successful reads.
- Robot/controller-level simulator errors such as `RobotFault` or `ServoDisabled` are structured backend results and do not imply TCP transport corruption.
- Transport counters should be available for diagnostics, including connect attempts, connect failures, suppressed connect attempts, last connect error name/message, and next retry timing.
- Simulator sockets use `TCP_NODELAY` and `SO_KEEPALIVE`. Linux builds also request conservative TCP keepalive probe timing for development use; keepalive option failures are warnings and do not fail an otherwise healthy simulator connection.

This hardening applies to the simulator backend only. It does not change rbpodo transport behavior or authorize real robot motion.

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

## RbscriptTcpBackend Experimental Semantics

`backend_type: rbscript_tcp` is an experimental comparison backend for raw
Rainbow UI Script over TCP. It does not replace `RbpodoBackend`, which remains
the primary real backend until a separate rbscript acceptance plan passes.
The validation order is simulator/read-only first; stress or motion probing is
not a real-ready signal.

The command transport is a persistent TCP socket to the Rainbow script command
port. The command port 5000 carries UI Script text. It is not UDP and must not
send commands directly to the controller over UDP; no UDP direct-to-controller
path is allowed.

The data transport uses a separate bounded TCP connection to the Rainbow data
port 5001 and sends `reqdata`. `RBSCRIPT-TCP-02` wires this into
`readState()`, but only publishes valid joint state for a recognized
`rbscript_tcp_state_v1` JSON fixture carrying six finite `q_actual_deg` values
in degrees. This conservative fixture parser exists to exercise the transport
and structured backend result path without guessing undocumented Rainbow binary
offsets. Unknown, short, malformed, binary, or otherwise unsupported responses
fail closed with `ProtocolError`, `UnsupportedSchema`, or
`InvalidJointState`; `has_valid_joint_state` remains false on failure.

Real controller binary data-port parsing is not production-ready until the
payload layout is documented and validated against a controller fixture. Until
then, `RbpodoBackend` remains the reference state backend.

Real rbscript TCP connection requires all normal real gates plus an additional
backend-specific gate:

```bash
RB_ALLOW_REAL_ROBOT=1
RB_ALLOW_RBSCRIPT_TCP=1
```

`sendServoJ()` is stricter still:

```bash
RB_ALLOW_REAL_MOTION=1
RB_ALLOW_RBSCRIPT_TCP_MOTION=1
```

Real Cartesian motion remains separately closed unless:

```bash
RB_ALLOW_REAL_CARTESIAN=1
```

No current rbscript task accepts real Cartesian motion.

The initial command formatter emits Rainbow script text:

```text
move_servo_j(jnt[j0,j1,j2,j3,j4,j5],t1,t2,gain,alpha)
```

Joint units are degrees. By default the backend waits for an ACK and accepts a
command only when the controller response contains a documented success phrase
such as `The command was executed`. Rejections such as `The command is not
allowed` map to `ControllerRejected`; timeouts and socket failures map to the
structured transport error taxonomy. `disable_waiting_ack` defaults to false and
must remain an explicit, separately gated choice. Send result telemetry exposes
`ack_policy` and `ack_observed` so no-ACK sends cannot be confused with
controller-acknowledged sends.

The staged acceptance order is:

1. no-motion connect
2. read-only state acquisition
3. no-motion command ACK timing
4. simulation-mode `servo_j` probe, only if an explicit future task accepts it
5. tiny real joint motion, only after separate approval and a motion runbook

Comparison reports should include ACK latency, command success/error counts,
M561/M568/M569/M570 counts when observed, state age, achieved rate, reconnect
count, timeout count, and parse failure count. A lower `rbscript_tcp` latency
does not bypass controller limits or make 100 Hz or 200 Hz motion safe.
`disable_waiting_ack` can improve apparent throughput but hides immediate ACK
errors and must not be used as proof of controller acceptance.

`rt_script` is future work. It is not part of the current backend contract and
must not be introduced as an undocumented controller setting or hidden bypass.

## State Publication

State JSON should expose:

- top-level observed mode/backend
- command source state
- fault context, including a backward-compatible top-level summary and optional per-arm latched contexts
- send policy
- per-arm read result
- per-arm send result
- worker telemetry, including `command_drops_total`,
  `pending_overwrites_total`, `last_dropped_seq`, `last_enqueued_seq`,
  `last_dispatched_seq`, and `last_completed_seq`
- simulator transport telemetry, including connect attempts, failures,
  suppressed attempts, reconnect counts, request/syscall counters, last connect
  error, next retry timing, and last transport error kind when using
  `RbsimBackend`
- TCP pose and quaternion when FK is available
- Cartesian solve/path telemetry when Cartesian modes are active

State publication is an operational truth surface. Do not hide backend or safety details behind a single `ok` field.
