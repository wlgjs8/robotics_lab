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

Read-only diagnostic startup is a controller bring-up mode, not a
motion-ready mode. When `servo.send_servo_commands: false` and the explicit
`servo.allow_readonly_*_startup` flags are set, startup may continue after
state acquisition succeeds even if motion readiness fails because of a robot
fault, controller-not-ready state, or configured joint-range violation. The
state stream must still publish the state as unsafe, including
`startup_validation`, per-arm `startup_invalid_reasons`,
`q_range_violations`, and backend readiness diagnostics such as
`motion_readiness_error_kind` / `motion_readiness_error_name`. This mode exists
only to surface raw controller diagnostics such as rbpodo status flags and
joint feedback in JSON; it must not send `servo_j`.

Rbpodo status fields are treated as firmware/SDK-layout-dependent diagnostics.
The backend preserves the raw values in per-arm state JSON under
`rbpodo_diagnostics.raw`, including `time`, `real_vs_simulation_mode`,
`init_state_info`, `init_error`, `op_stat_sos_flag`, `op_stat_ems_flag`,
`op_stat_soft_estop_occur`, `op_stat_collision_occur`, and
`op_stat_self_collision`. Boolean status flags with values other than `0` or
`1`, non-finite or implausibly tiny nonzero controller time, and unknown
`real_vs_simulation_mode` values mark `diagnostics_suspect=true`.
Suspicious diagnostics do not make `readState()` fail when joint acquisition is
otherwise valid, but they do make the state motion-unsafe and block
`sendServoJ()` with a `RobotFault` named `rbpodo_diagnostics_suspect`.
Garbage-looking raw flag values must be kept in `rbpodo_diagnostics.raw` rather
than used as the sole controller `error_code`.

Raw controller joint angles may use continuous or wrapped representations. When
`safety.joint_wrap_for_startup_validation: true`, startup range diagnostics may
normalize configured joints using `safety.joint_wrap_period_deg`; `0` disables
wrapping for that joint and `360` treats values separated by a full revolution
as equivalent for range diagnostics. For example, raw `-317 deg` with range
`[-190, 190]` and period `360` normalizes to `43 deg`.

This wrapping policy is for startup diagnostics only by default. State JSON must
keep raw `q_actual_deg`, publish any `q_range_wrapped` entries, and may publish
`q_actual_normalized_for_safety_deg` for the validation view. If the raw value
is already in range, it is left unchanged. If the configured range width is
greater than or equal to the wrap period, the normalized value is only a
deterministic diagnostic representative, not motion-ready evidence.
`safety.joint_wrap_for_motion_safety` is refused until a future task implements
continuous motion-safe unwrapping, because silently wrapping command targets can
create joint discontinuities.

If `servo.send_servo_commands: true`, startup remains strict. Robot faults,
wrong mode/not-ready state, and joint-range violations must fail startup rather
than being converted into a healthy condition.

Real `sendServoJ()` requires:

- valid state acquisition
- controller motion readiness
- config `send_servo_commands: true`
- `RB_ALLOW_REAL_ROBOT=1`
- `RB_ALLOW_REAL_MOTION=1`

Streaming Cartesian primitives remain simulator-first. In `run_mode: simulation`,
`TcpTwistStand`, `TcpTwistLocal`, `TcpLinearMove`, and
`TcpCircleMove` require `cartesian_control.enable: true` and
`cartesian_control.allow_in_simulation: true`.

The only real-controller carve-out is rbpodo controller `pgmode` simulation.
For those same streaming primitives, `run_mode: real` may execute Cartesian
target generation only when the selected backend is `rbpodo`, the robot
`operation_mode` is `simulation`,
`cartesian_control.allow_in_controller_simulation: true`,
`servo.allow_controller_simulation_motion: true`, and all of these env
gates are set:

- `RB_ALLOW_REAL_ROBOT=1`
- `RB_ALLOW_REAL_MOTION=1`
- `RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1`
- `RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1`
- `RB_RBPODO_PGMODE_SIMULATION_CONFIRMED=1`

This is not physical real Cartesian enablement. `operation_mode: real` remains
blocked for streaming Cartesian primitives even if `RB_ALLOW_REAL_CARTESIAN=1`
is present. State JSON must expose `cartesian_available`,
`cartesian_unavailable_reason`, and a `cartesian_gate` object with the
backend/run-mode/config/env decision fields so a controller-simulation
benchmark cannot be mistaken for physical motion approval.

Real stop/reset APIs remain conservative until verified. If no verified API is wired, return `DependencyUnavailable` and require operator intervention.

Rbpodo is the primary vendor-library real backend. `dual_real.example.yaml` and
the `dual_real_*_ack.example.yaml` files are tracked templates, not
ready-to-run real motion configs. Site-specific real configs belong under
`rb_servo_server/config/local/`, which is intentionally user-owned and
gitignored.

### Rbpodo ACK Semantics

`disable_waiting_ack: false` is the default and required baseline. In this
ACK-on mode, `sendServoJ().accepted=true` means rbpodo observed controller ACK
for the `move_servo_j` command. State JSON and servo logs expose this as:

- `ack_policy: "wait"`
- `ack_observed: true`
- `controller_acceptance_observed: true`
- `send_acceptance_semantics: "controller_ack_observed"`
- `rbpodo_waiting_ack: true`
- `ack_wait_duration_us`

`disable_waiting_ack: true` changes the meaning of a successful send. rbpodo's
SDK returns after the socket/API send path instead of waiting for immediate
controller ACK. In this ACK-off mode, `accepted=true` means the command was
sent through the client path; it does not prove the controller accepted it. The
telemetry must show:

- `ack_policy: "disabled"`
- `ack_observed: false`
- `controller_acceptance_observed: false`
- `send_acceptance_semantics: "socket_send_only"`
- `rbpodo_waiting_ack: false`

ACK-off is an experimental supervised acceptance mode, not a safe mode. Real
motion with ACK waiting disabled requires the normal real gates plus:

```bash
RB_ALLOW_RBPODO_ACK_DISABLED_MOTION=1
```

ACK-off runs need stronger monitoring because immediate controller rejection is
not observed on the command call. At minimum, review `readState()`, controller
`error_code`, state staleness, `q_ref`/`q_actual` tracking, and Rainbow system
codes such as M561/M568/M569/M570 when they are observable.

`command_timeout_sec` is the rbpodo command/ACK timeout used by
`move_servo_j`. The 100 Hz example uses `0.05`; the 200 Hz examples use `0.02`
so command stalls are visible quickly during acceptance. Values must be finite
and positive.

### Rbpodo Servo J Parameters

`RbpodoBackend::sendServoJ()` maps rbpodo config fields directly to Rainbow
UI Script `move_servo_j(jnt, t1, t2, gain, alpha)`:

- `servo_t1_sec` -> `t1`, the controller arrival/command time. For streaming
  Servo J this should match the server command period, `1 / servo.rate_hz`.
- `servo_t2_sec` -> `t2`, the controller hold time. This is not UR-style
  lookahead.
- `servo_gain` -> `gain`.
- `servo_alpha` -> `alpha`, the low-pass-filter gain. This is not
  acceleration.

Rainbow joint targets are in degrees. Config validation enforces the documented
Servo J ranges before the backend is used:

- `servo_t1_sec >= 0.002`
- `0.02 < servo_t2_sec < 0.2`
- `servo_gain > 0`
- `0 < servo_alpha < 1`

Deprecated aliases remain accepted with warnings while configs migrate:

- `servo_time_sec` -> `servo_t1_sec`
- `servo_lookahead_sec` -> `servo_t2_sec`
- `servo_acc` -> `servo_alpha`

Setting both a canonical key and its deprecated alias is refused to avoid
ambiguous real-controller parameters. For real rbpodo configs with
`servo.send_servo_commands=true`, `servo_t1_sec` must match the configured
`servo.rate_hz` period within `servo.servo_t1_rate_match_tolerance_ratio`
unless `servo.allow_servo_t1_rate_mismatch=true` is set after explicit
acceptance. Read-only configs with `send_servo_commands=false` may load with a
mismatch warning, but the warning is not motion approval.

New configs must not use `servo_acc`; use `servo_alpha`. New configs must not
use `servo_lookahead_sec`; use `servo_t2_sec`. The old names are compatibility
aliases only.

### Rbpodo Real Acceptance Sequence

Real rbpodo acceptance is staged. Later stages depend on clean artifacts from
earlier stages:

1. `read_only`: connect with `send_servo_commands: false`, verify valid joint
   state, low `state_age_us`, no fault latch, expected rbpodo backend, and no
   Servo J sends.
2. 100 Hz ACK-on no-op or controller simulation-mode Servo J: `servo.rate_hz:
   100`, `servo_t1_sec: 0.01`, `disable_waiting_ack: false`.
3. 200 Hz ACK-on: `servo.rate_hz: 200`, `servo_t1_sec: 0.005`,
   `disable_waiting_ack: false`.
4. 200 Hz ACK-off: same 200 Hz timing, `disable_waiting_ack: true`, with
   explicit ACK-off acceptance. Success is socket/API send evidence only.
5. Tiny real joint motion: not authorized by the documentation above. It
   requires a future explicit motion approval task and operator supervision.

Do not copy aggressive ACK-off or high-rate settings into a real baseline until
the matching acceptance artifacts have been reviewed.

Required log and state fields for review include:

- `send_duration_us`
- `ack_wait_duration_us`
- `ack_observed`
- `controller_acceptance_observed`
- `state_age_us`
- `error_code`
- M561/M568/M569/M570 when available
- `q_ref` and `q_actual`

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
- rbpodo raw diagnostics and interpretation status under
  `rbpodo_diagnostics` when using `RbpodoBackend`
- startup joint wrapping diagnostics, including raw-preserving
  `q_range_wrapped` and optional `q_actual_normalized_for_safety_deg`
- TCP pose and quaternion when FK is available
- Cartesian solve/path telemetry when Cartesian modes are active

State publication is an operational truth surface. Do not hide backend or safety details behind a single `ok` field.
