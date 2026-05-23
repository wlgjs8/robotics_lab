# Servo Backend Contract Migration

This document is the source of truth for the MIG backend-contract and
non-blocking servo-loop migration. The migration is not a real-motion
enablement. It must preserve the existing hardware gates:

```bash
RB_ALLOW_REAL_ROBOT=1
RB_ALLOW_REAL_MOTION=1
RB_ALLOW_REAL_CARTESIAN=1
```

Mock and simulator gates may prove software behavior only. They do not grant
permission to connect to or move physical RB3-730 hardware. Force, admittance,
and impedance control remain unavailable:

```yaml
force_control:
  provider: null
  enable: false
```

## Migration Target

`IRobotBackend` currently exposes operations that can collapse important robot
state into bool success/failure and log text. The target contract is structured
results:

- `BackendResult` for lifecycle, read, reset, and non-motion backend actions.
- `SendServoJResult` for joint servo send attempts.

Structured results must carry enough information for callers and logs to
distinguish accepted commands, rejected commands, disconnected backends,
timeouts, invalid configuration, safety-gate denial, unsupported operations,
and backend-specific failures. They must be explicit data, not parsed log text.

Do not fake rbpodo APIs while adding this contract. If rbpodo headers or docs
are not available, keep rbpodo builds gated behind the existing dependency
switches and report the limitation.

## Backend Result Taxonomy

The review baseline is a structured taxonomy, not bool + log-string behavior:

| Kind | Meaning | Typical operator action |
| --- | --- | --- |
| `RobotFault` | The controller or simulator reports a robot/controller fault state. | Stop the workflow, inspect the reported code/name, and recover through the approved robot procedure. |
| `TransportWriteFailed` | The command channel could not write a request, or a write-like operation failed before controller acceptance was known. | Treat the command as not accepted; investigate network/backend health. |
| `SuppressedByPolicy` | The server intentionally did not perform an operation because a safety gate, read-only mode, stopped worker, or timeout policy blocked it. | Check mode, environment gates, and command freshness rather than retrying blindly. |
| `WrongMode` | The controller-reported mode conflicts with the selected config, such as real/simulation mismatch. | Fix controller/config mode alignment before any motion attempt. |

These kinds are diagnostic and safety inputs. They do not by themselves enable
real connection or real motion.

## Servo Loop Ownership Boundary

`ServoLoop` should remain responsible for timing policy, command freshness,
fault latching, safety checks, and choosing hold versus commanded targets. It
should gradually stop owning blocking network I/O. Blocking connect, read,
reset, and `servo_j` send work should move behind arm-specific workers so loop
timing is not directly coupled to controller network latency.

The future target is:

```text
CommandBuffer -> ServoCoordinator -> Left ArmWorker  -> left IRobotBackend
                                \-> Right ArmWorker -> right IRobotBackend
```

`CommandBuffer` owns the latest operator or policy command. `ServoCoordinator`
owns dual-arm coordination, stop-both-arms policy, and result aggregation.
Each `ArmWorker` owns one arm backend instance, network I/O, reconnect/reset
behavior, and the latest structured backend result for diagnostics.

Current `io_model` status:

- `servo.io_model: direct` is the stable default for tracked mock, simulator,
  and real template configs.
- `servo.io_model: worker` is accepted for simulator evidence when the
  MIG-10/MIG-11 worker smoke passes with
  `rb_servo_server/config/dual_simulator_worker.yaml`.
- real + `worker` remains disabled or experimental until a separate real
  read-only acceptance task proves connection/read behavior without motion.

The target still preserves one endpoint per arm:

```text
left rbpodo backend  -> 172.28.60.200
right rbpodo backend -> 172.28.60.201
```

Simulator topology must remain isomorphic by using one independent simulator
endpoint per arm, never the physical robot IP addresses as simulator defaults.

## MIG-00 Gate Bootstrap

MIG-00 establishes only migration infrastructure and safety cleanup:

- `scripts/codex_gate.sh` recognizes `MIG-00` through `MIG-26`.
- MIG gates reuse existing hardware-free checks and never require real robot
  hardware.
- `MIG-00` runs shell syntax checks and documentation/config safety assertions.
- The tracked real robot template remains read-only by default with
  `servo.send_servo_commands: false`.

Later MIG tasks may change `rb_servo_server` source code, but each task must
keep real robot connection, real joint motion, and real Cartesian motion gated
by the environment variables above.

## MIG-03 Simulator Error Envelope

The hardware-free `rb_simulator` error response is structured. Failed responses
carry:

```json
{
  "ok": false,
  "error": {
    "kind": "RobotFault",
    "name": "fault_latched",
    "message": "left is faulted",
    "code": 2222,
    "retryable": false,
    "recoverable": true
  },
  "state": {}
}
```

`error.kind` uses the C++ `BackendErrorKind` spelling when the simulator can
classify the failure. `error.name` remains the simulator-specific symbolic name,
and `error.code` remains the numeric simulator/controller code. Stateful robot
or controller rejections include `state` in the same response so callers can
record `SendServoJResult.state_after` without issuing a second read.

Stateful simulator names map as follows:

| Simulator name | Backend kind | State included |
| --- | --- | --- |
| `fault_latched` | `RobotFault` | yes |
| `servo_disabled` | `ServoDisabled` | yes |
| `disconnected` | `RobotDisconnected` | yes |
| `not_initialized` | `RobotNotInitialized` | yes |
| `invalid_joint_state` | `InvalidJointState` | yes |
| `wrong_arm` | `WrongArm` | no |
| `wrong_endpoint` | `WrongEndpoint` | no |
| `unsupported_schema_version` | `UnsupportedSchema` | no |
| `bad_request`, `invalid_json`, `unknown_operation` | `ProtocolError` | no |

Injected transport-like failures are not robot faults:

| Simulator name | Backend kind |
| --- | --- |
| `send_failure_injected` | `TransportWriteFailed` |
| `read_failure_injected` | `TransportReadFailed` |
| `stop_failure_injected` | `TransportWriteFailed` |
| `reset_failure_injected` | `TransportWriteFailed` |

`RbsimBackend` parses `error.kind` when present, preserves
`error.name/code/message`, and falls back from older name-only simulator
responses to the same mapping. When an error response includes `state`,
`sendServoJ` rejects with `state_after_source="response"`; otherwise the source
is `"none"`.

## MIG-04 Rbpodo Structured Result Mapping

`RbpodoBackend` must keep the real-robot gates unchanged while preserving the
cause of controller and SDK failures in structured fields.

Verified rbpodo API surface for this migration:

- `rb::podo::Cobot<>(address)`
- `rb::podo::CobotData(address)`
- `rb::podo::CobotData::request_data(timeout)`
- `rb::podo::Cobot<>::move_servo_j(...)`
- `rb::podo::ReturnType::{is_success,is_timeout,is_error}`
- `rb::podo::ResponseCollector::has_error()`
- `rb::podo::SystemState::sdata.jnt_ang`, `jnt_ref`, `time`,
  `real_vs_simulation_mode`, `init_state_info`, `init_error`,
  `op_stat_sos_flag`, `op_stat_ems_flag`, `op_stat_soft_estop_occur`,
  `op_stat_collision_occur`, and `op_stat_self_collision`

Structured rbpodo mapping:

| Condition | Backend kind | Required details |
| --- | --- | --- |
| `RB_ALLOW_REAL_ROBOT` missing for real connect | `SuppressedByPolicy` | `error.name="rbpodo_real_robot_gate_closed"` |
| `RB_ALLOW_REAL_MOTION` missing for real `sendServoJ` | `SuppressedByPolicy` | `error.name="rbpodo_motion_gate_closed"` |
| `SystemState` fault code is nonzero | `RobotFault` | `error.code` is the controller code |
| `real_vs_simulation_mode` conflicts with config | `WrongMode` | `error.code` is the reported mode value |
| `init_state_info` is not activation done stage 6 | `ServoDisabled` | `error.code` is the activation stage |
| non-finite joint sample | `InvalidJointState` | state is not valid |
| connected backend returns no data | `TransportReadFailed` | no valid state is invented |
| known disconnected backend | `RobotDisconnected` | no transport read is attempted |
| command acknowledgement timeout | `CommandTimeout` | command was not accepted |
| controller error response | `ControllerRejected` | response category/message are preserved when present |
| command-channel exception during send | `TransportWriteFailed` | exception text is preserved |

`initialize()` is read-only in MIG-04. It requests and validates state, but does
not call `set_operation_mode`, `set_speed_bar`, or any command that enters a
motion mode.

Real robot policy:

- real read-only connect is allowed only when `RB_ALLOW_REAL_ROBOT=1`
- real `servo_j` motion requires both `RB_ALLOW_REAL_ROBOT=1` and
  `RB_ALLOW_REAL_MOTION=1`
- real Cartesian/TCP motion requires `RB_ALLOW_REAL_CARTESIAN=1` in addition to
  the real robot gates
- unverified rbpodo `stop()` and `resetFault()` must be reported as requiring
  operator intervention on a real fault; they must not be treated as automatic
  controller recovery

`sendServoJ()` may include the backend's recent timestamped state cache on
rejections. When included, `state_after_source` must be `"cache"`. The cache is
only diagnostic state from the last rbpodo state sample; it is not a hidden
retry or second read.

`dq_actual_deg_s` remains finite zero for rbpodo because the inspected
`SystemState` has no verified joint-velocity field. Replace it only after a
specific rbpodo header or official doc field is verified.

`stop()` and `resetFault()` remain unverified for controller-level servo
hold/fault-reset. They fail closed without issuing a robot API call:

| Operation | Backend kind | Required name |
| --- | --- | --- |
| `stop()` | `DependencyUnavailable` | `rbpodo_stop_unverified` |
| `resetFault()` | `DependencyUnavailable` | `rbpodo_reset_fault_unverified` |

Their messages must state that operator intervention is required. Reset must
not implicitly re-enable motion.

## MIG-12 Migration Rebaseline

MIG-12 closes the migration baseline by making review surfaces explicit:

- docs describe `CommandBuffer -> ServoCoordinator -> Left/Right ArmWorker`
  and the direct/worker `io_model` status above.
- tracked real config remains a template with real IPs only in
  `dual_real.example.yaml`, `servo.send_servo_commands: false`, and no implicit
  real connection unless `RB_ALLOW_REAL_ROBOT=1`.
- tracked simulator configs use loopback or compose service DNS, never the real
  robot IP addresses.
- deprecated simulator compatibility filenames are marked as historical or
  compatibility-only and are not new acceptance evidence.
- `network.state_pub_rate_hz` is wired to the UDP state publisher period.
- Pinocchio-enabled C++ validation is optional: `scripts/codex_gate.sh MIG-12`
  runs it when the `pinocchio` CMake package is available, while
  `scripts/hardware_free_validation.sh` keeps `RB_SERVO_ENABLE_PINOCCHIO=OFF`.

## MIG-13 Persistent Simulator Transport

`RbsimBackend` owns one persistent JSON-lines TCP client per backend instance,
which means one socket per configured simulator arm endpoint during healthy
operation. The client sends multiple request lines over the same connection and
uses buffered chunk reads to extract response lines, rather than opening a new
TCP connection or issuing one-byte reads for each operation.

The backend still increments `request_id` monotonically per `RbsimBackend`
instance and preserves the MIG-03 structured result semantics. Simulator error
responses that include `state` continue to populate `SendServoJResult.state_after`
with `state_after_source="response"`.

The persistent socket is closed on transport/protocol-corruption classes:

- `TransportConnectFailed`
- `TransportWriteFailed`
- `TransportReadFailed`
- `TransportTimeout`
- `ProtocolError`

Controller-level or protocol-successful robot rejections do not close the
transport by themselves:

- `RobotFault`
- `ServoDisabled`
- `WrongMode`
- `WrongArm`
- `InvalidTarget`

The simulator transport records internal counters for diagnostics and tests:
`connections_opened_total`, `reconnects_total`, `requests_total`,
`read_syscalls_total`, `write_syscalls_total`, and
`last_transport_error_kind`. These counters are hardware-free evidence only and
do not change any real robot gate.

## MIG-14 ArmWorker Latest-Wins Telemetry

`ArmWorker` keeps the latest-wins queue policy for streaming `servo_j` targets:
there is at most one pending request per arm worker. When a new request
overwrites an older pending request with a different command sequence, the
older request is counted as dropped/superseded exactly once. Drops are
diagnostic telemetry only and do not latch a fault by themselves.

The per-arm worker telemetry fields are:

- `worker_queue_policy`, always `latest_wins`
- `worker_command_drops_total`
- `worker_pending_overwrites_total`
- `worker_last_dropped_seq`
- `worker_last_enqueued_seq`
- `worker_last_dispatched_seq`
- `worker_last_completed_seq`

State JSON publishes the same data under each arm's `worker` object:

```json
"worker": {
  "enabled": true,
  "queue_policy": "latest_wins",
  "command_drops_total": 3,
  "pending_overwrites_total": 3,
  "last_dropped_seq": 1201,
  "last_enqueued_seq": 1204,
  "last_dispatched_seq": 1204,
  "last_completed_seq": 1204
}
```

Direct I/O mode may publish the same object with `enabled=false` and zero/default
sequence counters so schema consumers do not need a separate parser path.
