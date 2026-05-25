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
TCP connection or issuing one-byte reads for each operation. After each
successful TCP connect, the backend enables `TCP_NODELAY` so request/response
latency is not shaped by Nagle buffering.

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

The per-arm `ArmWorkerTelemetry` counters are:

- `worker_queue_policy`, always `latest_wins`
- `worker_command_drops_total`
- `worker_pending_overwrites_total`
- `worker_last_dropped_seq`
- `worker_last_enqueued_seq`
- `worker_last_dispatched_seq`
- `worker_last_completed_seq`

State JSON publishes the same data under each arm's `worker` object and adds
the configured worker read cadence plus cached-state age:

```json
"worker": {
  "enabled": true,
  "queue_policy": "latest_wins",
  "read_period_ns": 10000000,
  "read_period_sec": 0.01,
  "read_rate_hz": 100.0,
  "state_age_us": 4300.0,
  "command_drops_total": 3,
  "pending_overwrites_total": 3,
  "last_dropped_seq": 1201,
  "last_enqueued_seq": 1204,
  "last_dispatched_seq": 1204,
  "last_completed_seq": 1204
}
```

The read cadence is configured under `servo.worker_read_period_sec` or,
equivalently, `servo.worker_read_rate_hz`. The canonical stored value is the
period, and configs must not set both forms. The default is 10 ms / 100 Hz so
simulator worker mode does not poll at the servo loop's highest development
rates by default.

## MIG-23 Worker Lifecycle Queue

Worker mode separates streaming servo targets from lifecycle commands.
`servo_j` keeps the MIG-14 single-slot latest-wins policy because only the most
recent streaming target should be dispatched. Lifecycle commands have different
semantics and must not be routed through or overwritten by that slot.

`ArmWorker` now has a small bounded FIFO lifecycle lane for simulator/mock
worker mode:

- `ResetFault`
- `Stop` for backend stop/shutdown paths where the backend already implements
  structured stop behavior

Lifecycle enqueue and dispatch results are structured `BackendResult<RobotState>`
values. If the FIFO is full, the worker rejects the enqueue explicitly with
`BackendErrorKind::SuppressedByPolicy` and
`error.name="arm_worker_lifecycle_queue_full"`; it must not silently drop an
older lifecycle command. Expired lifecycle commands report
`BackendErrorKind::CommandTimeout`.

`DualArmServoLoop` uses the worker lifecycle lane for `ResetFault` when
`servo.io_model: worker`, then requires fresh valid worker state before
clearing the server fault latch. Direct mode still calls backend `resetFault()`
directly. Real mode still rejects `servo.io_model: worker` in the config parser,
and this migration does not change `RbpodoBackend` stop/reset behavior.

Direct I/O mode may publish the same object with `enabled=false` and zero/default
sequence counters so schema consumers do not need a separate parser path.

## MIG-16 Latched Fault Context

`ServoLoop` preserves the structured `FaultContext` that first causes a fault
latch. Per-arm `last_read` and `last_send` remain live telemetry and may later
show `SuppressedByPolicy` while regular `servo_j` is blocked by the latch. That
live suppression result must not replace the original latched diagnostic.

State JSON publishes a top-level `fault_context` object. It keeps the existing
latch summary fields and, when a context is latched, adds:

- `verdict`
- `domain`
- `arm`
- `backend_op`
- `backend_error_kind`
- `backend_error_name`
- `backend_error_code`
- `retryable`
- `recoverable`
- `robot_fault`
- `transport_fault`
- `state_after_source`
- `reason`

Unavailable detail fields are serialized as JSON null when no latched context
exists. The context is cleared only by an explicit successful `ResetFault` flow
or by server restart.

Latch policy:

- On the transition from not latched to latched, store the `FaultContext`
  returned by the fault classifier.
- Once latched, later regular send suppressions such as
  `backend_error_kind=SuppressedByPolicy` remain visible under each arm's
  `last_send` but do not overwrite `fault_context`.
- MIG-16 does not add an emergency override of a previously latched non-emergency
  context. A future task may define that policy explicitly if needed.

## MIG-15 Rbpodo Read-Only State Semantics

`RbpodoBackend::readState()` separates state acquisition from motion readiness.
A read is successful when the backend communicates with the controller and maps
finite, internally consistent joint state into `RobotState`. Controller state
that is valid for observation but not ready for motion remains an OK read:

- `servo_enabled=false` is reported in `RobotState` and does not by itself make
  `readState().ok=false`.
- Controller fault or emergency flags are reported as `has_error=true` with the
  controller error code when the joint sample is valid.
- Controller/config mode mismatch is a motion-readiness problem, not a state
  acquisition failure.
- Non-finite or missing joint values still fail the read with
  `InvalidJointState`.
- Data-channel request failures still fail the read with a transport read
  error.

Rbpodo `sendServoJ()` remains motion-gated. In real mode it checks
`RB_ALLOW_REAL_MOTION=1` before any send attempt. If the latest cached state is
not motion-ready, `sendServoJ()` rejects with the structured readiness error
such as `ServoDisabled`, `WrongMode`, or `RobotFault`, and may attach that
cached state with `state_after_source="cache"`. These rejections are not
transport write failures.

Real read-only acceptance therefore expects that `q_actual` can publish while
`servo_enabled=false`. The tracked read-only real template still requires
`RB_ALLOW_REAL_ROBOT=1`, keeps `servo.send_servo_commands: false`, and must not
attempt motion mode entry or `servo_j`.

## MIG-21 TCP Pose Quaternion Publishing

FK-enabled state JSON publishes TCP orientation as a normalized quaternion in
addition to the existing display pose fields. Each non-null `tcp_base` and
`tcp_stand` object keeps:

- `x`, `y`, `z` in meters
- `rx`, `ry`, `rz` in radians for display and legacy compatibility
- `quaternion_xyzw: [qx, qy, qz, qw]`
- scalar aliases `qx`, `qy`, `qz`, and `qw`

The quaternion order is explicitly `xyzw`; `qw` is the scalar component. The
publisher normalizes the quaternion before serialization. RPY/Euler values are
not the canonical control, GUI policy, or dataset orientation representation.
They remain available only so older consumers and operator displays continue to
work.

When Pinocchio/FK is disabled or unavailable, TCP pose publication remains
deferred: `tcp_base` and `tcp_stand` are `null`, `has_valid_tcp_pose=false`, and
`tcp_deferred=true`. GUI and `policy_runner` consumers must continue accepting
the legacy schema without quaternion fields; quaternion presence is an
orientation-quality improvement, not a new requirement for joint-only policy
paths.

## MIG-22 Command Source Lease

The command source lease is explicit metadata on accepted UDP command packets
and state publication. It is intended to prevent accidental concurrent command
sources such as GUI teleop and `policy_runner` teleop from both driving the
same server.

Default enforcement remains off:

```yaml
command_source:
  enforce_lease: false
  lease_timeout_sec: 1.0
```

With enforcement off, the server still parses and publishes command source
metadata, active source/session/token fields, and command lease verdict fields,
but non-owner commands are not rejected solely because of lease ownership. This
keeps legacy hardware-free tools compatible.

When `command_source.enforce_lease: true`, normal motion commands require the
active lease. A source acquires or refreshes the lease by sending an accepted
lease-owning motion command with `source_id` and optional `session_id` /
`lease_token`. `EmergencyStop` bypasses the lease so any accepted source can
stop motion. `ResetFault` requires the active lease when enforcement is enabled.
Rejections are explicit parser diagnostics such as
`command_source_lease_required`, `command_source_lease_conflict`, or
`command_source_lease_token_mismatch`.

The simulator TCP acceptance profile may enable lease enforcement as evidence.
Tracked default mock/simulator operator configs do not use lease enforcement as
a hidden behavior change.

## MIG-24 Camera Readiness And Policy Wiring

Camera readiness is a policy input, not a servo-loop requirement. Joint-only
`policy_runner` action sources can run without camera observations. Any
camera-dependent action source must declare `requires_camera` and a
`camera_stale_timeout_sec`; camera-geometry-dependent action sources must also
declare `requires_camera_geometry`.

Policy sources fail closed when required camera readiness is absent, stale, or
lacks measured accepted geometry. The active calibration remains a configured
estimate and is not real geometry acceptance evidence.

Real three-camera acceptance is documented separately in
`docs/runbooks/camera_acceptance.md`. It is a hardware workflow for RealSense
capture and policy-readiness evidence only; it does not imply real robot
connection, real `servo_j`, real Cartesian motion, or force-control readiness.

## MIG-26 Final Rebaseline

The current review and validation baseline after MIG-13+ is:

- Command flow is `CommandBuffer -> ServoCoordinator/DualArmServoLoop ->
  Left/Right ArmWorker -> simulator/rbpodo endpoint`.
- `servo.io_model: direct` remains the stable default. Worker I/O is accepted
  for simulator evidence; real worker mode still needs separate read-only
  hardware acceptance before any promotion.
- `RbsimBackend` uses persistent JSON-lines transport per simulator backend
  instance and exposes transport counters for tests/diagnostics.
- `ArmWorker` has latest-wins streaming `servo_j` telemetry and a separate
  bounded lifecycle queue for reset/stop-like commands.
- Rbpodo read-only state semantics remain separate from motion readiness.
  Unverified real `stop()` / `resetFault()` recovery still fails closed and
  requires operator intervention.
- `FaultContext` is latched as structured state and is not replaced by later
  routine suppression telemetry.
- Command lease enforcement defaults to off and must be enabled explicitly in
  acceptance profiles that need it.
- TCP Pose/Delta acceptance is simulator-only, Pinocchio-gated, and keeps
  `cartesian_control.allow_in_real: false`.
- Camera acceptance is separate from hardware-free validation and separate from
  real robot acceptance.

The one-command developer rebaseline is:

```bash
bash scripts/codex_gate.sh MIG-26
```

That gate must remain hardware-free by default. It may run optional Pinocchio
and simulator TCP acceptance when the dependency is already installed, but it
must not require rbpodo, RealSense hardware, real robot network access, or any
real motion environment gate.
