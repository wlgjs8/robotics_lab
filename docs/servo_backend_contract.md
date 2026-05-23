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

The target still preserves one endpoint per arm:

```text
left rbpodo backend  -> 172.28.60.200
right rbpodo backend -> 172.28.60.201
```

Simulator topology must remain isomorphic by using one independent simulator
endpoint per arm, never the physical robot IP addresses as simulator defaults.

## MIG-00 Gate Bootstrap

MIG-00 establishes only migration infrastructure and safety cleanup:

- `scripts/codex_gate.sh` recognizes `MIG-00` through `MIG-12`.
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
