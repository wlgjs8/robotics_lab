# Backend Contract

This file records the `rb_servo_server` side of the MIG backend-contract and
non-blocking servo-loop migration. The root source of truth remains
`../../servo_backend_contract.md`; this component note documents the
MIG-01 result vocabulary added inside `rb_servo_server`.

Archived note: this is MIG history, not current runtime guidance. When this
file conflicts with `docs/servo_backend_contract.md` or `docs/architecture.md`,
the current docs win.

MIG-01 adds the vocabulary only. It does not change `IRobotBackend` method
signatures, backend behavior, `DualArmServoLoop`, the state publisher schema,
or simulator protocol.

## Structured Result Vocabulary

The result vocabulary lives in
`include/rb_servo/robot/backend_result.hpp`:

- `BackendOp`: `Connect`, `Initialize`, `ReadState`, `SendServoJ`, `Stop`,
  `ResetFault`.
- `BackendErrorKind`: transport failures, protocol/schema mismatches,
  endpoint/arm mismatches, disconnected or uninitialized robot state, servo
  disabled/wrong mode, robot/controller faults, invalid state or target,
  controller rejection, command timeout, dependency unavailable, policy
  suppression, and unknown failures.
- `BackendError`: machine-readable kind/code/name/message plus explicit
  `retryable`, `recoverable`, `robot_fault`, and `transport_fault` flags.
- `BackendTiming`: operation start/end timestamps and microsecond duration.
- `BackendResult<T>`: structured result carrier for non-motion backend
  operations.
- `SendServoJRequest` and `SendServoJResult`: explicit joint-send request and
  acceptance/rejection record.
- `ArmSendResult` and `DualSendResult`: per-arm and dual-arm send aggregation
  records for the future non-blocking coordinator.

`RobotFault` and transport failures are intentionally separate. A robot fault
sets `robot_fault=true` and `transport_fault=false`; transport failures such as
`TransportTimeout` set `transport_fault=true` and `robot_fault=false`.
`SuppressedByPolicy` is diagnostic data for a safety/configuration gate and is
not classified as a robot or transport fault by default.

`SendServoJResult::state_after` is optional. When present,
`state_after_source` must make the source explicit:

```text
response | cache | none
```

Helpers in `backend_result.hpp`/`.cpp` provide canonical `toString` mappings,
default error flag policy, timing construction, `ReadState` success/failure
results, and accepted/rejected `SendServoJ` results.

## MIG-02 Boundary

MIG-02 may begin adapting backend implementations to produce these structured
results internally, but it must still preserve the current public
`IRobotBackend` signatures until that migration step is explicitly assigned.
The real robot connection, real joint motion, and real Cartesian motion gates
remain unchanged.
