# rb_simulator Architecture Plan

## Target Result

Create a hardware-free simulator that lets `rb_servo_server` validate backend
integration, state truthfulness, stop/reset semantics, and fault handling before
any Rainbow Robotics simulator or real robot session is approved.

`rb_simulator` is not a camera simulator, not a high-level policy runner, and
not a replacement for real hardware acceptance. It is a deterministic backend
contract exerciser for the servo server.

## Reference Constraints

- `mo_rbsim_docker` runs two RB cobot simulator containers with fixed IPs and
  Rainbow ports, but depends on an external OVA image and privileged runtime.
- `rb_servo_server` already has an `IRobotBackend` boundary with
  `MockBackend` and an intentionally incomplete `RbpodoBackend`.
- `dual_rb_simulator.yaml` and its `dual_rbsim.yaml` compatibility alias point
  at the repo-local `rbsim_local` backend on loopback TCP.

The new simulator should therefore use a new `rbsim` backend path instead of
expanding the mock backend or treating the incomplete rbpodo path as ready.

## Simulator Role

`rb_simulator` owns these responsibilities:

- Maintain one simulated state machine per arm: disconnected, connected,
  initialized, servo enabled, stopped, faulted.
- Maintain joint actual and target arrays for six joints per arm.
- Advance actual joints toward latest targets using deterministic timing and
  configurable rate/lag.
- Expose backend-like operations for state reads, servo targets, stop, and
  reset fault.
- Expose fault-injection controls for tests and smoke checks.
- Record an event log that can be joined with `rb_servo_server` snapshots.

It must not enforce high-level policy safety. Joint limits, command freshness,
dual-arm stop policy, fault latch behavior, and operator-visible truthfulness
remain `rb_servo_server` responsibilities.

## Communication With rb_servo_server

Preferred local contract:

- Transport: loopback TCP JSON Lines for request/response operations.
- One simulator process owns both arms. This is the standard topology for the
  current hardware-free gate.
- One `RbsimBackend` instance per arm connects to the simulator and includes
  `arm: "left"` or `arm: "right"` in each request.
- The simulator listens on a control endpoint for backend operations and an
  optional admin endpoint for test fault injection.

The current config points both servo-server arms at the same control endpoint,
`tcp://127.0.0.1:50200`. The simulator demultiplexes requests by the `arm`
field and keeps independent left/right state in one process. A future topology
may split left and right onto separate endpoints or separate simulator
processes, but that would be a new contract and must update configs, smoke
runner defaults, and docs together.

Initial request shapes:

```json
{"op":"connect","arm":"left"}
{"op":"initialize","arm":"left","operation_mode":"simulation","speed_bar":0.1}
{"op":"read_state","arm":"left"}
{"op":"send_servo_j","arm":"left","q_target_deg":[0,-30,80,0,60,0]}
{"op":"stop","arm":"left"}
{"op":"reset_fault","arm":"left"}
```

Initial response shape:

```json
{
  "ok": true,
  "arm": "left",
  "state": {
    "q_actual_deg": [0,-30,80,0,60,0],
    "q_target_deg": [0,-30,80,0,60,0],
    "dq_actual_deg_s": [0,0,0,0,0,0],
    "has_valid_joint_state": true,
    "connection_state": "Connected",
    "servo_enabled": true,
    "has_error": false,
    "error_code": 0,
    "robot_time_ns": 0
  }
}
```

Failure responses must be explicit and machine-testable:

```json
{"ok":false,"arm":"left","error":"fault_latched","error_code":2001}
```

## Backend Semantics To Mimic

`connect`

- Succeeds when the simulator endpoint is reachable and the requested arm is
  configured.
- Fails without mutating servo-server state when the endpoint is unreachable.

`initialize`

- Sets operation mode and speed parameters.
- Requires a connected arm.
- Makes subsequent `readState` calls valid unless a fault injection disables
  validity.

`readState`

- Returns current actual joint state, target joint state, velocity estimate,
  connection state, servo flag, fault flag, and error code.
- Can be configured to return invalid state, stale robot time, or disconnected
  state for safety-regression tests.

`sendServoJ`

- Requires connected, initialized, non-faulted state.
- Accepts a six-joint target and updates the simulator target.
- Returns failure without target mutation when send-failure injection is active.

`stop`

- Sets target to current actual position and disables active motion.
- Returns explicit failure when stop-failure injection is active.

`resetFault`

- Clears fault only when reset-failure injection is inactive.
- Does not silently re-enable motion. After reset, the server must still
  re-baseline from a fresh valid state and require a new `ArmMotion`.

## Fault Injection

The admin endpoint should support:

- disconnect/reconnect per arm
- invalid joint-state response
- send failure per arm
- stop failure per arm
- reset failure per arm
- tracking-error bias or frozen actual joints
- synthetic backend error code
- latency/jitter budget injection

Fault injection must be deterministic and resettable so CTest and Python smoke
checks can run without sleeps longer than the servo loop requires.

## Configuration

`rb_servo_server` should gain:

- `BackendType::Rbsim`
- `RbsimBackend`
- YAML parsing for `backend_type: rbsim`
- config fields for the simulator control endpoint
- a `config/dual_rb_simulator.yaml` profile that binds server command/state
  ports to loopback and points both arms at the same `rb_simulator` control
  endpoint

`rb_simulator` should carry its own config for arm names, initial joints, model
identity, listen endpoints, motion lag, and fault defaults.

## Verification Gates

Hardware-free gates:

- Unit tests for simulator state machine and protocol parsing.
- Unit tests for `RbsimBackend` mapping to `RobotState`.
- Integration smoke: start simulator, start `rb_servo_server` with rbsim config,
  send `ArmMotion`, send small joint target, verify state stream and servo log.
- Fault smoke: inject send failure, invalid state, stop failure, reset failure,
  and disconnect; verify servo server latches or holds truthfully.

Hardware-gated gates:

- Real Rainbow simulator / `mo_rbsim_docker` compatibility remains blocked.
- Real robot acceptance remains blocked.
- Any privileged Docker or external network exposure remains blocked.

## Stop Condition

This plan is complete when the new workspace exists, the simulator/backend
contract is documented, implementation/test/review work is split into child
tasks, and hardware-dependent validation remains explicitly blocked.
