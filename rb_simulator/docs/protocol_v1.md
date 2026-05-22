# rb_simulator Protocol v1

This protocol is a hardware-free contract for `RbsimBackend`. It is
not the Rainbow Robotics Virtual Simulator protocol, does not speak `rbpodo`,
and must bind to loopback by default.

## Transport

- Framing: one UTF-8 JSON object per TCP line, terminated by `\n`.
- Default control endpoint: `tcp://127.0.0.1:50200`.
- Default admin endpoint: `tcp://127.0.0.1:50201`.
- Default schema version: `rbsim.v1`.
- Client timeout guidance: fail a request if no response line arrives within
  the servo-server backend timeout budget. Initial local tests should use
  100-500 ms and avoid hidden retries.

The simulator rejects non-loopback bind addresses by default to avoid accidental
production or hardware-facing exposure.

## Topology

Protocol v1 standardizes a single dual-arm simulator process. Both
`rb_servo_server` `RbsimBackend` instances connect to the same control endpoint
and set the request `arm` field to `left` or `right`. The process owns
independent state for both arms and exposes one admin endpoint for deterministic
test hooks.

Splitting left and right across separate endpoints or separate simulator
processes is a future option, not the current contract. Such a change must
revise this protocol, the config pair, and smoke-runner defaults together.

## Request Envelope

```json
{
  "schema_version": "rbsim.v1",
  "request_id": "client-unique-id",
  "op": "send_servo_j",
  "arm": "left",
  "params": {
    "q_target_deg": [0, -30, 80, 0, 60, 0]
  }
}
```

Fields:

- `schema_version`: required string, currently `rbsim.v1`.
- `request_id`: optional string echoed in the response.
- `op`: required operation string.
- `arm`: required for per-arm control operations and most admin operations.
- `params`: optional object for operation-specific arguments.

## Response Envelope

Successful responses:

```json
{
  "schema_version": "rbsim.v1",
  "request_id": "client-unique-id",
  "ok": true,
  "arm": "left",
  "server_time_ns": 123456789,
  "state": {
    "arm": "left",
    "q_actual_deg": [0, -30, 80, 0, 60, 0],
    "q_target_deg": [0, -30, 80, 0, 60, 0],
    "dq_actual_deg_s": [0, 0, 0, 0, 0, 0],
    "stale_state": false,
    "has_valid_joint_state": true,
    "connection_state": "Connected",
    "servo_enabled": true,
    "has_error": false,
    "fault_recoverable": true,
    "error_code": 0,
    "robot_time_ns": 0,
    "lifecycle_state": "servo_enabled"
  }
}
```

Failure responses:

```json
{
  "schema_version": "rbsim.v1",
  "request_id": "client-unique-id",
  "ok": false,
  "arm": "left",
  "server_time_ns": 123456789,
  "error": {
    "name": "send_failure_injected",
    "message": "left send failure injected",
    "code": 2101
  }
}
```

Error names and codes are stable enough for tests:

| Code | Name | Meaning |
| --- | --- | --- |
| 1001 | `disconnected` | Arm is disconnected. |
| 1002 | `not_initialized` | Operation requires initialization. |
| 1003 | `servo_disabled` | Servo target requires servo enabled. |
| 1004 | `unknown_arm` | Arm key is not configured. |
| 2001 | `fault_latched` | Arm has a latched simulator fault. |
| 2002 | `invalid_joint_state` | Joint state validity was disabled. |
| 2101 | `send_failure_injected` | Deterministic send failure hook is active. |
| 2102 | `stop_failure_injected` | Deterministic stop failure hook is active. |
| 2103 | `reset_failure_injected` | Deterministic reset failure hook is active. |
| 2104 | `read_failure_injected` | Deterministic read-state failure hook is active. |
| 4000 | `bad_request` | Request envelope or operation parameters are invalid. |
| 4001 | `unsupported_schema_version` | Schema version is missing or unsupported. |
| 4002 | `invalid_json` | Request line is not valid UTF-8 JSON. |
| 4003 | `unknown_operation` | Operation is not defined for the endpoint. |
| 4004 | `wrong_endpoint` | Admin op hit control endpoint or the reverse. |

## Control Operations

`connect`

- Requires `arm`.
- Marks the arm connected and returns state.

`initialize`

- Requires `arm`.
- Marks the arm initialized.
- By default also enables servo motion so the minimal backend path can call
  `send_servo_j` after `initialize`.
- Set `params.enable_servo` to `false` to test a servo-disabled state.

`read_state`

- Requires `arm`.
- Returns the current arm snapshot without mutating state.
- Returns `read_failure_injected` when the per-arm read-failure hook is active.

`send_servo_j`

- Requires initialized, servo-enabled, connected, non-faulted, valid state.
- Requires `params.q_target_deg` as a six-number joint target in degrees.
- Updates only the target; actual joints advance by deterministic ticks using
  interpolation capped by `simulator.max_joint_velocity_deg_s`.

`stop`

- Requires connected arm.
- Holds current actual joints when joint state is valid, otherwise holds the
  last accepted safe target. It does not synthesize zero joint targets.
- Disables servo motion and returns stopped state unless stop-failure injection
  is active.

`reset_fault`

- Requires connected arm.
- Clears a recoverable latched fault only when explicitly called and unless
  reset-failure injection is active.
- Unrecoverable faults remain latched and return `fault_latched`.
- Does not re-enable motion or initialization.

## Admin Operations

Admin operations are test hooks and belong only on the admin endpoint.

- `admin.tick`: `params.steps` advances deterministic simulator ticks and
  returns all arm states.
- `admin.inject`: `params.hook` is one of `read_failure`, `send_failure`,
  `stop_failure`, or `reset_failure`; `params.enabled` toggles it.
- `admin.reset_hooks`: clears one arm's hooks or all hooks when `arm` is absent.
- `admin.set_latency`: sets deterministic per-arm response latency in ms.
- `admin.disconnect` / `admin.reconnect`: toggles connection state.
- `admin.set_joint_validity`: toggles `has_valid_joint_state`.
- `admin.set_stale_state`: freezes reported state timestamp and motion while
  preserving connection state.
- `admin.set_fault`: latches a fault with `params.error_code`; optional
  `params.recoverable=false` makes reset reject the fault.
- `admin.set_tracking_bias`: adds a deterministic reported actual-joint bias.
- `admin.freeze_motion`: freezes actual-joint motion while robot time advances.

These hooks are for unit, contract, and smoke tests only. They must not be used
as evidence of Rainbow Virtual Simulator or real-robot readiness.
