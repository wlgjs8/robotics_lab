# rb_simulator Protocol v1

Protocol v1 is a JSON Lines contract for the hardware-free simulator backend.
It is not the Rainbow Robotics simulator protocol, does not speak `rbpodo`, and
binds to loopback by default.

The schema version remains `rbsim.v1` for compatibility with existing clients.
Public configuration should still use `backend_type: simulator`.

## Transport

- Framing: one UTF-8 JSON object per TCP line, terminated by `\n`.
- Left host control endpoint: `tcp://127.0.0.1:50200`.
- Left host admin endpoint: `tcp://127.0.0.1:50201`.
- Right host control endpoint: `tcp://127.0.0.1:50210`.
- Right host admin endpoint: `tcp://127.0.0.1:50211`.
- Default schema version: `rbsim.v1`.

Non-loopback binds are rejected unless
`RB_SIMULATOR_ALLOW_NON_LOOPBACK=1` is set.

## Topology

One process owns exactly one arm. The configured arm is loaded from
`simulator.arm` in the process config.

```text
left process  owns left  and rejects right
right process owns right and rejects left
```

The `arm` request field is retained for compatibility. It is required for
control operations. Admin operations may omit it, in which case the process arm
is used. Any mismatched explicit arm is rejected fail-closed.

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
- `arm`: required for control operations; optional for admin operations.
- `params`: optional object for operation-specific arguments.

## Response Envelope

Successful response:

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

Failure response:

```json
{
  "schema_version": "rbsim.v1",
  "request_id": "client-unique-id",
  "ok": false,
  "arm": "right",
  "server_time_ns": 123456789,
  "error": {
    "name": "wrong_arm",
    "message": "wrong arm: simulator owns left, got right",
    "code": 1005
  }
}
```

## Error Names

| Code | Name | Meaning |
| --- | --- | --- |
| 1001 | `disconnected` | Arm is disconnected. |
| 1002 | `not_initialized` | Operation requires initialization. |
| 1003 | `servo_disabled` | Servo target requires servo enabled. |
| 1004 | `unknown_arm` | Arm key is not configured. |
| 1005 | `wrong_arm` | Request arm does not match the process arm. |
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

- Requires matching `arm`.
- Marks the arm connected and returns state.

`initialize`

- Requires matching `arm`.
- Marks the arm initialized.
- By default also enables servo motion so the minimal backend path can call
  `send_servo_j` after `initialize`.
- Set `params.enable_servo` to `false` to test a servo-disabled state.

`read_state`

- Requires matching `arm`.
- Returns the current arm snapshot without mutating state.
- Returns `read_failure_injected` when the read-failure hook is active.

`send_servo_j`

- Requires matching `arm`.
- Requires initialized, servo-enabled, connected, non-faulted, valid state.
- Requires `params.q_target_deg` as a six-number joint target in degrees.
- Updates only the target; actual joints advance by deterministic ticks.

`stop`

- Requires matching `arm` and connected state.
- Holds current actual joints when joint state is valid, otherwise holds the
  last accepted safe target.
- Disables servo motion unless stop-failure injection is active.

`reset_fault`

- Requires matching `arm` and connected state.
- Clears recoverable latched faults unless reset-failure injection is active.
- Does not re-enable motion or initialization.

## Admin Operations

Admin operations are test hooks and belong only on the admin endpoint.

- `admin.tick`: advances deterministic simulator ticks and returns one state
  under `states.left` or `states.right`.
- `admin.inject`: toggles `read_failure`, `send_failure`, `stop_failure`, or
  `reset_failure`.
- `admin.reset_hooks`: clears hooks for the process arm.
- `admin.set_latency`: sets deterministic response latency in ms.
- `admin.disconnect` / `admin.reconnect`: toggles connection state.
- `admin.set_joint_validity`: toggles `has_valid_joint_state`.
- `admin.set_stale_state`: freezes reported state timestamp and motion.
- `admin.set_fault`: latches a fault with `params.error_code`.
- `admin.set_tracking_bias`: adds reported actual-joint bias.
- `admin.freeze_motion`: freezes actual-joint motion while robot time advances.

These hooks are for unit, contract, and smoke tests only. They are not evidence
of Rainbow simulator or real robot readiness.
