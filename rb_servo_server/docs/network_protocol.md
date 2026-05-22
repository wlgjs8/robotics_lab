# Network Protocol

`rb_servo_server` currently uses UDP JSON for the Python → C++ command channel and
a lower-rate UDP JSON state stream for observability/dataset/camera-recorder
consumers.

Default:

```yaml
network:
  command_bind: "udp://127.0.0.1:50010"
  state_pub_bind: "udp://127.0.0.1:50110"
  command_source_allowlist: ["127.0.0.1/32"]
```

In real mode, exposed command or state publisher binds such as `udp://0.0.0.0:50010` or `tcp://0.0.0.0:50110` are rejected unless `RB_ALLOW_NETWORK_EXPOSURE=1` is set. Unknown bind formats fail closed in real mode.

The command server drops UDP packets whose source IP is not in
`network.command_source_allowlist`. Entries are IPv4 addresses or CIDR ranges;
the list must be non-empty. The Docker Compose mock config is explicitly
dev-only and allows loopback plus Docker bridge private addresses.

The hardware-free simulator config, `config/dual_rb_simulator.yaml`, is
stricter: it binds `command_bind` and `state_pub_bind` to `127.0.0.1` and points
both `rbsim_local` backends at `tcp://127.0.0.1:50200`. That shared endpoint is
one dual-arm `rb_simulator` process; left/right are selected by the backend
request `arm` field. The compose `sim` profile keeps that TCP endpoint
loopback-only by sharing the `rb_simulator` service network namespace; it does
not require host networking or exposed simulator ports.

## State publisher

`StatePublisher` publishes snapshots from `DualArmServoLoop::latestSnapshot()`;
it does not read robot backends. The high-rate servo loop remains the sole owner
of backend `readState()` calls.

The current state stream is UDP JSON to `network.state_pub_bind` at 20 Hz. `StatePublisher` accepts hostname endpoints such as `udp://rb_servo_gui:50110` for Docker Compose DNS, so static container IPs are not required. The
payload includes:

- `schema_version`, `tick`, `host_time_ns`, `loop_start_time_ns`, `loop_end_time_ns`
- `period_ms`, `jitter_ms`, `filter_dt_ms`, `command_seq`
- `left` / `right` objects with `mode`, `q_actual_deg`, `q_sent_deg`,
  `q_previous_sent_deg`, send timestamps/status/duration, connection/error fields
- `send_skew_us`, `safety_verdict`, `motion_state`, `fault_latched`,
  `latched_fault_reason`, `fault_reason`
- `logger_dropped_samples` / `logger_health`
- stand-frame mount transforms from config
- nullable/deferred TCP fields (`tcp_fields_deferred: true`) until Cartesian FK/IK
  is implemented

Consumers should join this stream with external RealSense logs by host/loop
timestamps. RealSense capture stays outside `rb_servo_server`.

## Important timing rule

The authoritative timestamp for timeout/staleness is the C++ receive time.

Python may send `host_time_ns` for debugging, but `CommandServer` overwrites the command's internal timestamp with `nowSteadyNs()` at packet receive time.

Every command packet must include an unsigned `seq`. Accepted sequence values
must strictly increase for the life of the server process; missing, repeated,
or stale `seq` values are dropped before the command buffer is updated.

`coupled_timeout` is retained for protocol compatibility, but v3 treats dual-arm commands as coupled: the earliest per-arm timeout makes both arms Hold. Future per-arm command streams should use separate command channels or a binary protocol with explicit per-arm timestamps.

## Minimal Hold command

```json
{
  "seq": 1,
  "mode": "Hold",
  "timeout_sec": 0.2,
  "left": {},
  "right": {}
}
```

## Joint target command

The server must first be armed:

```json
{"seq": 1, "mode": "ArmMotion"}
```

```json
{
  "seq": 2,
  "mode": "JointTarget",
  "timeout_sec": 0.2,
  "coupled_timeout": true,
  "left": {
    "q_target_deg": [0, -30, 80, 0, 60, 0]
  },
  "right": {
    "q_target_deg": [0, -30, 80, 0, 60, 0]
  }
}
```

Top-level `mode` applies to both arms unless an arm object has its own `mode`.

## Joint velocity command

```json
{
  "seq": 3,
  "mode": "JointVelocity",
  "timeout_sec": 0.2,
  "left": {
    "dq_target_deg_s": [1, 0, 0, 0, 0, 0]
  },
  "right": {
    "dq_target_deg_s": [-1, 0, 0, 0, 0, 0]
  }
}
```

## Future Cartesian command

These fields are parsed into `ArmCommand`, but Cartesian IK is intentionally deferred.

```json
{
  "seq": 4,
  "mode": "TcpDeltaStand",
  "timeout_sec": 0.2,
  "left": {
    "tcp_delta_stand": [0.001, 0, 0, 0, 0, 0]
  },
  "right": {
    "tcp_delta_stand": [-0.001, 0, 0, 0, 0, 0]
  }
}
```

## Future force-control fields

Force-control fields are parsed, but not connected to the active joint-only path.

```json
{
  "seq": 5,
  "mode": "TcpPoseTarget",
  "timeout_sec": 0.2,
  "left": {
    "tcp_target_stand": [0.3, 0.1, 0.4, 0, 0, 0],
    "force_control": {
      "mode": "Admittance",
      "target_wrench": [0, 0, -5, 0, 0, 0],
      "enabled_axis": {"z": true},
      "max_pos_offset_m": 0.01,
      "max_pos_step_m": 0.001
    }
  },
  "right": {
    "tcp_target_stand": [0.3, -0.1, 0.4, 0, 0, 0]
  }
}
```

## Future binary protocol

UDP JSON is fine for 10–30 Hz policy commands. If command rate, action chunks, or image/state payloads grow, replace this with one of:

- shared-memory ring buffer
- ZeroMQ
- msgpack
- flatbuffers
- protobuf

## ResetFault command

A latched fault can be cleared with:

```json
{"seq": 10, "mode": "ResetFault"}
```

After reset succeeds, the server re-baselines previous targets to a freshly read valid current actual q.
If backend reset fails or fresh state is invalid, the latch remains active.
Successful reset stays in `ConnectedHold`; send `ArmMotion` again before motion targets.

For rb_simulator smoke evidence, capture state/log artifacts that show the
reset did not silently resume motion. A valid reset artifact has a fresh valid
state, `ConnectedHold`, no new sent motion target until a later `ArmMotion`, and
an unchanged command-source allowlist.

## Stop and fault evidence

The rb_simulator admin endpoint can inject invalid state, disconnects, tracking
bias/frozen motion, and send/stop/reset failures for software-only regression
checks. Use those hooks only through `rb_simulator` tests or the local smoke
runner artifacts.

Acceptable simulator evidence includes:

- Stop: state and CSV rows show the last safe target is held after stop.
- Send failure: `left_send_ok` or `right_send_ok` becomes false and the server
  holds or latches according to the active safety policy.
- Invalid state or disconnect: the state stream/log carries the fault truthfully
  and the server does not synthesize a zero-joint target.
- Reset failure: the latch remains active and motion commands stay blocked.

These checks are not Rainbow rbsim/OVA, `rbpodo`, real robot, realtime, or
production-network acceptance.

## Missing payload safety

Motion modes require their payloads:

- `JointTarget` requires `q_target_deg` with 6 values.
- `JointVelocity` requires `dq_target_deg_s` with 6 values.
- `TcpPoseTarget` requires `tcp_target_stand` with 6 values.
- `TcpDeltaStand` requires `tcp_delta_stand` with 6 values.
- `TcpDeltaLocal` requires `tcp_delta_local` with 6 values.

If the required payload is absent or malformed, the packet is dropped and the command buffer remains unchanged.
