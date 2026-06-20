# Network Protocol

`rb_servo_server` currently uses UDP JSON for the Python → C++ command channel and
a lower-rate UDP JSON state stream for observability/dataset/camera-recorder
consumers.

Default:

```yaml
network:
  command_bind: "udp://127.0.0.1:50010"
  state_pub_endpoint: "udp://127.0.0.1:50110"
  command_source_allowlist: ["127.0.0.1/32"]

command_source:
  enforce_lease: false
  lease_timeout_sec: 1.0
```

`network.state_pub_bind` is a deprecated compatibility alias for
`network.state_pub_endpoint`. Do not use both keys in the same config.

In real mode, exposed command or state publisher endpoints such as `udp://0.0.0.0:50010` or `tcp://0.0.0.0:50110` are rejected unless `RB_ALLOW_NETWORK_EXPOSURE=1` is set. Unknown bind formats fail closed in real mode.

The command server drops UDP packets whose source IP is not in
`network.command_source_allowlist`. Entries are IPv4 addresses or CIDR ranges;
the list must be non-empty. The Docker Compose mock config is explicitly
dev-only and allows loopback plus Docker bridge private addresses.

Hardware-free runs use `config/dual_mock.yaml` (MockBackend), which binds
`command_bind` and `state_pub_endpoint` to `127.0.0.1`. The `rbpodo` backend
(real robot plus the controller `pgmode` simulation flavor) is the controller
backend.

## State publisher

`StatePublisher` publishes snapshots from `DualArmServoLoop::latestSnapshot()`;
it does not read robot backends. The high-rate servo loop remains the sole owner
of backend `readState()` calls.

The current state stream is UDP JSON to `network.state_pub_endpoint` at 20 Hz. `StatePublisher` accepts hostname endpoints such as `udp://rb_gui:50110` for Docker Compose DNS, so static container IPs are not required. The
payload includes:

- `schema_version`, `tick`, `host_time_ns`, `loop_start_time_ns`, `loop_end_time_ns`
- `period_ms`, `jitter_ms`, `filter_dt_ms`, `command_seq`
- `command_source` with accepted packet source metadata, active lease owner,
  lease expiry, enforcement flag, and command lease verdict
- `left` / `right` objects with `mode`, `q_actual_deg`, `q_sent_deg`,
  `q_previous_sent_deg`, send timestamps/status/duration, connection/error fields
- `send_skew_us`, `safety_verdict`, `motion_state`, `fault_latched`,
  `latched_fault_reason`, `fault_reason`
- `logger_dropped_samples` / `logger_health`
- stand-frame mount transforms from config
- TCP pose fields when kinematics are configured and available; otherwise
  nullable/deferred TCP fields with `tcp_fields_deferred: true`.
  `tcp_base` and `tcp_stand` remain aliases for actual TCP FK from
  `q_actual_deg`. New consumers should prefer the explicit fields:
  `tcp_actual_base`, `tcp_actual_stand`, `tcp_ref_base`, and `tcp_ref_stand`.
  Actual TCP is measured from `q_actual_deg`; reference TCP is computed from
  controller reference joints (`q_target_deg` / rbpodo `jnt_ref`) when finite.
  `tcp_actual_valid` and `tcp_ref_valid` report pose availability.
- `cartesian_solve` telemetry, including IK errors, path tracking fields,
  optional server-side circle benchmark fields, and
  twist limit fields such as `twist_clamped`,
  `requested_twist_linear_norm_m_s`, and `applied_twist_linear_norm_m_s`

For rbpodo controller `pgmode` simulation (`operation_mode: simulation`), the
per-arm `controller_simulation_mode` object recommends
`recommended_tracking_pose: "tcp_ref_stand"` when reference FK is valid and
sets `physical_motion_expected: false`. This is reporting guidance only. Real
physical safety decisions must continue to use actual robot state and the
existing safety gates.

Consumers should join this stream with external RealSense logs by host/loop
timestamps. RealSense capture stays outside `rb_servo_server`.

## Important timing rule

The authoritative timestamp for timeout/staleness is the C++ receive time.

Python may send `host_time_ns` for debugging, but `CommandServer` overwrites the command's internal timestamp with `nowSteadyNs()` at packet receive time.

Every command packet must include an unsigned `seq`. Accepted sequence values
must strictly increase per `(source_id, session_id)` for the life of the server
process. Commands without source metadata use a legacy process-wide sequence
stream. Missing, repeated, or stale `seq` values are dropped before the command
buffer is updated.

## Command source metadata and lease

Command packets may include command-source metadata:

- `source_id`: optional string command source name, for example `rb_gui` or
  `policy_runner`.
- `session_id`: optional string process/session identifier. New client
  processes should generate a fresh session id at startup so `seq` can restart
  at 1 without colliding with an older session.
- `lease_token`: optional opaque string. The server publishes the active token
  in state JSON; when a client supplies a token under lease enforcement it must
  match the active lease.
- `source_priority`: optional integer reserved for future arbitration. It is
  parsed and published, but it does not override the initial single-owner lease.

`command_source.enforce_lease` defaults to `false` for backward compatibility.
When it is `false`, source metadata is still parsed, sequence numbers are
tracked per source/session, and `ArmMotion` or `AcquireLease` updates the
published active lease, but non-owner commands are not rejected.
Packets without source metadata use a legacy source identity so older
hardware-free tools can still acquire and refresh a lease after enforcement is
enabled in acceptance profiles.

When `command_source.enforce_lease` is `true`, normal motion commands require
the active lease. A source acquires the lease by sending `ArmMotion` or a
dedicated no-motion `AcquireLease` command:

```json
{
  "seq": 1,
  "mode": "AcquireLease",
  "source_id": "policy_runner",
  "session_id": "7c6d9d26c9bc4d1d9e5f3f12a0d8d2d1"
}
```

There is no separate UDP ACK channel. Clients that need confirmation should
listen to the state stream and wait until `command_source.active_source_id`,
`command_source.active_session_id`, and `command_source.active_lease_token`
reflect the acquired lease:

```json
{
  "schema_version": 1,
  "tick": 42,
  "command_seq": 1,
  "command_source": {
    "source_id": "policy_runner",
    "session_id": "7c6d9d26c9bc4d1d9e5f3f12a0d8d2d1",
    "lease_token": "lease-11f71fb04cb-1-policy_runner",
    "source_priority": null,
    "enforce_lease": true,
    "lease_timeout_sec": 1.0,
    "active": true,
    "expired": false,
    "active_source_id": "policy_runner",
    "active_session_id": "7c6d9d26c9bc4d1d9e5f3f12a0d8d2d1",
    "active_lease_token": "lease-11f71fb04cb-1-policy_runner",
    "acquired_time_ns": 123000000,
    "expires_time_ns": 1123000000,
    "command_requires_lease": false,
    "command_has_lease": true,
    "verdict": "Ok",
    "reason": ""
  }
}
```

After readback, a client may include the token in later packets. Tokens are
opaque; clients must copy them exactly:

```json
{
  "seq": 2,
  "mode": "JointVelocity",
  "timeout_sec": 0.2,
  "source_id": "policy_runner",
  "session_id": "7c6d9d26c9bc4d1d9e5f3f12a0d8d2d1",
  "lease_token": "lease-11f71fb04cb-1-policy_runner",
  "left": {
    "dq_target_deg_s": [1, 0, 0, 0, 0, 0]
  },
  "right": {
    "dq_target_deg_s": [-1, 0, 0, 0, 0, 0]
  }
}
```

If the same source/session supplies a non-empty token that does not match the
active token, the packet is rejected with
`command_source_lease_token_mismatch`. A source may omit `lease_token`; the
server still validates source/session ownership when lease enforcement is on.

Lease-required commands are `ArmMotion`, `DisarmMotion`, `JointTarget`,
`JointVelocity`, `TcpPoseTarget`, `TcpDeltaStand`, `TcpDeltaLocal`,
`TcpLinearMove`, `TcpCircleMove`, `TcpTwistStand`, `TcpTwistLocal`, and `ResetFault`.
`EmergencyStop` bypasses the lease so any accepted UDP source can stop motion.
`ResetFault` requires the active lease when enforcement is enabled; it is not
an operator bypass and it does not implicitly resume motion.

Emergency stop remains a safety override even when another source owns the
lease:

```json
{
  "seq": 1,
  "mode": "EmergencyStop",
  "timeout_sec": 0.2,
  "source_id": "rb_gui",
  "session_id": "operator-console-session"
}
```

The lease expires using the server monotonic clock after
`command_source.lease_timeout_sec` without an accepted lease-owning motion
command. A non-owner lease-required command is rejected with a
`command_source_lease_required`, `command_source_lease_conflict`, or
`command_source_lease_token_mismatch` reason in command parser diagnostics.

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

## Units and Pose Convention

All command arrays are exactly six finite JSON numbers. `NaN`, `Infinity`,
overflowed numeric values, strings, nulls, short arrays, and long arrays are
invalid and the packet is dropped.

- `q_target_deg`: joint position target, degrees.
- `dq_target_deg_s`: joint velocity target, degrees/second.
- `tcp_target_stand`: `[x, y, z, rx, ry, rz]` TCP final-pose target in the
  `stand` frame for `TcpPoseTarget`. `x,y,z` are meters. `rx,ry,rz` are
  radians as roll, pitch, yaw Euler angles; the C++ transform convention is
  `Rz(rz) * Ry(ry) * Rx(rx)`. Object form may also carry
  `quaternion_xyzw: [qx, qy, qz, qw]`, which is preferred over RPY.
- `tcp_delta_stand`: `[dx, dy, dz, drx, dry, drz]` incremental TCP delta in
  the `stand` frame for a one-shot low-level jog/debug command. Translational
  components are meters. Rotational components are radians as an so(3)
  rotation-vector increment. The delta transform is pre-multiplied before the
  current stand-frame TCP pose. Do not stream SpaceMouse teleop through delta
  commands.
- `tcp_delta_local`: `[dx, dy, dz, drx, dry, drz]` incremental TCP delta in
  the current TCP local frame for a one-shot low-level jog/debug command.
  Units match `tcp_delta_stand`. The delta transform is post-multiplied after
  the current stand-frame TCP pose. Do not stream SpaceMouse teleop through
  delta commands.
- `tcp_twist_stand` / `tcp_twist_local`: `[vx, vy, vz, wx, wy, wz]`
  streaming Cartesian velocity command. Translational components are
  meters/second (`m/s`) and rotational components are radians/second
  (`rad/s`). `TcpTwistStand` expresses both vectors in the `stand` frame.
  `TcpTwistLocal` expresses both vectors in the current TCP local frame.

The accepted Cartesian command modes are `TcpPoseTarget`, `TcpDeltaStand`,
`TcpDeltaLocal`, `TcpLinearMove`, `TcpCircleMove`, `TcpTwistStand`, and
`TcpTwistLocal`. Runtime Cartesian verdicts include `Ok`,
`CartesianUnavailable`, `InvalidCommand`, and `IkFailed`. `TcpCircleMove` is a
diagnostic benchmark primitive and is rejected unless
`cartesian_control.enable_benchmark_primitives: true` is set. `TcpTwist*`, `TcpLinearMove`, and `TcpCircleMove` are bounded by
server-side `cartesian_control` limits before Jacobian velocity solving.
`TcpLinearMove` and `TcpCircleMove` path feedback use
`cartesian_control.path_kp_pos` for position error and
`cartesian_control.path_kp_ori` for orientation error. Constant-orientation
linear moves reject target orientation mismatch greater than
`cartesian_control.linear_move.constant_orientation_tolerance_rad`; hardware-free
configs set this to `0.005` rad. Real Cartesian motion remains closed unless
real mode is explicitly enabled, Cartesian control is configured for real, and
`RB_ALLOW_REAL_CARTESIAN=1` is set.

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

## TCP Cartesian command schema

TCP Cartesian commands use the same UDP JSON packet envelope as joint commands:

- `schema_version`: optional unsigned integer. If present, it must be `1`.
- `seq`: required unsigned integer, strictly increasing per server process.
- `mode`: top-level default command mode for both arms.
- `host_time_ns`: optional sender timestamp for debugging. The server uses its
  receive timestamp for timeout enforcement.
- `timeout_sec`: optional finite positive command timeout in seconds.
- `left` / `right`: per-arm command objects. A per-arm `mode` overrides the
  top-level `mode`, which lets one arm receive a TCP command while the other
  remains `Hold`.

`TcpPoseTarget` is a point-to-point final TCP pose command. It requires each
Cartesian arm object to include `tcp_target_stand`; the Cartesian path is not
guaranteed:

```json
{
  "schema_version": 1,
  "seq": 123,
  "mode": "TcpPoseTarget",
  "host_time_ns": 123456789,
  "timeout_sec": 0.2,
  "left": {
    "tcp_target_stand": [0.3, 0.1, 0.5, 0, 3.14, 0]
  },
  "right": {
    "tcp_target_stand": [0.3, -0.1, 0.5, 0, 3.14, 0]
  }
}
```

`TcpDeltaStand` requires `tcp_delta_stand`. The delta frame is `stand`. This is
a one-shot jog/debug primitive, not a continuous velocity command:

```json
{
  "schema_version": 1,
  "seq": 124,
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

`TcpDeltaLocal` requires `tcp_delta_local`. The delta frame is the current TCP
local frame. This is a one-shot jog/debug primitive, not a continuous velocity
command:

```json
{
  "schema_version": 1,
  "seq": 125,
  "mode": "TcpDeltaLocal",
  "timeout_sec": 0.2,
  "left": {
    "tcp_delta_local": [0, 0, 0.001, 0, 0, 0]
  },
  "right": {
    "tcp_delta_local": [0, 0, -0.001, 0, 0, 0]
  }
}
```

`TcpTwistLocal` and `TcpTwistStand` are streaming Cartesian velocity commands.
They require their matching twist array with `vx,vy,vz` in `m/s` and
`wx,wy,wz` in `rad/s`. `TcpTwistLocal` is intended for SpaceMouse-style
continuous teleop; `TcpTwistStand` is the stand-frame API variant. If a
requested twist exceeds `cartesian_control.max_twist_linear_m_s` or
`max_twist_angular_rad_s`, the server either clamps it or rejects it according
to `cartesian_control.exceed_limit_policy`. Requested angular velocity with
norm less than or equal to `cartesian_control.twist_angular_deadband_rad_s`
engages or maintains server-side orientation hold:

```json
{
  "schema_version": 1,
  "seq": 126,
  "mode": "TcpTwistLocal",
  "timeout_sec": 0.2,
  "left": {
    "tcp_twist_local": [0.01, 0, 0, 0, 0, 0]
  }
}
```

`TcpLinearMove` is a MoveL-like Cartesian straight-line path
primitive; it is not real-motion-ready (real mode blocked). The target pose is
in the `stand` frame. `duration_sec` is seconds,
`linear_speed_m_s` is meters/second (`m/s`), and `angular_speed_rad_s` is
radians/second (`rad/s`). Explicit `duration_sec` takes precedence over
speed-based timing; server limits can extend or reject the move if the implied
linear or angular speed is excessive. `orientation_mode` is `constant` or
`slerp`.

```json
{
  "schema_version": 1,
  "seq": 127,
  "mode": "Hold",
  "timeout_sec": 0.2,
  "left": {
    "mode": "TcpLinearMove",
    "target_tcp_stand": {
      "x": 0.35,
      "y": 0.10,
      "z": 0.45,
      "rx": 0.0,
      "ry": 0.0,
      "rz": 0.0,
      "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]
    },
    "duration_sec": 2.0,
    "linear_speed_m_s": 0.03,
    "angular_speed_rad_s": 0.2,
    "orientation_mode": "constant"
  },
  "right": {}
}
```

`TcpCircleMove` is an optional benchmark-only primitive (real mode blocked). It starts
from the current TCP pose without an initial jump, chooses the circle center so
the current pose is on the circle at phase zero, generates reference position
and velocity inside the servo loop, and holds the initial orientation. It is
for isolating server/controller behavior from Python UDP command streaming
jitter; it is not a real robot feature and remains rejected in real mode even
when real Cartesian environment gates are set.

It requires `cartesian_control.enable_benchmark_primitives: true`,
`cartesian_control.circle_move.allow_in_simulation: true`, and
`cartesian_control.circle_move.allow_in_real: false`. Supported command
options are currently `frame: "stand"`, `center_mode: "start_on_circle"`,
and `orientation_mode: "constant"`. Optional `phase_advance_sec` advances the
server-side circle reference and must be finite, non-negative, and no greater
than `0.25 * period_sec`.

```json
{
  "schema_version": 1,
  "seq": 128,
  "mode": "Hold",
  "timeout_sec": 4.2,
  "left": {
    "mode": "TcpCircleMove",
    "plane": "xy",
    "diameter_m": 0.15,
    "period_sec": 4.0,
    "repeat": 1,
    "phase_advance_sec": 0.005,
    "center_mode": "start_on_circle",
    "orientation_mode": "constant",
    "frame": "stand"
  },
  "right": {
    "mode": "Hold"
  }
}
```

State telemetry for active or recently completed circle moves includes
`circle_active`, `circle_phase`, `circle_repeat_index`, `circle_radius_m`,
`circle_period_sec`, `circle_position_error_m`,
`circle_orientation_error_rad`, and `circle_done` under `cartesian_solve`.
The primitive cancels through the same command/fault/stale-state paths as the
other Cartesian velocity-servo modes, including lease loss.

If a top-level Cartesian `mode` applies to both arms, both arm objects must
carry the matching payload. To command only one arm, set top-level `mode` to
`Hold` and put the Cartesian `mode` plus payload inside the selected arm object.

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

For hardware-free smoke evidence, capture state/log artifacts that show the
reset did not silently resume motion. A valid reset artifact has a fresh valid
state, `ConnectedHold`, no new sent motion target until a later `ArmMotion`, and
an unchanged command-source allowlist.

## Stop and fault evidence

These stop and fault behaviors are validated through mock-mode tests and the
rbpodo controller `pgmode` simulation. The checks cover invalid state,
disconnects, tracking bias/frozen motion, and send/stop/reset failures for
software-only regression.

Acceptable evidence includes:

- Stop: state and CSV rows show the last safe target is held after stop.
- Send failure: `left_send_ok` or `right_send_ok` becomes false and the server
  holds or latches according to the active safety policy.
- Invalid state or disconnect: the state stream/log carries the fault truthfully
  and the server does not synthesize a zero-joint target.
- Reset failure: the latch remains active and motion commands stay blocked.

These checks are not Rainbow external simulator/OVA, `rbpodo`, real robot,
realtime, or production-network acceptance.

## Missing payload safety

Motion modes require their payloads:

- `JointTarget` requires `q_target_deg` with 6 values.
- `JointVelocity` requires `dq_target_deg_s` with 6 values.
- `TcpPoseTarget` requires `tcp_target_stand` with 6 values.
- `TcpDeltaStand` requires `tcp_delta_stand` with 6 values.
- `TcpDeltaLocal` requires `tcp_delta_local` with 6 values.
- `TcpLinearMove` requires `target_tcp_stand` and either `duration_sec` or
  `linear_speed_m_s`; `orientation_mode` is `constant` or `slerp`.
- `TcpCircleMove` requires `plane`, positive `diameter_m`, positive
  `period_sec`, and positive integer `repeat`; optional `phase_advance_sec`
  must be finite, non-negative, and no greater than `0.25 * period_sec`; only
  `frame: "stand"`, `center_mode: "start_on_circle"`, and
  `orientation_mode: "constant"` are currently supported at runtime.

If the required payload is absent or malformed, the packet is dropped and the command buffer remains unchanged.
