# rb_servo_server Network Protocol

Commands are UDP JSON packets sent to `network.command_bind`. State is UDP JSON
fanout from `network.state_pub_endpoints`.

## Common Fields

Every command packet uses:

```json
{
  "schema_version": 1,
  "seq": 1,
  "mode": "Hold",
  "timeout_sec": 0.2,
  "source_id": "policy_runner",
  "session_id": "...",
  "lease_token": "..."
}
```

`lease_token` is optional and required only when command-source lease enforcement
is active. Per-arm packets live under `left` and `right`.

## Motion Primitives

The public motion primitive set is:

- `JointTarget`
- `TcpPoseTarget`
- `TcpLinearMove`

### JointTarget

Direct joint target:

```json
{
  "mode": "JointTarget",
  "left": {
    "mode": "JointTarget",
    "q_target_deg": [0, -30, 80, 0, 60, 0]
  },
  "right": {
    "mode": "Hold"
  }
}
```

Collision-free init profile:

```json
{
  "mode": "JointTarget",
  "left": {
    "mode": "JointTarget",
    "q_target_deg": [0, -30, 80, 0, 60, 0],
    "joint_target_profile": "init_motion"
  },
  "right": {
    "mode": "JointTarget",
    "q_target_deg": [0, -30, 80, 0, 60, 0],
    "joint_target_profile": "init_motion"
  }
}
```

If `joint_target_profile` is omitted, the profile is direct.

### TcpPoseTarget

```json
{
  "mode": "TcpPoseTarget",
  "left": {
    "mode": "TcpPoseTarget",
    "tcp_target_stand": {
      "x": 0.35,
      "y": 0.10,
      "z": 0.45,
      "rx": 0.0,
      "ry": 0.0,
      "rz": 0.0,
      "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]
    }
  },
  "right": {
    "mode": "Hold"
  }
}
```

The quaternion is authoritative when present. A six-element pose array is also
accepted for non-quaternion targets.

### TcpLinearMove

```json
{
  "mode": "TcpLinearMove",
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
    "orientation_mode": "slerp"
  },
  "right": {
    "mode": "Hold"
  }
}
```

`target_tcp_stand` and `tcp_target_stand` are accepted as the target-pose key for
linear moves. `orientation_mode` is `constant` or `slerp`.

## Lifecycle And Safety Modes

Supported non-motion command modes:

- `Hold`
- `ArmMotion`
- `DisarmMotion`
- `EmergencyStop`
- `ResetFault`
- `SetSafetyFloorZ`
- `SetSafetyFloorEnabled`
- `SetSafetyRoiBounds`
- `SetUserSafetyFloorPlane`
- `Freedrive`
- `AcquireLease`
- `ReleaseLease`

`SetSafetyFloorZ` is leaseless and carries top-level `floor_z_m`.
`SetSafetyFloorEnabled` carries top-level `floor_enabled`.
`SetSafetyRoiBounds` and `SetUserSafetyFloorPlane` carry their documented safety
payloads and are bounded by server config.

## State Highlights

State packets use schema `robotics_lab.servo_state.v1` and publish per-arm joint
state, TCP actual/reference poses, Cartesian solve telemetry, safety verdicts,
lease state, floor/ROI/self-collision status, and backend timing/fault context.
