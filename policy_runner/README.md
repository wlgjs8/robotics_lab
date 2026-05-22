# policy_runner

`policy_runner` is the Python action-source layer for `robotics_lab`. It sends
UDP JSON commands to `rb_servo_server` and consumes the UDP JSON state stream.

Supported P1 action sources:

- `hold`: receive state and keep policy output as no-op by default.
- `joint_sine`: small simulation-only joint target motion.
- `joint_velocity`: fixed simulation-only joint velocity command.

SpaceMouse support is intentionally left for P1-E. FK/IK is not available in P1,
so there are no Cartesian policy actions.

## Safety

Motion commands are blocked when:

- the state stream is stale or absent
- `fault_latched` is true
- `motion_state` is `FaultLatched` or `EmergencyLatched`
- joint state is invalid and `require_valid_joint_state` is true
- configured mode is `real` and `allow_real_motion` is false
- an action source requires camera or kinematics inputs that are unavailable

`joint_sine` and `joint_velocity` are simulation-only by default. In real mode
they do not send motion unless the config explicitly sets
`allow_real_motion: true`.

Only one command source should be active at a time. Do not run GUI teleop and
`policy_runner` teleop against the same `rb_servo_server` command port
simultaneously.

## Example

```bash
python3 -m policy_runner --config policy_runner/config/simulator_hold.yaml
```

The default command endpoint is `udp://127.0.0.1:50010`. The state subscriber
bind must match the `rb_servo_server` state publisher destination.

## Command Packets

All packets include `seq`, `mode`, and `timeout_sec`. Lifecycle packets use the
top-level mode:

```json
{"seq": 1, "mode": "ArmMotion", "timeout_sec": 0.2}
```

Joint actions use per-arm modes so either arm can hold independently:

```json
{
  "seq": 2,
  "mode": "Hold",
  "timeout_sec": 0.2,
  "left": {"mode": "JointTarget", "q_target_deg": [0, -30, 80, 0, 60, 0]},
  "right": {"mode": "Hold"}
}
```

Supported P1-D modes are `Hold`, `ArmMotion`, `DisarmMotion`,
`EmergencyStop`, `ResetFault`, `JointTarget`, and `JointVelocity`.
