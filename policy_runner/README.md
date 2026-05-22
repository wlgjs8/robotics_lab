# policy_runner

`policy_runner` is the Python action-source layer for `robotics_lab`. It sends
UDP JSON commands to `rb_servo_server` and consumes the UDP JSON state stream.

Supported P1 action sources:

- `hold`: receive state and keep policy output as no-op by default.
- `joint_sine`: small simulation-only joint target motion.
- `joint_velocity`: fixed simulation-only joint velocity command.
- `spacemouse_joint_velocity`: SpaceMouse axes mapped directly to joint
  velocities.

SpaceMouse support lives here, not in `rb_gui`. FK/IK is not available in P1, so
there is no Cartesian SpaceMouse mode.

## Safety

Motion commands are blocked when:

- the state stream is stale or absent
- `fault_latched` is true
- `motion_state` is `FaultLatched` or `EmergencyLatched`
- joint state is invalid and `require_valid_joint_state` is true
- configured mode is `real` and `allow_real_motion` is false

`joint_sine`, `joint_velocity`, and `spacemouse_joint_velocity` are
simulation-only by default. In real mode they do not send motion unless the
config explicitly sets `allow_real_motion: true`.

Only one command source should be active at a time. Do not run GUI teleop and
`policy_runner` teleop against the same `rb_servo_server` command port
simultaneously.

## SpaceMouse Joint Mapping

This is a joint velocity mapping, not Cartesian control:

```text
tx -> J1 velocity
ty -> J2 velocity
tz -> J3 velocity
rx -> J4 velocity
ry -> J5 velocity
rz -> J6 velocity
```

Button 0 is the default deadman switch. If the deadman is released, no motion
command is sent.

The real HID dependency is optional:

```bash
python3 -m pip install -e 'policy_runner[spacemouse]'
```

Tests use `FakeSpaceMouseReader` and do not require HID hardware.

## Example

```bash
python3 -m policy_runner --config policy_runner/config/simulator_hold.yaml
```

The default command endpoint is `udp://127.0.0.1:50010`. The state subscriber
bind must match the `rb_servo_server` state publisher destination.
