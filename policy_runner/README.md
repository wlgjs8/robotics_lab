# policy_runner

`policy_runner` is the Python action-source layer for `robotics_lab`. It sends
UDP JSON commands to `rb_servo_server` and consumes the UDP JSON state stream.

Supported P1 action sources:

- `hold`: receive state and keep policy output as no-op by default.
- `joint_sine`: small simulation-only joint target motion.
- `joint_velocity`: fixed simulation-only joint velocity command.
- `spacemouse_joint_velocity`: SpaceMouse six-axis input mapped directly to
  joint velocity commands.
- `tcp_delta`: small simulation-only `TcpDeltaStand` command.
- `spacemouse_cartesian`: simulation-only SpaceMouse input mapped to
  `TcpTwistLocal`.
- `dual_spacemouse_cartesian`: simulation-only two-SpaceMouse input mapped to
  per-arm `TcpTwistLocal` commands.

P1 joint actions remain joint-only. P2 adds geometry awareness for future
Cartesian and camera policies. P3 enables Cartesian action sources for
simulation only.

## Safety

Motion commands are blocked when:

- the state stream is stale or absent
- `fault_latched` is true
- `motion_state` is `FaultLatched` or `EmergencyLatched`
- joint state is invalid and `require_valid_joint_state` is true
- an action source declares `requires_geometry` and the calibration registry is
  missing, invalid, lacks robot mount geometry, or is not policy-ready for the
  configured mode
- an action source declares `requires_camera_geometry` and measured camera
  intrinsics/extrinsics are unavailable
- an action source declares `requires_valid_tcp_pose` and the state stream does
  not report valid TCP pose for both arms
- a Cartesian source does not observe `observed_mode: simulation`
- a Cartesian source sees a non-simulator `observed_backend`/`backend_type`
- configured mode is `real` and `allow_real_motion` is false
- an action source requires camera or kinematics inputs that are unavailable

`joint_sine` and `joint_velocity` are simulation-only by default. In real mode
they do not send motion unless the config explicitly sets
`allow_real_motion: true`.

`spacemouse_joint_velocity` also remains behind the same stale-state,
fault-state, and real-motion gates. Button 0 is the default deadman switch:
when it is released, the action source emits no motion command.

Joint-only sources do not require the geometry registry. Future Cartesian or
camera policy sources must declare geometry requirements before emitting
commands. The default geometry config is:

```yaml
geometry:
  path: "calibration/active_calibration.yaml"
safety:
  allow_configured_estimate_geometry_in_simulation: true
  allow_configured_estimate_geometry_in_real: false
```

The current active calibration is a configured estimate. It is acceptable for
simulation geometry-aware policy tests when the simulation toggle remains true,
but it is blocked for real geometry-dependent policy by default because
`geometry_valid_for_real_policy` is false.

Cartesian sources are stricter than joint sources. They require fresh state, no
fault latch, valid joint state, valid TCP pose for both arms, simulation as the
observed mode, and simulator backend when the backend is reported. Real
Cartesian commands remain blocked even when `allow_real_motion: true`; a future
separate `allow_real_cartesian` implementation must be added before real
Cartesian motion can be opened.

Only one command source should be active at a time. Do not run GUI teleop and
`policy_runner` teleop against the same `rb_servo_server` command port
simultaneously.

## Example

```bash
python3 -m policy_runner --config policy_runner/config/simulator_hold.yaml
```

The default command endpoint is `udp://127.0.0.1:50010`. The state subscriber
bind must match the `rb_servo_server` state publisher destination.

Simulation-only example configs:

- `policy_runner/config/simulator_hold.yaml`: no-op runner for state/command
  wiring checks.
- `policy_runner/config/simulator_spacemouse_joint_velocity.yaml`:
  SpaceMouse joint velocity teleop.
- `policy_runner/config/simulator_tcp_delta.yaml`: scripted stand-frame
  `TcpDeltaStand`.
- `policy_runner/config/simulator_spacemouse_cartesian.yaml`: SpaceMouse
  local-frame `TcpTwistLocal`.
- `policy_runner/config/simulator_tcp_twist_local.yaml`: same SpaceMouse
  `TcpTwistLocal` path with velocity-unit field names called out.
- `policy_runner/config/simulator_dual_spacemouse_cartesian.yaml`: two
  SpaceMouse devices mapped to left/right simulator TCP twists.

These examples use loopback simulator endpoints and do not enable real motion
or real Cartesian motion.

## Imitation Data Collection

The Docker simulator stack can run a passive policy recorder with `make sim-up`.
The recorder receives the `rb_servo_server` state stream and writes JSONL
episodes under `policy_runner/episodes` without sending commands.

SpaceMouse teleop collection is an explicit mode because it sends simulator
motion commands:

```bash
make sim-teleop-up
```

This uses `policy_runner/config/simulator_dual_spacemouse_cartesian_compose.yaml`
and records both `robot_state.jsonl` and `actions.jsonl` in the same episode.
The default HID paths are `/dev/hidraw1` for the left SpaceMouse and
`/dev/hidraw6` for the right SpaceMouse.

Train and run the V1 behavior-cloning baseline with:

```bash
make policy-train
make sim-infer-up
```

The V1 dataset is robot-state plus policy-command JSONL only. Camera/image
episodes remain a later merge step using camera timestamps and robot state
timestamps.

## SpaceMouse Joint Velocity

SpaceMouse support belongs in `policy_runner`, not `rb_gui`. P1 only maps the
device axes to joint velocities:

```text
tx -> J1 velocity
ty -> J2 velocity
tz -> J3 velocity
rx -> J4 velocity
ry -> J5 velocity
rz -> J6 velocity
```

This is not Cartesian control. FK/IK is not required for this action source.

Example config fields:

```yaml
action_source: spacemouse_joint_velocity
command_rate_hz: 30
spacemouse:
  selected_arm: left
  max_joint_velocity_deg_s: [5, 5, 5, 8, 8, 10]
  deadband: 0.08
  smoothing_alpha: 0.2
  require_deadman: true
  deadman_button: 0
```

Keep `command_rate_hz` in the 30-60 Hz range. The servo loop runs faster; the
policy runner should not attempt to publish at servo-loop frequency.

Hardware-free tests use `FakeSpaceMouseReader`. Real HID support is optional:

```bash
python3 -m pip install -e policy_runner[spacemouse]
```

Probe attached devices without starting `rb_servo_server`:

```bash
python3 policy_runner/scripts/probe_spacemouse.py --list
python3 policy_runner/scripts/probe_spacemouse.py
python3 policy_runner/scripts/probe_spacemouse.py \
  --left-path /dev/hidraw1 \
  --right-path /dev/hidraw6
```

The probe defaults to `/dev/hidraw1` for the left SpaceMouse,
`/dev/hidraw6` for the right SpaceMouse, and a 30 second duration.

## Cartesian TCP Commands

`tcp_delta` emits a small one-shot scripted stand-frame TCP delta using the
server `TcpDeltaStand` mode. `TcpDelta*` commands use meters/radians and are
debug jog primitives, not velocity streams. `spacemouse_cartesian` maps
SpaceMouse axes to a simulation-only local TCP velocity command using
`TcpTwistLocal`:

```text
tx, ty, tz -> TCP local vx, vy, vz in m/s
rx, ry, rz -> TCP local wx, wy, wz in rad/s
```

Example config fields:

```yaml
action_source: spacemouse_cartesian
runtime:
  startup_timeout_sec: 5.0
command_rate_hz: 30
spacemouse_cartesian:
  selected_arm: left
  frame: local
  max_linear_velocity_m_s: 0.03
  max_angular_velocity_rad_s: 0.2
  deadband: 0.08
  require_deadman: true
  deadman_button: 0
```

Button 0 is the default deadman switch. When it is released, the source emits
no command. The Cartesian SpaceMouse source always requires a deadman switch.
Linear and angular twist components are clamped as velocities. Deprecated
`max_linear_step_m` and `max_angular_step_rad` aliases still parse with a
warning for migration only.

`dual_spacemouse_cartesian` keeps one `policy_runner` command source and opens
two SpaceMouse devices. The left and right samples are aggregated into one UDP
command packet so the two arms are updated together:

```yaml
action_source: dual_spacemouse_cartesian
command_rate_hz: 30
spacemouse_cartesian_dual:
  frame: local
  max_linear_velocity_m_s: 0.03
  max_angular_velocity_rad_s: 0.2
  deadband: 0.08
  left:
    device_number: 0
  right:
    device_number: 1
```

Dual SpaceMouse Cartesian control does not use a deadman button; axis motion
alone emits TCP twist commands. If one device has no fresh sample or only
deadbanded axes, that arm is held while the other arm may continue. Use `path`
or `device` under each side when stable HID selection is needed.

`policy_runner` does not generate `TcpLinearMove` trajectories. Use
`rb_servo_server/tools/send_tcp_linear_move.py` for simulator-only MoveL-style
test packets.

## Runtime Startup

`policy_runner` fails closed if no first robot state packet arrives before
`runtime.startup_timeout_sec`. The default is 5 seconds:

```yaml
runtime:
  startup_timeout_sec: 5.0
```

This prevents a runner from waiting forever on a misconfigured or absent state
publisher. A missing or empty `geometry.path` does not block joint-only action
sources, but geometry-dependent Cartesian sources are blocked with an explicit
safety reason.

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
P3 Cartesian packets additionally use `TcpDeltaStand` with per-arm
`tcp_delta_stand` one-shot jog payloads and `TcpTwistLocal` with per-arm
`tcp_twist_local` velocity payloads in simulation only. `TcpTwistLocal` units
are `vx,vy,vz` in m/s and `wx,wy,wz` in rad/s.
