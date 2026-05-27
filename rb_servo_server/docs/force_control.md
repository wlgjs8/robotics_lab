# Force Control Design

Force control is included in the v3 design as an optional layer, but it is disabled by default and is not wired into the joint-only milestone.

The intended control chain is:

```text
Python policy / teleop
  nominal TCP target or delta
  optional desired wrench
        ↓
CommandServer
        ↓
DualArmServoLoop
        ↓
CartesianController
        ↓
ForceController
  measured wrench + desired wrench → TCP compensation
        ↓
IK
        ↓
TrajectoryFilter
        ↓
SafetyFilter
        ↓
servo_j
```

## Why force output is TCP compensation

Because `rb_servo_server` is built around Rainbow `servo_j`, the force controller should not output joint torque. Instead it should output one of:

- TCP pose offset
- TCP velocity offset
- Cartesian compliance correction

The corrected TCP target is then passed through IK and finally converted to a joint target for `servo_j`.

## New types

`types.hpp` now contains:

- `Wrench6D`
- `ForceControlAxis`
- `ForceControlMode`
- `ForceControlCommand`

`ArmCommand` contains:

```cpp
ForceControlCommand force_control;
```

## New config section

All example configs include:

```yaml
force_control:
  enable: false
  update_rate_hz: 200
  admittance_gain_pos: 0.0002
  admittance_gain_rot: 0.0001
  force_lpf_alpha: 0.2
  max_pos_offset_m: 0.01
  max_rot_offset_rad: 0.1
  max_pos_step_m: 0.001
  max_rot_step_rad: 0.01
```

## New files

```text
include/rb_servo/control/force_controller.hpp
src/control/force_controller.cpp
include/rb_servo/sensor/i_force_torque_sensor.hpp
include/rb_servo/sensor/mock_force_torque_sensor.hpp
src/sensor/mock_force_torque_sensor.cpp
```

## Minimal admittance fallback

The included `ForceController` is not a complete production force controller. It is a safe scaffold-level fallback:

```text
offset_dot = admittance_gain * (target_wrench - measured_wrench)
```

The output is clamped by:

- max positional offset
- max rotational offset
- max positional step
- max rotational step

Before real contact use, verify:

- sensor frame sign convention
- TCP frame convention
- wrench filtering
- tare/bias behavior
- emergency stop behavior
- force gain values

## How to integrate `mo_forcecontroller` later

The earlier `mo_forcecontroller` code should be wrapped behind the same interface:

```cpp
class ForceController {
public:
    Pose6D computeTcpCompensation(...);
};
```

Before integration, make its sampling time configurable. The observed `Ts = 0.002` implies 500 Hz. This server starts at 100–200 Hz, so `Ts` must come from `ForceControlConfig` or the actual loop `dt_sec`.

Recommended path:

1. joint `servo_j` stable
2. Cartesian FK/IK stable
3. mock force sensor stable
4. minimal admittance controller with very small gain
5. real F/T sensor integration
6. `mo_forcecontroller` wrapper integration
