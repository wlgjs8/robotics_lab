# Frame Contract

This document defines shared frames, transform directions, and geometry validity. It is the cross-component frame source of truth.

## Units

- translation: meters
- rotation vectors / RPY: radians
- quaternions: `xyzw`
- joint positions in public robot commands/states: degrees
- joint velocities in public robot commands/states: degrees/second

## Canonical Frames

```text
stand
  left_base
    left_tcp
      left_wrist_camera
      left_ft_sensor
  right_base
    right_tcp
      right_wrist_camera
      right_ft_sensor
  head_camera
```

`stand` is the physical steel frame / fixture reference. It is the common world frame for GUI visualization and dual-arm geometry.

`left_base` and `right_base` are the robot base frames mounted to the stand shoulders.

`left_tcp` and `right_tcp` are the robot tool center point frames defined by the URDF tip link.

Camera frames are named by physical mounting location.

## Transform Direction

Use `T_parent_child` to mean:

```text
point_in_parent = T_parent_child * point_in_child
```

Examples:

```text
T_stand_left_base
T_stand_right_base
T_left_tcp_left_wrist_camera
T_right_tcp_right_wrist_camera
T_stand_head_camera
T_left_tcp_left_ft_sensor
T_right_tcp_right_ft_sensor
```

Force-control configuration uses the arm-local generic name `T_tcp_sensor` for
the same direction: the pose of the F/T sensor frame expressed in that arm's
TCP frame. Sensor-fixed bias is removed before applying this wrench transform;
payload/gravity and residual tare are then removed in the TCP frame.

The active RB3-730/Pika URDFs explicitly define the Robotous RFT64-6A01 CAD
measurement frame as `ft_sensor_measurement`. The combined Pika CAD already
contains the sensor geometry: the RFT64 body matches the CAD from
`attachment_site` z=15 mm through z=45 mm. The sensor's model-defined
measurement site is the tool-side face at z=45 mm, with the Pika CAD's +90
degree yaw convention. Both active URDFs therefore encode the geometry estimate:

```text
T_attachment_ft_sensor_base        = xyz [0, 0, 0.015], rpy [0, 0, pi/2]
T_ft_sensor_base_measurement       = xyz [0, 0, 0.030], rpy [0, 0, 0]
T_tcp_sensor                       = xyz [0, 0, -0.202642], rpy [0, 0, pi/2]
```

The rbpodo EFT fields do not use that CAD yaw. In the 2026-07-12 right-arm
positive-endpoint-to-TCP capture, the former `+pi/2` runtime rotation produced
`+X -> +Y` and `+Y -> -X`, while Z polarity was correct. Removing that rotation
gives the accepted positive reaction mapping `+X -> +X`, `+Y -> +Y`, and
`+Z -> +Z`. The operator declared both installed arm/sensor assemblies
identical, so the tracked physical profile uses for both arms:

```text
T_tcp_rbpodo_eft = xyz [0, 0, -0.202642], rpy [0, 0, 0]
```

This acceptance covers the three positive force axes only. Torque-axis checks,
sensor serials, payload/COM, and production force-motion acceptance remain
pending. `rb_gui` still reads `ft_sensor_measurement` from the viewer URDF for
the CAD-local sensor overlay; use the TCP gizmo together with the transformed
surface/compliance wrench for the controller-axis check. CAD orientation must
not be substituted for the empirically accepted rbpodo EFT source frame.

## Mount Orientation Convention

Runtime `Pose6D` uses canonical URDF/ROS-style RPY:

```text
R = Rz(yaw) * Ry(pitch) * Rx(roll)
```

The original stand/robot mount rotations came from MJCF `euler xyz` values. MJCF Euler values must not be copied directly into canonical RPY fields. They must be converted first or represented as quaternions.

The current configured-estimate mount values use canonical RPY converted from the original MJCF convention. If map-style pose configs with `quaternion_xyzw` are present, quaternion orientation is the canonical orientation and RPY is display/legacy only.

## Calibration Registry

The current setup registry is:

```text
calibration/active_calibration.yaml
```

Current status is expected to be:

```yaml
status: configured_estimate
geometry_valid_for_real_policy: false
```

`configured_estimate` may be used for visualization and simulator work. It is not measured calibration and must not be used to justify real geometry-dependent policy.

## Runtime Source Of Truth

`rb_servo_server` runtime mount config is the current active source for FK/IK state publication. `calibration/active_calibration.yaml` is the global setup registry that future work should load or cross-check against runtime config.

If runtime config and calibration registry disagree, fail or warn explicitly. Do not silently mix frame definitions.

## Quaternion Policy

State publishers and GUI markers should prefer `quaternion_xyzw` over RPY when both are available. RPY is useful for human display but should not be treated as the highest-fidelity orientation representation.

## TCP State Terminology

State JSON distinguishes three TCP concepts:

- actual: FK from measured `q_actual_deg`; published as `tcp_actual_base` and
  `tcp_actual_stand`.
- reference: FK from controller/internal reference joints such as rbpodo
  `jnt_ref`; published as `tcp_ref_base` and `tcp_ref_stand` only when finite.
- desired: the benchmark or command trajectory target; this is not the same as
  actual or controller reference.

Legacy `tcp_base` and `tcp_stand` remain actual-pose aliases. In Rainbow
controller `pgmode` simulation, the physical arm may remain stationary while
the controller reference evolves, so tracking reports should compare the
desired trajectory against `tcp_ref_stand` when the state stream recommends it.
Physical real-motion reports should compare desired trajectory against
`tcp_actual_stand`.

## Joint-Only Versus Geometry-Dependent Behavior

Joint-only actions do not require measured calibration.

The following require stronger geometry validation before real use:

- Cartesian TCP policy based on camera geometry
- wrist-camera-to-TCP transforms
- head-camera-to-stand transforms
- real camera-driven manipulation policies
- dataset capture requiring metric replayability

## Collision Geometry Asset (unified URDF)

Two distinct URDF assets feed the server, and only one is tracked here:

| Use | Config key | Path | Tracked in this repo? |
|---|---|---|---|
| FK/IK kinematics (per arm) | `kinematics.urdf` | `rb_servo_server/descriptions/urdf/rb3_730e.urdf` (single arm) | **yes** |
| Self-collision guard (`CollisionMonitor`) | `safety.self_collision.mesh.unified_urdf` | `mo_robot_descriptions/.../robots/urdf/dual_rb3_730e/dual_rb3_730e_ver3.urdf` (stand + both arms) | **no** |

The **unified** (stand + both-arms) URDF is the most important geometry input to
the self-collision guard — `CollisionMonitor` builds its Pinocchio model and
mesh pairs directly from it (`collision_monitor.cpp`, wired in
`dual_arm_servo_loop.cpp`). It is **not** stored in `robotics_lab`: it lives in
the sibling `mo_robot_descriptions` repo, which is a **separate, manually managed
checkout — not a git submodule and not version-pinned from here**. Configs
reference it by relative path (`../mo_robot_descriptions/...`), resolved against
the config file location.

Provenance / regeneration is currently **not documented in `mo_robot_descriptions`
from this repo's side** — this is a known gap. When onboarding or reproducing the
collision model, treat the following as required-but-unspecified and pin them
explicitly in the site setup notes:

- which `mo_robot_descriptions` commit/tag the `dual_rb3_730e_ver*` URDF came from
- how the dual URDF is generated from the single-arm description + stand mount
  geometry (which `_ver` is current, and what changed between `ver2`/`ver3`)
- that link/mesh names stay consistent with the server's
  `stand_ignore_arm_substrings` (`dual_rb3_730e_left_` / `dual_rb3_730e_right_`)
  so arm-vs-stand pairing stays correct

Until that is captured, a fresh clone cannot rebuild the collision model without
out-of-band knowledge of the `mo_robot_descriptions` checkout.

## Measured Calibration Future Work

Measured calibration should add:

- calibration ID
- measurement date
- method/tool used
- robot serials and controller IPs
- camera serials and stream profiles
- transform covariance or quality flag, if available
- acceptance artifact paths

Until then, real geometry-dependent policy remains blocked.
