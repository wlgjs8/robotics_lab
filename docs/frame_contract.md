# Robotics Lab Frame Contract

This document is the shared coordinate-frame contract for `rb_servo_server`,
`camera_server`, `rb_gui`, and the future `policy_runner`.

The system source of truth is [architecture.md](architecture.md). Frame data is
for visualization, calibration, and policy inputs; it does not open any real
motion path by itself.

It defines names, transform direction, calibration file locations, and the
minimum schema needed before real calibration is available. Values already
present in configs are treated as current configured estimates, not measured
calibration.

## Conventions

- Frame names use lowercase snake case.
- Arm prefixes are `left` and `right`.
- Camera names are exactly `head`, `left_wrist`, and `right_wrist`.
- Transforms are named `T_parent_child`.
- `T_parent_child` maps coordinates expressed in `child` into `parent`:

```text
p_parent = T_parent_child * p_child
```

- Pose arrays use `[x, y, z, rx, ry, rz]`.
- Translation units are meters.
- Rotation units are radians.
- `rx`, `ry`, `rz` are roll, pitch, yaw Euler angles using the same convention
  currently used by `rb_servo_server` mount config and `rb_gui` display helpers.
- Components must not silently invert transform direction based on field name.
  If a component needs the inverse, it computes and names it explicitly.

## Canonical Frames

| Frame | Owner | Meaning |
| --- | --- | --- |
| `stand` | shared contract | World/root frame fixed to the dual-arm stand. |
| `left_base` | `rb_servo_server` | Left robot base frame. |
| `right_base` | `rb_servo_server` | Right robot base frame. |
| `left_tcp` | `rb_servo_server` | Left tool-center-point frame from FK or robot state. |
| `right_tcp` | `rb_servo_server` | Right tool-center-point frame from FK or robot state. |
| `head_camera` | `camera_server` + calibration | Optical frame for the head camera. |
| `left_wrist_camera` | `camera_server` + calibration | Optical frame for the left wrist camera. |
| `right_wrist_camera` | `camera_server` + calibration | Optical frame for the right wrist camera. |

For image streams, append stream names to camera names only for data routing,
not geometry frames:

```text
head.color
left_wrist.color
right_wrist.color
```

Geometry uses `head_camera`, `left_wrist_camera`, and `right_wrist_camera`.

## Current Configured Mounts

`rb_servo_server` publishes mount estimates from `left_mount` and `right_mount`
as state-stream `mounts.left.base_pose_in_stand` and
`mounts.right.base_pose_in_stand`.

Current configured estimates:

```yaml
T_stand_left_base:
  source: rb_servo_server/config/*.yaml left_mount.base_pose_in_stand
  xyz_rpy: [0.1601, -0.1725, 0.5825, 0.785, 2.35619, 0.0]
  status: configured_estimate

T_stand_right_base:
  source: rb_servo_server/config/*.yaml right_mount.base_pose_in_stand
  xyz_rpy: [-0.1601, -0.1725, 0.5825, 0.785, -2.35619, 0.0]
  status: configured_estimate
```

These are not measured acceptance calibration. Treat them as current setup
values until a hardware calibration run produces a signed calibration file.

## Transform Chain

Robot TCP in stand:

```text
T_stand_left_tcp(t)  = T_stand_left_base  * T_left_base_left_tcp(q_left(t))
T_stand_right_tcp(t) = T_stand_right_base * T_right_base_right_tcp(q_right(t))
```

Wrist camera in stand:

```text
T_stand_left_wrist_camera(t) =
  T_stand_left_tcp(t) * T_left_tcp_left_wrist_camera

T_stand_right_wrist_camera(t) =
  T_stand_right_tcp(t) * T_right_tcp_right_wrist_camera
```

Head camera in stand:

```text
T_stand_head_camera = calibrated fixed transform
```

The current system does not yet publish real `tcp_stand` or `tcp_base` fields;
`StatePublisher` marks TCP fields as deferred. Until FK/IK is implemented,
`policy_runner` must not assume camera-to-robot geometry is available from live
state. It may use joint state plus a versioned FK/calibration package once that
package exists. Real Cartesian/TCP motion remains closed until a separate
real-hardware acceptance procedure approves it, even after simulator validation.

## Component Responsibilities

### rb_servo_server

- Owns `stand`, `left_base`, `right_base`, `left_tcp`, and `right_tcp` names.
- Publishes `mounts.left.base_pose_in_stand` and
  `mounts.right.base_pose_in_stand` as `T_stand_left_base` and
  `T_stand_right_base`.
- Publishes joint state and, in the future, `tcp_base` and `tcp_stand`.
- Must label `tcp_stand` as `T_stand_<arm>_tcp`.
- Must label `tcp_base` as `T_<arm>_base_<arm>_tcp`.
- Must keep `tcp_fields_deferred=true` until TCP transforms are real.

### camera_server

- Owns camera identity and stream metadata:
  - `head`
  - `left_wrist`
  - `right_wrist`
- Does not own robot or stand transforms.
- Publishes frame metadata and bundles using stream keys such as
  `head.color`, `left_wrist.color`, and `right_wrist.color`.
- May publish camera serials and intrinsics after calibration support lands.
- Must not invent extrinsics when calibration files are absent.

### rb_gui

- Displays `stand` as the scene root.
- Uses state-stream mounts as current configured estimates.
- Uses `tcp_stand` when present; otherwise it may show visual fallback markers,
  but fallback markers are not calibration data.
- Must not write calibration files.

### future policy_runner

- Consumes robot state, camera bundles, and calibration files.
- Uses `T_parent_child` direction exactly as stored.
- Refuses geometry-dependent policy execution if a required extrinsic is absent,
  stale, or marked `configured_estimate` when a measured calibration is required.
- Logs the calibration package id with every episode/action trace.
- Does not route Cartesian/TCP targets into real motion unless the servo server
  has accepted the explicit real Cartesian gate and a real-hardware acceptance
  procedure has approved that path.

## Calibration File Locations

Draft shared layout:

```text
calibration/
  README.md
  active_calibration.yaml
  robot/
    stand_mounts.yaml
  cameras/
    intrinsics.yaml
    extrinsics.yaml
  hand_eye/
    wrist_cameras.yaml
  snapshots/
    YYYYMMDD_HHMMSS_<label>.yaml
```

Repository configs may continue to carry default configured estimates. Measured
calibration belongs under `calibration/` and should be copied into run artifacts
for each hardware or dataset session.

## Draft Calibration Schema

`calibration/active_calibration.yaml`:

```yaml
schema: robotics_lab.calibration.v1
calibration_id: "UNMEASURED_CONFIG_ESTIMATE"
created_utc: null
status: configured_estimate
source: "rb_servo_server config defaults"
files:
  stand_mounts: robot/stand_mounts.yaml
  camera_intrinsics: cameras/intrinsics.yaml
  camera_extrinsics: cameras/extrinsics.yaml
  wrist_hand_eye: hand_eye/wrist_cameras.yaml
```

`calibration/robot/stand_mounts.yaml`:

```yaml
schema: robotics_lab.transforms.v1
transforms:
  - name: T_stand_left_base
    parent: stand
    child: left_base
    xyz_rpy: [0.1601, -0.1725, 0.5825, 0.785, 2.35619, 0.0]
    units:
      translation: m
      rotation: rad
    status: configured_estimate
    source: rb_servo_server/config/dual_real.yaml
  - name: T_stand_right_base
    parent: stand
    child: right_base
    xyz_rpy: [-0.1601, -0.1725, 0.5825, 0.785, -2.35619, 0.0]
    units:
      translation: m
      rotation: rad
    status: configured_estimate
    source: rb_servo_server/config/dual_real.yaml
```

`calibration/cameras/intrinsics.yaml`:

```yaml
schema: robotics_lab.camera_intrinsics.v1
cameras:
  head:
    serial: "REPLACE_HEAD_SERIAL"
    status: unmeasured
    color: null
    depth: null
  left_wrist:
    serial: "REPLACE_LEFT_SERIAL"
    status: unmeasured
    color: null
    depth: null
  right_wrist:
    serial: "REPLACE_RIGHT_SERIAL"
    status: unmeasured
    color: null
    depth: null
```

`calibration/cameras/extrinsics.yaml`:

```yaml
schema: robotics_lab.transforms.v1
transforms:
  - name: T_stand_head_camera
    parent: stand
    child: head_camera
    xyz_rpy: null
    status: unmeasured
```

`calibration/hand_eye/wrist_cameras.yaml`:

```yaml
schema: robotics_lab.transforms.v1
transforms:
  - name: T_left_tcp_left_wrist_camera
    parent: left_tcp
    child: left_wrist_camera
    xyz_rpy: null
    status: unmeasured
  - name: T_right_tcp_right_wrist_camera
    parent: right_tcp
    child: right_wrist_camera
    xyz_rpy: null
    status: unmeasured
```

Null transform values mean unavailable. Consumers must fail closed when a
required transform is null or `status: unmeasured`.

## State and Dataset Metadata Requirements

Every recorded episode should include:

- `calibration_id`
- copy or checksum of `active_calibration.yaml`
- `T_stand_left_base` and `T_stand_right_base` source/status
- camera serial mapping for `head`, `left_wrist`, `right_wrist`
- whether `tcp_stand` came from live state, FK reconstruction, or was absent
- timestamp basis used for robot/camera alignment

`policy_runner` should include the same calibration id in action logs so model
inputs can be audited against the geometry used at runtime.

## Open Items

- Measure real stand origin and axis convention against the physical fixture.
- Decide whether Euler `xyz_rpy` remains sufficient or whether runtime geometry
  should require quaternions/matrices.
- Implement FK so `T_<arm>_base_<arm>_tcp` and `T_stand_<arm>_tcp` can be
  populated truthfully.
- Add camera intrinsics extraction from RealSense profiles.
- Add hand-eye calibration for wrist cameras.
- Add measured `T_stand_head_camera`.
- Add validation tooling that rejects missing, inverted, or stale transforms.
