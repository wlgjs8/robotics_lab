# Camera synchronization plan

## Important design decision

Do not place RealSense capture inside `rb_servo_server`.

Reason:

- camera frame waits can block
- USB scheduling and driver buffering can create jitter
- image encoding/writing is heavy
- servo loop should remain deterministic

## Recommended process split

```text
rb_servo_server
  - robot command/state
  - servo tick timestamp
  - command seq log

camera_recorder
  - RealSense head camera
  - RealSense left wrist camera
  - RealSense right wrist camera
  - image/depth save
  - camera frame timestamp log

dataset_builder
  - merges robot logs and camera logs by timestamp
```

## Host timestamp rule

All logs should use the same host monotonic clock concept.

In C++:

```cpp
std::chrono::steady_clock
```

In Python:

```python
time.monotonic_ns()
```

These clocks cannot be directly compared across processes if processes use different APIs with different epochs. For practical merging on one PC, either:

1. record both `system_time_ns` and `steady_time_ns`, or
2. start `camera_recorder` from a supervisor that passes a common offset, or
3. have `camera_recorder` subscribe to `rb_servo_server` state and record server timestamps.

For the first implementation, record both:

```text
host_steady_ns
host_system_ns
```

## Camera log schema

```json
{
  "camera_id": "head",
  "serial": "XXXXXXXX",
  "frame_number": 12345,
  "realsense_timestamp_ms": 123456.789,
  "host_steady_ns": 1710000000000000000,
  "host_system_ns": 1710000000000000000,
  "color_path": "images/head/color/00012345.jpg",
  "depth_path": "images/head/depth/00012345.png"
}
```

## Robot log schema needed for merge

`ServoSample` should include:

- tick
- loop start/end host time
- left/right q_actual
- left/right q_sent
- command seq
- mode
- send ok
- jitter

## Wrist camera transforms

The canonical cross-component frame naming and transform direction are defined
in `../../docs/frame_contract.md`. In short, `T_parent_child` maps child-frame
coordinates into the parent frame.

For wrist cameras, each frame should eventually be associated with:

```text
T_stand_camera_left(t)  = T_stand_tcp_left(t)  · T_tcp_camera_left
T_stand_camera_right(t) = T_stand_tcp_right(t) · T_tcp_camera_right
```

The canonical names for those transforms are
`T_stand_left_wrist_camera`, `T_stand_right_wrist_camera`,
`T_left_tcp_left_wrist_camera`, and `T_right_tcp_right_wrist_camera`.

Therefore the dataset builder needs:

- q_actual at camera timestamp
- FK result or logged TCP pose
- hand-eye calibration `T_tcp_camera`

## Sync levels

### Level 0: software timestamp sync

Use host timestamp nearest-neighbor or interpolation. This is the fastest useful implementation.

### Level 1: RealSense hardware sync

Use camera hardware sync if the RealSense model and wiring support it.

### Level 2: external trigger

Use external trigger generator for all cameras, then align robot state through host timestamps.

## Recommended first implementation

- `rb_servo_server`: no camera code
- `camera_recorder`: Python or C++ standalone
- merge by timestamp in offline `dataset_builder`
- RealSense hardware sync deferred
