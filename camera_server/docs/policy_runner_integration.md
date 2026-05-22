# policy_runner Integration

## 1. Data flow

```text
camera_server
  ├── writes image payload to shared memory
  └── publishes FrameBundle metadata over ZMQ

policy_runner
  ├── subscribes to camera.bundle
  ├── opens /camera_server_frames
  ├── validates slot seqlock
  ├── reads latest head/left/right images
  ├── subscribes to rb_servo_server state
  ├── runs model inference
  └── sends command to rb_servo_server
```

## 2. Latest bundle policy

`policy_runner` should keep only the latest valid bundle.

Do not process every camera bundle if inference is slower than 30 FPS.

Recommended behavior:

```text
camera_server publishes 30 FPS
policy_runner runs 10~30 Hz
policy_runner always selects latest complete bundle
older bundles are skipped
```

## 3. Stale detection

`policy_runner` should reject camera bundles if:

```text
now_ns - bundle_time_ns > max_camera_age_ms
complete == false
max_time_diff_ms > policy threshold
any required frame slot invalid
shared memory seqlock check fails repeatedly
```

Initial values:

```text
max_camera_age_ms = 100
max_time_diff_ms = 10 for software sync
max_time_diff_ms = 5 for hardware sync
```

## 4. Robot state alignment

`policy_runner` subscribes to `rb_servo_server` state.

For each inference:

```text
camera bundle timestamp t_img
robot state timestamp t_robot
```

Use latest robot state if age is acceptable, or interpolate if state history is available.

Reject inference if robot state is stale or faulted.

## 5. Zero-copy vs copy into tensor

Shared memory avoids process-to-process copy.

`policy_runner` still usually needs one copy/convert into a tensor or GPU buffer.

Initial acceptable pipeline:

```text
shared memory RGB → CPU tensor → GPU tensor
```

Future optimization:

```text
shared memory pinned buffer → async GPU upload
```

## 6. Policy input bundle

Recommended Python-side structure:

```python
obs = {
    "images": {
        "head": head_rgb,
        "left_wrist": left_rgb,
        "right_wrist": right_rgb,
    },
    "camera_time_ns": bundle_time_ns,
    "robot_state": latest_robot_state,
}
```

Do not make `rb_servo_server` depend on images.

## 구현된 sample reader

`tools/read_latest_bundle.py`는 `camera.bundle`을 구독하고 queued metadata를 drain하여 latest complete bundle만 유지한 뒤 `/dev/shm/<shm_name>`을 한 번 open/mmap해 재사용하면서 seqlock 검증 후 image bytes를 읽는다. 이 파일이 `policy_runner` 통합의 참조 구현이다.
