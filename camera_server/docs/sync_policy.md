# Sync Policy

## 1. Goal

`camera_server` must expose synchronized frame bundles from 3 RealSense cameras:

```text
head
left_wrist
right_wrist
```

Each camera should run at 30 FPS.

## 2. Timestamp sources

Record all useful timestamps:

```text
host_arrival_time_ns
  - timestamp captured by camera_server as soon as frame arrives
  - use CLOCK_MONOTONIC_RAW or CLOCK_MONOTONIC consistently

sensor_timestamp_ns
  - RealSense frame timestamp converted to ns when available

realsense_timestamp_ms
  - raw librealsense timestamp in milliseconds

frame_number
  - RealSense frame number
```

The most important timestamp for robot-camera alignment is `host_arrival_time_ns`, because `rb_servo_server` can use the same host monotonic clock.

## 3. Hardware sync mode

If RealSense model and cabling support hardware sync, configure:

```text
head       master, or external trigger master
left_wrist slave
right_wrist slave
```

Config field:

```yaml
sync:
  mode: hardware
  master_camera: head
  max_bundle_time_diff_ms: 5.0
```

In hardware mode, `camera_server` must set RealSense `RS2_OPTION_INTER_CAM_SYNC_MODE` on every configured device. If any device does not support the option or setting it fails, startup fails instead of publishing `hardware_synced=true` metadata. Hardware bundles are frame-number driven.

Still record host timestamps. Hardware sync aligns exposure/capture timing, not necessarily USB delivery timing.

## 4. Software sync mode

If hardware sync is unavailable, use nearest timestamp matching.

Config:

```yaml
sync:
  mode: software
  max_bundle_time_diff_ms: 10.0
  bundle_policy: nearest_timestamp
```

Algorithm:

1. Maintain a small queue of recent frames per camera stream.
2. Choose a reference stream, typically `head.color`.
3. For each head frame, find left/right frames with closest `host_arrival_time_ns` or sensor timestamp.
4. If max time diff is below threshold, publish complete bundle.
5. Otherwise publish incomplete bundle or drop, according to config.

Recommended default:

```text
policy path:
  drop incomplete or high-skew bundles

recording path:
  record all individual frames regardless of bundle completeness
```

## 5. Bundle timestamp

Recommended:

```text
bundle_time_ns = median(frame host timestamps)
```

Also record:

```text
min_time_ns
max_time_ns
max_time_diff_ms
```

## 6. Skew threshold

Initial values:

```text
hardware sync:
  max_bundle_time_diff_ms = 3~5 ms

software sync:
  max_bundle_time_diff_ms = 10~15 ms
```

`policy_runner` may reject bundles above its own threshold.

## 7. Robot-camera sync

`rb_servo_server` should publish robot state snapshots with the same clock base.

`policy_runner` can align:

```text
camera bundle time t
→ nearest robot state snapshot
→ or interpolated robot state between two samples
```

For dataset generation, offline `dataset_builder` should merge:

```text
camera metadata log
robot servo log
policy command log
```

## 8. Drift and diagnostics

Track:

- per-camera FPS
- frame number gaps
- max bundle time diff
- arrival-time jitter
- host-sensor timestamp drift
- queue depth
- USB reconnect count

Print concise health summary every second.

## 구현 파일

Software sync는 `src/sync/frame_synchronizer.cpp`에 구현되어 있다. Stream별 recent-frame deque를 유지하고 `sync.master_camera`의 color frame이 도착할 때만 bundle 생성을 시도한다. Software mode는 master timestamp에 가장 가까운 required stream frame을 선택하고, hardware mode는 동일 `frame_number` frame을 선택한다. Host arrival timestamp의 max-min skew가 `sync.max_bundle_time_diff_ms` 이하일 때만 `complete=true`가 된다. Policy path는 incomplete/high-skew bundle을 기본 drop하며, `publish_incomplete_bundles=true`일 때만 incomplete bundle도 publish한다.
