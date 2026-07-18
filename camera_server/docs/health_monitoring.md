# Health Monitoring

## 1. Required counters

Per stream:

- lifetime FPS and rolling-window FPS
- frame_count
- frame_number_gap_drop_count
- recorder_drop_count
- shared_memory_write_count
- last_frame_time_ns
- last_frame_age_ms
- queue_depth

Per camera:

- connected
- serial
- firmware and recommended firmware version
- product ID, USB type descriptor, and physical port
- reconnect attempt/success/disconnect counts
- consecutive failures and exhausted state
- last disconnect/reconnect timestamps and last_error
- color/depth status
- callback enqueue, queue wait, and frame-processing p95/max latency
- callback queue depth and drop count/delta

Per bundle group:

- bundle_seq
- complete_bundle_count
- incomplete retry count
- actually discarded master-frame count
- publish rate
- skew current/p50/p95/max
- stale_bundle_count

Server-level:

- uptime_sec
- librealsense SDK version and native/RSUSB backend label
- shm_size_bytes
- metadata_publish_count
- metadata_publish_errors
- recorder bytes/sec
- disk free space

## 2. Health topic

Publish `camera.health` once per second.

Also print concise log:

```text
[CAM] head 30.0fps drop=0 age=3.2ms | left 30.0fps drop=0 | right 30.0fps drop=0 | skew=2.1ms | shm_ok=1 | rec_q=0
```

## 3. Failure levels

```text
INFO
  normal startup, config, camera mapping

WARN
  frame drops detected
  bundle skew above threshold
  recorder queue growing
  camera timestamp metadata unavailable

ERROR
  required camera missing
  shared memory write failure
  repeated reconnect failure
  disk full
```

## 4. Policy runner safety

`camera_server` should not directly stop robots.

But `policy_runner` should stop sending motion commands if:

- camera bundle stale
- required camera disconnected
- image stream invalid
- robot state stale/faulted

This creates a clean separation:

```text
camera_server reports health
policy_runner decides whether perception is valid
rb_servo_server enforces robot command safety
```

## 구현 파일

Health JSON 생성은 `src/publish/metadata_publisher.cpp`의 `health_to_json`, 주기 publish/logging은 `src/health/health_monitor.cpp`, counter collection은 `src/camera/camera_manager.cpp`에 구현되어 있다. Health topic은 config의 `metadata.health_topic` 기본값 `camera.health`를 사용한다.

Drop alarms use the delta since the previous health sample, not the lifetime
counter. A past drop therefore remains visible in the cumulative counter but
does not leave health permanently degraded after the stream recovers.
