# Metadata Protocol over ZeroMQ

## 1. Goal

Full image payload is stored in shared memory. ZeroMQ only publishes small metadata messages.

Metadata tells subscribers:

- which camera frames are available
- where the image bytes live in shared memory
- timestamps
- frame numbers
- synchronization quality
- drop/health counters

## 2. Transport

Recommended:

```text
ZeroMQ PUB/SUB
bind: tcp://127.0.0.1:5600
```

For same-host-only deployment, TCP loopback is fine. Unix Domain Socket is also acceptable.

Do not expose metadata or control sockets on `0.0.0.0` by default.

## 3. Topics

Recommended topic names:

```text
camera.bundle
camera.frame.head.color
camera.frame.head.depth
camera.frame.left_wrist.color
camera.frame.left_wrist.depth
camera.frame.right_wrist.color
camera.frame.right_wrist.depth
camera.health
camera.event
```

For policy, the most important topic is:

```text
camera.bundle
```

## 4. Frame metadata schema

Example JSON payload:

```json
{
  "schema": "camera_server.frame.v1",
  "camera_name": "head",
  "serial": "1234567890",
  "stream": "color",
  "frame_number": 1042,
  "host_arrival_time_ns": 1234567890000,
  "sensor_timestamp_ns": 1234567889900,
  "realsense_timestamp_ms": 1234567.889,
  "actual_exposure_us": 6951,
  "gain_level": 16,
  "auto_exposure": true,
  "width": 640,
  "height": 480,
  "stride_bytes": 1920,
  "format": "rgb8",
  "shm_name": "/camera_server_frames",
  "ring_name": "head.color",
  "slot_index": 2,
  "shm_offset": 1048576,
  "size_bytes": 921600,
  "seq": 1042,
  "valid": true
}
```

`actual_exposure_us`, `gain_level`, `auto_exposure`는 optional RealSense
per-frame metadata다. 장치/stream/firmware/backend가 해당 metadata를 지원할 때만
포함하며 mock/UVC 또는 지원하지 않는 RealSense 프레임에서는 필드를 생략한다.
현재 sensor option을 per-frame 값으로 추정하여 대체하지 않는다. 이 optional 확장은
`camera_server.frame.v1`과 `camera_server.bundle.v1` schema version을 변경하지 않는다.

## 5. Bundle metadata schema

`FrameBundle` is what `policy_runner` should use.

Example:

```json
{
  "schema": "camera_server.bundle.v1",
  "bundle_seq": 10001,
  "bundle_time_ns": 1234567890000,
  "hardware_synced": false,
  "sync_policy": "nearest_timestamp",
  "max_time_diff_ms": 2.4,
  "complete": true,
  "frames": {
    "head.color": {
      "camera_name": "head",
      "serial": "123",
      "stream": "color",
      "frame_number": 1042,
      "host_arrival_time_ns": 1234567890000,
      "sensor_timestamp_ns": 1234567889900,
      "width": 640,
      "height": 480,
      "stride_bytes": 1920,
      "format": "rgb8",
      "ring_name": "head.color",
      "slot_index": 2,
      "shm_offset": 1048576,
      "size_bytes": 921600
    },
    "left_wrist.color": { },
    "right_wrist.color": { }
  },
  "drop_counters": {
    "head.color": 0,
    "left_wrist.color": 0,
    "right_wrist.color": 0
  }
}
```

For RGB-only policy, `complete=true` means all three color frames are available.

For RGB-D policy, `complete=true` means all configured streams are available.

## 6. Health schema

Publish periodically, for example 1 Hz.

```json
{
  "schema": "camera_server.health.v1",
  "host_time_ns": 1234567890000,
  "uptime_sec": 120.5,
  "mode": "capture_only",
  "cameras": {
    "head": {
      "serial": "123",
      "connected": true,
      "fps_color": 30.0,
      "fps_depth": 30.0,
      "drop_count_color": 0,
      "drop_count_depth": 0,
      "last_frame_age_ms": 3.1,
      "reconnect": {
        "attempt_count": 1,
        "success_count": 1,
        "disconnect_count": 1,
        "consecutive_failures": 0,
        "exhausted": false,
        "last_error": "frame timeout after 2000 ms"
      }
    }
  },
  "shared_memory": {
    "name": "/camera_server_frames",
    "size_bytes": 1073741824,
    "write_errors": 0
  },
  "recorder": {
    "enabled": false,
    "queue_depth": 0,
    "dropped_by_queue": 0,
    "write_latency_ms_mean": 0.0
  }
}
```

## 7. Metadata reliability

PUB/SUB may drop messages if subscriber is slow. This is acceptable for latest-frame policy.

`policy_runner` should:

- keep only the latest bundle
- detect stale bundle by timestamp
- detect sequence jumps
- read shared memory slot only after verifying seqlock

For lossless recording, do not rely only on PUB/SUB. Recording should happen inside `camera_server` through the recording queue.

## 8. Message encoding

Initial implementation may use JSON.

Future options:

- MessagePack
- FlatBuffers
- Cap'n Proto

JSON is acceptable because metadata is small and FPS is 30.

## 구현 파일

Frame/bundle/health JSON schema serializer는 `src/publish/metadata_publisher.cpp`에 구현되어 있다. PUB socket은 libzmq가 있을 때 multipart `[topic, json]`으로 publish한다. libzmq가 없는 build host에서는 동일 serializer를 사용하고 health를 stderr에 출력하는 stub backend로 build/test 가능성을 유지한다.
