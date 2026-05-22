# Config Schema

Example files:

- `config/triple_realsense.yaml`: canonical real-camera template, same profile
  as `config/triple_realsense_640x360.yaml`
- `config/triple_realsense_640x360.yaml`: canonical D435f head
  1280x720@30 RGB plus D405 wrists 640x360@30 RGB
- `config/triple_realsense_640x480.yaml`: optional D405 wrist
  640x480@30 RGB variant for explicitly approved hardware sessions
- `config/mock_triple_realsense.yaml`: hardware-free mock config

The real-camera templates intentionally contain `REPLACE_*` serial placeholders
and must fail validation until copied and filled with approved physical serials.

```yaml
server:
  name: camera_server
  mode: capture_only       # capture_only | record | replay | diagnostic
  clock: monotonic_raw     # monotonic | monotonic_raw
  simulate_cameras: false

shared_memory:
  name: "/camera_server_frames"
  size_mb: 1024
  ring_slots: 4
  unlink_on_start: true

metadata:
  transport: zmq_pub
  pub_bind: "tcp://127.0.0.1:5600"
  health_topic: "camera.health"
  bundle_topic: "camera.bundle"

sync:
  mode: software           # software | hardware
  master_camera: head
  bundle_policy: nearest_timestamp
  max_bundle_time_diff_ms: 10.0
  publish_incomplete_bundles: false

cameras:
  head:
    serial: "REPLACE_HEAD_SERIAL"
    required: true
    streams:
      color:
        enabled: true
        width: 1280
        height: 720
        fps: 30
        format: rgb8
      depth:
        enabled: false
        width: 1280
        height: 720
        fps: 30
        format: z16
  left_wrist:
    serial: "REPLACE_LEFT_SERIAL"
    required: true
    streams:
      color:
        enabled: true
        width: 640
        height: 360
        fps: 30
        format: rgb8
      depth:
        enabled: false
  right_wrist:
    serial: "REPLACE_RIGHT_SERIAL"
    required: true
    streams:
      color:
        enabled: true
        width: 640
        height: 360
        fps: 30
        format: rgb8
      depth:
        enabled: false

recording:
  enabled: false
  output_dir: "/data/episodes"
  queue_capacity_frames: 300
  writer_threads: 4
  drop_policy: drop_oldest
  raw_format: true

health:
  publish_rate_hz: 1
  warn_if_frame_age_ms_gt: 100
  warn_if_drop_count_increases: true
  warn_if_bundle_skew_ms_gt: 10

reconnect:
  enabled: false
  max_attempts: 0
  retry_interval_ms: 1000
```

## Validation rules

- all required camera serials must be present
- real-camera serial placeholders fail validation:
  - empty required serial
  - `REPLACE_*`
  - `TODO`
  - `CHANGEME`
  - `UNKNOWN`
- `MOCK_*` serials are accepted only when `server.simulate_cameras: true`
- initial sync combinations are strict:
  - `sync.mode: software` requires `sync.bundle_policy: nearest_timestamp`
  - `sync.mode: hardware` requires `sync.bundle_policy: frame_number`
- `reconnect.enabled: true` fails validation until reconnect is implemented
- stream FPS must be 30 initially unless explicitly changed
- width/height must be positive
- shared memory size must fit configured streams and ring slots
- metadata bind must default to `127.0.0.1`
- if `sync.mode=hardware`, RealSense `inter_cam_sync_mode` must be configured successfully for every camera or startup fails
- if recording enabled, output directory must be writable

Geometry and calibration naming are shared with `rb_servo_server`, `rb_gui`,
and the future `policy_runner` through `../../docs/frame_contract.md`.
`camera_server` camera names remain `head`, `left_wrist`, and `right_wrist`;
geometry frame names are `head_camera`, `left_wrist_camera`, and
`right_wrist_camera`. `camera_server` must not invent extrinsics when
calibration files are absent.

## Derived values

The server should compute:

```text
slot_bytes per stream
ring total bytes
expected bytes/sec
expected frame period ns
```

If shared memory size is insufficient, fail at startup.

## Dependency profiles

Hardware-free build/test profile:

```bash
sudo apt install cmake build-essential libyaml-cpp-dev
```

This profile is sufficient for:

```bash
cmake -S camera_server -B camera_server/build/hardware_free_gate \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCAMERA_SERVER_FORCE_MOCK_CAMERA=ON \
  -DCAMERA_SERVER_FORCE_ZMQ_STUB=ON \
  -DCAMERA_SERVER_BUILD_TESTS=ON
cmake --build camera_server/build/hardware_free_gate -j
ctest --test-dir camera_server/build/hardware_free_gate --output-on-failure
```

Real-camera profile additions:

```bash
sudo apt install libzmq3-dev
# Install librealsense2-dev from the Intel RealSense package source approved for the host OS.
```

Python diagnostic tools need `pyzmq` only when subscribing to live metadata:

```bash
python3 -m pip install pyzmq
```

`camera_server/tools/list_realsense_devices` is still a future helper; for P0
hardware sessions use `rs-enumerate-devices` and record serials in the approved
config copy.

## 구현 파일

Schema loader/validator는 `src/config/config.cpp`에 있다. 기본 production config는 `config/triple_realsense.yaml`, hardware 없는 검증 config는 `config/mock_triple_realsense.yaml`이다. Validator는 serial placeholder, mock serial gate, sync mode/policy 조합, reconnect disabled state, required serial, positive dimensions/FPS, supported formats, POSIX shm name, loopback metadata bind, shared-memory capacity를 검사한다.
