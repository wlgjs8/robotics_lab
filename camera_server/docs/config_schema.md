# Config Schema

Example files:

- `config/d435_head_1280x720.yaml`: current `make cam-up` default — D435 head
  RGB + 1280x720 IR stereo, plus two D405 wrist RealSense cameras with
  color+depth at 640x480
- `config/head_wrists.yaml`: alternate D435-head + dual-D405 profile using
  640x480 head IR stereo when the static 640x480 intrinsics/engine are required
- `config/triple_realsense.yaml` and `config/triple_realsense_640x360.yaml`:
  legacy three-RealSense RGB-only profile retained for older capture sessions
- `config/triple_realsense_640x480.yaml`: legacy D405 wrist 640x480 RGB variant
  for explicitly approved hardware sessions
- `config/dual_realsense_d405.yaml`: two D405 wrist cameras 640x480@30 RGB for
  flow-infer rollout on the local PC (camera names `left_realsense` /
  `right_realsense` pair with checkpoint camera names
  `left_realsense_color` / `right_realsense_color` via policy_runner's
  `resolve_frame()` `X_color` → `X.color` mapping); serials are filled with
  this site's physical units
- `config/mock_triple_realsense.yaml`: hardware-free mock config
- `config/quad_realsense_fisheye.yaml`: flow-infer fisheye-deploy profile — two
  D405 wrist RealSense (color + depth) PLUS two DECXIN/Sunplus UVC fisheye cameras
  (`backend: uvc`, color-only, MJPG transport), all published in every bundle so
  policy_runner selects which the checkpoint consumes (`left_fisheye.color` /
  `right_fisheye.color` ↔ checkpoint `left_fisheye_color` / `right_fisheye_color`)

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
  size_mb: 1536
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
  max_bundle_time_diff_ms: 33.0
  publish_incomplete_bundles: false

bundle_groups:              # optional; omitted profiles retain one legacy group
  policy:
    topic: "camera.bundle.policy"
    master_stream: "left_realsense.color"
    required_streams:
      - "left_realsense.color"
      - "left_realsense.depth"
      - "right_realsense.color"
      - "right_realsense.depth"
    max_bundle_time_diff_ms: 33.0
    publish_incomplete_bundles: false

cameras:
  head:
    backend: realsense      # realsense (default, depth-capable) | uvc (V4L2 fisheye)
    serial: "REPLACE_HEAD_SERIAL"
    # device: "/dev/v4l/by-path/...-video-index0"   # uvc only (path/index/by-path)
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
      ir_left:
        enabled: true
        width: 1280
        height: 720
        fps: 30
        format: y8
      ir_right:
        enabled: true
        width: 1280
        height: 720
        fps: 30
        format: y8
  left_realsense:
    serial: "REPLACE_LEFT_SERIAL"
    required: true
    streams:
      color:
        enabled: true
        width: 640
        height: 480
        fps: 30
        format: rgb8
      depth:
        enabled: true
        width: 640
        height: 480
        fps: 30
        format: z16
  right_realsense:
    serial: "REPLACE_RIGHT_SERIAL"
    required: true
    streams:
      color:
        enabled: true
        width: 640
        height: 480
        fps: 30
        format: rgb8
      depth:
        enabled: true
        width: 640
        height: 480
        fps: 30
        format: z16

recording:
  enabled: false
  output_dir: "/data/episodes"
  queue_capacity_frames: 300
  writer_threads: 4
  drop_policy: drop_oldest
  raw_format: true

health:
  publish_rate_hz: 1
  fps_window_sec: 5
  warn_if_fps_below: 29
  warn_if_frame_age_ms_gt: 100
  warn_if_drop_count_increases: true
  warn_if_bundle_skew_ms_gt: 10

reconnect:
  enabled: false
  max_attempts: 0
  retry_interval_ms: 1000
```

## Camera backends

- `backend: realsense` (default): librealsense device selected by `serial`,
  color + optional depth. Subject to the serial placeholder / MOCK rules below.
- `backend: uvc`: generic V4L2 UVC camera opened via OpenCV with an MJPG transport
  (low USB bandwidth, mirrors `pika/pika_win/fisheye.py`), used for the
  DECXIN/Sunplus wrist fisheye. Selected by `device` (a `/dev/videoN` node, an
  integer index, or a `/dev/v4l/by-path/...` symlink — by-path is preferred,
  stable across reboots). Color-only (no depth). The MJPG frame is decoded and
  converted BGR→RGB so the shared-memory payload is `rgb8`, identical to the
  realsense color format (policy_runner treats every 3-channel frame as RGB).
  Requires the OpenCV backend at build time (`libopencv-dev`); a `uvc` camera in a
  build without OpenCV fails at startup.

## Validation rules

- `backend` must be `realsense` or `uvc`
- a `uvc` camera must not enable a depth stream
- a required `uvc` camera must set a non-empty `device`; its presence is checked at
  startup by resolving the device path (not by serial — the DECXIN fisheye share a
  fixed serial, so serial/MOCK placeholder rules do not apply to `uvc`)
- all required RealSense camera serials must be present
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
- bundle group names and topics must be unique; every required/master stream
  must refer to an enabled stream
- stream FPS must be 30 initially unless explicitly changed
- width/height must be positive
- shared memory size must fit configured streams and ring slots
- metadata bind must default to `127.0.0.1`
- if `sync.mode=hardware`, RealSense `inter_cam_sync_mode` must be configured successfully for every camera or startup fails
- if recording enabled, output directory must be writable

Geometry and calibration naming are shared with `rb_servo_server`, `rb_gui`,
and the future `policy_runner` through `../../docs/frame_contract.md`.
The current stack profile uses `head`, `left_realsense`, and `right_realsense`
camera names. Older triple-RealSense profiles use `head`, `left_wrist`, and
`right_wrist`. Geometry frame names remain owned by `../../docs/frame_contract.md`.
`camera_server` must not invent extrinsics when calibration files are absent.

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

Schema loader/validator는 `src/config/config.cpp`에 있다. 현재 repository-root
`make cam-up` 기본 production config는 `config/d435_head_1280x720.yaml`,
hardware 없는 검증 config는 `config/mock_triple_realsense.yaml`이다. Validator는
serial placeholder, mock serial gate, sync mode/policy 조합, reconnect disabled
state, required serial, positive dimensions/FPS, supported formats, POSIX shm
name, loopback metadata bind, shared-memory capacity를 검사한다.
