# Camera Acceptance Runbook

This runbook defines real camera acceptance for `camera_server`. It is separate
from robot motion acceptance.

## Camera Profile

Canonical stack profile (`make cam-up`, default
`CAMERA_CONFIG=/app/config/d435_head_1280x720.yaml`):

- head: Intel RealSense D435, color 1280x720@30 plus IR left/right
  1280x720@30 for the stereo worker
- left wrist: Intel RealSense D405, color 640x480@30 plus depth 640x480@30
- right wrist: Intel RealSense D405, color 640x480@30 plus depth 640x480@30
- stereo worker: launched in the same `camera_server` container by
  `camera_server/stereo_worker/run_all.sh`

Supported explicit variants:

- `CAMERA_CONFIG=/app/config/head_wrists.yaml` for the older head/wrist profile
  with 640x480 head IR intrinsics.
- `CAMERA_CONFIG=/app/config/quad_realsense_fisheye.yaml` for the real-policy
  fisheye deploy profile: two D405 RealSense cameras plus two UVC fisheye
  cameras in the same bundle.

Any other profile must be documented in the config and acceptance record.

## Safety Boundary

Camera acceptance does not enable robot motion. Do not modify robot motion
configs or start robot command sources during this runbook.

## Config Requirements

Real camera configs must not contain placeholder serials:

```text
REPLACE_*
TODO
CHANGEME
UNKNOWN
```

Mock serials such as `MOCK_*` are only valid when mock/simulated cameras are enabled.

Tracked real-camera profiles enable per-camera reconnect. The current
high-bandwidth head+wrist profile uses a 5 s frame timeout. Verify that
unplugging one approved test camera leaves every other
camera streaming and that health reports a disconnect, attempt, and success.

## Dependency Preflight

Run:

```bash
./scripts/check_deps.sh --profile real-camera
```

Real camera work may require RealSense packages, udev rules, USB permission, and host access to `/dev/bus/usb`.

The tracked camera image pins librealsense SDK `2.58.1`, matching the deployed
D435 firmware `5.17.3.10`. Before acceptance, both D405 devices must report
firmware `5.17.0.10`. Startup fails closed for an older SDK, USB2 connection,
older D405 firmware, or mismatched D405 firmware pair.

Run native V4L2 first. If it reproduces USB `-71`/disconnect after the version
alignment, apply the D405-only autosuspend rule documented in
`camera_server/docs/docker_deployment.md` and repeat. Use
`LIBREALSENSE_BACKEND=rsusb` only as the final software-only diagnostic.

## Acceptance Duration

Minimum recommended test durations:

- short smoke: 2 minutes
- development acceptance: 10 minutes
- pre-policy acceptance: 30 to 60 minutes

For the shared `4-2.x` D405 path, use this fixed ladder:

Use `make cam-down` followed by `make cam-up ...` between stages. Do not use
`docker restart camera_server`: it can relaunch before the previous RealSense
pipelines have released their UVC interfaces, leaving a camera at 0 FPS.

1. dual D405-only native, 10 minutes;
2. full D435 + dual D405 native, 30 minutes;
3. if needed, repeat after the D405-only autosuspend rule;
4. if still failing, repeat with the pinned RSUSB image.

Each stage requires 29--31 FPS per enabled stream, frame age below 100 ms,
policy bundle rate at least 29 Hz, no increasing drops after warm-up, and no
kernel USB `-71`, disconnect, or re-enumeration. Failure of the RSUSB stage is
the stop condition for software-only remediation and requires physical USB
root-path separation.

## Metrics To Record

Record for each camera:

- serial
- model
- resolution
- FPS mean/min
- frame drops
- timestamp jitter
- USB disconnects
- stream restart behavior

Record for bundles:

- complete bundle rate
- incomplete bundle count
- max bundle skew
- shared-memory reader result
- metadata subscriber result
- CPU/memory usage, if available

## Policy Runner Readiness

Camera-dependent policy sources must declare or enforce:

```yaml
requires_camera: true
camera_stale_timeout_sec: <seconds>
```

Camera-geometry-dependent sources must also declare:

```yaml
requires_camera_geometry: true
```

`flow-infer` real-policy configs consume the ZMQ camera bundle on port 5600 and
perform source-level camera freshness checks for the expected bundle keys. If
camera metadata is stale, incomplete, or missing, policy_runner must fail closed
and stop sending motion commands.

## Acceptance Record Template

```yaml
camera_acceptance_id:
date:
operator:
repo_commit:
config_file:
profile:
  head: D435 RGB 1280x720@30 + IR stereo 1280x720@30
  left_wrist: D405 RGB-D 640x480@30
  right_wrist: D405 RGB-D 640x480@30
serials:
  head:
  left_wrist:
  right_wrist:
duration_min:
head_fps_mean:
left_wrist_fps_mean:
right_wrist_fps_mean:
head_drop_count:
left_wrist_drop_count:
right_wrist_drop_count:
complete_bundle_count:
incomplete_bundle_count:
bundle_publish_rate_hz_mean:
max_bundle_skew_ms:
shared_memory_reader_result:
metadata_health_json_path:
server_log_path:
notes:
```

## Pass Criteria

A run is acceptable only if:

- all configured cameras are found by serial
- all streams maintain expected FPS within tolerance
- complete bundles are produced at expected rate
- shared-memory and metadata consumers can read data
- no silent placeholder serials are accepted
- failures are visible in health metadata

## Not Robot Acceptance

Passing camera acceptance is not permission to move the robot.
