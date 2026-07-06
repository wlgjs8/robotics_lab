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

`reconnect.enabled: true` must remain rejected until reconnect is implemented and accepted.

## Dependency Preflight

Run:

```bash
./scripts/check_deps.sh --profile real-camera
```

Real camera work may require RealSense packages, udev rules, USB permission, and host access to `/dev/bus/usb`.

## Acceptance Duration

Minimum recommended test durations:

- short smoke: 2 minutes
- development acceptance: 10 minutes
- pre-policy acceptance: 30 to 60 minutes

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
