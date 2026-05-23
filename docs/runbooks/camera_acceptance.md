# Camera Acceptance Runbook

This runbook is the source-of-truth acceptance checklist for real three-camera
operation. It is a hardware acceptance workflow, not a code-only regression
gate. Do not use it to infer real robot or real motion readiness.

## Entry Gate

Before starting RealSense capture, confirm and record:

- Human approval for this exact real-camera session.
- Three reserved cameras: one D435f head camera and two D405 wrist cameras.
- USB access, udev rules, shared memory sizing, and storage target are approved
  for the test host.
- The camera server endpoints are isolated from production policy consumers.
- A local config copy was made from `camera_server/config/triple_realsense.yaml`
  or one of the explicit profile variants, and all serial placeholders were
  replaced with approved physical serials.
- `reconnect.enabled: false`; reconnect is not accepted until the runtime
  implementation exists.

Record:

```text
approval_owner:
approval_time:
test_host:
operator:
config_path:
metadata_endpoint:
shared_memory_name:
storage_path:
retention_plan:
production_isolation_notes:
```

## Dependency Profiles

Hardware-free profile:

- Purpose: local mock/stub builds and tests.
- Requires: CMake, C++17 compiler, Python 3, `yaml-cpp`, `nlohmann_json`.
- Check:

```bash
bash scripts/check_deps.sh --profile hardware-free
```

Real-camera profile:

- Purpose: RealSense camera capture and metadata publishing.
- Adds: `librealsense2-dev`, RealSense udev rules/tools, `libzmq3-dev`, USB
  device access, and shared memory permissions.
- Check:

```bash
bash scripts/check_deps.sh --profile real-camera
```

Real-robot profile:

- Purpose: RB3-730 controller SDK readiness only.
- Adds: rbpodo SDK/package.
- Check:

```bash
bash scripts/check_deps.sh --profile real-robot
```

This does not open `RB_ALLOW_REAL_ROBOT`, `RB_ALLOW_REAL_MOTION`, or
`RB_ALLOW_REAL_CARTESIAN`.

Kinematics profile:

- Purpose: FK/IK and TCP pose acceptance paths.
- Adds: Eigen3 and Pinocchio.
- Check:

```bash
bash scripts/check_deps.sh --profile kinematics
```

## Camera Profiles

Canonical profile:

```text
head D435f:  1280x720 RGB, 30 FPS
left D405:   640x360 RGB, 30 FPS
right D405:  640x360 RGB, 30 FPS
depth:       disabled
```

Use `camera_server/config/triple_realsense_640x360.yaml` or
`camera_server/config/triple_realsense.yaml` as the source template.

Optional wrist variant:

```text
head D435f:  1280x720 RGB, 30 FPS
left D405:   640x480 RGB, 30 FPS
right D405:  640x480 RGB, 30 FPS
depth:       disabled
```

Use `camera_server/config/triple_realsense_640x480.yaml` only when the session
explicitly accepts the larger D405 frame size.

## Serial Identification

Run before editing the local accepted config:

```bash
rs-enumerate-devices
rs-enumerate-devices -s
lsusb
```

Record:

```text
head_serial:
left_wrist_serial:
right_wrist_serial:
firmware_versions:
usb_bus_topology:
configured_sync_mode:
sync_cable_or_trigger_notes:
```

Acceptance:

- Each configured serial maps to the expected physical camera position.
- No checked-in template with `REPLACE_*`, `TODO`, `CHANGEME`, `UNKNOWN`, empty
  required serials, or `MOCK_*` real-mode serials is run directly.
- Firmware versions and USB topology are attached to the evidence.

## Stability Runs

Build and start with the approved local config copy. Keep metadata and shared
memory names non-production unless the approved session explicitly says
otherwise.

```bash
cmake -S camera_server -B camera_server/build/real_camera_acceptance -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build camera_server/build/real_camera_acceptance -j2
camera_server/build/real_camera_acceptance/camera_server --config <approved-config> --run-seconds 600
```

Run three stability windows:

- 10 minutes: initial acceptance.
- 30 minutes: operator workflow confidence.
- 60 minutes: extended soak before policy use.

For each window, record:

```text
window_minutes:
start_time:
end_time:
head_fps_mean:
left_wrist_fps_mean:
right_wrist_fps_mean:
head_drop_count:
left_wrist_drop_count:
right_wrist_drop_count:
complete_bundle_count:
incomplete_bundle_count:
bundle_publish_rate_hz_mean:
bundle_publish_rate_hz_min:
max_bundle_skew_ms_mean:
max_bundle_skew_ms_max:
shared_memory_reader_result:
metadata_health_json_path:
server_log_path:
```

Acceptance:

- Head D435f holds 1280x720 at approximately 30 FPS.
- Both D405 wrists hold the selected 640x360 canonical profile, or the approved
  640x480 variant, at approximately 30 FPS.
- Drop count remains zero, or every drop has a timestamp, affected camera,
  suspected cause, and explicit operator acceptance.
- Bundle publish rate remains approximately 30 Hz.
- Maximum bundle skew remains inside `sync.max_bundle_time_diff_ms`, or every
  excursion is explained and accepted.
- Shared memory reads pass seqlock validation.

## USB Disconnect And Restart

Run only after the operator identifies the camera that may be unplugged. Do not
unplug any device used by production workloads.

Record:

```text
camera_unplugged:
disconnect_time:
health_event_time:
server_crashed: yes/no
bundle_behavior_after_disconnect:
metadata_error_observed:
restart_command:
restart_success: yes/no
post_restart_serial_mapping_verified: yes/no
post_restart_bundle_rate_hz:
residual_errors:
```

Acceptance:

- Disconnect produces explicit health/error telemetry.
- The server does not crash silently.
- Bundles become incomplete or stop according to config; stale frames are not
  presented as fresh complete bundles.
- A clean server restart with the same approved serial mapping restores complete
  bundle publication.
- Reconnect remains disabled unless a later accepted implementation changes
  this runbook.

## Policy Runner Readiness Evidence

Policy code may run joint-only sources without cameras. Any future camera
observation source must declare `requires_camera: true` and a
`camera_stale_timeout_sec`; geometry-dependent camera sources must also declare
`requires_camera_geometry: true`.

Acceptance:

- Joint-only `policy_runner` actions remain allowed without camera readiness.
- Camera-dependent actions are blocked when camera readiness is absent.
- Camera-dependent actions are blocked when the latest camera observation is
  older than `camera_stale_timeout_sec`.
- Camera geometry remains blocked unless measured camera intrinsics/extrinsics
  are available and accepted for the configured mode.

## Artifact List

Attach these artifacts before marking real-camera acceptance complete:

- Approval record and operator notes.
- Exact dependency profile checks and output.
- Exact local config file path and config diff from the checked-in template.
- `rs-enumerate-devices`, `rs-enumerate-devices -s`, and `lsusb` output.
- 10/30/60-minute server logs.
- Health JSON snapshots or time series.
- Drop, skew, and bundle-rate metrics for each stability window.
- Shared memory reader result.
- USB disconnect and restart evidence.
- Residual risks and follow-up tasks.

Stop condition: if any required camera cannot hold the accepted profile, bundle
rate/skew exceeds accepted limits without explanation, shared memory reads fail,
or disconnect/restart behavior is silent or unsafe, keep the hardware gate
blocked and attach the evidence to a focused follow-up.
