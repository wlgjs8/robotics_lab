# Hardware Acceptance Runbook

This runbook is for the RealSense hardware session gated by kanban task
`t_8f430205`. It must not be used during code-only or mock-only validation.
The repository-level source for current three-camera acceptance criteria is
`../../docs/runbooks/camera_acceptance.md`; keep this component runbook aligned
with that checklist.

## 1. Entry gate

Do not start `camera_server`, Docker with USB access, RealSense capture, shared
memory production endpoints, or recording until all items below are true.

- Human approval for this exact hardware session is recorded.
- Three physical RealSense cameras are connected and reserved for this test:
  one D435f head camera and two D405 wrist cameras.
- USB and udev access are approved for the test host.
- Storage target, retention period, and cleanup owner are recorded.
- Test endpoints are isolated from production policy and metadata consumers.
- A copied config based on `config/triple_realsense_640x360.yaml` or the
  explicitly approved `config/triple_realsense_640x480.yaml` variant contains
  the approved serial mapping. Do not run the checked-in template directly;
  `REPLACE_*`, `TODO`, `CHANGEME`, `UNKNOWN`, empty required serials, and
  `MOCK_*` serials in real mode fail validation.
- First-wave code-only and mock-only gates have passed.

Record:

```text
approval_owner:
approval_time:
test_host:
operator:
storage_path:
retention_plan:
metadata_endpoint:
shared_memory_name:
production_isolation_notes:
```

## 1.1 Dependency profiles

Hardware-free config/build validation does not require RealSense devices or
ZeroMQ headers when forced to mock/stub mode:

```bash
sudo apt install cmake build-essential libyaml-cpp-dev
cmake -S camera_server -B camera_server/build/hardware_free_gate \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCAMERA_SERVER_FORCE_MOCK_CAMERA=ON \
  -DCAMERA_SERVER_FORCE_ZMQ_STUB=ON \
  -DCAMERA_SERVER_BUILD_TESTS=ON
cmake --build camera_server/build/hardware_free_gate -j
ctest --test-dir camera_server/build/hardware_free_gate --output-on-failure
```

Real-camera acceptance additionally needs the metadata backend and RealSense SDK:

```bash
sudo apt install libzmq3-dev
# Install librealsense2-dev from the Intel RealSense package source approved for the host OS.
```

Use `rs-enumerate-devices` for P0 serial discovery. A dedicated
`camera_server/tools/list_realsense_devices` helper is still a future task.
Real-camera Docker startup must use the explicit `real_camera` profile or the
`make camera-real-up` wrapper; this grants camera USB/shared-memory access only
and does not enable robot connection or robot motion gates.

```bash
docker compose --profile real_camera up --build camera_server
# or, from the repository root:
make camera-real-up
```

## 2. Hardware inventory

Capture the host and camera inventory before starting the server.

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
usb_topology_notes:
configured_sync_mode:
sync_cable_or_trigger_notes:
```

Acceptance:

- Each configured serial maps to the expected physical camera.
- Firmware and USB topology are recorded.
- Hardware sync claims are documented as observed wiring/configuration, or the
  run is explicitly marked software-sync-only.

## 3. Baseline triple-camera run

Run from the `camera_server` repository root. Use an approved per-session config
copy with a non-production shared memory name and metadata port unless the
approved session explicitly authorizes the defaults.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j2
./build/camera_server --config config/triple_realsense_640x360.approved.yaml --run-seconds 600
python3 tools/print_camera_health.py --once
python3 tools/read_latest_bundle.py --once --shm /camera_server_frames
```

If the approved config changes `shared_memory.name`, use that value for
`--shm`.

Record:

```text
artifact_dir: artifacts/camera_acceptance/<YYYYMMDD-HHMMSS>/10min/
start_time:
end_time:
run_seconds:
head_fps:
left_wrist_fps:
right_wrist_fps:
frame_number_gap_drop_count:
complete_bundle_count:
incomplete_bundle_count:
bundle_publish_rate_hz:
max_time_diff_ms_mean:
max_time_diff_ms_max:
shared_memory_read_result:
policy_reader_result:
health_json_path:
server_log_path:
```

Acceptance:

- Head color is 1280x720 at approximately 30 FPS.
- Left and right wrist color are 640x360 at approximately 30 FPS.
- Drop counts are zero for 10 minutes, or every drop is reported with timing and
  suspected cause.
- Bundle publish rate is approximately 30 FPS.
- Shared memory seqlock reads succeed.
- `tools/read_latest_bundle.py` can read the latest complete bundle.

## 4. Recording run

Enable recording only after the storage target is approved. Keep policy input on
shared memory and record through the async writer path.

Record the exact config diff or copied config path used for recording.

```bash
./build/camera_server --config config/triple_realsense_recording.approved.yaml --run-seconds 180
```

Record:

```text
episode_path:
recording_duration_sec:
metadata_jsonl_lines:
saved_frame_count:
recorder_queue_depth_max:
dropped_by_recorder_queue:
bytes_written_per_sec_mean:
bytes_written_per_sec_max:
disk_free_before:
disk_free_after:
recording_health_json_path:
```

Acceptance:

- `camera_metadata.jsonl` line count matches saved frame count.
- Recorder queue does not grow without bound.
- `dropped_by_recorder_queue` is zero, or drops are explicitly accepted with a
  throughput bottleneck note.
- Disk write errors are zero.

## 5. Disconnect run

Perform this only when the operator confirms which camera may be unplugged.
Never unplug a device used by production workloads.

`reconnect.enabled: true` is not implemented and fails config validation in P0.
This run observes disconnect health behavior only; leave reconnect disabled.

Record:

```text
camera_unplugged:
disconnect_time:
health_event_time:
server_crashed: yes/no
bundle_behavior: incomplete/dropped/other
reconnect_enabled: no
residual_errors:
```

Acceptance:

- Server does not crash unexpectedly.
- Health reports the disconnect.
- Bundles become incomplete or are dropped according to config.
- Reconnect remains disabled; any reconnect test is a follow-up after the
  implementation lands.

## 6. Docker and policy-reader run

Run this only after approving privileged Docker or a narrower equivalent USB
device mapping.

```bash
docker compose up camera_server
python3 tools/read_latest_bundle.py --once --shm /camera_server_frames
```

Record:

```text
docker_compose_file:
usb_access_mode:
ipc_mode:
shm_size:
policy_reader_container_or_host:
policy_reader_result:
permission_errors:
```

Acceptance:

- `policy_runner` or the sample reader can open the same POSIX shared memory.
- Metadata is received.
- Valid image bytes are read with seqlock validation.
- USB and shared memory permission errors are zero.

## 7. Completion report

Attach these artifacts to the task or hardware log before marking the hardware
gate complete.

- Human approval record.
- Commands run and exact config files used.
- Hardware inventory.
- Health JSON and server logs.
- Observed rate, drop, skew, and latency metrics.
- Recording evidence and storage cleanup decision.
- Disconnect/reconnect observations.
- Policy-reader compatibility result.
- Residual risks and recommended follow-up cards.

Stop condition:

- If any required camera cannot hold the target profile, shared memory reads
  fail, recording loses unaccepted frames, or disconnect handling crashes the
  server, keep the task blocked and create a focused follow-up with the captured
  evidence.
