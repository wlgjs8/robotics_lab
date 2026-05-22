# Hardware Acceptance Runbook

This runbook is for the RealSense hardware session gated by kanban task
`t_8f430205`. It must not be used during code-only or mock-only validation.

## 1. Entry gate

Do not start `camera_server`, Docker with USB access, RealSense capture, shared
memory production endpoints, or recording until all items below are true.

- Human approval for this exact hardware session is recorded.
- Three physical RealSense cameras are connected and reserved for this test.
- USB and udev access are approved for the test host.
- Storage target, retention period, and cleanup owner are recorded.
- Test endpoints are isolated from production policy and metadata consumers.
- `config/triple_realsense.yaml` contains the approved serial mapping.
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
./build/camera_server --config config/triple_realsense.approved.yaml --run-seconds 600
python3 tools/print_camera_health.py --once
python3 tools/read_latest_bundle.py --once --shm /camera_server_frames
```

If the approved config changes `shared_memory.name`, use that value for
`--shm`.

Record:

```text
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

## 5. Disconnect and reconnect run

Perform this only when the operator confirms which camera may be unplugged.
Never unplug a device used by production workloads.

Record:

```text
camera_unplugged:
disconnect_time:
health_event_time:
server_crashed: yes/no
bundle_behavior: incomplete/dropped/other
reconnect_enabled: yes/no
reconnect_time:
post_reconnect_fps:
residual_errors:
```

Acceptance:

- Server does not crash unexpectedly.
- Health reports the disconnect.
- Bundles become incomplete or are dropped according to config.
- Reconnect behavior matches the approved config.

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
