# Testing and Acceptance Criteria

## 1. Build test

```bash
cmake -S . -B build
cmake --build build -j
```

No warnings preferred. No errors allowed.

## 2. Single camera smoke test

Run only head camera at 1280x720 color 30 FPS.

Acceptance:

- device opens by serial
- frame metadata publishes
- shared memory slot updates
- no frame drops for 2 minutes

## 3. Camera-rig capture test

Run the approved rig profile. For the current default, that is head color
1280x720 30 FPS plus head IR stereo and left/right wrist D405 color+depth
640x480 30 FPS.

Acceptance:

- all required cameras maintain 30 FPS
- zero frame number gaps for 10 minutes, or drop count explicitly reported
- bundle metadata publishes at approximately 30 FPS
- policy_runner sample can read latest bundle from shared memory

## 4. RGB-D bandwidth test

Enable depth according to target config.

Acceptance:

- no USB disconnect
- drop counters stable
- CPU usage acceptable
- writer queue does not grow unbounded if recording enabled

## 5. Shared memory consistency test

Implement a test subscriber that repeatedly reads slots using seqlock.

Acceptance:

- no inconsistent reads
- image dimensions and size match metadata
- slot sequence increases monotonically

## 6. Metadata test

Subscriber validates JSON schema.

Acceptance:

- `camera.bundle` contains configured streams
- `complete` is true when all required streams are available
- `max_time_diff_ms` is finite
- frame numbers increase

## 7. Recording test

Enable recording for 3 minutes.

Acceptance:

- files written to expected layout
- metadata JSONL line count matches saved frame count
- recorder queue does not overflow under target profile
- disk write errors are zero

## 8. Disconnect test

Unplug one camera or simulate failure.

Acceptance:

- server does not crash unexpectedly
- health event reports disconnect
- bundles become incomplete or are dropped according to config
- reconnect remains disabled in P0; `reconnect.enabled: true` must fail config
  validation until reconnect is implemented

## 9. Docker test

Run `camera_server` and `policy_runner` in separate containers with `ipc: host`.

Acceptance:

- policy_runner can open shared memory
- policy_runner receives metadata
- policy_runner reads valid images
- no permission errors on USB devices

## 10. Definition of Done for camera-server plumbing

Camera-server plumbing is done when:

```text
- head RealSense and both wrist RealSense cameras match the approved profile at 30 FPS
- shared memory image ring works
- ZMQ camera.bundle metadata works
- policy_runner sample reads latest bundle
- frame drop counters are implemented
- health topic is published
- docs and config are included
```

## 자동 검증 명령

현재 repository에서 실행한 기본 검증은 다음과 같다.

```bash
cmake -S camera_server -B camera_server/build/hardware_free_gate \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCAMERA_SERVER_FORCE_MOCK_CAMERA=ON \
  -DCAMERA_SERVER_FORCE_ZMQ_STUB=ON \
  -DCAMERA_SERVER_BUILD_TESTS=ON
cmake --build camera_server/build/hardware_free_gate -j
ctest --test-dir camera_server/build/hardware_free_gate --output-on-failure
camera_server/build/hardware_free_gate/camera_server \
  --config camera_server/config/mock_triple_realsense.yaml \
  --run-seconds 5 &
server_pid=$!
sleep 1
python3 camera_server/tools/inspect_shm.py --shm /camera_server_frames_test
wait "${server_pid}"
```

Mock acceptance expects all three mock streams at 64x48 RGB and 30 FPS. Treat
mock drop/skew metrics as schema and plumbing evidence only; they are not real
camera hardware evidence.

Hardware acceptance(실제 camera rig 10분, Docker USB permission, disconnect
test)는 실제 장비가 연결된 host에서 승인된 rig profile을 복사하고 serial/UVC
placeholder를 채운 뒤 수행해야 한다. 현재 기본은
`config/d435_head_1280x720.yaml`이며, `config/head_wrists.yaml`,
`config/quad_realsense_fisheye.yaml`, 또는 legacy `config/triple_realsense*`
variant는 해당 세션에서 명시 승인된 경우에만 사용한다.
The gated operator procedure and evidence template live in
`docs/hardware_acceptance_runbook.md`.
