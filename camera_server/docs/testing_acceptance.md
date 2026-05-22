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

## 3. Three camera capture test

Run head color 1280x720 30 FPS and left_wrist/right_wrist color 640x360 30 FPS.

Acceptance:

- all 3 cameras maintain 30 FPS
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

## 10. Definition of Done for initial milestone

Initial milestone is done when:

```text
- head RealSense runs at 1280x720 color 30 FPS and both wrist RealSense cameras run at 640x360 color 30 FPS
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
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j2
ctest --test-dir build --output-on-failure
./build/camera_server --config config/mock_triple_realsense.yaml --run-seconds 2
python3 tools/inspect_shm.py --shm /camera_server_frames_test
```

Hardware acceptance(실제 RealSense 3대 10분, Docker USB permission, disconnect test)는 실제 장비가 연결된 host에서 `config/triple_realsense_640x360.yaml` 또는 명시 승인된 `config/triple_realsense_640x480.yaml` variant를 복사하고 serial placeholder를 채운 뒤 수행해야 한다.
The gated operator procedure and evidence template live in
`docs/hardware_acceptance_runbook.md`.
