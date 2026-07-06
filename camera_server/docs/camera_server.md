# camera_server Codex Task Spec

You are implementing `camera_server`.

## Goal

Build a C++ server that captures three Intel RealSense cameras at 30 FPS and provides low-latency image access to `policy_runner` using:

```text
Shared Memory Ring Buffer + ZeroMQ metadata
```

The server runs as Docker container name:

```text
camera_server
```

## Scope

Implement:

- RealSense 3-camera capture by serial
- shared memory image ring buffer
- ZeroMQ metadata publisher
- software synchronized frame bundles
- health monitoring
- frame drop detection
- optional recording skeleton
- sample subscriber tool
- Dockerfile/docker-compose example

Do not implement:

- robot control
- VLA inference
- `rb_servo_server` command sending
- heavy image processing inside capture callback

## Required behavior

1. Start from YAML config.
2. Fail startup if required camera is missing.
3. Capture configured streams at 30 FPS.
4. For each frame, immediately capture host timestamp.
5. Copy raw frame bytes into shared memory slot.
6. Publish metadata, not image payload, over ZMQ.
7. Build `FrameBundle` from head/left_wrist/right_wrist frames.
8. Publish `camera.bundle` only from the configured master camera cadence, using complete bundles by default.
9. Publish `camera.health` once per second.
10. Track frame number gaps and internal queue drops.
11. Provide `tools/read_latest_bundle.py` that subscribes metadata and reads shared memory.

## Performance rules

- Callback path must be lightweight.
- No disk write in callback.
- No image compression in policy path.
- No full-frame ZMQ payloads.
- Use preallocated shared memory.
- Use bounded queues for recorder.

## Docker rules

Docker compose should include:

```yaml
container_name: camera_server
ipc: host
shm_size: "2gb"
network_mode: host
privileged: true
```

and USB access to RealSense devices.

## Current default profile

The current default `make cam-up` profile is not RGB-only:

```text
head.color          1280x720 30 FPS
head.ir_left/right  1280x720 30 FPS
left_realsense      640x480 color + depth 30 FPS
right_realsense     640x480 color + depth 30 FPS
```

Older RGB-only milestone docs map to the retained `triple_realsense*` profiles
and are not the default stack.

## Tests

Add tests or tools to verify:

- shared memory writer/reader consistency
- metadata schema validity
- bundle sequence increments
- frame drop counters
- health publish
- 10-minute 3-camera diagnostic run

## Documentation

Keep `docs/*.md` updated. The design source of truth is this docs directory.

## 현재 구현 상태

Camera-server plumbing 산출물은 repository root에 구현되어 있다. RealSense가 설치된 환경에서는 serial로 required camera를 확인하고 없으면 startup 실패한다. Hardware가 없는 개발/CI 환경은 `--simulate` 또는 `server.simulate_cameras: true`로 mock capture를 실행해 shared memory, bundle metadata, health, drop counter 경로를 검증한다.
