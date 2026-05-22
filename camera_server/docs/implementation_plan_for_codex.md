# Codex Implementation Plan: camera_server

## Mission

Implement `camera_server`, a Docker-deployable RealSense 3-camera server that captures 3 cameras at 30 FPS, writes image frames to a shared memory ring buffer, and publishes frame/bundle metadata through ZeroMQ.

The server is independent from `rb_servo_server`. It must not control the robot.

## Required initial features

1. Load YAML config.
2. Open RealSense devices by serial number.
3. Capture 3 cameras at 30 FPS.
4. Write color frames to POSIX shared memory ring buffer.
5. Publish latest synchronized `FrameBundle` metadata over ZeroMQ.
6. Detect frame drops by frame number gaps.
7. Publish health message once per second.
8. Provide a sample subscriber that reads latest bundle from shared memory.
9. Provide Dockerfile and docker-compose example using container name `camera_server`.

Depth support may be included but can be disabled by default for wrist cameras.

## Suggested source layout

```text
camera_server/
├── CMakeLists.txt
├── config/
│   └── triple_realsense.yaml
├── docs/
│   └── *.md
├── include/
│   └── camera_server/
│       ├── core/
│       │   ├── types.hpp
│       │   ├── clock.hpp
│       │   └── bounded_queue.hpp
│       ├── config/
│       │   └── config.hpp
│       ├── camera/
│       │   ├── realsense_device.hpp
│       │   └── camera_manager.hpp
│       ├── shm/
│       │   ├── shared_memory_ring.hpp
│       │   └── shm_layout.hpp
│       ├── sync/
│       │   └── frame_synchronizer.hpp
│       ├── publish/
│       │   └── metadata_publisher.hpp
│       ├── recording/
│       │   └── recorder.hpp
│       └── health/
│           └── health_monitor.hpp
├── src/
│   ├── main.cpp
│   ├── core/
│   ├── config/
│   ├── camera/
│   ├── shm/
│   ├── sync/
│   ├── publish/
│   ├── recording/
│   └── health/
└── tools/
    ├── read_latest_bundle.py
    ├── print_camera_health.py
    └── inspect_shm.py
```

## Core data types

Implement equivalent types:

```cpp
struct CameraStreamConfig {
    bool enabled;
    int width;
    int height;
    int fps;
    std::string format;
};

struct CameraConfig {
    std::string name;
    std::string serial;
    bool required;
    CameraStreamConfig color;
    CameraStreamConfig depth;
};

struct FrameMeta {
    std::string camera_name;
    std::string serial;
    std::string stream;
    uint64_t frame_number;
    uint64_t host_arrival_time_ns;
    uint64_t sensor_timestamp_ns;
    double realsense_timestamp_ms;
    int width;
    int height;
    int stride_bytes;
    std::string format;
    std::string shm_name;
    std::string ring_name;
    uint32_t slot_index;
    uint64_t shm_offset;
    uint64_t size_bytes;
    bool valid;
};

struct FrameBundleMeta {
    uint64_t bundle_seq;
    uint64_t bundle_time_ns;
    bool hardware_synced;
    std::string sync_policy;
    double max_time_diff_ms;
    bool complete;
    std::map<std::string, FrameMeta> frames;
};
```

## Shared memory requirements

- Use POSIX `shm_open`, `ftruncate`, `mmap`.
- Use fixed-size ring slots.
- Use slot headers with sequence counters to avoid torn reads.
- `camera_server` creates and initializes shared memory.
- subscribers only open existing shared memory.
- include magic/version in header.

## Metadata requirements

- Use ZeroMQ PUB socket.
- Publish `camera.bundle` for complete latest bundles.
- Publish `camera.health` once per second.
- JSON is acceptable for metadata.

## Sync requirements

Initial implementation may use software sync:

- maintain recent frame metadata per required stream
- build bundle by nearest timestamp
- reject or skip bundle if `max_time_diff_ms` exceeds config

Hardware sync fields should exist in config but can be TODO initially.

## Recording requirements

Initial implementation may keep recording disabled by default.

If implemented:

- use async writer threads
- bounded queue
- raw file output
- metadata JSONL
- never block capture callback

## Acceptance tests

Codex must provide at least:

1. build test
2. config parse test or startup validation
3. shared memory layout initialization test
4. mock frame writer/reader test if RealSense is unavailable
5. metadata publisher smoke test
6. sample subscriber that prints bundle FPS and frame numbers

## Important safety/performance rules

- Do not send full image payload over ZMQ.
- Do not write to disk in capture callback.
- Do not run inference inside camera_server.
- Do not let recorder backpressure block frame capture.
- Do not identify cameras by enumeration order.
- Do not ignore frame drops; count and publish them.
- Do not use Docker default 64 MB `/dev/shm`.

## Initial run commands

```bash
cmake -S . -B build
cmake --build build -j
./build/camera_server --config config/triple_realsense.yaml
python3 tools/read_latest_bundle.py --metadata tcp://127.0.0.1:5600 --shm /camera_server_frames
```

## Definition of Done

The initial implementation is complete when:

- `camera_server` starts with config
- 3 cameras are opened by serial
- 30 FPS color frames are captured
- image bytes appear in shared memory
- `camera.bundle` metadata is published
- sample subscriber reads valid images
- health message reports FPS and drop counters
- Docker compose file starts a container named `camera_server`

## 구현 완료 기준 매핑

이 계획의 suggested layout은 현재 repository에 생성되어 있다. Acceptance tests는 `tests/test_camera_server.cpp`가 config parse, shared memory writer/reader consistency, bundle sequence/schema/drop counters, health schema를 검증하고, `./build/camera_server --config config/mock_triple_realsense.yaml --run-seconds 2`가 3-camera 30 FPS mock smoke를 검증한다.
