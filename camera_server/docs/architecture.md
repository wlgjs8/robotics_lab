# camera_server architecture

## 1. High-level architecture

```text
Intel RealSense x 3
  ├── head
  ├── left_wrist
  └── right_wrist
        │
        ▼
camera_server
  ├── CameraManager
  ├── RealsenseDevice x 3
  ├── FrameSynchronizerSet (consumer-specific groups)
  ├── SharedMemoryRingBuffer
  ├── MetadataPublisher, ZeroMQ PUB
  ├── Recorder, optional async writer
  └── HealthMonitor
        │
        ├── shared memory image ring
        │
        └── ZMQ metadata topic
              │
              ▼
policy_runner
  ├── metadata subscriber
  ├── shared memory reader
  ├── rb_servo_server state subscriber
  └── policy inference
```

## 2. Process boundary

`camera_server` must not control robots. It must not send `servo_j`, joint targets, or policy actions.

`camera_server` owns:

- RealSense device discovery and serial matching
- stream start/stop
- frame timestamping
- shared memory frame write
- metadata publication
- frame drop detection
- health reporting
- optional recording

`policy_runner` owns:

- selecting latest usable synchronized frame bundle
- reading images from shared memory
- preprocessing for model input
- inference
- sending actions to `rb_servo_server`

`rb_servo_server` owns:

- robot state
- safety state machine
- `servo_j` command streaming
- command timeout/fault logic

## 3. Runtime modes

```text
capture_only
  - RealSense capture + metadata publish
  - no disk recording
  - low-latency policy path only

record
  - capture + metadata publish + async recording

replay
  - read previously recorded episode
  - publish metadata and shared memory frames as if live

diagnostic
  - capture + health monitor + drop statistics
  - useful for USB/bandwidth tests
```

## 4. Thread model

Recommended threads:

```text
main thread
  - config load
  - signal handling
  - start/stop lifecycle

librealsense callback threads
  - timestamp frame
  - retain the owning `rs2::frame` in a bounded drop-oldest queue
  - return immediately

per-camera processing threads
  - copy frame streams into independent shared-memory rings
  - push small metadata objects to the synchronizer thread
  - never write to disk
  - never run inference

synchronizer thread
  - collect per-camera frame metadata
  - build latest complete FrameBundle
  - publish metadata

recorder thread pool, optional
  - encode/write frames and metadata
  - bounded queues

health monitor thread
  - print/publish FPS, drop counts, queue depth, USB errors, sync skew
```

## 5. Non-negotiable callback rule

The RealSense capture callback must remain lightweight.

Allowed in the librealsense callback:

- record `CLOCK_MONOTONIC_RAW` or `CLOCK_MONOTONIC` timestamp
- increment atomic counters
- enqueue an owning frame handle to the bounded per-camera queue

Forbidden in callback:

- PNG/JPEG encoding
- blocking disk writes
- network transmission of full image payload
- model inference
- `cv::imshow`
- heap allocation per frame, unless proven harmless
- long mutex waits

## 6. Failure handling

On any camera failure:

- do not crash silently
- publish health status
- keep other cameras alive if possible
- mark frame bundle as incomplete
- increment error counter
- optionally attempt reconnect according to config

`policy_runner` must be able to detect stale or incomplete camera bundles and avoid using them.

## 구현 매핑

이 아키텍처는 현재 코드에서 다음 컴포넌트로 구현된다. `CameraManager`가 lifecycle과 stats를 소유하고, `RealSenseDevice`는 librealsense callback에서 owning frame handle만 bounded queue에 넣은 뒤 per-camera worker에서 SHM copy를 수행한다. SHM 쓰기는 stream ring별 mutex를 사용해 서로 다른 카메라/스트림이 직렬화되지 않는다. `FrameSynchronizerSet`은 consumer별 독립 bundle group을 만들며 active profile은 legacy full-rig, policy wrist RGB-D, stereo head group을 각각 발행한다. `Recorder`는 별도 bounded queue를 사용하므로 policy shared-memory path를 막지 않는다.
