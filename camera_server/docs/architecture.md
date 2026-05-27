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
  ├── FrameSynchronizer
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

camera callback threads, librealsense-owned or wrapper-owned
  - timestamp frame
  - acquire/copy frame into shared memory slot
  - push frame metadata to synchronizer
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

Allowed in callback:

- record `CLOCK_MONOTONIC_RAW` or `CLOCK_MONOTONIC` timestamp
- copy frame bytes into preallocated shared memory slot
- capture frame number and RealSense metadata
- increment atomic counters
- push small metadata object to an internal queue

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

이 아키텍처는 현재 코드에서 다음 컴포넌트로 구현된다. `CameraManager`가 lifecycle과 stats를 소유하고, `RealsenseDevice`/mock backend가 capture callback을 제공하며, callback path는 timestamp capture, raw byte copy to shared memory, small metadata handoff만 수행한다. `FrameSynchronizer`는 master camera frame cadence에서만 bundle 생성을 시도하며 software mode는 nearest timestamp, hardware mode는 frame_number 기준으로 required streams를 선택한다. `Recorder`는 별도 bounded queue를 사용하므로 policy shared-memory path를 막지 않는다.
