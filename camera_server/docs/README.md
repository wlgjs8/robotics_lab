# camera_server docs

`camera_server`는 `rb_servo_server`와 분리된 RealSense 3대 전용 카메라 서버이다.

목표는 다음과 같다.

1. RealSense 3대를 각각 30 FPS로 안정적으로 capture한다.
2. capture callback에서 host timestamp와 RealSense metadata를 즉시 기록한다.
3. image payload는 POSIX shared memory ring buffer에 저장한다.
4. frame metadata만 ZeroMQ PUB/SUB로 publish한다.
5. `policy_runner`는 metadata를 구독하고 shared memory에서 최신 frame bundle을 직접 읽는다.
6. recording path는 policy low-latency path와 분리한다.
7. frame drop, queue overflow, USB disconnect, timestamp skew를 지속적으로 감시한다.

프로세스 역할은 다음처럼 분리한다.

```text
rb_servo_server
  - Rainbow robot 2대 제어
  - servo_j 100~200 Hz
  - command/state/safety/fault handling
  - image 처리 없음

camera_server
  - RealSense 3대 capture
  - hardware/software sync
  - shared memory latest image publish
  - metadata publish
  - async recording

policy_runner
  - camera_server shared memory에서 최신 image bundle read
  - rb_servo_server state subscribe
  - VLA / imitation inference
  - rb_servo_server로 action command send
```

이 문서 세트는 Codex agent가 `camera_server`를 구현하기 위한 작업 명세로 사용한다.

권장 구현 언어는 C++17 이상이다. Python prototype은 가능하지만, 최종 capture server는 C++를 권장한다.

## 구현된 산출물

- C++17 server: `src/main.cpp`, `include/camera_server/**`, `src/**`
- 기본 설정: `config/triple_realsense.yaml`, mock/CI 설정: `config/mock_triple_realsense.yaml`
- POSIX shared memory ring: `include/camera_server/shm/*`, `src/shm/shared_memory_ring.cpp`
- ZeroMQ metadata publisher with no-ZMQ build fallback: `include/camera_server/publish/metadata_publisher.hpp`, `src/publish/metadata_publisher.cpp`
- RealSense-by-serial backend and mock 30 FPS backend: `src/camera/realsense_device.cpp`
- master-camera-driven nearest timestamp/frame-number synchronizer: `src/sync/frame_synchronizer.cpp`
- sample policy reader and diagnostics: `tools/read_latest_bundle.py`, `tools/print_camera_health.py`, `tools/inspect_shm.py`
- Docker deployment: `Dockerfile`, `docker-compose.yml` with `container_name: camera_server`
- Verification: `tests/test_camera_server.cpp` plus mock smoke run.
- Hardware-gated acceptance procedure: `docs/hardware_acceptance_runbook.md`
