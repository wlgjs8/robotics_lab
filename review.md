# robotics_lab 통합 검토 및 향후 개발 필요사항

작성일: 2026-05-23

## 1. 요약

`robotics_lab`는 현재 네 축으로 구성되어 있다.

| 컴포넌트 | 책임 | 현재 판단 |
| --- | --- | --- |
| `rb_servo_server` | RB3-730 양팔 control layer | mock/rbsim joint servo 구조는 있음. 실제 `RbpodoBackend`, FK/IK, force path는 미완성 |
| `rb_simulator` | 실제 로봇 없이 servo backend를 검증하는 hardware-free simulator | deterministic dual-arm simulator가 있음. 표준 topology는 “단일 simulator process가 left/right arm state를 모두 소유”로 확정 |
| `camera_server` | RealSense 3대 capture, shared memory, metadata publish | mock/hardware-free 구조는 있음. 실제 3-camera 장시간 acceptance, serial config, reconnect, calibration은 남음 |
| `rb_servo_gui` | viser 기반 operator GUI | 동작 구조는 있음. 위치는 top-level `rb_gui`로 분리하는 편이 더 명확함 |

현재 전체 방향은 좋다. control, simulator, camera, GUI가 한 프로세스에 섞이지 않고 분리되어 있다. 다만 아직 “실제 양팔 로봇 + RealSense 3대 + policy loop”를 end-to-end로 운용 가능한 상태는 아니다.

가장 먼저 처리할 P0는 다음이다.

1. `rb_servo_server` hardware-free 검증 게이트 실패 수정
2. `rb_simulator` topology 확정
3. 실제 RB3-730용 `RbpodoBackend` 구현 계획과 acceptance 절차 수립
4. stand/robot/camera frame 계약 확정

## 2. 현재 시스템 구성

```text
robotics_lab/
├── rb_servo_server/   # control layer
├── rb_gui/                         # viser 기반 operator GUI
├── rb_simulator/                  # hardware-free simulator
├── camera_server/                 # 3-RealSense capture server
├── docs/                          # 공통 문서
└── scripts/                       # 공통 검증 스크립트
```

주요 endpoint:

| 종류 | 기본 endpoint | 방향 |
| --- | --- | --- |
| servo command | `udp://127.0.0.1:50010` | 외부/policy/GUI -> `rb_servo_server` |
| servo state | `udp://127.0.0.1:50110` | `rb_servo_server` -> 외부/policy/GUI |
| rbsim control | `tcp://127.0.0.1:50200` | `rb_servo_server` -> `rb_simulator` |
| rbsim admin | `tcp://127.0.0.1:50201` | tests/tools -> `rb_simulator` |
| camera metadata | `tcp://127.0.0.1:5600` ZMQ PUB | `camera_server` -> subscriber |
| camera shared memory | `/camera_server_frames` | `camera_server` -> mmap reader |
| GUI | `http://127.0.0.1:8080` | operator browser |

## 3. 검증 결과

다음 명령을 실행했다.

```bash
RBSIM_SMOKE_MODE=skip bash scripts/hardware_free_validation.sh
```

결과:

| 항목 | 결과 |
| --- | --- |
| `camera_server_tests` | 통과 |
| `rb_servo_gui_unittest` | 통과 |
| `rb_servo_tools_unittest` | 통과 |
| `safety_policy` | 실패 |
| `rbsim_hardware_free_gate` | 실패 |

확인된 실패:

- `safety_policy`: rbsim tracking error fault latch 테스트에서 `previousSentTarget`이 기대한 initial target과 같지 않아 실패했다.
- `rbsim_hardware_free_gate`: send failure latch snapshot을 기다리다 timeout이 났다. 최신 snapshot에는 `fault_latched: true`, `latched_fault_reason: SendFailure`, `left.send_ok: false`가 보였지만, test predicate가 요구한 `command_seq >= send_fail_seq` 조건을 만족하지 못한 것으로 보인다.

의미:

- 현재 `rb_simulator`와 `rb_servo_server`의 hardware-free integration path는 부분 구현 상태다.
- 이 실패가 고쳐지기 전에는 simulator-backed servo path를 안정적인 regression baseline으로 보면 안 된다.

## 4. 컴포넌트별 검토

### 4.1 rb_servo_server

구현된 부분:

- 양팔 같은 tick 기준 servo loop
- `MockBackend`, `RbsimBackend`, `RbpodoBackend` factory 구조
- UDP JSON command receiver와 UDP JSON state publisher
- `JointTarget`, `JointVelocity`, `Hold`, `ArmMotion`, `DisarmMotion`, `EmergencyStop`, `ResetFault` 흐름
- command source allowlist, command sequence, timeout, invalid payload fail-closed 처리
- joint limit, velocity/acceleration clamp, tracking error guard
- servo log CSV와 분석 도구
- state stream에 joint state, send result, timing, fault, mount transform 포함

미완성/스캐폴드:

| 항목 | 상태 | 영향 |
| --- | --- | --- |
| `RbpodoBackend` | `connect`, `readState`, `sendServoJ`, `stop`, `resetFault`가 실질 구현 없이 실패하거나 TODO | 실제 RB3-730 운용 차단 |
| Cartesian FK/IK | `CartesianController`/IK가 throw 또는 deferred | `TcpPoseTarget`, `TcpDeltaStand`, `TcpDeltaLocal` 실제 motion 불가 |
| TCP state fields | state publisher의 `tcp_stand`, `tcp_base`가 `null`, `tcp_deferred: true` | GUI/policy가 TCP pose를 직접 쓸 수 없음 |
| force control | config/controller scaffold는 있으나 active joint path 미연결 | force/admittance 작업 불가 |
| left/right send timing | 현재 순차 send | 고속/동시성 요구 시 timing budget 재검토 필요 |
| gripper | 서브모듈만 있고 servo command path 미통합 | 파지 작업 불가 |

명확히 정의할 것:

- 실제 로봇 startup sequence, IP/port, controller state, speed/servo parameter acceptance
- stand frame 원점/축, left/right base pose 측정 기준
- Cartesian command에서 IK 실패, joint limit clamp, timeout, fault latch를 어떻게 표현할지
- state publisher 주기 20 Hz가 policy/camera alignment에 충분한지
- gripper command를 servo UDP schema에 넣을지 별도 channel로 둘지

### 4.2 rb_simulator

구현된 부분:

- deterministic dual-arm state machine
- JSON Lines TCP control/admin server
- `connect`, `initialize`, `read_state`, `send_servo_j`, `stop`, `reset_fault`
- fault injection: send/read/stop/reset failure, stale state, tracking bias, disconnect 등
- unit tests와 smoke runner
- loopback-only endpoint 기본 정책

확정된 topology:

현재 표준은 하나의 simulator process가 left/right arm state를 모두 소유하는 dual-arm simulator다.

```text
rb_servo_server
  left RbsimBackend  ┐
                     ├── rb_simulator 1 process
  right RbsimBackend ┘       ├── left arm state
                             └── right arm state
```

config도 left/right 모두 같은 `tcp://127.0.0.1:50200` control endpoint를 바라본다. 각 backend request는 `arm: "left"` 또는 `arm: "right"`를 포함하고, simulator가 이를 in-process arm state로 demux한다.

이 선택은 양팔 동시 fault injection, deterministic tick, smoke runner 관리가 쉽기 때문이다. 향후 isolation 요구가 생기면 left/right endpoint 또는 process 분리를 future option으로 열어두되, 그때는 config, protocol, smoke runner 기본값, 문서를 함께 바꿔야 한다.

미구현/제한:

- 물리 엔진 없음
- FK/IK 없음
- gripper/F-T sensor simulation 없음
- mount 정보는 config에는 있으나 simulator motion에는 사용되지 않음
- 실 RB3-730의 정상 응답 지연/지터 profile은 아직 표준화되지 않음

### 4.3 camera_server

구현된 부분:

- RealSense 3대 구성 전제
- `head`, `left_wrist`, `right_wrist` camera config
- head color `1280x720@30`, wrist color `640x360@30`
- POSIX shared memory ring buffer
- ZMQ metadata publisher: `camera.bundle`, `camera.health`
- stream별 deque 기반 `FrameSynchronizer`
- software sync: nearest timestamp
- hardware sync: frame number matching 및 RealSense sync option 설정 코드
- mock camera mode와 hardware-free test path

현재 기준으로 정정할 점:

- 과거 `camera_server/review.md`에는 `FrameSynchronizer`가 latest-cache bundle에 가깝다고 적혀 있으나, 현재 코드는 stream별 deque와 master-camera-driven bundling으로 수정되어 있다.
- 과거 리뷰에는 hardware sync option 미설정이 P0로 적혀 있으나, 현재 `realsense_device.cpp`에는 `RS2_OPTION_INTER_CAM_SYNC_MODE` 설정 코드가 있다. 다만 실제 D435f/D405 조합에서 hardware sync acceptance는 아직 필요하다.
- shared memory write는 현재 `write_mutex_`로 보호된다. 따라서 과거의 `ring.next_slot++` race 지적은 현 코드 기준으로 재검증해야 한다.

아직 필요한 것:

| 항목 | 상태 | 영향 |
| --- | --- | --- |
| 실제 serial config | `REPLACE_*_SERIAL` placeholder | 실제 카메라 startup 불가 |
| 3-camera 장시간 acceptance | 증거 없음 | 실 capture 안정성 미확인 |
| reconnect | config는 있으나 실제 reconnect는 warning만 출력 | USB/camera disconnect 복구 불가 |
| calibration | intrinsics/extrinsics/hand-eye pipeline 없음 | policy/dataset geometry 불명확 |
| policy integration | sample reader와 문서만 있음 | 실제 inference loop 없음 |
| recording/dataset contract | episode schema/merge tool 없음 | 학습 데이터 생성 불완전 |

필요 acceptance:

- RealSense 3대 RGB 30 FPS, recording off, 10분
- recording on, 10분
- sample/policy reader가 shared memory에서 image read, seqlock failure rate 측정
- camera unplug 시 health degrade 확인
- USB controller bandwidth 확인

### 4.4 rb_servo_gui

구현된 부분:

- Python viser 기반 GUI
- UDP state stream 수신
- lifecycle command, bounded joint jog command 송신
- stand mesh/URDF asset 사용
- real mode motion 차단
- TCP target UI 초기 구현
- GUI contract tests

구조적 이슈:

- 현재 위치는 top-level `rb_gui`다.
- GUI는 control layer 내부 구현이라기보다 외부 observer/operator app이다.
- 향후 camera health, camera preview, recording 상태까지 표시할 가능성이 있으므로 control layer와 분리된 모듈로 유지하는 편이 맞다.

권장 구조:

```text
robotics_lab/
  rb_servo_server/
  rb_simulator/
  camera_server/
  rb_gui/
    rb_servo_gui/
    docs/
    tests/
```

분리 시 갱신 대상:

- compose build context
- GUI Dockerfile COPY 경로
- `PYTHONPATH`/test path
- `RB_GUI_DESCRIPTIONS_DIR` 기본값
- stand/URDF asset path 계약

## 5. Cross-component 격차

### 5.1 policy_runner 부재

현재 저장소에는 실제 `policy_runner` 구현이 없다. `camera_server/tools/read_latest_bundle.py`는 shared memory reader sample이고, `camera_server/docs/policy_runner_integration.md`는 계약 문서에 가깝다.

최소 `policy_runner`는 다음을 해야 한다.

- `camera.bundle` 구독
- shared memory mmap 1회 재사용
- latest complete bundle만 유지
- `rb_servo_server` state stream 구독
- camera timestamp와 robot state timestamp alignment
- stale camera, high skew, robot fault 시 action command 중지
- 10~30 Hz bounded UDP command 송신

### 5.2 timestamp alignment 미정의

camera path와 servo path 모두 host timestamp를 사용하지만, cross-process alignment 규칙이 아직 명확하지 않다.

정의 필요:

- `CLOCK_MONOTONIC_RAW`, `std::chrono::steady_clock`, Python `time.monotonic_ns()` 중 무엇을 공통 기준으로 볼지
- 각 process startup 시 `(monotonic_ns, wall_ns)` pair를 기록할지
- offline dataset builder가 어떤 timestamp를 기준으로 merge할지
- policy runtime에서는 latest state를 쓸지 interpolation을 쓸지

### 5.3 통합 런처 부재

현재 compose는 하위 프로젝트별로 나뉘어 있다.

- `rb_servo_server/docker-compose.yml`
- `camera_server/docker-compose.yml`

필요한 top-level 진입점:

- `scripts/up_mock.sh`
- `scripts/up_sim.sh`
- `scripts/up_real.sh`
- 또는 top-level `docker-compose.yml` with `mock`/`sim`/`real` profile

### 5.4 dataset_builder 부재

카메라 bundle, servo state/log, command log를 episode 단위로 합치는 도구가 없다.

최소 dataset key:

```text
episode_id
camera_bundle_seq
camera_bundle_time_ns
robot_state_time_ns
left/right q_actual
left/right q_sent
left/right motion_state/fault
head/left_wrist/right_wrist image refs
action command seq
calibration snapshot
```

### 5.5 최상위 문서 부족

서브프로젝트별 문서는 많지만, `robotics_lab` 루트에서 전체 관계를 설명하는 단일 README/architecture가 부족하다.

권장:

- `README.md`: 전체 구성, 빠른 시작, 포트 맵, 주요 시나리오
- `docs/architecture.md`: 데이터 흐름, 시간 동기, 좌표계, safety boundary
- `docs/hardware_acceptance.md`: 실제 로봇/카메라 acceptance runbook

## 6. 우선순위 백로그

### P0

| 항목 | 완료 기준 |
| --- | --- |
| hardware-free gate 실패 수정 | `RBSIM_SMOKE_MODE=skip bash scripts/hardware_free_validation.sh` 통과 |
| `rb_simulator` topology 확정 | 단일 dual-arm simulator process로 표준화. 문서/config/smoke runner는 같은 설명을 유지 |
| `RbpodoBackend` 구현 계획 | `rb_servo_server/docs/rbpodo_backend_plan.md`에 real robot acceptance runbook과 SDK integration plan 작성 |
| stand/robot/camera frame 계약 | `docs/frame_contract.md`에 stand/base/TCP/wrist/head camera frame naming, transform direction, calibration 위치/포맷 초안 정리 |

### P1

| 항목 | 완료 기준 |
| --- | --- |
| 최소 `policy_runner` 구현 | mock camera + mock/rbsim servo server로 dummy action loop 동작 |
| camera hardware acceptance | 3-camera 30 FPS 장시간 capture evidence 확보 |
| `rb_gui` top-level 분리 | GUI가 `rb_gui`에서 build/test/run 되고 asset path가 명시됨 |
| timestamp alignment spec | camera/robot/policy/dataset timestamp merge rule 문서화 |
| dataset/recording contract | episode metadata와 merge 입력/출력 schema 정의 |
| Cartesian FK/IK | `tcp_stand` state publish와 bounded TCP command path 구현 |

### P2

| 항목 | 완료 기준 |
| --- | --- |
| force/admittance control | sensor frame/gain/safety 검증 후 active path 연결 |
| gripper integration | command schema와 backend 구현 |
| camera reconnect | unplug/replug health 및 recovery acceptance |
| calibration pipeline | intrinsics/extrinsics/hand-eye 저장/배포 |
| high-rate optimization | parallel send, lock-free buffer, CPU affinity, shm reader library 검토 |
| top-level launcher | `mock`/`sim`/`real` 시나리오별 실행 진입점 제공 |

## 7. 권장 개발 순서

1. `rb_servo_server`와 `rb_simulator` failing gate를 먼저 고친다.
2. `rb_simulator` topology를 하나로 확정하고 config/docs를 맞춘다.
3. 실제 RB3-730 `RbpodoBackend` 구현 계획과 hardware acceptance checklist를 만든다.
4. stand, robot base, wrist/head camera frame 계약을 문서화한다.
5. `camera_server` 실제 serial config와 3-camera hardware acceptance를 진행한다.
6. 최소 `policy_runner`를 만들어 camera, robot state, command loop를 연결한다.
7. `rb_gui`를 top-level로 분리하고 mock/rbsim operator workflow를 정리한다.
8. FK/IK, TCP control, force control, gripper를 순서대로 연다.

## 8. 주요 근거 파일

- `rb_servo_server/README.md`
- `rb_servo_server/docs/network_protocol.md`
- `rb_servo_server/docs/gui_operator_console.md`
- `rb_servo_server/src/robot/rbpodo_backend.cpp`
- `rb_servo_server/src/robot/rbsim_backend.cpp`
- `rb_servo_server/src/control/dual_arm_servo_loop.cpp`
- `rb_servo_server/src/control/cartesian_controller.cpp`
- `rb_simulator/README.md`
- `rb_simulator/src/rbsim/server.py`
- `rb_simulator/src/rbsim/state_machine.py`
- `camera_server/config/triple_realsense.yaml`
- `camera_server/src/sync/frame_synchronizer.cpp`
- `camera_server/src/camera/camera_manager.cpp`
- `camera_server/src/camera/realsense_device.cpp`
- `camera_server/docs/policy_runner_integration.md`
- `rb_gui/rb_servo_gui/app.py`
- `rb_gui/rb_servo_gui/safety.py`

## 9. 최종 판단

`robotics_lab`는 방향이 맞는 초기 통합 구조다. 지금 가장 가치 있는 다음 목표는 실제 하드웨어로 가기 전, hardware-free 양팔 simulator path를 green gate로 만들고 camera bundle과 robot state를 하나의 `policy_runner` 계약으로 묶는 것이다.

실제 robot/camera 운용은 그 다음 단계다. `RbpodoBackend`, frame/calibration 계약, RealSense hardware acceptance, policy loop가 정리되기 전에는 실험 중 발생한 문제를 component bug인지 integration contract 문제인지 구분하기 어렵다.
