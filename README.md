# robotics_lab

한국어 기본 README입니다. 영어 원문은 [README.en.md](README.en.md)에 보존되어 있습니다.

`robotics_lab`는 dual-arm RB3-730 시스템을 통합하기 위한 작업 공간입니다. 서보 제어, 실제 토폴로지와 같은 형태의 로컬 시뮬레이터, 카메라 캡처, `policy_runner`, 운영자 GUI를 함께 다룹니다.

## 현재 단계

현재 프로젝트 단계는 **시뮬레이터 우선 Cartesian acceptance hardening**입니다.

다음 마일스톤은 시뮬레이터 측 동작을 반복 검증하는 것입니다.

- 팔별 독립 시뮬레이터 토폴로지
- 구조화된 backend result 및 fault telemetry
- `JointTarget` / `JointVelocity`
- `TcpPoseTarget`
- `TcpLinearMove`
- `TcpTwistLocal` / `TcpTwistStand`
- GUI 운영자 제어
- `policy_runner` SpaceMouse 경로
- command-source lease/arbitration
- 카메라 readiness contract

실제 로봇 구동은 현재 기본 마일스톤이 아닙니다.

## 현재 성숙도

mock/simulation에서 지원되는 항목:

- mock dual-arm servo control
- 팔별 로컬 simulator backend
- persistent simulator JSON-line transport
- simulator direct 및 worker I/O mode
- quaternion 필드를 포함한 FK/TCP state publication
- simulator-only TCP PTP, Linear, Twist command
- mock camera server
- mock/simulation용 GUI viewer/operator console
- `policy_runner` joint 및 Cartesian simulator action source
- simulator-only Cartesian acceptance script

아직 production-ready가 아닌 항목:

- 실제 RB3-730 motion
- 실제 Cartesian/TCP motion
- force control
- gripper control
- 실측 camera/robot calibration
- 실제 camera + policy + robot closed-loop behavior

## Source Of Truth

먼저 아래 문서부터 확인합니다.

- `AGENTS.md`: Codex/Claude/기타 에이전트 작업 지침
- `REVIEW.md`: 현재 review baseline 및 open item
- `docs/architecture.md`: 시스템 토폴로지, 용어, motion primitive contract, safety boundary
- `docs/servo_backend_contract.md`: backend result, fault, worker I/O, state telemetry contract
- `docs/frame_contract.md`: 공통 frame 및 calibration 상태
- `docs/hardware_free_validation.md`: hardware-free validation boundary
- `docs/runbooks/tcp_pose_simulator_acceptance.md`: Cartesian simulator acceptance
- `docs/runbooks/camera_acceptance.md`: 실제 3-camera acceptance
- `calibration/active_calibration.yaml`: configured-estimate robot/camera/stand setup registry

과거 prompt/planning 파일은 감사용 맥락입니다. 위 문서들과 충돌하면 위 문서들이 우선입니다.

## 표준 용어

```yaml
run_mode: mock | simulation | real
backend_type: mock | simulator | rbpodo
```

## 실제 및 시뮬레이터 토폴로지

실제 시스템:

```text
rb_servo_server
  left_robot  backend_type=rbpodo -> 172.28.60.200
  right_robot backend_type=rbpodo -> 172.28.60.201
```

시뮬레이터:

```text
rb_servo_server
  left_robot  backend_type=simulator -> rb_simulator_left
  right_robot backend_type=simulator -> rb_simulator_right
```

시뮬레이터는 팔마다 독립 controller endpoint를 갖는 실제 토폴로지를 반영해야 합니다. 기본값으로 실제 로봇 IP를 재사용하면 안 됩니다.

## Safety Gates

실제 로봇 연결:

```bash
RB_ALLOW_REAL_ROBOT=1
```

실제 joint servo motion:

```bash
RB_ALLOW_REAL_MOTION=1
```

실제 Cartesian/TCP motion:

```bash
RB_ALLOW_REAL_CARTESIAN=1
```

이 gate들은 필요 조건일 뿐 충분 조건은 아닙니다. Config와 real-hardware acceptance도 해당 동작을 명시적으로 허용해야 합니다.

Force control은 비활성 상태를 유지합니다.

```yaml
force_control:
  provider: null
  enable: false
```

## Motion Primitive 요약

- `TcpPoseTarget`: PTP / MoveJ-like Cartesian final-pose target입니다. Cartesian path는 보장하지 않습니다.
- `TcpLinearMove`: simulator-only MoveL-like Cartesian path primitive입니다.
- `TcpTwistLocal` / `TcpTwistStand`: simulator-only streaming Cartesian velocity primitive입니다.
- `TcpDeltaLocal` / `TcpDeltaStand`: low-level one-shot/debug jog primitive입니다.

## 자주 쓰는 명령

Python checks:

```bash
python3 -m unittest discover rb_gui/tests
python3 -m unittest discover policy_runner/tests
PYTHONPATH=rb_simulator/src python3 -m unittest discover rb_simulator/tests
```

Hardware-free gate:

```bash
./scripts/codex_gate.sh HARDEN-10
```

Cartesian simulator acceptance:

```bash
CODEX_RUN_CARTESIAN_ACCEPTANCE=1 ./scripts/codex_gate.sh CART-HARDEN-05
```

시뮬레이터 운영자 stack 시작:

```bash
make sim-up
```

접속:

```text
http://127.0.0.1:8080
```

## 표준 Config

Servo server simulation configs:

- `rb_servo_server/config/dual_simulator.yaml`
- `rb_servo_server/config/dual_simulator_compose.yaml`
- `rb_servo_server/config/dual_simulator_worker.yaml`
- `rb_servo_server/config/dual_simulator_tcp_acceptance.yaml`

Simulator configs:

- `rb_simulator/config/left_rb3_730e.yaml`
- `rb_simulator/config/right_rb3_730e.yaml`
- `rb_simulator/config/left_rb3_730e_compose.yaml`
- `rb_simulator/config/right_rb3_730e_compose.yaml`

Real robot template:

- `rb_servo_server/config/dual_real.example.yaml`

Site-local real configs:

- `rb_servo_server/config/local/dual_real_readonly.yaml`
- `rb_servo_server/config/local/dual_real_motion.yaml`

실행 가능한 tracked real robot config는 추가하면 안 됩니다.
