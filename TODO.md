# robotics_lab P0~P3 Module TODO

이 문서는 `robotics_lab` 저장소를 **실제 양팔 RB3-730 시스템과 topology-isomorphic한 simulator / servo / policy / GUI 플랫폼**으로 정리하기 위한 P0~P3 개발 TODO입니다.

작성 기준:

- 실제 로봇은 **팔 하나당 controller 하나**이다.
  - left controller: `172.28.60.200`
  - right controller: `172.28.60.201`
- simulator도 편의성보다 실제 구조와의 isomorphism을 우선한다.
  - left simulator process/container
  - right simulator process/container
- `rb_servo_server`는 joint-only 제어부터 real/sim 동일 topology로 안정화한다.
- FK는 TCP pose publish와 GUI visualization을 위해 먼저 붙인다.
- IK/TCP Pose command는 simulator에서 충분히 검증한 뒤 real gate를 별도로 연다.
- force control은 현재 구현하지 않는다. `mo_forcecontroller` 연동 전까지는 `null` provider로만 유지한다.
- `policy_runner`는 Python 기반 action source로 추가한다. SpaceMouse도 `rb_gui`가 아니라 `policy_runner`에 붙인다.

---

## 0. Agent 공통 규칙

여러 Codex agent가 동시에 작업할 것을 전제로 한다. 각 agent는 자기 work package 범위 밖의 파일을 최소한으로만 수정한다.

### 0.1 모든 agent 공통 금지사항

- 실제 로봇 motion을 가능하게 하는 코드를 암묵적으로 열지 않는다.
- `RB_ALLOW_REAL_ROBOT=1` 없이 real config가 시작되게 만들지 않는다.
- `RB_ALLOW_REAL_MOTION=1` 없이 real robot에 `servo_j`가 전송되게 만들지 않는다.
- FK/IK가 구현되기 전 GUI나 policy_runner에서 TCP Cartesian command를 실제 motion path로 보내지 않는다.
- force/admittance/impedance control을 임시 구현으로 활성화하지 않는다.
- camera serial placeholder인 `REPLACE_*`, `TODO`, `CHANGEME`를 real config에서 허용하지 않는다.
- simulator endpoint를 실제 robot IP인 `172.28.60.200/201`로 흉내내는 것을 기본값으로 만들지 않는다.
  - isomorphism은 “팔 하나당 controller endpoint 하나” 구조를 맞추는 것이다.
  - 실제 IP 숫자까지 simulator에서 재사용하면 real network와 충돌하거나 오조작 위험이 생긴다.

### 0.2 모든 agent 공통 완료 조건

각 work package를 완료할 때 다음을 반드시 남긴다.

- 변경한 파일 목록
- 추가/수정한 config 목록
- 실행한 test command
- 실패한 test가 있다면 실패 이유
- 아직 의도적으로 남긴 TODO

### 0.3 권장 test command

가능한 범위에서 아래를 실행한다.

```bash
# simulator Python tests
PYTHONPATH=rb_simulator/src python3 -m unittest discover rb_simulator/tests

# rb_gui tests
python3 -m unittest discover rb_gui/tests

# rb_servo_server CMake tests, hardware-free
cmake -S rb_servo_server -B rb_servo_server/build/hardware_free_gate \
  -DCMAKE_BUILD_TYPE=Debug \
  -DRB_SERVO_ENABLE_RBPODO=OFF \
  -DRB_SERVO_ALLOW_FETCHCONTENT=OFF \
  -DBUILD_TESTING=ON
cmake --build rb_servo_server/build/hardware_free_gate -j
ctest --test-dir rb_servo_server/build/hardware_free_gate --output-on-failure

# camera_server mock/stub tests
cmake -S camera_server -B camera_server/build/hardware_free_gate \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCAMERA_SERVER_FORCE_MOCK_CAMERA=ON \
  -DCAMERA_SERVER_FORCE_ZMQ_STUB=ON \
  -DCAMERA_SERVER_BUILD_TESTS=ON
cmake --build camera_server/build/hardware_free_gate -j
ctest --test-dir camera_server/build/hardware_free_gate --output-on-failure

# full local gate
./scripts/hardware_free_validation.sh
```

P0 이후에는 full gate가 **per-arm simulator** 기준으로 바뀌어야 한다.

---

## 1. Target architecture

### 1.1 실제 로봇 topology

```text
rb_servo_server
  left_robot backend=rbpodo  -> 172.28.60.200, one physical RB3-730 controller
  right_robot backend=rbpodo -> 172.28.60.201, one physical RB3-730 controller
```

### 1.2 simulator topology

```text
rb_servo_server
  left_robot backend=simulator  -> rb_simulator_left process/container
  right_robot backend=simulator -> rb_simulator_right process/container

rb_simulator_left
  arm: left
  control: tcp://0.0.0.0:50200 inside container
  admin:   tcp://0.0.0.0:50201 inside container

rb_simulator_right
  arm: right
  control: tcp://0.0.0.0:50200 inside container
  admin:   tcp://0.0.0.0:50201 inside container
```

Docker 내부에서는 두 simulator container가 각각 독립 network namespace를 가지므로 둘 다 내부 port `50200/50201`을 사용할 수 있다.

Host에서 직접 실행할 때는 port를 분리한다.

```text
left:
  control: tcp://127.0.0.1:50200
  admin:   tcp://127.0.0.1:50201

right:
  control: tcp://127.0.0.1:50210
  admin:   tcp://127.0.0.1:50211
```

### 1.3 용어 canonicalization

Public config, docs, GUI 표시에서는 아래 용어만 사용한다.

```yaml
run_mode: mock | simulation | real
backend_type: mock | simulator | rbpodo
```

Deprecated alias:

```text
rbsim_local -> simulator
rbsim       -> simulator
```

내부 class/file 이름 `RbsimBackend`, `rbsim` Python package는 P0에서 즉시 rename하지 않아도 된다. 단, public config와 GUI에는 `simulator`만 보이게 한다.

---

# P0. Architecture alignment and cleanup

P0 목표:

```text
- repo가 한 가지 architecture를 말하게 만든다.
- simulator를 per-arm process/container topology로 바꾼다.
- public 용어를 simulator/simulation으로 통일한다.
- hardware-free validation이 dependency preflight와 함께 명확히 동작한다.
- GUI, camera_server, docs/scripts의 명확한 오류를 정리한다.
```

---

## P0-A. Root source-of-truth 문서 추가

### 담당 모듈

- root docs
- architecture docs

### 수정/추가 파일

- `README.md` 추가
- `docs/architecture.md` 추가 또는 정리
- `docs/hardware_free_validation.md` 수정
- `docs/frame_contract.md` 수정
- `review.md` archive 이동 또는 header 추가
- `camera_server/review.md` archive 이동 또는 header 추가
- `camera_server/docs/implementation_plan_for_codex.md` archive 이동 또는 header 추가
- `rb_servo_server/docs/commit_slicing_advice.md` archive 이동 또는 header 추가
- `rb_servo_server/docs/required_tests_and_smoke_checks.md` 최신 gate 기준으로 수정 또는 archive

### 작업 내용

1. root `README.md`를 만든다.
2. README 상단에 현재 maturity를 명확히 쓴다.

   ```text
   Currently supported:
   - mock dual-arm servo control
   - per-arm local simulator backend
   - mock camera server
   - GUI viewer/operator console for mock/simulation

   Not production-ready yet:
   - real RB3-730 motion
   - Cartesian TCP motion in real mode
   - force control
   - gripper control
   - measured camera/robot calibration
   ```

3. “실제와 simulator topology”를 diagram으로 명시한다.
4. canonical terminology를 README와 docs에 명시한다.

   ```text
   backend_type: mock | simulator | rbpodo
   run_mode: mock | simulation | real
   ```

5. stale/agent-generated planning docs는 삭제하지 말고 `docs/archive/...`로 이동하거나 문서 상단에 다음 header를 추가한다.

   ```md
   > Historical review/planning document. Some findings may be obsolete.
   > The current source of truth is root README.md and docs/architecture.md.
   ```

6. P0~P3 이후 기능 road map을 README에서 짧게 연결한다.

### Acceptance criteria

- root `README.md`만 읽어도 다음이 명확해야 한다.
  - simulator가 per-arm process/container 구조라는 점
  - real robot IP가 left/right로 분리된다는 점
  - real motion은 아직 gate가 필요하다는 점
  - force control은 현재 null이라는 점
  - TCP Cartesian real motion은 P3 이후에도 별도 real acceptance 전에는 닫혀 있다는 점
- `rbsim_local`이라는 public 용어가 README와 새 docs에 나오지 않아야 한다.
- archive 처리된 문서는 source-of-truth로 오해되지 않아야 한다.

---

## P0-B. rb_simulator를 per-arm simulator로 변경

### 담당 모듈

- `rb_simulator`

### 수정/추가 파일

- `rb_simulator/src/rbsim/config.py`
- `rb_simulator/src/rbsim/state_machine.py`
- `rb_simulator/src/rbsim/protocol.py`
- `rb_simulator/src/rbsim/server.py`
- `rb_simulator/src/rbsim/__main__.py`
- `rb_simulator/config/left_rb3_730e.yaml` 추가
- `rb_simulator/config/right_rb3_730e.yaml` 추가
- `rb_simulator/config/dual_rb3_730e.yaml` deprecated 또는 test-only로 유지
- `rb_simulator/tests/test_state_machine.py`
- `rb_simulator/tests/test_protocol_contract.py`
- `rb_simulator/tests/test_protocol_server.py`
- `rb_simulator/README.md`
- `rb_simulator/docs/architecture.md`
- `rb_simulator/docs/protocol_v1.md`

### 현재 문제

현재 `rb_simulator`는 하나의 `DualArmSimulator` process가 left/right를 모두 갖는다. 이는 실제 시스템의 “팔 하나당 controller 하나” 구조와 다르다.

### 목표 구조

```text
ArmSimulator(config.arm = left)
ArmSimulator(config.arm = right)
```

각 process는 한 arm만 소유한다.

### 작업 내용

1. `SimulatorConfig`에 단일 arm identity를 추가한다.

   예시:

   ```python
   @dataclass(frozen=True)
   class SimulatorConfig:
       arm: str  # "left" | "right"
       control_bind: str
       admin_bind: str
       update_rate_hz: float
       motion_time_constant_sec: float
       max_joint_velocity_deg_s: float
       model: str
       arm_config: ArmConfig
       fault_defaults: FaultDefaults
   ```

2. 기존 `DualArmSimulator`를 다음 중 하나로 변경한다.

   권장:

   ```text
   class ArmSimulator
   ```

   또는 migration을 줄이기 위해:

   ```text
   class SingleArmSimulator
   class DualArmSimulator  # deprecated test helper only
   ```

3. 단일 simulator가 다른 arm request를 받으면 fail-closed로 reject한다.

   ```text
   left simulator receives arm=left  -> OK
   left simulator receives arm=right -> error name: wrong_arm 또는 unknown_arm
   right simulator receives arm=right -> OK
   right simulator receives arm=left -> error
   ```

4. `SimulatorProtocol`의 `FaultInjectionState`를 단일 arm 기준으로 초기화한다.

   ```python
   FaultInjectionState.for_arms([config.arm])
   ```

5. admin endpoint의 동작을 정리한다.

   - `admin.tick`은 단일 arm snapshot만 반환한다.
   - `states` field는 호환성을 위해 `{"left": ...}` 또는 `{"right": ...}` 형태로 유지 가능하다.
   - arm이 생략된 admin command는 configured arm을 default로 사용해도 된다.
   - 단, wrong arm이 명시되면 reject한다.

6. config file을 분리한다.

   `rb_simulator/config/left_rb3_730e.yaml`:

   ```yaml
   simulator:
     schema: robotics_lab.simulator.v1
     arm: left
     control_bind: "tcp://127.0.0.1:50200"
     admin_bind: "tcp://127.0.0.1:50201"
     update_rate_hz: 200
     motion_time_constant_sec: 0.04
     max_joint_velocity_deg_s: 360
     model: "RB3_730E"

   arm:
     name: "left_simulator"
     initial_q_deg: [0, -30, 80, 0, 60, 0]

   fault_defaults:
     connected: true
     initialized: false
     servo_enabled: false
     has_valid_joint_state: true
     has_error: false
     error_code: 0
   ```

   `right_rb3_730e.yaml`은 `arm: right`, port는 host-run 기준으로 `50210/50211`을 사용한다.

7. Docker container 안에서 `0.0.0.0` bind를 허용할 수 있게 한다.

   현재 `parse_tcp_bind()`는 loopback만 허용한다. 아래 environment gate를 추가한다.

   ```text
   RB_SIMULATOR_ALLOW_NON_LOOPBACK=1
   ```

   정책:

   - default: loopback only
   - `0.0.0.0` 또는 non-loopback bind는 `RB_SIMULATOR_ALLOW_NON_LOOPBACK=1`일 때만 허용

8. `rb_simulator` 실행 entrypoint를 명확히 한다.

   최소:

   ```bash
   PYTHONPATH=rb_simulator/src python3 -m rbsim --config rb_simulator/config/left_rb3_730e.yaml
   ```

   권장:

   - `rb_simulator/pyproject.toml` 추가
   - console script 추가

   ```text
   rb-simulator = rbsim.server:main
   ```

### Acceptance criteria

- 아래 두 process를 동시에 host에서 실행할 수 있어야 한다.

  ```bash
  PYTHONPATH=rb_simulator/src python3 -m rbsim --config rb_simulator/config/left_rb3_730e.yaml
  PYTHONPATH=rb_simulator/src python3 -m rbsim --config rb_simulator/config/right_rb3_730e.yaml
  ```

- left simulator에 `arm=right` request를 보내면 reject되어야 한다.
- right simulator에 `arm=left` request를 보내면 reject되어야 한다.
- simulator unit tests가 per-arm 기준으로 통과해야 한다.
- `dual_rb3_730e.yaml`가 남아 있다면 README에 deprecated라고 표시해야 한다.

---

## P0-C. rb_servo_server public terminology를 `simulator`로 정리

### 담당 모듈

- `rb_servo_server` config/schema
- docs

### 수정/추가 파일

- `rb_servo_server/include/rb_servo/core/types.hpp`
- `rb_servo_server/include/rb_servo/config/config.hpp`
- `rb_servo_server/src/config/config.cpp`
- `rb_servo_server/src/robot/backend_factory.cpp`
- `rb_servo_server/src/robot/rbsim_backend.cpp`
- `rb_servo_server/config/dual_rb_simulator.yaml` rename 또는 replace
- `rb_servo_server/config/dual_rb_simulator_compose.yaml` rename 또는 replace
- `rb_servo_server/config/dual_rbsim.yaml` remove/deprecate
- `rb_servo_server/docs/network_protocol.md`
- `rb_servo_server/docs/config_examples.md`

### 현재 문제

Config와 GUI에서 `rbsim`, `rbsim_local`, `rb_simulator`, `simulation`이 섞여 있다.

### 작업 내용

1. public config에서 다음을 canonical로 만든다.

   ```yaml
   backend_type: simulator
   run_mode: simulation
   simulator_control_endpoint: "tcp://127.0.0.1:50200"
   ```

2. 기존 alias는 parsing만 허용하고 warning을 출력한다.

   ```text
   backend_type: rbsim_local -> deprecated alias for simulator
   backend_type: rbsim       -> deprecated alias for simulator
   rbsim_control_endpoint    -> deprecated alias for simulator_control_endpoint
   ```

3. `BackendConfig`에 canonical field를 추가한다.

   ```cpp
   std::string simulator_control_endpoint = "tcp://127.0.0.1:50200";
   ```

   기존 `rbsim_control_endpoint`는 삭제하지 말고 migration 기간 동안 compatibility field 또는 alias로 처리한다.

4. `BackendType` enum은 가능한 경우 다음으로 rename한다.

   ```cpp
   enum class BackendType { Rbpodo, Mock, Simulator };
   ```

   단, 이 rename이 P0-B/P0-D와 충돌한다면 enum rename은 뒤로 미루고 parser/output만 먼저 고친다.

5. 새 canonical config를 만든다.

   ```text
   rb_servo_server/config/dual_simulator.yaml
   rb_servo_server/config/dual_simulator_compose.yaml
   ```

6. 기존 config는 다음 중 하나로 처리한다.

   - `dual_rb_simulator.yaml`은 compatibility alias로 유지하되 상단 comment에 deprecated 표시
   - `dual_rbsim.yaml`은 중복이면 삭제 또는 archive

7. simulator endpoint validation을 per-arm topology에 맞게 수정한다.

   현재는 simulator endpoint가 loopback만 허용된다. Docker compose service name을 사용해야 하므로 아래를 허용해야 한다.

   - host-run: `tcp://127.0.0.1:50200`, `tcp://127.0.0.1:50210`
   - compose-run: `tcp://rb_simulator_left:50200`, `tcp://rb_simulator_right:50200`

   단, `run_mode=real` + `backend_type=simulator`는 계속 금지한다.

### Acceptance criteria

- 다음 config가 정상 parse되어야 한다.

  ```yaml
  left_robot:
    backend_type: simulator
    run_mode: simulation
    simulator_control_endpoint: "tcp://127.0.0.1:50200"

  right_robot:
    backend_type: simulator
    run_mode: simulation
    simulator_control_endpoint: "tcp://127.0.0.1:50210"
  ```

- `rbsim_local` alias는 여전히 parse되지만 warning을 남긴다.
- 새 docs/config examples에는 `rbsim_local`을 쓰지 않는다.
- `run_mode=real` + `backend_type=simulator`는 fail해야 한다.

---

## P0-D. rb_servo_server config parser를 yaml-cpp로 교체

### 담당 모듈

- `rb_servo_server` config loading
- CMake dependency

### 수정/추가 파일

- `rb_servo_server/CMakeLists.txt`
- `rb_servo_server/src/config/config.cpp`
- `rb_servo_server/include/rb_servo/config/config.hpp`
- `rb_servo_server/tests/test_config_loader.cpp` 추가 권장
- `scripts/check_deps.sh`와 연동

### 현재 문제

`rb_servo_server`는 hand-written simple YAML parser를 사용한다. nested schema, strict validation, typo 방지에 약하다. `camera_server`는 이미 `yaml-cpp`를 사용하므로 dependency 통일이 낫다.

### 작업 내용

1. CMake에 `yaml-cpp`를 required로 추가한다.

   ```cmake
   find_package(yaml-cpp REQUIRED)
   target_link_libraries(rb_servo_core PUBLIC yaml-cpp)
   ```

2. 기존 `parseSimpleYaml()` 기반 parser를 `YAML::Node` 기반으로 교체한다.

3. schema version을 추가한다.

   ```yaml
   schema: robotics_lab.rb_servo_server.v1
   ```

   정책:

   - schema field가 없으면 warning 또는 compatibility mode
   - unknown schema면 fail

4. unknown key validation을 추가한다.

   권장 정책:

   - real config: unknown key는 fail
   - simulation/mock config: unknown key도 fail 권장

   이유: typo가 fallback default로 흘러가면 위험하다.

5. config section별 allowed keys를 명시한다.

   ```text
   left_robot/right_robot:
     backend_type
     run_mode
     name
     ip
     operation_mode
     simulator_control_endpoint
     simulator_*_timeout_sec
     initial_q_deg
     speed_bar
     servo_time_sec
     servo_lookahead_sec
     servo_gain
     servo_acc
     disable_waiting_ack
     allow_motion, P1에서 추가
   ```

6. `state_pub_bind` 이름을 public config에서 `state_pub_endpoint`로 변경한다.

   Compatibility:

   - `state_pub_bind`는 deprecated alias로 parse 가능
   - 새 config/docs는 `state_pub_endpoint`만 사용

7. `network.state_pub_rate_hz`를 추가한다.

   ```yaml
   network:
     command_bind: "udp://127.0.0.1:50010"
     state_pub_endpoint: "udp://127.0.0.1:50110"
     state_pub_rate_hz: 30
   ```

8. force control config를 null provider 기준으로 단순화한다.

   P0에서는 아래만 canonical로 사용한다.

   ```yaml
   force_control:
     provider: null
     enable: false
   ```

   기존 admittance gain fields는 deprecated 또는 future-only로 이동한다.

### Acceptance criteria

- `rb_servo_server/config/dual_mock.yaml` parse 성공
- `rb_servo_server/config/dual_simulator.yaml` parse 성공
- unknown key를 넣은 config는 test에서 fail
- `state_pub_endpoint`가 state publisher destination으로 사용됨
- `state_pub_bind` alias는 warning 후 동작
- `state_pub_rate_hz`로 publish period가 바뀜
- `yaml-cpp` 미설치 시 `scripts/check_deps.sh`가 먼저 친절히 실패해야 함

---

## P0-E. Docker Compose를 per-arm simulator topology로 수정

### 담당 모듈

- root compose
- Makefile 또는 scripts

### 수정/추가 파일

- `docker-compose.yml`
- `Makefile` 추가 권장
- `rb_simulator/docker/rb_simulator.Dockerfile`
- `rb_servo_server/config/dual_simulator_compose.yaml`
- `scripts/hardware_free_validation.sh`
- `docs/hardware_free_validation.md`

### 현재 문제

현재 root compose는 단일 `rb_simulator` service와 `network_mode: service:rb_simulator`를 사용한다. 또한 real camera server가 기본 compose에 privileged로 포함되어 있다.

### 목표 compose

```text
rb_gui
rb_simulator_left
rb_simulator_right
rb_servo_server
camera_server_mock, optional profile
camera_server_real, real_camera profile only
```

### 작업 내용

1. root compose에 `rb_simulator_left`, `rb_simulator_right`를 추가한다.

   예시:

   ```yaml
   rb_simulator_left:
     build:
       context: .
       dockerfile: rb_simulator/docker/rb_simulator.Dockerfile
     image: robotics_lab/rb_simulator:dev
     command: ["python", "-m", "rbsim", "--config", "rb_simulator/config/left_rb3_730e.compose.yaml"]
     environment:
       RB_SIMULATOR_ALLOW_NON_LOOPBACK: "1"
     networks:
       rb_servo_net:
         aliases:
           - left_controller_sim
     healthcheck:
       test: ["CMD", "python", "-c", "import socket; s=socket.create_connection(('127.0.0.1', 50200), 1.0); s.close()"]
   ```

2. right도 동일하게 추가한다.

3. `rb_servo_server`는 더 이상 `network_mode: service:rb_simulator`를 사용하지 않는다.

   ```yaml
   rb_servo_server:
     networks: [rb_servo_net]
     command: ["./build/rb_servo_server", "--config", "config/dual_simulator_compose.yaml"]
     depends_on:
       rb_simulator_left:
         condition: service_healthy
       rb_simulator_right:
         condition: service_healthy
   ```

4. GUI env를 수정한다.

   ```yaml
   RB_GUI_COMMAND_HOST: "rb_servo_server"
   RB_GUI_COMMAND_PORT: "50010"
   RB_GUI_OBSERVED_MODE: "simulation"
   RB_GUI_OBSERVED_BACKEND: "simulator"
   ```

   현재 `RB_GUI_COMMAND_HOST: rb_simulator`는 틀린 의미다. GUI command는 simulator가 아니라 `rb_servo_server`로 가야 한다.

5. real camera server를 default compose에서 분리한다.

   ```yaml
   camera_server:
     profiles: ["real_camera"]
     privileged: true
     network_mode: host
   ```

6. mock camera server를 optional profile로 추가한다.

   ```yaml
   camera_server_mock:
     profiles: ["mock_camera"]
     privileged: false
     networks: [rb_servo_net]
     command: ["--config", "/app/config/mock_triple_realsense.yaml"]
   ```

7. Makefile을 추가한다.

   ```makefile
   sim-up:
    docker compose --profile simulator up --build rb_gui rb_simulator_left rb_simulator_right rb_servo_server

   sim-down:
    docker compose down

   sim-smoke:
    ./scripts/hardware_free_validation.sh

   camera-mock-up:
    docker compose --profile mock_camera up --build camera_server_mock

   camera-real-up:
    docker compose --profile real_camera up --build camera_server
   ```

### Acceptance criteria

- `docker compose up --build rb_gui rb_simulator_left rb_simulator_right rb_servo_server`가 real camera 없이 시작된다.
- `rb_servo_server`가 left/right simulator endpoint에 각각 접속한다.
- `rb_gui`는 state를 수신하고 command를 `rb_servo_server`로 보낸다.
- default compose가 privileged camera container를 띄우지 않는다.
- root compose에 `rbsim_local` env가 남지 않는다.

---

## P0-F. hardware-free validation과 dependency preflight 정리

### 담당 모듈

- scripts
- dev environment

### 수정/추가 파일

- `scripts/check_deps.sh` 추가
- `scripts/install_deps_ubuntu.sh` 추가 권장
- `scripts/hardware_free_validation.sh` 수정
- `docs/hardware_free_validation.md` 수정
- `.gitignore` 수정

### 현재 문제

`hardware_free_validation.sh`는 dependency가 없을 때 CMake 단계에서 갑자기 실패한다. 또한 full simulator smoke가 `rb_simulator/build/rb_simulator` executable을 기대하는데 현재 simulator는 Python module이다.

### 작업 내용

1. dependency preflight script를 추가한다.

   ```bash
   ./scripts/check_deps.sh --profile hardware-free
   ./scripts/check_deps.sh --profile real-camera
   ./scripts/check_deps.sh --profile real-robot
   ./scripts/check_deps.sh --profile kinematics
   ```

2. hardware-free profile에서 확인할 항목:

   ```text
   cmake
   C++17 compiler
   yaml-cpp
   nlohmann_json
   python3
   python3 venv optional
   ```

3. real-camera profile에서 확인할 항목:

   ```text
   librealsense2-dev
   libzmq3-dev
   udev rules 안내
   USB access 안내
   ```

4. kinematics profile에서 확인할 항목:

   ```text
   pinocchio
   eigen3
   ```

5. hardware-free validation을 per-arm simulator smoke로 바꾼다.

   기존:

   ```text
   one dual-arm simulator process
   ```

   변경:

   ```text
   left simulator process + right simulator process + rb_servo_server
   ```

6. `RBSIM_EXECUTABLE` 대신 command array 또는 module mode를 지원한다.

   예:

   ```bash
   RBSIM_COMMAND="python3 -m rbsim"
   PYTHONPATH="${ROOT_DIR}/rb_simulator/src"
   ```

7. smoke artifact에 다음을 저장한다.

   ```text
   artifacts/
     state_stream.jsonl
     servo_log.csv
     left_simulator.log
     right_simulator.log
     rb_servo_server.log
     summary.json
   ```

8. `.gitignore`에 build/artifact/log/local config를 정리한다.

   ```gitignore
   **/build/
   **/logs/
   **/artifacts/
   rb_servo_server/config/local/
   calibration/local/
   geometry/local/
   ```

### Acceptance criteria

- dependency가 없을 때 CMake error 전에 `check_deps.sh`가 명확한 설치 안내를 출력한다.
- full smoke가 두 simulator process를 띄운다.
- smoke failure 시 left/right/server log를 artifact로 남긴다.
- `./scripts/hardware_free_validation.sh`가 P0 완료 기준으로 green이어야 한다.

---

## P0-G. camera_server dependency/config validation 정리

### 담당 모듈

- `camera_server`

### 수정/추가 파일

- `camera_server/src/config/config.cpp`
- `camera_server/config/triple_realsense.yaml`
- `camera_server/config/triple_realsense_640x360.yaml` 추가 또는 rename
- `camera_server/config/triple_realsense_640x480.yaml` 추가 권장
- `camera_server/docs/config_schema.md`
- `camera_server/docs/hardware_acceptance_runbook.md`
- `scripts/check_deps.sh`

### 현재 문제

- `yaml-cpp`가 없으면 mock build도 실패한다.
- real config serial이 `REPLACE_*` placeholder여도 validation에서 허용된다.
- wrist D405 resolution이 `640x360`인지 `640x480`인지 source-of-truth가 애매하다.
- `sync.mode`와 `sync.bundle_policy` 조합이 서로 모순될 수 있다.
- reconnect config는 있지만 reconnect 구현이 없다.

### 작업 내용

1. required camera serial validation을 강화한다.

   Fail values:

   ```text
   ""
   "REPLACE_*"
   "TODO"
   "CHANGEME"
   "UNKNOWN"
   ```

   단, `server.simulate_cameras: true`인 mock config의 `MOCK_*` serial은 허용한다.

2. sync 조합 validation을 추가한다.

   초기 정책:

   ```text
   sync.mode=software -> bundle_policy=nearest_timestamp only
   sync.mode=hardware -> bundle_policy=frame_number only
   ```

3. reconnect validation을 명확히 한다.

   현재 reconnect가 구현되어 있지 않다면:

   ```text
   reconnect.enabled=true -> fail with "reconnect is not implemented yet"
   ```

   또는 startup warning이 아니라 config validation fail로 바꾼다.

4. wrist resolution config를 분리한다.

   Canonical default:

   ```text
   head D435f: 1280x720@30 rgb8
   wrist D405: 640x360@30 rgb8
   depth: disabled
   ```

   파일:

   ```text
   camera_server/config/triple_realsense_640x360.yaml
   camera_server/config/triple_realsense_640x480.yaml
   ```

   `triple_realsense.yaml`은 canonical default를 가리키거나 동일 내용으로 유지한다.

5. docs에 dependency install을 명확히 추가한다.

   예:

   ```bash
   sudo apt install cmake build-essential libyaml-cpp-dev nlohmann-json3-dev libzmq3-dev
   # real camera only
   sudo apt install librealsense2-dev
   ```

6. `camera_server/tools/list_realsense_devices`는 P1/P2로 미뤄도 되지만, P0 docs에는 필요성을 남긴다.

### Acceptance criteria

- `REPLACE_HEAD_SERIAL`가 포함된 real config는 validation fail한다.
- `mock_triple_realsense.yaml`은 validation pass한다.
- `software + frame_number` 조합은 fail한다.
- `hardware + nearest_timestamp` 조합은 fail한다.
- `reconnect.enabled=true`는 구현 전까지 fail한다.
- docs에 hardware-free deps와 real-camera deps가 분리되어 있다.

---

## P0-H. rb_gui safety/mode mismatch와 TCP command UX 수정

### 담당 모듈

- `rb_gui`

### 수정/추가 파일

- `rb_gui/rb_servo_gui/safety.py`
- `rb_gui/rb_servo_gui/models.py`
- `rb_gui/rb_servo_gui/app.py`
- `rb_gui/rb_servo_gui/command_client.py`
- `rb_gui/tests/test_gui_contracts.py`
- root `docker-compose.yml`

### 현재 문제

- compose에서는 `RB_GUI_OBSERVED_MODE=rbsim_local`인데 GUI type은 `mock|simulation|real`이다.
- GUI command target env가 simulator를 가리키고 있다.
- TCP pose sender가 FK/IK 구현 전에도 command를 보낼 수 있다.
- `ArmSnapshot.parse()`가 `has_valid_joint_state=false`면 snapshot 전체를 버리므로 fault/status visibility가 손실될 수 있다.

### 작업 내용

1. GUI mode/backend를 분리한다.

   ```text
   RB_GUI_OBSERVED_MODE=simulation
   RB_GUI_OBSERVED_BACKEND=simulator
   ```

2. `Mode`는 계속 다음만 허용한다.

   ```python
   Mode = Literal["mock", "simulation", "real"]
   ```

3. backend display type을 추가한다.

   ```python
   Backend = Literal["mock", "simulator", "rbpodo", "unknown"]
   ```

4. `rbsim_local` env가 들어오면 deprecated alias로 `simulation/simulator`로 normalize하되 warning/status에 표시한다.

5. `send_tcp_pose_target()`는 P3 전까지 command를 보내지 않는다.

   변경:

   ```python
   return False, "TCP pose command disabled until FK/IK milestone is enabled"
   ```

   단, P3에서 다시 enable할 수 있도록 feature flag를 둔다.

   ```text
   RB_GUI_ENABLE_TCP_POSE_COMMANDS=0 default
   ```

6. `control_disabled_states()`에서 `tcp_pose`도 항상 disabled로 표시한다. P3에서 FK/IK readiness가 true일 때만 enable한다.

7. `ArmSnapshot.parse()`는 joint arrays가 invalid여도 status snapshot을 유지하도록 바꾼다.

   권장 model:

   ```python
   @dataclass(frozen=True)
   class ArmSnapshot:
       q_actual_deg: tuple[float, ...] | None
       q_sent_deg: tuple[float, ...] | None
       q_previous_sent_deg: tuple[float, ...] | None
       has_valid_joint_state: bool
       connection_state: str
       error_code: int | None
       tcp_stand: Mapping[str, Any] | None
       tcp_base: Mapping[str, Any] | None
   ```

8. GUI stale state와 invalid joint state를 구분해 표시한다.

   ```text
   state stream missing/stale
   joint state invalid
   robot/backend fault
   server fault latched
   ```

### Acceptance criteria

- compose simulation mode에서 GUI safety가 mode mismatch로 motion을 막지 않는다.
- `rbsim_local`은 public env에서 제거된다.
- TCP pose button/command는 P3 feature flag 전까지 disabled이다.
- invalid joint state packet이 와도 GUI는 fault/status를 볼 수 있다.
- `python3 -m unittest discover rb_gui/tests` 통과.

---

## P0-I. force control을 null provider로 고정

### 담당 모듈

- `rb_servo_server` force control path

### 수정/추가 파일

- `rb_servo_server/include/rb_servo/control/force_controller.hpp`
- `rb_servo_server/src/control/force_controller.cpp`
- `rb_servo_server/include/rb_servo/config/config.hpp`
- `rb_servo_server/src/config/config.cpp`
- `rb_servo_server/src/control/dual_arm_servo_loop.cpp`
- `rb_servo_server/docs/force_control.md`

### 현재 문제

force/admittance scaffold가 있지만 active path가 아니다. 이후 사내 `mo_forcecontroller`를 사용할 계획이므로 지금 임시 force controller를 활성화하면 혼란과 위험이 생긴다.

### 작업 내용

1. config canonical form을 다음으로 정한다.

   ```yaml
   force_control:
     provider: null
     enable: false
   ```

2. `ForceControlProvider` interface를 둘 수 있다.

   ```cpp
   class IForceControlProvider {
   public:
       virtual ~IForceControlProvider() = default;
       virtual bool enabled() const = 0;
   };

   class NullForceControlProvider final : public IForceControlProvider {
       bool enabled() const override { return false; }
   };
   ```

3. command에 `force_control.mode != Off`가 들어오면 provider가 null일 때 fail-closed한다.

   권장 verdict:

   ```text
   InvalidCommand
   ```

   또는 새 verdict:

   ```text
   ForceControlUnavailable
   ```

4. docs에 명확히 쓴다.

   ```text
   Force control is not implemented in P0~P3.
   mo_forcecontroller integration is a future milestone.
   ```

### Acceptance criteria

- `force_control.enable=true` 또는 `provider != null`은 구현 전까지 config validation fail한다.
- `force_control.mode=Admittance` command는 silent ignore가 아니라 reject/hold 처리된다.
- real/simulation joint-only path에는 force controller output이 개입하지 않는다.

---

# P1. Joint-only real/sim isomorphism and policy_runner

P1 목표:

```text
- simulator와 real backend가 모두 “팔 하나당 endpoint 하나” 구조로 동작한다.
- rb_servo_server는 real read-only mode를 지원한다.
- RbpodoBackend는 최소 connect/readState까지 붙인다.
- Python policy_runner를 추가하고 Hold / scripted joint / SpaceMouse joint velocity action을 보낼 수 있게 한다.
```

---

## P1-A. rb_servo_server read-only mode 추가

### 담당 모듈

- `rb_servo_server` servo loop safety

### 수정/추가 파일

- `rb_servo_server/include/rb_servo/config/config.hpp`
- `rb_servo_server/src/config/config.cpp`
- `rb_servo_server/src/control/dual_arm_servo_loop.cpp`
- `rb_servo_server/include/rb_servo/core/types.hpp`
- `rb_servo_server/src/network/state_publisher.cpp`
- `rb_servo_server/docs/fail_safe_policy.md`
- `rb_servo_server/tests/test_safety_policy.cpp`

### 현재 문제

현재 servo loop는 매 tick `sendServoJ()`를 호출한다. 실제 robot read-only acceptance를 하려면 connect/readState만 해야 하며 servo command send가 없어야 한다.

### 작업 내용

1. `ServoConfig`에 send gate를 추가한다.

   ```cpp
   bool send_servo_commands = true;
   ```

2. real mode에서 아래 정책을 적용한다.

   ```text
   run_mode=real && servo.send_servo_commands=true
     -> RB_ALLOW_REAL_MOTION=1 필요

   run_mode=real && servo.send_servo_commands=false
     -> RB_ALLOW_REAL_ROBOT=1만 필요
     -> read-only mode
   ```

3. `sendTargets()`에서 `send_servo_commands=false`일 때 backend `sendServoJ()`를 호출하지 않는다.

4. read-only mode에서 motion command가 들어오면 fail-closed한다.

   ```text
   ArmMotion / JointTarget / JointVelocity / Tcp* -> blocked
   Hold / EmergencyStop / ResetFault -> 허용 여부 명확히 정의
   ```

   권장:

   - Hold: 허용, no send
   - EmergencyStop: backend stop을 호출할지 여부는 real 안전 검토 전까지 no-op 또는 explicit disabled
   - ResetFault: read-only에서는 backend reset을 호출하지 않거나 별도 env gate 필요

5. state message에 send suppression을 표시한다.

   ```json
   "send_suppressed": true,
   "send_ok": true,
   "send_policy": "read_only"
   ```

   또는 `send_ok=false`로 두면 기존 safety가 fault로 해석할 수 있으므로 read-only에서는 별도 field가 필요하다.

6. logger에도 send policy를 남긴다.

### Acceptance criteria

- `servo.send_servo_commands=false`이면 backend `sendServoJ()`가 호출되지 않는다.
- read-only mode에서 state publish는 계속 된다.
- read-only mode에서 motion command는 robot command로 전송되지 않는다.
- real + send enabled + `RB_ALLOW_REAL_MOTION` 없음은 startup fail한다.
- mock/simulator에서도 read-only mode test가 가능해야 한다.

---

## P1-B. RbpodoBackend connect/readState 구현

### 담당 모듈

- `rb_servo_server` real backend

### 수정/추가 파일

- `rb_servo_server/CMakeLists.txt`
- `rb_servo_server/include/rb_servo/robot/rbpodo_backend.hpp`
- `rb_servo_server/src/robot/rbpodo_backend.cpp`
- `rb_servo_server/config/dual_real.example.yaml`
- `rb_servo_server/config/local/.gitkeep` 추가
- `rb_servo_server/docs/rbpodo_backend_plan.md` 최신화
- `rb_servo_server/docs/first_real_robot_motion.md` 추가 권장

### 현재 문제

`RbpodoBackend`는 `RB_SERVO_ENABLE_RBPODO=ON`이어도 실제 connect/read/send 구현이 없다.

### 작업 내용

1. CMake에서 `RB_SERVO_ENABLE_RBPODO=ON`일 때만 rbpodo를 찾는다.

   ```cmake
   option(RB_SERVO_ENABLE_RBPODO "Enable rbpodo backend" OFF)
   if(RB_SERVO_ENABLE_RBPODO)
     find_package(rbpodo REQUIRED)
     target_link_libraries(rb_servo_core PUBLIC rbpodo::rbpodo)
   endif()
   ```

2. rbpodo include는 `rbpodo_backend.cpp` 내부에만 둔다.

   ```cpp
   #ifdef RB_SERVO_ENABLE_RBPODO
   #include <...actual rbpodo header...>
   #endif
   ```

3. `RbpodoBackend::connect()`를 구현한다.

   - `config.ip` 사용
   - left: `172.28.60.200`
   - right: `172.28.60.201`
   - connection failure는 false
   - real mode에서는 `RB_ALLOW_REAL_ROBOT=1` 확인

4. `initialize()`는 read-only safe initialization만 수행한다.

   - operation mode 설정이 필요하다면 실제 rbpodo API에 맞춰 구현
   - speed bar 설정이 safe/read-only인지 검토
   - servo enable/motion mode 진입은 P1에서는 금지 또는 `send_servo_commands=true`일 때만 허용

5. `readState()`를 구현한다.

   채워야 할 field:

   ```cpp
   out_state.arm_id
   out_state.host_time_ns
   out_state.robot_time_ns, 가능하면
   out_state.q_actual_deg
   out_state.q_target_deg, 없으면 q_actual_deg 또는 last target
   out_state.dq_actual_deg_s, 없으면 0 또는 API feedback
   out_state.has_valid_joint_state
   out_state.connection_state
   out_state.servo_enabled
   out_state.has_error
   out_state.error_code
   ```

6. `sendServoJ()`는 P1에서 두 단계로 구현한다.

   - read-only config: 항상 호출되지 않아야 함
   - motion config + `RB_ALLOW_REAL_MOTION=1`: 실제 `servo_j` API 호출 가능

   실제 rbpodo API 이름은 설치된 rbpodo header/docs를 기준으로 확인하고 구현한다. API 이름을 추측해서 fake 구현하지 않는다.

7. `stop()`과 `resetFault()`는 실제 robot API 확인 전까지 매우 보수적으로 구현한다.

   - stop은 safe stop API가 명확할 때만 호출
   - resetFault는 별도 gate 없이 real에 호출하지 않는 것을 권장

8. real config template을 만든다.

   ```yaml
   schema: robotics_lab.rb_servo_server.v1

   left_robot:
     backend_type: rbpodo
     run_mode: real
     name: left_rb3
     ip: "172.28.60.200"
     operation_mode: real
     initial_q_deg: [0, -30, 80, 0, 60, 0]

   right_robot:
     backend_type: rbpodo
     run_mode: real
     name: right_rb3
     ip: "172.28.60.201"
     operation_mode: real
     initial_q_deg: [0, -30, 80, 0, 60, 0]

   servo:
     rate_hz: 200
     command_timeout_sec: 0.2
     send_servo_commands: false
     enable_realtime_priority: true

   safety:
     tracking_error_policy: fault_latch
     stop_both_arms_on_single_arm_error: true
     latch_fault_on_robot_state_error: true
   ```

9. `dual_real.yaml`은 runnable처럼 보이지 않게 바꾼다.

   권장:

   ```text
   rb_servo_server/config/dual_real.example.yaml
   rb_servo_server/config/local/dual_real_readonly.yaml  # gitignored
   rb_servo_server/config/local/dual_real_motion.yaml    # gitignored
   ```

### Acceptance criteria

- `RB_SERVO_ENABLE_RBPODO=OFF`일 때 build/test가 계속 통과한다.
- `RB_SERVO_ENABLE_RBPODO=ON`이고 rbpodo가 설치되어 있으면 build된다.
- real read-only config는 `RB_ALLOW_REAL_ROBOT=1` 없이는 fail한다.
- real motion config는 `RB_ALLOW_REAL_MOTION=1` 없이는 fail한다.
- read-only mode에서는 `sendServoJ()`가 호출되지 않는다.
- real read-only로 left/right state를 1분 이상 publish할 수 있다.

---

## P1-C. per-arm simulator + rb_servo_server integration smoke 갱신

### 담당 모듈

- integration tests
- smoke tools

### 수정/추가 파일

- `rb_simulator/tools/rbsim_servo_smoke.py`
- `rb_servo_server/tests/test_rbsim_hardware_free_gate.py`
- `scripts/hardware_free_validation.sh`
- `rb_servo_server/config/dual_simulator.yaml`
- `rb_servo_server/config/dual_simulator_compose.yaml`

### 현재 문제

현재 smoke runner는 “one dual-arm simulator process”라고 명시되어 있고 그 구조를 launch한다.

### 작업 내용

1. smoke runner CLI를 per-arm으로 바꾼다.

   예:

   ```bash
   python3 rb_simulator/tools/rbsim_servo_smoke.py \
     --left-simulator-command "python3 -m rbsim" \
     --left-simulator-config rb_simulator/config/left_rb3_730e.yaml \
     --right-simulator-command "python3 -m rbsim" \
     --right-simulator-config rb_simulator/config/right_rb3_730e.yaml \
     --server rb_servo_server/build/hardware_free_gate/rb_servo_server \
     --server-config rb_servo_server/config/dual_simulator.yaml
   ```

2. dynamic port allocation test를 per-arm으로 바꾼다.

   `test_rbsim_hardware_free_gate.py`는 다음을 생성해야 한다.

   ```text
   left_control_port
   left_admin_port
   right_control_port
   right_admin_port
   command_port
   state_port
   ```

3. generated test config는 canonical 용어를 사용한다.

   ```yaml
   backend_type: simulator
   run_mode: simulation
   simulator_control_endpoint: "tcp://127.0.0.1:${left_control_port}"
   ```

4. smoke에서 wrong-arm request test를 포함한다.

   - left admin/control에 right arm request -> error
   - right admin/control에 left arm request -> error

5. state stream validation은 left/right가 서로 다른 simulator process로부터 온다는 것을 확인할 수 있는 field를 추가한다.

   권장:

   - simulator state에 `controller_id` 또는 `simulator_name` 추가
   - state publisher에 backend name 포함

### Acceptance criteria

- per-arm smoke가 green이다.
- 기존 dual-arm smoke 문구가 docs/tools에서 제거된다.
- wrong-arm request가 test된다.
- artifact에 left/right simulator log가 분리되어 남는다.

---

## P1-D. Python policy_runner skeleton 추가

### 담당 모듈

- 새 모듈 `policy_runner`

### 추가 파일

- `policy_runner/pyproject.toml`
- `policy_runner/policy_runner/__init__.py`
- `policy_runner/policy_runner/main.py`
- `policy_runner/policy_runner/config.py`
- `policy_runner/policy_runner/robot_state_client.py`
- `policy_runner/policy_runner/servo_command_client.py`
- `policy_runner/policy_runner/safety.py`
- `policy_runner/policy_runner/action_sources/__init__.py`
- `policy_runner/policy_runner/action_sources/hold.py`
- `policy_runner/policy_runner/action_sources/joint_sine.py`
- `policy_runner/policy_runner/action_sources/joint_velocity.py`
- `policy_runner/tests/test_policy_runner_contract.py`
- `policy_runner/README.md`

### 목표

`policy_runner`는 future neural policy와 SpaceMouse teleop의 공통 action source가 된다.

### 작업 내용

1. Python package skeleton을 만든다.

   CLI 예:

   ```bash
   python3 -m policy_runner \
     --config policy_runner/config/simulator_hold.yaml
   ```

2. UDP state subscriber를 구현한다.

   - `rb_servo_server` state publisher 수신
   - latest snapshot cache
   - stale 판단

3. UDP command sender를 구현한다.

   - `ArmMotion`
   - `DisarmMotion`
   - `Hold`
   - `JointTarget`
   - `JointVelocity`
   - `EmergencyStop`
   - `ResetFault`

4. 초기 action source 3개를 만든다.

   ```text
   hold:
     state만 보고 Hold/no-op 유지

   joint_sine:
     simulator only
     작은 sine joint target

   joint_velocity:
     fixed small velocity command
     simulator only by default
   ```

5. policy safety를 추가한다.

   Command를 보내지 말아야 하는 조건:

   ```text
   state stream stale
   fault_latched true
   motion_state FaultLatched/EmergencyLatched
   observed_mode real and allow_real_motion false
   action source requires camera but camera stale
   action source requires kinematics but kinematics unavailable
   ```

6. policy runner config 예시를 만든다.

   ```yaml
   schema: robotics_lab.policy_runner.v1
   mode: simulation
   action_source: hold
   robot_state:
     bind: "udp://127.0.0.1:50120"
     stale_timeout_sec: 0.5
   servo_command:
     endpoint: "udp://127.0.0.1:50010"
     timeout_sec: 0.2
   safety:
     allow_real_motion: false
     require_valid_joint_state: true
   ```

7. GUI와 policy_runner가 동시에 command를 보낼 수 있는 충돌 문제를 문서화한다.

   초기 정책:

   ```text
   command source는 하나만 active.
   GUI teleop와 policy_runner teleop를 동시에 켜지 않는다.
   ```

### Acceptance criteria

- `policy_runner` unit tests가 통과한다.
- hold mode가 state를 수신하고 unsafe 시 command를 보내지 않는다.
- joint_sine mode가 simulator에서만 동작한다.
- real mode에서는 explicit allow 없이 motion command를 보내지 않는다.
- command packet schema가 `rb_servo_server` parser와 호환된다.

---

## P1-E. SpaceMouse joint velocity teleop를 policy_runner에 추가

### 담당 모듈

- `policy_runner`
- optional HID dependency

### 수정/추가 파일

- `policy_runner/policy_runner/spacemouse.py`
- `policy_runner/policy_runner/action_sources/spacemouse_joint_velocity.py`
- `policy_runner/pyproject.toml`
- `policy_runner/tests/test_spacemouse_mapping.py`
- `policy_runner/README.md`

### 목표

FK/IK 전에는 SpaceMouse를 Cartesian motion으로 쓰지 않는다. 우선 joint velocity teleop만 구현한다.

### 작업 내용

1. SpaceMouse input abstraction을 만든다.

   ```python
   @dataclass(frozen=True)
   class SpaceMouseSample:
       tx: float
       ty: float
       tz: float
       rx: float
       ry: float
       rz: float
       buttons: tuple[bool, ...]
       timestamp_monotonic: float
   ```

2. 실제 HID reader와 fake reader를 분리한다.

   ```text
   SpaceMouseReader interface
   HidSpaceMouseReader
   FakeSpaceMouseReader for tests
   ```

3. dependency는 optional로 둔다.

   ```toml
   [project.optional-dependencies]
   spacemouse = ["hidapi", "pyspacemouse"]
   ```

   실제 library 선택은 구현 agent가 확인한다. Tests는 fake reader로 돌린다.

4. deadman switch를 필수로 한다.

   정책 예:

   ```text
   button 0 pressed -> command active
   button 0 released -> Hold 또는 zero velocity
   ```

5. joint velocity mapping은 단순하고 명확하게 시작한다.

   예:

   ```text
   selected_arm: left | right | both
   axis tx -> J1 velocity
   axis ty -> J2 velocity
   axis tz -> J3 velocity
   axis rx -> J4 velocity
   axis ry -> J5 velocity
   axis rz -> J6 velocity
   ```

   이 mapping은 Cartesian이 아니라 joint velocity mapping이다. README에 명확히 쓴다.

6. clamp를 둔다.

   ```yaml
   spacemouse:
     max_joint_velocity_deg_s: [5, 5, 5, 8, 8, 10]
     deadband: 0.08
     smoothing_alpha: 0.2
     require_deadman: true
   ```

7. command rate를 servo rate보다 낮게 둔다.

   권장:

   ```text
   policy_runner command rate: 30~60 Hz
   rb_servo_server servo loop: 100~200 Hz
   ```

### Acceptance criteria

- fake SpaceMouse sample로 deterministic joint velocity command가 생성된다.
- deadman이 false면 motion command가 나가지 않는다.
- output velocity가 clamp된다.
- real mode에서는 `allow_real_motion=false`일 때 command가 나가지 않는다.
- FK/IK 없이도 simulator에서 joint velocity teleop가 가능하다.

---

## P1-F. Real/simulator config templates 정리

### 담당 모듈

- config
- docs

### 수정/추가 파일

- `rb_servo_server/config/dual_simulator.yaml`
- `rb_servo_server/config/dual_simulator_compose.yaml`
- `rb_servo_server/config/dual_mock.yaml`
- `rb_servo_server/config/dual_real.example.yaml`
- `rb_servo_server/config/local/.gitkeep`
- `.gitignore`
- `rb_servo_server/docs/config_examples.md`

### 작업 내용

1. simulator host-run config:

   ```yaml
   left_robot:
     backend_type: simulator
     run_mode: simulation
     name: left_simulator
     simulator_control_endpoint: "tcp://127.0.0.1:50200"

   right_robot:
     backend_type: simulator
     run_mode: simulation
     name: right_simulator
     simulator_control_endpoint: "tcp://127.0.0.1:50210"
   ```

2. simulator compose config:

   ```yaml
   left_robot:
     simulator_control_endpoint: "tcp://rb_simulator_left:50200"

   right_robot:
     simulator_control_endpoint: "tcp://rb_simulator_right:50200"
   ```

3. real example config:

   ```yaml
   left_robot:
     backend_type: rbpodo
     run_mode: real
     ip: "172.28.60.200"

   right_robot:
     backend_type: rbpodo
     run_mode: real
     ip: "172.28.60.201"
   ```

4. local real configs are gitignored.

   ```gitignore
   rb_servo_server/config/local/*.yaml
   ```

### Acceptance criteria

- No canonical config uses `rbsim_local`.
- Real example contains correct left/right IPs.
- Local real config directory exists but user-specific yaml is gitignored.

---

# P2. Forward kinematics and TCP state publish

P2 목표:

```text
- FK를 구현해서 tcp_base / tcp_stand를 state stream에 publish한다.
- GUI는 FK 기반 TCP marker를 볼 수 있다.
- 아직 IK/TCP command는 활성화하지 않는다.
- measured calibration이 없어도 geometry registry로 frame contract를 명확히 한다.
```

---

## P2-A. Kinematics module과 Pinocchio integration 추가

### 담당 모듈

- `rb_servo_server` kinematics

### 수정/추가 파일

- `rb_servo_server/CMakeLists.txt`
- `rb_servo_server/include/rb_servo/kinematics/i_kinematics.hpp` 추가
- `rb_servo_server/include/rb_servo/kinematics/pinocchio_kinematics.hpp` 추가
- `rb_servo_server/src/kinematics/pinocchio_kinematics.cpp` 추가
- `rb_servo_server/include/rb_servo/config/config.hpp`
- `rb_servo_server/src/config/config.cpp`
- `rb_servo_server/descriptions/urdf/rb3_730e.urdf`
- `rb_servo_server/tests/test_kinematics_fk.cpp` 추가

### 작업 내용

1. CMake option을 추가한다.

   ```cmake
   option(RB_SERVO_ENABLE_PINOCCHIO "Enable Pinocchio FK/IK" OFF)
   if(RB_SERVO_ENABLE_PINOCCHIO)
     find_package(pinocchio REQUIRED)
     target_link_libraries(rb_servo_core PUBLIC pinocchio::pinocchio)
     target_compile_definitions(rb_servo_core PUBLIC RB_SERVO_ENABLE_PINOCCHIO=1)
   endif()
   ```

2. config에 kinematics section을 추가한다.

   ```yaml
   kinematics:
     enable: true
     provider: pinocchio
     urdf: "descriptions/urdf/rb3_730e.urdf"
     base_link: "world"
     tip_link: "tcp"
     joint_names:
       - base_joint
       - shoulder_joint
       - elbow_joint
       - wrist1_joint
       - wrist2_joint
       - wrist3_joint
     q_units: deg
     publish_tcp: true
   ```

3. `IKinematics` interface를 만든다.

   ```cpp
   class IKinematics {
   public:
       virtual ~IKinematics() = default;
       virtual Pose6D computeTcpBase(const JointArray& q_deg) const = 0;
       virtual Pose6D computeTcpStand(
           ArmId arm,
           const JointArray& q_deg,
           const ArmMountConfig& mount
       ) const = 0;
   };
   ```

4. Pinocchio 내부는 radian을 사용하고 external protocol은 degree를 유지한다.

5. `Pose6D`의 rotation convention을 명확히 한다.

   P2에서는 publish 용도로만 `rx, ry, rz`를 사용한다. 내부 계산은 matrix/quaternion/SE(3)를 사용하고, `Pose6D` 변환은 serialization boundary에서 수행한다.

6. URDF의 `base_link`, `tip_link`, joint_names가 실제와 맞는지 test로 검증한다.

   - joint count 6
   - tip link exists
   - base link exists
   - q=initial에서 finite pose

### Acceptance criteria

- Pinocchio disabled build는 계속 통과한다.
- Pinocchio enabled build에서 FK test가 통과한다.
- q input degree -> internal radian conversion이 test된다.
- FK result가 finite pose를 반환한다.
- missing/invalid URDF path는 config validation fail한다.

---

## P2-B. geometry/calibration registry 추가

### 담당 모듈

- geometry/frame source-of-truth

### 추가 파일

- `calibration/active_calibration.yaml` 또는 `geometry/active_setup.yaml`
- `calibration/README.md` 또는 `geometry/README.md`
- `docs/frame_contract.md` 수정

### 목적

지금 당장 measured hand-eye calibration을 하자는 뜻이 아니다. 하지만 FK/TCP/camera/stand frame을 각 모듈이 제멋대로 해석하지 않도록 source-of-truth를 둔다.

### 권장 명칭

둘 중 하나를 선택한다.

```text
calibration/active_calibration.yaml
```

또는 부담을 줄이려면:

```text
geometry/active_setup.yaml
```

향후 measured calibration까지 이어갈 계획이면 `calibration/`을 추천한다.

### 작업 내용

1. active geometry file을 추가한다.

   예:

   ```yaml
   schema: robotics_lab.calibration.v1
   calibration_id: "CONFIG_ESTIMATE_001"
   status: configured_estimate
   geometry_valid_for_real_policy: false

   robot:
     T_stand_left_base:
       xyz_rpy: [0.1601, -0.1725, 0.5825, 2.186649, 0.523831, 2.526296]
       status: configured_estimate
     T_stand_right_base:
       xyz_rpy: [-0.1601, -0.1725, 0.5825, 2.186649, -0.523831, -2.526296]
       status: configured_estimate

   cameras:
     head:
       serial: "REPLACE_HEAD_SERIAL"
       intrinsics_status: unknown
       extrinsics_status: unmeasured
     left_wrist:
       serial: "REPLACE_LEFT_SERIAL"
       hand_eye_status: unmeasured
     right_wrist:
       serial: "REPLACE_RIGHT_SERIAL"
       hand_eye_status: unmeasured
   ```

2. `left_mount/right_mount`와 geometry registry의 관계를 문서화한다.

   정책:

   - servo config의 mount transform은 현재 servo runtime source
   - geometry registry는 global source-of-truth
   - P2 후에는 servo config가 geometry registry를 include/load하거나 두 값의 mismatch를 warning/fail

3. `geometry_valid_for_real_policy=false`일 때 policy_runner가 geometry-dependent policy를 실행하지 않게 한다.

4. P2에서 camera extrinsics는 unmeasured 상태로 둔다.

### Acceptance criteria

- frame contract에 `stand`, `left_base`, `right_base`, `tcp`, `camera` frame 관계가 명시된다.
- measured calibration이 없어도 FK/TCP publish는 configured estimate로 가능하다.
- real geometry-based policy는 `geometry_valid_for_real_policy=false`일 때 blocked된다.

---

## P2-C. rb_servo_server state에 FK TCP pose 채우기

### 담당 모듈

- `rb_servo_server` servo loop/state publisher

### 수정/추가 파일

- `rb_servo_server/src/control/dual_arm_servo_loop.cpp`
- `rb_servo_server/include/rb_servo/control/dual_arm_servo_loop.hpp`
- `rb_servo_server/src/network/state_publisher.cpp`
- `rb_servo_server/include/rb_servo/core/types.hpp`
- `rb_servo_server/tests/test_kinematics_fk.cpp`
- `rb_servo_server/tests/test_state_publisher.cpp` 추가 권장

### 현재 문제

`RobotState`에는 `tcp_base`, `tcp_stand` field가 있지만 publisher는 항상 null을 보낸다.

### 작업 내용

1. `DualArmServoLoop`에 optional `IKinematics` provider를 주입한다.

   방법 중 하나:

   ```cpp
   DualArmServoLoop(..., std::shared_ptr<IKinematics> kinematics)
   ```

   또는 내부 factory:

   ```cpp
   kinematics_ = makeKinematics(config.kinematics)
   ```

2. `readRobotStates()` 후 valid joint state이면 FK를 계산한다.

   ```cpp
   if (kinematics && state.has_valid_joint_state) {
       state.tcp_base = kinematics->computeTcpBase(state.q_actual_deg);
       state.tcp_stand = kinematics->computeTcpStand(arm, state.q_actual_deg, mount);
       state.has_valid_tcp_pose = true;  // 새 field 권장
   }
   ```

3. `RobotState`에 validity flag를 추가한다.

   ```cpp
   bool has_valid_tcp_pose = false;
   ```

4. state publisher를 수정한다.

   기존:

   ```json
   "tcp_stand": null,
   "tcp_base": null,
   "tcp_deferred": true
   ```

   변경:

   ```json
   "tcp_stand": {"x":..., "y":..., "z":..., "rx":..., "ry":..., "rz":...},
   "tcp_base": {...},
   "has_valid_tcp_pose": true,
   "tcp_deferred": false
   ```

   FK disabled이면 기존처럼 null + deferred true.

5. top-level state field도 수정한다.

   ```json
   "tcp_fields_deferred": false
   ```

   단, left/right 중 하나라도 invalid이면 per-arm validity를 기준으로 표시한다.

6. logger에 TCP pose를 추가할지 결정한다.

   P2에서는 optional. 최소 state publisher에만 추가해도 된다.

### Acceptance criteria

- FK enabled + valid q state이면 state stream에 tcp_base/tcp_stand가 null이 아니다.
- FK disabled이면 기존처럼 tcp fields deferred이다.
- invalid joint state이면 TCP pose도 invalid로 표시된다.
- state publisher tests가 pass한다.

---

## P2-D. GUI TCP marker visualization을 FK state 기반으로 표시

### 담당 모듈

- `rb_gui`

### 수정/추가 파일

- `rb_gui/rb_servo_gui/models.py`
- `rb_gui/rb_servo_gui/app.py`
- `rb_gui/tests/test_gui_contracts.py`

### 작업 내용

1. GUI model에 TCP pose field를 추가한다.

   ```python
   tcp_stand: Pose6D | None
   tcp_base: Pose6D | None
   has_valid_tcp_pose: bool
   ```

2. `tcp_deferred`와 `has_valid_tcp_pose`를 구분한다.

   ```text
   tcp_deferred=true:
     FK provider disabled or unavailable

   has_valid_tcp_pose=false:
     FK enabled but this arm has invalid q/state
   ```

3. viser scene에 TCP marker를 표시한다.

   - left TCP marker
   - right TCP marker
   - stand frame
   - base mount frame

4. P2에서는 marker만 표시한다. TCP command button은 여전히 disabled이다.

5. GUI status에 FK availability를 표시한다.

   ```text
   FK: available / deferred / invalid state
   ```

### Acceptance criteria

- state packet에 TCP pose가 있으면 GUI parser가 이를 유지한다.
- TCP pose가 null이어도 GUI state 전체를 버리지 않는다.
- TCP marker 표시가 command enable과 연결되지 않는다.
- TCP command는 P3 전까지 disabled이다.

---

## P2-E. policy_runner geometry awareness 추가

### 담당 모듈

- `policy_runner`

### 수정/추가 파일

- `policy_runner/policy_runner/geometry.py`
- `policy_runner/policy_runner/config.py`
- `policy_runner/policy_runner/safety.py`
- `policy_runner/tests/test_geometry_safety.py`

### 작업 내용

1. policy_runner가 geometry/calibration file을 읽을 수 있게 한다.

2. geometry-dependent action source를 실행하기 전에 status를 확인한다.

   ```text
   requires_geometry=true
   geometry_valid_for_real_policy=false
   mode=real
     -> block
   ```

3. simulation에서는 configured estimate geometry를 허용할 수 있다.

   ```yaml
   safety:
     allow_configured_estimate_geometry_in_simulation: true
     allow_configured_estimate_geometry_in_real: false
   ```

### Acceptance criteria

- joint-only action source는 geometry file이 없어도 동작한다.
- Cartesian/future camera policy는 geometry unavailable이면 blocked된다.
- real mode에서 configured estimate geometry는 blocked된다.

---

# P3. IK and TCP Pose command validation

P3 목표:

```text
- TCP Pose / TCP Delta command를 simulator에서 동작시킨다.
- IK는 Pinocchio FK/Jacobian 기반 numerical solver로 구현한다.
- GUI/policy_runner에서 TCP pose command를 simulation-only로 테스트한다.
- real Cartesian motion은 별도 gate 없이는 계속 닫아둔다.
```

---

## P3-A. IK solver 구현

### 담당 모듈

- `rb_servo_server` kinematics

### 수정/추가 파일

- `rb_servo_server/include/rb_servo/kinematics/i_kinematics.hpp`
- `rb_servo_server/include/rb_servo/kinematics/ik_solver.hpp` 추가
- `rb_servo_server/src/kinematics/ik_solver.cpp` 추가
- `rb_servo_server/src/kinematics/pinocchio_kinematics.cpp`
- `rb_servo_server/include/rb_servo/config/config.hpp`
- `rb_servo_server/src/config/config.cpp`
- `rb_servo_server/tests/test_kinematics_ik.cpp` 추가

### 작업 내용

1. `IKinematics`에 solve method를 추가한다.

   ```cpp
   struct IkResult {
       bool success = false;
       JointArray q_solution_deg{};
       double position_error_m = 0.0;
       double orientation_error_rad = 0.0;
       int iterations = 0;
       std::string reason;
   };

   virtual IkResult solveIk(
       ArmId arm,
       const Pose6D& target_tcp_stand,
       const JointArray& seed_q_deg,
       const ArmMountConfig& mount
   ) const = 0;
   ```

2. numerical Damped Least Squares IK를 구현한다.

   기본 algorithm:

   ```text
   q = seed
   repeat max_iterations:
     T_current = FK(q)
     error = log_SE3(T_current^-1 * T_target)
     J = frame Jacobian at tcp
     dq = J^T * (J J^T + lambda^2 I)^-1 * error
     clamp dq
     q = clamp_joint_limits(q + dq)
     if error < tolerance: success
   ```

3. config를 추가한다.

   ```yaml
   kinematics:
     ik:
       enable: true
       max_iterations: 50
       timeout_ms: 2.0
       damping: 0.001
       position_tolerance_m: 0.001
       orientation_tolerance_rad: 0.02
       max_step_deg: [2, 2, 2, 3, 3, 4]
   ```

4. joint limit을 solver 내부에도 적용한다.

   - safety filter가 최종 방어선
   - IK solver도 impossible q를 내면 안 된다.

5. singularity와 non-convergence를 명확히 report한다.

   ```text
   IkFailed
   reason: max_iterations / timeout / singular / joint_limit
   ```

6. DLS 연산에는 Eigen을 사용한다.

### Acceptance criteria

- FK(q_seed) pose를 target으로 넣으면 IK가 q_seed 근처 solution을 반환한다.
- 작은 TCP delta target에 대해 solution이 finite이고 joint limits 안에 있다.
- unreachable target은 success=false를 반환한다.
- non-convergence가 servo loop crash로 이어지지 않는다.

---

## P3-B. CartesianController를 실제 IK path에 연결

### 담당 모듈

- `rb_servo_server` control layer

### 수정/추가 파일

- `rb_servo_server/include/rb_servo/control/cartesian_controller.hpp`
- `rb_servo_server/src/control/cartesian_controller.cpp`
- `rb_servo_server/src/control/dual_arm_servo_loop.cpp`
- `rb_servo_server/include/rb_servo/core/types.hpp`
- `rb_servo_server/tests/test_cartesian_controller.cpp` 추가

### 현재 문제

`CartesianController::computeArmJointTarget()`는 현재 throw한다. `DualArmServoLoop::computeServoTarget()`는 Cartesian mode를 즉시 `CartesianUnavailable`로 처리한다.

### 작업 내용

1. `CartesianController`에 `IKinematics` provider를 주입한다.

   ```cpp
   CartesianController(
       const ArmMountConfig& left_mount,
       const ArmMountConfig& right_mount,
       std::shared_ptr<IKinematics> kinematics
   );
   ```

2. `TcpPoseTarget` 구현:

   ```text
   command.tcp_target_stand
   seed = state.q_actual_deg 또는 previous sent q
   IK target = tcp_target_stand
   return q_solution_deg
   ```

3. `TcpDeltaStand` 구현:

   ```text
   current = state.tcp_stand
   target = current ⊕ delta_in_stand
   IK(target)
   ```

   P3에서는 SE(3) composition을 사용한다. 단순 RPY addition만 사용하지 않는다.

4. `TcpDeltaLocal` 구현:

   ```text
   current = T_stand_tcp
   target = current * exp(delta_local)
   IK(target)
   ```

5. `DualArmServoLoop::computeServoTarget()`에서 Cartesian mode를 활성화한다.

   현재:

   ```cpp
   if (isCartesianMode(...)) return CartesianUnavailable;
   ```

   변경:

   ```cpp
   if Cartesian mode and kinematics/cartesian enabled:
       compute Cartesian target per arm
   else:
       CartesianUnavailable
   ```

6. 한쪽 arm만 Cartesian mode이고 다른 쪽은 Hold/Joint mode인 mixed command를 지원할지 결정한다.

   초기 권장:

   - left/right command mode를 독립적으로 처리
   - 단, packet-level timeout은 coupled 유지

7. IK 실패 시 safety verdict를 `IkFailed`로 둔다.

   - robot에는 previous safe target을 보낸다.
   - fault latch 여부는 config로 결정한다.
   - real mode에서는 IK failure가 repeated되면 fault latch 권장.

8. config gate를 추가한다.

   ```yaml
   cartesian_control:
     enable: true
     allow_in_simulation: true
     allow_in_real: false
   ```

   real에서 true로 하려면 별도 env 필요:

   ```text
   RB_ALLOW_REAL_CARTESIAN=1
   ```

### Acceptance criteria

- `TcpPoseTarget` command가 simulator에서 joint target으로 변환되어 motion한다.
- `TcpDeltaStand` small delta가 simulator에서 동작한다.
- `TcpDeltaLocal` small delta가 simulator에서 동작한다.
- IK 실패 시 robot command가 unsafe target으로 가지 않는다.
- FK/IK disabled build에서는 기존처럼 CartesianUnavailable이다.
- real mode에서는 `RB_ALLOW_REAL_CARTESIAN=1` 없이는 Cartesian command가 blocked된다.

---

## P3-C. TCP command protocol tests와 docs 갱신

### 담당 모듈

- command/state protocol
- docs/tests

### 수정/추가 파일

- `rb_servo_server/docs/network_protocol.md`
- `rb_servo_server/src/network/command_server.cpp`
- `rb_servo_server/tests/test_safety_policy.cpp`
- `rb_servo_server/tests/test_cartesian_command_parser.cpp` 추가 권장
- `rb_servo_server/tools/send_tcp_pose_target.py` 추가
- `rb_servo_server/tools/send_tcp_delta.py` 추가

### 작업 내용

1. TCP command JSON schema를 명확히 문서화한다.

   ```json
   {
     "schema_version": 1,
     "seq": 123,
     "mode": "TcpPoseTarget",
     "host_time_ns": 123456789,
     "timeout_sec": 0.2,
     "left": {
       "tcp_target_stand": [x, y, z, rx, ry, rz]
     },
     "right": {
       "tcp_target_stand": [x, y, z, rx, ry, rz]
     }
   }
   ```

2. units를 명확히 한다.

   ```text
   x,y,z: meter
   rx,ry,rz: radian, serialization convention documented
   q: degree
   dq: degree/sec
   ```

3. parser tests를 추가한다.

   - missing `tcp_target_stand` -> invalid command
   - array length != 6 -> reject
   - non-finite -> reject
   - valid command -> `has_tcp_target=true`

4. command send tool을 추가한다.

   ```bash
   python3 rb_servo_server/tools/send_tcp_pose_target.py \
     --left 0.3 0.1 0.5 0 3.14 0 \
     --right 0.3 -0.1 0.5 0 3.14 0
   ```

5. TCP delta send tool을 추가한다.

   ```bash
   python3 rb_servo_server/tools/send_tcp_delta.py --frame stand --left 0.01 0 0 0 0 0
   ```

### Acceptance criteria

- command parser rejects invalid TCP arrays.
- docs include units and frame names.
- tools can send valid TCP pose/delta packets to simulator stack.

---

## P3-D. GUI TCP Pose command를 simulation-only로 활성화

### 담당 모듈

- `rb_gui`

### 수정/추가 파일

- `rb_gui/rb_servo_gui/safety.py`
- `rb_gui/rb_servo_gui/app.py`
- `rb_gui/rb_servo_gui/command_client.py`
- `rb_gui/tests/test_gui_contracts.py`

### 작업 내용

1. feature flag를 추가한다.

   ```text
   RB_GUI_ENABLE_TCP_POSE_COMMANDS=1
   ```

2. enable 조건:

   ```text
   observed_mode == simulation
   observed_backend == simulator
   FK available
   IK/cartesian available in state/server readiness
   no fault latched
   state not stale
   ```

3. real mode에서는 계속 disabled이다.

   ```text
   real mode TCP command disabled until real Cartesian acceptance passes
   ```

4. GUI에서 TCP target은 current FK pose 기준으로 small delta를 만들도록 한다.

   초기에는 direct arbitrary pose input보다 아래가 안전하다.

   ```text
   +X / -X
   +Y / -Y
   +Z / -Z
   small roll/pitch/yaw
   ```

5. command 결과를 status에 표시한다.

   ```text
   sent TcpDeltaStand +0.005m x
   server verdict: Ok / IkFailed / CartesianUnavailable
   ```

### Acceptance criteria

- P2까지는 TCP command disabled.
- P3 feature flag + simulation에서만 TCP command enabled.
- real mode에서는 feature flag가 있어도 disabled.
- GUI test가 mode/backend/readiness 조건을 검증한다.

---

## P3-E. policy_runner Cartesian action source 추가

### 담당 모듈

- `policy_runner`

### 수정/추가 파일

- `policy_runner/policy_runner/action_sources/tcp_delta.py`
- `policy_runner/policy_runner/action_sources/spacemouse_cartesian.py`
- `policy_runner/policy_runner/safety.py`
- `policy_runner/tests/test_cartesian_action_source.py`

### 작업 내용

1. `tcp_delta` scripted action source를 추가한다.

   ```text
   simulator only
   small stand-frame delta
   command mode: TcpDeltaStand
   ```

2. SpaceMouse Cartesian mapping을 추가한다.

   단, P3에서는 simulation only이다.

   ```text
   tx,ty,tz -> translational TCP delta/twist
   rx,ry,rz -> rotational TCP delta/twist
   deadman required
   max linear step e.g. 0.002 m per command
   max angular step e.g. 0.01 rad per command
   ```

3. command rate와 clamp를 별도로 둔다.

   ```yaml
   spacemouse_cartesian:
     frame: stand  # or local
     command_rate_hz: 30
     max_linear_step_m: 0.002
     max_angular_step_rad: 0.01
     deadband: 0.08
     require_deadman: true
   ```

4. safety 조건:

   ```text
   FK/TCP state unavailable -> no command
   server cartesian unavailable -> no command or stop action
   geometry invalid in real -> no command
   observed_mode real -> no command unless allow_real_cartesian true and explicit env
   ```

5. P3에서는 `TcpDeltaStand`를 먼저 사용한다. `TcpDeltaLocal`은 test 후 별도 flag로 연다.

### Acceptance criteria

- fake SpaceMouse Cartesian input으로 deterministic TCP delta command가 나온다.
- deadman false이면 command가 나가지 않는다.
- simulation mode에서만 command가 나간다.
- real mode에서는 command가 block된다.

---

## P3-F. TCP Pose simulator acceptance runbook 추가

### 담당 모듈

- docs/scripts

### 수정/추가 파일

- `docs/runbooks/tcp_pose_simulator_acceptance.md` 추가
- `scripts/tcp_pose_simulator_acceptance.sh` 추가 권장
- `rb_simulator/artifacts/` gitignored

### 작업 내용

Acceptance runbook은 다음 단계를 포함한다.

1. per-arm simulator stack start

   ```bash
   make sim-up
   ```

2. state stream에서 FK TCP pose 확인

   ```text
   left.tcp_stand != null
   right.tcp_stand != null
   has_valid_tcp_pose == true
   ```

3. ArmMotion 전송

4. 작은 `TcpDeltaStand` 전송

   ```text
   left +0.005m x
   right no motion
   ```

5. state/log 확인

   - command verdict Ok
   - no fault latch
   - q target finite
   - q remains within joint limit
   - final FK TCP x moves expected direction within tolerance

6. IK failure test

   - unreachable target send
   - verdict IkFailed
   - previous safe target 유지
   - no crash

7. EmergencyStop/ResetFault flow 확인

8. artifacts 저장

   ```text
   tcp_pose_acceptance_summary.json
   state_stream.jsonl
   servo_log.csv
   rb_servo_server.log
   left_simulator.log
   right_simulator.log
   ```

### Acceptance criteria

- runbook을 따라 simulator에서 TCP Pose/Delta가 검증된다.
- IK failure가 safety failure로 안전하게 처리된다.
- real robot 실행 단계는 runbook에 포함하지 않는다.

---

# Cross-module cleanup items from initial review

아래 항목들은 P0~P3 중 해당 work package에 포함되어야 한다.

## C1. `state_pub_bind` naming cleanup

- Current: `network.state_pub_bind`
- New: `network.state_pub_endpoint`
- Add: `network.state_pub_rate_hz`
- Keep deprecated alias temporarily with warning.

## C2. rb_servo_server CMake에서 rb_gui test 등록 제거

현재 `rb_servo_server/CMakeLists.txt`가 `../rb_gui/tests`를 직접 등록한다. Component boundary가 흐려진다.

작업:

- `rb_servo_server` CTest에서는 servo server tests만 등록한다.
- root `scripts/hardware_free_validation.sh`가 `rb_gui` tests를 별도로 실행한다.

Acceptance:

- `ctest` in rb_servo_server does not depend on rb_gui folder.
- full hardware-free gate still runs rb_gui tests.

## C3. GUI command destination fix

- GUI command는 `rb_servo_server`로 보내야 한다.
- `RB_GUI_COMMAND_HOST=rb_simulator`는 잘못된 의미다.

New compose:

```yaml
RB_GUI_COMMAND_HOST: "rb_servo_server"
RB_GUI_COMMAND_PORT: "50010"
```

## C4. camera_server real config placeholder validation

- `REPLACE_HEAD_SERIAL` etc. must fail.
- mock `MOCK_*` remains allowed only when `simulate_cameras=true`.

## C5. sync policy validation

Allowed initial combinations:

```text
software + nearest_timestamp
hardware + frame_number
```

Everything else fails.

## C6. reconnect not implemented

- `reconnect.enabled=true` fails until implementation lands.
- Do not leave it as silent warning.

## C7. docs/scripts archive

- Agent-generated planning docs should not be source-of-truth.
- Move to `docs/archive/agent_plans/` or mark historical.

## C8. force control disabled

- `force_control.provider: null`
- `force_control.enable: false`
- force command rejected when provider null.

## C9. TCP command disabled until P3

- GUI: disabled P0~P2
- policy_runner: no Cartesian action P1~P2
- servo_server: P0~P2 returns `CartesianUnavailable`
- P3: simulation only

## C10. Real mode safety gates

Minimum gates:

```text
RB_ALLOW_REAL_ROBOT=1      # allow connect/read-only real robot process
RB_ALLOW_REAL_MOTION=1     # allow servo_j send in real mode
RB_ALLOW_REAL_CARTESIAN=1  # allow Cartesian-derived servo target in real mode, future only
```

P0~P3 recommended policy:

```text
real read-only: allowed with RB_ALLOW_REAL_ROBOT=1
real joint motion: P1 possible only after explicit acceptance
real Cartesian motion: not enabled by default even in P3
```

---

# Suggested parallel Codex agent assignment

## Batch 1: P0 foundation

### Agent 1: P0-B simulator topology

Scope:

- `rb_simulator/**`
- per-arm config/tests

Do not touch:

- `rb_servo_server` except docs references
- compose except if absolutely necessary

### Agent 2: P0-D config parser

Scope:

- `rb_servo_server/src/config/config.cpp`
- `rb_servo_server/include/rb_servo/config/config.hpp`
- `rb_servo_server/CMakeLists.txt`
- config loader tests

Do not touch:

- simulator topology
- policy_runner

### Agent 3: P0-E/P0-F compose + validation

Scope:

- root `docker-compose.yml`
- `scripts/hardware_free_validation.sh`
- `scripts/check_deps.sh`
- Makefile

Depends on:

- Agent 1 config names may change
- Agent 2 config schema may change

### Agent 4: P0-H GUI safety

Scope:

- `rb_gui/**`
- GUI tests

Do not touch:

- servo_server internals except protocol assumptions

### Agent 5: P0-G camera deps/config

Scope:

- `camera_server/src/config/config.cpp`
- camera configs/docs
- dependency preflight integration

### Agent 6: P0-A docs/source-of-truth

Scope:

- root README/docs/archive

Depends on:

- Use final canonical names from Agents 1~5.

## Batch 2: P1 joint-only integration

### Agent 7: P1-A read-only servo mode

Scope:

- `rb_servo_server/src/control/dual_arm_servo_loop.cpp`
- servo config/state publisher/logger

### Agent 8: P1-B rbpodo backend read-only

Scope:

- `rb_servo_server/src/robot/rbpodo_backend.cpp`
- rbpodo CMake/config docs

Requires:

- rbpodo headers/library available in dev environment

### Agent 9: P1-C per-arm integration smoke

Scope:

- smoke scripts/tests

Depends on:

- P0-B simulator topology
- P0-C/P0-D config schema

### Agent 10: P1-D/P1-E policy_runner + SpaceMouse joint teleop

Scope:

- new `policy_runner/**`

Do not touch:

- rb_gui teleop

## Batch 3: P2 FK

### Agent 11: P2-A Pinocchio FK module

Scope:

- `rb_servo_server/include/rb_servo/kinematics/**`
- `rb_servo_server/src/kinematics/**`
- CMake

### Agent 12: P2-C state publisher FK fields

Scope:

- servo loop state enrichment
- state publisher

Depends on:

- Agent 11

### Agent 13: P2-D GUI FK visualization

Scope:

- `rb_gui/**`

Depends on:

- state schema from Agent 12

### Agent 14: P2-B/P2-E geometry registry + policy awareness

Scope:

- `calibration/**` or `geometry/**`
- `policy_runner` geometry safety
- docs/frame contract

## Batch 4: P3 IK/TCP command

### Agent 15: P3-A IK solver

Scope:

- kinematics IK code/tests

Depends on:

- P2 FK module

### Agent 16: P3-B CartesianController integration

Scope:

- `cartesian_controller`
- servo loop Cartesian path

Depends on:

- Agent 15

### Agent 17: P3-C TCP command docs/tools/tests

Scope:

- protocol docs
- send_tcp tools
- command parser tests

### Agent 18: P3-D/P3-E GUI/policy TCP simulation-only control

Scope:

- GUI TCP feature flag
- policy_runner Cartesian action

Depends on:

- Agent 16 state/verdict behavior

### Agent 19: P3-F acceptance runbook

Scope:

- docs/runbooks
- acceptance script

Depends on:

- Batch 4 outputs

---

# Milestone exit criteria

## P0 exit criteria

```text
[ ] root README exists and is source-of-truth
[ ] public terms use backend_type=simulator, run_mode=simulation
[ ] per-arm simulator configs exist
[ ] two simulator processes can run concurrently
[ ] rb_servo_server config parser uses yaml-cpp
[ ] hardware-free validation preflight exists
[ ] compose default does not start real camera privileged container
[ ] GUI mode/backend mismatch fixed
[ ] GUI TCP command disabled
[ ] camera placeholder serial validation fixed
[ ] force control null/disabled
```

## P1 exit criteria

```text
[ ] rb_servo_server read-only mode exists
[ ] real read-only requires RB_ALLOW_REAL_ROBOT=1
[ ] real motion requires RB_ALLOW_REAL_MOTION=1
[ ] RbpodoBackend connect/readState implemented or blocked only by missing rbpodo dependency
[ ] real config template includes 172.28.60.200 and 172.28.60.201
[ ] per-arm simulator smoke passes
[ ] policy_runner exists
[ ] policy_runner Hold and JointVelocity action work in simulation
[ ] SpaceMouse joint velocity path exists with fake-reader tests
```

## P2 exit criteria

```text
[ ] Pinocchio FK module builds when enabled
[ ] FK test passes
[ ] state stream publishes tcp_base/tcp_stand when FK enabled
[ ] GUI parses and visualizes TCP markers
[ ] geometry/calibration registry exists
[ ] geometry-dependent policy is blocked in real when geometry_valid_for_real_policy=false
[ ] TCP command remains disabled
```

## P3 exit criteria

```text
[ ] IK solver test passes
[ ] CartesianController supports TcpPoseTarget in simulation
[ ] CartesianController supports small TcpDeltaStand in simulation
[ ] CartesianController supports small TcpDeltaLocal in simulation or has explicit deferred flag
[ ] IK failure returns safe verdict and holds previous target
[ ] TCP command protocol docs/tools exist
[ ] GUI TCP command is simulation-only behind feature flag
[ ] policy_runner Cartesian SpaceMouse is simulation-only behind safety gate
[ ] TCP Pose simulator acceptance runbook passes
[ ] real Cartesian motion remains disabled unless future explicit acceptance gates are added
```

---

# After P3: next features to attach

P3 이후 TCP Pose 기반 테스트/검증이 안정화되면 다음 순서로 기능을 붙인다.

```text
P4 camera real acceptance
  - RealSense serial discovery tool
  - 3-camera 30fps acceptance
  - bundle skew/drop/jitter metrics

P5 measured calibration
  - head camera extrinsic
  - wrist hand-eye
  - calibration acceptance artifact

P6 real joint motion acceptance
  - one-arm tiny joint motion
  - dual-arm hold
  - dual-arm tiny joint motion

P7 real Cartesian acceptance
  - FK/IK sanity with measured geometry
  - tiny TCP delta on one arm
  - dual-arm TCP constraints

P8 gripper integration

P9 mo_forcecontroller integration
  - NullForceControlProvider -> MoForceControlProvider adapter
  - force frame/sign convention tests
  - real safety envelope
```
