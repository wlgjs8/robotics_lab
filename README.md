# robotics_lab

한국어 기본 README입니다. 영어 원문은 [README.en.md](README.en.md)에 보존되어 있습니다.

> **참고:** ACKON500 circle-tracking benchmark와 그 시점의 root `GOAL.md`
> 스냅샷 프롬프트는 2026-06-20에 제거됐습니다. 현재 방향은 이 README / `AGENTS.md` /
> `docs/architecture.md`를 따르세요. 자세한 드리프트 목록은
> `docs/code_architecture_map.md`에 있습니다. `policy_runner/GOAL.md`는 별도
> policy-training 메모입니다.

`robotics_lab`는 dual-arm RB3-730 시스템을 통합하기 위한 작업 공간입니다. 서보 제어, rbpodo 백엔드(실로봇 + 컨트롤러 `pgmode` 시뮬레이션), 카메라 캡처, `policy_runner`, 운영자 GUI를 함께 다룹니다.

> **통합 제어기 스택 진행 중 (2026-08-16~)**: 회사 제어기
> `submodules/controller-manager`(읽기 전용 서브모듈, 펌웨어 26071103에서
> servo_j LPF-off + 큐 위상 조절)를 `cm_bridge/`가 프로세스 경계로 연결하는
> 작업이 진행 중입니다. 전환기 동안 `rb_servo_server`는 기존대로 동작하며,
> 설계와 단계 계획은 `cm_bridge/docs/design.md`를 보십시오.
> `submodules/mo_forcecontroller`/`mo_grippers`는 2026-08-16 제거됐습니다.

## 현재 단계

현재 프로젝트 단계는 **rbpodo pgmode-real 물리 로봇 브링업**입니다.
시뮬레이터 우선 Cartesian acceptance hardening은 대부분 마무리됐고, 이제
실제 RB3-730E 하드웨어에서 검증을 진행합니다.

mock / rbpodo 컨트롤러 시뮬레이션(pgmode) 측에서 반복 검증되어 안정화된 항목:

- 구조화된 backend result 및 fault telemetry
- `JointTarget`
- `TcpPoseTarget`
- `TcpLinearMove`
- GUI 운영자 제어
- `policy_runner` SpaceMouse 경로
- command-source lease/arbitration
- 카메라 readiness contract

실제 물리 로봇에서 추가로 검증된 항목은 아래 "현재 성숙도"를 참고합니다. 실제
모션 권한은 site-local config와 서버 안전 계층이 결정하며, 운영자 감독과
E-stop은 물리 운용 절차입니다. simulator acceptance 통과가 곧 하드웨어 구동
허가는 아닙니다.

## 현재 성숙도

mock / 컨트롤러 시뮬레이션에서 지원되는 항목:

- mock dual-arm servo control
- direct 및 worker I/O mode (mock/하드웨어프리)
- quaternion 필드를 포함한 FK/TCP state publication
- TCP PTP and Linear commands
- mock camera server
- mock/simulation용 GUI viewer/operator console
- `policy_runner` joint 및 Cartesian action source
- mandatory Eigen3/Pinocchio C++ Cartesian math path for `rb_servo_server`

pgmode-real(실제 RB3-730E 하드웨어)에서 구동/검증된 항목:

- read-only 물리 diagnostics parity (컨트롤러 `.200`/`.201`, `tcp_actual_stand` 기준)
- 양팔 실제 Cartesian circle 추종 — 저속, TUNED-1 프로파일, tracking 중앙값 ~1.42°
  (`docs/runbooks/rbpodo_real_physical_circle.md`)
- UMI 양팔 Cartesian 텔레옵(relative-init) `TcpPoseTarget` 실로봇 구동, UMI `data_tcp`
  리플레이 실차 검증(ee_local + r_align)
- **pi0.5(openpi) `flow-infer` `real_policy` 풀 클로즈드루프 rollout 실로봇 구동** —
  `TcpPoseTarget` + 그리퍼 명령 전송. 런타임/엔지니어링 검증 완료: 모션
  부드러움 + in-distribution(async chunking으로 500 Hz 루프 진동 제거,
  absolute-proprio 프레임갭은 reset-relative 재학습으로 해소). **task 성공률은 아직
  모델 한계**(아래 참고)
- 실제 그리퍼 구동 — Pika Gripper Backend, `RB_ALLOW_REAL_GRIPPER` +
  `measured_gripper_available` 게이트
- 서버측 자가충돌 가드 — async URDF-mesh `CollisionMonitor`(33 geom / 337 pair),
  real에서 enforce(velocity barrier), stale/hard-breach는 fail-closed
- policy측 real-Cartesian 안전 게이트 완화(PR #13) → `rb_servo_server`가 단일
  real-motion 안전 계층
- 컨트롤러 `-2001`(suspect diagnostics) 실모드 수용(PR #12); EMS/SOS/soft-estop/
  `collision_occur`/unknown-mode/init-error는 계속 latch

아직 미완(production-ready 아님) 항목:

- **정책 task 성공률** — rollout 모션은 부드럽지만 부정확(예: 좌완이 grasp 대신
  충돌). 런타임이 아니라 **모델 품질 / 데이터 커버리지 / appearance-domain gap**
  문제이며 init-pose 분포 매칭이 진행 중(`umi_init_from_grasp.py`)
- default-off project-native F/T monitor/contact guard/normal admittance
- 고속 물리 circle 단계 (15 cm / 16 s 이상, transition ladder P7–P9)
- 실측 hand-eye / 카메라 calibration은 일반 geometry-의존 정책엔 여전히 미완이지만,
  **현재 배포된 pika Sense≡Gripper + ee_local + 이미지조건 정책에는 불필요**
  (reset-relative가 steamvr→stand R을 상쇄, tool offset은 알려진 상수) — 즉 현 정책의
  블로커가 아님

## Source Of Truth

먼저 아래 문서부터 확인합니다.

- `AGENTS.md`: Codex/Claude/기타 에이전트 작업 지침
- `REVIEW.md`: 현재 review baseline 및 open item
- `docs/current_review.md`: `REVIEW.md`로 가는 짧은 redirect입니다. 내용을 중복하지 않습니다.
- `docs/architecture.md`: 시스템 토폴로지, 용어, motion primitive contract, safety boundary
- `docs/code_architecture_map.md`: 코드에서 검증한 컴포넌트 맵, 포트/와이어 포맷, 문서-코드 드리프트 목록
- `docs/servo_backend_contract.md`: backend result, fault, worker I/O, state telemetry contract
- `docs/frame_contract.md`: 공통 frame 및 calibration 상태
- `docs/joint_range_policy.md`: rbpodo raw joint angle/range policy
- `docs/hardware_free_validation.md`: hardware-free validation boundary
- `docs/runbooks/camera_acceptance.md`: 실제 3-camera acceptance
- `calibration/active_calibration.yaml`: configured-estimate robot/camera/stand setup registry

과거 prompt/planning 파일은 감사용 맥락입니다. 위 문서들과 충돌하면 위 문서들이 우선입니다.

## 표준 용어

```yaml
run_mode: mock | simulation | real
backend_type: mock | rbpodo
```

지원되는 real-controller backend는 `rbpodo` 하나뿐입니다. `mock`은
hardware-free 검증용으로 유지하며, `run_mode: simulation`은 이제 rbpodo
컨트롤러 `pgmode` 시뮬레이션 flavor만 지칭합니다. 제거된 소프트웨어
시뮬레이터 backend와 unsupported raw script TCP 비교 경로는 active
code/config/gate/runbook surface에 없습니다.

## 실제 및 컨트롤러 시뮬레이션 토폴로지

실제 시스템:

```text
rb_servo_server
  left_robot  backend_type=rbpodo -> 172.28.60.200
  right_robot backend_type=rbpodo -> 172.28.60.201
```

rbpodo 컨트롤러 `pgmode` 시뮬레이션은 위와 같은 팔별 rbpodo endpoint 구조를
그대로 쓰되, 대상이 Virtual ControlBox VM 또는 `pgmode`로 둔 실제 box입니다.
실행 설정은 추적되는 `stack_real.yaml`과 `stack_sim.yaml`에서만 관리합니다.

## Safety

실제 로봇 연결과 모션은 **더 이상 env 게이트로 제어하지 않습니다.** 과거의
`RB_ALLOW_REAL_ROBOT` / `RB_ALLOW_REAL_MOTION` / `RB_ALLOW_REAL_CARTESIAN` /
`RB_ALLOW_RBPODO_ACK_DISABLED_MOTION` /
`RB_ALLOW_RBPODO_SUSPECT_DIAGNOSTICS_REAL_MOTION` /
`RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION` /
`RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN` /
`RB_RBPODO_PGMODE_SIMULATION_CONFIRMED` 게이트는 서버 런타임에서 모두
제거됐습니다. `run_mode`/`operation_mode`는 이제 telemetry 라벨이며 실행 허용
여부를 결정하지 않습니다.

실제 모션의 실행 권한은 **추적되는 stack config + `rb_servo_server`의 mode-독립
안전 계층**이 결정합니다.

- safety filter (joint clamp, stand-frame floor plane)
- tracking-error latch
- URDF-캡슐 async self-collision 가드 (`CollisionMonitor`)
- command-source lease / arbitration
- client deadman
- 운영자 감독 + 하드웨어 E-stop은 물리 운용 절차로 유지

실제 동작은 `rb_servo_server/config/stack_real.yaml`이 명시적으로 허용해야
합니다. policy측 `SafetyGate`의 real-Cartesian 차단은 PR #13으로 은퇴했고,
stale state/fault/camera/kinematics 같은 client-side readiness만 남습니다. 실제
Cartesian 모션의 최종 허용/거부는 `rb_servo_server`가 맡습니다.

컨트롤러 `-2001`(suspect diagnostics, `op_stat_self_collision`/`robot_time`
필드 디코딩 garbage)은 per-arm config
`allow_real_motion_with_suspect_diagnostics: true`로만 수용합니다(env 불필요).
EMS/SOS/soft-estop/`collision_occur`/unknown-mode/init-error는 이 옵션과
무관하게 계속 latch합니다.

Rainbow controller box를 `pgmode` simulation으로 둔 `rbpodo`
controller-simulation 경로는 실제 controller IP에 접속하므로 config는
`run_mode: real`, `backend_type: rbpodo`를 쓰되 robot
`operation_mode: simulation`이어야 하고 physical robot은 움직이지
않습니다(`physical_motion_expected=false`). 이 carve-out은
`cartesian_control.allow_in_controller_simulation: true`와
`servo.allow_controller_simulation_motion: true` config로 열립니다(env 불필요).
Tracking은 controller reference인 `tcp_ref_stand`를 사용하며, tracked sim
profile은 physical control-box topology이므로 encoder motion을 fault-latch합니다.

`make run`(real, `operation_mode: real`)과 `make run MODE=sim`(pgmode-sim,
`operation_mode: simulation`)이 사용하는 두 stack config가 단일 진실원천이며,
실제/시뮬레이션 구분을 config만으로 결정합니다.

`rbpodo` real config의 Rainbow Servo J parameter는 새 이름만 사용합니다.

- `servo_t1_sec` -> `move_servo_j` `t1`
- `servo_t2_sec` -> `move_servo_j` `t2`
- `servo_gain` -> `gain`
- `servo_alpha` -> `alpha`

새 config에서 `servo_acc`나 `servo_lookahead_sec`를 쓰지 마세요. 기존 alias는
deprecated입니다. 지원되는 robot-control profile은 500 Hz이며
`servo_t1_sec: 0.002`가 command period와 맞아야 합니다. 자세한 절차는
`docs/runbooks/rbpodo_servo_acceptance.md`와
`docs/runbooks/real_robot_readonly.md`를 봅니다.

지원되는 500 Hz Servo J profile은 `t1=0.002`, `t2=0.021`, `gain=1.0`을
고정합니다. 컨트롤러가 `gain`/`alpha`를 내부에서 `0.1` 스케일하므로
script값 `alpha=10.0`은 실효 `1.0`, 즉 컨트롤러 내부 LPF off입니다. 하지만
physical real robot에서는 LPF-off profile이 jerk/jitter를 키우므로 tracked
real stack은 script-level `servo_alpha: 1.0`(실효 약 `0.1`)을 사용해
컨트롤러 LPF를 남겨 둡니다. Controller-simulation diagnostic transparency가
필요할 때만 `servo_alpha: 10.0` profile을 씁니다. 반응성·부드러움·정확성의
1차 튜닝 표면은 여전히 서버 측 제어루프(`TcpPoseTarget` →
`cartesian_control.pose_track_smd`)입니다. 자세한 내용은
`docs/servo_backend_contract.md`의 "Servo J Streaming Profiles"를 봅니다.

`rbpodo` joint state와 command는 raw controller degree 값을 보존합니다.
tracked real rbpodo template의 supported safety range는 명시적 per-joint
`q_min_deg: [-360, -360, -160, -360, -360, -360]` /
`q_max_deg: [360, 360, 160, 360, 360, 360]`입니다. `[-180, 180]`
정규화는 control/safety/tracking/log source-of-truth에 쓰지 않습니다.
자세한 내용은 `docs/joint_range_policy.md`를 봅니다.

Project-native F/T monitor, contact guard, 법선방향 unilateral admittance,
6D Cartesian compliance가 servo motion path에 통합되어 있습니다.
`stack_real.yaml`의 force path는 현재 양팔 supervised Gate 3D 시험 프로파일인
6축 Cartesian compliance입니다. `surface_source: none`과
`compliance_frame: tcp_origin`을 사용하므로, URDF와 일치하는 +90도 FT
sensor 축에서 순응 translation을 계산하고 TCP 끝점을 correction 원점으로
사용합니다. GUI의 runtime FT control 기즈모가 시험 축의 기준이며 일반 TCP
pose 기즈모는 X/Y force-axis 판정에 사용하지 않습니다.
세 회전축 방향/복귀가 확인되어 Roll/Pitch/Yaw가 동일한 측정-noise 기반
고감도 mass/damping/stiffness/deadband를 사용합니다. 명시적인 blockwise
release recenter를 사용해 translation/rotation 내부의 일부 축만 먼저
spring 복귀하지 않게 하고, 블록 전체 release 뒤에는 공통 feasible jerk scale로
복귀 방향을 유지합니다. 어느 축이 hard motion-envelope recovery를 필요로 하면
그 구간에는 축별 recovery jerk를 우선하고, 모든 축이 soft envelope로 돌아온 뒤
공통 scale을 재개합니다. payload/COM 검증 전까지는 동일 시작 자세의 작은 각도
Hold 시험으로 제한됩니다. 명시적인
운영자 결정으로 stand/user geometric
floor도 모두 꺼져 있어 TCP/gripper-tip floor velocity damper와 hard plane
backstop이 없습니다. ROI, self-collision, tracking, lease/deadman, E-stop은
유지되지만 wrist F/T는 upstream link 접촉을 모두 감지할 수 없습니다.
상세 계약은 `rb_servo_server/docs/force_control.md`, 실센서 특성화와
승격 evidence는 `docs/runbooks/ft_force_control_acceptance.md`에 기록합니다.

```yaml
force_control:
  provider: project_native
  enable: true
  operating_mode: cartesian_admittance
  allow_in_real: true
  supervised_experimental_real: true
```

## Motion Primitive 요약

현재 public motion primitive는 **3가지**입니다. 모두
`run_mode` 무관하게 동작 가능하며, 실제 동작 여부는 config가 결정합니다.

1. `JointTarget` — 절대 joint PTP. 목표 관절각으로 이동.
2. `TcpPoseTarget` — Cartesian final-pose PTP (MoveJ-like). Cartesian path는
   보장하지 않습니다. UMI 등 streaming teleop은 서버측 SMD pose tracking으로 추종.
3. `TcpLinearMove` — MoveL-like Cartesian 선형 경로 primitive
   (`constant`/`slerp` orientation mode).

`safety.floor_constraint`가 켜지면 위 모든 primitive는 최종 safety gate에서 floor
plane에 대해 FK 체크됩니다(Cartesian 경로는 평면을 따라 slide, joint-space는 hold).
로컬 stack config의 PIKA 그리퍼 `tcp_offset_points`는 TCP 원점과 양쪽 팁
`gripper_tip_a/b`를 함께 검사합니다. 현재 팁 오프셋은 줄자 실측 tip-to-tip
`118 mm` 기준으로 TCP x축 `±0.059 m`입니다.

참고:

- `SetSafetyFloorZ`는 motion primitive가 아니라 floor plane 높이를 config 범위
  내에서 조정하는 leaseless non-motion 명령입니다.

## 자주 쓰는 명령

Python checks:

```bash
python3 -m unittest discover rb_gui/tests
python3 -m unittest discover policy_runner/tests
```

Hardware-free C++ checks:

```bash
cmake -S rb_servo_server -B rb_servo_server/build
cmake --build rb_servo_server/build -j
ctest --test-dir rb_servo_server/build --output-on-failure
```

`rb_servo_server` C++ builds require Eigen3 and Pinocchio. Cartesian FK/IK,
orientation interpolation, frame conversion, and SE(3) delta math delegate to
Eigen/Pinocchio. Missing Pinocchio is a missing C++ dependency, not a fallback
runtime mode.

Cartesian math rebaseline is part of the Pinocchio-backed C++ test suite:

```bash
ctest --test-dir rb_servo_server/build --output-on-failure
```

Cartesian behavior is covered by the Pinocchio-backed C++ tests above, then
exercised on the active stack: mock smoke when a local mock config is available,
rbpodo controller `pgmode` simulation / VM, and real hardware only through the
separate supervised runbooks. The old software-simulator-oriented Cartesian
acceptance runner was removed with the retired simulator-first Cartesian
acceptance lane.

Supported scope hygiene is documented in `AGENTS.md`,
`docs/architecture.md`, and `docs/servo_backend_contract.md`; keep removed
backends and legacy real-motion env gates out of active code/config/docs.

This keeps the active surface rbpodo-only for real controllers and 500 Hz for
robot command/control defaults. It does not approve real motion.

rbpodo Servo J supervised acceptance:

```bash
python3 scripts/rbpodo_servo_acceptance.py --help
```

Start with read-only. Use the tracked `rb_servo_server/config/stack_real.yaml`
directly and change one reviewed acceptance-stage setting at a time. Do not
create parallel local real profiles.

통합 운영자 stack 시작(native — Docker 아님). `make run`이
`rb_servo_server` + viser GUI + `policy_runner`(SpaceMouse + UMI teleop)를 한
번에 띄웁니다:

```bash
make run            # pgmode real (+ gripper_server)
make run MODE=sim   # pgmode controller-simulation
```

소스를 고친 뒤에는 먼저 `make build`으로 stack을 빌드/설치합니다(rbpodo
backend 포함). `make vm-up`으로 관리하는 Rainbow Virtual ControlBox에서는
simulated `q_actual`이 움직이므로, 현재 physical-box용 `stack_sim.yaml`의
physical-motion fault latch와 양립하지 않습니다. VM 재승격에는 endpoint topology를
명시하는 별도 contract와 review가 필요합니다; local launch YAML로 우회하지 않습니다.

`make build`와 `make run`은 `STACK_PYTHON`이 지정되면 그 interpreter를 함께
사용하고, 아니면 repo의 `.venv/bin/python`을 우선 사용한 뒤 system `python3`로
fallback합니다. 이 선택은 viser GUI, scope dashboard, gripper server,
`policy_runner`에 동일하게 적용됩니다.

`make build`는 증분 빌드이며, layout-sensitive `config.hpp`가 바뀌면 서버
object 전체만 자동으로 다시 컴파일합니다. CMake cache/toolchain/build-tree를
완전히 초기화해야 할 때만 `make rebuild`를 사용합니다.

SpaceMouse / UMI teleop는 `make run`이 띄우는 `policy_runner`에서 상시 동시
운용됩니다(별도 teleop 모드 불필요). policy 학습은 GPU 서버에서 native
`python3 -m policy_runner flow-train`으로 수행하며, inference는
`python3 -m policy_runner flow-infer`(아래 `--rollout-mode` 참고)로 실행합니다.
실제 OpenPI `real_policy` rollout은 `make run`을 그대로 띄운 상태에서 별도
터미널의 얇은 wrapper로 시작합니다. `ACTION_SOURCE=none`은 필요하지 않습니다:

```bash
make run

# another terminal
OPENPI_REMOTE_SKIP_WARMUP=1 RB_ALLOW_REAL_GRIPPER=1 \
  make flow-infer-real
```

기본 wrapper는
`policy_runner/config/flow_real_realsense.yaml`의 외부 flow-infer state 포트
`50378`을 사용합니다(`make run` teleop_mux는 `50376`). `make run`의 joint
scope dashboard는 별도 fanout 포트 `50356`을 기본으로 수신하므로, dashboard가 켜진
상태에서도 기존 flow-infer 명령이 `50378`에 bind할 수 있습니다.
checkpoint/config/python은 각각 `FLOW_INFER_CHECKPOINT`, `FLOW_INFER_CONFIG`,
`FLOW_INFER_PYTHON`으로 바꿀 수 있고, `OPENPI_REMOTE_SKIP_WARMUP`,
`RB_ALLOW_REAL_GRIPPER`, `DISPLAY` 등 호출 환경은 그대로 상속됩니다.

실제 카메라 없이 controller pgmode simulation에서 같은 OpenPI 경로를
검증할 때도 서버는 추적되는 `stack_sim.yaml`만 사용하고, 녹화 HDF5의 손목
RGB-D는 물리 camera_server와
겹치지 않는 `:5700`으로 재생합니다:

```bash
ACTION_SOURCE=none SCOPE_DASHBOARD=0 GRIPPER_SERVER=0 make run MODE=sim

# another terminal; physical camera_server :5600 is untouched
python3 scripts/offline_camera_replay.py --episode /path/to/episode_000.hdf5

# first prove pgmode simulation telemetry before sending a policy command
python3 scripts/controller_sim_state_monitor.py --port 50356 --gate 20

OPENPI_REMOTE_SKIP_WARMUP=1 make flow-infer-sim-offline \
  FLOW_INFER_ARGS='--proprio-mode velocity --depth-z-near-mm 50 --depth-z-far-mm 700 --depth-units-m 1e-4'
```

`stack_sim.yaml` registers the same `spacemouse_precise`, `umi_large_smooth`,
and `flow_infer_smooth` command profiles as `stack_real.yaml`. In controller
simulation only, the closed-loop tracking pose is `tcp_ref_stand`/`q_target`;
an unavailable reference fails closed. `stack_sim.yaml` keeps
`cartesian_control.allow_in_real: false`, and the offline flow config keeps both
real arm and real gripper authority disabled. The sim-only Make target defaults
to the pgmode-validated 6-step execution window; the physical-real launcher
retains its existing 12-step default. Because the tracked sim topology targets
physical control boxes held in pgmode, any encoder motion indication is a
server-owned `fault_latch`; a Virtual ControlBox needs a separately reviewed,
explicit topology contract before using this profile.

The controller-box pgmode parameter comparison, storage provenance, long-run
evidence, and interpretation boundary are recorded in
[`docs/reports/flow_infer_pgmode_sim_param_search.md`](docs/reports/flow_infer_pgmode_sim_param_search.md).

DeltaTwistFollower + velocity-proprio 안정성 확인을 위한 첫 실행 baseline은
아래처럼 둡니다. RTC/ensemble은 기본 안정성 확인 뒤 별도로 opt-in합니다.

```bash
FLOW_INFER_STITCH=boundary \
FLOW_INFER_ACTION_HORIZON=24 \
FLOW_INFER_CHUNK_EXECUTE_STEPS=12 \
FLOW_INFER_CHUNK_OVERLAY_RUNWAY_STEPS=4 \
FLOW_INFER_VELPROPRIO_SOURCE=measured \
FLOW_INFER_VELPROPRIO_SAMPLE=camera_frame \
FLOW_INFER_PRINT_CHUNK=0 \
FLOW_INFER_PRINT_TRACKING=0 \
RB_ALLOW_REAL_GRIPPER=1 \
./tools/flow_infer_real_policy.sh \
  --proprio-mode velocity \
  --depth-z-near-mm 50 \
  --depth-z-far-mm 700 \
  --depth-units-m 1e-4
```

HDF5 policy episodes should be audited before `flow-train`:

```bash
python3 -m policy_runner hdf5-audit \
  --episodes-dir data/umi_episodes \
  --output-json data/umi_episodes/audit.json \
  --output-md data/umi_episodes/audit.md
```

The audit schema is `robotics_lab.policy_runner.hdf5_audit.v1`. Supported HDF5
layouts are `pika_umi_single_arm`, `pika_umi_bimanual`, and
`robotics_lab_dual_arm`. Use a `DatasetManifest` with schema
`robotics_lab.policy_runner.dataset_manifest.v1` to pin allowed formats,
camera filters, `single_arm_side`, required root attrs such as `pose_format`,
and retarget metadata. `pose_frame=steamvr_world` is not treated as `stand`;
real policy rollout remains blocked unless the manifest retarget transform to
`stand` is measured or accepted.

`flow-infer` requires an explicit `--rollout-mode` so inferred actions are not
implicitly routed by the old `mode: real` flag. Supported values are
`offline_eval`, `sim_dryrun`, `controller_sim`, `real_readonly`, and
`real_policy`; each run writes a machine-readable `rollout_summary` to
`outputs/rollout_summary.json` unless `--rollout-summary` is supplied.
`controller_sim` is the rbpodo `controller_simulation` carve-out only:
`run_mode=real`, `operation_mode=simulation`, controller-simulation Cartesian
gate evidence, and `physical_motion_expected=false`. `real_readonly` is the
current `real_supervised` observation lane and never sends motion commands.
`real_policy` requires real motion, measured or accepted retarget, collision,
gripper, and geometry gates to all be present — a hard gate, but a satisfiable one:
those gates were met via accepted/validated config and a full `real_policy` rollout
has run on the physical robot.

For the GENE-UMI policy-transition lane, keep `hdf5-audit`,
`flow-infer`, `real_supervised`/`real_readonly`, and pgmode transition outputs
in an Artifact manifest / `artifact_manifest`. Generate it with
`scripts/collect_gene_umi_artifact_manifest.py`; the manifest is documentation
and evidence inventory only, not physical real robot approval.

접속:

```text
http://127.0.0.1:8080
```

## 표준 Config

Tracked stack configs:

- `rb_servo_server/config/stack_sim.yaml` — rbpodo controller-simulation (`make run MODE=sim`)
- `rb_servo_server/config/stack_real.yaml` — physical real stack (`make run`, operator-supervised)

추가 real/local launch config는 만들지 않습니다. 실제 단계별 변경은
`stack_real.yaml` 한 파일에서 review와 CSV evidence를 남기며 진행합니다.
