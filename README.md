# robotics_lab

한국어 기본 README입니다. 영어 원문은 [README.en.md](README.en.md)에 보존되어 있습니다.

> **참고:** 루트의 `GOAL.md`는 프로젝트 목표 문서가 아니라 과거 단일 task 프롬프트
> (`ACKON500-GENE-GOAL-01`, rbpodo controller-sim 500 Hz circle-tracking 튜닝)의
> 시점 스냅샷입니다. 현재 방향은 이 README / `AGENTS.md` / `docs/architecture.md`를
> 따르세요. 자세한 드리프트 목록은 `docs/code_architecture_map.md`에 있습니다.

`robotics_lab`는 dual-arm RB3-730 시스템을 통합하기 위한 작업 공간입니다. 서보 제어, 실제 토폴로지와 같은 형태의 로컬 시뮬레이터, 카메라 캡처, `policy_runner`, 운영자 GUI를 함께 다룹니다.

## 현재 단계

현재 프로젝트 단계는 **rbpodo pgmode-real 물리 로봇 브링업**입니다.
시뮬레이터 우선 Cartesian acceptance hardening은 대부분 마무리됐고, 이제
실제 RB3-730E 하드웨어에서 검증을 진행합니다.

시뮬레이터 측에서 반복 검증되어 안정화된 항목:

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

실제 물리 로봇에서 추가로 검증된 항목은 아래 "현재 성숙도"를 참고합니다. 실제
모션은 여전히 운영자 감독 + E-stop 휴대 + 명시적 게이트가 필요한 fail-closed
동작이며, simulator acceptance 통과가 곧 하드웨어 구동 허가는 아닙니다.

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
- mandatory Eigen3/Pinocchio C++ Cartesian math path for `rb_servo_server`

pgmode-real(실제 RB3-730E 하드웨어)에서 구동/검증된 항목:

- read-only 물리 diagnostics parity (컨트롤러 `.200`/`.201`, `tcp_actual_stand` 기준)
- 양팔 실제 Cartesian circle 추종 — 저속, TUNED-1 프로파일, tracking 중앙값 ~1.42°
  (`docs/runbooks/rbpodo_real_physical_circle.md`)
- UMI 양팔 Cartesian 텔레옵(relative-init) `TcpPoseTarget` 실로봇 구동, UMI `data_tcp`
  리플레이 실차 검증(ee_local + r_align)
- **pi0.5(openpi) `flow-infer` `real_policy` 풀 클로즈드루프 rollout 실로봇 구동** —
  `TcpTwistLocal` 스트리밍 + 그리퍼 명령 전송. 런타임/엔지니어링 검증 완료: 모션
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
- force control (`provider: null`, `enable: false` 유지)
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
- `docs/runbooks/tcp_pose_simulator_acceptance.md`: Cartesian simulator acceptance
- `docs/runbooks/camera_acceptance.md`: 실제 3-camera acceptance
- `calibration/active_calibration.yaml`: configured-estimate robot/camera/stand setup registry

과거 prompt/planning 파일은 감사용 맥락입니다. 위 문서들과 충돌하면 위 문서들이 우선입니다.

## 표준 용어

```yaml
run_mode: mock | simulation | real
backend_type: mock | simulator | rbpodo
```

지원되는 real-controller backend는 `rbpodo` 하나뿐입니다. `mock`과
`simulator`는 hardware-free 검증용으로 유지하며, unsupported raw script TCP
비교 경로는 active code/config/gate/runbook surface에서 제거되었습니다.

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

## Safety

실제 로봇 연결과 모션은 **더 이상 env 게이트로 제어하지 않습니다.** 과거의
`RB_ALLOW_REAL_ROBOT` / `RB_ALLOW_REAL_MOTION` / `RB_ALLOW_REAL_CARTESIAN` /
`RB_ALLOW_RBPODO_ACK_DISABLED_MOTION` /
`RB_ALLOW_RBPODO_SUSPECT_DIAGNOSTICS_REAL_MOTION` /
`RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION` /
`RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN` /
`RB_RBPODO_PGMODE_SIMULATION_CONFIRMED` 게이트는 서버 런타임에서 모두
제거됐습니다(일부 인수-테스트 스크립트에 과거 이름이 남아 있을 수 있으나 서버
동작에는 영향이 없습니다). `run_mode`/`operation_mode`는 이제 telemetry
라벨이며 실행 허용 여부를 결정하지 않습니다.

실제 모션의 안전은 **site-local config + `rb_servo_server`의 mode-독립 안전
계층**이 단독으로 책임집니다.

- safety filter (joint clamp, stand-frame floor plane)
- tracking-error latch
- URDF-캡슐 async self-collision 가드 (`CollisionMonitor`)
- command-source lease / arbitration
- client deadman
- 그리고 운영자 감독 + 하드웨어 E-stop

실제 동작은 `rb_servo_server/config/local/`의 site config가 명시적으로 허용해야
하며, **config가 단일 결정자**입니다. (policy측 `SafetyGate`의 real-Cartesian
차단은 PR #13으로 완화되어, 실제 모션에서는 `rb_servo_server`가 단일 안전
계층입니다.)

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
Tracking은 보통 controller reference인 `tcp_ref_stand`를 사용합니다.

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

`rbpodo` joint state와 command는 raw controller degree 값을 보존합니다.
tracked real rbpodo template의 supported safety range는 명시적 per-joint
`q_min_deg: [-360, -360, -360, -360, -360, -360]` /
`q_max_deg: [360, 360, 360, 360, 360, 360]`입니다. `[-180, 180]`
정규화는 control/safety/tracking/log source-of-truth에 쓰지 않습니다.
자세한 내용은 `docs/joint_range_policy.md`를 봅니다.

Force control은 비활성 상태를 유지합니다.

```yaml
force_control:
  provider: null
  enable: false
```

## Motion Primitive 요약

현재 **9가지** motion primitive를 지원합니다(개발 완료/활성 기준). 모두
`run_mode` 무관하게 동작 가능하며, 실제 동작 여부는 config가 결정합니다.

1. `JointTarget` — 절대 joint PTP. 목표 관절각으로 이동.
2. `JointVelocity` — streaming joint velocity 명령.
3. `TcpPoseTarget` — Cartesian final-pose PTP (MoveJ-like). Cartesian path는
   보장하지 않습니다. UMI 등 streaming teleop은 서버측 SMD pose tracking으로 추종.
4. `TcpLinearMove` — MoveL-like Cartesian 선형 경로 primitive
   (`constant`/`slerp` orientation mode).
5. `TcpCircleMove` — 서버측 자율 원 추적 benchmark primitive.
   `cartesian_control.enable_benchmark_primitives: true`로 활성화되며 viser
   "Circle" 버튼으로 구동합니다.
6. `TcpTwistLocal` — local frame streaming Cartesian velocity (SpaceMouse teleop).
   deadman / lease / 서버측 velocity limit 필요.
7. `TcpTwistStand` — stand frame streaming Cartesian velocity.
8. `TcpDeltaLocal` — local frame low-level one-shot/debug jog primitive.
9. `TcpDeltaStand` — stand frame low-level one-shot/debug jog primitive.

`safety.floor_constraint`가 켜지면 위 모든 primitive는 최종 safety gate에서 floor
plane에 대해 FK 체크됩니다(Cartesian 경로는 평면을 따라 slide, joint-space는 hold).

참고:

- `TcpCircleTrack`은 미구현 비활성 스켈레톤입니다
  (`tcp_circle_track_not_implemented`). 위 9가지에 포함되지 않습니다.
- `SetSafetyFloorZ`는 motion primitive가 아니라 floor plane 높이를 config 범위
  내에서 조정하는 leaseless non-motion 명령입니다.

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

`rb_servo_server` C++ builds require Eigen3 and Pinocchio. Cartesian FK/IK,
orientation interpolation, frame conversion, and SE(3) delta math delegate to
Eigen/Pinocchio. Missing Pinocchio is a missing C++ dependency, not a fallback
runtime mode.

Cartesian math rebaseline:

```bash
./scripts/codex_gate.sh CART-MATH-03
```

Cartesian simulator acceptance:

```bash
CODEX_RUN_CARTESIAN_ACCEPTANCE=1 ./scripts/codex_gate.sh CART-HARDEN-05
```

Circle tracking benchmark:

```bash
./scripts/codex_gate.sh BENCH-CIRCLE-01
```

See `docs/runbooks/circle_tracking_benchmark.md` for simulator-only benchmark
profiles and artifact interpretation.

rbpodo controller-simulation circle templates:

```bash
tools/create_rbpodo_circle_local_configs.sh
tools/rbpodo_circle_tune.sh --matrix stage2_gain_split --arm left --help
less docs/runbooks/rbpodo_controller_sim_circle.md
```

Async ACK-supervised 500Hz runbook: `docs/runbooks/rbpodo_500hz_acceptance.md`.
The named ACKON500 best controller-simulation profile is
`tools/rbpodo_ackon500_gene_goal.sh --profile best`; it is controller-reference
lower-bound evidence only, not physical real tracking.

**ACKON500 PASS is controller-reference lower-bound evidence, not physical TCP tracking.**
ACKON500 reports must keep `physical_readiness.status=blocked` and
`physical_tracking_result.status=not_measured` until diagnostics parity, tiny
physical acceptance, and slow physical-circle acceptance are complete.

These configs target Rainbow controller boxes in `pgmode` simulation only; they
do not approve physical Cartesian motion.

Live `rb_gui` visualization for the rbpodo controller-simulation circle
benchmark uses server-side state fanout plus a separate benchmark overlay:

```text
state_pub_endpoints:
  50151 -> benchmark recorder
  50161 -> rb_gui
overlay:
  50261 -> rb_gui desired circle / live metrics
```

Runbook: `docs/runbooks/rbpodo_controller_sim_circle.md`.
Use `tcp_ref_stand` as the tracking source in pgmode simulation and keep
`physical_motion_expected=false`. The controller-simulation Cartesian carve-out
is config-driven (`cartesian_control.allow_in_controller_simulation: true`); no
env gate is required. `policy_runner` is separate from this live view; GUI and
benchmark state consumers do not route commands through it.

Supported scope hygiene:

```bash
CODEX_SKIP_MISSING_CPP_DEPS=1 ./scripts/codex_gate.sh 04_supported_scope_docs_ci_hygiene
```

This keeps the active surface rbpodo-only for real controllers and 500 Hz for
robot command/control defaults. It does not approve real motion.

rbpodo Servo J supervised acceptance:

```bash
python3 scripts/rbpodo_servo_acceptance.py --help
```

Start with read-only. Do not copy `dual_real.example.yaml` directly as a
ready-to-run real motion config; site-specific real configs live under
`rb_servo_server/config/local/` and are gitignored.

시뮬레이터 운영자 stack 시작:

```bash
make sim-local-up
```

`make sim-local-up` starts the same-PC GUI, servo server, per-arm simulators,
and a passive `policy_runner` recorder. `make sim-up` remains a compatibility
alias for this same-PC stack. The recorder writes
robot-state JSONL episodes to `policy_runner/episodes` and does not send motion
commands.

Split-PC simulator stack:

```bash
# On simulator PC 172.28.60.36:
make sim-backend-up

# On the control/GUI PC:
make sim-control-up
```

`sim-backend-up` publishes simulator TCP ports on all interfaces by default.
Use `SIM_BACKEND_BIND=172.28.60.36 make sim-backend-up` to bind only that NIC.

SpaceMouse teleop data collection and policy inference are explicit modes:

```bash
make sim-teleop-up
make policy-train
make sim-infer-up
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

For the GENE 26.5 / ACKON500 policy-transition lane, keep `hdf5-audit`,
`flow-infer`, `real_supervised`/`real_readonly`, and pgmode transition outputs
in an Artifact manifest / `artifact_manifest`. Generate it with
`scripts/collect_gene_umi_artifact_manifest.py`; the manifest is documentation
and evidence inventory only, not physical real robot approval.

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
- `rb_servo_server/config/dual_simulator_circle_stress.yaml`
- `rb_servo_server/config/dual_simulator_remote_172_28_60_36.yaml`
- `rb_servo_server/config/dual_simulator_circle_baseline_15cm16s.yaml`
- `rb_servo_server/config/dual_simulator_circle_stress_15cm4s.yaml`
- `rb_servo_server/config/dual_simulator_circle_real_candidate_conservative.yaml`

Circle benchmark profiles are simulator-only. Use the baseline profile for
15 cm / 16 s evidence, the stress profile only for explicit 15 cm / 4 s stress,
and the real-candidate conservative profile only as a simulator seed for future
low-speed planning.

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

Deprecated simulator config names are archived under `docs/archive/configs/`
for historical reference only. They are not runnable source-of-truth configs
and must not be used for new smoke or acceptance evidence.
