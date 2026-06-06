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
- mandatory Eigen3/Pinocchio C++ Cartesian math path for `rb_servo_server`

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
- `docs/current_review.md`: `REVIEW.md`로 가는 짧은 redirect입니다. 내용을 중복하지 않습니다.
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

## Safety Gates

실제 로봇 연결:

```bash
RB_ALLOW_REAL_ROBOT=1
```

실제 joint servo motion:

```bash
RB_ALLOW_REAL_MOTION=1
```

`rbpodo`에서 controller ACK 대기를 끈 상태로 Servo J motion을 테스트하려면
추가 gate가 필요합니다. ACK-off 성공은 controller acceptance가 아니라
socket/API send evidence입니다.

```bash
RB_ALLOW_RBPODO_ACK_DISABLED_MOTION=1
```

실제 Cartesian/TCP motion:

```bash
RB_ALLOW_REAL_CARTESIAN=1
```

Rainbow controller box를 `pgmode` simulation으로 둔 `rbpodo`
controller-simulation circle benchmark는 hardware-free `rb_simulator`와
future physical real robot benchmark 사이의 별도 evidence category입니다.
이 경로는 실제 controller IP에 접속하므로 config는 `run_mode: real`,
`backend_type: rbpodo`를 쓰지만, robot `operation_mode: simulation`이어야
하고 physical robot should not move입니다. Tracking은 보통 controller
reference인 `tcp_ref_stand`를 사용하며 summary에는
`physical_motion_expected=false`가 기록되어야 합니다.

Streaming Cartesian primitive를 controller simulation에서만 열려면 config와
env가 모두 필요합니다.

```yaml
cartesian_control:
  allow_in_controller_simulation: true
  allow_in_real: false
```

```bash
RB_ALLOW_REAL_ROBOT=1
RB_ALLOW_REAL_MOTION=1
RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1
RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1
RB_RBPODO_PGMODE_SIMULATION_CONFIRMED=1
```

`RB_ALLOW_REAL_CARTESIAN`은 이 workflow에 사용하지 않습니다. Servo J ACK가
보여도 circle이 실행되었다는 뜻은 아닙니다. `cartesian_solve.status`가
`unavailable`이고 `circle_fit`이 singular이면 tracking failure가 아니라
server-side Cartesian gate/configuration 문제입니다.

이 gate들은 필요 조건일 뿐 충분 조건은 아닙니다. Config와 real-hardware acceptance도 해당 동작을 명시적으로 허용해야 합니다.

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

Force control은 비활성 상태를 유지합니다.

```yaml
force_control:
  provider: null
  enable: false
```

## Motion Primitive 요약

- `TcpPoseTarget`: PTP / MoveJ-like Cartesian final-pose target입니다. Cartesian path는 보장하지 않습니다.
- `TcpLinearMove`: simulator-only MoveL-like Cartesian path primitive입니다.
- `TcpTwistLocal` / `TcpTwistStand`: 기본은 simulator-only streaming
  Cartesian velocity primitive입니다. 예외적으로 rbpodo controller
  `pgmode` simulation carve-out은 `operation_mode: simulation`과
  `physical_motion_expected=false` telemetry가 확인될 때만 허용됩니다.
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
Use `tcp_ref_stand` as the tracking source in pgmode simulation, keep
`physical_motion_expected=false`, and include
`RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1` only for the controller-simulation
Cartesian carve-out. `policy_runner` is separate from this live view; GUI and
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
`real_policy` remains blocked unless real motion, measured or accepted retarget,
collision, gripper, and geometry gates are present.

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
