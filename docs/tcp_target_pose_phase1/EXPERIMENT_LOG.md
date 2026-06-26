# TcpPoseTarget 튜닝 — 실험 정리 & 실행법 (2026-06-20)

UMI 수집 에피소드(`data/data_20260619_115712`, 13개)를 **TcpPoseTarget로 실로봇/pgmode에 재생**해 컨트롤러를 프로파일링·튜닝한 작업의 정리. 모든 도구·설정·발견·실행법.

---

## A. 목표
1. 큰/빠른 UMI 텔레옵류 모션을 받으면서, 위치기반 모션을 정확히·떨림 없이(wrist-cam 안정) 재생하는 TcpPoseTarget 제어 튜닝.
2. 학습 모델이 아직 없으니 **수집 에피소드를 프록시**로 재생해 추종/IK/smoothness를 프로파일.
3. servo_j 4값(t1=0.002,t2=0.021,gain=1.0,alpha=10)은 **고정 투명실행기**(별도 계약) — 튜닝은 전부 서버측(pose_track_smd / IK).

## B. 구축한 도구 (`tools/tcp_tuning/` + `scripts/` + `docs/tcp_target_pose_phase1/`)
| 파일 | 역할 |
|---|---|
| `docs/tcp_target_pose_phase1/CONTRACT.md` | 스키마·인터페이스·제약 단일 기준 |
| `docs/tcp_target_pose_phase1/01_inspection_report.md` | C++ tcp_target_pose 파이프라인 인스펙션(A 명령조건/B SMD/C polish 분리) |
| `tools/audit_episode_hdf5.py` | HDF5 audit(주파수·gap·세그먼트·속도·gripper 이벤트) |
| `tools/tcp_tuning/{se3,hdf5_io,config,smoothing,command_conditioner,trajectory_log,metrics}.py` | 공유 모듈(SE3·스키마탐지·컨디셔너·로그·메트릭) |
| `tools/generate_replay_target.py` | raw_zoh/raw_foh_se3/clean_foh_se3 500Hz npz 생성(+`--segment`, full-clean 위험 플래그) |
| `tools/analyze_tcp_replay_logs.py` | 로그/npz → metrics.json+summary+plots(추종/smoothness/PSD/IK health) |
| `scripts/replay_episode_tcp_pose_target.py` | **메인 드라이버**: data_tcp→ee_local 델타→**시작포즈 앵커 합성**→clean_foh_se3 컨디셔닝→TcpPoseTarget 스트림. fail-closed(ROI/floor/speed precheck·watchdog·lease). `--segment auto-largest`, `--action-scale`, `--time-scale`, `--anchor live/mock`, `--allow-controller-sim-arm-error` |
| `scripts/batch_replay_episodes.py` | 13개 루프: init-return(JointTarget)→replay→log. `--init-mode capture_current/rest_stow/joints`, `--skip-failed-episodes`, init-return lease grace + fault reset |

핵심 함수 재사용(검증된 것): `policy_runner.umi_pipeline.umi-convert`(retarget), `flow_dataset.pose_delta_local`/`pose_compose_local`(ee_local), `replay_episode_rollout.GroundTruthSource`(델타+r_align+scale).

## C. 실행법

### 0) 사전 — 에피소드를 data_tcp로 변환 (retarget, 1회)
```bash
for ep in data/data_20260619_115712/episode_*.hdf5; do
  python3 -m policy_runner umi-convert --input "$ep" \
    --output "data_tcp/data_20260619_115712/$(basename $ep)" \
    --format robotics_lab_dual_arm --retarget-config calibration/umi_retarget_eelocal.yaml
done
```
(이유: 원본 pose는 `steamvr_world` 프레임 → 절대 stand-frame으로 직접 재생 불가. ee_local + 시작포즈 앵커가 frame-gap-불변 정답.)

### 1) 오프라인 분석 (로봇 불필요)
```bash
EP=data/data_20260619_115712/episode_012.hdf5
python3 tools/audit_episode_hdf5.py --episode $EP --out-dir outputs/tcp_tuning --plots
python3 tools/generate_replay_target.py --episode $EP --mode clean_foh_se3 --segment auto-largest --out-dir outputs/tcp_tuning
python3 tools/analyze_tcp_replay_logs.py --npz <위 npz> --out-dir outputs/tcp_tuning --plots
```

### 2) 단일 에피소드 재생 (pgmode 권장; 물리 J5 결함 회피)
```bash
# pgmode 스택 기동 (teleop_mux 경쟁 제거 위해 ACTION_SOURCE=none)
ACTION_SOURCE=none make run MODE=sim
# state 포트 충돌 회피용 드라이버 config 생성 (state→단일 free 포트 50356)
#   = stack_sim.yaml 복사 + state_pub_endpoints를 50356 한 줄로
# 재생 (gate 제거됨 → --execute --i-am-at-the-estop 면 즉시 스트림)
python3 scripts/replay_episode_tcp_pose_target.py --source ee_local \
  --data-tcp data_tcp/data_20260619_115712/episode_012.hdf5 --mode clean_foh_se3 \
  --arms left,right --anchor live --segment auto-largest --action-scale 0.6 --time-scale 2.0 \
  --server-config rb_servo_server/config/local/stack_sim_replaybind.yaml \
  --out-dir outputs/tcp_tuning --execute --i-am-at-the-estop
```
- `--segment auto-largest`: gap 있는 에피소드(012)는 **gap 없는 최대 세그먼트만**(full clean npz는 gap 경계 속도스파이크 → real 금지).
- `--action-scale`: ee_local 델타 진폭 축소(특이점/floor 회피). **scale 0.5~0.6 권장**(0.8↑는 손목 특이점에서 컨트롤러 fault).
- `--time-scale 2.0`: 재생 절반속도(첫 테스트 보수적).

### 3) 13개 배치
```bash
printf 'RUN 13 capture_current\n' | python3 scripts/batch_replay_episodes.py \
  --episodes-dir data_tcp/data_20260619_115712 \
  --init-mode capture_current --source ee_local --mode clean_foh_se3 --segment auto-largest \
  --action-scale 0.5 --time-scale 2.0 --skip-failed-episodes \
  --server-config rb_servo_server/config/local/stack_sim_replaybind.yaml \
  --out-dir outputs/tcp_tuning --execute --i-am-at-the-estop
# 결과: outputs/tcp_tuning/batch_<ts>/batch_summary.{md,json}
```
- `--init-mode capture_current`: 시작 시 q_actual 캡처 → 매 에피소드 그 자세로 JointTarget 복귀(앵커 일관). **로봇/레퍼런스를 working 자세(z~0.2)에 두고 시작**해야 grasp가 bounds 안.
- `--skip-failed-episodes`: pre-flight 거부/mid-motion fault를 skip하고 다음 진행(pgmode 데이터수집용; 기본 off=real fail-closed).

### 4) pgmode 컨트롤러 fault 클리어 (ResetFault로 안 풀릴 때)
```bash
# 1) make run 정지 (TaskStop 또는 Ctrl-C — pkill은 샌드박스서 exit144)
# 2) 컨트롤러 재초기화
./tools/real_mode.sh ; sleep 3 ; ./tools/simulation_mode.sh
# 3) 재기동 → fault 풀림 (반드시 real_vs_simulation_mode==1 확인!)
ACTION_SOURCE=none make run MODE=sim
```

## D. 실험 흐름 & 핵심 발견 (시간순)
1. **프레임 갭**: 원본 pose=`steamvr_world`(z≈−2.5m). 절대 stand 재생 무효 → **ee_local 시작포즈 앵커**로 해결(init-delta 0, ROI 안). dry-run 게이트가 모션 전 차단.
2. **ZOH→FOH**: raw_zoh는 30Hz 스텝이 1틱 50 m/s 스파이크 → raw_foh_se3가 0.1 m/s(스파이크 제거). >5Hz 파워 수천 배↓.
3. **gap 세그먼트**: 012의 gap(frame59→60, 1.4s)=pick↔place 경계. full clean은 gap에서 1틱 0.16m jump(78 m/s) → `--segment auto-largest`로 회피(단 **반쪽만 재생** — pick 또는 place).
4. **lease 버그 2종 수정**: init-return watchdog이 첫 send 전 lease 검사(false lease_lost), 대화형 게이트 입력대기 중 lease drop → grace+게이트제거.
5. **실로봇 8 에피소드 완주**(scale 0.5): 좌완 추종 양호(1~3mm), **우완 005 스트레스(31.8mm/327µs)** = 손목 특이점.
6. **우완 J5 하드웨어 결함**(A231 abnormal joint motion → Emergency Stop): 펜던트 조그만으로도 발생 = HW. 실모션 보류.
7. **pgmode 전환**(물리 J5 회피, IK는 서버측이라 그대로 튜닝 가능): 우측 서보 강제활성화 후 양팔 init=6.
8. **IK 손목 특이점 = full-amplitude 한계**(기하학적):
   - TUNE-2 `branch_jump_rate_limit:true`(sim에 빠져있던 것) → jump 14.4°→2.0° 클램프.
   - TUNE-3 특이점 댐핑 `singular_region_eps:0.06/damping_max:0.08` → appl_damp ramp(0.01→0.15), 905틱까지.
   - TUNE-4 damping_max 0.20 → **악화**(689틱). **exact singularity(sigma_min=0)는 댐핑 강도 무관하게 IK 실패**.
   - 결론: scale 1.0/0.8은 손목 특이점 통과 → IkFailed/컨트롤러 fault. **scale ≤0.6(004 fault)·≤0.5(회피)**.
9. **batch 개선**: `--skip-failed-episodes`(거부 skip), init-return **fault reset**(서버 latch만; 컨트롤러 fault는 `real_mode/sim_mode` 토글 필요).

## E. 현재 설정 knob (`rb_servo_server/config/local/stack_sim.yaml`, sim 전용)
- `kinematics.ik`: `singular_region_eps:0.06`, `damping_max:0.08`(TUNE-3), `branch_jump_rate_limit:true`(TUNE-2), `max_solution_jump_deg:2.0`.
- `safety.self_collision.monitor_only:true`(SIM-ONLY, 분석용 — real은 enforce 유지).
- `safety.floor_constraint.z_min_m` / `roi_box.min_m[z]`: 사용자가 −0.2로 개방(grasp 깊이 허용, sim).
- 드라이버 로그에 `ik_min_singular_value`/`ik_applied_damping`/`ik_status`/`ik_reason` 추가(특이점 진단).
- **앵커**: capture_current는 pgmode 레퍼런스 드리프트에 취약 → 배치 전 **working 자세(z~0.2)에 둘 것**.

## D2. 방향 버그 — `pika_tip` r_align (root cause, 2026-06-20)
**증상**: 변환된 data_tcp 재생이 박스(stand −y)로 안 가고 엉뚱한 방향(+y/stand 쪽). episode_012만 그럴듯(접근축/회전 위주 세그먼트라 x,y flip이 안 드러남).
**원인**: 드라이버 `DEFAULT_R_ALIGN`이 `"0.0,0.0,1.0,-1.0,0.0,0.0,0.0,-1.0,0.0"` = **`pika_tip` 프리셋**(90° 순열, flow_inference.py:159). axis-probe(2026-06-15, Kabsch 6프로브 5/6 residual 0)가 **"a different wrong"**으로 판정한 옛 추측값. 측정된 정답은 **`pika_rz180` = diag(−1,−1,+1)**(접근축 z 기준 180° yaw, x·y만 flip). wiki umi-axis-probe / umi-data-replay-frames와 정확히 일치(증상 "Z 맞고 X,Y reach flip → +y(stand)로 감").
**적용 규약 동일성 확인**: 드라이버 `compose_ee_local_from_anchor`의 `deltas[:,0:3] @ r_lin.T` / `[:,3:6] @ r_ang.T`가 측정도구 `replay_episode_rollout.GroundTruthSource`(line 156-157)와 **바이트 동일** → 그 측정의 r_align이 그대로 유효.
**수정**: `DEFAULT_R_ALIGN = "pika_rz180"`(scripts/replay_episode_tcp_pose_target.py:56). batch는 override 없이 드라이버 기본 상속. 무-torch 환경용으로 `_resolve_r_align_without_torch`에 preset 이름맵(pika_rz180/pika_tip) 추가.
**라이브 검증(2026-06-20, pgmode-sim, real_vs_simulation_mode==1)**: episode_000 scale 0.5 재생 → 양팔 모두 **−y(박스)·하강** 방향(right Δy=−0.164m, z 0.22→0.16; left Δy=−0.161m, z 0.21→0.14). 5678틱 완주, fault_latched=False. pika_tip(옛) 대비 방향 반전 확인.
**함의(중요)**: D절의 scale/IK 실험(000-003 완주, 손목 특이점=full-amplitude 한계 등)은 **전부 잘못된 방향(pika_tip, +y/stand 뒤쪽)으로 재생된 것** → −y(박스, 앞쪽)로 바로잡으면 워크스페이스의 다른 영역을 지나므로 손목 특이점/floor/reach 거동이 달라질 수 있음. **scale/IK 결론은 방향 수정 후 재프로파일 필요**.

## D3. 배치 fault 캐스케이드 + 비단조 거동 (2026-06-20, 방향수정 후)
- **"한 에피소드 부드럽게 → 그 뒤 엄청 빠르게 끝" 정체**: 페이싱/파라미터 차이 아님(전 에피소드 동일하게 full setpoint 500Hz 스트림, 측정상 ok 에피소드는 비례 시간 ~40–64s). 한 에피소드가 **컨트롤러 RobotFault(code 1005)를 latch**하면 그 뒤 에피소드는 전부 init-return에서 `fault_latched`로 **즉시 SKIP** → viser에서 순삭처럼 보임. "부드러운 것=fault 전, 빠른 것=fault 후 no-op 스킵".
- **scale 비단조 fault(이상)**: scale 0.5 → ep000-004 ok, **ep005 fault**(left wrist). scale **0.4(더 낮음) → ep000 ok, ep001에서 더 빨리 fault**. 진폭↓인데 더 일찍 터짐 = 손목 특이점 진폭 단일 원인으로 설명 안 됨. 가설: (a) fault-clear 토글 직후 앵커/레퍼런스 미정착, (b) init-return JointTarget 자체가 유발, (c) controller-sim 비결정성. **미해결 — 재현·격리 필요**.
- **배치 구조적 한계 재확인**: 배치는 컨트롤러 fault를 in-process로 못 푼다(stop→real/sim 토글→restart만). 따라서 fault 한 번=나머지 전멸. 13개 완주하려면 fault를 아예 안 내야 함.
- **정리**: 세션 종료 시 `bp3vzebem` make run 정지 → real/sim 토글로 latch 클리어(simulation 복귀, servo_power_changed=false) → 전 프로세스 down 확인.

## E2. 전수 pgprofile 캠페인 (2026-06-21, scale=1.0 / ts=1.0 고정, 383개)
383개(`data_tcp/replay_profiling_20260620/`)를 **scale 1.0·time_scale 1.0 고정**으로 pgmode 전수 재생·분류. 실패는 고치지 않고 분류, 서버 profile(`outputs/tcp_pgprofile/current_smd/as_tested_stack_sim.yaml`) **무튜닝 평가**. 도구: `scripts/{build_pgprofile_manifest,analyze_pgprofile_run,run_pgprofile_campaign,plot_pgprofile,report_pgprofile}.py`. 산출물: `outputs/tcp_pgprofile/`(manifest.{json,csv}, per-ep audit/result, current_smd/{REPORT.md, plots/, as_tested yaml}).

- **3계층 goal metric**(사용자 명확화): A goal_conditioning_quality / **B controller_reference_goal_following = reference_after_B(tcp_ref_stand) vs conditioned_goal + IK·safety (pgmode HEADLINE)** / C physical_goal_tracking(=**not_measured**, q_actual frozen·span≈0 자동판정). actual_tcp 임계값 N/A 명시(무조작).
- **결과**: 컨트롤러 fault **0회**(방향 pika_rz180 수정 후 손목 특이점 cascade 비재현). 분류 — SPEED_LIMITED 286(74.7%), SELF_COLLISION_RISK 32(8.4%), TRACKING_LAG_HIGH 22(5.7%), IK_FAILURE 21(5.5%, mid-stream `safety_verdict=IkFailed`=손목 특이점), IK_BRANCH_RISK 17(4.4%), SINGULARITY_RISK 5(1.3%), **REAL_READY_TS_1P0 0**.
- **핵심**: B-tier 일관 우수(span_ratio≈1.0, endpoint p50 2.7mm, lag p50 94ms<150); pointwise pos p95(p50 24mm)는 SMD lag 산물(왜곡 아님→TRACKING_LAG_HIGH 보수적). IK 비병목(solve p95 ~14µs≪2ms). ts=1.0 속도budget 통과 25/383(제약=linear 0.25 m/s; required_ts p50 1.29). full amplitude 최대 실위험=양팔 self-collision 근접. → **scale 1.0/ts 1.0 동시 고정에선 REAL_READY 0 = 기하·속도적으로 미달성**(속도캡·self-collision·손목특이점). 드라이버 client 속도가드만 5.0/10.0로 올려 서버 SMD가 실제 클램프하게 함(서버 config 불변).

## F. 알려진 한계 & 다음
- **pgmode 추종 데이터는 무효**(q_actual 동결 → trackP95는 모션크기, 추종 아님). **서버측 IK(branch-jump/sigma_min/solve_us)만 유효**. 실추종은 J5 수리 후 real.
- **full-amplitude(scale≥0.6) replay는 손목 특이점에 막힘** — IK 튜닝으로 근접영역까진 견디나 exact singularity는 불가. 실용: scale ≤0.5, 또는 init-pose 재앵커로 특이영역 회피.
- **full pick+place 재생엔 gap-bridge + 그리퍼 동기 전송 미구현**(현재 segment 반쪽 + pose-only).
- 다음: (1) scale 0.5로 13개 최대 커버리지 + IK 프로파일, (2) gap-bridge/그리퍼, (3) J5 수리 후 real 추종 검증, (4) TUNE-2/3를 stack_real에 이식 검토.
