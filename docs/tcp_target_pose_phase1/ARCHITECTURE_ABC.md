# TcpTargetPose 제어 아키텍처 — A/B/C/D 책임 분리

`TcpPoseTarget`(위치제어) 경로를 **명령 컨디셔닝(A) → 레퍼런스 생성(B) → 최종 폴리시(C) →
물리 추종(D)** 네 계층으로 분리한다. 각 계층은 독립적으로 측정·튜닝 가능해야 하며, 한 계층의
artifact를 다른 계층에서 "고치지" 않는다(예: A-stage 30Hz 계단은 C의 출력 MA로 가리지 말고 A에서
제거).

서보 실행기(`move_servo_j`)는 **고정 투명 실행기**(t1=0.002/t2=0.021/gain=1.0/alpha=10=내부 LPF off)로,
응답성/부드러움/정확도는 전부 서버측이 소유한다(`docs/servo_backend_contract.md`,
robotics memory `servo-j-transparent-executor-contract`). 따라서 튜닝은 A/B/C에서만 한다.

---

## A — 명령 컨디셔닝 (Command conditioning)

저(低)레이트 소스 명령을 500Hz SE(3) 스트림으로: 노이즈 제거/스무딩, FOH 보간, dropout/hold/gap 처리,
소스-타이밍 기반 goal twist 추정.

| 경로 | 구현 |
|---|---|
| replay (오프라인/실로봇 재생) | `tools/tcp_tuning/command_conditioner.py`(`CommandConditioner`), `smoothing.py`, `se3.py`. 드라이버 `scripts/replay_episode_tcp_pose_target.py`가 wall-clock resample(`build_wall_clock_replay_stream`, time_scale가 episode-time만 늦추고 디스패치는 500Hz 유지)로 스트림. conditioning은 `--conditioning-config`/`--smoothing-*`/`--gap-*`로 튜닝(예시 `configs/tcp_conditioning_*.yaml`). |
| policy 롤아웃 (openpi/flow) | `policy_runner/policy_runner/tcp_target_pose_conditioner.py`(`OnlineTcpTargetPoseConditioner`, torch-free). 30Hz chunk step target → 500Hz SE(3) FOH. `flow-infer --tcp-target-pose-conditioning {legacy_step_hold(기본), foh_se3}` + `--tcp-target-pose-reanchor-mode {measured_legacy, last_emitted_continuous, measured_blend(기본)}`. **async-stream 경로 전용**; 비스트림(offline) 경로는 foh여도 legacy로 폴백. |
| teleop (UMI) | **A-stage online 컨디셔너 미적용**(현재 relative-init clutch + 서버 SMD가 직접 스무딩). 향후 foh_se3 컨디셔너를 텔레옵에 연결 가능(미구현). |

소스-타이밍 conditioned twist: `se3.twist_from_poses`(linear=stand frame, **angular=body frame
R0⁻¹R1** — SMD와 동일 convention). policy 측은 `OnlineTcpTargetPoseConditioner`가 동일 추정.

## B — 레퍼런스 생성 (Reference generation)

vel/accel(향후 jerk) 한계를 둔 2차 SMD가 conditioned goal을 추종해 500Hz 레퍼런스 생성.

- 구현: `rb_servo_server/src/control/smd_pose_tracker.cpp`(`SmdPoseTracker`),
  호출 `src/control/dual_arm_servo_loop.cpp::applyPoseTrackSmd`.
- 튜닝: `cartesian_control.pose_track_smd.{natural_frequency_*, damping_ratio_*, max_*_velocity/accel,
  velocity_feedforward, velocity_feedforward_source}` + `kinematics.ik.*`.
- **velocity feedforward source**(Patch 5): `finite_difference`(기본) / `command_twist` / `auto`.
  `command_twist`는 명령에 동봉된 conditioned twist(`tcp_target_twist_stand`)를 goal velocity로
  사용(A/B 계약을 닫음); 없거나 non-finite면 finite_difference로 안전 폴백(telemetry에 flag).
  replay `--send-conditioned-twist`, policy `--send-conditioned-twist`로 twist 동봉.
- 출력 `smd_ref_stand`(IK 전)는 B의 산출물이며 telemetry로 발행(아래).

## C — 최종 폴리시 (Final polish)

관절 출력단 boxcar moving average(`servo.output_moving_average_window`,
`include/rb_servo/control/joint_moving_average.hpp`). **command artifact의 1차 해결책이 아니라
최종 폴리시 단계**로만 사용(과도 사용 시 lag↑). 향후 wrist-camera HF notch / input shaping 자리.

## D — 물리 추종 (Physical tracking / robot execution)

서보 내부 루프가 q_target을 물리 추종. pgmode 컨트롤러-sim은 q_actual frozen이라 D 측정 불가
(real 하드웨어 또는 rbsim 필요).

---

## A/B/C/D Telemetry (Patch 4) — 측정 가능성 보장

서버가 per-arm `cartesian_solve`(state JSON `robotics_lab.servo_state.v1`)에 발행:
`smd_goal_stand`(B input) · `smd_ref_stand`(IK 전 SMD 출력=B output) · `smd_velocity_feedforward_used/
source/fallback` · `smd_{linear,angular}_{velocity,accel}_clipped` · `smd_goal_{linear,angular}_velocity_
ff_clipped`(VFF spike guard) · `smd_goal_{linear,angular}_velocity_norm_*` · `smd_reanchor_count` ·
`output_ma_present/window/active` · `q_target_before/after_output_ma_deg`. 기존 `tcp_ref_stand` 불변.

구현: `SmdPoseTracker::lastStepInfo()`/`reanchorCount()` → `dual_arm_servo_loop`가 `AbcTelemetry`로
capture → `mergeAbcTelemetry`로 `CartesianSolveTelemetry`에 병합 → `state_publisher.cpp::cartesianSolveJson`
직렬화. replay `log_row`가 이를 top-level 컬럼으로 방출.

분석기 매핑(`scripts/analyze_pgprofile_run.py`, `tools/analyze_tcp_replay_logs.py`):
- **A** `source_raw_target → conditioned_goal_after_A`(raw-clean p95, HF power/감소율).
- **B** `conditioned_goal_after_A → smd_ref_stand`(있으면) / 없으면 `tcp_ref_stand`+경고
  ("post-IK/safety/MA, not pure SMD"). lag/span_ratio/endpoint + `smd_clip` 분해(vel/accel/ff clip
  별 카운트=clip reason 분리) + vff source/used ratio.
- **C** `q_target_before/after_output_ma_deg`(HF 감소율·MA lag); 필드 없으면 unavailable.
- **D** `actual_tcp vs tcp_ref_stand`(pgmode=not_measured).

분류는 time_scale-aware(Patch/Fix 5): `REAL_READY_TS_<ts>`(1.0→1P0, 1.5→1P5, 1.25→1P25, 2.0→2P0).

---

## 경로 완성도

| 경로 | A | B | C | telemetry |
|---|---|---|---|---|
| replay | ✅ wall-clock resample + conditioning config | ✅ SMD(+command_twist) | ✅ output MA | ✅ |
| policy 롤아웃(openpi/flow) | ✅ foh_se3 online(async-stream) | ✅ SMD(+command_twist) | ✅ output MA | ✅ |
| teleop(UMI) | ⛔ online A 컨디셔너 미연결 | ✅ SMD | ✅ output MA | ✅ |

## 알려진 한계 / 미구현

- **jerk-limited OTG/Ruckig 미구현** — B는 현재 2차 SMD만. (의도적 보류; 먼저 SMD 구조를 측정 가능하게.)
- **wrist-camera 전용 HF filter / notch / input shaping 미구현** — C는 현재 boxcar MA만.
- **direct conditioned twist는 opt-in** — 기본 `finite_difference`. `command_twist`/`auto`는 config +
  `--send-conditioned-twist` 필요. twist 없는 기존 클라이언트 정상(완전 하위호환).
- **teleop A-stage online 컨디셔너 미연결**.
- **라이브 미검증** — C++는 빌드+ctest, Python은 단위테스트만(실로봇 모션 검증은 별도).

관련: `CONTRACT.md`(Phase-1 스키마), `REAL_PARAM_TUNING_ANALYSIS.md`(Phase 0 분석),
`RUNBOOK_TUNING_AFTER_ABC.md`(튜닝 순서/명령).
</content>
