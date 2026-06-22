# RUNBOOK — A/B/C 분리 후 TcpTargetPose 튜닝

`ARCHITECTURE_ABC.md`의 A/B/C/D 분리가 끝난 상태에서 파라미터를 튜닝하는 절차. 각 단계는 한 계층만
움직이고, 그 계층의 telemetry로 효과를 확인한 뒤 다음으로 넘어간다. servo_j는 튜닝 금지(고정 계약).

pytest는 ROS `launch_testing` 자동로드로 실패하니 항상 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`.
C++ 빌드: `cmake --build rb_servo_server/build -j` (Eigen3 + Pinocchio `/opt/openrobots`).

## 권장 튜닝 순서

1. **500Hz wall-clock 디스패치 확인** (time_scale이 디스패치율이 아니라 episode-time만 늦추는지).
   - replay `--time-scale-mode wall_clock_resample`(기본). run_meta의 `wall_clock_dispatch_rate_hz`=500,
     `effective_command_rate_hz` p50≈500, `stale_or_repeated_target_count`≈0(genuine hold 제외) 확인.
2. **VFF=true + clean_foh_se3 확인** — ramp lag≈0인지(`smd_velocity_feedforward_used`=true).
   command_twist를 검증하려면 config `velocity_feedforward_source: command_twist` + `--send-conditioned-twist`,
   telemetry `smd_velocity_feedforward_source=command_twist`·`smd_velocity_feedforward_fallback=false` 확인.
3. **A-stage 스무딩 sweep** — `--conditioning-config configs/tcp_conditioning_{savgol_w7,savgol_w15,
   lowpass_4hz}.yaml`. A-tier `conditioned_goal HF power`(>5Hz)와 raw→clean 감소율로 평가. 5–6Hz
   command-origin tremor(`REAL_PARAM_TUNING_ANALYSIS.md` §3)는 **여기서** 제거(C로 가리지 말 것).
4. **fn_lin/fn_ang sweep (VFF=true 유지)** — `pose_track_smd.natural_frequency_*`. B-tier lag/HF로 평가.
   ζ=1.0(임계감쇠) 유지(오버슈트/떨림 방지).
5. **출력 MA를 최종 폴리시로만** — `servo.output_moving_average_window`(1→4–8). C-tier
   `q_target_before/after_output_ma` HF 감소율·added lag로 평가. lag 비용이 있으니 소폭만, vff가 lag 흡수.
6. **그 다음에야 jerk-limited OTG 검토** (현재 미구현; B 구조 확정 후).

## 예시 명령

A-stage conditioning config로 replay(드라이런):
```bash
PYTHON... python3 scripts/replay_episode_tcp_pose_target.py \
  --source ee_local --data-tcp <ep.hdf5> --segment auto-largest \
  --server-config rb_servo_server/config/local/stack_real.yaml \
  --conditioning-config configs/tcp_conditioning_savgol_w15.yaml \
  --time-scale 1.0 --time-scale-mode wall_clock_resample \
  --out-dir outputs/tcp_tuning            # 실행은 --execute --i-am-at-the-estop 추가
```

conditioned twist를 SMD command_twist feedforward로(서버 config `velocity_feedforward_source:
command_twist` 필요):
```bash
  ... --send-conditioned-twist
```

배치 프로파일링(campaign):
```bash
python3 scripts/run_pgprofile_campaign.py --episodes-dir data_tcp/replay_profiling_20260620 \
  --server-config rb_servo_server/config/local/stack_real.yaml \
  --time-scale 1.0 --conditioning-config configs/tcp_conditioning_savgol_w15.yaml
```

policy 롤아웃(openpi tcp_target_pose, foh_se3 컨디셔닝):
```bash
python3 -m policy_runner flow-infer --checkpoint openpi://127.0.0.1:8000 \
  --rollout-mode real_policy --command-family tcp_target_pose --allow-tcp-target-pose \
  --tcp-target-pose-conditioning foh_se3 --tcp-target-pose-reanchor-mode measured_blend \
  --policy-dt-sec 0.0334            # command_twist까지: --send-conditioned-twist
```

분석/리포트:
```bash
python3 scripts/analyze_pgprofile_run.py --log <run>/log.csv --time-scale 1.0 \
  --speed-precheck-pass true --validity-class VALID_FULL_NO_GAP
python3 scripts/report_pgprofile.py ...        # REAL_READY는 REAL_READY_TS_<ts> 합산
python3 scripts/compare_pgprofile_stages.py ... # 스테이지 비교
```

## 경고

- **camera 안정성은 B-tier 추종만으로 판단하지 말 것** — D(물리 actual_tcp HF)와 실제 wrist-cam으로 확인.
  B가 좋아도 물리 팔이 공진 증폭할 수 있음(`REAL_PARAM_TUNING_ANALYSIS.md` §3).
- **self-collision-active 에피소드를 SMD smoothness 튜닝에 섞지 말 것** — velocity-damper 개입 구간은
  SMD 거동이 아님. 분류기의 `SELF_COLLISION_RISK`/`smd_clip`로 걸러서 본다.
- **servo_j 튜닝 금지** — t1/t2/gain/alpha는 고정 계약. 부드러움은 A/B/C에서.
- **command artifact를 C로 가리지 말 것** — 30Hz 계단/5–6Hz tremor는 A에서 제거가 정답. C는 최종 폴리시.
- **command_twist는 frame이 맞아야** — conditioned twist의 angular는 body-frame(R0⁻¹R1, SMD와 동일).
  `se3.twist_from_poses`가 이 convention(Fix 3). 직접 twist를 만들면 frame 주의.
- **라이브 전 dry-run 필수** — `VIOLATION:` 없는지, stream_speed가 client clamp 내인지 확인.
</content>
