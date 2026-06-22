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

## 라이브 세션 프로파일링 (UMI 텔레옵 / 모델 rollout)
`replay`와 **같은 analyzer/표**로 라이브를 보려면, 서버 state-fanout(50356, recorder 슬롯; make run 시
50366=viser·50376=policy만 점유하고 50356은 비어 있음)을 `profile_live_session.py`로 수동(passive) 녹화한다.
명령 권한·lease 없이 순수 구독이라 안전. 캡처되는 계층: **B(smd_goal→smd_ref) · C(출력 MA) · D(actual_tcp
vs ref = 물리추종/떨림, real에서만 유효)**. A-tier(raw source→conditioned)는 commander 로그에 있음(아래).

**UMI 텔레옵** (`make run MODE=real`로 텔레옵 기동 후, 별 터미널에서):
```bash
python3 scripts/profile_live_session.py --label teleop_run1 --analyze
# ... 손동작(느린/빠른/큰) 수행 ... Ctrl-C로 정지+분석
```
A-tier 원천: 텔레옵 수신측 per-step 로그(`POLICY_RUNNER_UMI_TELEOP_LOG`, pika/logs).

**모델 rollout** (`ACTION_SOURCE=none make run MODE=real` + flow-infer; 별 터미널에서 레코더):
```bash
python3 scripts/profile_live_session.py --label rollout_h8 --analyze
# flow-infer 예:
DISPLAY=:1 PYTHONPATH=policy_runner OPENPI_REMOTE_SKIP_WARMUP=1 RB_ALLOW_REAL_GRIPPER=1 \
  ~/openpi/.venv/bin/python -m policy_runner flow-infer \
  --checkpoint openpi://127.0.0.1:8000 --config policy_runner/config/flow_real.yaml \
  --rollout-mode real_policy --command-family tcp_target_pose --allow-tcp-target-pose \
  --ee-local-r-align pika_rz180 --max-linear-step-m 0.010 --policy-dt-sec 0.0334 \
  --chunk-execute-steps 8 --camera-preview \
  --tcp-target-pose-conditioning foh_se3   # A-stage를 replay와 동일 형태로 (전이성 위해 권장)
```
A-tier 원천: flow-infer per-step action 로그(`actions_*.jsonl`, raw_delta·emitted_target·conditioner telemetry).
`--chunk-execute-steps`(=action_horizon//2 실행)와 reanchor는 policy 고유라 이 캡처에서 함께 관찰된다.

→ 산출 `outputs/tcp_live_profile/<label>/{pgprofile_result.json, pgprofile_summary.md, run_meta.json}`.
raw `log.csv`(~1MB/s)는 `--analyze` 후 자동 삭제(원본 유지=`--keep-log`). replay 표와 나란히 비교하면
"단일 제어기 config가 replay·텔레옵·rollout 셋 다 만족하는지"를 같은 지표로 검증.

**반복 누적:** 같은 `--label`로 다시 돌리면 **덮어쓰지 않고** `<label>_02`, `_03`... 으로 쌓인다(한 log에
append 금지 — lag/span/HF가 런 경계에서 깨짐). 여러 런 비교:
```bash
python3 scripts/profile_live_session.py --out-dir outputs/tcp_live_profile --aggregate
# -> aggregate_table.csv: run/class/B_pos_p95/D_actual_vs_ref_p95·max/clip/branch/selfcol
```
반복은 **run-to-run 변동성·다양 조건·task success** 용도이고, 제어 파라미터 프로파일 자체는 1~몇 회면 충분.

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
