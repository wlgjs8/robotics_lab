# TcpTargetPose Real 파라미터 튜닝 — Phase 0 분석 (2026-06-22)

기존 pgmode 전수 프로파일링([[tcp-pgprofile-campaign]], `outputs/tcp_pgprofile*`)과 첫
Real 재생(`outputs/tcp_pgprofile_REAL/REAL_ANALYSIS.md`, episode_0008)을 종합해, **Real에서
TcpTargetPose 제어를 튜닝하기 전에 "무엇이 병목이고 어떤 파라미터를 어느 방향으로 움직여야
하는지"를 데이터로 확정**한 분석. 라이브 캠페인·파라미터 sweep의 출발점.

이 문서는 분석/제안이며 코드·config를 바꾸지 않는다. 실제 변경은 후속 단계(컨트롤러 코드 수정 →
파라미터 튜닝)에서 한다.

---

## 0. 목표 (사용자 고정)

1. **openpi(pi0.5) 정책을 정확히 추종** — wrist-cam 안정(떨림 없음), self-correcting 절대 추종.
2. **UMI류 큰/빠른 움직임을 부드럽게** — 진폭 손실(reach truncation)·끊김·떨림 없이.
3. 튜닝 표면은 전부 서버측. servo_j 4값(t1=0.002,t2=0.021,gain=1.0,alpha=10)은 **고정 투명실행기**
   계약이라 건드리지 않는다([[servo-j-transparent-executor-contract]]).

두 요구가 **같은 단일 튜닝 표면**(`cartesian_control.pose_track_smd` + `kinematics.ik` +
`safety.dq_max`)을 공유한다. openpi 롤아웃 deploy lane이 2026-06-21부터 `flow-infer
--command-family tcp_target_pose`(절대 `TcpPoseTarget` 위치제어, twist 경로 미안정)로 바뀌면서
이 표면은 **추론 critical path 위에 직접 올라와 있다**(`flow_inference.py:785-825`).

---

## 1. 핵심 발견 — 병목은 거의 전적으로 linear velocity cap

`data_tcp/replay_profiling_20260620/` 383개 전수 매니페스트
(`outputs/tcp_pgprofile_unified_fn1_vffon/episode_manifest.csv`,
`build_pgprofile_manifest.py`의 `required_time_scale_estimate`/`speed_precheck`) 분석:

| 지표 | 값 |
|---|---|
| ts=1.0 speed budget 통과 | **25 / 383 (6.5%)** |
| linear cap 0.25 m/s 초과 | **358 / 383 (93%)** |
| angular cap 1.745 rad/s(100°/s) 초과 | **1 / 383** |
| speed-limited 중 binding 제약 | linear **358** / angular **0** |
| max_stream_linear_speed | median 0.323 · p90 0.428 · **max 1.535** m/s |
| max_stream_angular_speed | median 0.977 · p90 1.314 · max 1.749 rad/s |
| required_time_scale_estimate | median 1.29 · p90 1.71 · max 6.14 |

**결론: 자연 진폭 UMI 동작의 93%가 현재 `max_linear_velocity_m_s=0.25`에서 클램프된다.**
angular cap은 거의 안 걸린다. 즉 "큰 UMI 움직임을 부드럽게"의 **단일 지배 레버 =
`pose_track_smd.max_linear_velocity_m_s` 상향**이다. angular cap은 현 상태(100°/s) 유지로 충분.

`required_time_scale` 분포(245개가 ≥1.2, 70개가 ≥1.6)는 "현재 캡에서 진폭을 살리려면 1.3~1.7배
느리게 재생해야 한다"는 뜻 — 실시간 정책/텔레옵에는 time_scale을 쓸 수 없으므로
([[tcp-pgprofile-campaign]]: time_scale은 lag knob이 아니고 speed-budget용·실시간 불가),
**캡 상향이 정답**.

---

## 2. 제약 — 캡 상향은 글로벌 dq_max와 직접 커플링

linear cap을 올리면 base축 근처 lateral 동작에서 관절속도 수요가 글로벌
`safety.dq_max_deg_s`(base joint 60°/s=1.047 rad/s)를 포화시킨다. 특이점/속도 원통 반경
R = v / dq_max ([[ik-deadzone-real-limits-tool]] "A 영역", [[ik-infeasible-region-overlay]]):

| max_linear_velocity | 원통 반경 R (base dq_max 60°/s) | speed-budget 통과율 |
|---|---|---|
| 0.25 (현재) | 0.239 m | 6.5% (25/383) |
| 0.33 (median) | 0.315 m | ~50% |
| 0.40 | 0.382 m | ~85% |
| 0.43 (p90) | 0.411 m | ~90% |

원통이 커지면 그 안에서 lateral 동작이 dq_max를 포화 → q_sent 포화 → 추종오차 → 컨트롤러
catch-up 버스트 = **떨림**([[umi-teleop-tremble-dqmax-saturation]]: SMD 캡 0.5/1.0이 dq_max 60을
초과시켜 226°/s 버스트 → 0.3/0.6으로 하향했던 그 현상). 따라서 캡↑는 다음 중 하나와 **묶어야**
한다:

- **dq_max 비례 상향** (base 60 → 90 등; 현 config는 이미 [60,60,60,90,90,120], 주석상 절반에서
  복원한 값) — 원통을 작게 유지. 단 하드웨어/안전 한계 내에서만.
- **accel 캡 보수 유지** (`max_linear_accel_m_s2=1.0`) — 높은 속도로의 램프를 완만하게 해
  dq_max 버스트를 줄임([[umi-teleop-tremble-dqmax-saturation]]: 낮은 accel이 떨림 완화).
- **A 영역 전용 가드** — base축 속도특이점 원통 가드 공백([[ik-deadzone-real-limits-tool]])을
  메우면 캡을 올려도 그 구역에서만 감속.

이 커플링이 **두 레짐을 실제로 가르는 지점**이고, 사용자가 "파라미터 튜닝 전에 컨트롤러 코드를
먼저 수정"하려는 이유로 보인다(§5 참조).

---

## 3. 두 레짐의 분기 (요구가 갈리는 곳)

| | openpi 롤아웃 | UMI 텔레옵(큰동작) |
|---|---|---|
| 명령 속도 | pi05 delta ~1.1mm/step @30Hz ≈ **0.033 m/s** | 자연 진폭 0.13~1.5 m/s |
| linear cap | **비병목** (cap≫명령속도) | **93% 병목** |
| 1차 과제 | 5–6Hz tremor 제거 + zero lag | 진폭 보존(캡↑) + 떨림 억제 |
| fn / ζ | fn=1.0, ζ=1.0(임계), vff=true 유지 | fn 상향(응답성), ζ=1.0 유지 |
| 캡 | 보수 0.25~0.32 | 0.40~0.45 + dq_max 동반 |
| 추가 | A-stage LPF 강화 + MA window 1→4 | max_solution_jump_deg(branch-jump 부드러움) |

**openpi 롤아웃**: 캡은 무관. 문제는 ep0008 Real에서 측정된 **5–6Hz tremor**
(`REAL_ANALYSIS.md` §2: A-stage clean_foh의 30Hz 소스 보간 잔류 ~0.1mm를 물리 팔이 공진 근처에서
~2× 증폭 → actual ~0.2mm + 가청 buzz). 추종 정밀도는 이미 production-grade(actual vs reference
p95 0.8–1.0mm, lag 52–58ms). 개선 레버: A-stage conditioning LPF cutoff 하향 / savgol 확대 →
MA window 소폭(1→4) → vff 기여 재점검 순.

**UMI 텔레옵**: 캡이 93% 병목 → 캡 상향이 1차. dq_max 커플링(§2) 동반 필수. branch-jump 데드락은
이미 `branch_jump_rate_limit:true`로 해결됨([[umi-rb-teleop]]); `max_solution_jump_deg`(현 2.0)는
이제 smoothness/lag knob.

→ **라이브 캠페인이 답할 질문**: "max_linear_velocity 0.40 + dq_max base 90"이 롤아웃 smoothness를
해치지 않고 양쪽을 만족하는 **단일 셋**이 되는가, 아니면 **2-프로파일**(롤아웃용 저캡 + 텔레옵용
고캡)이 필요한가. 사전 결정하지 않고 측정으로 정한다(사용자 지시).

---

## 4. 라이브 캠페인 설계 (소규모 5–8개, operator 실행)

### 4.1 대표 부분집합 (양 레짐 커버)

저속(롤아웃 대표, speed_precheck 통과, cap 비병목):

| episode | req_ts | max_lin | max_ang | spanL/R | n | 비고 |
|---|---|---|---|---|---|---|
| episode_0026 | 0.51 | 0.127 | 0.784 | 0.67/0.70 | 642 | 가장 느림 |
| episode_0008 | 0.86 | 0.215 | 0.703 | 0.77/0.83 | 427 | **Real 기 측정 baseline** |
| episode_0024 | 0.80 | 0.200 | 1.316 | 0.63/0.62 | 422 | 저lin·고ang |

큰동작 스트레스(cap-binding):

| episode | req_ts | max_lin | max_ang | spanL/R | 비고 |
|---|---|---|---|---|---|
| episode_0315 | 6.14 | **1.535** | 1.232 | 0.63/0.74 | peak linear(극단) |
| episode_0171 | 4.25 | 1.062 | 0.950 | 0.72/0.72 | 고속 |
| episode_0307 | 1.91 | 0.479 | 1.413 | **1.05/1.21** | peak span(대형) |
| episode_0371 | 2.07 | 0.517 | **1.465** | 0.74/0.82 | 고angular+대형 |

→ 7개. 저속 3 + 큰동작 4. (5개로 줄이면 0026 + 0008 + 0315 + 0307 + 0371.)

### 4.2 sweep 매트릭스 (REAL config key, CONTRACT §11 명명)

| knob | REAL key | 후보값 |
|---|---|---|
| 선형속도 캡 | `cartesian_control.pose_track_smd.max_linear_velocity_m_s` | 0.25, 0.33, 0.40, 0.45 |
| 관절속도 한계 | `safety.dq_max_deg_s` (base 3축) | 60, 90 |
| 출력 MA | `servo.output_moving_average_window` | 1, 4 |
| branch-jump | `kinematics.ik.max_solution_jump_deg` | 2.0, 4.0 |

전부 동시 격자(4×2×2×2=32) 대신, operator 시간 고려해 **단계적 1D sweep** 권장:
캡(고정 dq_max 90) → dq_max → MA → max_solution_jump. accel/angular/ζ/vff/fn은 1차 고정.

### 4.3 측정 (Real은 pgmode와 달리 q_actual 실측 → Tier C 활성)

Real은 q_actual이 물리적으로 움직이므로 pgmode에서 not_measured였던 **Tier C(physical goal
tracking)** 가 측정 가능. `analyze_pgprofile_run.py`(`tier_c()`)의 frozen-span 자동판정이 Real
로그에서는 정상 통과해야 한다(REAL_ANALYSIS.md가 이미 actual_tcp 추종을 측정).

캠페인 도구가 추가로 자동 측정/분류할 것:
- **tremor** — q_actual / TCP의 >5Hz HF RMS(공진 증폭 비율 ref→actual).
- **dq_max 포화율** — q_sent 관절속도가 dq_max에 닿는 틱 비율(캡↑의 부작용 정량).
- **self-collision 근접** — min_clearance / self_collision_flag 틱(ep0008은 0.37s 1회).
- **진폭 보존** — span_ratio(reference vs conditioned_goal), endpoint err(클램프 손실 정량).

---

## 5. 다음 단계 — 컨트롤러 코드 수정 (사용자 계획, 상세 TBD)

사용자는 파라미터 튜닝 **전에 컨트롤러(`rb_servo_server`, C++) 코드를 일부 수정**할 계획.
§2의 캡↔dq_max 커플링상, 후보가 될 만한 수정 방향(확정 아님, 사용자 의도 확인 필요):

- **base축 속도특이점("A 영역") 전용 가드** — 원통 안에서만 Cartesian 속도를 감속해, 캡을 전역으로
  올려도 dq_max 포화/떨림을 국소적으로 회피([[ik-deadzone-real-limits-tool]]의 가드 공백 메움).
- **dq_max 포화 시 SMD 속도 자동 백오프** — 관절속도 수요가 dq_max에 닿으면 SMD Cartesian 속도를
  되먹임 감속(현재는 outer 하드 클램프라 catch-up 버스트 발생).
- **A-stage conditioning LPF의 런타임화** — 현재 tremor 감소 권고가 offline `tools/tcp_tuning`
  smoothing에 머무름(REAL_ANALYSIS.md §4); 런타임 컨디셔너에 적용해 롤아웃 경로에서도 5–6Hz 제거.
- **vff 30Hz 전달 억제** — vff가 30Hz 소스 미분을 reference에 주입(우완 HF↑); A-stage가 깨끗하면
  clean feedforward가 되도록.

→ 코드 수정 범위가 정해지면 이 문서의 §4 sweep을 그에 맞춰 갱신한다.

---

## 6. 참고 (불변 — 건드리지 말 것)

- servo_j 4값 고정 계약([[servo-j-transparent-executor-contract]]); alpha=10=내부 LPF off,
  되돌리면 서버 SMD가 smoothing 소유권을 뺏김.
- rbpodo move_servo_j alpha/gain은 컨트롤러 내부 0.1 스케일([[rbpodo-servo-j-alpha-gain-0p1-scaling]]).
- Real 모션은 fail-closed: site-local config(`config/local/`) + 안전 레이어 + operator 감독 +
  E-stop. simulator acceptance 통과가 하드웨어 이동 허가가 아님.
- pgmode 컨트롤러 fault는 ResetFault로 안 풀림 → real/sim 모드 토글 재기동 필요
  ([[pgmode-controller-fault-clear-mode-toggle]]).
- 캠페인 client 속도가드는 높게(lin 5.0/ang 10.0) 유지해 스트림이 서버 도달 → **서버 SMD가 실제
  클램프**([[tcp-pgprofile-campaign]], `run_pgprofile_campaign.py:221-223`).
</content>
</invoke>
