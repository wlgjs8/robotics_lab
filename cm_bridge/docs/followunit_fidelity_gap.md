# FollowUnit 실행 충실도 격차 — 정책 추론 스트리밍에서 레거시 Chunk Follower 대비

**2026-08-17, robotics_lab. 논의 대상: controller-manager FollowUnit(0025)의 청크 재생 시맨틱.**

## 요약

동일한 pi0.5 정책(velocity proprio, 30 Hz 델타 청크)을 두 컨트롤러로 실기 롤아웃한 결과,
CM FollowUnit 경로는 레거시 rb_servo_server Chunk Follower 대비 **틱 단위 실행 충실도가
유의미하게 낮았다**. 총 이동량은 양쪽 다 보존되지만(전달률 ≈1.0) 타이밍 재현이 무너져,
정속 캡(150 mm/s = 5 mm/33.4 ms)을 넘는 **버스트 틱이 좌팔 기준 0 → 6.4%로 급증**했고,
로봇에서 육안으로 진동/흔들림·정확도 저하로 관찰됐다. velocity proprio 모델은 실행된
속도를 다음 관측으로 되먹임받으므로, 거친 실행이 더 거친 액션을 부르는 복리 루프가 생겨
모델 의도 델타 자체도 평균 ~2배 커졌다.

## 지표 (flow-infer 스텝 로그, 30 Hz 틱)

| 런 | 팔 | 의도↔실현 상관¹ | 버스트² | cmd–meas 오차 | 평균 의도 델타 |
|---|---|---|---|---|---|
| **CM** P0_e4 (8/17 18:39, 96 s) | 좌 | 0.82 | **6.4 %** | 2.4 mm | 1.49 mm |
| | 우 | 0.75 | 8.9 % | 3.8 mm | 2.00 mm |
| legacy B_p1_e4 (8/16) | 좌 | 0.91 | **0.7 %** | 0.8 mm | 0.65 mm |
| | 우 | 0.88 | 8.1 % | 2.3 mm | 2.32 mm |
| legacy baseline (8/16) | 좌 | 0.86 | **0.0 %** | 0.9 mm | 0.53 mm |
| | 우 | 0.82 | 7.8 % | 3.0 mm | 2.03 mm |

¹ 모델 raw delta 노름 vs 명령 스트림(tcp cmd) 틱 변위 노름의 Pearson, 지연 0–7틱 중 최적(전 런 lag=1).
² 실현 변위 > 5 mm/틱(= follow vmax 150 mm/s × 33.4 ms 예산) 틱 비율.

**읽는 법 — 좌팔이 대조군이다.** 우팔은 과제(볼트 파지)의 공격적 구간을 담당해 모든 런에서
버스트 ~8%가 나온다(모델 성질). 반면 좌팔은 조용한 팔인데 레거시 0~0.7% → CM 6.4%로 뛰었다.
즉 증가분은 모델이 아니라 **실행 경로가 주입**한 것이다. CM 런에서는 CM이 자체 캡 위반을
자른 `OVERSIZED delta` 경고도 3건 발생(정상 시 0건). 총 전달률(0.99–1.12)은 전 런 동일 —
"결국 다 가긴 가는데, 제때 못 간다"가 문제다.

## 메커니즘 (가설, 코드 관찰 기반)

- **레거시 Chunk Follower**: 청크 행들을 타임스탬프 있는 세트포인트 타임라인으로 취급,
  500 Hz Ruckig가 가감속·저크 연속으로 추종. 행 사이·청크 경계에서 속도 연속.
- **CM FollowUnit(0025)**: 주기 T=33.4 ms마다 델타 1개를 소비, 그 구간을 ‖δ‖/T **정속**으로
  재생("다음 커맨드 도착 전 도착 금지" 설계). 결과적으로 **매 주기 경계에서 속도 스텝**이
  구조적으로 발생하고, 도착 위상 슬립·경계 REPLACE(수신 청크가 미소비 꼬리를 교체)가
  경계 스텝으로 접힌다. 채널 내부에 cart a/j 제한은 있으나 주기 단위 정속 목표 자체가
  불연속의 근원.
- **복리 루프**: velocity proprio(= command 스트림 유한차분)가 이 불연속을 그대로 관측 →
  분포 밖 속도 입력 → 모델이 더 큰/거친 델타 출력(CM 런 평균 의도 델타 좌팔 0.65→1.49 mm)
  → 더 큰 스텝. 레거시에서 "모델이 추론한 action을 거의 그대로" 갔기에 이 루프가 안정했다.

## 재현 방법

1. 스택: `cm_bridge/run_cm_stack.sh real` + enable→mode real→task on→task idle,
   리셋 포즈 MOVJ (bridge 50256 JointTarget). follow 프로파일:
   `cm_bridge/config/follow.monkey.yaml` (T=33.4 ms, vmax 150 mm/s, admittance ON — 단
   이번 런들에서 wrench는 deadzone 미만, 오버레이 개입 0).
2. 롤아웃 (양 스택 동일):
   ```
   FLOW_INFER_CHECKPOINT=openpi://127.0.0.1:8001 FLOW_INFER_ACTION_HORIZON=24 \
   FLOW_INFER_CHUNK_EXECUTE_STEPS=4 FLOW_INFER_CHUNK_OVERLAY_RUNWAY_STEPS=4 \
   FLOW_INFER_RTC=1 FLOW_INFER_RTC_SCHEDULE=zeros FLOW_INFER_SPEED_SCALE=1.0 \
   RB_ALLOW_REAL_GRIPPER=1 ./tools/flow_infer_sweep_run.sh <tag> --proprio-mode velocity_grip
   ```
3. 분석: `python3 tools/analyze_follow_fidelity.py outputs/sweep/<run>.jsonl`
   (위 표를 그대로 출력; 원 데이터 8/17 `20260817_183922_P0_e4.jsonl`,
   8/16 `20260816_170005_B_p1_e4.jsonl`, `20260816_165637_baseline.jsonl`).

주의(교란 변수): CM 런은 P0(prefetch_at=0), 레거시는 B_p1(prefetch_at=1)·baseline 혼재,
날짜/씬 상이. 다만 좌팔 대조 논리와 OVERSIZED 로그는 이 차이로 설명되지 않는다.
짝지은 동일-날짜 재검(레거시 스택 재기동)으로 확정 가능.

## 논의하고 싶은 것

1. FollowUnit에 **행 타임라인 연속 추종 모드**(주기 정속 재생 대신, 델타 열을 시간 파라미터화된
   경로로 보고 Ruckig가 연속 추종 — 레거시 의미론)를 옵션으로 둘 수 있는지.
2. 차선책: 경계 스텝 완화(주기 경계에서 rate 슬루 제한), 또는 T보다 촘촘한 내부 보간.
3. 반대 방향 의견도 환영: 정속 재생 설계 의도("도착 전 도달 금지")와 정책 스트리밍 요구가
   충돌하는 지점을 어떻게 보시는지.

— robotics_lab 김재현 (분석 스크립트·원 로그 포함, 재현 가능)
