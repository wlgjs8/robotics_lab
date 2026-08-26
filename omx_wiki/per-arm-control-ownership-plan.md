# P2: Per-Arm Control Ownership (설계 확정, 실행 대기)

2026-08-26. P0(wire 정밀도+계측)·P1(플랜-팔 결합)·P3(IK/도달성)은 커밋됨
(f80e0ee, bc4a840, f0904df). 이 문서는 남은 구조 작업 — 생성과 송신을 한
팔·한 시계로 합치는 것 — 의 실행 계획이다. 하루짜리가 아니고, 각 단계가
하드웨어 검증을 요구하므로 전용 세션에서 실행한다.

## 왜 (남은 페이오프)

두 시계 구조(loop 생성 500.006 Hz 고정 / worker 송신 ~499.35 Hz 슬루)가 남긴 것:

1. **리샘플링**: 팔당 ~0.66 setpoint/s가 mailbox 덮어쓰기로 wire에서 스킵
   (1.5 s 비트마다 박스 FIFO가 2틱 스텝 재생). P0 계측
   (`*_worker_pending_overwrites_total`, `*_worker_wire_send_start_ns`)으로
   이제 정량 관측 가능. 소리 원인 후보로서의 비중은 wire %.7f A/B 결과에
   따라 재평가.
2. **mailbox 홉 지연 ~1.1틱** (`sent->ref` 잔차 +3.0 중 +1.0).
3. 150 ms deadline race는 fault_classifier 강등으로 **이미 무해화**
   (fault가 아니라 hold). 구조 수정의 긴급 사유에서는 빠짐.

CM의 요지 (submodules/controller-manager, 읽기 전용 참고):
`Arm::run` 한 스레드가 wait(트림 절대 타이머) → send(지난 사이클 cmd) →
read → 생성 → qsync_step을 전부 소유. "생성과 송신이 다른 시계면 0.13%는
스트레치가 아니라 1/(500−499.35)≈1.5 s 비트가 된다"(결정문서 0010의 2.2 s
phase-crossing 분석). rt-hot-path 불변식: wakeup과 actuation write 사이
간격을 흔드는 모든 것은 FLL이 락 거는 스트림을 위상 변조한다.

## 단계 (각각 독립 배포·검증 가능)

### 2-a. computeServoTarget 팔별 추출 (동작 불변 리팩터)

`dual_arm_servo_loop.cpp:5610+` 705줄, `left_`/`right_` 참조 312개를
`computeArmServoTarget(ArmControlContext&)`로 추출. seam(`ArmControlContext`)은
존재. 여전히 loop에서 양팔 호출 — 출력 동일성은 servo_log 리플레이로 검증
(같은 명령 시퀀스 → q_sent byte-diff 0).

### 2-b. applySafety 팔별 분리

per-arm 스테이지(clamp/배리어/velocity/accel)를 팔별 함수로, cross-arm 투영
(`solveVelocityProjection`, 12-DoF)은 인터페이스 뒤로 격리. P1에서 이미
projection 텔레메트리·hyst 상태가 loop 멤버로 분리되어 있음.
이때 **스테이지 재배열도 함께**: accel 클램프를 투영 이후 최종단으로
(CM의 "step-limit guard는 send 직전 마지막 줄" 원칙). 현재는 투영이 accel
클램프 하류라 보정량이 accel 재검사 없이 주입된다(P1은 상한만 dq_max로 교체).

### 2-c. ArmWorker를 완전한 팔별 제어 루프로 승격 (본체)

```
wait(트림 절대 타이머)        ← qsync 트림이 전체 주기를 슬루 (CM PeriodicTimer:
                                deadline에서 전진, lateness 비누적)
send(지난 사이클의 cmd)       ← 1-cycle-delay 계약: 송신 위상이 상수
read state (pipelined)
명령 슬롯 소비 → follower tick → IK → per-arm safety
cross-arm 제약 적용 (async monitor의 row)
qsync_step → 다음 cmd 준비
```

- 게이트: 새 `servo.io_model: worker_owned` (기존 `worker`는 불변) — 기본
  경로를 바꾸지 않고 config 플립으로 활성화. 실패 시 즉시 원복 가능.
- `send_at_top`(pending_top_send) 기제가 1-cycle-delay 계약의 씨앗.
- loop는 감독자로: lifecycle(InitMotion 플래닝은 CM처럼 off-RT plan/on-RT
  replay), fault 집계, 상태 발행, 로깅(로거 큐는 이미 lock-free ring).
- RT 예산: 팔당 IK ~20 µs + safety + follower — 실측 합산 최악 1.6 ms는
  양팔+ROI 경로였고 P1이 그 경로도 가볍게 함. 팔당 0.8 ms 이내 예상, 2 ms
  예산 내. `worker_loop_read_duration_us`/새 wire 컬럼으로 실측.

### 2-d. 명령 인입: 팔별 latest-value 슬롯 + stale→brake

CommandServer가 팔별 seqlock 슬롯 기록(CM FollowUnit 방식), worker가
케이던스 틱에서 소비. 신선도 판정은 사용 지점에서 단 한 번 — dispatch
데드라인 검사(`isExpired`) 삭제(분류기 강등의 구조적 완성). stale =
P1 fault-hold 램프와 같은 브레이크 프리미티브로 감속 후 hold. fault는
link/상태 사망만 (CM: "명령 침묵은 절대 fault가 아니다").

### 2-e. [결정 포인트] cross-arm 충돌 투영의 소유권

12-DoF 결합 투영을 팔별 세계에서 어떻게 풀 것인가:

- **권고안**: monitor(이미 async)가 per-pair row 발행; 각 팔이 상대 팔의
  최신 intent 속도를 상수로 가정하고 자기 관절 성분만 적용(보수적 — 양팔
  동시 제동). 프로토타입 후 보수성 손실을 controller-sim에서 실측.
- 대안: 결합 투영을 monitor 스레드에서 풀어 팔별 보정 배포(+1틱 지연).

## 검증 사다리

1. 2-a/2-b: 리플레이 byte-동일성 + ctest 전체 (39개).
2. 2-c: MODE=sim(pgmode VM)에서 `worker_owned` 플립 —
   `analyze_smoothness.py`로 wire 주기/스킵 0 확인, `analyze_box_latency.py`로
   `sent->ref` 잔차 +3.0→+2.0틱(홉 제거분) 확인.
3. real: 기존 감독 하 러너 절차. 판정: overwrites/repeats 카운터 0,
   fill 5 락 유지, end-to-end ≤ 현재 22.3 ms − 2 ms.

## P2가 더 이상 풀 필요 없는 것 (이번에 해결됨)

- deadline race → fault_classifier 강등 (f0904df)
- run-ahead 런지/재앵커 stop-go → plan gate + 속도 상속 (bc4a840)
- wire 0.001° 계단 → servo_j_text_precision 7 (f80e0ee; A/B로 소리 기여 판정)
