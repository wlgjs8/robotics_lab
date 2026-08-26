# 검토 요청: rb_servo_server queue_sync 셋포인트 드롭 → 박스 레퍼런스 2배 스텝

작성 2026-08-26. 작성자 세션이 로그 분석으로 도출한 결론이며, **독립 검토를 요청합니다.**
숫자는 전부 재현 가능한 형태로 적었습니다. 반증을 우선해 주세요.

---

## 0. 검토자가 판정해 줬으면 하는 것

1. **§3의 인과 사슬이 성립하는가** — 특히 lag-8 상관이 진짜 인과인지, 아니면 판독 앨리어싱/양자화의 산물인지.
2. **§4의 "현재 코드에서도 그대로 발생한다"는 판정이 맞는가** — 커밋 P0/P1/P3를 읽고 반박해 주세요.
3. **§6의 대안 가설 중 빠뜨린 것이 있는가.**
4. **§7의 권고 우선순위가 타당한가.**

동의하는 부분보다 **틀린 부분을 찾아주시는 게 훨씬 유용합니다.**

---

## 1. 배경 / 환경

- 리포: `/home/plaif/workspace/robotics_lab` (dual-arm RB3-730E, rbpodo 백엔드)
- 제어박스 펌웨어: **v8.7.3**. v8.6.1의 latest-queue → **FIFO**로 변경됨.
  `박스 지연 = RBACK queue fill + 1 tick` 이 정확히 성립 (기존 실측).
  근거 문서: `omx_wiki/rainbow-control-box-servo-j-latency-fw-v8-7-3.md`
- 그래서 호스트가 큐 깊이를 잡아야 함 → `queue_sync` (`target_fill: 5`, `servo.io_model: worker`)
- 관련 config (`rb_servo_server/config/stack_real.yaml`):
  - `servo_alpha: 10.0` → 벤더가 내부에서 0.1배 스케일 → 실효 1.0 = **컨트롤러 LPF OFF**
  - `output_moving_average_window: 1` → **호스트 MA도 OFF**
  - `servo_t1_sec: 0.002`, `servo_t2_sec: 0.021`, `servo_gain: 1.0`
  - `queue_sync.enable: true`, `target_fill: 5`
- 증상 (사용자 보고): 반복 동작 시 **가끔 모터에서 부드럽지 않은 소리**. 특정 런에서는 좌팔에서 나서 즉시 Ctrl+C.

### 커밋 타임라인 (중요)

분석에 쓴 로그와 바이너리는 **구코드**입니다.

| 시각 | 항목 |
|---|---|
| 08:03:17 | `rb_servo_server/build/rb_servo_server` 빌드 (분석 대상 런들이 이 바이너리로 돎) |
| 08:01~08:15 | 분석에 사용한 로그들 |
| 17:25 | `f80e0ee` P0: wire %.7f via eval(), wire/projection observability, smoothness regression gate |
| 17:42 | `bc4a840` P1: safety plan gate, live hysteresis, bounded-jerk clip |
| 17:52 | `f0904df` P3: pinned-final IK acceptance, reach backstop, tip-equivalent IK weighting |
| 17:53 | `a7e472a` P2 plan (문서만) |

**P0/P1/P3는 아직 빌드/하드웨어 검증 0회입니다.**

---

## 2. 측정 방법과 함정 (여기부터 읽어주세요)

재현 시 반드시 알아야 할 함정 3가지입니다. 초기에 저를 오진으로 이끌었습니다.

### 함정 1 — `left/right_send_start_ns` 는 워커의 실제 송신 시각이 아닙니다

worker I/O 모드에서 루프가 `worker.sendServoJ()` 를 호출하면 즉시 `acceptedSend()` 결과가
`storeImmediateAsyncResultLocked()` 로 `last_send_result_` 에 박힙니다. 로거는 거기서 읽습니다.
**즉 이 컬럼은 enqueue 시각입니다.** 이걸로 재면 송신 주기가 항상 2000µs로 나와서
"queue_sync 트림이 안 먹는다"는 잘못된 결론에 도달합니다.

> P0 커밋 메시지도 같은 말을 합니다: *"send_start_ns was the LOOP-side enqueue stamp all along"*.
> P0가 `worker_last_wire_send_start_ns` / `worker_wire_dispatches_total` 를 신설했습니다.

### 함정 2 — 실제 송신 수는 `rback_seq` 증가분으로 세야 합니다

`{arm}_rback_seq` 는 파싱된 RBACK 당 1 증가하고, RBACK은 **실제 송신 1회당 1개**입니다.
따라서 `Δrback_seq / Δt` = 워커의 진짜 와이어 송신률. 이게 §3의 핵심 계측입니다.

### 함정 3 — `q_actual` 의 가속도/저크는 판독 앨리어싱이 지배합니다

실측 `|a_act| p99` 가 `|v_sent| max` 에 **정확히 1000배(=2/dt)** 비례합니다. 이는 프레임이
한 번 반복 판독될 때 속도가 0, 2v 로 교대하는 신호이지 실제 진동이 아닙니다.
**저크/가속도 통계로 진동을 판정하면 안 됩니다.** (wiki의 Open 항목도 같은 경고를 합니다.)

또한 분석 당시 CSV는 0.001도 양자화였습니다. P0가 유효숫자 9자리로 올렸으므로
재측정하면 더 깨끗합니다.

---

## 3. 발견 1 — 메일박스가 셋포인트를 조용히 버리고, 박스가 2배 스텝을 실행합니다

### 3.1 메커니즘 (코드)

- 루프는 **고정 500.000 Hz** 로 셋포인트를 생성해 latest-wins 메일박스에 넣습니다.
- `queue_sync` 는 **송신 주기** 를 트림합니다 (`arm_worker.cpp:689-697`).
  박스 클럭에 맞추려면 그래야 합니다 — 트림은 옳은 동작입니다.
- 생성률과 송신률의 차이는 메일박스에서 **덮어써져 사라집니다**
  (`arm_worker.cpp:279-285` 에서 카운트, `:303` 에서 무조건 덮어씀).
- 박스는 FIFO를 틱당 1개씩 꺼내므로, **버려진 셋포인트 1개 = 박스 레퍼런스가 한 틱에 두 칸 점프**.

### 3.2 실측 (3개 런, 양팔)

`Δrback_seq` 로 실제 박스 수신 수를 세고 루프 틱 수와 비교:

```
run 080650  left  track :  11.296s  loop=5648 (500.000 Hz)  box=5641 (499.380 Hz)  drop 0.62/s (0.12%)
                  drain :   0.226s  loop= 113 (499.998 Hz)  box= 103 (455.751 Hz)  drop 44.25/s (8.85%)
            right track :  11.436s  loop=5718 (500.000 Hz)  box=5710 (499.300 Hz)  drop 0.70/s (0.14%)
run 073816  left  track : 116.102s  loop=58051(500.000 Hz)  box=57976(499.354 Hz)  drop 0.65/s (0.13%)
            right track : 116.022s  loop=58011(500.000 Hz)  box=57936(499.354 Hz)  drop 0.65/s (0.13%)
            right drain :   0.182s  loop=  91 (500.000 Hz)  box=  82 (450.549 Hz)  drop 49.45/s (9.89%)
run 060642  left  track : 152.012s  loop=75998(499.947 Hz)  box=75908(499.355 Hz)  drop 0.59/s (0.12%)
                  drain :   0.228s  loop= 114 (499.999 Hz)  box= 104 (456.139 Hz)  drop 43.86/s (8.77%)
```

측정된 송신률이 트림 예측치와 일치합니다: `trim_med = 2.60µs → 1e6/2002.6 = 499.35 Hz` (실측 499.35).
drain에서는 `trim = 200µs → 454.5 Hz` (실측 455.8).

### 3.3 인과 검증 — 드롭 시점과 박스 레퍼런스 2배 스텝의 lag 상관

`rback_seq` 가 안 오른 틱(=그 셋포인트가 덮어써진 틱)을 기준으로, `q_ref` 의 스텝이
주변 스텝 중앙값 대비 몇 배인지를 lag별로 측정. 대조군은 track 구간 무작위 틱 400개.

```
run 073816, 좌팔, 120초
 lag(틱)   드롭 시점 평균 배율   1.6배 초과 비율     대조군 1.6배 초과
    6           1.201            41/177 (23%)          7.45%
    7           0.688            13/186  (7%)          6.23%   ← 직전 스텝이 짧음
    8           1.874           131/181 (72%)          5.84%   ← 박스 dead time (fill 5 + transport)
    9           0.858            26/184 (14%)          7.01%
   10           1.234            50/178 (28%)          5.60%
```

lag 8에서만 **12.4배 증폭**. 그리고 lag 7이 0.688로 꺼지는 것이 스킵의 전형적 지문입니다
(짧은 스텝 하나 → 두 배 스텝 하나).

### 3.4 충격량

```
run 073816, 좌팔, lag-8 확정 이벤트 131개 / 120초 (1.09/s, 평균 0.92초 간격)
  정상 1틱 속도  : 중앙 6.80 deg/s   p90 41.42
  스파이크       : 중앙 13.30 deg/s  p90 83.20   최대 132.80
  환산 1틱 가속  : 중앙 3,487 deg/s²  p90 20,888  최대 33,187
```

비교: `safety.ddq_max_deg_s2` 는 **1500~3000**. 즉 **세이프티 필터 하류에서 자체 한계의
10~20배 가속 임펄스가 초당 한 번꼴로 주입**되고, `servo_alpha: 10.0`(LPF OFF) +
`output_moving_average_window: 1`(MA OFF) 이라 이를 거를 단이 없습니다.

소리가 났던 런(080650, 좌팔이 더 빠르게 움직인 구간)에서는 더 큽니다:

```
run 080650, 좌팔, lag-8 확정 이벤트 11개 / 11.3초 (0.97/s, 평균 1.03초 간격)
  정상 1틱 속도  : 중앙 21.50 deg/s   p90 24.73
  스파이크       : 중앙 43.60 deg/s   p90 49.50   최대 49.55
  환산 1틱 가속  : 중앙 10,875 deg/s²  p90 12,375  최대 12,412
```

> 주의: 073816 의 중앙값 3,487은 0.001도 양자화 바닥 위에서 계산돼 거칠고, p90/max 쪽이 신뢰도가 높습니다.

### 3.5 좌팔이었던 이유

드롭 자체는 좌우 대칭입니다 (073816: 좌 75, 우 75). 임펄스 크기가 관절 속도에 비례하는데,
소리 난 런(080650)에서 좌팔은 14~37 deg/s, 우팔은 1.6~6.9 deg/s 였습니다. 빠른 쪽이 들립니다.
그 런은 운동학적으로 깨끗했습니다 — 관절 한계 핀 없음, IK 1 iteration, 브랜치 점프 0, damping 일정.

---

## 4. 발견 2 — drain 이 실모션 중에 돕니다

run 080650 타임라인 (좌팔):

```
12.404s  스트림 개시 (idle → warmup)
12.406~12.440  박스가 ~40ms 간 소비 안 함, fill 0 보고
12.446   fill 11 → 12.452 fill 15 (백로그 일시 노출)
12.45~12.81  warmup 0.4초 동안 fill 15 고정 (박스 dead time 32ms), trim = 0
12.808~13.034  drain (trim 200µs) — 셋포인트 8.85% 드롭, 초당 44회 임펄스
13.034~  track (fill 5 lock)
```

이때 팔은 이미 `motion_state: Running`, `left_mode: TcpPoseTarget` 으로 ~9 deg/s 움직이는 중이었습니다.
스트림이 첫 모션 명령에 열리는 구조라, **매 스트림 개시마다 실모션과 warmup 0.4s + drain 0.2s 가 겹칩니다.**

---

## 5. queue_sync 자체는 정상입니다 (반증 시도 결과)

"queue sync가 안 된 것 아니냐"를 먼저 의심했고, 아래 근거로 **기각**했습니다.

- 오늘 30개 런 전체에서 `underrun / stall / highwater / redrain / no_consumption` 이벤트 **전부 0**
- fill 중앙값 5.0, 정확히 5인 틱 87~99%
- 적분항 양팔 **+2.60µs** 수렴 = 클럭 미스매치 0.13% 와 일치 (500.006 Hz 송신 vs 499.34 Hz 배출)
- drain 실효 배출률 45.5/s = trim 200µs 예측치와 정확히 일치

즉 **레귤레이터는 설계대로 동작합니다. 문제는 액추에이터(송신 주기)와 생성기(고정 500Hz)가
분리돼 있고, 그 차이를 메일박스가 조용히 버린다는 구조입니다.**

이건 wiki가 이미 Open 으로 적어둔 항목입니다:
> *"Resampling. The loop generates at 500.1 Hz and the worker sends at 499.0, so ~1.1 setpoints/s
> are overwritten. With the LPF off that discontinuity goes straight to the servo.
> **Whether it matters is unmeasured.**"*

이번에 측정된 셈입니다.

---

## 6. 현재 코드(HEAD = a7e472a)에서도 발생하는가 — 제 판정

| 발견 | 판정 | 근거 |
|---|---|---|
| 메일박스 드롭 → 2배 스텝 | **그대로 발생** | `arm_worker.cpp:279-285,303` 여전히 무조건 덮어씀. P0는 카운터(`worker_pending_overwrites_total`)만 추가. 와이어단 델타 클램프/보간 없음. `queue_sync_controller.cpp` 는 05:57 이후 무변경 |
| drain 실모션 중첩 | **그대로 발생** | `queue_sync` config/컨트롤러 무변경 |
| `send_start_ns` 계측 함정 | **계측만 수정** | P0가 와이어 시각 컬럼 신설. 기존 컬럼은 여전히 enqueue 시각 |
| LPF OFF + MA OFF | **무변경** | `servo_alpha: 10.0`, `output_moving_average_window: 1` |

드롭률은 박스-호스트 클럭 미스매치(0.13%)가 결정하므로 **0.6/s 그대로**로 예상합니다.

### 6.1 새로 들어온 것 중 소리와 직접 연결되는 것 — `servo_j_text_precision: 7` (P0)

제가 놓쳤던 **독립적인 두 번째 원인**입니다.

벤더 SDK가 `move_servo_j` 관절값을 **유효숫자 6자리**로 포맷 → J1 작업범위(209~278도)에서
와이어 위 0.001도 계단 → 박스의 t2 lookahead 가 이를 `jnt_ref` jerk 로 증폭.
CM은 이미 `%.7f` 로 못박아 뒀습니다 (`RobotLink.cpp`). P0가 `Cobot::eval()` 로
같은 채널에 직접 텍스트를 보내는 경로를 추가했습니다 (`rbpodo_backend.cpp:1926-1948`).

- **확인함**: `foldRbackResponses()` 가 분기 밖에 있어 eval() 경로에서도 RBACK 파싱은 유지됩니다
  (`rbpodo_backend.cpp:1975-1981`). 즉 §2 함정2의 측정법은 변경 후에도 유효합니다.
- **함의**: 제가 측정한 q_ref 고주파의 일부는 드롭이 아니라 이 계단일 수 있습니다.
  다만 §3.3 은 `rback_seq` 미증가 틱을 기준으로 잡았으므로 드롭 결론 자체는 독립적으로 성립합니다.
  → **검토자가 이 분리를 다시 따져봐 주세요.**

### 6.2 P1 이 잡은 경계 진동 (실측값이 제 임펄스보다 한 자릿수 큼)

P1 커밋 메시지에 적힌 실측값들:

- 클램프 이탈 릴리스 런지 **10,050 deg/s²**, RoiViolation 8회에 걸쳐 40.7mm 리드 축적
  → `safety.plan_gate` (플랜 시계를 realized/requested 비율로 감속)
- 재앵커마다 v=a=0 하드 스텝 **2,300~9,200 deg/s²** → 속도/가속도 승계
- overshoot 클립이 대입(teleport)이라 호버링 타겟에서 **±125,000 deg/s² 루프레이트 구형파**
  → 레이트 제한 접근으로 교체 (`safety_filter.cpp` clampAcceleration)
- projection 관절 실링이 InitMotion SMD 프로파일이라 한 틱에 조그 속도로 클램프
  **-55,000~-110,000 deg/s²** → `dq_max` 로 교체
- `hyst_m` 이 읽히지도 않아 ROI 면에서 **~10Hz 온/오프 플래핑** (0.6초에 6회)

**따라서 소리의 원인 분리 기준을 제안합니다:**
- 경계(ROI/self-collision/joint limit) 근처에서 나는 소리 → P1 이 상당 부분 가져갈 가능성 높음
- **자유공간 정속 구간에서 초당 한 번꼴** 로 나는 소리 → 드롭(§3)이 남음

---

## 7. 권고 (우선순위)

1. **`make build` 먼저.** 현재 실기에서 도는 건 08:03 바이너리로 P0/P1/P3 미포함입니다.
2. **A/B 는 한 번에 하나씩.** 지금 계단(precision)과 드롭(메일박스)이 같은 대역에서 겹쳐 있어
   동시에 바꾸면 기여 분리가 불가능합니다.
   - `servo_j_text_precision: 7 → 0`
   - `servo_alpha: 10.0 → 1.0` (LPF ON. 박스 1차 필터 tau ~19ms 가 임펄스를 먹음. 대가는 지연 +10틱)
3. **드롭 근본 수정 (미구현).** 워커 송신단에 per-tick 델타 클램프 또는 보간.
   메일박스에서 최신 셋포인트를 집을 때 직전 전송값 대비 1틱 분량 이상 진행하지 않게 제한하면
   버려진 셋포인트가 2배 스텝이 아니라 다음 틱으로 이월됩니다. drain 구간 9% 드롭도 함께 해소.
4. **per-arm 셋포인트 생성** (wiki Open 항목, P2 plan 문서 `omx_wiki/per-arm-control-ownership-plan.md`).
   생성 케이던스를 송신 케이던스에 종속시키면 드롭이 원리적으로 0. 비용 큼.
5. **스트림 개시 시 warmup+drain 을 모션 전에 끝내기.**

---

## 8. 제가 확신하지 못하는 것 (반증 환영)

- **"들린 소리 = 이 임펄스" 는 확정 못 했습니다.** 음향 측정이 없습니다.
  발생률(~1/s), 크기(세이프티 한계의 10~20배), 팔 편향(빠른 쪽), LPF-off 조건이 정합할 뿐입니다.
- §3.4 의 중앙값은 0.001도 양자화 바닥 위 계산이라 거칩니다.
- §3.3 의 lag-8 상관에서, 드롭 틱이 2~5개씩 군집으로 관측됩니다(로그가 루프율로 샘플링한 결과로 봄).
  순 드롭 수(75개/116초)와 no-advance 틱 수(194개)가 다릅니다. **이 불일치의 해석을 검토해 주세요.**
- 500Hz 제어 자체의 타당성(사용자가 별도로 제기한 질문)은 아직 안 봤습니다.
- 박스가 스트림 개시 후 ~40ms(과거 관측 ~254ms) 무시하는 이유는 여전히 미상입니다.

---

## 9. 재현 방법

```bash
cd /home/plaif/workspace/robotics_lab
# 분석 대상 로그 (구코드, 08:03 바이너리)
#   logs/servo_log_20260826_080650.csv   24.3s, 좌팔 소리 나서 Ctrl+C 한 런
#   logs/servo_log_20260826_073816.csv   129.9s, 통계 표본
#   logs/servo_log_20260826_060642.csv   334s, 최장 런
```

### 9.1 드롭률 (§3.2)

```python
# rate.py — Δrback_seq 로 실제 박스 수신 수를 세고 루프 생성 수와 비교
import csv, sys
import numpy as np
path = sys.argv[1]
f = open(path, newline=''); r = csv.reader(f); hdr = next(r)
idx = {n: i for i, n in enumerate(hdr)}
rows = [row for row in r if len(row) >= len(hdr)]
N = len(rows)
def num(n):
    i = idx[n]; o = np.empty(N)
    for k, row in enumerate(rows):
        try: o[k] = float(row[i])
        except Exception: o[k] = np.nan
    return o
def txt(n):
    i = idx[n]; return np.array([row[i] for row in rows])
t = (num('loop_start_time_ns') - num('loop_start_time_ns')[0]) / 1e9
run = txt('motion_state') == 'Running'
for arm in ('left', 'right'):
    ph = txt(f'{arm}_qsync_phase'); seq = num(f'{arm}_rback_seq'); trim = num(f'{arm}_qsync_trim_us')
    for phname in ('track', 'drain'):
        m = run & (ph == phname)
        if m.sum() < 60: continue
        i = np.where(m)[0]
        br = np.where(np.diff(i) > 1)[0]
        seg = max(np.split(i, br + 1), key=len)
        dt = t[seg[-1]] - t[seg[0]]
        loop_ticks = len(seg) - 1
        sends = seq[seg[-1]] - seq[seg[0]]
        tr = np.median(trim[seg])
        print(f"{arm:5s} {phname:6s}: {dt:7.3f}s loop={loop_ticks} ({loop_ticks/dt:8.3f} Hz) "
              f"box={sends:.0f} ({sends/dt:8.3f} Hz) trim={tr:.2f}us "
              f"pred={1e6/(2000+tr):8.3f} Hz DROP={(loop_ticks-sends)/dt:6.2f}/s")
```

### 9.2 lag 상관 (§3.3)

`rback_seq` 가 안 오른 틱을 찾고, 그 뒤 lag 0~14 에서 `q_ref` 스텝이 주변 중앙값의 몇 배인지
집계. 대조군은 track 구간 무작위 틱. lag 8 에서만 튀어야 인과입니다.
전체 스크립트는 `lag_correlation.py` 로 동봉했습니다. 핵심 판정만 옮기면:

```python
noadv = [k for k in track_idx[1:] if seq[k] == seq[k-1]]
dQR = np.vstack([np.zeros(6), np.diff(QR, axis=0)])   # QR = q_ref 6열
# lag 별로: j = argmax|dQR[k+lag]|, 주변 6틱 중앙값 대비 배율 > 1.6 인 비율
```

### 9.3 P0 이후 재측정 (권장)

빌드 후에는 새 컬럼으로 직접 셀 수 있습니다:

- `{arm}_worker_pending_overwrites_total` 증가율 = 드롭률 (예측: track **0.6/s**, drain **44~49/s**)
- `{arm}_worker_wire_dispatches_total` = 실제 와이어 송신 수 (`send_start_ns` 대신 이걸 사용)
- `{arm}_worker_repeated_sends_total` = 메일박스가 비어 마지막 셋포인트를 재전송한 횟수
- `scripts/analyze_smoothness.py` = P0가 오늘 서명들을 회귀 리포트로 만들어 둠

### 9.4 동봉 스크립트

`.venv/bin/python` 으로 실행 (numpy 만 필요, pandas 불필요):

```bash
cd /home/plaif/workspace/robotics_lab
.venv/bin/python review_request_2026-08-26/qsync_health.py      logs/servo_log_20260826_073816.csv
.venv/bin/python review_request_2026-08-26/drop_rate.py         logs/servo_log_20260826_073816.csv
.venv/bin/python review_request_2026-08-26/lag_correlation.py   logs/servo_log_20260826_073816.csv left
.venv/bin/python review_request_2026-08-26/impulse_magnitude.py logs/servo_log_20260826_073816.csv left
```

- `qsync_health.py` — phase 분포, fill 히스토그램, trim/적분항, 이벤트 카운터 (§5 근거)
- `drop_rate.py` — Δrback_seq 기반 실제 송신률 vs 루프 생성률 (§3.2)
- `lag_correlation.py` — 드롭 틱 대비 lag 0~14 의 2배-스텝 배율 + 대조군 (§3.3)
- `impulse_magnitude.py` — lag-8 확정 이벤트의 속도/가속 임펄스 분포 (§3.4)

주의: `drop_rate.py` 안의 "send_period" 계산에 `send_start_ns` 를 쓰지 마세요 (§2 함정1).
