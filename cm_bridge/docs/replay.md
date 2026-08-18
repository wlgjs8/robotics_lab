# cm_replay — 2 ms 3D 리플레이 (chunk 입력 · follow 목표/참조 · 명령 · 실측 · 그리퍼)

**목적.** pi0.5 롤아웃에서 "chunk follow의 궤적 수렴 문제인지, 그리퍼 타이밍 문제인지"를
2 ms 단위로 되돌려 보며 분리한다. 한 타임라인 위에 다섯 가지를 함께 올린다:

| 층 | 무엇 | 출처 |
|---|---|---|
| **입력 chunk** (노란 폴리라인) | 정책이 보낸 청크의 절대 row들, 지금 재생 중인 row 표시. 흐리게 = 수신됐지만 아직 채택 안 된 최신 청크(REPLACE 지연) | 브릿지 사이드카 (`logs/cm_bridge_sidecar_*.jsonl`) |
| **follow cmd** (마젠타 점/궤적) | FollowUnit의 델타 적분 체인 = 정책이 *요청한* 포즈 | 캡처 `fol_cmd_*` |
| **follow ref** (주황) | 두 Ruckig 채널이 만든 IK 목표 = *추종 중인 trajectory* | 캡처 `fol_ref_*` |
| **command** (파란 고스트 로봇 + 궤적) | 컨트롤러가 실제 송신한 관절 명령(`cmd`)과 그 FK(`tcp_cmd`), 그리퍼 **명령** % | 캡처 `cmd*`, `tcp_cmd_*`, `ext0` |
| **actual** (실체 로봇 + 초록 궤적) | 측정 관절(`jnt_ang`)과 FK(`tcp_act`), 그리퍼 **피드백** % | 캡처 `jnt_ang*`, `tcp_act_*`, `ext1` |

사이드 패널에는 커서 시점의 수치(lag/gap/추종오차, 게이트, 편차, |F|, 그리퍼 cmd/fb와 각각의
나이(ms), 청크 seq/idx/n)가, 아래에는 팔별 plotly 그래프(추종오차·lag·|F|, 그리퍼 cmd vs fb)가
커서선과 함께 뜬다. 조작: play/pause, 속도, **2 ms 스텝**, 청크 채택 경계/그리퍼 명령 변화 지점으로
점프.

## 데이터가 어디서 오나 — 한 클럭

controller-manager의 `func write` 캡처(`DataRecorder`, RT-safe 링 → writer 스레드, `.bin`)는
팔마다 2 ms 한 행이다. 이번에 **schema 4**로 확장했다(`cm_bridge/upstream/0001-*.patch`, 업스트림
PR 대기):

- `mono_ns` — 그 틱의 **CLOCK_MONOTONIC**. 브릿지 사이드카의 모든 레코드도 `time.monotonic_ns()`
  (= 같은 클럭)로 찍히므로 두 로그는 시간 비교만으로 합쳐진다.
- `fol_*` — FollowUnit 텔레메트리: cmd/ref 포즈(+rpy), 이번 주기 델타 속도, lag/gap, 게이트,
  **채택 중인 청크의 stamp**(`fol_chunk_stamp_ns`)/idx/n, playing.
- `dev_*` — 어드미턴스 오버레이 편차, `overlay_bounded`.
- `ext0..7`, `ext_stamp_ns`, `ext_seq` — **외부 프로세스 값**. 새 토픽
  `/monkey/<side>/cmd/ext_scalars`(sensor_msgs/JointState)로 브릿지가 넣는다:
  `[grip_cmd_pct, grip_fb_pct, grip_cmd_mono_ns, grip_fb_mono_ns, chunk_seq, chunk_pub_mono_ns, 0, 0]`.
  즉 **pi0.5 그리퍼 명령값과 피드백이 컨트롤러의 2 ms 행에 실린다** (통신·추종 지연은 각각의
  mono_ns 스탬프로 그대로 드러난다 — 감춰지지 않는다).

브릿지는 `cmd/follow` PoseArray의 `header.stamp`에 자기 monotonic ns를 넣고, 컨트롤러는 그 값을
`FollowChunk::stamp_ns`로 들고 다니다가 재생 중인 청크의 것을 `fol_chunk_stamp_ns`에 적는다.
사이드카의 `chunk` 레코드에는 같은 수가 `pub_mono_ns`로 있고 청크의 **전체 내용**(절대 row, 델타,
per-row grip, execute/runway steps)이 실려 있다 — 이 쌍이 "어느 청크가 재생 중인가 ↔ 그 청크가
무엇이었나"의 조인 키다. 게이트(`cm_bridge/tests/record_gate.sh`)가 이 조인을 매번 검증한다
(29/29 채택 청크 일치, 그리퍼 스탬프 1 ms 이내).

## 사용

```bash
# 0. 전제: 스택이 이 빌드로 떠 있을 것 (컨트롤러 = schema 4 바이너리, 브릿지 = 사이드카 빌드).
#    run_cm_stack.sh 가 바이너리의 schema-4 마커를 확인하고 아니면 거부한다.

# 1. 녹화 (컨트롤러 콘솔 `func write start/stop`, 양팔)
./cm_bridge/tools/cm_record.sh start
#    ... pi0.5 롤아웃 ...
./cm_bridge/tools/cm_record.sh stop          # 최신 캡처 쌍 + 사이드카 + 리플레이 커맨드를 찍어준다

# 2. 리플레이 (viser, http://127.0.0.1:8082)
.venv/bin/python cm_bridge/tools/cm_replay.py \
    --bin ~/.local/state/plaif-chimpanzee/logs/data_left_<stamp>.bin \
          ~/.local/state/plaif-chimpanzee/logs/data_right_<stamp>.bin \
    --sidecar logs/cm_bridge_sidecar_<stamp>.jsonl
#    또는 한 디렉터리에 다 있으면:  --capture-dir <dir>   (최신 쌍 + 사이드카 자동)
#    --start-t 12.3   시작 커서   |   --port 8082
```

cockpit 콘솔에서 `func write start`를 쳐도 같다(cockpit은 컨트롤러 콘솔의 프런트엔드). 캡처는
컨트롤러의 로그 디렉터리(`CONTROL_MANAGER_LOG_DIR` 또는 `~/.local/state/plaif-chimpanzee/logs`)에
`data_left_<ts>.bin` / `data_right_<ts>.bin`으로, 사이드카는 브릿지 프로세스 단위로
`logs/cm_bridge_sidecar_<ts>.jsonl`에 계속 쌓인다(청크 7 Hz × ~2 KB, 90 s에 ~1.3 MB).

## 프레임과 단위

- 3D는 컨트롤러 `base_frame`(= follow.yaml `telemetry_frame: root`)이다. 캡처의 `tcp_*`/`fol_*`은
  root mm, 사이드카 청크 row는 flow-infer가 낸 절대 row(m; 브릿지 state fanout이 컨트롤러
  `cmd/pose`를 그대로 재발행하므로 같은 프레임). 팔 URDF는 디바이스 파일의 `transform`으로
  마운트되므로 로그의 관절 FK가 로그의 TCP 위에 떨어진다.
- 그리퍼 애니메이션: rb_gui의 articulated pika URDF(`finger_*_joint`, 0 % = 닫힘 … 100 % = 열림 →
  `_finger_position_m`). 실체 = 피드백(`ext1`, 없으면 사이드카 `grip_fb`), 고스트 = 명령(`ext0`).
- schema 4 이전 캡처도 열린다(cmd/act/ref만; follow/청크/그리퍼 층은 그 파일에 없다).

## 읽는 법 — 두 가설을 가르는 신호

- **궤적 수렴 문제**라면: 노란 청크(입력) → 마젠타 cmd(요청) → 주황 ref(추종) → 파랑 tcp_cmd(송신) →
  초록 act(실측) 사이 간격이 커진다. `lag(cmd→ref)`는 설계상 제동거리 지연이라 0이 아니지만,
  `gap(cmd→emitted)`와 `|cmd−act|`가 청크 경계에서 튀거나 게이트(`gate t/r`)가 1 아래로 내려가
  있으면 컨트롤러 쪽이다. 청크 경계마다 "chunk ▶"로 점프해서 REPLACE 순간의 흐린 청크(수신)와
  노란 청크(채택) 사이 시간차도 본다.
- **그리퍼 타이밍 문제**라면: 그래프에서 `grip cmd`(계단)와 `grip fb`의 시간차, 그리고 커서 시점의
  "cmd N % (x ms ago) · fb M % (y ms ago)"로 정량화된다. 청크의 per-row grip 값(사이드카)과
  실제 dispatch된 `grip_cmd`의 차이는 flow-infer의 그리퍼 dispatch 정책(absolute, close-bias)
  자체다.

## commit + 시간 늘리기 구조(2026-08-19)에서 사이드카가 더 담는 것

브릿지는 envelope(`max_vel·T`, `max_rot·T`)를 넘는 정책 스텝을 **서브델타로 나눠** 보내므로(컷 없음),
컨트롤러의 delta idx는 서브델타 번호다. `follow_pub.step_of_sub[idx]`가 정책 row 번호이고, 그리퍼는
그 스텝의 **마지막 서브델타가 끝난 이벤트**(prev_aux0 유한)에서만 나간다. 리플레이는 이 매핑으로 노란
청크의 진행 row를 표시한다.

- `follow_pub` — 브릿지가 컨트롤러에 실제로 발행한 슬라이스: `seq`(러너 청크), `act`(러너 activation
  step), `abs0`(이 메시지 델타 0의 절대 스텝), `skip`, `n`, `reason`(start / boundary / window-refresh),
  `pub_mono_ns`(= 캡처 `fol_chunk_stamp_ns`), 델타별 grip.
- `follow_step` — 컨트롤러의 주기 경계 이벤트: `kind`(0 굶음 / 1 델타 시작 / 2 시작+채택 / 3 종료),
  `stamp`/`idx`/`n`, 방금 **끝난** 델타 `prev_stamp`/`prev_idx`/`prev_aux0`(= 그 스텝에 실린 그리퍼 목표).
- `grip_cmd`는 이제 이 이벤트에서 나간 명령이고, 러너의 command 채널 그리퍼는 `grip_cmd_runner_ignored`로만 남는다.
리플레이는 노란 청크를 `skip`부터 그리고 패널에 `slice skip/abs0`를 표시한다.

## 관측 덤프 + 오프라인 재추론 (chunk를 만든 "그 순간"까지 저장)

리플레이가 컨트롤러·브릿지 쪽을 덮는다면, 이쪽은 **정책 쪽**이다: 각 추론이 실제로 본 것과 답한
것을 그대로 저장한다 (`policy_runner/policy_runner/observation_dump.py`, `--observation-dump-dir`).

- 추론마다: 손목 이미지 2장 **무손실 PNG**(전송된 배열과 바이트 단위로 동일 — 재추론 시 차이의
  원인을 프로프리오 하나로 한정하기 위해 손실 압축은 쓰지 않는다), `observation/state`, prompt,
  RTC 입력(`prev_action_chunk`, delay/horizon/schedule), 서버가 돌려준 `actions`/`rtc_raw_actions`,
  러너가 실제로 쓴 스케일된 청크, velproprio 진단, 카메라 bundle seq, 그 순간의 state payload
  포즈들, `request/ready_mono_ns`(같은 CLOCK_MONOTONIC).
- 조인: `inference_seq` = chunk overlay 패킷의 `chunk_metadata.inference_seq` = 사이드카 `chunk`
  레코드(이제 `chunk_metadata`/`inference_timing`을 실음) → `pub_mono_ns` → 캡처의
  `fol_chunk_stamp_ns`. 즉 리플레이의 노란 청크 하나에서 그 청크를 만든 관측(영상)까지 역추적된다.
- `tools/flow_infer_sweep_run.sh`가 기본으로 `outputs/obs_dump/<stamp>_<tag>/`에 켠다
  (`FLOW_INFER_OBS_DUMP=0`으로 끔). 실제 카메라 이미지 PNG는 장당 ~300 KB → 7.5 Hz에서 90 s에
  수백 MB. 인코딩은 백그라운드 스레드(추론 스레드 비용 ~0.4 ms).

**리플레이에서 보기** — `cm_replay.py --obs-dump <dir>`: 커서 시점 채택 청크의 원본 손목 영상 2장, 그 추론의
state·velproprio·지연(request→ready, 커서 기준 영상 나이, ready→청크 수신), 그리고 **4단계 체인 요약**
(① 모델 raw 청크 → ② 러너 row = ③ 브릿지 델타 → ④ 컨트롤러 소비)이 패널에 뜬다. 3D에는 모델 raw 청크를
사이드카 row 0 앵커에서 러너의 `pose_compose_local`로 합성한 흰 폴리라인이 노란(러너 row) 청크 옆에 그려져,
①↔② 차이(RTC 정렬·band-limit·앵커 적분)가 눈에 보인다. `--variants reinfer.json`을 주면 재추론 변형 청크가
같은 앵커·같은 합성으로 색깔별 오버레이된다 — 즉 **영상에서 다시 뽑은 청크와 컨트롤러가 재생한 청크가 같은
시점, 같은 앵커 위에 놓인다.** 조인은 추정이 아니라 키다: `inference_seq`(청크 패킷 metadata에 실림; 없으면
"청크 수신 직전 ready된 추론"으로 시간 폴백) → 사이드카 → `pub_mono_ns` → 캡처.

**재추론** — `cm_bridge/tools/reinfer.py` (openpi venv 파이썬으로 실행):
```bash
/home/plaif/workspace/openpi/.venv/bin/python cm_bridge/tools/reinfer.py \
    --dump outputs/obs_dump/<stamp>_<tag> --server openpi://127.0.0.1:8001 \
    --seq 120 --repeat 2 --samples 8 --vel-scale 0.5,0.75,1.0,1.25,1.5 --csv out.csv
```
같은 관측을 그대로 다시 넣고(노이즈 바닥), 그 다음 `observation/state`의 **속도 차원만** 배율을
바꿔(`--arm`, `--part`로 팔/병진·회전 선택, `--set i=v`로 임의 항 덮어쓰기) 청크가 어떻게 달라지는지
팔별 dT[mm]·dR[deg]·dG[%]·방향 코사인·평균 |δ|로 표를 낸다.

**꼭 알고 볼 것 — 서빙되는 정책은 확률적이다.** 같은 입력을 두 번 넣어도 스텝당 dT 최대 ~1–2 mm,
코사인 0.7–0.96 수준으로 다르다(flow matching의 노이즈 초기화). 한 번씩만 비교하면 속도 변화 효과가
노이즈에 묻힌다. `--samples N`으로 변형마다 N번 평균해서 **변형 평균 ↔ 무변형 평균**을 비교하면
효과가 드러난다(합성 입력으로 확인: 우팔 평균 |δ| 1.55 mm → ×0.5에서 1.20, ×2에서 2.20 — 모델이
속도 프로프리오에 비례해 출력 속도를 키우는 것이 오프라인에서 재현됨; 이것이 fidelity 리포트의
"복리 루프"의 실물이다).

## 검증

- `cm_bridge/tests/record_gate.sh` — 라이브 스택 옆에서 **격리** 실행되는 레코더 게이트 (ROS 도메인 77,
  대체 UDP 포트, 죽은 그리퍼 엔드포인트, `noaffinity.c` LD_PRELOAD 심으로 cpu1/2 접근 차단,
  `taskset` 12-15). 2026-08-19 PASS: schema 4, 2.000 ms 케이던스, 29/29 청크 조인, 그리퍼 레벨·스탬프
  조인.
- `cm_replay.py --headless-check` — 로드·렌더·모든 GUI 콜백 경로를 브라우저 없이 한 번 돈다. 레코더
  게이트가 합성 관측 덤프까지 만들어 `[OBS] 29/29 inference_seq 조인` + 이 헤드리스 체크를 마지막 단계로 돈다.
