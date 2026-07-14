# flow-infer pgmode controller-simulation 파라미터 검증

**날짜:** 2026-07-13  
**대상:** 실제 RB3-730 제어박스의 rbpodo pgmode simulation + 녹화 RGB-D replay  
**실물 동작:** 없음 (`physical_motion_expected=false`)  
**실제 카메라/그리퍼:** 사용하지 않음

> 최종 경로는 `make run MODE=sim`에서 추적되는 `stack_sim.yaml`만 사용한다.

## 결론

오프라인 서치와 실제 제어박스 pgmode simulation은 같은 방향을 가리켰다.

- `velproprio=camera_frame`, `max_linear_velocity=0.45 m/s`,
  `boundary + crossfade 2`는 그대로 유지할 근거가 있다.
- 가장 큰 런타임 knob는 `CHUNK_EXECUTE_STEPS`였다. teacher-forced 오프라인
  검증에서 W6의 6-step drift는 **5.02 mm**로 W12 **11.09 mm**의 절반 이하였고,
  pgmode에서도 W6는 90초/7,879 packet 동안 fault 없이 통과했다.
- 같은 controller state가 누적된 뒤 수행한 W12 비교 trial은
  `delta_preview_actual_lead_fault`를 재현했다. W6에 유리한 방향성은
  일치하지만, 이 W6/W12 한 쌍은 시작 reference가 완전히 동일한 통제 실험은
  아니므로 절대 성능 비교가 아니라 controller gate의 구분력 증거로 해석한다.
- controller pgmode는 물리 encoder의 `q_actual`을 움직이지 않으므로, 폐루프
  상태는 controller reference(`q_target`, `tcp_ref_stand`)를 사용해야 한다.
  이 경로를 server와 policy 양쪽에서 일관되게 만든 뒤 녹화 이미지로 반복
  rollout이 가능해졌다.
- 표현 왕복은 무손실이고 runtime이 예측을 정상 소비하지만, W6에서도 모델의
  teacher-forced BC 오차가 남는다. task success의 주된 잔여 gap은 여전히
  모델/데이터 쪽이라는 이전 결론과 일치한다.
- 최종 physical-box `fault_latch` 구성에서 **3,720.003초**, W6 rollout
  **45/45 PASS**, **327,383 commands**, **10,033 inferences**, command drop 0,
  server physical-motion detection 0, fault 0으로 장기 gate를 통과했다.

## 1. 먼저 확인한 히스토리와 데이터 경로

최근 문서·git history·실행 프로세스·scratch artifact를 교차 확인했다.

- 학습 원본은 스토리지 서버 `plaif@192.168.8.50`의
  `/data/pika/bolt/data` 752 episode와 `/data/pika/bolt/data_right` 178 episode,
  합계 **930 episode**다.
- 이 workspace에 NFS/sshfs mount는 남아 있지 않으며 `sshfs`도 설치되어 있지
  않다. 이전 분석은 storage server에서 원격 처리한 뒤 pose NPZ와 일부 HDF5만
  로컬 scratch로 복사한 방식이었다.
- 이번 controller 시험에는 로컬에 이미 있던 held-out HDF5
  `episode_008.hdf5`(67 frames)를 사용했다. 실 camera server `:5600`은 건드리지
  않고 replay publisher `:5700`을 사용했다.
- OpenPI `:8000`은 기존 배포 checkpoint
  `pi05_pika_umi_video_tcp_gripabs_velproprio_depth_z50_h24`를 그대로 사용했다.
- `172.28.60.200/.201:5000`은 VM/DNAT가 아니라 실제 양팔 control box이며,
  두 box 모두 pgmode `operation_mode=simulation`을 확인한 뒤에만 명령을 보냈다.

## 2. 최종 실행 경로

Server는 표준 실행만 사용한다.

```bash
ACTION_SOURCE=none GRIPPER_SERVER=0 SCOPE_DASHBOARD=0 make run MODE=sim
```

오프라인 RGB-D는 물리 camera port와 격리한다.

```bash
python3 scripts/offline_camera_replay.py \
  --episode /path/to/episode_008.hdf5 --port 5700
```

명령 전에는 독립 telemetry gate를 통과시킨다.

```bash
python3 scripts/controller_sim_state_monitor.py --port 50356 --gate 100
```

Flow rollout은 simulation 전용 policy config를 사용한다. 이 config는 real arm과
real gripper authority를 모두 끄며, server config를 대체하지 않는다.

```bash
OPENPI_REMOTE_SKIP_WARMUP=1 make flow-infer-sim-offline \
  FLOW_INFER_ARGS='--proprio-mode velocity \
    --depth-z-near-mm 50 --depth-z-far-mm 700 --depth-units-m 1e-4 \
    --chunk-execute-steps 6 --translation-only --max-linear-velocity-m-s 0.005'
```

## 3. 발견하고 수정한 pgmode 결함

### 3.1 `stack_sim.yaml`과 real profile 불일치

기존 tracked sim stack에는 `flow_infer_smooth`가 없어 flow command가
`unknown_tcp_target_profile`로 거부됐다. `stack_real.yaml`의 세 command-facing
profile(`spacemouse_precise`, `umi_large_smooth`, `flow_infer_smooth`)을
`stack_sim.yaml`에도 동일하게 두었다. 단 authority는 sim에만 좁혔다.

```yaml
cartesian_control:
  allow_in_real: false
  allow_in_controller_simulation: true
```

### 3.2 one-shot pgmode startup의 stale frame

`make run MODE=sim`은 양팔을 simulation/servo-on으로 전환한 뒤에도 transition
전에 queue된 `servo_disabled` state 하나를 마지막 startup verdict로 읽어 간헐적으로
실패했다. backend는 operation mode와 activation을 이미 확인한 경우에 한해 최대
5초 동안 fresh state를 bounded refresh한다. 일치하는 state가 없으면 기존처럼
startup을 fail closed한다.

### 3.3 pgmode Cartesian state source

물리 실제 모드의 Cartesian loop는 measured `q_actual/tcp_actual`이 맞지만,
controller simulation은 물리축을 움직이지 않고 controller reference만 갱신한다.
따라서 pgmode에서만 다음을 tracking state로 선택한다.

```text
q_target -> FK -> tcp_ref_stand -> IK seed / follower feedback / policy proprio
```

reference가 없거나 invalid이면 guessed actual/zero pose로 fallback하지 않고 명령을
거부한다. physical-real 경로의 measured-state 선택은 바꾸지 않았다.

### 3.4 tick-start send history의 1 tick 추가 지연

`send_at_tick_start=true`인 pgmode에서 방금 성공적으로 보낸 target이 현재 tick의
safety acceleration history에 즉시 반영되지 않았다. 결과적으로 two-send-old
target을 기준으로 clamp가 반복되어 `q_sent`가 교대로 커지는 recurrence가 생겼다.
controller-simulation에 한해 tick-start send 성공 직후 bookkeeping을 갱신했다.
physical-real send timing/bookkeeping은 변경하지 않았다.

### 3.5 physical control-box motion guard

기존 sim stack은 Virtual ControlBox 호환을 위해 physical-motion detection을
`warn_only`로 두고 있었다. 이번 topology는 실제 control box이므로 encoder motion은
정상 simulation이 아니라 mode/safety 위반이다. 최종 tracked config는
`controller_simulation_physical_motion_policy: fault_latch`로 바꿨다. 따라서 외부
monitor가 보고하기 전에 server 자체가 즉시 명령을 억제한다. `q_actual`이 실제로
움직이는 Virtual ControlBox는 향후 명시적 topology contract 없이는 이 physical-box
profile을 사용할 수 없다.

### 3.6 real stack과 같은 값 / 의도적으로 다른 값

| 항목 | sim vs real |
|---|---|
| backend/IP/run mode | 동일: `rbpodo`, `.200/.201`, `run_mode: real` |
| Servo J | 동일: 500 Hz, `t1=0.002`, `t2=0.021`, gain 1.0 |
| named TCP profiles | 세 profile의 전체 mapping 동일 |
| operation mode | simulation vs real |
| `servo_alpha` | sim 10.0(transparent pgmode), real 1.0(physical jerk 억제) |
| Cartesian authority | sim은 controller-simulation만, real은 physical-real만 |
| tracking state | sim reference, real measured actual |
| physical-motion guard | sim physical-box profile은 fault latch |

`servo_alpha=1.0`도 실제 box pgmode에서 시험했지만 activation이 fault해 채택하지
않았다. “real과 최대한 동일”은 검증 없이 이 차이를 지우는 것이 아니라 command
contract를 같게 하고 controller/plant 의미가 다른 값만 명시적으로 남기는 것으로
적용했다.

## 4. 오프라인 파라미터 재검증

현재 OpenPI server에 8 episode / 170 anchor를 다시 통과시켰다.

### 4.1 모델 입력

| velproprio | normalized MSE | 12-step drift mean |
|---|---:|---:|
| **single-step / camera-frame** | **0.750** | **11.1 mm** |
| window-12 average | 0.773 | 12.0 mm |
| zero | 0.797 | 13.6 mm |

학습 방식과 같은 camera-frame single-step이 다시 최상위였다.

### 4.2 실행 window

| execute window | drift mean | drift p95 |
|---:|---:|---:|
| **6** | **5.02 mm** | **11.51 mm** |
| 8 | 7.90 mm | — |
| 12 | 11.09 mm | 26.27 mm |
| 16 | 15.11 mm | — |
| 24 | 20.93 mm | — |

W6는 W12보다 inference 호출 빈도가 높지만 open-loop 누적 오차가 절반 이하라
controller 시험의 우선 후보로 선택했다.

### 4.3 stitch와 clamp

| stitch | boundary jump mean | p95 |
|---|---:|---:|
| **crossfade 2** | **0.318 mm** | **0.715 mm** |
| ensemble | 0.577 mm | — |
| raw boundary | 0.953 mm | — |

`0.15 m/s` clamp는 step 약 11%를 잘랐고 `0.45 m/s`는 학습 분포의 정상 motion을
보존했다. ee_local encode/decode 자체는 다시 수치적으로 무손실이었다. W12의
모델 BC error(mean 11.4 mm, p95 26.3 mm)는 clamp/stitch 오차보다 크다.

## 5. 실제 control box pgmode 결과

| trial | controller telemetry | 물리 TCP 변위 | verdict |
|---|---|---:|---|
| W6, 60 s rollout + 90 s monitor | 7,879 packets, `Ok` 7,879, fault 0 | L/R 0.0 m | **PASS** |
| W12 strict-lead comparison | 7,323 packets, `Ok` 7,014, `IkFailed` 308, fault 1 | L/R 0.0 m | `actual_lead=6.446 mm`에서 fail closed |
| simulated reference reset | 4,029 packets, `Ok` 4,029, fault 0 | L/R 0.0 m | InitMotion으로 reference만 안전 복귀 |

W12 fault는 `preview_max_actual_lead_m=0.006`를 3회 넘겨 발생했고 물리축은
움직이지 않았다. 비교 순서상 W12가 W6 이후 reference에서 시작했다는 confound가
있으므로 “W12는 언제나 실패”로 일반화하지 않는다. 반면 W6 장기 시험은 매 trial
사이에 지원되는 InitMotion으로 동일 reference target에 복귀시켜 이 confound를
줄인다.

## 6. 장기 시험과 최종 validator

최종 구성(`physical_motion_policy=fault_latch`)에서 W6 60초 rollout 45회,
trial 사이 simulation reference reset 20초, 독립 monitor 3,720초를 실행했다.
중간의 `warn_only` 구성으로 끝낸 11개 trial은 안전 감사에서 중단했고 최종
acceptance 수에 포함하지 않는다. stop condition은 다음 전부다.

- 양팔 `operation_mode=simulation` 유지
- backend startup raw field가 양팔 `controller_mode=simulation`으로 decode
- `physical_motion_expected=false` 및 physical-motion detection 0
- server-owned physical-motion detection 0; FK 진단 drift는 별도 기록
- fault latch 0
- 45 rollout 모두 command > 0, command drop 0, real-motion authority false
- real-gripper authority false, 생성된 gripper proposal 전량 suppression
- 모든 trial log가 W6, translation-only, 5 mm/s bounded search profile을 증명
- tracked `stack_sim.yaml`의 세 named TCP profile 전체가 real과 동일

최종 결과는 **21/21 checks PASS**였다.

| 항목 | 결과 |
|---|---:|
| 독립 monitor | 3,720.003 s / 323,641 packets |
| W6 rollout | 45/45 PASS |
| `TcpPoseTarget` command | 327,383 |
| OpenPI inference | 10,033 |
| command drop | 0 |
| gripper proposal / suppression | 119,820 / 119,820 |
| server physical-motion detection | 0 |
| fault-latched packets | 0 |
| max measured FK drift L/R | 1.342 µm / 0.0 µm |
| max simulated reference displacement L/R | 87.07 mm / 114.31 mm |

왼팔 1.342 µm 값은 `q_actual` FK의 수치/encoder 미세 변동 진단값이다. 물리-motion
판정은 임의 TCP 상수가 아니라 tracked config의 권위 있는 0.05° joint 임계값과
server telemetry가 소유하며, 323,641 packet 전체에서 detection은 0이었다. 검증기는
이 진단값을 보존하면서 `fault_latch`와 server detection 0을 안전 gate로 사용한다.

최종 machine-readable 판정은 다음 파일에 남겼다.

```text
outputs/flow_infer_pgmode_sim_20260713/long_w6_fault_latch/controller_monitor.json
outputs/flow_infer_pgmode_sim_20260713/long_w6_fault_latch/trials.tsv
outputs/flow_infer_pgmode_sim_20260713/long_w6_fault_latch/server.log
outputs/flow_infer_pgmode_sim_20260713/long_w6_fault_latch/validation.json
```

## 7. 해석과 적용 경계

W6는 pgmode에서 controller reference tracking과 inference throughput을 장시간
검증하는 simulation 기본값으로 채택했다(`make flow-infer-sim-offline`에만 적용).
다만 실제 카메라가 로봇 motion에
반응하지 않는 open-loop replay이므로 task success, 접촉, physical servo tracking,
실제 영상 피드백을 검증하지 않는다. 따라서 이 결과만으로 physical-real 기본
W12를 W6로 바꾸거나 실제 motion 권한을 넓히지 않는다. physical-real 승격은 별도
operator-supervised acceptance에서 판단해야 한다.

Force motion, physical Cartesian motion, 실제 camera/gripper는 이번 시험 범위 밖이며
활성화하지 않았다.
