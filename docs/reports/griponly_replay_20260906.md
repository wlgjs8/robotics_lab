# griponly 8003: InitMotion 수정 및 delta_preview 오프라인 분석

분석일: 2026-09-06. 사용자 기준 모델은 `boltv2_40k`; 이번에 재생한 것은
`boltv2_griponly_40k`의 단일 rollout이다. 모델 간 우열이나 학습 원인을 판정하지 않는다.

## 결과

1. InitMotion 재전송에 같은 논리 요청 ID를 쓰고, 새로운 start/retry에 새 ID를
   쓰도록 수정했다. Done 이후에는 Hold로 tare를 기다린다. 양수 tare 대기시간
   만료는 차단 상태를 표시하며, 유효한 영점 없이 자동 재개하지 않는다.
2. 왼팔 +84.5–87.5초는 회전 제한보다 **translation 제약과 force advance gate의
   상호작용**을 우선 조사할 근거가 강하다. 목표 가속도를 0으로 만들거나 회전
   가속도/jerk 제한만 완화한 재생은 거의 개선되지 않았다. 실기 제한을 높일
   근거로 해석해서는 안 된다.
3. 동일한 재생 상태에서 시작한 delta overlap은 +129초/+39초의 목표 속도 변화와
   output-SMD 진동 성분을 줄였다. 그러나 경로 차이가 약 19/38 mm(p95) 생겼다.
   +85초에는 실질적인 개선이 없다. 실기 기본 경로에 overlap을 적용하지 않았다.

## 입력 및 재생 범위

- Servo: `logs/servo_log_20260906_114336.csv`.
- Chunks: `outputs/sweep/20260906_114429_boltv2_griponly_40k.chunks.jsonl`.
- 모델: `pi05_pika_umi_boltv2_anchAB_griponly_h24_40k`, checkpoint 39999, 포트 8003.
- H24, execute 4, runway 4, policy dt 0.0334 s, boundary, RTC off, crossfade 0.
- 상대시간 원점: 첫 chunk, monotonic 1569382.485903465,
  2026-09-06 11:44:30.336 KST.
- 재생: 처음부터 +132초, 팔마다 입력 66,000 ticks / 활성 65,933 ticks /
  984개 수신 frame. 각 문제 구간만 임의의 초기 속도에서 시작하지 않았다.
- 산출물: `outputs/griponly_replay_20260906/`. 원본 판독은
  `outputs/griponly_diagnosis_20260906/report.md`.

`chunk_frame_wire_seq`를 실제 수신 tick과 연결하여 기록된 `left_delta` /
`right_delta`를 C++ `CartesianChunkFollower::submitDeltaFrame`에 넣었다.
`filter_dt_ms`, 이전 tick의 force gate와 보상된 **필터 전 stand wrench**,
plan-rate gate를 사용한다. 활성화 reference는 이전 전송 FK에서 standing force
deviation을 제거한 nominal pose다. 실제 `delta_preview`와 같은
Ruckig → `FollowerOutputSmd` 경로를 실행한다.

Python absolute overlay나 FOH만 바꾸어 효과를 추정한 결과가 아니다. 반면 IK,
전송 safety/filter, 모터, 실측 응답, 바뀐 궤적에 따른 접촉력은 재시뮬레이션하지
않았다. 아래 평활도는 **output-SMD 뒤 Cartesian reference**의 수치다.
실제 전송 관절/실측 개선은 아직 검증하지 않았다. 기록된 힘을 고정한 가상
비교이므로 경로가 크게 바뀌는 실험의 물리적 해석에는 특히 한계가 있다.

### Baseline 재현 정확도

| 구간 | Ruckig 위치 오차 p99 | segment duration 오차 p99 / 최대 |
|---|---:|---:|
| 왼팔 +84.5–87.5 | 0.658 mm | 0.539 / 0.780 ms |
| 왼팔 +128.5–131.5 | 1.756 mm | 0.047 / 10.099 ms |
| 오른팔 +38–41 | 0.002 mm | <0.001 / 4.873 ms |

완전한 tick 단위 동일 재현은 아니다. 기존 CSV의 정밀도, 실행 feedback 및
plan gate 복원 한계가 남는다. 왼팔 접촉 구간의 완료 가능 비율 16.7%, duration/dt
p90 2.417, reference RMS 1.344 mm는 원본 판독의 16.7%, 약 2.42,
전송 명령 RMS 약 1.34 mm와 가깝다. 이 일치가 IK/실측 모델 검증을 대신하지 않는다.

## 1. InitMotion, tare, 재개

### 수정 동작

- 각 arm의 `init_motion_request_id`를 재전송 동안 유지한다. 같은 목표라도
  새 start/retry는 새 ID다. 서버는 ID가 같은 완료 요청을 새 계획/tare로 재시작하지 않는다.
- 새 ID에 이전 Done acknowledgment가 뒤늦게 도착하면 무시한다.
- Done을 클라이언트에 latch하고 Hold를 보낸다. 서버 exec가 Idle로 돌아가도
  이미 시작한 tare settling은 유지되어, stillness 확인과 250개 sample 수집을 마친다.
- F/T enabled + connected + bias valid + auto/sample settling 종료를 기다린다.
  enabled를 확인한 뒤 telemetry가 빠지거나 연결이 없으면 계속 기다린다.
- 기본 `ft_tare_wait_sec=2.0`은 경고/차단 표시 시점이다. 만료 시
  `init tare blocked` + Hold. 이후 정상 tare가 오면 해제한다.
- tare 도중 Failed가 도착하면 failure latch가 우선하며, 나중에 bias가 유효해져도
  완료로 자동 승격하지 않는다.

기존 명시적 예외의 의미도 문서화했다. `ft_tare_wait_sec=0`은 대기 opt-out이며
로그에 남는다. F/T disabled, F/T가 한 번도 등장하지 않는 legacy/mock state는
클라이언트 tare 대기가 없다. 수동 cancel, 설정된 resume-on-failure, override를
거치지 않은 외부 InitMotion의 동작은 유지된다. 어느 경우도 bias를 만들어주지
않으며, 서버는 bias 없이 force coverage를 제공하지 않는다. ID를 생략한 legacy
클라이언트는 기존 packet seq/목표 비교를 사용한다. ID deduplication은 현재 exec
범위이며 cancel/프로세스 재시작을 넘는 영구 transaction 보장은 아니다.

### 재개 상태 전환 점검

원본은 InitMotion +163.77초부터 bias가 사라졌고 +169.348초 policy 재개,
+169.482초 follower 재진입 이후에도 약 42초간 bias가 복구되지 않았다.
Done 대기 중 tare settling 재진입은 19번, 최장 약 66 ms여서 500 ms 조건을
채우지 못했다. 이번 수정은 반복 재시작과 timeout 자동 재개를 각각 막는다.

생산 코드에서 InitMotion의 joint mode 진입은 streaming follower를 중단하고,
output SMD 재진입은 이전 sent FK reference와 follower velocity로 초기화한다.
joint mode를 지난 pinned-IK low-pass의 유효 상태/engagement도 초기화한다.
기존 follower hold/resume/reanchor 및 output SMD 단위검증을 다시 실행했다.
추가 `follower_output_smd_reseeded`, request ID, 기존 mode/IK/전송 관절 로그로
다음 재개에서 상태 전환을 직접 연결할 수 있다. 실기 재개 과도응답이 해결됐다는
결론은 아직 아니다. 원본 왼팔 약 6001.3 deg/s²는 검사 기준 6000보다 1.3만 높으므로
그 작은 문턱 초과 자체를 큰 사건으로 해석하지 않는다.

## 2. 왼팔 +84.5–87.5초 제약 분리

각 실험은 같은 delta, 기록된 dt/힘을 사용한다. 제한/guard 비교는 +0초부터
적용했으므로 변경된 이전 경로/자세 이력의 영향이 포함된다. `no_force_gate`는
+83.5초에 켠 동일 사전 상태 재생에서도 아래 결과가 동일했다.

| 오프라인 조건 | duration ≤ dt 비율 | duration/dt p90 | 3–15 Hz RMS | 국소 경로 차이 p95 |
|---|---:|---:|---:|---:|
| 현재 baseline | 16.7% | 2.417 | 1.344 mm | 0 |
| linear accel/jerk ×4 | 93.3% | 1.000 | 0.635 mm | 69.4 mm |
| angular accel/jerk ×4 | 17.8% | 2.428 | 1.339 mm | 1.0 mm |
| force advance gate 제거 | 92.2% | 1.000 | 0.453 mm | 252.3 mm |
| 목표 acceleration damping=0, 양쪽 | 16.7% | 2.441 | 1.346 mm | 1.4 mm |
| corner velocity brake 제거 | 16.7% | 2.425 | 1.342 mm | 1.0 mm |

linear/rotation acceleration만 각각 0으로 한 경우도 개선이 작았다. rotation-only는
94.4%지만 translation을 없애 작업 자체가 달라지고, translation-only는 21.1%로
좋지 않다. 이 둘은 분해 진단이며 task 성능 비교가 아니다.

독립 축 최소시간이 dt를 넘는 69개 segment에서 가장 긴 축은 stand z 40회,
x 15회, y 9회, rotation tangent z 3회, x 2회였다. 동기화된 전체 duration은
축별 최소시간 최댓값과 다를 수 있으므로, 이를 관절 축 제한이나 하드웨어
최대치 초과로 읽으면 안 된다.

`stepToNextSegment()`는 접촉 방향의 전진 위치를 gate로 줄인다.
`buildSample()`은 세 knot에 같은 `plan_shift`를 더하므로, 중앙차분으로 계산하는
목표 velocity/acceleration에서는 이 shift가 상쇄된다. **줄어든 이동 거리와
줄지 않은 목표 미분값의 조합**이 짧은 dt에서 불리해지는 설명과 재생 결과가
일치한다. 힘에는 동적 센서 반응도 있을 수 있어 단일 물리 원인으로 확정하지
않는다. gate 제거/제한 증가는 진단 실험일 뿐이며 실기 해결책으로 적용하지 않았다.

## 3. Chunk 교체와 continuity

현재 Ruckig segment는 이미 이전 출력의 p/v/a를 다음 시작 상태로 넘긴다.
기존 `test_chunk_follower_core`의 C² seam 검증도 통과했다. delta frame 교체 역시
chained state를 유지한다. 따라서 “위치만 이어서 속도/가속도가 무조건 reset된다”는
설명은 맞지 않는다. 새 knot의 목표 미분값과 jerk, SMD 이후의 변화량을 봐야 한다.
회전 tangent 재선형화까지 전 구간의 물리적 각가속도 C²를 보증한 것은 아니다.

가용한 이전 preview의 아직 소비하지 않은 delta와 새 delta를 2/4행 선형 또는
4행 quintic weight로 혼합한 **오프라인 대안**을 같은 C++ 소비 경로에 넣었다.
translation은 local twist 성분을 혼합하고 rotation은 SO(3) slerp 후 log를 사용한다.
그리퍼는 바꾸지 않는다. quintic weight가 끝점에서 평평하다는 것만으로 전체
이산 trajectory의 C²가 보장되지는 않는다. 별도의 endpoint p/v/a 최적화 bridge를
실기에 추가하거나 검증했다고 주장하지 않는다.

비교 상태를 맞추기 위해 baseline으로 먼저 진행한 뒤 +83.5 / +127.5 / +37초에만
변형을 켰다. 각각 `seams/left_85`, `seams/left_129`, `seams/right_39`에 저장했다.
처음부터 overlap을 켠 결과에는 누적 경로 차이가 있으므로 이 국소 비교를 우선한다.

| 구간 | baseline RMS → 4행 linear | reference accel p99 | 경로 차이 p95 / 끝점 |
|---|---:|---:|---:|
| 왼팔 +84.5–87.5 | 1.344 → 1.342 mm | 5.60 → 5.57 m/s² | 18.9 / 22.3 mm |
| 왼팔 +128.5–131.5 | 1.461 → 0.885 mm (−39%) | 4.92 → 3.06 m/s² | 18.7 / 10.9 mm |
| 오른팔 +38–41 | 0.646 → 0.521 mm (−19%) | 2.88 → 1.73 m/s² | 38.5 / 41.5 mm |

교체 시 linear 목표 velocity 변화의 최대 norm은 왼팔 +129초에서
295 → 116 mm/s, 오른팔 +39초에서 154 → 106 mm/s다. 4행 quintic RMS는
각각 +85초 1.351, +129초 0.983, +39초 0.549 mm로 linear보다 좋지 않았다.
RMS는 전체 재생에 3–15 Hz Butterworth 필터를 적용한 뒤 해당 구간의 vector RMS다.
속도/가속도는 기록된 시간 격자의 reference 유한차분이다. 경로 차이는 구간
첫 위치를 정렬한 displacement 차이이며 성공률/충돌 거리/실측 오차가 아니다.

![왼팔 129초의 동일 사전 상태 비교](/home/plaif/workspace/robotics_lab/outputs/griponly_replay_20260906/seams/left_129/comparison.png)

기존 Python overlap/ensemble을 **정확히** 재구성하거나 RTC를 재추론한 실험은
수행하지 않았다. 기존 로그는 실제 보낸 8행만 있어서 최소 3R=12행을 쓰는 ensemble과
원래 activation/conditioner 이력을 모두 복원할 수 없다. RTC는 prefix로 조건화한
모델 재생성이 필요하여, 고정된 예측을 섞는 것만으로 대체할 수 없다.
이번 full activated horizon + metadata 로깅은 다음 비교의 증거를 확보한다.

## 추가 로그와 파일

실행 코드:

- `policy_runner/policy_runner/{arm_init_control,config,flow_inference,rollout_step_log}.py`.
- `rb_servo_server/include/rb_servo/core/types.hpp`.
- `rb_servo_server/include/rb_servo/control/{dual_arm_servo_loop,cartesian_chunk_follower,chunk_follower_core}.hpp`.
- `rb_servo_server/src/control/dual_arm_servo_loop.cpp`.
- `rb_servo_server/src/network/{command_server,state_publisher}.cpp` 및 `src/logging/servo_logger.cpp`.
- 새 `rb_servo_server/tools/delta_follower_replay.cpp`,
  `tools/{prepare_delta_replay,analyze_delta_replay}.py`, CMake target.
- 관련 Python/C++ 회귀 tests, `policy_runner/README.md`, network protocol 문서, 이 보고서.

schema/config 변경:

- per-arm command `init_motion_request_id`, state `init_motion.<arm>.request_id`.
  기존 schema version 유지, optional additive 필드.
- CSV: per-arm `follower_axis_duration_sec_0..5`, `follower_target_velocity_0..5`,
  `follower_target_acceleration_0..5`, `follower_segments`, `follower_advance_gate`,
  `follower_plan_rate_gate`, `follower_advance_dir_x/y/z`, `follower_output_smd_reseeded`.
  duration은 초, translation 미분값은 m/s 및 m/s², rotation tangent는 rad/s 및 rad/s².
  segment 수/ID로 deduplicate하며 follower 비활성/실패 tick은 유효 solve로 세지 않는다.
- `[arm_init_event]` edge JSON 및 `arm_init` 상태의 request ID/대기시간/timeout.
- chunk JSONL: `active_model_horizon`, `chunk_metadata`, `rtc_enabled`.
  H24 원본 전체가 아니라 activation에서 만료 행이 제거된 전체 chunk일 수 있다.
- YAML force/safety/하드웨어 제한, RTC/crossfade 기본값은 이 작업에서 변경하지 않았다.
  기존 양수 tare timeout의 동작 의미만 경고 후 Hold로 바뀐다.
- ensemble 관련 기존 FIR 회귀검사에서 `_CHUNK_POSE_DIMS` 미정의로 opt-in 필터가
  조용히 no-op이던 결함을 발견했다. pose 12축만 필터링하도록 상수를 복구했다.
  그리퍼는 보존하며, 이 rollout에서 꺼져 있던 FIR을 켜지는 않았다.

이 작업과 동시에 진행 중이던 camera/ZMQ, GUI, DH calibration 및 stack safety
수정은 보존했다. workspace 전체 diff가 전부 이 작업의 변경은 아니다.
재생 시 follower/output-SMD profile은 HEAD `d7bf446`과 동일함을 확인했고,
profile snapshot과 입력/도구 hash를 산출물의 `provenance.json`에 남겼다.

## 재현 명령: 하드웨어 연결 없음

아래 executable은 config를 읽지만 backend/socket/device를 생성하지 않는다.

```bash
cmake -S rb_servo_server -B rb_servo_server/build
cmake --build rb_servo_server/build --target delta_follower_replay -j 6
/home/plaif/workspace/openpi/.venv/bin/python tools/prepare_delta_replay.py \
  logs/servo_log_20260906_114336.csv \
  outputs/sweep/20260906_114429_boltv2_griponly_40k.chunks.jsonl \
  outputs/griponly_replay_20260906 --end-sec 132
rb_servo_server/build/delta_follower_replay \
  rb_servo_server/config/stack_real.yaml \
  outputs/griponly_replay_20260906/left.events.jsonl \
  outputs/griponly_replay_20260906/left_baseline.csv baseline
/home/plaif/workspace/openpi/.venv/bin/python tools/analyze_delta_replay.py \
  outputs/griponly_replay_20260906 --window left:84.5:87.5
```

다른 variant는 CLI 마지막 인수를 `no_force_gate`, `linear_relaxed`,
`angular_relaxed`, `zero_af`, `zero_af_linear`, `zero_af_angular`,
`no_corner_brake`, `translation_only`, `rotation_only`, `overlap_linear2`,
`overlap_linear4`, `overlap_quintic4`로 지정한다. 각 출력 파일은
`left_<variant>.csv` / `right_<variant>.csv`로 저장한다.
선택적인 마지막 `VARIANT_START_SEC`는 baseline, gate 및 overlap에만 지원된다.
국소 seam 비교는 해당 폴더에 baseline과 observed.csv를 함께 두고,
예를 들어 `overlap_linear4 127.5`로 전체 입력을 재생한 뒤
`--window left:128.5:131.5`로 분석한다.

## 검증 및 남은 일

- C++ 전체 build 성공. CTest 44개 전체 통과; 요청 ID 파싱/재전송,
  InitMotion pursuit/tare, follower C²/hold-resume/reanchor, output SMD,
  CSV header/row 일치 및 replay 검증 포함.
- runtime venv Python suite 577개 실행, 실패 없음, fixture 부재로 3개 skip
  (`episode_002.hdf5`를 필요로 하는 dataset/HDF5 audit/viewer 검사).
- replay는 상수 delta의 overlap 무왜곡, 변경 전 동일 상태 유지,
  누락 frame/잘못된 dt/역행 시간 거부를 검증한다.
- Python compileall 및 `git diff --check` 확인.
- 초기 검증 중 시스템 Python의 누락 dependency 및 build linking과 Python
  executable 테스트의 동시 접근 오류가 있었다. runtime venv 및 build 완료 후
  재검증한 결과를 위에 기록했다.
- 실기/pgmode backend acceptance, 모델 RTC 재추론, 모델 A/B rollout은 수행하지 않았다.
  로봇/모델 프로세스를 시작하거나 현재 실행 중인 프로세스를 재시작하지 않았다.

다음 단계는 새 로그로 InitMotion 1회에 request/settle/tare/resume가 각각 한 번인지,
bias valid/coverage 이후 follower와 SMD가 어떻게 진입하는지 확인하는 것이다.
그 뒤 force gate를 유지하면서 위치 제약에 맞게 목표 p/v/a를 함께 구성하는
오프라인 대안을 검증하고, 경로 변화/접촉 편향까지 제한해야 한다.
완전한 ensemble/RTC는 full horizon과 observation/activation 이력을 확보한
별도 비교가 필요하다. 제어 조건을 맞춘 뒤에만 동일 초기 배치에서
`boltv2_40k`와 griponly를 비교한다.

**Real-mode 동작 영향 있음:** 다음 배포부터 InitMotion/tare latch의 재개 동작과
새 optional request ID, 로깅, opt-in FIR 결함 수정이 실제 실행 코드에도 적용된다.
실제 하드웨어 동작은 이번 작업에서 실행하지 않았으며, 힘 제어/안전 제한을
완화하거나 실험용 gate 제거/overlap을 production에 적용하지 않았다.
