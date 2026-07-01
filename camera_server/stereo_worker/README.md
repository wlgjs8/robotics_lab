# stereo_worker (camera_server 통합)

Fast-FoundationStereo GPU 추론 로직. **`camera_server` 단일 컨테이너에 통합**되어, 같은
이미지가 C++ 캡처(librealsense, RGB+IR)와 스테레오 추론(torch)을 모두 갖는다. IR 스테레오
페어 → disparity → depth → RGB-매핑 pointcloud 를 만들어 publish 한다. `rb_gui`(viser
렌더)는 torch-free로 유지된다.

## 아키텍처
```
[camera_server 컨테이너 (CUDA 베이스)]
  C++ 캡처(RGB+IR-L+IR-R)  ──ZMQ 5600 + /dev/shm 링──►  stereo_worker(python/torch)
                                                          Fast-FoundationStereo → cloud
                                                          └─ publish ──► rb_gui (고급→Pointcloud)
```
캡처와 워커가 같은 컨테이너라 `/dev/shm` 링·localhost ZMQ를 그대로 공유한다.

## 구성요소
- `stereo_model.py` — `FoundationStereoModel`: 모델 로드 + `infer_disparity` + `disparity_to_cloud`.
- `worker.py` — 현재 `--smoke`(데모 페어 추론 검증), 이후 `--run`(live, 번들 구독→publish).
- `requirements.txt` — 모델 deps (torch는 camera_server/Dockerfile에서 cu128로 설치).
- 이미지/서비스: 루트 `docker-compose.yml`의 `camera_server`(profile `real_camera`).
  Dockerfile은 `camera_server/Dockerfile`(CUDA 12.8 + torch cu128 + librealsense + C++ 빌드).

## 빌드 & 스모크테스트
```bash
cd robotics_lab
# 단일 이미지 빌드 (librealsense+C++ + torch cu128+deps, 수 GB / 십수 분)
docker compose --profile real_camera build camera_server
# 데모 스테레오 페어로 모델 구동 검증 (캡처는 건너뛰고 워커만 GPU로 실행)
docker compose --profile real_camera run --rm \
    --entrypoint python3 camera_server /app/stereo_worker/worker.py --smoke
# 결과: camera_server/stereo_worker/out/disp_vis.png, out/cloud.npz
```
기본 기동(캡처 서버 + 스테레오 워커)은 `make cam-up` / `docker compose --profile real_camera up camera_server`.

## 환경변수
- `FFS_DIR` (기본 `/app/Fast-FoundationStereo`) — 마운트된 submodule 경로.
- `STEREO_WEIGHTS` — 기본 `weights/23-36-37/...`(정확). 빠른 모델은 `weights/20-30-48/...`.
- `STEREO_DETECT` (기본 `1`) — head stereo 박스 검출 publish(`stereo.boxes`) 활성화.
- `STEREO_DETECT_ICP` (기본 `1`) — 검출 후보에 known-model ICP refinement 적용.
- `STEREO_DETECT_ICP_METHOD` (기본 `yaw_se2`) — `yaw_se2` | `point_to_point` |
  `point_to_plane`. `yaw_se2`(기본)는 box up=stand z 고정 SE(2) 정합으로 부분관측/
  노이즈에 robust(scipy). `point_to_plane`은 opt-in 실험 경로이며 실패 시
  `point_to_point`로 fallback한다. 미인식 값은 `yaw_se2`로 정규화된다.
- `STEREO_TEMPORAL_ICP` (기본 `1`) — **tracking-by-registration**. ICP를 매 프레임
  minAreaRect가 아니라 **직전 포즈**(label별, `TEMPORAL_GATE_M`=0.10m 이내일 때)에서
  init → 정지 박스의 프레임간 jitter를 ~30mm→~1mm로 줄인다(스테레오 노이즈를 매 프레임
  처음부터 fit하지 않음). 박스가 게이트보다 멀리 점프하면 minAreaRect로 재획득. 재시작/
  재획득 직후 ~수초 transient는 정상.
- `STEREO_GRAY_OPEN_K` (기본 `3`) — 회색(colorless) 박스 de-bridge OPEN 커널(px).
  회색은 색 신호가 없어 테이블/잡음과 붙어 oversize 되기 쉬운데, CLOSE 없이 OPEN으로
  연결부를 끊어 분리한다. 클수록 강하게 끊지만 희박/가림 회색 박스를 침식한다
  (가림이 잦으면 작게, 잡음이 많아 oversize가 잦으면 크게: 3≈보존, 5≈치수 깔끔, 7≈과침식).

### Safety ROI 클립 (viz·검출 영역 정렬)
박스 검출과 publish 클라우드(head + 손목) 모두 rb_gui **Safety ROI 박스**(stand 프레임)
밖의 점을 버린다 → viser 시각화와 검출이 같은 영역만 보고, ROI 밖 noise가 추정에 안 들어간다.
ROI는 rb_gui가 `settings.json`의 `roi_min_m`/`roi_max_m`(stand m, `[x,y,z]`)로 영속화하고
(슬라이더 변경/`Send ROI box` 시 기록, 원자적 쓰기), worker가 1Hz로 읽는다. 키가 없으면
클래스 기본값(`x±0.55, y[-1.05,0.05], z[-0.05,1.0]`).
- `STEREO_ROI_CLIP` (기본 `1`) — publish 클라우드 ROI 클립 on/off. 검출 RoI 게이트는
  항상 적용. 손목 클라우드는 state(T_stand_tcp)+핸드아이가 있을 때만 stand 변환이 되어 클립된다.
- **stand-Z 점 상한** (`settings.json` `pc_clip_z_max_m`, rb_gui "stand Z 상한(mm)" 슬라이더,
  또는 env `STEREO_CLIP_Z_MAX`) — Safety ROI의 Z는 **로봇이 움직일 높이**라 perception용으로
  낮추기 어렵다. 이와 **독립한 perception 전용 stand-Z 상한**으로 그보다 높은 점(로봇팔·배경
  noise)을 viz·검출 양쪽에서 버린다. 없으면 상한 없음. worker 로그 `clip RoI update: ... z_cap=`
  로 현재 ROI/Z캡(=GUI 수정 반영 여부)을 확인한다.

### 손목(D405) 융합 — 아이디어2 (head 가림 시 커버리지 보강)
head 박스 검출에 손목 클라우드를 stand 프레임(`P_stand = T_stand_tcp @ T_tcp_cam @
P_wrist_cam`)으로 병합한다. **활성 조건**: rb_servo_server가 state fanout을
`udp://127.0.0.1:50386`로 publish해야 한다(`stack_sim.yaml`/`stack_real.yaml`의
`network.state_pub_endpoints`에 이미 포함). state 미수신/stale 시 자동 head-only.
- `STEREO_FUSE_WRIST` (기본 `1`) — 융합 on/off.
- `STEREO_FUSE_MOTION_GATE` (기본 `1`) — **동기화 게이트**. 손목 프레임과 TCP pose는
  하드웨어 타임스탬프 동기가 안 되므로(번들 `Frame`에 ts 없음) 팔이 움직이면 손목 점이
  어긋난다. 팔이 (거의) 정지일 때만 융합하고, 이동 중에는 그 팔을 head-only로 둔다.
- `STEREO_FUSE_MAX_LIN_MPS` (기본 `0.03`) / `STEREO_FUSE_MAX_ANG_RPS` (기본 `0.15`) —
  정지 판정 임계(최근 ~0.2s TCP 선/각속도). worker 로그의 `fuse[rx=.. fused=.. gated=..]`로
  수신/융합/게이트 상태를 확인한다(`rx=0`이면 state 미수신 = rb_servo_server 미기동).

`stereo.boxes` payload는 기존 `T`/`dims`/`footprint`/`n`/`label`에 더해, 가능하면
`fitness`, `rmse`, `track_id`, `icp_method`, `source_n`, `icp_sample_n`, `coasting`
telemetry를 additive field로 싣는다. 기존 consumer는 필드를 무시해도 된다.

## 다음 단계
1. 스모크테스트 통과 확인 (PyTorch 경로).
2. camera_server IR 지원(C++) 후 `--run`: 번들(RGB+IR-L/R) 구독 → cloud publish. 두 프로세스
   동시 기동 런처(entrypoint)로 전환.
3. TensorRT 가속(`scripts/make_*onnx.py` → `build_plugin_trt.py`), 컨테이너 내 엔진 빌드.
