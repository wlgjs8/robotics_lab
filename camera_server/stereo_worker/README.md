# stereo_worker (camera_server 통합)

손목 D405 포인트클라우드 발행기. **`camera_server` 단일 컨테이너에 통합**되어, 같은
이미지가 C++ 캡처(librealsense)와 이 파이썬 워커를 모두 갖는다.

**모델은 없다.** D405가 하드웨어로 만든 depth(z16)를 카메라 프레임 3D점으로
deproject해서 ZMQ로 내보낼 뿐이다. head D435 IR 스테레오 → Fast-FoundationStereo
disparity → head 클라우드 / 박스검출 / external-box 송신 경로는 2026-08-16에 전부
제거됐다(배경과 포기한 기능 목록: `docs/archive/head_stereo/README.md`).
이름이 `stereo_worker`인 것은 발행 토픽(`stereo.wrist`) 호환을 위한 잔재다.

## 아키텍처
```
[camera_server 컨테이너 (ubuntu:22.04, GPU 불필요)]
  C++ 캡처(손목 color+depth) ──ZMQ 5600 + /dev/shm 링──► worker.py
                                                          deproject → stereo.wrist
                                                          └─ publish 5601 ──► rb_gui
```
캡처와 워커가 같은 컨테이너라 `/dev/shm` 링·localhost ZMQ를 그대로 공유한다.

## 구성요소
- `worker.py` — 손목 번들 구독 → `wrist_cloud()` deprojection → `stereo.wrist` publish.
- `bundle_reader.py` — 번들 메타(ZMQ) + 공유메모리 링에서 최신 프레임 읽기.
- `run_all.sh` — compose entrypoint. C++ 캡처와 워커를 함께 기동/재기동한다.
- `requirements.txt` — numpy, pyzmq (그게 전부다).
- 이미지/서비스: 루트 `docker-compose.yml`의 `camera_server`(profile `real_camera`).

## 기동
```bash
make cam-up            # 캡처 + 워커 (head D435 색상 + 손목 D405 2개)
make cam-up-wrists     # 손목 D405 2개만
make cam-status
```
워커만 따로 돌리려면:
```bash
docker compose --profile real_camera run --rm \
    --entrypoint python3 camera_server /app/stereo_worker/worker.py --run
```

## 환경변수
- `CAMERA_BUNDLE_ENDPOINT` (기본 `tcp://127.0.0.1:5600`) — 번들 메타 SUB 엔드포인트.
- `STEREO_WRIST_BUNDLE_TOPIC` (기본: 미설정) — 미설정이면 좌/우 독립 그룹
  (`camera.bundle.wrist_left`, `camera.bundle.wrist_right`)을 각각 구독한다. 한쪽 손목
  카메라가 죽어도 건강한 쪽 클라우드가 계속 흐르게 하기 위함이다. 값을 주면 그 단일
  토픽만 구독한다.
- `CLOUD_PUB_BIND` (기본 `tcp://127.0.0.1:5601`) — `stereo.wrist` PUB bind.
- `STEREO_WRIST_EVERY` (기본 `3`) — N프레임마다 1회 발행(계산·대역 절감).
- `STEREO_VIZ_MAX_PTS` (기본 `30000`) — 프레임당 발행 점수 상한. `0`=무제한.
- `STEREO_PROFILE` (기본 `1`) — fps 로그에 단계별 ms 브레이크다운 포함.
- `STEREO_WORKER_AUTOSTART` (기본 `1`) — `run_all.sh`가 워커를 함께 띄울지 여부.
  `0`이면 캡처만 뜬다. 발행을 끄고 싶으면 이 값을 쓴다 — `STEREO_WRIST=0`은
  워커가 할 일이 없다는 뜻이라 즉시 종료(fail-closed)한다.

## 발행 포맷
`stereo.wrist` 멀티파트: `[b"stereo.wrist", header_json, xyz_f32, rgb_u8]`.
header는 `{"arm": "left"|"right", "seq": int, "n": int}`. 점은 **카메라 프레임**이며,
`rb_gui`가 TCP × hand-eye(`T_tcp_cam`)로 stand 프레임에 배치한다.
