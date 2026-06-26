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
기본 기동(캡처 서버)은 기존대로 `make camera-real-up` / `docker compose --profile real_camera up camera_server`.

## 환경변수
- `FFS_DIR` (기본 `/app/Fast-FoundationStereo`) — 마운트된 submodule 경로.
- `STEREO_WEIGHTS` — 기본 `weights/23-36-37/...`(정확). 빠른 모델은 `weights/20-30-48/...`.

## 다음 단계
1. 스모크테스트 통과 확인 (PyTorch 경로).
2. camera_server IR 지원(C++) 후 `--run`: 번들(RGB+IR-L/R) 구독 → cloud publish. 두 프로세스
   동시 기동 런처(entrypoint)로 전환.
3. TensorRT 가속(`scripts/make_*onnx.py` → `build_plugin_trt.py`), 컨테이너 내 엔진 빌드.
