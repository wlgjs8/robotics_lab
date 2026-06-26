# camera_server 단일 이미지: C++ 캡처(librealsense) + GPU 스테레오 추론(Fast-FoundationStereo).
# Blackwell(RTX 5090, sm_120) -> CUDA 12.8 베이스 + torch cu128.
FROM nvidia/cuda:12.8.0-cudnn-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_NO_CACHE_DIR=1 \
    PYTHONUNBUFFERED=1

# --- 시스템 패키지: C++ 빌드 + librealsense + python + opencv/open3d 런타임 ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake pkg-config ca-certificates curl gnupg apt-transport-https \
    libyaml-cpp-dev libzmq3-dev libopencv-dev python3 python3-pip python3-dev git \
    libgl1 libglib2.0-0 libgomp1 \
    && mkdir -p /etc/apt/keyrings \
    && gpg --batch --keyserver hkps://keyserver.ubuntu.com --recv-keys FB0B24895113F120 && gpg --batch --export FB0B24895113F120 > /etc/apt/keyrings/librealsense.gpg \
    && echo "deb [signed-by=/etc/apt/keyrings/librealsense.gpg] https://librealsense.intel.com/Debian/apt-repo jammy main" > /etc/apt/sources.list.d/librealsense.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends librealsense2-dev \
    && rm -rf /var/lib/apt/lists/*

# --- Python: torch(cu128, Blackwell) + Fast-FoundationStereo deps (torch 레이어 캐시 위해 먼저) ---
RUN python3 -m pip install --upgrade pip \
    && python3 -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
COPY stereo_worker/requirements.txt /tmp/requirements.txt
RUN python3 -m pip install -r /tmp/requirements.txt
# TensorRT 가속은 다음 단계: pip install tensorrt-cu12 onnx onnxruntime-gpu (TRT 10.x)

# --- C++ camera_server 빌드 ---
WORKDIR /app
COPY . /app
RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build -j"$(nproc)"

# Fast-FoundationStereo 코드/가중치는 compose 볼륨으로 마운트 (FFS_DIR), 큰 파일은 이미지에 안 굽는다.
ENV CAMERA_SERVER_CONFIG=/app/config/triple_realsense.yaml \
    FFS_DIR=/app/Fast-FoundationStereo \
    STEREO_WEIGHTS=/app/Fast-FoundationStereo/weights/23-36-37/model_best_bp2_serialize.pth

# 기본 진입점은 C++ 캡처 서버. 스테레오 워커는 같은 이미지에서
#   docker compose ... run --entrypoint python3 camera_server /app/stereo_worker/worker.py --smoke
# 로 실행(추후 live 모드는 두 프로세스 동시 기동 런처로 전환).
ENTRYPOINT ["/app/build/camera_server"]
CMD ["--config", "/app/config/triple_realsense.yaml"]
