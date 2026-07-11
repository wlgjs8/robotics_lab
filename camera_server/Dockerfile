# camera_server 단일 이미지: C++ 캡처(librealsense) + GPU 스테레오 추론(Fast-FoundationStereo).
# Blackwell(RTX 5090, sm_120) -> CUDA 12.8 베이스 + torch cu128.
FROM nvidia/cuda:12.8.0-cudnn-devel-ubuntu22.04

ARG LIBREALSENSE_VERSION=2.58.1
ARG LIBREALSENSE_REF=bf2778061d5dd29776e9aca8765f75852671760b
ARG LIBREALSENSE_BACKEND=native

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_NO_CACHE_DIR=1 \
    PYTHONUNBUFFERED=1

# --- 시스템 패키지: C++ 빌드 + python + opencv/open3d 런타임 ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake pkg-config ca-certificates curl \
    libyaml-cpp-dev libzmq3-dev libopencv-dev python3 python3-pip python3-dev git \
    libgl1 libglib2.0-0 libgomp1 libssl-dev libusb-1.0-0-dev libudev-dev \
    && rm -rf /var/lib/apt/lists/*

# Pin the SDK to the release matched to the deployed D435 5.17.3.10 firmware.
# The two D405 cameras use 5.17.0.10. Native V4L2 is the production default;
# RSUSB is an explicit diagnostic fallback that bypasses the kernel UVC stack.
RUN case "${LIBREALSENSE_BACKEND}" in \
      native) rsusb=OFF ;; \
      rsusb) rsusb=ON ;; \
      *) echo "LIBREALSENSE_BACKEND must be native or rsusb" >&2; exit 2 ;; \
    esac \
    && git clone --filter=blob:none https://github.com/realsenseai/librealsense.git /tmp/librealsense \
    && git -C /tmp/librealsense checkout --detach "${LIBREALSENSE_REF}" \
    && test "$(git -C /tmp/librealsense describe --tags --exact-match)" = "v${LIBREALSENSE_VERSION}" \
    && cmake -S /tmp/librealsense -B /tmp/librealsense/build \
         -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_INSTALL_PREFIX=/usr/local \
         -DBUILD_EXAMPLES=OFF \
         -DBUILD_GRAPHICAL_EXAMPLES=OFF \
         -DBUILD_TOOLS=OFF \
         -DBUILD_UNIT_TESTS=OFF \
         -DBUILD_PYTHON_BINDINGS=OFF \
         -DFORCE_RSUSB_BACKEND="${rsusb}" \
    && cmake --build /tmp/librealsense/build -j"$(nproc)" \
    && cmake --install /tmp/librealsense/build \
    && ldconfig \
    && rm -rf /tmp/librealsense

# --- Python: torch(cu128, Blackwell) + Fast-FoundationStereo deps (torch 레이어 캐시 위해 먼저) ---
RUN python3 -m pip install --upgrade pip \
    && python3 -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
COPY stereo_worker/requirements.txt /tmp/requirements.txt
RUN python3 -m pip install -r /tmp/requirements.txt
# TensorRT 가속 (Blackwell sm_120 -> TRT 10.x cu12). 엔진 빌드/실행은 tensorrt python API.
# TRT 10.x 고정: 11.x는 FP16 BuilderFlag 제거(strongly-typed 전환)라 repo의 --fp16 경로와 불일치.
# 10.7+ 는 Blackwell(sm_120) 지원. 엔진은 런타임과 동일 TRT 버전이어야 함.
RUN python3 -m pip install "tensorrt<11" onnx onnxscript

# --- C++ camera_server 빌드 ---
WORKDIR /app
COPY . /app
RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
        -DCAMERA_SERVER_REALSENSE_BACKEND="${LIBREALSENSE_BACKEND}" \
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
