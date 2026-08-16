# camera_server 단일 이미지: C++ 캡처(librealsense) + stereo_worker(손목 클라우드 발행).
# head 스테레오 추론(Fast-FoundationStereo) 제거(2026-08-16)로 GPU 의존이 사라져
# CUDA 베이스 + torch + TensorRT 레이어를 걷어냈다. 남은 워커는 numpy/pyzmq 뿐이다.
FROM ubuntu:22.04

ARG LIBREALSENSE_VERSION=2.58.1
ARG LIBREALSENSE_REF=bf2778061d5dd29776e9aca8765f75852671760b
ARG LIBREALSENSE_BACKEND=native

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_NO_CACHE_DIR=1 \
    PYTHONUNBUFFERED=1

# --- 시스템 패키지: C++ 빌드 + python 런타임 ---
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

# --- Python: stereo_worker(wrist-only) 런타임 ---
RUN python3 -m pip install --upgrade pip
COPY stereo_worker/requirements.txt /tmp/requirements.txt
RUN python3 -m pip install -r /tmp/requirements.txt

# --- C++ camera_server 빌드 ---
WORKDIR /app
COPY . /app
RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
        -DCAMERA_SERVER_REALSENSE_BACKEND="${LIBREALSENSE_BACKEND}" \
    && cmake --build build -j"$(nproc)"

ENV CAMERA_SERVER_CONFIG=/app/config/triple_realsense.yaml

# 기본 진입점은 C++ 캡처 서버. 운영 기동은 compose entrypoint(run_all.sh)가 캡처와
# 워커를 함께 띄운다. 워커만 따로 돌리려면:
#   docker compose ... run --entrypoint python3 camera_server /app/stereo_worker/worker.py --run
ENTRYPOINT ["/app/build/camera_server"]
CMD ["--config", "/app/config/triple_realsense.yaml"]
