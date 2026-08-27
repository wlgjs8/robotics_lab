# camera_server 이미지: C++ 캡처(librealsense) 전용.
# 학습형 스테레오(Fast-FoundationStereo) 제거 전까지는 torch/TensorRT 때문에 CUDA 12.8
# 베이스를 썼다. C++ 쪽은 CUDA를 쓰지 않으므로(camera_server/CMakeLists.txt에 CUDA 의존
# 없음) 평범한 22.04 베이스로 되돌린다.
FROM ubuntu:22.04

ARG LIBREALSENSE_VERSION=2.58.1
ARG LIBREALSENSE_REF=bf2778061d5dd29776e9aca8765f75852671760b
ARG LIBREALSENSE_BACKEND=native

ENV DEBIAN_FRONTEND=noninteractive

# --- 시스템 패키지: C++ 빌드 + opencv(UVC/fisheye 백엔드) 런타임 ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake pkg-config ca-certificates curl \
    libyaml-cpp-dev libzmq3-dev libopencv-dev git \
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

# --- C++ camera_server 빌드 ---
WORKDIR /app
COPY . /app
RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
        -DCAMERA_SERVER_REALSENSE_BACKEND="${LIBREALSENSE_BACKEND}" \
    && cmake --build build -j"$(nproc)"

ENV CAMERA_SERVER_CONFIG=/app/config/triple_realsense.yaml

# 진입점은 C++ 캡처 서버. CMD 로 --config 를 굽지 않는다: argv 가 env 보다 우선이라
# 하드코딩하면 CAMERA_SERVER_CONFIG/CAMERA_CONFIG 오버라이드가 전부 무시된다.
# (compose 는 docker-compose.yml 의 command 로 리그를 명시한다.)
ENTRYPOINT ["/app/build/camera_server"]
