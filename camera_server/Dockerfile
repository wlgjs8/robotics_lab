FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake pkg-config ca-certificates curl gnupg apt-transport-https \
    libyaml-cpp-dev libzmq3-dev libopencv-dev python3 python3-pip \
    && mkdir -p /etc/apt/keyrings \
    && gpg --batch --keyserver hkps://keyserver.ubuntu.com --recv-keys FB0B24895113F120 && gpg --batch --export FB0B24895113F120 > /etc/apt/keyrings/librealsense.gpg \
    && echo "deb [signed-by=/etc/apt/keyrings/librealsense.gpg] https://librealsense.intel.com/Debian/apt-repo jammy main" > /etc/apt/sources.list.d/librealsense.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends librealsense2-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY . /app
RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build -j"$(nproc)"

ENV CAMERA_SERVER_CONFIG=/app/config/triple_realsense.yaml
ENTRYPOINT ["/app/build/camera_server"]
CMD ["--config", "/app/config/triple_realsense.yaml"]
