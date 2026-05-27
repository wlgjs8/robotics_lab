FROM ubuntu:24.04 AS build

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       ca-certificates \
       cmake \
       g++ \
       make \
       libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY camera_server /app
RUN cmake -S . -B build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCAMERA_SERVER_FORCE_MOCK_CAMERA=ON \
      -DCAMERA_SERVER_FORCE_ZMQ_STUB=ON \
      -DCAMERA_SERVER_BUILD_TESTS=OFF \
    && cmake --build build -j"$(nproc)"

FROM ubuntu:24.04 AS runtime

ENV DEBIAN_FRONTEND=noninteractive \
    CAMERA_SERVER_CONFIG=/app/config/mock_triple_realsense.yaml
RUN apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates libstdc++6 libyaml-cpp0.8 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY --from=build /app/build/camera_server /app/build/camera_server
COPY camera_server/config /app/config
RUN mkdir -p /data/episodes
ENTRYPOINT ["/app/build/camera_server"]
CMD ["--config", "/app/config/mock_triple_realsense.yaml", "--simulate"]
