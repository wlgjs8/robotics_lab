FROM ubuntu:24.04 AS build

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       ca-certificates \
       cmake \
       g++ \
       make \
       git \
       python3 \
       libyaml-cpp-dev \
       nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY rb_servo_server /workspace/rb_servo_server
COPY rb_gui /workspace/rb_gui
RUN cmake -S rb_servo_server -B rb_servo_server/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DRB_SERVO_ENABLE_RBPODO=OFF \
      -DRB_SERVO_ALLOW_FETCHCONTENT=OFF \
      -DBUILD_TESTING=OFF \
    && cmake --build rb_servo_server/build -j"$(nproc)"

FROM ubuntu:24.04 AS runtime

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates libstdc++6 libyaml-cpp0.8 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY --from=build /workspace/rb_servo_server/build/rb_servo_server /workspace/build/rb_servo_server
COPY rb_servo_server/config /workspace/config
COPY rb_servo_server/docs /workspace/docs
RUN mkdir -p /workspace/logs
ENTRYPOINT ["/workspace/build/rb_servo_server"]
CMD ["--config", "config/dual_mock_compose.yaml"]
