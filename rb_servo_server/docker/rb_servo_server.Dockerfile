FROM ubuntu:24.04 AS build

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       ca-certificates \
       cmake \
       g++ \
       make \
       git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY . /workspace
RUN cmake -S . -B build -DBUILD_TESTING=ON \
    && cmake --build build -j"$(nproc)"
CMD ["./build/rb_servo_server", "--config", "config/dual_mock_compose.yaml"]

FROM ubuntu:24.04 AS runtime
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates libstdc++6 \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /workspace
COPY --from=build /workspace/build/rb_servo_server /workspace/build/rb_servo_server
COPY config /workspace/config
COPY docs /workspace/docs
RUN mkdir -p /workspace/logs
ENTRYPOINT ["/workspace/build/rb_servo_server"]
CMD ["--config", "config/dual_mock_compose.yaml"]
