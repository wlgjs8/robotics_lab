FROM ubuntu:22.04 AS build

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       ca-certificates \
       curl \
       gnupg \
       cmake \
       g++ \
       make \
       git \
       python3 \
       libeigen3-dev \
       libyaml-cpp-dev \
       nlohmann-json3-dev \
    && install -d -m 0755 /etc/apt/keyrings \
    && curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc -o /etc/apt/keyrings/robotpkg.asc \
    && echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub jammy robotpkg" > /etc/apt/sources.list.d/robotpkg.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends robotpkg-pinocchio \
    && rm -rf /var/lib/apt/lists/*

ENV CMAKE_PREFIX_PATH=/opt/openrobots

WORKDIR /workspace
COPY rb_servo_server /workspace/rb_servo_server
COPY rb_gui /workspace/rb_gui
RUN cmake -S rb_servo_server -B rb_servo_server/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DRB_SERVO_ENABLE_RBPODO=OFF \
      -DRB_SERVO_ENABLE_PINOCCHIO=ON \
      -DRB_SERVO_ALLOW_FETCHCONTENT=OFF \
      -DBUILD_TESTING=OFF \
    && cmake --build rb_servo_server/build -j"$(nproc)"

FROM ubuntu:22.04 AS runtime

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates curl gnupg libstdc++6 libyaml-cpp0.7 \
    && install -d -m 0755 /etc/apt/keyrings \
    && curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc -o /etc/apt/keyrings/robotpkg.asc \
    && echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub jammy robotpkg" > /etc/apt/sources.list.d/robotpkg.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends robotpkg-pinocchio \
    && rm -rf /var/lib/apt/lists/*

ENV LD_LIBRARY_PATH=/opt/openrobots/lib

WORKDIR /workspace
COPY --from=build /workspace/rb_servo_server/build/rb_servo_server /workspace/build/rb_servo_server
COPY rb_servo_server/config /workspace/config
COPY rb_servo_server/docs /workspace/docs
COPY rb_servo_server/descriptions /workspace/descriptions
RUN mkdir -p /workspace/logs
ENTRYPOINT ["/workspace/build/rb_servo_server"]
CMD ["--config", "config/dual_mock_compose.yaml"]
