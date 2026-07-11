# Docker Deployment

## 1. Container name

The Docker service/container should be named:

```text
camera_server
```

## 2. Required Docker settings

RealSense USB access and shared memory require special Docker configuration.

Recommended docker-compose snippet:

```yaml
services:
  camera_server:
    image: camera_server:latest
    container_name: camera_server
    ipc: host
    shm_size: "2gb"
    privileged: true
    network_mode: host
    devices:
      - /dev/bus/usb:/dev/bus/usb
    volumes:
      - /dev:/dev
      - /run/udev:/run/udev:ro
      - ./config:/app/config:ro
      - ./episodes:/data/episodes
    environment:
      - CAMERA_SERVER_CONFIG=/app/config/d435_head_1280x720.yaml
```

`policy_runner` runs natively (not in Docker) and opens the same POSIX shared
memory directly; `ipc: host` on the camera container is what makes that shared
memory visible to host processes.

Key fields:

```text
ipc: host
  - lets host-native policy_runner open the same POSIX shared memory

shm_size
  - Docker default 64 MB is too small

/dev/bus/usb
  - required for RealSense devices

/run/udev
  - helps device enumeration
```

## 3. Security note

`privileged: true` is easy but broad. For production, reduce privileges if possible.

However, for early RealSense development, `privileged: true` is acceptable to avoid USB permission issues.

## 4. Networking

Metadata should bind to loopback by default:

```yaml
metadata:
  pub_bind: "tcp://127.0.0.1:5600"
```

If using `network_mode: host`, `127.0.0.1` works across host processes but not across separate network namespaces. With host networking, the camera container shares the host network namespace, so host-native `policy_runner` can reach it on `127.0.0.1`.

## 5. Shared memory naming

Use POSIX names:

```text
/camera_server_frames
```

Do not use random names unless published through config or discovery endpoint.

## 6. Librealsense version and backend

The image builds librealsense from the pinned official release rather than the
legacy Jammy package repository. The deployed D435 firmware `5.17.3.10`
requires SDK `2.58.1`; the D405 pair remains on firmware `5.17.0.10`.

```bash
# Production/default: native V4L2 backend
make cam-up

# Diagnostic fallback only, after the native acceptance run fails
make cam-up LIBREALSENSE_BACKEND=rsusb
```

Build arguments are `LIBREALSENSE_VERSION`, `LIBREALSENSE_REF`, and
`LIBREALSENSE_BACKEND=native|rsusb`. The tracked defaults pin both the tag and
commit. Do not move the ref independently of the version.

The server fails before opening streams when a required RealSense is USB2, the
SDK is older than `2.58.1`, a D405 is older than `5.17.0.10`, or the configured
D405 cameras have different firmware versions.

## 7. Optional D405 autosuspend diagnostic

Do not install the rule initially. If the version-aligned native test still
disconnects, install `deploy/99-realsense-d405-power.rules` on the host, reload
udev, replug/reset the cameras, and repeat the same acceptance profile. The rule
targets D405 product ID `8086:0b5b` only.

## 8. Host prerequisites

Recommended packages:

```text
libzmq3-dev
nlohmann-json3-dev
```

If offline deployment is needed, vendor small header-only dependencies such as nlohmann/json.

## 구현 파일

Docker 설정은 `camera_server/Dockerfile`과 repository root의
`docker-compose.yml`에 구현되어 있다. Compose service는
`container_name: camera_server`, `ipc: host`, `shm_size: "2gb"`,
`network_mode: host`, `privileged: true`, USB/udev mounts, config/episode
volumes를 포함한다.
