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
      - CAMERA_SERVER_CONFIG=/app/config/triple_realsense.yaml

  policy_runner:
    image: policy_runner:latest
    container_name: policy_runner
    ipc: host
    shm_size: "2gb"
    network_mode: host
    volumes:
      - ./config:/app/config:ro
```

Key fields:

```text
ipc: host
  - allows policy_runner to open same POSIX shared memory

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

If using `network_mode: host`, `127.0.0.1` works across host processes but not across separate network namespaces. With host networking, both containers share host network namespace.

## 5. Shared memory naming

Use POSIX names:

```text
/camera_server_frames
```

Do not use random names unless published through config or discovery endpoint.

## 6. Host prerequisites

Recommended packages:

```text
librealsense2
librealsense2-dev
libzmq3-dev
nlohmann-json3-dev
```

If offline deployment is needed, vendor small header-only dependencies such as nlohmann/json.

## 구현 파일

Docker 설정은 repository root의 `Dockerfile`과 `docker-compose.yml`에 구현되어 있다. Compose service는 `container_name: camera_server`, `ipc: host`, `shm_size: "2gb"`, `network_mode: host`, `privileged: true`, USB/udev mounts, config/episode volumes를 포함한다.
