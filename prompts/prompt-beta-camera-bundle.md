# Prompt β — Camera bundle client + Hdf5EpisodeRecorder image integration

## Task

`policy_runner` 에 `CameraBundleClient` 를 추가한다. ZMQ SUB 로 `camera.bundle` 메타데이터를 받고, POSIX 공유메모리에서 이미지 픽셀을 seqlock 으로 읽어 NumPy 배열로 반환한다. 그리고 prompt α 에서 만든 `Hdf5EpisodeRecorder` 를 확장해서 카메라 이미지를 `/observations/images/<cam_name>` 로 기록한다.

## Context

Prerequisites: prompt α 가 완료된 상태. `Hdf5EpisodeRecorder` 가 state+action 만 기록 중. 이번 prompt 에서 카메라를 추가한다.

`camera_server` 의 contract (이미 존재):

- ZMQ PUB 가 `camera.bundle` topic 으로 JSON metadata 발행 (기본 `tcp://127.0.0.1:5600`).
- 각 bundle JSON 에는 `bundle_seq`, `bundle_time_ns`, `hardware_synced`, `complete`, `frames` (map of camera_name → FrameMeta), `drop_counters` 가 포함.
- 각 FrameMeta 에 `shm_name`, `shm_offset`, `size_bytes`, `width`, `height`, `stride_bytes`, `format`, `frame_number`, `host_arrival_time_ns`, `sensor_timestamp_ns`, `seq`, `valid` 가 포함.
- POSIX 공유메모리 path 는 `/dev/shm/<shm_name>`. slot 마다 12-uint64 header (seqlock pattern) 앞에 두고 payload 가 뒤따른다. seqlock format: `SLOT_HEADER = struct.Struct("<QQQQQQIIIIII")` — 첫 uint64 `a` (start tag, even=valid), 두번째 `b` (end tag), 마지막 12번째 `valid` flag.
- safe read 알고리즘 (이미 `camera_server/tools/read_latest_bundle.py` 에 구현됨): `a` 읽고 odd 면 retry → header 전체 unpack → payload copy → `c` (start tag 재읽기) → `a == b == c` 이고 even 이면 valid.

이 reader 패턴을 reusable class 로 추출해서 `policy_runner` 에서 쓴다. ZMQ pyzmq library 와 `multiprocessing.shared_memory` 또는 `mmap` 사용 (기존 tool 은 `mmap` 사용).

핵심 design decisions:
- **pyzmq 와 numpy 는 optional dependency** → `policy_runner[camera]` extra. recording 안 쓰는 사용자는 의존성 없음.
- **shared memory cache** → 한 번 mmap 한 SHM 은 persistent. bundle 의 shm_name 이 바뀌면 unmount.
- **latest bundle only** → drop intermediate bundles. ZMQ SUB 의 conflate option (`zmq.CONFLATE = 1`) 사용 또는 매 poll 마다 drain 후 latest 사용.
- **bundle staleness check** → bundle_time_ns 와 현재 시각 차이 > `max_age_ms` 이면 stale 로 표시. Hdf5EpisodeRecorder 가 매 frame 마다 `bundle_age_us` 도 기록.
- **이미지 디코딩**: 기본은 BGR/RGB raw uint8 (RealSense color stream 기본). depth stream 도 지원해야 하지만 이번 prompt 는 color only 로 한정. depth 는 future prompt.
- **NoneType 결과**: bundle 못 받았으면 NumPy zero-fill image (검정) 기록 + `bundle_age_us = INT64 MAX` 로 marker.

## Files to change

- `policy_runner/policy_runner/camera_bundle_client.py` — 신규
- `policy_runner/policy_runner/recording.py` — `Hdf5EpisodeRecorder` 확장
- `policy_runner/policy_runner/config.py` — `CameraConfig` 추가
- `policy_runner/policy_runner/main.py` — `hdf5-record` subcommand 에 카메라 통합
- `policy_runner/pyproject.toml` — `[camera]` optional extra 추가
- `policy_runner/tests/test_camera_bundle_client.py` — 신규
- `policy_runner/tests/test_hdf5_recording.py` — 카메라 integration 케이스 추가

## Required changes

### 1. `pyproject.toml` extras

```toml
[project.optional-dependencies]
spacemouse = ["pyspacemouse>=1.1.0"]
recording = ["h5py>=3.10.0", "numpy>=1.24.0"]
camera = ["pyzmq>=25.0.0", "numpy>=1.24.0"]
```

### 2. `CameraBundleClient` — 신규 모듈

`policy_runner/policy_runner/camera_bundle_client.py`:

```python
from __future__ import annotations

import json
import mmap
import struct
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional


# Mirror of camera_server/tools/read_latest_bundle.py SLOT_HEADER layout.
# (start_tag, end_tag, frame_number, host_arrival_time_ns, sensor_timestamp_ns,
#  reserved_u64, width, height, stride_bytes, size_bytes, format_code, valid_flag)
_SLOT_HEADER = struct.Struct("<QQQQQQIIIIII")


@dataclass(frozen=True)
class CameraFrame:
    """Decoded camera frame with metadata."""
    camera_name: str
    width: int
    height: int
    pixels: Any   # numpy.ndarray of shape (H, W, C) dtype=uint8
    format: str
    frame_number: int
    host_arrival_time_ns: int
    sensor_timestamp_ns: int


@dataclass(frozen=True)
class CameraBundle:
    """Latest complete bundle with all camera frames decoded."""
    bundle_seq: int
    bundle_time_ns: int
    hardware_synced: bool
    complete: bool
    received_monotonic: float
    frames: Dict[str, CameraFrame]   # keyed by camera_name


class _ShmCache:
    """Cache of mmap'd POSIX SHM files. Closes the previous mapping when name changes."""
    def __init__(self) -> None:
        self._name: Optional[str] = None
        self._file = None
        self._mmap: Optional[mmap.mmap] = None
    
    def get(self, name: str) -> mmap.mmap:
        if self._name == name and self._mmap is not None:
            return self._mmap
        self.close()
        path = "/dev/shm/" + name.lstrip("/")
        self._file = open(path, "r+b")
        self._mmap = mmap.mmap(self._file.fileno(), 0, access=mmap.ACCESS_READ)
        self._name = name
        return self._mmap
    
    def close(self) -> None:
        if self._mmap is not None:
            self._mmap.close()
            self._mmap = None
        if self._file is not None:
            self._file.close()
            self._file = None
        self._name = None


class CameraBundleClient:
    """ZMQ SUB to camera.bundle + shm reader for image payloads.
    
    Drains pending ZMQ messages on each poll() and returns the latest complete
    bundle (or None if no bundle is available within timeout_sec).
    
    Optional dependency: pyzmq + numpy. Importing this class without
    `policy_runner[camera]` installed raises RuntimeError at instantiation.
    """
    
    def __init__(
        self,
        zmq_endpoint: str = "tcp://127.0.0.1:5600",
        topic: str = "camera.bundle",
        *,
        max_age_ms: float = 100.0,
    ):
        try:
            import zmq  # noqa: F401
            import numpy  # noqa: F401
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "CameraBundleClient requires optional dependencies: "
                "install policy_runner with the camera extra"
            ) from exc
        import zmq
        self._zmq = zmq
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.SUB)
        self._sock.setsockopt(zmq.SUBSCRIBE, topic.encode())
        # CONFLATE keeps only the latest message in the recv queue.
        # This is critical: at 30 Hz bundle rate and 100 Hz policy rate,
        # we never want to consume a stale queued bundle.
        self._sock.setsockopt(zmq.CONFLATE, 1)
        self._sock.connect(zmq_endpoint)
        self._endpoint = zmq_endpoint
        self._topic = topic
        self._max_age_ms = float(max_age_ms)
        self._shm_cache = _ShmCache()
        self._latest: Optional[CameraBundle] = None
    
    def poll(self, timeout_ms: int = 0) -> Optional[CameraBundle]:
        """Try to receive one bundle. Returns the latest decoded bundle or
        None if nothing arrived within timeout_ms.
        
        Even with CONFLATE the socket may return None on the first call; the
        caller can keep polling and read self.latest() to get the most
        recently received bundle (possibly older than max_age_ms; see is_fresh).
        """
        ...
    
    def latest(self) -> Optional[CameraBundle]:
        return self._latest
    
    def is_fresh(self, bundle: Optional[CameraBundle] = None) -> bool:
        b = bundle if bundle is not None else self._latest
        if b is None:
            return False
        age_ns = time.time_ns() - b.bundle_time_ns
        return age_ns >= 0 and age_ns < self._max_age_ms * 1_000_000
    
    def close(self) -> None:
        self._sock.close(linger=0)
        self._shm_cache.close()
        # do not terminate the shared zmq Context — it may be in use elsewhere
```

`poll()` 구현 핵심:

```python
def poll(self, timeout_ms: int = 0) -> Optional[CameraBundle]:
    poller = self._zmq.Poller()
    poller.register(self._sock, self._zmq.POLLIN)
    socks = dict(poller.poll(timeout=int(timeout_ms)))
    if self._sock not in socks:
        return None
    # CONFLATE means there is exactly one message waiting.
    try:
        raw = self._sock.recv(self._zmq.NOBLOCK)
    except self._zmq.Again:
        return None
    
    # ZMQ pub-sub with a topic prefix: the topic is part of the message body.
    # camera_server publishes without a separate topic frame, so the raw bytes
    # start with the topic name followed by a space and the JSON body — but
    # current camera_server uses a single send() with just JSON (verify in the
    # existing read_latest_bundle.py reference; if topic is a prefix, strip it).
    text = raw.decode("utf-8", errors="replace")
    # Handle "<topic> {...}" prefix style if used. Otherwise text is the raw JSON.
    if text.startswith(self._topic + " "):
        text = text[len(self._topic) + 1:]
    try:
        meta = json.loads(text)
    except json.JSONDecodeError:
        return None
    if not isinstance(meta, dict):
        return None
    if not meta.get("complete", False):
        return None
    
    frames_meta = meta.get("frames", {})
    if not isinstance(frames_meta, dict):
        return None
    
    decoded: Dict[str, CameraFrame] = {}
    for cam_name, frame_meta in frames_meta.items():
        if not isinstance(frame_meta, dict):
            continue
        if not frame_meta.get("valid", False):
            continue
        try:
            decoded[cam_name] = self._decode_frame(cam_name, frame_meta)
        except Exception:
            # If any frame fails to decode, drop the whole bundle.
            return None
    if not decoded:
        return None
    
    bundle = CameraBundle(
        bundle_seq=int(meta.get("bundle_seq", 0)),
        bundle_time_ns=int(meta.get("bundle_time_ns", 0)),
        hardware_synced=bool(meta.get("hardware_synced", False)),
        complete=True,
        received_monotonic=time.monotonic(),
        frames=decoded,
    )
    self._latest = bundle
    return bundle

def _decode_frame(self, camera_name: str, frame_meta: Dict[str, Any]) -> CameraFrame:
    import numpy as np
    
    shm_name = frame_meta["shm_name"]
    width = int(frame_meta["width"])
    height = int(frame_meta["height"])
    stride = int(frame_meta["stride_bytes"])
    fmt = str(frame_meta.get("format", "bgr8"))
    shm_offset = int(frame_meta["shm_offset"])
    size_bytes = int(frame_meta["size_bytes"])
    
    mm = self._shm_cache.get(shm_name)
    header_off = shm_offset - _SLOT_HEADER.size
    if header_off < 0 or shm_offset + size_bytes > len(mm):
        raise RuntimeError(f"shm offsets out of bounds for {camera_name}")
    
    # Seqlock read: retry up to N times.
    payload: Optional[bytes] = None
    for _ in range(1000):
        a = struct.unpack_from("<Q", mm, header_off)[0]
        if a & 1:
            continue
        vals = _SLOT_HEADER.unpack_from(mm, header_off)
        slot_size = vals[9]
        valid_flag = vals[10]
        if slot_size != size_bytes:
            raise RuntimeError(f"shm slot size mismatch for {camera_name}")
        copy = bytes(mm[shm_offset:shm_offset + size_bytes])
        b = vals[1]
        c = struct.unpack_from("<Q", mm, header_off)[0]
        if valid_flag and a == b == c and not (b & 1):
            payload = copy
            break
    if payload is None:
        raise RuntimeError(f"seqlock retry exhausted for {camera_name}")
    
    # Decode based on format
    arr = np.frombuffer(payload, dtype=np.uint8)
    expected = height * stride
    if arr.size != expected:
        raise RuntimeError(
            f"payload size {arr.size} != height*stride={expected} for {camera_name}"
        )
    
    if fmt in ("bgr8", "rgb8"):
        channels = 3
    elif fmt in ("bgra8", "rgba8"):
        channels = 4
    else:
        raise RuntimeError(f"unsupported camera format '{fmt}' for {camera_name}")
    expected_row_bytes = width * channels
    if stride < expected_row_bytes:
        raise RuntimeError(f"stride {stride} < width*channels={expected_row_bytes}")
    
    arr_2d = arr.reshape(height, stride)[:, :expected_row_bytes].reshape(height, width, channels)
    # Convert BGR → RGB for downstream consistency (most ML libraries expect RGB).
    if fmt == "bgr8":
        arr_2d = arr_2d[..., ::-1]
    elif fmt == "bgra8":
        arr_2d = arr_2d[..., [2, 1, 0, 3]]
    
    # Make a contiguous copy so the buffer survives mmap remap.
    pixels = np.ascontiguousarray(arr_2d)
    
    return CameraFrame(
        camera_name=camera_name,
        width=width,
        height=height,
        pixels=pixels,
        format=fmt,
        frame_number=int(frame_meta.get("frame_number", 0)),
        host_arrival_time_ns=int(frame_meta.get("host_arrival_time_ns", 0)),
        sensor_timestamp_ns=int(frame_meta.get("sensor_timestamp_ns", 0)),
    )
```

### 3. Config additions

`policy_runner/policy_runner/config.py` 에 `CameraConfig` 추가:

```python
@dataclass
class CameraConfig:
    enable: bool = False
    zmq_endpoint: str = "tcp://127.0.0.1:5600"
    bundle_topic: str = "camera.bundle"
    max_age_ms: float = 100.0
    expected_cameras: list[str] = field(default_factory=list)
    # When recording, missing-or-stale bundles cause the recorder to insert
    # zero-filled images with bundle_age_us = INT64_MAX as a marker.
    record_zero_on_missing: bool = True
```

`PolicyRunnerConfig.camera: CameraConfig = field(default_factory=CameraConfig)` 추가. YAML loader 에 `camera = CameraConfig(**raw.get("camera", {}))`.

`expected_cameras` 가 비어있으면 bundle 의 모든 frame 을 기록. 값이 들어있으면 그 카메라들만 기록 (없는 카메라는 zero-fill).

### 4. `Hdf5EpisodeRecorder` 확장

기존 `Hdf5EpisodeRecorder` 에 옵션 `camera_client: CameraBundleClient | None` 인자를 추가. None 이면 기존 동작 그대로 (state+action 만 기록).

`__init__` 시그니처:

```python
def __init__(
    self,
    output_dir,
    *,
    recording_rate_hz=30.0,
    camera_client: "CameraBundleClient | None" = None,
    expected_cameras: list[str] | None = None,
    record_zero_on_missing: bool = True,
):
    ...
    self.camera_client = camera_client
    self.expected_cameras = list(expected_cameras or [])
    self.record_zero_on_missing = bool(record_zero_on_missing)
```

`_EpisodeBuffer` 에 이미지 buffer 필드 추가:

```python
@dataclass
class _EpisodeBuffer:
    ...
    # 이미지 버퍼: per-camera 키, list of (H,W,C) uint8 arrays (numpy)
    images: Dict[str, list]    # cam_name -> list[np.ndarray]
    image_shapes: Dict[str, tuple[int, int, int]]   # locked at first observation
    bundle_seq: list[int]
    bundle_time_ns: list[int]
    bundle_age_us: list[int]
```

`start_episode` 의 buffer 초기화 시:
```python
images = {}
image_shapes = {}
```

`record_frame` 에서 camera client 있으면 latest bundle 가져오기:

```python
def record_frame(self, *, state_snapshot, action_packet, action_host_time_ns, action_seq):
    ep = self._current_episode
    if ep is None:
        raise RuntimeError("no active episode")
    now = time.monotonic()
    if (now - ep.last_appended_monotonic) < self._period_sec - 1e-6:
        return
    ep.last_appended_monotonic = now
    
    # ... (기존 state/action append 로직) ...
    
    # Camera frames
    if self.camera_client is not None:
        bundle = self.camera_client.poll(timeout_ms=0)
        if bundle is None:
            bundle = self.camera_client.latest()
        
        if bundle is not None:
            bundle_age_ns = time.time_ns() - bundle.bundle_time_ns
            ep.bundle_seq.append(int(bundle.bundle_seq))
            ep.bundle_time_ns.append(int(bundle.bundle_time_ns))
            ep.bundle_age_us.append(int(bundle_age_ns // 1000))
        else:
            ep.bundle_seq.append(0)
            ep.bundle_time_ns.append(0)
            ep.bundle_age_us.append(2**62)   # marker for missing
        
        cams = self.expected_cameras
        if not cams and bundle is not None:
            cams = sorted(bundle.frames.keys())
        
        for cam_name in cams:
            if bundle is not None and cam_name in bundle.frames:
                frame = bundle.frames[cam_name]
                pixels = frame.pixels
                # lock shape at first observation
                shape = (frame.height, frame.width, pixels.shape[2])
                ep.image_shapes.setdefault(cam_name, shape)
            else:
                # missing camera in this bundle: zero-fill if allowed, else skip episode
                if not self.record_zero_on_missing:
                    raise RuntimeError(
                        f"camera '{cam_name}' missing from bundle and "
                        "record_zero_on_missing is False"
                    )
                if cam_name not in ep.image_shapes:
                    # haven't seen this camera yet — skip, image_shapes still empty
                    continue
                import numpy as np
                shape = ep.image_shapes[cam_name]
                pixels = np.zeros(shape, dtype=np.uint8)
            
            ep.images.setdefault(cam_name, []).append(pixels)
```

`end_episode` 에 image dataset 생성:

```python
if ep.images:
    images_grp = obs.create_group("images")
    for cam_name, frames_list in ep.images.items():
        if not frames_list:
            continue
        # Stack
        stacked = np.stack(frames_list, axis=0)
        h, w, c = stacked.shape[1], stacked.shape[2], stacked.shape[3]
        images_grp.create_dataset(
            cam_name,
            data=stacked,
            dtype=np.uint8,
            chunks=(1, h, w, c),
            compression="gzip",
            compression_opts=1,
        )
    obs.create_dataset("bundle_seq", data=np.asarray(ep.bundle_seq, dtype=np.int64))
    obs.create_dataset("bundle_host_time_ns", data=np.asarray(ep.bundle_time_ns, dtype=np.int64))
    obs.create_dataset("bundle_age_us", data=np.asarray(ep.bundle_age_us, dtype=np.int64))
```

> 주의: images buffer 가 매우 클 수 있음. 720x1280x3 frame 30Hz × 60sec ≈ 5GB. memory 부담이지만 30sec episode 면 2.5GB 로 처리 가능. 장기 episode 는 streaming write 로 가야 하지만 이번 phase 에선 in-memory buffer 유지.

### 5. CLI 통합

`policy_runner/policy_runner/main.py` 의 `hdf5-record` subcommand 에 카메라 옵션 추가:

```python
hdf5_record.add_argument("--with-camera", action="store_true",
                         help="Subscribe to camera.bundle and record images")
hdf5_record.add_argument("--zmq-endpoint", default=None,
                         help="Override camera.zmq_endpoint")
```

handler 안에서:

```python
camera_client = None
if args.with_camera or config.camera.enable:
    from .camera_bundle_client import CameraBundleClient
    camera_client = CameraBundleClient(
        zmq_endpoint=args.zmq_endpoint or config.camera.zmq_endpoint,
        topic=config.camera.bundle_topic,
        max_age_ms=config.camera.max_age_ms,
    )

recorder = Hdf5EpisodeRecorder(
    output_dir,
    recording_rate_hz=rate_hz,
    camera_client=camera_client,
    expected_cameras=config.camera.expected_cameras,
    record_zero_on_missing=config.camera.record_zero_on_missing,
)

try:
    ...
finally:
    if camera_client is not None:
        camera_client.close()
```

## Tests

### `tests/test_camera_bundle_client.py`

Mock ZMQ publisher fixture 사용. `pyzmq` 와 `numpy` 가 설치 안 되어 있으면 skip.

```python
class CameraBundleClientTest(unittest.TestCase):
    
    def setUp(self):
        try:
            import zmq  # noqa
            import numpy  # noqa
        except ModuleNotFoundError:
            self.skipTest("camera extras not installed")
        # spin up an in-process ZMQ PUB socket on a free port
        ...
    
    def test_poll_returns_none_when_no_bundle_published(self):
        # publish nothing; poll(timeout_ms=10) returns None
        ...
    
    def test_poll_decodes_complete_bundle_with_one_camera(self):
        # write a fake SHM with a known image pattern + slot header
        # publish a bundle JSON with frame metadata pointing at the SHM
        # poll(timeout_ms=50) returns a CameraBundle with one CameraFrame
        # whose pixels match the pattern
        ...
    
    def test_poll_skips_incomplete_bundles(self):
        # publish a bundle with complete=false; poll returns None
        ...
    
    def test_poll_skips_invalid_frames(self):
        # publish a bundle with one frame valid=false; that frame is missing
        # from the decoded bundle (or whole bundle dropped if it was the only one)
        ...
    
    def test_bgr8_format_is_converted_to_rgb(self):
        # write SHM with known BGR pattern; assert returned pixels are RGB
        # (pixel[0,0,:] matches expected RGB ordering)
        ...
    
    def test_seqlock_retry_succeeds_after_concurrent_write(self):
        # simulate a write in progress by setting start_tag=1 (odd), then
        # transitioning to even with valid payload. reader should succeed
        # within retry limit.
        ...
    
    def test_is_fresh_returns_false_for_old_bundle(self):
        # manually set self._latest with bundle_time_ns far in the past
        # is_fresh() returns False
        ...
    
    def test_close_releases_zmq_and_shm(self):
        # client.close() does not raise; subsequent poll returns None
        # without crash
        ...
```

SHM fixture 작성 시 `mmap` 으로 `/dev/shm/policy_runner_test_<uuid>` 파일 만들고, slot header (`_SLOT_HEADER`) + payload 쓰기. tearDown 에서 unlink.

### `tests/test_hdf5_recording.py` 추가 케이스

```python
def test_recording_with_camera_client_writes_image_datasets(self):
    # fake camera client that returns a fixed bundle every poll
    # start_episode; record_frame several times; end_episode
    # open hdf5 file: assert /observations/images/<cam_name> dataset shape and dtype
    ...

def test_recording_zero_fills_when_bundle_missing(self):
    # camera client returns None every poll
    # start_episode (with at least one cam_name in expected_cameras and a prior shape)
    # record_frame; assert images/cam[0] is all zero and bundle_age_us == INT64_MAX marker
    ...

def test_recording_uses_first_observed_shape_for_zero_fills(self):
    # First record_frame: camera returns 360x640x3 frame.
    # Second record_frame: camera returns None.
    # The zero-fill in second frame should be 360x640x3 (not arbitrary).
    ...

def test_recording_camera_chunking_is_per_frame(self):
    # record 5 frames with a 360x640x3 camera.
    # open file: assert dataset chunks == (1, 360, 640, 3).
    ...
```

### Fake camera client for HDF5 tests

```python
class FakeCameraClient:
    def __init__(self, frames_by_call: list[Optional[CameraBundle]]):
        self._frames = list(frames_by_call)
        self._latest = None
    
    def poll(self, timeout_ms=0):
        if not self._frames:
            return self._latest
        bundle = self._frames.pop(0)
        if bundle is not None:
            self._latest = bundle
        return bundle
    
    def latest(self):
        return self._latest
    
    def is_fresh(self, bundle=None):
        return True
    
    def close(self):
        pass
```

## Do not change

- `camera_server` 측 어떤 코드도 변경 금지. metadata JSON 포맷 그대로 사용
- `camera_server/tools/read_latest_bundle.py` 변경 금지
- `Hdf5EpisodeRecorder` 의 state+action recording 로직 (prompt α 의 결과). 이번 prompt 는 이미지 추가만
- `EpisodeRecorder` (JSONL)
- 다른 action source
- server 측 어떤 코드도 변경 금지

## Acceptance

- `PYTHONPATH=policy_runner python3 -m unittest discover -s policy_runner/tests -p "test_*.py"` 모든 테스트 통과
- 기존 테스트 회귀 없음
- `test_camera_bundle_client.py` 최소 8개 케이스 통과
- `test_hdf5_recording.py` 의 camera 통합 케이스 최소 4개 통과
- `python -m policy_runner hdf5-record --with-camera --help` 가 `--with-camera`, `--zmq-endpoint` 옵션을 보여줌
- 실제 `camera_server` (또는 mock) 와 연동 시 생성된 HDF5 파일에 `/observations/images/<cam_name>` dataset 이 있고 dtype=uint8, chunks=(1,H,W,3), compression=gzip
- `policy_runner[camera]` extras 가 `pip install -e ".[camera]"` 로 설치 가능
