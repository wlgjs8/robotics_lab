from __future__ import annotations

import json
import mmap
import struct
import time
from collections import Counter
from dataclasses import dataclass, field
from typing import Any


_SLOT_HEADER = struct.Struct("<QQQQQQIIIIII")
_MISSING_BUNDLE_AGE_US = 2**62
# Color formats this client can decode. camera_server may also publish depth
# (z16) frames; those are skipped in poll() so they don't drop the whole bundle.
_COLOR_FORMATS = frozenset({"bgr8", "rgb8", "bgra8", "rgba8"})
_DEPTH_FORMATS = frozenset({"z16", "mono16"})  # opt-in (include_depth); decoded as uint16 (H,W)
_IR_FORMATS = frozenset({"y8", "mono8"})  # opt-in (include_ir); decoded as uint8 (H,W) IR-left/right


def bundle_clock_ns() -> int:
    """Now on the camera_server bundle clock.

    camera_server stamps bundle_time_ns with its configured server.clock
    (default monotonic_raw, see camera_server/src/core/clock.cpp). The stack
    runs camera_server on the same host (compose network_mode/ipc: host), so
    CLOCK_MONOTONIC_RAW is shared with the container and ages are directly
    comparable. Wall-clock time.time_ns() is NOT comparable to these stamps.
    """
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)


def resolve_frame(frames: dict[str, Any] | None, name: str) -> Any | None:
    """Look up a bundle frame by dataset camera name or raw bundle key.

    camera_server keys frames as '<camera>.<stream>' (stream_key() in
    camera_server/include/camera_server/core/types.hpp) while dataset and
    checkpoint camera names use '<camera>_<stream>' (e.g.
    'left_realsense_color'). Try the exact key first, then map the last '_'
    to '.'.
    """
    if not frames or not isinstance(frames, dict):
        return None
    frame = frames.get(name)
    if frame is not None:
        return frame
    if "." not in name:
        base, sep, stream = name.rpartition("_")
        if sep:
            return frames.get(f"{base}.{stream}")
    return None


@dataclass(frozen=True)
class CameraFrame:
    """Decoded camera frame with metadata."""

    camera_name: str
    width: int
    height: int
    pixels: Any
    format: str
    frame_number: int
    host_arrival_time_ns: int
    sensor_timestamp_ns: int


@dataclass(frozen=True)
class CameraBundle:
    """Latest complete bundle with decoded camera frames."""

    bundle_seq: int
    bundle_time_ns: int
    hardware_synced: bool
    complete: bool
    received_monotonic: float
    frames: dict[str, CameraFrame]
    sync_policy: str = ""
    max_time_diff_ms: float = 0.0
    drop_counters: dict[str, int] = field(default_factory=dict)


class _ShmCache:
    """Cache of mmap'd POSIX SHM files."""

    def __init__(self) -> None:
        self._name: str | None = None
        self._file = None
        self._mmap: mmap.mmap | None = None

    def get(self, name: str) -> mmap.mmap:
        if self._name == name and self._mmap is not None:
            return self._mmap
        self.close()
        path = "/dev/shm/" + name.lstrip("/")
        # Read-only: the ring is written by camera_server (often root inside
        # the container); opening r+b fails on the host with EACCES.
        self._file = open(path, "rb")
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
    """ZMQ SUB to camera.bundle plus shared-memory image reader."""

    def __init__(
        self,
        zmq_endpoint: str = "tcp://127.0.0.1:5600",
        topic: str = "camera.bundle",
        *,
        max_age_ms: float = 100.0,
        include_depth: bool = False,
        include_ir: bool = False,
        health_topic: str = "camera.health",
    ):
        try:
            import numpy  # noqa: F401
            import zmq  # noqa: F401
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "CameraBundleClient requires optional dependencies: "
                "install policy_runner with the camera extra"
            ) from exc
        import zmq

        self._zmq = zmq
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.SUB)
        self._sock.setsockopt_string(zmq.SUBSCRIBE, topic)
        # Receive buffer depth. ZMQ SUB drops the NEWEST bundles when the queue is
        # full (keep-oldest), so a small HWM makes poll() return a STALE bundle after
        # any polling gap: flow-infer only polls at chunk boundaries (every
        # chunk_execute_steps, ~400 ms at 12 steps x 33 ms) plus the blocking server
        # inference. With HWM=4 (~133 ms) the queue fills mid-gap and the freshest
        # surviving bundle ages past max_age_ms -> intermittent "stale" even though
        # camera_server streams a solid 30 fps. Sizing the buffer to cover ~2 s of
        # 30 fps bundles means nothing is dropped during a gap, so draining to the
        # tail yields a genuinely fresh frame. (CONFLATE would be ideal but does not
        # work with the multipart [topic, json] bundle messages.) Measured 2026-06-22:
        # HWM=4 -> 400 ms gap gives a 427 ms-old frame (stale); HWM>=16 -> ~30 ms.
        self._sock.setsockopt(zmq.RCVHWM, 60)
        self._sock.connect(zmq_endpoint)
        self._health_sock = self._ctx.socket(zmq.SUB)
        self._health_sock.setsockopt_string(zmq.SUBSCRIBE, health_topic)
        self._health_sock.setsockopt(zmq.RCVHWM, 8)
        self._health_sock.connect(zmq_endpoint)
        self._endpoint = zmq_endpoint
        self._topic = topic
        self._health_topic = str(health_topic)
        self._max_age_ms = float(max_age_ms)
        self._include_depth = bool(include_depth)  # opt-in z16 decode (collection only)
        self._include_ir = bool(include_ir)  # opt-in y8 IR decode
        self._shm_cache = _ShmCache()
        self._latest: CameraBundle | None = None
        self._latest_health: dict[str, Any] | None = None
        self._poll_outcomes: Counter[str] = Counter()
        self._last_poll: dict[str, Any] = {"outcome": "never_polled"}
        self._closed = False

    def _set_poll_outcome(self, outcome: str, **details: Any) -> None:
        self._poll_outcomes[str(outcome)] += 1
        self._last_poll = {
            "outcome": str(outcome),
            "monotonic_ns": time.monotonic_ns(),
            **details,
        }

    def _poll_health(self) -> None:
        latest: list[bytes] | None = None
        while True:
            try:
                latest = self._health_sock.recv_multipart(flags=self._zmq.NOBLOCK)
            except self._zmq.Again:
                break
        if latest is not None:
            decoded = self._decode_message_for_topic(latest, self._health_topic)
            if decoded is not None:
                self._latest_health = decoded

    def poll(self, timeout_ms: int = 0) -> CameraBundle | None:
        """Receive and decode the latest complete camera bundle if available."""

        if self._closed:
            return None
        self._poll_health()
        poller = self._zmq.Poller()
        poller.register(self._sock, self._zmq.POLLIN)
        socks = dict(poller.poll(timeout=int(timeout_ms)))
        if self._sock not in socks:
            self._set_poll_outcome("no_message")
            return None
        try:
            parts = self._sock.recv_multipart(flags=self._zmq.NOBLOCK)
        except self._zmq.Again:
            self._set_poll_outcome("recv_race")
            return None
        # ZMQ subscriptions are prefix matches. A legacy `camera.bundle`
        # subscriber therefore also receives `camera.bundle.policy` and
        # `camera.bundle.stereo`; retain the newest exact-topic message only.
        meta = self._decode_message(parts)
        while True:
            try:
                parts = self._sock.recv_multipart(flags=self._zmq.NOBLOCK)
            except self._zmq.Again:
                break
            candidate = self._decode_message(parts)
            if candidate is not None:
                meta = candidate
        if meta is None:
            self._set_poll_outcome("no_exact_topic_message")
            return None
        if not bool(meta.get("complete", False)):
            self._set_poll_outcome("incomplete_bundle", bundle_seq=int(meta.get("bundle_seq", 0) or 0))
            return None
        frames_meta = meta.get("frames", {})
        if not isinstance(frames_meta, dict):
            self._set_poll_outcome("malformed_frames")
            return None

        decoded: dict[str, CameraFrame] = {}
        for cam_name, frame_meta in frames_meta.items():
            if not isinstance(frame_meta, dict) or not frame_meta.get("valid", False):
                continue
            # Skip non-color streams (e.g. depth z16) instead of dropping the
            # whole bundle — camera_server may publish color+depth, but this
            # client decodes color only (UNLESS include_depth: then z16 is kept
            # as a uint16 frame, for collection/RGB-D capture).
            fmt = str(frame_meta.get("format", "")).lower()
            if (fmt not in _COLOR_FORMATS
                    and not (self._include_depth and fmt in _DEPTH_FORMATS)
                    and not (self._include_ir and fmt in _IR_FORMATS)):
                continue
            try:
                decoded[str(cam_name)] = self._decode_frame(str(cam_name), frame_meta)
            except Exception as exc:
                self._set_poll_outcome(
                    "frame_decode_error",
                    camera_name=str(cam_name),
                    error_type=type(exc).__name__,
                )
                return None
        if not decoded:
            self._set_poll_outcome("no_decodable_frames")
            return None

        bundle = CameraBundle(
            bundle_seq=int(meta.get("bundle_seq", 0) or 0),
            bundle_time_ns=int(meta.get("bundle_time_ns", 0) or 0),
            hardware_synced=bool(meta.get("hardware_synced", False)),
            complete=True,
            received_monotonic=time.monotonic(),
            frames=decoded,
            sync_policy=str(meta.get("sync_policy", "") or ""),
            max_time_diff_ms=float(meta.get("max_time_diff_ms", 0.0) or 0.0),
            drop_counters={
                str(key): int(value)
                for key, value in (meta.get("drop_counters", {}) or {}).items()
                if isinstance(value, (int, float))
            },
        )
        self._poll_health()
        self._latest = bundle
        self._set_poll_outcome(
            "ok",
            bundle_seq=bundle.bundle_seq,
            frame_count=len(bundle.frames),
        )
        return bundle

    def latest(self) -> CameraBundle | None:
        return self._latest

    def is_fresh(self, bundle: CameraBundle | None = None) -> bool:
        active = bundle if bundle is not None else self._latest
        if active is None:
            return False
        age_ns = bundle_clock_ns() - active.bundle_time_ns
        return age_ns >= 0 and age_ns < self._max_age_ms * 1_000_000

    def diagnostics_snapshot(self) -> dict[str, Any]:
        latest = self._latest
        age_ms: float | None = None
        if latest is not None:
            age_ns = bundle_clock_ns() - latest.bundle_time_ns
            if age_ns >= 0:
                age_ms = age_ns / 1_000_000.0
        return {
            "endpoint": self._endpoint,
            "topic": self._topic,
            "last_poll": dict(self._last_poll),
            "poll_outcome_counts": dict(sorted(self._poll_outcomes.items())),
            "latest_bundle": None
            if latest is None
            else {
                "bundle_seq": latest.bundle_seq,
                "bundle_time_ns": latest.bundle_time_ns,
                "age_ms": age_ms,
                "fresh": self.is_fresh(latest),
                "hardware_synced": latest.hardware_synced,
                "sync_policy": latest.sync_policy,
                "max_time_diff_ms": latest.max_time_diff_ms,
                "frame_names": sorted(latest.frames),
                "drop_counters": dict(latest.drop_counters),
            },
            "camera_health": None if self._latest_health is None else dict(self._latest_health),
        }

    def close(self) -> None:
        if self._closed:
            return
        self._sock.close(linger=0)
        self._health_sock.close(linger=0)
        self._shm_cache.close()
        self._closed = True

    def _decode_message(self, parts: list[bytes]) -> dict[str, Any] | None:
        return self._decode_message_for_topic(parts, self._topic)

    @staticmethod
    def _decode_message_for_topic(parts: list[bytes], topic_name: str) -> dict[str, Any] | None:
        if len(parts) == 2:
            topic, payload = parts
            if topic.decode("utf-8", errors="replace") != topic_name:
                return None
            text = payload.decode("utf-8", errors="replace")
        elif len(parts) == 1:
            text = parts[0].decode("utf-8", errors="replace")
            if text.startswith(topic_name + " "):
                text = text[len(topic_name) + 1:]
        else:
            return None
        try:
            meta = json.loads(text)
        except json.JSONDecodeError:
            return None
        return meta if isinstance(meta, dict) else None

    def _decode_frame(self, camera_name: str, frame_meta: dict[str, Any]) -> CameraFrame:
        import numpy as np

        shm_name = str(frame_meta["shm_name"])
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

        payload: bytes | None = None
        for _ in range(1000):
            start = struct.unpack_from("<Q", mm, header_off)[0]
            if start & 1:
                continue
            vals = _SLOT_HEADER.unpack_from(mm, header_off)
            slot_size = vals[9]
            valid_flag = vals[10]
            if slot_size != size_bytes:
                raise RuntimeError(f"shm slot size mismatch for {camera_name}")
            copy = bytes(mm[shm_offset:shm_offset + size_bytes])
            end = vals[1]
            reread_start = struct.unpack_from("<Q", mm, header_off)[0]
            if valid_flag and start == end == reread_start and not (end & 1):
                payload = copy
                break
        if payload is None:
            raise RuntimeError(f"seqlock retry exhausted for {camera_name}")

        arr = np.frombuffer(payload, dtype=np.uint8)
        expected = height * stride
        if arr.size != expected:
            raise RuntimeError(f"payload size {arr.size} != height*stride={expected} for {camera_name}")
        if fmt in _DEPTH_FORMATS:  # z16/mono16 -> uint16 (H,W), no channel/BGR handling
            if stride % 2 != 0:
                raise RuntimeError(f"odd depth stride {stride} for {camera_name}")
            depth = np.frombuffer(payload, dtype=np.uint16).reshape(height, stride // 2)[:, :width]
            return CameraFrame(
                camera_name=camera_name, width=width, height=height,
                pixels=np.ascontiguousarray(depth), format=fmt,
                frame_number=int(frame_meta.get("frame_number", 0) or 0),
                host_arrival_time_ns=int(frame_meta.get("host_arrival_time_ns", 0) or 0),
                sensor_timestamp_ns=int(frame_meta.get("sensor_timestamp_ns", 0) or 0),
            )
        if fmt in _IR_FORMATS:  # y8/mono8 -> uint8 (H,W) IR
            ir = arr.reshape(height, stride)[:, :width]
            return CameraFrame(
                camera_name=camera_name, width=width, height=height,
                pixels=np.ascontiguousarray(ir), format=fmt,
                frame_number=int(frame_meta.get("frame_number", 0) or 0),
                host_arrival_time_ns=int(frame_meta.get("host_arrival_time_ns", 0) or 0),
                sensor_timestamp_ns=int(frame_meta.get("sensor_timestamp_ns", 0) or 0),
            )
        if fmt in {"bgr8", "rgb8"}:
            channels = 3
        elif fmt in {"bgra8", "rgba8"}:
            channels = 4
        else:
            raise RuntimeError(f"unsupported camera format '{fmt}' for {camera_name}")
        expected_row_bytes = width * channels
        if stride < expected_row_bytes:
            raise RuntimeError(f"stride {stride} < width*channels={expected_row_bytes}")

        pixels = arr.reshape(height, stride)[:, :expected_row_bytes].reshape(height, width, channels)
        if fmt == "bgr8":
            pixels = pixels[..., ::-1]
        elif fmt == "bgra8":
            pixels = pixels[..., [2, 1, 0, 3]]
        pixels = np.ascontiguousarray(pixels)

        return CameraFrame(
            camera_name=camera_name,
            width=width,
            height=height,
            pixels=pixels,
            format=fmt,
            frame_number=int(frame_meta.get("frame_number", 0) or 0),
            host_arrival_time_ns=int(frame_meta.get("host_arrival_time_ns", 0) or 0),
            sensor_timestamp_ns=int(frame_meta.get("sensor_timestamp_ns", 0) or 0),
        )
