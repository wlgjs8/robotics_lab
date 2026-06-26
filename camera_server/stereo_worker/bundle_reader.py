"""camera_server 번들(camera.bundle ZMQ + /dev/shm 링)에서 선택 스트림을 읽는 최소 리더.

policy_runner/camera_bundle_client.py 의 seqlock 읽기 로직을 워커 자체 포함용으로 옮긴 것.
색(rgb8/bgr8) 및 IR(y8/mono8) 디코드 지원. 같은 컨테이너(ipc/net host)에서 동작.
"""
from __future__ import annotations
import json
import mmap
import os
import struct
from dataclasses import dataclass
import numpy as np

_SLOT_HEADER = struct.Struct("<QQQQQQIIIIII")
_COLOR = {"rgb8", "bgr8", "bgra8", "rgba8"}
_IR = {"y8", "mono8"}


@dataclass
class Frame:
    key: str
    width: int
    height: int
    format: str
    pixels: np.ndarray
    frame_number: int


class _ShmCache:
    def __init__(self):
        self._m: dict[str, mmap.mmap] = {}

    def get(self, shm_name: str) -> mmap.mmap:
        if shm_name not in self._m:
            path = "/dev/shm/" + shm_name.lstrip("/")
            fd = os.open(path, os.O_RDONLY)
            try:
                sz = os.fstat(fd).st_size
                self._m[shm_name] = mmap.mmap(fd, sz, prot=mmap.PROT_READ)
            finally:
                os.close(fd)
        return self._m[shm_name]

    def close(self):
        for m in self._m.values():
            try: m.close()
            except Exception: pass
        self._m.clear()


class BundleReader:
    def __init__(self, endpoint="tcp://127.0.0.1:5600", topic="camera.bundle", rcvhwm=8):
        import zmq
        self._zmq = zmq
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.SUB)
        self._sock.setsockopt_string(zmq.SUBSCRIBE, topic)
        self._sock.setsockopt(zmq.RCVHWM, rcvhwm)
        self._sock.setsockopt(zmq.CONFLATE, 0)
        self._sock.connect(endpoint)
        self._topic = topic
        self._shm = _ShmCache()

    def poll(self, want_keys, timeout_ms=200):
        """returns {key: Frame} for matching keys in the latest bundle, or {}."""
        if not self._sock.poll(timeout_ms):
            return {}
        parts = self._sock.recv_multipart()
        meta = self._parse(parts)
        if not meta:
            return {}
        out = {}
        for key, fm in (meta.get("frames") or {}).items():
            if key not in want_keys:
                continue
            try:
                out[key] = self._decode(key, fm)
            except Exception:
                continue
        return out

    def _parse(self, parts):
        if len(parts) == 2:
            topic, payload = parts
            if topic.decode("utf-8", "replace") != self._topic:
                return None
            text = payload.decode("utf-8", "replace")
        elif len(parts) == 1:
            text = parts[0].decode("utf-8", "replace")
            if text.startswith(self._topic + " "):
                text = text[len(self._topic) + 1:]
        else:
            return None
        try:
            m = json.loads(text)
        except json.JSONDecodeError:
            return None
        return m if isinstance(m, dict) else None

    def _decode(self, key, fm) -> Frame:
        width = int(fm["width"]); height = int(fm["height"]); stride = int(fm["stride_bytes"])
        fmt = str(fm.get("format", "")).lower()
        off = int(fm["shm_offset"]); size = int(fm["size_bytes"])
        mm = self._shm.get(str(fm["shm_name"]))
        hoff = off - _SLOT_HEADER.size
        if hoff < 0 or off + size > len(mm):
            raise RuntimeError("shm out of bounds")
        payload = None
        for _ in range(1000):
            start = struct.unpack_from("<Q", mm, hoff)[0]
            if start & 1:
                continue
            vals = _SLOT_HEADER.unpack_from(mm, hoff)
            if vals[9] != size:
                raise RuntimeError("slot size mismatch")
            copy = bytes(mm[off:off + size])
            end = vals[1]
            reread = struct.unpack_from("<Q", mm, hoff)[0]
            if vals[10] and start == end == reread and not (end & 1):
                payload = copy
                break
        if payload is None:
            raise RuntimeError("seqlock retry exhausted")
        arr = np.frombuffer(payload, dtype=np.uint8)
        if fmt in _IR:
            px = arr.reshape(height, stride)[:, :width]
        elif fmt in {"rgb8", "bgr8"}:
            px = arr.reshape(height, stride)[:, :width * 3].reshape(height, width, 3)
            if fmt == "bgr8":
                px = px[..., ::-1]
        else:
            raise RuntimeError(f"unsupported format {fmt}")
        return Frame(key, width, height, fmt, np.ascontiguousarray(px),
                     int(fm.get("frame_number", 0) or 0))

    def close(self):
        self._sock.close(linger=0)
        self._shm.close()
