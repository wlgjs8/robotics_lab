"""Read-only head-camera (D435 color) preview for the operator GUI.

The wrist previews come from the quality diagnostics path (`camera_quality.py`),
which is wired to the two per-wrist bundle groups and analyzes every frame.  The
head camera needs no analysis — the operator only wants to *see* the scene — so
this module is a deliberately thin second consumer of the same camera_server
transport: ZMQ metadata on 5600 plus the POSIX shared-memory ring.

It is display-only: no robot command, no CSV, no image ever written to disk, and
never an input to a motion or safety decision.  The head rig is optional
(`make cam-up-wrists` runs without the D435), so a missing topic is a normal
`waiting` state rather than an error.

Cost control: the head color stream is 1280x720 at 30 fps.  Decoding every frame
would cost ~83 MB/s of memcpy for a panel that repaints at 5 Hz, so the receiver
copies a frame out of shared memory only while the operator has the preview
enabled, and only at `DECODE_INTERVAL_SEC`.
"""
from __future__ import annotations

import json
import threading
import time
from typing import Any, Mapping

import numpy as np

from .camera_quality import _ShmCache, _read_rgb_frame


DEFAULT_HEAD_ENDPOINT = "tcp://127.0.0.1:5600"
# Head rig bundle group (`bundle_groups.stereo` in the camera_server rig config).
# It carries head.color + both IR streams and is published independently of the
# wrist groups, so a stalled wrist cannot suppress the head view.
DEFAULT_HEAD_TOPIC = "camera.bundle.stereo"
DEFAULT_HEAD_STREAM = "head.color"
# Preview width in the GUI panel; the 1280-wide source is decimated to roughly
# this before it is JPEG-encoded for the browser.
PREVIEW_WIDTH = 480
# 5 Hz, matching the wrist preview repaint in `_update_camera_quality`.
DECODE_INTERVAL_SEC = 0.2
STALE_SEC = 1.0


def _decimate(pixels: np.ndarray, target_width: int = PREVIEW_WIDTH) -> np.ndarray:
    """Nearest-neighbour decimation to ~`target_width` (no OpenCV dependency)."""

    height, width = pixels.shape[:2]
    if width <= target_width or target_width <= 0:
        return np.ascontiguousarray(pixels)
    step = max(1, int(round(width / float(target_width))))
    return np.ascontiguousarray(pixels[::step, ::step])


class HeadPreviewStore:
    """Latest head preview frame plus receiver status, shared with the GUI loop."""

    def __init__(self, *, topic: str = DEFAULT_HEAD_TOPIC, stream: str = DEFAULT_HEAD_STREAM) -> None:
        self._lock = threading.Lock()
        self._topic = str(topic)
        self._stream = str(stream)
        self._enabled = False
        self._preview: np.ndarray | None = None
        self._source_size: tuple[int, int] | None = None
        self._camera_name = ""
        self._bundle_seq = 0
        self._frame_age_ms: float | None = None
        self._received_monotonic: float | None = None
        self._status = "waiting"
        self._error = ""

    @property
    def topic(self) -> str:
        return self._topic

    @property
    def stream(self) -> str:
        return self._stream

    @property
    def enabled(self) -> bool:
        with self._lock:
            return self._enabled

    def set_enabled(self, enabled: bool) -> None:
        """Operator gate. While off, the receiver skips shared-memory reads."""

        with self._lock:
            self._enabled = bool(enabled)

    def set_status(self, status: str, error: str = "") -> None:
        with self._lock:
            self._status = str(status)
            self._error = str(error)

    def update(
        self,
        preview: np.ndarray,
        *,
        source_size: tuple[int, int],
        camera_name: str,
        bundle_seq: int,
        frame_age_ms: float | None,
        received_monotonic: float,
    ) -> None:
        with self._lock:
            self._preview = preview
            self._source_size = source_size
            self._camera_name = str(camera_name)
            self._bundle_seq = int(bundle_seq)
            self._frame_age_ms = frame_age_ms
            self._received_monotonic = float(received_monotonic)
            self._status = "live"
            self._error = ""

    def preview(self) -> np.ndarray | None:
        with self._lock:
            return self._preview

    def status_text(self, *, now: float | None = None) -> str:
        timestamp = time.monotonic() if now is None else float(now)
        with self._lock:
            status = self._status
            error = self._error
            received = self._received_monotonic
            source_size = self._source_size
            preview = self._preview
            camera_name = self._camera_name
            age_ms = self._frame_age_ms
            topic = self._topic
            stream = self._stream
            enabled = self._enabled
        if status in {"disabled", "unavailable"}:
            return f"{status}: {error}" if error else status
        if received is None or preview is None:
            hint = "" if enabled else " (표시 토글 off)"
            detail = f" · {error}" if error else ""
            return f"{status}{hint} · topic {topic}/{stream}{detail}"
        idle = timestamp - received
        size_text = ""
        if source_size is not None:
            size_text = (
                f" {source_size[0]}x{source_size[1]}"
                f"→{preview.shape[1]}x{preview.shape[0]}"
            )
        age_text = "N/A" if age_ms is None else f"{age_ms:.1f} ms"
        state = "stale" if idle > STALE_SEC else status
        detail = f" · {error}" if error else ""
        return (
            f"{state} · {camera_name or stream}{size_text} · "
            f"frame age {age_text} · {idle:.1f} s ago{detail}"
        )


class HeadPreviewReceiver:
    """ZMQ SUB on the head bundle group; copies a frame only when enabled."""

    def __init__(
        self,
        store: HeadPreviewStore,
        *,
        endpoint: str = DEFAULT_HEAD_ENDPOINT,
        decode_interval_sec: float = DECODE_INTERVAL_SEC,
    ) -> None:
        self.store = store
        self.endpoint = str(endpoint)
        self.decode_interval_sec = float(decode_interval_sec)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        try:
            import zmq  # noqa: F401
        except ModuleNotFoundError as exc:
            self.store.set_status(
                "disabled",
                f"missing dependency {exc.name}; install rb_gui camera dependencies",
            )
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="rb-head-preview-receiver",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _run(self) -> None:
        import zmq

        cache = _ShmCache()
        context = zmq.Context.instance()
        sock = context.socket(zmq.SUB)
        # Small queue: only the newest frame matters for a 5 Hz viewer, and a deep
        # keep-oldest SUB queue would hand us stale frames after any idle gap.
        sock.setsockopt(zmq.RCVHWM, 4)
        sock.setsockopt_string(zmq.SUBSCRIBE, self.store.topic)
        sock.connect(self.endpoint)
        self.store.set_status("waiting")
        last_decode = float("-inf")
        try:
            while not self._stop.is_set():
                if not sock.poll(timeout=200, flags=zmq.POLLIN):
                    continue
                try:
                    parts = sock.recv_multipart(flags=zmq.NOBLOCK)
                except zmq.Again:
                    continue
                if len(parts) != 2:
                    continue
                if parts[0].decode("utf-8", errors="replace") != self.store.topic:
                    continue
                if not self.store.enabled:
                    continue
                now = time.monotonic()
                if now - last_decode < self.decode_interval_sec:
                    continue
                try:
                    document = json.loads(parts[1].decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    continue
                meta = _head_frame_meta(document, self.store.stream)
                if meta is None:
                    continue
                try:
                    pixels = _read_rgb_frame(cache, meta)
                except Exception as exc:  # keep the GUI and wrist previews alive
                    self.store.set_status(
                        "degraded",
                        f"head frame read {type(exc).__name__}: {exc}",
                    )
                    continue
                last_decode = now
                self.store.update(
                    _decimate(pixels),
                    source_size=(int(pixels.shape[1]), int(pixels.shape[0])),
                    camera_name=str(meta.get("camera_name", "")),
                    bundle_seq=int(document.get("bundle_seq", 0) or 0),
                    frame_age_ms=_frame_age_ms(meta),
                    received_monotonic=now,
                )
        except Exception as exc:
            self.store.set_status("unavailable", f"{type(exc).__name__}: {exc}")
        finally:
            sock.close(linger=0)
            cache.close()


def _head_frame_meta(document: Any, stream: str) -> Mapping[str, Any] | None:
    if (
        not isinstance(document, Mapping)
        or document.get("schema") != "camera_server.bundle.v1"
        or document.get("complete") is not True
    ):
        return None
    frames = document.get("frames")
    if not isinstance(frames, Mapping):
        return None
    meta = frames.get(stream)
    if not isinstance(meta, Mapping) or meta.get("valid") is not True:
        return None
    return meta


def _frame_age_ms(meta: Mapping[str, Any]) -> float | None:
    host_arrival = int(meta.get("host_arrival_time_ns", 0) or 0)
    if host_arrival <= 0:
        return None
    age_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW) - host_arrival
    return None if age_ns < 0 else age_ns / 1e6
