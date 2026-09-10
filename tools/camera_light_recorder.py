#!/usr/bin/env python3
"""Read-only wrist-camera lighting recorder: scene photometry over the day.

Why this exists: the D405 rig runs auto-exposure at 90 fps, where the AE loop is
railed at its exposure ceiling (measured constant `actual_exposure_us=9947`,
`gain_level=0` across every tracked `logs/camera_quality/*.csv` since 2026-09-02).
A railed AE cannot compensate ambient light, so room illumination lands directly
in the policy's input pixels. This tool logs that photometry continuously and
keeps a bounded set of full-resolution reference frames, so a "morning works,
afternoon degrades" claim becomes a measurement instead of an impression.

It mirrors `rb_gui/rb_servo_gui/camera_quality.py`'s posture: it subscribes to the
existing camera_server ZMQ metadata and reads the POSIX shared-memory rings.  It
never publishes a robot command, never opens a camera device, and never writes to
camera_server's memory.

Usage (morning "good" reference, images at 1 Hz, metrics at 5 Hz, 8 h budget):

    .venv/bin/python tools/camera_light_recorder.py --label am_good \
        --duration-min 480 --image-hz 1.0 --metrics-hz 5.0 --max-gb 12

Outputs under `logs/light/<label>_<YYYYmmdd_HHMMSS>/`:
  session.json  run provenance (topic, cameras, budget, camera.health snapshot)
  metrics.csv   one row per sampled bundle per camera (photometry + AE metadata)
  images/       `<idx>_<HHMMSS>_<side>.jpg` full-resolution RGB reference frames
"""
from __future__ import annotations

import argparse
import csv
import json
import mmap
import os
import shutil
import signal
import struct
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

# Same seqlock slot header as camera_server/tools/read_latest_bundle.py.
_SLOT_HEADER = struct.Struct("<QQQQQQIIIIII")
_COLOR_FORMATS = frozenset({"rgb8", "bgr8", "rgba8", "bgra8"})
_DEFAULT_CAMERAS = ("left_realsense", "right_realsense")
_SIDE_OF = {"left_realsense": "left", "right_realsense": "right"}

_METRIC_FIELDS = [
    "schema",
    "sample_index",
    "wall_iso",
    "wall_epoch_s",
    "hour_local",
    "monotonic_s",
    "arm",
    "camera_name",
    "serial",
    "bundle_seq",
    "frame_number",
    "bundle_age_ms",
    "max_time_diff_ms",
    "actual_exposure_us",
    "gain_level",
    "auto_exposure",
    "depth_gain_level",
    "lum_mean",
    "lum_p01",
    "lum_p05",
    "lum_p50",
    "lum_p95",
    "lum_p99",
    "lum_mean_center",
    "r_mean",
    "g_mean",
    "b_mean",
    "clip_hi_frac",
    "clip_lo_frac",
    "focus_gradient_energy",
    "image_file",
]
METRICS_SCHEMA = "robotics_lab.camera_light.v1"


def _shm_path(name: str) -> str:
    return "/dev/shm/" + name.lstrip("/")


def read_slot(mm: mmap.mmap, frame: dict[str, Any], retries: int = 1000) -> bytes:
    """Seqlock-validated copy of one ring slot (reference reader semantics)."""
    header_off = int(frame["shm_offset"]) - _SLOT_HEADER.size
    size = int(frame["size_bytes"])
    payload_off = int(frame["shm_offset"])
    if header_off < 0 or payload_off < 0 or size < 0:
        raise RuntimeError(f"invalid shm metadata for {frame.get('ring_name')}")
    if header_off + _SLOT_HEADER.size > len(mm) or payload_off + size > len(mm):
        raise RuntimeError(f"shm read out of bounds for {frame.get('ring_name')}")
    for _ in range(retries):
        a = struct.unpack_from("<Q", mm, header_off)[0]
        if a & 1:
            continue
        vals = _SLOT_HEADER.unpack_from(mm, header_off)
        if vals[9] != size:
            raise RuntimeError(f"bundle size does not match shm slot for {frame.get('ring_name')}")
        payload = mm[payload_off : payload_off + size]
        b = vals[1]
        c = struct.unpack_from("<Q", mm, header_off)[0]
        if vals[10] and a == b == c and not (b & 1):
            return payload
    raise RuntimeError(f"seqlock failed for {frame.get('ring_name')}")


class ShmCache:
    """Lazily maps each shm ring by the name embedded in bundle metadata."""

    def __init__(self) -> None:
        self._maps: dict[str, tuple[Any, mmap.mmap]] = {}

    def get(self, name: str) -> mmap.mmap:
        cached = self._maps.get(name)
        if cached is not None:
            return cached[1]
        handle = open(_shm_path(name), "rb")
        mapping = mmap.mmap(handle.fileno(), 0, access=mmap.ACCESS_READ)
        self._maps[name] = (handle, mapping)
        return mapping

    def close(self) -> None:
        for handle, mapping in self._maps.values():
            mapping.close()
            handle.close()
        self._maps.clear()


def decode_color(payload: bytes, frame: dict[str, Any]) -> np.ndarray:
    """Bytes -> HxWx3 RGB, honouring the slot stride and the declared format."""
    fmt = str(frame.get("format", "")).lower()
    if fmt not in _COLOR_FORMATS:
        raise ValueError(f"not a color format: {fmt}")
    height = int(frame["height"])
    width = int(frame["width"])
    stride = int(frame["stride_bytes"])
    channels = 4 if fmt in {"rgba8", "bgra8"} else 3
    raw = np.frombuffer(payload, dtype=np.uint8)
    if raw.size < height * stride:
        raise ValueError("payload shorter than height*stride")
    image = raw[: height * stride].reshape(height, stride)[:, : width * channels]
    image = image.reshape(height, width, channels)[:, :, :3]
    if fmt.startswith("bgr"):
        image = image[:, :, ::-1]
    return image


def photometry(rgb: np.ndarray, stride: int = 2, center_frac: float = 0.65) -> dict[str, float]:
    """Luminance / colour-cast / clipping / focus indicators on a subsampled frame."""
    sample = np.asarray(rgb[::stride, ::stride, :3], dtype=np.float32)
    lum = 0.2126 * sample[..., 0] + 0.7152 * sample[..., 1] + 0.0722 * sample[..., 2]
    p01, p05, p50, p95, p99 = np.percentile(lum, (1.0, 5.0, 50.0, 95.0, 99.0))
    height, width = lum.shape
    cy, cx = int(height * (1.0 - center_frac) / 2.0), int(width * (1.0 - center_frac) / 2.0)
    center = lum[cy : height - cy, cx : width - cx] if height > 2 * cy and width > 2 * cx else lum
    dx = np.diff(lum, axis=1)
    dy = np.diff(lum, axis=0)
    focus = float(np.mean(dx * dx)) + float(np.mean(dy * dy))
    return {
        "lum_mean": float(np.mean(lum)),
        "lum_p01": float(p01),
        "lum_p05": float(p05),
        "lum_p50": float(p50),
        "lum_p95": float(p95),
        "lum_p99": float(p99),
        "lum_mean_center": float(np.mean(center)),
        "r_mean": float(np.mean(sample[..., 0])),
        "g_mean": float(np.mean(sample[..., 1])),
        "b_mean": float(np.mean(sample[..., 2])),
        "clip_hi_frac": float(np.mean(sample >= 250.0)),
        "clip_lo_frac": float(np.mean(sample <= 8.0)),
        "focus_gradient_energy": focus,
    }


def _dir_bytes(path: Path) -> int:
    total = 0
    for entry in path.rglob("*"):
        if entry.is_file():
            total += entry.stat().st_size
    return total


class Recorder:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        label = args.label.strip() or "session"
        self.out_dir = Path(args.out_dir).expanduser() if args.out_dir else Path("logs/light") / f"{label}_{stamp}"
        self.image_dir = self.out_dir / "images"
        self.cameras = tuple(name.strip() for name in args.cameras.split(",") if name.strip())
        self.shm = ShmCache()
        self.metrics_period = 1.0 / max(1e-6, args.metrics_hz)
        self.image_period = 0.0 if args.image_hz <= 0.0 else 1.0 / args.image_hz
        self.sample_index = 0
        self.image_index = 0
        self.written_rows = 0
        self.stop = False

    def _open_sockets(self):
        import zmq

        ctx = zmq.Context()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt_string(zmq.SUBSCRIBE, self.args.topic)
        sock.setsockopt(zmq.RCVHWM, 16)
        sock.connect(self.args.metadata)
        health = ctx.socket(zmq.SUB)
        health.setsockopt_string(zmq.SUBSCRIBE, "camera.health")
        health.setsockopt(zmq.RCVHWM, 4)
        health.connect(self.args.metadata)
        return zmq, ctx, sock, health

    def _latest_bundle(self, zmq, sock, timeout_ms: int) -> dict[str, Any] | None:
        poller = zmq.Poller()
        poller.register(sock, zmq.POLLIN)
        if not dict(poller.poll(timeout_ms)):
            return None
        latest = None
        while True:
            try:
                topic, payload = sock.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            if topic.decode() == self.args.topic:
                latest = json.loads(payload.decode())
        return latest

    def _write_session(self, health: dict[str, Any] | None) -> None:
        session = {
            "schema": METRICS_SCHEMA,
            "started_iso": datetime.now().astimezone().isoformat(timespec="seconds"),
            "label": self.args.label,
            "metadata_endpoint": self.args.metadata,
            "topic": self.args.topic,
            "cameras": list(self.cameras),
            "metrics_hz": self.args.metrics_hz,
            "image_hz": self.args.image_hz,
            "image_format": self.args.image_format,
            "jpeg_quality": self.args.jpeg_quality,
            "duration_min": self.args.duration_min,
            "max_gb": self.args.max_gb,
            "note": self.args.note,
            "camera_health": health,
        }
        (self.out_dir / "session.json").write_text(json.dumps(session, indent=2) + "\n")

    def run(self) -> int:
        import cv2

        zmq, ctx, sock, health_sock = self._open_sockets()
        self.image_dir.mkdir(parents=True, exist_ok=True)
        deadline = time.monotonic() + self.args.duration_min * 60.0
        next_metric = 0.0
        next_image = 0.0
        max_bytes = int(self.args.max_gb * (1 << 30))
        health_seen: dict[str, Any] | None = None
        session_written = False
        budget_stop = ""

        with (self.out_dir / "metrics.csv").open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=_METRIC_FIELDS)
            writer.writeheader()
            while not self.stop and time.monotonic() < deadline:
                try:
                    topic, payload = health_sock.recv_multipart(flags=zmq.NOBLOCK)
                    health_seen = json.loads(payload.decode())
                except zmq.Again:
                    pass
                if not session_written:
                    self._write_session(health_seen)
                    session_written = True

                now = time.monotonic()
                if now < next_metric:
                    time.sleep(min(0.02, next_metric - now))
                    continue
                bundle = self._latest_bundle(zmq, sock, timeout_ms=500)
                if bundle is None or not bundle.get("complete"):
                    continue
                next_metric = time.monotonic() + self.metrics_period
                save_image = self.image_period > 0.0 and time.monotonic() >= next_image
                rows = self._sample(bundle, save_image, cv2)
                if not rows:
                    continue
                if save_image:
                    next_image = time.monotonic() + self.image_period
                    self.image_index += 1
                for row in rows:
                    writer.writerow(row)
                self.written_rows += len(rows)
                if self.written_rows % 200 < len(rows):
                    handle.flush()
                    used = _dir_bytes(self.out_dir)
                    free = shutil.disk_usage(self.out_dir).free
                    if used > max_bytes:
                        budget_stop = f"max_gb {self.args.max_gb} reached ({used / (1 << 30):.2f} GiB)"
                        break
                    if free < self.args.min_free_gb * (1 << 30):
                        budget_stop = f"free space below {self.args.min_free_gb} GiB"
                        break

        self.shm.close()
        sock.close()
        health_sock.close()
        ctx.term()
        summary = {
            "out_dir": str(self.out_dir),
            "rows": self.written_rows,
            "images": self.image_index,
            "bytes": _dir_bytes(self.out_dir),
            "stopped": budget_stop or ("signal" if self.stop else "duration"),
        }
        (self.out_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        print(json.dumps(summary), flush=True)
        return 0

    def _sample(self, bundle: dict[str, Any], save_image: bool, cv2) -> list[dict[str, Any]]:
        frames = bundle.get("frames") or {}
        now_wall = time.time()
        local = datetime.fromtimestamp(now_wall).astimezone()
        bundle_time_ns = int(bundle.get("bundle_time_ns") or 0)
        age_ms = max(0.0, (time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW) - bundle_time_ns) / 1e6)
        rows: list[dict[str, Any]] = []
        for camera in self.cameras:
            frame = frames.get(f"{camera}.color")
            if frame is None:
                continue
            depth = frames.get(f"{camera}.depth") or {}
            try:
                payload = read_slot(self.shm.get(str(frame["shm_name"])), frame)
                rgb = decode_color(payload, frame)
            except (OSError, RuntimeError, ValueError) as exc:
                print(f"[light] skip {camera}: {exc}", file=sys.stderr, flush=True)
                continue
            side = _SIDE_OF.get(camera, camera)
            image_file = ""
            if save_image:
                name = f"{self.image_index:06d}_{local.strftime('%H%M%S')}_{side}.{self.args.image_format}"
                params = (
                    [cv2.IMWRITE_JPEG_QUALITY, self.args.jpeg_quality]
                    if self.args.image_format == "jpg"
                    else [cv2.IMWRITE_PNG_COMPRESSION, 3]
                )
                if cv2.imwrite(str(self.image_dir / name), rgb[:, :, ::-1], params):
                    image_file = name
            row: dict[str, Any] = {
                "schema": METRICS_SCHEMA,
                "sample_index": self.sample_index,
                "wall_iso": local.isoformat(timespec="milliseconds"),
                "wall_epoch_s": round(now_wall, 3),
                "hour_local": round(local.hour + local.minute / 60.0 + local.second / 3600.0, 4),
                "monotonic_s": round(time.monotonic(), 3),
                "arm": side,
                "camera_name": camera,
                "serial": frame.get("serial", ""),
                "bundle_seq": bundle.get("bundle_seq"),
                "frame_number": frame.get("frame_number"),
                "bundle_age_ms": round(age_ms, 3),
                "max_time_diff_ms": bundle.get("max_time_diff_ms"),
                "actual_exposure_us": frame.get("actual_exposure_us"),
                "gain_level": frame.get("gain_level"),
                "auto_exposure": frame.get("auto_exposure"),
                "depth_gain_level": depth.get("gain_level"),
                "image_file": image_file,
            }
            row.update({key: round(value, 4) for key, value in photometry(rgb).items()})
            rows.append(row)
        self.sample_index += 1
        return rows


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--metadata", default="tcp://127.0.0.1:5600")
    parser.add_argument("--topic", default="camera.bundle.policy", help="bundle group the policy consumes")
    parser.add_argument("--cameras", default=",".join(_DEFAULT_CAMERAS))
    parser.add_argument("--label", default="session", help="session label used in the output directory name")
    parser.add_argument("--out-dir", default="", help="explicit output directory (overrides --label)")
    parser.add_argument("--metrics-hz", type=float, default=5.0)
    parser.add_argument("--image-hz", type=float, default=1.0, help="0 disables image writing")
    parser.add_argument("--image-format", choices=("jpg", "png"), default="jpg")
    parser.add_argument("--jpeg-quality", type=int, default=95)
    parser.add_argument("--duration-min", type=float, default=60.0)
    parser.add_argument("--max-gb", type=float, default=12.0, help="stop when the session directory exceeds this")
    parser.add_argument("--min-free-gb", type=float, default=30.0, help="stop when the filesystem drops below this")
    parser.add_argument("--note", default="", help="free-text note stored in session.json")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    recorder = Recorder(args)

    def _handle(signum, _frame):  # noqa: ANN001 - signal handler signature
        recorder.stop = True

    signal.signal(signal.SIGINT, _handle)
    signal.signal(signal.SIGTERM, _handle)
    print(f"[light] recording to {recorder.out_dir} (pid {os.getpid()})", flush=True)
    return recorder.run()


if __name__ == "__main__":
    raise SystemExit(main())
