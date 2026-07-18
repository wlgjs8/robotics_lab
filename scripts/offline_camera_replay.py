#!/usr/bin/env python3
"""Replay wrist RGB-D frames from a training HDF5 as camera.bundle.

The publisher defaults to tcp://127.0.0.1:5700, separate from the physical
camera_server on :5600. It mirrors the camera_server shared-memory seqlock wire
format consumed by policy_runner.camera_bundle_client.
"""
from __future__ import annotations

import argparse
import json
import mmap
import os
import struct
import threading
import time

import cv2
import h5py
import numpy as np
import zmq

_SLOT_HEADER = struct.Struct("<QQQQQQIIIIII")
_MONOTONIC_RAW = time.CLOCK_MONOTONIC_RAW


def _decode_color(value: np.ndarray) -> np.ndarray:
    bgr = cv2.imdecode(np.asarray(value, dtype=np.uint8), cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError("failed to decode HDF5 color frame")
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def _decode_depth(value: np.ndarray) -> np.ndarray:
    depth = cv2.imdecode(np.asarray(value, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    if depth is None:
        raise ValueError("failed to decode HDF5 depth frame")
    return depth.astype(np.uint16, copy=False)


class _ShmSlot:
    def __init__(self, name: str, width: int, height: int, stride: int, size: int):
        self.name = name.lstrip("/")
        self.path = "/dev/shm/" + self.name
        total = _SLOT_HEADER.size + size
        fd = os.open(self.path, os.O_CREAT | os.O_RDWR, 0o644)
        os.ftruncate(fd, total)
        self._mapping = mmap.mmap(
            fd, total, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE
        )
        os.close(fd)
        _SLOT_HEADER.pack_into(
            self._mapping,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            width,
            height,
            stride,
            size,
            1,
            0,
        )

    def write(self, payload: bytes, sequence: int, frame_number: int) -> None:
        struct.pack_into("<Q", self._mapping, 0, sequence | 1)
        self._mapping[_SLOT_HEADER.size : _SLOT_HEADER.size + len(payload)] = payload
        struct.pack_into("<Q", self._mapping, 16, frame_number)
        struct.pack_into("<Q", self._mapping, 8, sequence)
        struct.pack_into("<Q", self._mapping, 0, sequence)

    def close(self) -> None:
        self._mapping.close()
        try:
            os.unlink(self.path)
        except FileNotFoundError:
            pass


class ReplayPublisher:
    def __init__(
        self,
        episode: str,
        *,
        port: int = 5700,
        fps: float = 30.0,
        hold_frame: int | None = None,
        shm_prefix: str = "robotics_lab_offline_replay",
    ):
        if fps <= 0.0:
            raise ValueError("fps must be positive")
        self.episode = episode
        self.port = int(port)
        self.fps = float(fps)
        self.hold_frame = hold_frame
        self.shm_prefix = shm_prefix
        self._stop = threading.Event()
        self._slots: dict[str, _ShmSlot] = {}
        self._thread: threading.Thread | None = None
        self._load()

    def _load(self) -> None:
        self.color: dict[str, list[np.ndarray]] = {}
        self.depth: dict[str, list[np.ndarray]] = {}
        with h5py.File(self.episode, "r") as handle:
            for arm in ("left", "right"):
                color = handle[f"observations/{arm}/images/realsense_color"]
                depth = handle[f"observations/{arm}/images/realsense_depth"]
                self.color[arm] = [_decode_color(color[index]) for index in range(len(color))]
                self.depth[arm] = [_decode_depth(depth[index]) for index in range(len(depth))]
        self.frame_count = min(len(values) for values in (*self.color.values(), *self.depth.values()))
        if self.frame_count <= 0:
            raise ValueError("offline episode contains no complete dual-arm RGB-D frames")

    def _slot(
        self, name: str, width: int, height: int, stride: int, size: int
    ) -> _ShmSlot:
        slot = self._slots.get(name)
        if slot is None:
            slot = _ShmSlot(name, width, height, stride, size)
            self._slots[name] = slot
        return slot

    def _frame_metadata(self, index: int, sequence: int) -> dict[str, dict]:
        frames: dict[str, dict] = {}
        for arm in ("left", "right"):
            color = self.color[arm][index]
            height, width = color.shape[:2]
            stride = width * 3
            payload = color.tobytes()
            name = f"{self.shm_prefix}_{arm}_color"
            self._slot(name, width, height, stride, len(payload)).write(
                payload, sequence, index
            )
            frames[f"{arm}_realsense.color"] = {
                "width": width,
                "height": height,
                "stride_bytes": stride,
                "format": "rgb8",
                "shm_name": name,
                "shm_offset": _SLOT_HEADER.size,
                "size_bytes": len(payload),
                "frame_number": index,
                "host_arrival_time_ns": 0,
                "sensor_timestamp_ns": 0,
                "valid": True,
            }
            depth = self.depth[arm][index]
            depth_height, depth_width = depth.shape[:2]
            depth_stride = depth_width * 2
            depth_payload = depth.tobytes()
            depth_name = f"{self.shm_prefix}_{arm}_depth"
            self._slot(
                depth_name,
                depth_width,
                depth_height,
                depth_stride,
                len(depth_payload),
            ).write(depth_payload, sequence, index)
            frames[f"{arm}_realsense.depth"] = {
                "width": depth_width,
                "height": depth_height,
                "stride_bytes": depth_stride,
                "format": "z16",
                "shm_name": depth_name,
                "shm_offset": _SLOT_HEADER.size,
                "size_bytes": len(depth_payload),
                "frame_number": index,
                "host_arrival_time_ns": 0,
                "sensor_timestamp_ns": 0,
                "valid": True,
            }
        return frames

    def _run(self) -> None:
        socket = zmq.Context.instance().socket(zmq.PUB)
        socket.bind(f"tcp://127.0.0.1:{self.port}")
        time.sleep(0.3)
        sequence = 2
        frame_index = 0 if self.hold_frame is None else max(
            0, min(int(self.hold_frame), self.frame_count - 1)
        )
        period = 1.0 / self.fps
        next_frame = time.monotonic() + period
        try:
            while not self._stop.is_set():
                metadata = {
                    "schema": "camera_server.bundle.v1",
                    "bundle_seq": sequence,
                    "bundle_time_ns": time.clock_gettime_ns(_MONOTONIC_RAW),
                    "hardware_synced": False,
                    "sync_policy": "offline_replay",
                    "max_time_diff_ms": 0.0,
                    "complete": True,
                    "frames": self._frame_metadata(frame_index, sequence),
                    "drop_counters": {},
                }
                socket.send_multipart([b"camera.bundle", json.dumps(metadata).encode()])
                sequence += 2
                now = time.monotonic()
                if self.hold_frame is None and now >= next_frame:
                    next_frame += period
                    frame_index = (frame_index + 1) % self.frame_count
                time.sleep(min(period, 0.02))
        finally:
            socket.close(linger=0)

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        for slot in self._slots.values():
            slot.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", required=True)
    parser.add_argument("--port", type=int, default=5700)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--hold-frame", type=int)
    args = parser.parse_args()
    publisher = ReplayPublisher(
        args.episode, port=args.port, fps=args.fps, hold_frame=args.hold_frame
    )
    print(
        f"[offline-camera] episode={args.episode} frames={publisher.frame_count} "
        f"endpoint=tcp://127.0.0.1:{args.port}",
        flush=True,
    )
    publisher.start()
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        return 0
    finally:
        publisher.close()


if __name__ == "__main__":
    raise SystemExit(main())
