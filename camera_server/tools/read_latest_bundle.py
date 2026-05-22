#!/usr/bin/env python3
"""Subscribe to camera.bundle and read image payloads from POSIX shared memory.

Policy runners should keep only the latest complete bundle. This tool demonstrates
that behavior and validates the seqlock before copying each frame from /dev/shm.
"""
import argparse
import json
import mmap
import os
import struct
import sys
import time

try:
    import zmq  # type: ignore
except Exception:  # pragma: no cover
    zmq = None

SLOT_HEADER = struct.Struct("<QQQQQQIIIIII")


def shm_path(name: str) -> str:
    return "/dev/shm/" + name.lstrip("/")


def read_slot(mm: mmap.mmap, frame: dict, retries: int = 1000) -> bytes:
    header_off = int(frame["shm_offset"]) - SLOT_HEADER.size
    size = int(frame["size_bytes"])
    payload_off = int(frame["shm_offset"])
    if header_off < 0 or payload_off < 0 or size < 0:
        raise RuntimeError(f"invalid shm metadata for {frame.get('ring_name')}")
    if header_off + SLOT_HEADER.size > len(mm) or payload_off + size > len(mm):
        raise RuntimeError(f"shm read out of bounds for {frame.get('ring_name')}")
    for _ in range(retries):
        mm.seek(header_off)
        a = struct.unpack_from("<Q", mm, header_off)[0]
        if a & 1:
            continue
        vals = SLOT_HEADER.unpack_from(mm, header_off)
        slot_size = vals[9]
        if slot_size != size:
            raise RuntimeError(f"bundle size does not match shm slot for {frame.get('ring_name')}")
        if payload_off + slot_size > len(mm):
            raise RuntimeError(f"shm slot size out of bounds for {frame.get('ring_name')}")
        payload = mm[payload_off:payload_off + size]
        b = vals[1]
        c = struct.unpack_from("<Q", mm, header_off)[0]
        valid = vals[10]
        if valid and a == b == c and not (b & 1):
            return payload
    raise RuntimeError(f"seqlock failed for {frame.get('ring_name')} slot={frame.get('slot_index')}")


class ShmCache:
    def __init__(self):
        self._name = None
        self._file = None
        self._mmap = None

    def get(self, name: str) -> mmap.mmap:
        if self._name == name and self._mmap is not None:
            return self._mmap
        self.close()
        self._name = name
        self._file = open(shm_path(name), "r+b")
        self._mmap = mmap.mmap(self._file.fileno(), 0, access=mmap.ACCESS_READ)
        return self._mmap

    def close(self) -> None:
        if self._mmap is not None:
            self._mmap.close()
            self._mmap = None
        if self._file is not None:
            self._file.close()
            self._file = None
        self._name = None


def handle_bundle(bundle: dict, shm_override: str | None, shm_cache: ShmCache) -> None:
    if not bundle.get("complete"):
        return
    frames = bundle.get("frames", {})
    if not frames:
        return
    shm_name = shm_override or next(iter(frames.values())).get("shm_name")
    if not shm_name:
        raise RuntimeError("bundle does not include shm_name and --shm was not provided")
    mm = shm_cache.get(shm_name)
    summary = []
    for key, frame in sorted(frames.items()):
        data = read_slot(mm, frame)
        summary.append(f"{key}:fn={frame['frame_number']}:bytes={len(data)}")
    print(f"bundle={bundle['bundle_seq']} skew_ms={bundle['max_time_diff_ms']} " + " ".join(summary), flush=True)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--metadata", default="tcp://127.0.0.1:5600")
    ap.add_argument("--topic", default="camera.bundle")
    ap.add_argument("--shm", default=None, help="override POSIX shm name, e.g. /camera_server_frames")
    ap.add_argument("--once", action="store_true")
    args = ap.parse_args()

    if zmq is None:
        print("pyzmq is required for live subscription: pip install pyzmq", file=sys.stderr)
        return 2
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt_string(zmq.SUBSCRIBE, args.topic)
    sock.connect(args.metadata)
    latest = None
    shm_cache = ShmCache()
    try:
        while True:
            topic, payload = sock.recv_multipart()
            if topic.decode() != args.topic:
                continue
            latest = json.loads(payload.decode())
            # latest-frame-wins: discard any older queued bundles by draining non-blocking.
            while True:
                try:
                    t2, p2 = sock.recv_multipart(flags=zmq.NOBLOCK)
                    if t2.decode() == args.topic:
                        latest = json.loads(p2.decode())
                except zmq.Again:
                    break
            handle_bundle(latest, args.shm, shm_cache)
            if args.once:
                return 0
    finally:
        shm_cache.close()


if __name__ == "__main__":
    raise SystemExit(main())
