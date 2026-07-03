#!/usr/bin/env python3
"""Dump chunk-overlay/chunk-frame packets to JSONL for offline analysis.

Binds an extra UDP port (default 50265) and appends one JSON line per received
chunk packet, stamped with the local receive time. Add the port to the
producer's fan-out to capture the FULL predicted chunk (every step, both arms)
alongside the servo CSV:

  RB_GUI_CHUNK_OVERLAY_ENDPOINT=udp://127.0.0.1:50262,udp://127.0.0.1:50263,udp://127.0.0.1:50264,udp://127.0.0.1:50265

Join against the servo CSV on the producer `seq` (the CSV's follower_seq is the
server-receiver seq, monotonic per server run; use recv time for alignment).

Usage: python3 tools/chunk_frame_dump.py [--bind 0.0.0.0:50265] [--out logs/chunk_frames.jsonl]
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bind", default="0.0.0.0:50265")
    ap.add_argument("--out", default="logs/chunk_frames.jsonl")
    args = ap.parse_args()

    host, port = args.bind.rsplit(":", 1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, int(port)))
    print(f"[chunk-dump] listening on {args.bind} -> {args.out}", flush=True)

    count = 0
    with open(args.out, "a") as f:
        while True:
            data, _ = sock.recvfrom(1 << 16)
            try:
                packet = json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            packet["_recv_unix_sec"] = time.time()
            packet["_recv_monotonic_sec"] = time.monotonic()
            f.write(json.dumps(packet, separators=(",", ":")) + "\n")
            f.flush()
            count += 1
            print(f"\r[chunk-dump] frames={count} seq={packet.get('seq')}", end="", flush=True)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n[chunk-dump] stopped")
        sys.exit(0)
