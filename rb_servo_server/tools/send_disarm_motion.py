#!/usr/bin/env python3
"""Disarm motion commands without clearing faults."""

import argparse
import json
import socket
import time


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=50010)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    seq = time.monotonic_ns()
    msg = {
        "seq": seq,
        "mode": "DisarmMotion",
        "host_time_ns": seq,
        "timeout_sec": 0.2,
        "coupled_timeout": True,
        "left": {},
        "right": {},
    }
    sock.sendto(json.dumps(msg).encode("utf-8"), (args.host, args.port))
    print("sent DisarmMotion")


if __name__ == "__main__":
    main()
