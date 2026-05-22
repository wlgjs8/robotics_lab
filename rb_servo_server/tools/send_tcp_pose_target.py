#!/usr/bin/env python3
"""Send TcpPoseTarget command packets to rb_servo_server over UDP JSON."""

import argparse
import json
import math
import socket
import time
from typing import Dict, List, Optional, Tuple


DEFAULT_ENDPOINT = "udp://127.0.0.1:50010"


def parse_endpoint(value: str) -> Tuple[str, int]:
    prefix = "udp://"
    if not value.startswith(prefix):
        raise argparse.ArgumentTypeError("endpoint must use udp://host:port")
    host_port = value[len(prefix) :]
    if ":" not in host_port:
        raise argparse.ArgumentTypeError("endpoint must use udp://host:port")
    host, port_text = host_port.rsplit(":", 1)
    if not host:
        raise argparse.ArgumentTypeError("endpoint host must not be empty")
    try:
        port = int(port_text, 10)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("endpoint port must be an integer") from exc
    if port <= 0 or port > 65535:
        raise argparse.ArgumentTypeError("endpoint port must be in 1..65535")
    return host, port


def finite_pose(values: List[float], label: str) -> List[float]:
    if len(values) != 6:
        raise argparse.ArgumentTypeError(f"{label} requires 6 values: x y z rx ry rz")
    if not all(math.isfinite(value) for value in values):
        raise argparse.ArgumentTypeError(f"{label} values must be finite")
    return values


def add_arm(packet: Dict[str, object], arm: str, pose: Optional[List[float]]) -> None:
    if pose is None:
        packet[arm] = {}
        return
    packet[arm] = {
        "mode": "TcpPoseTarget",
        "tcp_target_stand": pose,
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Send a TCP pose target in the stand frame. x,y,z are meters; "
            "rx,ry,rz are roll/pitch/yaw radians."
        )
    )
    parser.add_argument("--endpoint", default=DEFAULT_ENDPOINT, help=f"UDP endpoint, default {DEFAULT_ENDPOINT}")
    parser.add_argument("--timeout-sec", type=float, default=0.2)
    parser.add_argument("--seq", type=int)
    parser.add_argument("--dry-run", action="store_true", help="print the packet without sending UDP")
    parser.add_argument("--left", nargs=6, type=float, metavar=("x", "y", "z", "rx", "ry", "rz"))
    parser.add_argument("--right", nargs=6, type=float, metavar=("x", "y", "z", "rx", "ry", "rz"))
    args = parser.parse_args()

    if args.left is None and args.right is None:
        parser.error("provide --left and/or --right")
    if not math.isfinite(args.timeout_sec) or args.timeout_sec <= 0.0:
        parser.error("--timeout-sec must be finite and positive")

    left = finite_pose(args.left, "--left") if args.left is not None else None
    right = finite_pose(args.right, "--right") if args.right is not None else None
    seq = args.seq if args.seq is not None else time.monotonic_ns()
    if seq < 0:
        parser.error("--seq must be non-negative")

    packet: Dict[str, object] = {
        "schema_version": 1,
        "seq": seq,
        "mode": "TcpPoseTarget" if left is not None and right is not None else "Hold",
        "host_time_ns": time.monotonic_ns(),
        "timeout_sec": args.timeout_sec,
        "coupled_timeout": True,
    }
    add_arm(packet, "left", left)
    add_arm(packet, "right", right)

    payload = json.dumps(packet, indent=2, sort_keys=True, allow_nan=False)
    print(payload)

    if args.dry_run:
        return

    host, port = parse_endpoint(args.endpoint)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.sendto(payload.encode("utf-8"), (host, port))
    print(f"sent TcpPoseTarget to {args.endpoint}")


if __name__ == "__main__":
    main()
