#!/usr/bin/env python3
"""Send TcpTwistLocal/TcpTwistStand command packets over UDP JSON."""

from __future__ import annotations

import argparse
import json
import math
import socket
import time
from typing import Any


DEFAULT_ENDPOINT = "udp://127.0.0.1:50010"
ZERO_TWIST = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def parse_endpoint(value: str) -> tuple[str, int]:
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


def finite_twist(values: list[float], label: str) -> list[float]:
    if len(values) != 6:
        raise ValueError(f"{label} requires 6 values: vx vy vz wx wy wz")
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{label} values must be finite")
    return [float(value) for value in values]


def build_packet(
    *,
    arm: str = "left",
    frame: str = "local",
    twist: list[float] | tuple[float, ...] | None = None,
    timeout_sec: float = 0.2,
    seq: int | None = None,
    source_id: str | None = None,
    session_id: str | None = None,
    lease_token: str | None = None,
    host_time_ns: int | None = None,
) -> dict[str, Any]:
    if arm not in {"left", "right", "both"}:
        raise ValueError("arm must be left, right, or both")
    if frame not in {"local", "stand"}:
        raise ValueError("frame must be local or stand")
    if not math.isfinite(timeout_sec) or timeout_sec <= 0.0:
        raise ValueError("timeout_sec must be finite and positive")
    command_twist = finite_twist(list(ZERO_TWIST if twist is None else twist), "twist")
    packet_seq = time.monotonic_ns() if seq is None else int(seq)
    if packet_seq < 0:
        raise ValueError("seq must be non-negative")

    mode = "TcpTwistLocal" if frame == "local" else "TcpTwistStand"
    key = "tcp_twist_local" if frame == "local" else "tcp_twist_stand"
    packet: dict[str, Any] = {
        "schema_version": 1,
        "seq": packet_seq,
        "mode": mode if arm == "both" else "Hold",
        "host_time_ns": time.monotonic_ns() if host_time_ns is None else int(host_time_ns),
        "timeout_sec": float(timeout_sec),
        "coupled_timeout": True,
        "left": _arm_payload("left", arm, mode, key, command_twist),
        "right": _arm_payload("right", arm, mode, key, command_twist),
    }
    if source_id is not None:
        packet["source_id"] = source_id
    if session_id is not None:
        packet["session_id"] = session_id
    if lease_token is not None:
        packet["lease_token"] = lease_token
    return packet


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Send a Cartesian TCP twist. vx,vy,vz are m/s; wx,wy,wz are rad/s "
            "in the selected local or stand frame."
        )
    )
    parser.add_argument("--endpoint", default=DEFAULT_ENDPOINT, help=f"UDP endpoint, default {DEFAULT_ENDPOINT}")
    parser.add_argument("--arm", choices=("left", "right", "both"), default="left")
    parser.add_argument("--frame", choices=("local", "stand"), default="local")
    parser.add_argument("--twist", nargs=6, type=float, metavar=("vx", "vy", "vz", "wx", "wy", "wz"))
    parser.add_argument("--timeout-sec", type=float, default=0.2)
    parser.add_argument("--seq", type=int)
    parser.add_argument("--source-id")
    parser.add_argument("--session-id")
    parser.add_argument("--lease-token")
    parser.add_argument("--print-packet", action="store_true", help="print the JSON packet before sending")
    args = parser.parse_args()

    try:
        packet = build_packet(
            arm=args.arm,
            frame=args.frame,
            twist=args.twist,
            timeout_sec=args.timeout_sec,
            seq=args.seq,
            source_id=args.source_id,
            session_id=args.session_id,
            lease_token=args.lease_token,
        )
    except ValueError as exc:
        parser.error(str(exc))

    payload = json.dumps(packet, indent=2, sort_keys=True, allow_nan=False)
    if args.print_packet:
        print(payload)

    host, port = parse_endpoint(args.endpoint)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.sendto(payload.encode("utf-8"), (host, port))
    print(f"sent {packet_mode(packet)} to {args.endpoint}")


def _arm_payload(
    side: str,
    selected_arm: str,
    mode: str,
    key: str,
    twist: list[float],
) -> dict[str, Any]:
    if selected_arm not in {side, "both"}:
        return {"mode": "Hold"}
    return {"mode": mode, key: twist}


def packet_mode(packet: dict[str, Any]) -> str:
    left = packet.get("left", {})
    if isinstance(left, dict) and left.get("mode") != "Hold":
        return str(left.get("mode"))
    right = packet.get("right", {})
    if isinstance(right, dict) and right.get("mode") != "Hold":
        return str(right.get("mode"))
    return str(packet.get("mode"))


if __name__ == "__main__":
    main()
