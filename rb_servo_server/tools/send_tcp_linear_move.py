#!/usr/bin/env python3
"""Send TcpLinearMove command packets over UDP JSON."""

from __future__ import annotations

import argparse
import json
import math
import socket
import time
from typing import Any


DEFAULT_ENDPOINT = "udp://127.0.0.1:50010"


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


def finite_target(values: list[float], label: str) -> list[float]:
    if len(values) != 7:
        raise ValueError(f"{label} requires 7 values: x y z qx qy qz qw")
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{label} values must be finite")
    q_norm = math.sqrt(sum(value * value for value in values[3:7]))
    if q_norm <= 0.0:
        raise ValueError(f"{label} quaternion must be nonzero")
    return [float(value) for value in values]


def build_packet(
    *,
    arm: str,
    target: list[float] | tuple[float, ...],
    timeout_sec: float = 0.2,
    duration_sec: float | None = None,
    linear_speed_m_s: float | None = None,
    angular_speed_rad_s: float | None = None,
    orientation_mode: str = "constant",
    seq: int | None = None,
    source_id: str | None = None,
    session_id: str | None = None,
    lease_token: str | None = None,
    host_time_ns: int | None = None,
) -> dict[str, Any]:
    if arm not in {"left", "right", "both"}:
        raise ValueError("arm must be left, right, or both")
    if orientation_mode not in {"constant", "slerp"}:
        raise ValueError("orientation_mode must be constant or slerp")
    if not math.isfinite(timeout_sec) or timeout_sec <= 0.0:
        raise ValueError("timeout_sec must be finite and positive")
    if duration_sec is None and linear_speed_m_s is None:
        raise ValueError("provide duration_sec or linear_speed_m_s")
    if duration_sec is not None and (not math.isfinite(duration_sec) or duration_sec <= 0.0):
        raise ValueError("duration_sec must be finite and positive")
    if linear_speed_m_s is not None and (not math.isfinite(linear_speed_m_s) or linear_speed_m_s <= 0.0):
        raise ValueError("linear_speed_m_s must be finite and positive")
    if angular_speed_rad_s is not None and (
        not math.isfinite(angular_speed_rad_s) or angular_speed_rad_s <= 0.0
    ):
        raise ValueError("angular_speed_rad_s must be finite and positive")

    target_values = finite_target(list(target), "target")
    packet_seq = time.monotonic_ns() if seq is None else int(seq)
    if packet_seq < 0:
        raise ValueError("seq must be non-negative")
    arm_payload = _linear_move_payload(
        target_values,
        duration_sec=duration_sec,
        linear_speed_m_s=linear_speed_m_s,
        angular_speed_rad_s=angular_speed_rad_s,
        orientation_mode=orientation_mode,
    )
    packet: dict[str, Any] = {
        "schema_version": 1,
        "seq": packet_seq,
        "mode": "TcpLinearMove" if arm == "both" else "Hold",
        "host_time_ns": time.monotonic_ns() if host_time_ns is None else int(host_time_ns),
        "timeout_sec": float(timeout_sec),
        "coupled_timeout": True,
        "left": arm_payload if arm in {"left", "both"} else {"mode": "Hold"},
        "right": arm_payload if arm in {"right", "both"} else {"mode": "Hold"},
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
            "Send a simulator-only Cartesian TcpLinearMove. Target x,y,z are "
            "stand-frame meters; qx,qy,qz,qw is target quaternion_xyzw."
        )
    )
    parser.add_argument("--endpoint", default=DEFAULT_ENDPOINT, help=f"UDP endpoint, default {DEFAULT_ENDPOINT}")
    parser.add_argument("--arm", choices=("left", "right", "both"), required=True)
    parser.add_argument("--target", nargs=7, type=float, metavar=("x", "y", "z", "qx", "qy", "qz", "qw"), required=True)
    parser.add_argument("--duration-sec", type=float)
    parser.add_argument("--linear-speed-m-s", type=float)
    parser.add_argument("--angular-speed-rad-s", type=float)
    parser.add_argument("--orientation-mode", choices=("constant", "slerp"), default="constant")
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
            target=args.target,
            timeout_sec=args.timeout_sec,
            duration_sec=args.duration_sec,
            linear_speed_m_s=args.linear_speed_m_s,
            angular_speed_rad_s=args.angular_speed_rad_s,
            orientation_mode=args.orientation_mode,
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
    print(f"sent TcpLinearMove to {args.endpoint}")


def _linear_move_payload(
    target: list[float],
    *,
    duration_sec: float | None,
    linear_speed_m_s: float | None,
    angular_speed_rad_s: float | None,
    orientation_mode: str,
) -> dict[str, Any]:
    x, y, z, qx, qy, qz, qw = target
    payload: dict[str, Any] = {
        "mode": "TcpLinearMove",
        "target_tcp_stand": {
            "x": x,
            "y": y,
            "z": z,
            "rx": 0.0,
            "ry": 0.0,
            "rz": 0.0,
            "quaternion_xyzw": [qx, qy, qz, qw],
        },
        "orientation_mode": orientation_mode,
    }
    if duration_sec is not None:
        payload["duration_sec"] = float(duration_sec)
    if linear_speed_m_s is not None:
        payload["linear_speed_m_s"] = float(linear_speed_m_s)
    if angular_speed_rad_s is not None:
        payload["angular_speed_rad_s"] = float(angular_speed_rad_s)
    return payload


if __name__ == "__main__":
    main()
