#!/usr/bin/env python3
"""Bridge rb_servo_server joint streams to PlotJuggler (UDP JSON).

Subscribes to the rb_servo_server state fanout (default udp://127.0.0.1:50356,
the spare endpoint in network.state_pub_endpoints) and re-emits a compact JSON
datagram per state packet (state_pub_rate_hz, typically 100 Hz) for
PlotJuggler's "UDP Server" streaming plugin:

  {
    "t": <host_time seconds>,
    "left":  {"q_sent_deg": [6], "q_ref_deg": [6], "q_actual_deg": [6],
              "sent_minus_ref_deg": [6], "sent_minus_actual_deg": [6]},
    "right": {...}
  }

q_sent_deg   — the 6 joint targets the server sent to the Rainbow controller
q_ref_deg    — the controller's own reference readback (rbpodo sdata.jnt_ref)
q_actual_deg — encoder joint positions

Usage:
  python3 scripts/plotjuggler_joint_bridge.py \
      [--listen 127.0.0.1:50356] [--target 127.0.0.1:9870] [--verbose]

PlotJuggler side: Streaming -> "UDP Server", Port 9870, Message Protocol JSON,
and enable "use field as timestamp if available" with key "t".
"""
from __future__ import annotations

import argparse
import json
import socket
import sys
import time


def parse_endpoint(value: str) -> tuple[str, int]:
    host, _, port = value.rpartition(":")
    return (host or "127.0.0.1", int(port))


def joints(arm: dict, key: str) -> list[float] | None:
    raw = arm.get(key)
    if not isinstance(raw, list) or len(raw) != 6:
        return None
    try:
        out = [float(v) for v in raw]
    except (TypeError, ValueError):
        return None
    return out


def arm_message(arm: object) -> dict | None:
    if not isinstance(arm, dict):
        return None
    sent = joints(arm, "q_sent_deg")
    ref = joints(arm, "q_ref_deg")
    actual = joints(arm, "q_actual_deg")
    out: dict = {}
    if sent is not None:
        out["q_sent_deg"] = sent
    if ref is not None:
        out["q_ref_deg"] = ref
    if actual is not None:
        out["q_actual_deg"] = actual
    if sent is not None and ref is not None:
        out["sent_minus_ref_deg"] = [s - r for s, r in zip(sent, ref)]
    if sent is not None and actual is not None:
        out["sent_minus_actual_deg"] = [s - a for s, a in zip(sent, actual)]
    return out or None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--listen", default="127.0.0.1:50356",
                        help="rb_servo_server state endpoint to bind (default 127.0.0.1:50356)")
    parser.add_argument("--target", default="127.0.0.1:9870",
                        help="PlotJuggler UDP server address (default 127.0.0.1:9870)")
    parser.add_argument("--verbose", action="store_true", help="print forward rate once per second")
    args = parser.parse_args()

    listen = parse_endpoint(args.listen)
    target = parse_endpoint(args.target)

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(listen)
    rx.settimeout(1.0)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"listening on udp://{listen[0]}:{listen[1]} -> PlotJuggler udp://{target[0]}:{target[1]}")
    forwarded = 0
    last_report = time.monotonic()
    while True:
        try:
            data, _ = rx.recvfrom(262144)
        except socket.timeout:
            if args.verbose:
                now = time.monotonic()
                if now - last_report >= 1.0:
                    print(f"{forwarded} msg/s (waiting for state packets...)" if forwarded == 0
                          else f"{forwarded} msg/s")
                    forwarded = 0
                    last_report = now
            continue
        except KeyboardInterrupt:
            return 0
        try:
            state = json.loads(data)
        except Exception:
            continue
        host_time_ns = state.get("host_time_ns")
        message: dict = {
            "t": float(host_time_ns) * 1e-9 if isinstance(host_time_ns, (int, float)) else time.time(),
        }
        for side in ("left", "right"):
            arm = arm_message(state.get(side))
            if arm is not None:
                message[side] = arm
        if len(message) == 1:
            continue
        tx.sendto(json.dumps(message).encode("utf-8"), target)
        forwarded += 1
        if args.verbose:
            now = time.monotonic()
            if now - last_report >= 1.0:
                print(f"{forwarded} msg/s")
                forwarded = 0
                last_report = now


if __name__ == "__main__":
    sys.exit(main())
