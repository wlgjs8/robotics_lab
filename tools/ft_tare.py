#!/usr/bin/env python3
"""Send TareForceSensor to a running rb_servo_server.

A tare records the sensor's ZERO. Force control refuses to cover an arm whose bias
has never been established, because a law driven by an untared sensor regulates
against the bias instead of against contact — silently, and in whatever direction
the bias happens to point.

*** THE ARM MUST BE STILL AND CARRYING NOTHING BUT THE TOOL. *** Nothing in the
server can check that: whatever load stands at this moment becomes the new zero. If
the gripper is holding a part, or someone is resting a hand on the wrist, that mass
is what the arm will from then on believe weighs nothing.

The command is LEASELESS, so it works while a policy holds the command lease.

Usage:
  python3 tools/ft_tare.py                    # both arms
  python3 tools/ft_tare.py --arm right        # one arm
  python3 tools/ft_tare.py --host 127.0.0.1 --port 50256
"""
from __future__ import annotations

import argparse
import json
import socket
import sys
import time


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1", help="rb_servo_server command bind host")
    p.add_argument("--port", type=int, default=50256, help="rb_servo_server command bind port")
    p.add_argument("--arm", choices=("left", "right", "both"), default="both",
                   help="which arm to tare (default: both)")
    p.add_argument("--seq", type=int, default=None,
                   help="command sequence number (default: derived from the wall clock, "
                        "which is monotonic enough for a one-shot and avoids colliding "
                        "with a running client's sequence)")
    p.add_argument("--repeat", type=int, default=5,
                   help="how many times to send (UDP: a single datagram can be lost, and "
                        "the server folds repeats into ONE tare request)")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    seq = args.seq if args.seq is not None else int(time.time()) % 1_000_000_000

    packet = {
        "schema_version": 1,
        "seq": seq,
        "mode": "TareForceSensor",
        "timeout_sec": 0.5,
        "source_id": "ft_tare",
        "session_id": f"ft_tare_{seq}",
        # Per-arm selector, top-level. Both false would mean "both" to the server, but
        # saying it explicitly keeps the packet readable in a capture.
        "tare_left": args.arm in ("left", "both"),
        "tare_right": args.arm in ("right", "both"),
        # EMPTY ARM OBJECTS ON PURPOSE: a per-arm "mode" OVERRIDES the top-level one,
        # so naming a mode here would turn this into that mode and the tare would be
        # dropped without a word.
        "left": {},
        "right": {},
    }

    payload = json.dumps(packet).encode("utf-8")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        for i in range(max(1, args.repeat)):
            packet["seq"] = seq + i
            sock.sendto(json.dumps(packet).encode("utf-8"), (args.host, args.port))
            time.sleep(0.02)

    print(f"sent TareForceSensor ({args.arm}) to {args.host}:{args.port}")
    print("The server averages 250 ticks (0.5 s) and then logs 'tare accepted' with the")
    print("measured bias. Until it does, force control keeps refusing to cover the arm.")
    print()
    print("Check the server console for:")
    print("  [INFO] F/T <arm> tare accepted: ... (bias F [...] N, M [...] Nm, generation N)")
    print("  [INFO] force control <arm>: COVERED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
