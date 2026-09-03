#!/usr/bin/env python3
"""Send dual-arm joint sine commands to rb_servo_server over UDP JSON."""

import argparse
import json
import math
import socket
import time


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    # The tracked configs bind commands on 50256 (network.command_bind in both
    # stack_real.yaml and stack_sim.yaml). 50010 was the legacy simulator default and
    # survives only in docs/archive; a tool left pointing there sends into nothing --
    # silently, because UDP. That already cost a run once
    # (docs/reports/flow_infer_pgmode_sim_param_search.md: a reset went to 50010 and
    # never arrived), and send_emergency_stop.py carried the same default.
    parser.add_argument("--port", type=int, default=50256)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--amp-deg", type=float, default=3.0)
    parser.add_argument("--freq", type=float, default=0.1)
    parser.add_argument("--arm-settle-sec", type=float, default=0.1)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (args.host, args.port)

    base = [0.0, -30.0, 80.0, 0.0, 60.0, 0.0]
    seq = time.monotonic_ns()
    dt = 1.0 / args.rate
    arm_msg = {
        "seq": seq,
        "mode": "ArmMotion",
        "host_time_ns": time.monotonic_ns(),
        "timeout_sec": 0.2,
        "coupled_timeout": True,
        "left": {},
        "right": {},
    }
    sock.sendto(json.dumps(arm_msg).encode("utf-8"), addr)
    seq += 1
    if args.arm_settle_sec > 0.0:
        time.sleep(args.arm_settle_sec)

    while True:
        t = time.monotonic()
        q_left = list(base)
        q_right = list(base)
        q_left[0] += args.amp_deg * math.sin(2.0 * math.pi * args.freq * t)
        q_right[0] -= args.amp_deg * math.sin(2.0 * math.pi * args.freq * t)

        msg = {
            "seq": seq,
            "mode": "JointTarget",
            "host_time_ns": time.monotonic_ns(),
            "timeout_sec": 0.2,
            "coupled_timeout": True,
            "left": {"q_target_deg": q_left},
            "right": {"q_target_deg": q_right},
        }
        sock.sendto(json.dumps(msg).encode("utf-8"), addr)
        seq += 1
        time.sleep(dt)


if __name__ == "__main__":
    main()
