#!/usr/bin/env python3
"""Send ResetFault command to rb_servo_server over UDP JSON."""

import argparse
import json
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
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    seq = time.monotonic_ns()
    msg = {
        "seq": seq,
        "mode": "ResetFault",
        "host_time_ns": seq,
        "timeout_sec": 0.2,
        "left": {},
        "right": {},
    }
    sock.sendto(json.dumps(msg).encode("utf-8"), (args.host, args.port))
    print("sent ResetFault")


if __name__ == "__main__":
    main()
