#!/usr/bin/env python3
"""Stream a tiny joint sine AROUND THE ARMS' CURRENT POSE, to exercise the box queue.

WHY NOT send_dual_joint_sine.py
===============================
That tool sines around a HARD-CODED base, [0,-30,80,0,60,0]. Sending it while the arms
are anywhere else commands a large move to that pose first -- and on the RB5-850E that
particular pose puts the arm 90.6 mm BELOW the stand floor (measured 2026-09-03). It is
an RB3-era fixture and is not safe to fire blind.

This reads the CURRENT joints off each control box first (read-only, the same
CobotData path tools/rbpodo_read_state uses) and dithers around them, so t=0 is
continuous with where the arm already is. Amplitude ramps in over --ramp-sec, so there
is no step at the start either.

WHAT IT IS FOR
==============
The control box's queue only fills when the server is actually streaming servo_j;
in ConnectedHold it sends nothing (measured: rback_observed 0, rback_fill -1,
send_duration_us 0). Likewise q_sent / q_ref / q_actual are identical at rest, so the
transport lag they encode is only visible when the command is MOVING. A small dither
gives both without taking the arm anywhere.

Defaults are deliberately timid: 0.5 deg on ONE joint at 0.2 Hz.

  rb_servo_server/tools/send_joint_dither.py --duration-sec 30
  rb_servo_server/tools/send_joint_dither.py --joint 5 --amp-deg 1.0 --freq 0.3
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import subprocess
import sys
import time

READ_STATE_BIN_CANDIDATES = (
    "rb_servo_server/build/rbpodo_real_gate/rbpodo_read_state",
    "rb_servo_server/build/rbpodo_read_state",
    "/tmp/rb5_build/rbpodo_read_state",
)
DEFAULT_IPS = ("172.28.60.200", "172.28.60.201")


def read_current_joints(ips: tuple[str, ...]) -> dict[str, list[float]]:
    """Current measured joints per arm, or raise. Never commands anything."""
    import os

    binary = next((p for p in READ_STATE_BIN_CANDIDATES if os.path.isfile(p)), None)
    if binary is None:
        raise SystemExit(
            "rbpodo_read_state not built; build it with -DRB_SERVO_ENABLE_RBPODO=ON")
    out = subprocess.run([binary, "--json", "--samples", "3", *ips],
                         capture_output=True, text=True, timeout=30)
    if out.returncode != 0:
        raise SystemExit(f"read_state failed:\n{out.stdout}\n{out.stderr}")
    data = json.loads(out.stdout)
    by_ip = {c["ip"]: c for c in data["controllers"]}
    result = {}
    for side, ip in zip(("left", "right"), ips):
        c = by_ip.get(ip)
        if c is None or not c.get("ok"):
            raise SystemExit(f"{side} arm ({ip}) did not read back: {c}")
        if c["max_sample_spread_deg"] >= 0.05:
            raise SystemExit(
                f"{side} arm is MOVING ({c['max_sample_spread_deg']:.4f} deg spread); "
                "refusing to dither around a pose that is not settled")
        result[side] = list(c["jnt_ang_deg"])
    return result


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    # The tracked configs bind commands on 50256 (network.command_bind in both
    # stack_real.yaml and stack_sim.yaml). 50010 was the legacy simulator default and
    # survives only in docs/archive; a tool left pointing there sends into nothing --
    # silently, because UDP. That already cost a run once
    # (docs/reports/flow_infer_pgmode_sim_param_search.md: a reset went to 50010 and
    # never arrived), and send_emergency_stop.py carried the same default.
    ap.add_argument("--port", type=int, default=50256)
    ap.add_argument("--ips", nargs=2, default=list(DEFAULT_IPS))
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--joint", type=int, default=0, help="0-based joint index to dither")
    ap.add_argument("--amp-deg", type=float, default=0.5)
    ap.add_argument("--freq", type=float, default=0.2)
    ap.add_argument("--ramp-sec", type=float, default=3.0)
    ap.add_argument("--duration-sec", type=float, default=30.0)
    args = ap.parse_args()

    if not 0 <= args.joint <= 5:
        raise SystemExit("--joint must be 0..5")
    if abs(args.amp_deg) > 5.0:
        raise SystemExit("--amp-deg above 5 deg is not a dither; use a motion tool")

    base = read_current_joints(tuple(args.ips))
    print(f"left  base = {[round(v, 3) for v in base['left']]}")
    print(f"right base = {[round(v, 3) for v in base['right']]}")
    print(f"dithering joint J{args.joint + 1} by +/-{args.amp_deg} deg at {args.freq} Hz "
          f"(ramp {args.ramp_sec}s, total {args.duration_sec}s)")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (args.host, args.port)
    seq = time.monotonic_ns()
    sock.sendto(json.dumps({
        "seq": seq, "mode": "ArmMotion", "host_time_ns": time.monotonic_ns(),
        "timeout_sec": 0.2, "coupled_timeout": True, "left": {}, "right": {},
    }).encode(), addr)
    seq += 1
    time.sleep(0.1)

    dt = 1.0 / args.rate
    t0 = time.monotonic()
    try:
        while True:
            t = time.monotonic() - t0
            if t >= args.duration_sec:
                break
            # Ramp the amplitude in so t=0 is continuous with the parked pose.
            amp = args.amp_deg * min(1.0, t / max(args.ramp_sec, 1e-6))
            d = amp * math.sin(2.0 * math.pi * args.freq * t)
            ql = list(base["left"])
            qr = list(base["right"])
            ql[args.joint] += d
            qr[args.joint] += d
            sock.sendto(json.dumps({
                "seq": seq, "mode": "JointTarget", "host_time_ns": time.monotonic_ns(),
                "timeout_sec": 0.2, "coupled_timeout": True,
                "left": {"q_target_deg": ql}, "right": {"q_target_deg": qr},
            }).encode(), addr)
            seq += 1
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        # Settle back onto the exact starting pose, then hand control back to Hold.
        for _ in range(int(args.rate)):
            sock.sendto(json.dumps({
                "seq": seq, "mode": "JointTarget", "host_time_ns": time.monotonic_ns(),
                "timeout_sec": 0.2, "coupled_timeout": True,
                "left": {"q_target_deg": base["left"]},
                "right": {"q_target_deg": base["right"]},
            }).encode(), addr)
            seq += 1
            time.sleep(dt)
        sock.sendto(json.dumps({
            "seq": seq, "mode": "Hold", "host_time_ns": time.monotonic_ns(),
            "timeout_sec": 0.2, "coupled_timeout": True, "left": {}, "right": {},
        }).encode(), addr)
        print("returned to base pose and sent Hold")


if __name__ == "__main__":
    main()
