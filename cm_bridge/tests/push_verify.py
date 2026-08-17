#!/usr/bin/env python3
"""push_verify — §6 hand-push force-control verification on the CM stack.

Streams HOLD chunks to both arms (keeps FollowUnit engaged, zero commanded
motion) while logging, from the bridge's own servo_state fanout (50378):
  - cmd deviation : |tcp_command_stand - hold pose|  (the admittance offset,
                    what the overlay writes into the emitted command)
  - act deviation : |tcp_stand - hold pose|          (what the robot did)
  - |F| wrench_af : push force magnitude

Expected with adm_overlay=ON: a sustained push of F newtons deflects the
command by ~F/k (k=1000 N/m -> 30 N = 30 mm), tau = b/k = 0.44 s, spring-back
on release, hard fence at 40 mm. Ctrl-C or SIGTERM to stop (stream stops ->
silence exit -> Idle).
"""
import json
import signal
import socket
import sys
import time

CHUNK_ADDR = ("127.0.0.1", 50264)
STATE_PORT = 50378
DT = 1.0 / 29.94
ROWS = 8
REPLAN = 4

run = True
def stop(*_):
    global run
    run = False
signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGINT, stop)


def main():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("127.0.0.1", STATE_PORT))
    rx.settimeout(1.0)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # latch the hold pose per arm (drain first: the fanout buffers)
    hold = {}
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline and len(hold) < 2:
        try:
            data, _ = rx.recvfrom(65535)
            st = json.loads(data)
        except (OSError, json.JSONDecodeError):
            continue
        for side in ("left", "right"):
            a = st.get(side, {})
            cmd = a.get("tcp_command_stand") or a.get("tcp_stand")
            act = a.get("tcp_stand")
            if side not in hold and cmd and act:
                hold[side] = [cmd["x"], cmd["y"], cmd["z"]] + act["quaternion_xyzw"]
    if len(hold) < 2:
        print("FAIL: no state for both arms on", STATE_PORT, flush=True)
        return 1
    for s in ("left", "right"):
        print(f"hold {s}: [{hold[s][0]:+.4f} {hold[s][1]:+.4f} {hold[s][2]:+.4f}]", flush=True)

    peak = {"left": [0.0, 0.0, 0.0], "right": [0.0, 0.0, 0.0]}  # cmd_dev, act_dev, |F|
    seq = 0
    t0 = time.monotonic()
    next_send = t0
    next_print = t0 + 1.0
    while run:
        now = time.monotonic()
        if now >= next_send:                       # one replan every REPLAN rows
            rows = {s: [list(hold[s]) for _ in range(ROWS)] for s in hold}
            pkt = {"schema_version": "robotics_lab.chunk_overlay.v3", "seq": seq,
                   "policy_dt_sec": DT, "left": rows["left"], "right": rows["right"]}
            tx.sendto(json.dumps(pkt).encode(), CHUNK_ADDR)
            seq += 1
            next_send += DT * REPLAN
        # drain state
        while True:
            try:
                rx.setblocking(False)
                data, _ = rx.recvfrom(65535)
            except (BlockingIOError, OSError):
                rx.setblocking(True)
                break
            finally:
                rx.setblocking(True)
            try:
                st = json.loads(data)
            except json.JSONDecodeError:
                continue
            line = []
            for side in ("left", "right"):
                a = st.get(side, {})
                cmd, act = a.get("tcp_command_stand"), a.get("tcp_stand")
                w = a.get("wrench_af") or [0.0] * 6
                if not (cmd and act):
                    continue
                h = hold[side]
                cd = ((cmd["x"] - h[0]) ** 2 + (cmd["y"] - h[1]) ** 2 + (cmd["z"] - h[2]) ** 2) ** 0.5
                ad = ((act["x"] - h[0]) ** 2 + (act["y"] - h[1]) ** 2 + (act["z"] - h[2]) ** 2) ** 0.5
                fn = (w[0] ** 2 + w[1] ** 2 + w[2] ** 2) ** 0.5
                p = peak[side]
                p[0] = max(p[0], cd); p[1] = max(p[1], ad); p[2] = max(p[2], fn)
                line.append(f"{side[0].upper()} cmd_dev={cd*1000:6.1f}mm act_dev={ad*1000:6.1f}mm |F|={fn:5.1f}N")
            if line and now >= next_print:
                print(f"[{now-t0:6.1f}s] " + "  |  ".join(line), flush=True)
                next_print = now + 0.5
        time.sleep(0.005)

    print("--- peaks ---", flush=True)
    for s in ("left", "right"):
        p = peak[s]
        print(f"{s:5s}: cmd_dev {p[0]*1000:.1f} mm | act_dev {p[1]*1000:.1f} mm | |F| {p[2]:.1f} N", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
