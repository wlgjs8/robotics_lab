#!/usr/bin/env python3
"""cm-sils-gate — P1 acceptance for the cm_bridge command+state loop in SILS.

Host-side, no ROS: drives the bridge exactly like policy_runner would (chunk
UDP 50264) and judges from the bridge's own servo_state.v1 republish
(tcp_command_stand = CM cmd/pose, i.e. the controller's 500 Hz reference
sampled at the fanout rate). Three checks:

  1. CADENCE   6 s of 30 Hz rows in 4-row-replan chunks (7 deltas each →
               natural runway): the commanded pose must advance continuously
               (no dwell > 150 ms) and reach ≈ the commanded displacement.
  2. DROP-ONE  one skipped replan mid-stream must NOT freeze the motion
               (leftover runway deltas cover the gap; dwell < 250 ms) and
               must NOT exit Follow (motion resumes).
  3. SILENCE   after the stream stops, motion must settle (silence exit) and
               STAY settled.

Prereq: monkey-sils container up, arms at OnTask(Idle), bridge running
(docker exec -d ... cm_bridge_node.py). See cm_bridge/run_cm_stack.sh.
"""
import json
import socket
import sys
import time

CHUNK_ADDR = ("127.0.0.1", 50264)
STATE_PORT = 50378  # legacy external-readback port; not used by rb_gui in SILS
DT = 1.0 / 29.94
ROWS = 8          # execute 4 + runway 4 → 7 deltas per chunk
REPLAN = 4        # rows consumed per replan (REPLACE cadence)
VY = 0.030        # m/s commanded


def state_listener():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", STATE_PORT))
    s.settimeout(0.5)
    return s


def poll_y(sock, samples, t0):
    """Drain state packets, append (t, cmd_y, act_y)."""
    while True:
        try:
            sock.setblocking(False)
            data, _ = sock.recvfrom(65535)
        except BlockingIOError:
            return
        except OSError:
            return
        finally:
            sock.setblocking(True)
        try:
            st = json.loads(data)
        except json.JSONDecodeError:
            continue
        left = st.get("left", {})
        cmd = left.get("tcp_command_stand") or left.get("tcp_stand")
        act = left.get("tcp_stand")
        if cmd and act:
            samples.append((time.monotonic() - t0, cmd["y"], act["y"]))


def max_dwell(samples, lo, hi, eps=1e-5):
    """Longest time span with no cmd_y progress inside window [lo, hi]."""
    win = [(t, y) for t, y, _ in samples if lo <= t <= hi]
    worst, t_last, y_last = 0.0, None, None
    for t, y in win:
        if y_last is None or abs(y - y_last) > eps:
            if t_last is not None and y_last is not None:
                pass
            t_last, y_last = t, y
        else:
            worst = max(worst, t - t_last)
    return worst, win


def main():
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx = state_listener()
    # wait for state stream + capture start pose
    t0 = time.monotonic()
    start = None
    deadline = t0 + 5
    while time.monotonic() < deadline and start is None:
        try:
            data, _ = rx.recvfrom(65535)
            st = json.loads(data)
            cmd = st["left"].get("tcp_command_stand") or st["left"]["tcp_stand"]
            base = [cmd["x"], cmd["y"], cmd["z"]] + st["left"]["tcp_stand"]["quaternion_xyzw"]
            start = base
        except (OSError, KeyError, json.JSONDecodeError):
            continue
    if start is None:
        print("GATE FAIL: no servo_state.v1 on port", STATE_PORT)
        return 1
    print(f"start pose y={start[1]:+.4f}")

    samples = []
    t0 = time.monotonic()
    seq = 0
    y = start[1]
    drop_at = 20          # replan index to skip (DROP-ONE, ~2.7 s in)
    total_replans = 45    # ~6 s
    for i in range(total_replans):
        rows = []
        for k in range(ROWS):
            r = list(start)
            r[1] = y + VY * DT * k
            rows.append(r)
        if i != drop_at:
            pkt = {"schema_version": "robotics_lab.chunk_overlay.v3", "seq": seq,
                   "policy_dt_sec": DT, "left": rows}
            tx.sendto(json.dumps(pkt).encode(), CHUNK_ADDR)
        seq += 1
        y += VY * DT * REPLAN
        t_next = t0 + (i + 1) * DT * REPLAN
        while time.monotonic() < t_next:
            poll_y(rx, samples, t0)
            time.sleep(0.002)
    stream_end = time.monotonic() - t0
    # observe the silence exit + settle
    while time.monotonic() - t0 < stream_end + 2.5:
        poll_y(rx, samples, t0)
        time.sleep(0.002)

    if len(samples) < 100:
        print("GATE FAIL: too few state samples", len(samples))
        return 1

    results = []
    # 1. CADENCE: steady window before the drop
    dwell, win = max_dwell(samples, 1.0, drop_at * REPLAN * DT - 0.2)
    moved = win[-1][1] - win[0][1] if win else 0.0
    expect = VY * (win[-1][0] - win[0][0]) if win else 0.0
    ok1 = dwell < 0.150 and moved > 0.6 * expect
    results.append(("CADENCE", ok1,
                    f"dwell_max={dwell*1000:.0f}ms moved={moved*1000:.1f}mm expect~{expect*1000:.0f}mm"))
    # 2. DROP-ONE: window around the skipped replan
    t_drop = drop_at * REPLAN * DT
    dwell2, _ = max_dwell(samples, t_drop - 0.1, t_drop + 0.6)
    ok2 = dwell2 < 0.250
    results.append(("DROP-ONE", ok2, f"dwell_max={dwell2*1000:.0f}ms (limit 250)"))
    # 3. SILENCE: settled and stays settled
    tail = [(t, y) for t, y, _ in samples if t > stream_end + 1.0]
    drift = (tail[-1][1] - tail[0][1]) if len(tail) > 2 else 1e9
    ok3 = abs(drift) < 0.002
    results.append(("SILENCE", ok3, f"post-settle drift={drift*1000:.2f}mm over {tail[-1][0]-tail[0][0] if len(tail)>2 else 0:.1f}s"))

    fail = False
    for name, ok, detail in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name:9s} {detail}")
        fail |= not ok
    print("GATE", "FAIL" if fail else "PASS")
    return 1 if fail else 0


if __name__ == "__main__":
    sys.exit(main())
