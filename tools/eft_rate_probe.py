#!/usr/bin/env python3
"""Probe the rbpodo data channel for external F/T sensor (eft_*) update rate.

Standalone: this probe decodes the type-0x03 wire format itself and does not
use rb_servo_server. That matters because the server stopped reading eft_* on
2026-08-26 when the v1 force stack was removed, so this is currently the only
thing in the repo that reads the sensor at all — keep it for the
controller-manager-referenced rebuild, where the real eft update rate is one
of the first things worth measuring.

Read-only bring-up diagnostic: connects to a controller's
CobotData port (TCP 5001), streams `reqdata` type-0x03 SystemState frames as
fast as the round trip allows, and reports

  - frame rate (this probe's request/response throughput),
  - controller tick rate (unique sdata.time values → the 500 Hz data update),
  - eft update rate: across unique controller ticks, how often the 6 floats
    (sdata.eft_fx..eft_mz, wire offset 488..512) actually change value,
  - per-component min/mean/max (all-zero ⇒ no external FT sensor selected).

The eft fields cannot update faster than the controller tick, so counting eft
changes per unique tick cleanly separates "refreshes every tick (500 Hz)" from
"refreshes every Nth tick (500/N Hz)" regardless of probe throughput.

Usage:
  python3 tools/eft_rate_probe.py --ip 172.28.60.200 --seconds 5   # left
  python3 tools/eft_rate_probe.py --ip 172.28.60.201 --seconds 5   # right

Same wire framing as rb_servo_server's extractNewestRbpodoStateFrame():
'$', size lo, size hi, type; total frame = size + 4 bytes. Offsets are pinned
in C++ by the static_assert next to snapshotFromSystemState().
"""

import argparse
import socket
import struct
import time

REQ = b"reqdata"
FRAME_TYPE_STATE = 0x03
OFF_TIME = 4          # float sdata.time
OFF_EFT = 488         # float eft_fx..eft_mz
EFT_END = 512         # == kRbpodoStateFrameEftEndOffsetBytes
EFT_NAMES = ["fx_N", "fy_N", "fz_N", "tx_Nm", "ty_Nm", "tz_Nm"]


def parse_frames(buf: bytearray):
    """Consume complete frames from buf; yield (frame_type, frame_bytes)."""
    pos = 0
    while len(buf) >= pos + 4:
        if buf[pos] != ord("$"):
            pos += 1
            continue
        size = 4 + (buf[pos + 2] << 8 | buf[pos + 1])
        if size > 8192:  # corrupt length claim: resync
            pos += 1
            continue
        if len(buf) < pos + size:
            break
        yield buf[pos + 3], bytes(buf[pos : pos + size])
        pos += size
    del buf[:pos]


def probe(ip: str, port: int, seconds: float, request_hz: float):
    sock = socket.create_connection((ip, port), timeout=2.0)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.settimeout(0.5)

    buf = bytearray()
    samples = []  # (host_time, sdata_time, eft6) one per received state frame
    deadline = time.monotonic() + seconds
    period = 1.0 / request_hz if request_hz > 0 else 0.0
    frames = short_frames = 0

    sock.sendall(REQ)
    last_req = time.monotonic()
    while time.monotonic() < deadline:
        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            sock.sendall(REQ)  # lost response; re-prime
            last_req = time.monotonic()
            continue
        if not chunk:
            raise ConnectionError("controller closed the data channel")
        buf.extend(chunk)
        got_state = False
        for frame_type, frame in parse_frames(buf):
            if frame_type != FRAME_TYPE_STATE:
                continue
            frames += 1
            if len(frame) < EFT_END:
                short_frames += 1
                continue
            (sdata_time,) = struct.unpack_from("<f", frame, OFF_TIME)
            eft = struct.unpack_from("<6f", frame, OFF_EFT)
            samples.append((time.monotonic(), sdata_time, eft))
            got_state = True
        if got_state:
            if period > 0.0:
                now = time.monotonic()
                sleep_for = last_req + period - now
                if sleep_for > 0:
                    time.sleep(sleep_for)
            sock.sendall(REQ)
            last_req = time.monotonic()
    sock.close()
    return samples, frames, short_frames


def analyze(ip: str, samples, frames: int, short_frames: int, seconds: float):
    print(f"\n=== {ip} ===")
    if not samples:
        print("NO state frames received")
        return
    wall = samples[-1][0] - samples[0][0]
    print(f"state frames: {frames} in {wall:.2f}s -> probe throughput {frames / max(wall, 1e-9):.0f} fps"
          + (f" ({short_frames} frames shorter than eft end {EFT_END}B!)" if short_frames else ""))

    # Deduplicate by controller tick (sdata.time).
    ticks = []  # (sdata_time, eft)
    for _, sdata_time, eft in samples:
        if not ticks or ticks[-1][0] != sdata_time:
            ticks.append((sdata_time, eft))
    tick_span = ticks[-1][0] - ticks[0][0]
    tick_rate = (len(ticks) - 1) / tick_span if tick_span > 0 else float("nan")
    deltas = [b[0] - a[0] for a, b in zip(ticks, ticks[1:]) if b[0] > a[0]]
    min_dt = min(deltas) if deltas else float("nan")
    print(f"controller ticks seen: {len(ticks)} over {tick_span:.3f}s of robot time "
          f"-> observed tick rate {tick_rate:.0f} Hz (min tick delta {min_dt * 1000:.2f} ms)")
    print("NOTE: observed tick rate is a LOWER bound if probe throughput < controller rate")

    # eft change rate across unique ticks.
    changes = sum(1 for a, b in zip(ticks, ticks[1:]) if a[1] != b[1])
    change_rate = changes / tick_span if tick_span > 0 else float("nan")
    print(f"eft changes: {changes}/{len(ticks) - 1} tick transitions "
          f"-> eft update rate ~{change_rate:.0f} Hz "
          f"({(changes / max(len(ticks) - 1, 1)) * 100:.0f}% of ticks carry a new value)")

    eft_rows = [t[1] for t in ticks]
    all_zero = all(all(v == 0.0 for v in row) for row in eft_rows)
    if all_zero:
        print("eft values: ALL ZERO -> external FT sensor not selected/connected on the controller")
    else:
        print("eft component stats over unique ticks:")
        for i, name in enumerate(EFT_NAMES):
            vals = [row[i] for row in eft_rows]
            mean = sum(vals) / len(vals)
            print(f"  {name:>6}: min {min(vals):+9.3f}  mean {mean:+9.3f}  max {max(vals):+9.3f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", action="append", required=True,
                    help="controller IP (repeatable for both arms)")
    ap.add_argument("--port", type=int, default=5001)
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--request-hz", type=float, default=0.0,
                    help="cap request rate (0 = as fast as the round trip allows)")
    args = ap.parse_args()
    for ip in args.ip:
        samples, frames, short = probe(ip, args.port, args.seconds, args.request_hz)
        analyze(ip, samples, frames, short, args.seconds)


if __name__ == "__main__":
    main()
