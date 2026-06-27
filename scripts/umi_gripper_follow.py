#!/usr/bin/env python3
"""Robot-side Pika Gripper follower for the UMI teleop UDP stream.

The pika publisher (`pika/scripts/umi_teleop_publish.py` on the SteamVR PC)
used to drive the robot-mounted Pika Grippers over its own local serial ports.
The grippers are now attached to the robotics_lab control PC, so the publisher
streams the raw Sense encoder angle instead and this bridge replays it onto
the local serial grippers.

Wire schema (same combined packet as the UMI pose stream, one extra port):

  {"t": <publisher monotonic>,
   "left":  {"pose": [7], "gripper": <0..1>, "gripper_rad": <rad>, "deadman": <bool>},
   "right": {...}}

- Sides are robot-arm frame (the publisher's --swap-lr is already applied).
- "gripper_rad" is the raw Sense encoder angle in rad; per the Pika manual the
  Sense and Gripper share motor parameters, so it is a 1:1 POSITION_CTRL
  passthrough (range clamp only).
- A side missing "gripper_rad" (old publisher, no Sense) is held.
- Staleness is measured against the LOCAL arrival clock (publisher "t" is from
  another host's monotonic domain); a stale side is held, never re-targeted.

The publisher sends the identical packet to the pose ports (50380/50381,
consumed by policy_runner UdpUmiPoseReader) and to this bridge's port (50382).
This script binds its own port because the pose readers already own theirs.

Usage (robot PC, grippers on local serial):
  python3 scripts/umi_gripper_follow.py \
      [--listen 0.0.0.0:50382] [--left-port /dev/pika-left] [--right-port /dev/pika-right]

  --swap-ports          flip the left/right serial mapping (use --ident to check)
  --ident left|right    wiggle one gripper open/close to identify the mapping
  --engaged-only        follow only while that side's deadman/clutch is engaged
  --selftest            validate the pure decision/parse helpers, no hardware

The Pika SDK (pika.gripper) is not on the default sys.path of this PC; the
package copied from the SteamVR PC's conda env lives at --pika-sdk-path
(default /home/plaif/workspace/pika_sdk).
"""
from __future__ import annotations

import argparse
import json
import logging
import math
import os
import socket
import sys
import time

log = logging.getLogger("umi_gripper_follow")

SIDES = ("left", "right")


# --------------------- pure helpers (selftest-able, no hardware) ---------------------
def gripper_send_decision(rad, last, now, min_rad, max_rad, deadband_rad, min_period):
    """Decide the motor angle (rad) to send — clamp + deadband + rate limit.

    rad: Sense encoder angle (rad, None/NaN = unknown), last: (monotonic, rad)|None.
    Returns the rad to send, or None to skip. (Moved verbatim from the pika
    publisher's former local GripperFollower.)
    """
    if rad is None or (isinstance(rad, float) and math.isnan(rad)):
        return None
    clamped = max(float(min_rad), min(float(max_rad), float(rad)))
    if last is not None:
        last_t, last_rad = last
        if now - last_t < min_period:
            return None
        if abs(clamped - last_rad) < deadband_rad:
            return None
    return clamped


def parse_packet_sides(data: bytes) -> dict:
    """UDP JSON packet -> {side: {"gripper_rad": float|None, "deadman": bool}}.

    Sides absent from the packet (pose invalid on the publisher) are omitted;
    a present side without a finite "gripper_rad" maps to None (hold).
    """
    raw = json.loads(data.decode("utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("UMI UDP packet must be a JSON object")
    sides = {}
    for name in SIDES:
        entry = raw.get(name)
        if not isinstance(entry, dict):
            continue
        rad = entry.get("gripper_rad")
        if not isinstance(rad, (int, float)) or (
            isinstance(rad, float) and not math.isfinite(rad)
        ):
            rad = None
        sides[name] = {
            "gripper_rad": None if rad is None else float(rad),
            "deadman": bool(entry.get("deadman", False)),
        }
    return sides


def parse_listen(value: str) -> tuple[str, int]:
    host, _, port = value.rpartition(":")
    return (host or "0.0.0.0", int(port))


# --------------------------------- hardware-side ---------------------------------
class _LogThrottle(logging.Filter):
    """Limit a logger to one record per period (Pika SDK telemetry-parse spam)."""

    def __init__(self, period_sec: float = 2.0):
        super().__init__()
        self.period_sec = period_sec
        self._last = 0.0

    def filter(self, record):
        now = time.monotonic()
        if now - self._last >= self.period_sec:
            self._last = now
            return True
        return False


def import_gripper_class(sdk_path: str):
    if sdk_path:
        sys.path.insert(0, sdk_path)
    try:
        from pika.gripper import Gripper  # AgileX Pika SDK (pyserial only)
    except ImportError as exc:
        raise SystemExit(
            f"pika.gripper import failed ({exc}). The AgileX Pika SDK package is "
            f"expected at {sdk_path or '<sys.path>'}/pika (copied from the SteamVR "
            "PC conda env); pass --pika-sdk-path if it lives elsewhere."
        )
    return Gripper


class GripperDriver:
    """Owns the serial grippers; serial errors warn (throttled), never kill teleop."""

    def __init__(self, ports: dict, min_rad: float, max_rad: float,
                 deadband_rad: float, max_hz: float, gripper_cls=None):
        self.ports = dict(ports)            # {"left": "/dev/...", "right": "/dev/..."}
        self.min_rad = float(min_rad)
        self.max_rad = float(max_rad)
        self.deadband_rad = float(deadband_rad)
        self.min_period = 1.0 / max_hz if max_hz > 0 else 0.0
        self._gripper_cls = gripper_cls
        self.grippers = {}
        self._last_sent = {}                # side -> (monotonic, rad)
        self._warned = {}                   # side -> last warn monotonic

    def start(self):
        logging.getLogger("pika.serial_comm").addFilter(_LogThrottle(2.0))
        for side, port in self.ports.items():
            if not os.path.exists(port):
                raise RuntimeError(f"[gripper:{side}] serial port not found: {port}")
            g = self._gripper_cls(port=port)
            if not g.connect():
                detail = port
                if os.path.islink(port):
                    detail = f"{port} -> {os.path.realpath(port)}"
                raise RuntimeError(f"[gripper:{side}] {detail} connect failed")
            if not g.enable():
                raise RuntimeError(f"[gripper:{side}] {port} enable failed")
            self.grippers[side] = g
            log.info("[gripper] %s <- %s connected+enabled", side, port)
        return self

    def update(self, side: str, rad) -> None:
        g = self.grippers.get(side)
        if g is None:
            return
        now = time.monotonic()
        decided = gripper_send_decision(
            rad, self._last_sent.get(side), now,
            self.min_rad, self.max_rad, self.deadband_rad, self.min_period)
        if decided is None:
            return
        try:
            g.set_motor_angle(decided)
            self._last_sent[side] = (now, decided)
        except Exception as exc:
            if now - self._warned.get(side, 0.0) > 2.0:
                self._warned[side] = now
                log.warning("[gripper:%s] send failed: %s", side, exc)

    def close(self) -> None:
        for side, g in self.grippers.items():
            for fn in (g.disable, g.disconnect):
                try:
                    fn()
                except Exception:
                    pass
        self.grippers = {}


def run_ident(driver: GripperDriver, side: str) -> None:
    """Wiggle one gripper (gentle open/close) so the operator can confirm
    which physical arm the serial port maps to."""
    log.info("[ident] wiggling %s gripper (%s) — watch which arm moves",
             side, driver.ports.get(side))
    lo = driver.min_rad
    hi = min(driver.max_rad, lo + 0.4)
    for target in (hi, lo, hi, lo):
        driver.update(side, target)
        time.sleep(0.6)
        driver._last_sent.pop(side, None)  # bypass deadband/rate gate per step
    log.info("[ident] done")


def selftest() -> None:
    # decision: clamp / deadband / rate limit / invalid input
    assert gripper_send_decision(None, None, 0.0, 0.0, 1.75, 0.005, 1 / 60) is None
    assert gripper_send_decision(float("nan"), None, 0.0, 0.0, 1.75, 0.005, 1 / 60) is None
    assert gripper_send_decision(0.5, None, 0.0, 0.0, 1.75, 0.005, 1 / 60) == 0.5
    assert gripper_send_decision(9.0, None, 0.0, 0.0, 1.75, 0.005, 1 / 60) == 1.75
    assert gripper_send_decision(-1.0, None, 0.0, 0.0, 1.75, 0.005, 1 / 60) == 0.0
    assert gripper_send_decision(0.5, (0.0, 0.5), 1.0, 0.0, 1.75, 0.005, 1 / 60) is None
    assert gripper_send_decision(0.6, (0.99, 0.5), 1.0, 0.0, 1.75, 0.005, 1 / 60) is None
    assert gripper_send_decision(0.6, (0.0, 0.5), 1.0, 0.0, 1.75, 0.005, 1 / 60) == 0.6
    # packet parsing: per-side rad/deadman, missing/invalid rad -> None, absent side omitted
    pkt = json.dumps({
        "t": 1.0,
        "left": {"pose": [0] * 7, "gripper": 0.4, "gripper_rad": 0.7, "deadman": True},
        "right": {"pose": [0] * 7, "gripper": 0.1, "deadman": False},
    }).encode()
    sides = parse_packet_sides(pkt)
    assert sides["left"] == {"gripper_rad": 0.7, "deadman": True}, sides
    assert sides["right"] == {"gripper_rad": None, "deadman": False}, sides
    only_right = parse_packet_sides(b'{"t": 2.0, "right": {"gripper_rad": 1.2}}')
    assert "left" not in only_right
    assert only_right["right"] == {"gripper_rad": 1.2, "deadman": False}
    bad_rad = parse_packet_sides(b'{"left": {"gripper_rad": "x", "deadman": true}}')
    assert bad_rad["left"]["gripper_rad"] is None
    nan_rad = parse_packet_sides(b'{"left": {"gripper_rad": NaN, "deadman": true}}')
    assert nan_rad["left"]["gripper_rad"] is None
    # listen endpoint parsing
    assert parse_listen("0.0.0.0:50382") == ("0.0.0.0", 50382)
    assert parse_listen("50382") == ("0.0.0.0", 50382)
    print("selftest OK")


def get_arguments() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="UMI teleop robot-side Pika Gripper follower")
    ap.add_argument("--selftest", action="store_true",
                    help="validate pure helpers without hardware, then exit")
    ap.add_argument("--listen", default="0.0.0.0:50382",
                    help="UDP bind host:port for the publisher's gripper stream")
    ap.add_argument("--left-port", default="/dev/pika-left",
                    help="serial port of the LEFT robot arm's Pika Gripper")
    ap.add_argument("--right-port", default="/dev/pika-right",
                    help="serial port of the RIGHT robot arm's Pika Gripper")
    ap.add_argument("--swap-ports", action="store_true",
                    help="flip the left/right serial mapping")
    ap.add_argument("--ident", choices=SIDES, default=None,
                    help="wiggle one gripper to identify the port mapping, then exit")
    ap.add_argument("--min-rad", type=float, default=0.0, help="motor angle lower clamp (rad)")
    ap.add_argument("--max-rad", type=float, default=1.75,
                    help="motor angle upper clamp (rad) — Sense open measures ~1.71 rad")
    ap.add_argument("--deadband-rad", type=float, default=0.005,
                    help="skip resend below this change")
    ap.add_argument("--max-hz", type=float, default=60.0,
                    help="POSITION_CTRL max send rate per gripper")
    ap.add_argument("--stale-sec", type=float, default=0.5,
                    help="hold a side whose last packet (local arrival clock) is older")
    ap.add_argument("--engaged-only", action="store_true",
                    help="follow only while that side's deadman/clutch is engaged")
    ap.add_argument("--pika-sdk-path", default="/home/plaif/workspace/pika_sdk",
                    help="directory containing the AgileX `pika` SDK package")
    ap.add_argument("--verbose", action="store_true")
    return ap.parse_args()


def main() -> None:
    args = get_arguments()
    if args.selftest:
        selftest()
        return
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(message)s")

    ports = {"left": args.left_port, "right": args.right_port}
    if args.swap_ports:
        ports = {"left": ports["right"], "right": ports["left"]}
    gripper_cls = import_gripper_class(args.pika_sdk_path)
    driver = GripperDriver(ports, args.min_rad, args.max_rad,
                           args.deadband_rad, args.max_hz, gripper_cls=gripper_cls).start()
    try:
        if args.ident:
            run_ident(driver, args.ident)
            return

        host, port = parse_listen(args.listen)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((host, port))
        sock.setblocking(False)
        log.info("[umi] gripper bridge listening on udp://%s:%d  ports=%s%s",
                 host, port, ports, "  (engaged-only)" if args.engaged_only else "")

        latest = {}     # side -> {"gripper_rad":..., "deadman":...}
        arrival = {}    # side -> local monotonic of last packet carrying that side
        last_stale_log = 0.0
        try:
            while True:
                drained = 0
                while drained < 64:
                    try:
                        data, _addr = sock.recvfrom(65536)
                    except BlockingIOError:
                        break
                    drained += 1
                    try:
                        sides = parse_packet_sides(data)
                    except (ValueError, UnicodeDecodeError) as exc:
                        log.warning("[umi] bad packet: %s", exc)
                        continue
                    now = time.monotonic()
                    for side, entry in sides.items():
                        latest[side] = entry
                        arrival[side] = now

                now = time.monotonic()
                for side, entry in latest.items():
                    if now - arrival.get(side, -math.inf) > args.stale_sec:
                        if now - last_stale_log > 5.0:
                            last_stale_log = now
                            log.info("[umi] %s stream stale > %.1fs — holding", side, args.stale_sec)
                        continue
                    if args.engaged_only and not entry["deadman"]:
                        continue
                    driver.update(side, entry["gripper_rad"])
                time.sleep(0.005)   # 200 Hz poll; serial sends are rate-limited anyway
        except KeyboardInterrupt:
            pass
        finally:
            sock.close()
    finally:
        driver.close()
        log.info("[umi] gripper bridge stopped")


if __name__ == "__main__":
    main()
