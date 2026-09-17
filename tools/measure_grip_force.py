#!/usr/bin/env python3
"""Grip effort vs jaw closure on the robot gripper, measured with the motor current.

The collection rig CANNOT record grip force: the Pika Sense is a passive handheld with an encoder
and nothing else (its whole SDK surface, the official API_Doc's Sense section, and the manual's
"Output Data" row all agree -- 6D pose, shaft angle, depth, RGB, IMU; no force, no current). The
ROBOT gripper does have it (`get_motor_current`, mA), so the only way to stop relying on an
accidental squeeze is to calibrate the robot side against a real object.

This does that, with the bolt still in the jaws, and gets a second thing for free.

`get_gripper_distance()` is measured from whatever angle `set_zero()` last called closed. Homing
runs on every connect, so if a bolt was in the gripper at startup the "closed stop" was defined at
the BOLT's width and the whole session's scale is shifted. Observed 2026-09-16: both arms reported
a 1.3-1.6 mm opening while holding a 12 mm shank at -580/-720 mA.

The squeeze region fixes both: current falls monotonically to zero AS THE JAW REACHES THE OBJECT's
width, so extrapolating the current-vs-position fit to zero gives the contact point WITHOUT ever
releasing the object. That contact point is a known width (`--object-mm`), so it also yields the
zero offset -- the common physical reference the collection and deploy rigs never shared.

    tools/measure_grip_force.py probe                      # read-only
    tools/measure_grip_force.py sweep --move --object-mm 12

Only the gripper moves. The object is never released: the open leg stops as soon as the squeeze
decays, and the closed leg stops at --max-current-ma.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
import time

import numpy as np

SDK_PATH = "/home/plaif/workspace/pika_sdk"
PORTS = {"left": "/dev/pika-left", "right": "/dev/pika-right"}


def connect(arms):
    if SDK_PATH not in sys.path:
        sys.path.insert(0, SDK_PATH)
    from pika.gripper import Gripper

    out = {}
    for arm in arms:
        g = Gripper(port=PORTS[arm])
        if not g.connect():
            raise SystemExit(f"{arm}: connect failed on {PORTS[arm]}")
        if not g.enable():
            raise SystemExit(f"{arm}: enable failed")
        out[arm] = g
    time.sleep(0.5)
    return out


def sample(g, n=8, dwell=0.04):
    """Median of n telemetry reads: the pika frame lands at ~18.5 Hz, so a single read can repeat."""
    cur, jaw, pos = [], [], []
    for _ in range(n):
        time.sleep(dwell)
        cur.append(float(g.get_motor_current()))
        jaw.append(float(g.get_gripper_distance()))
        pos.append(float(g.get_motor_position()))
    return float(np.median(cur)), float(np.median(jaw)), float(np.median(pos))


def probe(args) -> int:
    grippers = connect(args.arms)
    try:
        print(f"{'arm':<7}{'current mA':>12}{'jaw mm':>10}{'motor rad':>12}{'verdict':>28}")
        print("-" * 69)
        for arm, g in grippers.items():
            cur, jaw, pos = sample(g)
            v = "HOLDING something" if abs(cur) > args.hold_current_ma else "free (no squeeze)"
            print(f"{arm:<7}{cur:>12.0f}{jaw:>10.2f}{pos:>12.4f}{v:>28}")
        print("\n# a nonzero steady current with the jaw nearly shut means the zero was set while an")
        print("# object was in the jaws -- run `sweep` to recover the true contact width.")
    finally:
        for g in grippers.values():
            try:
                g.disconnect()
            except Exception:  # noqa: BLE001
                pass
    return 0


def _leg(g, start_rad, direction, args, rows, arm):
    """Walk the motor angle one way, sampling. direction +1 = open, -1 = squeeze."""
    step = args.step_rad * direction
    rad = start_rad
    for _ in range(args.max_steps):
        rad = max(0.0, min(args.max_rad, rad + step))
        g.set_motor_angle(rad)
        time.sleep(args.settle_s)
        cur, jaw, pos = sample(g)
        rows.append({"arm": arm, "cmd_rad": rad, "meas_rad": pos, "jaw_mm": jaw, "current_ma": cur})
        print(f"    cmd {rad:6.4f} -> {pos:6.4f} rad  jaw {jaw:7.2f} mm  current {cur:8.0f} mA",
              flush=True)
        if direction > 0 and abs(cur) < args.release_current_ma:
            print("    >> squeeze decayed: at/past the contact point, stopping the open leg")
            return
        if direction < 0 and abs(cur) > args.max_current_ma:
            print(f"    >> hit the {args.max_current_ma:.0f} mA cap, stopping the squeeze leg")
            return


def sweep(args) -> int:
    if not args.move:
        print("sweep MOVES the gripper (it never releases the object). Re-run with --move.",
              file=sys.stderr)
        return 2
    grippers = connect(args.arms)
    rows: list[dict] = []
    try:
        for arm, g in grippers.items():
            cur0, jaw0, rad0 = sample(g)
            print(f"\n=== {arm}: holding at {jaw0:.2f} mm (reported), {cur0:.0f} mA, {rad0:.4f} rad")
            if abs(cur0) < args.hold_current_ma:
                print(f"    !! only {abs(cur0):.0f} mA -- this arm does not look like it is gripping;"
                      f" the fit below will be meaningless")
            if args.skip_open:
                # The operator had to nearly close the jaws to get the object in, so opening even
                # a step risks dropping it. Contact is then already AT the starting position --
                # it is where the current sits at the empty-jaw baseline -- so the closing leg
                # alone carries the whole curve.
                print("  (open leg skipped: --skip-open, contact is taken as the start point)")
            else:
                print("  opening until the squeeze decays:")
                _leg(g, rad0, +1, args, rows, arm)
                g.set_motor_angle(rad0)
                time.sleep(args.settle_s)
            print("  squeezing in:")
            _leg(g, rad0, -1, args, rows, arm)
            g.set_motor_angle(rad0)
            time.sleep(args.settle_s)
            print(f"  restored to {rad0:.4f} rad")
    finally:
        for g in grippers.values():
            try:
                g.disconnect()
            except Exception:  # noqa: BLE001
                pass

    print("\n" + "=" * 78)
    summary = {}
    for arm in args.arms:
        r = [x for x in rows if x["arm"] == arm]
        if len(r) < 4:
            print(f"{arm}: too few samples")
            continue
        jaw = np.array([x["jaw_mm"] for x in r])
        cur = np.abs(np.array([x["current_ma"] for x in r]))
        sq = cur > args.release_current_ma
        if sq.sum() < 3:
            print(f"{arm}: not enough squeezing samples to fit")
            continue
        # |current| vs jaw over the squeeze region; contact = where the fit crosses zero
        slope, icept = np.polyfit(jaw[sq], cur[sq], 1)
        contact = -icept / slope if slope != 0 else float("nan")
        offset = args.object_mm - contact
        stiff = -slope  # mA per mm of further closure
        summary[arm] = {
            "contact_reported_mm": contact, "object_mm": args.object_mm,
            "zero_offset_mm": offset, "ma_per_mm": stiff,
            "samples": int(sq.sum()),
        }
        print(f"{arm}:")
        print(f"  squeeze stiffness      : {stiff:.0f} mA per mm of extra closure ({int(sq.sum())} pts)")
        print(f"  contact point (fitted) : {contact:+.2f} mm on THIS session's reported scale")
        print(f"  the object is          : {args.object_mm:.2f} mm")
        print(f"  -> ZERO OFFSET         : {offset:+.2f} mm  (add to the reported jaw to get true mm)")
        for target in (args.target_current_ma, 2 * args.target_current_ma):
            j = contact - target / stiff if stiff > 0 else float("nan")
            print(f"  to hold at {target:5.0f} mA: report {j:+.2f} mm "
                  f"= {j + offset:.2f} mm true = {args.object_mm - (j + offset):.2f} mm of squeeze")

    if args.json:
        pathlib.Path(args.json).write_text(json.dumps({"rows": rows, "summary": summary}, indent=1))
        print(f"\n# wrote {args.json}", file=sys.stderr)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name, fn in (("probe", probe), ("sweep", sweep)):
        p = sub.add_parser(name)
        p.add_argument("--arms", nargs="+", default=["left", "right"], choices=["left", "right"])
        p.add_argument("--hold-current-ma", type=float, default=200.0)
        if name == "sweep":
            p.add_argument("--move", action="store_true", help="required: this MOVES the gripper")
            p.add_argument("--object-mm", type=float, default=12.0,
                           help="true width of what is held (M12 shank = 12)")
            p.add_argument("--step-rad", type=float, default=0.01)
            p.add_argument("--max-steps", type=int, default=14)
            p.add_argument("--settle-s", type=float, default=0.5)
            p.add_argument("--max-rad", type=float, default=1.66)
            p.add_argument("--release-current-ma", type=float, default=120.0)
            p.add_argument("--max-current-ma", type=float, default=1800.0)
            p.add_argument("--target-current-ma", type=float, default=700.0)
            p.add_argument("--skip-open", action="store_true",
                           help="do not open first (the object would fall); sweep the closing leg only")
            p.add_argument("--json", default="")
        p.set_defaults(func=fn)
    args = ap.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
