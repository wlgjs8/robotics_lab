#!/usr/bin/env python3
"""What physical jaw opening does the number in `observation/state` actually mean, per device?

The pika policies are trained on the COLLECTION rig's `get_gripper_distance()`, which the SDK
documents as millimetres of jaw opening (`pika_sdk/pika/sense.py:162`). The deploy runtime does NOT
use that: `policy_runner/gripper.py:329` reports `(rad - min_rad)/(max_rad - min_rad) * 100`, a
fraction of MOTOR ANGLE, with `max_rad` a config constant (0.0/1.75, `config.py:180`). The SDK
linkage between angle and opening is a four-bar, so those two numbers are not proportional, and the
config constant has never been checked against the hardware.

This measures both at once, on the real gripper:

  probe   (default, NO MOTION) connect + enable and read `get_motor_position()`,
          `get_gripper_distance()` and the runtime's percent side by side.
  sweep   (needs --move) home to the CLOSED mechanical stop exactly as the runtime does
          (`set_motor_angle(0)` -> settle -> `set_zero`), then walk the angle open in small steps,
          recording the commanded angle, the angle the motor actually reached, and the jaw
          distance. The OPEN mechanical stop is where the reached angle stops following the
          command -- that, not the config constant, is the real `max_rad`.

Only the gripper moves. Nothing here touches the arm.

    tools/measure_gripper_units.py probe
    tools/measure_gripper_units.py sweep --move
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys
import time

SDK_PATH = "/home/plaif/workspace/pika_sdk"
PORTS = {"left": "/dev/pika-left", "right": "/dev/pika-right"}
# policy_runner/policy_runner/config.py:180-181 -- what the runtime assumes today.
RUNTIME_MIN_RAD, RUNTIME_MAX_RAD = 0.0, 1.75
# The SDK's own inverse search bound; also the geometric limit of the linkage.
SDK_MAX_ANGLE = (180.0 - 43.99) / 180.0 * math.pi
PIKA_CALIB = "/home/plaif/workspace/pika/config/gripper_calib.json"


def sdk_distance(angle: float) -> float:
    """pika_sdk/pika/gripper.py:220 -- the four-bar, in mm of half-span."""
    a = SDK_MAX_ANGLE - angle
    height = 0.0325 * math.sin(a)
    width_d = 0.0325 * math.cos(a)
    return (math.sqrt(0.058**2 - (height - 0.01456) ** 2) + width_d) * 1000.0


def sdk_jaw_mm(angle: float) -> float:
    """What `get_gripper_distance()` returns for a motor angle: mm of jaw opening."""
    return (sdk_distance(angle) - sdk_distance(0.0)) * 2.0


def runtime_percent(rad: float) -> float:
    return (rad - RUNTIME_MIN_RAD) / (RUNTIME_MAX_RAD - RUNTIME_MIN_RAD) * 100.0


def connect(arms: list[str]):
    if SDK_PATH not in sys.path:
        sys.path.insert(0, SDK_PATH)
    try:
        from pika.gripper import Gripper
    except ImportError as exc:
        raise SystemExit(f"pika SDK import failed from {SDK_PATH}: {exc}") from exc
    out = {}
    for arm in arms:
        g = Gripper(port=PORTS[arm])
        if not g.connect():
            raise SystemExit(f"{arm}: connect failed on {PORTS[arm]}")
        if not g.enable():
            raise SystemExit(f"{arm}: enable failed on {PORTS[arm]}")
        out[arm] = g
    time.sleep(0.4)  # let the telemetry thread deliver a first frame
    return out


def read(g) -> tuple[float, float]:
    return float(g.get_motor_position()), float(g.get_gripper_distance())


def settle(g, timeout: float = 3.0, eps: float = 2e-3, poll: float = 0.05) -> float:
    """Poll until two consecutive angle reads agree (the jaw stopped). Returns the final angle."""
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        time.sleep(poll)
        cur = float(g.get_motor_position())
        if last is not None and abs(cur - last) < eps:
            return cur
        last = cur
    return float(g.get_motor_position())


def probe(args) -> int:
    grippers = connect(args.arms)
    try:
        print(f"{'arm':<8}{'motor rad':>11}{'jaw mm (SDK)':>15}{'jaw mm (formula)':>18}"
              f"{'runtime pct':>13}")
        print("-" * 65)
        for arm, g in grippers.items():
            rad, mm = read(g)
            print(f"{arm:<8}{rad:>11.4f}{mm:>15.2f}{sdk_jaw_mm(rad):>18.2f}{runtime_percent(rad):>13.2f}")
        print("\n# 'jaw mm (SDK)' is what the COLLECTION rig writes into the dataset.")
        print("# 'runtime pct' is what the DEPLOY runtime puts in observation/state today.")
        print("# They are the same number only where the four-bar happens to cross.")
    finally:
        for g in grippers.values():
            try:
                g.disconnect()
            except Exception:  # noqa: BLE001
                pass
    return 0


def sweep(args) -> int:
    if not args.move:
        print("sweep MOVES the gripper. Re-run with --move once the jaws are clear.", file=sys.stderr)
        return 2
    grippers = connect(args.arms)
    results: dict[str, dict] = {}
    try:
        for arm, g in grippers.items():
            rad0, mm0 = read(g)
            print(f"\n=== {arm}: starting at {rad0:.4f} rad / {mm0:.2f} mm")

            # Home exactly as the runtime does, so the zero we measure from is the runtime's zero.
            print("  homing to the closed stop ...", flush=True)
            g.set_motor_angle(0.0)
            closed_rad = settle(g, timeout=args.settle_s)
            closed_mm_before = float(g.get_gripper_distance())
            if hasattr(g, "set_zero"):
                g.set_zero()
                time.sleep(0.3)
            z_rad, z_mm = read(g)
            print(f"  closed stop: {closed_rad:.4f} rad -> after set_zero {z_rad:.4f} rad, "
                  f"jaw {z_mm:.2f} mm (was {closed_mm_before:.2f} before re-zero)")

            rows = []
            stop_rad = None
            cmd = 0.0
            while cmd <= args.max_rad + 1e-9:
                g.set_motor_angle(cmd)
                reached = settle(g, timeout=args.settle_s)
                mm = float(g.get_gripper_distance())
                lag = cmd - reached
                rows.append({"cmd_rad": cmd, "reached_rad": reached, "jaw_mm": mm,
                             "lag_rad": lag, "formula_mm": sdk_jaw_mm(reached),
                             "runtime_pct": runtime_percent(cmd)})
                print(f"  cmd {cmd:5.3f} -> reached {reached:6.4f} (lag {lag:+6.4f})  "
                      f"jaw {mm:7.2f} mm   runtime would call this {runtime_percent(cmd):5.1f}")
                if lag > args.stall_lag_rad:
                    stop_rad = reached
                    print(f"  >> motor stopped following: OPEN MECHANICAL STOP at "
                          f"{reached:.4f} rad / {mm:.2f} mm")
                    break
                cmd += args.step_rad

            # leave it where it started
            g.set_motor_angle(max(0.0, min(args.max_rad, rad0 if stop_rad is None else min(rad0, stop_rad))))
            settle(g, timeout=args.settle_s)
            results[arm] = {
                "closed_zero_rad": z_rad, "closed_zero_mm": z_mm,
                "open_stop_rad": stop_rad, "open_stop_mm": rows[-1]["jaw_mm"] if rows else None,
                "rows": rows,
            }
    finally:
        for g in grippers.values():
            try:
                g.disconnect()
            except Exception:  # noqa: BLE001
                pass

    print("\n" + "=" * 78)
    calib = None
    p = pathlib.Path(PIKA_CALIB)
    if p.exists():
        calib = {r["arm"]: r for r in json.loads(p.read_text())["results"]}
        print(f"# collection rig calibration ({PIKA_CALIB}):")
        for arm, r in calib.items():
            print(f"#   {arm:<6} closed {r['closed']:+7.2f} mm   open(bolt stop) {r['open']:7.2f} mm")
    print()
    for arm, res in results.items():
        stop = res["open_stop_rad"]
        print(f"{arm}:")
        print(f"  measured full open    : {stop if stop is not None else float('nan'):.4f} rad "
              f"/ {res['open_stop_mm']:.2f} mm")
        print(f"  runtime assumes max_rad: {RUNTIME_MAX_RAD:.4f} rad "
              f"(= {sdk_jaw_mm(RUNTIME_MAX_RAD):.2f} mm by the linkage)")
        if stop:
            print(f"  -> runtime 100% commands {runtime_percent(stop):.1f}% past / short of the real stop")
        print(f"  closed stop reads      : {res['closed_zero_mm']:+.2f} mm after set_zero")
        if calib and arm in calib:
            off = calib[arm]["closed"] - res["closed_zero_mm"]
            print(f"  collection closed reads: {calib[arm]['closed']:+.2f} mm  -> ZERO OFFSET "
                  f"{off:+.2f} mm (add this to the robot's mm to match the dataset)")

    if args.json:
        pathlib.Path(args.json).write_text(json.dumps(results, indent=1))
        print(f"\n# wrote {args.json}", file=sys.stderr)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name, fn in (("probe", probe), ("sweep", sweep)):
        p = sub.add_parser(name)
        p.add_argument("--arms", nargs="+", default=["left", "right"], choices=["left", "right"])
        if name == "sweep":
            p.add_argument("--move", action="store_true", help="required: this MOVES the gripper")
            p.add_argument("--step-rad", type=float, default=0.10)
            p.add_argument("--max-rad", type=float, default=SDK_MAX_ANGLE)
            p.add_argument("--settle-s", type=float, default=2.0)
            p.add_argument("--stall-lag-rad", type=float, default=0.08,
                           help="commanded-minus-reached angle that counts as the mechanical stop")
            p.add_argument("--json", default="")
        p.set_defaults(func=fn)
    args = ap.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
