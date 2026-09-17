#!/usr/bin/env python3
"""Pick success from rollout step logs alone, with no video, no force sensor and no annotation.

WHY A NEW METRIC. The two outcome proxies we had both turned out to be blind:

  * jaw stall (`grip_stall_pct` in analyze_grasp_events.py) assumes the fingers JAM at bolt width
    when something is between them. They do not. Measured over 647 closed holds on 2026-09-16/17,
    the jaw reaches its commanded opening to within +/-1.0% (p50 +0.1%, p90 +0.5%): the pika jaw is
    a position servo and the bolt is held by rubber compliance, not by blocking the linkage. The
    signal is absent, not merely noisy -- no threshold recovers it.
  * `wrench_tcp_fz` is not written by the current runner at all, so `fz_after_N` is NaN everywhere.

  And an earlier CARRY proxy (closed segment travels > 150 mm in xy) was retracted: it fired on
  lateral transport generally rather than on picking.

WHAT IS ACTUALLY IN THE LOG. The policy's own commitment. A pick that the policy believes succeeded
is followed by a transport to the box and a release OVER THE BOX; a pick it believes failed is
followed by a release in place, over the foam, and another descent. Those two release sites are
separated by the cell's own geometry, which is measured rather than fitted:

  the dark foam work surface is 500 x 500 mm centred at x = 0.385 m in stand frame
  (simulation/config/work_surface.json, operator-measured 2026-09-06), so ALL bolts lie at
  x <= 0.635 m, and both boxes sit beyond that edge.

Observed release x over 674 closed holds confirms the partition with an empty guard band:
releases pile up at x <= 0.65 (in place) and at x >= 0.70 (over a box); the interval
[0.635, 0.700] holds 5 of 674 samples. DELIVERY_X sits in the middle of that gap.

  ATTEMPT   a jaw-close command crossing, with the TCP over the foam.
  DELIVERY  the closed hold that follows ends with the jaw opening past DELIVERY_X, on the side
            (sign of y) belonging to that arm. Opening past DELIVERY_X on the WRONG side is
            reported separately -- a real pick, a wrong placement.
  rate      deliveries / attempts, per arm.

HOW WE KNOW THE PROXY TRACKS TRUTH, without labels. Delivery rate is a sharp monotone function of
how deep the jaw closed, which is what physics demands and what a proxy uncorrelated with the
outcome could not produce (2026-09-16/17, 36 runs):

    close z <= -0.285 m   right 88% (49/56)   left 38% (60/158)
    close z in -0.28..-0.27  right 20% (27/133)  left  6% (5/82)
    close z >  -0.27 m    right  2% (3/176)   left  1% (1/68)

So this reports BOTH axes. The rate is the headline; the close-z distribution is the mechanism,
and it is the lower-variance one -- one sample per close event rather than one bit per attempt --
which is what makes it usable for an A/B between checkpoints on a handful of runs.

WHAT IT STILL DOES NOT PROVE. A delivery means the policy committed to the box, not that a bolt was
between the fingers. Carrying air to the box inflates the rate. `gripper_current_ma` (empty jaw
0-203 mA vs bolt held 580-720 mA, measured on this hardware) is the physical confirmation; it is
logged from 2026-09-17 onward and is reported here whenever present.
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import pathlib
import statistics
import sys

ARMS = ("left", "right")

# Far edge of the 500x500 foam patch centred at x=0.385 m: no bolt can lie beyond it.
FOAM_FAR_X_M = 0.635
# Midpoint of the empty band between in-place releases and over-box releases.
DELIVERY_X_M = 0.675
# Sign band that assigns a box to an arm; the observed clusters are y in [+0.16,+0.32] (left)
# and [-0.30,-0.10] (right), so anything inside +/-0.05 is unassignable rather than mislabelled.
BOX_SIDE_Y_M = 0.05
# Table contact height: stand table -0.295 + foam 0.020 + bolt head radius 0.0092.
REST_Z_M = -0.2658
# A hold shorter than this is a jaw twitch, not a grasp-and-carry.
MIN_HOLD_S = 0.8
# Fraction of the run's own commanded jaw range below which the jaw counts as closed. Per-run
# because the absolute open/close percentages move with gripper units and bias settings.
CLOSED_FRACTION = 0.25


def _load(path: pathlib.Path) -> list[dict]:
    out = []
    with path.open() as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                d = json.loads(line)
            except json.JSONDecodeError:
                continue
            if d.get("arms"):
                out.append(d)
    return out


def _xyz(step: dict, arm: str) -> list[float] | None:
    m = step["arms"][arm].get("meas_pose")
    return m[:3] if isinstance(m, list) and len(m) >= 3 else None


def _side(y: float) -> str:
    if y > BOX_SIDE_Y_M:
        return "left"
    if y < -BOX_SIDE_Y_M:
        return "right"
    return "middle"


def holds(steps: list[dict], arm: str) -> list[dict]:
    """Every closed hold: where the jaw shut, where it opened again, and what it did between."""
    cmd = [s["arms"][arm].get("gripper_cmd_pct") for s in steps]
    known = [c for c in cmd if c is not None]
    if len(known) < 20 or max(known) - min(known) < 20.0:
        return []                              # the jaw never worked in this run
    lo, hi = min(known), max(known)
    closed_level = lo + CLOSED_FRACTION * (hi - lo)
    t = [s["t_mono"] for s in steps]

    out, i = [], 0
    while i < len(steps):
        if cmd[i] is None or cmd[i] > closed_level:
            i += 1
            continue
        j = i
        while j + 1 < len(steps) and cmd[j + 1] is not None and cmd[j + 1] <= closed_level:
            j += 1
        a, b = _xyz(steps[i], arm), _xyz(steps[j], arm)
        if t[j] - t[i] >= MIN_HOLD_S and a and b:
            zs = [p[2] for p in (_xyz(steps[k], arm) for k in range(i, j + 1)) if p]
            cur = [steps[k]["arms"][arm].get("gripper_current_ma") for k in range(i, j + 1)]
            cur = [abs(c) for c in cur if isinstance(c, (int, float))]
            delivered = b[0] > DELIVERY_X_M
            out.append({
                "step_index": i,
                "t_wall": steps[i].get("t_wall"),
                "hold_s": round(t[j] - t[i], 2),
                "close_x_m": round(a[0], 4), "close_z_m": round(a[2], 4),
                "release_x_m": round(b[0], 4), "release_y_m": round(b[1], 4),
                "release_z_m": round(b[2], 4),
                "xy_travel_mm": round(math.hypot(b[0] - a[0], b[1] - a[1]) * 1000.0, 1),
                "lift_mm": round((max(zs) - min(zs)) * 1000.0, 1) if zs else float("nan"),
                # over the foam = the policy could have been aiming at a bolt
                "attempt": a[0] <= FOAM_FAR_X_M,
                "delivered": delivered,
                "wrong_box": delivered and _side(b[1]) not in (arm, "middle"),
                # physical grip confirmation; empty on runs before 2026-09-17
                "grip_current_ma_p50": round(statistics.median(cur), 1) if cur else "",
            })
        i = j + 1
    return out


def _pct(num: int, den: int) -> str:
    return f"{100.0 * num / den:5.1f}%" if den else "    - "


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+", help="outputs/sweep/*.jsonl (globs ok)")
    ap.add_argument("--out", default="outputs/pick_success.csv")
    ap.add_argument("--per-run", action="store_true", help="one line per run per arm as well")
    ap.add_argument("--depth-bin-mm", type=float, default=5.0,
                    help="close-z bin width for the depth/outcome table")
    args = ap.parse_args()

    paths: list[pathlib.Path] = []
    for pat in args.logs:
        paths += [pathlib.Path(p) for p in sorted(glob.glob(pat)) if "chunks" not in p]
    if not paths:
        sys.exit("no logs matched")

    rows = []
    for p in paths:
        steps = _load(p)
        if len(steps) < 60:
            continue
        for arm in ARMS:
            for h in holds(steps, arm):
                h["run"], h["arm"] = p.stem, arm
                rows.append(h)
    if not rows:
        sys.exit("no closed holds found -- did the run command the gripper?")

    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    cols = ["run", "arm", "step_index", "t_wall", "hold_s", "attempt", "delivered", "wrong_box",
            "close_x_m", "close_z_m", "release_x_m", "release_y_m", "release_z_m",
            "xy_travel_mm", "lift_mm", "grip_current_ma_p50"]
    with out.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=cols)
        w.writeheader()
        for r in sorted(rows, key=lambda x: (x["run"], x["arm"], x["step_index"])):
            w.writerow({k: r.get(k, "") for k in cols})
    print(f"{len(rows)} closed holds from {len(paths)} runs -> {out}\n")

    if args.per_run:
        print(f"{'run':<34} {'arm':<6} {'att':>4} {'deliv':>6} {'rate':>7}  close-z p50")
        for run in sorted({r['run'] for r in rows}):
            for arm in ARMS:
                a = [r for r in rows if r["run"] == run and r["arm"] == arm and r["attempt"]]
                if not a:
                    continue
                d = sum(r["delivered"] for r in a)
                z = statistics.median(r["close_z_m"] for r in a)
                print(f"{run[:34]:<34} {arm:<6} {len(a):>4} {d:>6} {_pct(d, len(a)):>7}  {z:+.4f}")
        print()

    for arm in ARMS:
        a = [r for r in rows if r["arm"] == arm and r["attempt"]]
        if not a:
            continue
        air = [r for r in rows if r["arm"] == arm and not r["attempt"]]
        d = sum(r["delivered"] for r in a)
        wrong = sum(r["wrong_box"] for r in a)
        print(f"== {arm}: {len(a)} attempts over the foam, {len(air)} closes past the foam edge")
        print(f"   delivered {d} ({_pct(d, len(a))})   of which wrong box {wrong}")
        cur = [r["grip_current_ma_p50"] for r in a if r["grip_current_ma_p50"] != ""]
        if cur:
            dc = [r["grip_current_ma_p50"] for r in a
                  if r["delivered"] and r["grip_current_ma_p50"] != ""]
            mc = [r["grip_current_ma_p50"] for r in a
                  if not r["delivered"] and r["grip_current_ma_p50"] != ""]
            print(f"   grip current mA  delivered p50 "
                  f"{statistics.median(dc) if dc else float('nan'):.0f} (n={len(dc)})   "
                  f"not delivered p50 {statistics.median(mc) if mc else float('nan'):.0f} "
                  f"(n={len(mc)})")
        else:
            print("   grip current: not logged in these runs "
                  "(pre-2026-09-17 runner, or gripper server without --units)")
        w = args.depth_bin_mm / 1000.0
        bins: dict[float, list[int]] = {}
        for r in a:
            b = math.floor(r["close_z_m"] / w) * w
            bins.setdefault(b, [0, 0])
            bins[b][0] += 1
            bins[b][1] += bool(r["delivered"])
        print(f"   delivery vs close depth (REST_Z = {REST_Z_M:+.4f} m):")
        for b in sorted(bins):
            n, k = bins[b]
            bar = "#" * int(round(20.0 * k / n)) if n else ""
            print(f"      z {b:+.3f}..{b + w:+.3f}  n={n:4d}  delivered {k:4d} "
                  f"{_pct(k, n)}  {bar}")
        print()


if __name__ == "__main__":
    main()
