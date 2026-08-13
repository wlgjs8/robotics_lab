#!/usr/bin/env python3
"""Per-grasp forensics from flow-infer step logs: what was true at the instant the gripper closed.

Why this exists. Val measurements have ruled out the model side of the pick failure:
  * at the deployment operating point (5 rows = 167 ms, then replan) the policy predicts the grasp
    point to 2.7 mm in z on the right arm, 1.9 mm on the left -- better than a non-parametric kNN
    over frozen DINOv3 features, which is a ceiling no trained model can be blamed for missing;
  * grasp-instant z error is 0.37 mm median, and close-event timing is one frame (33 ms) with 100%
    detection;
  * z is the policy's BEST-predicted axis, not its worst.
So the ~5-10 mm the arm visibly comes up short is not perception. The leading remaining suspect is
that the gripper closes on schedule while the ARM is still behind its own plan: cmd-vs-meas sits
near 1 mm typically but was seen at 5.5 mm, which is exactly the observed shortfall. Only five close
events existed in the logs to hand -- far too few. This turns each rollout into labelled evidence.

Outputs one row per close event with the state that would explain a miss, plus two outcome proxies
that need no human labelling:

  grip_stall_pct  -- the opening the fingers SETTLE at after closing. Closing on a bolt jams the
                     fingers at roughly the bolt width; closing on air runs them to ~0. This is the
                     most direct "did it get the bolt" signal in the log.
  fz_after_N      -- tool-frame normal force after the close. A held bolt shows sustained load.

Both are proxies. Annotate the CSV's `outcome` column from the video for the runs you watch, then
`--check-proxy` reports how well they agree, so later runs can be scored without watching.
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


def _series(steps: list[dict], arm: str, key: str):
    return [s["arms"][arm].get(key) for s in steps]


def _gap_mm(s: dict, arm: str) -> tuple[float, float]:
    """(euclidean, signed z) command-minus-measured TCP gap in mm."""
    c = s["arms"][arm].get("cmd_pose") or []
    m = s["arms"][arm].get("meas_pose") or []
    if len(c) < 3 or len(m) < 3:
        return float("nan"), float("nan")
    d = [(c[i] - m[i]) * 1000.0 for i in range(3)]
    return math.sqrt(sum(x * x for x in d)), d[2]


def find_close_events(steps: list[dict], arm: str, settle_steps: int) -> list[dict]:
    """A close event = commanded opening crosses down through the midpoint of its own range.

    Threshold is per-run, from the commanded percentiles, because the absolute open/close values
    differ per arm and per gripper-bias setting rather than being fixed constants.
    """
    g = [x for x in _series(steps, arm, "gripper_cmd_pct") if x is not None]
    if len(g) < 20:
        return []
    lo, hi = min(g), max(g)
    if hi - lo < 5.0:                       # gripper never really moved in this run
        return []
    mid = lo + 0.5 * (hi - lo)
    open_level = lo + 0.8 * (hi - lo)
    cmd = _series(steps, arm, "gripper_cmd_pct")
    meas = _series(steps, arm, "gripper_meas_pct")

    events, armed = [], False
    for i in range(1, len(cmd)):
        if cmd[i] is None or cmd[i - 1] is None:
            continue
        if cmd[i] >= open_level:
            armed = True
            continue
        if armed and cmd[i - 1] >= mid > cmd[i]:
            armed = False
            gap, gapz = _gap_mm(steps[i], arm)
            # approach state over the ~0.3 s before the close decision
            w = [j for j in range(max(0, i - 10), i)]
            gaps = [_gap_mm(steps[j], arm)[0] for j in w]
            gaps = [x for x in gaps if not math.isnan(x)]
            zs = [steps[j]["arms"][arm]["meas_pose"][2] for j in w
                  if len(steps[j]["arms"][arm].get("meas_pose") or []) >= 3]
            dt = (steps[i]["t_mono"] - steps[w[0]]["t_mono"]) if w else float("nan")
            vz = ((zs[-1] - zs[0]) / dt * 1000.0) if len(zs) >= 2 and dt and dt > 0 else float("nan")
            # settle window AFTER the close: where the fingers end up, and what force is held
            s2 = [j for j in range(i, min(len(steps), i + settle_steps))]
            mv = [meas[j] for j in s2 if meas[j] is not None]
            fz = [steps[j]["arms"][arm].get("wrench_tcp_fz") for j in s2]
            fz = [x for x in fz if x is not None]
            fz_pre = [steps[j]["arms"][arm].get("wrench_tcp_fz") for j in w]
            fz_pre = [x for x in fz_pre if x is not None]
            events.append({
                "arm": arm,
                "step_index": i,
                "t_wall": steps[i].get("t_wall"),
                "chunk_id": steps[i].get("chunk_id"),
                "gap_mm": round(gap, 3),
                "gap_z_mm": round(gapz, 3),
                "gap_mm_max_pre": round(max(gaps), 3) if gaps else float("nan"),
                "gap_mm_mean_pre": round(statistics.fmean(gaps), 3) if gaps else float("nan"),
                "meas_z_m": round(steps[i]["arms"][arm]["meas_pose"][2], 5),
                "descent_mm_s": round(vz, 2) if not math.isnan(vz) else float("nan"),
                "grip_cmd_pct": round(cmd[i], 2),
                "grip_meas_pct_at_close": round(meas[i], 2) if meas[i] is not None else float("nan"),
                # PROXY 1: fingers jam at roughly bolt width if something is between them
                "grip_stall_pct": round(min(mv), 2) if mv else float("nan"),
                "fz_pre_N": round(statistics.fmean(fz_pre), 2) if fz_pre else float("nan"),
                # PROXY 2: load still carried after the close
                "fz_after_N": round(statistics.fmean(fz), 2) if fz else float("nan"),
                "fz_after_max_N": round(max(fz, key=abs), 2) if fz else float("nan"),
                "outcome": "",          # annotate: success | miss | knocked | (blank = unlabelled)
            })
    return events


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+", help="outputs/sweep/*.jsonl (globs ok)")
    ap.add_argument("--out", default="outputs/grasp_events.csv")
    ap.add_argument("--settle-steps", type=int, default=75,
                    help="steps after the close over which the fingers settle (~2.5 s at 30 Hz). "
                         "MUST exceed the closing duration or grip_stall_pct measures closing LATENCY "
                         "rather than grip: the right gripper takes ~44 frames (1.5 s) to close in the "
                         "training data, and at a 0.5 s window 18/46 right-arm events looked 'gripped' "
                         "that were merely still moving -- only 4 survive at 2.5 s.")
    ap.add_argument("--append", action="store_true",
                    help="append to an existing csv, keeping outcomes already annotated")
    ap.add_argument("--check-proxy", action="store_true",
                    help="report agreement between the annotated outcome and the two proxies")
    args = ap.parse_args()

    paths: list[pathlib.Path] = []
    for pat in args.logs:
        paths += [pathlib.Path(p) for p in sorted(glob.glob(pat))]
    if not paths:
        sys.exit("no logs matched")

    rows = []
    for p in paths:
        steps = _load(p)
        if len(steps) < 30:
            continue
        for arm in ARMS:
            for ev in find_close_events(steps, arm, args.settle_steps):
                ev["run"] = p.stem
                rows.append(ev)
    if not rows:
        sys.exit("no close events found -- did the run actually command the gripper?")

    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    prior: dict[tuple, str] = {}
    if args.append and out.exists():
        with out.open() as fh:
            for r in csv.DictReader(fh):
                if r.get("outcome"):
                    prior[(r["run"], r["arm"], r["step_index"])] = r["outcome"]
    for r in rows:
        key = (r["run"], r["arm"], str(r["step_index"]))
        if key in prior:
            r["outcome"] = prior[key]

    cols = ["run", "arm", "step_index", "t_wall", "chunk_id", "outcome",
            "gap_mm", "gap_z_mm", "gap_mm_max_pre", "gap_mm_mean_pre",
            "meas_z_m", "descent_mm_s", "grip_cmd_pct", "grip_meas_pct_at_close",
            "grip_stall_pct", "fz_pre_N", "fz_after_N", "fz_after_max_N"]
    with out.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=cols)
        w.writeheader()
        for r in sorted(rows, key=lambda x: (x["run"], x["step_index"])):
            w.writerow({k: r.get(k, "") for k in cols})
    print(f"{len(rows)} close events from {len(paths)} runs -> {out}")

    def med(xs):
        xs = [x for x in xs if isinstance(x, float) and not math.isnan(x)]
        return round(statistics.median(xs), 2) if xs else float("nan")

    for arm in ARMS:
        a = [r for r in rows if r["arm"] == arm]
        if not a:
            continue
        print(f"  {arm:5} n={len(a):3d}  gap {med([r['gap_mm'] for r in a]):5.2f} mm "
              f"(pre-max {med([r['gap_mm_max_pre'] for r in a]):5.2f})  "
              f"z gap {med([r['gap_z_mm'] for r in a]):+5.2f}  "
              f"stall {med([r['grip_stall_pct'] for r in a]):5.2f}%  "
              f"fz_after {med([r['fz_after_N'] for r in a]):6.2f} N")

    labelled = [r for r in rows if r["outcome"]]
    if not labelled:
        print("\nNo outcomes annotated yet. Fill the `outcome` column (success | miss | knocked) for the\n"
              "attempts you watched, then re-run with --check-proxy. The question this answers:\n"
              "  does gap_mm at the close instant separate the misses from the successes?")
        return

    print(f"\nlabelled {len(labelled)}")
    for lab in sorted({r["outcome"] for r in labelled}):
        g = [r for r in labelled if r["outcome"] == lab]
        print(f"  {lab:8} n={len(g):3d}  gap {med([r['gap_mm'] for r in g]):5.2f} mm  "
              f"z gap {med([r['gap_z_mm'] for r in g]):+5.2f}  "
              f"pre-max {med([r['gap_mm_max_pre'] for r in g]):5.2f}  "
              f"stall {med([r['grip_stall_pct'] for r in g]):5.2f}%  "
              f"fz_after {med([r['fz_after_N'] for r in g]):6.2f} N")
    if args.check_proxy:
        ok = [r for r in labelled if r["outcome"] == "success"]
        bad = [r for r in labelled if r["outcome"] != "success"]
        if ok and bad:
            print("\nproxy separation (higher |difference| = the proxy can replace watching):")
            for key in ("grip_stall_pct", "fz_after_N", "gap_mm"):
                print(f"  {key:16} success {med([r[key] for r in ok]):7.2f}   "
                      f"non-success {med([r[key] for r in bad]):7.2f}")


if __name__ == "__main__":
    main()
