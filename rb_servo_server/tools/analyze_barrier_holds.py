#!/usr/bin/env python3
"""Where the self-collision barrier slowed or HELD an arm, per episode, from a servo log.

Run it after a session; it streams the CSV (they run to tens of GB) and prints one row
per episode plus a summary by arm / class / pair / stand-frame x.

    python3 rb_servo_server/tools/analyze_barrier_holds.py logs/servo_log_*.csv
    python3 rb_servo_server/tools/analyze_barrier_holds.py --min-s 0.1 --json out.json logs/servo_log_x.csv

WHY THIS IS NOT `self_collision_clamp_count`. That counter, the SelfCollision verdict and
motion_state only move when the projection removes more than 2 deg/s from an arm. While
the barrier HOLDS a pair at its floor the hold fold re-books the plan onto the held pose
every tick, so the per-tick correction stays near 1 deg/s and none of the three move: a
1.57 s hold measured in servo_log_20260910_165651 (left link6 <-> right gripper) logged
Ok / Running / clamp +0 for its whole length while ~21 mm of commanded motion was
discarded. Over 09-10..09-15 that state ran 24% of the right arm's time above 150 mm at
stand-frame x < 0.45 (arm<->arm and arm<->stand pairs), against under 1% farther out.

Logs from 2026-09-15 or later carry `<side>_barrier_*` and are read directly. Older logs
are RECONSTRUCTED from the columns they do have (applied correction + hold fold +
projection headroom); reconstruction is labelled in the output and cannot name the pair
per arm, because the log only ever carried the tightest pair of the whole solve.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import pathlib
import sys
from typing import Any

SIDES = ("left", "right")
# The episode rules mirror control/barrier_status.hpp so live reports and this offline
# reader cannot disagree.
BRIDGE_S = 0.05
BLOCKED_DEG_S = 2.0
HELD_HEADROOM_M = 0.0005
CORRECTION_EPS_DEG_S = 0.05  # reconstruction only; the live rule is the solver's own row


def _f(row: dict[str, str], key: str, default: float = float("nan")) -> float:
    value = row.get(key)
    if value is None or value == "":
        return default
    try:
        return float(value)
    except ValueError:
        return default


def _truthy(row: dict[str, str], key: str) -> bool:
    return str(row.get(key, "")).strip().lower() in {"1", "true"}


class EpisodeBuilder:
    """One arm's held (or braking) episodes from a stream of ticks."""

    def __init__(self, arm: str, kind: str) -> None:
        self.arm = arm
        self.kind = kind
        self.open: dict[str, Any] | None = None
        self.done: list[dict[str, Any]] = []

    def update(self, t: float, active: bool, *, pair: str, klass: str, headroom_m: float,
               correction: float, folded_m: float, tcp: tuple[float, float, float],
               dt: float) -> None:
        if active:
            if self.open is None:
                self.open = {
                    "arm": self.arm, "kind": self.kind, "t_start_s": t, "t_end_s": t,
                    "duration_s": 0.0, "ticks": 0, "blocked_ticks": 0,
                    "min_headroom_mm": float("nan"), "pair": pair, "class": klass,
                    "folded_mm": 0.0, "max_correction_deg_s": 0.0,
                    "tcp_start_m": [round(v, 4) for v in tcp],
                }
            e = self.open
            e["t_end_s"] = t
            e["duration_s"] = t - e["t_start_s"] + dt
            e["ticks"] += 1
            if correction > BLOCKED_DEG_S:
                e["blocked_ticks"] += 1
            e["folded_mm"] += folded_m * 1000.0
            e["max_correction_deg_s"] = max(e["max_correction_deg_s"], correction)
            if math.isfinite(headroom_m):
                mm = headroom_m * 1000.0
                if math.isnan(e["min_headroom_mm"]) or mm < e["min_headroom_mm"]:
                    e["min_headroom_mm"] = mm
                    if pair:
                        e["pair"], e["class"] = pair, klass
            if not e["pair"] and pair:
                e["pair"], e["class"] = pair, klass
        elif self.open is not None and t - self.open["t_end_s"] > BRIDGE_S:
            self.done.append(self.open)
            self.open = None

    def finish(self) -> list[dict[str, Any]]:
        if self.open is not None:
            self.done.append(self.open)
            self.open = None
        for e in self.done:
            e["seen_by_clamp_count_pct"] = (
                100.0 * e["blocked_ticks"] / e["ticks"] if e["ticks"] else 0.0
            )
            e["folded_mm"] = round(e["folded_mm"], 1)
            e["duration_s"] = round(e["duration_s"], 3)
        return self.done


def analyze(path: pathlib.Path) -> dict[str, Any]:
    with path.open(newline="") as fh:
        reader = csv.DictReader(fh)
        fields = set(reader.fieldnames or [])
        native = {f"{s}_barrier_held" for s in SIDES} <= fields
        held = {s: EpisodeBuilder(s, "held") for s in SIDES}
        braking = {s: EpisodeBuilder(s, "braking") for s in SIDES}
        t0: float | None = None
        t_prev: dict[str, float] = {}
        occupancy: dict[tuple[str, str], float] = {}
        last_t = 0.0
        for row in reader:
            ns = _f(row, "loop_start_time_ns")
            if not math.isfinite(ns):
                continue
            t = ns * 1e-9
            if t0 is None:
                t0 = t
            t -= t0
            dt = min(max(t - last_t, 0.0), 0.1)
            last_t = t
            for side in SIDES:
                tcp = (_f(row, f"{side}_tcp_command_stand_x_m"),
                       _f(row, f"{side}_tcp_command_stand_y_m"),
                       _f(row, f"{side}_tcp_command_stand_z_m"))
                correction = _f(row, f"{side}_projection_applied_correction_deg_s", 0.0)
                folded = _f(row, f"{side}_hold_fold_m", 0.0)
                if native:
                    is_held = _truthy(row, f"{side}_barrier_held")
                    is_braking = _truthy(row, f"{side}_barrier_braking") or is_held
                    pair = row.get(f"{side}_barrier_pair", "") or ""
                    klass = row.get(f"{side}_barrier_class", "") or ""
                    headroom = _f(row, f"{side}_barrier_headroom_m")
                else:
                    # Reconstruction: this arm's command was cut, the hold fold booked the
                    # shortfall, and the solve's tightest row was at its floor.
                    headroom = _f(row, "projection_min_headroom_m")
                    pair = row.get("projection_min_headroom_pair", "") or ""
                    klass = row.get("projection_min_headroom_class", "") or ""
                    is_braking = correction > CORRECTION_EPS_DEG_S
                    is_held = (is_braking and folded > 0.0
                               and math.isfinite(headroom) and headroom <= HELD_HEADROOM_M)
                held[side].update(t, is_held, pair=pair, klass=klass, headroom_m=headroom,
                                  correction=correction, folded_m=folded, tcp=tcp, dt=dt)
                braking[side].update(t, is_braking and not is_held, pair=pair, klass=klass,
                                     headroom_m=headroom, correction=correction,
                                     folded_m=folded, tcp=tcp, dt=dt)
                if math.isfinite(tcp[0]):
                    key = (side, _xbin(tcp[0]))
                    occupancy[key] = occupancy.get(key, 0.0) + dt
                t_prev[side] = t
    episodes = []
    for side in SIDES:
        episodes += held[side].finish() + braking[side].finish()
    episodes.sort(key=lambda e: e["t_start_s"])
    return {"log": path.name, "native_columns": native, "episodes": episodes,
            "occupancy_s": {f"{s}|{b}": round(v, 1) for (s, b), v in sorted(occupancy.items())}}


def _xbin(x: float) -> str:
    for edge in (0.42, 0.45, 0.48, 0.52, 0.56):
        if x < edge:
            return f"x<{edge}"
    return "x>=0.56"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+", help="servo_log_*.csv (globs expanded by the shell)")
    ap.add_argument("--min-s", type=float, default=0.2,
                    help="only print episodes at least this long (default 0.2)")
    ap.add_argument("--kind", choices=("held", "braking", "both"), default="both")
    ap.add_argument("--json", help="also write the full result (all episodes) here")
    args = ap.parse_args()

    results = []
    for name in args.logs:
        path = pathlib.Path(name)
        if not path.exists():
            print(f"  [skip] {name}: not found", file=sys.stderr)
            continue
        results.append(analyze(path))

    if not results:
        print("no logs read", file=sys.stderr)
        return 2

    total_held = {s: 0.0 for s in SIDES}
    by_class: dict[str, float] = {}
    by_pair: dict[str, float] = {}
    by_x: dict[str, float] = {}
    shown = 0
    for res in results:
        label = "" if res["native_columns"] else "  (RECONSTRUCTED from a pre-2026-09-15 log)"
        print(f"\n== {res['log']}{label}")
        print(f"{'kind':8}{'arm':6}{'t_start':>9}{'dur_s':>7}{'headroom':>9}{'folded':>8}"
              f"{'corr':>6}{'clampcnt':>9}  pair [class]")
        for e in res["episodes"]:
            if e["kind"] == "held":
                total_held[e["arm"]] += e["duration_s"]
                by_class[e["class"] or "?"] = by_class.get(e["class"] or "?", 0.0) + e["duration_s"]
                by_pair[e["pair"] or "?"] = by_pair.get(e["pair"] or "?", 0.0) + e["duration_s"]
                by_x[_xbin(e["tcp_start_m"][0])] = (
                    by_x.get(_xbin(e["tcp_start_m"][0]), 0.0) + e["duration_s"])
            if e["duration_s"] < args.min_s:
                continue
            if args.kind != "both" and e["kind"] != args.kind:
                continue
            shown += 1
            print(f"{e['kind']:8}{e['arm']:6}{e['t_start_s']:9.2f}{e['duration_s']:7.2f}"
                  f"{e['min_headroom_mm']:9.2f}{e['folded_mm']:8.1f}"
                  f"{e['max_correction_deg_s']:6.1f}{e['seen_by_clamp_count_pct']:8.0f}%  "
                  f"{e['pair']} [{e['class']}]")
    if shown == 0:
        print("\n(no episode reached --min-s)")
    print("\n== held time")
    for side in SIDES:
        print(f"  {side:6}{total_held[side]:8.2f} s")
    for title, table in (("by class", by_class), ("by stand-frame x at episode start", by_x),
                         ("by pair (top 8)", dict(sorted(by_pair.items(), key=lambda kv: -kv[1])[:8]))):
        if table:
            print(f"\n== held time {title}")
            for key, value in sorted(table.items(), key=lambda kv: -kv[1]):
                print(f"  {value:8.2f} s  {key}")
    if args.json:
        pathlib.Path(args.json).write_text(json.dumps(results, indent=1))
        print(f"\nwrote {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
