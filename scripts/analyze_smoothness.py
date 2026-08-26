#!/usr/bin/env python3
"""Command-stream smoothness regression report for a 500 Hz servo log.

Turns the smoothness signatures measured on 2026-08-26 into a repeatable
gate so control changes (wire precision, safety-layer shaping, per-arm
cadence work) can be A/B'd run-to-run instead of by ear:

  accel tail       per-arm |d2 q_sent| and |d2 q_ref| exceedance rates
                   (>500/2000/5000/10000 deg/s^2), worst event + timestamp
  transitions      safety-verdict boundary entries/exits (clamp classes and
                   IkFailed separately), toggle rate, worst accel in a
                   +/-4-tick window around each transition
  re-anchors       chunk-follower re-anchor/warm-resume counter steps and the
                   peak accel within 10 ticks after each event
  fault onsets     fault_latched rising edges: accel at the latch tick and the
                   command age vs timeout (deadline-race signature: age within
                   4 ms of the timeout at the moment of the latch)
  pre-send         in-tick compute latency (send_start - loop_start) overall
                   and split Ok vs stressed verdicts
  wire (new cols)  worker mailbox skips/repeats and true wire send-period
                   stats from *_worker_wire_send_start_ns, when present
  projection       geometric-projection engagement, ceiling-clamp ticks
                   (1-tick velocity step -- should stay 0), min margin,
                   collision verdict age, when present

Columns are resolved by NAME with fallbacks across logger generations; any
missing group degrades to "n/a" rather than failing the whole report.

Usage:
  scripts/analyze_smoothness.py [logs/servo_log.csv]
      [--start-sec S] [--duration-sec D] [--json] [--top N]
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from collections import deque

TICK_SEC = 0.002  # 500 Hz servo tick; period_ms is stable enough to fix this
ACCEL_BINS = (500.0, 2000.0, 5000.0, 10000.0)  # deg/s^2 exceedance thresholds
CLAMP_VERDICTS = {
    "JointLimitClamped",
    "SelfCollision",
    "RoiViolation",
    "FloorViolation",
}
TRANSITION_WINDOW = 4     # ticks on each side of a verdict transition
REANCHOR_WINDOW = 10      # ticks after a re-anchor counter step
DEADLINE_RACE_MARGIN_MS = 4.0

ARMS = ("left", "right")

# Counter columns whose per-tick increase marks a follower discontinuity event.
# Names vary across logger generations; each entry lists accepted fallbacks.
REANCHOR_COLUMN_VARIANTS = (
    ("follower_reanchor_count",),
    ("follower_divergence_reanchor_count",),
    ("follower_lead_reanchor_explained_count",),
    ("follower_lead_reanchor_unexplained_count",),
    ("follower_warm_resume_count",),
)


def _f(row, idx, default=0.0):
    if idx is None:
        return default
    v = row[idx]
    if v == "" or v is None:
        return default
    try:
        return float(v)
    except ValueError:
        return default


class AccelTracker:
    """Second difference of a 6-joint column block, in deg/s^2 (max over joints)."""

    def __init__(self, header, prefix):
        self.idx = []
        for j in range(6):
            name = f"{prefix}_{j}"
            self.idx.append(header.index(name) if name in header else None)
        self.available = all(i is not None for i in self.idx)
        self._q = None
        self._d = None

    def step(self, row):
        if not self.available:
            return None
        q = [_f(row, i) for i in self.idx]
        accel = None
        if self._q is not None:
            d = [a - b for a, b in zip(q, self._q)]
            if self._d is not None:
                accel = max(abs(a - b) for a, b in zip(d, self._d)) / (TICK_SEC * TICK_SEC)
            self._d = d
        self._q = q
        return accel


class TailStats:
    def __init__(self):
        self.n = 0
        self.exceed = [0] * len(ACCEL_BINS)
        self.max = 0.0
        self.max_t = 0.0

    def add(self, accel, t_sec):
        self.n += 1
        for k, th in enumerate(ACCEL_BINS):
            if accel > th:
                self.exceed[k] += 1
        if accel > self.max:
            self.max = accel
            self.max_t = t_sec

    def report(self):
        dur = self.n * TICK_SEC
        return {
            "ticks": self.n,
            "exceed_per_sec": {
                str(int(th)): (self.exceed[k] / dur if dur > 0 else 0.0)
                for k, th in enumerate(ACCEL_BINS)
            },
            "max_deg_s2": self.max,
            "max_t_sec": self.max_t,
        }


class TransitionTracker:
    """Verdict-class boundary transitions with a +/-window accel peak."""

    def __init__(self, name):
        self.name = name
        self.entries = 0
        self.exits = 0
        self.window_peaks = []      # (peak_accel, t_sec, kind)
        self._recent = deque(maxlen=TRANSITION_WINDOW)
        self._pending = []          # [remaining_ticks, peak, t_sec, kind]
        self._in = False

    def step(self, active, accel, t_sec):
        a = accel if accel is not None else 0.0
        if active != self._in:
            kind = "enter" if active else "exit"
            peak = max(list(self._recent) + [a]) if self._recent else a
            self._pending.append([TRANSITION_WINDOW, peak, t_sec, kind])
            if active:
                self.entries += 1
            else:
                self.exits += 1
            self._in = active
        for p in self._pending:
            p[0] -= 1
            if a > p[1]:
                p[1] = a
        done = [p for p in self._pending if p[0] <= 0]
        self._pending = [p for p in self._pending if p[0] > 0]
        for p in done:
            self.window_peaks.append((p[1], p[2], p[3]))
        self._recent.append(a)

    def report(self, duration_sec, top):
        peaks = sorted(self.window_peaks, reverse=True)
        n = len(self.window_peaks)
        over = sum(1 for p, _, _ in self.window_peaks if p > 2000.0)
        return {
            "entries": self.entries,
            "exits": self.exits,
            "toggle_per_min": (self.entries * 60.0 / duration_sec) if duration_sec > 0 else 0.0,
            "windows_over_2000": over,
            "windows_total": n,
            "worst": [
                {"accel_deg_s2": p, "t_sec": t, "kind": k} for p, t, k in peaks[:top]
            ],
        }


def analyze(path, start_sec=0.0, duration_sec=None, top=5):
    out = {"file": str(path)}
    with open(path, newline="") as handle:
        reader = csv.reader(handle)
        header = next(reader)
        col = {name: i for i, name in enumerate(header)}

        def find(*names):
            for n in names:
                if n in col:
                    return col[n]
            return None

        i_tick = find("tick")
        i_loop_ns = find("loop_start_time_ns")
        i_verdict = find("safety_verdict")
        i_fault = find("fault_latched")
        i_reason = find("fault_reason")
        i_age = find("command_buffer_returned_age_ms")
        i_timeout = find("command_buffer_latest_timeout_ms")
        send_start = {a: find(f"{a}_send_start_ns") for a in ARMS}
        wire_start = {a: find(f"{a}_worker_wire_send_start_ns") for a in ARMS}
        wire_counters = {
            a: {
                "overwrites": find(f"{a}_worker_pending_overwrites_total"),
                "repeats": find(f"{a}_worker_repeated_sends_total"),
                "dispatches": find(f"{a}_worker_wire_dispatches_total"),
            }
            for a in ARMS
        }
        state_stamp = {a: find(f"{a}_state_host_time_ns") for a in ARMS}
        i_proj_active = find("projection_active")
        i_proj_count = find("projection_constraint_count")
        i_proj_ceiling = find("projection_ceiling_clamped")
        i_proj_margin = find("projection_min_margin_m")
        i_proj_age = find("selfcol_verdict_age_ms")
        proj_corr = {a: find(f"{a}_projection_correction_deg_s") for a in ARMS}

        reanchor_cols = {a: [] for a in ARMS}
        for a in ARMS:
            for variants in REANCHOR_COLUMN_VARIANTS:
                idx = find(*[f"{a}_{v}" for v in variants])
                if idx is not None:
                    reanchor_cols[a].append(idx)

        sent_acc = {a: AccelTracker(header, f"{a}_q_sent") for a in ARMS}
        ref_acc = {a: AccelTracker(header, f"{a}_q_ref") for a in ARMS}
        sent_tail = {a: TailStats() for a in ARMS}
        ref_tail = {a: TailStats() for a in ARMS}
        clamp_tr = {a: TransitionTracker("clamp") for a in ARMS}
        ik_tr = {a: TransitionTracker("ik") for a in ARMS}

        reanchor_prev = {a: None for a in ARMS}
        reanchor_events = {a: [] for a in ARMS}   # (t_sec, peak_after)
        reanchor_pending = {a: [] for a in ARMS}  # [remaining, peak, t_sec]

        fault_prev = 0
        fault_onsets = []
        # Readback sampling repeats: consecutive ticks whose cached state frame
        # carries the SAME host stamp. Their 0-then-2x catch-up pair in q_ref
        # mimics a wire doubled step (2026-08-26: contaminated the lag-8 event
        # set ~40%); q_ref analyses must be filtered on this.
        state_dup = {a: 0 for a in ARMS}
        state_prev = {a: None for a in ARMS}
        presend = {"ok": [0.0, 0], "stressed": [0.0, 0], "max": 0.0}
        wire_prev = {a: None for a in ARMS}
        wire_periods = {a: TailStatsLike() for a in ARMS}
        wire_first = {a: None for a in ARMS}
        wire_last = {a: None for a in ARMS}
        proj = {
            "ticks_active": 0,
            "ceiling_clamped_ticks": 0,
            "max_constraints": 0,
            "min_margin_m": None,
            "max_verdict_age_ms": 0.0,
            "max_correction_deg_s": {a: 0.0 for a in ARMS},
        }
        min_abs_dq0 = None
        prev_q0 = None
        rows = 0
        end_sec = None if duration_sec is None else start_sec + duration_sec

        for row in reader:
            if i_tick is None:
                break
            t_sec = _f(row, i_tick) * TICK_SEC
            if t_sec < start_sec:
                # keep trackers warm so derivatives are valid at the window edge
                for a in ARMS:
                    sent_acc[a].step(row)
                    ref_acc[a].step(row)
                continue
            if end_sec is not None and t_sec > end_sec:
                break
            rows += 1
            verdict = row[i_verdict] if i_verdict is not None else ""
            fault = int(_f(row, i_fault))

            for a in ARMS:
                sa = sent_acc[a].step(row)
                ra = ref_acc[a].step(row)
                if sa is not None:
                    sent_tail[a].add(sa, t_sec)
                if ra is not None:
                    ref_tail[a].add(ra, t_sec)
                clamp_tr[a].step(verdict in CLAMP_VERDICTS, sa, t_sec)
                ik_tr[a].step(verdict == "IkFailed", sa, t_sec)

                # re-anchor counter steps -> post-event accel window
                if reanchor_cols[a]:
                    total = sum(_f(row, i) for i in reanchor_cols[a])
                    if reanchor_prev[a] is not None and total > reanchor_prev[a]:
                        reanchor_pending[a].append([REANCHOR_WINDOW, 0.0, t_sec])
                    reanchor_prev[a] = total
                aa = sa if sa is not None else 0.0
                for p in reanchor_pending[a]:
                    p[0] -= 1
                    if aa > p[1]:
                        p[1] = aa
                done = [p for p in reanchor_pending[a] if p[0] <= 0]
                reanchor_pending[a] = [p for p in reanchor_pending[a] if p[0] > 0]
                for p in done:
                    reanchor_events[a].append((p[2], p[1]))

                # readback sampling repeats (new column)
                st = state_stamp[a]
                if st is not None:
                    stamp = _f(row, st)
                    if stamp > 0:
                        if state_prev[a] is not None and stamp == state_prev[a]:
                            state_dup[a] += 1
                        state_prev[a] = stamp

                # true wire cadence (new columns)
                w = wire_start[a]
                if w is not None:
                    ws = _f(row, w)
                    if ws > 0:
                        if wire_first[a] is None:
                            wire_first[a] = ws
                        wire_last[a] = ws
                        if wire_prev[a] is not None and ws > wire_prev[a]:
                            wire_periods[a].add((ws - wire_prev[a]) / 1000.0)
                        if ws != wire_prev[a]:
                            wire_prev[a] = ws

            # pre-send latency (loop-side enqueue stamp)
            if i_loop_ns is not None:
                loop_ns = _f(row, i_loop_ns)
                for a in ARMS:
                    if send_start[a] is None:
                        continue
                    ss = _f(row, send_start[a])
                    if ss > 0 and loop_ns > 0:
                        lat_us = (ss - loop_ns) / 1000.0
                        if 0.0 <= lat_us <= 2100.0:
                            key = "ok" if verdict == "Ok" else "stressed"
                            presend[key][0] += lat_us
                            presend[key][1] += 1
                            if lat_us > presend["max"]:
                                presend["max"] = lat_us

            # fault rising edge + deadline-race signature
            if fault == 1 and fault_prev == 0:
                age = _f(row, i_age, default=-1.0)
                timeout = _f(row, i_timeout, default=-1.0)
                race = (
                    age >= 0.0
                    and timeout > 0.0
                    and (timeout - age) <= DEADLINE_RACE_MARGIN_MS
                )
                fault_onsets.append(
                    {
                        "t_sec": t_sec,
                        "reason": (row[i_reason][:80] if i_reason is not None else ""),
                        "cmd_age_ms": age,
                        "cmd_timeout_ms": timeout,
                        "deadline_race": race,
                    }
                )
            fault_prev = fault

            # projection block (new columns)
            if i_proj_active is not None and _f(row, i_proj_active) > 0:
                proj["ticks_active"] += 1
                proj["max_constraints"] = max(
                    proj["max_constraints"], int(_f(row, i_proj_count))
                )
                if _f(row, i_proj_ceiling) > 0:
                    proj["ceiling_clamped_ticks"] += 1
                margin = _f(row, i_proj_margin, default=-1.0)
                if margin >= 0.0 and (
                    proj["min_margin_m"] is None or margin < proj["min_margin_m"]
                ):
                    proj["min_margin_m"] = margin
                proj["max_verdict_age_ms"] = max(
                    proj["max_verdict_age_ms"], _f(row, i_proj_age, default=-1.0)
                )
                for a in ARMS:
                    if proj_corr[a] is not None:
                        proj["max_correction_deg_s"][a] = max(
                            proj["max_correction_deg_s"][a], _f(row, proj_corr[a])
                        )

            # CSV/wire quantization indicator on left j0
            if sent_acc["left"].idx[0] is not None:
                q0 = _f(row, sent_acc["left"].idx[0])
                if prev_q0 is not None:
                    d = abs(q0 - prev_q0)
                    if d > 1e-12 and (min_abs_dq0 is None or d < min_abs_dq0):
                        min_abs_dq0 = d
                prev_q0 = q0

        duration = rows * TICK_SEC
        out["rows"] = rows
        out["duration_sec"] = duration
        out["accel_tail"] = {
            a: {
                "q_sent": sent_tail[a].report() if sent_acc[a].available else "n/a",
                "q_ref": ref_tail[a].report() if ref_acc[a].available else "n/a",
            }
            for a in ARMS
        }
        out["boundary_transitions"] = {
            a: clamp_tr[a].report(duration, top) for a in ARMS
        }
        out["ik_failed_transitions"] = {a: ik_tr[a].report(duration, top) for a in ARMS}
        out["reanchor"] = {
            a: {
                "events": len(reanchor_events[a]),
                "post_peaks_over_2000": sum(
                    1 for _, p in reanchor_events[a] if p > 2000.0
                ),
                "worst": [
                    {"t_sec": t, "peak_accel_deg_s2": p}
                    for t, p in sorted(
                        reanchor_events[a], key=lambda e: e[1], reverse=True
                    )[:top]
                ],
            }
            for a in ARMS
        }
        out["fault_onsets"] = fault_onsets
        out["presend_latency_us"] = {
            "ok_mean": (presend["ok"][0] / presend["ok"][1]) if presend["ok"][1] else 0.0,
            "stressed_mean": (
                presend["stressed"][0] / presend["stressed"][1]
            )
            if presend["stressed"][1]
            else 0.0,
            "stressed_ticks": presend["stressed"][1],
            "max": presend["max"],
        }
        wire = {}
        for a in ARMS:
            _ = wire_counters[a]  # cumulative totals come from wire_counter_totals()
            if wire_start[a] is None:
                wire[a] = "n/a (pre-wire-telemetry log)"
                continue
            stats = wire_periods[a]
            span_sec = (
                (wire_last[a] - wire_first[a]) / 1e9
                if wire_first[a] is not None and wire_last[a] is not None
                else 0.0
            )
            wire[a] = {
                "wire_period_us": stats.report(),
                "effective_hz": (stats.n / span_sec) if span_sec > 0 else 0.0,
            }
        out["wire"] = wire
        out["state_readback_dups_per_sec"] = {
            a: (state_dup[a] / duration if duration > 0 else 0.0)
            if state_stamp[a] is not None
            else "n/a"
            for a in ARMS
        }
        out["projection"] = proj if i_proj_active is not None else "n/a"
        out["csv_min_abs_delta_left_q0_deg"] = min_abs_dq0
        return out


class TailStatsLike:
    """Mean/min/max + out-of-band counts for wire send periods (us)."""

    def __init__(self):
        self.n = 0
        self.sum = 0.0
        self.min = None
        self.max = None
        self.over_2500 = 0
        self.under_1500 = 0

    def add(self, v):
        self.n += 1
        self.sum += v
        self.min = v if self.min is None else min(self.min, v)
        self.max = v if self.max is None else max(self.max, v)
        if v > 2500.0:
            self.over_2500 += 1
        if v < 1500.0:
            self.under_1500 += 1

    def report(self):
        return {
            "n": self.n,
            "mean": (self.sum / self.n) if self.n else 0.0,
            "min": self.min,
            "max": self.max,
            "over_2500us": self.over_2500,
            "under_1500us": self.under_1500,
        }


def wire_counter_totals(path):
    """Read the LAST row's cumulative wire counters cheaply (tail scan)."""
    totals = {}
    try:
        with open(path, "rb") as handle:
            handle.seek(0, 2)
            size = handle.tell()
            handle.seek(max(0, size - 65536))
            tail = handle.read().decode(errors="replace").strip().splitlines()
        if len(tail) < 1:
            return totals
        last = next(csv.reader([tail[-1]]))
        with open(path, newline="") as handle:
            header = next(csv.reader(handle))
        col = {n: i for i, n in enumerate(header)}
        for a in ARMS:
            for key in ("worker_pending_overwrites_total",
                        "worker_repeated_sends_total",
                        "worker_wire_dispatches_total",
                        "worker_interp_rebase_total",
                        "worker_interp_hold_total"):
                name = f"{a}_{key}"
                if name in col and col[name] < len(last):
                    try:
                        totals[name] = int(float(last[col[name]]))
                    except ValueError:
                        pass
    except OSError:
        pass
    return totals


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("log", nargs="?", default="logs/servo_log.csv")
    parser.add_argument("--start-sec", type=float, default=0.0)
    parser.add_argument("--duration-sec", type=float, default=None)
    parser.add_argument("--top", type=int, default=5)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)

    result = analyze(args.log, args.start_sec, args.duration_sec, args.top)
    result["wire_counter_totals"] = wire_counter_totals(args.log)

    if args.json:
        json.dump(result, sys.stdout, indent=2)
        print()
        return 0

    print(f"== smoothness report: {result['file']}")
    print(f"rows={result['rows']}  duration={result['duration_sec']:.1f}s")
    for a in ARMS:
        tail = result["accel_tail"][a]
        for kind in ("q_sent", "q_ref"):
            t = tail[kind]
            if t == "n/a":
                print(f"[{a}] {kind}: n/a")
                continue
            rates = "  ".join(
                f">{k}:{v:.2f}/s" for k, v in t["exceed_per_sec"].items()
            )
            print(
                f"[{a}] {kind} accel tail: {rates}  max={t['max_deg_s2']:.0f} deg/s2 @ {t['max_t_sec']:.1f}s"
            )
        bt = result["boundary_transitions"][a]
        print(
            f"[{a}] boundary transitions: entries={bt['entries']} toggles/min={bt['toggle_per_min']:.1f} "
            f"windows>2000: {bt['windows_over_2000']}/{bt['windows_total']}"
        )
        for w in bt["worst"][:3]:
            print(f"      {w['kind']:5s} @ {w['t_sec']:.1f}s  {w['accel_deg_s2']:.0f} deg/s2")
        ik = result["ik_failed_transitions"][a]
        if ik["entries"]:
            print(
                f"[{a}] IkFailed transitions: entries={ik['entries']} windows>2000: "
                f"{ik['windows_over_2000']}/{ik['windows_total']}"
            )
        ra = result["reanchor"][a]
        if ra["events"]:
            print(
                f"[{a}] re-anchors: {ra['events']} events, post-window >2000: {ra['post_peaks_over_2000']}"
            )
            for w in ra["worst"][:3]:
                print(f"      @ {w['t_sec']:.1f}s  peak {w['peak_accel_deg_s2']:.0f} deg/s2")
    ps = result["presend_latency_us"]
    print(
        f"pre-send latency: ok_mean={ps['ok_mean']:.0f}us stressed_mean={ps['stressed_mean']:.0f}us "
        f"(n={ps['stressed_ticks']}) max={ps['max']:.0f}us"
    )
    for f in result["fault_onsets"]:
        race = "  << DEADLINE-RACE SIGNATURE" if f["deadline_race"] else ""
        print(
            f"fault @ {f['t_sec']:.1f}s age={f['cmd_age_ms']:.1f}/{f['cmd_timeout_ms']:.0f}ms "
            f"{f['reason']}{race}"
        )
    if result["projection"] != "n/a":
        pj = result["projection"]
        ceiling = pj["ceiling_clamped_ticks"]
        flag = "  << 1-TICK VELOCITY STEP, INVESTIGATE" if ceiling else ""
        print(
            f"projection: active={pj['ticks_active']} ticks, ceiling_clamped={ceiling}{flag}, "
            f"min_margin={pj['min_margin_m']}, max_verdict_age={pj['max_verdict_age_ms']:.1f}ms"
        )
    for a in ARMS:
        w = result["wire"][a]
        if isinstance(w, str):
            print(f"[{a}] wire: {w}")
        else:
            p = w["wire_period_us"]
            print(
                f"[{a}] wire period: mean={p['mean']:.1f}us [{p['min']:.0f}..{p['max']:.0f}] "
                f"n={p['n']} >2.5ms:{p['over_2500us']} <1.5ms:{p['under_1500us']}"
            )
    totals = result.get("wire_counter_totals") or {}
    if totals:
        print("wire counters (cumulative): " + "  ".join(f"{k}={v}" for k, v in sorted(totals.items())))
    dups = result.get("state_readback_dups_per_sec")
    if dups and not all(v == "n/a" for v in dups.values()):
        print(
            "state readback dups: "
            + "  ".join(
                f"{a}={v:.2f}/s" if v != "n/a" else f"{a}=n/a" for a, v in dups.items()
            )
            + "  (filter q_ref analyses on these ticks)"
        )
    if result["csv_min_abs_delta_left_q0_deg"] is not None:
        print(
            f"csv min |dq| left j0: {result['csv_min_abs_delta_left_q0_deg']:.7f} deg "
            "(0.001 = 6-sig-digit quantization floor)"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
