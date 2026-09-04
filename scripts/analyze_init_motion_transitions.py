#!/usr/bin/env python3
"""InitMotion transition acceptance report for a 500 Hz servo log.

Turns the 2026-09-04 InitMotion findings into a repeatable gate. For every
InitMotion episode in the log (per arm: the ticks where the server's init status
is planning/executing) it reports:

  onset        the largest one-tick command step and joint acceleration in the
               0.4 s around the brake/plan hand-off (the "hold entry step")
  resume       in the 0.5 s after the arm returns to TcpPoseTarget: the velocity
               and accel clamp deltas (a frozen IK low-pass or any other stale
               state shows up here first), peak joint accel, the TCP command
               excursion and peak speed, and how long until the chunk follower
               re-engaged
  peer         when only one arm was reset: whether the OTHER arm kept its chunk
               follower and its TcpPoseTarget profile for the whole episode, and
               the worst one-tick speed drop it took
  force        after the resume: whether force control re-covered the arm
               (the auto-tare after InitMotion needs the arm parked 0.5 s) and
               how long that took

Each block carries a PASS/FAIL against the thresholds below so a run can be
judged from the log alone. Columns are resolved by name with fallbacks and any
missing group degrades to "n/a".

Usage:
  scripts/analyze_init_motion_transitions.py logs/servo_log_XXXX.csv [--json]
      [--resume-window-sec S] [--force-cover-sec S]
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys

TICK_SEC = 0.002
ARMS = ("left", "right")
OTHER = {"left": "right", "right": "left"}
INIT_ACTIVE = {"planning", "executing"}

# Acceptance thresholds (measured defects were 10-1000x these).
RESUME_VELOCITY_CLAMP_MAX_DEG = 0.01   # frozen-state blend showed 58.95 deg
RESUME_ACCEL_MAX_DEG_S2 = 6000.0       # 2x the largest ddq_max; the kick was 12,021
ONSET_ACCEL_MAX_DEG_S2 = 3000.0        # the measured-reanchor snap was 4.4-6.9k; a brake
                                       # hand-off from streaming stays under ~1.3k
COLD_START_GAP_SEC = 5.0               # resume this long after done = a new episode, not a resume
PEER_FOLLOWER_MIN_FRACTION = 0.99
PEER_SPEED_DROP_MAX_MM_S = 60.0        # the profile drop stopped 145 -> 10 mm/s in one tick


def _f(row, key, default=float("nan")):
    if key is None:
        return default
    v = row.get(key, "")
    if v == "" or v is None:
        return default
    try:
        return float(v)
    except ValueError:
        return default


def _first_present(fields, *names):
    for n in names:
        if n in fields:
            return n
    return None


class Columns:
    def __init__(self, fields):
        self.fields = set(fields)
        self.time = _first_present(fields, "loop_start_time_ns")
        self.init_status = {}
        self.mode = {}
        self.follower_active = {}
        self.profile = {}
        self.vclamp = {}
        self.aclamp = {}
        self.q_sent = {}
        self.tcp_cmd = {}
        self.fc_covered = {}
        self.fc_enabled = {}
        self.fc_reason = {}
        for arm in ARMS:
            self.init_status[arm] = _first_present(
                fields, f"init_motion_{arm}_status", "init_motion_aggregate_status")
            self.mode[arm] = _first_present(fields, f"{arm}_mode")
            self.follower_active[arm] = _first_present(fields, f"{arm}_follower_active")
            self.profile[arm] = _first_present(fields, f"{arm}_tcp_target_profile")
            self.vclamp[arm] = _first_present(fields, f"{arm}_safety_velocity_clamp_max_delta_deg")
            self.aclamp[arm] = _first_present(fields, f"{arm}_safety_accel_clamp_max_delta_deg")
            self.q_sent[arm] = [f"{arm}_q_sent_{j}" for j in range(6)]
            if any(c not in self.fields for c in self.q_sent[arm]):
                self.q_sent[arm] = None
            tcp = [f"{arm}_tcp_command_stand_{c}_m" for c in "xyz"]
            self.tcp_cmd[arm] = tcp if all(c in self.fields for c in tcp) else None
            self.fc_covered[arm] = _first_present(fields, f"{arm}_fc_covered")
            self.fc_enabled[arm] = _first_present(fields, f"{arm}_fc_enabled")
            self.fc_reason[arm] = _first_present(fields, f"{arm}_fc_coverage_reason")


def load_rows(path):
    with open(path, newline="") as handle:
        reader = csv.DictReader(handle)
        cols = Columns(reader.fieldnames or [])
        rows = list(reader)
    return cols, rows


def _time_axis(cols, rows):
    if cols.time is None:
        return [i * TICK_SEC for i in range(len(rows))]
    t0 = _f(rows[0], cols.time, 0.0)
    return [(_f(r, cols.time, 0.0) - t0) * 1e-9 for r in rows]


def find_episodes(cols, rows):
    """Per arm, maximal tick runs with init status planning/executing."""
    episodes = []
    for arm in ARMS:
        key = cols.init_status[arm]
        if key is None:
            continue
        start = None
        for i, r in enumerate(rows):
            active = str(r.get(key, "")).strip().lower() in INIT_ACTIVE
            if active and start is None:
                start = i
            elif not active and start is not None:
                episodes.append((arm, start, i))
                start = None
        if start is not None:
            episodes.append((arm, start, len(rows)))
    # The sequencer reports a short planning/hold tail after "done" (the runner
    # re-emits its init latch until the policy resumes), which would split one
    # InitMotion into two. Merge runs of the same arm when the arm never returned
    # to TcpPoseTarget in between and the gap is short.
    merged = []
    for arm, s, e in sorted(episodes, key=lambda e: (e[0], e[1])):
        if merged and merged[-1][0] == arm:
            _, ps, pe = merged[-1]
            gap_ticks = s - pe
            mode_key = cols.mode[arm]
            streamed = any(rows[i].get(mode_key) == "TcpPoseTarget" for i in range(pe, s)) \
                if mode_key else False
            if gap_ticks < int(0.5 / TICK_SEC) and not streamed:
                merged[-1] = (arm, ps, e)
                continue
        merged.append((arm, s, e))
    merged.sort(key=lambda e: (e[1], e[0]))
    return merged


def _joint_series(cols, rows, arm, lo, hi):
    keys = cols.q_sent[arm]
    if keys is None:
        return None
    return [[_f(rows[i], k, 0.0) for k in keys] for i in range(max(0, lo), min(len(rows), hi))]


def _max_step_and_accel(q):
    """Largest one-tick |dq| (deg) and |d2q| (deg/s^2) over a joint series."""
    step = 0.0
    accel = 0.0
    for i in range(1, len(q)):
        for j in range(6):
            step = max(step, abs(q[i][j] - q[i - 1][j]))
            if i >= 2:
                a = abs(q[i][j] - 2.0 * q[i - 1][j] + q[i - 2][j]) / (TICK_SEC * TICK_SEC)
                accel = max(accel, a)
    return step, accel


def _tcp_series(cols, rows, arm, lo, hi):
    keys = cols.tcp_cmd[arm]
    if keys is None:
        return None
    return [[_f(rows[i], k, 0.0) * 1000.0 for k in keys] for i in range(max(0, lo), min(len(rows), hi))]


def _speed(tcp, i):
    d = math.sqrt(sum((tcp[i][k] - tcp[i - 1][k]) ** 2 for k in range(3)))
    return d / TICK_SEC


def analyze_episode(cols, rows, t, arm, start, end, resume_window_sec, force_cover_sec):
    n = len(rows)
    out = {"arm": arm, "t_start": t[start], "t_end": t[min(end, n - 1)],
           "duration_sec": t[min(end, n - 1)] - t[start]}
    mode_key = cols.mode[arm]

    # ---- onset: the hand-off into the sequencer (brake -> planning hold) ----
    q = _joint_series(cols, rows, arm, start - 50, start + 150)
    if q:
        step, accel = _max_step_and_accel(q)
        out["onset"] = {"max_step_deg": step, "max_accel_deg_s2": accel,
                        "pass": accel <= ONSET_ACCEL_MAX_DEG_S2}
    else:
        out["onset"] = {"pass": None}

    # ---- peer arm during a single-arm episode ----
    peer = OTHER[arm]
    peer_mode = cols.mode[peer]
    peer_streaming = 0
    for i in range(start, min(end, n)):
        if peer_mode is not None and rows[i].get(peer_mode) == "TcpPoseTarget":
            peer_streaming += 1
    span = max(1, min(end, n) - start)
    if peer_mode is not None and peer_streaming >= 0.5 * span:
        pf = cols.follower_active[peer]
        active = [_f(rows[i], pf, 0.0) for i in range(start, min(end, n))] if pf else []
        profiles = sorted({str(rows[i].get(cols.profile[peer], "")) for i in range(start, min(end, n))}
                          if cols.profile[peer] else set())
        tcp = _tcp_series(cols, rows, peer, start - 5, min(end, n))
        drop = 0.0
        if tcp and len(tcp) > 2:
            for i in range(2, len(tcp)):
                drop = max(drop, _speed(tcp, i - 1) - _speed(tcp, i))
        frac = (sum(1 for a in active if a > 0.5) / len(active)) if active else float("nan")
        ok = (frac >= PEER_FOLLOWER_MIN_FRACTION if active else None)
        if ok is not None and len(profiles) > 1:
            ok = False
        if ok is not None and drop > PEER_SPEED_DROP_MAX_MM_S:
            ok = False
        out["peer"] = {"arm": peer, "follower_active_fraction": frac, "profiles": profiles,
                       "max_one_tick_speed_drop_mm_s": drop, "pass": ok}
    else:
        out["peer"] = {"arm": peer, "streaming": False, "pass": None}

    # ---- resume: first TcpPoseTarget tick after the episode ----
    resume = None
    if mode_key is not None:
        for i in range(min(end, n), min(n, end + int(30.0 / TICK_SEC))):
            if rows[i].get(mode_key) == "TcpPoseTarget":
                resume = i
                break
    if resume is None:
        out["resume"] = {"pass": None, "note": "no TcpPoseTarget within 30 s"}
        out["force"] = {"pass": None}
        return out
    win = int(resume_window_sec / TICK_SEC)
    hi = min(n, resume + win)
    vmax = max((_f(rows[i], cols.vclamp[arm], 0.0) for i in range(resume, hi)), default=0.0) \
        if cols.vclamp[arm] else float("nan")
    amax = max((_f(rows[i], cols.aclamp[arm], 0.0) for i in range(resume, hi)), default=0.0) \
        if cols.aclamp[arm] else float("nan")
    q = _joint_series(cols, rows, arm, resume - 2, hi)
    step, accel = _max_step_and_accel(q) if q else (float("nan"), float("nan"))
    tcp = _tcp_series(cols, rows, arm, resume, hi)
    excursion = 0.0
    peak_speed = 0.0
    if tcp:
        p0 = tcp[0]
        for i in range(1, len(tcp)):
            excursion = max(excursion, math.sqrt(sum((tcp[i][k] - p0[k]) ** 2 for k in range(3))))
            peak_speed = max(peak_speed, _speed(tcp, i))
    engage_sec = None
    pf = cols.follower_active[arm]
    if pf:
        for i in range(resume, min(n, resume + int(5.0 / TICK_SEC))):
            if _f(rows[i], pf, 0.0) > 0.5:
                engage_sec = t[i] - t[resume]
                break
    ok = None
    if not math.isnan(vmax):
        ok = vmax <= RESUME_VELOCITY_CLAMP_MAX_DEG and (math.isnan(accel) or accel <= RESUME_ACCEL_MAX_DEG_S2)
    gap_sec = t[resume] - t[min(end, n) - 1]
    out["resume"] = {"t_resume": t[resume], "gap_after_done_sec": gap_sec,
                     "cold_start": gap_sec > COLD_START_GAP_SEC,
                     "max_velocity_clamp_deg": vmax,
                     "max_accel_clamp_deg": amax, "max_accel_deg_s2": accel,
                     "max_one_tick_step_deg": step, "tcp_excursion_mm": excursion,
                     "tcp_peak_speed_mm_s": peak_speed, "follower_engage_sec": engage_sec,
                     "pass": ok}

    # ---- force control re-coverage after the resume ----
    fe = cols.fc_enabled[arm]
    fcv = cols.fc_covered[arm]
    if fcv is None or (fe is not None and _f(rows[resume], fe, 0.0) < 0.5):
        out["force"] = {"pass": None, "note": "force control disabled or not logged"}
    else:
        cover_sec = None
        hi2 = min(n, resume + int(force_cover_sec / TICK_SEC))
        for i in range(resume, hi2):
            if _f(rows[i], fcv, 0.0) > 0.5:
                cover_sec = t[i] - t[resume]
                break
        reason = str(rows[hi2 - 1].get(cols.fc_reason[arm], "")) if cols.fc_reason[arm] else ""
        out["force"] = {"covered_after_sec": cover_sec, "reason_at_window_end": reason,
                        "pass": cover_sec is not None}
    return out


def analyze(path, resume_window_sec=0.5, force_cover_sec=2.0):
    cols, rows = load_rows(path)
    if not rows:
        return {"path": path, "episodes": []}
    t = _time_axis(cols, rows)
    episodes = [analyze_episode(cols, rows, t, arm, s, e, resume_window_sec, force_cover_sec)
                for arm, s, e in find_episodes(cols, rows)]
    return {"path": path, "duration_sec": t[-1], "episodes": episodes}


def _verdict(block):
    p = block.get("pass")
    return "n/a" if p is None else ("PASS" if p else "FAIL")


def render(report):
    lines = [f"== InitMotion transitions: {report['path']}  ({report.get('duration_sec', 0):.1f} s)"]
    if not report["episodes"]:
        lines.append("no InitMotion episode found")
        return "\n".join(lines)
    for ep in report["episodes"]:
        lines.append(f"[{ep['arm']}] init {ep['t_start']:.2f}-{ep['t_end']:.2f} s ({ep['duration_sec']:.2f} s)")
        o = ep["onset"]
        if o.get("pass") is not None:
            lines.append(f"    onset   {_verdict(o)}  step {o['max_step_deg']:.3f} deg  accel {o['max_accel_deg_s2']:.0f} deg/s^2")
        r = ep["resume"]
        if r.get("pass") is not None:
            eng = "-" if r["follower_engage_sec"] is None else f"{r['follower_engage_sec']*1000:.0f} ms"
            cold = "  (cold start, %.0f s after done)" % r["gap_after_done_sec"] if r.get("cold_start") else ""
            lines.append(
                f"    resume  {_verdict(r)}  @{r['t_resume']:.2f} s{cold}  vclamp {r['max_velocity_clamp_deg']:.3f} deg  "
                f"aclamp {r['max_accel_clamp_deg']:.3f} deg  accel {r['max_accel_deg_s2']:.0f} deg/s^2  "
                f"excursion {r['tcp_excursion_mm']:.1f} mm  peak {r['tcp_peak_speed_mm_s']:.0f} mm/s  engage {eng}")
        else:
            lines.append(f"    resume  n/a  {r.get('note', '')}")
        p = ep["peer"]
        if p.get("pass") is not None:
            lines.append(
                f"    peer    {_verdict(p)}  {p['arm']} follower {p['follower_active_fraction']*100:.1f} %  "
                f"profiles {p['profiles']}  speed drop {p['max_one_tick_speed_drop_mm_s']:.0f} mm/s")
        f = ep["force"]
        if f.get("pass") is not None:
            cov = "never" if f["covered_after_sec"] is None else f"{f['covered_after_sec']*1000:.0f} ms"
            lines.append(f"    force   {_verdict(f)}  re-covered after {cov}  ({f['reason_at_window_end']})")
    counts = {}
    for ep in report["episodes"]:
        for block in ("onset", "resume", "peer", "force"):
            v = _verdict(ep[block])
            if v != "n/a":
                counts.setdefault(block, {"PASS": 0, "FAIL": 0})[v] += 1
    lines.append("summary: " + "  ".join(f"{b} {c['PASS']}/{c['PASS'] + c['FAIL']} pass" for b, c in counts.items()))
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("path", nargs="?", default="logs/servo_log.csv")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--resume-window-sec", type=float, default=0.5)
    parser.add_argument("--force-cover-sec", type=float, default=2.0)
    args = parser.parse_args(argv)
    report = analyze(args.path, args.resume_window_sec, args.force_cover_sec)
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        print(render(report))
    return 0


if __name__ == "__main__":
    sys.exit(main())
