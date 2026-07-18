#!/usr/bin/env python3
"""Estimate the effective F/T application point (tau = r x F) from servo log CSVs.

Verifies the wrench-transform moment reference of the F/T pipeline
(force_torque.<arm>.T_tcp_sensor) against physical push evidence, per the gate
recorded in rb_servo_server/config/stack_real.yaml: force motion stays disabled
until fingertip-centre and known-offset pushes show that the TCP torque follows
tau = r x F with the configured geometry.

Method
  - Reads {arm}_ft_fast_external_{fx,fy,fz,tx,ty,tz} (TCP-frame fast external
    wrench: post T_tcp_sensor transform + gravity model + residual tare) from an
    rb_servo_server servo log CSV.
  - Reports the quiet baseline |F| (median of the lowest-|F| quartile). A large
    baseline means the residual tare was never applied (auto tare fires only
    after Init Motion), so absolute wrench values still contain sensor bias +
    tool gravity. Push-event analysis below stays valid regardless, because it
    is baseline-subtracted.
  - Detects push events on the quiet-baseline-subtracted force magnitude
    |F - F_quiet| (enter threshold, hysteresis exit, minimum length), so an
    untared bias cannot mask a push orthogonal to it. Each event then subtracts
    its own pre-event local baseline, and the script estimates the
    effective application point r (TCP frame) of the contact delta (dF, dtau):
      per-sample   r_perp = (dF x dtau) / |dF|^2   (component perpendicular to dF)
      per-event    least-squares fit of dtau = r x dF (primary: a baseline-
                   subtracted point contact carries no couple), plus a
                   dtau = tau0 + r x dF fit as a drift diagnostic (a steady
                   push cannot separate tau0 from r, so tau0 is never part of
                   the verdict)
  - If the force direction barely varies within an event, the r component along
    the push direction is unobservable; the report says so and any --expect-r
    verdict compares only the observable perpendicular components.

Reference points on the tool axis (TCP-frame z, from
rb_servo_server/descriptions/urdf/rb3_730e.urdf and stack_real.yaml):
      0.000       fingertip centre (= TCP): expected r for a fingertip push
                  when the configured transform is correct
     -0.202642    configured sensor measurement origin (tool-side face)
     -0.217642    sensor body centre (attachment +15 mm adapter +30 mm body)

Pure stdlib (csv/math/statistics); no numpy, so it runs on the operator PC.

Usage
  python3 scripts/analyze_ft_application_point.py logs/servo_log_*.csv --arm right
  # fingertip-centre push gate (tau = r x F with r = 0):
  python3 scripts/analyze_ft_application_point.py run.csv --arm right \
      --expect-r 0,0,0 --tol-mm 15 --json out.json
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys

WRENCH_FIELDS = ("fx_n", "fy_n", "fz_n", "tx_nm", "ty_nm", "tz_nm")

# Tool-axis reference points, TCP frame z (see module docstring for sources).
REFERENCE_POINTS = {
    "tcp_fingertip": (0.0, 0.0, 0.0),
    "sensor_measurement_face": (0.0, 0.0, -0.202642),
    "sensor_body_centre": (0.0, 0.0, -0.217642),
}


def load_rows(path, arm):
    """Load per-tick wrench rows for one arm. Fails loudly on missing columns."""
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        idx = {name: i for i, name in enumerate(header)}
        wanted = [f"{arm}_ft_fast_external_{c}" for c in WRENCH_FIELDS]
        missing = [c for c in wanted if c not in idx]
        if missing:
            raise KeyError(
                f"{path}: missing columns {missing} (arm={arm}); "
                "is this an rb_servo_server servo log with F/T telemetry?"
            )
        wcols = [idx[c] for c in wanted]
        tcol = idx.get("loop_start_time_ns")
        hcol = idx.get(f"{arm}_ft_healthy")
        mcol = idx.get(f"{arm}_force_control_operating_mode")
        wrench, time_s, healthy, modes = [], [], 0, set()
        for row in reader:
            try:
                w = tuple(float(row[c]) for c in wcols)
            except (ValueError, IndexError):
                continue  # partial/corrupt line (e.g. truncated tail)
            wrench.append(w)
            time_s.append(float(row[tcol]) * 1e-9 if tcol is not None else None)
            if hcol is not None and row[hcol] in ("1", "true", "True"):
                healthy += 1
            if mcol is not None and row[mcol]:
                modes.add(row[mcol])
    return {"wrench": wrench, "time_s": time_s, "healthy": healthy, "modes": modes}


def force_mag(w):
    return math.sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2])


def quiet_baseline(wrench):
    """Median |F| and per-component median wrench over the lowest-|F| quartile."""
    order = sorted(range(len(wrench)), key=lambda i: force_mag(wrench[i]))
    quiet = [wrench[i] for i in order[: max(1, len(wrench) // 4)]]
    base_mag = statistics.median(force_mag(w) for w in quiet)
    base_w = tuple(statistics.median(w[k] for w in quiet) for k in range(6))
    return base_mag, base_w


def detect_events(dmags, enter_n, exit_n, min_len):
    """Contiguous [start, end) ranges where the force deviation exceeds enter_n.

    dmags must already be baseline-relative (|F - F_quiet|), so a constant
    untared bias cannot mask a push orthogonal to it.
    """
    events, in_ev, start = [], False, 0
    for i, d in enumerate(dmags):
        if not in_ev and d > enter_n:
            in_ev, start = True, i
        elif in_ev and d < exit_n:
            in_ev = False
            events.append((start, i))
    if in_ev:
        events.append((start, len(dmags)))
    return [(s, e) for s, e in events if e - s >= min_len]


def _solve(ata, atb):
    """Solve n x n normal equations by Gaussian elimination; None if singular."""
    n = len(atb)
    m = [row[:] + [atb[i]] for i, row in enumerate(ata)]
    for c in range(n):
        piv = max(range(c, n), key=lambda r: abs(m[r][c]))
        if abs(m[piv][c]) < 1e-12:
            return None
        m[c], m[piv] = m[piv], m[c]
        d = m[c][c]
        m[c] = [v / d for v in m[c]]
        for r in range(n):
            if r != c and m[r][c] != 0.0:
                f = m[r][c]
                m[r] = [vr - f * vc for vr, vc in zip(m[r], m[c])]
    return [m[i][n] for i in range(n)]


def fit_lever(deltas, fit_couple=True):
    """Least-squares fit of dtau = [tau0 +] r x dF over (dF, dtau) samples.

    With fit_couple=False, tau0 is pinned to zero (3 unknowns): the right model
    for a baseline-subtracted point contact, and it stays well-conditioned for
    a steady push, where tau0 and r x (constant dF) are indistinguishable.
    Returns {r, tau0, rms_nm, n} or None if the system is singular.
    """
    dim = 6 if fit_couple else 3
    ata = [[0.0] * dim for _ in range(dim)]
    atb = [0.0] * dim
    n = 0
    for fx, fy, fz, tx, ty, tz in deltas:
        # rows of [-dF]_x (| I3 when fitting the couple), stacked per sample
        rows3 = (
            (0.0, fz, -fy, 1.0, 0.0, 0.0, tx),
            (-fz, 0.0, fx, 0.0, 1.0, 0.0, ty),
            (fy, -fx, 0.0, 0.0, 0.0, 1.0, tz),
        )
        for row in rows3:
            for a in range(dim):
                if row[a] == 0.0:
                    continue
                atb[a] += row[a] * row[6]
                for b in range(dim):
                    if row[b] != 0.0:
                        ata[a][b] += row[a] * row[b]
        n += 1
    if n < 4:
        return None
    # Tiny relative Tikhonov term: leaves well-posed fits unchanged (1e-9
    # relative) and pins unobservable directions (steady single-direction push)
    # to ~0 instead of noise-driven garbage, matching the r_perp semantics.
    max_diag = max(ata[i][i] for i in range(dim))
    if max_diag <= 0.0:
        return None
    for i in range(dim):
        ata[i][i] += 1e-9 * max_diag
    x = _solve(ata, atb)
    if x is None:
        return None
    r = x[:3]
    tau0 = x[3:] if fit_couple else [0.0, 0.0, 0.0]
    ss = 0.0
    for fx, fy, fz, tx, ty, tz in deltas:
        px = tau0[0] + (fz * r[1] - fy * r[2])
        py = tau0[1] + (-fz * r[0] + fx * r[2])
        pz = tau0[2] + (fy * r[0] - fx * r[1])
        ss += (px - tx) ** 2 + (py - ty) ** 2 + (pz - tz) ** 2
    return {"r": r, "tau0": tau0, "rms_nm": math.sqrt(ss / (3 * n)), "n": n}


def per_sample_points(deltas):
    """Per-sample r_perp = (dF x dtau)/|dF|^2 (component of r perpendicular to dF)."""
    points = []
    for fx, fy, fz, tx, ty, tz in deltas:
        f2 = fx * fx + fy * fy + fz * fz
        if f2 <= 0.0:
            continue
        points.append((
            (fy * tz - fz * ty) / f2,
            (fz * tx - fx * tz) / f2,
            (fx * ty - fy * tx) / f2,
        ))
    return points


def direction_spread_deg(deltas):
    """RMS angle (deg) of dF directions about their mean direction."""
    sx = sy = sz = 0.0
    for fx, fy, fz, *_ in deltas:
        m = math.sqrt(fx * fx + fy * fy + fz * fz)
        if m <= 0.0:
            continue
        sx += fx / m
        sy += fy / m
        sz += fz / m
    mm = math.sqrt(sx * sx + sy * sy + sz * sz)
    if mm <= 0.0:
        return None, 90.0
    mean = (sx / mm, sy / mm, sz / mm)
    acc, n = 0.0, 0
    for fx, fy, fz, *_ in deltas:
        m = math.sqrt(fx * fx + fy * fy + fz * fz)
        if m <= 0.0:
            continue
        c = max(-1.0, min(1.0, (fx * mean[0] + fy * mean[1] + fz * mean[2]) / m))
        acc += math.acos(c) ** 2
        n += 1
    return mean, math.degrees(math.sqrt(acc / n)) if n else 90.0


def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _perp(v, unit):
    d = sum(x * u for x, u in zip(v, unit))
    return tuple(x - d * u for x, u in zip(v, unit))


def impact_profile(all_deltas, tick_s):
    """Peak |dF|, time-to-peak, and 10-90% rise time of one contact event.

    Freedrive floor-impact reproductions use this to calibrate contact and
    hard-limit thresholds from measured evidence (peak is an upper bound vs a
    compliant rollout, where the admittance absorbs part of the impact).
    """
    mags = [force_mag(d) for d in all_deltas]
    peak = max(mags)
    i_peak = mags.index(peak)
    i10 = next((i for i, m in enumerate(mags) if m >= 0.1 * peak), 0)
    i90 = next((i for i, m in enumerate(mags) if m >= 0.9 * peak), i_peak)
    out = {"peak_df_n": peak, "peak_at_tick": i_peak, "rise_10_90_ticks": i90 - i10}
    if tick_s:
        out["peak_at_s"] = i_peak * tick_s
        out["rise_10_90_s"] = (i90 - i10) * tick_s
    return out


def analyze_event(wrench, start, end, *, pre_baseline, baseline_gap, min_delta_n,
                  tick_s=None):
    """Baseline-subtracted application-point analysis of one push event."""
    b0 = start - baseline_gap - pre_baseline
    b1 = start - baseline_gap
    if b0 < 0 or b1 <= b0:
        return {"start": start, "end": end, "skipped": "no pre-event baseline window"}
    base = tuple(
        statistics.median(w[k] for w in wrench[b0:b1]) for k in range(6)
    )
    all_deltas = [_sub(w, base) for w in wrench[start:end]]
    deltas = [d for d in all_deltas if force_mag(d) >= min_delta_n]
    if len(deltas) < 4:
        return {
            "start": start,
            "end": end,
            "skipped": f"only {len(deltas)} samples with |dF| >= {min_delta_n} N",
            **impact_profile(all_deltas, tick_s),
        }
    fit = fit_lever(deltas, fit_couple=False)
    if fit is None:
        return {"start": start, "end": end, "skipped": "singular fit (no excitation)"}
    fit_couple = fit_lever(deltas, fit_couple=True)
    points = per_sample_points(deltas)
    r_med = tuple(statistics.median(p[k] for p in points) for k in range(3))
    mean_dir, spread = direction_spread_deg(deltas)
    df_med = statistics.median(force_mag(d) for d in deltas)
    dtau_med = statistics.median(
        math.sqrt(d[3] ** 2 + d[4] ** 2 + d[5] ** 2) for d in deltas
    )
    result = {
        "start": start,
        "end": end,
        "n_used": len(deltas),
        **impact_profile(all_deltas, tick_s),
        "df_median_n": df_med,
        "dtau_median_nm": dtau_med,
        "tau_over_f_m": dtau_med / df_med if df_med > 0 else None,
        "r_fit_m": fit["r"],
        "fit_rms_nm": fit["rms_nm"],
        "r_perp_median_m": r_med,
        "mean_force_dir": mean_dir,
        "dir_spread_deg": spread,
    }
    if fit_couple is not None:
        result["couple_diag"] = {
            "r_m": fit_couple["r"],
            "tau0_nm": fit_couple["tau0"],
            "fit_rms_nm": fit_couple["rms_nm"],
        }
    return result


def analyze_arm(path, arm, args):
    data = load_rows(path, arm)
    wrench = data["wrench"]
    result = {
        "csv": path,
        "arm": arm,
        "rows": len(wrench),
        "healthy_rows": data["healthy"],
        "operating_modes": sorted(data["modes"]),
    }
    if not wrench:
        result["error"] = "no parsable wrench rows"
        return result
    times = [t for t in data["time_s"] if t is not None]
    tick_s = None
    if len(times) >= 2 and times[-1] > times[0]:
        result["duration_s"] = times[-1] - times[0]
        tick_s = (times[-1] - times[0]) / (len(times) - 1)
    base_mag, base_wrench = quiet_baseline(wrench)
    result["quiet_baseline_f_n"] = base_mag
    result["tare_suspect"] = base_mag > args.warn_baseline_n
    dmags = [force_mag(_sub(w, base_wrench)) for w in wrench]
    events = detect_events(dmags, args.enter_n, args.exit_n, args.min_len)
    result["events"] = [
        analyze_event(
            wrench, s, e,
            pre_baseline=args.pre_baseline,
            baseline_gap=args.baseline_gap,
            min_delta_n=args.min_delta_n,
            tick_s=tick_s,
        )
        for s, e in events
    ]
    # Whole-log correlation fit: with no pushes this exposes where the *varying*
    # wrench component acts (sensor-internal noise / cable tug / origin offset).
    whole = fit_lever(wrench)
    if whole is not None:
        mean_f = tuple(sum(w[k] for w in wrench) / len(wrench) for k in range(3))
        std_f = tuple(
            math.sqrt(sum((w[k] - mean_f[k]) ** 2 for w in wrench) / len(wrench))
            for k in range(3)
        )
        result["whole_log_fit"] = {
            "r_m": whole["r"],
            "fit_rms_nm": whole["rms_nm"],
            "force_std_n": std_f,
        }
    if args.expect_r is not None:
        result["verdicts"] = [
            verdict_for_event(ev, args.expect_r, args.tol_mm)
            for ev in result["events"]
        ]
    return result


def verdict_for_event(ev, expect_r, tol_mm, unobservable_spread_deg=10.0):
    """PASS/FAIL of one event's fitted r against an expected application point.

    When the push direction barely varies, only the components of r
    perpendicular to the mean push direction are observable; the comparison is
    then restricted to that plane (and says so).
    """
    if "skipped" in ev:
        return {"start": ev["start"], "end": ev["end"], "verdict": "SKIPPED",
                "reason": ev["skipped"]}
    r = ev["r_fit_m"]
    restricted = (
        ev["mean_force_dir"] is not None
        and ev["dir_spread_deg"] < unobservable_spread_deg
    )
    if restricted:
        u = ev["mean_force_dir"]
        err = _sub(_perp(r, u), _perp(expect_r, u))
        basis = "perpendicular-to-push components only (push direction ~constant)"
    else:
        err = _sub(r, expect_r)
        basis = "all components"
    err_mm = math.sqrt(sum(v * v for v in err)) * 1000.0
    return {
        "start": ev["start"],
        "end": ev["end"],
        "error_mm": err_mm,
        "basis": basis,
        "verdict": "PASS" if err_mm <= tol_mm else "FAIL",
    }


def _fmt_vec(v, scale=1.0, unit="", digits=4):
    return "(" + ", ".join(f"{x * scale:+.{digits}f}" for x in v) + ")" + unit


def print_report(result):
    print(f"\n=== {result['csv']} [{result['arm']}] ===")
    if "error" in result:
        print(f"  ERROR: {result['error']}")
        return
    dur = f", {result['duration_s']:.1f} s" if "duration_s" in result else ""
    print(
        f"  rows={result['rows']}{dur}  healthy={result['healthy_rows']}"
        f"  mode={','.join(result['operating_modes']) or '?'}"
    )
    print(f"  quiet baseline |F| = {result['quiet_baseline_f_n']:.2f} N")
    if result["tare_suspect"]:
        print(
            "  WARNING: baseline |F| is large -> residual tare likely NOT applied\n"
            "           (auto tare fires only after Init Motion). Absolute wrench\n"
            "           includes sensor bias + tool gravity; the event analysis\n"
            "           below is baseline-subtracted and remains valid."
        )
    if not result["events"]:
        print("  no push events detected")
    for i, ev in enumerate(result["events"]):
        impact = ""
        if "peak_df_n" in ev:
            impact = f"  peak|dF|={ev['peak_df_n']:.1f} N"
            if "rise_10_90_s" in ev:
                impact += (
                    f" @+{ev['peak_at_s'] * 1000:.0f} ms"
                    f" (10-90% rise {ev['rise_10_90_s'] * 1000:.0f} ms)"
                )
        if "skipped" in ev:
            print(
                f"  event {i} [{ev['start']}:{ev['end']}]: skipped"
                f" ({ev['skipped']}){impact}"
            )
            continue
        print(
            f"  event {i} [{ev['start']}:{ev['end']}]  n_used={ev['n_used']}"
            f"  |dF|med={ev['df_median_n']:.1f} N{impact}"
            f"  |dtau|med={ev['dtau_median_nm']:.2f} Nm"
            f"  |dtau|/|dF|={ev['tau_over_f_m']:.4f} m"
        )
        print(
            f"    r_fit  = {_fmt_vec(ev['r_fit_m'])} m"
            f"   fit RMS={ev['fit_rms_nm'] * 1000:.1f} mNm"
        )
        print(
            f"    r_perp = {_fmt_vec(ev['r_perp_median_m'])} m"
            f"   push-dir spread={ev['dir_spread_deg']:.1f} deg"
        )
        if ev["dir_spread_deg"] < 10.0:
            print(
                "    note: push direction ~constant; r along "
                f"{_fmt_vec(ev['mean_force_dir'], digits=2)} is unobservable"
            )
        if "couple_diag" in ev:
            cd = ev["couple_diag"]
            tau0_norm = math.sqrt(sum(v * v for v in cd["tau0_nm"]))
            if tau0_norm > 0.05 or cd["fit_rms_nm"] < 0.5 * ev["fit_rms_nm"]:
                print(
                    f"    drift diag (tau0 fit): r = {_fmt_vec(cd['r_m'])} m"
                    f"  |tau0|={tau0_norm:.3f} Nm  fit RMS={cd['fit_rms_nm'] * 1000:.1f} mNm"
                    "\n    note: a large tau0 or much lower RMS means baseline drift or"
                    " an applied moment during the event"
                )
        for name, ref in REFERENCE_POINTS.items():
            d = math.sqrt(sum((a - b) ** 2 for a, b in zip(ev["r_fit_m"], ref)))
            print(f"    vs {name:26s} ({ref[2]:+.6f} z): {d * 1000:7.1f} mm")
    if "whole_log_fit" in result:
        wl = result["whole_log_fit"]
        print(
            f"  whole-log correlation fit: r = {_fmt_vec(wl['r_m'])} m"
            f"  fit RMS={wl['fit_rms_nm'] * 1000:.1f} mNm"
            f"  F std={_fmt_vec(wl['force_std_n'], digits=2)} N"
        )
        print(
            "    (diagnostic only: with no pushes this is where the VARYING wrench\n"
            "     component acts - sensor noise correlation, cable tug, or a\n"
            "     measurement-origin offset if it sits behind the sensor face)"
        )
    for v in result.get("verdicts", []):
        if v["verdict"] == "SKIPPED":
            print(f"  verdict [{v['start']}:{v['end']}]: SKIPPED ({v['reason']})")
        else:
            print(
                f"  verdict [{v['start']}:{v['end']}]: {v['verdict']}"
                f"  error={v['error_mm']:.1f} mm  ({v['basis']})"
            )


def parse_expect_r(text):
    parts = [p for p in text.replace(",", " ").split() if p]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("--expect-r needs 3 comma-separated values (m)")
    return tuple(float(p) for p in parts)


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0], allow_abbrev=False
    )
    ap.add_argument("csv_paths", nargs="+", help="rb_servo_server servo log CSV(s)")
    ap.add_argument("--arm", choices=("left", "right", "both"), default="both")
    ap.add_argument("--enter-n", type=float, default=8.0,
                    help="event enter threshold above quiet baseline |F| (N)")
    ap.add_argument("--exit-n", type=float, default=4.0,
                    help="event exit threshold above quiet baseline |F| (N)")
    ap.add_argument("--min-len", type=int, default=20,
                    help="minimum event length (ticks)")
    ap.add_argument("--min-delta-n", type=float, default=5.0,
                    help="minimum baseline-subtracted |dF| for a usable sample (N)")
    ap.add_argument("--pre-baseline", type=int, default=200,
                    help="pre-event local-baseline window (ticks)")
    ap.add_argument("--baseline-gap", type=int, default=20,
                    help="gap between baseline window and event start (ticks)")
    ap.add_argument("--warn-baseline-n", type=float, default=2.0,
                    help="quiet baseline |F| above this warns that tare is missing (N)")
    ap.add_argument("--expect-r", type=parse_expect_r, default=None,
                    help="expected application point in TCP frame, m (e.g. 0,0,0 "
                         "for a fingertip-centre push)")
    ap.add_argument("--tol-mm", type=float, default=15.0,
                    help="PASS tolerance for --expect-r (mm)")
    ap.add_argument("--json", default=None, help="write machine-readable results here")
    args = ap.parse_args(argv)

    arms = ("left", "right") if args.arm == "both" else (args.arm,)
    results = []
    for path in args.csv_paths:
        for arm in arms:
            result = analyze_arm(path, arm, args)
            results.append(result)
            print_report(result)
    if args.json:
        with open(args.json, "w") as f:
            json.dump(results, f, indent=2)
        print(f"\nwrote {args.json}")
    verdicts = [v for r in results for v in r.get("verdicts", [])]
    if verdicts and any(v["verdict"] == "FAIL" for v in verdicts):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
