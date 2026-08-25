#!/usr/bin/env python3
"""Measure Rainbow control-box latency from a 500 Hz rb_servo_server servo log.

Decomposes the joint-space pipeline recorded per servo tick:

    q_sent    the servo_j target rb_servo_server handed to the control box
    q_ref     the box's own reference readback (rbpodo sdata.jnt_ref)
    q_actual  encoder position

into three delays -- sent->ref (box ingest + queue + interpolation),
ref->actual (servo/mechanical tracking), and sent->actual (end to end) --
plus the read-freshness accounting needed to trust them.

Latency is estimated as the integer tick shift minimising residual RMSE, with a
parabolic sub-tick refinement, fitted only on contiguous moving segments. A
least-squares gain is reported alongside so a delay is not confused with the
amplitude attenuation of a low-pass stage.

Requires the q_ref columns added to the servo logger; refuses to guess if the
log predates them.

Usage:
  scripts/analyze_box_latency.py [logs/servo_log.csv] [--arm left|right|both]
      [--start-sec S] [--duration-sec D] [--max-lag-ticks N] [--segments K] [--json]
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

ARMS = ("left", "right")
DOF = 6
# A joint is "moving" when the commanded signal actually excites the pipeline.
# Below this the delay is unobservable and the fit degenerates to noise.
MOVING_DEG_PER_TICK = 2e-4          # ~0.1 deg/s at 500 Hz
MIN_SEGMENT_TICKS = 250             # 0.5 s at 500 Hz
MIN_EXCITATION_DEG = 0.05           # peak-to-peak floor for a usable joint


class LogError(Exception):
    """The log cannot support a trustworthy latency estimate."""


def _read_columns(path: Path, wanted: dict[str, list[str]]) -> tuple[dict[str, np.ndarray], list[str]]:
    """Stream the CSV once, materialising only the requested columns."""
    with path.open(newline="") as handle:
        reader = csv.reader(handle)
        try:
            header = next(reader)
        except StopIteration as exc:
            raise LogError(f"{path} is empty") from exc
        index = {name: i for i, name in enumerate(header)}

        missing_required = [c for c in wanted["required"] if c not in index]
        if missing_required:
            raise LogError(
                "log is missing required columns: "
                + ", ".join(missing_required)
                + "\n\nIf q_ref columns are absent this log predates the servo-logger "
                  "q_ref change. Rebuild (tools/build_stack.sh) and re-record."
            )
        optional = [c for c in wanted["optional"] if c in index]
        absent_optional = [c for c in wanted["optional"] if c not in index]

        columns = wanted["required"] + optional
        picks = [index[c] for c in columns]
        rows: list[list[str]] = []
        width = len(header)
        for row in reader:
            if len(row) != width:
                continue  # torn final line from a killed process
            rows.append([row[p] for p in picks])

    if not rows:
        raise LogError(f"{path} has a header but no data rows")
    raw = np.array(rows, dtype=object)
    out: dict[str, np.ndarray] = {}
    for i, name in enumerate(columns):
        col = raw[:, i]
        empty = col == ""
        col = np.where(empty, "nan", col)
        try:
            out[name] = col.astype(np.float64)
        except ValueError:
            out[name] = col  # non-numeric (e.g. a source label); kept as strings
    return out, absent_optional


def _joint_matrix(cols: dict[str, np.ndarray], arm: str, kind: str) -> np.ndarray:
    return np.column_stack([cols[f"{arm}_q_{kind}_{j}"] for j in range(DOF)])


def _segments(mask: np.ndarray, min_len: int) -> list[tuple[int, int]]:
    """Contiguous [start, stop) runs of True at least min_len long."""
    if mask.size == 0:
        return []
    padded = np.r_[False, mask, False]
    edges = np.flatnonzero(padded[1:] != padded[:-1])
    out = []
    for start, stop in zip(edges[0::2], edges[1::2]):
        if stop - start >= min_len:
            out.append((int(start), int(stop)))
    return out


def _lag_rmse(src: np.ndarray, dst: np.ndarray, spans: list[tuple[int, int]], max_lag: int) -> dict:
    """Tick shift k minimising ||src[t-k] - dst[t]||, fitted over spans only.

    Returns the integer lag, a parabolic sub-tick refinement, the residual RMSE
    there, the residual at zero lag (so a flat objective is visible), and the
    least-squares gain dst ~ gain * src[t-k].
    """
    curve = np.full(max_lag + 1, np.nan)
    for k in range(max_lag + 1):
        num = 0.0
        count = 0
        for start, stop in spans:
            lo = start + k
            if stop - lo < 2:
                continue
            diff = src[lo - k: stop - k] - dst[lo:stop]
            num += float(np.dot(diff, diff))
            count += diff.size
        if count:
            curve[k] = math.sqrt(num / count)
    if np.all(np.isnan(curve)):
        return {"usable": False}

    best = int(np.nanargmin(curve))
    # Parabolic vertex through (best-1, best, best+1) for sub-tick resolution.
    sub = float(best)
    if 0 < best < max_lag and np.isfinite(curve[best - 1]) and np.isfinite(curve[best + 1]):
        y0, y1, y2 = curve[best - 1], curve[best], curve[best + 1]
        denom = y0 - 2.0 * y1 + y2
        if denom > 0:
            sub = best + 0.5 * (y0 - y2) / denom

    sn = sd = 0.0
    for start, stop in spans:
        lo = start + best
        if stop - lo < 2:
            continue
        s = src[lo - best: stop - best]
        d = dst[lo:stop]
        sn += float(np.dot(s, d))
        sd += float(np.dot(s, s))
    gain = sn / sd if sd > 0 else float("nan")

    return {
        "usable": True,
        "lag_ticks": best,
        "lag_ticks_subtick": sub,
        "rmse_deg": float(curve[best]),
        "rmse_at_zero_lag_deg": float(curve[0]) if np.isfinite(curve[0]) else None,
        "gain": gain,
        "at_search_bound": best == max_lag,
    }


def _first_order_fit(src: np.ndarray, dst: np.ndarray, spans: list[tuple[int, int]],
                     max_delay: int) -> dict:
    """Fit dst[k] = (1-a)*dst[k-1] + a*src[k-D] over the moving spans.

    A single "lag" number cannot tell a transport delay from a filter: both push
    the response later. This separates them -- D is dead time in ticks, a is the
    first-order coefficient, and their sum of effects is the ramp lag that a
    shift-fit would have reported on its own.
    """
    best = None
    for delay in range(max_delay + 1):
        num = den = 0.0
        count = 0
        for start, stop in spans:
            lo = max(start, delay) + 1
            if stop - lo < 2:
                continue
            k = np.arange(lo, stop)
            d_dst = dst[k] - dst[k - 1]
            d_src = src[k - delay] - dst[k - 1]
            num += float(np.dot(d_dst, d_src))
            den += float(np.dot(d_src, d_src))
            count += k.size
        if den <= 0 or count < 10:
            continue
        a = num / den
        if not 0.0 < a <= 1.5:
            continue
        resid_sq = 0.0
        for start, stop in spans:
            lo = max(start, delay) + 1
            if stop - lo < 2:
                continue
            k = np.arange(lo, stop)
            r = (dst[k] - dst[k - 1]) - a * (src[k - delay] - dst[k - 1])
            resid_sq += float(np.dot(r, r))
        rms = math.sqrt(resid_sq / count)
        if best is None or rms < best["resid_deg"]:
            best = {"delay_ticks": delay, "a": a, "resid_deg": rms}
    if best is None:
        return {"usable": False}
    a = best["a"]
    best["usable"] = True
    # a >= 1 means the stage adds no first-order lag at all (or slightly
    # overshoots); there is no time constant to report in that case.
    best["tau_ticks"] = -1.0 / math.log(1.0 - a) if a < 1.0 else 0.0
    best["ramp_lag_ticks"] = best["delay_ticks"] + (1.0 - a) / a
    return best


def _decay_spans(src: np.ndarray, moving: np.ndarray, max_len: int) -> list[tuple[int, int]]:
    """Ticks right after each move where src is held constant -- free decay."""
    out = []
    for start, stop in _segments(moving, 2):
        lo = stop
        hi = min(src.size, lo + max_len)
        held = src[lo:hi]
        if held.size < 5:
            continue
        changed = np.flatnonzero(held != held[0])
        if changed.size:
            hi = lo + int(changed[0])
        if hi - lo >= 5:
            out.append((lo, hi))
    return out


def _free_decay_pole(err: np.ndarray, spans: list[tuple[int, int]], floor_deg: float) -> dict:
    """Pole of the error decay after the command stops, by log-linear regression.

    Regression rather than a ratio median so a decay that is not actually
    exponential shows up as a poor r2 instead of being reported as a clean pole.
    """
    xs: list[np.ndarray] = []
    ys: list[np.ndarray] = []
    for start, stop in spans:
        e = np.abs(err[start:stop])
        usable = np.flatnonzero(e > floor_deg)
        if usable.size < 8:
            continue
        # Contiguous leading run only: once the error reaches the quantisation
        # floor the samples are noise and would flatten the slope.
        end = usable[0]
        while end + 1 < e.size and e[end + 1] > floor_deg:
            end += 1
        idx = np.arange(usable[0], end + 1)
        if idx.size < 8:
            continue
        xs.append(idx.astype(np.float64))
        ys.append(np.log(e[idx]))
    if not xs:
        return {"usable": False}
    x = np.concatenate(xs)
    y = np.concatenate(ys)
    slope, intercept = np.polyfit(x, y, 1)
    pred = slope * x + intercept
    ss_res = float(np.sum((y - pred) ** 2))
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    pole = math.exp(float(slope))
    if not 0.0 < pole < 1.0:
        return {"usable": False, "pole": pole, "r2": r2}
    return {
        "usable": True,
        "pole": pole,
        "a": 1.0 - pole,
        "tau_ticks": -1.0 / math.log(pole),
        "r2": r2,
        "samples": int(x.size),
        "settling_ticks": {
            label: math.log(frac) / math.log(pole)
            for label, frac in (("63%", 0.368), ("90%", 0.1), ("95%", 0.05),
                                ("99%", 0.01), ("99.9%", 0.001))
        },
    }


def _hold_stats(mat: np.ndarray, spans: list[tuple[int, int]]) -> dict:
    """How often the readback repeated verbatim -- i.e. no fresh box sample."""
    total = held = 0
    for start, stop in spans:
        block = mat[start:stop]
        if block.shape[0] < 2:
            continue
        same = np.all(block[1:] == block[:-1], axis=1)
        total += same.size
        held += int(np.count_nonzero(same))
    if not total:
        return {"usable": False}
    frac = held / total
    return {
        "usable": True,
        "held_fraction": frac,
        "samples": total,
        # Fresh readbacks per second, given the tick rate is filled in by caller.
        "fresh_fraction": 1.0 - frac,
    }


def analyze(path: Path, arms: list[str], start_sec: float, duration_sec: float,
            max_lag: int, n_segments: int, decay_floor_deg: float = 0.02) -> dict:
    required = ["loop_start_time_ns"]
    for arm in arms:
        for kind in ("actual", "sent", "ref"):
            required += [f"{arm}_q_{kind}_{j}" for j in range(DOF)]
    optional: list[str] = ["tick"]
    for arm in arms:
        optional += [
            f"{arm}_rback_observed", f"{arm}_rback_fill", f"{arm}_rback_fill_min",
            f"{arm}_rback_fill_max", f"{arm}_rback_seq", f"{arm}_rback_parsed_total",
            f"{arm}_rback_drained_total", f"{arm}_rback_malformed_total",
            f"{arm}_qsync_enabled", f"{arm}_qsync_trim_us", f"{arm}_qsync_fill_lpf",
            f"{arm}_qsync_integral_us", f"{arm}_qsync_phase", f"{arm}_qsync_locked",
            f"{arm}_qsync_underrun_events", f"{arm}_qsync_stall_events",
            f"{arm}_qsync_highwater_events", f"{arm}_qsync_redrain_events",
            f"{arm}_qsync_no_consumption_events", f"{arm}_qsync_hold_send",
            f"{arm}_qsync_warmup_holds_total", f"{arm}_state_source",
            f"{arm}_init_state_info", f"{arm}_servo_enabled",
            f"{arm}_q_ref_valid", f"{arm}_q_actual_valid",
            f"{arm}_state_age_us", f"{arm}_send_start_ns", f"{arm}_send_end_ns",
            f"{arm}_reqdata_exchange_sequence", f"{arm}_reqdata_call_duration_us",
            f"{arm}_send_ok",
        ]

    cols, absent = _read_columns(path, {"required": required, "optional": optional})

    t = cols["loop_start_time_ns"] * 1e-9
    t = t - t[0]
    keep = t >= start_sec
    if duration_sec > 0:
        keep &= t < start_sec + duration_sec
    if np.count_nonzero(keep) < MIN_SEGMENT_TICKS:
        raise LogError(
            f"window [{start_sec}, {start_sec + duration_sec if duration_sec > 0 else 'end'}) "
            f"keeps only {int(np.count_nonzero(keep))} ticks; need >= {MIN_SEGMENT_TICKS}"
        )
    idx = np.flatnonzero(keep)
    cols = {k: (v[idx] if isinstance(v, np.ndarray) and v.dtype != object else v) for k, v in cols.items()}
    t = t[idx]

    dt = np.diff(t)
    tick_hz = 1.0 / float(np.median(dt))
    report: dict = {
        "log": str(path),
        "window_sec": [float(t[0]), float(t[-1])],
        "ticks": int(t.size),
        "tick_rate_hz": tick_hz,
        "tick_dt_ms": {
            "median": float(np.median(dt) * 1e3),
            "p95": float(np.percentile(dt, 95) * 1e3),
            "max": float(dt.max() * 1e3),
            "over_2x_period": int(np.count_nonzero(dt > 2 * np.median(dt))),
        },
        "columns_absent": absent,
        "arms": {},
    }

    for arm in arms:
        sent = _joint_matrix(cols, arm, "sent")
        ref = _joint_matrix(cols, arm, "ref")
        actual = _joint_matrix(cols, arm, "actual")

        finite = np.all(np.isfinite(sent), axis=1) & np.all(np.isfinite(ref), axis=1) & np.all(np.isfinite(actual), axis=1)
        step = np.zeros(sent.shape[0])
        step[1:] = np.abs(np.diff(sent, axis=0)).max(axis=1)
        moving = (step > MOVING_DEG_PER_TICK) & finite
        spans = _segments(moving, MIN_SEGMENT_TICKS)

        arm_report: dict = {
            "moving_ticks": int(np.count_nonzero(moving)),
            "moving_segments": len(spans),
            "longest_segment_sec": (max((b - a) for a, b in spans) / tick_hz) if spans else 0.0,
        }

        ref_valid = cols.get(f"{arm}_q_ref_valid")
        if isinstance(ref_valid, np.ndarray):
            arm_report["q_ref_valid_fraction"] = float(np.nanmean(ref_valid))

        if not spans:
            arm_report["error"] = (
                "no moving segment of >= %d ticks; the arm was idle in this window, "
                "so box latency is not observable here" % MIN_SEGMENT_TICKS
            )
            report["arms"][arm] = arm_report
            continue

        hold_ref = _hold_stats(ref, spans)
        hold_act = _hold_stats(actual, spans)
        if hold_ref.get("usable"):
            hold_ref["implied_update_hz"] = hold_ref["fresh_fraction"] * tick_hz
        if hold_act.get("usable"):
            hold_act["implied_update_hz"] = hold_act["fresh_fraction"] * tick_hz
        arm_report["readback_freshness"] = {"q_ref": hold_ref, "q_actual": hold_act}

        # Control-box command-queue occupancy (RBACK). fill == -1 means the
        # firmware never reported one; that is a distinct state from an empty
        # queue and must not be averaged in as a zero.
        fill = cols.get(f"{arm}_rback_fill")
        if isinstance(fill, np.ndarray) and np.any(np.isfinite(fill)):
            valid = fill[np.isfinite(fill) & (fill >= 0)]
            parsed = cols.get(f"{arm}_rback_parsed_total")
            drained = cols.get(f"{arm}_rback_drained_total")
            malformed = cols.get(f"{arm}_rback_malformed_total")
            qa: dict = {
                "reported_ticks": int(valid.size),
                "never_reported_ticks": int(np.count_nonzero(fill < 0)),
            }
            if valid.size:
                counts = {int(v): int(c) for v, c in zip(*np.unique(valid, return_counts=True))}
                qa.update({
                    "fill_median": float(np.median(valid)),
                    "fill_min": int(valid.min()),
                    "fill_max": int(valid.max()),
                    "fill_histogram": counts,
                    "fill_constant": len(counts) == 1,
                })
                mov_fill = fill[moving] if moving.size == fill.size else valid
                mov_fill = mov_fill[np.isfinite(mov_fill) & (mov_fill >= 0)]
                if mov_fill.size:
                    qa["fill_median_while_moving"] = float(np.median(mov_fill))
                # The box queue is an integrator: dfill/dt = f_send - f_box. An
                # unregulated stream therefore shows a LINEAR fill ramp, and that
                # slope IS the latency drift rate. A constant fill means the
                # send rate is locked to the box; a rising one means it is not.
                reported = np.isfinite(fill) & (fill >= 0)
                if int(np.count_nonzero(reported)) > 100:
                    tt = t[reported]
                    ff = fill[reported]
                    # Skip the startup charge-up so it cannot bias the slope. The
                    # fixed 0.2 s guess was far too short -- a measured convergence
                    # took ~6 s, so the fit swallowed the warmup ramp and reported
                    # DRIFTING on a queue that was simply still settling. When the
                    # regulator logs its phase, use the real thing: fit from the
                    # first tick it reached Track. Everything after that is kept,
                    # including a later fall OUT of Track, because dropping those
                    # ticks would hide exactly the failure worth seeing.
                    warm_from = tt[0] + 0.2
                    phase = cols.get(f"{arm}_qsync_phase")
                    if isinstance(phase, np.ndarray) and phase.dtype == object:
                        track = np.flatnonzero(phase[reported] == "track")
                        if track.size:
                            warm_from = tt[track[0]]
                            qa["drift_fit_from_sec"] = float(warm_from)
                            qa["drift_fit_gate"] = "qsync phase reached track"
                    warm = tt >= warm_from
                    qa["drift_fit_skipped_ticks"] = int(np.count_nonzero(~warm))
                    if int(np.count_nonzero(warm)) > 100:
                        slope, intercept = np.polyfit(tt[warm], ff[warm], 1)
                        resid = ff[warm] - (slope * tt[warm] + intercept)
                        qa["fill_drift"] = {
                            "ticks_per_sec": float(slope),
                            "ms_per_sec": float(slope) * 1000.0 / tick_hz,
                            "residual_std_ticks": float(resid.std()),
                            "implied_box_drain_hz": float(tick_hz - slope),
                        }
            for key, series in (("parsed_total", parsed), ("drained_total", drained),
                                ("malformed_total", malformed)):
                if isinstance(series, np.ndarray) and np.any(np.isfinite(series)):
                    fin = series[np.isfinite(series)]
                    qa[key] = int(fin.max())
            if qa.get("parsed_total") and qa.get("drained_total"):
                qa["parsed_fraction_of_drained"] = qa["parsed_total"] / qa["drained_total"]
            arm_report["rback_queue"] = qa

        # The regulator beside the plant. Reported separately because a fill that
        # looks wrong and a controller that IS wrong are different findings: the
        # trim is the actuator, so a trim pinned at its clamp is saturation, not
        # a tuning opinion.
        enabled = cols.get(f"{arm}_qsync_enabled")
        if isinstance(enabled, np.ndarray) and np.any(np.isfinite(enabled)) and np.any(enabled > 0):
            on = np.isfinite(enabled) & (enabled > 0)
            qs: dict = {"enabled_ticks": int(np.count_nonzero(on))}
            trim = cols.get(f"{arm}_qsync_trim_us")
            if isinstance(trim, np.ndarray):
                v = trim[on]
                v = v[np.isfinite(v)]
                if v.size:
                    qs["trim_us"] = {
                        "median": float(np.median(v)), "p5": float(np.percentile(v, 5)),
                        "p95": float(np.percentile(v, 95)),
                        "min": float(v.min()), "max": float(v.max()),
                    }
            for key, name in (("fill_lpf", "fill_lpf"), ("integral_us", "integral_us")):
                series = cols.get(f"{arm}_qsync_{key}")
                if isinstance(series, np.ndarray):
                    v = series[on]
                    v = v[np.isfinite(v)]
                    if v.size:
                        qs[name] = {"median": float(np.median(v)),
                                    "last": float(v[-1]), "max_abs": float(np.abs(v).max())}
            locked = cols.get(f"{arm}_qsync_locked")
            if isinstance(locked, np.ndarray):
                v = locked[on]
                v = v[np.isfinite(v)]
                if v.size:
                    qs["locked_fraction"] = float(np.count_nonzero(v > 0) / v.size)
            phase = cols.get(f"{arm}_qsync_phase")
            if isinstance(phase, np.ndarray) and phase.dtype == object:
                names, counts = np.unique(phase[on], return_counts=True)
                total = int(counts.sum())
                qs["phase_fractions"] = {str(n): int(c) / total for n, c in zip(names, counts)}
                qs["phase_last"] = str(phase[on][-1])
            holds = cols.get(f"{arm}_qsync_hold_send")
            if isinstance(holds, np.ndarray) and np.any(np.isfinite(holds)):
                h = holds[np.isfinite(holds)]
                qs["warmup_held_ticks"] = int(np.count_nonzero(h > 0))
                if np.any(h > 0):
                    first = int(np.flatnonzero(h > 0)[0])
                    last = int(np.flatnonzero(h > 0)[-1])
                    qs["warmup_hold_window_sec"] = [float(t[first]), float(t[last])]
            events = {}
            for key in ("underrun", "stall", "highwater", "redrain", "no_consumption"):
                series = cols.get(f"{arm}_qsync_{key}_events")
                if isinstance(series, np.ndarray):
                    v = series[np.isfinite(series)]
                    if v.size:
                        events[key] = int(v.max())
            if events:
                qs["events_total"] = events
            arm_report["queue_sync"] = qs

        # WHICH read path served each tick, and how often it had to hold. This is
        # the number that decides whether the pipelined (non-blocking) state read is
        # usable on real hardware; `state_age_us` cannot show it, because a held
        # state keeps its original stamp and so reports an honest -- but
        # indistinguishable -- age.
        src = cols.get(f"{arm}_state_source")
        if isinstance(src, np.ndarray) and src.dtype == object:
            names, counts = np.unique(src, return_counts=True)
            total = int(counts.sum())
            if total:
                dist = {str(n): int(c) for n, c in zip(names, counts)}
                held = sum(c for n, c in dist.items() if "held" in n or "hold" in n)
                runs = []
                is_held = np.array(["held" in str(x) or "hold" in str(x) for x in src])
                run = 0
                for v in is_held:
                    if v:
                        run += 1
                    elif run:
                        runs.append(run)
                        run = 0
                if run:
                    runs.append(run)
                arm_report["state_source"] = {
                    "distribution": dist,
                    "held_fraction": held / total,
                    "held_ticks": int(held),
                    "longest_held_run_ticks": int(max(runs)) if runs else 0,
                    "held_runs": len(runs),
                }

        # The box's own activation stage (init_state_info; 6 = servo on). This
        # exists to answer whether the startup queue backlog is streamed into a
        # box that has not finished activating -- if so, gating the stream on
        # activation removes it at the source; if the box was already at 6, that
        # gate is a no-op and the cause is elsewhere.
        stage = cols.get(f"{arm}_init_state_info")
        enabled = cols.get(f"{arm}_servo_enabled")
        if isinstance(stage, np.ndarray) and np.any(np.isfinite(stage)):
            act: dict = {}
            fin = stage[np.isfinite(stage)]
            if fin.size:
                names, counts = np.unique(fin, return_counts=True)
                act["stage_histogram"] = {int(n): int(c) for n, c in zip(names, counts)}
            if isinstance(enabled, np.ndarray) and np.any(np.isfinite(enabled)):
                on = np.isfinite(enabled) & (enabled > 0)
                act["enabled_fraction"] = float(np.count_nonzero(on) / on.size)
                if np.any(on) and not on[0]:
                    first = int(np.flatnonzero(on)[0])
                    act["enabled_first_tick"] = first
                    act["enabled_first_sec"] = float(t[first])
                elif np.all(on):
                    act["enabled_from_first_tick"] = True
            # Was the queue already growing before the box was enabled?
            fill_col = cols.get(f"{arm}_rback_fill")
            if (isinstance(fill_col, np.ndarray) and isinstance(enabled, np.ndarray)
                    and "enabled_first_tick" in act):
                pre = fill_col[: act["enabled_first_tick"]]
                pre = pre[np.isfinite(pre) & (pre >= 0)]
                if pre.size:
                    act["fill_max_before_enabled"] = int(pre.max())
            arm_report["box_activation"] = act

        seq = cols.get(f"{arm}_reqdata_exchange_sequence")
        if isinstance(seq, np.ndarray) and np.any(np.isfinite(seq)):
            adv = np.diff(seq)
            adv = adv[np.isfinite(adv)]
            if adv.size:
                arm_report["read_call_advance_fraction"] = float(np.mean(adv > 0))

        for label, series in (
            ("state_age_us", cols.get(f"{arm}_state_age_us")),
            ("reqdata_call_duration_us", cols.get(f"{arm}_reqdata_call_duration_us")),
        ):
            if isinstance(series, np.ndarray) and np.any(np.isfinite(series)):
                vals = series[np.isfinite(series)]
                arm_report[label] = {
                    "median": float(np.median(vals)),
                    "p95": float(np.percentile(vals, 95)),
                    "max": float(vals.max()),
                }
        send_start = cols.get(f"{arm}_send_start_ns")
        send_end = cols.get(f"{arm}_send_end_ns")
        if isinstance(send_start, np.ndarray) and isinstance(send_end, np.ndarray):
            ok = np.isfinite(send_start) & np.isfinite(send_end) & (send_start > 0) & (send_end >= send_start)
            if np.any(ok):
                dur = (send_end[ok] - send_start[ok]) * 1e-3
                arm_report["servo_j_send_us"] = {
                    "median": float(np.median(dur)),
                    "p95": float(np.percentile(dur, 95)),
                    "max": float(dur.max()),
                }

        stages: dict = {}
        for stage, (src_mat, dst_mat) in {
            "sent_to_ref": (sent, ref),
            "ref_to_actual": (ref, actual),
            "sent_to_actual": (sent, actual),
        }.items():
            per_joint = []
            for j in range(DOF):
                excitation = 0.0
                for a, b in spans:
                    block = src_mat[a:b, j]
                    excitation = max(excitation, float(block.max() - block.min()))
                if excitation < MIN_EXCITATION_DEG:
                    per_joint.append({"joint": j, "usable": False, "excitation_deg": excitation})
                    continue
                fit = _lag_rmse(src_mat[:, j], dst_mat[:, j], spans, max_lag)
                fit["joint"] = j
                fit["excitation_deg"] = excitation
                per_joint.append(fit)
            # Transport-vs-filter split on the joint with the most excitation,
            # plus the free decay after the command stops. The shift fit alone
            # would read a 1-tick dead time behind a slow filter as a 10-tick
            # transport delay.
            best_joint = max(range(DOF), key=lambda jj: max(
                (float(src_mat[a:b, jj].max() - src_mat[a:b, jj].min()) for a, b in spans),
                default=0.0))
            model = _first_order_fit(src_mat[:, best_joint], dst_mat[:, best_joint],
                                     spans, max_lag)
            model["joint"] = best_joint
            decay = _free_decay_pole(
                src_mat[:, best_joint] - dst_mat[:, best_joint],
                _decay_spans(src_mat[:, best_joint], moving, max_len=4 * max_lag),
                floor_deg=decay_floor_deg,
            )
            decay["joint"] = best_joint
            # A free decay is only a FILTER measurement when the stage has no
            # meaningful dead time. With a command queue the tail after the
            # command stops is the queue DRAINING -- dead time, not an
            # exponential -- and fitting a pole to it reports a filter that is
            # not there. Flag it rather than letting the number stand.
            dead = model.get("delay_ticks") if model.get("usable") else None
            if dead is not None and dead >= 3:
                decay["confounded_by_dead_time"] = True
                decay["dead_time_ticks"] = int(dead)

            usable = [f for f in per_joint if f.get("usable")]
            summary = {}
            if usable:
                lags = np.array([f["lag_ticks_subtick"] for f in usable])
                weights = np.array([f["excitation_deg"] for f in usable])
                summary = {
                    "lag_ticks_weighted": float(np.average(lags, weights=weights)),
                    "lag_ms_weighted": float(np.average(lags, weights=weights) / tick_hz * 1e3),
                    "lag_ticks_min": float(lags.min()),
                    "lag_ticks_max": float(lags.max()),
                    "joints_used": [f["joint"] for f in usable],
                    "any_at_search_bound": any(f.get("at_search_bound") for f in usable),
                }
            stages[stage] = {"per_joint": per_joint, "summary": summary,
                             "model": model, "free_decay": decay}
        arm_report["stages"] = stages

        # Stationarity: split the moving span budget into K chunks and refit, so a
        # slowly growing box queue shows up as a rising lag rather than an average.
        if n_segments > 1:
            drift = []
            total_moving = sum(b - a for a, b in spans)
            chunk = total_moving // n_segments
            if chunk >= MIN_SEGMENT_TICKS:
                acc = 0
                bucket: list[tuple[int, int]] = []
                for a, b in spans:
                    cur = a
                    while cur < b:
                        take = min(b - cur, chunk - acc)
                        bucket.append((cur, cur + take))
                        acc += take
                        cur += take
                        if acc >= chunk:
                            j = max(range(DOF), key=lambda jj: max(
                                (float(sent[s:e, jj].max() - sent[s:e, jj].min()) for s, e in bucket), default=0.0))
                            fit = _lag_rmse(sent[:, j], ref[:, j], [s for s in bucket if s[1] - s[0] >= 2], max_lag)
                            drift.append({
                                "t_start_sec": float(t[bucket[0][0]]),
                                "t_end_sec": float(t[bucket[-1][1] - 1]),
                                "joint": j,
                                "sent_to_ref_lag_ticks": fit.get("lag_ticks_subtick"),
                                "sent_to_ref_lag_ms": (fit["lag_ticks_subtick"] / tick_hz * 1e3) if fit.get("usable") else None,
                            })
                            bucket, acc = [], 0
                arm_report["sent_to_ref_drift"] = drift

        report["arms"][arm] = arm_report

    return report


def format_report(report: dict) -> str:
    lines: list[str] = []
    w = report["window_sec"]
    lines.append(f"log            {report['log']}")
    lines.append(
        f"window         {w[0]:.2f}..{w[1]:.2f} s  ({report['ticks']} ticks, "
        f"{report['tick_rate_hz']:.1f} Hz)"
    )
    d = report["tick_dt_ms"]
    lines.append(
        f"tick dt        median={d['median']:.3f} ms  p95={d['p95']:.3f}  max={d['max']:.3f}  "
        f"overruns(>2x)={d['over_2x_period']}"
    )
    if report["columns_absent"]:
        lines.append(f"absent columns {', '.join(report['columns_absent'])}")

    for arm, r in report["arms"].items():
        lines.append("")
        lines.append(f"--- {arm} ---")
        if "error" in r:
            lines.append(f"  {r['error']}")
            continue
        lines.append(
            f"  moving        {r['moving_ticks']} ticks in {r['moving_segments']} segment(s), "
            f"longest {r['longest_segment_sec']:.2f} s"
        )
        if "q_ref_valid_fraction" in r:
            lines.append(f"  q_ref_valid   {r['q_ref_valid_fraction'] * 100:.2f} %")
        fresh = r.get("readback_freshness", {})
        for name, st in fresh.items():
            if st.get("usable"):
                lines.append(
                    f"  {name:<12}  fresh {st['fresh_fraction'] * 100:5.1f} % of ticks "
                    f"-> implied box update ~{st['implied_update_hz']:.0f} Hz"
                )
        qa = r.get("rback_queue")
        if qa:
            if not qa.get("reported_ticks"):
                lines.append("  RBACK queue    firmware reported no occupancy on any tick")
            else:
                const = "  CONSTANT" if qa.get("fill_constant") else ""
                lines.append(
                    f"  RBACK queue    fill median={qa['fill_median']:.1f}  "
                    f"min={qa['fill_min']}  max={qa['fill_max']}{const}"
                )
                if "fill_median_while_moving" in qa:
                    lines.append(f"                 while moving: median={qa['fill_median_while_moving']:.1f}")
                hist = qa.get("fill_histogram", {})
                if hist:
                    total = sum(hist.values())
                    shown = sorted(hist.items())[:8]
                    lines.append("                 histogram: " + "  ".join(
                        f"{v}:{c * 100.0 / total:.1f}%" for v, c in shown)
                        + ("  ..." if len(hist) > 8 else ""))
                drift = qa.get("fill_drift")
                if drift:
                    verdict = ("LOCKED" if abs(drift["ticks_per_sec"]) < 0.05
                               else "DRIFTING -- the send rate is not locked to the box")
                    lines.append(
                        f"                 drift {drift['ticks_per_sec']:+.3f} tk/s "
                        f"({drift['ms_per_sec']:+.2f} ms/s)  implied box drain "
                        f"{drift['implied_box_drain_hz']:.2f} Hz  -> {verdict}"
                    )
                if qa.get("never_reported_ticks"):
                    lines.append(f"                 {qa['never_reported_ticks']} tick(s) before the first RBACK")
                if qa.get("malformed_total"):
                    lines.append(
                        f"                 !! {qa['malformed_total']} drained response(s) mentioned RBACK "
                        f"but did not parse -- wire format may have changed")
                if "parsed_fraction_of_drained" in qa:
                    lines.append(
                        f"                 parsed {qa['parsed_total']} RBACK of {qa['drained_total']} drained "
                        f"({qa['parsed_fraction_of_drained'] * 100:.1f} %)")
                if "drift_fit_from_sec" in qa:
                    lines.append(
                        f"                 drift fit from t={qa['drift_fit_from_sec']:.2f}s "
                        f"({qa['drift_fit_gate']}); skipped {qa.get('drift_fit_skipped_ticks', 0)} warmup tick(s)")

        ss = r.get("state_source")
        if ss:
            shown = "  ".join(f"{n}:{c}" for n, c in sorted(
                ss["distribution"].items(), key=lambda kv: -kv[1]))
            lines.append(f"  state source   {shown}")
            lines.append(
                f"                 held {ss['held_ticks']} tick(s) "
                f"({ss['held_fraction'] * 100:.2f} %) in {ss['held_runs']} run(s), "
                f"longest {ss['longest_held_run_ticks']} tick(s)")

        act = r.get("box_activation")
        if act:
            hist = act.get("stage_histogram", {})
            shown = "  ".join(f"{k}:{v}" for k, v in sorted(hist.items()))
            lines.append(f"  box activation  init_state_info  {shown}   (6 = servo on)")
            if "enabled_first_sec" in act:
                lines.append(
                    f"                 servo_enabled first true at t={act['enabled_first_sec']:.3f}s "
                    f"(tick {act['enabled_first_tick']})"
                    + (f"; RBACK fill reached {act['fill_max_before_enabled']} BEFORE that"
                       if "fill_max_before_enabled" in act else ""))
            elif act.get("enabled_from_first_tick"):
                lines.append("                 servo_enabled was already true on the first logged tick "
                             "-- an activation gate would not have changed this run")

        qs = r.get("queue_sync")
        if qs:
            trim = qs.get("trim_us", {})
            if trim:
                lines.append(
                    f"  qsync trim     median={trim['median']:+.1f} us  "
                    f"p5..p95={trim['p5']:+.1f}..{trim['p95']:+.1f}  "
                    f"min/max={trim['min']:+.1f}/{trim['max']:+.1f}")
            integ = qs.get("integral_us", {})
            if integ:
                lines.append(
                    f"                 integral last={integ['last']:+.2f} us "
                    f"(median {integ['median']:+.2f}, |max| {integ['max_abs']:.2f}) "
                    f"-- this is the learned clock mismatch")
            if "phase_fractions" in qs:
                shown = "  ".join(f"{n}:{f * 100:.1f}%" for n, f in sorted(
                    qs["phase_fractions"].items(), key=lambda kv: -kv[1]))
                lines.append(f"                 phase: {shown}  (last={qs.get('phase_last', '?')})")
            if "locked_fraction" in qs:
                lines.append(f"                 locked {qs['locked_fraction'] * 100:.1f} % of regulated ticks")
            if qs.get("warmup_held_ticks"):
                win = qs.get("warmup_hold_window_sec")
                where = f"  t={win[0]:.2f}..{win[1]:.2f}s" if win else ""
                lines.append(
                    f"                 warmup back-pressure: {qs['warmup_held_ticks']} send(s) held"
                    f"{where}  -- the box showed no evidence it was consuming")
            elif "warmup_held_ticks" in qs:
                lines.append("                 warmup back-pressure: never engaged "
                             "(the queue showed depth before the trigger)")
            ev = qs.get("events_total", {})
            if ev:
                hot = {k: v for k, v in ev.items() if v}
                lines.append(
                    "                 events: " + ("  ".join(f"{k}={v}" for k, v in ev.items()))
                    + ("" if not hot else "   <-- nonzero: " + ", ".join(hot)))

        if "read_call_advance_fraction" in r:
            lines.append(f"  read calls    advanced on {r['read_call_advance_fraction'] * 100:.1f} % of ticks")
        for key, label in (("servo_j_send_us", "servo_j send"),
                           ("state_age_us", "state age"),
                           ("reqdata_call_duration_us", "reqdata call")):
            if key in r:
                s = r[key]
                lines.append(
                    f"  {label:<12}  median={s['median']:.1f} us  p95={s['p95']:.1f}  max={s['max']:.1f}"
                )

        lines.append("")
        lines.append("  stage           lag(ticks)   lag(ms)   per-joint ticks [gain, resid deg]")
        for stage, data in r["stages"].items():
            s = data["summary"]
            if not s:
                lines.append(f"  {stage:<14}  (no joint had enough excitation)")
                continue
            detail = "  ".join(
                f"J{f['joint']}:{f['lag_ticks_subtick']:.2f}[{f['gain']:.3f},{f['rmse_deg']:.4f}]"
                for f in data["per_joint"] if f.get("usable")
            )
            bound = "  !! at search bound" if s["any_at_search_bound"] else ""
            lines.append(
                f"  {stage:<14}  {s['lag_ticks_weighted']:>8.2f}   {s['lag_ms_weighted']:>7.2f}   {detail}{bound}"
            )

        lines.append("")
        lines.append("  transport vs filter (first-order + dead time, fitted on the most excited joint)")
        lines.append("    stage           dead time   filter a    tau       ramp lag   resid deg")
        for stage, data in r["stages"].items():
            m = data.get("model", {})
            if not m.get("usable"):
                lines.append(f"    {stage:<14}  (not identifiable in this window)")
                continue
            lines.append(
                f"    {stage:<14}  {m['delay_ticks']:>5d} tk    {m['a']:.4f}    "
                f"{m['tau_ticks']:6.2f} tk  {m['ramp_lag_ticks']:7.2f} tk   {m['resid_deg']:.5f}   (J{m['joint']})"
            )
        for stage, data in r["stages"].items():
            d = data.get("free_decay", {})
            if not d.get("usable"):
                continue
            st = d["settling_ticks"]
            lines.append("")
            lines.append(
                f"    {stage} free decay after the command stops (J{d['joint']}): "
                f"pole={d['pole']:.4f}/tick  a={d['a']:.4f}  tau={d['tau_ticks']:.2f} tk  "
                f"r2={d['r2']:.4f}  n={d['samples']}"
            )
            if d.get("confounded_by_dead_time"):
                lines.append(
                    f"      !! NOT a filter measurement: this stage has {d['dead_time_ticks']} ticks "
                    f"of dead time, so the tail is the queue draining, not an exponential."
                )
            ms_per_tick = 1000.0 / report["tick_rate_hz"]
            lines.append("      error settles to: " + "  ".join(
                f"{lbl} in {n:.1f} tk ({n * ms_per_tick:.0f} ms)" for lbl, n in st.items()))

        drift = r.get("sent_to_ref_drift")
        if drift:
            lines.append("")
            lines.append("  sent->ref stationarity (is the box queue growing?)")
            for chunk in drift:
                if chunk["sent_to_ref_lag_ms"] is None:
                    continue
                lines.append(
                    f"    {chunk['t_start_sec']:7.2f}..{chunk['t_end_sec']:7.2f} s  J{chunk['joint']}  "
                    f"{chunk['sent_to_ref_lag_ticks']:.2f} ticks  ({chunk['sent_to_ref_lag_ms']:.2f} ms)"
                )
    return "\n".join(lines)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("log", nargs="?", default="logs/servo_log.csv")
    p.add_argument("--arm", choices=("left", "right", "both"), default="both")
    p.add_argument("--start-sec", type=float, default=0.0)
    p.add_argument("--duration-sec", type=float, default=0.0, help="0 = to end of log")
    p.add_argument("--max-lag-ticks", type=int, default=40, help="search bound (40 ticks = 80 ms at 500 Hz)")
    p.add_argument("--segments", type=int, default=6, help="chunks for the sent->ref stationarity check; 1 disables")
    p.add_argument("--decay-floor-deg", type=float, default=0.02,
                   help="ignore free-decay samples below this error; must stay well "
                        "above the log's 0.001 deg quantisation (default 0.02)")
    p.add_argument("--json", action="store_true")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    arms = list(ARMS) if args.arm == "both" else [args.arm]
    try:
        report = analyze(
            Path(args.log), arms, args.start_sec, args.duration_sec,
            args.max_lag_ticks, args.segments, args.decay_floor_deg,
        )
    except LogError as exc:
        print(f"[analyze-box-latency] {exc}", file=sys.stderr)
        return 2
    print(json.dumps(report, indent=2) if args.json else format_report(report))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
