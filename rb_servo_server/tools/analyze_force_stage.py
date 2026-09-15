#!/usr/bin/env python3
"""Read-only force-stage audit. No robot or backend is constructed.

New logs use one target, a dynamic excess-force law and a confidence-weighted
force gate. Law checks use its own filtered stand vector, never raw sensor norm.
Legacy pair logs remain readable with explicit legacy parameters. PASS describes
logged invariants, not proof of contact stability or physical safety. A net F/T
measurement cannot label a floor or separate simultaneous hand/floor forces.
Requires numpy only.
"""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


ARMS = ("left", "right")
SCHEMA = "robotics_lab.analyze_force_stage.v2"
PASS, FAIL, SKIPPED = "PASS", "FAIL", "SKIPPED"

# Columns whose values are words, not numbers.
STRING_SUFFIXES = ("_fc_source", "_fc_fold_sink", "_fc_coverage_reason", "_fc_law")


def column_map(arm):
    """Every column one arm's checks may want, by role. Lists are vectors."""
    return {
        "time": "loop_start_time_ns",
        "fault": "fault_latched",
        "covered": f"{arm}_fc_covered",
        "force_nodz": [f"{arm}_ft_comp_sensor_nodz_f{ax}_n" for ax in "xyz"],
        "gate_force": f"{arm}_fc_gate_force_n",
        "force_filt": [f"{arm}_fc_wrench_filt_f{ax}_n" for ax in "xyz"],
        "force_stand": [f"{arm}_ft_comp_stand_f{ax}_n" for ax in "xyz"],
        "vel": [f"{arm}_fc_vel_{ax}_m_s" for ax in "xyz"],
        "cmd": [f"{arm}_tcp_command_stand_{ax}_m" for ax in "xyz"],
        "source": f"{arm}_fc_source",
        "demand": f"{arm}_fc_source_demand_m_s",
        "gate": f"{arm}_fc_gate_translation",
        "fold_sink": f"{arm}_fc_fold_sink",
        "normal": [f"{arm}_fc_contact_normal_{ax}" for ax in "xyz"],
        "bounded": f"{arm}_fc_bounded",
        "accel": [f"{arm}_q_sent_accel_deg_s2_{j}" for j in range(6)],
        "target": f"{arm}_fc_target_force_n",
        "mass": f"{arm}_fc_gate_m_eff",
        "damping": f"{arm}_fc_gate_b_eff",
        "confidence": f"{arm}_fc_contact_confidence",
        "physical_gate": f"{arm}_fc_physical_gate",
        "osc_frozen": f"{arm}_fc_osc_frozen",
        "law": f"{arm}_fc_law",           # pre-2026-09-15 logs only
    }


def flatten(mapping):
    out = []
    for value in mapping.values():
        out.extend(value if isinstance(value, list) else [value])
    return out


def is_string_column(name):
    return name.endswith(STRING_SUFFIXES)


def read_log(path, arms):
    """Only the wanted columns, parsed once. Returns (arrays, header_set, malformed)."""
    wanted = set()
    for arm in arms:
        wanted.update(flatten(column_map(arm)))
    with Path(path).open(newline="") as stream:
        reader = csv.reader(stream)
        header = next(reader)
        selected = [(i, name) for i, name in enumerate(header) if name in wanted]
        result = {name: [] for _, name in selected}
        malformed = 0
        for row in reader:
            if len(row) != len(header):
                malformed += 1
                continue
            for index, name in selected:
                raw = row[index]
                if is_string_column(name):
                    result[name].append(raw)
                elif name == "loop_start_time_ns":
                    try:
                        result[name].append(int(raw))
                    except ValueError:
                        result[name].append(0)
                else:
                    try:
                        result[name].append(float(raw))
                    except ValueError:
                        result[name].append(float("nan"))
    arrays = {}
    for name, values in result.items():
        if is_string_column(name):
            arrays[name] = np.asarray(values, dtype=object)
        elif name == "loop_start_time_ns":
            arrays[name] = np.asarray(values, dtype=np.int64)
        else:
            arrays[name] = np.asarray(values, dtype=float)
    return arrays, set(header), malformed


class Missing(Exception):
    """A required column is absent: the check is SKIPPED."""


class Columns:
    """Role -> array access with a single failure mode (Missing)."""

    def __init__(self, arrays, arm):
        self.arrays = arrays
        self.map = column_map(arm)

    def has(self, role):
        names = self.map[role]
        names = names if isinstance(names, list) else [names]
        return all(name in self.arrays for name in names)

    def get(self, role):
        names = self.map[role]
        if isinstance(names, list):
            missing = [n for n in names if n not in self.arrays]
            if missing:
                raise Missing(", ".join(missing))
            return np.column_stack([self.arrays[n] for n in names])
        if names not in self.arrays:
            raise Missing(names)
        return self.arrays[names]


def verdict(name, status, detail, **numbers):
    clean = {}
    for key, value in numbers.items():
        if isinstance(value, (np.floating, float)):
            clean[key] = None if not math.isfinite(float(value)) else float(value)
        elif isinstance(value, (np.integer, int, np.bool_, bool)):
            clean[key] = int(value)
        else:
            clean[key] = value
    return {"check": name, "status": status, "detail": detail, "numbers": clean}


def runs_of(mask, time_s, max_gap_s):
    """Contiguous True runs as [start, stop) pairs, split where the clock jumps."""
    mask = np.asarray(mask, dtype=bool)
    out, start = [], None
    for i in range(len(mask)):
        connected = i > 0 and 0.0 < time_s[i] - time_s[i - 1] <= max_gap_s
        if start is not None and (not mask[i] or not connected):
            out.append((start, i))
            start = None
        if mask[i] and start is None:
            start = i
    if start is not None:
        out.append((start, len(mask)))
    return out


def sliding_range_norm(points, window):
    """Per-window norm of the per-axis (max - min) of `points` (N x 3): an upper
    bound on the displacement between any two samples in the window."""
    n = len(points)
    if n == 0:
        return np.zeros(0)
    window = max(1, min(window, n))
    view = np.lib.stride_tricks.sliding_window_view(points, window, axis=0)
    ranges = view.max(axis=-1) - view.min(axis=-1)
    return np.linalg.norm(ranges, axis=1)


def sliding_count(events, window):
    """Number of True in every `window`-long slice of `events`."""
    events = np.asarray(events, dtype=int)
    n = len(events)
    if n == 0:
        return np.zeros(0, dtype=int)
    window = max(1, min(window, n))
    cumulative = np.concatenate([[0], np.cumsum(events)])
    return cumulative[window:] - cumulative[:-window]


def gate_model(force_n, demand_m_s, peak_n, v_cross_m_s):
    """CM 0049's curve, ignoring the open/close slew."""
    force_n = np.asarray(force_n, dtype=float)
    demand = np.asarray(demand_m_s, dtype=float)
    g = np.ones_like(force_n)
    active = np.isfinite(demand) & (demand > v_cross_m_s) & np.isfinite(force_n)
    ratio = np.where(active, v_cross_m_s / np.where(active, demand, 1.0), 1.0)
    exponent = np.clip(force_n / peak_n, 0.0, None) ** 2
    g[active] = ratio[active] ** exponent[active]
    return np.clip(g, 0.0, 1.0)


class ArmAudit:
    def __init__(self, arrays, arm, params):
        self.arm = arm
        self.c = Columns(arrays, arm)
        self.p = params
        self.time_s = (arrays["loop_start_time_ns"] - arrays["loop_start_time_ns"][0]) * 1e-9
        positive = np.diff(arrays["loop_start_time_ns"])
        positive = positive[positive > 0]
        self.dt = float(np.median(positive)) * 1e-9 if len(positive) else 0.002
        self.max_gap = 2.5 * self.dt
        self.covered = self.c.get("covered") >= 0.5 if self.c.has("covered") else None
        self.new_law = self.c.has("target") or params.target_n is not None
        self.target_n = params.target_n if params.target_n is not None else params.rest_n
        self.mass, self.damping = params.mass, params.b
        for role, attr in (("target", "target_n"), ("mass", "mass"), ("damping", "damping")):
            if self.c.has(role):
                values = self.c.get(role)
                values = values[np.isfinite(values) & (values > 0)]
                if len(values):
                    setattr(self, attr, float(np.median(values)))
        self.force_n, self.force_source = self._force_magnitude()
        self.force_dir, self.dir_source = self._force_direction()

    # ---- inputs ---------------------------------------------------------------
    def _force_magnitude(self):
        if self.c.has("force_filt"):
            return np.linalg.norm(self.c.get("force_filt"), axis=1), "fc_wrench_filt"
        return None, None

    def _force_direction(self):
        for role in ("force_filt", "force_stand"):
            if self.c.has(role):
                vec = self.c.get(role)
                norm = np.linalg.norm(vec, axis=1)
                with np.errstate(invalid="ignore", divide="ignore"):
                    unit = vec / np.where(norm > 0, norm, np.nan)[:, None]
                return unit, role
        return None, None

    def ticks(self, seconds):
        return max(1, int(round(seconds / self.dt)))

    def need_covered(self):
        if self.covered is None:
            raise Missing(self.c.map["covered"])
        if not self.covered.any():
            raise Missing("no covered tick")
        return self.covered

    def need_force(self):
        if self.force_n is None:
            raise Missing(", ".join(self.c.map["force_nodz"]) + " / " + self.c.map["gate_force"])
        return self.force_n

    # ---- checks ---------------------------------------------------------------
    def check_rest_equilibrium(self):
        covered = self.need_covered()
        force = self.need_force()
        vel = np.linalg.norm(self.c.get("vel"), axis=1)
        target = self.target_n if self.new_law else self.p.rest_n
        rest = covered & (force <= target)
        # The integrator coasts after release; zero drive does not mean zero
        # velocity on that same tick. Judge after six m/b time constants.
        if self.new_law:
            settled = np.zeros_like(rest)
            delay = self.ticks(6 * self.mass / self.damping)
            for start, stop in runs_of(rest, self.time_s, self.max_gap):
                settled[min(start + delay, stop):stop] = True
            rest = settled
        if not rest.any():
            return verdict("rest_equilibrium", SKIPPED, "no covered tick with |F| <= rest")
        still = vel[rest] < 0.5e-3
        still_frac = float(still.mean())
        detail = (f"settled |F| <= {target:g} N on {int(rest.sum())} ticks: |fc_vel| < 0.5 mm/s "
                  f"on {100 * still_frac:.2f} % (need >= 99 %)")
        ok = still_frac >= 0.99
        numbers = {"rest_ticks": int(rest.sum()), "still_fraction": still_frac}
        # Command drift only means something while nothing else moves the command:
        # the Hold source. Without a source column (pre-2026-09-15) fall back to the
        # legacy law name, and failing that judge every rest tick and say so.
        eligible, basis = rest, "all covered rest ticks"
        if self.c.has("source"):
            eligible = rest & (self.c.get("source") == "hold")
            basis = "source == hold"
        elif self.c.has("law"):
            eligible = rest & (self.c.get("law") == "hold")
            basis = "legacy fc_law == hold"
        if self.c.has("cmd") and eligible.any():
            cmd = self.c.get("cmd")
            stride = max(1, self.ticks(0.02))
            worst = 0.0
            for start, stop in runs_of(eligible, self.time_s, self.max_gap):
                points = cmd[start:stop:stride]
                window = max(1, int(round(5.0 / (self.dt * stride))))
                drift = sliding_range_norm(points, window)
                if len(drift):
                    worst = max(worst, float(drift.max()))
            detail += f"; command drift max {1e3 * worst:.2f} mm per 5 s window ({basis}; need < 1 mm)"
            numbers["command_drift_max_m"] = worst
            ok = ok and worst < 1e-3
        else:
            detail += "; command drift not judged (no hold-at-rest ticks or no tcp_command columns)"
        return verdict("rest_equilibrium", PASS if ok else FAIL, detail, **numbers)

    def check_yield_law(self):
        if self.new_law:
            return self.check_dynamic_yield_law()
        covered = self.need_covered()
        force = self.need_force()
        vel = self.c.get("vel")
        speed = np.linalg.norm(vel, axis=1)
        above = covered & (force > self.p.rest_n)
        if not above.any():
            return verdict("yield_law", SKIPPED, "no covered tick with |F| > rest")
        law_speed = (force[above] - self.p.rest_n) / self.p.b
        ratio = speed[above] / law_speed
        ratio_med = float(np.nanmedian(ratio))
        numbers = {"yield_ticks": int(above.sum()), "speed_ratio_median": ratio_med}
        detail = (f"|F| > rest on {int(above.sum())} ticks: median |fc_vel| / ((|F|-rest)/b) = "
                  f"{ratio_med:.3f} (need 0.75..1.25)")
        ok = 0.75 <= ratio_med <= 1.25
        if self.force_dir is not None:
            with np.errstate(invalid="ignore", divide="ignore"):
                unit_v = vel[above] / np.where(speed[above] > 0, speed[above], np.nan)[:, None]
            cos = np.einsum("ij,ij->i", unit_v, self.force_dir[above])
            cos = cos[np.isfinite(cos)]
            if len(cos):
                cos_med = float(np.median(cos))
                numbers["cos_median"] = cos_med
                detail += f"; median cos(fc_vel, F_hat) = {cos_med:.3f} (need > 0.95, direction from {self.dir_source})"
                ok = ok and cos_med > 0.95
            else:
                detail += "; direction not judged (no moving tick)"
        else:
            detail += "; direction not judged (no stand-frame force vector column)"
        return verdict("yield_law", PASS if ok else FAIL, detail, **numbers)

    def check_dynamic_yield_law(self):
        covered = self.need_covered()
        force = self.c.get("force_filt")
        vel = self.c.get("vel")
        mag = np.linalg.norm(force, axis=1)
        drive = force * (np.maximum(0., mag - self.target_n) / np.maximum(mag, 1e-30))[:, None]
        accel = (drive[1:] - self.damping * vel[:-1]) / self.mass
        norm = np.linalg.norm(accel, axis=1)
        accel *= np.minimum(1., self.p.accel_cap / np.maximum(norm, 1e-30))[:, None]
        predicted = vel[:-1] + self.p.control_dt * accel
        speed = np.linalg.norm(predicted, axis=1)
        predicted *= np.minimum(1., self.p.velocity_cap / np.maximum(speed, 1e-30))[:, None]
        # A log gap, force freeze or fence changes the state outside this law.
        valid = covered.copy()
        for role in ("bounded", "osc_frozen"):
            if self.c.has(role):
                valid &= self.c.get(role) < .5
        pair = valid[1:] & valid[:-1] & np.isclose(np.diff(self.time_s), self.p.control_dt, atol=1e-4)
        pair &= np.isfinite(force[1:]).all(axis=1) & np.isfinite(vel[1:]).all(axis=1)
        err = np.linalg.norm(vel[1:] - predicted, axis=1)[pair]
        if not len(err):
            return verdict("yield_law", SKIPPED, "no contiguous unfrozen law samples")
        p99, peak = float(np.percentile(err, 99)), float(np.max(err))
        return verdict("yield_law", PASS if peak < 2e-6 else FAIL,
                       f"one-step m*v_dot+b*v=max(|F|-{self.target_n:g},0)*F_hat residual: "
                       f"p99 {p99:.3g}, max {peak:.3g} m/s; need max < 2e-6 (CSV precision)",
                       dynamic_ticks=len(err), p99_residual_m_s=p99, max_residual_m_s=peak,
                       target_n=self.target_n, mass_kg=self.mass, damping_n_s_m=self.damping)

    def check_hold_source(self):
        covered = self.need_covered()
        source = self.c.get("source")
        demand = self.c.get("demand")
        gate = self.c.get("gate")
        hold = covered & (source == "hold")
        if not hold.any():
            return verdict("hold_source", SKIPPED, "no covered tick with source == hold")
        zero_demand = np.abs(demand[hold]) <= 1e-9
        open_gate = gate[hold] >= 0.999
        numbers = {"hold_ticks": int(hold.sum()),
                   "zero_demand_fraction": float(zero_demand.mean()),
                   "gate_open_fraction": float(open_gate.mean()),
                   "demand_max_m_s": float(np.nanmax(np.abs(demand[hold]))),
                   "gate_min": float(np.nanmin(gate[hold]))}
        ok = bool(zero_demand.all())
        detail = (f"source == hold on {int(hold.sum())} ticks: demand == 0 on "
                  f"{100 * numbers['zero_demand_fraction']:.2f} % (max {1e3 * numbers['demand_max_m_s']:.3f} mm/s), "
                  f"gate open on {100 * numbers['gate_open_fraction']:.2f} % (min {numbers['gate_min']:.3f}); gate may close at zero demand")
        if self.c.has("fold_sink") and self.force_n is not None:
            pushed = hold & (self.force_n > self.p.rest_n)
            if pushed.any():
                sink = self.c.get("fold_sink")
                frac = float((sink[pushed] == "hold").mean())
                numbers["fold_sink_hold_fraction"] = frac
                detail += f"; fold_sink == hold on {100 * frac:.1f} % of {int(pushed.sum())} pushed ticks (need >= 95 %)"
                ok = ok and frac >= 0.95
            else:
                detail += "; fold sink not judged (no pushed hold tick)"
        return verdict("hold_source", PASS if ok else FAIL, detail, **numbers)

    def check_floor_episodes(self):
        if self.new_law:
            force = self.need_force()
            covered = self.need_covered()
            return verdict("floor_episodes", SKIPPED,
                           "net force alone cannot identify floor contact; operator-labelled contact interval required",
                           max_covered_net_force_n=float(np.max(force[covered])) if covered.any() else 0,
                           above_target_ticks=int((covered & (force > self.target_n)).sum()))
        covered = self.need_covered()
        force = self.need_force()
        contact = covered & (force > self.p.rest_n)
        min_ticks = self.ticks(0.5)
        episodes = [(a, b) for a, b in runs_of(contact, self.time_s, self.max_gap) if b - a >= min_ticks]
        if not episodes:
            return verdict("floor_episodes", SKIPPED, "no contact episode (|F| > rest) lasting >= 0.5 s")
        source = self.c.get("source") if self.c.has("source") else None
        rows, ok = [], True
        for a, b in episodes:
            tail = force[a + min_ticks:b]
            above = float((tail > self.p.peak_n).mean()) if len(tail) else 0.0
            last = force[max(a, b - self.ticks(1.0)):b]
            mean_last = float(last.mean())
            lo, hi = self.p.rest_n - 1.0, self.p.peak_n - 0.5
            good = above <= 0.05 and lo <= mean_last <= hi
            ok = ok and good
            who = "?"
            if source is not None:
                values, counts = np.unique(source[a:b].astype(str), return_counts=True)
                who = str(values[np.argmax(counts)])
            rows.append({"start_s": float(self.time_s[a]), "duration_s": float(self.time_s[b - 1] - self.time_s[a]),
                         "above_peak_fraction_after_0p5s": above, "last_second_mean_n": mean_last,
                         "peak_n": float(force[a:b].max()), "source": who, "ok": good})
        worst_above = max(r["above_peak_fraction_after_0p5s"] for r in rows)
        detail = (f"{len(rows)} episode(s) >= 0.5 s; worst above-peak fraction after 0.5 s "
                  f"{100 * worst_above:.1f} % (need <= 5 %); last-second means "
                  + ", ".join(f"{r['last_second_mean_n']:.1f}" for r in rows)
                  + f" N (need {self.p.rest_n - 1:g}..{self.p.peak_n - 0.5:g}); peaks "
                  + ", ".join(f"{r['peak_n']:.1f}" for r in rows) + " N")
        return verdict("floor_episodes", PASS if ok else FAIL, detail, episodes=rows)

    def check_gate_model(self):
        if self.new_law:
            return self.check_confidence_gate()
        covered = self.need_covered()
        gate = self.c.get("gate")
        demand = self.c.get("demand")
        force = self.c.get("gate_force") if self.c.has("gate_force") else self.need_force()
        model = gate_model(force, demand, self.p.peak_n, self.p.v_cross_mm_s * 1e-3)
        # Judged where the gate is ENGAGED (a demand the curve can cut, or a gate that
        # is cutting): over every covered tick a log that is mostly at rest would pass
        # with a trivial median of 0 whatever the gate did in contact.
        engaged = covered & ((demand > self.p.v_cross_mm_s * 1e-3) | (gate < 0.98))
        err = np.abs(gate[engaged] - model[engaged])
        err = err[np.isfinite(err)]
        if not len(err):
            return verdict("gate_model", PASS, "gate never engaged (no demand above v_cross, gate open): trivially g = 1",
                           engaged_ticks=0)
        med = float(np.median(err))
        detail = (f"median |gate - g(|F|, v_s)| = {med:.4f} over {len(err)} engaged ticks "
                  f"(need < 0.05; slew ignored); max {float(err.max()):.3f}")
        return verdict("gate_model", PASS if med < 0.05 else FAIL, detail,
                       median_abs_error=med, max_abs_error=float(err.max()), engaged_ticks=int(len(err)))

    def check_confidence_gate(self):
        covered = self.need_covered()
        force, gate = self.c.get("gate_force"), self.c.get("gate")
        physical, confidence = self.c.get("physical_gate"), self.c.get("confidence")
        smooth = lambda x: np.clip(x, 0., 1.)**2 * (3. - 2.*np.clip(x, 0., 1.))
        c = smooth((force - self.p.noise_low) / (self.p.noise_full - self.p.noise_low))
        desired = 1. - smooth(force / self.target_n)
        tau = np.where(desired[1:] < physical[:-1], self.p.close_tau, self.p.open_tau)
        expected_physical = physical[:-1] + np.minimum(self.p.control_dt / tau, 1.)*(desired[1:] - physical[:-1])
        pair = covered[1:] & covered[:-1] & np.isclose(np.diff(self.time_s), self.p.control_dt, atol=1e-4)
        errors = [np.abs(gate[covered] - (1. - c[covered]*(1. - physical[covered]))),
                  np.abs(confidence[covered] - c[covered]),
                  np.abs(physical[1:][pair] - expected_physical[pair])]
        err = np.concatenate(errors)
        if not len(err):
            return verdict("gate_model", SKIPPED, "no covered gate samples")
        peak = float(np.max(err))
        return verdict("gate_model", PASS if np.isfinite(peak) and peak < 2e-5 else FAIL,
                       f"confidence + physical-gate recurrence max residual {peak:.3g}; need < 2e-5",
                       max_abs_error=peak, noise_low_n=self.p.noise_low, noise_full_n=self.p.noise_full)

    def check_deadlock(self):
        covered = self.need_covered()
        force = self.need_force()
        gate = self.c.get("gate")
        speed = np.linalg.norm(self.c.get("vel"), axis=1)
        threshold = (self.target_n + self.damping * .5e-3) if self.new_law else self.p.peak_n
        stuck = covered & (gate < 0.05) & (speed < 0.5e-3) & (force > threshold)
        longest = 0.0
        for a, b in runs_of(stuck, self.time_s, self.max_gap):
            longest = max(longest, float(self.time_s[b - 1] - self.time_s[a]) + self.dt)
        detail = (f"longest window with gate < 0.05 and |fc_vel| < 0.5 mm/s and |F| > peak: "
                  f"{longest:.3f} s over {int(stuck.sum())} ticks (need <= 0.2 s)")
        return verdict("deadlock", PASS if longest <= 0.2 else FAIL, detail,
                       longest_s=longest, stuck_ticks=int(stuck.sum()))

    def check_chatter_gate_transitions(self):
        covered = self.need_covered()
        gate = self.c.get("gate")
        cut = (gate < 0.98) & covered
        transitions = np.concatenate([[False], cut[1:] != cut[:-1]]) & covered
        window = self.ticks(5.0)
        counts = sliding_count(transitions, window)
        span = min(5.0, self.dt * min(window, len(transitions)))
        worst = float(counts.max()) if len(counts) else 0.0
        rate = worst / span if span > 0 else 0.0
        detail = (f"max {int(worst)} gate-cut transitions in any {span:.1f} s window = "
                  f"{rate:.2f}/s (need < 2/s); {int(transitions.sum())} transitions total")
        return verdict("chatter_gate_transitions", PASS if rate < 2.0 else FAIL, detail,
                       max_transitions_per_s=rate, transitions_total=int(transitions.sum()))

    def check_chatter_contact_normal(self):
        covered = self.need_covered()
        force = self.need_force()
        normal = self.c.get("normal")
        norm = np.linalg.norm(normal, axis=1)
        valid = covered & (force > 5.0) & (norm > 0.5)
        pair = valid[1:] & valid[:-1]
        if not pair.any():
            return verdict("chatter_contact_normal", SKIPPED, "no consecutive ticks with |F| > 5 N and a normal")
        dots = np.einsum("ij,ij->i", normal[1:][pair], normal[:-1][pair]) / (norm[1:][pair] * norm[:-1][pair])
        angle = np.degrees(np.arccos(np.clip(dots, -1.0, 1.0)))
        med = float(np.median(angle))
        detail = (f"median per-tick contact-normal angle change {med:.3f} deg over {int(pair.sum())} "
                  f"ticks with |F| > 5 N (need < 2 deg); p99 {float(np.percentile(angle, 99)):.2f} deg")
        return verdict("chatter_contact_normal", PASS if med < 2.0 else FAIL, detail,
                       median_deg=med, p99_deg=float(np.percentile(angle, 99)))

    def check_global(self):
        if self.covered is None:
            raise Missing(self.c.map["covered"])
        parts, ok, numbers = [], True, {}
        # fault_latched == 0 everywhere (the whole log, not only covered ticks).
        if self.c.has("fault"):
            faults = int((self.c.get("fault") >= 0.5).sum())
            numbers["fault_ticks"] = faults
            parts.append(f"fault_latched != 0 on {faults} ticks (need 0)")
            ok = ok and faults == 0
        else:
            parts.append("fault_latched: column missing")
        # Sent-joint acceleration: judged on the stage's own (covered) ticks, whole-log
        # maximum reported beside it because InitMotion is not the force stage's doing.
        if self.c.has("accel") and self.covered is not None and self.covered.any():
            accel = np.abs(self.c.get("accel"))
            peak_cov = float(np.nanmax(accel[self.covered]))
            peak_all = float(np.nanmax(accel))
            numbers["q_sent_accel_max_covered_deg_s2"] = peak_cov
            numbers["q_sent_accel_max_all_deg_s2"] = peak_all
            parts.append(f"max |q_sent_accel| {peak_cov:.0f} deg/s2 on covered ticks (need < 1500; whole log {peak_all:.0f})")
            ok = ok and peak_cov < 1500.0
        else:
            parts.append("q_sent_accel: column missing or nothing covered")
        if self.c.has("bounded") and self.covered is not None:
            if self.c.has("source"):
                source = self.c.get("source")
                fold_paths = self.covered & ((source == "hold") | (source == "chunk_follower"))
                hits = int((self.c.get("bounded")[fold_paths] >= 0.5).sum())
                numbers["bounded_ticks_on_fold_paths"] = hits
                parts.append(f"fc_bounded on {hits} of {int(fold_paths.sum())} hold/chunk_follower ticks (need 0)")
                ok = ok and hits == 0
            else:
                parts.append("fc_bounded on fold paths: fc_source column missing, not judged")
        else:
            parts.append("fc_bounded: column missing")
        status = PASS if ok else FAIL
        if not numbers:
            status = SKIPPED
        return verdict("global", status, "; ".join(parts), **numbers)

    def run(self):
        checks = [self.check_rest_equilibrium, self.check_yield_law, self.check_hold_source,
                  self.check_floor_episodes, self.check_gate_model, self.check_deadlock,
                  self.check_chatter_gate_transitions, self.check_chatter_contact_normal, self.check_global]
        results = []
        for check in checks:
            name = check.__name__[len("check_"):]
            try:
                results.append(check())
            except Missing as missing:
                results.append(verdict(name, SKIPPED, f"missing: {missing}"))
        covered_ticks = int(self.covered.sum()) if self.covered is not None else 0
        return {
            "arm": self.arm,
            "law_schema": "single_target" if self.new_law else "legacy_pair",
            "effective_target_n": self.target_n,
            "effective_mass_kg": self.mass,
            "effective_damping_n_s_m": self.damping,
            "ticks": int(len(self.time_s)),
            "covered_ticks": covered_ticks,
            "duration_s": float(self.time_s[-1]) if len(self.time_s) else 0.0,
            "dt_s": self.dt,
            "force_magnitude_source": self.force_source,
            "force_direction_source": self.dir_source,
            "checks": results,
        }


class Params:
    def __init__(self, rest_n=20., peak_n=24., b=500., v_cross_mm_s=8., *,
                 target_n=None, mass=20., noise_low=2., noise_full=3., close_tau=.1, open_tau=.4,
                 control_dt=.002, accel_cap=5., velocity_cap=.5):
        self.rest_n, self.peak_n, self.b, self.v_cross_mm_s = rest_n, peak_n, b, v_cross_mm_s
        self.target_n, self.mass = target_n, mass
        self.noise_low, self.noise_full = noise_low, noise_full
        self.close_tau, self.open_tau, self.control_dt = close_tau, open_tau, control_dt
        self.accel_cap, self.velocity_cap = accel_cap, velocity_cap
        values = (rest_n, peak_n, b, v_cross_mm_s, mass, noise_low, noise_full,
                  close_tau, open_tau, control_dt, accel_cap, velocity_cap)
        if not all(math.isfinite(x) and x > 0 for x in values) or noise_low >= noise_full:
            raise ValueError("finite positive law/gate parameters and noise_low < noise_full required")
        if target_n is not None and (not math.isfinite(target_n) or target_n <= noise_full):
            raise ValueError("target_n must exceed noise_full")


def analyze(path, arms, params):
    arrays, header, malformed = read_log(path, arms)
    if "loop_start_time_ns" not in arrays or len(arrays["loop_start_time_ns"]) == 0:
        raise SystemExit("no loop_start_time_ns column or no rows in " + str(path))
    report = {"schema": SCHEMA, "log": str(path), "malformed_rows": malformed,
              "params": vars(params).copy(),
              "arms": [ArmAudit(arrays, arm, params).run() for arm in arms]}
    return report


def format_report(report):
    lines = [f"{report['log']} (malformed rows {report['malformed_rows']}; log invariant audit, physical acceptance not implied)"]
    for arm in report["arms"]:
        lines.append(f"== {arm['arm']} [{arm['law_schema']}, target {arm['effective_target_n']:g} N]: {arm['covered_ticks']} covered of {arm['ticks']} ticks, "
                     f"{arm['duration_s']:.1f} s, dt {1e3 * arm['dt_s']:.2f} ms, |F| from {arm['force_magnitude_source']}, "
                     f"F_hat from {arm['force_direction_source']} ==")
        for check in arm["checks"]:
            lines.append(f"  [{check['status']:7s}] {check['check']:24s} {check['detail']}")
        counts = {s: sum(1 for c in arm["checks"] if c["status"] == s) for s in (PASS, FAIL, SKIPPED)}
        lines.append(f"  -> {counts[PASS]} PASS, {counts[FAIL]} FAIL, {counts[SKIPPED]} SKIPPED")
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log", help="servo_log CSV")
    parser.add_argument("--arm", choices=("left", "right", "both"), default="both")
    parser.add_argument("--rest-n", type=float, default=20.0, help="force_gate.rest_force_n (stack_real.yaml since 2026-09-15 pm; 10.0 before)")
    parser.add_argument("--peak-n", type=float, default=24.0, help="force_gate.peak_force_n (24.0 since 2026-09-15 pm; 12.0 before)")
    parser.add_argument("--b", type=float, default=500.0, help="explicit damping N*s/m; new telemetry overrides")
    parser.add_argument("--v-cross-mm-s", type=float, default=8.0, help="force_gate.peak_vel_mm_s (8.0 since 2026-09-15 pm; 4.0 before)")
    parser.add_argument("--target-n", type=float, help="single-target law (new logs auto-detect target telemetry)")
    parser.add_argument("--mass", type=float, default=20., help="law mass kg; new telemetry overrides")
    parser.add_argument("--noise-low", type=float, default=2.)
    parser.add_argument("--noise-full", type=float, default=3.)
    parser.add_argument("--control-dt", type=float, default=.002)
    parser.add_argument("--close-tau", type=float, default=.1)
    parser.add_argument("--open-tau", type=float, default=.4)
    parser.add_argument("--accel-cap", type=float, default=5.)
    parser.add_argument("--velocity-cap", type=float, default=.5)
    parser.add_argument("--json", type=Path, help="write the full report here")
    args = parser.parse_args(argv)
    arms = ARMS if args.arm == "both" else (args.arm,)
    report = analyze(args.log, arms, Params(args.rest_n, args.peak_n, args.b, args.v_cross_mm_s, target_n=args.target_n, mass=args.mass, noise_low=args.noise_low, noise_full=args.noise_full, control_dt=args.control_dt, close_tau=args.close_tau, open_tau=args.open_tau, accel_cap=args.accel_cap, velocity_cap=args.velocity_cap))
    print(format_report(report))
    if args.json:
        args.json.write_text(json.dumps(report, indent=2))
    failed = any(c["status"] == FAIL for arm in report["arms"] for c in arm["checks"])
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
