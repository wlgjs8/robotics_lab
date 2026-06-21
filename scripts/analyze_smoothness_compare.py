#!/usr/bin/env python3
"""Compare reference-trajectory SMOOTHNESS (vibration / wrist-blur axis) across the
three pgprofile conditions, for the shared clean 22-episode subset.

Accuracy (pos error / lag) was compared elsewhere. Here we quantify what drives
robot vibration and wrist-camera motion blur: the high-frequency content and jerk
of the controller reference (reference_after_B = tcp_ref_stand) that the physical
arm would track. q_actual is frozen in pgmode, so the reference is the best
available proxy for executed motion.

For each condition we aggregate, over 22 episodes x {left,right}, on reference_after_B:
  - linear / angular velocity power > 5 Hz   (blur band)
  - linear / angular jerk RMS & p95          (vibration)
  - linear velocity sign-reversals / sec     (chatter)
Also reports the same for conditioned_goal_after_A (the identical input) as a baseline.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
for p in (str(REPO_ROOT), str(REPO_ROOT / "tools"), str(REPO_ROOT / "scripts")):
    if p not in sys.path:
        sys.path.insert(0, p)

import analyze_pgprofile_run as ana  # noqa: E402
from tcp_tuning.config import load_config  # noqa: E402
from tcp_tuning.metrics import smoothness_metrics  # noqa: E402

CONDITIONS = {
    "baseline_ts1_vffx": "outputs/tcp_pgprofile_ts1_vffx",
    "exp1_ts1_vffon": "outputs/tcp_pgprofile_ts1_vffon",
    "exp2_ts1p5_vffx": "outputs/tcp_pgprofile_ts1p5_vffx",
}


def latest_log(cond_dir: Path, eid: str) -> Path | None:
    runs = cond_dir / eid / "runs"
    if not runs.exists():
        return None
    logs = sorted(runs.glob("batch_*/log.csv"))
    return logs[-1] if logs else None


def nested(d, *keys):
    for k in keys:
        if not isinstance(d, dict):
            return None
        d = d.get(k)
    return d


def main() -> int:
    cfg = load_config(None).metrics
    subset = Path("/tmp/clean_subset.txt").read_text().strip().split(",")
    eid_pre = "replay_profiling_20260620__"

    agg: dict[str, dict[str, list[float]]] = {}
    for cond, d in CONDITIONS.items():
        cond_dir = Path(d)
        acc = {k: [] for k in (
            "ref_lin_hf5", "ref_ang_hf5", "ref_lin_jerk_rms", "ref_ang_jerk_rms",
            "ref_lin_sign_rev", "ref_ang_vel_p95",
            "cond_lin_hf5", "cond_ang_hf5", "cond_lin_jerk_rms")}
        n_ok = 0
        for stem in subset:
            eid = eid_pre + stem
            log = latest_log(cond_dir, eid)
            if log is None:
                continue
            arms = ana.load_arms(log)
            for arm, ad in arms.items():
                t = ad["t"]
                ref = ad.get("reference_after_B")
                cond_goal = ad.get("conditioned_goal_after_A")
                if ref is not None:
                    sm = smoothness_metrics(t, ref, cfg=cfg, policy_rate_hz=cfg.chunk_rate_hz,
                                            metric_name="ref")
                    acc["ref_lin_hf5"].append(nested(sm, "linear_velocity_spectrum", "power_above_cutoff"))
                    acc["ref_ang_hf5"].append(nested(sm, "angular_velocity_spectrum", "power_above_cutoff"))
                    acc["ref_lin_jerk_rms"].append(nested(sm, "linear_jerk_m_s3", "rms"))
                    acc["ref_ang_jerk_rms"].append(nested(sm, "angular_jerk_rad_s3", "rms"))
                    acc["ref_lin_sign_rev"].append(nested(sm, "linear_velocity_sign_reversals_per_sec", "per_sec"))
                    acc["ref_ang_vel_p95"].append(nested(sm, "angular_velocity_rad_s", "p95"))
                if cond_goal is not None:
                    sm2 = smoothness_metrics(t, cond_goal, cfg=cfg, policy_rate_hz=cfg.chunk_rate_hz,
                                             metric_name="cond")
                    acc["cond_lin_hf5"].append(nested(sm2, "linear_velocity_spectrum", "power_above_cutoff"))
                    acc["cond_ang_hf5"].append(nested(sm2, "angular_velocity_spectrum", "power_above_cutoff"))
                    acc["cond_lin_jerk_rms"].append(nested(sm2, "linear_jerk_m_s3", "rms"))
            n_ok += 1
        agg[cond] = {"_n_episodes": n_ok, **acc}

    def med(xs):
        xs = [x for x in xs if x is not None and np.isfinite(x)]
        return float(np.median(xs)) if xs else None

    def p95(xs):
        xs = [x for x in xs if x is not None and np.isfinite(x)]
        return float(np.percentile(xs, 95)) if xs else None

    rows = [
        ("episodes", lambda a: a["_n_episodes"], None),
        ("REFERENCE smoothness (robot-executed proxy):", None, None),
        ("  ref linear vel >5Hz power (median)", lambda a: med(a["ref_lin_hf5"]), "blur"),
        ("  ref ANGULAR vel >5Hz power (median)", lambda a: med(a["ref_ang_hf5"]), "WRIST BLUR"),
        ("  ref linear jerk RMS (median)", lambda a: med(a["ref_lin_jerk_rms"]), "vibration"),
        ("  ref ANGULAR jerk RMS (median)", lambda a: med(a["ref_ang_jerk_rms"]), "wrist vibration"),
        ("  ref linear vel sign-reversals/s (median)", lambda a: med(a["ref_lin_sign_rev"]), "chatter"),
        ("  ref angular vel p95 (rad/s, median)", lambda a: med(a["ref_ang_vel_p95"]), "rotation speed"),
        ("INPUT conditioned_goal (identical target):", None, None),
        ("  cond linear vel >5Hz power (median)", lambda a: med(a["cond_lin_hf5"]), None),
        ("  cond angular vel >5Hz power (median)", lambda a: med(a["cond_ang_hf5"]), None),
        ("  cond linear jerk RMS (median)", lambda a: med(a["cond_lin_jerk_rms"]), None),
    ]
    names = list(CONDITIONS)
    print(f"{'metric':46s} " + " ".join(f"{n:>20s}" for n in names) + "   note")
    print("-" * (46 + 21 * len(names) + 12))
    for label, fn, note in rows:
        if fn is None:
            print(label)
            continue
        cells = []
        for n in names:
            v = fn(agg[n])
            cells.append(f"{v:>20.4g}" if isinstance(v, (int, float)) and v is not None else f"{str(v):>20s}")
        print(f"{label:46s} " + " ".join(cells) + (f"   <- {note}" if note else ""))

    out = Path("outputs/tcp_pgprofile_comparison/smoothness_compare.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    summ = {n: {k: (med(v) if isinstance(v, list) else v) for k, v in agg[n].items()} for n in names}
    out.write_text(json.dumps(summ, indent=2, default=lambda o: None) + "\n")
    print(f"\nwrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
