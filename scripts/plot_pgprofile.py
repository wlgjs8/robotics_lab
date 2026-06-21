#!/usr/bin/env python3
"""Phase-5 plots for the TcpTargetPose pgmode profiling campaign.

Reads the offline manifest (``episode_manifest.json``) and the live campaign
results (``episode_results.json``) and writes plots under
``outputs/tcp_pgprofile/current_smd/plots/``.

The campaign is fixed at time_scale=1.0, so the spec's "vs time_scale" axis
collapses to a single operating point. Where a time_scale sweep is requested we
instead plot the *offline* ``required_time_scale_estimate`` distribution (the
speed precheck answer to "what time_scale would each episode need"), which is the
meaningful, data-grounded substitute. This is labelled on every such plot.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent

TS_BUCKETS = [1.0, 1.25, 1.5, 2.0]


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text()) if path.exists() else {}


def _save(fig, out: Path) -> None:
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


def plot_class_histogram(results: dict, out_dir: Path) -> None:
    hist = results.get("class_histogram", {})
    if not hist:
        return
    items = sorted(hist.items(), key=lambda kv: -kv[1])
    labels = [k for k, _ in items]
    vals = [v for _, v in items]
    fig, ax = plt.subplots(figsize=(10, 5))
    colors = ["#2a9d8f" if l.startswith("REAL_READY") else "#e76f51" for l in labels]
    ax.bar(range(len(labels)), vals, color=colors)
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels, rotation=40, ha="right", fontsize=8)
    ax.set_ylabel("episodes")
    ax.set_title(f"Primary class histogram @ scale=1.0 ts=1.0 (n={sum(vals)})")
    for i, v in enumerate(vals):
        ax.text(i, v, str(v), ha="center", va="bottom", fontsize=8)
    _save(fig, out_dir / "class_histogram.png")


def plot_speed_margin(manifest: dict, out_dir: Path) -> None:
    eps = manifest.get("episodes", [])
    lin = sorted(float(e.get("linear_speed_margin", 0) or 0) for e in eps if e.get("linear_speed_margin") is not None)
    ang = sorted(float(e.get("angular_speed_margin", 0) or 0) for e in eps if e.get("angular_speed_margin") is not None)
    if not lin:
        return
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(range(len(lin)), lin, label="linear margin (peak/0.25 m/s)", color="#264653")
    ax.plot(range(len(ang)), ang, label="angular margin (peak/1.745 rad/s)", color="#e9c46a")
    ax.axhline(1.0, color="red", ls="--", lw=1, label="SMD cap")
    ax.set_xlabel("episode (sorted)")
    ax.set_ylabel("max_stream_speed / configured_limit")
    ax.set_title("Speed margin vs configured SMD limit (offline, ts=1.0)")
    ax.legend()
    _save(fig, out_dir / "speed_margin.png")


def plot_required_time_scale_cdf(manifest: dict, out_dir: Path) -> None:
    """Pass-rate vs time_scale: fraction within the SMD speed budget at each ts."""
    eps = manifest.get("episodes", [])
    req = np.array([float(e["required_time_scale_estimate"]) for e in eps
                    if e.get("required_time_scale_estimate") is not None])
    if req.size == 0:
        return
    xs = np.linspace(0.8, max(2.5, float(req.max())), 120)
    frac = np.array([(req <= x).mean() for x in xs]) * 100.0
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(xs, frac, color="#2a9d8f")
    for ts in TS_BUCKETS:
        f = (req <= ts).mean() * 100.0
        ax.axvline(ts, color="grey", ls=":", lw=0.8)
        ax.annotate(f"ts={ts}: {f:.0f}%", (ts, f), fontsize=8)
    ax.set_xlabel("time_scale (replay slowdown factor)")
    ax.set_ylabel("% episodes within SMD speed budget")
    ax.set_title("Speed-budget pass rate vs time_scale (offline precheck; campaign ran ts=1.0)")
    _save(fig, out_dir / "passrate_vs_time_scale.png")


def _collect_b(results: dict) -> dict[str, list[float]]:
    out = {"pos_p95_mm": [], "ori_p95_deg": [], "lag_ms": [], "span_ratio": [], "endpoint_mm": []}
    for r in results.get("episodes", []):
        b = r.get("B_ref_following") or {}
        for arm in ("left", "right"):
            a = b.get(arm) or {}
            for k in out:
                v = a.get(k)
                if v is not None:
                    out[k].append(float(v))
    return out


def _collect_ik(results: dict) -> dict[str, list[float]]:
    out = {"ik_solve_us_p95": [], "ik_branch_jump_count": [], "ik_min_singular_value_p05": []}
    for r in results.get("episodes", []):
        ik = r.get("ik_safety") or {}
        for arm in ("left", "right"):
            a = ik.get(arm) or {}
            for k in out:
                v = a.get(k)
                if v is not None:
                    out[k].append(float(v))
    return out


def plot_hist(values: list[float], title: str, xlabel: str, out: Path, vline: float | None = None) -> None:
    vals = [v for v in values if v is not None and np.isfinite(v)]
    if not vals:
        return
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.hist(vals, bins=40, color="#457b9d", edgecolor="white")
    if vline is not None:
        ax.axvline(vline, color="red", ls="--", lw=1, label=f"threshold {vline:g}")
        ax.legend()
    ax.set_xlabel(xlabel)
    ax.set_ylabel("arm-episodes")
    ax.set_title(f"{title} (n={len(vals)})")
    _save(fig, out)


def plot_safety_heatmap(results: dict, out_dir: Path) -> None:
    risks = ["IK_FAILURE", "IK_BRANCH_RISK", "SINGULARITY_RISK", "SELF_COLLISION_RISK",
             "FLOOR_RISK", "WORKSPACE_ROI_RISK", "WATCHDOG_OR_LEASE_FAILURE",
             "SPEED_LIMITED", "TRACKING_LAG_HIGH", "SMOOTHNESS_HF_HIGH"]
    eps = [r for r in results.get("episodes", []) if r.get("risk_flags") is not None]
    eps = sorted(eps, key=lambda r: r.get("stem", ""))
    if not eps:
        return
    mat = np.zeros((len(risks), len(eps)))
    for j, r in enumerate(eps):
        for rf in (r.get("risk_flags") or []):
            if rf in risks:
                mat[risks.index(rf), j] = 1
    fig, ax = plt.subplots(figsize=(min(20, max(8, len(eps) * 0.04)), 4.5))
    ax.imshow(mat, aspect="auto", cmap="Reds", interpolation="nearest")
    ax.set_yticks(range(len(risks)))
    ax.set_yticklabels(risks, fontsize=7)
    ax.set_xlabel(f"episode (sorted, n={len(eps)})")
    ax.set_title("Risk-flag heatmap: episode × risk (scale=1.0, ts=1.0)")
    _save(fig, out_dir / "safety_risk_heatmap.png")


def plot_branch_per_episode(results: dict, out_dir: Path) -> None:
    eps = sorted(results.get("episodes", []), key=lambda r: r.get("stem", ""))
    counts = []
    for r in eps:
        ik = r.get("ik_safety") or {}
        c = sum(int((ik.get(a) or {}).get("ik_branch_jump_count") or 0) for a in ("left", "right"))
        counts.append(c)
    if not counts or max(counts) == 0:
        # still emit an informative empty plot
        counts = counts or [0]
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.bar(range(len(counts)), counts, color="#6a4c93")
    ax.set_xlabel("episode (sorted)")
    ax.set_ylabel("branch-jump ticks (L+R)")
    ax.set_title(f"Branch-jump count per episode (max={max(counts)})")
    _save(fig, out_dir / "branch_jump_per_episode.png")


def plot_episode_timeline(log_path: Path, out: Path) -> None:
    sys.path.insert(0, str(REPO_ROOT / "scripts"))
    sys.path.insert(0, str(REPO_ROOT / "tools"))
    import analyze_pgprofile_run as ana
    arms = ana.load_arms(log_path)
    fig, axes = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    for arm, color in (("left", "#1d3557"), ("right", "#e63946")):
        d = arms.get(arm)
        if not d:
            continue
        t = d["t"]
        cg = d.get("conditioned_goal_after_A")
        ref = d.get("reference_after_B")
        if cg is not None and ref is not None:
            err = np.linalg.norm(cg[:, :3] - ref[:, :3], axis=1) * 1000.0
            axes[0].plot(t, err, color=color, lw=0.7, label=arm)
        axes[1].plot(t, d["ik_solve_us"], color=color, lw=0.7, label=arm)
        axes[2].plot(t, d["ik_min_singular_value"], color=color, lw=0.7, label=arm)
        axes[3].plot(t, d["self_collision_flag"].astype(float) * 1.0, color=color, lw=0.8, label=f"{arm} selfcol")
        axes[3].plot(t, d["roi_flag"].astype(float) * 1.05, color=color, lw=0.8, ls=":", label=f"{arm} roi")
        axes[3].plot(t, d["branch_jump_flag"].astype(float) * 0.95, color=color, lw=0.8, ls="--", label=f"{arm} branch")
    axes[0].set_ylabel("ref-vs-goal\npos err (mm)"); axes[0].axhline(10, color="red", ls="--", lw=0.8)
    axes[1].set_ylabel("ik_solve_us"); axes[1].axhline(2000, color="red", ls="--", lw=0.8)
    axes[2].set_ylabel("min singular val"); axes[2].axhline(0.06, color="red", ls="--", lw=0.8)
    axes[3].set_ylabel("safety flags"); axes[3].set_xlabel("t (s)")
    for ax in axes:
        ax.legend(fontsize=6, ncol=2)
    axes[0].set_title(f"timeline: {log_path.parent.parent.parent.name}")
    _save(fig, out)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--pgprofile-dir", default="outputs/tcp_pgprofile")
    p.add_argument("--max-timelines", type=int, default=12)
    args = p.parse_args(argv)

    base = Path(args.pgprofile_dir)
    out_dir = base / "current_smd" / "plots"
    out_dir.mkdir(parents=True, exist_ok=True)
    manifest = _load(base / "episode_manifest.json")
    results = _load(base / "episode_results.json")

    plot_class_histogram(results, out_dir)
    plot_speed_margin(manifest, out_dir)
    plot_required_time_scale_cdf(manifest, out_dir)
    plot_safety_heatmap(results, out_dir)
    plot_branch_per_episode(results, out_dir)

    b = _collect_b(results)
    plot_hist(b["pos_p95_mm"], "B: reference_after_B vs conditioned_goal position p95",
              "pos p95 (mm)", out_dir / "B_position_p95_dist.png", vline=10.0)
    plot_hist(b["ori_p95_deg"], "B: reference vs conditioned orientation p95",
              "ori p95 (deg)", out_dir / "B_orientation_p95_dist.png", vline=2.0)
    plot_hist(b["lag_ms"], "B: tcp_ref lag vs conditioned_goal", "lag (ms)",
              out_dir / "B_lag_dist.png", vline=150.0)
    plot_hist(b["span_ratio"], "B: reference span ratio", "span ratio",
              out_dir / "B_span_ratio_dist.png", vline=1.0)
    ik = _collect_ik(results)
    plot_hist(ik["ik_solve_us_p95"], "IK solve p95", "us", out_dir / "ik_solve_p95_dist.png", vline=1000.0)
    plot_hist(ik["ik_min_singular_value_p05"], "IK min singular value p05", "sigma",
              out_dir / "ik_min_singular_p05_dist.png", vline=0.06)

    # conditioned_goal >5Hz power (A-tier). actual_tcp HF is N/A (frozen) — note in report.
    hf = []
    for r in results.get("episodes", []):
        rd = r.get("run_dir")
        if not rd:
            continue
        rj = Path(rd) / "pgprofile_result.json"
        if rj.exists():
            try:
                res = json.loads(rj.read_text())
                for arm in ("left", "right"):
                    v = (res.get("goal_conditioning_quality_A", {}).get(arm, {})
                         .get("conditioned_goal_high_frequency_power_above_5hz"))
                    if v is not None:
                        hf.append(float(v))
            except (OSError, json.JSONDecodeError):
                pass
    plot_hist(hf, "A: conditioned_goal >5Hz linear-velocity power", "power",
              out_dir / "conditioned_goal_hf_power_dist.png")

    # per-episode timelines for failure episodes
    fail_eps = [r for r in results.get("episodes", [])
                if r.get("primary_class") and not r["primary_class"].startswith("REAL_READY")
                and r.get("run_dir")]
    fail_eps = fail_eps[: args.max_timelines]
    tdir = out_dir / "timelines"
    for r in fail_eps:
        log_path = Path(r["run_dir"]) / "log.csv"
        if log_path.exists():
            try:
                plot_episode_timeline(log_path, tdir / f"{r['stem']}_{r['primary_class']}.png")
            except Exception as exc:  # noqa: BLE001
                print(f"timeline failed for {r['stem']}: {exc}")

    print(f"wrote plots under {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
