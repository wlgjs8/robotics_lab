#!/usr/bin/env python3
"""Compare multiple pgprofile campaign stages side by side.

Usage:
  python3 scripts/compare_pgprofile_stages.py \
      --stage ts1_vffx=outputs/tcp_pgprofile_ts1_vffx \
      --stage ts1_vffon=outputs/tcp_pgprofile_ts1_vffon \
      --stage ts1p5_vffx=outputs/tcp_pgprofile_ts1p5_vffx \
      --out outputs/tcp_pgprofile_comparison

Writes comparison.md + comparison.json + a class-histogram bar chart.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ALL_CLASSES = ["REAL_READY_TS_1P0", "SPEED_LIMITED", "TRACKING_LAG_HIGH",
               "SMOOTHNESS_HF_HIGH", "SELF_COLLISION_RISK", "IK_BRANCH_RISK",
               "IK_FAILURE", "SINGULARITY_RISK", "FLOOR_RISK", "WORKSPACE_ROI_RISK",
               "WATCHDOG_OR_LEASE_FAILURE", "DATA_QUALITY_BAD", "NEEDS_MANUAL_REVIEW"]


def _num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def _arm_vals(rec, group, key):
    out = []
    for a in ("left", "right"):
        n = _num(((rec.get(group) or {}).get(a) or {}).get(key))
        if n is not None:
            out.append(n)
    return out


def _pctl(xs, p):
    xs = sorted(x for x in xs if x is not None)
    if not xs:
        return None
    return xs[min(len(xs) - 1, int(round(p / 100 * (len(xs) - 1))))]


def load_stage(path: Path) -> dict[str, Any]:
    res = json.loads((path / "episode_results.json").read_text())
    eps = res.get("episodes", [])
    pos = [v for r in eps for v in _arm_vals(r, "B_ref_following", "pos_p95_mm")]
    lag = [v for r in eps for v in _arm_vals(r, "B_ref_following", "lag_ms")]
    span = [v for r in eps for v in _arm_vals(r, "B_ref_following", "span_ratio")]
    end = [v for r in eps for v in _arm_vals(r, "B_ref_following", "endpoint_err_mm")]
    solve = [v for r in eps for v in _arm_vals(r, "ik_safety", "ik_solve_us_p95")]
    return {
        "n": res.get("n_done", len(eps)),
        "hist": res.get("class_histogram", {}),
        "B_pos_p95_mm": {"p50": _pctl(pos, 50), "p95": _pctl(pos, 95), "max": _pctl(pos, 100)},
        "B_lag_ms": {"p50": _pctl(lag, 50), "p95": _pctl(lag, 95), "max": _pctl(lag, 100)},
        "B_span_ratio": {"p50": _pctl(span, 50), "p95": _pctl(span, 95)},
        "B_endpoint_mm": {"p50": _pctl(end, 50), "p95": _pctl(end, 95)},
        "ik_solve_p95_us": {"p50": _pctl(solve, 50), "p95": _pctl(solve, 95)},
    }


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--stage", action="append", required=True, help="name=dir")
    p.add_argument("--out", default="outputs/tcp_pgprofile_comparison")
    args = p.parse_args(argv)

    stages = {}
    for spec in args.stage:
        name, _, d = spec.partition("=")
        path = Path(d)
        if (path / "episode_results.json").exists():
            stages[name] = load_stage(path)
        else:
            print(f"skip {name}: no episode_results.json at {d}")
    if not stages:
        print("no stages loaded")
        return 2

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "comparison.json").write_text(json.dumps(stages, indent=2, default=lambda o: None) + "\n")

    names = list(stages)
    L = ["# pgprofile stage comparison", "",
         "| metric | " + " | ".join(names) + " |",
         "|---|" + "---|" * len(names)]

    def row(label, getter):
        return "| " + label + " | " + " | ".join(_fmt(getter(stages[n])) for n in names) + " |"

    L.append(row("episodes", lambda s: s["n"]))
    L.append(row("REAL_READY_TS_1P0", lambda s: s["hist"].get("REAL_READY_TS_1P0", 0)))
    L.append("")
    L.append("## class histogram")
    L.append("| class | " + " | ".join(names) + " |")
    L.append("|---|" + "---|" * len(names))
    for c in ALL_CLASSES:
        if any(stages[n]["hist"].get(c) for n in names):
            L.append("| " + c + " | " + " | ".join(str(stages[n]["hist"].get(c, 0)) for n in names) + " |")
    L.append("")
    L.append("## Tier B — controller reference following")
    for metric, key, sub in [
        ("ref-vs-cond pos p95 (mm) [p50/p95/max]", "B_pos_p95_mm", ("p50", "p95", "max")),
        ("tcp_ref lag (ms) [p50/p95/max]", "B_lag_ms", ("p50", "p95", "max")),
        ("span ratio [p50/p95]", "B_span_ratio", ("p50", "p95")),
        ("endpoint err (mm) [p50/p95]", "B_endpoint_mm", ("p50", "p95")),
        ("ik_solve p95 (us) [p50/p95]", "ik_solve_p95_us", ("p50", "p95")),
    ]:
        L.append(row(metric, lambda s, k=key, ss=sub: "/".join(_fmt(s[k].get(x)) for x in ss)))
    (out / "comparison.md").write_text("\n".join(L) + "\n")

    # bar chart of class histograms
    fig, ax = plt.subplots(figsize=(13, 6))
    classes = [c for c in ALL_CLASSES if any(stages[n]["hist"].get(c) for n in names)]
    x = np.arange(len(classes))
    w = 0.8 / len(names)
    for i, n in enumerate(names):
        ax.bar(x + i * w, [stages[n]["hist"].get(c, 0) for c in classes], w, label=n)
    ax.set_xticks(x + 0.4 - w / 2)
    ax.set_xticklabels(classes, rotation=40, ha="right", fontsize=8)
    ax.set_ylabel("episodes")
    ax.set_title("Primary class by stage")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out / "class_histogram_comparison.png", dpi=130)
    plt.close(fig)

    print(f"wrote {out}/comparison.md, comparison.json, class_histogram_comparison.png")
    print("REAL_READY per stage:", {n: stages[n]["hist"].get("REAL_READY_TS_1P0", 0) for n in names})
    return 0


def _fmt(v):
    if v is None:
        return "-"
    try:
        return f"{float(v):.3g}"
    except (TypeError, ValueError):
        return str(v)


if __name__ == "__main__":
    raise SystemExit(main())
