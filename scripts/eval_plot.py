#!/usr/bin/env python3
"""Plot GT-vs-prediction from eval_viz_dump.py output (run LOCALLY; needs matplotlib).

Produces, per visualized episode, a 3D figure with:
  - the full demonstration gripper path (left blue, right green)
  - per-anchor short horizon segments: GT (solid, dark) vs predicted (dashed, red/orange)
and one bar chart of per-dimension raw RMSE (14 action dims).

Usage:
  python3 scripts/eval_plot.py --viz _eval_viz_best_viz.json --out-dir eval_plots
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


def _plot_arm(ax, path, anchors, key_gt, key_pred, path_color, label):
    path = np.asarray(path)
    if path.size:
        ax.plot(path[:, 0], path[:, 1], path[:, 2], color=path_color, lw=1.0,
                alpha=0.45, label=f"{label} demo path")
    first = True
    for a in anchors:
        if key_gt not in a:
            continue
        gt = np.asarray(a[key_gt])
        pr = np.asarray(a[key_pred])
        ax.plot(gt[:, 0], gt[:, 1], gt[:, 2], color="black", lw=2.0,
                label="GT chunk" if first else None)
        ax.plot(pr[:, 0], pr[:, 1], pr[:, 2], color="red", lw=2.0, ls="--",
                label="pred chunk" if first else None)
        ax.scatter(*gt[0], color=path_color, s=18, zorder=5)
        first = False


def _set_equal(ax, pts):
    pts = np.asarray(pts)
    if not pts.size:
        return
    c = pts.mean(axis=0)
    r = max(1e-3, (pts.max(axis=0) - pts.min(axis=0)).max() / 2)
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r, c[2] + r)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--viz", required=True)
    ap.add_argument("--out-dir", default="eval_plots")
    args = ap.parse_args()
    doc = json.loads(Path(args.viz).read_text())
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)

    for i, ep in enumerate(doc["episodes"]):
        fig = plt.figure(figsize=(14, 7))
        allpts = []
        for col, spec in enumerate([
            ("left_path", "gt_left", "pred_left", "tab:blue", "left"),
            ("right_path", "gt_right", "pred_right", "tab:green", "right"),
        ]):
            path_key, key_gt, key_pred, color, label = spec
            ax = fig.add_subplot(1, 2, col + 1, projection="3d")
            _plot_arm(ax, ep.get(path_key, []), ep["anchors"], key_gt, key_pred, color, label)
            pts = list(ep.get(path_key, []))
            for a in ep["anchors"]:
                if key_pred in a:
                    pts += a[key_pred]
            _set_equal(ax, pts) if pts else None
            allpts += pts
            ax.set_title(f"{label} arm")
            ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
            ax.view_init(elev=22, azim=-60)
            if col == 0:
                ax.legend(loc="upper left", fontsize=7)
        fig.suptitle(f"{ep['name']}  (GT solid black vs pred dashed red; demo path faint)")
        fig.tight_layout()
        p = out / f"episode_{i:02d}_traj.png"
        fig.savefig(p, dpi=130)
        plt.close(fig)
        print("wrote", p)

    # per-dim RMSE: 3 panels with proper units (translation mm / rotation deg / gripper)
    rmse = doc.get("per_dim_raw_rmse", {})
    panels = [
        ("translation per-step RMSE (mm)", 1000.0,
         [("left_dx", "left_dy", "left_dz"), ("right_dx", "right_dy", "right_dz")]),
        ("rotation per-step RMSE (deg)", 180.0 / np.pi,
         [("left_drx", "left_dry", "left_drz"), ("right_drx", "right_dry", "right_drz")]),
        ("gripper per-step RMSE (units)", 1.0,
         [("left_grip",), ("right_grip",)]),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(14, 4))
    for ax, (title, scale, groups) in zip(axes, panels):
        labels, vals, colors = [], [], []
        for color, grp in zip(["tab:blue", "tab:green"], groups):
            for n in grp:
                labels.append(n)
                vals.append((rmse.get(n) or 0.0) * scale)
                colors.append(color)
        ax.bar(range(len(labels)), vals, color=colors)
        ax.set_xticks(range(len(labels)))
        ax.set_xticklabels(labels, rotation=45, ha="right", fontsize=8)
        ax.set_title(title, fontsize=10)
        ax.grid(axis="y", alpha=0.3)
    fig.suptitle("Per-dimension action RMSE — best model, validation (GT vs predicted)")
    fig.tight_layout()
    p = out / "per_dim_rmse.png"
    fig.savefig(p, dpi=130)
    plt.close(fig)
    print("wrote", p)


if __name__ == "__main__":
    main()
