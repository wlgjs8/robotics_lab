#!/usr/bin/env python3
"""Plot GT vs prediction from eval_viz_dump.py output (run LOCALLY; needs matplotlib+ffmpeg).

Per visualized episode produces:
  - <ep>_traj.png : static 3D, GT demonstration gripper path vs the model's OPEN-LOOP
    rollout path (chained 1-step predictions, teacher-forced images), left+right arms.
  - <ep>_rollout.mp4 : the same, animated in time (real-time fps), both paths growing.
Plus per_dim_rmse.png (per-dimension action RMSE, translation/rotation/gripper panels).

Usage: python3 scripts/eval_plot.py --viz _eval_viz_best_viz.json --out-dir eval_plots
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

ARMS = [("left", "tab:blue", "left_path", "rollout_left"),
        ("right", "tab:green", "right_path", "rollout_right")]


def _set_equal(ax, pts):
    pts = np.asarray(pts)
    if not pts.size:
        return
    c = pts.mean(axis=0)
    r = max(1e-3, (pts.max(axis=0) - pts.min(axis=0)).max() / 2) * 1.05
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)


def _episode_arms(ep):
    out = []
    for label, color, pk, rk in ARMS:
        gt = np.asarray(ep.get(pk, []), float)
        roll = np.asarray(ep.get(rk, []), float)
        if gt.size and roll.size:
            out.append((label, color, gt, roll))
    return out


def static_plot(ep, out_path):
    arms = _episode_arms(ep)
    if not arms:
        return
    fig = plt.figure(figsize=(15, 7))
    for col, (label, color, gt, roll) in enumerate(arms):
        ax = fig.add_subplot(1, len(arms), col + 1, projection="3d")
        ax.plot(gt[:, 0], gt[:, 1], gt[:, 2], color=color, lw=2.0, alpha=0.9, label="GT demo")
        ax.plot(roll[:, 0], roll[:, 1], roll[:, 2], color="red", lw=2.0, ls="--", label="rollout (open-loop)")
        ax.scatter(*gt[0], color="black", s=40, zorder=6, label="start")
        ax.scatter(*gt[-1], color=color, s=40, marker="*", zorder=6)
        ax.scatter(*roll[-1], color="red", s=40, marker="*", zorder=6)
        _set_equal(ax, np.concatenate([gt, roll], axis=0))
        ax.view_init(elev=22, azim=-60)
        ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z"); ax.set_title(f"{label} arm")
        if col == 0:
            ax.legend(loc="upper left", fontsize=8)
    drift = np.mean([np.linalg.norm(r[-1] - g[-1]) for _, _, g, r in arms]) * 1000
    fig.suptitle(f"{ep['name']}  GT vs open-loop rollout  (endpoint drift ~{drift:.0f}mm)")
    fig.tight_layout(); fig.savefig(out_path, dpi=130); plt.close(fig)
    print("wrote", out_path)


def animate(ep, out_path, fps=30.0):
    arms = _episode_arms(ep)
    if not arms:
        return
    n = min(min(len(g), len(r)) for _, _, g, r in arms)
    fig = plt.figure(figsize=(16, 9))
    axes = []
    for col, (label, color, gt, roll) in enumerate(arms):
        ax = fig.add_subplot(1, len(arms), col + 1, projection="3d")
        axes.append((ax, label, color, gt[:n], roll[:n]))
    writer = FFMpegWriter(fps=fps, bitrate=8000, codec="libx264")
    with writer.saving(fig, str(out_path), dpi=120):
        for f in range(1, n + 1):
            for (ax, label, color, gt, roll) in axes:
                ax.cla()
                _set_equal(ax, np.concatenate([gt, roll], axis=0))
                ax.view_init(elev=22, azim=-60)
                ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
                ax.plot(gt[:, 0], gt[:, 1], gt[:, 2], color=color, lw=1.0, alpha=0.25)
                ax.plot(gt[:f, 0], gt[:f, 1], gt[:f, 2], color=color, lw=2.4, label="GT")
                ax.plot(roll[:f, 0], roll[:f, 1], roll[:f, 2], color="red", lw=2.4, ls="--", label="rollout")
                ax.scatter(*gt[f - 1], color=color, s=40, zorder=6)
                ax.scatter(*roll[f - 1], color="red", s=40, zorder=6)
                ax.set_title(f"{label} arm  t={f}/{n}")
                ax.legend(loc="upper left", fontsize=8)
            fig.suptitle(f"{ep['name']}  GT (solid) vs open-loop rollout (red dashed)  ~{fps:.0f}fps", fontsize=13)
            writer.grab_frame()
    plt.close(fig)
    print("wrote", out_path)


def per_dim_plot(doc, out_path):
    rmse = doc.get("per_dim_raw_rmse", {})
    panels = [("translation per-step RMSE (mm)", 1000.0,
               [("left_dx", "left_dy", "left_dz"), ("right_dx", "right_dy", "right_dz")]),
              ("rotation per-step RMSE (deg)", 180.0 / np.pi,
               [("left_drx", "left_dry", "left_drz"), ("right_drx", "right_dry", "right_drz")]),
              ("gripper per-step RMSE (units)", 1.0,
               [("left_grip",), ("right_grip",)])]
    fig, axes = plt.subplots(1, 3, figsize=(14, 4))
    for ax, (title, scale, groups) in zip(axes, panels):
        labels, vals, colors = [], [], []
        for color, grp in zip(["tab:blue", "tab:green"], groups):
            for nm in grp:
                labels.append(nm); vals.append((rmse.get(nm) or 0.0) * scale); colors.append(color)
        ax.bar(range(len(labels)), vals, color=colors)
        ax.set_xticks(range(len(labels))); ax.set_xticklabels(labels, rotation=45, ha="right", fontsize=8)
        ax.set_title(title, fontsize=10); ax.grid(axis="y", alpha=0.3)
    fig.suptitle(f"Per-dimension action RMSE — {doc.get('action_frame','?')} (validation)")
    fig.tight_layout(); fig.savefig(out_path, dpi=130); plt.close(fig)
    print("wrote", out_path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--viz", required=True)
    ap.add_argument("--out-dir", default="eval_plots")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--no-video", action="store_true")
    args = ap.parse_args()
    doc = json.loads(Path(args.viz).read_text())
    out = Path(args.out_dir); out.mkdir(parents=True, exist_ok=True)
    print(f"action_frame={doc.get('action_frame')}  episodes={len(doc['episodes'])}")
    for i, ep in enumerate(doc["episodes"]):
        safe = ep["name"].replace("/", "__")
        static_plot(ep, out / f"episode_{i:02d}_{safe}_traj.png")
        if not args.no_video:
            animate(ep, out / f"episode_{i:02d}_{safe}_rollout.mp4", fps=args.fps)
    per_dim_plot(doc, out / "per_dim_rmse.png")


if __name__ == "__main__":
    main()
