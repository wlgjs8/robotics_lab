#!/usr/bin/env python3
"""Render high-res MP4 trajectory animations from eval_anim_dump.py output (run LOCALLY).

Per validation episode, plays through the episode in time (real-time = recorded fps):
  - faint full demonstration path (context, both arms)
  - growing "traveled" path up to the current timestep
  - current gripper marker
  - the model's predicted next action-chunk (red dashed) vs the GT next chunk (black)
Left and right arms are shown as two 3D subplots.

Usage:
  python3 scripts/eval_animate.py --npz anim.npz --out-dir eval_videos --dpi 120
Needs matplotlib + ffmpeg (matplotlib 'ffmpeg' writer).
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


def _limits(ax, pts):
    pts = np.asarray(pts)
    c = pts.mean(axis=0)
    r = max(1e-3, (pts.max(axis=0) - pts.min(axis=0)).max() / 2) * 1.1
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r, c[2] + r)


def animate_episode(z, ei, meta_ep, out_path, dpi, fps_override=None):
    horizon = None
    ts = z[f"ep{ei}_t"]
    amask = meta_ep["arm_mask"]
    arms = []
    if amask[0] > 0:
        arms.append(("left", "tab:blue", z[f"ep{ei}_left_path"], z[f"ep{ei}_predL"],
                     z[f"ep{ei}_gtL"], z[f"ep{ei}_gripL"]))
    if amask[1] > 0:
        arms.append(("right", "tab:green", z[f"ep{ei}_right_path"], z[f"ep{ei}_predR"],
                     z[f"ep{ei}_gtR"], z[f"ep{ei}_gripR"]))
    if not arms:
        return None

    fig = plt.figure(figsize=(16, 9))
    axes = []
    for col, (label, color, path, pred, gt, grip) in enumerate(arms):
        ax = fig.add_subplot(1, len(arms), col + 1, projection="3d")
        allpts = np.concatenate([path, pred.reshape(-1, 3)], axis=0)
        _limits(ax, allpts)
        ax.view_init(elev=22, azim=-60)
        ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
        ax.set_title(f"{label} arm")
        axes.append((ax, label, color, path, pred, gt, grip))

    fps = float(fps_override or meta_ep.get("fps", 30.0))
    writer = FFMpegWriter(fps=fps, bitrate=8000, codec="libx264")
    n_frames = len(ts)
    with writer.saving(fig, str(out_path), dpi=dpi):
        for f in range(n_frames):
            t = int(ts[f])
            for (ax, label, color, path, pred, gt, grip) in axes:
                # clear only the dynamic artists by redrawing the axis content
                ax.cla()
                allpts = np.concatenate([path, pred.reshape(-1, 3)], axis=0)
                _limits(ax, allpts)
                ax.view_init(elev=22, azim=-60)
                ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
                ax.plot(path[:, 0], path[:, 1], path[:, 2], color=color, lw=1.0,
                        alpha=0.30, label="demo path")
                trav = path[:t + 1]
                if len(trav) > 1:
                    ax.plot(trav[:, 0], trav[:, 1], trav[:, 2], color=color, lw=2.2,
                            alpha=0.9, label="traveled")
                g = gt[f]; p = pred[f]
                ax.plot(g[:, 0], g[:, 1], g[:, 2], color="black", lw=2.6, label="GT chunk")
                ax.plot(p[:, 0], p[:, 1], p[:, 2], color="red", lw=2.6, ls="--", label="pred chunk")
                ax.scatter(*g[0], color=color, s=45, zorder=6)
                gval = float(grip[t]) if t < len(grip) else float("nan")
                ax.set_title(f"{label} arm   t={t}/{meta_ep['length']}   grip={gval:.1f}")
                ax.legend(loc="upper left", fontsize=8)
            fig.suptitle(f"{meta_ep['name']}   (GT black vs pred red dashed; ~{fps:.0f} fps real-time)",
                         fontsize=13)
            writer.grab_frame()
    plt.close(fig)
    return out_path


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--npz", required=True)
    ap.add_argument("--out-dir", default="eval_videos")
    ap.add_argument("--dpi", type=int, default=120)  # 16x9 @120 = 1920x1080
    ap.add_argument("--fps", type=float, default=None, help="override real-time fps")
    ap.add_argument("--save-frame", action="store_true",
                    help="also dump a mid PNG per episode for quick verification")
    args = ap.parse_args()

    z = np.load(args.npz)
    meta = json.loads(Path(args.npz + ".meta.json").read_text())
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    for ei, mep in enumerate(meta["episodes"]):
        safe = mep["name"].replace("/", "__")
        mp4 = out / f"episode_{ei:02d}_{safe}.mp4"
        animate_episode(z, ei, mep, mp4, args.dpi, args.fps)
        print("wrote", mp4)


if __name__ == "__main__":
    main()
