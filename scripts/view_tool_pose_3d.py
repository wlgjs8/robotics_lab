#!/usr/bin/env python3
"""3D tool-pose viewer for converted data_tcp episodes (matplotlib).

Plots observations/tcp_stand_{left,right} (the retargeted tool / device-TCP pose)
as a 3D trajectory with orientation triads (tool x/y/z axes) sampled along the
path, start/end markers, and gripper-state coloring.

  # interactive window
  python3 scripts/view_tool_pose_3d.py data_tcp/<sess>/episode_000.hdf5 --arm both
  # save a PNG (headless)
  python3 scripts/view_tool_pose_3d.py <ep.hdf5> --arm right --out /tmp/tool3d.png

Triad colors: red=tool x, green=tool y, blue=tool z (= approach axis).
"""
import argparse
import numpy as np
import h5py


def quat_to_R(q):  # xyzw -> 3x3
    x, y, z, w = np.asarray(q, float) / (np.linalg.norm(q) + 1e-12)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def read_arm(f, arm):
    key = f"observations/tcp_stand_{arm}"
    if key not in f:
        return None, None
    P = np.asarray(f[key], float)                       # (T,7) x,y,z,qx,qy,qz,qw
    g = None
    gk = f"observations/gripper_{arm}"
    if gk in f:
        g = np.asarray(f[gk], float)
        g = g[:, 0] if g.ndim == 2 else g.reshape(-1)   # (T,) percent (open~98 closed~16)
    return P, g


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("episode", help="data_tcp episode .hdf5")
    ap.add_argument("--arm", choices=["left", "right", "both"], default="both")
    ap.add_argument("--stride", type=int, default=15, help="draw an orientation triad every N frames")
    ap.add_argument("--axis-len", type=float, default=0.03, help="triad arrow length (m)")
    ap.add_argument("--out", default=None, help="save PNG here; omit for an interactive window")
    args = ap.parse_args()

    import matplotlib
    if args.out:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    arms = ["left", "right"] if args.arm == "both" else [args.arm]
    line_color = {"left": "tab:blue", "right": "tab:orange"}

    fig = plt.figure(figsize=(11, 9))
    ax = fig.add_subplot(111, projection="3d")
    pts_all = []

    with h5py.File(args.episode, "r") as f:
        attrs = {k: (v.decode() if isinstance(v, bytes) else v) for k, v in f.attrs.items()}
        sub = {k: attrs[k] for k in ("retarget_status", "retarget_config_hash", "frame_count") if k in attrs}
        print("episode attrs:", sub)
        for arm in arms:
            P, g = read_arm(f, arm)
            if P is None:
                print(f"  (no tcp_stand_{arm})")
                continue
            xyz = P[:, :3]
            pts_all.append(xyz)
            if g is not None and len(g) == len(xyz):
                sccol = ax.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], c=g, cmap="viridis",
                                   s=7, label=f"{arm}  (color=gripper %)")
            else:
                ax.plot(xyz[:, 0], xyz[:, 1], xyz[:, 2], color=line_color[arm], lw=1.2, label=arm)
            # orientation triads (tool axes)
            for i in range(0, len(P), max(1, args.stride)):
                R = quat_to_R(P[i, 3:7])
                o = P[i, :3]
                for k, c in zip(range(3), ("r", "g", "b")):
                    v = R[:, k] * args.axis_len
                    ax.quiver(o[0], o[1], o[2], v[0], v[1], v[2], color=c, linewidth=1.0)
            ax.scatter(*xyz[0], c="lime", s=70, marker="o", edgecolors="k", label=f"{arm} start")
            ax.scatter(*xyz[-1], c="red", s=80, marker="X", edgecolors="k", label=f"{arm} end")

    if not pts_all:
        print("nothing to plot (no tcp_stand_* found)")
        return 1
    ax.set_xlabel("x (m, stand)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)")
    ax.set_title("data_tcp tool pose (tcp_stand)  |  triad: red=x green=y blue=z(approach)")
    pts = np.vstack(pts_all)
    ctr = pts.mean(0)
    rad = max((pts.max(0) - pts.min(0)).max() / 2, 1e-3)
    ax.set_xlim(ctr[0] - rad, ctr[0] + rad)
    ax.set_ylim(ctr[1] - rad, ctr[1] + rad)
    ax.set_zlim(ctr[2] - rad, ctr[2] + rad)
    try:
        ax.set_box_aspect((1, 1, 1))  # equal visual aspect 1:1:1 (cube ranges set above)
    except Exception:
        pass
    ax.legend(loc="upper left", fontsize=8)
    fig.tight_layout()
    if args.out:
        fig.savefig(args.out, dpi=120)
        print("saved", args.out)
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
