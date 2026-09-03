#!/usr/bin/env python3
"""Section pika_gripper.STL, to show what its 247.642 mm is measured FROM.

This exists because getting that wrong cost a day. The tool offset was briefly changed
to 262.642 mm on the strength of two hand-parked contact poses that each read ~15 mm
long; the agreement between them looked like confirmation but was not, because nothing
checked that the FINGERTIP PLANE was the part making contact, and anything else on the
gripper touching first biases both the same way.

The CAD settles it with no robot involved. The output shows:

  * XY cuts at z = 1..55 mm are a Ø64 body inside a Ø70 base on a BOLT CIRCLE -- the
    RFT64-6A01-A force/torque sensor, not gripper structure.
  * The radius bulges to 60 mm at z = 20..25: the sensor's cable connector, which the
    controller-manager part file calls out as "protruding past the body dia".
  * That stack ends at z ~ 45, which is exactly the sensor_offset_mm the force-control
    config carries (flange -> sensing reference origin).

So the STL starts AT THE FLANGE and contains the sensor, and flange -> fingertip is
247.642 mm. The force config reaches the same number the other way round:
sensor_offset_mm 45 + tool_xyz_mm 202.642.

  rb_servo_server/tools/plot_pika_gripper_sections.py [--out PATH]
"""
from __future__ import annotations

import argparse
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
STL = REPO / "rb_servo_server/descriptions/meshes/robots/rb5_850e/visual/tool/pika_gripper.STL"
OUT = REPO / "docs/reference/pika_gripper_sections.png"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--stl", type=Path, default=STL)
    ap.add_argument("--out", type=Path, default=OUT)
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    import trimesh

    mesh = trimesh.load(args.stl, process=False)

    def cut(normal, origin):
        s = mesh.section(plane_origin=origin, plane_normal=normal)
        return [] if s is None else [np.asarray(d) for d in s.discrete]

    fig = plt.figure(figsize=(23, 11))
    gs = fig.add_gridspec(1, 3, width_ratios=[1, 1, 1.2])
    ax = fig.add_subplot(gs[0, 0])
    for d in cut([0, 1, 0], [0, 0, 0]):
        ax.plot(d[:, 0], d[:, 2], lw=1.1, c="0.15")
    ax.set_title("XZ cut (plane y=0)\nside view; z = tool axis", fontsize=13)
    ax.set_xlabel("x (mm)"); ax.set_ylabel("z (mm)")
    ax2 = fig.add_subplot(gs[0, 1])
    for d in cut([1, 0, 0], [0, 0, 0]):
        ax2.plot(d[:, 1], d[:, 2], lw=1.1, c="0.15")
    ax2.set_title("YZ cut (plane x=0)\nthe other side view", fontsize=13)
    ax2.set_xlabel("y (mm)"); ax2.set_ylabel("z (mm)")
    for a in (ax, ax2):
        a.set_aspect("equal"); a.grid(alpha=0.3); a.set_ylim(-20, 265)
        x0 = a.get_xlim()[0]
        for z, lab, col in (
            (0.0, "z=0   STL origin = FLANGE FACE", "tab:blue"),
            (45.0, "z=45  top of the RFT64 stack\n        (= sensor_offset_mm)", "tab:orange"),
            (247.642, "z=247.642  fingertip plane -> tcp", "tab:green"),
        ):
            a.axhline(z, color=col, lw=1.8, ls="--")
            a.text(x0 + 2, z + 5, lab, color=col, fontsize=10, va="bottom")
    ax3 = fig.add_subplot(gs[0, 2])
    for c, z0 in zip(plt.cm.plasma(np.linspace(0, 0.85, 6)), [1, 10, 20, 30, 40, 55]):
        for d in cut([0, 0, 1], [0, 0, z0]):
            ax3.plot(d[:, 0], d[:, 1], lw=1.7, color=c)
        ax3.plot([], [], color=c, lw=1.7, label=f"z = {z0} mm")
    th = np.linspace(0, 2 * np.pi, 240)
    for r, lab, col, st in (
        (35, "r=35 (Ø70) STL base", "k", "-"),
        (32, "r=32 (Ø64) RFT64 sensor body", "crimson", "--"),
        (29, "r=29 (Ø58) RFT64 flange adapter", "tab:blue", ":"),
    ):
        ax3.plot(r * np.cos(th), r * np.sin(th), st, lw=1.7, color=col, label=lab)
    ax3.set_title("XY cuts through the base, z = 1..55 mm\n(bolt circle + Ø64 body = the F/T sensor)",
                  fontsize=13)
    ax3.set_xlabel("x (mm)"); ax3.set_ylabel("y (mm)")
    ax3.set_aspect("equal"); ax3.grid(alpha=0.3)
    ax3.legend(fontsize=9, loc="upper left", bbox_to_anchor=(1.01, 1))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout(); plt.savefig(args.out, dpi=105, bbox_inches="tight")
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
