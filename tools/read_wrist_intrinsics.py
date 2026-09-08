#!/usr/bin/env python3
"""Read the INFERENCE wrist cameras' colour intrinsics and print the FLOW_INFER_K_* env lines.

`FLOW_INFER_K_NORMALIZE=1` remaps every wrist frame onto the virtual camera the
`boltv2r6knorm*` checkpoints were trained on, which needs this cell's own K per arm. That value is
a measurement, so the runtime refuses to guess it (a wrong fy silently rescales the image the
policy aims with). This script is the authoritative source: it asks librealsense for the active
colour profile of each wrist unit.

The wrist cameras are normally held by camera_server, and librealsense gives exclusive access to
one process, so stop it first:

    make cam-down
    python3 tools/read_wrist_intrinsics.py
    make cam-up-wrists

Serial numbers come from the tracked camera_server rig config unless given with --left/--right.
"""

from __future__ import annotations

import argparse
import sys


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--left", default=None, help="left wrist D405 serial (default: auto-detect by role)")
    ap.add_argument("--right", default=None, help="right wrist D405 serial")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    args = ap.parse_args()

    try:
        import pyrealsense2 as rs
    except ImportError:
        print(
            "pyrealsense2 is not importable here. Run this on the robot host with the librealsense "
            "python bindings installed; do NOT substitute a hand-typed value.",
            file=sys.stderr,
        )
        return 2

    ctx = rs.context()
    serials = [d.get_info(rs.camera_info.serial_number) for d in ctx.devices]
    if not serials:
        print(
            "no RealSense devices visible. If camera_server is running it holds them exclusively: "
            "`make cam-down` first.",
            file=sys.stderr,
        )
        return 3
    print(f"# devices present: {', '.join(serials)}", file=sys.stderr)

    wanted = {"LEFT": args.left, "RIGHT": args.right}
    if not any(wanted.values()):
        print(
            "# --left/--right not given; printing every device so you can map serial -> arm from\n"
            "# camera_server's rig config (the wrist role is a wiring fact, not a device property).",
            file=sys.stderr,
        )
        wanted = {sn: sn for sn in serials}

    rc = 0
    for label, serial in wanted.items():
        if serial is None:
            continue
        if serial not in serials:
            print(f"# {label}: serial {serial} not present", file=sys.stderr)
            rc = 4
            continue
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, 30)
        try:
            profile = pipe.start(cfg)
            intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        finally:
            try:
                pipe.stop()
            except Exception:
                pass
        if (intr.width, intr.height) != (args.width, args.height):
            print(f"# {label}: got {intr.width}x{intr.height}, expected {args.width}x{args.height}", file=sys.stderr)
            rc = 5
        name = label if label in ("LEFT", "RIGHT") else f"<arm for {serial}>"
        print(f"export FLOW_INFER_K_{name}='{intr.fx:.4f},{intr.fy:.4f},{intr.ppx:.4f},{intr.ppy:.4f}'  # {serial}")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
