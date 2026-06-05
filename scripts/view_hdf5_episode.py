#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="View HDF5 episode images, poses, deltas, and actions in an OpenCV window."
    )
    parser.add_argument("episode", help="HDF5 episode file")
    parser.add_argument("--single-arm-side", choices=("left", "right"), default="left")
    parser.add_argument("--camera-names", default=None, help="Comma-separated camera allow-list")
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--fps", type=float, default=None)
    parser.add_argument("--image-size", type=int, default=320)
    parser.add_argument("--trail-length", type=int, default=120)
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)

    repo_root = Path(__file__).resolve().parents[1]
    policy_runner_root = repo_root / "policy_runner"
    if str(policy_runner_root) not in sys.path:
        sys.path.insert(0, str(policy_runner_root))

    from policy_runner.hdf5_viewer import run_hdf5_viewer_cli

    return run_hdf5_viewer_cli(args)


if __name__ == "__main__":
    raise SystemExit(main())
