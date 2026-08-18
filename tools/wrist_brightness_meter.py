"""Live wrist-camera brightness/noise meter for tuning scene lighting.

Why: at 90 fps the D405 AE is pinned (exposure capped 9.9 ms, analog gain
saturated at 248 -- measured 2026-08-16), so the only remaining lever to reach
the 30 fps brightness is more light. This reads the live camera.bundle.policy
stream so you can add/aim lamps and watch the numbers move, without stopping
camera_server (the pyrealsense2 probes in probe_d405_*.py cannot -- they need
exclusive device access).

Workflow:
  make cam-up-wrists                                   # 30 fps reference
  .venv/bin/python tools/wrist_brightness_meter.py --save-baseline
  make cam-up-wrists-90                                # 90 fps
  .venv/bin/python tools/wrist_brightness_meter.py     # add light until ratio ~1.00

Temporal noise is a per-pixel std over the sample window, so it is only
meaningful on a STATIC scene (robot parked, nothing moving in view).
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "policy_runner"))

from policy_runner.camera_bundle_client import CameraBundleClient  # noqa: E402

BASELINE = "/tmp/wrist_brightness_baseline.json"
CAMS = ["left_realsense.color", "right_realsense.color"]


def sample(client: CameraBundleClient, n: int) -> dict[str, np.ndarray]:
    stacks: dict[str, list[np.ndarray]] = {k: [] for k in CAMS}
    last_seq = -1
    deadline = time.monotonic() + 10.0
    while len(stacks[CAMS[0]]) < n:
        if time.monotonic() > deadline:
            break
        bundle = client.poll(timeout_ms=300)
        if bundle is None or bundle.bundle_seq == last_seq:
            continue
        if any(k not in bundle.frames for k in CAMS):
            continue
        last_seq = bundle.bundle_seq
        for k in CAMS:
            stacks[k].append(np.asarray(bundle.frames[k].pixels, dtype=np.float32))
    return {k: np.stack(v) for k, v in stacks.items() if v}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="camera.bundle.policy")
    ap.add_argument("--frames", type=int, default=20, help="frames per sample window")
    ap.add_argument("--interval", type=float, default=2.0, help="seconds between readouts")
    ap.add_argument("--save-baseline", action="store_true", help="store this reading as the 30 fps target")
    args = ap.parse_args()

    client = CameraBundleClient(zmq_endpoint="tcp://127.0.0.1:5600", topic=args.topic)
    base = {}
    if not args.save_baseline and os.path.exists(BASELINE):
        base = json.load(open(BASELINE))
        print(f"baseline {BASELINE}: " + "  ".join(f"{k}={v:.1f}" for k, v in base.items()))
    elif not args.save_baseline:
        print(f"(no baseline at {BASELINE} -- run with --save-baseline at 30 fps first)")

    try:
        while True:
            stacks = sample(client, args.frames)
            if len(stacks) < len(CAMS):
                print("waiting for complete bundles on " + args.topic)
                time.sleep(args.interval)
                continue
            row = []
            means = {}
            for k in CAMS:
                mean = float(stacks[k].mean())
                noise = float(stacks[k].std(axis=0).mean())
                means[k] = mean
                cell = f"{k.split('.')[0]:<15} mean={mean:6.1f} noise={noise:4.2f}"
                if k in base and mean > 0:
                    need = base[k] / mean
                    mark = "OK " if 0.95 <= need <= 1.05 else "   "
                    cell += f"  target={base[k]:6.1f} need_light=x{need:4.2f} {mark}"
                row.append(cell)
            print(" | ".join(row), flush=True)
            if args.save_baseline:
                json.dump(means, open(BASELINE, "w"))
                print(f"saved baseline -> {BASELINE}")
                return 0
            time.sleep(args.interval)
    except KeyboardInterrupt:
        return 0
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
