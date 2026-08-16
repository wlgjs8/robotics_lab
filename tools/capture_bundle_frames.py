"""Capture spaced fisheye+realsense color frames from live camera.bundle to an npz.

Usage: .venv/bin/python tools/capture_bundle_frames.py <out.npz> [n_pairs] [spacing_sec]
Saves left/right fisheye and realsense color stacks plus timing metadata, for the
30-vs-90 fps paired policy A/B. Frames are spaced to decorrelate AE flicker.
"""
import sys
import time

import numpy as np

from policy_runner.camera_bundle_client import CameraBundleClient, bundle_clock_ns

OUT = sys.argv[1]
N = int(sys.argv[2]) if len(sys.argv) > 2 else 60
SPACING = float(sys.argv[3]) if len(sys.argv) > 3 else 0.4
KEYS = ["left_fisheye.color", "right_fisheye.color", "left_realsense.color", "right_realsense.color"]


def main():
    client = CameraBundleClient(zmq_endpoint="tcp://127.0.0.1:5600", topic="camera.bundle")
    stacks = {k: [] for k in KEYS}
    ages, seqs = [], []
    while len(seqs) < N:
        bundle = client.poll(timeout_ms=300)
        if bundle is None:
            continue
        if any(k not in bundle.frames for k in KEYS):
            continue
        for k in KEYS:
            stacks[k].append(np.asarray(bundle.frames[k].pixels).copy())
        ages.append((bundle_clock_ns() - bundle.bundle_time_ns) / 1e6)
        seqs.append(bundle.bundle_seq)
        time.sleep(SPACING)
    client.close()

    out = {k.replace(".", "_"): np.stack(v) for k, v in stacks.items()}
    out["poll_age_ms"] = np.array(ages)
    out["bundle_seq"] = np.array(seqs)
    np.savez_compressed(OUT, **out)
    for k in KEYS:
        arr = out[k.replace(".", "_")]
        print(f"{k}: {arr.shape} mean_brightness={arr.mean():.1f}")
    print(f"saved {OUT}; poll_age p50={np.percentile(out['poll_age_ms'], 50):.2f}ms")


if __name__ == "__main__":
    main()
