"""Measure camera.bundle publish rate, freshness age, and intra-bundle skew.

Usage: .venv/bin/python tools/measure_bundle_rate.py [seconds] [topic]
Run while camera_server is up. Reports per-topic bundle Hz, poll-time age
(camera clock -> now, same MONOTONIC_RAW domain camera_server stamps), and
skew between the earliest and latest frame inside each bundle.
"""
import sys
import time

import numpy as np

from policy_runner.camera_bundle_client import CameraBundleClient, bundle_clock_ns

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
TOPIC = sys.argv[2] if len(sys.argv) > 2 else "camera.bundle"


def main():
    client = CameraBundleClient(zmq_endpoint="tcp://127.0.0.1:5600", topic=TOPIC)
    seqs, ages_ms, skews_ms, cameras = [], [], [], set()
    t_end = time.time() + DURATION
    while time.time() < t_end:
        bundle = client.poll(timeout_ms=200)
        if bundle is None:
            continue
        seqs.append(bundle.bundle_seq)
        ages_ms.append((bundle_clock_ns() - bundle.bundle_time_ns) / 1e6)
        stamps = [f.host_arrival_time_ns for f in bundle.frames.values()]
        skews_ms.append((max(stamps) - min(stamps)) / 1e6)
        cameras.update(bundle.frames.keys())
    client.close()

    if len(seqs) < 2:
        raise SystemExit(f"no bundles received on {TOPIC}")

    seq_arr = np.array(seqs)
    span = seq_arr[-1] - seq_arr[0]
    pub_hz = span / DURATION  # seq advances even when we drain past bundles
    age = np.array(ages_ms)
    skew = np.array(skews_ms)
    print(f"topic={TOPIC} received={len(seqs)} bundles in {DURATION:.0f}s")
    print(f"  streams per bundle: {sorted(cameras)}")
    print(f"  publish rate (seq span / time): {pub_hz:.1f} Hz")
    print(f"  poll-age ms: p50={np.percentile(age, 50):.2f} p95={np.percentile(age, 95):.2f} max={age.max():.2f}")
    print(f"  intra-bundle skew ms: p50={np.percentile(skew, 50):.2f} p95={np.percentile(skew, 95):.2f} max={skew.max():.2f}")


if __name__ == "__main__":
    main()
