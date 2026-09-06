#!/usr/bin/env python3
"""Replay recorded inference durations through the production Python dispatcher.

No model/robot/camera/network is constructed. This uses the unit-test in-memory
I/O fixture; only scheduling, row alignment and timing are evaluated. It cannot
predict a changed policy's actions, camera timing or physical motion quality.
"""
from __future__ import annotations

import argparse
import collections
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "policy_runner"), str(ROOT / "policy_runner" / "tests")]

import numpy as np

from test_chunk_activation_scheduler import OfflineStreamSource


def summary(values):
    values = np.asarray(values, dtype=float)
    return {"count": len(values), **({
        "p50": float(np.quantile(values, .5)),
        "p95": float(np.quantile(values, .95)),
        "p99": float(np.quantile(values, .99)),
        "max": float(values.max()),
    } if len(values) else {})}


def replay(latencies_ms, *, mode, dt, tick_sec, execute_steps):
    source = OfflineStreamSource(mode, dt=dt, execute_steps=execute_steps)
    due = None
    consumed = 0
    tick = 0
    max_ticks = int((sum(latencies_ms) / 1000 + len(latencies_ms) * execute_steps * dt + 10) / tick_sec)
    while tick <= max_ticks:
        now = 1.0 + tick * tick_sec
        if due is not None and now >= due:
            source.complete_request(due)
            due = None
        source.tick(now)
        if len(source.published) == len(latencies_ms):
            break
        if due is None and source._stream_request is not None and consumed < len(latencies_ms):
            requested = source._stream_request[2] / 1e9
            due = requested + latencies_ms[consumed] / 1000
            consumed += 1
        tick += 1
    else:
        raise RuntimeError("dispatcher replay did not finish within its finite bound")
    activations = [row[0] for row in source.published]
    timings = source.activation_timings
    return {
        "mode": mode,
        "requests_replayed": consumed,
        "activated_chunks": len(activations),
        "policy_rows_emitted": len(source.emitted),
        "elapsed_sec": now - 1.0,
        "ready_wait_ms": summary([t["ready_wait_ms"] for t in timings]),
        "activation_period_ms": summary(np.diff(activations) * 1000),
        "request_to_activation_ms": summary([
            (t["activation_monotonic_ns"] - t["request_monotonic_ns"]) / 1e6 for t in timings
        ]),
        "source_start_index_counts": dict(collections.Counter(row[2]["source_start_index"] for row in source.published)),
        "replaced_chunk_steps_counts": dict(collections.Counter(row[2]["replaced_chunk_steps"] for row in source.published[1:])),
        "stall_count": source._stream_stall_count,
        "stall_per_1000_activations": source._stream_stall_count * 1000 / len(activations),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("step_log", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--policy-dt-sec", type=float, required=True)
    parser.add_argument("--tick-sec", type=float, required=True)
    parser.add_argument("--execute-steps", type=int, required=True)
    parser.add_argument("--exclude", action="append", default=[], help="Relative start:end seconds, repeatable")
    args = parser.parse_args()
    if args.policy_dt_sec <= 0 or args.tick_sec <= 0 or args.execute_steps <= 0:
        parser.error("dt, tick and execute steps must be positive")
    excluded = [tuple(map(float, item.split(":"))) for item in args.exclude]
    latencies = []
    first = None
    for line in args.step_log.open():
        row = json.loads(line)
        if first is None:
            first = float(row["t_mono"])
        relative = float(row["t_mono"]) - first
        value = row.get("inference_latency_ms")
        if row.get("chunk_step_index") != 0 or value is None:
            continue
        if any(start <= relative <= end for start, end in excluded):
            continue
        if not np.isfinite(value) or value < 0:
            raise ValueError("recorded inference latency must be finite and nonnegative")
        latencies.append(float(value))
    if not latencies:
        parser.error("no activation inference latencies selected")
    result = {
        "input": str(args.step_log.resolve()),
        "excluded_relative_seconds": excluded,
        "policy_dt_sec": args.policy_dt_sec,
        "tick_sec": args.tick_sec,
        "execute_steps": args.execute_steps,
        "inference_latency_ms": summary(latencies),
        "assumptions": [
            "Identical recorded inference durations applied in sequence to both schedulers.",
            "Zero queue wait; an ideal fixed command tick grid; synthetic H24 rows used only to verify alignment.",
            "Selected normal phases are concatenated; InitMotion and camera sampling are not simulated.",
            "Production dispatch/activation/request code runs; inference completion and I/O are in-memory fixtures.",
            "This predicts scheduler timing only; changed model outputs, inference contention and robot stability require separate validation.",
        ],
        "results": [replay(latencies, mode=mode, dt=args.policy_dt_sec, tick_sec=args.tick_sec,
                           execute_steps=args.execute_steps) for mode in ("fixed_steps", "ready_event")],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
