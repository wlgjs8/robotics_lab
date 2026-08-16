"""Compare flow-infer rollout-step JSONL logs across ablation runs.

Reads one or more logs written by ``flow-infer --rollout-step-log`` (schema
``robotics_lab.policy_runner.rollout_step.v1``) and prints one compact metrics
row per run so single-variable sweeps (speed_scale, chunk_execute_steps,
blend_steps, chunk_anchor, ...) can be ranked on smoothness and tracking
quality side by side. Task success stays a manual column — record it in the
run notes; this tool covers everything measurable from telemetry.

Metrics per run:
  steps           logged policy steps
  stall%          fraction of steps flagged stall (boundary starvation)
  hold%           fraction of steps flagged hold
  lat p50/p90     inference_latency_ms percentiles (when present)
  v_cmd mm/s      mean commanded TCP speed over both arms (from cmd_pose)
  zgap p95 mm     p95 of |cmd_minus_meas_z_mm| (command-vs-measured z tracking)
  grip tog        gripper command open/close toggle count (both arms)
  d_cfg           configured --rtc-inference-delay (the rows the server freezes)
  d_real          mean REALIZED delay (source_start_index: policy steps emitted
                  between the chunk's observation and its activation)
  d_ok%           fraction of chunks where realized == configured. Below ~100 means
                  the frozen prefix does not line up with the dropped prefix:
                  d_real < d_cfg replays stale plan, d_real > d_cfg jumps at the
                  boundary. Pair --rtc-inference-delay with
                  (chunk_execute_steps - stream_prefetch_at) to fix it.

Usage:
  python3 scripts/compare_rollout_step_logs.py outputs/sweep/*.jsonl
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

EXPECTED_SCHEMA = "robotics_lab.policy_runner.rollout_step.v1"
GRIPPER_TOGGLE_THRESHOLD_PCT = 20.0


def _percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    pos = (len(ordered) - 1) * q
    lo = math.floor(pos)
    hi = math.ceil(pos)
    if lo == hi:
        return ordered[lo]
    frac = pos - lo
    return ordered[lo] * (1.0 - frac) + ordered[hi] * frac


def summarize_log(path: Path) -> dict[str, object]:
    steps = 0
    stalls = 0
    holds = 0
    latencies: list[float] = []
    speeds_mm_s: list[float] = []
    z_gaps_mm: list[float] = []
    gripper_toggles = 0
    prev_t: float | None = None
    prev_cmd: dict[str, list[float] | None] = {"left": None, "right": None}
    prev_grip_open: dict[str, bool | None] = {"left": None, "right": None}
    bad_lines = 0
    rtc_configured: set[int] = set()
    rtc_realized: list[int] = []
    rtc_errors: list[int] = []
    prev_rtc_chunk_id: object = object()

    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                bad_lines += 1
                continue
            if record.get("schema") != EXPECTED_SCHEMA:
                bad_lines += 1
                continue
            steps += 1
            if record.get("stall"):
                stalls += 1
            if record.get("hold"):
                holds += 1
            latency = record.get("inference_latency_ms")
            if isinstance(latency, (int, float)) and math.isfinite(latency):
                latencies.append(float(latency))

            # RTC delay accounting is per chunk, but the record is per step: sample it
            # once per chunk_id so a long chunk does not outweigh a short one.
            rtc = record.get("rtc")
            chunk_id = record.get("chunk_id")
            if isinstance(rtc, dict) and chunk_id != prev_rtc_chunk_id:
                prev_rtc_chunk_id = chunk_id
                configured = rtc.get("configured_delay")
                realized = rtc.get("realized_delay")
                error = rtc.get("delay_error")
                if isinstance(configured, int):
                    rtc_configured.add(configured)
                if isinstance(realized, int):
                    rtc_realized.append(realized)
                if isinstance(error, int):
                    rtc_errors.append(error)

            t_mono = record.get("t_mono")
            dt = None
            if isinstance(t_mono, (int, float)) and prev_t is not None:
                dt = float(t_mono) - prev_t
            if isinstance(t_mono, (int, float)):
                prev_t = float(t_mono)

            arms = record.get("arms")
            arms = arms if isinstance(arms, dict) else {}
            for arm in ("left", "right"):
                arm_rec = arms.get(arm)
                arm_rec = arm_rec if isinstance(arm_rec, dict) else {}

                cmd_pose = arm_rec.get("cmd_pose")
                if (
                    isinstance(cmd_pose, list)
                    and len(cmd_pose) >= 3
                    and prev_cmd[arm] is not None
                    and dt is not None
                    and dt > 0.0
                ):
                    prev_pose = prev_cmd[arm]
                    dist_m = math.sqrt(
                        sum((cmd_pose[i] - prev_pose[i]) ** 2 for i in range(3))
                    )
                    speeds_mm_s.append(dist_m * 1000.0 / dt)
                prev_cmd[arm] = (
                    [float(v) for v in cmd_pose[:3]]
                    if isinstance(cmd_pose, list) and len(cmd_pose) >= 3
                    else None
                )

                z_gap = arm_rec.get("cmd_minus_meas_z_mm")
                if isinstance(z_gap, (int, float)) and math.isfinite(z_gap):
                    z_gaps_mm.append(abs(float(z_gap)))

                grip = arm_rec.get("gripper_cmd_pct")
                if isinstance(grip, (int, float)) and math.isfinite(grip):
                    is_open = float(grip) >= GRIPPER_TOGGLE_THRESHOLD_PCT
                    if prev_grip_open[arm] is not None and is_open != prev_grip_open[arm]:
                        gripper_toggles += 1
                    prev_grip_open[arm] = is_open

    return {
        "run": path.stem,
        "steps": steps,
        "stall_pct": 100.0 * stalls / steps if steps else None,
        "hold_pct": 100.0 * holds / steps if steps else None,
        "lat_p50_ms": _percentile(latencies, 0.50),
        "lat_p90_ms": _percentile(latencies, 0.90),
        "v_cmd_mm_s": (sum(speeds_mm_s) / len(speeds_mm_s)) if speeds_mm_s else None,
        "zgap_p95_mm": _percentile(z_gaps_mm, 0.95),
        "gripper_toggles": gripper_toggles,
        "rtc_d_cfg": (
            str(sorted(rtc_configured)[0]) if len(rtc_configured) == 1
            else ("/".join(str(v) for v in sorted(rtc_configured)) if rtc_configured else None)
        ),
        "rtc_d_real": (sum(rtc_realized) / len(rtc_realized)) if rtc_realized else None,
        "rtc_d_match_pct": (
            100.0 * sum(1 for e in rtc_errors if e == 0) / len(rtc_errors)
            if rtc_errors else None
        ),
        "bad_lines": bad_lines,
    }


def _fmt(value: object, digits: int = 1) -> str:
    if value is None:
        return "-"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compare flow-infer rollout-step JSONL logs across runs."
    )
    parser.add_argument("logs", nargs="+", help="rollout-step JSONL paths")
    args = parser.parse_args()

    rows = [summarize_log(Path(p)) for p in args.logs]
    header = (
        f"{'run':<32} {'steps':>6} {'stall%':>7} {'hold%':>7} "
        f"{'lat_p50':>8} {'lat_p90':>8} {'v_cmd':>8} {'zgap_p95':>9} {'grip_tog':>8} "
        f"{'d_cfg':>6} {'d_real':>7} {'d_ok%':>6}"
    )
    print(header)
    print("-" * len(header))
    for row in rows:
        print(
            f"{str(row['run'])[:32]:<32} {_fmt(row['steps']):>6} "
            f"{_fmt(row['stall_pct']):>7} {_fmt(row['hold_pct']):>7} "
            f"{_fmt(row['lat_p50_ms']):>8} {_fmt(row['lat_p90_ms']):>8} "
            f"{_fmt(row['v_cmd_mm_s']):>8} {_fmt(row['zgap_p95_mm']):>9} "
            f"{_fmt(row['gripper_toggles']):>8} "
            f"{_fmt(row['rtc_d_cfg']):>6} {_fmt(row['rtc_d_real'], 2):>7} "
            f"{_fmt(row['rtc_d_match_pct']):>6}"
        )
        if row["bad_lines"]:
            print(f"  (warning: {row['bad_lines']} unparseable/foreign lines skipped)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
