#!/usr/bin/env python3
"""Summarize flow-infer per-policy-step z descent and force telemetry."""
from __future__ import annotations

import argparse
import json
import math
import statistics
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable


SCHEMA = "robotics_lab.policy_runner.rollout_step.v1"
ARMS = ("left", "right")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Analyze flow-infer rollout-step JSONL for commanded/measured z "
            "descent, compliance offset, wrench, and gripper toggles."
        )
    )
    parser.add_argument("jsonl", help="Path written by flow-infer --rollout-step-log.")
    parser.add_argument(
        "--png",
        default=None,
        help="Optional plot output. If matplotlib is unavailable, text output still succeeds.",
    )
    parser.add_argument(
        "--descent-epsilon-mm",
        type=float,
        default=0.01,
        help="Minimum adjacent commanded-z decrease counted as descent.",
    )
    parser.add_argument(
        "--gripper-toggle-threshold-pct",
        type=float,
        default=50.0,
        help="Opening threshold used only to label OPEN/CLOSE toggle events.",
    )
    parser.add_argument(
        "--max-points-per-segment",
        type=int,
        default=80,
        help=(
            "Maximum printed rows per descent segment and full gap/offset trace; "
            "0 prints every row."
        ),
    )
    parser.add_argument(
        "--blocked-follow-ratio",
        type=float,
        default=0.25,
        help="Diagnostic-only measured/commanded descent ratio hint threshold.",
    )
    parser.add_argument(
        "--offset-growth-mm",
        type=float,
        default=0.5,
        help="Diagnostic-only compliance-offset magnitude growth hint threshold.",
    )
    return parser.parse_args()


def load_records(path: Path) -> tuple[list[dict[str, Any]], int]:
    records: list[dict[str, Any]] = []
    rejected = 0
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            try:
                value = json.loads(line)
            except json.JSONDecodeError:
                rejected += 1
                continue
            if not isinstance(value, dict) or value.get("schema") != SCHEMA:
                rejected += 1
                continue
            records.append(value)
    records.sort(
        key=lambda value: (
            float("inf")
            if _number(value.get("t_mono")) is None
            else float(_number(value.get("t_mono")))
        )
    )
    return records, rejected


def arm_samples(records: Iterable[dict[str, Any]], arm: str) -> list[dict[str, Any]]:
    samples = []
    for record in records:
        arms = record.get("arms")
        arm_data = arms.get(arm) if isinstance(arms, dict) else None
        if not isinstance(arm_data, dict):
            arm_data = {}
        cmd_pose = _vector(arm_data.get("cmd_pose"), 7)
        meas_pose = _vector(arm_data.get("meas_pose"), 7)
        offset = _vector(arm_data.get("compliance_offset_surface"), 6)
        samples.append(
            {
                "record": record,
                "t_mono": _number(record.get("t_mono")),
                "t_wall": _number(record.get("t_wall")),
                "cmd_z_mm": None if cmd_pose is None else cmd_pose[2] * 1000.0,
                "meas_z_mm": None if meas_pose is None else meas_pose[2] * 1000.0,
                "gap_z_mm": _number(arm_data.get("cmd_minus_meas_z_mm")),
                "offset_z_mm": None if offset is None else offset[2] * 1000.0,
                "correction_mm": _scaled(arm_data.get("correction_m"), 1000.0),
                "wrench_tcp_fz": _number(arm_data.get("wrench_tcp_fz")),
                "control_external_wrench_fz": _number(
                    arm_data.get("control_external_wrench_fz")
                ),
                "gripper_cmd_pct": _number(arm_data.get("gripper_cmd_pct")),
                "gripper_meas_pct": _number(arm_data.get("gripper_meas_pct")),
            }
        )
    return samples


def descent_segments(
    samples: list[dict[str, Any]],
    *,
    epsilon_mm: float,
) -> list[list[int]]:
    segments: list[list[int]] = []
    active: list[int] | None = None
    for index in range(1, len(samples)):
        previous = samples[index - 1]["cmd_z_mm"]
        current = samples[index]["cmd_z_mm"]
        descending = (
            previous is not None
            and current is not None
            and previous - current > epsilon_mm
        )
        if descending:
            if active is None:
                active = [index - 1, index]
            elif active[-1] == index - 1:
                active.append(index)
            else:
                segments.append(active)
                active = [index - 1, index]
        elif active is not None:
            segments.append(active)
            active = None
    if active is not None:
        segments.append(active)
    return segments


def gripper_toggles(
    samples: list[dict[str, Any]],
    *,
    threshold_pct: float,
) -> list[tuple[int, str, float]]:
    events: list[tuple[int, str, float]] = []
    previous: str | None = None
    for index, sample in enumerate(samples):
        value = sample["gripper_cmd_pct"]
        if value is None:
            continue
        state = "OPEN" if value >= threshold_pct else "CLOSE"
        if previous is not None and state != previous:
            events.append((index, state, value))
        previous = state
    return events


def print_summary(
    records: list[dict[str, Any]],
    *,
    rejected: int,
    descent_epsilon_mm: float,
    gripper_threshold_pct: float,
    max_points: int,
    blocked_follow_ratio: float,
    offset_growth_mm: float,
) -> dict[str, tuple[list[dict[str, Any]], list[list[int]]]]:
    first_mono = min(
        (value for value in (_number(r.get("t_mono")) for r in records) if value is not None),
        default=0.0,
    )
    print(f"records={len(records)} rejected_lines={rejected} schema={SCHEMA}")
    latencies = [
        value
        for value in (_number(record.get("inference_latency_ms")) for record in records)
        if value is not None
    ]
    if latencies:
        ordered = sorted(latencies)
        p95 = ordered[max(0, math.ceil(0.95 * len(ordered)) - 1)]
        print(
            "inference_latency_ms "
            f"median={statistics.median(ordered):.2f} p95={p95:.2f} max={ordered[-1]:.2f}"
        )
    result: dict[str, tuple[list[dict[str, Any]], list[list[int]]]] = {}
    diagnoses: list[str] = []
    for arm in ARMS:
        samples = arm_samples(records, arm)
        segments = descent_segments(samples, epsilon_mm=descent_epsilon_mm)
        result[arm] = (samples, segments)
        valid_cmd = sum(sample["cmd_z_mm"] is not None for sample in samples)
        valid_meas = sum(sample["meas_z_mm"] is not None for sample in samples)
        valid_offset = sum(sample["offset_z_mm"] is not None for sample in samples)
        print()
        print(
            f"[{arm}] samples={len(samples)} cmd_pose={valid_cmd} "
            f"meas_pose={valid_meas} offset={valid_offset} "
            f"descent_segments={len(segments)}"
        )
        force_like = False
        for number, segment in enumerate(segments, start=1):
            start = samples[segment[0]]
            end = samples[segment[-1]]
            cmd_drop = _difference(start["cmd_z_mm"], end["cmd_z_mm"])
            meas_drop = _difference(start["meas_z_mm"], end["meas_z_mm"])
            offset_growth = _abs_growth(start["offset_z_mm"], end["offset_z_mm"])
            follow_ratio = (
                None
                if cmd_drop is None or cmd_drop <= 0.0 or meas_drop is None
                else meas_drop / cmd_drop
            )
            if (
                follow_ratio is not None
                and follow_ratio < blocked_follow_ratio
                and offset_growth is not None
                and offset_growth >= offset_growth_mm
            ):
                force_like = True
            print(
                f"  descent#{number} {_time_label(start, first_mono)}"
                f" -> {_time_label(end, first_mono)} points={len(segment)}"
                f" cmd_drop={_fmt(cmd_drop, 'mm')}"
                f" meas_drop={_fmt(meas_drop, 'mm')}"
                f" follow_ratio={_fmt(follow_ratio, '')}"
                f" |offset_z|_growth={_fmt(offset_growth, 'mm')}"
            )
            print(
                "    t_rel_s  cmd_z_mm  meas_z_mm  gap_z_mm  "
                "offset_z_mm  correction_mm  wrench_fz  control_fz"
            )
            for index in _limited_indices(segment, max_points):
                sample = samples[index]
                print(
                    f"    {_relative_time(sample, first_mono):7.3f}"
                    f"  {_cell(sample['cmd_z_mm'])}"
                    f"  {_cell(sample['meas_z_mm'])}"
                    f"  {_cell(sample['gap_z_mm'])}"
                    f"  {_cell(sample['offset_z_mm'])}"
                    f"  {_cell(sample['correction_mm'])}"
                    f"  {_cell(sample['wrench_tcp_fz'])}"
                    f"  {_cell(sample['control_external_wrench_fz'])}"
                )
            if max_points > 0 and len(segment) > max_points:
                print(f"    ... {len(segment) - max_points} trajectory rows omitted")

        trace_indices = _limited_indices(list(range(len(samples))), max_points)
        print("  full gap/offset trace: t_rel_s  gap_z_mm  offset_z_mm")
        for index in trace_indices:
            sample = samples[index]
            print(
                f"    {_relative_time(sample, first_mono):7.3f}"
                f"  {_cell(sample['gap_z_mm'])}"
                f"  {_cell(sample['offset_z_mm'])}"
            )
        if max_points > 0 and len(samples) > max_points:
            print(f"    ... {len(samples) - max_points} trace rows omitted")

        toggles = gripper_toggles(samples, threshold_pct=gripper_threshold_pct)
        print(
            f"  gripper toggles (threshold={gripper_threshold_pct:g}%): "
            f"{len(toggles)}"
        )
        for index, state, value in toggles:
            sample = samples[index]
            print(
                f"    {_time_label(sample, first_mono)} -> {state} "
                f"cmd={value:.2f}% meas={_fmt(sample['gripper_meas_pct'], '%')}"
            )

        if not segments:
            diagnoses.append(f"{arm}: cmd z 자체의 연속 하강이 관측되지 않음 → 모델/인지 문제 후보")
        elif force_like:
            diagnoses.append(
                f"{arm}: cmd 하강 대비 meas 추종이 작고 offset이 증가함 → force-control 차단 후보"
            )
        else:
            diagnoses.append(
                f"{arm}: cmd 하강은 관측됨; meas/offset 궤적으로 서보·force 추종 여부 추가 확인"
            )

    print()
    print("판정 힌트:")
    print("  cmd z가 내려가는데 meas z가 안 따라가고 offset이 커짐 → force-control 차단 가능성")
    print("  cmd z 자체가 안 내려감 → 모델/인지 문제 가능성")
    for diagnosis in diagnoses:
        print(f"  관측: {diagnosis}")
    print("  위 판정은 텔레메트리 휴리스틱이며 안전 승인이나 force-control acceptance가 아닙니다.")
    return result


def save_plot(
    path: Path,
    analyzed: dict[str, tuple[list[dict[str, Any]], list[list[int]]]],
) -> bool:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib unavailable: PNG skipped; text summary is complete")
        return False

    all_times = [
        sample["t_mono"]
        for samples, _segments in analyzed.values()
        for sample in samples
        if sample["t_mono"] is not None
    ]
    first_mono = min(all_times, default=0.0)
    figure, axes = plt.subplots(4, 2, figsize=(14, 12), sharex="col")
    for column, arm in enumerate(ARMS):
        samples, segments = analyzed[arm]
        times = [_relative_time(sample, first_mono) for sample in samples]
        axes[0][column].plot(times, _plot_values(samples, "cmd_z_mm"), label="cmd z")
        axes[0][column].plot(times, _plot_values(samples, "meas_z_mm"), label="meas z")
        axes[0][column].set_title(arm)
        axes[0][column].set_ylabel("z (mm)")
        axes[0][column].legend()
        axes[1][column].plot(times, _plot_values(samples, "gap_z_mm"), label="cmd-meas z")
        axes[1][column].set_ylabel("gap (mm)")
        axes[1][column].legend()
        axes[2][column].plot(
            times,
            _plot_values(samples, "offset_z_mm"),
            label="compliance offset z",
        )
        axes[2][column].plot(
            times,
            _plot_values(samples, "correction_mm"),
            label="correction",
            alpha=0.7,
        )
        axes[2][column].set_ylabel("offset (mm)")
        axes[2][column].legend()
        axes[3][column].step(
            times,
            _plot_values(samples, "gripper_cmd_pct"),
            where="post",
            label="gripper cmd",
        )
        axes[3][column].plot(
            times,
            _plot_values(samples, "gripper_meas_pct"),
            label="gripper meas",
            alpha=0.7,
        )
        axes[3][column].set_ylabel("opening (%)")
        axes[3][column].set_xlabel("t - first sample (s)")
        axes[3][column].legend()
        for segment in segments:
            start = _relative_time(samples[segment[0]], first_mono)
            end = _relative_time(samples[segment[-1]], first_mono)
            for row in axes:
                row[column].axvspan(start, end, color="tab:blue", alpha=0.08)
        for row in axes:
            row[column].grid(True, alpha=0.25)
    figure.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=150)
    plt.close(figure)
    print(f"wrote plot: {path}")
    return True


def _limited_indices(indices: list[int], limit: int) -> list[int]:
    if limit <= 0 or len(indices) <= limit:
        return indices
    if limit == 1:
        return [indices[-1]]
    selected = {
        indices[round(position * (len(indices) - 1) / (limit - 1))]
        for position in range(limit)
    }
    return sorted(selected)


def _time_label(sample: dict[str, Any], first_mono: float) -> str:
    relative = _relative_time(sample, first_mono)
    wall = sample.get("t_wall")
    if wall is None:
        return f"t+{relative:.3f}s"
    return f"t+{relative:.3f}s/{datetime.fromtimestamp(wall).isoformat(timespec='milliseconds')}"


def _relative_time(sample: dict[str, Any], first_mono: float) -> float:
    value = sample.get("t_mono")
    return 0.0 if value is None else float(value) - first_mono


def _plot_values(samples: list[dict[str, Any]], key: str) -> list[float]:
    return [
        float("nan") if sample.get(key) is None else float(sample[key])
        for sample in samples
    ]


def _difference(start: float | None, end: float | None) -> float | None:
    return None if start is None or end is None else start - end


def _abs_growth(start: float | None, end: float | None) -> float | None:
    return None if start is None or end is None else abs(end) - abs(start)


def _scaled(value: Any, scale: float) -> float | None:
    resolved = _number(value)
    return None if resolved is None else resolved * scale


def _vector(value: Any, length: int) -> list[float] | None:
    if not isinstance(value, list) or len(value) != length:
        return None
    result = [_number(item) for item in value]
    if any(item is None for item in result):
        return None
    return [float(item) for item in result]


def _number(value: Any) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _fmt(value: float | None, suffix: str) -> str:
    return "n/a" if value is None else f"{value:.3f}{suffix}"


def _cell(value: float | None) -> str:
    return f"{'n/a':>10}" if value is None else f"{value:10.3f}"


def main() -> int:
    args = parse_args()
    if args.descent_epsilon_mm < 0.0:
        raise SystemExit("--descent-epsilon-mm must be non-negative")
    if args.max_points_per_segment < 0:
        raise SystemExit("--max-points-per-segment must be non-negative")
    path = Path(args.jsonl)
    records, rejected = load_records(path)
    if not records:
        print(f"no {SCHEMA} records found in {path}")
        return 1
    analyzed = print_summary(
        records,
        rejected=rejected,
        descent_epsilon_mm=float(args.descent_epsilon_mm),
        gripper_threshold_pct=float(args.gripper_toggle_threshold_pct),
        max_points=int(args.max_points_per_segment),
        blocked_follow_ratio=float(args.blocked_follow_ratio),
        offset_growth_mm=float(args.offset_growth_mm),
    )
    if args.png:
        save_plot(Path(args.png), analyzed)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
