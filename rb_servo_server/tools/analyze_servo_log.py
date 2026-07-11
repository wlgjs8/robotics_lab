#!/usr/bin/env python3
"""Analyze rb_servo_server CSV logs against mock/rbsim servo budgets.

The analyzer intentionally uses only the Python standard library so it can run in
minimal smoke-test environments.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


@dataclass(frozen=True)
class ProfileBudget:
    name: str
    min_duration_s: float
    target_period_ms: float
    period_tolerance_ms: float
    jitter_p95_ms: float
    jitter_max_ms: float
    send_skew_p95_us: float
    send_skew_max_us: float | None
    send_duration_p95_us: float | None
    dropped_samples_max: int
    send_failure_count_max: int
    tracking_error_max_deg: float | None = None


BUDGETS: dict[str, ProfileBudget] = {
    "mock200": ProfileBudget(
        name="mock200",
        min_duration_s=60.0,
        target_period_ms=5.0,
        period_tolerance_ms=0.25,
        jitter_p95_ms=1.0,
        jitter_max_ms=5.0,
        send_skew_p95_us=100.0,
        send_skew_max_us=500.0,
        send_duration_p95_us=500.0,
        dropped_samples_max=0,
        send_failure_count_max=0,
        tracking_error_max_deg=2.0,
    ),
    "rbsim-local100": ProfileBudget(
        name="rbsim-local100",
        min_duration_s=2.0,
        target_period_ms=10.0,
        period_tolerance_ms=0.75,
        jitter_p95_ms=3.0,
        jitter_max_ms=15.0,
        send_skew_p95_us=3000.0,
        send_skew_max_us=None,
        send_duration_p95_us=None,
        dropped_samples_max=0,
        send_failure_count_max=0,
        tracking_error_max_deg=2.0,
    ),
    "rbsim100": ProfileBudget(
        name="rbsim100",
        min_duration_s=30.0,
        target_period_ms=10.0,
        period_tolerance_ms=0.5,
        jitter_p95_ms=2.0,
        jitter_max_ms=10.0,
        send_skew_p95_us=2000.0,
        send_skew_max_us=None,
        send_duration_p95_us=None,
        dropped_samples_max=0,
        send_failure_count_max=0,
        tracking_error_max_deg=2.0,
    ),
}

BASE_REQUIRED_COLUMNS = [
    "period_ms",
    "jitter_ms",
    "send_skew_us",
    "left_send_start_ns",
    "left_send_end_ns",
    "right_send_start_ns",
    "right_send_end_ns",
    "left_send_duration_us",
    "right_send_duration_us",
    "logger_dropped_samples",
    "left_send_ok",
    "right_send_ok",
]

Q_REQUIRED_COLUMNS = [
    f"{arm}_q_{kind}_{joint}"
    for arm in ("left", "right")
    for kind in ("actual", "sent")
    for joint in range(6)
]

TIMESTAMP_COLUMNS = ["loop_start_time_ns", "loop_end_time_ns"]
DELTA_TWIST_STEP_KIND_LABELS = {
    0: "inactive",
    1: "normal",
    2: "reserve",
    3: "residual_drain",
    4: "ringdown",
}
CHUNK_DIAGNOSTIC_INTEGER_COLUMNS = (
    "chunk_frame_wire_seq",
    "chunk_frame_recv_seq",
    "chunk_frame_horizon",
    "chunk_frame_execute_steps",
    "chunk_frame_runway_steps",
    "chunk_inference_seq",
    "chunk_inference_stall_count",
    "chunk_camera_bundle_seq",
    "chunk_camera_left_frame_number",
    "chunk_camera_right_frame_number",
)
CHUNK_DIAGNOSTIC_FLOAT_COLUMNS = (
    "chunk_frame_policy_dt_sec",
    "chunk_frame_age_ms",
    "chunk_frame_interarrival_ms",
    "chunk_inference_queue_wait_ms",
    "chunk_inference_latency_ms",
    "chunk_inference_ready_wait_ms",
    "chunk_inference_period_ms",
    "chunk_inference_period_jitter_ms",
    "chunk_camera_bundle_age_ms",
    "chunk_camera_max_skew_ms",
    "chunk_camera_left_frame_age_ms",
    "chunk_camera_right_frame_age_ms",
    "chunk_camera_left_focus_score",
    "chunk_camera_right_focus_score",
)
DELTA_TWIST_CLAMP_MASK_LABELS = (
    "pending_linear",
    "pending_angular",
    "xi_ref_velocity_linear",
    "xi_ref_velocity_angular",
    "desired_accel_linear",
    "desired_accel_angular",
    "desired_jerk_linear",
    "desired_jerk_angular",
    "accel_cmd_linear",
    "accel_cmd_angular",
    "xi_cmd_velocity_linear",
    "xi_cmd_velocity_angular",
    "lead_linear",
    "lead_angular",
)
DELTA_TWIST_ACCEL_COMMAND_SUFFIXES = (
    "x_m_s2",
    "y_m_s2",
    "z_m_s2",
    "rx_rad_s2",
    "ry_rad_s2",
    "rz_rad_s2",
)


class AnalysisError(Exception):
    """Raised for invalid input that prevents log analysis."""


def parse_float(value: str, column: str, row_number: int) -> float:
    try:
        number = float(value)
    except ValueError as exc:
        raise AnalysisError(f"row {row_number}: column {column!r} is not numeric: {value!r}") from exc
    if not math.isfinite(number):
        raise AnalysisError(f"row {row_number}: column {column!r} is not finite: {value!r}")
    return number


def parse_bool(value: str, column: str, row_number: int) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "t", "yes", "y", "ok"}:
        return True
    if normalized in {"0", "false", "f", "no", "n", "fail", "failed"}:
        return False
    raise AnalysisError(f"row {row_number}: column {column!r} is not boolean-like: {value!r}")


def optional_float(row: dict[str, str], column: str, row_number: int) -> float | None:
    value = row.get(column)
    if value is None or value == "":
        return None
    return parse_float(value, column, row_number)


def optional_bool(row: dict[str, str], column: str, row_number: int) -> bool | None:
    value = row.get(column)
    if value is None or value == "":
        return None
    return parse_bool(value, column, row_number)


def optional_int(row: dict[str, str], column: str, row_number: int) -> int | None:
    value = optional_float(row, column, row_number)
    if value is None:
        return None
    integer = int(value)
    if value != integer:
        raise AnalysisError(f"row {row_number}: column {column} must be an integer")
    return integer


def percentile_nearest_rank(values: Sequence[float], percentile: float) -> float:
    if not values:
        raise AnalysisError("cannot compute percentile of an empty series")
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, math.ceil((percentile / 100.0) * len(ordered)) - 1))
    return ordered[index]


def mean(values: Sequence[float]) -> float:
    if not values:
        raise AnalysisError("cannot compute mean of an empty series")
    return statistics.fmean(values)


def series_summary(values: Sequence[float]) -> dict[str, float]:
    return {
        "mean": mean(values),
        "p95": percentile_nearest_rank(values, 95.0),
        "max": max(values),
    }


def percentile_summary(values: Sequence[float]) -> dict[str, float | int | None]:
    if not values:
        return {"count": 0, "p50": None, "p90": None, "p99": None}
    return {
        "count": len(values),
        "p50": percentile_nearest_rank(values, 50.0),
        "p90": percentile_nearest_rank(values, 90.0),
        "p99": percentile_nearest_rank(values, 99.0),
    }


def require_columns(fieldnames: Iterable[str] | None, required: Sequence[str]) -> list[str]:
    available = set(fieldnames or [])
    return [column for column in required if column not in available]


def make_delta_twist_bucket() -> dict[str, object]:
    return {
        "rows": 0,
        "active_ticks": 0,
        "stall_ticks": 0,
        "saturated_ticks": 0,
        "pending_clamped_ticks": 0,
        "xi_ref_clamped_ticks": 0,
        "xi_cmd_clamped_ticks": 0,
        "min_time_to_go_ticks": 0,
        "controller_counts": {},
        "step_counts": {},
        "step_kind_counts": {},
        "accel_clamp_counts": {},
        "feedback_source_counts": {},
        "clamp_mask_counts": {},
        "frame_rows": [],
        "normal_budget": [],
        "total_budget": [],
        "steps_remaining": [],
        "accel_command": {
            suffix: [] for suffix in DELTA_TWIST_ACCEL_COMMAND_SUFFIXES
        },
        "pending_linear": [],
        "pending_angular": [],
        "xi_ref_linear": [],
        "xi_ref_angular": [],
        "xi_linear": [],
        "xi_angular": [],
        "requested_yaw": [],
        "realized_yaw": [],
        "yaw_ratio": [],
        "linear_ratio": [],
        "angular_ratio": [],
        "stage_positions": [],
    }


def make_chunk_diagnostics_bucket() -> dict[str, object]:
    return {
        "rows": 0,
        "values": {
            column: []
            for column in (*CHUNK_DIAGNOSTIC_INTEGER_COLUMNS, *CHUNK_DIAGNOSTIC_FLOAT_COLUMNS)
        },
        "wire_sequences": set(),
    }


def bump_count(counts: dict[str, int], key: str) -> None:
    counts[key] = counts.get(key, 0) + 1


def finalize_delta_twist_bucket(bucket: dict[str, object]) -> dict[str, object]:
    step_counts = bucket["step_counts"]
    step_kind_counts = bucket["step_kind_counts"]
    controller_counts = bucket["controller_counts"]
    accel_clamp_counts = bucket["accel_clamp_counts"]
    feedback_source_counts = bucket["feedback_source_counts"]
    clamp_mask_counts = bucket["clamp_mask_counts"]
    accel_command = bucket["accel_command"]
    assert isinstance(step_counts, dict)
    assert isinstance(step_kind_counts, dict)
    assert isinstance(controller_counts, dict)
    assert isinstance(accel_clamp_counts, dict)
    assert isinstance(feedback_source_counts, dict)
    assert isinstance(clamp_mask_counts, dict)
    assert isinstance(accel_command, dict)
    kind_total = sum(int(value) for value in step_kind_counts.values())
    kind_percent = {
        key: (100.0 * float(value) / float(kind_total)) if kind_total > 0 else 0.0
        for key, value in sorted(step_kind_counts.items())
    }
    requested_yaw = bucket["requested_yaw"]
    realized_yaw = bucket["realized_yaw"]
    assert isinstance(requested_yaw, list)
    assert isinstance(realized_yaw, list)
    yaw_pairs = [
        (float(req), float(realized))
        for req, realized in zip(requested_yaw, realized_yaw)
        if abs(float(req)) > 1e-6 and abs(float(realized)) > 1e-6
    ]
    yaw_sign_match_percent = None
    if yaw_pairs:
        yaw_sign_match_percent = 100.0 * sum(
            (req > 0.0) == (realized > 0.0) for req, realized in yaw_pairs
        ) / float(len(yaw_pairs))

    stage_positions = bucket["stage_positions"]
    assert isinstance(stage_positions, list)
    stage_path_length_m = None
    stage_net_displacement_m = None
    stage_path_to_net_ratio = None
    if len(stage_positions) >= 2:
        def distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
            return math.sqrt(
                (b[0] - a[0]) * (b[0] - a[0]) +
                (b[1] - a[1]) * (b[1] - a[1]) +
                (b[2] - a[2]) * (b[2] - a[2])
            )

        stage_path_length_m = sum(
            distance(stage_positions[i - 1], stage_positions[i])
            for i in range(1, len(stage_positions))
        )
        stage_net_displacement_m = distance(stage_positions[0], stage_positions[-1])
        if stage_net_displacement_m > 1e-6:
            stage_path_to_net_ratio = stage_path_length_m / stage_net_displacement_m

    warnings: list[str] = []
    if int(bucket["pending_clamped_ticks"]) > 0:
        warnings.append("pending residual hit configured clamp")
    if int(bucket["xi_ref_clamped_ticks"]) > 0:
        warnings.append("xi_ref hit configured vector norm limit")
    if int(bucket["xi_cmd_clamped_ticks"]) > 0:
        warnings.append("xi_cmd hit configured vector norm limit")
    if stage_path_to_net_ratio is not None and stage_path_to_net_ratio > 5.0:
        warnings.append(f"stage path/net displacement ratio is high ({stage_path_to_net_ratio:.3g})")
    if yaw_sign_match_percent is not None and yaw_sign_match_percent < 70.0:
        warnings.append(f"requested/realized yaw sign match is low ({yaw_sign_match_percent:.1f}%)")

    return {
        "rows": bucket["rows"],
        "active_ticks": bucket["active_ticks"],
        "stall_ticks": bucket["stall_ticks"],
        "saturated_ticks": bucket["saturated_ticks"],
        "pending_clamped_ticks": bucket["pending_clamped_ticks"],
        "xi_ref_clamped_ticks": bucket["xi_ref_clamped_ticks"],
        "xi_cmd_clamped_ticks": bucket["xi_cmd_clamped_ticks"],
        "min_time_to_go_ticks": bucket["min_time_to_go_ticks"],
        "controller_counts": dict(sorted(controller_counts.items())),
        "step_kind_counts": dict(sorted(step_kind_counts.items())),
        "step_kind_percent": kind_percent,
        "accel_clamp_counts": dict(sorted(accel_clamp_counts.items())),
        "feedback_source_counts": dict(sorted(feedback_source_counts.items())),
        "clamp_mask_counts": dict(sorted(clamp_mask_counts.items())),
        "frame_rows": percentile_summary(bucket["frame_rows"]),  # type: ignore[arg-type]
        "normal_budget": percentile_summary(bucket["normal_budget"]),  # type: ignore[arg-type]
        "total_budget": percentile_summary(bucket["total_budget"]),  # type: ignore[arg-type]
        "steps_remaining": percentile_summary(bucket["steps_remaining"]),  # type: ignore[arg-type]
        "accel_command": {
            suffix: percentile_summary(values)
            for suffix, values in accel_command.items()
        },
        "follower_step_distribution": dict(
            sorted(step_counts.items(), key=lambda item: int(item[0]))
        ),
        "pending_residual_linear_norm_m": percentile_summary(bucket["pending_linear"]),  # type: ignore[arg-type]
        "pending_residual_angular_norm_rad": percentile_summary(bucket["pending_angular"]),  # type: ignore[arg-type]
        "xi_ref_linear_norm_m_s": percentile_summary(bucket["xi_ref_linear"]),  # type: ignore[arg-type]
        "xi_ref_angular_norm_rad_s": percentile_summary(bucket["xi_ref_angular"]),  # type: ignore[arg-type]
        "commanded_linear_velocity_m_s": percentile_summary(bucket["xi_linear"]),  # type: ignore[arg-type]
        "commanded_angular_velocity_rad_s": percentile_summary(bucket["xi_angular"]),  # type: ignore[arg-type]
        "requested_yaw_delta_rad": percentile_summary(bucket["requested_yaw"]),  # type: ignore[arg-type]
        "realized_yaw_delta_rad": percentile_summary(bucket["realized_yaw"]),  # type: ignore[arg-type]
        "yaw_realized_ratio": percentile_summary(bucket["yaw_ratio"]),  # type: ignore[arg-type]
        "linear_realized_ratio": percentile_summary(bucket["linear_ratio"]),  # type: ignore[arg-type]
        "angular_realized_ratio": percentile_summary(bucket["angular_ratio"]),  # type: ignore[arg-type]
        "yaw_sign_match_percent": yaw_sign_match_percent,
        "stage_path_length_m": stage_path_length_m,
        "stage_net_displacement_m": stage_net_displacement_m,
        "stage_path_to_net_ratio": stage_path_to_net_ratio,
        "warnings": warnings,
    }


def finalize_chunk_diagnostics_bucket(bucket: dict[str, object]) -> dict[str, object]:
    values = bucket["values"]
    wire_sequences = bucket["wire_sequences"]
    assert isinstance(values, dict)
    assert isinstance(wire_sequences, set)
    return {
        "rows": bucket["rows"],
        "unique_wire_sequences": len(wire_sequences),
        "series": {
            column: percentile_summary(samples)
            for column, samples in values.items()
        },
    }


def analyze_csv(path: Path) -> dict[str, object]:
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        missing = require_columns(reader.fieldnames, BASE_REQUIRED_COLUMNS + Q_REQUIRED_COLUMNS)
        if missing:
            raise AnalysisError("missing required CSV columns: " + ", ".join(missing))

        period_ms: list[float] = []
        jitter_ms: list[float] = []
        send_skew_us: list[float] = []
        left_send_duration_us: list[float] = []
        right_send_duration_us: list[float] = []
        dropped_samples: list[int] = []
        tracking_errors: dict[str, list[float]] = {"left": [], "right": []}
        send_failures_by_arm = {"left": 0, "right": 0}
        rows_with_send_failure = 0
        safety_verdict_counts: dict[str, int] = {}
        delta_twist_series = {
            "left": make_delta_twist_bucket(),
            "right": make_delta_twist_bucket(),
        }
        chunk_diagnostics = make_chunk_diagnostics_bucket()
        first_loop_start_ns: float | None = None
        last_loop_end_ns: float | None = None
        rows = 0

        for rows, row in enumerate(reader, start=1):
            row_number = rows + 1  # include header line for user-facing errors
            period_ms.append(parse_float(row["period_ms"], "period_ms", row_number))
            jitter_ms.append(abs(parse_float(row["jitter_ms"], "jitter_ms", row_number)))
            send_skew_us.append(abs(parse_float(row["send_skew_us"], "send_skew_us", row_number)))
            left_send_start_ns = parse_float(row["left_send_start_ns"], "left_send_start_ns", row_number)
            left_send_end_ns = parse_float(row["left_send_end_ns"], "left_send_end_ns", row_number)
            right_send_start_ns = parse_float(row["right_send_start_ns"], "right_send_start_ns", row_number)
            right_send_end_ns = parse_float(row["right_send_end_ns"], "right_send_end_ns", row_number)
            if left_send_end_ns < left_send_start_ns:
                raise AnalysisError(f"row {row_number}: left_send_end_ns is before left_send_start_ns")
            if right_send_end_ns < right_send_start_ns:
                raise AnalysisError(f"row {row_number}: right_send_end_ns is before right_send_start_ns")
            left_send_duration_us.append(parse_float(row["left_send_duration_us"], "left_send_duration_us", row_number))
            right_send_duration_us.append(parse_float(row["right_send_duration_us"], "right_send_duration_us", row_number))

            dropped_value = parse_float(row["logger_dropped_samples"], "logger_dropped_samples", row_number)
            dropped_samples.append(int(dropped_value))
            if dropped_value != int(dropped_value):
                raise AnalysisError(f"row {row_number}: logger_dropped_samples must be an integer")

            left_ok = parse_bool(row["left_send_ok"], "left_send_ok", row_number)
            right_ok = parse_bool(row["right_send_ok"], "right_send_ok", row_number)
            if not left_ok:
                send_failures_by_arm["left"] += 1
            if not right_ok:
                send_failures_by_arm["right"] += 1
            if not left_ok or not right_ok:
                rows_with_send_failure += 1

            safety_verdict = row.get("safety_verdict")
            if safety_verdict:
                bump_count(safety_verdict_counts, safety_verdict)

            chunk_row_present = False
            chunk_values = chunk_diagnostics["values"]
            assert isinstance(chunk_values, dict)
            for column in CHUNK_DIAGNOSTIC_INTEGER_COLUMNS:
                value = optional_int(row, column, row_number)
                if value is not None:
                    chunk_row_present = True
                    chunk_values[column].append(value)
                    if column == "chunk_frame_wire_seq" and value > 0:
                        chunk_diagnostics["wire_sequences"].add(value)  # type: ignore[union-attr]
            for column in CHUNK_DIAGNOSTIC_FLOAT_COLUMNS:
                value = optional_float(row, column, row_number)
                if value is not None:
                    chunk_row_present = True
                    chunk_values[column].append(value)
            if chunk_row_present:
                chunk_diagnostics["rows"] = int(chunk_diagnostics["rows"]) + 1

            for arm in ("left", "right"):
                for joint in range(6):
                    actual = parse_float(row[f"{arm}_q_actual_{joint}"], f"{arm}_q_actual_{joint}", row_number)
                    sent = parse_float(row[f"{arm}_q_sent_{joint}"], f"{arm}_q_sent_{joint}", row_number)
                    tracking_errors[arm].append(abs(actual - sent))

                controller = (row.get(f"{arm}_follower_controller") or "").strip()
                delta_columns = (
                    f"{arm}_delta_twist_pending_linear_norm_m",
                    f"{arm}_delta_twist_pending_angular_norm_rad",
                    f"{arm}_delta_twist_step_yaw_rad",
                    f"{arm}_delta_twist_realized_yaw_rad",
                    f"{arm}_delta_twist_realized_yaw_ratio",
                    f"{arm}_delta_twist_xi_ref_angular_norm_rad_s",
                    f"{arm}_delta_twist_xi_cmd_linear_norm_m_s",
                    f"{arm}_delta_twist_xi_cmd_angular_norm_rad_s",
                    f"{arm}_delta_twist_saturated",
                    f"{arm}_delta_twist_frame_rows",
                    f"{arm}_delta_twist_clamp_mask",
                    f"{arm}_delta_twist_accel_cmd_x_m_s2",
                )
                has_delta_columns = any(column in row for column in delta_columns)
                if controller != "delta_twist" and not (has_delta_columns and not controller):
                    continue

                bucket = delta_twist_series[arm]
                bucket["rows"] = int(bucket["rows"]) + 1
                if controller:
                    bump_count(bucket["controller_counts"], controller)  # type: ignore[arg-type]
                active = optional_bool(row, f"{arm}_follower_active", row_number)
                if active:
                    bucket["active_ticks"] = int(bucket["active_ticks"]) + 1
                stall = optional_bool(row, f"{arm}_follower_stall", row_number)
                if stall:
                    bucket["stall_ticks"] = int(bucket["stall_ticks"]) + 1
                saturated = optional_bool(row, f"{arm}_delta_twist_saturated", row_number)
                if saturated:
                    bucket["saturated_ticks"] = int(bucket["saturated_ticks"]) + 1
                for column, key in (
                    (f"{arm}_delta_twist_pending_clamped", "pending_clamped_ticks"),
                    (f"{arm}_delta_twist_xi_ref_clamped_norm", "xi_ref_clamped_ticks"),
                    (f"{arm}_delta_twist_xi_cmd_clamped_norm", "xi_cmd_clamped_ticks"),
                    (f"{arm}_delta_twist_min_time_to_go_used", "min_time_to_go_ticks"),
                ):
                    value = optional_bool(row, column, row_number)
                    if value:
                        bucket[key] = int(bucket[key]) + 1
                feedback_source = optional_float(row, f"{arm}_delta_twist_feedback_source", row_number)
                if feedback_source is not None:
                    feedback_source_int = int(feedback_source)
                    if feedback_source != feedback_source_int:
                        raise AnalysisError(f"row {row_number}: column {arm}_delta_twist_feedback_source must be an integer")
                    bump_count(bucket["feedback_source_counts"], str(feedback_source_int))  # type: ignore[arg-type]
                step = optional_float(row, f"{arm}_follower_step", row_number)
                if step is not None:
                    step_int = int(step)
                    if step != step_int:
                        raise AnalysisError(f"row {row_number}: column {arm}_follower_step must be an integer")
                    bump_count(bucket["step_counts"], str(step_int))  # type: ignore[arg-type]
                step_kind = optional_float(row, f"{arm}_delta_twist_step_kind", row_number)
                if step_kind is not None:
                    step_kind_int = int(step_kind)
                    if step_kind != step_kind_int:
                        raise AnalysisError(f"row {row_number}: column {arm}_delta_twist_step_kind must be an integer")
                    step_kind_name = DELTA_TWIST_STEP_KIND_LABELS.get(step_kind_int, str(step_kind_int))
                    bump_count(bucket["step_kind_counts"], step_kind_name)  # type: ignore[arg-type]
                for suffix, key in (
                    ("frame_rows", "frame_rows"),
                    ("normal_budget", "normal_budget"),
                    ("total_budget", "total_budget"),
                    ("steps_remaining", "steps_remaining"),
                ):
                    value = optional_int(row, f"{arm}_delta_twist_{suffix}", row_number)
                    if value is not None:
                        bucket[key].append(value)  # type: ignore[union-attr]
                clamp_mask = optional_int(row, f"{arm}_delta_twist_clamp_mask", row_number)
                if clamp_mask is not None:
                    if clamp_mask < 0:
                        raise AnalysisError(
                            f"row {row_number}: column {arm}_delta_twist_clamp_mask must be non-negative"
                        )
                    for bit, label in enumerate(DELTA_TWIST_CLAMP_MASK_LABELS):
                        if clamp_mask & (1 << bit):
                            bump_count(bucket["clamp_mask_counts"], label)  # type: ignore[arg-type]
                accel_command = bucket["accel_command"]
                assert isinstance(accel_command, dict)
                for suffix in DELTA_TWIST_ACCEL_COMMAND_SUFFIXES:
                    value = optional_float(
                        row, f"{arm}_delta_twist_accel_cmd_{suffix}", row_number
                    )
                    if value is not None:
                        accel_command[suffix].append(value)
                for column, key in (
                    (f"{arm}_safety_accel_clamped", "safety_accel_clamped"),
                    (f"{arm}_smd_linear_accel_clipped", "smd_linear_accel_clipped"),
                    (f"{arm}_smd_angular_accel_clipped", "smd_angular_accel_clipped"),
                ):
                    value = optional_bool(row, column, row_number)
                    if value:
                        bump_count(bucket["accel_clamp_counts"], key)  # type: ignore[arg-type]
                for column, key in (
                    (f"{arm}_delta_twist_pending_linear_norm_m", "pending_linear"),
                    (f"{arm}_delta_twist_pending_angular_norm_rad", "pending_angular"),
                    (f"{arm}_delta_twist_xi_ref_linear_norm_m_s", "xi_ref_linear"),
                    (f"{arm}_delta_twist_xi_ref_angular_norm_rad_s", "xi_ref_angular"),
                    (f"{arm}_delta_twist_xi_cmd_linear_norm_m_s", "xi_linear"),
                    (f"{arm}_delta_twist_xi_cmd_angular_norm_rad_s", "xi_angular"),
                    (f"{arm}_delta_twist_step_yaw_rad", "requested_yaw"),
                    (f"{arm}_delta_twist_realized_yaw_rad", "realized_yaw"),
                    (f"{arm}_delta_twist_realized_yaw_ratio", "yaw_ratio"),
                    (f"{arm}_delta_twist_realized_linear_ratio", "linear_ratio"),
                    (f"{arm}_delta_twist_realized_angular_ratio", "angular_ratio"),
                ):
                    value = optional_float(row, column, row_number)
                    if value is not None:
                        bucket[key].append(value)  # type: ignore[union-attr]
                stage_xyz = [
                    optional_float(row, f"{arm}_stage_tcp_target_stand_{axis}_m", row_number)
                    for axis in ("x", "y", "z")
                ]
                if all(value is not None for value in stage_xyz):
                    bucket["stage_positions"].append(tuple(float(value) for value in stage_xyz))  # type: ignore[union-attr]

            if all(column in row and row[column] != "" for column in TIMESTAMP_COLUMNS):
                loop_start = parse_float(row["loop_start_time_ns"], "loop_start_time_ns", row_number)
                loop_end = parse_float(row["loop_end_time_ns"], "loop_end_time_ns", row_number)
                if first_loop_start_ns is None:
                    first_loop_start_ns = loop_start
                last_loop_end_ns = loop_end

    if rows == 0:
        raise AnalysisError(f"{path} contains no data rows")

    if first_loop_start_ns is not None and last_loop_end_ns is not None and last_loop_end_ns >= first_loop_start_ns:
        duration_s = (last_loop_end_ns - first_loop_start_ns) / 1_000_000_000.0
        duration_source = "loop timestamps"
    else:
        duration_s = sum(period_ms) / 1000.0
        duration_source = "sum(period_ms)"

    left_error_summary = series_summary(tracking_errors["left"])
    right_error_summary = series_summary(tracking_errors["right"])

    return {
        "rows": rows,
        "duration_s": duration_s,
        "duration_source": duration_source,
        "period_ms": series_summary(period_ms),
        "jitter_ms": series_summary(jitter_ms),
        "send_skew_us": series_summary(send_skew_us),
        "send_duration_us": {
            "left": series_summary(left_send_duration_us),
            "right": series_summary(right_send_duration_us),
        },
        "logger_dropped_samples_max": max(dropped_samples),
        "tracking_error_deg": {
            "left": left_error_summary,
            "right": right_error_summary,
            "max": max(left_error_summary["max"], right_error_summary["max"]),
        },
        "send_failures": {
            "left": send_failures_by_arm["left"],
            "right": send_failures_by_arm["right"],
            "total_arm_failures": send_failures_by_arm["left"] + send_failures_by_arm["right"],
            "rows_with_failure": rows_with_send_failure,
        },
        "safety_verdict_counts": dict(sorted(safety_verdict_counts.items())),
        "chunk_diagnostics": finalize_chunk_diagnostics_bucket(chunk_diagnostics),
        "delta_twist": {
            "left": finalize_delta_twist_bucket(delta_twist_series["left"]),
            "right": finalize_delta_twist_bucket(delta_twist_series["right"]),
        },
    }


def check_budget(metrics: dict[str, object], budget: ProfileBudget) -> list[str]:
    failures: list[str] = []

    def get(path: str) -> float:
        current: object = metrics
        for part in path.split("."):
            if not isinstance(current, dict):
                raise KeyError(path)
            current = current[part]
        return float(current)

    duration_s = get("duration_s")
    if duration_s < budget.min_duration_s:
        failures.append(f"duration_s {duration_s:.6g} < {budget.min_duration_s:.6g}")

    period_mean = get("period_ms.mean")
    lower = budget.target_period_ms - budget.period_tolerance_ms
    upper = budget.target_period_ms + budget.period_tolerance_ms
    if not (lower <= period_mean <= upper):
        failures.append(f"period_ms.mean {period_mean:.6g} outside [{lower:.6g}, {upper:.6g}]")

    checks = [
        ("jitter_ms.p95", budget.jitter_p95_ms),
        ("jitter_ms.max", budget.jitter_max_ms),
        ("send_skew_us.p95", budget.send_skew_p95_us),
    ]
    if budget.send_skew_max_us is not None:
        checks.append(("send_skew_us.max", budget.send_skew_max_us))
    if budget.send_duration_p95_us is not None:
        checks.extend(
            [
                ("send_duration_us.left.p95", budget.send_duration_p95_us),
                ("send_duration_us.right.p95", budget.send_duration_p95_us),
            ]
        )
    if budget.tracking_error_max_deg is not None:
        checks.append(("tracking_error_deg.max", budget.tracking_error_max_deg))

    for path, limit in checks:
        value = get(path)
        if value > limit:
            failures.append(f"{path} {value:.6g} > {limit:.6g}")

    dropped = int(get("logger_dropped_samples_max"))
    if dropped > budget.dropped_samples_max:
        failures.append(f"logger_dropped_samples_max {dropped} > {budget.dropped_samples_max}")

    send_failures = int(get("send_failures.total_arm_failures"))
    if send_failures > budget.send_failure_count_max:
        failures.append(f"send_failures.total_arm_failures {send_failures} > {budget.send_failure_count_max}")

    return failures


def format_report(metrics: dict[str, object], budget: ProfileBudget, failures: Sequence[str]) -> str:
    def get(path: str) -> object:
        current: object = metrics
        for part in path.split("."):
            if not isinstance(current, dict):
                raise KeyError(path)
            current = current[part]
        return current

    def fmt_percentiles(summary: object) -> str:
        if not isinstance(summary, dict) or int(summary.get("count", 0)) == 0:
            return "count=0 p50=n/a p90=n/a p99=n/a"
        return (
            f"count={summary['count']} "
            f"p50={float(summary['p50']):.6f} "
            f"p90={float(summary['p90']):.6f} "
            f"p99={float(summary['p99']):.6f}"
        )

    lines = [
        f"profile: {budget.name}",
        f"verdict: {'FAIL' if failures else 'PASS'}",
        f"rows: {get('rows')}",
        f"duration_s: {float(get('duration_s')):.6f} ({get('duration_source')})",
        f"period_ms: mean={float(get('period_ms.mean')):.6f} p95={float(get('period_ms.p95')):.6f} max={float(get('period_ms.max')):.6f}",
        f"jitter_ms: mean={float(get('jitter_ms.mean')):.6f} p95={float(get('jitter_ms.p95')):.6f} max={float(get('jitter_ms.max')):.6f}",
        f"send_skew_us: mean={float(get('send_skew_us.mean')):.6f} p95={float(get('send_skew_us.p95')):.6f} max={float(get('send_skew_us.max')):.6f}",
        f"left_send_duration_us: mean={float(get('send_duration_us.left.mean')):.6f} p95={float(get('send_duration_us.left.p95')):.6f} max={float(get('send_duration_us.left.max')):.6f}",
        f"right_send_duration_us: mean={float(get('send_duration_us.right.mean')):.6f} p95={float(get('send_duration_us.right.p95')):.6f} max={float(get('send_duration_us.right.max')):.6f}",
        f"logger_dropped_samples_max: {get('logger_dropped_samples_max')}",
        f"tracking_error_deg: left_max={float(get('tracking_error_deg.left.max')):.6f} right_max={float(get('tracking_error_deg.right.max')):.6f} max={float(get('tracking_error_deg.max')):.6f}",
        f"send_failures: left={get('send_failures.left')} right={get('send_failures.right')} total_arm_failures={get('send_failures.total_arm_failures')} rows_with_failure={get('send_failures.rows_with_failure')}",
    ]
    safety_counts = metrics.get("safety_verdict_counts")
    if isinstance(safety_counts, dict) and safety_counts:
        counts = ", ".join(f"{key}={value}" for key, value in safety_counts.items())
        lines.append(f"safety_verdict_counts: {counts}")
    chunk_diagnostics = metrics.get("chunk_diagnostics")
    if isinstance(chunk_diagnostics, dict) and int(chunk_diagnostics.get("rows", 0)) > 0:
        lines.append(
            "chunk_diagnostics: "
            f"rows={chunk_diagnostics['rows']} "
            f"unique_wire_sequences={chunk_diagnostics['unique_wire_sequences']}"
        )
        chunk_series = chunk_diagnostics.get("series")
        if isinstance(chunk_series, dict):
            for column in (*CHUNK_DIAGNOSTIC_INTEGER_COLUMNS, *CHUNK_DIAGNOSTIC_FLOAT_COLUMNS):
                summary = chunk_series.get(column)
                if isinstance(summary, dict) and int(summary.get("count", 0)) > 0:
                    lines.append(f"  {column}: {fmt_percentiles(summary)}")
    delta_twist = metrics.get("delta_twist")
    if isinstance(delta_twist, dict) and any(
        isinstance(delta_twist.get(arm), dict) and int(delta_twist[arm].get("rows", 0)) > 0
        for arm in ("left", "right")
    ):
        lines.append("delta_twist:")
        for arm in ("left", "right"):
            arm_metrics = delta_twist.get(arm)
            if not isinstance(arm_metrics, dict) or int(arm_metrics.get("rows", 0)) <= 0:
                continue
            lines.append(
                f"  {arm}: rows={arm_metrics['rows']} "
                f"active_ticks={arm_metrics['active_ticks']} "
                f"stall_ticks={arm_metrics['stall_ticks']} "
                f"saturated_ticks={arm_metrics['saturated_ticks']} "
                f"pending_clamped_ticks={arm_metrics.get('pending_clamped_ticks', 0)} "
                f"xi_ref_clamped_ticks={arm_metrics.get('xi_ref_clamped_ticks', 0)} "
                f"xi_cmd_clamped_ticks={arm_metrics.get('xi_cmd_clamped_ticks', 0)} "
                f"min_time_to_go_ticks={arm_metrics.get('min_time_to_go_ticks', 0)} "
                f"follower_steps={arm_metrics['follower_step_distribution']} "
                f"step_kind_counts={arm_metrics.get('step_kind_counts', {})} "
                f"step_kind_percent={arm_metrics.get('step_kind_percent', {})}"
            )
            feedback_sources = arm_metrics.get("feedback_source_counts")
            if isinstance(feedback_sources, dict) and feedback_sources:
                lines.append(f"  {arm} feedback_source_counts: {feedback_sources}")
            accel_counts = arm_metrics.get("accel_clamp_counts")
            if isinstance(accel_counts, dict) and accel_counts:
                lines.append(f"  {arm} accel_clamp_counts: {accel_counts}")
            clamp_mask_counts = arm_metrics.get("clamp_mask_counts")
            if isinstance(clamp_mask_counts, dict) and clamp_mask_counts:
                lines.append(f"  {arm} clamp_mask_counts: {clamp_mask_counts}")
            for name in ("frame_rows", "normal_budget", "total_budget", "steps_remaining"):
                summary = arm_metrics.get(name)
                if isinstance(summary, dict) and int(summary.get("count", 0)) > 0:
                    lines.append(f"  {arm} {name}: {fmt_percentiles(summary)}")
            accel_command = arm_metrics.get("accel_command")
            if isinstance(accel_command, dict):
                for suffix in DELTA_TWIST_ACCEL_COMMAND_SUFFIXES:
                    summary = accel_command.get(suffix)
                    if isinstance(summary, dict) and int(summary.get("count", 0)) > 0:
                        lines.append(
                            f"  {arm} accel_cmd_{suffix}: {fmt_percentiles(summary)}"
                        )
            if arm_metrics.get("stage_path_length_m") is not None:
                ratio = arm_metrics.get("stage_path_to_net_ratio")
                ratio_text = "n/a" if ratio is None else f"{float(ratio):.6f}"
                lines.append(
                    f"  {arm} stage_path: length_m={float(arm_metrics['stage_path_length_m']):.6f} "
                    f"net_m={float(arm_metrics['stage_net_displacement_m']):.6f} "
                    f"path_to_net={ratio_text}"
                )
            if arm_metrics.get("yaw_sign_match_percent") is not None:
                lines.append(
                    f"  {arm} yaw_sign_match_percent: "
                    f"{float(arm_metrics['yaw_sign_match_percent']):.3f}"
                )
            warnings = arm_metrics.get("warnings")
            if isinstance(warnings, list) and warnings:
                lines.append(f"  {arm} warnings: " + "; ".join(str(warning) for warning in warnings))
            lines.append(
                f"  {arm} pending_linear_norm_m: "
                f"{fmt_percentiles(arm_metrics.get('pending_residual_linear_norm_m'))}"
            )
            lines.append(
                f"  {arm} pending_angular_norm_rad: "
                f"{fmt_percentiles(arm_metrics.get('pending_residual_angular_norm_rad'))}"
            )
            lines.append(
                f"  {arm} xi_ref_angular_norm_rad_s: "
                f"{fmt_percentiles(arm_metrics.get('xi_ref_angular_norm_rad_s'))}"
            )
            lines.append(
                f"  {arm} xi_cmd_angular_norm_rad_s: "
                f"{fmt_percentiles(arm_metrics.get('commanded_angular_velocity_rad_s'))}"
            )
            lines.append(
                f"  {arm} requested_yaw_delta_rad: "
                f"{fmt_percentiles(arm_metrics.get('requested_yaw_delta_rad'))}"
            )
            lines.append(
                f"  {arm} realized_yaw_delta_rad: "
                f"{fmt_percentiles(arm_metrics.get('realized_yaw_delta_rad'))}"
            )
            lines.append(
                f"  {arm} yaw_realized_ratio: "
                f"{fmt_percentiles(arm_metrics.get('yaw_realized_ratio'))}"
            )
            lines.append(
                f"  {arm} linear_realized_ratio: "
                f"{fmt_percentiles(arm_metrics.get('linear_realized_ratio'))}"
            )
            lines.append(
                f"  {arm} angular_realized_ratio: "
                f"{fmt_percentiles(arm_metrics.get('angular_realized_ratio'))}"
            )
    if failures:
        lines.append("budget_failures:")
        lines.extend(f"- {failure}" for failure in failures)
    return "\n".join(lines)


def write_sample_csv(path: Path, rows: int, period_ms: float) -> None:
    fieldnames = BASE_REQUIRED_COLUMNS + Q_REQUIRED_COLUMNS + TIMESTAMP_COLUMNS
    start_ns = 1_000_000_000
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for i in range(rows):
            row: dict[str, object] = {
                "period_ms": period_ms,
                "jitter_ms": 0.1,
                "send_skew_us": 10.0,
                "left_send_start_ns": start_ns + int(i * period_ms * 1_000_000) + 100_000,
                "left_send_end_ns": start_ns + int(i * period_ms * 1_000_000) + 150_000,
                "right_send_start_ns": start_ns + int(i * period_ms * 1_000_000) + 110_000,
                "right_send_end_ns": start_ns + int(i * period_ms * 1_000_000) + 165_000,
                "left_send_duration_us": 50.0,
                "right_send_duration_us": 55.0,
                "logger_dropped_samples": 0,
                "left_send_ok": "true",
                "right_send_ok": "true",
                "loop_start_time_ns": start_ns + int(i * period_ms * 1_000_000),
                "loop_end_time_ns": start_ns + int((i + 1) * period_ms * 1_000_000),
            }
            for arm in ("left", "right"):
                for joint in range(6):
                    row[f"{arm}_q_actual_{joint}"] = float(joint)
                    row[f"{arm}_q_sent_{joint}"] = float(joint) + 0.25
            writer.writerow(row)


def run_self_test() -> int:
    with tempfile.TemporaryDirectory() as tmpdir:
        mock_log = Path(tmpdir) / "mock.csv"
        rbsim_local_log = Path(tmpdir) / "rbsim-local.csv"
        rbsim_log = Path(tmpdir) / "rbsim.csv"
        write_sample_csv(mock_log, rows=12_000, period_ms=5.0)
        write_sample_csv(rbsim_local_log, rows=200, period_ms=10.0)
        write_sample_csv(rbsim_log, rows=3_000, period_ms=10.0)
        for profile, path in (
            ("mock200", mock_log),
            ("rbsim-local100", rbsim_local_log),
            ("rbsim100", rbsim_log),
        ):
            metrics = analyze_csv(path)
            failures = check_budget(metrics, BUDGETS[profile])
            if failures:
                print(format_report(metrics, BUDGETS[profile], failures), file=sys.stderr)
                return 1
    print("self-test: PASS")
    return 0


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze logs/servo_log.csv against mock200 or rbsim servo budgets.",
    )
    parser.add_argument("log", nargs="?", default="logs/servo_log.csv", help="CSV log path (default: logs/servo_log.csv)")
    parser.add_argument("--profile", choices=sorted(BUDGETS), required=False, default="mock200")
    parser.add_argument("--json", action="store_true", help="emit JSON metrics/verdict instead of text")
    parser.add_argument("--self-test", action="store_true", help="run an internal standalone self-test and exit")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    if args.self_test:
        return run_self_test()

    budget = BUDGETS[args.profile]
    try:
        metrics = analyze_csv(Path(args.log))
        failures = check_budget(metrics, budget)
    except (OSError, AnalysisError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    if args.json:
        print(json.dumps({"profile": budget.name, "passed": not failures, "failures": failures, "metrics": metrics}, indent=2, sort_keys=True))
    else:
        print(format_report(metrics, budget, failures))
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
