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
GRASP_PHASE_LABELS = {
    0: "normal",
    1: "surface_approach",
    2: "pregrasp_commit",
    3: "closing_hold",
    4: "lift_out",
    5: "resume_wait_fresh_chunk",
}
DEFAULT_NEAR_FLOOR_MARGIN_M = 0.012
DEFAULT_SLIDING_THRESHOLD_M = 0.0005
DEFAULT_LEAD_LINEAR_WARN_M = 0.006
DEFAULT_LEAD_ANGULAR_WARN_RAD = 0.025


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
    try:
        number = float(value)
    except ValueError as exc:
        raise AnalysisError(f"row {row_number}: column {column!r} is not numeric: {value!r}") from exc
    if not math.isfinite(number):
        return None
    return number


def optional_bool(row: dict[str, str], column: str, row_number: int) -> bool | None:
    value = row.get(column)
    if value is None or value == "":
        return None
    return parse_bool(value, column, row_number)


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
        "surface_active_ticks": 0,
        "surface_close_soon_ticks": 0,
        "surface_hull_scaled_ticks": 0,
        "surface_mode_counts": {},
        "grasp_commit_active_ticks": 0,
        "grasp_close_soon_ticks": 0,
        "grasp_ready_ticks": 0,
        "grasp_gripper_override_ticks": 0,
        "grasp_policy_delta_dropped_ticks": 0,
        "grasp_resume_wait_ticks": 0,
        "grasp_phase_counts": {},
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
        "surface_min_tip_dist": [],
        "surface_down_scale": [],
        "surface_tangent_scale": [],
        "surface_hull_alpha": [],
        "surface_raw_linear": [],
        "surface_projected_linear": [],
        "surface_discarded_linear": [],
        "surface_raw_angular": [],
        "surface_projected_angular": [],
        "surface_discarded_angular": [],
        "grasp_sync_wait": [],
        "grasp_closing_hold_elapsed": [],
        "grasp_lift_elapsed": [],
        "grasp_lift_progress": [],
        "stage_positions": [],
    }


def make_near_floor_bucket() -> dict[str, object]:
    return {
        "near_floor_ticks": 0,
        "near_floor_duration_s": 0.0,
        "close_soon_ticks": 0,
        "gripper_closing_ticks": 0,
        "sliding_risk_ticks": 0,
        "closing_translation_ticks": 0,
        "projector_inactive_ticks": 0,
        "blocked_ticks": 0,
        "blocked_step_consumed_ticks": 0,
        "commits": 0,
        "grasp_ready_ticks": 0,
        "both_arm_ready_ticks": 0,
        "phase_counts": {},
        "safety_verdict_counts": {},
        "min_tip_dist": [],
        "down_scale": [],
        "tangent_scale": [],
        "raw_linear": [],
        "projected_linear": [],
        "discarded_linear": [],
        "raw_tangent": [],
        "projected_tangent": [],
        "discarded_tangent": [],
        "raw_dz": [],
        "projected_dz": [],
        "discarded_dz": [],
        "lead_linear": [],
        "lead_angular": [],
        "sync_wait": [],
        "closing_hold_elapsed": [],
        "lift_elapsed": [],
        "lift_progress": [],
        "_last_grasp_phase": None,
        "_last_follower_step": None,
    }


def optional_delta_xyz(
    row: dict[str, str],
    arm: str,
    prefix: str,
    row_number: int,
) -> tuple[float, float, float] | None:
    columns = [f"{arm}_{prefix}_{axis}_m" for axis in ("dx", "dy", "dz")]
    values = [optional_float(row, column, row_number) for column in columns]
    if any(value is None for value in values):
        return None
    return (float(values[0]), float(values[1]), float(values[2]))


def tangent_proxy_norm(
    row: dict[str, str],
    arm: str,
    prefix: str,
    fallback_column: str,
    row_number: int,
) -> float | None:
    xyz = optional_delta_xyz(row, arm, prefix, row_number)
    if xyz is not None:
        return math.hypot(xyz[0], xyz[1])
    return optional_float(row, fallback_column, row_number)


def finalize_near_floor_bucket(bucket: dict[str, object]) -> dict[str, object]:
    phase_counts = bucket["phase_counts"]
    safety_counts = bucket["safety_verdict_counts"]
    assert isinstance(phase_counts, dict)
    assert isinstance(safety_counts, dict)
    warnings: list[str] = []
    near_floor_ticks = int(bucket["near_floor_ticks"])
    if near_floor_ticks > 0:
        if int(bucket["projector_inactive_ticks"]) > 0:
            warnings.append("Projector inactive near floor")
        if int(bucket["close_soon_ticks"]) == 0:
            warnings.append("No close_soon detected before floor contact")
        if int(bucket["sliding_risk_ticks"]) > 0 or int(bucket["closing_translation_ticks"]) > 0:
            warnings.append("Sliding risk: arm translation during close")
        if int(bucket["blocked_step_consumed_ticks"]) > 0:
            warnings.append("Safety phase mismatch: step consumed while blocked")
        if not any(key in phase_counts for key in ("pregrasp_commit", "closing_hold", "lift_out")):
            warnings.append("GraspCommit never entered")
        lead_linear = bucket["lead_linear"]
        lead_angular = bucket["lead_angular"]
        assert isinstance(lead_linear, list)
        assert isinstance(lead_angular, list)
        if lead_linear and percentile_nearest_rank(lead_linear, 90.0) > DEFAULT_LEAD_LINEAR_WARN_M:
            warnings.append("High linear lead near floor")
        if lead_angular and percentile_nearest_rank(lead_angular, 90.0) > DEFAULT_LEAD_ANGULAR_WARN_RAD:
            warnings.append("High angular lead near floor")

    return {
        "near_floor_ticks": near_floor_ticks,
        "near_floor_duration_s": bucket["near_floor_duration_s"],
        "close_soon_ticks": bucket["close_soon_ticks"],
        "gripper_closing_ticks": bucket["gripper_closing_ticks"],
        "sliding_risk_ticks": bucket["sliding_risk_ticks"],
        "closing_translation_ticks": bucket["closing_translation_ticks"],
        "projector_inactive_ticks": bucket["projector_inactive_ticks"],
        "blocked_ticks": bucket["blocked_ticks"],
        "blocked_step_consumed_ticks": bucket["blocked_step_consumed_ticks"],
        "commits": bucket["commits"],
        "grasp_ready_ticks": bucket["grasp_ready_ticks"],
        "phase_counts": dict(sorted(phase_counts.items())),
        "safety_verdict_counts": dict(sorted(safety_counts.items())),
        "surface_min_tip_dist_m": percentile_summary(bucket["min_tip_dist"]),  # type: ignore[arg-type]
        "surface_down_scale": percentile_summary(bucket["down_scale"]),  # type: ignore[arg-type]
        "surface_tangent_scale": percentile_summary(bucket["tangent_scale"]),  # type: ignore[arg-type]
        "surface_raw_linear_norm_m": percentile_summary(bucket["raw_linear"]),  # type: ignore[arg-type]
        "surface_projected_linear_norm_m": percentile_summary(bucket["projected_linear"]),  # type: ignore[arg-type]
        "surface_discarded_linear_norm_m": percentile_summary(bucket["discarded_linear"]),  # type: ignore[arg-type]
        "raw_tangent_norm_m": percentile_summary(bucket["raw_tangent"]),  # type: ignore[arg-type]
        "projected_tangent_norm_m": percentile_summary(bucket["projected_tangent"]),  # type: ignore[arg-type]
        "discarded_tangent_norm_m": percentile_summary(bucket["discarded_tangent"]),  # type: ignore[arg-type]
        "raw_dz_m": percentile_summary(bucket["raw_dz"]),  # type: ignore[arg-type]
        "projected_dz_m": percentile_summary(bucket["projected_dz"]),  # type: ignore[arg-type]
        "discarded_dz_m": percentile_summary(bucket["discarded_dz"]),  # type: ignore[arg-type]
        "delta_twist_lead_linear_norm_m": percentile_summary(bucket["lead_linear"]),  # type: ignore[arg-type]
        "delta_twist_lead_angular_norm_rad": percentile_summary(bucket["lead_angular"]),  # type: ignore[arg-type]
        "grasp_sync_wait_sec": percentile_summary(bucket["sync_wait"]),  # type: ignore[arg-type]
        "grasp_closing_hold_elapsed_sec": percentile_summary(bucket["closing_hold_elapsed"]),  # type: ignore[arg-type]
        "grasp_lift_elapsed_sec": percentile_summary(bucket["lift_elapsed"]),  # type: ignore[arg-type]
        "grasp_lift_progress": percentile_summary(bucket["lift_progress"]),  # type: ignore[arg-type]
        "warnings": warnings,
    }


def bump_count(counts: dict[str, int], key: str) -> None:
    counts[key] = counts.get(key, 0) + 1


def finalize_delta_twist_bucket(bucket: dict[str, object]) -> dict[str, object]:
    step_counts = bucket["step_counts"]
    step_kind_counts = bucket["step_kind_counts"]
    controller_counts = bucket["controller_counts"]
    accel_clamp_counts = bucket["accel_clamp_counts"]
    feedback_source_counts = bucket["feedback_source_counts"]
    surface_mode_counts = bucket["surface_mode_counts"]
    grasp_phase_counts = bucket["grasp_phase_counts"]
    assert isinstance(step_counts, dict)
    assert isinstance(step_kind_counts, dict)
    assert isinstance(controller_counts, dict)
    assert isinstance(accel_clamp_counts, dict)
    assert isinstance(feedback_source_counts, dict)
    assert isinstance(surface_mode_counts, dict)
    assert isinstance(grasp_phase_counts, dict)
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
        "surface_active_ticks": bucket["surface_active_ticks"],
        "surface_close_soon_ticks": bucket["surface_close_soon_ticks"],
        "surface_hull_scaled_ticks": bucket["surface_hull_scaled_ticks"],
        "surface_mode_counts": dict(sorted(surface_mode_counts.items())),
        "grasp_commit_active_ticks": bucket["grasp_commit_active_ticks"],
        "grasp_close_soon_ticks": bucket["grasp_close_soon_ticks"],
        "grasp_ready_ticks": bucket["grasp_ready_ticks"],
        "grasp_gripper_override_ticks": bucket["grasp_gripper_override_ticks"],
        "grasp_policy_delta_dropped_ticks": bucket["grasp_policy_delta_dropped_ticks"],
        "grasp_resume_wait_ticks": bucket["grasp_resume_wait_ticks"],
        "grasp_phase_counts": dict(sorted(grasp_phase_counts.items())),
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
        "surface_min_tip_dist_m": percentile_summary(bucket["surface_min_tip_dist"]),  # type: ignore[arg-type]
        "surface_down_scale": percentile_summary(bucket["surface_down_scale"]),  # type: ignore[arg-type]
        "surface_tangent_scale": percentile_summary(bucket["surface_tangent_scale"]),  # type: ignore[arg-type]
        "surface_hull_alpha": percentile_summary(bucket["surface_hull_alpha"]),  # type: ignore[arg-type]
        "surface_raw_linear_norm_m": percentile_summary(bucket["surface_raw_linear"]),  # type: ignore[arg-type]
        "surface_projected_linear_norm_m": percentile_summary(bucket["surface_projected_linear"]),  # type: ignore[arg-type]
        "surface_discarded_linear_norm_m": percentile_summary(bucket["surface_discarded_linear"]),  # type: ignore[arg-type]
        "surface_raw_angular_norm_rad": percentile_summary(bucket["surface_raw_angular"]),  # type: ignore[arg-type]
        "surface_projected_angular_norm_rad": percentile_summary(bucket["surface_projected_angular"]),  # type: ignore[arg-type]
        "surface_discarded_angular_norm_rad": percentile_summary(bucket["surface_discarded_angular"]),  # type: ignore[arg-type]
        "grasp_sync_wait_sec": percentile_summary(bucket["grasp_sync_wait"]),  # type: ignore[arg-type]
        "grasp_closing_hold_elapsed_sec": percentile_summary(bucket["grasp_closing_hold_elapsed"]),  # type: ignore[arg-type]
        "grasp_lift_elapsed_sec": percentile_summary(bucket["grasp_lift_elapsed"]),  # type: ignore[arg-type]
        "grasp_lift_progress": percentile_summary(bucket["grasp_lift_progress"]),  # type: ignore[arg-type]
        "yaw_sign_match_percent": yaw_sign_match_percent,
        "stage_path_length_m": stage_path_length_m,
        "stage_net_displacement_m": stage_net_displacement_m,
        "stage_path_to_net_ratio": stage_path_to_net_ratio,
        "warnings": warnings,
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
        near_floor_series = {
            "left": make_near_floor_bucket(),
            "right": make_near_floor_bucket(),
        }
        near_floor_bimanual: dict[str, object] = {
            "near_floor_ticks": 0,
            "both_ready_ticks": 0,
            "one_ready_wait_ticks": 0,
            "sync_wait": [],
        }
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

            left_min_tip = optional_float(row, "left_surface_min_tip_dist_m", row_number)
            right_min_tip = optional_float(row, "right_surface_min_tip_dist_m", row_number)
            left_near_floor = left_min_tip is not None and left_min_tip < DEFAULT_NEAR_FLOOR_MARGIN_M
            right_near_floor = right_min_tip is not None and right_min_tip < DEFAULT_NEAR_FLOOR_MARGIN_M
            if left_near_floor or right_near_floor:
                near_floor_bimanual["near_floor_ticks"] = int(near_floor_bimanual["near_floor_ticks"]) + 1
                left_ready = optional_bool(row, "left_grasp_ready", row_number)
                right_ready = optional_bool(row, "right_grasp_ready", row_number)
                if left_ready and right_ready:
                    near_floor_bimanual["both_ready_ticks"] = int(near_floor_bimanual["both_ready_ticks"]) + 1
                elif left_ready or right_ready:
                    near_floor_bimanual["one_ready_wait_ticks"] = int(near_floor_bimanual["one_ready_wait_ticks"]) + 1
                for column in ("left_grasp_sync_wait_sec", "right_grasp_sync_wait_sec"):
                    sync_wait = optional_float(row, column, row_number)
                    if sync_wait is not None:
                        near_floor_bimanual["sync_wait"].append(sync_wait)  # type: ignore[union-attr]

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
                    f"{arm}_surface_min_tip_dist_m",
                    f"{arm}_surface_projected_linear_norm_m",
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
                surface_mode = optional_float(row, f"{arm}_surface_mode", row_number)
                if surface_mode is not None:
                    surface_mode_int = int(surface_mode)
                    if surface_mode != surface_mode_int:
                        raise AnalysisError(f"row {row_number}: column {arm}_surface_mode must be an integer")
                    bump_count(bucket["surface_mode_counts"], str(surface_mode_int))  # type: ignore[arg-type]
                grasp_phase = optional_float(row, f"{arm}_grasp_phase", row_number)
                if grasp_phase is not None:
                    grasp_phase_int = int(grasp_phase)
                    if grasp_phase != grasp_phase_int:
                        raise AnalysisError(f"row {row_number}: column {arm}_grasp_phase must be an integer")
                    grasp_phase_name = GRASP_PHASE_LABELS.get(grasp_phase_int, str(grasp_phase_int))
                    bump_count(bucket["grasp_phase_counts"], grasp_phase_name)  # type: ignore[arg-type]
                for column, key in (
                    (f"{arm}_surface_active", "surface_active_ticks"),
                    (f"{arm}_surface_close_soon", "surface_close_soon_ticks"),
                    (f"{arm}_surface_hull_scaled", "surface_hull_scaled_ticks"),
                    (f"{arm}_grasp_commit_active", "grasp_commit_active_ticks"),
                    (f"{arm}_grasp_close_soon", "grasp_close_soon_ticks"),
                    (f"{arm}_grasp_ready", "grasp_ready_ticks"),
                    (f"{arm}_grasp_gripper_override_active", "grasp_gripper_override_ticks"),
                    (f"{arm}_grasp_policy_delta_dropped", "grasp_policy_delta_dropped_ticks"),
                    (f"{arm}_grasp_resume_wait_fresh_chunk", "grasp_resume_wait_ticks"),
                ):
                    value = optional_bool(row, column, row_number)
                    if value:
                        bucket[key] = int(bucket[key]) + 1
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
                    (f"{arm}_surface_min_tip_dist_m", "surface_min_tip_dist"),
                    (f"{arm}_surface_down_scale", "surface_down_scale"),
                    (f"{arm}_surface_tangent_scale", "surface_tangent_scale"),
                    (f"{arm}_surface_hull_alpha", "surface_hull_alpha"),
                    (f"{arm}_surface_raw_linear_norm_m", "surface_raw_linear"),
                    (f"{arm}_surface_projected_linear_norm_m", "surface_projected_linear"),
                    (f"{arm}_surface_discarded_linear_norm_m", "surface_discarded_linear"),
                    (f"{arm}_surface_raw_angular_norm_rad", "surface_raw_angular"),
                    (f"{arm}_surface_projected_angular_norm_rad", "surface_projected_angular"),
                    (f"{arm}_surface_discarded_angular_norm_rad", "surface_discarded_angular"),
                    (f"{arm}_grasp_sync_wait_sec", "grasp_sync_wait"),
                    (f"{arm}_grasp_closing_hold_elapsed_sec", "grasp_closing_hold_elapsed"),
                    (f"{arm}_grasp_lift_elapsed_sec", "grasp_lift_elapsed"),
                    (f"{arm}_grasp_lift_progress", "grasp_lift_progress"),
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

                near_bucket = near_floor_series[arm]
                min_tip = optional_float(row, f"{arm}_surface_min_tip_dist_m", row_number)
                near_floor = min_tip is not None and min_tip < DEFAULT_NEAR_FLOOR_MARGIN_M
                if near_floor:
                    near_bucket["near_floor_ticks"] = int(near_bucket["near_floor_ticks"]) + 1
                    near_bucket["near_floor_duration_s"] = float(near_bucket["near_floor_duration_s"]) + period_ms[-1] / 1000.0
                    near_bucket["min_tip_dist"].append(min_tip)  # type: ignore[union-attr]
                    if safety_verdict:
                        bump_count(near_bucket["safety_verdict_counts"], safety_verdict)  # type: ignore[arg-type]

                    surface_active = optional_bool(row, f"{arm}_surface_active", row_number)
                    if surface_active is False:
                        near_bucket["projector_inactive_ticks"] = int(near_bucket["projector_inactive_ticks"]) + 1

                    close_soon = (
                        optional_bool(row, f"{arm}_gripper_close_soon", row_number) or
                        optional_bool(row, f"{arm}_grasp_close_soon", row_number) or
                        optional_bool(row, f"{arm}_surface_close_soon", row_number)
                    )
                    if close_soon:
                        near_bucket["close_soon_ticks"] = int(near_bucket["close_soon_ticks"]) + 1

                    raw_xyz = optional_delta_xyz(row, arm, "surface_raw", row_number)
                    projected_xyz = optional_delta_xyz(row, arm, "surface_projected", row_number)
                    discarded_xyz = optional_delta_xyz(row, arm, "surface_discarded", row_number)
                    if raw_xyz is not None:
                        near_bucket["raw_tangent"].append(math.hypot(raw_xyz[0], raw_xyz[1]))  # type: ignore[union-attr]
                        near_bucket["raw_dz"].append(raw_xyz[2])  # type: ignore[union-attr]
                    if projected_xyz is not None:
                        near_bucket["projected_dz"].append(projected_xyz[2])  # type: ignore[union-attr]
                    if discarded_xyz is not None:
                        near_bucket["discarded_tangent"].append(math.hypot(discarded_xyz[0], discarded_xyz[1]))  # type: ignore[union-attr]
                        near_bucket["discarded_dz"].append(discarded_xyz[2])  # type: ignore[union-attr]
                    projected_tangent = tangent_proxy_norm(
                        row,
                        arm,
                        "surface_projected",
                        f"{arm}_surface_projected_linear_norm_m",
                        row_number,
                    )
                    if projected_tangent is not None:
                        near_bucket["projected_tangent"].append(projected_tangent)  # type: ignore[union-attr]

                    for column, key in (
                        (f"{arm}_surface_down_scale", "down_scale"),
                        (f"{arm}_surface_tangent_scale", "tangent_scale"),
                        (f"{arm}_surface_raw_linear_norm_m", "raw_linear"),
                        (f"{arm}_surface_projected_linear_norm_m", "projected_linear"),
                        (f"{arm}_surface_discarded_linear_norm_m", "discarded_linear"),
                        (f"{arm}_delta_twist_lead_linear_norm_m", "lead_linear"),
                        (f"{arm}_delta_twist_lead_angular_norm_rad", "lead_angular"),
                        (f"{arm}_grasp_sync_wait_sec", "sync_wait"),
                        (f"{arm}_grasp_closing_hold_elapsed_sec", "closing_hold_elapsed"),
                        (f"{arm}_grasp_lift_elapsed_sec", "lift_elapsed"),
                        (f"{arm}_grasp_lift_progress", "lift_progress"),
                    ):
                        value = optional_float(row, column, row_number)
                        if value is not None:
                            near_bucket[key].append(value)  # type: ignore[union-attr]

                    grasp_phase_value = optional_float(row, f"{arm}_grasp_phase", row_number)
                    grasp_phase_int: int | None = None
                    if grasp_phase_value is not None:
                        grasp_phase_int = int(grasp_phase_value)
                        if grasp_phase_value != grasp_phase_int:
                            raise AnalysisError(f"row {row_number}: column {arm}_grasp_phase must be an integer")
                        grasp_phase_name = GRASP_PHASE_LABELS.get(grasp_phase_int, str(grasp_phase_int))
                        bump_count(near_bucket["phase_counts"], grasp_phase_name)  # type: ignore[arg-type]
                        last_phase = near_bucket["_last_grasp_phase"]
                        if grasp_phase_int in {2, 3, 4} and last_phase not in {2, 3, 4}:
                            near_bucket["commits"] = int(near_bucket["commits"]) + 1
                        near_bucket["_last_grasp_phase"] = grasp_phase_int

                    ready = optional_bool(row, f"{arm}_grasp_ready", row_number)
                    if ready:
                        near_bucket["grasp_ready_ticks"] = int(near_bucket["grasp_ready_ticks"]) + 1

                    gripper_closing = (
                        optional_bool(row, f"{arm}_gripper_closing_hold_active", row_number) or
                        optional_bool(row, f"{arm}_grasp_gripper_override_active", row_number) or
                        (grasp_phase_int in {3, 4} if grasp_phase_int is not None else False)
                    )
                    if gripper_closing:
                        near_bucket["gripper_closing_ticks"] = int(near_bucket["gripper_closing_ticks"]) + 1

                    if close_soon and projected_tangent is not None and projected_tangent > DEFAULT_SLIDING_THRESHOLD_M:
                        near_bucket["sliding_risk_ticks"] = int(near_bucket["sliding_risk_ticks"]) + 1
                    if gripper_closing and projected_tangent is not None and projected_tangent > DEFAULT_SLIDING_THRESHOLD_M:
                        near_bucket["closing_translation_ticks"] = int(near_bucket["closing_translation_ticks"]) + 1

                    blocked = optional_bool(row, f"{arm}_delta_twist_blocked", row_number)
                    step_consumed = optional_bool(row, f"{arm}_delta_twist_step_consumed_this_tick", row_number)
                    follower_step = optional_float(row, f"{arm}_follower_step", row_number)
                    follower_step_int: int | None = None
                    if follower_step is not None:
                        follower_step_int = int(follower_step)
                        if follower_step != follower_step_int:
                            raise AnalysisError(f"row {row_number}: column {arm}_follower_step must be an integer")
                    if blocked:
                        near_bucket["blocked_ticks"] = int(near_bucket["blocked_ticks"]) + 1
                        last_step = near_bucket["_last_follower_step"]
                        step_changed_while_blocked = (
                            step_consumed is None and
                            follower_step_int is not None and
                            last_step is not None and
                            follower_step_int != last_step
                        )
                        if step_consumed or step_changed_while_blocked:
                            near_bucket["blocked_step_consumed_ticks"] = int(near_bucket["blocked_step_consumed_ticks"]) + 1
                    if follower_step_int is not None:
                        near_bucket["_last_follower_step"] = follower_step_int

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
        "delta_twist": {
            "left": finalize_delta_twist_bucket(delta_twist_series["left"]),
            "right": finalize_delta_twist_bucket(delta_twist_series["right"]),
        },
        "near_floor_pick_analysis": {
            "left": finalize_near_floor_bucket(near_floor_series["left"]),
            "right": finalize_near_floor_bucket(near_floor_series["right"]),
            "bimanual": {
                "near_floor_ticks": near_floor_bimanual["near_floor_ticks"],
                "both_ready_ticks": near_floor_bimanual["both_ready_ticks"],
                "one_ready_wait_ticks": near_floor_bimanual["one_ready_wait_ticks"],
                "sync_wait_sec": percentile_summary(near_floor_bimanual["sync_wait"]),  # type: ignore[arg-type]
            },
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
            surface_counts = arm_metrics.get("surface_mode_counts")
            if isinstance(surface_counts, dict) and surface_counts:
                lines.append(
                    f"  {arm} surface: active_ticks={arm_metrics.get('surface_active_ticks', 0)} "
                    f"close_soon_ticks={arm_metrics.get('surface_close_soon_ticks', 0)} "
                    f"hull_scaled_ticks={arm_metrics.get('surface_hull_scaled_ticks', 0)} "
                    f"mode_counts={surface_counts}"
                )
            grasp_counts = arm_metrics.get("grasp_phase_counts")
            if isinstance(grasp_counts, dict) and grasp_counts:
                lines.append(
                    f"  {arm} grasp: commit_active_ticks={arm_metrics.get('grasp_commit_active_ticks', 0)} "
                    f"close_soon_ticks={arm_metrics.get('grasp_close_soon_ticks', 0)} "
                    f"ready_ticks={arm_metrics.get('grasp_ready_ticks', 0)} "
                    f"gripper_override_ticks={arm_metrics.get('grasp_gripper_override_ticks', 0)} "
                    f"policy_delta_dropped_ticks={arm_metrics.get('grasp_policy_delta_dropped_ticks', 0)} "
                    f"resume_wait_ticks={arm_metrics.get('grasp_resume_wait_ticks', 0)} "
                    f"phase_counts={grasp_counts}"
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
            if isinstance(surface_counts, dict) and surface_counts:
                lines.append(
                    f"  {arm} surface_min_tip_dist_m: "
                    f"{fmt_percentiles(arm_metrics.get('surface_min_tip_dist_m'))}"
                )
                lines.append(
                    f"  {arm} surface_down_scale: "
                    f"{fmt_percentiles(arm_metrics.get('surface_down_scale'))}"
                )
                lines.append(
                    f"  {arm} surface_tangent_scale: "
                    f"{fmt_percentiles(arm_metrics.get('surface_tangent_scale'))}"
                )
                lines.append(
                    f"  {arm} surface_discarded_linear_norm_m: "
                    f"{fmt_percentiles(arm_metrics.get('surface_discarded_linear_norm_m'))}"
                )
                lines.append(
                    f"  {arm} surface_discarded_angular_norm_rad: "
                    f"{fmt_percentiles(arm_metrics.get('surface_discarded_angular_norm_rad'))}"
                )
            if isinstance(grasp_counts, dict) and grasp_counts:
                lines.append(
                    f"  {arm} grasp_sync_wait_sec: "
                    f"{fmt_percentiles(arm_metrics.get('grasp_sync_wait_sec'))}"
                )
                lines.append(
                    f"  {arm} grasp_closing_hold_elapsed_sec: "
                    f"{fmt_percentiles(arm_metrics.get('grasp_closing_hold_elapsed_sec'))}"
                )
                lines.append(
                    f"  {arm} grasp_lift_elapsed_sec: "
                    f"{fmt_percentiles(arm_metrics.get('grasp_lift_elapsed_sec'))}"
                )
                lines.append(
                    f"  {arm} grasp_lift_progress: "
                    f"{fmt_percentiles(arm_metrics.get('grasp_lift_progress'))}"
                )
    near_floor = metrics.get("near_floor_pick_analysis")
    if isinstance(near_floor, dict) and any(
        isinstance(near_floor.get(arm), dict) and int(near_floor[arm].get("near_floor_ticks", 0)) > 0
        for arm in ("left", "right")
    ):
        lines.append("Near-floor pick analysis:")
        bimanual = near_floor.get("bimanual")
        if isinstance(bimanual, dict) and int(bimanual.get("near_floor_ticks", 0)) > 0:
            lines.append(
                "  bimanual: "
                f"near_floor_ticks={bimanual.get('near_floor_ticks', 0)} "
                f"both_ready_ticks={bimanual.get('both_ready_ticks', 0)} "
                f"one_ready_wait_ticks={bimanual.get('one_ready_wait_ticks', 0)} "
                f"sync_wait={fmt_percentiles(bimanual.get('sync_wait_sec'))}"
            )
        for arm in ("left", "right"):
            arm_metrics = near_floor.get(arm)
            if not isinstance(arm_metrics, dict) or int(arm_metrics.get("near_floor_ticks", 0)) <= 0:
                continue
            lines.append(
                f"  {arm}: near_floor_ticks={arm_metrics.get('near_floor_ticks', 0)} "
                f"near_floor_duration_s={float(arm_metrics.get('near_floor_duration_s', 0.0)):.6f} "
                f"close_soon_ticks={arm_metrics.get('close_soon_ticks', 0)} "
                f"gripper_closing_ticks={arm_metrics.get('gripper_closing_ticks', 0)} "
                f"sliding_risk_ticks={arm_metrics.get('sliding_risk_ticks', 0)} "
                f"closing_translation_ticks={arm_metrics.get('closing_translation_ticks', 0)} "
                f"blocked_ticks={arm_metrics.get('blocked_ticks', 0)} "
                f"blocked_step_consumed_ticks={arm_metrics.get('blocked_step_consumed_ticks', 0)} "
                f"commits={arm_metrics.get('commits', 0)}"
            )
            safety_counts = arm_metrics.get("safety_verdict_counts")
            if isinstance(safety_counts, dict) and safety_counts:
                lines.append(f"  {arm} near_floor_safety_verdict_counts: {safety_counts}")
            phase_counts = arm_metrics.get("phase_counts")
            if isinstance(phase_counts, dict) and phase_counts:
                lines.append(f"  {arm} near_floor_grasp_phase_counts: {phase_counts}")
            lines.append(
                f"  {arm} near_floor_min_tip_dist_m: "
                f"{fmt_percentiles(arm_metrics.get('surface_min_tip_dist_m'))}"
            )
            lines.append(
                f"  {arm} near_floor_down_scale: "
                f"{fmt_percentiles(arm_metrics.get('surface_down_scale'))}"
            )
            lines.append(
                f"  {arm} near_floor_tangent_scale: "
                f"{fmt_percentiles(arm_metrics.get('surface_tangent_scale'))}"
            )
            lines.append(
                f"  {arm} raw_vs_projected_linear_norm_m: "
                f"raw={fmt_percentiles(arm_metrics.get('surface_raw_linear_norm_m'))} "
                f"projected={fmt_percentiles(arm_metrics.get('surface_projected_linear_norm_m'))} "
                f"discarded={fmt_percentiles(arm_metrics.get('surface_discarded_linear_norm_m'))}"
            )
            lines.append(
                f"  {arm} projected_tangent_norm_m: "
                f"{fmt_percentiles(arm_metrics.get('projected_tangent_norm_m'))}"
            )
            lines.append(
                f"  {arm} action_tangent_norm_m: "
                f"raw={fmt_percentiles(arm_metrics.get('raw_tangent_norm_m'))} "
                f"projected={fmt_percentiles(arm_metrics.get('projected_tangent_norm_m'))} "
                f"discarded={fmt_percentiles(arm_metrics.get('discarded_tangent_norm_m'))}"
            )
            lines.append(
                f"  {arm} action_dz_m: "
                f"raw={fmt_percentiles(arm_metrics.get('raw_dz_m'))} "
                f"projected={fmt_percentiles(arm_metrics.get('projected_dz_m'))} "
                f"discarded={fmt_percentiles(arm_metrics.get('discarded_dz_m'))}"
            )
            lines.append(
                f"  {arm} near_floor_lead: "
                f"linear={fmt_percentiles(arm_metrics.get('delta_twist_lead_linear_norm_m'))} "
                f"angular={fmt_percentiles(arm_metrics.get('delta_twist_lead_angular_norm_rad'))}"
            )
            lines.append(
                f"  {arm} grasp_hold_lift: "
                f"sync_wait={fmt_percentiles(arm_metrics.get('grasp_sync_wait_sec'))} "
                f"closing_hold={fmt_percentiles(arm_metrics.get('grasp_closing_hold_elapsed_sec'))} "
                f"lift_elapsed={fmt_percentiles(arm_metrics.get('grasp_lift_elapsed_sec'))} "
                f"lift_progress={fmt_percentiles(arm_metrics.get('grasp_lift_progress'))}"
            )
            warnings = arm_metrics.get("warnings")
            if isinstance(warnings, list) and warnings:
                lines.append(f"  {arm} near_floor_warnings: " + "; ".join(str(warning) for warning in warnings))
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
