#!/usr/bin/env python3
"""Generate circle benchmark reporting tables and promotion guidance."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import circle_tracking_benchmark as profile_bench
import compare_circle_benchmarks as compare
import generate_rbpodo_measurement_reliability_report as reliability_report


BASELINE_PROFILE = "circle_15cm_16s"
STRESS_PROFILE = "gene_15cm_4s"
BASELINE_RULES = {
    "radius_gain_min": 0.98,
    "rms_error_m_max": 0.005,
    "p95_error_m_max": 0.006,
    "max_orientation_drift_rad_max": 0.005,
}
STRESS_RULES = {
    "radius_gain_min": 0.90,
}
RBPODO_STABLE_RULES = {
    "radius_gain_min": 0.98,
    "radius_gain_max": 1.02,
    "rms_error_m_max": 0.005,
    "p95_error_m_max": 0.007,
}
RBPODO_STRESS_RULES = {
    "radius_gain_min": 0.90,
    "radius_gain_max": 1.10,
}
REPORT_COLUMNS = [
    "run_name",
    "benchmark_category",
    "benchmark_lane",
    "control_loop_location",
    "trajectory_generation_location",
    "feedback_loop_location",
    "low_level_send_mode",
    "backend",
    "controller_mode",
    "controller",
    "arm",
    "profile",
    "tracking_source",
    "kp_pos",
    "kp_ori",
    "state_pub_rate_hz",
    "speed_bar_left",
    "speed_bar_right",
    "diameter_m",
    "period_sec",
    "required_tangential_speed_m_s",
    "stress_level",
    "command_rate_hz",
    "servo_rate_hz",
    "async_mode",
    "acceptance_semantics",
    "controller_ack_observed",
    "sdk_worker_ack_observed",
    "socket_send_only",
    "q_ref_supervised",
    "commands_enqueued_total",
    "commands_sent_total",
    "commands_acked_total",
    "commands_socket_sent_total",
    "commands_overwritten_total",
    "commands_dropped_total",
    "reference_supervision_state",
    "q_ref_target_error_deg_max",
    "repeat_evidence_count",
    "measurement_reliability_level",
    "reliability_caveats",
    "benchmark_interpretation",
    "physical_real_blockers",
    "physical_readiness_status",
    "controller_reference_status",
    "physical_tracking_status",
    "radius_gain",
    "rms_error_mm",
    "median_error_mm",
    "p95_error_mm",
    "max_error_mm",
    "tail_ratio",
    "center_removed_rms_mm",
    "phase_aligned_rms_mm",
    "orientation_position_equiv_50mm_mm",
    "error_classification",
    "p95_orientation_drift_mrad",
    "max_orientation_drift_mrad",
    "estimated_latency_ms",
    "worker_command_drops_total",
    "integrator_clamps_total",
    "integrator_divergence_total",
    "send_success_rate",
    "controller_acceptance_observed_rate",
    "send_duration_p99_us",
    "send_duration_max_us",
    "send_command_deadline_missed_count",
    "deadline_miss_count",
    "command_interval_max_ms",
    "servo_jitter_p99_ms",
    "servo_jitter_max_ms",
    "timing_classification",
    "ack_spike_count_10ms",
    "ack_spike_count_20ms",
    "state_gap_count",
    "command_gap_count",
    "p95_error_near_ack_spike_mm",
    "p95_error_away_from_ack_spike_mm",
    "p95_error_near_command_gap_mm",
    "p95_error_away_from_command_gap_mm",
    "physical_motion_expected",
    "physical_motion_detected",
    "q_ref_update_rate_hz",
    "q_ref_valid_ratio",
    "q_actual_update_rate_hz",
    "ack_policy",
    "controller_acceptance_observed_count",
    "command_timeout_count",
    "controller_rejected_count",
    "tcp_ref_valid_ratio",
    "tcp_actual_valid_ratio",
    "diagnostics_suspect_count",
    "saturation_ratio",
    "orientation_p95_deg",
    "center_error_mm",
    "score",
    "result",
    "run_result_status",
    "benchmark_threshold_status",
    "ackon500_goal_status",
    "diagnostic_warning_count",
    "result_reason",
    "server_rejected_cartesian",
    "cartesian_unavailable_count",
    "classification",
    "real_candidate_policy",
    "performance_warnings",
    "promotion_notes",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate a markdown and CSV report for circle benchmark "
            "summary.json artifacts, including baseline/stress classification and "
            "real-candidate parameter policy."
        )
    )
    parser.add_argument("summary_json", nargs="*", type=Path, help="Circle benchmark summary.json files")
    parser.add_argument("--ablation-summary-csv", type=Path, help="Optional ablation_summary.csv input")
    parser.add_argument("--output-md", type=Path, help="Write markdown report to this path instead of stdout")
    parser.add_argument("--csv", dest="csv_path", type=Path, help="Optional CSV table output path")
    parser.add_argument("--min-baseline-repeats", type=int, default=3)
    parser.add_argument("--title", default="Circle Benchmark Report")
    return parser.parse_args()


def finite_number(value: Any) -> float | None:
    return compare.finite_number(value)


def as_bool(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"true", "1", "yes"}:
            return True
        if lowered in {"false", "0", "no"}:
            return False
    return None


def load_ablation_rows(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            row["_source"] = str(path)
            row["run_name"] = row.get("name") or row.get("run_name")
            row["performance_warnings"] = row.get("warnings") or row.get("performance_warnings")
            row["run_result_status"] = row.get("run_result_status") or compare.infer_run_result_status(row)
            row["benchmark_threshold_status"] = row.get("benchmark_threshold_status") or compare.infer_benchmark_threshold_status(row)
            row["ackon500_goal_status"] = row.get("ackon500_goal_status") or compare.infer_ackon500_goal_status(row)
            if not row.get("diagnostic_warning_count"):
                row["diagnostic_warning_count"] = compare.diagnostic_warning_count(row)
            row.update(compare.async_report_fields(row))
            if not row.get("kp_pos"):
                row["kp_pos"] = row.get("feedback_kp_pos")
            if not row.get("kp_ori"):
                row["kp_ori"] = row.get("feedback_kp_ori")
            if not row.get("orientation_p95_deg") and row.get("p95_orientation_drift_mrad"):
                orientation_mrad = finite_number(row.get("p95_orientation_drift_mrad"))
                if orientation_mrad is not None:
                    row["orientation_p95_deg"] = orientation_mrad / 1000.0 * 180.0 / math.pi
                    row["p95_orientation_drift_rad"] = orientation_mrad / 1000.0
            if not row.get("center_error_mm") and row.get("fit_center_error_mm"):
                row["center_error_mm"] = row.get("fit_center_error_mm")
                center_mm = finite_number(row.get("fit_center_error_mm"))
                if center_mm is not None:
                    row["fit_center_error_m"] = center_mm / 1000.0
            if row.get("ack_policy") or row.get("q_ref_update_rate_hz") or row.get("tracking_source"):
                row.setdefault("backend", "rbpodo")
                row.setdefault("benchmark_category", "rbpodo_controller_simulation")
                row.setdefault("controller_mode", "pgmode_simulation")
                row.setdefault("physical_motion_expected", "false")
            else:
                row.setdefault("backend", "simulator")
                row.setdefault("benchmark_category", "rb_simulator")
                row.setdefault("controller_mode", "rb_simulator")
            if not row.get("required_tangential_speed_m_s"):
                speed = compare.required_tangential_speed(row)
                if speed is not None:
                    row["required_tangential_speed_m_s"] = speed
            if not row.get("stress_level"):
                row["stress_level"] = compare.stress_level(row)
            if not row.get("send_success_rate"):
                send_count = finite_number(row.get("send_count") or row.get("command_count"))
                success_count = finite_number(row.get("send_success_count"))
                failure_count = finite_number(row.get("send_failure_count"))
                if success_count is not None and send_count is not None and send_count > 0:
                    row["send_success_rate"] = success_count / send_count
                elif failure_count is not None and send_count is not None and send_count > 0:
                    row["send_success_rate"] = max(0.0, 1.0 - failure_count / send_count)
            if not row.get("controller_acceptance_observed_rate"):
                send_count = finite_number(row.get("send_count") or row.get("command_count"))
                acceptance_count = finite_number(row.get("controller_acceptance_observed_count"))
                if acceptance_count is not None and send_count is not None and send_count > 0:
                    row["controller_acceptance_observed_rate"] = acceptance_count / send_count
            profile_bench.apply_canonical_lane_metadata(row)
            rows.append(row)
    return rows


def load_rows(summary_paths: list[Path], ablation_csv: Path | None) -> list[dict[str, Any]]:
    rows = [compare.comparison_row(compare.load_summary(path)) for path in summary_paths]
    if ablation_csv is not None:
        rows.extend(load_ablation_rows(ablation_csv))
    return rows


def repeat_count(row: dict[str, Any]) -> int:
    repeat = finite_number(row.get("repeat"))
    if repeat is not None and repeat >= 1:
        return int(repeat)
    return 1


def row_group_key(row: dict[str, Any]) -> tuple[Any, ...]:
    return (
        row.get("profile"),
        row.get("controller"),
        row.get("arm"),
        row.get("diameter_m"),
        row.get("period_sec"),
    )


def annotate_repeat_evidence(rows: list[dict[str, Any]]) -> None:
    group_counts: dict[tuple[Any, ...], int] = {}
    for row in rows:
        group_counts[row_group_key(row)] = group_counts.get(row_group_key(row), 0) + repeat_count(row)
    for row in rows:
        row["repeat_evidence_count"] = group_counts.get(row_group_key(row), repeat_count(row))


def zero_metric(row: dict[str, Any], key: str) -> bool:
    value = finite_number(row.get(key))
    return value is not None and value == 0.0


def false_metric(row: dict[str, Any], key: str) -> bool:
    value = row.get(key)
    parsed = as_bool(value)
    if parsed is not None:
        return parsed is False
    return value in (None, "")


def true_metric(row: dict[str, Any], key: str) -> bool:
    value = row.get(key)
    parsed = as_bool(value)
    return parsed is True


def explicit_false_metric(row: dict[str, Any], key: str) -> bool:
    value = row.get(key)
    parsed = as_bool(value)
    return parsed is False


def metric_le(row: dict[str, Any], key: str, limit: float) -> bool:
    value = finite_number(row.get(key))
    return value is not None and value <= limit


def metric_ge(row: dict[str, Any], key: str, limit: float) -> bool:
    value = finite_number(row.get(key))
    return value is not None and value >= limit


def metric_between(row: dict[str, Any], key: str, low: float, high: float) -> bool:
    value = finite_number(row.get(key))
    return value is not None and low <= value <= high


def metric_present(row: dict[str, Any], key: str) -> bool:
    return finite_number(row.get(key)) is not None


def zero_or_missing_metric(row: dict[str, Any], key: str) -> bool:
    value = finite_number(row.get(key))
    return value is None or value == 0.0


def positive_metric(row: dict[str, Any], key: str) -> bool:
    value = finite_number(row.get(key))
    return value is not None and value > 0.0


def row_category(row: dict[str, Any]) -> str:
    category = str(row.get("benchmark_category") or "")
    if category:
        return category
    backend = str(row.get("backend") or "")
    controller_mode = str(row.get("controller_mode") or "")
    if backend == "rbpodo" and controller_mode == "pgmode_simulation":
        return "rbpodo_controller_simulation"
    if backend == "simulator" or controller_mode == "rb_simulator":
        return "rb_simulator"
    if true_metric(row, "physical_motion_expected"):
        return "real_physical_benchmark"
    return "unknown"


def first_present(row: dict[str, Any], *keys: str) -> Any:
    for key in keys:
        if key in row and row.get(key) is not None:
            return row.get(key)
    return None


def first_number(row: dict[str, Any], *keys: str) -> float | None:
    for key in keys:
        value = finite_number(row.get(key))
        if value is not None:
            return value
    return None


def normalize_rbpodo_tuning_fields(row: dict[str, Any]) -> None:
    kp_pos = first_present(row, "kp_pos", "feedback_kp_pos")
    kp_ori = first_present(row, "kp_ori", "feedback_kp_ori")
    if kp_pos is not None:
        row["kp_pos"] = kp_pos
        row.setdefault("feedback_kp_pos", kp_pos)
    if kp_ori is not None:
        row["kp_ori"] = kp_ori
        row.setdefault("feedback_kp_ori", kp_ori)

    orientation_rad = first_number(row, "p95_orientation_drift_rad")
    if orientation_rad is None:
        orientation_mrad = first_number(row, "p95_orientation_drift_mrad")
        if orientation_mrad is not None:
            orientation_rad = orientation_mrad / 1000.0
            row["p95_orientation_drift_rad"] = orientation_rad
    if orientation_rad is None:
        orientation_deg = first_number(row, "orientation_p95_deg")
        if orientation_deg is not None:
            orientation_rad = orientation_deg * math.pi / 180.0
            row["p95_orientation_drift_rad"] = orientation_rad
    if orientation_rad is not None:
        row["orientation_p95_deg"] = orientation_rad * 180.0 / math.pi

    center_m = first_number(row, "fit_center_error_m")
    if center_m is None:
        center_mm = first_number(row, "center_error_mm", "fit_center_error_mm")
        if center_mm is not None:
            center_m = center_mm / 1000.0
            row["fit_center_error_m"] = center_m
    if center_m is not None:
        row["center_error_mm"] = center_m * 1000.0

    saturation_count = first_number(row, "feedback_saturation_count")
    command_count = first_number(
        row,
        "command_count",
        "ack_observed_count",
        "controller_acceptance_observed_count",
    )
    if saturation_count is not None and command_count is not None and command_count > 0.0:
        row["saturation_ratio"] = saturation_count / command_count
    elif "saturation_ratio" not in row:
        row["saturation_ratio"] = None


def is_feedback_controller(row: dict[str, Any]) -> bool:
    return str(row.get("controller") or "").endswith("_feedback")


def is_rbpodo_stress_row(row: dict[str, Any]) -> bool:
    return row.get("profile") == STRESS_PROFILE


def state_pub_speed_mismatch(row: dict[str, Any]) -> bool:
    state_pub_rate = first_number(row, "state_pub_rate_hz")
    speed_left = first_number(row, "speed_bar_left")
    speed_right = first_number(row, "speed_bar_right")
    speed_values = [value for value in (speed_left, speed_right) if value is not None]
    return (
        is_rbpodo_stress_row(row)
        and state_pub_rate is not None
        and state_pub_rate >= 100.0
        and bool(speed_values)
        and max(speed_values) <= 0.1
    )


def rbpodo_tuning_rejection_reasons(row: dict[str, Any]) -> list[str]:
    reasons: list[str] = []
    if true_metric(row, "fault_latched"):
        reasons.append("fault_latched=true")
    if true_metric(row, "physical_motion_detected"):
        reasons.append("physical_motion_detected=true")
    reference_state = str(row.get("reference_supervision_state") or "")
    if reference_state in {"fault", "failed", "watchdog_failed"}:
        reasons.append(f"reference_supervision_state={reference_state}")
    timing = str(row.get("timing_classification") or "")
    if timing and timing != "clean_timing":
        reasons.append(f"timing_classification={timing}")
    unavailable_count = first_number(row, "cartesian_unavailable_count")
    if unavailable_count is not None and unavailable_count > 0.0:
        reasons.append("cartesian_unavailable_count > 0")
    saturation = first_number(row, "saturation_ratio")
    if saturation is not None and saturation > 0.2:
        reasons.append("feedback_saturation_count > 0.2 * command_count")
    orientation_rad = first_number(row, "p95_orientation_drift_rad")
    if orientation_rad is not None and orientation_rad > 0.25:
        reasons.append("p95_orientation_drift_rad > 0.25")
    center_m = first_number(row, "fit_center_error_m")
    if is_rbpodo_stress_row(row) and center_m is not None and center_m > 0.05:
        reasons.append("fit_center_error_m > 0.05 for 4s")
    radius = first_number(row, "radius_gain")
    if radius is not None and not (0.85 <= radius <= 1.15):
        reasons.append("radius_gain outside [0.85, 1.15]")
    return reasons


def rbpodo_tuning_score(row: dict[str, Any]) -> float | None:
    metric_keys = ("radius_gain", "rms_error_mm", "p95_error_mm")
    if not any(first_number(row, key) is not None for key in metric_keys):
        return None

    score = 0.0
    saturation = first_number(row, "saturation_ratio") or 0.0
    center_mm = first_number(row, "center_error_mm")
    orientation_deg = first_number(row, "orientation_p95_deg")
    radius = first_number(row, "radius_gain")
    rms_mm = first_number(row, "rms_error_mm")
    p95_mm = first_number(row, "p95_error_mm")

    score += saturation * 10000.0
    score += (center_mm if center_mm is not None else 100.0) * 5.0
    score += (orientation_deg if orientation_deg is not None else 30.0) * 3.0
    score += (abs(radius - 1.0) if radius is not None else 0.5) * 200.0
    score += (rms_mm if rms_mm is not None else 250.0) * 0.30
    score += (p95_mm if p95_mm is not None else 250.0) * 0.15

    for reason in rbpodo_tuning_rejection_reasons(row):
        if reason in {"fault_latched=true", "physical_motion_detected=true", "cartesian_unavailable_count > 0"}:
            score += 100000.0
        elif reason == "feedback_saturation_count > 0.2 * command_count":
            score += 50000.0
        elif reason == "p95_orientation_drift_rad > 0.25":
            score += 40000.0
        elif reason == "fit_center_error_m > 0.05 for 4s":
            score += 30000.0
        elif reason == "radius_gain outside [0.85, 1.15]":
            score += 20000.0
    if not is_feedback_controller(row):
        score += 50000.0
    return score


def missing_rbpodo_candidate_metrics(row: dict[str, Any]) -> list[str]:
    missing: list[str] = []
    for key in ("radius_gain", "rms_error_mm", "p95_error_mm"):
        if first_number(row, key) is None:
            missing.append(key)
    if is_feedback_controller(row) and first_number(row, "saturation_ratio") is None:
        missing.append("saturation_ratio")
    return missing


def baseline_failures(row: dict[str, Any], min_repeats: int) -> list[str]:
    failures: list[str] = []
    if row.get("profile") != BASELINE_PROFILE:
        failures.append(f"profile is not {BASELINE_PROFILE}")
    if not metric_ge(row, "radius_gain", BASELINE_RULES["radius_gain_min"]):
        failures.append("radius_gain < 0.98 or missing")
    if not metric_le(row, "rms_error_mm", BASELINE_RULES["rms_error_m_max"] * 1000.0):
        failures.append("rms_error_m > 0.005 or missing")
    if not metric_le(row, "p95_error_mm", BASELINE_RULES["p95_error_m_max"] * 1000.0):
        failures.append("p95_error_m > 0.006 or missing")
    if not metric_le(
        row,
        "max_orientation_drift_mrad",
        BASELINE_RULES["max_orientation_drift_rad_max"] * 1000.0,
    ):
        failures.append("max_orientation_drift_rad > 0.005 or missing")
    if not zero_metric(row, "worker_command_drops_total"):
        failures.append("worker_command_drops_total != 0 or missing")
    if not zero_metric(row, "send_command_deadline_missed_count"):
        failures.append("send_command_deadline_missed_count != 0 or missing")
    if not false_metric(row, "fault_latched"):
        failures.append("fault_latched is true or unknown")
    if int(row.get("repeat_evidence_count") or 0) < min_repeats:
        failures.append(f"repeat evidence < {min_repeats}")
    return failures


def rbpodo_stable_failures(row: dict[str, Any]) -> list[str]:
    failures: list[str] = []
    if true_metric(row, "server_rejected_cartesian") or row.get("run_result_status") == "blocked" or row.get("result") == "blocked":
        failures.append("Cartesian commands were rejected by server before path execution")
    if row_category(row) != "rbpodo_controller_simulation":
        failures.append("category is not rbpodo_controller_simulation")
    if row.get("backend") != "rbpodo":
        failures.append("backend is not rbpodo")
    if row.get("controller_mode") != "pgmode_simulation":
        failures.append("controller_mode is not pgmode_simulation")
    if row.get("profile") != BASELINE_PROFILE:
        failures.append(f"profile is not {BASELINE_PROFILE}")
    if row.get("tracking_source") != "tcp_ref_stand":
        failures.append("tracking_source is not tcp_ref_stand")
    if not explicit_false_metric(row, "physical_motion_expected"):
        failures.append("physical_motion_expected is not false")
    if not explicit_false_metric(row, "physical_motion_detected"):
        failures.append("physical_motion_detected is true or unknown")
    if not explicit_false_metric(row, "fault_latched"):
        failures.append("fault_latched is true or unknown")
    if not metric_between(
        row,
        "radius_gain",
        RBPODO_STABLE_RULES["radius_gain_min"],
        RBPODO_STABLE_RULES["radius_gain_max"],
    ):
        failures.append("radius_gain outside [0.98, 1.02] or missing")
    if not metric_le(row, "rms_error_mm", RBPODO_STABLE_RULES["rms_error_m_max"] * 1000.0):
        failures.append("rms_error_m > 0.005 or missing")
    if not metric_le(row, "p95_error_mm", RBPODO_STABLE_RULES["p95_error_m_max"] * 1000.0):
        failures.append("p95_error_m > 0.007 or missing")
    for key in ("worker_command_drops_total", "send_command_deadline_missed_count"):
        if not zero_or_missing_metric(row, key):
            failures.append(f"{key} != 0")
    for key in ("command_timeout_count", "controller_rejected_count"):
        if not zero_metric(row, key):
            failures.append(f"{key} != 0 or missing")
    if row.get("ack_policy") != "ack_on":
        failures.append("stable controller-simulation baseline requires ACK-on")
    elif not positive_metric(row, "controller_acceptance_observed_count"):
        failures.append("controller_acceptance_observed_count missing or zero for ACK-on")
    return failures


def stress_failures(row: dict[str, Any]) -> list[str]:
    failures: list[str] = []
    if row.get("profile") != STRESS_PROFILE:
        failures.append(f"profile is not {STRESS_PROFILE}")
    if not metric_ge(row, "radius_gain", STRESS_RULES["radius_gain_min"]):
        failures.append("radius_gain < 0.90 or missing")
    if not false_metric(row, "fault_latched"):
        failures.append("fault_latched is true or unknown")
    if not zero_metric(row, "worker_command_drops_total"):
        failures.append("worker_command_drops_total != 0 or missing")
    if not zero_metric(row, "integrator_divergence_total"):
        failures.append("integrator_divergence_total != 0 or missing")
    return failures


def rbpodo_stress_failures(row: dict[str, Any]) -> list[str]:
    failures: list[str] = []
    if true_metric(row, "server_rejected_cartesian") or row.get("run_result_status") == "blocked" or row.get("result") == "blocked":
        failures.append("Cartesian commands were rejected by server before path execution")
    if row_category(row) != "rbpodo_controller_simulation":
        failures.append("category is not rbpodo_controller_simulation")
    if row.get("backend") != "rbpodo":
        failures.append("backend is not rbpodo")
    if row.get("controller_mode") != "pgmode_simulation":
        failures.append("controller_mode is not pgmode_simulation")
    if row.get("profile") != STRESS_PROFILE:
        failures.append(f"profile is not {STRESS_PROFILE}")
    if not metric_between(
        row,
        "radius_gain",
        RBPODO_STRESS_RULES["radius_gain_min"],
        RBPODO_STRESS_RULES["radius_gain_max"],
    ):
        failures.append("radius_gain outside [0.90, 1.10] or missing")
    if not metric_present(row, "rms_error_mm"):
        failures.append("rms_error_m missing")
    if not metric_present(row, "p95_error_mm"):
        failures.append("p95_error_m missing")
    if str(row.get("controller") or "").endswith("_feedback") and not metric_present(row, "feedback_saturation_count"):
        failures.append("feedback_saturation_count missing for feedback controller")
    if not explicit_false_metric(row, "physical_motion_expected"):
        failures.append("physical_motion_expected is not false")
    if not explicit_false_metric(row, "physical_motion_detected"):
        failures.append("physical_motion_detected is true or unknown")
    if not explicit_false_metric(row, "fault_latched"):
        failures.append("fault_latched is true or unknown")
    return failures


def classify_rbpodo_tuning_row(row: dict[str, Any]) -> None:
    normalize_rbpodo_tuning_fields(row)
    row["score"] = rbpodo_tuning_score(row)
    reasons = rbpodo_tuning_rejection_reasons(row)
    missing = missing_rbpodo_candidate_metrics(row)

    if true_metric(row, "server_rejected_cartesian") or row.get("run_result_status") == "blocked" or row.get("result") == "blocked":
        row["classification"] = "rbpodo_controller_sim_cartesian_blocked"
        row["real_candidate_policy"] = "not_real_ready"
        row["promotion_notes"] = (
            "server rejected Cartesian commands before attempting path; check "
            "cartesian_control.allow_in_controller_simulation, "
            "RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN, operation_mode=simulation, "
            "and pgmode simulation confirmation"
        )
        return

    if not is_feedback_controller(row):
        row["classification"] = "open_loop_baseline"
        row["real_candidate_policy"] = "not_real_ready"
        notes = [
            "open-loop radius can be good while center drift is bad",
            "closed-loop is structurally needed for rbpodo controller simulation",
        ]
        if reasons:
            notes.append("rejected for candidate ranking: " + "; ".join(reasons))
        row["promotion_notes"] = "; ".join(notes)
        return

    if state_pub_speed_mismatch(row):
        row["classification"] = "state_pub_speed_mismatch"
    elif first_number(row, "saturation_ratio") is not None and first_number(row, "saturation_ratio") > 0.2:
        row["classification"] = "saturation_limited"
    elif first_number(row, "p95_orientation_drift_rad") is not None and first_number(row, "p95_orientation_drift_rad") > 0.25:
        row["classification"] = "orientation_unstable"
    elif (
        is_rbpodo_stress_row(row)
        and first_number(row, "fit_center_error_m") is not None
        and first_number(row, "fit_center_error_m") > 0.05
    ):
        row["classification"] = "center_drift_limited"
    elif reasons or missing:
        row["classification"] = "stress_only"
    else:
        row["classification"] = "closed_loop_candidate"

    policy = "controller_sim_stress_not_real_ready" if is_rbpodo_stress_row(row) else "future_low_speed_seed_only_not_real_ready"
    row["real_candidate_policy"] = policy
    notes: list[str] = []
    if row["classification"] == "closed_loop_candidate":
        notes.append("closed-loop candidate for rbpodo controller-simulation tuning; not physical real evidence")
    elif row["classification"] == "state_pub_speed_mismatch":
        notes.append("pub_rate=100 with speed_bar=0.1 can destabilize; speed_bar>=0.2 can restore stability")
    elif row["classification"] == "saturation_limited":
        notes.append("feedback saturation dominates this row")
    elif row["classification"] == "orientation_unstable":
        notes.append("orientation drift exceeds the tuning cutoff")
    elif row["classification"] == "center_drift_limited":
        notes.append("center drift makes the radius metric misleading")
    elif row["classification"] == "stress_only":
        notes.append("stress-only or incomplete tuning evidence; do not promote as a candidate")
    if reasons:
        notes.append("rejected or heavily penalized: " + "; ".join(reasons))
    if missing:
        notes.append("missing candidate metrics: " + ", ".join(missing))
    error_classification = row.get("error_classification")
    if error_classification:
        notes.append(f"error decomposition: {error_classification}")
    row["promotion_notes"] = "; ".join(notes)


def classify_row(row: dict[str, Any], min_repeats: int) -> None:
    category = row_category(row)
    row["benchmark_category"] = category
    if not row.get("required_tangential_speed_m_s"):
        speed = compare.required_tangential_speed(row)
        if speed is not None:
            row["required_tangential_speed_m_s"] = speed
    if not row.get("stress_level"):
        row["stress_level"] = compare.stress_level(row)
    if category == "rbpodo_controller_simulation" and row.get("backend") in (None, ""):
        row["backend"] = "rbpodo"
    if category == "rbpodo_controller_simulation" and row.get("controller_mode") in (None, ""):
        row["controller_mode"] = "pgmode_simulation"
    if category == "rb_simulator" and row.get("backend") in (None, ""):
        row["backend"] = "simulator"
    profile_bench.apply_canonical_lane_metadata(row)
    baseline = baseline_failures(row, min_repeats)
    stress = stress_failures(row)
    if category == "rbpodo_controller_simulation":
        classify_rbpodo_tuning_row(row)
        return
    if category == "real_physical_benchmark":
        row["classification"] = "real_physical_benchmark_future_or_unreviewed"
        row["real_candidate_policy"] = "requires_separate_real_physical_acceptance"
        row["promotion_notes"] = "real physical circle reporting is future scope; do not infer approval from this report"
    elif row.get("profile") == BASELINE_PROFILE and not baseline:
        row["classification"] = "stable_simulator_baseline_candidate"
        row["real_candidate_policy"] = "simulator_seed_only_after_real_acceptance"
        row["promotion_notes"] = "meets simulator baseline criteria; still not real-ready"
    elif row.get("profile") == STRESS_PROFILE and not stress:
        row["classification"] = "stress_benchmark_candidate"
        row["real_candidate_policy"] = "stress_only_not_real_ready"
        row["promotion_notes"] = "GENE-style stress evidence; do not copy speed/gains directly to real"
    elif row.get("profile") == BASELINE_PROFILE:
        row["classification"] = "not_baseline_candidate"
        row["real_candidate_policy"] = "not_real_ready"
        row["promotion_notes"] = "; ".join(baseline)
    elif row.get("profile") == STRESS_PROFILE:
        row["classification"] = "stress_rejected_or_incomplete"
        row["real_candidate_policy"] = "stress_only_not_real_ready"
        row["promotion_notes"] = "; ".join(stress)
    else:
        row["classification"] = "informational"
        row["real_candidate_policy"] = "not_real_ready"
        row["promotion_notes"] = "not a baseline or GENE-style stress profile"


def classify_rows(rows: list[dict[str, Any]], min_repeats: int = 3) -> list[dict[str, Any]]:
    normalized = [dict(row) for row in rows]
    annotate_repeat_evidence(normalized)
    for row in normalized:
        classify_row(row, min_repeats)
        reliability_report.annotate_row(row)
    return normalized


def format_cell(value: Any) -> str:
    return compare.format_cell(value)


def markdown_table(rows: list[dict[str, Any]]) -> str:
    lines = [
        "| " + " | ".join(REPORT_COLUMNS) + " |",
        "| " + " | ".join("---" for _ in REPORT_COLUMNS) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(format_cell(row.get(key)) for key in REPORT_COLUMNS) + " |")
    return "\n".join(lines)


def criteria_markdown(min_repeats: int) -> str:
    return f"""## Evidence Categories

Reports keep these categories separate:

- `rb_simulator`: hardware-free simulator benchmark evidence.
- `rbpodo_controller_simulation`: rbpodo path through real Rainbow controller boxes in `pgmode` simulation.
- `real_physical_benchmark`: future physical-motion evidence; this report does not create or approve it.

**{reliability_report.ACKON500_PHYSICAL_WARNING}**

Transition ladder before fast physical circles:

1. Controller pgmode simulation repeatability
2. Right arm
3. Dual arm
4. P0 diagnostics root cause
5. Real controller read-only
6. Tiny physical acceptance
7. Slow physical circle
8. Fast physical circle only after approval

Required controller-reference report boundary fields:

```yaml
physical_readiness:
  status: blocked
  blockers:
    - diagnostics_suspect_unresolved
    - physical_reference_to_actual_error_unmeasured
    - stop_resetFault_unverified
    - camera_tcp_calibration_unresolved
    - no_tiny_physical_acceptance
  next_required_acceptance:
    - read-only diagnostics parity
    - tiny joint no-op physical or approved safe mode
    - tiny physical joint move
    - tiny physical Cartesian move
    - low-speed circle
    - then speed ladder
controller_reference_result:
  status: pass|fail
  explanation: "tcp_ref_stand lower-bound evidence"
physical_tracking_result:
  status: not_measured
```

## Simulator Baseline Promotion Criteria

A stable simulator baseline candidate must satisfy all of:

- profile `{BASELINE_PROFILE}`
- `radius_gain >= 0.98`
- `rms_error_m <= 0.005`
- `p95_error_m <= 0.006`
- `max_orientation_drift_rad <= 0.005`
- `worker_command_drops_total == 0`
- `send_command_deadline_missed_count == 0`
- `fault_latched == false`
- repeated at least `{min_repeats}` times, either through one summary with `repeat >= {min_repeats}` or matching repeated summaries

## Stress Interpretation

A `circle_15cm_8s` run is middle-speed ablation evidence. It is intended to
separate bandwidth, latency, and limit constraints from basic 15 cm tracking
errors before attempting the GENE-style 4 s stress profile.

A `{STRESS_PROFILE}` run can be a stress benchmark candidate when `radius_gain >= 0.90`,
`fault_latched == false`, worker drops are zero, and integrator divergence is zero.
Stress evidence is not real-ready evidence; error metrics are recorded for comparison
but are not a permission gate for hardware.

## Real-Candidate Parameter Policy

Simulator parameters can seed real testing only when they come from a stable
simulator baseline, speeds are scaled down for real, real read-only acceptance
has passed, real tiny joint motion acceptance has passed, and real tiny Cartesian
PTP acceptance has passed. Do not copy simulator motion time constants,
aggressive stress gains, GENE-style 4 s speed, or Python sender timing
assumptions. Copy cautiously: frame conventions, conservative path gains,
lease/deadman requirements, telemetry thresholds, and safety gates.

## rbpodo Controller-Simulation Tuning Rules

open-loop radius can be good while center drift is bad.
closed-loop is structurally needed for rbpodo controller simulation.
Kp=2 was aggressive in previous stage.
Kp_pos and Kp_ori should be tuned separately.
`timing_classification` must be `clean_timing` before a feedback row is treated
as a closed-loop tuning candidate; non-clean timing classifications are
measurement-limited evidence, not gain quality evidence.

`measurement_reliability_level` must be read before tuning notes. For rbpodo
pgmode simulation, `controller_reference_valid` means the controller-reference
path is usable as a lower-bound measurement only. It is not physical TCP
tracking and it is not physical-ready evidence. `unreliable` and `suspect`
rows should not drive gain selection or IL data selection.

The rbpodo tuning report classifies rows as:

- `open_loop_baseline`: useful reference evidence, not a candidate.
- `closed_loop_candidate`: feedback row that avoids the structural rejection cutoffs.
- `saturation_limited`: feedback saturation ratio is above the candidate cutoff.
- `orientation_unstable`: p95 orientation drift is above the candidate cutoff.
- `center_drift_limited`: 4 s fit-center drift makes radius gain misleading.
- `state_pub_speed_mismatch`: state publication and `speed_bar=0.1` stress behavior.
- `stress_only`: incomplete or heavily penalized evidence that should not be promoted.

The separate `error_classification` column decomposes measurement/tracking
error into `phase_lag_limited`, `center_drift_limited`,
`tail_spike_limited`, `orientation_limited`, `saturation_limited`, and
`timing_jitter_limited`. Use it to decide whether to tune gains, fix timing,
separate orientation gain, or treat RMS/p95 as tail-spike dominated.

Candidate scoring rejects or heavily penalizes `fault_latched=true`,
`physical_motion_detected=true`, `cartesian_unavailable_count > 0`,
non-clean `timing_classification`,
`feedback_saturation_count > 0.2 * command_count`,
`p95_orientation_drift_rad > 0.25`, `fit_center_error_m > 0.05` for the 4 s
profile, and `radius_gain` outside `[0.85, 1.15]`. Among non-rejected
closed-loop candidates, lower scores prioritize low saturation, low center
error, low orientation drift, radius gain near 1, then RMS and p95 error.

Orientation feedback may worsen orientation drift, so position and orientation
gains should be compared independently. `pub_rate=100` with `speed_bar=0.1`
can destabilize the 4 s stress row; `speed_bar>=0.2` can restore stability but
still needs radius, orientation, and center-drift review.

rbpodo controller-simulation results can guide future low-speed parameter
selection, but cannot be copied directly to real physical motion.
"""


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=REPORT_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key) for key in REPORT_COLUMNS})


def row_label(row: dict[str, Any] | None) -> str:
    if row is None:
        return "_None._"
    name = row.get("run_name") or row.get("name") or Path(str(row.get("artifact_dir", ""))).name
    score = first_number(row, "score")
    details = []
    if score is not None:
        details.append(f"score {score:.3f}")
    kp_pos = first_number(row, "kp_pos")
    kp_ori = first_number(row, "kp_ori")
    if kp_pos is not None:
        details.append(f"Kp_pos {kp_pos:g}")
    if kp_ori is not None:
        details.append(f"Kp_ori {kp_ori:g}")
    classification = row.get("classification")
    if classification:
        details.append(str(classification))
    suffix = f" ({', '.join(details)})" if details else ""
    return f"{name}{suffix}"


def best_by(rows: list[dict[str, Any]], keys: tuple[str, ...]) -> dict[str, Any] | None:
    candidates: list[dict[str, Any]] = []
    for row in rows:
        if all(first_number(row, key) is not None for key in keys):
            candidates.append(row)
    if not candidates:
        return None
    return min(candidates, key=lambda row: tuple(first_number(row, key) or 0.0 for key in keys))


def error_classification_counts(rows: list[dict[str, Any]]) -> str:
    counts: dict[str, int] = {}
    for row in rows:
        value = str(row.get("error_classification") or "")
        if not value:
            continue
        counts[value] = counts.get(value, 0) + 1
    if not counts:
        return "_None._"
    return ", ".join(f"{key}={counts[key]}" for key in sorted(counts))


def row_has_orientation_limited_evidence(row: dict[str, Any]) -> bool:
    orientation_equiv_mm = first_number(row, "orientation_position_equiv_50mm_mm")
    orientation_p95_deg = first_number(row, "orientation_p95_deg")
    return (
        row.get("error_classification") == "orientation_limited"
        or (orientation_equiv_mm is not None and orientation_equiv_mm >= 5.0)
        or (orientation_p95_deg is not None and orientation_p95_deg >= 5.0)
    )


def rbpodo_error_diagnosis_notes(rows: list[dict[str, Any]]) -> list[str]:
    rbpodo_rows = [row for row in rows if row.get("benchmark_category") == "rbpodo_controller_simulation"]
    open_loop = [row for row in rbpodo_rows if row.get("classification") == "open_loop_baseline"]
    closed_loop = [row for row in rbpodo_rows if row.get("classification") != "open_loop_baseline" and is_feedback_controller(row)]
    notes: list[str] = []
    for row in open_loop:
        radius = first_number(row, "radius_gain")
        center_mm = first_number(row, "center_error_mm")
        if radius is not None and 0.95 <= radius <= 1.05 and center_mm is not None and center_mm >= 50.0:
            notes.append("open-loop has good radius but large center drift")
            break
    open_centers = [first_number(row, "center_error_mm") for row in open_loop]
    closed_centers = [first_number(row, "center_error_mm") for row in closed_loop]
    open_values = [value for value in open_centers if value is not None]
    closed_values = [value for value in closed_centers if value is not None]
    if open_values and closed_values and min(closed_values) < min(open_values) * 0.5:
        notes.append("closed-loop reduces center drift")
    if any(row.get("error_classification") == "saturation_limited" for row in rbpodo_rows):
        notes.append("high-gain rows can become saturation-limited")
    if any(row_has_orientation_limited_evidence(row) for row in rbpodo_rows):
        notes.append("orientation drift is a separate error source from position radius/center")
    if not notes:
        notes.append("no dominant rbpodo error decomposition pattern detected")
    return list(dict.fromkeys(notes))


def rbpodo_stage_summary_markdown(rows: list[dict[str, Any]]) -> str:
    rbpodo_rows = [row for row in rows if row.get("benchmark_category") == "rbpodo_controller_simulation"]
    closed_loop = [row for row in rbpodo_rows if row.get("classification") == "closed_loop_candidate"]
    open_loop = [row for row in rbpodo_rows if row.get("classification") == "open_loop_baseline"]
    best_overall = best_by(closed_loop, ("score",))
    best_low_saturation = best_by(closed_loop, ("saturation_ratio", "score"))
    best_orientation = best_by(closed_loop, ("orientation_p95_deg", "score"))
    best_center = best_by(closed_loop, ("center_error_mm", "score"))
    best_open_loop = best_by(open_loop, ("score",))
    lines = [
        "## rbpodo Tuning Stage Summary",
        "",
        f"- best_overall_candidate: {row_label(best_overall)}",
        f"- best_low_saturation_candidate: {row_label(best_low_saturation)}",
        f"- best_orientation_candidate: {row_label(best_orientation)}",
        f"- best_center_candidate: {row_label(best_center)}",
        f"- best_open_loop_baseline: {row_label(best_open_loop)}",
        f"- error_classification_counts: {error_classification_counts(rbpodo_rows)}",
    ]
    lines.extend(f"- {note}" for note in rbpodo_error_diagnosis_notes(rows))
    return "\n".join(lines)


def rbpodo_decision_split_markdown(rows: list[dict[str, Any]]) -> str:
    rbpodo_rows = [row for row in rows if row.get("benchmark_category") == "rbpodo_controller_simulation"]
    if not rbpodo_rows:
        return "_None supplied._"
    columns = [
        "run_name",
        "classification",
        "score",
        "measurement_reliability_level",
        "reliability_caveats",
        "physical_real_blockers",
        "physical_readiness_status",
        "controller_reference_status",
        "physical_tracking_status",
    ]
    lines = [
        "| " + " | ".join(columns) + " |",
        "| " + " | ".join("---" for _ in columns) + " |",
    ]
    for row in rbpodo_rows:
        lines.append("| " + " | ".join(format_cell(row.get(key)) for key in columns) + " |")
    return "\n".join(lines)


def lane_group_markdown(rows: list[dict[str, Any]]) -> str:
    columns = [
        "benchmark_lane",
        "count",
        "benchmark_categories",
        "controllers",
        "async_modes",
        "low_level_send_modes",
        "acceptance_semantics",
        "tracking_sources",
    ]
    groups: dict[str, dict[str, Any]] = {}
    for row in rows:
        lane = str(row.get("benchmark_lane") or "")
        if not lane:
            continue
        group = groups.setdefault(
            lane,
            {
                "benchmark_lane": lane,
                "count": 0,
                "benchmark_categories": set(),
                "controllers": set(),
                "async_modes": set(),
                "low_level_send_modes": set(),
                "acceptance_semantics": set(),
                "tracking_sources": set(),
            },
        )
        group["count"] += 1
        for field, target in (
            ("benchmark_category", "benchmark_categories"),
            ("controller", "controllers"),
            ("async_mode", "async_modes"),
            ("low_level_send_mode", "low_level_send_modes"),
            ("acceptance_semantics", "acceptance_semantics"),
            ("tracking_source", "tracking_sources"),
        ):
            value = row.get(field)
            if value not in (None, ""):
                group[target].add(str(value))
    if not groups:
        return "_None supplied._"
    lines = [
        "| " + " | ".join(columns) + " |",
        "| " + " | ".join("---" for _ in columns) + " |",
    ]
    for lane in sorted(groups):
        group = groups[lane]
        row = {
            key: (
                ", ".join(sorted(value))
                if isinstance(value, set)
                else value
            )
            for key, value in group.items()
        }
        lines.append("| " + " | ".join(format_cell(row.get(key)) for key in columns) + " |")
    return "\n".join(lines)


def report_markdown(rows: list[dict[str, Any]], title: str, min_repeats: int) -> str:
    baseline = [row for row in rows if row.get("classification") == "stable_simulator_baseline_candidate"]
    stress = [row for row in rows if row.get("classification") == "stress_benchmark_candidate"]
    rbpodo_rows = [row for row in rows if row.get("benchmark_category") == "rbpodo_controller_simulation"]
    rbpodo_closed_loop = [row for row in rbpodo_rows if row.get("classification") == "closed_loop_candidate"]
    rbpodo_open_loop = [row for row in rbpodo_rows if row.get("classification") == "open_loop_baseline"]
    rbpodo_structural = [
        row for row in rbpodo_rows
        if row.get("classification") in {
            "saturation_limited",
            "orientation_unstable",
            "center_drift_limited",
            "state_pub_speed_mismatch",
            "stress_only",
            "rbpodo_controller_sim_cartesian_blocked",
        }
    ]
    simulator_rows = [row for row in rows if row.get("benchmark_category") == "rb_simulator"]
    real_physical_rows = [row for row in rows if row.get("benchmark_category") == "real_physical_benchmark"]
    rejected = [
        row for row in rows
        if row.get("classification") in {
            "not_baseline_candidate",
            "stress_rejected_or_incomplete",
            "rbpodo_controller_sim_baseline_incomplete",
            "rbpodo_controller_sim_stress_rejected_or_incomplete",
            "rbpodo_controller_sim_cartesian_blocked",
            "saturation_limited",
            "orientation_unstable",
            "center_drift_limited",
            "state_pub_speed_mismatch",
            "stress_only",
        }
    ]
    parts = [
        f"# {title}",
        "",
        "This report separates simulator, rbpodo controller-simulation, and future real physical evidence. It does not authorize real robot motion.",
        "",
        f"**{reliability_report.ACKON500_PHYSICAL_WARNING}**",
        "",
        criteria_markdown(min_repeats).rstrip(),
        "",
        "## Canonical Benchmark Lanes",
        "",
        lane_group_markdown(rows),
        "",
        "## All Runs",
        "",
        markdown_table(rows) if rows else "_No runs supplied._",
        "",
        "## rb_simulator Benchmarks",
        "",
        markdown_table(simulator_rows) if simulator_rows else "_None supplied._",
        "",
        "## rbpodo Controller-Simulation Benchmarks",
        "",
        markdown_table(rbpodo_rows) if rbpodo_rows else "_None supplied._",
        "",
        "## Measurement Reliability And Caveats",
        "",
        reliability_report.markdown_table(rbpodo_rows) if rbpodo_rows else "_None supplied._",
        "",
        "## rbpodo Tuning Result vs Measurement Reliability",
        "",
        "This table separates the tuning classification from measurement reliability and physical-readiness blockers.",
        "",
        rbpodo_decision_split_markdown(rows),
        "",
        rbpodo_stage_summary_markdown(rows),
        "",
        "## rbpodo Closed-Loop Candidates",
        "",
        markdown_table(rbpodo_closed_loop) if rbpodo_closed_loop else "_None._",
        "",
        "## rbpodo Open-Loop Baselines",
        "",
        markdown_table(rbpodo_open_loop) if rbpodo_open_loop else "_None._",
        "",
        "## rbpodo Structural Rejections",
        "",
        markdown_table(rbpodo_structural) if rbpodo_structural else "_None._",
        "",
        "## Real Physical Benchmarks",
        "",
        markdown_table(real_physical_rows) if real_physical_rows else "_Future/not run. No real physical circle tracking evidence is claimed._",
        "",
        "## Stable Simulator Baseline Candidates",
        "",
        markdown_table(baseline) if baseline else "_None._",
        "",
        "## Stress Benchmark Candidates",
        "",
        markdown_table(stress) if stress else "_None._",
        "",
        "## Rejected Or Incomplete Baseline/Stress Runs",
        "",
        markdown_table(rejected) if rejected else "_None._",
        "",
        "## REVIEW.md Recording Template",
        "",
        "Paste a concise evidence note, for example:",
        "",
        "```text",
        "Current circle tracking simulator baseline: <artifact path>, profile circle_15cm_16s, "
        "controller <controller>, repeat <N>, radius_gain <value>, rms_error_m <value>, "
        "p95_error_m <value>, result <completed/pass>. Simulator-only; not real-ready.",
        "",
        "Current rbpodo controller-simulation circle baseline: <artifact path>, profile circle_15cm_16s, "
        "tracking_source tcp_ref_stand, radius_gain <value>, rms_error_m <value>, "
        "p95_error_m <value>, physical_motion_detected false, measurement_reliability_level "
        "<level>, benchmark_interpretation <values>, physical_real_blockers <values>. "
        "pgmode simulation only; not real-ready.",
        "```",
    ]
    return "\n".join(parts) + "\n"


def main() -> int:
    args = parse_args()
    if args.min_baseline_repeats < 1:
        raise SystemExit("--min-baseline-repeats must be >= 1")
    if not args.summary_json and args.ablation_summary_csv is None:
        raise SystemExit("provide at least one summary.json or --ablation-summary-csv")
    rows = classify_rows(load_rows(args.summary_json, args.ablation_summary_csv), args.min_baseline_repeats)
    markdown = report_markdown(rows, args.title, args.min_baseline_repeats)
    if args.output_md:
        args.output_md.parent.mkdir(parents=True, exist_ok=True)
        args.output_md.write_text(markdown, encoding="utf-8")
    else:
        print(markdown, end="")
    if args.csv_path:
        write_csv(args.csv_path, rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
