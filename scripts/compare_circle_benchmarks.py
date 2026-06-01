#!/usr/bin/env python3
"""Compare circle tracking benchmark summary.json artifacts."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import circle_tracking_benchmark as profile_bench
import generate_rbpodo_measurement_reliability_report as reliability_report


COLUMNS = [
    ("run name", "run_name"),
    ("category", "benchmark_category"),
    ("backend", "backend"),
    ("controller_mode", "controller_mode"),
    ("controller", "controller"),
    ("arm", "arm"),
    ("profile", "profile"),
    ("tracking_source", "tracking_source"),
    ("kp_pos", "kp_pos"),
    ("kp_ori", "kp_ori"),
    ("state_pub_rate_hz", "state_pub_rate_hz"),
    ("speed_bar_left", "speed_bar_left"),
    ("speed_bar_right", "speed_bar_right"),
    ("diameter_m", "diameter_m"),
    ("period_sec", "period_sec"),
    ("required_tangential_speed_m_s", "required_tangential_speed_m_s"),
    ("stress_level", "stress_level"),
    ("command_rate_hz", "command_rate_hz"),
    ("servo_rate_hz", "servo_rate_hz"),
    ("measurement_reliability_level", "measurement_reliability_level"),
    ("reliability_caveats", "reliability_caveats"),
    ("benchmark_interpretation", "benchmark_interpretation"),
    ("physical_real_blockers", "physical_real_blockers"),
    ("radius_gain", "radius_gain"),
    ("mean_error_mm", "mean_error_mm"),
    ("rms_error_mm", "rms_error_mm"),
    ("median_error_mm", "median_error_mm"),
    ("p95_error_mm", "p95_error_mm"),
    ("max_error_mm", "max_error_mm"),
    ("tail_ratio", "tail_ratio"),
    ("center_removed_rms_mm", "center_removed_rms_mm"),
    ("phase_aligned_rms_mm", "phase_aligned_rms_mm"),
    ("orientation_position_equiv_50mm_mm", "orientation_position_equiv_50mm_mm"),
    ("error_classification", "error_classification"),
    ("p95_orientation_drift_mrad", "p95_orientation_drift_mrad"),
    ("max_orientation_drift_mrad", "max_orientation_drift_mrad"),
    ("estimated_latency_ms", "estimated_latency_ms"),
    ("worker_command_drops_total", "worker_command_drops_total"),
    ("integrator_clamps_total", "integrator_clamps_total"),
    ("integrator_divergence_total", "integrator_divergence_total"),
    ("send_success_rate", "send_success_rate"),
    ("controller_acceptance_observed_rate", "controller_acceptance_observed_rate"),
    ("send_duration_p99_us", "send_duration_p99_us"),
    ("send_duration_max_us", "send_duration_max_us"),
    ("send_command_deadline_missed_count", "send_command_deadline_missed_count"),
    ("deadline_miss_count", "deadline_miss_count"),
    ("command_interval_max_ms", "command_interval_max_ms"),
    ("servo_jitter_p99_ms", "servo_jitter_p99_ms"),
    ("servo_jitter_max_ms", "servo_jitter_max_ms"),
    ("timing_classification", "timing_classification"),
    ("ack_spike_count_10ms", "ack_spike_count_10ms"),
    ("ack_spike_count_20ms", "ack_spike_count_20ms"),
    ("state_gap_count", "state_gap_count"),
    ("command_gap_count", "command_gap_count"),
    ("p95_error_near_ack_spike_mm", "p95_error_near_ack_spike_mm"),
    ("p95_error_away_from_ack_spike_mm", "p95_error_away_from_ack_spike_mm"),
    ("p95_error_near_command_gap_mm", "p95_error_near_command_gap_mm"),
    ("p95_error_away_from_command_gap_mm", "p95_error_away_from_command_gap_mm"),
    ("mean_feedback_linear_norm_m_s", "mean_feedback_linear_norm_m_s"),
    ("max_feedback_linear_norm_m_s", "max_feedback_linear_norm_m_s"),
    ("feedback_saturation_count", "feedback_saturation_count"),
    ("saturation_ratio", "saturation_ratio"),
    ("orientation_p95_deg", "orientation_p95_deg"),
    ("center_error_mm", "center_error_mm"),
    ("stale_state_feedback_skips", "stale_state_feedback_skips"),
    ("physical_motion_expected", "physical_motion_expected"),
    ("physical_motion_detected", "physical_motion_detected"),
    ("q_actual_moved", "q_actual_moved"),
    ("q_sent_moved", "q_sent_moved"),
    ("q_ref_moved", "q_ref_moved"),
    ("tcp_ref_moved", "tcp_ref_moved"),
    ("tcp_actual_moved", "tcp_actual_moved"),
    ("q_sent_update_rate_hz", "q_sent_update_rate_hz"),
    ("q_ref_update_rate_hz", "q_ref_update_rate_hz"),
    ("q_ref_valid_ratio", "q_ref_valid_ratio"),
    ("q_actual_update_rate_hz", "q_actual_update_rate_hz"),
    ("q_ref_reason", "q_ref_reason"),
    ("ack_policy", "ack_policy"),
    ("controller_acceptance_observed_count", "controller_acceptance_observed_count"),
    ("command_timeout_count", "command_timeout_count"),
    ("controller_rejected_count", "controller_rejected_count"),
    ("tcp_ref_valid_ratio", "tcp_ref_valid_ratio"),
    ("tcp_actual_valid_ratio", "tcp_actual_valid_ratio"),
    ("diagnostics_suspect_count", "diagnostics_suspect_count"),
    ("controller_simulation_diagnostic_override_active_count", "controller_simulation_diagnostic_override_active_count"),
    ("reset_rate_hz", "reset_rate_hz"),
    ("divergence_rate_hz", "divergence_rate_hz"),
    ("result", "result"),
    ("result_reason", "result_reason"),
    ("server_rejected_cartesian", "server_rejected_cartesian"),
    ("cartesian_unavailable_count", "cartesian_unavailable_count"),
    ("performance_warnings", "performance_warnings"),
]

PROFILE_BY_DIMENSION = {
    defaults: profile
    for profile, defaults in profile_bench.PROFILE_DEFAULTS.items()
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Print a markdown comparison table for circle tracking benchmark summary.json files."
    )
    parser.add_argument("summary_json", nargs="+", type=Path, help="One or more circle benchmark summary.json files")
    parser.add_argument("--csv", dest="csv_path", type=Path, help="Optional CSV output path")
    parser.add_argument(
        "--sort",
        choices=("input", "rms_error"),
        default="input",
        help="Sort rows by input order or ascending rms_error_m",
    )
    return parser.parse_args()


def load_summary(path: Path) -> dict[str, Any]:
    try:
        with path.open(encoding="utf-8") as handle:
            value = json.load(handle)
    except Exception as exc:
        raise SystemExit(f"compare_circle_benchmarks: failed to read {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise SystemExit(f"compare_circle_benchmarks: {path} does not contain a JSON object")
    value["_summary_path"] = str(path)
    return value


def finite_number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        number = float(value)
        return number if math.isfinite(number) else None
    if isinstance(value, str):
        try:
            number = float(value)
        except ValueError:
            return None
        return number if math.isfinite(number) else None
    return None


def scaled(summary: dict[str, Any], key: str, factor: float) -> float | None:
    value = finite_number(summary.get(key))
    return value * factor if value is not None else None


def scaled_value(value: Any, factor: float) -> float | None:
    number = finite_number(value)
    return number * factor if number is not None else None


def nested_dict(summary: dict[str, Any], key: str) -> dict[str, Any]:
    value = summary.get(key)
    return value if isinstance(value, dict) else {}


def nested_metric(summary: dict[str, Any], key: str, metric: str) -> float | None:
    value = summary.get(key)
    if not isinstance(value, dict):
        return None
    return finite_number(value.get(metric))


def first_number(summary: dict[str, Any], *keys: str) -> float | None:
    for key in keys:
        value = finite_number(summary.get(key))
        if value is not None:
            return value
    return None


def first_present(summary: dict[str, Any], *keys: str) -> Any:
    for key in keys:
        if key in summary and summary.get(key) is not None:
            return summary.get(key)
    return None


def first_value(*values: Any) -> Any:
    for value in values:
        if value is not None:
            return value
    return None


def ratio_from_counts(numerator: Any, denominator: Any) -> float | None:
    top = finite_number(numerator)
    bottom = finite_number(denominator)
    if top is None or bottom is None or bottom <= 0.0:
        return None
    return top / bottom


def saturation_ratio(summary: dict[str, Any]) -> float | None:
    saturation_count = finite_number(summary.get("feedback_saturation_count"))
    if saturation_count is None:
        return None
    command_count = first_number(
        summary,
        "command_count",
        "ack_observed_count",
        "controller_acceptance_observed_count",
    )
    if command_count is None or command_count <= 0.0:
        return None
    return saturation_count / command_count


def run_name(summary: dict[str, Any]) -> str:
    artifact_dir = summary.get("artifact_dir")
    if isinstance(artifact_dir, str) and artifact_dir:
        return Path(artifact_dir).name
    path = Path(str(summary.get("_summary_path", "summary.json")))
    return path.parent.name or path.name


def inferred_profile(summary: dict[str, Any]) -> Any:
    profile = summary.get("profile")
    if profile:
        return profile
    diameter = finite_number(summary.get("diameter_m"))
    period = finite_number(summary.get("period_sec"))
    if diameter is None or period is None:
        return None
    for (known_diameter, known_period), known_profile in PROFILE_BY_DIMENSION.items():
        if abs(diameter - known_diameter) < 1e-9 and abs(period - known_period) < 1e-9:
            return known_profile
    return None


def summary_profile_catalog_entry(summary: dict[str, Any]) -> dict[str, Any]:
    embedded = summary.get("profile_catalog_entry")
    if isinstance(embedded, dict):
        return embedded
    preflight = safety_preflight(summary)
    embedded = preflight.get("profile_catalog_entry")
    if isinstance(embedded, dict):
        return embedded
    profile = inferred_profile(summary)
    if isinstance(profile, str) and profile in profile_bench.PROFILE_CATALOG:
        return profile_bench.benchmark_profile_metadata(profile)
    return {}


def required_tangential_speed(summary: dict[str, Any]) -> float | None:
    for source in (summary, safety_preflight(summary)):
        value = finite_number(source.get("required_tangential_speed_m_s"))
        if value is not None:
            return value
    diameter = finite_number(summary.get("diameter_m"))
    period = finite_number(summary.get("period_sec"))
    if diameter is not None and period is not None and period > 0.0:
        return math.pi * diameter / period
    value = finite_number(summary_profile_catalog_entry(summary).get("required_tangential_speed_m_s"))
    if value is not None:
        return value
    return None


def stress_level(summary: dict[str, Any]) -> str:
    for source in (summary, safety_preflight(summary), summary_profile_catalog_entry(summary)):
        value = source.get("stress_level")
        if isinstance(value, str) and value:
            return value
    return ""


def radius_gain(summary: dict[str, Any]) -> float | None:
    existing = finite_number(summary.get("radius_gain"))
    if existing is not None:
        return existing
    fit_radius = finite_number(summary.get("fit_radius_m"))
    reference_radius = finite_number(summary.get("reference_radius_m"))
    if reference_radius is None:
        reference_radius = finite_number(summary.get("radius_m"))
    if fit_radius is None or reference_radius is None or reference_radius <= 0.0:
        return None
    return fit_radius / reference_radius


def safety_preflight(summary: dict[str, Any]) -> dict[str, Any]:
    value = summary.get("safety_preflight")
    return value if isinstance(value, dict) else {}


def infer_backend(summary: dict[str, Any]) -> str:
    preflight = safety_preflight(summary)
    backend = summary.get("backend") or preflight.get("backend")
    if isinstance(backend, str) and backend:
        return backend
    schema = str(summary.get("schema") or "")
    if "rbpodo_circle_tracking_benchmark" in schema:
        return "rbpodo"
    if any(summary.get(key) for key in ("left_simulator_log", "right_simulator_log", "simulator_motion_time_constant_sec")):
        return "simulator"
    return ""


def infer_category(summary: dict[str, Any]) -> str:
    preflight = safety_preflight(summary)
    backend = infer_backend(summary)
    physical_expected = summary.get("physical_motion_expected")
    if physical_expected is None:
        physical_expected = preflight.get("physical_motion_expected")
    if physical_expected is True:
        return "real_physical_benchmark"
    if backend == "rbpodo" and (
        summary.get("controller_simulation_only") is True
        or preflight.get("controller_simulation_only") is True
        or preflight.get("pgmode_simulation_confirmed") is True
    ):
        return "rbpodo_controller_simulation"
    if backend in {"simulator", "mock"} or any(
        summary.get(key) for key in ("left_simulator_log", "right_simulator_log", "simulator_motion_time_constant_sec")
    ):
        return "rb_simulator"
    return "unknown"


def infer_controller_mode(summary: dict[str, Any]) -> str:
    preflight = safety_preflight(summary)
    category = infer_category(summary)
    if category == "rbpodo_controller_simulation":
        return "pgmode_simulation"
    if category == "rb_simulator":
        return "rb_simulator"
    if category == "real_physical_benchmark":
        return "real_physical"
    value = summary.get("controller_mode") or preflight.get("controller_mode")
    return str(value) if value is not None else ""


def infer_tracking_source(summary: dict[str, Any]) -> str:
    source = summary.get("tracking_source_used") or summary.get("tracking_source") or summary.get("tracking_source_requested")
    if source:
        return str(source)
    if infer_category(summary) == "rb_simulator":
        return "simulator_tcp_actual"
    return ""


def infer_physical_motion_expected(summary: dict[str, Any]) -> Any:
    preflight = safety_preflight(summary)
    if "physical_motion_expected" in summary:
        return summary.get("physical_motion_expected")
    return preflight.get("physical_motion_expected")


def infer_ack_policy(summary: dict[str, Any]) -> str:
    preflight = safety_preflight(summary)
    if "ack_policy" in summary:
        return str(summary.get("ack_policy"))
    distribution = summary.get("ack_policy_distribution")
    if isinstance(distribution, dict) and distribution:
        keys = sorted(str(key) for key in distribution)
        if any("disabled" in key or "no_ack" in key or "ack_off" in key for key in keys):
            return "ack_off"
        return "ack_on"
    if preflight.get("disable_waiting_ack") is True:
        return "ack_off"
    if preflight.get("disable_waiting_ack") is False:
        return "ack_on"
    return ""


def artifact_path(summary: dict[str, Any], filename: str) -> Path:
    artifact_dir = summary.get("artifact_dir")
    if isinstance(artifact_dir, str) and artifact_dir:
        return Path(artifact_dir) / filename
    summary_path = Path(str(summary.get("_summary_path", "summary.json")))
    return summary_path.parent / filename


def command_interval_max_ms(summary: dict[str, Any]) -> float | None:
    existing = finite_number(summary.get("command_interval_max_ms"))
    if existing is not None:
        return existing
    timestamp_alignment = nested_dict(summary, "timestamp_alignment")
    nested = timestamp_alignment.get("command_interval_ms")
    if isinstance(nested, dict):
        existing = finite_number(nested.get("max"))
        if existing is not None:
            return existing
    existing = nested_metric(summary, "command_interval_ms", "max")
    if existing is not None:
        return existing
    path_text = summary.get("command_packets")
    path = Path(path_text) if isinstance(path_text, str) and path_text else artifact_path(summary, "command_packets.jsonl")
    if not path.is_file():
        return None
    host_times: list[int] = []
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            try:
                packet = json.loads(line)
            except json.JSONDecodeError:
                continue
            mode = (
                packet.get("left", {}).get("mode")
                if isinstance(packet.get("left"), dict)
                else None
            ) or (
                packet.get("right", {}).get("mode")
                if isinstance(packet.get("right"), dict)
                else None
            ) or packet.get("mode")
            if mode not in {
                "TcpTwistStand",
                "TcpTwistLocal",
                "TcpLinearMove",
                "TcpCircleMove",
            }:
                continue
            host_time_ns = packet.get("host_time_ns")
            if isinstance(host_time_ns, int):
                host_times.append(host_time_ns)
    if len(host_times) < 2:
        return None
    return max((b - a) / 1e6 for a, b in zip(host_times, host_times[1:]))


def send_success_rate(summary: dict[str, Any]) -> float | None:
    existing = finite_number(summary.get("send_success_rate"))
    if existing is not None:
        return existing
    count = first_number(
        summary,
        "send_count",
        "command_count",
        "ack_observed_count",
        "controller_acceptance_observed_count",
    )
    success = first_number(summary, "send_success_count", "send_ok_count")
    if success is not None:
        return ratio_from_counts(success, count)
    failures = first_number(summary, "send_failure_count")
    if failures is not None and count is not None and count > 0.0:
        return max(0.0, 1.0 - failures / count)
    return None


def controller_acceptance_observed_rate(summary: dict[str, Any]) -> float | None:
    existing = first_number(
        summary,
        "controller_acceptance_observed_rate",
        "controller_acceptance_ratio",
    )
    if existing is not None:
        return existing
    accepted = first_number(summary, "controller_acceptance_observed_count")
    count = first_number(summary, "send_count", "command_count", "ack_observed_count")
    return ratio_from_counts(accepted, count)


def nested_percentile_metric(summary: dict[str, Any], block: str, metric: str) -> float | None:
    existing = nested_metric(summary, block, metric)
    if existing is not None:
        return existing
    value = summary.get(block)
    if isinstance(value, dict):
        return finite_number(value.get(metric))
    return None


def deadline_miss_count(summary: dict[str, Any]) -> float | None:
    return first_number(
        summary,
        "deadline_miss_count",
        "send_deadline_missed_count",
        "send_command_deadline_missed_count",
        "command_sender_deadline_missed_count",
    )


def servo_jitter_p99_ms(summary: dict[str, Any]) -> float | None:
    existing = finite_number(summary.get("servo_jitter_p99_ms"))
    if existing is not None:
        return existing
    return nested_percentile_metric(summary, "servo_jitter_ms", "p99")


def csv_max(path: Path, field: str) -> float | None:
    if not path.is_file():
        return None
    values: list[float] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            value = finite_number(row.get(field))
            if value is not None:
                values.append(value)
    return max(values) if values else None


def servo_jitter_max_ms(summary: dict[str, Any]) -> float | None:
    existing = finite_number(summary.get("servo_jitter_max_ms"))
    if existing is not None:
        return existing
    servo_log = summary.get("servo_log")
    path: Path | None = None
    if isinstance(servo_log, dict) and isinstance(servo_log.get("path"), str):
        path = Path(servo_log["path"])
    if path is None:
        path = artifact_path(summary, "servo_log.csv")
    for field in ("jitter_ms", "servo_jitter_ms"):
        value = csv_max(path, field)
        if value is not None:
            return value
    return None


def warning_text(summary: dict[str, Any]) -> str:
    warnings = summary.get("performance_warnings")
    values: list[str] = []
    if isinstance(warnings, list):
        values.extend(str(item) for item in warnings)
    elif isinstance(warnings, str) and warnings:
        values.append(warnings)
    timing = first_present(summary, "timing_classification")
    if timing is None:
        timing = nested_dict(summary, "timestamp_alignment").get("timing_classification")
    if timing not in (None, "", "clean_timing"):
        values.append(f"timestamp_alignment timing_classification={timing}")
    error_classification = decomposition_value(summary, "error_classification")
    if error_classification:
        values.append(f"error_classification={error_classification}")
    return "; ".join(dict.fromkeys(values))


def error_decomposition_block(summary: dict[str, Any]) -> dict[str, Any]:
    return nested_dict(summary, "error_decomposition")


def decomposition_value(summary: dict[str, Any], key: str) -> Any:
    value = summary.get(key)
    if value is not None:
        return value
    return error_decomposition_block(summary).get(key)


def comparison_row(summary: dict[str, Any]) -> dict[str, Any]:
    category = infer_category(summary)
    backend = infer_backend(summary)
    timestamp_alignment = nested_dict(summary, "timestamp_alignment")
    tail_error_correlation = nested_dict(summary, "tail_error_correlation")
    error_decomposition = error_decomposition_block(summary)
    row = {
        "run_name": run_name(summary),
        "benchmark_category": category,
        "backend": backend,
        "controller_mode": infer_controller_mode(summary),
        "controller": summary.get("controller"),
        "arm": summary.get("arm"),
        "profile": inferred_profile(summary),
        "tracking_source": infer_tracking_source(summary),
        "kp_pos": first_present(summary, "feedback_kp_pos", "kp_pos"),
        "kp_ori": first_present(summary, "feedback_kp_ori", "kp_ori"),
        "state_pub_rate_hz": summary.get("state_pub_rate_hz"),
        "speed_bar_left": summary.get("speed_bar_left"),
        "speed_bar_right": summary.get("speed_bar_right"),
        "diameter_m": summary.get("diameter_m"),
        "period_sec": summary.get("period_sec"),
        "required_tangential_speed_m_s": required_tangential_speed(summary),
        "stress_level": stress_level(summary),
        "repeat": summary.get("repeat"),
        "command_rate_hz": first_present(summary, "command_rate_hz", "requested_rate_hz"),
        "servo_rate_hz": summary.get("servo_rate_hz"),
        "radius_gain": radius_gain(summary),
        "mean_error_mm": scaled(summary, "mean_error_m", 1000.0),
        "rms_error_mm": scaled(summary, "rms_error_m", 1000.0),
        "median_error_mm": scaled_value(decomposition_value(summary, "median_error_m"), 1000.0),
        "p95_error_mm": scaled(summary, "p95_error_m", 1000.0),
        "max_error_mm": scaled(summary, "max_error_m", 1000.0),
        "tail_ratio": decomposition_value(summary, "tail_ratio"),
        "center_removed_rms_mm": scaled_value(decomposition_value(summary, "center_removed_rms_error_m"), 1000.0),
        "phase_aligned_rms_mm": scaled_value(decomposition_value(summary, "phase_aligned_rms_error_m"), 1000.0),
        "orientation_position_equiv_50mm_mm": scaled_value(
            decomposition_value(summary, "orientation_position_equiv_50mm_m"),
            1000.0,
        ),
        "error_classification": first_value(
            decomposition_value(summary, "error_classification"),
            error_decomposition.get("error_classification"),
        ),
        "p95_orientation_drift_mrad": scaled(summary, "p95_orientation_drift_rad", 1000.0),
        "max_orientation_drift_mrad": scaled(summary, "max_orientation_drift_rad", 1000.0),
        "estimated_latency_ms": summary.get("estimated_latency_ms"),
        "worker_command_drops_total": summary.get("worker_command_drops_total"),
        "integrator_clamps_total": summary.get("integrator_clamps_total"),
        "integrator_divergence_total": summary.get("integrator_divergence_total"),
        "send_success_rate": send_success_rate(summary),
        "controller_acceptance_observed_rate": controller_acceptance_observed_rate(summary),
        "send_duration_p99_us": first_value(
            first_number(summary, "send_duration_p99_us"),
            nested_percentile_metric(summary, "send_duration_us", "p99"),
        ),
        "send_duration_max_us": first_value(
            first_number(summary, "send_duration_max_us"),
            nested_percentile_metric(summary, "send_duration_us", "max"),
        ),
        "send_command_deadline_missed_count": summary.get("send_command_deadline_missed_count"),
        "deadline_miss_count": deadline_miss_count(summary),
        "command_interval_max_ms": command_interval_max_ms(summary),
        "servo_jitter_p99_ms": servo_jitter_p99_ms(summary),
        "servo_jitter_max_ms": servo_jitter_max_ms(summary),
        "timing_classification": first_value(
            first_present(summary, "timing_classification"),
            timestamp_alignment.get("timing_classification"),
        ),
        "ack_spike_count_10ms": first_value(
            first_present(summary, "ack_spike_count_10ms"),
            timestamp_alignment.get("ack_spike_count_10ms"),
        ),
        "ack_spike_count_20ms": first_value(
            first_present(summary, "ack_spike_count_20ms"),
            timestamp_alignment.get("ack_spike_count_20ms"),
        ),
        "state_gap_count": first_value(first_present(summary, "state_gap_count"), timestamp_alignment.get("state_gap_count")),
        "command_gap_count": first_value(first_present(summary, "command_gap_count"), timestamp_alignment.get("command_gap_count")),
        "p95_error_near_ack_spike_mm": scaled_value(
            first_value(
                first_present(summary, "p95_error_near_ack_spike_m"),
                tail_error_correlation.get("p95_error_near_ack_spike_m"),
            ),
            1000.0,
        ),
        "p95_error_away_from_ack_spike_mm": scaled_value(
            first_value(
                first_present(summary, "p95_error_away_from_ack_spike_m"),
                tail_error_correlation.get("p95_error_away_from_ack_spike_m"),
            ),
            1000.0,
        ),
        "p95_error_near_command_gap_mm": scaled_value(
            first_value(
                first_present(summary, "p95_error_near_command_gap_m"),
                tail_error_correlation.get("p95_error_near_command_gap_m"),
            ),
            1000.0,
        ),
        "p95_error_away_from_command_gap_mm": scaled_value(
            first_value(
                first_present(summary, "p95_error_away_from_command_gap_m"),
                tail_error_correlation.get("p95_error_away_from_command_gap_m"),
            ),
            1000.0,
        ),
        "mean_feedback_linear_norm_m_s": summary.get("mean_feedback_linear_norm_m_s"),
        "max_feedback_linear_norm_m_s": summary.get("max_feedback_linear_norm_m_s"),
        "feedback_saturation_count": summary.get("feedback_saturation_count"),
        "saturation_ratio": saturation_ratio(summary),
        "orientation_p95_deg": scaled(summary, "p95_orientation_drift_rad", 180.0 / math.pi),
        "center_error_mm": scaled(summary, "fit_center_error_m", 1000.0),
        "stale_state_feedback_skips": summary.get("stale_state_feedback_skips"),
        "physical_motion_expected": infer_physical_motion_expected(summary),
        "physical_motion_detected": summary.get("physical_motion_detected"),
        "q_actual_moved": summary.get("q_actual_moved"),
        "q_sent_moved": summary.get("q_sent_moved"),
        "q_ref_moved": summary.get("q_ref_moved"),
        "tcp_ref_moved": summary.get("tcp_ref_moved"),
        "tcp_actual_moved": summary.get("tcp_actual_moved"),
        "q_sent_update_rate_hz": summary.get("q_sent_update_rate_hz"),
        "q_ref_update_rate_hz": summary.get("q_ref_update_rate_hz"),
        "q_ref_valid_ratio": first_value(
            first_present(summary, "q_ref_valid_ratio"),
            summary.get("q_reference_for_servo_valid_ratio"),
        ),
        "q_actual_update_rate_hz": summary.get("q_actual_update_rate_hz"),
        "q_ref_reason": summary.get("q_ref_reason"),
        "ack_policy": infer_ack_policy(summary),
        "controller_acceptance_observed_count": summary.get("controller_acceptance_observed_count"),
        "command_timeout_count": summary.get("command_timeout_count"),
        "controller_rejected_count": summary.get("controller_rejected_count"),
        "tcp_ref_valid_ratio": summary.get("tcp_ref_valid_ratio"),
        "tcp_actual_valid_ratio": summary.get("tcp_actual_valid_ratio"),
        "diagnostics_suspect_count": summary.get("diagnostics_suspect_count"),
        "controller_simulation_diagnostic_override_active_count": summary.get(
            "controller_simulation_diagnostic_override_active_count"
        ),
        "reset_rate_hz": summary.get("reset_rate_hz"),
        "divergence_rate_hz": summary.get("divergence_rate_hz"),
        "physical_actual_csv": summary.get("physical_actual_csv"),
        "fault_latched": summary.get("fault_latched"),
        "result": summary.get("result"),
        "result_reason": summary.get("result_reason"),
        "server_rejected_cartesian": summary.get("server_rejected_cartesian"),
        "cartesian_unavailable_count": summary.get("cartesian_unavailable_count"),
        "cartesian_unavailable_reason_counts": summary.get("cartesian_unavailable_reason_counts"),
        "performance_warnings": warning_text(summary),
    }
    reliability_report.annotate_row(row)
    return row


def format_cell(value: Any) -> str:
    if value is None:
        return ""
    number = finite_number(value)
    if number is not None:
        if abs(number) >= 100.0:
            return f"{number:.3f}"
        if abs(number) >= 1.0:
            return f"{number:.4f}"
        return f"{number:.6f}"
    text = str(value)
    return text.replace("|", "\\|").replace("\n", " ")


def write_markdown(rows: list[dict[str, Any]]) -> None:
    headers = [title for title, _key in COLUMNS]
    print("| " + " | ".join(headers) + " |")
    print("| " + " | ".join("---" for _ in headers) + " |")
    for row in rows:
        print("| " + " | ".join(format_cell(row.get(key)) for _title, key in COLUMNS) + " |")


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [key for _title, key in COLUMNS]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key) for key in fieldnames})


def main() -> int:
    args = parse_args()
    summaries = [load_summary(path) for path in args.summary_json]
    rows = [comparison_row(summary) for summary in summaries]
    if args.sort == "rms_error":
        def rms_sort_key(row: dict[str, Any]) -> tuple[int, float]:
            blocked = row.get("result") == "blocked" or row.get("server_rejected_cartesian") is True
            if blocked:
                return (2, math.inf)
            rms = finite_number(row.get("rms_error_mm"))
            if rms is None:
                return (1, math.inf)
            return (0, rms)

        rows.sort(
            key=rms_sort_key
        )
    write_markdown(rows)
    if args.csv_path:
        write_csv(args.csv_path, rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
