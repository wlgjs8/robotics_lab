#!/usr/bin/env python3
"""Generate a 100 Hz vs 500 Hz rbpodo controller-simulation report.

This script is reporting-only. It reads existing no-op, circle summary, and
ablation CSV artifacts; it does not connect to controllers or send commands.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import compare_circle_benchmarks as compare
import generate_rbpodo_measurement_reliability_report as reliability_report


SCHEMA = "robotics_lab.rbpodo_500hz_comparison_report.v1"
DEFAULT_TITLE = "rbpodo 100 Hz vs 500 Hz Controller-Simulation Report"

COMPARISONS = [
    ("noop_acceptance", "no-op acceptance"),
    ("safe_5cm_10s", "safe_5cm_10s"),
    ("circle_15cm_16s", "15cm_16s"),
    ("circle_15cm_8s", "15cm_8s"),
    ("gene_15cm_4s", "15cm_4s"),
]

KEY_FIELDS = [
    "async_mode",
    "acceptance_semantics",
    "controller_ack_observed",
    "sdk_worker_ack_observed",
    "socket_send_only",
    "q_ref_supervised",
    "send_success_rate",
    "controller_acceptance_observed_rate",
    "send_duration_p99_us",
    "send_duration_max_us",
    "servo_jitter_p99_ms",
    "deadline_miss_count",
    "command_interval_max_ms",
    "state_pub_rate_hz",
    "commands_enqueued_total",
    "commands_sent_total",
    "commands_acked_total",
    "commands_socket_sent_total",
    "commands_overwritten_total",
    "commands_dropped_total",
    "reference_supervision_state",
    "q_ref_update_rate_hz",
    "q_ref_target_error_deg_max",
    "rms_error_mm",
    "p95_error_mm",
    "tail_ratio",
    "orientation_p95_deg",
    "feedback_saturation_count",
    "measurement_reliability_level",
]

RATE_COLUMNS = [
    "comparison",
    "rate_hz",
    "run_name",
    "source_kind",
    "profile",
    "controller",
    "result",
    *KEY_FIELDS,
    "tracking_source",
    "reliability_caveats",
    "physical_real_blockers",
    "artifact_dir",
]

COMPARISON_COLUMNS = [
    "comparison",
    "classification",
    "rate_100_run",
    "rate_500_run",
    "rate_100_lane",
    "rate_500_lane",
    "tracking_delta_interpretation",
    "recommendation",
    "classification_reason",
    "caveats",
]
for _field in KEY_FIELDS:
    COMPARISON_COLUMNS.extend([f"rate_100_{_field}", f"rate_500_{_field}"])

COMPARATIVE_LANES = [
    ("100hz_ack_on_best", 100.0),
    ("500hz_ack_on", 500.0),
    ("500hz_socket_send_supervised", 500.0),
    ("500hz_async_sdk_ack_worker", 500.0),
]

COMPARATIVE_COLUMNS = [
    "comparison",
    "lane",
    "evidence_present",
    "run_name",
    "rate_hz",
    "async_mode",
    "acceptance_semantics",
    "controller_ack_observed",
    "sdk_worker_ack_observed",
    "socket_send_only",
    "q_ref_supervised",
    "result",
    "rms_error_mm",
    "p95_error_mm",
    "servo_jitter_p99_ms",
    "q_ref_update_rate_hz",
    "q_ref_target_error_deg_max",
    "reference_supervision_state",
    "commands_enqueued_total",
    "commands_sent_total",
    "commands_acked_total",
    "commands_socket_sent_total",
    "commands_overwritten_total",
    "commands_dropped_total",
    "tracking_source",
    "reliability_caveats",
    "artifact_dir",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate a reporting-only 100 Hz vs 500 Hz rbpodo controller-"
            "simulation comparison from existing no-op and circle artifacts."
        )
    )
    parser.add_argument(
        "--noop-summary",
        action="append",
        type=Path,
        default=[],
        help="No-op summary.json from rainbow_rate_probe.py or rbpodo_500hz_acceptance.py.",
    )
    parser.add_argument(
        "--circle-summary",
        action="append",
        type=Path,
        default=[],
        help="Circle benchmark summary.json. May be passed multiple times.",
    )
    parser.add_argument(
        "--ablation-summary-csv",
        action="append",
        type=Path,
        default=[],
        help="run_rbpodo_circle_ablation.py ablation_summary.csv. May be passed multiple times.",
    )
    parser.add_argument("--output-md", type=Path, help="Write markdown report to this path instead of stdout.")
    parser.add_argument("--csv", dest="csv_path", type=Path, help="Write comparison CSV to this path.")
    parser.add_argument("--json", dest="json_path", type=Path, help="Write structured comparison JSON to this path.")
    parser.add_argument("--title", default=DEFAULT_TITLE)
    return parser.parse_args()


def finite_number(value: Any) -> float | None:
    return compare.finite_number(value)


def as_bool(value: Any) -> bool | None:
    return reliability_report.as_bool(value)


def first_present(row: dict[str, Any], *keys: str) -> Any:
    for key in keys:
        if key in row and row.get(key) not in (None, ""):
            return row.get(key)
    return None


def first_number(row: dict[str, Any], *keys: str) -> float | None:
    for key in keys:
        value = finite_number(row.get(key))
        if value is not None:
            return value
    return None


def nested_dict(row: dict[str, Any], key: str) -> dict[str, Any]:
    value = row.get(key)
    return value if isinstance(value, dict) else {}


def nested_number(row: dict[str, Any], block: str, metric: str) -> float | None:
    return finite_number(nested_dict(row, block).get(metric))


def ratio(numerator: Any, denominator: Any) -> float | None:
    top = finite_number(numerator)
    bottom = finite_number(denominator)
    if top is None or bottom is None or bottom <= 0.0:
        return None
    return top / bottom


def append_cell_value(row: dict[str, Any], key: str, value: str) -> None:
    existing = row.get(key)
    parts: list[str] = []
    if isinstance(existing, str):
        parts = [part.strip() for part in existing.split(";") if part.strip()]
    elif isinstance(existing, list):
        parts = [str(part).strip() for part in existing if str(part).strip()]
    if value not in parts:
        parts.append(value)
    row[key] = "; ".join(parts)


def load_json_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise SystemExit(f"failed to read JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise SystemExit(f"{path} does not contain a JSON object")
    value["_source"] = str(path)
    return value


def profile_key(value: Any) -> str:
    profile = str(value or "")
    aliases = {
        "noop": "noop_acceptance",
        "servo_j_noop": "noop_acceptance",
        "servo_j_noop_500hz": "noop_acceptance",
        "servo_j_simulation_only": "noop_acceptance",
        "rbpodo_servo_j_noop": "noop_acceptance",
        "safe_5cm10s": "safe_5cm_10s",
        "circle_safe_5cm_10s": "safe_5cm_10s",
        "15cm_16s": "circle_15cm_16s",
        "circle_15cm16s": "circle_15cm_16s",
        "15cm_8s": "circle_15cm_8s",
        "circle_15cm8s": "circle_15cm_8s",
        "15cm_4s": "gene_15cm_4s",
        "circle_15cm_4s": "gene_15cm_4s",
    }
    return aliases.get(profile, profile)


def comparison_label(profile: str) -> str:
    for key, label in COMPARISONS:
        if key == profile:
            return label
    return profile


def row_name(row: dict[str, Any]) -> str:
    for key in ("run_name", "name"):
        value = row.get(key)
        if isinstance(value, str) and value:
            return value
    artifact_dir = row.get("artifact_dir")
    if isinstance(artifact_dir, str) and artifact_dir:
        return Path(artifact_dir).name
    source = row.get("_source")
    if isinstance(source, str) and source:
        return Path(source).parent.name or Path(source).name
    return ""


def command_count(row: dict[str, Any]) -> float | None:
    return first_number(row, "send_count", "command_count", "ack_observed_count", "controller_acceptance_observed_count")


def success_rate(row: dict[str, Any]) -> float | None:
    existing = first_number(row, "send_success_rate")
    if existing is not None:
        return existing
    count = command_count(row)
    from_success = ratio(first_present(row, "send_success_count", "send_ok_count"), count)
    if from_success is not None:
        return from_success
    failures = finite_number(row.get("send_failure_count"))
    if failures is not None and count is not None and count > 0.0:
        return max(0.0, 1.0 - failures / count)
    return None


def controller_acceptance_rate(row: dict[str, Any]) -> float | None:
    existing = first_number(row, "controller_acceptance_observed_rate", "controller_acceptance_ratio")
    if existing is not None:
        return existing
    return ratio(row.get("controller_acceptance_observed_count"), command_count(row))


def p99_loop_jitter_ms(row: dict[str, Any]) -> float | None:
    existing = first_number(row, "servo_jitter_p99_ms")
    if existing is not None:
        return existing
    nested = nested_number(row, "servo_jitter_ms", "p99")
    if nested is not None:
        return nested
    loop_p99 = first_number(row, "loop_interval_p99_ms")
    if loop_p99 is None:
        return None
    feedback_rate = first_number(row, "feedback_rate_hz", "rate_hz", "requested_rate_hz")
    if feedback_rate is None or feedback_rate <= 0.0:
        return None
    return abs(loop_p99 - 1000.0 / feedback_rate)


def flatten_metrics(row: dict[str, Any]) -> None:
    async_fields = compare.async_report_fields(row)
    row.update(async_fields)
    row["send_success_rate"] = success_rate(row)
    row["controller_acceptance_observed_rate"] = 0.0 if row.get("socket_send_only") else controller_acceptance_rate(row)
    row["send_duration_p99_us"] = first_number(row, "send_duration_p99_us") or nested_number(row, "send_duration_us", "p99")
    row["send_duration_max_us"] = first_number(row, "send_duration_max_us") or nested_number(row, "send_duration_us", "max")
    row["servo_jitter_p99_ms"] = p99_loop_jitter_ms(row)
    row["deadline_miss_count"] = first_number(
        row,
        "deadline_miss_count",
        "send_deadline_missed_count",
        "send_command_deadline_missed_count",
        "command_sender_deadline_missed_count",
    )
    command_interval = first_number(row, "command_interval_max_ms", "loop_interval_max_ms")
    if command_interval is None:
        interval_block = nested_dict(nested_dict(row, "timestamp_alignment"), "command_interval_ms")
        command_interval = finite_number(interval_block.get("max"))
    row["command_interval_max_ms"] = command_interval
    if first_number(row, "orientation_p95_deg") is None:
        orientation_rad = first_number(row, "p95_orientation_drift_rad")
        if orientation_rad is not None:
            row["orientation_p95_deg"] = orientation_rad * 180.0 / math.pi
    if first_number(row, "rms_error_mm") is None:
        rms_m = first_number(row, "rms_error_m")
        if rms_m is not None:
            row["rms_error_mm"] = rms_m * 1000.0
    if first_number(row, "p95_error_mm") is None:
        p95_m = first_number(row, "p95_error_m")
        if p95_m is not None:
            row["p95_error_mm"] = p95_m * 1000.0
    if first_number(row, "tail_ratio") is None:
        tail = nested_dict(row, "error_decomposition").get("tail_ratio")
        if tail is not None:
            row["tail_ratio"] = tail
    if row.get("socket_send_only"):
        append_cell_value(row, "reliability_caveats", "socket_send_only_not_controller_ack")
        append_cell_value(row, "benchmark_interpretation", "reference_supervision_required")


def normalize_row(row: dict[str, Any], *, source_kind: str) -> dict[str, Any]:
    normalized = dict(row)
    normalized["source_kind"] = source_kind
    if not normalized.get("run_name"):
        normalized["run_name"] = row_name(normalized)
    profile = profile_key(first_present(normalized, "profile", "mode"))
    normalized["profile"] = profile
    normalized["comparison"] = comparison_label(profile)
    rate = first_number(normalized, "rate_hz", "command_rate_hz", "servo_rate_hz", "requested_rate_hz")
    normalized["rate_hz"] = rate
    normalized.setdefault("backend", "rbpodo")
    normalized.setdefault("benchmark_category", "rbpodo_controller_simulation")
    normalized.setdefault("controller_mode", "pgmode_simulation")
    normalized.setdefault("physical_motion_expected", False)
    flatten_metrics(normalized)
    if source_kind == "noop_acceptance":
        normalized.setdefault("measurement_reliability_level", "acceptance_stage_noop")
        normalized.setdefault(
            "reliability_caveats",
            "single_arm_noop_only; not_physical_real_motion_proof",
        )
        normalized.setdefault("physical_real_blockers", "dual_arm_acceptance_missing; circle_tracking_missing")
    elif not normalized.get("measurement_reliability_level"):
        reliability_report.annotate_row(normalized)
    return normalized


def noop_rows_from_rate_probe(summary: dict[str, Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    preflight = nested_dict(summary, "safety_preflight")
    disable_waiting_ack = as_bool(first_present(summary, "disable_waiting_ack") if "disable_waiting_ack" in summary else preflight.get("disable_waiting_ack"))
    ack_policy = "ack_off" if disable_waiting_ack is True else "ack_on" if disable_waiting_ack is False else ""
    for result in summary.get("rate_results", []):
        if not isinstance(result, dict):
            continue
        row = dict(result)
        row["mode"] = "servo_j_simulation_only"
        row["profile"] = "noop_acceptance"
        row["source_kind"] = "noop_acceptance"
        row["run_name"] = f"servo_j_noop_{format_rate(row.get('requested_rate_hz'))}hz"
        row["artifact_dir"] = summary.get("artifact_dir")
        row["backend"] = summary.get("backend", "rbpodo")
        row["result"] = row.get("result") or summary.get("result")
        row["physical_motion_expected"] = False
        row["physical_motion_detected"] = False
        row["fault_latched"] = False
        row["ack_policy"] = ack_policy
        if ack_policy == "ack_off":
            row["acceptance_semantics"] = "socket_send_only"
        if ack_policy == "ack_on" and row.get("controller_acceptance_observed_rate") is None:
            row["controller_acceptance_observed_rate"] = row.get("send_success_rate")
        rows.append(normalize_row(row, source_kind="noop_acceptance"))
    return rows


def noop_row_from_server_acceptance(summary: dict[str, Any]) -> dict[str, Any]:
    row = dict(summary)
    row["profile"] = "noop_acceptance"
    row["source_kind"] = "noop_acceptance"
    row["rate_hz"] = first_number(row, "command_rate_hz", "servo_rate_hz") or 500.0
    row["run_name"] = row.get("run_name") or f"server_noop_{format_rate(row['rate_hz'])}hz"
    if row.get("controller_acceptance_observed_rate") is None:
        row["controller_acceptance_observed_rate"] = row.get("controller_acceptance_ratio")
    return normalize_row(row, source_kind="noop_acceptance")


def load_noop_summary(path: Path) -> list[dict[str, Any]]:
    summary = load_json_object(path)
    if isinstance(summary.get("rate_results"), list):
        return noop_rows_from_rate_probe(summary)
    return [noop_row_from_server_acceptance(summary)]


def load_circle_summary(path: Path) -> dict[str, Any]:
    summary = load_json_object(path)
    row = compare.comparison_row(summary)
    row["_source"] = str(path)
    return normalize_row(row, source_kind="circle")


def load_ablation_csv(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            row = dict(raw)
            row["_source"] = str(path)
            row["run_name"] = row.get("run_name") or row.get("name") or ""
            rows.append(normalize_row(row, source_kind="circle"))
    return rows


def format_rate(value: Any) -> str:
    number = finite_number(value)
    if number is None:
        return "unknown"
    if abs(number - round(number)) < 1e-9:
        return str(int(round(number)))
    return f"{number:g}"


def rate_matches(row: dict[str, Any], expected: float) -> bool:
    rate = finite_number(row.get("rate_hz"))
    return rate is not None and abs(rate - expected) <= 1e-6


def row_lane(row: dict[str, Any] | None) -> str:
    if row is None:
        return ""
    rate = first_number(row, "rate_hz")
    if rate is not None and abs(rate - 100.0) <= 1e-6:
        return "100hz_ack_on_best"
    async_mode = str(row.get("async_mode") or "disabled")
    if async_mode == "sdk_ack_worker":
        return "500hz_async_sdk_ack_worker"
    if async_mode == "socket_send_supervised" or row.get("socket_send_only"):
        return "500hz_socket_send_supervised"
    return "500hz_ack_on"


def reference_watchdog_failed(row: dict[str, Any] | None) -> bool:
    if row is None:
        return False
    reference_state = str(row.get("reference_supervision_state") or "")
    if reference_state in {"fault", "failed", "watchdog_failed", "stale", "timeout"}:
        return True
    if (first_number(row, "q_ref_watchdog_miss_count") or 0.0) > 0.0:
        return True
    if (first_number(row, "supervision_fault_count") or 0.0) > 0.0:
        return True
    if row.get("socket_send_only") and reference_state not in {"", "ok", "configured"}:
        return True
    return False


def ack_on_blocking_limited(row: dict[str, Any] | None) -> bool:
    if row is None or row.get("socket_send_only"):
        return False
    async_mode = str(row.get("async_mode") or "disabled")
    if async_mode == "sdk_ack_worker":
        return False
    if as_bool(row.get("servo_loop_blocked_by_ack")) is True:
        return True
    joined = " ".join(
        str(row.get(key) or "")
        for key in ("failure_classification", "result_reason", "timing_classification")
    ).lower()
    if "ack_timeout" in joined or "blocked_by_ack" in joined:
        return True
    if (first_number(row, "deadline_miss_count") or 0.0) > 0.0:
        return True
    send_p99 = first_number(row, "send_duration_p99_us")
    return send_p99 is not None and send_p99 > 2000.0


def unstable_reasons(row: dict[str, Any] | None) -> list[str]:
    if row is None:
        return ["missing_row"]
    reasons: list[str] = []
    result = str(row.get("result") or "").lower()
    if result in {"fail", "failed", "error", "blocked", "faulted", "startup_fault", "state_stream_timeout"}:
        reasons.append(f"result={result}")
    if as_bool(row.get("fault_latched")) is True:
        reasons.append("fault_latched=true")
    if as_bool(row.get("physical_motion_detected")) is True:
        reasons.append("physical_motion_detected=true")
    if as_bool(row.get("server_rejected_cartesian")) is True:
        reasons.append("server_rejected_cartesian=true")
    if (first_number(row, "cartesian_unavailable_count") or 0.0) > 0.0:
        reasons.append("cartesian_unavailable_count>0")
    if (first_number(row, "deadline_miss_count") or 0.0) > 0.0:
        reasons.append("deadline_miss_count>0")
    if reference_watchdog_failed(row):
        reasons.append("reference_supervision_failed")
    if row.get("measurement_reliability_level") == "unreliable":
        reasons.append("measurement_reliability_level=unreliable")
    if row.get("source_kind") == "noop_acceptance":
        if (first_number(row, "send_success_rate") or 0.0) < 0.98:
            reasons.append("send_success_rate<0.98")
        acceptance = first_number(row, "controller_acceptance_observed_rate")
        if acceptance is not None and acceptance < 0.98:
            reasons.append("controller_acceptance_observed_rate<0.98")
    return reasons


def row_unstable(row: dict[str, Any] | None) -> bool:
    return bool(unstable_reasons(row))


def reliability_usable(row: dict[str, Any] | None) -> bool:
    if row is None:
        return False
    if row.get("source_kind") == "noop_acceptance":
        return True
    level = str(row.get("measurement_reliability_level") or "")
    return level in {"controller_reference_valid", "measured", "not_applicable"}


def noop_pass(row: dict[str, Any] | None) -> bool:
    if row is None or row_unstable(row):
        return False
    if row.get("ack_policy") == "ack_off":
        return False
    success = first_number(row, "send_success_rate")
    if success is None or success < 0.999:
        return False
    acceptance = first_number(row, "controller_acceptance_observed_rate")
    if acceptance is not None and acceptance < 0.98:
        return False
    send_max = first_number(row, "send_duration_max_us")
    if send_max is not None and send_max > 1000.0:
        return False
    drift = first_number(row, "q_actual_drift_max_deg", "q_actual_drift_from_start_deg")
    if drift is not None and drift > 0.05:
        return False
    error_counts = [
        first_number(row, "m561_count") or 0.0,
        first_number(row, "m568_count") or 0.0,
        first_number(row, "m569_count") or 0.0,
        first_number(row, "m570_count") or 0.0,
        first_number(row, "send_failure_count") or 0.0,
    ]
    return all(value == 0.0 for value in error_counts)


def supervised_pass(row: dict[str, Any] | None) -> bool:
    if row is None:
        return False
    if row.get("source_kind") == "noop_acceptance":
        if row.get("socket_send_only"):
            return bool(row.get("q_ref_supervised")) and not row_unstable(row)
        return noop_pass(row)
    if row_unstable(row):
        return False
    if not reliability_usable(row):
        return False
    if row.get("socket_send_only") and not row.get("q_ref_supervised"):
        return False
    return first_number(row, "rms_error_mm") is not None and first_number(row, "p95_error_mm") is not None


def row_sort_key(row: dict[str, Any]) -> tuple[Any, ...]:
    if row.get("source_kind") == "noop_acceptance":
        return (not noop_pass(row), row_unstable(row), row_name(row))
    controller = str(row.get("controller") or "")
    is_open_loop = not controller.endswith("_feedback")
    return (
        row_unstable(row),
        not reliability_usable(row),
        is_open_loop,
        first_number(row, "rms_error_mm") is None,
        first_number(row, "rms_error_mm") or math.inf,
        first_number(row, "p95_error_mm") or math.inf,
        first_number(row, "servo_jitter_p99_ms") or math.inf,
        first_number(row, "feedback_saturation_count") or 0.0,
        row_name(row),
    )


def selected_row(rows: list[dict[str, Any]], profile: str, rate_hz: float) -> dict[str, Any] | None:
    candidates = [row for row in rows if row.get("profile") == profile and rate_matches(row, rate_hz)]
    if not candidates:
        return None
    return sorted(candidates, key=row_sort_key)[0]


def row_matches_lane(row: dict[str, Any], lane: str, rate_hz: float) -> bool:
    if not rate_matches(row, rate_hz):
        return False
    if lane == "100hz_ack_on_best":
        return not row.get("socket_send_only") and str(row.get("async_mode") or "disabled") in {"", "disabled"}
    if lane == "500hz_ack_on":
        return (
            not row.get("socket_send_only")
            and str(row.get("async_mode") or "disabled") in {"", "disabled"}
            and rate_matches(row, 500.0)
        )
    if lane == "500hz_socket_send_supervised":
        return row.get("socket_send_only") is True or str(row.get("async_mode") or "") == "socket_send_supervised"
    if lane == "500hz_async_sdk_ack_worker":
        return str(row.get("async_mode") or "") == "sdk_ack_worker"
    return False


def selected_lane_row(rows: list[dict[str, Any]], profile: str, lane: str, rate_hz: float) -> dict[str, Any] | None:
    candidates = [
        row for row in rows
        if row.get("profile") == profile and row_matches_lane(row, lane, rate_hz)
    ]
    if not candidates:
        return None
    return sorted(candidates, key=row_sort_key)[0]


def comparative_row(profile: str, lane: str, row: dict[str, Any] | None) -> dict[str, Any]:
    base = {
        "comparison": comparison_label(profile),
        "profile": profile,
        "lane": lane,
        "evidence_present": row is not None,
    }
    if row is None:
        return base
    for key in COMPARATIVE_COLUMNS:
        if key not in base:
            base[key] = row.get(key)
    base["run_name"] = row_name(row)
    return base


def build_comparative_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    comparative: list[dict[str, Any]] = []
    for profile, _label in COMPARISONS:
        for lane, rate_hz in COMPARATIVE_LANES:
            comparative.append(comparative_row(profile, lane, selected_lane_row(rows, profile, lane, rate_hz)))
    return comparative


def metric_improved(row100: dict[str, Any], row500: dict[str, Any]) -> bool | None:
    rms100 = first_number(row100, "rms_error_mm")
    rms500 = first_number(row500, "rms_error_mm")
    if rms100 is None or rms500 is None:
        return None
    rms_delta = rms100 - rms500
    rms_better = rms_delta > max(0.25, rms100 * 0.02)
    p95_100 = first_number(row100, "p95_error_mm")
    p95_500 = first_number(row500, "p95_error_mm")
    p95_not_worse = True if p95_100 is None or p95_500 is None else p95_500 <= p95_100 * 1.02
    jitter100 = first_number(row100, "servo_jitter_p99_ms")
    jitter500 = first_number(row500, "servo_jitter_p99_ms")
    jitter_not_worse = True if jitter100 is None or jitter500 is None else jitter500 <= jitter100 * 1.05
    return rms_better and p95_not_worse and jitter_not_worse


def tracking_delta_interpretation(row100: dict[str, Any] | None, row500: dict[str, Any] | None) -> str:
    if row100 is None or row500 is None:
        return "missing_reference_or_candidate"
    improved = metric_improved(row100, row500)
    if improved is True:
        return "tracking_improved"
    if improved is False:
        jitter100 = first_number(row100, "servo_jitter_p99_ms")
        jitter500 = first_number(row500, "servo_jitter_p99_ms")
        send100 = first_number(row100, "send_duration_p99_us")
        send500 = first_number(row500, "send_duration_p99_us")
        timing_improved = (
            (jitter100 is not None and jitter500 is not None and jitter500 < jitter100)
            or (send100 is not None and send500 is not None and send500 < send100)
        )
        return "timing_only_or_no_tracking_improvement" if timing_improved else "no_tracking_improvement"
    return "missing_tracking_metrics"


def caveats_for_pair(row100: dict[str, Any] | None, row500: dict[str, Any] | None) -> list[str]:
    caveats = [
        "socket_send_only is not per-command controller ACK",
        "tcp_ref_stand is controller-reference lower bound",
        "diagnostics_suspect unresolved",
        "physical real not proven",
        "dual-arm acceptance required before default change",
    ]
    for row in (row100, row500):
        if row is None:
            continue
        if "diagnostics_suspect" in str(row.get("reliability_caveats") or ""):
            caveats.append("diagnostics_suspect_unresolved")
        if row.get("measurement_reliability_level") == "suspect":
            caveats.append("suspect_measurement_not_promotion_evidence")
    return list(dict.fromkeys(caveats))


def classify_pair(profile: str, row100: dict[str, Any] | None, row500: dict[str, Any] | None) -> dict[str, Any]:
    label = comparison_label(profile)
    reasons: list[str] = []
    tracking_interpretation = tracking_delta_interpretation(row100, row500)
    if row500 is None:
        classification = "insufficient_evidence"
        reasons.append("missing 500 Hz row")
    elif as_bool(row500.get("physical_motion_detected")) is True:
        classification = "500hz_unstable"
        reasons.append("physical_motion_detected=true")
    elif reference_watchdog_failed(row500):
        classification = "500hz_reference_watchdog_failed"
        reasons.append("q_ref/tcp_ref reference supervision watchdog failed")
    elif ack_on_blocking_limited(row500):
        classification = "500hz_ack_on_blocking_limited"
        reasons.append("500 Hz synchronous ACK-on appears limited by blocking ACK/timing")
    elif row_unstable(row500):
        classification = "500hz_unstable"
        reasons.extend(unstable_reasons(row500))
    elif row500.get("socket_send_only"):
        if supervised_pass(row500):
            classification = "500hz_socket_send_only_promising"
            reasons.append("socket_send_only tracking is promising but is not controller ACK evidence")
            if tracking_interpretation:
                reasons.append(tracking_interpretation)
        else:
            classification = "insufficient_evidence"
            reasons.append("socket_send_only row lacks q_ref-supervised acceptance evidence")
    elif profile == "noop_acceptance":
        if supervised_pass(row500):
            classification = "500hz_async_supervised_pass"
            reasons.append("500 Hz no-op satisfied ACK/q_ref-supervised report checks")
        else:
            classification = "insufficient_evidence"
            reasons.append("500 Hz no-op missing required supervised pass metrics")
    elif row100 is None:
        classification = "insufficient_evidence"
        reasons.append("missing 100 Hz row")
    elif row_unstable(row100):
        classification = "insufficient_evidence"
        reasons.append("100 Hz reference row is unstable")
        reasons.extend(unstable_reasons(row100))
    elif not reliability_usable(row500):
        classification = "insufficient_evidence"
        reasons.append(f"500 Hz measurement reliability is {row500.get('measurement_reliability_level')}")
    elif not reliability_usable(row100):
        classification = "insufficient_evidence"
        reasons.append(f"100 Hz measurement reliability is {row100.get('measurement_reliability_level')}")
    else:
        improved = metric_improved(row100, row500)
        if improved is None:
            classification = "insufficient_evidence"
            reasons.append("missing RMS comparison metric")
        elif supervised_pass(row500):
            classification = "500hz_async_supervised_pass"
            if improved:
                reasons.append("500 Hz has lower RMS without worse p95 or p99 jitter evidence")
            else:
                reasons.append("500 Hz supervised row is stable but tracking improvement is not shown")
            reasons.append(tracking_interpretation)
        else:
            classification = "insufficient_evidence"
            reasons.append("500 Hz row lacks supervised pass evidence")

    return {
        "comparison": label,
        "profile": profile,
        "classification": classification,
        "rate_100_run": row_name(row100) if row100 else "",
        "rate_500_run": row_name(row500) if row500 else "",
        "rate_100_lane": row_lane(row100),
        "rate_500_lane": row_lane(row500),
        "tracking_delta_interpretation": tracking_interpretation,
        "recommendation": recommendation_for_classification(classification),
        "classification_reason": "; ".join(dict.fromkeys(reasons)),
        "caveats": "; ".join(caveats_for_pair(row100, row500)),
        **prefixed_metrics("rate_100", row100),
        **prefixed_metrics("rate_500", row500),
    }


def prefixed_metrics(prefix: str, row: dict[str, Any] | None) -> dict[str, Any]:
    return {f"{prefix}_{field}": (row.get(field) if row else None) for field in KEY_FIELDS}


def recommendation_for_classification(classification: str) -> str:
    if classification == "500hz_async_supervised_pass":
        return "controller_simulation_experimental_candidate_only"
    if classification == "500hz_socket_send_only_promising":
        return "keep_as_promising_socket_send_q_ref_supervised_evidence_only"
    if classification == "500hz_ack_on_blocking_limited":
        return "do_not_promote_500hz_ack_on_without_async_or_timeout_fix"
    if classification == "500hz_reference_watchdog_failed":
        return "stop_and_fix_reference_supervision_before_interpreting_tracking"
    if classification == "500hz_unstable":
        return "stop_stage_sequence_and_fix_faults_timing_or_safety"
    return "collect_missing_100hz_500hz_evidence"


def build_report(rows: list[dict[str, Any]]) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    selected: list[dict[str, Any]] = []
    comparisons: list[dict[str, Any]] = []
    for profile, _label in COMPARISONS:
        row100 = selected_row(rows, profile, 100.0)
        row500 = selected_row(rows, profile, 500.0)
        if row100 is not None:
            selected.append(row100)
        if row500 is not None:
            selected.append(row500)
        comparisons.append(classify_pair(profile, row100, row500))
    return selected, comparisons, build_comparative_rows(rows)


def format_cell(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        if abs(value) >= 100.0:
            return f"{value:.3f}"
        if abs(value) >= 1.0:
            return f"{value:.4f}"
        return f"{value:.6f}"
    text = str(value)
    return text.replace("|", "\\|").replace("\n", " ")


def markdown_table(rows: list[dict[str, Any]], columns: list[str]) -> str:
    lines = [
        "| " + " | ".join(columns) + " |",
        "| " + " | ".join("---" for _ in columns) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(format_cell(row.get(key)) for key in columns) + " |")
    return "\n".join(lines)


def report_markdown(
    rate_rows: list[dict[str, Any]],
    comparisons: list[dict[str, Any]],
    comparative_rows: list[dict[str, Any]],
    title: str,
) -> str:
    return "\n".join(
        [
            f"# {title}",
            "",
            "This is rbpodo controller-simulation report evidence only. It does not authorize physical robot motion or a default-rate change.",
            "",
            "## Acceptance Semantics",
            "",
            "- `controller_ack_observed`: synchronous ACK-on command calls observed per-command controller ACK.",
            "- `sdk_worker_ack_observed`: async `sdk_ack_worker` observed controller ACK in the worker thread.",
            "- `socket_send_only`: command write/socket/API send evidence only; not per-command controller ACK.",
            "- `q_ref_supervised`: controller-reference watchdog evidence was present and OK.",
            "",
            "## Comparison Classifications",
            "",
            markdown_table(comparisons, COMPARISON_COLUMNS),
            "",
            "## Comparative Evidence Table",
            "",
            markdown_table(comparative_rows, COMPARATIVE_COLUMNS) if comparative_rows else "_No comparative rows._",
            "",
            "## Selected Rate Evidence",
            "",
            markdown_table(rate_rows, RATE_COLUMNS) if rate_rows else "_No selected rate rows._",
            "",
            "## Caveats",
            "",
            "- socket_send_only is not per-command controller ACK.",
            "- `tcp_ref_stand` is controller-reference lower bound.",
            "- diagnostics_suspect unresolved.",
            "- physical real not proven.",
            "- dual-arm acceptance required before default change.",
            "",
            "## Recommendation",
            "",
            "- Do not change the default rate automatically.",
            "- Allow a 500 Hz rbpodo controller-simulation experimental profile.",
            "- Promote only after stable 5 cm / 10 s and 15 cm / 16 s controller-simulation evidence passes with usable measurement reliability.",
        ]
    ) + "\n"


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=COMPARISON_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in COMPARISON_COLUMNS})


def write_json(
    path: Path,
    rate_rows: list[dict[str, Any]],
    comparisons: list[dict[str, Any]],
    comparative_rows: list[dict[str, Any]],
) -> None:
    payload = {
        "schema": SCHEMA,
        "rate_rows": rate_rows,
        "comparisons": comparisons,
        "comparative_rows": comparative_rows,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def load_all(args: argparse.Namespace) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for path in args.noop_summary:
        rows.extend(load_noop_summary(path))
    for path in args.circle_summary:
        rows.append(load_circle_summary(path))
    for path in args.ablation_summary_csv:
        rows.extend(load_ablation_csv(path))
    return rows


def main() -> int:
    args = parse_args()
    rows = load_all(args)
    rate_rows, comparisons, comparative_rows = build_report(rows)
    markdown = report_markdown(rate_rows, comparisons, comparative_rows, args.title)
    if args.output_md:
        args.output_md.parent.mkdir(parents=True, exist_ok=True)
        args.output_md.write_text(markdown, encoding="utf-8")
    else:
        print(markdown, end="")
    if args.csv_path:
        write_csv(args.csv_path, comparisons)
    if args.json_path:
        write_json(args.json_path, rate_rows, comparisons, comparative_rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
