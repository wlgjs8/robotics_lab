#!/usr/bin/env python3
"""Generate ACKON500-GENE-GOAL-01 pass/fail evidence.

The report is intentionally stricter than the generic ablation report. It only
passes a GENE 15 cm / 4 s 500 Hz row when ACK-observed command telemetry,
tracking quality, timing, artifact, and pgmode-simulation safety criteria all
hold at the same time.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any

import circle_tracking_benchmark as sim_bench


SCHEMA = "robotics_lab.ackon500_gene_goal_report.v1"
ACKON500_PHYSICAL_WARNING = (
    "ACKON500 PASS is controller-reference lower-bound evidence, not physical TCP tracking."
)
CONTROLLER_REFERENCE_EXPLANATION = "tcp_ref_stand lower-bound evidence"
PHYSICAL_READINESS_BLOCKERS = [
    "diagnostics_suspect_unresolved",
    "physical_reference_to_actual_error_unmeasured",
    "stop_resetFault_unverified",
    "camera_tcp_calibration_unresolved",
    "no_tiny_physical_acceptance",
]
NEXT_REQUIRED_ACCEPTANCE = [
    "read-only diagnostics parity",
    "tiny joint no-op physical or approved safe mode",
    "tiny physical joint move",
    "tiny physical Cartesian move",
    "low-speed circle",
    "then speed ladder",
]
PASS_THRESHOLDS = {
    "min_repeat": 5,
    "servo_rate_hz": 500.0,
    "servo_t1_sec": 0.002,
    "command_rate_hz": 500.0,
    "phase_advance_sec": 0.005,
    "min_ack_ratio": 0.98,
    "min_effective_goal_command_rate_hz": 490.0,
    "max_effective_goal_command_rate_hz": 510.0,
    "max_saturation_ratio": 0.01,
    "max_rms_error_m": 0.003,
    "max_p95_error_m": 0.006,
    "max_fit_center_error_m": 0.003,
    "min_radius_gain": 0.98,
    "max_radius_gain": 1.02,
    "max_p95_orientation_drift_rad": 0.02,
    "max_effective_phase_latency_abs_ms": 5.0,
    "max_state_age_p95_us": 5000.0,
}
REPEATABILITY_THRESHOLDS = {
    "required_repeats_per_arm": 3,
    "required_arms": ["left", "right"],
    "min_ack_ratio": 0.98,
    "max_median_rms_error_mm": 3.0,
    "max_worst_rms_error_mm": 3.5,
    "max_median_p95_error_mm": 6.0,
    "max_worst_p95_error_mm": 8.0,
}
REPEATABILITY_REQUIRED_RUN_NAMES = [
    "best_left_run01",
    "best_left_run02",
    "best_left_run03",
    "best_right_run01",
    "best_right_run02",
    "best_right_run03",
]
REPEATABILITY_ROW_COLUMNS = [
    "name",
    "arm",
    "profile",
    "controller",
    "goal_pass",
    "ackon500_goal_status",
    "run_result_status",
    "safety_result_status",
    "benchmark_lane",
    "low_level_send_mode",
    "async_mode",
    "acceptance_semantics",
    "tracking_source",
    "repeat",
    "command_rate_hz",
    "phase_advance_sec",
    "rms_error_mm",
    "p95_error_mm",
    "latency_ms",
    "ack_observed_ratio",
    "state_age_p95_us",
    "socket_send_only_count",
    "left_disable_waiting_ack",
    "right_disable_waiting_ack",
    "cartesian_allow_in_controller_simulation",
    "cartesian_allow_in_real",
    "fault_latched",
    "physical_motion_detected",
    "physical_motion_expected",
    "measurement_reliability_level",
    "diagnostic_warnings",
    "failures",
    "artifact_dir",
]
REPEATABILITY_AGGREGATE_COLUMNS = [
    "classification",
    "required_run_count",
    "required_pass_count",
    "rms_mean",
    "rms_std",
    "rms_min",
    "rms_max",
    "p95_mean",
    "p95_std",
    "p95_min",
    "p95_max",
    "latency_mean",
    "latency_std",
    "latency_min",
    "latency_max",
    "ack_observed_ratio_min",
    "state_age_p95_max",
]
REPEATABILITY_GROUP_COLUMNS = [
    "group",
    "count",
    "pass_count",
    "fail_count",
    "rows",
]
REPEATABILITY_ARM_AGGREGATE_COLUMNS = [
    "arm",
    "status",
    "required_run_count",
    "required_pass_count",
    "rms_median",
    "rms_max",
    "p95_median",
    "p95_max",
    "ack_observed_ratio_min",
    "state_age_p95_max",
]


class ReportError(RuntimeError):
    pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate ACKON500-GENE-GOAL-01 summary.json and markdown reports."
    )
    parser.add_argument("--artifact-root", type=Path, required=True)
    parser.add_argument("--output-summary", type=Path)
    parser.add_argument("--output-md", type=Path)
    parser.add_argument("--timing-report", type=Path)
    parser.add_argument("--error-report", type=Path)
    parser.add_argument("--repeatability-summary-json", type=Path)
    parser.add_argument("--repeatability-summary-csv", type=Path)
    parser.add_argument("--repeatability-report", type=Path)
    parser.add_argument("--require-pass", action="store_true")
    parser.add_argument("--require-repeatable", action="store_true")
    return parser.parse_args()


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


def load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise ReportError(f"failed to read JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ReportError(f"expected JSON object: {path}")
    return value


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.rstrip() + "\n", encoding="utf-8")


def physical_readiness() -> dict[str, Any]:
    return {
        "status": "blocked",
        "blockers": list(PHYSICAL_READINESS_BLOCKERS),
        "next_required_acceptance": list(NEXT_REQUIRED_ACCEPTANCE),
    }


def controller_reference_result(passed: bool) -> dict[str, str]:
    return {
        "status": "pass" if passed else "fail",
        "explanation": CONTROLLER_REFERENCE_EXPLANATION,
    }


def physical_tracking_result() -> dict[str, str]:
    return {"status": "not_measured"}


def nested_metric(summary: dict[str, Any], key: str, metric: str) -> float | None:
    value = summary.get(key)
    if isinstance(value, dict):
        return finite_number(value.get(metric))
    return None


def first_number(*values: Any) -> float | None:
    for value in values:
        number = finite_number(value)
        if number is not None:
            return number
    return None


def first_value(*values: Any) -> Any:
    for value in values:
        if value not in (None, ""):
            return value
    return None


def scaled_number(value: Any, factor: float) -> float | None:
    number = finite_number(value)
    return number * factor if number is not None else None


def nested_dict(summary: dict[str, Any], key: str) -> dict[str, Any]:
    value = summary.get(key)
    return value if isinstance(value, dict) else {}


def summary_or_nested(summary: dict[str, Any], nested_key: str, key: str) -> Any:
    if key in summary and summary.get(key) is not None:
        return summary.get(key)
    return nested_dict(summary, nested_key).get(key)


def text_list(value: Any) -> list[str]:
    if value is None:
        return []
    if isinstance(value, list):
        return [str(item) for item in value if str(item)]
    if isinstance(value, str):
        if not value:
            return []
        if ";" in value:
            return [item.strip() for item in value.split(";") if item.strip()]
        return [value]
    return [str(value)]


def infer_run_result(summary: dict[str, Any]) -> dict[str, str]:
    if summary.get("run_result_status") not in (None, ""):
        return {
            "status": str(summary.get("run_result_status")),
            "reason": str(summary.get("run_result_reason") or summary.get("result_reason") or ""),
        }
    status = summary_or_nested(summary, "run_result", "status")
    reason = summary_or_nested(summary, "run_result", "reason") or summary.get("result_reason")
    if status not in (None, ""):
        return {"status": str(status), "reason": str(reason or "")}
    result = str(summary.get("result") or "")
    result_reason = str(reason or "")
    if result in {"completed", "pass"}:
        return {"status": "completed", "reason": result_reason or "run completed"}
    if result == "fail" and "threshold" in result_reason:
        return {"status": "completed", "reason": result_reason}
    if result in {"error", "blocked", "faulted", "startup_fault"}:
        return {"status": result, "reason": result_reason}
    return {"status": result or "completed", "reason": result_reason}


def infer_benchmark_threshold_result(summary: dict[str, Any]) -> dict[str, Any]:
    if summary.get("benchmark_threshold_status") not in (None, ""):
        return {
            "status": str(summary.get("benchmark_threshold_status")),
            "threshold_failures": text_list(summary.get("threshold_failures")),
            "threshold_warnings": text_list(summary.get("threshold_warnings") or summary.get("performance_warnings")),
        }
    block = nested_dict(summary, "benchmark_threshold_result")
    if block.get("status") not in (None, ""):
        return {
            "status": block.get("status"),
            "threshold_failures": text_list(block.get("threshold_failures")),
            "threshold_warnings": text_list(block.get("threshold_warnings")),
        }
    failures = text_list(summary.get("threshold_failures"))
    warnings = text_list(summary.get("threshold_warnings"))
    if not warnings:
        warnings = text_list(summary.get("performance_warnings"))
    result = str(summary.get("result") or "")
    reason = str(summary.get("result_reason") or "")
    if failures:
        status = "fail"
    elif "threshold" in reason and result in {"pass", "fail"}:
        status = result
    else:
        status = "not_evaluated"
    return {
        "status": status,
        "threshold_failures": failures,
        "threshold_warnings": warnings,
    }


def infer_safety_result(summary: dict[str, Any], run_result: dict[str, str]) -> dict[str, Any]:
    if summary.get("safety_result_status") not in (None, ""):
        return {
            "fault_latched": as_bool(summary.get("fault_latched")) is True,
            "physical_motion_detected": as_bool(summary.get("physical_motion_detected")) is True,
            "cartesian_unavailable_count": int(finite_number(summary.get("cartesian_unavailable_count")) or 0),
            "status": str(summary.get("safety_result_status")),
        }
    block = nested_dict(summary, "safety_result")
    if block.get("status") not in (None, ""):
        return {
            "fault_latched": as_bool(block.get("fault_latched")) is True,
            "physical_motion_detected": as_bool(block.get("physical_motion_detected")) is True,
            "cartesian_unavailable_count": int(finite_number(block.get("cartesian_unavailable_count")) or 0),
            "status": str(block.get("status")),
        }
    fault_latched = as_bool(summary.get("fault_latched")) is True or run_result.get("status") in {"faulted", "startup_fault"}
    physical_motion = as_bool(summary.get("physical_motion_detected")) is True
    cartesian_unavailable = int(finite_number(summary.get("cartesian_unavailable_count")) or 0)
    status = (
        "fail"
        if fault_latched or physical_motion or cartesian_unavailable > 0 or run_result.get("status") in {"error", "blocked"}
        else "pass"
    )
    return {
        "fault_latched": fault_latched,
        "physical_motion_detected": physical_motion,
        "cartesian_unavailable_count": cartesian_unavailable,
        "status": status,
    }


def diagnostic_warnings_from_summary(summary: dict[str, Any], threshold_result: dict[str, Any]) -> list[str]:
    warnings = text_list(summary.get("diagnostic_warnings"))
    joined = " ".join(
        [
            *text_list(threshold_result.get("threshold_failures")),
            *text_list(threshold_result.get("threshold_warnings")),
        ]
    )
    if "max_orientation_drift_rad" in joined and "max_orientation_drift_spike" not in warnings:
        warnings.append("max_orientation_drift_spike")
    if (finite_number(summary.get("controller_simulation_diagnostic_override_active_count")) or 0.0) > 0.0:
        warnings.append("diagnostics_suspect_override_active")
    elif (finite_number(summary.get("diagnostics_suspect_count")) or 0.0) > 0.0:
        warnings.append("diagnostics_suspect_override_active")
    if (summary.get("tracking_source_used") or summary.get("tracking_source")) == "tcp_ref_stand":
        warnings.append("controller_reference_lower_bound")
    if (finite_number(summary.get("max_over_p95")) or 0.0) >= 3.0:
        warnings.append("max_error_spike")
    timing = summary.get("timing_classification")
    if timing not in (None, "", "clean_timing"):
        warnings.append("timing_spike")
    return list(dict.fromkeys(warnings))


def read_ablation_rows(artifact_root: Path) -> dict[str, dict[str, str]]:
    path = artifact_root / "ablation_summary.csv"
    if not path.is_file():
        return {}
    rows: dict[str, dict[str, str]] = {}
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            artifact_dir = row.get("artifact_dir")
            if artifact_dir:
                rows[str(Path(artifact_dir).resolve())] = row
            name = row.get("name")
            if name:
                rows[name] = row
    return rows


def summary_paths(artifact_root: Path) -> list[Path]:
    paths = [
        path for path in artifact_root.rglob("summary.json")
        if path.parent != artifact_root
    ]
    return sorted(paths)


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    rows: list[dict[str, Any]] = []
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                value = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(value, dict):
                rows.append(value)
    return rows


def arm_state(snapshot: dict[str, Any], arm: str) -> dict[str, Any] | None:
    value = snapshot.get(arm)
    return value if isinstance(value, dict) else None


def async_state(snapshot: dict[str, Any], arm: str) -> dict[str, Any] | None:
    state = arm_state(snapshot, arm)
    if state is None:
        return None
    value = state.get("async_streaming")
    return value if isinstance(value, dict) else None


def max_counter(samples: list[dict[str, Any]], key: str) -> int:
    values: list[int] = []
    for sample in samples:
        number = finite_number(sample.get(key))
        if number is not None and number >= 0:
            values.append(int(number))
    return max(values, default=0)


def timestamp_extent(samples: list[dict[str, Any]], key: str, *, first: bool) -> int:
    values: list[int] = []
    for sample in samples:
        number = finite_number(sample.get(key))
        if number is not None and number > 0:
            values.append(int(number))
    if not values:
        return 0
    return min(values) if first else max(values)


def async_metrics(states: list[dict[str, Any]], arm: str) -> dict[str, Any]:
    samples = [
        sample
        for snapshot in states
        for sample in [async_state(snapshot, arm)]
        if sample is not None
    ]
    modes = [str(sample.get("mode")) for sample in samples if sample.get("mode") not in (None, "")]
    mode = max(set(modes), key=modes.count) if modes else None
    phases = [str(sample.get("command_phase")) for sample in samples if sample.get("command_phase") not in (None, "")]
    phase = max(set(phases), key=phases.count) if phases else None
    return {
        "sample_count": len(samples),
        "enabled_observed": any(sample.get("enabled") is True for sample in samples),
        "mode": mode,
        "commands_enqueued_total": max_counter(samples, "commands_enqueued_total"),
        "commands_sent_total": max_counter(samples, "commands_sent_total"),
        "commands_acked_total": max_counter(samples, "commands_acked_total"),
        "commands_socket_sent_total": max_counter(samples, "commands_socket_sent_total"),
        "commands_dropped_total": max_counter(samples, "commands_dropped_total"),
        "commands_overwritten_total": max_counter(samples, "commands_overwritten_total"),
        "goal_window_commands_sent": max_counter(samples, "goal_window_commands_sent"),
        "goal_window_commands_acked": max_counter(samples, "goal_window_commands_acked"),
        "first_goal_command_send_ns": timestamp_extent(samples, "first_goal_command_send_ns", first=True),
        "last_goal_command_send_ns": timestamp_extent(samples, "last_goal_command_send_ns", first=False),
        "first_worker_send_ns": timestamp_extent(samples, "first_worker_send_ns", first=True),
        "last_worker_send_ns": timestamp_extent(samples, "last_worker_send_ns", first=False),
        "command_phase": phase,
        "ack_timeout_count": max_counter(samples, "ack_timeout_count"),
        "missing_ack_count": max_counter(samples, "missing_ack_count"),
        "reference_supervision_fault_count": max_counter(samples, "reference_supervision_fault_count"),
    }


def write_async_ack_telemetry(path: Path, states: list[dict[str, Any]], arm: str) -> int:
    rows: list[dict[str, Any]] = []
    for snapshot in states:
        sample = async_state(snapshot, arm)
        if sample is None:
            continue
        rows.append(
            {
                "host_time_ns": snapshot.get("host_time_ns"),
                "arm": arm,
                "async_streaming": sample,
            }
        )
    if not rows:
        return 0
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, sort_keys=True) + "\n")
    return len(rows)


def command_mode(packet: dict[str, Any], arm: str) -> str:
    arm_packet = packet.get(arm)
    if isinstance(arm_packet, dict):
        value = arm_packet.get("mode")
        if isinstance(value, str):
            return value
    value = packet.get("mode")
    return value if isinstance(value, str) else ""


def command_packet_rate(command_packets: list[dict[str, Any]], arm: str) -> dict[str, Any]:
    times = [
        int(packet["host_time_ns"])
        for packet in command_packets
        if isinstance(packet.get("host_time_ns"), int)
    ]
    tracking_times = [
        int(packet["host_time_ns"])
        for packet in command_packets
        if isinstance(packet.get("host_time_ns"), int)
        and command_mode(packet, arm).startswith("Tcp")
    ]
    if len(times) < 2:
        return {
            "udp_command_count": len(times),
            "udp_tracking_command_count": len(tracking_times),
            "udp_effective_command_rate_hz": None,
        }
    elapsed_sec = (times[-1] - times[0]) / 1e9
    rate = (len(times) - 1) / elapsed_sec if elapsed_sec > 0 else None
    return {
        "udp_command_count": len(times),
        "udp_tracking_command_count": len(tracking_times),
        "udp_command_start_ns": times[0],
        "udp_command_end_ns": times[-1],
        "udp_effective_command_rate_hz": rate,
    }


def positive_int(value: Any) -> int | None:
    number = finite_number(value)
    if number is None or number <= 0.0:
        return None
    return int(round(number))


def official_tracking_window(summary: dict[str, Any], duration_sec: float | None) -> tuple[int | None, int | None, float | None]:
    window_sec = first_number(summary.get("official_tracking_window_sec"), duration_sec)
    start_ns = positive_int(first_value(summary.get("official_tracking_start_ns"), summary.get("benchmark_start_ns")))
    end_ns = positive_int(summary.get("official_tracking_end_ns"))
    if end_ns is None and start_ns is not None and window_sec is not None:
        end_ns = start_ns + int(round(window_sec * 1e9))
    if window_sec is None and start_ns is not None and end_ns is not None and end_ns > start_ns:
        window_sec = (end_ns - start_ns) / 1e9
    return start_ns, end_ns, window_sec


def snapshot_time_ns(snapshot: dict[str, Any]) -> int | None:
    for key in ("loop_start_time_ns", "host_time_ns", "loop_end_time_ns"):
        value = positive_int(snapshot.get(key))
        if value is not None:
            return value
    return None


def server_servo_tick_count(states: list[dict[str, Any]], start_ns: int | None, end_ns: int | None) -> int | None:
    if start_ns is None or end_ns is None or end_ns <= start_ns:
        return None
    ticks: list[int] = []
    for snapshot in states:
        time_ns = snapshot_time_ns(snapshot)
        if time_ns is None or time_ns < start_ns or time_ns >= end_ns:
            continue
        tick = positive_int(snapshot.get("tick"))
        if tick is not None:
            ticks.append(tick)
    if len(ticks) >= 2:
        return max(ticks) - min(ticks) + 1
    return None


def first_counter(summary: dict[str, Any], row: dict[str, str], async_info: dict[str, Any], *keys: str) -> int | None:
    for key in keys:
        value = first_number(summary.get(key), row.get(key), async_info.get(key))
        if value is not None and value >= 0.0:
            return int(value)
    return None


def rate_accounting_fields(
    summary: dict[str, Any],
    row: dict[str, str],
    async_info: dict[str, Any],
    states: list[dict[str, Any]],
    *,
    duration_sec: float | None,
    servo_rate_hz: float | None,
    commands_sent: int,
    commands_acked: int,
) -> dict[str, Any]:
    start_ns, end_ns, window_sec = official_tracking_window(summary, duration_sec)
    tick_count = first_counter(summary, row, async_info, "server_servo_tick_count")
    if tick_count is None:
        tick_count = server_servo_tick_count(states, start_ns, end_ns)
    expected_ticks = (
        int(round(servo_rate_hz * window_sec))
        if servo_rate_hz is not None and window_sec is not None and window_sec > 0.0
        else first_counter(summary, row, async_info, "expected_servo_ticks")
    )
    async_enqueued = first_counter(summary, row, async_info, "async_commands_enqueued_total", "commands_enqueued_total")
    async_sent = first_counter(summary, row, async_info, "async_commands_sent_total", "commands_sent_total")
    async_acked = first_counter(summary, row, async_info, "async_commands_acked_total", "commands_acked_total")
    if async_sent is None:
        async_sent = commands_sent if commands_sent > 0 else None
    if async_acked is None:
        async_acked = commands_acked if commands_acked > 0 else None
    goal_sent = first_counter(summary, row, async_info, "goal_window_commands_sent")
    goal_acked = first_counter(summary, row, async_info, "goal_window_commands_acked")
    if goal_sent is not None and goal_sent <= 0:
        goal_sent = None
    if goal_acked is not None and goal_acked < 0:
        goal_acked = None
    goal_count_source = first_value(summary.get("goal_window_count_source"), row.get("goal_window_count_source"))
    if goal_sent is None and tick_count is not None:
        dropped = first_counter(summary, row, async_info, "commands_dropped_total") or 0
        overwritten = first_counter(summary, row, async_info, "commands_overwritten_total") or 0
        if dropped == 0 and overwritten == 0:
            goal_sent = tick_count
            if async_acked is not None and async_acked >= goal_sent:
                goal_acked = goal_sent
            goal_count_source = goal_count_source or "server_servo_tick_count_inferred"
    effective_goal_rate = (
        goal_sent / window_sec
        if goal_sent is not None and goal_sent > 0 and window_sec is not None and window_sec > 0.0
        else None
    )
    ack_coverage = (
        goal_acked / goal_sent
        if goal_sent is not None and goal_sent > 0 and goal_acked is not None
        else None
    )
    first_worker_ns = first_counter(summary, row, async_info, "first_worker_send_ns")
    last_worker_ns = first_counter(summary, row, async_info, "last_worker_send_ns")
    worker_window_sec = (
        (last_worker_ns - first_worker_ns) / 1e9
        if first_worker_ns is not None and last_worker_ns is not None and last_worker_ns > first_worker_ns
        else finite_number(summary.get("measured_worker_window_sec"))
    )
    worker_send_rate = (
        async_sent / worker_window_sec
        if async_sent is not None and async_sent > 0 and worker_window_sec is not None and worker_window_sec > 0.0
        else None
    )
    outside = async_sent - goal_sent if async_sent is not None and goal_sent is not None else None
    return {
        "official_tracking_start_ns": start_ns,
        "official_tracking_end_ns": end_ns,
        "official_tracking_window_sec": window_sec,
        "official_servo_rate_hz": servo_rate_hz,
        "expected_servo_ticks": expected_ticks,
        "server_servo_tick_count": tick_count,
        "async_commands_enqueued_total": async_enqueued,
        "async_commands_sent_total": async_sent,
        "async_commands_acked_total": async_acked,
        "goal_window_commands_sent": goal_sent,
        "goal_window_commands_acked": goal_acked,
        "goal_window_count_source": goal_count_source,
        "ack_coverage_ratio": ack_coverage,
        "effective_goal_command_rate_hz": effective_goal_rate,
        "first_worker_send_ns": first_worker_ns,
        "last_worker_send_ns": last_worker_ns,
        "measured_worker_window_sec": worker_window_sec,
        "worker_active_window_sec": worker_window_sec,
        "worker_send_rate_hz": worker_send_rate,
        "worker_lifetime_send_rate_hz": worker_send_rate,
        "worker_sends_outside_official_window": outside,
        "worker_sends_outside_official_window_detected": outside not in (None, 0),
    }


def semantics_count(summary: dict[str, Any], name: str) -> int:
    value = summary.get("send_acceptance_semantics_distribution")
    if isinstance(value, dict):
        number = finite_number(value.get(name))
        return int(number) if number is not None and number >= 0 else 0
    return 0


def derive_acceptance_semantics(summary: dict[str, Any], row: dict[str, str], async_info: dict[str, Any]) -> str | None:
    explicit = first_value(summary.get("acceptance_semantics"), summary.get("ack_semantics"), row.get("acceptance_semantics"))
    async_mode = first_value(summary.get("async_mode"), summary.get("async_streaming_mode"), row.get("async_mode"), async_info.get("mode"))
    if async_mode == "sdk_ack_worker" and int(async_info.get("commands_acked_total") or 0) > 0:
        return "sdk_worker_ack_observed"
    if int(async_info.get("commands_socket_sent_total") or 0) > 0:
        return "socket_send_only"
    if explicit:
        if async_mode == "sdk_ack_worker" and explicit == "controller_ack_observed":
            return "sdk_worker_ack_observed"
        return str(explicit)
    if semantics_count(summary, "controller_ack_observed") > 0:
        return "controller_ack_observed"
    if semantics_count(summary, "socket_send_only") > 0:
        return "socket_send_only"
    return None


def server_side_circle_packet_observed(commands: list[dict[str, Any]], arm: str) -> bool:
    for packet in commands:
        arm_payload = packet.get(arm)
        if not isinstance(arm_payload, dict):
            continue
        if arm_payload.get("mode") in sim_bench.SERVER_SIDE_CIRCLE_MODES:
            return True
    return False


def artifact_exists(path_text: Any, fallback: Path) -> bool:
    path = Path(str(path_text)) if isinstance(path_text, str) and path_text else fallback
    return path.is_file()


def candidate_from_summary(path: Path, ablation_rows: dict[str, dict[str, str]]) -> dict[str, Any]:
    summary = load_json(path)
    artifact_dir = path.parent.resolve()
    preflight = nested_dict(summary, "safety_preflight")
    operation_modes = nested_dict(preflight, "operation_modes")
    row = ablation_rows.get(str(artifact_dir), {})
    if not row and summary.get("artifact_dir"):
        row = ablation_rows.get(str(Path(str(summary["artifact_dir"])).resolve()), {})
    if not row:
        row = ablation_rows.get(str(row.get("name") or summary.get("name") or ""), {})
    arm = str(first_value(summary.get("arm"), row.get("arm"), "left"))
    states = load_jsonl(artifact_dir / "state_stream.jsonl")
    commands = load_jsonl(artifact_dir / "command_packets.jsonl")
    async_info = async_metrics(states, arm)
    packet_rate = command_packet_rate(commands, arm)
    server_side_circle_observed = server_side_circle_packet_observed(commands, arm)
    async_telemetry_path = artifact_dir / "async_ack_telemetry.jsonl"
    async_telemetry_rows = 0
    if async_info.get("enabled_observed") or async_info.get("mode") == "sdk_ack_worker":
        async_telemetry_rows = write_async_ack_telemetry(async_telemetry_path, states, arm)
    duration_sec = first_number(summary.get("duration_sec"))
    if duration_sec is None:
        period = finite_number(summary.get("period_sec"))
        repeat = finite_number(summary.get("repeat"))
        duration_sec = period * repeat if period is not None and repeat is not None else None
    commands_sent = int(async_info.get("commands_sent_total") or 0)
    commands_acked = int(async_info.get("commands_acked_total") or 0)
    socket_sent = int(async_info.get("commands_socket_sent_total") or 0)
    if commands_sent <= 0:
        commands_sent = int(first_number(summary.get("commands_sent_total"), row.get("commands_sent_total")) or 0)
    if commands_acked <= 0:
        commands_acked = int(first_number(summary.get("commands_acked_total"), row.get("commands_acked_total")) or 0)
    if socket_sent <= 0:
        socket_sent = int(first_number(summary.get("socket_send_only_count"), row.get("socket_send_only_count")) or 0)
    if commands_sent <= 0 and derive_acceptance_semantics(summary, row, async_info) == "controller_ack_observed":
        commands_sent = int(first_number(summary.get("command_count"), row.get("command_count")) or 0)
        commands_acked = int(first_number(summary.get("controller_ack_observed_count"), summary.get("controller_acceptance_observed_count"), row.get("controller_ack_observed_count")) or 0)
    ack_ratio = commands_acked / commands_sent if commands_sent > 0 else None
    servo_rate_hz = first_number(summary.get("servo_rate_hz"), row.get("servo_rate_hz"))
    rate_accounting = rate_accounting_fields(
        summary,
        row,
        async_info,
        states,
        duration_sec=duration_sec,
        servo_rate_hz=servo_rate_hz,
        commands_sent=commands_sent,
        commands_acked=commands_acked,
    )
    state_age_p95 = first_number(
        nested_metric(summary, "state_age_us", "p95"),
        nested_metric(summary.get("timestamp_alignment", {}) if isinstance(summary.get("timestamp_alignment"), dict) else {}, "state_age_us", "p95"),
    )
    estimated_latency_ms = finite_number(summary.get("estimated_latency_ms"))
    phase_advance_ms = first_number(summary.get("commanded_phase_advance_ms"), row.get("commanded_phase_advance_ms"), 0.0)
    run_result = infer_run_result(summary)
    safety_result = infer_safety_result(summary, run_result)
    threshold_result = infer_benchmark_threshold_result(summary)
    diagnostics = diagnostic_warnings_from_summary(summary, threshold_result)
    candidate = {
        "name": first_value(row.get("name"), summary.get("name"), artifact_dir.name),
        "artifact_dir": str(artifact_dir),
        "summary_json": str(path.resolve()),
        "profile": first_value(summary.get("profile"), row.get("profile")),
        "arm": arm,
        "benchmark_category": "rbpodo_controller_simulation",
        "backend": "rbpodo",
        "controller_mode": "pgmode_simulation",
        "controller": first_value(
            summary.get("controller"),
            row.get("controller"),
            "server_circle" if server_side_circle_observed else None,
        ),
        "command_family": first_value(
            summary.get("command_family"),
            row.get("command_family"),
            sim_bench.SERVER_SIDE_CIRCLE_COMMAND_FAMILY if server_side_circle_observed else None,
        ),
        "repeat": first_number(summary.get("repeat"), row.get("repeat")),
        "tracking_source": first_value(summary.get("tracking_source_used"), summary.get("tracking_source"), row.get("tracking_source")),
        "servo_rate_hz": servo_rate_hz,
        "servo_t1_sec": first_number(summary.get("servo_t1_sec"), row.get("servo_t1_sec")),
        "servo_t2_sec": first_number(summary.get("servo_t2_sec"), row.get("servo_t2_sec")),
        "servo_alpha": first_number(summary.get("servo_alpha"), row.get("servo_alpha")),
        "speed_bar": first_number(summary.get("speed_bar"), row.get("speed_bar")),
        "path_kp_pos": first_number(summary.get("path_kp_pos"), row.get("path_kp_pos"), row.get("feedback_kp_pos")),
        "path_kp_ori": first_number(summary.get("path_kp_ori"), row.get("path_kp_ori"), row.get("feedback_kp_ori")),
        "command_rate_hz": first_number(summary.get("command_rate_hz"), row.get("command_rate_hz")),
        "phase_advance_sec": first_number(
            summary.get("phase_advance_sec"),
            row.get("phase_advance_sec"),
            scaled_number(summary.get("commanded_phase_advance_ms"), 0.001),
            scaled_number(row.get("commanded_phase_advance_ms"), 0.001),
        ),
        "async_mode": first_value(summary.get("async_mode"), summary.get("async_streaming_mode"), row.get("async_mode"), async_info.get("mode")),
        "acceptance_semantics": derive_acceptance_semantics(summary, row, async_info),
        "commands_sent_total": commands_sent,
        "commands_acked_total": commands_acked,
        "socket_send_only_count": socket_sent,
        "ack_observed_ratio": ack_ratio,
        **rate_accounting,
        **packet_rate,
        "async_streaming_metrics": async_info,
        "rms_error_m": first_number(summary.get("rms_error_m"), scaled_number(row.get("rms_error_mm"), 0.001)),
        "p95_error_m": first_number(summary.get("p95_error_m"), scaled_number(row.get("p95_error_mm"), 0.001)),
        "fit_center_error_m": first_number(summary.get("fit_center_error_m"), scaled_number(row.get("fit_center_error_mm"), 0.001)),
        "radius_gain": first_number(summary.get("radius_gain"), row.get("radius_gain")),
        "p95_orientation_drift_rad": first_number(summary.get("p95_orientation_drift_rad"), scaled_number(row.get("p95_orientation_drift_mrad"), 0.001)),
        "max_orientation_drift_rad": first_number(summary.get("max_orientation_drift_rad"), summary.get("orientation_max_drift_rad")),
        "estimated_latency_ms": estimated_latency_ms,
        "commanded_phase_advance_ms": phase_advance_ms,
        "uncompensated_latency_estimate_ms": estimated_latency_ms + phase_advance_ms if estimated_latency_ms is not None and phase_advance_ms is not None else None,
        "effective_phase_latency_abs_ms": abs(estimated_latency_ms) if estimated_latency_ms is not None else None,
        "state_age_p95_us": state_age_p95,
        "feedback_saturation_count": first_number(summary.get("feedback_saturation_count"), row.get("feedback_saturation_count")),
        "command_count": first_number(summary.get("command_count"), row.get("command_count")),
        "fault_latched": as_bool(first_value(summary.get("fault_latched"), row.get("fault_latched"))),
        "physical_motion_detected": as_bool(first_value(summary.get("physical_motion_detected"), row.get("physical_motion_detected"))),
        "physical_motion_expected": as_bool(first_value(summary.get("physical_motion_expected"), row.get("physical_motion_expected"), False)),
        "disable_waiting_ack": as_bool(
            first_value(summary.get("disable_waiting_ack"), row.get("disable_waiting_ack"), preflight.get("disable_waiting_ack"))
        ),
        "left_disable_waiting_ack": as_bool(
            first_value(
                summary.get("left_disable_waiting_ack"),
                row.get("left_disable_waiting_ack"),
                preflight.get("left_disable_waiting_ack"),
                summary.get("disable_waiting_ack"),
                row.get("disable_waiting_ack"),
                preflight.get("disable_waiting_ack"),
            )
        ),
        "right_disable_waiting_ack": as_bool(
            first_value(
                summary.get("right_disable_waiting_ack"),
                row.get("right_disable_waiting_ack"),
                preflight.get("right_disable_waiting_ack"),
                summary.get("disable_waiting_ack"),
                row.get("disable_waiting_ack"),
                preflight.get("disable_waiting_ack"),
            )
        ),
        "left_operation_mode": first_value(
            summary.get("left_operation_mode"),
            row.get("left_operation_mode"),
            preflight.get("left_operation_mode"),
            operation_modes.get("left"),
        ),
        "right_operation_mode": first_value(
            summary.get("right_operation_mode"),
            row.get("right_operation_mode"),
            preflight.get("right_operation_mode"),
            operation_modes.get("right"),
        ),
        "cartesian_allow_in_controller_simulation": as_bool(
            first_value(
                summary.get("cartesian_allow_in_controller_simulation"),
                row.get("cartesian_allow_in_controller_simulation"),
                preflight.get("cartesian_allow_in_controller_simulation"),
            )
        ),
        "cartesian_allow_in_real": as_bool(
            first_value(
                summary.get("cartesian_allow_in_real"),
                row.get("cartesian_allow_in_real"),
                preflight.get("cartesian_allow_in_real"),
            )
        ),
        "cartesian_unavailable_count": first_number(summary.get("cartesian_unavailable_count"), row.get("cartesian_unavailable_count")),
        "measurement_reliability_level": first_value(summary.get("measurement_reliability_level"), row.get("measurement_reliability_level")),
        "timing_classification": first_value(summary.get("timing_classification"), row.get("timing_classification")),
        "run_result": run_result,
        "run_result_status": run_result.get("status"),
        "safety_result": safety_result,
        "safety_result_status": safety_result.get("status"),
        "benchmark_threshold_result": threshold_result,
        "benchmark_threshold_status": threshold_result.get("status"),
        "diagnostic_warnings": diagnostics,
        "diagnostic_warning_count": len(diagnostics),
        "legacy_result": summary.get("result"),
        "result": run_result.get("status"),
        "result_reason": summary.get("result_reason"),
        "threshold_failures": threshold_result.get("threshold_failures"),
        "threshold_warnings": threshold_result.get("threshold_warnings"),
        "state_stream": str((artifact_dir / "state_stream.jsonl").resolve()),
        "command_packets": str((artifact_dir / "command_packets.jsonl").resolve()),
        "async_ack_telemetry": str(async_telemetry_path.resolve()) if async_telemetry_rows else None,
        "async_ack_telemetry_rows": async_telemetry_rows,
        "alignment_report": str((artifact_dir / "alignment_report.md").resolve()) if (artifact_dir / "alignment_report.md").is_file() else None,
        "error_decomposition_json": summary.get("error_decomposition_json") or str((artifact_dir / "error_decomposition.json").resolve()),
    }
    command_count = finite_number(candidate.get("command_count"))
    saturation_denominator = max(
        value
        for value in (
            command_count or 0.0,
            finite_number(candidate.get("commands_sent_total")) or 0.0,
        )
    )
    saturation_count = finite_number(candidate.get("feedback_saturation_count"))
    candidate["feedback_saturation_ratio"] = (
        saturation_count / saturation_denominator
        if saturation_count is not None and saturation_denominator > 0
        else None
    )
    sim_bench.apply_canonical_lane_metadata(candidate)
    candidate["required_artifacts_present"] = required_artifacts_present(candidate)
    candidate["failures"] = goal_failures(candidate)
    candidate["ackon500_goal_result"] = {
        "evaluated": True,
        "status": "fail" if candidate["failures"] else "pass",
        "failures": list(candidate["failures"]),
        "warnings": list(candidate.get("diagnostic_warnings") or []),
    }
    candidate["ackon500_goal_status"] = candidate["ackon500_goal_result"]["status"]
    candidate["goal_pass"] = candidate["ackon500_goal_status"] == "pass"
    candidate["pass"] = candidate["goal_pass"]
    candidate["physical_readiness"] = physical_readiness()
    candidate["controller_reference_result"] = controller_reference_result(
        candidate["goal_pass"] and candidate.get("tracking_source") == "tcp_ref_stand"
    )
    candidate["physical_tracking_result"] = physical_tracking_result()
    candidate["physical_readiness_status"] = candidate["physical_readiness"]["status"]
    candidate["controller_reference_status"] = candidate["controller_reference_result"]["status"]
    candidate["physical_tracking_status"] = candidate["physical_tracking_result"]["status"]
    return candidate


def near(actual: Any, expected: float, tolerance: float = 1e-9) -> bool:
    number = finite_number(actual)
    return number is not None and abs(number - expected) <= tolerance


def required_artifacts_present(candidate: dict[str, Any]) -> dict[str, bool]:
    artifact_dir = Path(str(candidate["artifact_dir"]))
    required = {
        "state_stream.jsonl": artifact_exists(candidate.get("state_stream"), artifact_dir / "state_stream.jsonl"),
        "command_packets.jsonl": artifact_exists(candidate.get("command_packets"), artifact_dir / "command_packets.jsonl"),
        "error_decomposition.json": artifact_exists(candidate.get("error_decomposition_json"), artifact_dir / "error_decomposition.json"),
    }
    if candidate.get("async_mode") == "sdk_ack_worker":
        required["async_ack_telemetry.jsonl"] = artifact_exists(candidate.get("async_ack_telemetry"), artifact_dir / "async_ack_telemetry.jsonl")
    return required


def check_min(candidate: dict[str, Any], key: str, minimum: float, failures: list[str]) -> None:
    value = finite_number(candidate.get(key))
    if value is None:
        failures.append(f"{key} unavailable")
    elif value < minimum:
        failures.append(f"{key} {value:.9g} < {minimum:.9g}")


def check_max(candidate: dict[str, Any], key: str, maximum: float, failures: list[str]) -> None:
    value = finite_number(candidate.get(key))
    if value is None:
        failures.append(f"{key} unavailable")
    elif value > maximum:
        failures.append(f"{key} {value:.9g} > {maximum:.9g}")


def goal_failures(candidate: dict[str, Any]) -> list[str]:
    failures: list[str] = []
    if candidate.get("profile") != "gene_15cm_4s":
        failures.append(f"profile {candidate.get('profile')} != gene_15cm_4s")
    if candidate.get("controller") != "server_circle":
        failures.append(f"controller {candidate.get('controller')} != server_circle")
    if candidate.get("run_result_status") != "completed":
        failures.append(f"run_result_status {candidate.get('run_result_status')} != completed")
    if candidate.get("safety_result_status") != "pass":
        failures.append(f"safety_result_status {candidate.get('safety_result_status')} != pass")
    check_min(candidate, "repeat", PASS_THRESHOLDS["min_repeat"], failures)
    if candidate.get("tracking_source") != "tcp_ref_stand":
        failures.append(f"tracking_source {candidate.get('tracking_source')} != tcp_ref_stand")
    if not near(candidate.get("servo_rate_hz"), PASS_THRESHOLDS["servo_rate_hz"]):
        failures.append(f"servo_rate_hz {candidate.get('servo_rate_hz')} != 500")
    if not near(candidate.get("servo_t1_sec"), PASS_THRESHOLDS["servo_t1_sec"]):
        failures.append(f"servo_t1_sec {candidate.get('servo_t1_sec')} != 0.002")
    if not near(candidate.get("command_rate_hz"), PASS_THRESHOLDS["command_rate_hz"]):
        failures.append(f"command_rate_hz {candidate.get('command_rate_hz')} != 500")
    if not near(candidate.get("phase_advance_sec"), PASS_THRESHOLDS["phase_advance_sec"]):
        failures.append(f"phase_advance_sec {candidate.get('phase_advance_sec')} != 0.005")
    if candidate.get("async_mode") != "sdk_ack_worker":
        failures.append(f"async_mode {candidate.get('async_mode')} != sdk_ack_worker")
    if candidate.get("acceptance_semantics") not in {"controller_ack_observed", "sdk_worker_ack_observed"}:
        failures.append(f"acceptance_semantics {candidate.get('acceptance_semantics')} is not ACK-observed")
    if candidate.get("benchmark_lane") != "rbpodo_server_side_circle_ackon500_sdk_worker":
        failures.append(
            "benchmark_lane "
            f"{candidate.get('benchmark_lane')} != rbpodo_server_side_circle_ackon500_sdk_worker"
        )
    check_min(candidate, "official_tracking_window_sec", 1e-9, failures)
    check_min(candidate, "goal_window_commands_sent", 1.0, failures)
    check_min(candidate, "ack_coverage_ratio", PASS_THRESHOLDS["min_ack_ratio"], failures)
    if int(candidate.get("socket_send_only_count") or 0) != 0:
        failures.append(f"socket_send_only_count {candidate.get('socket_send_only_count')} != 0")
    for key in ("left_disable_waiting_ack", "right_disable_waiting_ack"):
        if candidate.get(key) is not False:
            failures.append(f"{key} {candidate.get(key)} is not false")
    for key in ("left_operation_mode", "right_operation_mode"):
        if candidate.get(key) not in {"simulation", "sim"}:
            failures.append(f"{key} {candidate.get(key)} is not simulation")
    if candidate.get("cartesian_allow_in_controller_simulation") is not True:
        failures.append(
            "cartesian_allow_in_controller_simulation "
            f"{candidate.get('cartesian_allow_in_controller_simulation')} is not true"
        )
    if candidate.get("cartesian_allow_in_real") is not False:
        failures.append(f"cartesian_allow_in_real {candidate.get('cartesian_allow_in_real')} is not false")
    check_min(
        candidate,
        "effective_goal_command_rate_hz",
        PASS_THRESHOLDS["min_effective_goal_command_rate_hz"],
        failures,
    )
    check_max(
        candidate,
        "effective_goal_command_rate_hz",
        PASS_THRESHOLDS["max_effective_goal_command_rate_hz"],
        failures,
    )
    if candidate.get("fault_latched") is not False:
        failures.append(f"fault_latched {candidate.get('fault_latched')} is not false")
    if candidate.get("physical_motion_detected") is not False:
        failures.append(f"physical_motion_detected {candidate.get('physical_motion_detected')} is not false")
    if candidate.get("physical_motion_expected") is not False:
        failures.append(f"physical_motion_expected {candidate.get('physical_motion_expected')} is not false")
    if int(candidate.get("cartesian_unavailable_count") or 0) != 0:
        failures.append(f"cartesian_unavailable_count {candidate.get('cartesian_unavailable_count')} != 0")
    check_max(candidate, "feedback_saturation_ratio", PASS_THRESHOLDS["max_saturation_ratio"], failures)
    check_max(candidate, "rms_error_m", PASS_THRESHOLDS["max_rms_error_m"], failures)
    check_max(candidate, "p95_error_m", PASS_THRESHOLDS["max_p95_error_m"], failures)
    check_max(candidate, "fit_center_error_m", PASS_THRESHOLDS["max_fit_center_error_m"], failures)
    check_min(candidate, "radius_gain", PASS_THRESHOLDS["min_radius_gain"], failures)
    check_max(candidate, "radius_gain", PASS_THRESHOLDS["max_radius_gain"], failures)
    check_max(candidate, "p95_orientation_drift_rad", PASS_THRESHOLDS["max_p95_orientation_drift_rad"], failures)
    check_max(candidate, "effective_phase_latency_abs_ms", PASS_THRESHOLDS["max_effective_phase_latency_abs_ms"], failures)
    check_max(candidate, "state_age_p95_us", PASS_THRESHOLDS["max_state_age_p95_us"], failures)
    if candidate.get("measurement_reliability_level") == "unreliable":
        failures.append("measurement_reliability_level is unreliable")
    missing = [name for name, present in candidate.get("required_artifacts_present", {}).items() if not present]
    if missing:
        failures.append("missing required artifacts: " + ", ".join(missing))
    return failures


def score(candidate: dict[str, Any]) -> tuple[int, float, float, float]:
    passed = 0 if candidate.get("pass") else 1
    rms = finite_number(candidate.get("rms_error_m")) or float("inf")
    p95 = finite_number(candidate.get("p95_error_m")) or float("inf")
    failures = len(candidate.get("failures") or [])
    return (passed, failures, rms, p95)


def format_cell(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        if math.isnan(value) or math.isinf(value):
            return ""
        return f"{value:.6g}"
    if isinstance(value, list):
        return "; ".join(str(item) for item in value)
    return str(value)


def table(rows: list[dict[str, Any]], columns: list[str]) -> str:
    if not rows:
        return "_None._"
    out = [
        "| " + " | ".join(columns) + " |",
        "| " + " | ".join("---" for _ in columns) + " |",
    ]
    for row in rows:
        out.append("| " + " | ".join(format_cell(row.get(column)) for column in columns) + " |")
    return "\n".join(out)


def mean_std_min_max(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {"mean": None, "std": None, "min": None, "max": None}
    mean = sum(values) / len(values)
    variance = sum((value - mean) ** 2 for value in values) / len(values)
    return {
        "mean": mean,
        "std": math.sqrt(variance),
        "min": min(values),
        "max": max(values),
    }


def median(values: list[float]) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return (ordered[mid - 1] + ordered[mid]) / 2.0


def mm_value(value: Any) -> float | None:
    number = finite_number(value)
    return number * 1000.0 if number is not None else None


def required_names_for_arm(arm: str) -> list[str]:
    prefix = f"best_{arm}_"
    return [name for name in REPEATABILITY_REQUIRED_RUN_NAMES if name.startswith(prefix)]


def select_repeatability_required_candidates(candidates: list[dict[str, Any]]) -> tuple[list[dict[str, Any]], list[str]]:
    by_name = {
        str(candidate.get("name")): candidate
        for candidate in candidates
        if candidate.get("name") not in (None, "")
    }
    missing = [name for name in REPEATABILITY_REQUIRED_RUN_NAMES if name not in by_name]
    return [by_name[name] for name in REPEATABILITY_REQUIRED_RUN_NAMES if name in by_name], missing


def repeatability_row(candidate: dict[str, Any]) -> dict[str, Any]:
    ack_ratio = first_number(candidate.get("ack_coverage_ratio"), candidate.get("ack_observed_ratio"))
    rms_mm = mm_value(candidate.get("rms_error_m"))
    p95_mm = mm_value(candidate.get("p95_error_m"))
    latency_ms = first_number(candidate.get("effective_phase_latency_abs_ms"))
    failures = list(candidate.get("failures") or [])
    hard_failures: list[str] = []
    if candidate.get("fault_latched") is not False:
        hard_failures.append(f"fault_latched {candidate.get('fault_latched')} is not false")
    if candidate.get("physical_motion_detected") is not False:
        hard_failures.append(
            f"physical_motion_detected {candidate.get('physical_motion_detected')} is not false"
        )
    if candidate.get("physical_motion_expected") is not False:
        hard_failures.append(
            f"physical_motion_expected {candidate.get('physical_motion_expected')} is not false"
        )
    if int(candidate.get("socket_send_only_count") or 0) != 0:
        hard_failures.append(f"socket_send_only_count {candidate.get('socket_send_only_count')} != 0")
    if ack_ratio is None:
        hard_failures.append("ack_observed_ratio unavailable")
    elif ack_ratio < REPEATABILITY_THRESHOLDS["min_ack_ratio"]:
        hard_failures.append(
            f"ack_observed_ratio {ack_ratio:.9g} < {REPEATABILITY_THRESHOLDS['min_ack_ratio']:.9g}"
        )
    for failure in hard_failures:
        if failure not in failures:
            failures.append(failure)
    return {
        "name": candidate.get("name"),
        "arm": candidate.get("arm"),
        "profile": candidate.get("profile"),
        "controller": candidate.get("controller"),
        "goal_pass": candidate.get("goal_pass") is True,
        "ackon500_goal_status": candidate.get("ackon500_goal_status"),
        "run_result_status": candidate.get("run_result_status"),
        "safety_result_status": candidate.get("safety_result_status"),
        "benchmark_lane": candidate.get("benchmark_lane"),
        "low_level_send_mode": candidate.get("low_level_send_mode"),
        "async_mode": candidate.get("async_mode"),
        "acceptance_semantics": candidate.get("acceptance_semantics"),
        "tracking_source": candidate.get("tracking_source"),
        "repeat": candidate.get("repeat"),
        "command_rate_hz": candidate.get("command_rate_hz"),
        "phase_advance_sec": candidate.get("phase_advance_sec"),
        "rms_error_mm": rms_mm,
        "p95_error_mm": p95_mm,
        "latency_ms": latency_ms,
        "ack_observed_ratio": ack_ratio,
        "state_age_p95_us": candidate.get("state_age_p95_us"),
        "socket_send_only_count": candidate.get("socket_send_only_count"),
        "left_disable_waiting_ack": candidate.get("left_disable_waiting_ack"),
        "right_disable_waiting_ack": candidate.get("right_disable_waiting_ack"),
        "left_operation_mode": candidate.get("left_operation_mode"),
        "right_operation_mode": candidate.get("right_operation_mode"),
        "cartesian_allow_in_controller_simulation": candidate.get("cartesian_allow_in_controller_simulation"),
        "cartesian_allow_in_real": candidate.get("cartesian_allow_in_real"),
        "fault_latched": candidate.get("fault_latched"),
        "physical_motion_detected": candidate.get("physical_motion_detected"),
        "physical_motion_expected": candidate.get("physical_motion_expected"),
        "measurement_reliability_level": candidate.get("measurement_reliability_level"),
        "diagnostic_warnings": candidate.get("diagnostic_warnings"),
        "failures": failures,
        "hard_failures": hard_failures,
        "artifact_dir": candidate.get("artifact_dir"),
    }


def aggregate_repeatability(rows: list[dict[str, Any]]) -> dict[str, Any]:
    rms_values = [value for row in rows for value in [finite_number(row.get("rms_error_mm"))] if value is not None]
    p95_values = [value for row in rows for value in [finite_number(row.get("p95_error_mm"))] if value is not None]
    latency_values = [value for row in rows for value in [finite_number(row.get("latency_ms"))] if value is not None]
    ack_values = [
        value for row in rows for value in [finite_number(row.get("ack_observed_ratio"))] if value is not None
    ]
    state_age_values = [
        value for row in rows for value in [finite_number(row.get("state_age_p95_us"))] if value is not None
    ]
    rms_stats = mean_std_min_max(rms_values)
    p95_stats = mean_std_min_max(p95_values)
    latency_stats = mean_std_min_max(latency_values)
    return {
        "units": {
            "rms": "mm",
            "p95": "mm",
            "latency": "ms",
            "state_age_p95": "us",
        },
        "rms_mean": rms_stats["mean"],
        "rms_std": rms_stats["std"],
        "rms_min": rms_stats["min"],
        "rms_max": rms_stats["max"],
        "rms_median": median(rms_values),
        "p95_mean": p95_stats["mean"],
        "p95_std": p95_stats["std"],
        "p95_min": p95_stats["min"],
        "p95_max": p95_stats["max"],
        "p95_median": median(p95_values),
        "latency_mean": latency_stats["mean"],
        "latency_std": latency_stats["std"],
        "latency_min": latency_stats["min"],
        "latency_max": latency_stats["max"],
        "ack_observed_ratio_min": min(ack_values) if ack_values else None,
        "state_age_p95_max": max(state_age_values) if state_age_values else None,
    }


def aggregate_threshold_failures(aggregate: dict[str, Any]) -> list[str]:
    failures: list[str] = []
    checks = [
        ("rms_median", "max_median_rms_error_mm"),
        ("rms_max", "max_worst_rms_error_mm"),
        ("p95_median", "max_median_p95_error_mm"),
        ("p95_max", "max_worst_p95_error_mm"),
    ]
    for metric, threshold_key in checks:
        value = finite_number(aggregate.get(metric))
        threshold = float(REPEATABILITY_THRESHOLDS[threshold_key])
        if value is None:
            failures.append(f"{metric} unavailable")
        elif value > threshold:
            failures.append(f"{metric} {value:.9g} > {threshold:.9g}")
    return failures


def metric_missing_rows(rows: list[dict[str, Any]]) -> list[str]:
    return [
        str(row.get("name"))
        for row in rows
        if finite_number(row.get("rms_error_mm")) is None
        or finite_number(row.get("p95_error_mm")) is None
        or finite_number(row.get("latency_ms")) is None
        or finite_number(row.get("ack_observed_ratio")) is None
        or finite_number(row.get("state_age_p95_us")) is None
    ]


def build_arm_repeatability_aggregate(
    arm: str,
    rows: list[dict[str, Any]],
    missing_required: list[str],
) -> dict[str, Any]:
    required_names = required_names_for_arm(arm)
    arm_rows = [row for row in rows if row.get("name") in required_names]
    missing = [name for name in required_names if name in missing_required]
    present_names = {str(row.get("name")) for row in arm_rows}
    missing.extend(name for name in required_names if name not in present_names and name not in missing)
    aggregate = aggregate_repeatability(arm_rows)
    reasons: list[str] = []
    if missing:
        reasons.extend(f"missing {name}" for name in missing)

    wrong_arm = [row for row in arm_rows if row.get("arm") != arm]
    reasons.extend(f"{row.get('name')}: arm {row.get('arm')} != {arm}" for row in wrong_arm)

    metric_missing = metric_missing_rows(arm_rows)
    if metric_missing:
        reasons.append("missing per-run aggregate metrics: " + ", ".join(metric_missing))

    hard_failed = [row for row in arm_rows if row.get("hard_failures")]
    reasons.extend(
        f"{row.get('name')}: " + "; ".join(str(item) for item in row.get("hard_failures") or [])
        for row in hard_failed
    )

    failed_rows = [row for row in arm_rows if row.get("goal_pass") is not True]
    reasons.extend(
        f"{row.get('name')}: " + "; ".join(str(item) for item in row.get("failures") or [])
        for row in failed_rows
    )

    aggregate_failures = []
    if not missing and not metric_missing:
        aggregate_failures = aggregate_threshold_failures(aggregate)
        reasons.extend(aggregate_failures)

    required_count = len(required_names)
    if missing or len(arm_rows) < required_count or metric_missing:
        status = "insufficient_evidence"
    elif wrong_arm or hard_failed or failed_rows or aggregate_failures:
        status = "fail"
    else:
        status = "pass"

    return {
        "arm": arm,
        "status": status,
        "pass": status == "pass",
        "required_run_names": required_names,
        "missing_required_runs": list(dict.fromkeys(missing)),
        "required_run_count": len(arm_rows),
        "required_expected_run_count": required_count,
        "required_pass_count": sum(1 for row in arm_rows if row.get("goal_pass") is True),
        "failed_runs": failed_rows,
        "aggregate": aggregate,
        "aggregate_failures": aggregate_failures,
        "reasons": list(dict.fromkeys(reason for reason in reasons if reason)),
        **aggregate,
    }


def group_repeatability_rows(rows: list[dict[str, Any]], field: str) -> list[dict[str, Any]]:
    groups: dict[str, dict[str, Any]] = {}
    for row in rows:
        key = str(row.get(field) if row.get(field) not in (None, "") else "<missing>")
        group = groups.setdefault(
            key,
            {
                "group": key,
                "count": 0,
                "pass_count": 0,
                "fail_count": 0,
                "rows": [],
            },
        )
        group["count"] += 1
        if row.get("goal_pass") is True:
            group["pass_count"] += 1
        else:
            group["fail_count"] += 1
        group["rows"].append(row.get("name"))
    return [groups[key] for key in sorted(groups)]


def classify_repeatability(
    rows: list[dict[str, Any]],
    missing_required: list[str],
    per_arm: dict[str, dict[str, Any]],
) -> tuple[str, list[str]]:
    reasons: list[str] = []
    required_total = len(REPEATABILITY_REQUIRED_RUN_NAMES)
    if missing_required or len(rows) < required_total:
        reasons.extend(f"missing {name}" for name in missing_required)
        if len(rows) < required_total:
            reasons.append(f"required_run_count {len(rows)} < {required_total}")
        return "insufficient_evidence", list(dict.fromkeys(reasons))

    metric_missing = metric_missing_rows(rows)
    if metric_missing:
        reasons.append("missing per-run aggregate metrics: " + ", ".join(str(name) for name in metric_missing))
        return "insufficient_evidence", reasons

    if all(per_arm.get(arm, {}).get("pass") is True for arm in REPEATABILITY_THRESHOLDS["required_arms"]):
        return "repeatable_pass", []

    for arm in REPEATABILITY_THRESHOLDS["required_arms"]:
        arm_status = per_arm.get(arm, {})
        if arm_status.get("status") == "insufficient_evidence":
            reasons.extend(f"{arm}: {reason}" for reason in arm_status.get("reasons") or [])
            return "insufficient_evidence", list(dict.fromkeys(reasons))
        reasons.extend(f"{arm}: {reason}" for reason in arm_status.get("reasons") or [])
    return "not_repeatable", list(dict.fromkeys(reasons))


def build_repeatability_summary(candidates: list[dict[str, Any]], artifact_root: Path) -> dict[str, Any]:
    required_candidates, missing_required = select_repeatability_required_candidates(candidates)
    rows = [repeatability_row(candidate) for candidate in required_candidates]
    aggregate = aggregate_repeatability(rows)
    per_arm = {
        arm: build_arm_repeatability_aggregate(arm, rows, missing_required)
        for arm in REPEATABILITY_THRESHOLDS["required_arms"]
    }
    classification, classification_reasons = classify_repeatability(rows, missing_required, per_arm)
    failed_runs = [row for row in rows if row.get("goal_pass") is not True]
    optional_dual_rows = [
        repeatability_row(candidate)
        for candidate in candidates
        if str(candidate.get("name") or "").startswith("best_dual_sequential_")
    ]
    caveats = [
        "controller_reference_lower_bound_not_physical_real_tracking",
        "diagnostics_suspect_caveat_remains",
        "physical_real_motion_not_approved",
    ]
    if any(row.get("measurement_reliability_level") == "suspect" for row in rows):
        caveats.append("suspect_measurement_reliability")
    return {
        "schema": "robotics_lab.ackon500_repeatability_report.v1",
        "artifact_root": str(artifact_root.resolve()),
        "classification": classification,
        "classification_reasons": classification_reasons,
        "thresholds": REPEATABILITY_THRESHOLDS,
        "required_run_names": REPEATABILITY_REQUIRED_RUN_NAMES,
        "missing_required_runs": missing_required,
        "required_run_count": len(rows),
        "required_expected_run_count": len(REPEATABILITY_REQUIRED_RUN_NAMES),
        "required_pass_count": sum(1 for row in rows if row.get("goal_pass") is True),
        "global_repeatability_pass": classification == "repeatable_pass",
        "left_arm_aggregate": per_arm.get("left"),
        "right_arm_aggregate": per_arm.get("right"),
        "per_arm_aggregates": per_arm,
        "failed_runs": failed_runs,
        "aggregate": aggregate,
        "groups": {
            "arm": group_repeatability_rows(rows, "arm"),
            "benchmark_lane": group_repeatability_rows(rows, "benchmark_lane"),
            "acceptance_semantics": group_repeatability_rows(rows, "acceptance_semantics"),
            "tracking_source": group_repeatability_rows(rows, "tracking_source"),
        },
        "optional_dual_sequential_rows": optional_dual_rows,
        "physical_readiness": physical_readiness(),
        "caveats": caveats,
        "rows": rows,
    }


def repeatability_markdown(summary: dict[str, Any]) -> str:
    aggregate = dict(summary.get("aggregate") or {})
    aggregate_row = {
        "classification": summary.get("classification"),
        "required_run_count": summary.get("required_run_count"),
        "required_pass_count": summary.get("required_pass_count"),
        **aggregate,
    }
    arm_rows = [
        summary.get("left_arm_aggregate") or {},
        summary.get("right_arm_aggregate") or {},
    ]
    physical = summary.get("physical_readiness") or physical_readiness()
    groups = summary.get("groups") or {}
    reasons = summary.get("classification_reasons") or []
    missing = summary.get("missing_required_runs") or []
    failed = summary.get("failed_runs") or []
    return "\n".join(
        [
            "# ACKON500-REPEATABILITY-VALIDATION-01 Report",
            "",
            f"Classification: **{summary.get('classification')}**",
            "",
            "This is rbpodo controller pgmode-simulation evidence only. Physical robot motion is not approved.",
            "The metrics are controller-reference lower-bound tracking (`tcp_ref_stand`), not physical real TCP tracking.",
            "The diagnostics_suspect caveat remains and is not retired by repeatability validation.",
            "",
            "## Physical Readiness",
            "",
            f"- physical_readiness.status: `{physical.get('status')}`",
            "- physical_readiness.blockers: "
            + ", ".join(f"`{item}`" for item in physical.get("blockers", [])),
            "",
            "## Per-Arm Aggregate Pass",
            "",
            table(arm_rows, REPEATABILITY_ARM_AGGREGATE_COLUMNS),
            "",
            "## Aggregate",
            "",
            table([aggregate_row], REPEATABILITY_AGGREGATE_COLUMNS),
            "",
            "## Rows By Arm",
            "",
            table(groups.get("arm", []), REPEATABILITY_GROUP_COLUMNS),
            "",
            "## Rows By Benchmark Lane",
            "",
            table(groups.get("benchmark_lane", []), REPEATABILITY_GROUP_COLUMNS),
            "",
            "## Rows By Acceptance Semantics",
            "",
            table(groups.get("acceptance_semantics", []), REPEATABILITY_GROUP_COLUMNS),
            "",
            "## Rows By Tracking Source",
            "",
            table(groups.get("tracking_source", []), REPEATABILITY_GROUP_COLUMNS),
            "",
            "## Per-Run Metrics",
            "",
            table(summary.get("rows", []), REPEATABILITY_ROW_COLUMNS),
            "",
            "## Optional Dual Sequential Rows",
            "",
            table(summary.get("optional_dual_sequential_rows", []), REPEATABILITY_ROW_COLUMNS),
            "",
            "## Failed Or Missing Evidence",
            "",
            "\n".join(f"- missing: {item}" for item in missing)
            or "\n".join(
                f"- {row.get('name')}: " + "; ".join(str(item) for item in row.get("failures") or [])
                for row in failed
            )
            or "_None._",
            "",
            "## Classification Reasons",
            "",
            "\n".join(f"- {item}" for item in reasons) or "_None._",
            "",
            "## Caveats",
            "",
            "\n".join(f"- {item}" for item in summary.get("caveats", [])) or "_None._",
        ]
    )


def write_repeatability_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=REPEATABILITY_ROW_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: format_cell(row.get(field)) for field in REPEATABILITY_ROW_COLUMNS})


def report_markdown(summary: dict[str, Any]) -> str:
    best = summary.get("best_candidate") or {}
    candidates = summary.get("candidates", [])
    parts = [
        "# ACKON500-GENE-GOAL-01 Report",
        "",
        f"Official goal result: **{summary['official_goal_result'].upper()}**",
        "",
        f"**{ACKON500_PHYSICAL_WARNING}**",
        "",
        "This is rbpodo controller pgmode-simulation evidence only. Physical robot motion is not approved.",
        "",
        "Official pass lane: `rbpodo_server_side_circle_ackon500_sdk_worker`.",
        "",
        "The official orientation criterion is `p95_orientation_drift_rad <= 0.02`. "
        "`max_orientation_drift_rad` remains visible as a non-fatal diagnostic spike unless it is promoted in GOAL.md.",
        "",
        "## Canonical lane evidence",
        "",
        table(
            candidates,
            [
                "name",
                "benchmark_lane",
                "control_loop_location",
                "trajectory_generation_location",
                "feedback_loop_location",
                "low_level_send_mode",
                "acceptance_semantics",
                "tracking_source",
                "physical_motion_expected",
            ],
        ),
        "",
        "## Best candidate",
        "",
        table(
            [best] if best else [],
            [
                "name",
                "goal_pass",
                "ackon500_goal_status",
                "run_result_status",
                "safety_result_status",
                "benchmark_threshold_status",
                "diagnostic_warning_count",
                "benchmark_lane",
                "low_level_send_mode",
                "acceptance_semantics",
                "tracking_source",
                "udp_command_count",
                "server_servo_tick_count",
                "async_commands_sent_total",
                "async_commands_acked_total",
                "official_tracking_window_sec",
                "goal_window_commands_sent",
                "goal_window_commands_acked",
                "ack_coverage_ratio",
                "effective_goal_command_rate_hz",
                "worker_send_rate_hz",
                "worker_sends_outside_official_window",
                "commands_sent_total",
                "commands_acked_total",
                "rms_error_m",
                "p95_error_m",
                "radius_gain",
                "effective_phase_latency_abs_ms",
                "state_age_p95_us",
                "measurement_reliability_level",
            ],
        ),
        "",
        "## Physical readiness",
        "",
        f"- physical_readiness.status: `{summary['physical_readiness']['status']}`",
        "- physical_readiness.blockers: "
        + ", ".join(f"`{item}`" for item in summary["physical_readiness"]["blockers"]),
        "- physical_readiness.next_required_acceptance: "
        + ", ".join(f"`{item}`" for item in summary["physical_readiness"]["next_required_acceptance"]),
        f"- controller_reference_result.status: `{summary['controller_reference_result']['status']}`",
        f"- controller_reference_result.explanation: {summary['controller_reference_result']['explanation']}",
        f"- physical_tracking_result.status: `{summary['physical_tracking_result']['status']}`",
        "",
        "## Run execution result",
        "",
        table(
            candidates,
            ["name", "run_result_status", "safety_result_status", "result_reason"],
        ),
        "",
        "## Generic diagnostic threshold result",
        "",
        "Generic benchmark thresholds are reported separately from the official ACKON500 goal. "
        "A candidate can be goal PASS while carrying a diagnostic warning, for example a "
        "`max_orientation_drift_rad` spike when p95 orientation still satisfies the official goal.",
        "",
        table(
            candidates,
            [
                "name",
                "benchmark_threshold_status",
                "threshold_failures",
                "diagnostic_warnings",
            ],
        ),
        "",
        "## Limiting factors",
        "",
        "\n".join(f"- {item}" for item in summary.get("limiting_factors", [])) or "_None._",
        "",
        "## Candidate rows",
        "",
        table(
            candidates,
            [
                "name",
                "goal_pass",
                "ackon500_goal_status",
                "run_result_status",
                "benchmark_threshold_status",
                "benchmark_lane",
                "low_level_send_mode",
                "async_mode",
                "acceptance_semantics",
                "tracking_source",
                "repeat",
                "servo_rate_hz",
                "servo_t1_sec",
                "udp_command_count",
                "server_servo_tick_count",
                "official_tracking_window_sec",
                "goal_window_commands_sent",
                "goal_window_commands_acked",
                "ack_coverage_ratio",
                "effective_goal_command_rate_hz",
                "worker_send_rate_hz",
                "rms_error_m",
                "p95_error_m",
                "fit_center_error_m",
                "radius_gain",
                "p95_orientation_drift_rad",
                "effective_phase_latency_abs_ms",
                "state_age_p95_us",
                "feedback_saturation_ratio",
                "diagnostic_warning_count",
            ],
        ),
        "",
        "## Safety",
        "",
        "- Required mode: rbpodo controller pgmode simulation.",
        "- Required physical flags: physical_motion_expected=false and physical_motion_detected=false.",
        "- Socket-send-only evidence is always rejected for this goal.",
    ]
    return "\n".join(parts)


def timing_markdown(summary: dict[str, Any]) -> str:
    return "\n".join(
        [
            "# ACKON500-GENE-GOAL-01 Timing Report",
            "",
            table(
                summary.get("candidates", []),
                [
                    "name",
                    "goal_pass",
                    "run_result_status",
                    "benchmark_threshold_status",
                    "benchmark_lane",
                    "low_level_send_mode",
                    "async_mode",
                    "acceptance_semantics",
                    "udp_command_count",
                    "server_servo_tick_count",
                    "async_commands_enqueued_total",
                    "async_commands_sent_total",
                    "async_commands_acked_total",
                    "commands_sent_total",
                    "commands_acked_total",
                    "ack_observed_ratio",
                    "ack_coverage_ratio",
                    "official_tracking_window_sec",
                    "measured_worker_window_sec",
                    "official_servo_rate_hz",
                    "goal_window_commands_sent",
                    "goal_window_commands_acked",
                    "effective_goal_command_rate_hz",
                    "worker_send_rate_hz",
                    "worker_lifetime_send_rate_hz",
                    "worker_sends_outside_official_window",
                    "udp_effective_command_rate_hz",
                    "state_age_p95_us",
                    "estimated_latency_ms",
                    "commanded_phase_advance_ms",
                    "uncompensated_latency_estimate_ms",
                    "effective_phase_latency_abs_ms",
                    "timing_classification",
                ],
            ),
        ]
    )


def error_markdown(summary: dict[str, Any]) -> str:
    return "\n".join(
        [
            "# ACKON500-GENE-GOAL-01 Error Decomposition Report",
            "",
            table(
                summary.get("candidates", []),
                [
                    "name",
                    "goal_pass",
                    "benchmark_threshold_status",
                    "rms_error_m",
                    "p95_error_m",
                    "fit_center_error_m",
                    "radius_gain",
                    "p95_orientation_drift_rad",
                    "max_orientation_drift_rad",
                    "diagnostic_warnings",
                    "feedback_saturation_ratio",
                    "measurement_reliability_level",
                    "failures",
                ],
            ),
        ]
    )


def build_summary(artifact_root: Path) -> dict[str, Any]:
    if not artifact_root.is_dir():
        raise ReportError(f"artifact root not found: {artifact_root}")
    ablation_rows = read_ablation_rows(artifact_root)
    candidates = [
        candidate_from_summary(path, ablation_rows)
        for path in summary_paths(artifact_root)
    ]
    candidates = [
        candidate for candidate in candidates
        if candidate.get("profile") == "gene_15cm_4s"
    ]
    candidates.sort(key=score)
    best = candidates[0] if candidates else None
    result = "pass" if best and best.get("pass") else "fail"
    limiting = []
    if best:
        limiting = list(best.get("failures") or [])
    elif not candidates:
        limiting = ["no gene_15cm_4s summary.json artifacts found"]
    repeatability = build_repeatability_summary(candidates, artifact_root)
    return {
        "schema": SCHEMA,
        "artifact_root": str(artifact_root.resolve()),
        "result": result,
        "pass": result == "pass",
        "goal_pass": result == "pass",
        "official_goal_result": result,
        "physical_readiness": physical_readiness(),
        "controller_reference_result": controller_reference_result(
            result == "pass"
            and best is not None
            and best.get("tracking_source") == "tcp_ref_stand"
        ),
        "physical_tracking_result": physical_tracking_result(),
        "thresholds": PASS_THRESHOLDS,
        "candidate_count": len(candidates),
        "best_candidate": best,
        "limiting_factors": limiting,
        "candidates": candidates,
        "repeatability": repeatability,
    }


def main() -> int:
    args = parse_args()
    artifact_root = args.artifact_root.resolve()
    output_summary = args.output_summary or artifact_root / "summary.json"
    output_md = args.output_md or artifact_root / "gene_goal_report.md"
    timing_report = args.timing_report or artifact_root / "timing_report.md"
    error_report = args.error_report or artifact_root / "error_decomposition_report.md"
    repeatability_json = args.repeatability_summary_json or artifact_root / "repeatability_summary.json"
    repeatability_csv = args.repeatability_summary_csv or artifact_root / "repeatability_summary.csv"
    repeatability_report = args.repeatability_report or artifact_root / "repeatability_report.md"
    try:
        summary = build_summary(artifact_root)
        summary["gene_goal_report"] = str(output_md.resolve())
        summary["timing_report"] = str(timing_report.resolve())
        summary["error_decomposition_report"] = str(error_report.resolve())
        summary["repeatability_summary_json"] = str(repeatability_json.resolve())
        summary["repeatability_summary_csv"] = str(repeatability_csv.resolve())
        summary["repeatability_report"] = str(repeatability_report.resolve())
        write_json(output_summary, summary)
        write_text(output_md, report_markdown(summary))
        write_text(timing_report, timing_markdown(summary))
        write_text(error_report, error_markdown(summary))
        repeatability = summary["repeatability"]
        write_json(repeatability_json, repeatability)
        write_repeatability_csv(repeatability_csv, repeatability.get("rows", []))
        write_text(repeatability_report, repeatability_markdown(repeatability))
    except Exception as exc:
        print(f"generate_ackon500_gene_goal_report: FAIL: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(summary, indent=2, sort_keys=True))
    if args.require_pass and summary.get("goal_pass") is not True:
        return 2
    if args.require_repeatable and summary.get("repeatability", {}).get("classification") != "repeatable_pass":
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
