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


SCHEMA = "robotics_lab.ackon500_gene_goal_report.v1"
PASS_THRESHOLDS = {
    "min_repeat": 5,
    "servo_rate_hz": 500.0,
    "servo_t1_sec": 0.002,
    "min_ack_ratio": 0.98,
    "min_effective_command_rate_hz": 490.0,
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
    parser.add_argument("--require-pass", action="store_true")
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


def async_metrics(states: list[dict[str, Any]], arm: str) -> dict[str, Any]:
    samples = [
        sample
        for snapshot in states
        for sample in [async_state(snapshot, arm)]
        if sample is not None
    ]
    modes = [str(sample.get("mode")) for sample in samples if sample.get("mode") not in (None, "")]
    mode = max(set(modes), key=modes.count) if modes else None
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
        and command_mode(packet, arm).startswith("Tcp")
    ]
    if len(times) < 2:
        return {"udp_command_count": len(times), "udp_effective_command_rate_hz": None}
    elapsed_sec = (times[-1] - times[0]) / 1e9
    rate = (len(times) - 1) / elapsed_sec if elapsed_sec > 0 else None
    return {
        "udp_command_count": len(times),
        "udp_command_start_ns": times[0],
        "udp_command_end_ns": times[-1],
        "udp_effective_command_rate_hz": rate,
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


def artifact_exists(path_text: Any, fallback: Path) -> bool:
    path = Path(str(path_text)) if isinstance(path_text, str) and path_text else fallback
    return path.is_file()


def candidate_from_summary(path: Path, ablation_rows: dict[str, dict[str, str]]) -> dict[str, Any]:
    summary = load_json(path)
    artifact_dir = path.parent.resolve()
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
    controller_send_rate = commands_sent / duration_sec if commands_sent > 0 and duration_sec and duration_sec > 0 else None
    effective_command_rate = first_number(controller_send_rate, packet_rate.get("udp_effective_command_rate_hz"))
    state_age_p95 = first_number(
        nested_metric(summary, "state_age_us", "p95"),
        nested_metric(summary.get("timestamp_alignment", {}) if isinstance(summary.get("timestamp_alignment"), dict) else {}, "state_age_us", "p95"),
    )
    estimated_latency_ms = finite_number(summary.get("estimated_latency_ms"))
    phase_advance_ms = first_number(summary.get("commanded_phase_advance_ms"), row.get("commanded_phase_advance_ms"), 0.0)
    candidate = {
        "name": first_value(row.get("name"), summary.get("name"), artifact_dir.name),
        "artifact_dir": str(artifact_dir),
        "summary_json": str(path.resolve()),
        "profile": first_value(summary.get("profile"), row.get("profile")),
        "repeat": first_number(summary.get("repeat"), row.get("repeat")),
        "tracking_source": first_value(summary.get("tracking_source_used"), summary.get("tracking_source"), row.get("tracking_source")),
        "servo_rate_hz": first_number(summary.get("servo_rate_hz"), row.get("servo_rate_hz")),
        "servo_t1_sec": first_number(summary.get("servo_t1_sec"), row.get("servo_t1_sec")),
        "command_rate_hz": first_number(summary.get("command_rate_hz"), row.get("command_rate_hz")),
        "async_mode": first_value(summary.get("async_mode"), summary.get("async_streaming_mode"), row.get("async_mode"), async_info.get("mode")),
        "acceptance_semantics": derive_acceptance_semantics(summary, row, async_info),
        "commands_sent_total": commands_sent,
        "commands_acked_total": commands_acked,
        "socket_send_only_count": socket_sent,
        "ack_observed_ratio": ack_ratio,
        "effective_command_rate_hz": effective_command_rate,
        "controller_send_rate_hz": controller_send_rate,
        **packet_rate,
        "async_streaming_metrics": async_info,
        "rms_error_m": first_number(summary.get("rms_error_m"), scaled_number(row.get("rms_error_mm"), 0.001)),
        "p95_error_m": first_number(summary.get("p95_error_m"), scaled_number(row.get("p95_error_mm"), 0.001)),
        "fit_center_error_m": first_number(summary.get("fit_center_error_m"), scaled_number(row.get("fit_center_error_mm"), 0.001)),
        "radius_gain": first_number(summary.get("radius_gain"), row.get("radius_gain")),
        "p95_orientation_drift_rad": first_number(summary.get("p95_orientation_drift_rad"), scaled_number(row.get("p95_orientation_drift_mrad"), 0.001)),
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
        "cartesian_unavailable_count": first_number(summary.get("cartesian_unavailable_count"), row.get("cartesian_unavailable_count")),
        "measurement_reliability_level": first_value(summary.get("measurement_reliability_level"), row.get("measurement_reliability_level")),
        "timing_classification": first_value(summary.get("timing_classification"), row.get("timing_classification")),
        "result": summary.get("result"),
        "result_reason": summary.get("result_reason"),
        "threshold_failures": summary.get("threshold_failures"),
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
    candidate["required_artifacts_present"] = required_artifacts_present(candidate)
    candidate["failures"] = goal_failures(candidate)
    candidate["pass"] = not candidate["failures"]
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
    check_min(candidate, "repeat", PASS_THRESHOLDS["min_repeat"], failures)
    if candidate.get("tracking_source") != "tcp_ref_stand":
        failures.append(f"tracking_source {candidate.get('tracking_source')} != tcp_ref_stand")
    if not near(candidate.get("servo_rate_hz"), PASS_THRESHOLDS["servo_rate_hz"]):
        failures.append(f"servo_rate_hz {candidate.get('servo_rate_hz')} != 500")
    if not near(candidate.get("servo_t1_sec"), PASS_THRESHOLDS["servo_t1_sec"]):
        failures.append(f"servo_t1_sec {candidate.get('servo_t1_sec')} != 0.002")
    if candidate.get("acceptance_semantics") not in {"controller_ack_observed", "sdk_worker_ack_observed"}:
        failures.append(f"acceptance_semantics {candidate.get('acceptance_semantics')} is not ACK-observed")
    check_min(candidate, "ack_observed_ratio", PASS_THRESHOLDS["min_ack_ratio"], failures)
    if int(candidate.get("socket_send_only_count") or 0) != 0:
        failures.append(f"socket_send_only_count {candidate.get('socket_send_only_count')} != 0")
    check_min(candidate, "effective_command_rate_hz", PASS_THRESHOLDS["min_effective_command_rate_hz"], failures)
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


def report_markdown(summary: dict[str, Any]) -> str:
    best = summary.get("best_candidate") or {}
    candidates = summary.get("candidates", [])
    parts = [
        "# ACKON500-GENE-GOAL-01 Report",
        "",
        f"Result: **{summary['result'].upper()}**",
        "",
        "This is rbpodo controller pgmode-simulation evidence only. Physical robot motion is not approved.",
        "",
        "## Best candidate",
        "",
        table(
            [best] if best else [],
            [
                "name",
                "pass",
                "acceptance_semantics",
                "commands_sent_total",
                "commands_acked_total",
                "ack_observed_ratio",
                "effective_command_rate_hz",
                "rms_error_m",
                "p95_error_m",
                "radius_gain",
                "effective_phase_latency_abs_ms",
                "state_age_p95_us",
                "measurement_reliability_level",
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
                "pass",
                "async_mode",
                "acceptance_semantics",
                "repeat",
                "servo_rate_hz",
                "servo_t1_sec",
                "ack_observed_ratio",
                "effective_command_rate_hz",
                "rms_error_m",
                "p95_error_m",
                "fit_center_error_m",
                "radius_gain",
                "p95_orientation_drift_rad",
                "effective_phase_latency_abs_ms",
                "state_age_p95_us",
                "feedback_saturation_ratio",
                "result",
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
                    "async_mode",
                    "commands_sent_total",
                    "commands_acked_total",
                    "ack_observed_ratio",
                    "controller_send_rate_hz",
                    "udp_effective_command_rate_hz",
                    "effective_command_rate_hz",
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
                    "rms_error_m",
                    "p95_error_m",
                    "fit_center_error_m",
                    "radius_gain",
                    "p95_orientation_drift_rad",
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
    return {
        "schema": SCHEMA,
        "artifact_root": str(artifact_root.resolve()),
        "result": result,
        "pass": result == "pass",
        "thresholds": PASS_THRESHOLDS,
        "candidate_count": len(candidates),
        "best_candidate": best,
        "limiting_factors": limiting,
        "candidates": candidates,
    }


def main() -> int:
    args = parse_args()
    artifact_root = args.artifact_root.resolve()
    output_summary = args.output_summary or artifact_root / "summary.json"
    output_md = args.output_md or artifact_root / "gene_goal_report.md"
    timing_report = args.timing_report or artifact_root / "timing_report.md"
    error_report = args.error_report or artifact_root / "error_decomposition_report.md"
    try:
        summary = build_summary(artifact_root)
        summary["gene_goal_report"] = str(output_md.resolve())
        summary["timing_report"] = str(timing_report.resolve())
        summary["error_decomposition_report"] = str(error_report.resolve())
        write_json(output_summary, summary)
        write_text(output_md, report_markdown(summary))
        write_text(timing_report, timing_markdown(summary))
        write_text(error_report, error_markdown(summary))
    except Exception as exc:
        print(f"generate_ackon500_gene_goal_report: FAIL: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(summary, indent=2, sort_keys=True))
    if args.require_pass and summary.get("result") != "pass":
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
