#!/usr/bin/env python3
"""Compare circle tracking benchmark summary.json artifacts."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any


COLUMNS = [
    ("run name", "run_name"),
    ("controller", "controller"),
    ("arm", "arm"),
    ("profile", "profile"),
    ("diameter_m", "diameter_m"),
    ("period_sec", "period_sec"),
    ("radius_gain", "radius_gain"),
    ("mean_error_mm", "mean_error_mm"),
    ("rms_error_mm", "rms_error_mm"),
    ("p95_error_mm", "p95_error_mm"),
    ("max_error_mm", "max_error_mm"),
    ("p95_orientation_drift_mrad", "p95_orientation_drift_mrad"),
    ("max_orientation_drift_mrad", "max_orientation_drift_mrad"),
    ("estimated_latency_ms", "estimated_latency_ms"),
    ("worker_command_drops_total", "worker_command_drops_total"),
    ("integrator_clamps_total", "integrator_clamps_total"),
    ("integrator_divergence_total", "integrator_divergence_total"),
    ("send_command_deadline_missed_count", "send_command_deadline_missed_count"),
    ("command_interval_max_ms", "command_interval_max_ms"),
    ("servo_jitter_max_ms", "servo_jitter_max_ms"),
    ("mean_feedback_linear_norm_m_s", "mean_feedback_linear_norm_m_s"),
    ("max_feedback_linear_norm_m_s", "max_feedback_linear_norm_m_s"),
    ("feedback_saturation_count", "feedback_saturation_count"),
    ("stale_state_feedback_skips", "stale_state_feedback_skips"),
    ("result", "result"),
    ("performance_warnings", "performance_warnings"),
]

PROFILE_BY_DIMENSION = {
    (0.05, 10.0): "safe_5cm_10s",
    (0.15, 16.0): "circle_15cm_16s",
    (0.15, 8.0): "circle_15cm_8s",
    (0.15, 4.0): "gene_15cm_4s",
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
    if isinstance(warnings, list):
        return "; ".join(str(item) for item in warnings)
    if isinstance(warnings, str):
        return warnings
    return ""


def comparison_row(summary: dict[str, Any]) -> dict[str, Any]:
    return {
        "run_name": run_name(summary),
        "controller": summary.get("controller"),
        "arm": summary.get("arm"),
        "profile": inferred_profile(summary),
        "diameter_m": summary.get("diameter_m"),
        "period_sec": summary.get("period_sec"),
        "repeat": summary.get("repeat"),
        "radius_gain": radius_gain(summary),
        "mean_error_mm": scaled(summary, "mean_error_m", 1000.0),
        "rms_error_mm": scaled(summary, "rms_error_m", 1000.0),
        "p95_error_mm": scaled(summary, "p95_error_m", 1000.0),
        "max_error_mm": scaled(summary, "max_error_m", 1000.0),
        "p95_orientation_drift_mrad": scaled(summary, "p95_orientation_drift_rad", 1000.0),
        "max_orientation_drift_mrad": scaled(summary, "max_orientation_drift_rad", 1000.0),
        "estimated_latency_ms": summary.get("estimated_latency_ms"),
        "worker_command_drops_total": summary.get("worker_command_drops_total"),
        "integrator_clamps_total": summary.get("integrator_clamps_total"),
        "integrator_divergence_total": summary.get("integrator_divergence_total"),
        "send_command_deadline_missed_count": summary.get("send_command_deadline_missed_count"),
        "command_interval_max_ms": command_interval_max_ms(summary),
        "servo_jitter_max_ms": servo_jitter_max_ms(summary),
        "mean_feedback_linear_norm_m_s": summary.get("mean_feedback_linear_norm_m_s"),
        "max_feedback_linear_norm_m_s": summary.get("max_feedback_linear_norm_m_s"),
        "feedback_saturation_count": summary.get("feedback_saturation_count"),
        "stale_state_feedback_skips": summary.get("stale_state_feedback_skips"),
        "fault_latched": summary.get("fault_latched"),
        "result": summary.get("result"),
        "performance_warnings": warning_text(summary),
    }


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
        rows.sort(
            key=lambda row: (
                finite_number(row.get("rms_error_mm"))
                if finite_number(row.get("rms_error_mm")) is not None
                else math.inf
            )
        )
    write_markdown(rows)
    if args.csv_path:
        write_csv(args.csv_path, rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
