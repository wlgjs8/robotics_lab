#!/usr/bin/env python3
"""Generate circle benchmark reporting tables and promotion guidance."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import compare_circle_benchmarks as compare


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
    "backend",
    "controller_mode",
    "controller",
    "arm",
    "profile",
    "tracking_source",
    "diameter_m",
    "period_sec",
    "repeat_evidence_count",
    "radius_gain",
    "rms_error_mm",
    "p95_error_mm",
    "max_error_mm",
    "p95_orientation_drift_mrad",
    "max_orientation_drift_mrad",
    "estimated_latency_ms",
    "worker_command_drops_total",
    "integrator_clamps_total",
    "integrator_divergence_total",
    "send_command_deadline_missed_count",
    "command_interval_max_ms",
    "servo_jitter_max_ms",
    "physical_motion_expected",
    "physical_motion_detected",
    "q_ref_update_rate_hz",
    "q_actual_update_rate_hz",
    "ack_policy",
    "controller_acceptance_observed_count",
    "command_timeout_count",
    "controller_rejected_count",
    "tcp_ref_valid_ratio",
    "tcp_actual_valid_ratio",
    "result",
    "classification",
    "real_candidate_policy",
    "performance_warnings",
    "promotion_notes",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate a markdown and CSV report for simulator-only circle benchmark "
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
            if row.get("ack_policy") or row.get("q_ref_update_rate_hz") or row.get("tracking_source"):
                row.setdefault("backend", "rbpodo")
                row.setdefault("benchmark_category", "rbpodo_controller_simulation")
                row.setdefault("controller_mode", "pgmode_simulation")
                row.setdefault("physical_motion_expected", "false")
            else:
                row.setdefault("backend", "simulator")
                row.setdefault("benchmark_category", "rb_simulator")
                row.setdefault("controller_mode", "rb_simulator")
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


def classify_row(row: dict[str, Any], min_repeats: int) -> None:
    category = row_category(row)
    row["benchmark_category"] = category
    if category == "rbpodo_controller_simulation" and row.get("backend") in (None, ""):
        row["backend"] = "rbpodo"
    if category == "rbpodo_controller_simulation" and row.get("controller_mode") in (None, ""):
        row["controller_mode"] = "pgmode_simulation"
    if category == "rb_simulator" and row.get("backend") in (None, ""):
        row["backend"] = "simulator"
    baseline = baseline_failures(row, min_repeats)
    stress = stress_failures(row)
    rbpodo_baseline = rbpodo_stable_failures(row)
    rbpodo_stress = rbpodo_stress_failures(row)
    if category == "rbpodo_controller_simulation" and row.get("profile") == BASELINE_PROFILE and not rbpodo_baseline:
        row["classification"] = "stable_rbpodo_controller_sim_baseline_candidate"
        row["real_candidate_policy"] = "future_low_speed_seed_only_not_real_ready"
        row["promotion_notes"] = (
            "meets rbpodo pgmode-simulation baseline criteria using tcp_ref_stand; "
            "may guide future low-speed planning but is not physical-motion evidence"
        )
    elif category == "rbpodo_controller_simulation" and row.get("profile") == STRESS_PROFILE and not rbpodo_stress:
        row["classification"] = "rbpodo_controller_sim_stress_candidate"
        row["real_candidate_policy"] = "controller_sim_stress_not_real_ready"
        row["promotion_notes"] = (
            "GENE-style rbpodo pgmode-simulation stress evidence; not real-ready; "
            "do not copy speed/gains directly to real"
        )
    elif category == "rbpodo_controller_simulation" and row.get("profile") == BASELINE_PROFILE:
        row["classification"] = "rbpodo_controller_sim_baseline_incomplete"
        row["real_candidate_policy"] = "not_real_ready"
        row["promotion_notes"] = "; ".join(rbpodo_baseline)
    elif category == "rbpodo_controller_simulation" and row.get("profile") == STRESS_PROFILE:
        row["classification"] = "rbpodo_controller_sim_stress_rejected_or_incomplete"
        row["real_candidate_policy"] = "controller_sim_stress_not_real_ready"
        row["promotion_notes"] = "; ".join(rbpodo_stress)
    elif category == "real_physical_benchmark":
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

## rbpodo Controller-Simulation Decision Rules

A stable rbpodo controller-simulation baseline candidate must satisfy all of:

- category `rbpodo_controller_simulation`
- `backend == rbpodo`
- `controller_mode == pgmode_simulation`
- profile `{BASELINE_PROFILE}`
- `tracking_source == tcp_ref_stand`
- `physical_motion_expected == false`
- `physical_motion_detected == false`
- `radius_gain` in `[0.98, 1.02]`
- `rms_error_m <= 0.005`
- `p95_error_m <= 0.007`
- `fault_latched == false`
- command/source drops, timeouts, and controller rejections are zero when reported
- ACK-on runs have `controller_acceptance_observed_count > 0`

A `{STRESS_PROFILE}` rbpodo controller-simulation run is only a stress candidate
when `radius_gain` is in `[0.90, 1.10]`, RMS and p95 errors are recorded,
feedback saturation is recorded for feedback controllers,
`physical_motion_detected == false`, and `fault_latched == false`.

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


def report_markdown(rows: list[dict[str, Any]], title: str, min_repeats: int) -> str:
    baseline = [row for row in rows if row.get("classification") == "stable_simulator_baseline_candidate"]
    stress = [row for row in rows if row.get("classification") == "stress_benchmark_candidate"]
    rbpodo_rows = [row for row in rows if row.get("benchmark_category") == "rbpodo_controller_simulation"]
    rbpodo_baseline = [
        row for row in rows
        if row.get("classification") == "stable_rbpodo_controller_sim_baseline_candidate"
    ]
    rbpodo_stress = [
        row for row in rows
        if row.get("classification") == "rbpodo_controller_sim_stress_candidate"
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
        }
    ]
    parts = [
        f"# {title}",
        "",
        "This report separates simulator, rbpodo controller-simulation, and future real physical evidence. It does not authorize real robot motion.",
        "",
        criteria_markdown(min_repeats).rstrip(),
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
        "## Stable rbpodo Controller-Simulation Baseline Candidates",
        "",
        markdown_table(rbpodo_baseline) if rbpodo_baseline else "_None._",
        "",
        "## GENE-Style rbpodo Controller-Simulation Stress Candidates",
        "",
        markdown_table(rbpodo_stress) if rbpodo_stress else "_None._",
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
        "p95_error_m <value>, physical_motion_detected false. pgmode simulation only; not real-ready.",
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
