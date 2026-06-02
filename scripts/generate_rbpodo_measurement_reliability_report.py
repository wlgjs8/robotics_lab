#!/usr/bin/env python3
"""Generate rbpodo pgmode benchmark measurement reliability reports.

The grading in this module is reporting-only. It does not connect to
controllers, send commands, or change robot state.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any


SCHEMA = "robotics_lab.rbpodo_measurement_reliability_report.v1"
OUTPUT_MD = "measurement_reliability_report.md"
OUTPUT_CSV = "measurement_reliability_summary.csv"
OUTPUT_JSON = "measurement_reliability_summary.json"
Q_REF_VALID_RATIO_MIN = 0.95
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

RELIABILITY_COLUMNS = [
    "run_name",
    "profile",
    "tracking_source",
    "result",
    "measurement_reliability_level",
    "reliability_caveats",
    "benchmark_interpretation",
    "physical_real_blockers",
]


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


def split_list_cell(value: Any) -> list[str]:
    if isinstance(value, list):
        return [str(item) for item in value if str(item)]
    if isinstance(value, str):
        pieces = [piece.strip() for piece in value.split(";")]
        return [piece for piece in pieces if piece]
    return []


def unique(values: list[str]) -> list[str]:
    return list(dict.fromkeys(value for value in values if value))


def list_cell(values: list[str]) -> str:
    return "; ".join(unique(values))


def nested_dict(row: dict[str, Any], key: str) -> dict[str, Any]:
    value = row.get(key)
    return value if isinstance(value, dict) else {}


def first_present(row: dict[str, Any], *keys: str) -> Any:
    for key in keys:
        if key in row and row.get(key) is not None:
            return row.get(key)
    return None


def row_name(row: dict[str, Any]) -> str:
    for key in ("run_name", "name"):
        value = row.get(key)
        if isinstance(value, str) and value:
            return value
    artifact_dir = row.get("artifact_dir")
    if isinstance(artifact_dir, str) and artifact_dir:
        return Path(artifact_dir).name
    summary_path = row.get("_summary_path")
    if isinstance(summary_path, str) and summary_path:
        return Path(summary_path).parent.name or Path(summary_path).name
    return ""


def tracking_source(row: dict[str, Any]) -> str:
    value = first_present(row, "tracking_source", "tracking_source_used", "tracking_source_requested")
    return str(value) if value is not None else ""


def benchmark_category(row: dict[str, Any]) -> str:
    category = str(row.get("benchmark_category") or "")
    if category:
        return category
    backend = str(row.get("backend") or "")
    controller_mode = str(row.get("controller_mode") or "")
    if backend == "rbpodo" and controller_mode == "pgmode_simulation":
        return "rbpodo_controller_simulation"
    schema = str(row.get("schema") or "")
    if "rbpodo_circle_tracking_benchmark" in schema:
        return "rbpodo_controller_simulation"
    preflight = nested_dict(row, "safety_preflight")
    if preflight.get("controller_simulation_only") is True or preflight.get("pgmode_simulation_confirmed") is True:
        return "rbpodo_controller_simulation"
    return category


def is_rbpodo_pgmode(row: dict[str, Any]) -> bool:
    return benchmark_category(row) == "rbpodo_controller_simulation" or str(row.get("controller_mode") or "") == "pgmode_simulation"


def diagnostics_suspect_active(row: dict[str, Any]) -> bool:
    if (finite_number(row.get("diagnostics_suspect_count")) or 0.0) > 0.0:
        return True
    if (finite_number(row.get("controller_simulation_diagnostic_override_active_count")) or 0.0) > 0.0:
        return True
    if as_bool(row.get("controller_simulation_diagnostic_override_active")) is True:
        return True
    warnings = str(row.get("warnings") or row.get("performance_warnings") or "")
    return "diagnostics_suspect" in warnings or "diagnostic_override" in warnings


def diagnostics_override_active(row: dict[str, Any]) -> bool:
    if (finite_number(row.get("controller_simulation_diagnostic_override_active_count")) or 0.0) > 0.0:
        return True
    if as_bool(row.get("controller_simulation_diagnostic_override_active")) is True:
        return True
    warnings = str(row.get("warnings") or row.get("performance_warnings") or "")
    return "diagnostic_override" in warnings


def state_parity_result(row: dict[str, Any]) -> str:
    for key in (
        "state_parity_result",
        "rbpodo_state_parity_result",
        "python_cpp_state_parity_result",
        "parity_result",
    ):
        value = row.get(key)
        if isinstance(value, str) and value:
            return value
    for key in ("state_parity", "rbpodo_state_parity", "python_cpp_state_parity", "parity_summary"):
        nested = nested_dict(row, key)
        value = nested.get("result")
        if isinstance(value, str) and value:
            return value
    return ""


def state_parity_failed(row: dict[str, Any]) -> bool:
    result = state_parity_result(row)
    if not result:
        return False
    if result in {"passed", "ok", "suspect_but_consistent", "parity_suspect"}:
        return False
    return result.startswith("failed") or result in {"mismatch", "error", "transport_error"}


def state_parity_passed(row: dict[str, Any]) -> bool:
    return state_parity_result(row) in {"passed", "ok"}


def state_available(row: dict[str, Any]) -> bool:
    state_packets = finite_number(row.get("state_packet_count"))
    if state_packets is not None:
        return state_packets > 0.0
    if finite_number(row.get("tcp_ref_valid_ratio")) is not None:
        return True
    if finite_number(row.get("q_ref_update_rate_hz")) is not None:
        return True
    return row.get("result") not in {"state_stream_timeout", "no_state"}


def blocked_or_cartesian_unavailable(row: dict[str, Any]) -> bool:
    if row.get("result") == "blocked":
        return True
    if as_bool(row.get("server_rejected_cartesian")) is True:
        return True
    unavailable = finite_number(row.get("cartesian_unavailable_count"))
    return unavailable is not None and unavailable > 0.0


def q_ref_visible(row: dict[str, Any]) -> bool:
    q_ref_valid_ratio = finite_number(first_present(row, "q_ref_valid_ratio", "q_reference_for_servo_valid_ratio"))
    if q_ref_valid_ratio is not None:
        return q_ref_valid_ratio >= Q_REF_VALID_RATIO_MIN
    update_rate = finite_number(row.get("q_ref_update_rate_hz"))
    if update_rate is not None and update_rate > 0.0:
        return True
    reason = str(row.get("q_ref_reason") or "")
    return "published" in reason and "not " not in reason


def q_ref_low_or_missing(row: dict[str, Any]) -> bool:
    ratio = finite_number(first_present(row, "q_ref_valid_ratio", "q_reference_for_servo_valid_ratio"))
    if ratio is not None and ratio < Q_REF_VALID_RATIO_MIN:
        return True
    reason = str(row.get("q_ref_reason") or "")
    if "not published" in reason or "not valid" in reason:
        return True
    return not q_ref_visible(row)


def q_ref_ratio_missing_or_low(row: dict[str, Any]) -> bool:
    ratio = finite_number(first_present(row, "q_ref_valid_ratio", "q_reference_for_servo_valid_ratio"))
    return ratio is None or ratio < Q_REF_VALID_RATIO_MIN


def tcp_ref_high(row: dict[str, Any]) -> bool:
    ratio = finite_number(row.get("tcp_ref_valid_ratio"))
    return ratio is not None and ratio >= 0.95


def timing_jitter_present(row: dict[str, Any]) -> bool:
    timing = str(row.get("timing_classification") or "")
    if timing and timing != "clean_timing":
        return True
    for key in ("state_gap_count", "command_gap_count", "ack_spike_count_10ms", "ack_spike_count_20ms"):
        value = finite_number(row.get(key))
        if value is not None and value > 0.0:
            return True
    return False


def saturation_ratio(row: dict[str, Any]) -> float | None:
    value = finite_number(row.get("saturation_ratio"))
    if value is not None:
        return value
    count = finite_number(row.get("feedback_saturation_count"))
    command_count = finite_number(first_present(row, "command_count", "ack_observed_count", "controller_acceptance_observed_count"))
    if count is None or command_count is None or command_count <= 0.0:
        return None
    return count / command_count


def feedback_saturation_present(row: dict[str, Any]) -> bool:
    ratio = saturation_ratio(row)
    if ratio is not None and ratio > 0.0:
        return True
    count = finite_number(row.get("feedback_saturation_count"))
    return count is not None and count > 0.0


def orientation_high(row: dict[str, Any]) -> bool:
    equiv_mm = finite_number(row.get("orientation_position_equiv_50mm_mm"))
    if equiv_mm is not None and equiv_mm >= 5.0:
        return True
    equiv_m = finite_number(row.get("orientation_position_equiv_50mm_m"))
    if equiv_m is not None and equiv_m >= 0.005:
        return True
    orientation_rad = finite_number(first_present(row, "p95_orientation_drift_rad", "orientation_p95_rad"))
    if orientation_rad is not None and orientation_rad >= 0.10:
        return True
    orientation_deg = finite_number(row.get("orientation_p95_deg"))
    return orientation_deg is not None and orientation_deg >= 5.0


def tail_spike_limited(row: dict[str, Any]) -> bool:
    tail = finite_number(row.get("tail_ratio"))
    return tail is not None and tail >= 3.0


def base_physical_blockers(row: dict[str, Any]) -> list[str]:
    if not is_rbpodo_pgmode(row):
        return []
    blockers = [
        "stop_resetFault_unverified",
        "physical_reference_to_actual_error_unmeasured",
        "camera_tcp_calibration_unresolved",
        "no_tiny_physical_acceptance",
    ]
    if diagnostics_suspect_active(row):
        blockers.insert(0, "diagnostics_suspect_unresolved")
    if state_parity_failed(row):
        blockers.insert(0, "state_parity_failed")
    return blockers


def physical_readiness() -> dict[str, Any]:
    return {
        "status": "blocked",
        "blockers": list(PHYSICAL_READINESS_BLOCKERS),
        "next_required_acceptance": list(NEXT_REQUIRED_ACCEPTANCE),
    }


def physical_tracking_result() -> dict[str, str]:
    return {"status": "not_measured"}


def controller_reference_result(row: dict[str, Any], level: str) -> dict[str, str]:
    source = tracking_source(row)
    status = "pass" if source == "tcp_ref_stand" and level == "controller_reference_valid" else "fail"
    return {
        "status": status,
        "explanation": CONTROLLER_REFERENCE_EXPLANATION,
    }


def grade_row(row: dict[str, Any]) -> dict[str, Any]:
    caveats: list[str] = []
    interpretation: list[str] = []
    blockers = base_physical_blockers(row)

    source = tracking_source(row)
    pgmode = is_rbpodo_pgmode(row)
    if not pgmode:
        return {
            "measurement_reliability_level": "not_applicable",
            "reliability_caveats": [],
            "benchmark_interpretation": [],
            "physical_real_blockers": [],
            "reliability_reasons": ["not_rbpodo_pgmode_simulation"],
            "physical_ready_candidate": False,
            "physical_readiness": physical_readiness(),
            "controller_reference_result": {
                "status": "fail",
                "explanation": CONTROLLER_REFERENCE_EXPLANATION,
            },
            "physical_tracking_result": physical_tracking_result(),
        }
    if source == "tcp_ref_stand":
        caveats.append("tcp_ref_lower_bound_only")
        interpretation.append("controller_reference_lower_bound")
    if pgmode:
        interpretation.append("not_physical_tracking")
    if pgmode and (q_ref_ratio_missing_or_low(row) or not state_parity_passed(row)):
        caveats.append("q_ref_not_directly_validated")
    if diagnostics_suspect_active(row):
        caveats.append("diagnostics_suspect_unresolved")
    if diagnostics_override_active(row):
        caveats.append("diagnostics_suspect_override_active")
    elif diagnostics_suspect_active(row):
        caveats.append("diagnostics_suspect_override_active")
    if state_parity_failed(row):
        caveats.append("state_parity_failed")
    if timing_jitter_present(row):
        caveats.append("timing_jitter_spikes")
    if feedback_saturation_present(row):
        caveats.append("feedback_saturation")
        interpretation.append("saturation_limited")
    if orientation_high(row):
        caveats.append("orientation_drift_high")
        interpretation.append("orientation_limited")
    if tail_spike_limited(row) or row.get("error_classification") == "tail_spike_limited":
        interpretation.append("tail_spike_limited")
    if row.get("error_classification") in {"phase_lag_limited", "center_drift_limited", "timing_jitter_limited"}:
        interpretation.append(str(row["error_classification"]))
    if row.get("profile") == "gene_15cm_4s":
        interpretation.append("stress_profile")
        interpretation.append("IL_data_not_recommended")

    unreliable_reasons: list[str] = []
    if not state_available(row):
        unreliable_reasons.append("no_valid_state")
    if row.get("result") == "startup_fault":
        unreliable_reasons.append("startup_fault")
    if blocked_or_cartesian_unavailable(row):
        unreliable_reasons.append("cartesian_unavailable")
    if as_bool(row.get("fault_latched")) is True or row.get("result") == "faulted":
        unreliable_reasons.append("fault_latched")
    if pgmode and as_bool(row.get("physical_motion_detected")) is True:
        unreliable_reasons.append("physical_motion_detected")

    suspect_reasons: list[str] = []
    if q_ref_low_or_missing(row):
        suspect_reasons.append("q_ref_not_directly_validated")
    if diagnostics_suspect_active(row):
        suspect_reasons.append("diagnostics_suspect_unresolved")
    if state_parity_failed(row):
        suspect_reasons.append("state_parity_failed")
    if timing_jitter_present(row):
        suspect_reasons.append("timing_jitter_spikes")
    state_gap = finite_number(row.get("state_gap_count"))
    command_gap = finite_number(row.get("command_gap_count"))
    if state_gap is not None and state_gap > 0.0:
        suspect_reasons.append("state_gaps")
    if command_gap is not None and command_gap > 0.0:
        suspect_reasons.append("command_gaps")

    completed = row.get("result") in {"completed", "pass", None, ""}
    controller_reference_valid = (
        pgmode
        and completed
        and as_bool(row.get("physical_motion_detected")) is not True
        and not diagnostics_suspect_active(row)
        and not state_parity_failed(row)
        and tcp_ref_high(row)
        and q_ref_visible(row)
        and as_bool(row.get("fault_latched")) is not True
    )

    if unreliable_reasons:
        level = "unreliable"
        reasons = unreliable_reasons
        interpretation.append("IL_data_not_recommended")
    elif suspect_reasons:
        level = "suspect"
        reasons = suspect_reasons
        interpretation.append("IL_data_not_recommended")
    elif controller_reference_valid:
        level = "controller_reference_valid"
        reasons = ["controller_reference_lower_bound"]
        if row.get("profile") == "circle_15cm_16s":
            interpretation.append("stable_demo_candidate")
    else:
        level = "suspect" if pgmode else "unreliable"
        reasons = ["insufficient_reliability_evidence"]
        interpretation.append("IL_data_not_recommended")

    if diagnostics_suspect_active(row) and "diagnostics_suspect_unresolved" not in blockers and pgmode:
        blockers.insert(0, "diagnostics_suspect_unresolved")
    if state_parity_failed(row) and "state_parity_failed" not in blockers and pgmode:
        blockers.insert(0, "state_parity_failed")

    return {
        "measurement_reliability_level": level,
        "reliability_caveats": unique(caveats),
        "benchmark_interpretation": unique(interpretation),
        "physical_real_blockers": unique(blockers),
        "reliability_reasons": unique(reasons),
        "physical_ready_candidate": False,
        "physical_readiness": physical_readiness(),
        "controller_reference_result": controller_reference_result(row, level),
        "physical_tracking_result": physical_tracking_result(),
    }


def annotate_row(row: dict[str, Any]) -> dict[str, Any]:
    grade = grade_row(row)
    row["measurement_reliability_level"] = grade["measurement_reliability_level"]
    row["reliability_caveats"] = list_cell(grade["reliability_caveats"])
    row["benchmark_interpretation"] = list_cell(grade["benchmark_interpretation"])
    row["physical_real_blockers"] = list_cell(grade["physical_real_blockers"])
    row["reliability_reasons"] = list_cell(grade["reliability_reasons"])
    row["physical_ready_candidate"] = False
    row["physical_readiness"] = grade["physical_readiness"]
    row["controller_reference_result"] = grade["controller_reference_result"]
    row["physical_tracking_result"] = grade["physical_tracking_result"]
    row["physical_readiness_status"] = grade["physical_readiness"]["status"]
    row["controller_reference_status"] = grade["controller_reference_result"]["status"]
    row["physical_tracking_status"] = grade["physical_tracking_result"]["status"]
    return row


def annotated_copy(row: dict[str, Any]) -> dict[str, Any]:
    return annotate_row(dict(row))


def reliability_json_row(row: dict[str, Any]) -> dict[str, Any]:
    grade = grade_row(row)
    return {
        "run_name": row_name(row),
        "profile": row.get("profile"),
        "tracking_source": tracking_source(row),
        "result": row.get("result"),
        **grade,
    }


def reliability_cell_row(row: dict[str, Any]) -> dict[str, Any]:
    annotated = annotated_copy(row)
    return {
        "run_name": row_name(row),
        "profile": row.get("profile"),
        "tracking_source": tracking_source(row),
        "result": row.get("result"),
        "measurement_reliability_level": annotated.get("measurement_reliability_level"),
        "reliability_caveats": annotated.get("reliability_caveats"),
        "benchmark_interpretation": annotated.get("benchmark_interpretation"),
        "physical_real_blockers": annotated.get("physical_real_blockers"),
    }


def format_cell(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, list):
        return list_cell([str(item) for item in value])
    number = finite_number(value)
    if number is not None:
        return f"{number:.6f}" if abs(number) < 1.0 else f"{number:.3f}"
    return str(value).replace("|", "\\|").replace("\n", " ")


def markdown_table(rows: list[dict[str, Any]]) -> str:
    table_rows = [reliability_cell_row(row) for row in rows]
    lines = [
        "| " + " | ".join(RELIABILITY_COLUMNS) + " |",
        "| " + " | ".join("---" for _ in RELIABILITY_COLUMNS) + " |",
    ]
    for row in table_rows:
        lines.append("| " + " | ".join(format_cell(row.get(key)) for key in RELIABILITY_COLUMNS) + " |")
    return "\n".join(lines)


def report_markdown(rows: list[dict[str, Any]]) -> str:
    return "\n".join(
        [
            "# rbpodo Measurement Reliability Report",
            "",
            f"**{ACKON500_PHYSICAL_WARNING}**",
            "",
            "This report grades measurement reliability before tuning interpretation. "
            "Controller `pgmode` `tcp_ref_stand` evidence is a controller-reference lower bound, not physical TCP tracking.",
            "",
            markdown_table(rows) if rows else "_No rows supplied._",
            "",
            "## Physical Readiness",
            "",
            f"- status: `{physical_readiness()['status']}`",
            "- blockers: " + ", ".join(f"`{item}`" for item in PHYSICAL_READINESS_BLOCKERS),
            "- next_required_acceptance: " + ", ".join(f"`{item}`" for item in NEXT_REQUIRED_ACCEPTANCE),
            f"- controller_reference_result.explanation: {CONTROLLER_REFERENCE_EXPLANATION}",
            "- physical_tracking_result.status: `not_measured`",
            "",
            "Physical-ready candidate status is reserved for future physical real acceptance and is not assigned while diagnostics_suspect remains unresolved.",
        ]
    ) + "\n"


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=RELIABILITY_COLUMNS)
        writer.writeheader()
        for row in rows:
            cell_row = reliability_cell_row(row)
            writer.writerow({key: cell_row.get(key) for key in RELIABILITY_COLUMNS})


def write_json(path: Path, rows: list[dict[str, Any]]) -> None:
    json_rows = [reliability_json_row(row) for row in rows]
    controller_reference_status = (
        "pass"
        if any(row.get("controller_reference_result", {}).get("status") == "pass" for row in json_rows)
        else "fail"
    )
    payload = {
        "schema": SCHEMA,
        "physical_readiness": physical_readiness(),
        "controller_reference_result": {
            "status": controller_reference_status,
            "explanation": CONTROLLER_REFERENCE_EXPLANATION,
        },
        "physical_tracking_result": physical_tracking_result(),
        "rows": json_rows,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_artifacts(output_dir: Path, rows: list[dict[str, Any]]) -> dict[str, str]:
    output_dir.mkdir(parents=True, exist_ok=True)
    md_path = output_dir / OUTPUT_MD
    csv_path = output_dir / OUTPUT_CSV
    json_path = output_dir / OUTPUT_JSON
    md_path.write_text(report_markdown(rows), encoding="utf-8")
    write_csv(csv_path, rows)
    write_json(json_path, rows)
    return {
        "measurement_reliability_report": str(md_path),
        "measurement_reliability_summary_csv": str(csv_path),
        "measurement_reliability_summary_json": str(json_path),
    }


def normalize_summary_row(summary: dict[str, Any]) -> dict[str, Any]:
    preflight = nested_dict(summary, "safety_preflight")
    row = dict(summary)
    row.setdefault("run_name", row_name(summary))
    if "tracking_source" not in row:
        row["tracking_source"] = tracking_source(summary)
    if "benchmark_category" not in row:
        row["benchmark_category"] = benchmark_category(summary)
    if "backend" not in row and preflight.get("backend"):
        row["backend"] = preflight.get("backend")
    if "controller_mode" not in row and benchmark_category(row) == "rbpodo_controller_simulation":
        row["controller_mode"] = "pgmode_simulation"
    return row


def load_summary(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as handle:
        value = json.load(handle)
    if not isinstance(value, dict):
        raise SystemExit(f"{path} does not contain a JSON object")
    value["_summary_path"] = str(path)
    return normalize_summary_row(value)


def load_ablation_rows(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            row.setdefault("benchmark_category", "rbpodo_controller_simulation")
            row.setdefault("backend", "rbpodo")
            row.setdefault("controller_mode", "pgmode_simulation")
            rows.append(row)
    return rows


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate rbpodo pgmode measurement reliability report artifacts.")
    parser.add_argument("summary_json", nargs="*", type=Path)
    parser.add_argument("--ablation-summary-csv", type=Path)
    parser.add_argument("--artifact-dir", type=Path, default=Path("."))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.summary_json and args.ablation_summary_csv is None:
        raise SystemExit("provide at least one summary.json or --ablation-summary-csv")
    rows = [load_summary(path) for path in args.summary_json]
    if args.ablation_summary_csv is not None:
        rows.extend(load_ablation_rows(args.ablation_summary_csv))
    write_artifacts(args.artifact_dir, rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
