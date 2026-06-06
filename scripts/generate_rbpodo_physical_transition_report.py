#!/usr/bin/env python3
"""Generate and validate rbpodo physical transition ladder reports."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any


SCHEMA = "robotics_lab.rbpodo_physical_transition.report.v1"
STAGE_ORDER = (
    ("P0", "controller_sim_repeatability_done"),
    ("P1", "real_readonly_diagnostics_parity"),
    ("P2", "stop_resetFault_or_operator_stop_policy_verified"),
    ("P3", "real_hold_no_motion"),
    ("P4", "tiny_joint_noop_or_tiny_joint_motion"),
    ("P5", "tiny_cartesian_delta"),
    ("P6", "slow_physical_circle_5cm_10s"),
    ("P7", "stable_physical_circle_15cm_16s"),
    ("P8", "medium_physical_circle_15cm_8s"),
    ("P9", "fast_physical_circle_15cm_4s_only_after_explicit_approval"),
)
READINESS_PREREQUISITES = tuple(stage_id for stage_id, _ in STAGE_ORDER[:-1])
PASSLIKE_STATUSES = {"pass", "passed", "completed", "accepted", "preflight_pass"}
SUMMARY_SCHEMAS = {
    "robotics_lab.rbpodo_physical_transition.stage.v1",
    "robotics_lab.rbpodo_physical_transition.external_stage.v1",
}


class ReportError(RuntimeError):
    pass


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate a Markdown/JSON report from rbpodo physical transition "
            "stage artifacts. Physical passes are accepted only from tcp_actual_stand."
        )
    )
    parser.add_argument("--artifact-dir", required=True, type=Path)
    parser.add_argument("--output-md", type=Path, help="Write Markdown report to this path instead of stdout.")
    parser.add_argument("--json", dest="json_path", type=Path, help="Write structured report JSON.")
    parser.add_argument("--allow-invalid", action="store_true", help="Write a report even when validation errors exist.")
    return parser.parse_args(argv)


def load_json(path: Path) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def discover_stage_artifacts(artifact_dir: Path) -> list[dict[str, Any]]:
    if not artifact_dir.is_dir():
        raise ReportError(f"artifact dir not found: {artifact_dir}")
    rows: list[dict[str, Any]] = []
    for path in sorted(artifact_dir.rglob("*.json")):
        data = load_json(path)
        if not data:
            continue
        schema = data.get("schema")
        if schema not in SUMMARY_SCHEMAS and "stage" not in data:
            continue
        stage = normalize_stage(data)
        if stage.get("id"):
            data = dict(data)
            data["_artifact_path"] = str(path)
            rows.append(data)
    return rows


def normalize_stage(summary: dict[str, Any]) -> dict[str, Any]:
    stage = summary.get("stage")
    if isinstance(stage, dict):
        return {
            "id": str(stage.get("id", "")).upper(),
            "name": str(stage.get("ladder_name") or stage.get("name") or stage.get("cli_name") or ""),
        }
    stage_id = summary.get("stage_id")
    if stage_id is None:
        stage_id = summary.get("ladder_stage")
    return {
        "id": str(stage_id or "").upper(),
        "name": str(summary.get("stage_name") or summary.get("ladder_name") or ""),
    }


def result_status(summary: dict[str, Any]) -> str:
    result = summary.get("result")
    if isinstance(result, dict):
        return str(result.get("status") or "unknown")
    return str(summary.get("status") or summary.get("result_status") or "unknown")


def artifact_ref(summary: dict[str, Any]) -> str:
    direct = summary.get("artifact_ref")
    if direct:
        return str(direct)
    return str(summary.get("_artifact_path", ""))


def physical_tracking(summary: dict[str, Any]) -> dict[str, Any]:
    value = summary.get("physical_tracking_result")
    return value if isinstance(value, dict) else {}


def controller_reference(summary: dict[str, Any]) -> dict[str, Any]:
    value = summary.get("controller_reference_result")
    return value if isinstance(value, dict) else {}


def telemetry(summary: dict[str, Any]) -> dict[str, Any]:
    value = summary.get("telemetry_requirements")
    if isinstance(value, dict):
        return value
    value = summary.get("telemetry")
    return value if isinstance(value, dict) else {}


def metric_value(block: dict[str, Any], *keys: str) -> Any:
    for key in keys:
        if key in block and block.get(key) is not None:
            return block.get(key)
    return None


def validate_summary(summary: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    stage = normalize_stage(summary)
    stage_id = stage.get("id") or "<unknown>"
    physical = physical_tracking(summary)
    physical_status = str(physical.get("status") or "not_run")
    physical_source = physical.get("tracking_source") or summary.get("tracking_source")
    if physical and physical_source != "tcp_actual_stand":
        errors.append(
            f"{stage_id}: physical_tracking_result.status={physical_status} requires tracking_source tcp_actual_stand, got {physical_source}"
        )
    if physical_status == "pass" and physical_source != "tcp_actual_stand":
        errors.append(f"{stage_id}: refusing physical pass from non-actual tracking source")
    controller = controller_reference(summary)
    controller_status = str(controller.get("status") or "informational_only")
    controller_source = controller.get("tracking_source")
    if controller and controller_status != "informational_only":
        errors.append(f"{stage_id}: controller_reference_result.status must be informational_only")
    if controller and controller_source != "tcp_ref_stand":
        errors.append(f"{stage_id}: controller_reference_result.tracking_source must be tcp_ref_stand")
    return errors


def latest_by_stage(summaries: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    by_stage: dict[str, dict[str, Any]] = {}
    for summary in summaries:
        stage_id = normalize_stage(summary).get("id")
        if not stage_id:
            continue
        by_stage[stage_id] = summary
    return by_stage


def readiness(by_stage: dict[str, dict[str, Any]], validation_errors: list[str]) -> dict[str, Any]:
    blockers: list[str] = []
    missing = [stage_id for stage_id in READINESS_PREREQUISITES if not artifact_ref(by_stage.get(stage_id, {}))]
    for stage_id in missing:
        blockers.append(f"missing_{stage_id}_artifact")
    for stage_id in READINESS_PREREQUISITES:
        summary = by_stage.get(stage_id)
        if not summary:
            continue
        status = result_status(summary)
        if status not in PASSLIKE_STATUSES:
            blockers.append(f"{stage_id}_status_{status}")
    if validation_errors:
        blockers.append("invalid_physical_tracking_source")
    calibration_blockers = calibration_blockers_from(by_stage)
    blockers.extend(calibration_blockers)
    return {
        "status": "ready_for_p9_approval" if not blockers else "blocked",
        "blockers": blockers,
        "required_prerequisite_artifacts": list(READINESS_PREREQUISITES),
        "next_required_acceptance": [] if not blockers else [
            "read-only diagnostics parity",
            "stop/resetFault or operator-stop policy evidence",
            "real hold no motion",
            "tiny joint acceptance",
            "tiny Cartesian delta using tcp_actual_stand",
            "slow-to-medium physical circle ladder",
        ],
    }


def calibration_blockers_from(by_stage: dict[str, dict[str, Any]]) -> list[str]:
    blockers: list[str] = []
    for summary in by_stage.values():
        calibration = summary.get("calibration")
        if isinstance(calibration, dict) and calibration.get("measured") is False:
            blockers.append("camera_robot_calibration_not_measured")
            break
    return blockers


def stage_row(stage_id: str, name: str, summary: dict[str, Any] | None) -> dict[str, Any]:
    if summary is None:
        return {
            "stage_id": stage_id,
            "stage_name": name,
            "status": "missing",
            "artifact_ref": "",
            "physical_tracking_source": "",
            "controller_reference_source": "",
            "rms_error_m": None,
            "p95_error_m": None,
            "max_error_m": None,
            "state_age_us": None,
            "state_jitter_us": None,
            "q_actual_update_rate_hz": None,
            "q_ref_update_rate_hz": None,
            "fault_latch_status": "missing",
            "cartesian_availability": "missing",
            "stop_reset_behavior_result": "missing",
            "physical_motion_expected": None,
            "physical_motion_detected": None,
            "calibration_status": "",
            "calibration_measured": None,
            "geometry_valid_for_real_policy": None,
        }
    physical = physical_tracking(summary)
    controller = controller_reference(summary)
    tel = telemetry(summary)
    calibration = summary.get("calibration")
    calibration = calibration if isinstance(calibration, dict) else {}
    return {
        "stage_id": stage_id,
        "stage_name": name,
        "status": result_status(summary),
        "artifact_ref": artifact_ref(summary),
        "physical_tracking_source": physical.get("tracking_source"),
        "controller_reference_source": controller.get("tracking_source"),
        "rms_error_m": metric_value(physical, "rms_error_m", "tracking_rms_error_m"),
        "p95_error_m": metric_value(physical, "p95_error_m", "tracking_p95_error_m"),
        "max_error_m": metric_value(physical, "max_error_m", "tracking_max_error_m"),
        "state_age_us": tel.get("state_age_us"),
        "state_jitter_us": tel.get("state_jitter_us"),
        "q_actual_update_rate_hz": tel.get("q_actual_update_rate_hz"),
        "q_ref_update_rate_hz": tel.get("q_ref_update_rate_hz"),
        "fault_latch_status": tel.get("fault_latch_status"),
        "cartesian_availability": tel.get("cartesian_availability"),
        "stop_reset_behavior_result": tel.get("stop_reset_behavior_result"),
        "physical_motion_expected": tel.get("physical_motion_expected"),
        "physical_motion_detected": tel.get("physical_motion_detected"),
        "calibration_status": calibration.get("status"),
        "calibration_measured": calibration.get("measured"),
        "geometry_valid_for_real_policy": calibration.get("geometry_valid_for_real_policy"),
    }


def build_report(artifact_dir: Path) -> dict[str, Any]:
    summaries = discover_stage_artifacts(artifact_dir)
    validation_errors = [error for summary in summaries for error in validate_summary(summary)]
    by_stage = latest_by_stage(summaries)
    rows = [stage_row(stage_id, name, by_stage.get(stage_id)) for stage_id, name in STAGE_ORDER]
    return {
        "schema": SCHEMA,
        "artifact_dir": str(artifact_dir),
        "physical_readiness": readiness(by_stage, validation_errors),
        "validation_errors": validation_errors,
        "stage_rows": rows,
        "artifact_schema": {
            "physical_tracking_result": {
                "status": "pass|fail|not_run",
                "tracking_source": "tcp_actual_stand",
                "rms_error_m": "number|null",
                "p95_error_m": "number|null",
                "max_error_m": "number|null",
            },
            "controller_reference_result": {
                "status": "informational_only",
                "tracking_source": "tcp_ref_stand",
            },
            "telemetry_requirements": [
                "state_age_us",
                "state_jitter_us",
                "q_actual_update_rate_hz",
                "q_ref_update_rate_hz",
                "fault_latch_status",
                "cartesian_availability",
                "stop_reset_behavior_result",
                "physical_motion_expected",
                "physical_motion_detected",
                "calibration.status",
                "calibration.measured",
                "calibration.geometry_valid_for_real_policy",
            ],
        },
    }


def format_cell(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, dict):
        return json.dumps(value, sort_keys=True)
    return str(value)


def markdown(report: dict[str, Any]) -> str:
    readiness = report["physical_readiness"]
    lines = [
        "# rbpodo Physical Transition Report",
        "",
        f"Artifact dir: `{report['artifact_dir']}`",
        "",
        "## Readiness",
        "",
        f"- status: `{readiness['status']}`",
        f"- blockers: `{', '.join(readiness['blockers']) if readiness['blockers'] else 'none'}`",
        "",
        "## Ladder",
        "",
        "| Stage | Status | Artifact | Physical source | Controller source | RMS m | P95 m | Max m |",
        "| --- | --- | --- | --- | --- | ---: | ---: | ---: |",
    ]
    for row in report["stage_rows"]:
        lines.append(
            "| "
            + " | ".join(
                format_cell(row.get(key))
                for key in (
                    "stage_id",
                    "status",
                    "artifact_ref",
                    "physical_tracking_source",
                    "controller_reference_source",
                    "rms_error_m",
                    "p95_error_m",
                    "max_error_m",
                )
            )
            + " |"
        )
    lines.extend(
        [
            "",
            "## Calibration",
            "",
            "| Stage | Status | Measured | Geometry valid for real policy |",
            "| --- | --- | --- | --- |",
        ]
    )
    for row in report["stage_rows"]:
        lines.append(
            "| "
            + " | ".join(
                format_cell(row.get(key))
                for key in (
                    "stage_id",
                    "calibration_status",
                    "calibration_measured",
                    "geometry_valid_for_real_policy",
                )
            )
            + " |"
        )
    lines.extend(
        [
            "",
            "## Source Semantics",
            "",
            "- `physical_tracking_result.status=pass` is valid only with `tracking_source=tcp_actual_stand`.",
            "- `controller_reference_result` is informational only and uses `tcp_ref_stand`.",
            "- Controller-simulation best parameters can be listed as a seed record, not as promoted physical defaults.",
        ]
    )
    if report["validation_errors"]:
        lines.extend(["", "## Validation Errors", ""])
        lines.extend(f"- {error}" for error in report["validation_errors"])
    return "\n".join(lines) + "\n"


def main(argv: list[str] | None = None) -> int:
    try:
        args = parse_args(argv)
        report = build_report(args.artifact_dir)
        text = markdown(report)
        if args.output_md:
            args.output_md.parent.mkdir(parents=True, exist_ok=True)
            args.output_md.write_text(text, encoding="utf-8")
        else:
            print(text, end="")
        if args.json_path:
            args.json_path.parent.mkdir(parents=True, exist_ok=True)
            args.json_path.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
        if report["validation_errors"] and not args.allow_invalid:
            for error in report["validation_errors"]:
                print(f"error: {error}", file=sys.stderr)
            return 2
        return 0
    except ReportError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
