#!/usr/bin/env python3
"""Generate an rbpodo diagnostics-suspect root-cause report.

The report is offline/read-only. It consumes existing diagnostic artifacts and
does not connect to controllers, send motion commands, set pgmode, reset
faults, or change robot/controller state.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import generate_rbpodo_measurement_reliability_report as reliability_report
import rbpodo_state_dump


SCHEMA = "robotics_lab.rbpodo_diagnostics_rootcause_report.v1"
ROOT_CAUSES = {
    "sdk_firmware_layout_mismatch",
    "field_semantics_unknown",
    "controller_reports_real_fault",
    "python_cpp_decode_mismatch",
    "payload_unstable",
    "insufficient_evidence",
}
PHYSICAL_REAL_BLOCKERS = [
    "diagnostics_suspect_unresolved",
    "stop_resetFault_unverified",
    reliability_report.UNMEASURED_PHYSICAL_BLOCKER,
]
RAW_FIELDS = rbpodo_state_dump.DIAGNOSTIC_FIELDS
BOOLEAN_FAULT_FIELDS = (
    "op_stat_soft_estop_occur",
    "op_stat_collision_occur",
    "op_stat_self_collision",
)
CONTROLLER_FAULT_FIELDS = (
    "init_error",
    "op_stat_sos_flag",
    "op_stat_ems_flag",
    *BOOLEAN_FAULT_FIELDS,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate an offline rbpodo diagnostics_suspect root-cause report "
            "from state-dump, Python/C++ parity, and raw data-port capture "
            "artifacts."
        )
    )
    parser.add_argument("--state-dump", type=Path, help="rbpodo_state_dump.py JSON artifact.")
    parser.add_argument("--parity-summary", type=Path, help="rbpodo_state_parity_check.py summary.json.")
    parser.add_argument("--raw-capture", type=Path, help="rainbow_data_port_capture.py summary.json.")
    parser.add_argument("--physical-stage-summary", type=Path, help="Optional rbpodo_physical_stage_measure.py summary.json.")
    parser.add_argument("--output-md", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    return parser.parse_args()


def finite_number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def integer(value: Any) -> int | None:
    if isinstance(value, bool):
        return int(value)
    try:
        number = int(value)
    except (TypeError, ValueError):
        return None
    return number


def as_bool(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"1", "true", "yes"}:
            return True
        if lowered in {"0", "false", "no"}:
            return False
    return None


def unique(values: list[str]) -> list[str]:
    return list(dict.fromkeys(value for value in values if value))


def nested_dict(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def load_json_artifact(path: Path | None, label: str) -> tuple[dict[str, Any] | None, dict[str, Any]]:
    status: dict[str, Any] = {
        "label": label,
        "path": str(path) if path is not None else None,
        "present": False,
        "load_error": None,
    }
    if path is None:
        status["load_error"] = f"{label} not provided"
        return None, status
    try:
        with path.open(encoding="utf-8") as handle:
            loaded = json.load(handle)
    except OSError as exc:
        status["load_error"] = f"{type(exc).__name__}: {exc}"
        return None, status
    except json.JSONDecodeError as exc:
        status["load_error"] = f"JSONDecodeError: {exc}"
        return None, status
    if not isinstance(loaded, dict):
        status["load_error"] = f"{label} does not contain a JSON object"
        return None, status
    status["present"] = True
    return loaded, status


def state_results(state_dump: dict[str, Any] | None) -> list[dict[str, Any]]:
    if not isinstance(state_dump, dict):
        return []
    results = state_dump.get("results")
    return [item for item in results if isinstance(item, dict)] if isinstance(results, list) else []


def controller_ips(state_dump: dict[str, Any] | None, raw_capture: dict[str, Any] | None) -> list[str]:
    ips: list[str] = []
    if isinstance(state_dump, dict) and isinstance(state_dump.get("ips"), list):
        ips.extend(str(ip) for ip in state_dump.get("ips") if str(ip))
    for item in state_results(state_dump):
        if item.get("ip") is not None:
            ips.append(str(item.get("ip")))
    if isinstance(raw_capture, dict) and isinstance(raw_capture.get("ips"), list):
        ips.extend(str(ip) for ip in raw_capture.get("ips") if str(ip))
    return unique(ips)


def raw_diagnostic_rows(state_dump: dict[str, Any] | None) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for item in state_results(state_dump):
        raw = nested_dict(item.get("raw"))
        rows.append({
            "ip": item.get("ip"),
            "ok": bool(item.get("ok", False)),
            "controller_mode": item.get("controller_mode"),
            "controller_mode_is_simulation": item.get("controller_mode_is_simulation"),
            "controller_mode_warning": item.get("controller_mode_warning"),
            "diagnostics_suspect": bool(item.get("diagnostics_suspect", False)),
            "diagnostics_suspect_reasons": list(item.get("diagnostics_suspect_reasons") or []),
            "clear_error_flags": list(item.get("clear_error_flags") or []),
            "raw": {field: raw.get(field) for field in RAW_FIELDS},
        })
    return rows


def operation_mode_evidence(rows: list[dict[str, Any]]) -> dict[str, Any]:
    simulated = [
        row.get("controller_mode_is_simulation")
        for row in rows
        if row.get("controller_mode_is_simulation") is not None
    ]
    warnings = [
        str(row.get("controller_mode_warning"))
        for row in rows
        if row.get("controller_mode_warning")
    ]
    modes = {
        str(row.get("ip")): {
            "real_vs_simulation_mode": row.get("raw", {}).get("real_vs_simulation_mode"),
            "controller_mode": row.get("controller_mode"),
            "controller_mode_is_simulation": row.get("controller_mode_is_simulation"),
            "controller_mode_warning": row.get("controller_mode_warning"),
        }
        for row in rows
    }
    return {
        "pgmode_simulation_confirmed_by_state_dump": bool(simulated) and all(bool(value) for value in simulated),
        "controller_mode_warnings": warnings,
        "per_controller": modes,
    }


def collect_suspect_reasons(rows: list[dict[str, Any]], parity_summary: dict[str, Any] | None) -> list[str]:
    reasons: list[str] = []
    for row in rows:
        for reason in row.get("diagnostics_suspect_reasons") or []:
            reasons.append(f"{row.get('ip')}: {reason}")
        if row.get("diagnostics_suspect") and not row.get("diagnostics_suspect_reasons"):
            reasons.append(f"{row.get('ip')}: diagnostics_suspect true without detailed reason")
    if isinstance(parity_summary, dict):
        result = str(parity_summary.get("result") or "")
        if result == "suspect_but_consistent":
            reasons.append("parity: Python and C++ agree, but diagnostics_suspect remains unresolved")
        for caveat in parity_summary.get("caveats") or []:
            if "diagnostics_suspect" in str(caveat):
                reasons.append(f"parity caveat: {caveat}")
    return unique(reasons)


def clear_controller_faults(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    faults: list[dict[str, Any]] = []
    for row in rows:
        raw = nested_dict(row.get("raw"))
        clear_flags = set(str(flag) for flag in row.get("clear_error_flags") or [])
        for field in CONTROLLER_FAULT_FIELDS:
            value = integer(raw.get(field))
            if field in BOOLEAN_FAULT_FIELDS and value == 1:
                faults.append({"ip": row.get("ip"), "field": field, "value": value})
            elif field in clear_flags:
                faults.append({"ip": row.get("ip"), "field": field, "value": raw.get(field)})
            elif field == "init_error" and value not in {None, 0} and abs(value) < 1_000_000:
                faults.append({"ip": row.get("ip"), "field": field, "value": value})
    return faults


def suspect_has_layout_smell(rows: list[dict[str, Any]]) -> bool:
    for row in rows:
        raw = nested_dict(row.get("raw"))
        for field in RAW_FIELDS:
            value = integer(raw.get(field))
            if value is not None and abs(value) >= 1_000_000:
                return True
        for reason in row.get("diagnostics_suspect_reasons") or []:
            text = str(reason)
            if "expected 0/1" in text or "huge value" in text or "implausible" in text:
                return True
    return False


def parity_evidence(parity_summary: dict[str, Any] | None) -> dict[str, Any]:
    metrics = nested_dict(parity_summary.get("metrics")) if isinstance(parity_summary, dict) else {}
    result = str(parity_summary.get("result") or "") if isinstance(parity_summary, dict) else ""
    raw_match_rate = finite_number(metrics.get("raw_field_match_rate"))
    suspect_agreement_rate = finite_number(metrics.get("diagnostics_suspect_agreement_rate"))
    mismatch = result == "failed_parity_mismatch"
    if raw_match_rate is not None and raw_match_rate < 1.0:
        mismatch = True
    if suspect_agreement_rate is not None and suspect_agreement_rate < 1.0:
        mismatch = True
    agreement = result in {"passed", "suspect_but_consistent"} and not mismatch
    return {
        "present": isinstance(parity_summary, dict),
        "result": result or None,
        "reason": parity_summary.get("reason") if isinstance(parity_summary, dict) else None,
        "caveats": parity_summary.get("caveats", []) if isinstance(parity_summary, dict) else [],
        "metrics": metrics,
        "python_cpp_decode_mismatch": mismatch,
        "python_cpp_agree": agreement,
        "cpp_backend_hints": nested_dict(parity_summary.get("cpp_backend_hints")) if isinstance(parity_summary, dict) else {},
    }


def raw_capture_evidence(raw_capture: dict[str, Any] | None) -> dict[str, Any]:
    fixture = nested_dict(raw_capture.get("fixture_comparison")) if isinstance(raw_capture, dict) else {}
    lengths = raw_capture.get("unique_payload_lengths") if isinstance(raw_capture, dict) else []
    lengths = lengths if isinstance(lengths, list) else []
    success_count = int(finite_number(raw_capture.get("success_count")) or 0) if isinstance(raw_capture, dict) else 0
    unique_hash_count = int(finite_number(raw_capture.get("unique_hash_count")) or 0) if isinstance(raw_capture, dict) else 0
    q_ref_changed = as_bool(fixture.get("q_ref_change_observed"))
    payload_changed = as_bool(fixture.get("payload_change_observed"))
    payload_changes_when_q_ref_changes = as_bool(fixture.get("payload_changes_when_q_ref_changes"))
    unstable_reasons: list[str] = []
    if len(lengths) > 1:
        unstable_reasons.append("raw_payload_length_varied")
    if q_ref_changed is True and payload_changes_when_q_ref_changes is False:
        unstable_reasons.append("q_ref_changed_without_payload_hash_change")
    return {
        "present": isinstance(raw_capture, dict),
        "result": raw_capture.get("result") if isinstance(raw_capture, dict) else None,
        "reason": raw_capture.get("reason") if isinstance(raw_capture, dict) else None,
        "success_count": success_count,
        "sample_count": raw_capture.get("sample_count") if isinstance(raw_capture, dict) else None,
        "unique_payload_lengths": lengths,
        "unique_hash_count": unique_hash_count,
        "stable_prefix_bytes_len": raw_capture.get("stable_prefix_bytes_len") if isinstance(raw_capture, dict) else None,
        "stable_prefix_hex": raw_capture.get("stable_prefix_hex") if isinstance(raw_capture, dict) else None,
        "payload_change_observed": payload_changed,
        "q_ref_unique_count": fixture.get("q_ref_unique_count"),
        "q_ref_change_observed": q_ref_changed,
        "q_ref_payload_pair_count": fixture.get("q_ref_payload_pair_count"),
        "payload_changes_when_q_ref_changes": payload_changes_when_q_ref_changes,
        "payload_unstable": bool(unstable_reasons),
        "payload_unstable_reasons": unstable_reasons,
    }


def classify_root_cause(
    rows: list[dict[str, Any]],
    parity: dict[str, Any],
    raw_capture: dict[str, Any],
    input_statuses: dict[str, dict[str, Any]],
) -> tuple[str, list[str]]:
    faults = clear_controller_faults(rows)
    if faults:
        return "controller_reports_real_fault", [
            "At least one raw controller fault flag is a clear fault value.",
            "Do not reinterpret this as an SDK layout issue until the controller fault is cleared or separately explained.",
        ]

    if parity["python_cpp_decode_mismatch"]:
        return "python_cpp_decode_mismatch", [
            "Python rbpodo and C++ rb_servo_server samples do not agree for the sampled fields.",
            "Fix parity before assigning controller or firmware semantics.",
        ]

    missing = [
        status["label"]
        for status in input_statuses.values()
        if not status.get("present") and not status.get("optional")
    ]
    if missing:
        return "insufficient_evidence", [
            "Missing required evidence: " + ", ".join(missing),
            "Collect state dump, parity summary, and raw data-port capture before accepting a root cause.",
        ]

    if not raw_capture["present"] or raw_capture["success_count"] <= 0:
        return "insufficient_evidence", [
            "Raw data-port capture has no successful payload samples.",
            "Capture raw 5001 payloads with --also-rbpodo-python before resolving diagnostics_suspect.",
        ]

    if raw_capture["payload_unstable"]:
        return "payload_unstable", list(raw_capture["payload_unstable_reasons"])

    any_suspect = any(bool(row.get("diagnostics_suspect")) for row in rows)
    if any_suspect and parity["python_cpp_agree"]:
        if suspect_has_layout_smell(rows):
            return "sdk_firmware_layout_mismatch", [
                "Python and C++ agree on suspicious raw values.",
                "The suspicious values look like field-layout or ABI/firmware interpretation evidence.",
            ]
        return "field_semantics_unknown", [
            "Python and C++ agree, but the meaning of one or more raw diagnostic fields is not verified.",
        ]

    if any_suspect:
        return "field_semantics_unknown", [
            "diagnostics_suspect is present, but the available evidence does not isolate layout, semantics, parity, or payload stability.",
        ]

    return "insufficient_evidence", [
        "The supplied artifacts do not show an unresolved diagnostics_suspect condition or a verified healthy root-cause closure.",
    ]


def root_cause_checklist(root_cause: str, report: dict[str, Any]) -> list[dict[str, Any]]:
    input_statuses = report["input_statuses"]
    missing = [
        status["label"]
        for status in input_statuses.values()
        if not status.get("present") and not status.get("optional")
    ]
    checklist = [
        {
            "item": "State dump captured with raw rbpodo diagnostic fields",
            "status": "blocked" if "state_dump" in missing else "done",
            "evidence": input_statuses["state_dump"].get("path") or input_statuses["state_dump"].get("load_error"),
        },
        {
            "item": "Python/C++ parity summary captured",
            "status": "blocked" if "parity_summary" in missing else "done",
            "evidence": report["python_cpp_parity"].get("result"),
        },
        {
            "item": "Raw 5001 data-port capture captured",
            "status": "blocked" if "raw_capture" in missing else "done",
            "evidence": report["raw_capture_payload"].get("result"),
        },
        {
            "item": "Physical-real blockers remain active",
            "status": "blocked",
            "evidence": ", ".join(report["physical_real_blockers"]),
        },
    ]
    if root_cause == "controller_reports_real_fault":
        checklist.append({
            "item": "Resolve or safety-review controller fault before any motion-capable acceptance",
            "status": "blocked",
            "evidence": json.dumps(report["clear_controller_faults"], sort_keys=True),
        })
    elif root_cause == "python_cpp_decode_mismatch":
        checklist.append({
            "item": "Inspect parity mismatches and C++ raw field mapping before interpreting diagnostics",
            "status": "blocked",
            "evidence": report["python_cpp_parity"].get("reason"),
        })
    elif root_cause == "sdk_firmware_layout_mismatch":
        checklist.append({
            "item": "Verify rbpodo SDK/firmware field layout against vendor docs or a controlled fixture",
            "status": "blocked",
            "evidence": "; ".join(report["suspect_reasons"]),
        })
    elif root_cause == "payload_unstable":
        checklist.append({
            "item": "Repeat raw capture and explain payload instability before mapping fields",
            "status": "blocked",
            "evidence": "; ".join(report["raw_capture_payload"].get("payload_unstable_reasons") or []),
        })
    else:
        checklist.append({
            "item": "Collect missing or inconclusive evidence and rerun this report",
            "status": "blocked",
            "evidence": "; ".join(report["root_cause_reasons"]),
        })
    return checklist


def build_report(
    state_dump: dict[str, Any] | None,
    parity_summary: dict[str, Any] | None,
    raw_capture: dict[str, Any] | None,
    input_statuses: dict[str, dict[str, Any]] | None = None,
    physical_stage_summary: dict[str, Any] | None = None,
) -> dict[str, Any]:
    if input_statuses is None:
        input_statuses = {
            "state_dump": {"label": "state_dump", "path": None, "present": state_dump is not None, "load_error": None},
            "parity_summary": {"label": "parity_summary", "path": None, "present": parity_summary is not None, "load_error": None},
            "raw_capture": {"label": "raw_capture", "path": None, "present": raw_capture is not None, "load_error": None},
            "physical_stage_summary": {
                "label": "physical_stage_summary",
                "path": None,
                "present": physical_stage_summary is not None,
                "load_error": None,
                "optional": True,
            },
        }
    else:
        input_statuses = dict(input_statuses)
        input_statuses.setdefault(
            "physical_stage_summary",
            {
                "label": "physical_stage_summary",
                "path": None,
                "present": physical_stage_summary is not None,
                "load_error": None,
                "optional": True,
            },
        )

    rows = raw_diagnostic_rows(state_dump)
    parity = parity_evidence(parity_summary)
    raw_evidence = raw_capture_evidence(raw_capture)
    root_cause, root_reasons = classify_root_cause(rows, parity, raw_evidence, input_statuses)
    assert root_cause in ROOT_CAUSES
    report = {
        "schema": SCHEMA,
        "read_only": True,
        "safety_note": (
            "Offline report only. It does not send motion, set pgmode, reset faults, "
            "or mark diagnostics_suspect healthy."
        ),
        "input_statuses": input_statuses,
        "sdk_info": nested_dict(state_dump.get("sdk_info")) if isinstance(state_dump, dict) else {},
        "cpp_backend_hints": parity.get("cpp_backend_hints", {}),
        "controller_ips": controller_ips(state_dump, raw_capture),
        "operation_mode_evidence": operation_mode_evidence(rows),
        "raw_diagnostic_fields": rows,
        "suspect_reasons": collect_suspect_reasons(rows, parity_summary),
        "clear_controller_faults": clear_controller_faults(rows),
        "python_cpp_parity": parity,
        "raw_capture_payload": raw_evidence,
        "q_ref_vs_payload": {
            "q_ref_change_observed": raw_evidence.get("q_ref_change_observed"),
            "q_ref_unique_count": raw_evidence.get("q_ref_unique_count"),
            "q_ref_payload_pair_count": raw_evidence.get("q_ref_payload_pair_count"),
            "payload_change_observed": raw_evidence.get("payload_change_observed"),
            "payload_changes_when_q_ref_changes": raw_evidence.get("payload_changes_when_q_ref_changes"),
        },
        "likely_root_cause": root_cause,
        "root_cause_reasons": root_reasons,
        "physical_real_blockers": reliability_report.physical_blockers_with_measurement(
            PHYSICAL_REAL_BLOCKERS,
            physical_stage_summary,
        ),
        "physical_tracking_result": reliability_report.physical_tracking_result(physical_stage_summary),
        "physical_ready_candidate": False,
    }
    report["root_cause_checklist"] = root_cause_checklist(root_cause, report)
    return report


def format_value(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, bool):
        return "true" if value else "false"
    number = finite_number(value)
    if number is not None:
        return f"{number:.6g}"
    if isinstance(value, list):
        return ", ".join(format_value(item) for item in value)
    text = str(value)
    return text.replace("|", "\\|").replace("\n", " ")


def markdown_table(headers: list[str], rows: list[list[Any]]) -> str:
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(format_value(cell) for cell in row) + " |")
    return "\n".join(lines)


def report_markdown(report: dict[str, Any]) -> str:
    sdk = nested_dict(report.get("sdk_info"))
    cpp = nested_dict(report.get("cpp_backend_hints"))
    parity = nested_dict(report.get("python_cpp_parity"))
    raw = nested_dict(report.get("raw_capture_payload"))
    q_ref = nested_dict(report.get("q_ref_vs_payload"))

    raw_rows = []
    for row in report.get("raw_diagnostic_fields") or []:
        raw_fields = nested_dict(row.get("raw"))
        raw_rows.append([
            row.get("ip"),
            row.get("controller_mode"),
            row.get("diagnostics_suspect"),
            raw_fields.get("time"),
            raw_fields.get("real_vs_simulation_mode"),
            raw_fields.get("init_state_info"),
            raw_fields.get("init_error"),
            raw_fields.get("op_stat_sos_flag"),
            raw_fields.get("op_stat_ems_flag"),
            raw_fields.get("op_stat_soft_estop_occur"),
            raw_fields.get("op_stat_collision_occur"),
            raw_fields.get("op_stat_self_collision"),
        ])

    checklist_rows = [
        [item.get("status"), item.get("item"), item.get("evidence")]
        for item in report.get("root_cause_checklist") or []
    ]

    return "\n".join([
        "# rbpodo Diagnostics Root-Cause Report",
        "",
        report["safety_note"],
        "",
        "## Likely Root Cause",
        "",
        f"`{report['likely_root_cause']}`",
        "",
        *[f"- {reason}" for reason in report.get("root_cause_reasons") or []],
        "",
        "## Physical-Real Blockers",
        "",
        *[f"- `{blocker}`" for blocker in report.get("physical_real_blockers") or []],
        "",
        "## SDK / Python Module",
        "",
        f"- `module_available`: {format_value(sdk.get('module_available'))}",
        f"- `module_file`: {format_value(sdk.get('module_file'))}",
        f"- `module_version`: {format_value(sdk.get('module_version'))}",
        f"- `CobotData`: {format_value(sdk.get('cobot_data_available'))}",
        "",
        "## C++ Backend Hints",
        "",
        f"- `sample_count`: {format_value(cpp.get('sample_count'))}",
        f"- `rbpodo_sdk_state_sources`: {format_value(cpp.get('rbpodo_sdk_state_sources'))}",
        f"- `rbpodo_state_decode_policies`: {format_value(cpp.get('rbpodo_state_decode_policies'))}",
        f"- `q_ref_sources`: {format_value(cpp.get('q_ref_sources'))}",
        "",
        "## Controller / pgmode Evidence",
        "",
        f"- controller IPs: {format_value(report.get('controller_ips'))}",
        f"- pgmode simulation confirmed by state dump: "
        f"{format_value(nested_dict(report.get('operation_mode_evidence')).get('pgmode_simulation_confirmed_by_state_dump'))}",
        "",
        "## Raw Diagnostic Fields",
        "",
        markdown_table([
            "ip",
            "mode",
            "suspect",
            "time",
            "real_vs_simulation_mode",
            "init_state_info",
            "init_error",
            "op_stat_sos_flag",
            "op_stat_ems_flag",
            "op_stat_soft_estop_occur",
            "op_stat_collision_occur",
            "op_stat_self_collision",
        ], raw_rows) if raw_rows else "_No state-dump raw diagnostic rows supplied._",
        "",
        "## Suspect Reasons",
        "",
        "\n".join(f"- {reason}" for reason in report.get("suspect_reasons") or []) or "- none recorded",
        "",
        "## Python/C++ Parity",
        "",
        f"- result: `{format_value(parity.get('result'))}`",
        f"- reason: {format_value(parity.get('reason'))}",
        f"- raw field match rate: {format_value(nested_dict(parity.get('metrics')).get('raw_field_match_rate'))}",
        f"- diagnostics suspect agreement rate: "
        f"{format_value(nested_dict(parity.get('metrics')).get('diagnostics_suspect_agreement_rate'))}",
        "",
        "## Raw Payload Stability",
        "",
        f"- result: `{format_value(raw.get('result'))}`",
        f"- success_count: {format_value(raw.get('success_count'))}",
        f"- unique_payload_lengths: {format_value(raw.get('unique_payload_lengths'))}",
        f"- unique_hash_count: {format_value(raw.get('unique_hash_count'))}",
        f"- stable_prefix_bytes_len: {format_value(raw.get('stable_prefix_bytes_len'))}",
        "",
        "## q_ref vs Payload Changes",
        "",
        f"- q_ref_change_observed: {format_value(q_ref.get('q_ref_change_observed'))}",
        f"- q_ref_unique_count: {format_value(q_ref.get('q_ref_unique_count'))}",
        f"- q_ref_payload_pair_count: {format_value(q_ref.get('q_ref_payload_pair_count'))}",
        f"- payload_change_observed: {format_value(q_ref.get('payload_change_observed'))}",
        f"- payload_changes_when_q_ref_changes: {format_value(q_ref.get('payload_changes_when_q_ref_changes'))}",
        "",
        "## Root-Cause Checklist",
        "",
        markdown_table(["status", "item", "evidence"], checklist_rows),
        "",
        "The diagnostics_suspect override remains controller-simulation-only. Physical real is blocked until this root cause is resolved or explicitly accepted by a separate safety review.",
        "",
    ]) + "\n"


def write_outputs(report: dict[str, Any], output_md: Path, output_json: Path) -> None:
    output_md.parent.mkdir(parents=True, exist_ok=True)
    output_json.parent.mkdir(parents=True, exist_ok=True)
    output_md.write_text(report_markdown(report), encoding="utf-8")
    output_json.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    state_dump, state_status = load_json_artifact(args.state_dump, "state_dump")
    parity_summary, parity_status = load_json_artifact(args.parity_summary, "parity_summary")
    raw_capture, raw_status = load_json_artifact(args.raw_capture, "raw_capture")
    physical_stage, physical_stage_status = load_json_artifact(args.physical_stage_summary, "physical_stage_summary")
    physical_stage_status["optional"] = True
    statuses = {
        "state_dump": state_status,
        "parity_summary": parity_status,
        "raw_capture": raw_status,
        "physical_stage_summary": physical_stage_status,
    }
    report = build_report(state_dump, parity_summary, raw_capture, statuses, physical_stage)
    write_outputs(report, args.output_md, args.output_json)
    print(f"generate_rbpodo_diagnostics_report: {report['likely_root_cause']}")
    print(f"markdown: {args.output_md}")
    print(f"json: {args.output_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
