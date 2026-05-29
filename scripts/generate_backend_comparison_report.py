#!/usr/bin/env python3
"""Generate a decision-oriented rbpodo vs rbscript_tcp comparison report."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any

import compare_backend_ablation as compare_backend


BACKENDS = ("rbpodo", "rbscript_tcp")
METRIC_GROUPS = ("connect_only", "read_state", "command_ack_no_motion", "servo_j_noop")
READ_STATE_STABLE_MAX_HZ = 100.0
READ_STATE_MIN_SUCCESS_RATE = 0.99
ACHIEVED_RATE_MIN_RATIO = 0.95
ACK_ISSUE_THRESHOLD = 0

EVIDENCE_COLUMNS = [
    "metric_group",
    "backend",
    "classification",
    "status",
    "artifact_path",
    "requested_rate_hz",
    "achieved_rate_hz",
    "success_rate",
    "timeout_count",
    "error_count",
    "unrecognized_response_count",
    "read_state_capability",
    "comparable",
    "reason",
    "note",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate Markdown/CSV/JSON decision evidence from backend comparison "
            "summary.json artifacts. Missing paths are recorded as not-yet-run evidence."
        )
    )
    parser.add_argument("summary_json", nargs="*", type=Path)
    parser.add_argument("--output-md", type=Path)
    parser.add_argument("--output-csv", type=Path)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument(
        "--require-default-missing-rows",
        action="store_true",
        help="Add not-yet-run rows for backend/metric groups with no supplied evidence.",
    )
    return parser.parse_args()


def finite_number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        number = float(value)
        return number if math.isfinite(number) else None
    return None


def fmt(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.3f}" if math.isfinite(value) else ""
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def load_json(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as handle:
        value = json.load(handle)
    if not isinstance(value, dict):
        raise SystemExit(f"{path} does not contain a JSON object")
    return value


def infer_backend(summary: dict[str, Any], path: Path | None = None) -> str:
    backend = compare_backend.infer_backend(summary)
    if backend:
        return str(backend)
    text = str(path or summary.get("artifact_dir", "")).lower()
    if "rbscript" in text:
        return "rbscript_tcp"
    if "rbpodo" in text:
        return "rbpodo"
    return ""


def normalize_metric_group(mode: Any) -> str:
    text = str(mode or "")
    if text == "ack_no_motion":
        return "command_ack_no_motion"
    if text == "read_only":
        return "read_state"
    if text == "servo_j_noop_controller_simulation":
        return "servo_j_noop"
    if text in METRIC_GROUPS:
        return text
    return text


def artifact_path(summary: dict[str, Any], path: Path | None) -> str:
    if path is not None:
        return str(path)
    artifact_dir = summary.get("artifact_dir")
    return str(artifact_dir) if artifact_dir is not None else ""


def nested_count(mapping: dict[str, Any], key: str) -> int:
    value = mapping.get(key)
    if isinstance(value, bool):
        return 0
    if isinstance(value, (int, float)) and math.isfinite(float(value)):
        return int(value)
    return 0


def unrecognized_response_count(item: dict[str, Any], summary: dict[str, Any]) -> int:
    total = nested_count(item, "unrecognized_response_count")
    total += nested_count(summary, "unrecognized_response_count")
    errors = item.get("other_error_counts")
    if isinstance(errors, dict):
        total += nested_count(errors, "unrecognized_response")
    summary_errors = summary.get("other_error_counts")
    if isinstance(summary_errors, dict):
        total += nested_count(summary_errors, "unrecognized_response")
    return total


def item_error_count(item: dict[str, Any], summary: dict[str, Any]) -> int:
    if item.get("error_count") is not None:
        return nested_count(item, "error_count")
    send_count = int(item.get("send_count", 0) or 0)
    success_rate = finite_number(item.get("success_rate"))
    timeout = nested_count(item, "ack_timeout_count") + nested_count(item, "data_timeout_count")
    if send_count > 0 and success_rate is not None:
        return max(send_count - int(send_count * success_rate) - timeout, 0)
    if summary.get("send_count") is not None:
        return int(summary.get("send_failure_count", 0) or 0)
    sample_count = int(summary.get("sample_count", 0) or 0)
    success_count = int(summary.get("success_count", 0) or 0)
    return max(sample_count - success_count, 0)


def base_row(summary: dict[str, Any], path: Path | None, item: dict[str, Any] | None = None) -> dict[str, Any]:
    item = item or {}
    backend = str(item.get("backend") or infer_backend(summary, path))
    metric_group = normalize_metric_group(item.get("mode") or summary.get("mode"))
    metadata = compare_backend.read_state_metadata(summary, item)
    timeout_count = (
        nested_count(item, "timeout_count")
        + nested_count(item, "ack_timeout_count")
        + nested_count(item, "data_timeout_count")
        + nested_count(summary, "command_timeout_count")
        + nested_count(summary, "data_port_timeout_count")
    )
    row = {
        "metric_group": metric_group,
        "backend": backend,
        "classification": "",
        "status": str(summary.get("result") or "measured"),
        "artifact_path": artifact_path(summary, path),
        "requested_rate_hz": item.get("requested_rate_hz", compare_backend.requested_rate(summary)),
        "achieved_rate_hz": item.get("achieved_rate_hz", summary.get("achieved_rate_hz")),
        "success_rate": item.get("success_rate", compare_backend.success_rate(summary)),
        "timeout_count": timeout_count,
        "error_count": item_error_count(item, summary),
        "unrecognized_response_count": unrecognized_response_count(item, summary),
        "read_state_capability": metadata.get("read_state_capability", ""),
        "comparable": metadata.get("comparable"),
        "reason": metadata.get("not_comparable_reason") or str(summary.get("result_reason") or ""),
        "note": "",
    }
    row["classification"], row["note"] = classify_row(row, summary)
    return row


def rows_from_summary(summary: dict[str, Any], path: Path | None) -> list[dict[str, Any]]:
    rate_results = summary.get("rate_results")
    if isinstance(rate_results, list) and rate_results:
        rows = [base_row(summary, path, item) for item in rate_results if isinstance(item, dict)]
        return rows or [base_row(summary, path)]
    return [base_row(summary, path)]


def classify_row(row: dict[str, Any], summary: dict[str, Any] | None = None) -> tuple[str, str]:
    backend = row.get("backend")
    group = row.get("metric_group")
    status = row.get("status")
    if status == "not_yet_run":
        return "not_yet_run", row.get("note") or "summary artifact missing or no evidence supplied"
    if status in {"unsupported", "skipped"}:
        return "unsupported" if status == "unsupported" else "not_yet_run", str(row.get("reason") or status)
    if group == "read_state" and backend == "rbscript_tcp":
        if row.get("read_state_capability") == "unsupported" or row.get("comparable") is False:
            return "unsupported", str(row.get("reason") or "rbscript_tcp read_state unsupported")
    if group == "command_ack_no_motion":
        if backend == "rbscript_tcp":
            return "measured_not_comparable", "rbscript_tcp command ACK test is not ServoJ test"
        return "unsupported", "command_ack_no_motion is only defined for rbscript_tcp probes"
    if group == "servo_j_noop":
        if row.get("comparable") is False:
            return "measured_not_comparable", str(row.get("reason") or "ServoJ no-op row is not comparable")
        return "measured_and_comparable", ""
    if group in {"connect_only", "read_state"}:
        comparable = row.get("comparable")
        if comparable is False:
            return "measured_not_comparable", str(row.get("reason") or "not comparable")
        return "measured_and_comparable", ""
    return "measured_not_comparable", "unknown metric group"


def missing_row(backend: str, metric_group: str, reason: str) -> dict[str, Any]:
    return {
        "metric_group": metric_group,
        "backend": backend,
        "classification": "not_yet_run",
        "status": "not_yet_run",
        "artifact_path": "",
        "requested_rate_hz": None,
        "achieved_rate_hz": None,
        "success_rate": None,
        "timeout_count": None,
        "error_count": None,
        "unrecognized_response_count": None,
        "read_state_capability": "",
        "comparable": None,
        "reason": reason,
        "note": reason,
    }


def missing_path_row(path: Path) -> dict[str, Any]:
    name = path.as_posix().lower()
    backend = "rbscript_tcp" if "rbscript" in name else "rbpodo" if "rbpodo" in name else ""
    group = "servo_j_noop" if "servo" in name else "read_state" if "read" in name else ""
    return {
        "metric_group": group,
        "backend": backend,
        "classification": "not_yet_run",
        "status": "not_yet_run",
        "artifact_path": str(path),
        "requested_rate_hz": None,
        "achieved_rate_hz": None,
        "success_rate": None,
        "timeout_count": None,
        "error_count": None,
        "unrecognized_response_count": None,
        "read_state_capability": "",
        "comparable": None,
        "reason": f"summary missing: {path}",
        "note": "summary artifact missing",
    }


def add_default_missing_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    out = list(rows)
    seen = {(row.get("backend"), row.get("metric_group")) for row in rows if row.get("classification") != "not_yet_run"}
    for backend in BACKENDS:
        for group in METRIC_GROUPS:
            if (backend, group) not in seen:
                out.append(missing_row(backend, group, f"{backend} {group} not yet run"))
    return out


def read_state_rows(rows: list[dict[str, Any]], backend: str) -> list[dict[str, Any]]:
    return [row for row in rows if row.get("backend") == backend and row.get("metric_group") == "read_state"]


def rate_rows_up_to(rows: list[dict[str, Any]], hz: float) -> list[dict[str, Any]]:
    out = []
    for row in rows:
        rate = finite_number(row.get("requested_rate_hz"))
        if rate is not None and rate <= hz:
            out.append(row)
    return out


def rbpodo_read_state_stable(rows: list[dict[str, Any]]) -> bool:
    candidates = [
        row for row in rate_rows_up_to(read_state_rows(rows, "rbpodo"), READ_STATE_STABLE_MAX_HZ)
        if row.get("classification") == "measured_and_comparable"
        and finite_number(row.get("success_rate")) is not None
        and finite_number(row.get("achieved_rate_hz")) is not None
    ]
    if not candidates:
        return False
    for row in candidates:
        success = finite_number(row.get("success_rate"))
        requested = finite_number(row.get("requested_rate_hz"))
        achieved = finite_number(row.get("achieved_rate_hz"))
        if success is None or success < READ_STATE_MIN_SUCCESS_RATE:
            return False
        if requested and achieved is not None and achieved < requested * ACHIEVED_RATE_MIN_RATIO:
            return False
    return True


def rbpodo_read_only_diagnostic_works(rows: list[dict[str, Any]]) -> bool:
    for row in read_state_rows(rows, "rbpodo"):
        if row.get("classification") != "measured_and_comparable":
            continue
        path = str(row.get("artifact_path", "")).lower()
        reason = str(row.get("reason", "")).lower()
        if "read_only" in path or "state stream captured" in reason:
            success = finite_number(row.get("success_rate"))
            if success is None or success >= READ_STATE_MIN_SUCCESS_RATE:
                return True
    return False


def rbpodo_servo_contradiction(rows: list[dict[str, Any]]) -> bool:
    for row in rows:
        if row.get("backend") != "rbpodo" or row.get("metric_group") != "servo_j_noop":
            continue
        if row.get("classification") == "measured_and_comparable":
            success = finite_number(row.get("success_rate"))
            if success is not None and success < READ_STATE_MIN_SUCCESS_RATE:
                return True
        if row.get("status") == "failed":
            return True
    return False


def rbscript_read_state_unsupported(rows: list[dict[str, Any]]) -> bool:
    return any(
        row.get("backend") == "rbscript_tcp"
        and row.get("metric_group") == "read_state"
        and row.get("classification") == "unsupported"
        for row in rows
    )


def rbscript_ack_has_issues(rows: list[dict[str, Any]]) -> bool:
    for row in rows:
        if row.get("backend") != "rbscript_tcp" or row.get("metric_group") != "command_ack_no_motion":
            continue
        timeout = row.get("timeout_count")
        errors = row.get("error_count")
        unrecognized = row.get("unrecognized_response_count")
        if any((value or 0) > ACK_ISSUE_THRESHOLD for value in (timeout, errors, unrecognized)):
            return True
    return False


def servo_apples_to_apples_complete(rows: list[dict[str, Any]]) -> bool:
    return all(
        any(
            row.get("backend") == backend
            and row.get("metric_group") == "servo_j_noop"
            and row.get("classification") == "measured_and_comparable"
            for row in rows
        )
        for backend in BACKENDS
    )


def recommendation(rows: list[dict[str, Any]]) -> dict[str, Any]:
    stable_read = rbpodo_read_state_stable(rows)
    read_only = rbpodo_read_only_diagnostic_works(rows)
    contradicted = rbpodo_servo_contradiction(rows)
    if stable_read and read_only and not contradicted:
        primary = "rbpodo"
    else:
        primary = "undecided; rbpodo remains the primary candidate pending complete evidence"

    rbscript_reasons: list[str] = []
    if rbscript_read_state_unsupported(rows):
        rbscript_reasons.append("data port read_state unsupported")
    if rbscript_ack_has_issues(rows):
        rbscript_reasons.append("command ACK has timeout/error/unrecognized responses")
    if not servo_apples_to_apples_complete(rows):
        rbscript_reasons.append("no complete ServoJ no-op apples-to-apples result")
    experimental = "rbscript_tcp remains experimental"
    if rbscript_reasons:
        experimental += ": " + "; ".join(rbscript_reasons)

    next_required: list[str] = []
    if not stable_read or not read_only:
        next_required.append("Repeat rbpodo read_state/read-only diagnostic evidence through 100 Hz.")
    if not any(row.get("backend") == "rbpodo" and row.get("metric_group") == "servo_j_noop" and row.get("classification") == "measured_and_comparable" for row in rows):
        next_required.append("Run rbpodo ServoJ ACK-on/off controller-simulation no-op acceptance.")
    if rbscript_read_state_unsupported(rows):
        next_required.append("Implement or verify rbscript_tcp real Rainbow 5001 read_state parsing before comparing state performance.")
    if not any(row.get("backend") == "rbscript_tcp" and row.get("metric_group") == "servo_j_noop" and row.get("classification") == "measured_and_comparable" for row in rows):
        next_required.append("Run rbscript_tcp ServoJ no-op controller-simulation acceptance after data/q-source preflight.")

    return {
        "primary_backend_recommendation": primary,
        "experimental_backend_status": experimental,
        "next_required_experiments": next_required,
        "decision_inputs": {
            "rbpodo_read_state_success_rate_min": READ_STATE_MIN_SUCCESS_RATE,
            "rbpodo_read_state_required_through_hz": READ_STATE_STABLE_MAX_HZ,
            "achieved_rate_min_ratio": ACHIEVED_RATE_MIN_RATIO,
            "ack_issue_threshold": ACK_ISSUE_THRESHOLD,
        },
    }


def group_summary(rows: list[dict[str, Any]], backend: str, group: str) -> str:
    matches = [row for row in rows if row.get("backend") == backend and row.get("metric_group") == group]
    if not matches:
        return "not yet run"
    if any(row.get("classification") == "unsupported" for row in matches):
        reason = next((row.get("reason") for row in matches if row.get("reason")), "")
        return "unsupported" + (f" ({reason})" if reason else "")
    measured = [row for row in matches if str(row.get("classification", "")).startswith("measured")]
    if not measured:
        return str(matches[0].get("classification"))
    success_values = [finite_number(row.get("success_rate")) for row in measured]
    rates = [finite_number(row.get("requested_rate_hz")) for row in measured]
    max_rate = max((rate for rate in rates if rate is not None), default=None)
    min_success = min((value for value in success_values if value is not None), default=None)
    details = []
    if max_rate is not None:
        details.append(f"through {fmt(max_rate)} Hz")
    if min_success is not None:
        details.append(f"min success {fmt(min_success)}")
    state = measured[0].get("classification")
    return f"{state}" + (": " + ", ".join(details) if details else "")


def markdown_table(rows: list[dict[str, Any]], columns: list[str]) -> str:
    if not rows:
        return "_No evidence rows._\n"
    lines = ["| " + " | ".join(columns) + " |", "| " + " | ".join("---" for _ in columns) + " |"]
    for row in rows:
        lines.append("| " + " | ".join(fmt(row.get(column)).replace("|", "\\|") for column in columns) + " |")
    return "\n".join(lines) + "\n"


def generate_markdown(rows: list[dict[str, Any]], rec: dict[str, Any]) -> str:
    notes = []
    if any(row.get("backend") == "rbscript_tcp" and row.get("metric_group") == "command_ack_no_motion" for row in rows):
        notes.append("- rbscript_tcp command ACK test is not ServoJ test.")
    if rbscript_read_state_unsupported(rows):
        notes.append("- rbscript_tcp read_state unsupported.")
    if not any(row.get("backend") == "rbpodo" and row.get("metric_group") == "servo_j_noop" and row.get("classification") == "measured_and_comparable" for row in rows):
        notes.append("- rbpodo ServoJ ACK-on/off not yet run.")
    if not any(row.get("backend") == "rbscript_tcp" and row.get("metric_group") == "servo_j_noop" and row.get("classification") == "measured_and_comparable" for row in rows):
        notes.append("- rbscript_tcp ServoJ apples-to-apples result not yet run.")

    next_required = rec["next_required_experiments"] or ["No additional experiment required by the current rule set."]
    sections = [
        "# Backend Comparison Report",
        "",
        "## Summary",
        f"- rbpodo read_state: {group_summary(rows, 'rbpodo', 'read_state')}",
        f"- rbscript_tcp read_state: {group_summary(rows, 'rbscript_tcp', 'read_state')}",
        f"- rbscript_tcp no-motion ACK: {group_summary(rows, 'rbscript_tcp', 'command_ack_no_motion')}",
        f"- ServoJ apples-to-apples: {'complete' if servo_apples_to_apples_complete(rows) else 'pending'}",
        *notes,
        "",
        "## Recommendation",
        f"- primary_backend_recommendation: {rec['primary_backend_recommendation']}",
        f"- experimental_backend_status: {rec['experimental_backend_status']}",
        "- next_required_experiments:",
        *[f"  - {item}" for item in next_required],
        "",
        "## Decision Rules",
        (
            f"- rbpodo primary candidate requires read_state success_rate >= {READ_STATE_MIN_SUCCESS_RATE} "
            f"through {READ_STATE_STABLE_MAX_HZ:.0f} Hz, achieved rate >= {ACHIEVED_RATE_MIN_RATIO:.0%} "
            "of requested rate, read-only diagnostic state stream evidence, and no contradicting command-path evidence."
        ),
        (
            "- rbscript_tcp remains experimental when real data-port read_state is unsupported, command ACK has "
            "timeout/error/unrecognized responses, or ServoJ no-op apples-to-apples evidence is missing."
        ),
        "- command_ack_no_motion evidence is reported separately from ServoJ no-op evidence.",
        "",
        "## Evidence Table",
        markdown_table(rows, EVIDENCE_COLUMNS),
    ]
    return "\n".join(sections)


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=EVIDENCE_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow({column: row.get(column) for column in EVIDENCE_COLUMNS})


def write_json(path: Path, rows: list[dict[str, Any]], rec: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps({"recommendation": rec, "evidence": rows}, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def collect_rows(paths: list[Path], require_default_missing_rows: bool = False) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for path in paths:
        if not path.is_file():
            rows.append(missing_path_row(path))
            continue
        summary = load_json(path)
        rows.extend(rows_from_summary(summary, path))
    if require_default_missing_rows or not paths:
        rows = add_default_missing_rows(rows)
    return rows


def main() -> int:
    args = parse_args()
    rows = collect_rows(args.summary_json, args.require_default_missing_rows)
    rec = recommendation(rows)
    markdown = generate_markdown(rows, rec)

    if args.output_md:
        args.output_md.parent.mkdir(parents=True, exist_ok=True)
        args.output_md.write_text(markdown, encoding="utf-8")
    else:
        sys.stdout.write(markdown)
        if not markdown.endswith("\n"):
            sys.stdout.write("\n")
    if args.output_csv:
        write_csv(args.output_csv, rows)
    if args.output_json:
        write_json(args.output_json, rows, rec)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
