#!/usr/bin/env python3
"""Offline report for Rainbow data-port raw capture fixtures."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import rainbow_data_port_capture as capture


RECOMMENDED_NEXT_STEPS = [
    "collect motion/no-op fixture",
    "compare firmware SDK docs",
    "do not infer layout yet",
]


class FixtureReportError(RuntimeError):
    pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Summarize a read-only Rainbow TCP 5001 raw capture directory into "
            "repeatable fixture evidence. The report does not parse binary "
            "layout and writes only Markdown/JSON report files."
        )
    )
    parser.add_argument("--capture-dir", type=Path, required=True)
    parser.add_argument(
        "--output-md",
        type=Path,
        required=True,
        help="Markdown report path. Relative paths are resolved inside --capture-dir.",
    )
    parser.add_argument(
        "--output-json",
        type=Path,
        required=True,
        help="JSON report path. Relative paths are resolved inside --capture-dir.",
    )
    parser.add_argument("--top-offset-count", type=int, default=20)
    return parser.parse_args()


def read_json(path: Path) -> dict[str, Any]:
    try:
        loaded = json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise FixtureReportError(f"failed to read {path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise FixtureReportError(f"failed to parse JSON {path}: {exc}") from exc
    if not isinstance(loaded, dict):
        raise FixtureReportError(f"expected JSON object in {path}")
    return loaded


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.exists():
        return rows
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise FixtureReportError(f"failed to read {path}: {exc}") from exc
    for line_number, line in enumerate(lines, start=1):
        if not line.strip():
            continue
        try:
            loaded = json.loads(line)
        except json.JSONDecodeError as exc:
            raise FixtureReportError(f"failed to parse {path}:{line_number}: {exc}") from exc
        if not isinstance(loaded, dict):
            raise FixtureReportError(f"expected JSON object at {path}:{line_number}")
        rows.append(loaded)
    return rows


def resolve_capture_child(capture_dir: Path, value: str | None) -> Path | None:
    if not value:
        return None
    root = capture_dir.resolve()
    path = Path(value)
    candidate = path.resolve() if path.is_absolute() else (root / path).resolve()
    if candidate != root and root not in candidate.parents:
        raise FixtureReportError(f"refusing path outside capture dir: {value}")
    return candidate


def resolve_output_path(capture_dir: Path, requested: Path) -> Path:
    root = capture_dir.resolve()
    candidate = requested.resolve() if requested.is_absolute() else (root / requested).resolve()
    if candidate != root and root not in candidate.parents:
        raise FixtureReportError(f"refusing output outside capture dir: {requested}")
    return candidate


def load_payloads(
    capture_dir: Path,
    samples: list[dict[str, Any]],
) -> dict[tuple[int | None, str | None], bytes]:
    payloads: dict[tuple[int | None, str | None], bytes] = {}
    for sample in samples:
        path = resolve_capture_child(capture_dir, sample.get("binary_path"))
        if path is None or not path.exists():
            continue
        payloads[capture.sample_key(sample)] = path.read_bytes()
    return payloads


def histogram_from(value: Any) -> dict[str, int]:
    if isinstance(value, dict):
        out: dict[str, int] = {}
        for key, count in value.items():
            try:
                out[str(int(key))] = int(count)
            except (TypeError, ValueError):
                continue
        return {key: out[key] for key in sorted(out, key=lambda item: int(item))}
    if isinstance(value, list):
        counter: dict[str, int] = {}
        for item in value:
            if not isinstance(item, dict):
                continue
            try:
                offset = str(int(item.get("offset")))
                count = int(item.get("count"))
            except (TypeError, ValueError):
                continue
            counter[offset] = count
        return {key: counter[key] for key in sorted(counter, key=lambda item: int(item))}
    return {}


def first_present(*values: Any) -> Any:
    for value in values:
        if value is not None:
            return value
    return None


def build_report(capture_dir: Path, *, top_offset_count: int) -> dict[str, Any]:
    if top_offset_count <= 0:
        raise FixtureReportError("--top-offset-count must be positive")
    summary_path = capture_dir / "summary.json"
    samples_path = capture_dir / "samples.jsonl"
    if not summary_path.exists():
        raise FixtureReportError(f"missing summary.json in {capture_dir}")
    if not samples_path.exists():
        raise FixtureReportError(f"missing samples.jsonl in {capture_dir}")

    summary = read_json(summary_path)
    samples = read_jsonl(samples_path)
    python_samples = read_jsonl(capture_dir / "python_decoded_samples.jsonl")
    payloads = load_payloads(capture_dir, samples)
    success_count = sum(1 for sample in samples if sample.get("ok"))
    fixture = summary.get("fixture_comparison")
    fixture = fixture if isinstance(fixture, dict) else {}

    if payloads and len(payloads) == success_count:
        local_samples = [dict(sample) for sample in samples]
        local_python_samples = [dict(sample) for sample in python_samples]
        fixture = capture.analyze_payload_patterns(
            local_samples,
            payloads,
            local_python_samples,
            annotate=True,
        )

    histogram = histogram_from(first_present(
        fixture.get("changed_offsets_histogram"),
        summary.get("changed_offsets_histogram"),
    ))
    top_offsets = capture.top_changed_offsets(histogram, limit=top_offset_count)
    unique_payload_lengths = first_present(
        fixture.get("unique_payload_lengths"),
        summary.get("unique_payload_lengths"),
        [],
    )
    unique_hash_count = first_present(
        fixture.get("unique_hash_count"),
        summary.get("unique_hash_count"),
        0,
    )

    return {
        "schema": "robotics_lab.rainbow_data_fixture_report.v1",
        "read_only": True,
        "capture_dir": str(capture_dir),
        "capture_schema": summary.get("schema"),
        "capture_result": summary.get("result"),
        "sample_count": len(samples),
        "success_count": success_count,
        "timeout_count": summary.get("timeout_count"),
        "error_count": summary.get("error_count"),
        "binary_storage_mode": summary.get("binary_storage_mode"),
        "save_each_sample": summary.get("save_each_sample"),
        "unique_payload_lengths": unique_payload_lengths,
        "unique_hash_count": unique_hash_count,
        "stable_prefix_hex": first_present(
            fixture.get("stable_prefix_hex"),
            summary.get("stable_prefix_hex"),
            "",
        ),
        "stable_suffix_hex": first_present(
            fixture.get("stable_suffix_hex"),
            summary.get("stable_suffix_hex"),
            "",
        ),
        "changed_offsets_histogram": histogram,
        "offsets_that_change_most_often": top_offsets,
        "q_ref_payload_transition_count": first_present(
            fixture.get("q_ref_payload_transition_count"),
            summary.get("q_ref_payload_transition_count"),
            0,
        ),
        "q_ref_changed_payload_changed_count": first_present(
            fixture.get("q_ref_changed_payload_changed_count"),
            summary.get("q_ref_changed_payload_changed_count"),
            0,
        ),
        "q_ref_changed_payload_static_count": first_present(
            fixture.get("q_ref_changed_payload_static_count"),
            summary.get("q_ref_changed_payload_static_count"),
            0,
        ),
        "q_ref_static_payload_changed_count": first_present(
            fixture.get("q_ref_static_payload_changed_count"),
            summary.get("q_ref_static_payload_changed_count"),
            0,
        ),
        "q_ref_static_payload_static_count": first_present(
            fixture.get("q_ref_static_payload_static_count"),
            summary.get("q_ref_static_payload_static_count"),
            0,
        ),
        "recommended_next_steps": list(RECOMMENDED_NEXT_STEPS),
        "parser_policy": "no_speculative_binary_parser",
        "safety_note": (
            "Offline report only. Raw capture was read-only evidence; this "
            "report does not infer binary layout or authorize motion."
        ),
    }


def markdown_report(report: dict[str, Any]) -> str:
    lines = [
        "# Rainbow Data-Port Fixture Report",
        "",
        f"Capture result: `{report.get('capture_result')}`",
        f"Capture dir: `{report.get('capture_dir')}`",
        "",
        "This is offline/read-only fixture evidence. It does not parse binary layout or authorize motion.",
        "",
        "## Payload Evidence",
        "",
        f"- `unique_payload_lengths`: {report.get('unique_payload_lengths')}",
        f"- `unique_hash_count`: {report.get('unique_hash_count')}",
        f"- `stable_prefix_hex`: `{report.get('stable_prefix_hex') or ''}`",
        f"- `stable_suffix_hex`: `{report.get('stable_suffix_hex') or ''}`",
        "",
        "## Offsets That Change Most Often",
        "",
        "| offset | count |",
        "| ---: | ---: |",
    ]
    offsets = report.get("offsets_that_change_most_often") or []
    if offsets:
        for row in offsets:
            lines.append(f"| {row.get('offset')} | {row.get('count')} |")
    else:
        lines.append("| none | 0 |")

    lines.extend([
        "",
        "## q_ref / Payload Transitions",
        "",
        f"- `q_ref_changed_payload_changed_count`: {report.get('q_ref_changed_payload_changed_count')}",
        f"- `q_ref_changed_payload_static_count`: {report.get('q_ref_changed_payload_static_count')}",
        f"- `q_ref_static_payload_changed_count`: {report.get('q_ref_static_payload_changed_count')}",
        "",
        "## Recommended Next Step",
        "",
    ])
    for step in report.get("recommended_next_steps") or []:
        lines.append(f"- {step}")
    return "\n".join(lines) + "\n"


def main() -> int:
    args = parse_args()
    capture_dir = args.capture_dir.resolve()
    try:
        if not capture_dir.is_dir():
            raise FixtureReportError(f"--capture-dir is not a directory: {capture_dir}")
        report = build_report(capture_dir, top_offset_count=args.top_offset_count)
        output_md = resolve_output_path(capture_dir, args.output_md)
        output_json = resolve_output_path(capture_dir, args.output_json)
        output_md.parent.mkdir(parents=True, exist_ok=True)
        output_json.parent.mkdir(parents=True, exist_ok=True)
        output_json.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        output_md.write_text(markdown_report(report), encoding="utf-8")
    except FixtureReportError as exc:
        print(f"rainbow_data_fixture_report: FAIL: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({
        "result": "completed",
        "output_md": str(output_md),
        "output_json": str(output_json),
    }, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
