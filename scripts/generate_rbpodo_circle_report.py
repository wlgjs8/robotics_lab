#!/usr/bin/env python3
"""Generate rbpodo controller-simulation circle benchmark reports."""

from __future__ import annotations

import argparse
from pathlib import Path

import generate_circle_benchmark_report as report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate a Markdown/CSV report for rbpodo controller-simulation "
            "circle benchmark summary.json files or rbpodo ablation summaries. "
            "The report keeps pgmode-simulation evidence separate from "
            "rb_simulator and future real physical evidence."
        )
    )
    parser.add_argument("summary_json", nargs="*", type=Path, help="rbpodo circle benchmark summary.json files")
    parser.add_argument("--ablation-summary-csv", type=Path, help="Optional rbpodo ablation_summary.csv input")
    parser.add_argument("--output-md", type=Path, help="Write markdown report to this path instead of stdout")
    parser.add_argument("--csv", dest="csv_path", type=Path, help="Optional CSV table output path")
    parser.add_argument("--min-baseline-repeats", type=int, default=1)
    parser.add_argument("--title", default="rbpodo Controller-Simulation Circle Report")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.min_baseline_repeats < 1:
        raise SystemExit("--min-baseline-repeats must be >= 1")
    if not args.summary_json and args.ablation_summary_csv is None:
        raise SystemExit("provide at least one summary.json or --ablation-summary-csv")
    rows = report.classify_rows(
        report.load_rows(args.summary_json, args.ablation_summary_csv),
        args.min_baseline_repeats,
    )
    markdown = report.report_markdown(rows, args.title, args.min_baseline_repeats)
    if args.output_md:
        args.output_md.parent.mkdir(parents=True, exist_ok=True)
        args.output_md.write_text(markdown, encoding="utf-8")
    else:
        print(markdown, end="")
    if args.csv_path:
        report.write_csv(args.csv_path, rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
