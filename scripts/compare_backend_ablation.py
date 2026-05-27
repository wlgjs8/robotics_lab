#!/usr/bin/env python3
"""Compare rb_backend_ablation summary.json artifacts."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any


COLUMNS = [
    ("backend", "backend"),
    ("mode", "mode"),
    ("rate_hz", "requested_rate_hz"),
    ("success_rate", "success_rate"),
    ("p50_ack_us", "p50_ack_us"),
    ("p95_ack_us", "p95_ack_us"),
    ("p99_ack_us", "p99_ack_us"),
    ("timeout_count", "timeout_count"),
    ("error_count", "error_count"),
    ("achieved_rate_hz", "achieved_rate_hz"),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Print a markdown table for rb backend ablation summaries.")
    parser.add_argument("summary_json", nargs="+", type=Path)
    parser.add_argument("--csv", dest="csv_path", type=Path)
    return parser.parse_args()


def finite_number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        number = float(value)
        return number if math.isfinite(number) else None
    return None


def nested(summary: dict[str, Any], key: str, metric: str) -> float | None:
    value = summary.get(key)
    if not isinstance(value, dict):
        return None
    return finite_number(value.get(metric))


def load_summary(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise SystemExit(f"{path} does not contain a JSON object")
    data["_path"] = str(path)
    return data


def comparison_row(summary: dict[str, Any]) -> dict[str, Any]:
    ack = summary.get("command_ack_latency_us") if isinstance(summary.get("command_ack_latency_us"), dict) else {}
    read = summary.get("read_duration_us") if isinstance(summary.get("read_duration_us"), dict) else {}
    connect = summary.get("connect_latency_us") if isinstance(summary.get("connect_latency_us"), dict) else {}
    timing = ack or read or connect
    timeout_count = summary.get("command_timeout_count") or summary.get("data_port_timeout_count") or 0
    error_count = max(int(summary.get("sample_count", 0) or 0) - int(summary.get("success_count", 0) or 0), 0)
    return {
        "backend": summary.get("backend"),
        "mode": summary.get("mode"),
        "requested_rate_hz": summary.get("requested_rate_hz"),
        "success_rate": summary.get("success_rate"),
        "p50_ack_us": finite_number(timing.get("p50")) if isinstance(timing, dict) else None,
        "p95_ack_us": finite_number(timing.get("p95")) if isinstance(timing, dict) else None,
        "p99_ack_us": finite_number(timing.get("p99")) if isinstance(timing, dict) else None,
        "timeout_count": timeout_count,
        "error_count": error_count,
        "achieved_rate_hz": summary.get("achieved_rate_hz"),
    }


def comparison_rows(summary: dict[str, Any]) -> list[dict[str, Any]]:
    rate_results = summary.get("rate_results")
    if not isinstance(rate_results, list):
        return [comparison_row(summary)]
    rows: list[dict[str, Any]] = []
    for item in rate_results:
        if not isinstance(item, dict):
            continue
        send_count = int(item.get("send_count", 0) or 0)
        success_rate = finite_number(item.get("success_rate"))
        success_count = int(send_count * success_rate) if success_rate is not None else 0
        rows.append({
            "backend": summary.get("backend"),
            "mode": summary.get("mode"),
            "requested_rate_hz": item.get("requested_rate_hz"),
            "success_rate": item.get("success_rate"),
            "p50_ack_us": item.get("p50_ack_us"),
            "p95_ack_us": item.get("p95_ack_us"),
            "p99_ack_us": item.get("p99_ack_us"),
            "timeout_count": item.get("ack_timeout_count") or item.get("data_timeout_count") or 0,
            "error_count": max(send_count - success_count - int(item.get("ack_timeout_count") or 0), 0),
            "achieved_rate_hz": item.get("achieved_rate_hz"),
        })
    return rows or [comparison_row(summary)]


def fmt(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        if not math.isfinite(value):
            return ""
        return f"{value:.3f}"
    return str(value)


def print_markdown(rows: list[dict[str, Any]]) -> None:
    headers = [header for header, _ in COLUMNS]
    print("| " + " | ".join(headers) + " |")
    print("| " + " | ".join("---" for _ in headers) + " |")
    for row in rows:
        print("| " + " | ".join(fmt(row.get(key)) for _, key in COLUMNS) + " |")


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=[key for _, key in COLUMNS])
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key) for _, key in COLUMNS})


def main() -> int:
    args = parse_args()
    rows: list[dict[str, Any]] = []
    for path in args.summary_json:
        rows.extend(comparison_rows(load_summary(path)))
    print_markdown(rows)
    if args.csv_path:
        write_csv(args.csv_path, rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
