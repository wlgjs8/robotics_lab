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
    ("persistent_socket", "persistent_socket"),
    ("reconnect_count", "reconnect_count"),
    ("rate_hz", "requested_rate_hz"),
    ("read_state_capability", "read_state_capability"),
    ("comparable", "comparable"),
    ("success_rate", "success_rate"),
    ("p50_ack_us", "p50_ack_us"),
    ("p95_ack_us", "p95_ack_us"),
    ("p99_ack_us", "p99_ack_us"),
    ("timeout_count", "timeout_count"),
    ("error_count", "error_count"),
    ("achieved_rate_hz", "achieved_rate_hz"),
    ("not_comparable_reason", "not_comparable_reason"),
]
RBSCRIPT_READ_STATE_NOT_COMPARABLE_REASON = (
    "rbscript_tcp real Rainbow data port 5001 parser is unsupported; "
    "capture raw data-port payloads before comparing read_state performance with rbpodo"
)
RBSCRIPT_READ_STATE_FIXTURE_ONLY_REASON = (
    "rbscript_tcp read_state parser accepted only the rbscript_tcp_state_v1 JSON fixture; "
    "real Rainbow 5001 parsing is not verified"
)


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


def infer_backend(summary: dict[str, Any]) -> Any:
    if summary.get("backend"):
        return summary.get("backend")
    safety = summary.get("safety_preflight")
    if isinstance(safety, dict) and safety.get("backend"):
        return safety.get("backend")
    observed = summary.get("observed_backend")
    if observed:
        return observed
    config = str(summary.get("config", ""))
    if "rbscript" in config:
        return "rbscript_tcp"
    if "rbpodo" in config or "dual_real" in config:
        return "rbpodo"
    return None


def requested_rate(summary: dict[str, Any]) -> Any:
    if summary.get("requested_rate_hz") is not None:
        return summary.get("requested_rate_hz")
    safety = summary.get("safety_preflight")
    if isinstance(safety, dict) and safety.get("servo_rate_hz") is not None:
        return safety.get("servo_rate_hz")
    return None


def success_rate(summary: dict[str, Any]) -> Any:
    if summary.get("success_rate") is not None:
        return summary.get("success_rate")
    send_count = int(summary.get("send_count", 0) or 0)
    if send_count > 0:
        return int(summary.get("send_success_count", 0) or 0) / send_count
    return None


def read_state_metadata(summary: dict[str, Any], item: dict[str, Any] | None = None) -> dict[str, Any]:
    item = item or {}
    backend = item.get("backend") or infer_backend(summary)
    mode = item.get("mode") or summary.get("mode")
    capability = item.get("read_state_capability")
    if capability is None:
        capability = summary.get("read_state_capability")
    data_port_mode = item.get("rbscript_tcp_data_port_mode")
    if data_port_mode is None:
        data_port_mode = summary.get("rbscript_tcp_data_port_mode")
    comparable = item.get("comparable")
    if comparable is None:
        comparable = summary.get("comparable")
    reason = item.get("not_comparable_reason") or summary.get("not_comparable_reason") or ""

    if mode != "read_state":
        return {"read_state_capability": capability or "", "comparable": comparable, "not_comparable_reason": reason}
    if backend == "rbpodo":
        return {"read_state_capability": capability or "supported", "comparable": True, "not_comparable_reason": ""}
    if backend == "rbscript_tcp":
        if capability == "supported" or data_port_mode == "real_controller_parsed":
            return {"read_state_capability": "supported", "comparable": True, "not_comparable_reason": ""}
        if capability == "experimental" or data_port_mode == "json_fixture":
            return {
                "read_state_capability": "experimental",
                "comparable": False,
                "not_comparable_reason": reason or RBSCRIPT_READ_STATE_FIXTURE_ONLY_REASON,
            }
        return {
            "read_state_capability": "unsupported",
            "comparable": False,
            "not_comparable_reason": reason or RBSCRIPT_READ_STATE_NOT_COMPARABLE_REASON,
        }
    return {"read_state_capability": capability or "", "comparable": comparable, "not_comparable_reason": reason}


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
    send = summary.get("send_duration_us") if isinstance(summary.get("send_duration_us"), dict) else {}
    ack_wait = summary.get("ack_wait_duration_us") if isinstance(summary.get("ack_wait_duration_us"), dict) else {}
    timing = ack or ack_wait or send or read or connect
    timeout_count = summary.get("command_timeout_count") or summary.get("data_port_timeout_count") or 0
    if summary.get("send_count") is not None:
        error_count = int(summary.get("send_failure_count", 0) or 0)
    else:
        error_count = max(int(summary.get("sample_count", 0) or 0) - int(summary.get("success_count", 0) or 0), 0)
    metadata = read_state_metadata(summary)
    comparable = metadata["comparable"]
    hide_performance = summary.get("mode") == "read_state" and comparable is False
    return {
        "backend": infer_backend(summary),
        "mode": summary.get("mode"),
        "persistent_socket": summary.get("persistent_socket"),
        "reconnect_count": summary.get("reconnect_count"),
        "requested_rate_hz": requested_rate(summary),
        "read_state_capability": metadata["read_state_capability"],
        "comparable": comparable,
        "success_rate": None if hide_performance else success_rate(summary),
        "p50_ack_us": None if hide_performance else (finite_number(timing.get("p50")) if isinstance(timing, dict) else None),
        "p95_ack_us": None if hide_performance else (finite_number(timing.get("p95")) if isinstance(timing, dict) else None),
        "p99_ack_us": None if hide_performance else (finite_number(timing.get("p99")) if isinstance(timing, dict) else None),
        "timeout_count": timeout_count,
        "error_count": error_count,
        "achieved_rate_hz": summary.get("achieved_rate_hz"),
        "not_comparable_reason": metadata["not_comparable_reason"],
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
        metadata = read_state_metadata(summary, item)
        hide_performance = summary.get("mode") == "read_state" and metadata["comparable"] is False
        rows.append({
            "backend": summary.get("backend"),
            "mode": summary.get("mode"),
            "persistent_socket": item.get("persistent_socket", summary.get("persistent_socket")),
            "reconnect_count": item.get("reconnect_count"),
            "requested_rate_hz": item.get("requested_rate_hz"),
            "read_state_capability": metadata["read_state_capability"],
            "comparable": metadata["comparable"],
            "success_rate": None if hide_performance else item.get("success_rate"),
            "p50_ack_us": None if hide_performance else item.get("p50_ack_us"),
            "p95_ack_us": None if hide_performance else item.get("p95_ack_us"),
            "p99_ack_us": None if hide_performance else item.get("p99_ack_us"),
            "timeout_count": item.get("ack_timeout_count") or item.get("data_timeout_count") or 0,
            "error_count": max(
                send_count
                - success_count
                - int(item.get("ack_timeout_count") or item.get("data_timeout_count") or 0),
                0,
            ),
            "achieved_rate_hz": item.get("achieved_rate_hz"),
            "not_comparable_reason": metadata["not_comparable_reason"],
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
