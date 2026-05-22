#!/usr/bin/env python3
"""Analyze rb_servo_server CSV logs against mock/rbsim servo budgets.

The analyzer intentionally uses only the Python standard library so it can run in
minimal smoke-test environments.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


@dataclass(frozen=True)
class ProfileBudget:
    name: str
    min_duration_s: float
    target_period_ms: float
    period_tolerance_ms: float
    jitter_p95_ms: float
    jitter_max_ms: float
    send_skew_p95_us: float
    send_skew_max_us: float | None
    send_duration_p95_us: float | None
    dropped_samples_max: int
    send_failure_count_max: int
    tracking_error_max_deg: float | None = None


BUDGETS: dict[str, ProfileBudget] = {
    "mock200": ProfileBudget(
        name="mock200",
        min_duration_s=60.0,
        target_period_ms=5.0,
        period_tolerance_ms=0.25,
        jitter_p95_ms=1.0,
        jitter_max_ms=5.0,
        send_skew_p95_us=100.0,
        send_skew_max_us=500.0,
        send_duration_p95_us=500.0,
        dropped_samples_max=0,
        send_failure_count_max=0,
        tracking_error_max_deg=2.0,
    ),
    "rbsim-local100": ProfileBudget(
        name="rbsim-local100",
        min_duration_s=2.0,
        target_period_ms=10.0,
        period_tolerance_ms=0.75,
        jitter_p95_ms=3.0,
        jitter_max_ms=15.0,
        send_skew_p95_us=3000.0,
        send_skew_max_us=None,
        send_duration_p95_us=None,
        dropped_samples_max=0,
        send_failure_count_max=0,
        tracking_error_max_deg=2.0,
    ),
    "rbsim100": ProfileBudget(
        name="rbsim100",
        min_duration_s=30.0,
        target_period_ms=10.0,
        period_tolerance_ms=0.5,
        jitter_p95_ms=2.0,
        jitter_max_ms=10.0,
        send_skew_p95_us=2000.0,
        send_skew_max_us=None,
        send_duration_p95_us=None,
        dropped_samples_max=0,
        send_failure_count_max=0,
        tracking_error_max_deg=2.0,
    ),
}

BASE_REQUIRED_COLUMNS = [
    "period_ms",
    "jitter_ms",
    "send_skew_us",
    "left_send_start_ns",
    "left_send_end_ns",
    "right_send_start_ns",
    "right_send_end_ns",
    "left_send_duration_us",
    "right_send_duration_us",
    "logger_dropped_samples",
    "left_send_ok",
    "right_send_ok",
]

Q_REQUIRED_COLUMNS = [
    f"{arm}_q_{kind}_{joint}"
    for arm in ("left", "right")
    for kind in ("actual", "sent")
    for joint in range(6)
]

TIMESTAMP_COLUMNS = ["loop_start_time_ns", "loop_end_time_ns"]


class AnalysisError(Exception):
    """Raised for invalid input that prevents log analysis."""


def parse_float(value: str, column: str, row_number: int) -> float:
    try:
        number = float(value)
    except ValueError as exc:
        raise AnalysisError(f"row {row_number}: column {column!r} is not numeric: {value!r}") from exc
    if not math.isfinite(number):
        raise AnalysisError(f"row {row_number}: column {column!r} is not finite: {value!r}")
    return number


def parse_bool(value: str, column: str, row_number: int) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "t", "yes", "y", "ok"}:
        return True
    if normalized in {"0", "false", "f", "no", "n", "fail", "failed"}:
        return False
    raise AnalysisError(f"row {row_number}: column {column!r} is not boolean-like: {value!r}")


def percentile_nearest_rank(values: Sequence[float], percentile: float) -> float:
    if not values:
        raise AnalysisError("cannot compute percentile of an empty series")
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, math.ceil((percentile / 100.0) * len(ordered)) - 1))
    return ordered[index]


def mean(values: Sequence[float]) -> float:
    if not values:
        raise AnalysisError("cannot compute mean of an empty series")
    return statistics.fmean(values)


def series_summary(values: Sequence[float]) -> dict[str, float]:
    return {
        "mean": mean(values),
        "p95": percentile_nearest_rank(values, 95.0),
        "max": max(values),
    }


def require_columns(fieldnames: Iterable[str] | None, required: Sequence[str]) -> list[str]:
    available = set(fieldnames or [])
    return [column for column in required if column not in available]


def analyze_csv(path: Path) -> dict[str, object]:
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        missing = require_columns(reader.fieldnames, BASE_REQUIRED_COLUMNS + Q_REQUIRED_COLUMNS)
        if missing:
            raise AnalysisError("missing required CSV columns: " + ", ".join(missing))

        period_ms: list[float] = []
        jitter_ms: list[float] = []
        send_skew_us: list[float] = []
        left_send_duration_us: list[float] = []
        right_send_duration_us: list[float] = []
        dropped_samples: list[int] = []
        tracking_errors: dict[str, list[float]] = {"left": [], "right": []}
        send_failures_by_arm = {"left": 0, "right": 0}
        rows_with_send_failure = 0
        first_loop_start_ns: float | None = None
        last_loop_end_ns: float | None = None
        rows = 0

        for rows, row in enumerate(reader, start=1):
            row_number = rows + 1  # include header line for user-facing errors
            period_ms.append(parse_float(row["period_ms"], "period_ms", row_number))
            jitter_ms.append(abs(parse_float(row["jitter_ms"], "jitter_ms", row_number)))
            send_skew_us.append(abs(parse_float(row["send_skew_us"], "send_skew_us", row_number)))
            left_send_start_ns = parse_float(row["left_send_start_ns"], "left_send_start_ns", row_number)
            left_send_end_ns = parse_float(row["left_send_end_ns"], "left_send_end_ns", row_number)
            right_send_start_ns = parse_float(row["right_send_start_ns"], "right_send_start_ns", row_number)
            right_send_end_ns = parse_float(row["right_send_end_ns"], "right_send_end_ns", row_number)
            if left_send_end_ns < left_send_start_ns:
                raise AnalysisError(f"row {row_number}: left_send_end_ns is before left_send_start_ns")
            if right_send_end_ns < right_send_start_ns:
                raise AnalysisError(f"row {row_number}: right_send_end_ns is before right_send_start_ns")
            left_send_duration_us.append(parse_float(row["left_send_duration_us"], "left_send_duration_us", row_number))
            right_send_duration_us.append(parse_float(row["right_send_duration_us"], "right_send_duration_us", row_number))

            dropped_value = parse_float(row["logger_dropped_samples"], "logger_dropped_samples", row_number)
            dropped_samples.append(int(dropped_value))
            if dropped_value != int(dropped_value):
                raise AnalysisError(f"row {row_number}: logger_dropped_samples must be an integer")

            left_ok = parse_bool(row["left_send_ok"], "left_send_ok", row_number)
            right_ok = parse_bool(row["right_send_ok"], "right_send_ok", row_number)
            if not left_ok:
                send_failures_by_arm["left"] += 1
            if not right_ok:
                send_failures_by_arm["right"] += 1
            if not left_ok or not right_ok:
                rows_with_send_failure += 1

            for arm in ("left", "right"):
                for joint in range(6):
                    actual = parse_float(row[f"{arm}_q_actual_{joint}"], f"{arm}_q_actual_{joint}", row_number)
                    sent = parse_float(row[f"{arm}_q_sent_{joint}"], f"{arm}_q_sent_{joint}", row_number)
                    tracking_errors[arm].append(abs(actual - sent))

            if all(column in row and row[column] != "" for column in TIMESTAMP_COLUMNS):
                loop_start = parse_float(row["loop_start_time_ns"], "loop_start_time_ns", row_number)
                loop_end = parse_float(row["loop_end_time_ns"], "loop_end_time_ns", row_number)
                if first_loop_start_ns is None:
                    first_loop_start_ns = loop_start
                last_loop_end_ns = loop_end

    if rows == 0:
        raise AnalysisError(f"{path} contains no data rows")

    if first_loop_start_ns is not None and last_loop_end_ns is not None and last_loop_end_ns >= first_loop_start_ns:
        duration_s = (last_loop_end_ns - first_loop_start_ns) / 1_000_000_000.0
        duration_source = "loop timestamps"
    else:
        duration_s = sum(period_ms) / 1000.0
        duration_source = "sum(period_ms)"

    left_error_summary = series_summary(tracking_errors["left"])
    right_error_summary = series_summary(tracking_errors["right"])

    return {
        "rows": rows,
        "duration_s": duration_s,
        "duration_source": duration_source,
        "period_ms": series_summary(period_ms),
        "jitter_ms": series_summary(jitter_ms),
        "send_skew_us": series_summary(send_skew_us),
        "send_duration_us": {
            "left": series_summary(left_send_duration_us),
            "right": series_summary(right_send_duration_us),
        },
        "logger_dropped_samples_max": max(dropped_samples),
        "tracking_error_deg": {
            "left": left_error_summary,
            "right": right_error_summary,
            "max": max(left_error_summary["max"], right_error_summary["max"]),
        },
        "send_failures": {
            "left": send_failures_by_arm["left"],
            "right": send_failures_by_arm["right"],
            "total_arm_failures": send_failures_by_arm["left"] + send_failures_by_arm["right"],
            "rows_with_failure": rows_with_send_failure,
        },
    }


def check_budget(metrics: dict[str, object], budget: ProfileBudget) -> list[str]:
    failures: list[str] = []

    def get(path: str) -> float:
        current: object = metrics
        for part in path.split("."):
            if not isinstance(current, dict):
                raise KeyError(path)
            current = current[part]
        return float(current)

    duration_s = get("duration_s")
    if duration_s < budget.min_duration_s:
        failures.append(f"duration_s {duration_s:.6g} < {budget.min_duration_s:.6g}")

    period_mean = get("period_ms.mean")
    lower = budget.target_period_ms - budget.period_tolerance_ms
    upper = budget.target_period_ms + budget.period_tolerance_ms
    if not (lower <= period_mean <= upper):
        failures.append(f"period_ms.mean {period_mean:.6g} outside [{lower:.6g}, {upper:.6g}]")

    checks = [
        ("jitter_ms.p95", budget.jitter_p95_ms),
        ("jitter_ms.max", budget.jitter_max_ms),
        ("send_skew_us.p95", budget.send_skew_p95_us),
    ]
    if budget.send_skew_max_us is not None:
        checks.append(("send_skew_us.max", budget.send_skew_max_us))
    if budget.send_duration_p95_us is not None:
        checks.extend(
            [
                ("send_duration_us.left.p95", budget.send_duration_p95_us),
                ("send_duration_us.right.p95", budget.send_duration_p95_us),
            ]
        )
    if budget.tracking_error_max_deg is not None:
        checks.append(("tracking_error_deg.max", budget.tracking_error_max_deg))

    for path, limit in checks:
        value = get(path)
        if value > limit:
            failures.append(f"{path} {value:.6g} > {limit:.6g}")

    dropped = int(get("logger_dropped_samples_max"))
    if dropped > budget.dropped_samples_max:
        failures.append(f"logger_dropped_samples_max {dropped} > {budget.dropped_samples_max}")

    send_failures = int(get("send_failures.total_arm_failures"))
    if send_failures > budget.send_failure_count_max:
        failures.append(f"send_failures.total_arm_failures {send_failures} > {budget.send_failure_count_max}")

    return failures


def format_report(metrics: dict[str, object], budget: ProfileBudget, failures: Sequence[str]) -> str:
    def get(path: str) -> object:
        current: object = metrics
        for part in path.split("."):
            if not isinstance(current, dict):
                raise KeyError(path)
            current = current[part]
        return current

    lines = [
        f"profile: {budget.name}",
        f"verdict: {'FAIL' if failures else 'PASS'}",
        f"rows: {get('rows')}",
        f"duration_s: {float(get('duration_s')):.6f} ({get('duration_source')})",
        f"period_ms: mean={float(get('period_ms.mean')):.6f} p95={float(get('period_ms.p95')):.6f} max={float(get('period_ms.max')):.6f}",
        f"jitter_ms: mean={float(get('jitter_ms.mean')):.6f} p95={float(get('jitter_ms.p95')):.6f} max={float(get('jitter_ms.max')):.6f}",
        f"send_skew_us: mean={float(get('send_skew_us.mean')):.6f} p95={float(get('send_skew_us.p95')):.6f} max={float(get('send_skew_us.max')):.6f}",
        f"left_send_duration_us: mean={float(get('send_duration_us.left.mean')):.6f} p95={float(get('send_duration_us.left.p95')):.6f} max={float(get('send_duration_us.left.max')):.6f}",
        f"right_send_duration_us: mean={float(get('send_duration_us.right.mean')):.6f} p95={float(get('send_duration_us.right.p95')):.6f} max={float(get('send_duration_us.right.max')):.6f}",
        f"logger_dropped_samples_max: {get('logger_dropped_samples_max')}",
        f"tracking_error_deg: left_max={float(get('tracking_error_deg.left.max')):.6f} right_max={float(get('tracking_error_deg.right.max')):.6f} max={float(get('tracking_error_deg.max')):.6f}",
        f"send_failures: left={get('send_failures.left')} right={get('send_failures.right')} total_arm_failures={get('send_failures.total_arm_failures')} rows_with_failure={get('send_failures.rows_with_failure')}",
    ]
    if failures:
        lines.append("budget_failures:")
        lines.extend(f"- {failure}" for failure in failures)
    return "\n".join(lines)


def write_sample_csv(path: Path, rows: int, period_ms: float) -> None:
    fieldnames = BASE_REQUIRED_COLUMNS + Q_REQUIRED_COLUMNS + TIMESTAMP_COLUMNS
    start_ns = 1_000_000_000
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for i in range(rows):
            row: dict[str, object] = {
                "period_ms": period_ms,
                "jitter_ms": 0.1,
                "send_skew_us": 10.0,
                "left_send_start_ns": start_ns + int(i * period_ms * 1_000_000) + 100_000,
                "left_send_end_ns": start_ns + int(i * period_ms * 1_000_000) + 150_000,
                "right_send_start_ns": start_ns + int(i * period_ms * 1_000_000) + 110_000,
                "right_send_end_ns": start_ns + int(i * period_ms * 1_000_000) + 165_000,
                "left_send_duration_us": 50.0,
                "right_send_duration_us": 55.0,
                "logger_dropped_samples": 0,
                "left_send_ok": "true",
                "right_send_ok": "true",
                "loop_start_time_ns": start_ns + int(i * period_ms * 1_000_000),
                "loop_end_time_ns": start_ns + int((i + 1) * period_ms * 1_000_000),
            }
            for arm in ("left", "right"):
                for joint in range(6):
                    row[f"{arm}_q_actual_{joint}"] = float(joint)
                    row[f"{arm}_q_sent_{joint}"] = float(joint) + 0.25
            writer.writerow(row)


def run_self_test() -> int:
    with tempfile.TemporaryDirectory() as tmpdir:
        mock_log = Path(tmpdir) / "mock.csv"
        rbsim_local_log = Path(tmpdir) / "rbsim-local.csv"
        rbsim_log = Path(tmpdir) / "rbsim.csv"
        write_sample_csv(mock_log, rows=12_000, period_ms=5.0)
        write_sample_csv(rbsim_local_log, rows=200, period_ms=10.0)
        write_sample_csv(rbsim_log, rows=3_000, period_ms=10.0)
        for profile, path in (
            ("mock200", mock_log),
            ("rbsim-local100", rbsim_local_log),
            ("rbsim100", rbsim_log),
        ):
            metrics = analyze_csv(path)
            failures = check_budget(metrics, BUDGETS[profile])
            if failures:
                print(format_report(metrics, BUDGETS[profile], failures), file=sys.stderr)
                return 1
    print("self-test: PASS")
    return 0


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze logs/servo_log.csv against mock200 or rbsim servo budgets.",
    )
    parser.add_argument("log", nargs="?", default="logs/servo_log.csv", help="CSV log path (default: logs/servo_log.csv)")
    parser.add_argument("--profile", choices=sorted(BUDGETS), required=False, default="mock200")
    parser.add_argument("--json", action="store_true", help="emit JSON metrics/verdict instead of text")
    parser.add_argument("--self-test", action="store_true", help="run an internal standalone self-test and exit")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    if args.self_test:
        return run_self_test()

    budget = BUDGETS[args.profile]
    try:
        metrics = analyze_csv(Path(args.log))
        failures = check_budget(metrics, budget)
    except (OSError, AnalysisError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    if args.json:
        print(json.dumps({"profile": budget.name, "passed": not failures, "failures": failures, "metrics": metrics}, indent=2, sort_keys=True))
    else:
        print(format_report(metrics, budget, failures))
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
