#!/usr/bin/env python3
"""Offline timestamp alignment and jitter audit for circle benchmark artifacts."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sys
from pathlib import Path
from typing import Any


SCHEMA = "robotics_lab.timestamp_alignment_audit.v1"
COMMAND_MODES = {
    "TcpTwistStand",
    "TcpTwistLocal",
    "TcpLinearMove",
    "TcpCircleMove",
}
ACK_THRESHOLDS_MS = (5, 10, 20, 40)
DEFAULT_SPIKE_WINDOW_MS = 100.0


class AuditError(RuntimeError):
    pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Offline/read-only audit for rbpodo controller-simulation circle "
            "timestamp alignment, jitter tails, ACK waits, state freshness, "
            "and error correlation. This tool reads artifacts only and never "
            "sends robot commands."
        )
    )
    parser.add_argument("--artifact-dir", type=Path, required=True)
    parser.add_argument("--output-md", type=Path, default=Path("alignment_report.md"))
    parser.add_argument("--output-json", type=Path, default=Path("alignment_summary.json"))
    parser.add_argument("--output-csv", type=Path, help="Optional CSV for worst timing/error rows.")
    parser.add_argument("--expected-command-rate-hz", type=float)
    parser.add_argument("--expected-state-rate-hz", type=float)
    parser.add_argument("--spike-window-ms", type=float, default=DEFAULT_SPIKE_WINDOW_MS)
    return parser.parse_args()


def finite_number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        parsed = float(value)
        return parsed if math.isfinite(parsed) else None
    if isinstance(value, str):
        try:
            parsed = float(value)
        except ValueError:
            return None
        return parsed if math.isfinite(parsed) else None
    return None


def int_ns(value: Any) -> int | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value if value >= 0 else None
    if isinstance(value, float) and math.isfinite(value) and value >= 0.0:
        return int(value)
    if isinstance(value, str):
        try:
            parsed = int(value)
        except ValueError:
            return None
        return parsed if parsed >= 0 else None
    return None


def first_non_none(*values: Any) -> Any:
    for value in values:
        if value is not None:
            return value
    return None


def percentile(values: list[float], pct: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * pct / 100.0
    lo = math.floor(index)
    hi = math.ceil(index)
    if lo == hi:
        return ordered[int(index)]
    return ordered[lo] * (hi - index) + ordered[hi] * (index - lo)


def metric_block(values: list[float]) -> dict[str, float | None]:
    return {
        "p50": percentile(values, 50.0),
        "p95": percentile(values, 95.0),
        "p99": percentile(values, 99.0),
        "max": max(values) if values else None,
    }


def load_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise AuditError(f"failed to read {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise AuditError(f"{path} is not a JSON object")
    return value


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.is_file():
        return rows
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                value = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(value, dict):
                rows.append(value)
    return rows


def load_error_samples(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.is_file():
        return rows
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            host_time_ns = int_ns(row.get("host_time_ns"))
            error_m = finite_number(first_non_none(row.get("position_error_m"), row.get("error_m")))
            if host_time_ns is None or error_m is None:
                continue
            rows.append({"host_time_ns": host_time_ns, "error_m": error_m, "row": row})
    rows.sort(key=lambda item: item["host_time_ns"])
    return rows


def selected_arm(summary: dict[str, Any], states: list[dict[str, Any]]) -> str:
    arm = summary.get("arm")
    if arm in {"left", "right"}:
        return str(arm)
    for state in states:
        if isinstance(state.get("left"), dict):
            return "left"
        if isinstance(state.get("right"), dict):
            return "right"
    return "left"


def command_mode(packet: dict[str, Any], arm: str) -> str:
    arm_packet = packet.get(arm)
    if isinstance(arm_packet, dict) and isinstance(arm_packet.get("mode"), str):
        return str(arm_packet["mode"])
    mode = packet.get("mode")
    return str(mode) if isinstance(mode, str) else ""


def command_rows(packets: list[dict[str, Any]], arm: str) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for packet in packets:
        host_time_ns = int_ns(first_non_none(packet.get("host_time_ns"), packet.get("seq")))
        if host_time_ns is None:
            continue
        mode = command_mode(packet, arm)
        if mode not in COMMAND_MODES:
            continue
        rows.append({"host_time_ns": host_time_ns, "mode": mode, "packet": packet})
    if rows:
        return rows
    for packet in packets:
        host_time_ns = int_ns(first_non_none(packet.get("host_time_ns"), packet.get("seq")))
        if host_time_ns is not None:
            rows.append({"host_time_ns": host_time_ns, "mode": command_mode(packet, arm), "packet": packet})
    return rows


def intervals_ms(times_ns: list[int]) -> list[float]:
    return [(b - a) / 1e6 for a, b in zip(times_ns, times_ns[1:]) if b >= a]


def monotonicity(times_ns: list[int]) -> dict[str, Any]:
    violations = sum(1 for a, b in zip(times_ns, times_ns[1:]) if b < a)
    return {
        "monotonic": violations == 0,
        "violation_count": violations,
    }


def expected_interval_ms(rate_hz: Any, intervals: list[float]) -> tuple[float | None, str]:
    rate = finite_number(rate_hz)
    if rate is not None and rate > 0.0:
        return 1000.0 / rate, "configured_rate"
    median = percentile(intervals, 50.0)
    if median is not None and median > 0.0:
        return median, "observed_median"
    return None, "unavailable"


def worst_intervals(rows: list[dict[str, Any]], limit: int = 10) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for previous, current in zip(rows, rows[1:]):
        a = int(previous["host_time_ns"])
        b = int(current["host_time_ns"])
        if b < a:
            continue
        out.append({
            "start_host_time_ns": a,
            "end_host_time_ns": b,
            "interval_ms": (b - a) / 1e6,
            "mode": current.get("mode"),
        })
    out.sort(key=lambda item: item["interval_ms"], reverse=True)
    return out[:limit]


def command_generation_summary(
    commands: list[dict[str, Any]],
    expected_rate_hz: Any,
) -> tuple[dict[str, Any], list[int], list[dict[str, Any]]]:
    times = [int(row["host_time_ns"]) for row in commands]
    intervals = intervals_ms(times)
    expected_ms, expected_source = expected_interval_ms(expected_rate_hz, intervals)
    over_budget = 0
    gap_times: list[int] = []
    if expected_ms is not None:
        over_budget = sum(1 for value in intervals if value > expected_ms * 1.5)
        for index, value in enumerate(intervals):
            if value > expected_ms * 2.0:
                gap_times.append(times[index + 1])
    return (
        {
            "sample_count": len(commands),
            "expected_interval_ms": expected_ms,
            "expected_interval_source": expected_source,
            "command_interval_ms": metric_block(intervals),
            "command_interval_over_budget_count": over_budget,
            "command_gap_count": len(gap_times),
            "command_host_time_monotonicity": monotonicity(times),
        },
        gap_times,
        worst_intervals(commands),
    )


def state_rows(states: list[dict[str, Any]], arm: str) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for state in states:
        host_time_ns = int_ns(first_non_none(state.get("host_time_ns"), state.get("loop_end_time_ns")))
        if host_time_ns is None:
            continue
        arm_state = state.get(arm)
        if not isinstance(arm_state, dict):
            arm_state = {}
        last_send = arm_state.get("last_send")
        if not isinstance(last_send, dict):
            last_send = {}
        rows.append({
            "host_time_ns": host_time_ns,
            "state": state,
            "arm_state": arm_state,
            "last_send": last_send,
        })
    rows.sort(key=lambda item: item["host_time_ns"])
    return rows


def numeric_series(rows: list[dict[str, Any]], getter) -> list[float]:
    values: list[float] = []
    for row in rows:
        value = finite_number(getter(row))
        if value is not None:
            values.append(value)
    return values


def state_stream_summary(
    rows: list[dict[str, Any]],
    expected_rate_hz: Any,
) -> tuple[dict[str, Any], list[int]]:
    times = [int(row["host_time_ns"]) for row in rows]
    intervals = intervals_ms(times)
    expected_ms, expected_source = expected_interval_ms(expected_rate_hz, intervals)
    gap_times: list[int] = []
    if expected_ms is not None:
        for index, value in enumerate(intervals):
            if value > expected_ms * 2.0:
                gap_times.append(times[index + 1])
    state_ages = numeric_series(rows, lambda row: row["arm_state"].get("state_age_us"))
    return (
        {
            "sample_count": len(rows),
            "expected_interval_ms": expected_ms,
            "expected_interval_source": expected_source,
            "state_interval_ms": metric_block(intervals),
            "state_age_us": metric_block(state_ages),
            "state_drop_or_gap_count": len(gap_times),
            "state_host_time_monotonicity": monotonicity(times),
        },
        gap_times,
    )


def send_ack_summary(rows: list[dict[str, Any]]) -> tuple[dict[str, Any], list[int], list[dict[str, Any]]]:
    send_durations = numeric_series(
        rows,
        lambda row: first_non_none(row["last_send"].get("duration_us"), row["last_send"].get("send_duration_us")),
    )
    ack_waits = numeric_series(rows, lambda row: row["last_send"].get("ack_wait_duration_us"))
    spike_times_by_threshold: dict[str, list[int]] = {}
    for threshold_ms in ACK_THRESHOLDS_MS:
        key = f"ack_spike_times_{threshold_ms}ms"
        spike_times_by_threshold[key] = [
            int(row["host_time_ns"])
            for row in rows
            if (finite_number(row["last_send"].get("ack_wait_duration_us")) or 0.0) >= threshold_ms * 1000.0
        ]
    last_send_rows = [row for row in rows if row["last_send"]]
    acceptance_count = sum(1 for row in last_send_rows if row["last_send"].get("controller_acceptance_observed") is True)
    ack_observed_count = sum(1 for row in last_send_rows if row["last_send"].get("ack_observed") is True)
    worst_ack: list[dict[str, Any]] = []
    for row in rows:
        value = finite_number(row["last_send"].get("ack_wait_duration_us"))
        if value is None:
            continue
        worst_ack.append({
            "host_time_ns": int(row["host_time_ns"]),
            "ack_wait_duration_us": value,
            "send_duration_us": finite_number(
                first_non_none(row["last_send"].get("duration_us"), row["last_send"].get("send_duration_us"))
            ),
            "controller_acceptance_observed": row["last_send"].get("controller_acceptance_observed"),
            "ack_observed": row["last_send"].get("ack_observed"),
        })
    worst_ack.sort(key=lambda item: item["ack_wait_duration_us"], reverse=True)
    summary: dict[str, Any] = {
        "send_duration_us": metric_block(send_durations),
        "ack_wait_duration_us": metric_block(ack_waits),
        "ack_observed_count": ack_observed_count,
        "controller_acceptance_observed_count": acceptance_count,
        "controller_acceptance_observed_ratio": (
            acceptance_count / len(last_send_rows) if last_send_rows else None
        ),
    }
    ack_spike_times_10ms = spike_times_by_threshold["ack_spike_times_10ms"]
    for threshold_ms in ACK_THRESHOLDS_MS:
        key = f"ack_spike_count_{threshold_ms}ms"
        summary[key] = len(spike_times_by_threshold[f"ack_spike_times_{threshold_ms}ms"])
    return summary, ack_spike_times_10ms, worst_ack[:10]


def nearest_delta_ms(times: list[int], target_ns: int) -> float | None:
    if not times:
        return None
    best = min(times, key=lambda value: abs(value - target_ns))
    return (target_ns - best) / 1e6


def nearest_command_summary(state_times: list[int], command_times: list[int]) -> dict[str, Any]:
    deltas = [abs(delta) for delta in (nearest_delta_ms(command_times, t) for t in state_times) if delta is not None]
    signed = [delta for delta in (nearest_delta_ms(command_times, t) for t in state_times) if delta is not None]
    return {
        "nearest_command_delta_abs_ms": metric_block(deltas),
        "nearest_command_delta_signed_ms": metric_block(signed),
    }


def within_window(sample_time_ns: int, event_times_ns: list[int], window_ns: int) -> bool:
    return any(abs(sample_time_ns - event_time_ns) <= window_ns for event_time_ns in event_times_ns)


def p95_near_away(
    samples: list[dict[str, Any]],
    event_times_ns: list[int],
    window_ns: int,
) -> tuple[float | None, float | None, int, int]:
    near: list[float] = []
    away: list[float] = []
    for sample in samples:
        target = near if within_window(int(sample["host_time_ns"]), event_times_ns, window_ns) else away
        target.append(float(sample["error_m"]))
    return percentile(near, 95.0), percentile(away, 95.0), len(near), len(away)


def feedback_rows(path: Path) -> list[dict[str, Any]]:
    return load_jsonl(path)


def feedback_summary(rows: list[dict[str, Any]]) -> tuple[dict[str, Any], list[int]]:
    state_ages = [value for value in (finite_number(row.get("feedback_state_age_us")) for row in rows) if value is not None]
    stale_rows = [row for row in rows if row.get("stale_or_invalid_state") is True]
    stale_times = [value for value in (int_ns(row.get("host_time_ns")) for row in stale_rows) if value is not None]
    use_current_state_time = None
    for row in rows:
        if "feedback_use_current_state_time" in row:
            use_current_state_time = row.get("feedback_use_current_state_time")
            break
    return (
        {
            "feedback_state_age_us": metric_block(state_ages),
            "stale_state_feedback_skips": len(stale_rows),
            "feedback_use_current_state_time": use_current_state_time,
            "sample_count": len(rows),
        },
        stale_times,
    )


def overlay_summary(overlay_rows: list[dict[str, Any]], state_times: list[int]) -> dict[str, Any]:
    times = [value for value in (int_ns(row.get("host_time_ns")) for row in overlay_rows) if value is not None]
    intervals = intervals_ms(times)
    skews = [abs(nearest_delta_ms(state_times, time_ns) or 0.0) for time_ns in times if state_times]
    return {
        "available": bool(overlay_rows),
        "sample_count": len(overlay_rows),
        "overlay_interval_ms": metric_block(intervals),
        "overlay_state_time_skew_ms": metric_block(skews),
    }


def classify_timing(
    command_summary: dict[str, Any],
    state_summary: dict[str, Any],
    ack_summary: dict[str, Any],
) -> tuple[str, list[str], str]:
    reasons: list[str] = []
    command_max = finite_number(command_summary.get("command_interval_ms", {}).get("max"))
    command_expected = finite_number(command_summary.get("expected_interval_ms"))
    command_gaps = int(command_summary.get("command_gap_count") or 0)
    state_gaps = int(state_summary.get("state_drop_or_gap_count") or 0)
    ack_20 = int(ack_summary.get("ack_spike_count_20ms") or 0)
    ack_10 = int(ack_summary.get("ack_spike_count_10ms") or 0)
    ack_5 = int(ack_summary.get("ack_spike_count_5ms") or 0)

    if (
        command_gaps > 0
        and command_max is not None
        and command_expected is not None
        and command_max >= max(100.0, 5.0 * command_expected)
    ):
        reasons.append(f"command interval max {command_max:.3f} ms exceeds severe gap threshold")
        return "command_generation_limited", reasons, "fix jitter before gain tuning; consider moving feedback generation server-side"
    if ack_20 > 0:
        reasons.append(f"ACK waits over 20 ms observed: {ack_20}")
        return "ack_spike_limited", reasons, "inspect ACK wait spikes; consider an explicit ACK-off controller-simulation experiment"
    if state_gaps > 0:
        reasons.append(f"state publish gaps over 2x expected observed: {state_gaps}")
        return "state_publish_limited", reasons, "increase state_pub_rate or inspect state fanout/server loop timing"
    if command_gaps > 0 or ack_10 > 0 or ack_5 > 0 or int(command_summary.get("command_interval_over_budget_count") or 0) > 0:
        if command_gaps > 0:
            reasons.append(f"command gaps over 2x expected observed: {command_gaps}")
        if ack_10 > 0:
            reasons.append(f"ACK waits over 10 ms observed: {ack_10}")
        elif ack_5 > 0:
            reasons.append(f"ACK waits over 5 ms observed: {ack_5}")
        return "jitter_limited", reasons, "fix timing jitter before interpreting tail errors as controller tracking behavior"
    reasons.append("no command, state publish, or ACK timing spikes above audit thresholds")
    return "clean_timing", reasons, "tune gain or decompose geometric tracking error if error tails remain"


def correlation_summary(
    error_samples: list[dict[str, Any]],
    ack_spike_times_ns: list[int],
    command_gap_times_ns: list[int],
    stale_times_ns: list[int],
    state_times_ns: list[int],
    command_times_ns: list[int],
    window_ms: float,
) -> dict[str, Any]:
    window_ns = int(window_ms * 1e6)
    ack_near, ack_away, ack_near_count, ack_away_count = p95_near_away(error_samples, ack_spike_times_ns, window_ns)
    cmd_near, cmd_away, cmd_near_count, cmd_away_count = p95_near_away(error_samples, command_gap_times_ns, window_ns)
    stale_near, stale_away, stale_near_count, stale_away_count = p95_near_away(error_samples, stale_times_ns, window_ns)
    all_spikes = sorted(set(ack_spike_times_ns + command_gap_times_ns + stale_times_ns))
    worst_errors: list[dict[str, Any]] = []
    for sample in sorted(error_samples, key=lambda item: item["error_m"], reverse=True)[:10]:
        sample_time = int(sample["host_time_ns"])
        nearest_spike_delta_ms = nearest_delta_ms(all_spikes, sample_time)
        worst_errors.append({
            "host_time_ns": sample_time,
            "error_m": float(sample["error_m"]),
            "near_timing_spike": within_window(sample_time, all_spikes, window_ns),
            "nearest_spike_delta_ms": nearest_spike_delta_ms,
        })
    return {
        "spike_window_ms": window_ms,
        "nearest_command_to_state": nearest_command_summary(state_times_ns, command_times_ns),
        "p95_error_near_ack_spike_m": ack_near,
        "p95_error_away_from_ack_spike_m": ack_away,
        "error_sample_count_near_ack_spike": ack_near_count,
        "error_sample_count_away_from_ack_spike": ack_away_count,
        "p95_error_near_command_gap_m": cmd_near,
        "p95_error_away_from_command_gap_m": cmd_away,
        "error_sample_count_near_command_gap": cmd_near_count,
        "error_sample_count_away_from_command_gap": cmd_away_count,
        "p95_error_near_stale_state_feedback_skip_m": stale_near,
        "p95_error_away_from_stale_state_feedback_skip_m": stale_away,
        "error_sample_count_near_stale_state_feedback_skip": stale_near_count,
        "error_sample_count_away_from_stale_state_feedback_skip": stale_away_count,
        "worst_error_alignment": worst_errors,
    }


def find_server_log_events(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {"available": False, "warning_count": 0, "error_count": 0}
    warning_count = 0
    error_count = 0
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if re.search(r"\bWARN(ING)?\b", line, re.IGNORECASE):
            warning_count += 1
        if re.search(r"\bERROR\b|\bFAIL", line, re.IGNORECASE):
            error_count += 1
    return {"available": True, "warning_count": warning_count, "error_count": error_count}


def resolve_output_path(artifact_dir: Path, path: Path | None) -> Path | None:
    if path is None:
        return None
    return path if path.is_absolute() else artifact_dir / path


def benchmark_timestamp_alignment_block(audit: dict[str, Any]) -> dict[str, Any]:
    command = audit.get("command_generation", {})
    state = audit.get("state_stream", {})
    ack = audit.get("server_send_ack", {})
    classification = audit.get("classification", {})
    return {
        "command_interval_ms": command.get("command_interval_ms"),
        "state_interval_ms": state.get("state_interval_ms"),
        "state_age_us": state.get("state_age_us"),
        "send_duration_us": ack.get("send_duration_us"),
        "ack_wait_duration_us": ack.get("ack_wait_duration_us"),
        "ack_spike_count_10ms": ack.get("ack_spike_count_10ms"),
        "ack_spike_count_20ms": ack.get("ack_spike_count_20ms"),
        "state_gap_count": state.get("state_drop_or_gap_count"),
        "command_gap_count": command.get("command_gap_count"),
        "timing_classification": classification.get("timing_classification"),
    }


def benchmark_tail_error_correlation_block(audit: dict[str, Any]) -> dict[str, Any]:
    correlation = audit.get("correlation", {})
    return {
        "p95_error_near_ack_spike_m": correlation.get("p95_error_near_ack_spike_m"),
        "p95_error_away_from_ack_spike_m": correlation.get("p95_error_away_from_ack_spike_m"),
        "p95_error_near_command_gap_m": correlation.get("p95_error_near_command_gap_m"),
        "p95_error_away_from_command_gap_m": correlation.get("p95_error_away_from_command_gap_m"),
    }


def audit_artifact_dir(
    artifact_dir: Path,
    *,
    summary: dict[str, Any] | None = None,
    expected_command_rate_hz: float | None = None,
    expected_state_rate_hz: float | None = None,
    spike_window_ms: float = DEFAULT_SPIKE_WINDOW_MS,
    output_md_path: Path | None = None,
    output_json_path: Path | None = None,
    output_csv_path: Path | None = None,
) -> dict[str, Any]:
    artifact_dir = artifact_dir.resolve()
    if not artifact_dir.is_dir():
        raise AuditError(f"artifact dir not found: {artifact_dir}")
    if not math.isfinite(spike_window_ms) or spike_window_ms <= 0.0:
        raise AuditError("--spike-window-ms must be finite and positive")

    summary_path = artifact_dir / "summary.json"
    loaded_summary = dict(summary) if summary is not None else load_json(summary_path)
    states = load_jsonl(artifact_dir / "state_stream.jsonl")
    arm = selected_arm(loaded_summary, states)
    commands = command_rows(load_jsonl(artifact_dir / "command_packets.jsonl"), arm)
    state_records = state_rows(states, arm)
    overlay_rows = load_jsonl(artifact_dir / "overlay_stream.jsonl")
    feedback = feedback_rows(artifact_dir / "feedback_terms.jsonl")
    error_samples = load_error_samples(artifact_dir / "samples.csv")

    command_rate = expected_command_rate_hz
    if command_rate is None:
        command_rate = finite_number(loaded_summary.get("command_rate_hz"))
    state_rate = expected_state_rate_hz
    if state_rate is None:
        state_rate = finite_number(loaded_summary.get("state_pub_rate_hz"))

    command_summary, command_gap_times, worst_command_gaps = command_generation_summary(commands, command_rate)
    state_summary, state_gap_times = state_stream_summary(state_records, state_rate)
    ack_summary, ack_spike_times, worst_ack_waits = send_ack_summary(state_records)
    feedback_freshness, stale_times = feedback_summary(feedback)
    if feedback_freshness.get("feedback_use_current_state_time") is None:
        feedback_freshness["feedback_use_current_state_time"] = loaded_summary.get("feedback_use_current_state_time")
    state_times = [int(row["host_time_ns"]) for row in state_records]
    command_times = [int(row["host_time_ns"]) for row in commands]
    correlation = correlation_summary(
        error_samples,
        ack_spike_times,
        command_gap_times,
        stale_times,
        state_times,
        command_times,
        spike_window_ms,
    )
    timing_classification, reasons, recommendation = classify_timing(command_summary, state_summary, ack_summary)
    audit = {
        "schema": SCHEMA,
        "read_only": True,
        "artifact_dir": str(artifact_dir),
        "arm": arm,
        "input_paths": {
            "summary": str(summary_path),
            "state_stream": str(artifact_dir / "state_stream.jsonl"),
            "command_packets": str(artifact_dir / "command_packets.jsonl"),
            "overlay_stream": str(artifact_dir / "overlay_stream.jsonl") if (artifact_dir / "overlay_stream.jsonl").is_file() else None,
            "samples_csv": str(artifact_dir / "samples.csv") if (artifact_dir / "samples.csv").is_file() else None,
            "feedback_terms": str(artifact_dir / "feedback_terms.jsonl") if (artifact_dir / "feedback_terms.jsonl").is_file() else None,
            "rb_servo_server_log": str(artifact_dir / "rb_servo_server.log") if (artifact_dir / "rb_servo_server.log").is_file() else None,
        },
        "command_generation": command_summary,
        "state_stream": state_summary,
        "server_send_ack": ack_summary,
        "correlation": correlation,
        "feedback_freshness": feedback_freshness,
        "overlay": overlay_summary(overlay_rows, state_times),
        "server_log": find_server_log_events(artifact_dir / "rb_servo_server.log"),
        "classification": {
            "timing_classification": timing_classification,
            "reasons": reasons,
            "recommended_next_action": recommendation,
        },
        "top_10_worst_command_gaps": worst_command_gaps,
        "top_10_worst_ack_waits": worst_ack_waits,
        "state_gap_times_ns": state_gap_times[:100],
        "command_gap_times_ns": command_gap_times[:100],
        "ack_spike_times_10ms_ns": ack_spike_times[:100],
        "stale_state_feedback_skip_times_ns": stale_times[:100],
        "benchmark_timestamp_alignment": {},
        "benchmark_tail_error_correlation": {},
    }
    audit["benchmark_timestamp_alignment"] = benchmark_timestamp_alignment_block(audit)
    audit["benchmark_tail_error_correlation"] = benchmark_tail_error_correlation_block(audit)

    if output_json_path is not None:
        output_json_path.parent.mkdir(parents=True, exist_ok=True)
        output_json_path.write_text(json.dumps(audit, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if output_md_path is not None:
        output_md_path.parent.mkdir(parents=True, exist_ok=True)
        output_md_path.write_text(render_markdown_report(audit), encoding="utf-8")
    if output_csv_path is not None:
        write_csv_report(output_csv_path, audit)
    return audit


def fmt(value: Any, digits: int = 3) -> str:
    number = finite_number(value)
    if number is None:
        return ""
    return f"{number:.{digits}f}"


def metric_row(label: str, block: dict[str, Any], unit: str = "") -> str:
    return (
        f"| {label} | {fmt(block.get('p50'))} | {fmt(block.get('p95'))} | "
        f"{fmt(block.get('p99'))} | {fmt(block.get('max'))} | {unit} |"
    )


def render_markdown_report(audit: dict[str, Any]) -> str:
    command = audit.get("command_generation", {})
    state = audit.get("state_stream", {})
    ack = audit.get("server_send_ack", {})
    feedback = audit.get("feedback_freshness", {})
    overlay = audit.get("overlay", {})
    classification = audit.get("classification", {})
    correlation = audit.get("correlation", {})
    parts = [
        "# Timestamp Alignment Audit",
        "",
        "Offline/read-only artifact audit. This report does not validate motion safety or mark suspicious measurements reliable.",
        "",
        "## Classification",
        "",
        f"- timing_classification: `{classification.get('timing_classification')}`",
        f"- recommended_next_action: {classification.get('recommended_next_action')}",
        "",
        "Reasons:",
        "",
    ]
    reasons = classification.get("reasons")
    if isinstance(reasons, list) and reasons:
        parts.extend(f"- {reason}" for reason in reasons)
    else:
        parts.append("- _None._")
    parts.extend([
        "",
        "## Timing Histogram Table",
        "",
        "| metric | p50 | p95 | p99 | max | unit |",
        "| --- | ---: | ---: | ---: | ---: | --- |",
        metric_row("command_interval", command.get("command_interval_ms") or {}, "ms"),
        metric_row("state_interval", state.get("state_interval_ms") or {}, "ms"),
        metric_row("state_age", state.get("state_age_us") or {}, "us"),
        metric_row("send_duration", ack.get("send_duration_us") or {}, "us"),
        metric_row("ack_wait", ack.get("ack_wait_duration_us") or {}, "us"),
        metric_row("feedback_state_age", feedback.get("feedback_state_age_us") or {}, "us"),
        metric_row("overlay_interval", overlay.get("overlay_interval_ms") or {}, "ms"),
        metric_row("overlay_state_skew", overlay.get("overlay_state_time_skew_ms") or {}, "ms"),
        "",
        "## Counts",
        "",
        f"- command_interval_over_budget_count: {command.get('command_interval_over_budget_count')}",
        f"- command_gap_count: {command.get('command_gap_count')}",
        f"- state_drop_or_gap_count: {state.get('state_drop_or_gap_count')}",
        f"- ack_spike_count_5ms: {ack.get('ack_spike_count_5ms')}",
        f"- ack_spike_count_10ms: {ack.get('ack_spike_count_10ms')}",
        f"- ack_spike_count_20ms: {ack.get('ack_spike_count_20ms')}",
        f"- ack_spike_count_40ms: {ack.get('ack_spike_count_40ms')}",
        f"- stale_state_feedback_skips: {feedback.get('stale_state_feedback_skips')}",
        f"- controller_acceptance_observed_ratio: {fmt(ack.get('controller_acceptance_observed_ratio'))}",
        "",
        "## Tail Error Correlation",
        "",
        f"- p95_error_near_ack_spike_m: {fmt(correlation.get('p95_error_near_ack_spike_m'), 6)}",
        f"- p95_error_away_from_ack_spike_m: {fmt(correlation.get('p95_error_away_from_ack_spike_m'), 6)}",
        f"- p95_error_near_command_gap_m: {fmt(correlation.get('p95_error_near_command_gap_m'), 6)}",
        f"- p95_error_away_from_command_gap_m: {fmt(correlation.get('p95_error_away_from_command_gap_m'), 6)}",
        f"- p95_error_near_stale_state_feedback_skip_m: {fmt(correlation.get('p95_error_near_stale_state_feedback_skip_m'), 6)}",
        "",
        "## Top 10 Worst Command Gaps",
        "",
        "| start_host_time_ns | end_host_time_ns | interval_ms | mode |",
        "| ---: | ---: | ---: | --- |",
    ])
    for row in audit.get("top_10_worst_command_gaps", []):
        parts.append(
            f"| {row.get('start_host_time_ns')} | {row.get('end_host_time_ns')} | "
            f"{fmt(row.get('interval_ms'))} | {row.get('mode') or ''} |"
        )
    if not audit.get("top_10_worst_command_gaps"):
        parts.append("|  |  |  |  |")
    parts.extend([
        "",
        "## Top 10 Worst ACK Waits",
        "",
        "| host_time_ns | ack_wait_duration_us | send_duration_us | controller_acceptance_observed |",
        "| ---: | ---: | ---: | --- |",
    ])
    for row in audit.get("top_10_worst_ack_waits", []):
        parts.append(
            f"| {row.get('host_time_ns')} | {fmt(row.get('ack_wait_duration_us'))} | "
            f"{fmt(row.get('send_duration_us'))} | {row.get('controller_acceptance_observed')} |"
        )
    if not audit.get("top_10_worst_ack_waits"):
        parts.append("|  |  |  |  |")
    parts.extend([
        "",
        "## Worst Error Samples",
        "",
        "| host_time_ns | error_m | near_timing_spike | nearest_spike_delta_ms |",
        "| ---: | ---: | --- | ---: |",
    ])
    for row in correlation.get("worst_error_alignment", []):
        parts.append(
            f"| {row.get('host_time_ns')} | {fmt(row.get('error_m'), 6)} | "
            f"{row.get('near_timing_spike')} | {fmt(row.get('nearest_spike_delta_ms'))} |"
        )
    if not correlation.get("worst_error_alignment"):
        parts.append("|  |  |  |  |")
    return "\n".join(parts) + "\n"


def write_csv_report(path: Path, audit: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, Any]] = []
    for row in audit.get("top_10_worst_command_gaps", []):
        item = dict(row)
        item["kind"] = "command_gap"
        rows.append(item)
    for row in audit.get("top_10_worst_ack_waits", []):
        item = dict(row)
        item["kind"] = "ack_wait"
        rows.append(item)
    fieldnames: list[str] = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    if not fieldnames:
        fieldnames = ["kind"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key) for key in fieldnames})


def main() -> int:
    args = parse_args()
    artifact_dir = args.artifact_dir.resolve()
    output_md = resolve_output_path(artifact_dir, args.output_md)
    output_json = resolve_output_path(artifact_dir, args.output_json)
    output_csv = resolve_output_path(artifact_dir, args.output_csv)
    try:
        audit = audit_artifact_dir(
            artifact_dir,
            expected_command_rate_hz=args.expected_command_rate_hz,
            expected_state_rate_hz=args.expected_state_rate_hz,
            spike_window_ms=args.spike_window_ms,
            output_md_path=output_md,
            output_json_path=output_json,
            output_csv_path=output_csv,
        )
    except Exception as exc:
        print(f"timestamp_alignment_audit: FAIL: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(audit, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
