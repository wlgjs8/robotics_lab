#!/usr/bin/env python3
"""Explicit Rainbow external command/read-state rate probe."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import rb_backend_ablation as backend_probe


DEFAULT_RATES = "50,75,100,125,150,200"


class RateProbeError(RuntimeError):
    pass


def parse_rates(text: str) -> list[float]:
    rates: list[float] = []
    for raw in text.split(","):
        item = raw.strip()
        if not item:
            continue
        try:
            value = float(item)
        except ValueError as exc:
            raise RateProbeError(f"invalid rate value: {item}") from exc
        if not math.isfinite(value) or value <= 0.0:
            raise RateProbeError(f"rate must be finite and positive: {item}")
        rates.append(value)
    if not rates:
        raise RateProbeError("--rates must contain at least one positive rate")
    return rates


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Sweep Rainbow external command/read-state rates and record ACK, "
            "timeout, error, and loop interval behavior. No motion is sent by default."
        )
    )
    parser.add_argument("--ip", required=True)
    parser.add_argument("--backend", choices=("rbscript_tcp", "rbpodo"), required=True)
    parser.add_argument(
        "--mode",
        choices=("ack_no_motion", "read_state", "servo_j_simulation_only"),
        required=True,
    )
    parser.add_argument("--rates", default=DEFAULT_RATES)
    parser.add_argument("--duration-sec", type=float, default=10.0)
    parser.add_argument("--artifact-dir", type=Path, required=True)
    parser.add_argument("--command-port", type=int, default=5000)
    parser.add_argument("--data-port", type=int, default=5001)
    parser.add_argument("--connect-timeout-sec", type=float, default=1.0)
    parser.add_argument("--read-timeout-sec", type=float, default=0.2)
    parser.add_argument("--command-timeout-sec", type=float, default=0.2)
    parser.add_argument(
        "--rbscript-no-motion-command",
        help="Explicit verified no-motion Rainbow script command for ack_no_motion.",
    )
    parser.add_argument(
        "--i-understand-this-connects-to-real-controller",
        action="store_true",
        help="Required for known real controller IPs.",
    )
    parser.add_argument(
        "--allow-simulation-servo-j",
        action="store_true",
        help="Reserved for a future simulation-only servo_j probe; still rejected in this task.",
    )
    parser.add_argument(
        "--disable-waiting-ack",
        action="store_true",
        help="Rejected by default; included to document that no-ACK rate probes lose immediate ACK/error data.",
    )
    parser.add_argument("--skip-plots", action="store_true")
    return parser.parse_args()


def mode_for_backend_probe(mode: str) -> str:
    if mode == "ack_no_motion":
        return "command_ack_no_motion"
    if mode == "read_state":
        return "read_state"
    return "servo_j_dry_run"


def preflight(args: argparse.Namespace) -> dict[str, Any]:
    rates = parse_rates(args.rates)
    if not math.isfinite(args.duration_sec) or args.duration_sec <= 0.0:
        raise RateProbeError("--duration-sec must be finite and positive")
    if args.disable_waiting_ack:
        raise RateProbeError("--disable-waiting-ack is not supported in RBSCRIPT-RATE-PROBE-01")
    if args.mode == "servo_j_simulation_only":
        raise RateProbeError("servo_j_simulation_only is reserved and not implemented in RBSCRIPT-RATE-PROBE-01")
    if args.mode == "ack_no_motion":
        if args.backend != "rbscript_tcp":
            raise RateProbeError("ack_no_motion is currently implemented only for rbscript_tcp")
        if not args.rbscript_no_motion_command:
            raise RateProbeError("ack_no_motion requires explicit --rbscript-no-motion-command")
        if backend_probe.command_text_looks_motion_capable(args.rbscript_no_motion_command):
            raise RateProbeError("refusing ack_no_motion command text with motion-capable token")

    adapter = SimpleNamespace(
        left_ip=args.ip,
        right_ip=args.ip,
        arm="left",
        backend=args.backend,
        mode=mode_for_backend_probe(args.mode),
        duration_sec=args.duration_sec,
        rate_hz=rates[0],
        artifact_dir=args.artifact_dir,
        command_port=args.command_port,
        data_port=args.data_port,
        connect_timeout_sec=args.connect_timeout_sec,
        read_timeout_sec=args.read_timeout_sec,
        command_timeout_sec=args.command_timeout_sec,
        rbscript_no_motion_command=args.rbscript_no_motion_command,
        allow_motion=False,
        max_delta_deg=None,
        i_understand_this_connects_to_real_controller=args.i_understand_this_connects_to_real_controller,
        skip_plots=args.skip_plots,
    )
    try:
        safety = backend_probe.preflight(adapter)
    except backend_probe.AblationError as exc:
        raise RateProbeError(str(exc)) from exc
    safety["rates"] = rates
    safety["rate_probe_mode"] = args.mode
    safety["disable_waiting_ack"] = False
    return safety


def adapter_for_rate(args: argparse.Namespace, rate: float) -> argparse.Namespace:
    return SimpleNamespace(
        left_ip=args.ip,
        right_ip=args.ip,
        arm="left",
        backend=args.backend,
        mode=mode_for_backend_probe(args.mode),
        duration_sec=args.duration_sec,
        rate_hz=rate,
        artifact_dir=args.artifact_dir,
        command_port=args.command_port,
        data_port=args.data_port,
        connect_timeout_sec=args.connect_timeout_sec,
        read_timeout_sec=args.read_timeout_sec,
        command_timeout_sec=args.command_timeout_sec,
        rbscript_no_motion_command=args.rbscript_no_motion_command,
        allow_motion=False,
        max_delta_deg=None,
        i_understand_this_connects_to_real_controller=args.i_understand_this_connects_to_real_controller,
        skip_plots=args.skip_plots,
    )


def run_rate(args: argparse.Namespace, rate: float) -> tuple[list[backend_probe.Sample], dict[str, Any]]:
    adapter = adapter_for_rate(args, rate)
    safety = {
        "target_ip": args.ip,
        "env": backend_probe.env_snapshot(),
        "safety_mode": "no_motion",
        "target_is_known_real_robot_ip": args.ip in backend_probe.REAL_ROBOT_IPS,
    }
    samples = backend_probe.run_samples(adapter)
    summary = backend_probe.summarize(adapter, safety, samples)
    return samples, summary


def rate_summary_row(summary: dict[str, Any]) -> dict[str, Any]:
    ack = summary.get("command_ack_latency_us")
    read = summary.get("read_duration_us")
    timing = ack if isinstance(ack, dict) and ack.get("p50") is not None else read
    timing = timing if isinstance(timing, dict) else {}
    m_counts = summary.get("m_code_counts") if isinstance(summary.get("m_code_counts"), dict) else {}
    response_errors = summary.get("response_error_names") if isinstance(summary.get("response_error_names"), dict) else {}
    other_error_counts = {
        key: value
        for key, value in response_errors.items()
        if key not in set(backend_probe.M_CODES) and key not in {"TransportTimeout"}
    }
    timeout_count = int(summary.get("command_timeout_count") or summary.get("data_port_timeout_count") or 0)
    send_count = int(summary.get("sample_count") or 0)
    success_count = int(summary.get("success_count") or 0)
    error_count = max(send_count - success_count - timeout_count, 0)
    loop = summary.get("loop_interval_ms") if isinstance(summary.get("loop_interval_ms"), dict) else {}
    return {
        "requested_rate_hz": summary.get("requested_rate_hz"),
        "achieved_rate_hz": summary.get("achieved_rate_hz"),
        "send_count": send_count,
        "ack_success_count": success_count if summary.get("mode") == "command_ack_no_motion" else 0,
        "ack_timeout_count": timeout_count if summary.get("mode") == "command_ack_no_motion" else 0,
        "ack_error_count": error_count if summary.get("mode") == "command_ack_no_motion" else 0,
        "p50_ack_us": timing.get("p50"),
        "p95_ack_us": timing.get("p95"),
        "p99_ack_us": timing.get("p99"),
        "max_ack_us": timing.get("max"),
        "loop_interval_p50_ms": loop.get("p50"),
        "loop_interval_p95_ms": loop.get("p95"),
        "loop_interval_max_ms": loop.get("max"),
        "m561_count": m_counts.get("M561", 0),
        "m568_count": m_counts.get("M568", 0),
        "m569_count": m_counts.get("M569", 0),
        "m570_count": m_counts.get("M570", 0),
        "other_error_counts": other_error_counts,
        "reconnect_count": summary.get("reconnect_count"),
        "data_success_count": summary.get("state_valid_count") if summary.get("mode") == "read_state" else 0,
        "data_timeout_count": summary.get("data_port_timeout_count") if summary.get("mode") == "read_state" else 0,
        "success_rate": summary.get("success_rate"),
    }


def write_samples_csv(path: Path, samples: list[backend_probe.Sample]) -> None:
    fieldnames = [
        "index", "mode", "backend", "success", "duration_us", "start_ns", "end_ns",
        "error_name", "error_message", "state_valid", "q_actual_finite", "state_age_ms",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for sample in samples:
            writer.writerow(backend_probe.sample_row(sample))


def write_responses_jsonl(path: Path, samples: list[backend_probe.Sample]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        for sample in samples:
            if sample.response or sample.error_name:
                handle.write(json.dumps({
                    "index": sample.index,
                    "success": sample.success,
                    "response": sample.response,
                    "error_name": sample.error_name,
                    "error_message": sample.error_message,
                    "response_error_names": sample.response_error_names,
                }) + "\n")


def write_summary_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fieldnames = [
        "requested_rate_hz", "achieved_rate_hz", "send_count", "ack_success_count",
        "ack_timeout_count", "ack_error_count", "p50_ack_us", "p95_ack_us",
        "p99_ack_us", "max_ack_us", "loop_interval_p50_ms", "loop_interval_p95_ms",
        "loop_interval_max_ms", "m561_count", "m568_count", "m569_count",
        "m570_count", "other_error_counts", "reconnect_count", "data_success_count",
        "data_timeout_count", "success_rate",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            flat = dict(row)
            flat["other_error_counts"] = json.dumps(flat["other_error_counts"], sort_keys=True)
            writer.writerow(flat)


def write_plots(artifact_dir: Path, rows: list[dict[str, Any]]) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        (artifact_dir / "plot_skip_reason.txt").write_text(f"matplotlib unavailable: {exc}\n", encoding="utf-8")
        return
    rates = [float(row["requested_rate_hz"]) for row in rows]
    p95 = [row.get("p95_ack_us") for row in rows]
    success = [row.get("success_rate") for row in rows]
    loop_max = [row.get("loop_interval_max_ms") for row in rows]

    plt.figure(figsize=(8, 4))
    plt.plot(rates, p95, marker="o")
    plt.xlabel("requested rate Hz")
    plt.ylabel("p95 latency us")
    plt.tight_layout()
    plt.savefig(artifact_dir / "ack_latency_by_rate.png")
    plt.close()

    plt.figure(figsize=(8, 4))
    plt.plot(rates, success, marker="o")
    plt.xlabel("requested rate Hz")
    plt.ylabel("success rate")
    plt.ylim(-0.05, 1.05)
    plt.tight_layout()
    plt.savefig(artifact_dir / "success_rate_by_rate.png")
    plt.close()

    plt.figure(figsize=(8, 4))
    plt.plot(rates, loop_max, marker="o")
    plt.xlabel("requested rate Hz")
    plt.ylabel("max loop interval ms")
    plt.tight_layout()
    plt.savefig(artifact_dir / "loop_interval_by_rate.png")
    plt.close()


def write_artifacts(
    args: argparse.Namespace,
    safety: dict[str, Any],
    per_rate: list[dict[str, Any]],
    rows: list[dict[str, Any]],
) -> dict[str, Any]:
    artifact_dir = args.artifact_dir.resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    for item in per_rate:
        rate = float(item["rate"])
        rate_label = str(int(rate)) if rate.is_integer() else str(rate).replace(".", "p")
        write_samples_csv(artifact_dir / f"samples_{rate_label}.csv", item["samples"])
        write_responses_jsonl(artifact_dir / f"responses_{rate_label}.jsonl", item["samples"])
    summary = {
        "backend": args.backend,
        "mode": args.mode,
        "ip": args.ip,
        "rates": safety["rates"],
        "duration_sec": args.duration_sec,
        "artifact_dir": str(artifact_dir),
        "safety_preflight": safety,
        "rate_results": rows,
        "result": "completed",
        "caveat": "ACK/read-state rate evidence only; not motion readiness",
    }
    (artifact_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    write_summary_csv(artifact_dir / "summary.csv", rows)
    (artifact_dir / "README.txt").write_text(
        "\n".join([
            "rainbow_rate_probe artifacts",
            "",
            "command: " + " ".join(sys.argv),
            f"backend: {args.backend}",
            f"mode: {args.mode}",
            "No motion is commanded by default. ACK/read success is not motion readiness.",
            "",
        ]),
        encoding="utf-8",
    )
    if not args.skip_plots:
        write_plots(artifact_dir, rows)
    return summary


def main() -> int:
    args = parse_args()
    try:
        safety = preflight(args)
        print(json.dumps({"event": "safety_preflight", **safety}, indent=2, sort_keys=True))
        per_rate: list[dict[str, Any]] = []
        rows: list[dict[str, Any]] = []
        for rate in safety["rates"]:
            samples, summary = run_rate(args, rate)
            row = rate_summary_row(summary)
            per_rate.append({"rate": rate, "samples": samples, "summary": summary})
            rows.append(row)
            print(json.dumps({"event": "rate_complete", **row}, sort_keys=True))
        summary = write_artifacts(args, safety, per_rate, rows)
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 0
    except (RateProbeError, backend_probe.AblationError) as exc:
        print(f"rainbow_rate_probe: FAIL: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
