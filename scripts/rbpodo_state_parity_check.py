#!/usr/bin/env python3
"""Read-only Python-vs-C++ rbpodo state parity checker."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

import rbpodo_state_dump as state_dump


RAW_FIELDS = state_dump.DIAGNOSTIC_FIELDS
ARM_NAMES = ("left", "right")
DEFAULT_TOLERANCE_DEG = 1e-6


class StateParityError(RuntimeError):
    pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare read-only Python rbpodo CobotData samples with C++ "
            "rb_servo_server state JSON. The tool never sends ServoJ, pgmode, "
            "fault reset, or controller-state-changing commands."
        )
    )
    parser.add_argument("--server", type=Path, help="rb_servo_server binary to start for read-only diagnostics.")
    parser.add_argument("--server-config", type=Path, help="rb_servo_server config used when starting --server.")
    parser.add_argument("--use-running-server", action="store_true", help="Do not start a server; only listen to --state-endpoint.")
    parser.add_argument("--ips", nargs="+", required=True, help="Controller IPs to sample through Python rbpodo CobotData.")
    parser.add_argument("--duration-sec", type=float, default=5.0)
    parser.add_argument("--sample-rate-hz", type=float, default=10.0)
    parser.add_argument("--state-endpoint", required=True, help="UDP endpoint receiving rb_servo_server state JSON, for example udp://127.0.0.1:50151.")
    parser.add_argument("--artifact-dir", type=Path, required=True)
    parser.add_argument("--tolerance-deg", type=float, default=DEFAULT_TOLERANCE_DEG)
    parser.add_argument("--nearest-max-delta-sec", type=float, default=0.5)
    parser.add_argument("--request-timeout-sec", type=float, default=0.2)
    parser.add_argument(
        "--i-understand-this-connects-to-real-controller",
        action="store_true",
        help="Required before connecting to known real controller IPs.",
    )
    return parser.parse_args()


def parse_udp_endpoint(endpoint: str) -> tuple[str, int]:
    parsed = urlparse(endpoint)
    if parsed.scheme != "udp" or not parsed.hostname or parsed.port is None:
        raise StateParityError(f"state endpoint must be udp://host:port, got {endpoint!r}")
    return parsed.hostname, parsed.port


def bind_state_socket(endpoint: str) -> socket.socket:
    host, port = parse_udp_endpoint(endpoint)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.02)
    sock.bind((host, port))
    return sock


def config_send_servo_commands_value(config_path: Path) -> bool | None:
    text = config_path.read_text(encoding="utf-8")
    try:
        import yaml  # type: ignore[import-not-found]
    except Exception:
        for line in text.splitlines():
            stripped = line.split("#", 1)[0].strip()
            if stripped.startswith("send_servo_commands:"):
                value = stripped.split(":", 1)[1].strip().lower()
                if value in {"false", "no", "0"}:
                    return False
                if value in {"true", "yes", "1"}:
                    return True
        return None
    loaded = yaml.safe_load(text)
    if not isinstance(loaded, dict):
        return None
    servo = loaded.get("servo")
    if not isinstance(servo, dict):
        return None
    value = servo.get("send_servo_commands")
    return value if isinstance(value, bool) else None


def ensure_safe_args(args: argparse.Namespace) -> None:
    if not math.isfinite(args.duration_sec) or args.duration_sec <= 0.0:
        raise StateParityError("--duration-sec must be finite and positive")
    if not math.isfinite(args.sample_rate_hz) or args.sample_rate_hz <= 0.0:
        raise StateParityError("--sample-rate-hz must be finite and positive")
    if not math.isfinite(args.request_timeout_sec) or args.request_timeout_sec <= 0.0:
        raise StateParityError("--request-timeout-sec must be finite and positive")
    if not math.isfinite(args.tolerance_deg) or args.tolerance_deg < 0.0:
        raise StateParityError("--tolerance-deg must be finite and non-negative")
    if not math.isfinite(args.nearest_max_delta_sec) or args.nearest_max_delta_sec <= 0.0:
        raise StateParityError("--nearest-max-delta-sec must be finite and positive")
    real_ips = sorted(set(args.ips) & state_dump.REAL_ROBOT_IPS)
    if real_ips and not args.i_understand_this_connects_to_real_controller:
        joined = ", ".join(real_ips)
        raise StateParityError(
            "refusing real controller connection without "
            f"--i-understand-this-connects-to-real-controller for {joined}"
        )
    if not args.use_running_server:
        if not args.server or not args.server_config:
            raise StateParityError("--server and --server-config are required unless --use-running-server is set")
        if not args.server.exists():
            raise StateParityError(f"--server does not exist: {args.server}")
        if not args.server_config.exists():
            raise StateParityError(f"--server-config does not exist: {args.server_config}")
        send_servo_commands = config_send_servo_commands_value(args.server_config)
        if send_servo_commands is not False:
            raise StateParityError(
                "refusing to start rb_servo_server because the config does not explicitly set "
                "servo.send_servo_commands: false; use a read-only config or --use-running-server"
            )


def start_server(args: argparse.Namespace, log_path: Path) -> subprocess.Popen[bytes] | None:
    if args.use_running_server:
        return None
    assert args.server is not None
    assert args.server_config is not None
    env = os.environ.copy()
    if set(args.ips) & state_dump.REAL_ROBOT_IPS:
        env["RB_ALLOW_REAL_ROBOT"] = "1"
    command = [str(args.server), "--config", str(args.server_config)]
    log_file = log_path.open("wb")
    process = subprocess.Popen(
        command,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        env=env,
    )
    process._rbpodo_state_parity_log_file = log_file  # type: ignore[attr-defined]
    return process


def stop_server(process: subprocess.Popen[bytes] | None) -> None:
    if process is None:
        return
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5.0)
    log_file = getattr(process, "_rbpodo_state_parity_log_file", None)
    if log_file is not None:
        log_file.close()


def max_abs_diff(lhs: list[float | None], rhs: list[float | None]) -> float | None:
    diffs: list[float] = []
    for left, right in zip(lhs, rhs):
        if left is None or right is None:
            continue
        diffs.append(abs(left - right))
    return max(diffs) if diffs else None


def list_number(value: Any) -> list[float | None]:
    values, _, _ = state_dump.finite_joint_array(value)
    return values


def raw_time_plausible(raw: dict[str, Any]) -> bool:
    value = state_dump.numeric(raw.get("time"))
    return value is not None and value >= 0.0 and not (0.0 < value < 1e-6)


def normalize_python_sdata(ip: str, arm: str, sdata: Any, sample_time_ns: int) -> dict[str, Any]:
    report = state_dump.build_report_for_sdata(ip, sdata, None, None, None)
    return {
        "schema": "robotics_lab.rbpodo_python_state_sample.v1",
        "arm": arm,
        "ip": ip,
        "sample_time_ns": sample_time_ns,
        "q_actual_deg": report["q_actual_deg"],
        "q_ref_deg": report["q_ref_deg"],
        "q_target_deg": report["q_ref_deg"],
        "q_ref_source": report["q_ref_source"],
        "raw": report["raw"],
        "diagnostics_suspect": report["diagnostics_suspect"],
        "diagnostics_suspect_reasons": report["diagnostics_suspect_reasons"],
        "python_time_plausible": raw_time_plausible(report["raw"]),
        "source": "python_rbpodo.CobotData.request_data",
    }


def normalize_cpp_arm_state(arm: str, arm_state: dict[str, Any], fallback_time_ns: int | None = None) -> dict[str, Any]:
    diagnostics = arm_state.get("rbpodo_diagnostics")
    diagnostics = diagnostics if isinstance(diagnostics, dict) else {}
    raw = diagnostics.get("raw")
    raw = raw if isinstance(raw, dict) else {}
    q_ref = arm_state.get("q_ref_deg", arm_state.get("q_target_deg"))
    return {
        "schema": "robotics_lab.rbpodo_cpp_state_sample.v1",
        "arm": arm,
        "host_time_ns": int(arm_state.get("host_time_ns") or fallback_time_ns or 0),
        "q_actual_deg": list_number(arm_state.get("q_actual_deg")),
        "q_ref_deg": list_number(q_ref),
        "q_target_deg": list_number(arm_state.get("q_target_deg", q_ref)),
        "q_ref_source": arm_state.get("q_ref_source"),
        "q_ref_valid": bool(arm_state.get("q_ref_valid", False)),
        "q_actual_valid": bool(arm_state.get("q_actual_valid", False)),
        "rbpodo_sdk_state_source": arm_state.get("rbpodo_sdk_state_source"),
        "rbpodo_state_decode_policy": arm_state.get("rbpodo_state_decode_policy"),
        "raw": {field: raw.get(field) for field in RAW_FIELDS},
        "diagnostics_suspect": bool(diagnostics.get("diagnostics_suspect", False)),
        "diagnostics_suspect_reason": diagnostics.get("reason"),
        "cpp_time_plausible": raw_time_plausible(raw),
        "source": "cpp_rb_servo_server.state_json",
    }


def normalize_cpp_state_message(message: dict[str, Any]) -> list[dict[str, Any]]:
    fallback_time_ns = message.get("host_time_ns")
    if fallback_time_ns is not None:
        try:
            fallback_time_ns = int(fallback_time_ns)
        except (TypeError, ValueError):
            fallback_time_ns = None
    samples: list[dict[str, Any]] = []
    for arm in ARM_NAMES:
        arm_state = message.get(arm)
        if isinstance(arm_state, dict):
            samples.append(normalize_cpp_arm_state(arm, arm_state, fallback_time_ns))
    return samples


def nearest_cpp_sample(
    python_sample: dict[str, Any],
    cpp_samples_by_arm: dict[str, list[dict[str, Any]]],
    max_delta_ns: int,
) -> tuple[dict[str, Any] | None, int | None]:
    arm = python_sample.get("arm")
    python_time_ns = int(python_sample.get("sample_time_ns") or 0)
    candidates = cpp_samples_by_arm.get(str(arm), [])
    best: dict[str, Any] | None = None
    best_delta: int | None = None
    for candidate in candidates:
        cpp_time_ns = int(candidate.get("host_time_ns") or 0)
        if cpp_time_ns <= 0 or python_time_ns <= 0:
            continue
        delta = abs(cpp_time_ns - python_time_ns)
        if best_delta is None or delta < best_delta:
            best = candidate
            best_delta = delta
    if best_delta is None or best_delta > max_delta_ns:
        return None, best_delta
    return best, best_delta


def compare_samples(
    python_samples: list[dict[str, Any]],
    cpp_samples: list[dict[str, Any]],
    tolerance_deg: float = DEFAULT_TOLERANCE_DEG,
    nearest_max_delta_sec: float = 0.5,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    cpp_samples_by_arm: dict[str, list[dict[str, Any]]] = {arm: [] for arm in ARM_NAMES}
    for sample in cpp_samples:
        arm = str(sample.get("arm"))
        cpp_samples_by_arm.setdefault(arm, []).append(sample)

    rows: list[dict[str, Any]] = []
    mismatch_fields: set[str] = set()
    raw_match_count = 0
    raw_total_count = 0
    suspect_agreements = 0
    suspect_total = 0
    q_actual_diffs: list[float] = []
    q_ref_diffs: list[float] = []
    python_time_values: list[bool] = []
    cpp_time_values: list[bool] = []
    q_ref_sources_available: list[bool] = []
    any_suspect = False
    max_delta_ns = int(nearest_max_delta_sec * 1_000_000_000)

    for python_sample in python_samples:
        cpp_sample, delta_ns = nearest_cpp_sample(python_sample, cpp_samples_by_arm, max_delta_ns)
        if cpp_sample is None:
            mismatch_fields.add(f"{python_sample.get('arm')}.sample_time")
            rows.append({
                "arm": python_sample.get("arm"),
                "python_time_ns": python_sample.get("sample_time_ns"),
                "cpp_host_time_ns": None,
                "time_delta_ms": None if delta_ns is None else delta_ns / 1_000_000.0,
                "q_actual_max_abs_diff_deg": None,
                "q_ref_max_abs_diff_deg": None,
                "raw_field_matches": 0,
                "raw_field_total": 0,
                "raw_mismatch_fields": "missing_cpp_sample",
                "python_diagnostics_suspect": python_sample.get("diagnostics_suspect"),
                "cpp_diagnostics_suspect": None,
            })
            continue

        q_actual_diff = max_abs_diff(python_sample["q_actual_deg"], cpp_sample["q_actual_deg"])
        q_ref_diff = max_abs_diff(python_sample["q_ref_deg"], cpp_sample["q_ref_deg"])
        q_ref_alias_diff = max_abs_diff(cpp_sample["q_ref_deg"], cpp_sample["q_target_deg"])
        if any(value is None for value in python_sample["q_actual_deg"]) or any(value is None for value in cpp_sample["q_actual_deg"]):
            mismatch_fields.add(f"{python_sample.get('arm')}.q_actual_deg")
        if any(value is None for value in python_sample["q_ref_deg"]) or any(value is None for value in cpp_sample["q_ref_deg"]):
            mismatch_fields.add(f"{python_sample.get('arm')}.q_ref_deg")
        if q_actual_diff is not None:
            q_actual_diffs.append(q_actual_diff)
            if q_actual_diff > tolerance_deg:
                mismatch_fields.add(f"{python_sample.get('arm')}.q_actual_deg")
        if q_ref_diff is not None:
            q_ref_diffs.append(q_ref_diff)
            if q_ref_diff > tolerance_deg:
                mismatch_fields.add(f"{python_sample.get('arm')}.q_ref_deg")
        if q_ref_alias_diff is None or q_ref_alias_diff > tolerance_deg:
            mismatch_fields.add(f"{python_sample.get('arm')}.q_target_deg/q_ref_deg")

        raw_mismatches: list[str] = []
        raw_matches = 0
        raw_total = 0
        python_raw = python_sample.get("raw", {})
        cpp_raw = cpp_sample.get("raw", {})
        for field in RAW_FIELDS:
            raw_total += 1
            raw_total_count += 1
            if python_raw.get(field) == cpp_raw.get(field):
                raw_matches += 1
                raw_match_count += 1
            else:
                raw_mismatches.append(field)
                mismatch_fields.add(f"{python_sample.get('arm')}.raw.{field}")

        python_suspect = bool(python_sample.get("diagnostics_suspect", False))
        cpp_suspect = bool(cpp_sample.get("diagnostics_suspect", False))
        any_suspect = any_suspect or python_suspect or cpp_suspect
        suspect_total += 1
        if python_suspect == cpp_suspect:
            suspect_agreements += 1
        else:
            mismatch_fields.add(f"{python_sample.get('arm')}.diagnostics_suspect")
        python_time_values.append(bool(python_sample.get("python_time_plausible", raw_time_plausible(python_raw))))
        cpp_time_values.append(bool(cpp_sample.get("cpp_time_plausible", raw_time_plausible(cpp_raw))))
        q_ref_source_available = bool(python_sample.get("q_ref_source")) and bool(cpp_sample.get("q_ref_source"))
        q_ref_sources_available.append(q_ref_source_available)
        if not q_ref_source_available:
            mismatch_fields.add(f"{python_sample.get('arm')}.q_ref_source")

        rows.append({
            "arm": python_sample.get("arm"),
            "python_time_ns": python_sample.get("sample_time_ns"),
            "cpp_host_time_ns": cpp_sample.get("host_time_ns"),
            "time_delta_ms": None if delta_ns is None else delta_ns / 1_000_000.0,
            "q_actual_max_abs_diff_deg": q_actual_diff,
            "q_ref_max_abs_diff_deg": q_ref_diff,
            "raw_field_matches": raw_matches,
            "raw_field_total": raw_total,
            "raw_mismatch_fields": ",".join(raw_mismatches),
            "python_diagnostics_suspect": python_suspect,
            "cpp_diagnostics_suspect": cpp_suspect,
        })

    if not python_samples:
        mismatch_fields.add("python_samples")
    if not cpp_samples:
        mismatch_fields.add("cpp_samples")

    metrics = {
        "max_q_actual_diff_deg": max(q_actual_diffs) if q_actual_diffs else None,
        "max_q_ref_diff_deg": max(q_ref_diffs) if q_ref_diffs else None,
        "raw_field_match_rate": raw_match_count / raw_total_count if raw_total_count else 0.0,
        "diagnostics_suspect_agreement_rate": suspect_agreements / suspect_total if suspect_total else 0.0,
        "python_time_plausible": all(python_time_values) if python_time_values else False,
        "cpp_time_plausible": all(cpp_time_values) if cpp_time_values else False,
        "q_ref_source_available": all(q_ref_sources_available) if q_ref_sources_available else False,
    }

    if mismatch_fields:
        result = "failed"
        reason = "mismatched fields: " + ", ".join(sorted(mismatch_fields))
    elif any_suspect:
        result = "suspect_but_consistent"
        reason = "Python and C++ agree, but one or both sides report suspect diagnostics"
    else:
        result = "passed"
        reason = "Python rbpodo and C++ rb_servo_server samples agree within tolerance"

    summary = {
        "schema": "robotics_lab.rbpodo_state_parity.summary.v1",
        "read_only": True,
        "result": result,
        "reason": reason,
        "metrics": metrics,
        "sample_counts": {
            "python": len(python_samples),
            "cpp_state": len(cpp_samples),
            "matched_pairs": len([row for row in rows if row.get("cpp_host_time_ns") is not None]),
        },
        "tolerance_deg": tolerance_deg,
        "nearest_max_delta_sec": nearest_max_delta_sec,
    }
    return summary, rows


def read_cpp_state_samples(sock: socket.socket) -> list[dict[str, Any]]:
    samples: list[dict[str, Any]] = []
    while True:
        try:
            payload, _ = sock.recvfrom(1_000_000)
        except socket.timeout:
            return samples
        message = json.loads(payload.decode("utf-8"))
        if isinstance(message, dict):
            samples.extend(normalize_cpp_state_message(message))


def collect_live_samples(args: argparse.Namespace) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    python_samples: list[dict[str, Any]] = []
    cpp_samples: list[dict[str, Any]] = []
    arm_by_ip = {
        ip: ARM_NAMES[index] if index < len(ARM_NAMES) else f"arm_{index}"
        for index, ip in enumerate(args.ips)
    }
    period_sec = 1.0 / args.sample_rate_hz
    deadline = time.monotonic() + args.duration_sec
    next_python_sample = time.monotonic()
    with bind_state_socket(args.state_endpoint) as sock:
        while time.monotonic() < deadline:
            cpp_samples.extend(read_cpp_state_samples(sock))
            now = time.monotonic()
            if now >= next_python_sample:
                for ip in args.ips:
                    sdata = state_dump.read_controller(ip, args.request_timeout_sec)
                    python_samples.append(
                        normalize_python_sdata(ip, arm_by_ip[ip], sdata, time.monotonic_ns())
                    )
                next_python_sample = now + period_sec
            time.sleep(0.005)
        cpp_samples.extend(read_cpp_state_samples(sock))
    return python_samples, cpp_samples


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8") as file:
        for row in rows:
            file.write(json.dumps(row, sort_keys=True) + "\n")


def write_parity_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fieldnames = [
        "arm",
        "python_time_ns",
        "cpp_host_time_ns",
        "time_delta_ms",
        "q_actual_max_abs_diff_deg",
        "q_ref_max_abs_diff_deg",
        "raw_field_matches",
        "raw_field_total",
        "raw_mismatch_fields",
        "python_diagnostics_suspect",
        "cpp_diagnostics_suspect",
    ]
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in fieldnames})


def parity_report_markdown(summary: dict[str, Any], rows: list[dict[str, Any]]) -> str:
    metrics = summary["metrics"]
    lines = [
        "# rbpodo State Parity Report",
        "",
        f"Result: `{summary['result']}`",
        f"Reason: {summary['reason']}",
        "",
        "This is read-only measurement evidence. It does not permit physical motion.",
        "",
        "## Metrics",
        "",
    ]
    for key, value in metrics.items():
        lines.append(f"- `{key}`: {value}")
    lines.extend([
        "",
        "## Matched Samples",
        "",
        "| arm | dt ms | q_actual max diff deg | q_ref max diff deg | raw mismatches |",
        "| --- | ---: | ---: | ---: | --- |",
    ])
    for row in rows:
        lines.append(
            f"| {row.get('arm')} | {row.get('time_delta_ms')} | "
            f"{row.get('q_actual_max_abs_diff_deg')} | {row.get('q_ref_max_abs_diff_deg')} | "
            f"{row.get('raw_mismatch_fields')} |"
        )
    return "\n".join(lines) + "\n"


def write_artifacts(
    artifact_dir: Path,
    summary: dict[str, Any],
    python_samples: list[dict[str, Any]],
    cpp_samples: list[dict[str, Any]],
    rows: list[dict[str, Any]],
) -> None:
    artifact_dir.mkdir(parents=True, exist_ok=True)
    write_jsonl(artifact_dir / "samples_python.jsonl", python_samples)
    write_jsonl(artifact_dir / "samples_cpp_state.jsonl", cpp_samples)
    write_parity_csv(artifact_dir / "parity.csv", rows)
    (artifact_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (artifact_dir / "parity_report.md").write_text(
        parity_report_markdown(summary, rows),
        encoding="utf-8",
    )


def main() -> int:
    args = parse_args()
    try:
        ensure_safe_args(args)
    except StateParityError as exc:
        print(f"rbpodo_state_parity_check: FAIL: {exc}", file=sys.stderr)
        return 2

    process: subprocess.Popen[bytes] | None = None
    try:
        args.artifact_dir.mkdir(parents=True, exist_ok=True)
        process = start_server(args, args.artifact_dir / "rb_servo_server.log")
        if process is not None:
            time.sleep(1.0)
            if process.poll() is not None:
                raise StateParityError("rb_servo_server exited before state sampling")
        python_samples, cpp_samples = collect_live_samples(args)
        summary, rows = compare_samples(
            python_samples,
            cpp_samples,
            tolerance_deg=args.tolerance_deg,
            nearest_max_delta_sec=args.nearest_max_delta_sec,
        )
        summary["state_endpoint"] = args.state_endpoint
        summary["ips"] = list(args.ips)
        summary["artifact_dir"] = str(args.artifact_dir)
        write_artifacts(args.artifact_dir, summary, python_samples, cpp_samples, rows)
    except Exception as exc:
        summary = {
            "schema": "robotics_lab.rbpodo_state_parity.summary.v1",
            "read_only": True,
            "result": "failed",
            "reason": f"{type(exc).__name__}: {exc}",
            "metrics": {},
        }
        args.artifact_dir.mkdir(parents=True, exist_ok=True)
        (args.artifact_dir / "summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(f"rbpodo_state_parity_check: FAIL: {exc}", file=sys.stderr)
        return 1
    finally:
        stop_server(process)

    print(f"rbpodo_state_parity_check: {summary['result']}: {summary['reason']}")
    print(f"artifacts: {args.artifact_dir}")
    return 0 if summary["result"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
