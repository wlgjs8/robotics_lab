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
DEFAULT_STARTUP_TIMEOUT_SEC = 12.0
READONLY_MEASUREMENT_CONFIG = "rb_servo_server/config/dual_real_rbpodo_readonly_measurement.example.yaml"
READONLY_MEASUREMENT_ENDPOINT = "udp://127.0.0.1:50171"
PASS_RESULTS = {"passed", "suspect_but_consistent"}


class StateParityError(RuntimeError):
    pass


class ParityRunFailure(StateParityError):
    def __init__(
        self,
        result: str,
        reason: str,
        *,
        details: dict[str, Any] | None = None,
    ) -> None:
        super().__init__(reason)
        self.result = result
        self.reason = reason
        self.details = details or {}


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
    parser.add_argument("--startup-timeout-sec", type=float, default=DEFAULT_STARTUP_TIMEOUT_SEC)
    parser.add_argument("--state-endpoint", required=True, help=f"UDP endpoint receiving rb_servo_server state JSON, for example {READONLY_MEASUREMENT_ENDPOINT}.")
    parser.add_argument("--artifact-dir", type=Path, required=True)
    parser.add_argument("--tolerance-deg", type=float, default=DEFAULT_TOLERANCE_DEG)
    parser.add_argument("--nearest-max-delta-sec", type=float, default=0.5)
    parser.add_argument("--request-timeout-sec", type=float, default=0.2)
    parser.add_argument(
        "--i-understand-this-connects-to-real-controller",
        action="store_true",
        help="Required before connecting to controller IPs.",
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
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
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
    if not math.isfinite(args.startup_timeout_sec) or args.startup_timeout_sec < 0.0:
        raise StateParityError("--startup-timeout-sec must be finite and non-negative")
    if not math.isfinite(args.tolerance_deg) or args.tolerance_deg < 0.0:
        raise StateParityError("--tolerance-deg must be finite and non-negative")
    if not math.isfinite(args.nearest_max_delta_sec) or args.nearest_max_delta_sec <= 0.0:
        raise StateParityError("--nearest-max-delta-sec must be finite and positive")
    if not args.i_understand_this_connects_to_real_controller:
        real_ips = sorted(set(args.ips) & state_dump.REAL_ROBOT_IPS)
        suffix = f" for {', '.join(real_ips)}" if real_ips else ""
        raise StateParityError(
            "refusing controller connection without "
            f"--i-understand-this-connects-to-real-controller{suffix}"
        )
    if args.server_config is not None:
        if not args.server_config.exists():
            raise StateParityError(f"--server-config does not exist: {args.server_config}")
        send_servo_commands = config_send_servo_commands_value(args.server_config)
        if send_servo_commands is not False:
            raise StateParityError(
                "refusing to use rb_servo_server config because it does not explicitly set "
                "servo.send_servo_commands: false; use the read-only measurement template "
                f"{READONLY_MEASUREMENT_CONFIG}"
            )
    if not args.use_running_server:
        if not args.server or not args.server_config:
            raise StateParityError(
                "--server and --server-config are required unless --use-running-server is set; "
                f"suggested read-only measurement config: {READONLY_MEASUREMENT_CONFIG} "
                f"with --state-endpoint {READONLY_MEASUREMENT_ENDPOINT}"
            )
        if not args.server.exists():
            raise StateParityError(f"--server does not exist: {args.server}")


def start_server(args: argparse.Namespace, log_path: Path) -> subprocess.Popen[bytes] | None:
    if args.use_running_server:
        return None
    assert args.server is not None
    assert args.server_config is not None
    env = os.environ.copy()
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


def tail_lines(path: Path, max_lines: int = 120) -> list[str]:
    try:
        return path.read_text(encoding="utf-8", errors="replace").splitlines()[-max_lines:]
    except OSError:
        return []


def server_returncode_text(process: subprocess.Popen[bytes] | None) -> str:
    if process is None:
        return "not_started"
    returncode = process.poll()
    return str(returncode) if returncode is not None else "still_running"


def startup_failure(
    result: str,
    reason: str,
    *,
    process: subprocess.Popen[bytes] | None,
    log_path: Path,
    state_endpoint: str,
    packet_count: int = 0,
) -> ParityRunFailure:
    details = {
        "state_endpoint": state_endpoint,
        "server_returncode": server_returncode_text(process),
        "state_packets_received": packet_count,
        "server_log_tail": tail_lines(log_path),
    }
    return ParityRunFailure(result, reason, details=details)


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


def raw_values_match(field: str, python_value: Any, cpp_value: Any, nearest_delta_sec: float | None) -> bool:
    if field != "time":
        return python_value == cpp_value
    python_number = state_dump.numeric(python_value)
    cpp_number = state_dump.numeric(cpp_value)
    if python_number is None or cpp_number is None:
        return python_value == cpp_value
    if nearest_delta_sec is None:
        return python_number == cpp_number
    # Python and C++ sample the controller independently. The controller time
    # field should agree within the host-time pairing window, not bit-for-bit.
    return abs(python_number - cpp_number) <= max(nearest_delta_sec + 0.02, 0.05)


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
        "jnt_ref": report["jnt_ref"],
        "jnt_ref_deg": report["jnt_ref_deg"],
        "q_ref_source": report["q_ref_source"],
        "raw": report["raw"],
        "diagnostics_suspect": report["diagnostics_suspect"],
        "diagnostics_suspect_reasons": report["diagnostics_suspect_reasons"],
        "python_time_plausible": raw_time_plausible(report["raw"]),
        "source": "python_rbpodo.CobotData.request_data",
    }


def normalize_cpp_arm_state(
    arm: str,
    arm_state: dict[str, Any],
    fallback_time_ns: int | None = None,
    fault_latched: bool | None = None,
) -> dict[str, Any]:
    diagnostics = arm_state.get("rbpodo_diagnostics")
    diagnostics = diagnostics if isinstance(diagnostics, dict) else {}
    raw = diagnostics.get("raw")
    raw = raw if isinstance(raw, dict) else {}
    q_ref_published = "q_ref_deg" in arm_state
    q_target_published = "q_target_deg" in arm_state
    q_ref = arm_state.get("q_ref_deg")
    q_target = arm_state.get("q_target_deg")
    return {
        "schema": "robotics_lab.rbpodo_cpp_state_sample.v1",
        "arm": arm,
        "host_time_ns": int(arm_state.get("host_time_ns") or fallback_time_ns or 0),
        "q_actual_deg": list_number(arm_state.get("q_actual_deg")),
        "q_ref_deg": list_number(q_ref),
        "q_target_deg": list_number(q_target),
        "jnt_ref_deg": list_number(q_ref),
        "q_ref_published": q_ref_published,
        "q_target_published": q_target_published,
        "q_ref_source": arm_state.get("q_ref_source"),
        "q_ref_valid": bool(arm_state.get("q_ref_valid", False)),
        "q_actual_valid": bool(arm_state.get("q_actual_valid", False)),
        "rbpodo_sdk_state_source": arm_state.get("rbpodo_sdk_state_source"),
        "rbpodo_state_decode_policy": arm_state.get("rbpodo_state_decode_policy"),
        "raw": {field: raw.get(field) for field in RAW_FIELDS},
        "diagnostics_suspect": bool(diagnostics.get("diagnostics_suspect", False)),
        "diagnostics_suspect_reason": diagnostics.get("reason"),
        "fault_latched": bool(fault_latched) if fault_latched is not None else None,
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
    fault_latched = message.get("fault_latched")
    for arm in ARM_NAMES:
        arm_state = message.get(arm)
        if isinstance(arm_state, dict):
            samples.append(normalize_cpp_arm_state(arm, arm_state, fallback_time_ns, bool(fault_latched)))
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
    q_target_diffs: list[float] = []
    python_time_values: list[bool] = []
    cpp_time_values: list[bool] = []
    q_ref_sources_available: list[bool] = []
    any_suspect = False
    any_fault_latched = any(bool(sample.get("fault_latched", False)) for sample in cpp_samples)
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
                "cpp_q_ref_published": None,
            })
            continue

        python_q_actual = python_sample.get("q_actual_deg", [None] * 6)
        python_q_ref = python_sample.get("q_ref_deg", [None] * 6)
        python_q_target = python_sample.get("q_target_deg", [None] * 6)
        cpp_q_actual = cpp_sample.get("q_actual_deg", [None] * 6)
        cpp_q_ref = cpp_sample.get("q_ref_deg", [None] * 6)
        cpp_q_target = cpp_sample.get("q_target_deg", [None] * 6)
        q_actual_diff = max_abs_diff(python_q_actual, cpp_q_actual)
        q_ref_diff = max_abs_diff(python_q_ref, cpp_q_ref)
        q_target_diff = max_abs_diff(python_q_target, cpp_q_target)
        q_ref_alias_diff = max_abs_diff(cpp_q_ref, cpp_q_target)
        if any(value is None for value in python_q_actual) or any(value is None for value in cpp_q_actual):
            mismatch_fields.add(f"{python_sample.get('arm')}.q_actual_deg")
        if not cpp_sample.get("q_ref_published", False):
            mismatch_fields.add(f"{python_sample.get('arm')}.q_ref_not_published")
        if not cpp_sample.get("q_target_published", False):
            mismatch_fields.add(f"{python_sample.get('arm')}.q_target_deg")
        if any(value is None for value in python_q_ref) or any(value is None for value in cpp_q_ref):
            mismatch_fields.add(f"{python_sample.get('arm')}.q_ref_deg")
        if any(value is None for value in python_q_target) or any(value is None for value in cpp_q_target):
            mismatch_fields.add(f"{python_sample.get('arm')}.q_target_deg")
        if q_actual_diff is not None:
            q_actual_diffs.append(q_actual_diff)
            if q_actual_diff > tolerance_deg:
                mismatch_fields.add(f"{python_sample.get('arm')}.q_actual_deg")
        if q_ref_diff is not None:
            q_ref_diffs.append(q_ref_diff)
            if q_ref_diff > tolerance_deg:
                mismatch_fields.add(f"{python_sample.get('arm')}.q_ref_deg")
        if q_target_diff is not None:
            q_target_diffs.append(q_target_diff)
            if q_target_diff > tolerance_deg:
                mismatch_fields.add(f"{python_sample.get('arm')}.q_target_deg")
        if q_ref_alias_diff is None or q_ref_alias_diff > tolerance_deg:
            mismatch_fields.add(f"{python_sample.get('arm')}.q_target_deg/q_ref_deg")

        raw_mismatches: list[str] = []
        raw_matches = 0
        raw_total = 0
        python_raw = python_sample.get("raw", {})
        cpp_raw = cpp_sample.get("raw", {})
        nearest_delta_sec = None if delta_ns is None else delta_ns / 1_000_000_000.0
        for field in RAW_FIELDS:
            raw_total += 1
            raw_total_count += 1
            if raw_values_match(field, python_raw.get(field), cpp_raw.get(field), nearest_delta_sec):
                raw_matches += 1
                raw_match_count += 1
            else:
                raw_mismatches.append(field)
                mismatch_fields.add(f"{python_sample.get('arm')}.raw.{field}")

        python_suspect = bool(python_sample.get("diagnostics_suspect", False))
        cpp_suspect = bool(cpp_sample.get("diagnostics_suspect", False))
        any_fault_latched = any_fault_latched or bool(cpp_sample.get("fault_latched", False))
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
            "q_target_max_abs_diff_deg": q_target_diff,
            "raw_field_matches": raw_matches,
            "raw_field_total": raw_total,
            "raw_mismatch_fields": ",".join(raw_mismatches),
            "python_diagnostics_suspect": python_suspect,
            "cpp_diagnostics_suspect": cpp_suspect,
            "cpp_q_ref_published": cpp_sample.get("q_ref_published", False),
        })

    if not python_samples:
        mismatch_fields.add("python_samples")
    if not cpp_samples:
        mismatch_fields.add("cpp_samples")

    metrics = {
        "max_q_actual_diff_deg": max(q_actual_diffs) if q_actual_diffs else None,
        "max_q_ref_diff_deg": max(q_ref_diffs) if q_ref_diffs else None,
        "max_q_target_diff_deg": max(q_target_diffs) if q_target_diffs else None,
        "raw_field_match_rate": raw_match_count / raw_total_count if raw_total_count else 0.0,
        "diagnostics_suspect_agreement_rate": suspect_agreements / suspect_total if suspect_total else 0.0,
        "python_time_plausible": all(python_time_values) if python_time_values else False,
        "cpp_time_plausible": all(cpp_time_values) if cpp_time_values else False,
        "q_ref_source_available": all(q_ref_sources_available) if q_ref_sources_available else False,
    }

    caveats: list[str] = []
    if mismatch_fields:
        result = "failed_parity_mismatch"
        if any(field.endswith("q_ref_not_published") for field in mismatch_fields):
            caveats.append("q_ref_not_published")
        reason = "mismatched fields: " + ", ".join(sorted(mismatch_fields))
    elif any_suspect:
        result = "suspect_but_consistent"
        reason = "Python and C++ agree, but one or both sides report suspect diagnostics"
        caveats.append("diagnostics_suspect_unresolved")
    else:
        result = "passed"
        reason = "Python rbpodo and C++ rb_servo_server samples agree within tolerance"
    if any_fault_latched:
        caveats.append("parity_suspect")

    summary = {
        "schema": "robotics_lab.rbpodo_state_parity.summary.v1",
        "read_only": True,
        "result": result,
        "reason": reason,
        "caveats": sorted(set(caveats)),
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
        try:
            message = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            continue
        if isinstance(message, dict):
            samples.extend(normalize_cpp_state_message(message))


def wait_for_initial_cpp_state(
    sock: socket.socket,
    timeout_sec: float,
    process: subprocess.Popen[bytes] | None,
    log_path: Path,
    state_endpoint: str,
) -> list[dict[str, Any]]:
    deadline = time.monotonic() + timeout_sec
    invalid_packets = 0
    while time.monotonic() <= deadline:
        timeout = max(0.001, min(0.1, deadline - time.monotonic()))
        sock.settimeout(timeout)
        try:
            payload, _ = sock.recvfrom(1_000_000)
        except (socket.timeout, BlockingIOError):
            if process is not None and process.poll() is not None:
                raise startup_failure(
                    "failed_server_exit",
                    "rb_servo_server exited before publishing a state packet",
                    process=process,
                    log_path=log_path,
                    state_endpoint=state_endpoint,
                    packet_count=0,
                )
            if time.monotonic() >= deadline:
                break
            continue
        try:
            message = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            invalid_packets += 1
            continue
        if not isinstance(message, dict):
            invalid_packets += 1
            continue
        samples = normalize_cpp_state_message(message)
        if samples:
            sock.settimeout(0.02)
            return samples
        invalid_packets += 1
    if process is not None and process.poll() is not None:
        raise startup_failure(
            "failed_server_exit",
            "rb_servo_server exited before publishing a usable state packet",
            process=process,
            log_path=log_path,
            state_endpoint=state_endpoint,
            packet_count=invalid_packets,
        )
    raise startup_failure(
        "failed_transport",
        "no rb_servo_server state packets were received before startup timeout",
        process=process,
        log_path=log_path,
        state_endpoint=state_endpoint,
        packet_count=invalid_packets,
    )


def collect_live_samples(
    args: argparse.Namespace,
    sock: socket.socket,
    initial_cpp_samples: list[dict[str, Any]] | None = None,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    python_samples: list[dict[str, Any]] = []
    cpp_samples: list[dict[str, Any]] = list(initial_cpp_samples or [])
    arm_by_ip = {
        ip: ARM_NAMES[index] if index < len(ARM_NAMES) else f"arm_{index}"
        for index, ip in enumerate(args.ips)
    }
    period_sec = 1.0 / args.sample_rate_hz
    deadline = time.monotonic() + args.duration_sec
    next_python_sample = time.monotonic()
    sock.settimeout(0.02)
    while time.monotonic() < deadline:
        cpp_samples.extend(read_cpp_state_samples(sock))
        now = time.monotonic()
        if now >= next_python_sample:
            for ip in args.ips:
                try:
                    sdata = state_dump.read_controller(ip, args.request_timeout_sec)
                except Exception as exc:
                    raise ParityRunFailure(
                        "failed_transport",
                        f"Python rbpodo state read failed for {ip}: {type(exc).__name__}: {exc}",
                    ) from exc
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
        "q_target_max_abs_diff_deg",
        "raw_field_matches",
        "raw_field_total",
        "raw_mismatch_fields",
        "python_diagnostics_suspect",
        "cpp_diagnostics_suspect",
        "cpp_q_ref_published",
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
        f"Caveats: {', '.join(summary.get('caveats') or []) or 'none'}",
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
    write_jsonl(artifact_dir / "python_samples.jsonl", python_samples)
    write_jsonl(artifact_dir / "cpp_state_samples.jsonl", cpp_samples)
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
    python_samples: list[dict[str, Any]] = []
    cpp_samples: list[dict[str, Any]] = []
    rows: list[dict[str, Any]] = []
    log_path = args.artifact_dir / "rb_servo_server.log"
    try:
        args.artifact_dir.mkdir(parents=True, exist_ok=True)
        with bind_state_socket(args.state_endpoint) as sock:
            process = start_server(args, log_path)
            initial_cpp_samples = wait_for_initial_cpp_state(
                sock,
                args.startup_timeout_sec,
                process,
                log_path,
                args.state_endpoint,
            )
            python_samples, cpp_samples = collect_live_samples(args, sock, initial_cpp_samples)
        summary, rows = compare_samples(
            python_samples,
            cpp_samples,
            tolerance_deg=args.tolerance_deg,
            nearest_max_delta_sec=args.nearest_max_delta_sec,
        )
        summary["state_endpoint"] = args.state_endpoint
        summary["ips"] = list(args.ips)
        summary["artifact_dir"] = str(args.artifact_dir)
        summary["server_returncode"] = server_returncode_text(process)
        write_artifacts(args.artifact_dir, summary, python_samples, cpp_samples, rows)
    except ParityRunFailure as exc:
        summary = {
            "schema": "robotics_lab.rbpodo_state_parity.summary.v1",
            "read_only": True,
            "result": exc.result,
            "reason": exc.reason,
            "caveats": exc.details.get("caveats", []),
            "metrics": {},
            "state_endpoint": args.state_endpoint,
            "ips": list(args.ips),
            "artifact_dir": str(args.artifact_dir),
            **exc.details,
        }
        write_artifacts(args.artifact_dir, summary, python_samples, cpp_samples, rows)
        print(f"rbpodo_state_parity_check: {summary['result']}: {summary['reason']}", file=sys.stderr)
        return 1
    except Exception as exc:
        summary = {
            "schema": "robotics_lab.rbpodo_state_parity.summary.v1",
            "read_only": True,
            "result": "failed_transport",
            "reason": f"{type(exc).__name__}: {exc}",
            "caveats": [],
            "metrics": {},
            "state_endpoint": args.state_endpoint,
            "ips": list(args.ips),
            "artifact_dir": str(args.artifact_dir),
            "server_returncode": server_returncode_text(process),
            "server_log_tail": tail_lines(log_path),
        }
        write_artifacts(args.artifact_dir, summary, python_samples, cpp_samples, rows)
        print(f"rbpodo_state_parity_check: FAIL: {exc}", file=sys.stderr)
        return 1
    finally:
        stop_server(process)

    print(f"rbpodo_state_parity_check: {summary['result']}: {summary['reason']}")
    print(f"artifacts: {args.artifact_dir}")
    return 0 if summary["result"] in PASS_RESULTS else 1


if __name__ == "__main__":
    raise SystemExit(main())
