#!/usr/bin/env python3
"""Read-only Rainbow TCP 5001 data-port raw capture."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import socket
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

import rbpodo_state_dump


DEFAULT_DATA_PORT = 5001
DEFAULT_REQUEST_PAYLOAD = "reqdata"
DEFAULT_MAX_BYTES_PER_SAMPLE = 65536
PREFIX_BYTES = 64


class CaptureError(RuntimeError):
    pass


@dataclass
class CaptureConfig:
    ips: list[str]
    port: int
    duration_sec: float
    rate_hz: float
    request_payload: str
    timeout_sec: float
    include_hex: bool
    max_bytes_per_sample: int
    also_rbpodo_python: bool
    output_prefix: str
    artifact_dir: Path
    confirmed_real_controller: bool


ConnectFn = Callable[[tuple[str, int], float], Any]
ReadPythonSDataFn = Callable[[str, float], Any]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Capture raw Rainbow TCP data-port responses for measurement "
            "reliability investigation. This tool only connects to data port "
            "5001, sends the read-only reqdata request by default, and never "
            "sends motion, pgmode, reset, or command-port traffic. Known real "
            "controller IPs also require RB_ALLOW_REAL_ROBOT=1."
        )
    )
    parser.add_argument("--ip", help="Single controller IP to capture.")
    parser.add_argument("--ips", nargs="+", help="One or more controller IPs to capture.")
    parser.add_argument("--port", type=int, default=DEFAULT_DATA_PORT)
    parser.add_argument("--duration-sec", type=float, default=5.0)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--request-payload", default=DEFAULT_REQUEST_PAYLOAD)
    parser.add_argument("--timeout-sec", type=float, default=0.2)
    parser.add_argument(
        "--hex",
        action="store_true",
        help="Include the full response hex string in samples.jsonl. Binary files are always written.",
    )
    parser.add_argument("--max-bytes-per-sample", type=int, default=DEFAULT_MAX_BYTES_PER_SAMPLE)
    parser.add_argument(
        "--also-rbpodo-python",
        action="store_true",
        help="Also sample rbpodo.CobotData.request_data and write python_decoded_samples.jsonl.",
    )
    parser.add_argument("--output-prefix", default="samples")
    parser.add_argument("--artifact-dir", type=Path, required=True)
    parser.add_argument(
        "--i-understand-this-connects-to-real-controller",
        action="store_true",
        help="Required for known real controller IPs.",
    )
    return parser.parse_args()


def unique_ordered(values: list[str]) -> list[str]:
    seen: set[str] = set()
    out: list[str] = []
    for value in values:
        if value in seen:
            continue
        seen.add(value)
        out.append(value)
    return out


def config_from_args(args: argparse.Namespace) -> CaptureConfig:
    ips: list[str] = []
    if args.ip:
        ips.append(args.ip)
    if args.ips:
        ips.extend(args.ips)
    return CaptureConfig(
        ips=unique_ordered(ips),
        port=args.port,
        duration_sec=args.duration_sec,
        rate_hz=args.rate_hz,
        request_payload=args.request_payload,
        timeout_sec=args.timeout_sec,
        include_hex=args.hex,
        max_bytes_per_sample=args.max_bytes_per_sample,
        also_rbpodo_python=args.also_rbpodo_python,
        output_prefix=args.output_prefix,
        artifact_dir=args.artifact_dir,
        confirmed_real_controller=args.i_understand_this_connects_to_real_controller,
    )


def validate_config(config: CaptureConfig, *, enforce_data_port: bool = True) -> None:
    if not config.ips:
        raise CaptureError("--ip or --ips is required")
    if enforce_data_port and config.port != DEFAULT_DATA_PORT:
        raise CaptureError("refusing non-data-port connection; --port must be 5001")
    if config.port < 1 or config.port > 65535:
        raise CaptureError("--port must be in [1, 65535]")
    if not math.isfinite(config.duration_sec) or config.duration_sec <= 0.0:
        raise CaptureError("--duration-sec must be finite and positive")
    if not math.isfinite(config.rate_hz) or config.rate_hz <= 0.0:
        raise CaptureError("--rate-hz must be finite and positive")
    if not math.isfinite(config.timeout_sec) or config.timeout_sec <= 0.0:
        raise CaptureError("--timeout-sec must be finite and positive")
    if config.max_bytes_per_sample <= 0:
        raise CaptureError("--max-bytes-per-sample must be positive")
    if not config.request_payload:
        raise CaptureError("--request-payload must not be empty")
    real_ips = sorted(set(config.ips) & rbpodo_state_dump.REAL_ROBOT_IPS)
    if real_ips and not config.confirmed_real_controller:
        joined = ", ".join(real_ips)
        raise CaptureError(
            "refusing known real controller IP without "
            f"--i-understand-this-connects-to-real-controller: {joined}"
        )
    if real_ips and os.environ.get("RB_ALLOW_REAL_ROBOT") != "1":
        raise CaptureError("real controller raw data-port capture requires RB_ALLOW_REAL_ROBOT=1")


def request_bytes(request_payload: str) -> bytes:
    payload = request_payload.encode("utf-8")
    return payload if payload.endswith(b"\n") else payload + b"\n"


def now_ns() -> int:
    return time.monotonic_ns()


def response_prefix_hex(payload: bytes) -> str:
    return payload[:PREFIX_BYTES].hex()


def printable_ascii_prefix(payload: bytes) -> str:
    chars: list[str] = []
    for value in payload[:PREFIX_BYTES]:
        chars.append(chr(value) if 32 <= value <= 126 else ".")
    return "".join(chars)


def sanitize_for_filename(value: str) -> str:
    out = []
    for char in value:
        out.append(char if char.isalnum() or char in {"-", "_"} else "_")
    return "".join(out).strip("_") or "ip"


def read_response(sock: Any, timeout_sec: float, max_bytes: int) -> tuple[bytes, bool, str]:
    chunks: list[bytes] = []
    total = 0
    truncated = False
    stop_reason = "closed"
    quiet_timeout_sec = min(timeout_sec, 0.02)
    while total < max_bytes:
        try:
            chunk = sock.recv(min(4096, max_bytes - total))
        except socket.timeout:
            if chunks:
                stop_reason = "quiet_timeout"
                break
            raise
        if not chunk:
            stop_reason = "closed"
            break
        chunks.append(chunk)
        total += len(chunk)
        if total >= max_bytes:
            truncated = True
            stop_reason = "max_bytes"
            break
        if chunks:
            try:
                sock.settimeout(quiet_timeout_sec)
            except AttributeError:
                pass
    return b"".join(chunks), truncated, stop_reason


def close_socket(sock: Any) -> None:
    try:
        sock.close()
    except AttributeError:
        pass


def capture_one_sample(
    ip: str,
    port: int,
    sample_index: int,
    config: CaptureConfig,
    connect_fn: ConnectFn,
) -> tuple[dict[str, Any], bytes]:
    started_ns = now_ns()
    payload = b""
    record: dict[str, Any] = {
        "schema": "robotics_lab.rainbow_data_port_capture.sample.v1",
        "sample_index": sample_index,
        "host_time_ns": started_ns,
        "ip": ip,
        "port": port,
        "request_payload": config.request_payload,
        "request_payload_sha256": hashlib.sha256(request_bytes(config.request_payload)).hexdigest(),
        "bytes_len": 0,
        "response_sha256": None,
        "response_prefix_hex": "",
        "printable_ascii_prefix": "",
        "timeout": False,
        "error_name": "",
        "error_message": "",
        "truncated": False,
        "read_stop_reason": "",
        "ok": False,
    }
    sock = None
    try:
        sock = connect_fn((ip, port), config.timeout_sec)
        try:
            sock.settimeout(config.timeout_sec)
        except AttributeError:
            pass
        sock.sendall(request_bytes(config.request_payload))
        payload, truncated, stop_reason = read_response(
            sock,
            config.timeout_sec,
            config.max_bytes_per_sample,
        )
        record.update({
            "bytes_len": len(payload),
            "response_sha256": hashlib.sha256(payload).hexdigest(),
            "response_prefix_hex": response_prefix_hex(payload),
            "printable_ascii_prefix": printable_ascii_prefix(payload),
            "truncated": truncated,
            "read_stop_reason": stop_reason,
            "ok": len(payload) > 0,
            "error_name": "" if payload else "EmptyResponse",
            "error_message": "" if payload else "data port returned zero bytes",
        })
        if config.include_hex:
            record["response_hex"] = payload.hex()
    except socket.timeout as exc:
        record.update({
            "timeout": True,
            "error_name": "TransportTimeout",
            "error_message": str(exc),
            "read_stop_reason": "timeout",
        })
    except TimeoutError as exc:
        record.update({
            "timeout": True,
            "error_name": "TransportTimeout",
            "error_message": str(exc),
            "read_stop_reason": "timeout",
        })
    except OSError as exc:
        record.update({
            "error_name": type(exc).__name__,
            "error_message": str(exc),
            "read_stop_reason": "transport_error",
        })
    finally:
        if sock is not None:
            close_socket(sock)
    record["duration_us"] = (now_ns() - started_ns) / 1000.0
    return record, payload


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, sort_keys=True) + "\n")


def write_binary_payload(
    artifact_dir: Path,
    output_prefix: str,
    ip: str,
    sample_index: int,
    payload: bytes,
) -> str:
    filename = f"{output_prefix}_{sanitize_for_filename(ip)}_{sample_index:06d}.bin"
    path = artifact_dir / filename
    path.write_bytes(payload)
    raw_response = artifact_dir / "raw_response.bin"
    if not raw_response.exists():
        raw_response.write_bytes(payload)
    return str(path)


def common_prefix(payloads: list[bytes]) -> bytes:
    if not payloads:
        return b""
    prefix = payloads[0]
    for payload in payloads[1:]:
        limit = min(len(prefix), len(payload))
        index = 0
        while index < limit and prefix[index] == payload[index]:
            index += 1
        prefix = prefix[:index]
        if not prefix:
            break
    return prefix


def q_ref_key(sample: dict[str, Any]) -> str:
    return json.dumps(sample.get("q_ref_deg"), sort_keys=True, separators=(",", ":"))


def payload_pattern_summary(
    samples: list[dict[str, Any]],
    payloads: list[bytes],
    python_samples: list[dict[str, Any]],
) -> dict[str, Any]:
    successful = [sample for sample in samples if sample.get("ok")]
    hashes = [sample["response_sha256"] for sample in successful if sample.get("response_sha256")]
    lengths = sorted({int(sample.get("bytes_len") or 0) for sample in successful})
    prefix = common_prefix(payloads)
    raw_hash_by_sample = {
        (sample.get("sample_index"), sample.get("ip")): sample.get("response_sha256")
        for sample in successful
        if sample.get("response_sha256")
    }
    q_ref_hash_pairs: list[tuple[str, str]] = []
    for sample in python_samples:
        if not sample.get("ok"):
            continue
        raw_hash = raw_hash_by_sample.get((sample.get("sample_index"), sample.get("ip")))
        if raw_hash:
            q_ref_hash_pairs.append((q_ref_key(sample), str(raw_hash)))
    q_ref_keys = {q_ref_key(sample) for sample in python_samples if sample.get("ok")}
    q_ref_changed = len(q_ref_keys) > 1
    payload_changed = len(set(hashes)) > 1
    paired_hashes = {raw_hash for _q_ref, raw_hash in q_ref_hash_pairs}
    return {
        "unique_payload_lengths": lengths,
        "unique_hash_count": len(set(hashes)),
        "stable_prefix_hex": prefix[:PREFIX_BYTES].hex() if prefix else "",
        "stable_prefix_bytes_len": len(prefix),
        "payload_change_observed": payload_changed,
        "q_ref_unique_count": len(q_ref_keys),
        "q_ref_change_observed": q_ref_changed,
        "q_ref_payload_pair_count": len(q_ref_hash_pairs),
        "payload_changes_when_q_ref_changes": (
            len(paired_hashes) > 1
            if q_ref_changed and q_ref_hash_pairs
            else None
        ),
    }


def python_decoded_sample(
    ip: str,
    sample_index: int,
    timeout_sec: float,
    read_python_sdata: ReadPythonSDataFn,
) -> dict[str, Any]:
    started_ns = now_ns()
    try:
        sdata = read_python_sdata(ip, timeout_sec)
        report = rbpodo_state_dump.build_report_for_sdata(ip, sdata, None, None, None)
        return {
            "schema": "robotics_lab.rainbow_data_port_capture.python_decode.v1",
            "sample_index": sample_index,
            "host_time_ns": started_ns,
            "ip": ip,
            "ok": True,
            "q_actual_deg": report["q_actual_deg"],
            "q_ref_deg": report["q_ref_deg"],
            "q_ref_source": report["q_ref_source"],
            "raw": report["raw"],
            "diagnostics_suspect": report["diagnostics_suspect"],
            "diagnostics_suspect_reasons": report["diagnostics_suspect_reasons"],
            "error_name": "",
            "error_message": "",
            "duration_us": (now_ns() - started_ns) / 1000.0,
        }
    except Exception as exc:
        return {
            "schema": "robotics_lab.rainbow_data_port_capture.python_decode.v1",
            "sample_index": sample_index,
            "host_time_ns": started_ns,
            "ip": ip,
            "ok": False,
            "q_actual_deg": None,
            "q_ref_deg": None,
            "q_ref_source": "python_rbpodo.sdata.jnt_ref",
            "raw": {},
            "diagnostics_suspect": None,
            "diagnostics_suspect_reasons": [],
            "error_name": type(exc).__name__,
            "error_message": str(exc),
            "duration_us": (now_ns() - started_ns) / 1000.0,
        }


def summarize_capture(
    config: CaptureConfig,
    samples: list[dict[str, Any]],
    payloads: list[bytes],
    python_samples: list[dict[str, Any]],
) -> dict[str, Any]:
    success_count = sum(1 for sample in samples if sample.get("ok"))
    timeout_count = sum(1 for sample in samples if sample.get("timeout"))
    error_count = sum(1 for sample in samples if not sample.get("ok") and not sample.get("timeout"))
    pattern = payload_pattern_summary(samples, payloads, python_samples)
    suspect_values = [
        bool(sample.get("diagnostics_suspect"))
        for sample in python_samples
        if sample.get("ok") and sample.get("diagnostics_suspect") is not None
    ]
    if success_count > 0:
        result = "completed"
        reason = "captured one or more raw data-port responses"
    elif timeout_count > 0:
        result = "unsupported_or_timeout"
        reason = "data port did not respond before timeout"
    else:
        result = "failed"
        reason = "no raw data-port response captured"
    return {
        "schema": "robotics_lab.rainbow_data_port_capture.summary.v1",
        "read_only": True,
        "result": result,
        "reason": reason,
        "ips": config.ips,
        "port": config.port,
        "request_payload": config.request_payload,
        "duration_sec": config.duration_sec,
        "rate_hz": config.rate_hz,
        "sample_count": len(samples),
        "success_count": success_count,
        "timeout_count": timeout_count,
        "error_count": error_count,
        "unique_payload_lengths": pattern["unique_payload_lengths"],
        "unique_hash_count": pattern["unique_hash_count"],
        "stable_prefix_hex": pattern["stable_prefix_hex"],
        "stable_prefix_bytes_len": pattern["stable_prefix_bytes_len"],
        "fixture_comparison": pattern,
        "rbpodo_python_sample_count": len(python_samples),
        "rbpodo_python_diagnostics_suspect_rate": (
            sum(1 for value in suspect_values if value) / len(suspect_values)
            if suspect_values else None
        ),
        "safety_note": (
            "Read-only raw capture only. No command port, motion command, "
            "pgmode, fault reset, or binary parser is used."
        ),
        "parser_policy": "no_speculative_binary_parser",
    }


def run_capture(
    config: CaptureConfig,
    *,
    connect_fn: ConnectFn = socket.create_connection,
    read_python_sdata: ReadPythonSDataFn = rbpodo_state_dump.read_controller,
) -> dict[str, Any]:
    validate_config(config)
    artifact_dir = config.artifact_dir.resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    samples: list[dict[str, Any]] = []
    payloads: list[bytes] = []
    python_samples: list[dict[str, Any]] = []
    sample_count_per_ip = max(1, int(math.ceil(config.duration_sec * config.rate_hz)))
    period_sec = 1.0 / config.rate_hz
    sample_index = 0
    started = time.monotonic()
    for cycle in range(sample_count_per_ip):
        for ip in config.ips:
            record, payload = capture_one_sample(ip, config.port, sample_index, config, connect_fn)
            if payload:
                record["binary_path"] = write_binary_payload(
                    artifact_dir,
                    config.output_prefix,
                    ip,
                    sample_index,
                    payload,
                )
                payloads.append(payload)
            else:
                record["binary_path"] = None
            samples.append(record)
            if config.also_rbpodo_python:
                python_samples.append(
                    python_decoded_sample(ip, sample_index, config.timeout_sec, read_python_sdata)
                )
            sample_index += 1
        if cycle + 1 < sample_count_per_ip:
            next_time = started + (cycle + 1) * period_sec
            remaining = next_time - time.monotonic()
            if remaining > 0.0:
                time.sleep(min(remaining, period_sec))
    write_jsonl(artifact_dir / "samples.jsonl", samples)
    if config.also_rbpodo_python:
        write_jsonl(artifact_dir / "python_decoded_samples.jsonl", python_samples)
    summary = summarize_capture(config, samples, payloads, python_samples)
    (artifact_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (artifact_dir / "README.txt").write_text(
        "\n".join([
            "Rainbow data-port raw capture artifacts",
            "",
            "These files are for rbpodo measurement reliability investigation.",
            "Raw payloads may contain hardware-specific details; do not commit real payloads unless sanitized.",
            "This capture does not parse binary data and does not validate motion safety.",
            "",
        ]),
        encoding="utf-8",
    )
    return summary


def main() -> int:
    args = parse_args()
    config = config_from_args(args)
    try:
        validate_config(config)
        summary = run_capture(config)
    except CaptureError as exc:
        print(f"rainbow_data_port_capture: FAIL: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary.get("success_count", 0) > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
