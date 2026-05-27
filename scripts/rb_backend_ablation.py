#!/usr/bin/env python3
"""No-motion rbpodo vs rbscript_tcp backend timing ablation tool."""

from __future__ import annotations

import argparse
import csv
import importlib
import json
import math
import os
import socket
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


REAL_ROBOT_IPS = {"172.28.60.200", "172.28.60.201"}
REAL_ENV_KEYS = (
    "RB_ALLOW_REAL_ROBOT",
    "RB_ALLOW_REAL_MOTION",
    "RB_ALLOW_RBSCRIPT_TCP",
    "RB_ALLOW_RBSCRIPT_TCP_MOTION",
)
M_CODES = ("M561", "M568", "M569", "M570")
FORBIDDEN_NO_MOTION_TOKENS = (
    "move_",
    "servo",
    "pgmode",
    "speed_bar",
    "set_speed",
    "collision",
    "task",
    "joint",
    "jnt[",
)


class AblationError(RuntimeError):
    pass


@dataclass
class Sample:
    index: int
    mode: str
    backend: str
    success: bool
    duration_us: float
    start_ns: int
    end_ns: int
    error_name: str = ""
    error_message: str = ""
    response: str = ""
    state_valid: bool | None = None
    q_actual_finite: bool | None = None
    state_age_ms: float | None = None
    response_error_names: list[str] = field(default_factory=list)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare rbpodo and experimental rbscript_tcp connection, ACK, and "
            "read-state timing without commanding real motion by default."
        )
    )
    parser.add_argument("--left-ip", required=True)
    parser.add_argument("--right-ip", required=True)
    parser.add_argument("--arm", choices=("left", "right"), required=True)
    parser.add_argument("--backend", choices=("rbpodo", "rbscript_tcp"), required=True)
    parser.add_argument(
        "--mode",
        choices=("connect_only", "read_state", "command_ack_no_motion", "servo_j_dry_run"),
        required=True,
    )
    parser.add_argument("--duration-sec", type=float, default=10.0)
    parser.add_argument("--rate-hz", type=float, default=100.0)
    parser.add_argument("--artifact-dir", type=Path, required=True)
    parser.add_argument("--command-port", type=int, default=5000)
    parser.add_argument("--data-port", type=int, default=5001)
    parser.add_argument("--connect-timeout-sec", type=float, default=1.0)
    parser.add_argument("--read-timeout-sec", type=float, default=0.2)
    parser.add_argument("--command-timeout-sec", type=float, default=0.2)
    parser.add_argument(
        "--rbscript-no-motion-command",
        help=(
            "Explicit safe script text for command_ack_no_motion. The tool will "
            "not invent a Rainbow no-motion command."
        ),
    )
    parser.add_argument("--allow-motion", action="store_true")
    parser.add_argument("--max-delta-deg", type=float)
    parser.add_argument(
        "--i-understand-this-connects-to-real-controller",
        action="store_true",
        help="Required when the selected arm IP is a known real robot controller IP.",
    )
    parser.add_argument("--skip-plots", action="store_true")
    return parser.parse_args()


def selected_ip(args: argparse.Namespace) -> str:
    return args.left_ip if args.arm == "left" else args.right_ip


def env_snapshot() -> dict[str, str | None]:
    return {key: os.environ.get(key) for key in REAL_ENV_KEYS}


def env_enabled(name: str) -> bool:
    return os.environ.get(name) == "1"


def is_motion_mode(mode: str) -> bool:
    return mode == "servo_j_dry_run"


def command_text_looks_motion_capable(command: str) -> bool:
    lowered = command.lower()
    return any(token in lowered for token in FORBIDDEN_NO_MOTION_TOKENS)


def preflight(args: argparse.Namespace) -> dict[str, Any]:
    if not math.isfinite(args.duration_sec) or args.duration_sec <= 0.0:
        raise AblationError("--duration-sec must be finite and positive")
    if not math.isfinite(args.rate_hz) or args.rate_hz <= 0.0:
        raise AblationError("--rate-hz must be finite and positive")
    for name in ("command_port", "data_port"):
        value = getattr(args, name)
        if value < 1 or value > 65535:
            raise AblationError(f"--{name.replace('_', '-')} must be in [1, 65535]")

    target_ip = selected_ip(args)
    real_ip = target_ip in REAL_ROBOT_IPS
    motion_mode = is_motion_mode(args.mode)

    if real_ip and not args.i_understand_this_connects_to_real_controller:
        raise AblationError(
            "refusing known real controller IP without "
            "--i-understand-this-connects-to-real-controller"
        )
    if motion_mode and not args.allow_motion:
        raise AblationError("refusing motion-capable mode without --allow-motion")
    if motion_mode:
        if args.max_delta_deg is None or args.max_delta_deg <= 0.0:
            raise AblationError("motion-capable modes require explicit positive --max-delta-deg")
        required = ["RB_ALLOW_REAL_ROBOT", "RB_ALLOW_REAL_MOTION"]
        if args.backend == "rbscript_tcp":
            required.extend(["RB_ALLOW_RBSCRIPT_TCP", "RB_ALLOW_RBSCRIPT_TCP_MOTION"])
        missing = [key for key in required if not env_enabled(key)]
        if missing:
            raise AblationError("refusing motion-capable mode; missing env gates: " + ", ".join(missing))

    if real_ip:
        if args.backend == "rbpodo" and not env_enabled("RB_ALLOW_REAL_ROBOT"):
            raise AblationError("rbpodo real connect/read requires RB_ALLOW_REAL_ROBOT=1")
        if args.backend == "rbscript_tcp":
            missing = [key for key in ("RB_ALLOW_REAL_ROBOT", "RB_ALLOW_RBSCRIPT_TCP") if not env_enabled(key)]
            if missing:
                raise AblationError("rbscript_tcp real connect/read requires env gates: " + ", ".join(missing))

    if args.mode == "command_ack_no_motion":
        if args.backend != "rbscript_tcp":
            raise AblationError("command_ack_no_motion is currently implemented only for rbscript_tcp")
        if not args.rbscript_no_motion_command:
            raise AblationError("command_ack_no_motion requires explicit --rbscript-no-motion-command")
        if command_text_looks_motion_capable(args.rbscript_no_motion_command):
            raise AblationError(
                "refusing command_ack_no_motion command text with motion-capable token; "
                "use only a verified no-motion script command"
            )

    if args.mode == "servo_j_dry_run":
        raise AblationError("servo_j_dry_run is intentionally not implemented in RBSCRIPT-ABLATION-01")

    return {
        "target_ip": target_ip,
        "target_is_known_real_robot_ip": real_ip,
        "motion_mode": motion_mode,
        "env": env_snapshot(),
        "safety_mode": "no_motion" if not motion_mode else "motion_rejected_or_explicit",
        "confirmation_flag": bool(args.i_understand_this_connects_to_real_controller),
    }


def now_ns() -> int:
    return time.monotonic_ns()


def percentile(values: list[float], pct: float) -> float | None:
    if not values:
        return None
    if len(values) == 1:
        return values[0]
    ordered = sorted(values)
    pos = (len(ordered) - 1) * pct
    low = math.floor(pos)
    high = math.ceil(pos)
    if low == high:
        return ordered[low]
    return ordered[low] + (ordered[high] - ordered[low]) * (pos - low)


def finite_number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        number = float(value)
        return number if math.isfinite(number) else None
    return None


def tcp_connect(ip: str, port: int, timeout_sec: float) -> socket.socket:
    sock = socket.create_connection((ip, port), timeout=timeout_sec)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.settimeout(timeout_sec)
    return sock


def recv_line(sock: socket.socket, timeout_sec: float, limit: int = 4096) -> str:
    sock.settimeout(timeout_sec)
    chunks: list[bytes] = []
    total = 0
    while total < limit:
        data = sock.recv(1)
        if not data:
            break
        if data == b"\n":
            break
        if data != b"\r":
            chunks.append(data)
            total += len(data)
    return b"".join(chunks).decode("utf-8", errors="replace")


def classify_response(text: str) -> tuple[bool, list[str]]:
    lowered = text.lower()
    names = [code for code in M_CODES if code.lower() in lowered]
    if "not allowed" in lowered or "error" in lowered or "fail" in lowered:
        if not names:
            names.append("controller_error")
        return False, names
    if "executed" in lowered or "ok" in lowered or "success" in lowered:
        return True, names
    return False, names or ["unrecognized_response"]


def parse_rbscript_state_response(text: str) -> tuple[bool, bool, float | None, list[str]]:
    try:
        payload = json.loads(text)
    except json.JSONDecodeError:
        return False, False, None, ["parse_error"]
    if not isinstance(payload, dict) or payload.get("schema") != "rbscript_tcp_state_v1":
        return False, False, None, ["unsupported_schema"]
    q_actual = payload.get("q_actual_deg")
    finite = (
        isinstance(q_actual, list)
        and len(q_actual) == 6
        and all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in q_actual)
    )
    robot_time_sec = finite_number(payload.get("robot_time_sec"))
    state_age_ms = None
    if robot_time_sec is not None:
        # Robot time is controller-relative in most APIs, so this is only useful
        # for fixture/local comparisons. Real age needs a synchronized source.
        state_age_ms = 0.0
    return finite, finite, state_age_ms, [] if finite else ["invalid_joint_state"]


class RbscriptProbe:
    def __init__(self, args: argparse.Namespace, ip: str) -> None:
        self.args = args
        self.ip = ip

    def connect_only(self, index: int) -> Sample:
        start = now_ns()
        try:
            with tcp_connect(self.ip, self.args.command_port, self.args.connect_timeout_sec):
                end = now_ns()
                return Sample(index, "connect_only", "rbscript_tcp", True, (end - start) / 1000.0, start, end)
        except Exception as exc:
            end = now_ns()
            return Sample(index, "connect_only", "rbscript_tcp", False, (end - start) / 1000.0, start, end, type(exc).__name__, str(exc))

    def read_state(self, index: int) -> Sample:
        start = now_ns()
        response = ""
        try:
            with tcp_connect(self.ip, self.args.data_port, self.args.connect_timeout_sec) as sock:
                sock.settimeout(self.args.read_timeout_sec)
                sock.sendall(b"reqdata\n")
                response = recv_line(sock, self.args.read_timeout_sec)
            state_valid, q_finite, state_age_ms, names = parse_rbscript_state_response(response)
            end = now_ns()
            return Sample(
                index,
                "read_state",
                "rbscript_tcp",
                state_valid,
                (end - start) / 1000.0,
                start,
                end,
                "" if state_valid else (names[0] if names else "invalid_state"),
                "",
                response,
                state_valid,
                q_finite,
                state_age_ms,
                names,
            )
        except socket.timeout as exc:
            end = now_ns()
            return Sample(index, "read_state", "rbscript_tcp", False, (end - start) / 1000.0, start, end, "TransportTimeout", str(exc), response)
        except Exception as exc:
            end = now_ns()
            return Sample(index, "read_state", "rbscript_tcp", False, (end - start) / 1000.0, start, end, type(exc).__name__, str(exc), response)

    def command_ack_no_motion(self, index: int) -> Sample:
        command = self.args.rbscript_no_motion_command or ""
        if not command.endswith("\n"):
            command += "\n"
        start = now_ns()
        response = ""
        try:
            with tcp_connect(self.ip, self.args.command_port, self.args.connect_timeout_sec) as sock:
                sock.settimeout(self.args.command_timeout_sec)
                sock.sendall(command.encode("utf-8"))
                response = recv_line(sock, self.args.command_timeout_sec)
            ok, names = classify_response(response)
            end = now_ns()
            return Sample(
                index,
                "command_ack_no_motion",
                "rbscript_tcp",
                ok,
                (end - start) / 1000.0,
                start,
                end,
                "" if ok else (names[0] if names else "ack_error"),
                "",
                response,
                None,
                None,
                None,
                names,
            )
        except socket.timeout as exc:
            end = now_ns()
            return Sample(index, "command_ack_no_motion", "rbscript_tcp", False, (end - start) / 1000.0, start, end, "TransportTimeout", str(exc), response)
        except Exception as exc:
            end = now_ns()
            return Sample(index, "command_ack_no_motion", "rbscript_tcp", False, (end - start) / 1000.0, start, end, type(exc).__name__, str(exc), response)


class RbpodoProbe:
    def __init__(self, args: argparse.Namespace, ip: str) -> None:
        self.args = args
        self.ip = ip
        try:
            self.rbpodo = importlib.import_module("rbpodo")
        except Exception as exc:
            raise AblationError(
                "Python rbpodo module is unavailable; rbpodo read/connect ablation "
                "requires an installed Python binding or a future C++ probe wrapper"
            ) from exc
        self.data = None

    def _cobot_data(self) -> Any:
        if not hasattr(self.rbpodo, "CobotData"):
            raise AblationError("Python rbpodo module does not expose CobotData")
        if self.data is None:
            self.data = self.rbpodo.CobotData(self.ip)
        return self.data

    def connect_only(self, index: int) -> Sample:
        start = now_ns()
        try:
            self._cobot_data()
            end = now_ns()
            return Sample(index, "connect_only", "rbpodo", True, (end - start) / 1000.0, start, end)
        except Exception as exc:
            end = now_ns()
            return Sample(index, "connect_only", "rbpodo", False, (end - start) / 1000.0, start, end, type(exc).__name__, str(exc))

    def read_state(self, index: int) -> Sample:
        start = now_ns()
        try:
            state = self._cobot_data().request_data(self.args.read_timeout_sec)
            sdata = getattr(state, "sdata", state)
            q_actual = list(getattr(sdata, "jnt_ang", []))
            q_finite = len(q_actual) == 6 and all(math.isfinite(float(value)) for value in q_actual)
            end = now_ns()
            return Sample(
                index,
                "read_state",
                "rbpodo",
                q_finite,
                (end - start) / 1000.0,
                start,
                end,
                "" if q_finite else "invalid_joint_state",
                "",
                "",
                q_finite,
                q_finite,
            )
        except Exception as exc:
            end = now_ns()
            return Sample(index, "read_state", "rbpodo", False, (end - start) / 1000.0, start, end, type(exc).__name__, str(exc))

    def command_ack_no_motion(self, index: int) -> Sample:
        start = now_ns()
        end = now_ns()
        return Sample(
            index,
            "command_ack_no_motion",
            "rbpodo",
            False,
            (end - start) / 1000.0,
            start,
            end,
            "UnsupportedMode",
            "No verified rbpodo no-motion ACK command is wired in this ablation task.",
        )


def make_probe(args: argparse.Namespace, ip: str) -> Any:
    if args.backend == "rbscript_tcp":
        return RbscriptProbe(args, ip)
    return RbpodoProbe(args, ip)


def run_samples(args: argparse.Namespace) -> list[Sample]:
    probe = make_probe(args, selected_ip(args))
    deadline = time.monotonic() + args.duration_sec
    period = 1.0 / args.rate_hz
    samples: list[Sample] = []
    index = 0
    next_time = time.monotonic()
    while time.monotonic() < deadline:
        op = getattr(probe, args.mode)
        samples.append(op(index))
        index += 1
        next_time += period
        sleep_sec = next_time - time.monotonic()
        if sleep_sec > 0.0:
            time.sleep(sleep_sec)
    return samples


def loop_intervals_ms(samples: list[Sample]) -> list[float]:
    return [
        (b.start_ns - a.start_ns) / 1e6
        for a, b in zip(samples, samples[1:])
        if b.start_ns >= a.start_ns
    ]


def metric_block(values: list[float]) -> dict[str, float | None]:
    return {
        "p50": percentile(values, 0.50),
        "p95": percentile(values, 0.95),
        "p99": percentile(values, 0.99),
        "max": max(values) if values else None,
    }


def summarize(args: argparse.Namespace, safety: dict[str, Any], samples: list[Sample]) -> dict[str, Any]:
    durations = [sample.duration_us for sample in samples]
    successes = [sample for sample in samples if sample.success]
    errors = [sample for sample in samples if not sample.success]
    intervals = loop_intervals_ms(samples)
    elapsed_sec = (samples[-1].end_ns - samples[0].start_ns) / 1e9 if len(samples) >= 2 else 0.0
    response_error_names: dict[str, int] = {}
    for sample in samples:
        sample_names: set[str] = set()
        for name in sample.response_error_names:
            sample_names.add(name)
        if sample.error_name:
            sample_names.add(sample.error_name)
        for name in sample_names:
            response_error_names[name] = response_error_names.get(name, 0) + 1

    ack_values = durations if args.mode == "command_ack_no_motion" else []
    read_values = durations if args.mode == "read_state" else []
    connect_values = durations if args.mode == "connect_only" else []
    timeout_count = sum(1 for sample in samples if "timeout" in sample.error_name.lower())
    parse_error_count = sum(
        1
        for sample in samples
        if sample.error_name in {"parse_error", "unsupported_schema", "invalid_joint_state"}
        or any(name in {"parse_error", "unsupported_schema", "invalid_joint_state"} for name in sample.response_error_names)
    )
    state_samples = [sample for sample in samples if sample.state_valid is not None]
    finite_count = sum(1 for sample in state_samples if sample.q_actual_finite)

    return {
        "backend": args.backend,
        "mode": args.mode,
        "arm": args.arm,
        "target_ip": safety["target_ip"],
        "requested_rate_hz": args.rate_hz,
        "duration_sec": args.duration_sec,
        "achieved_rate_hz": (len(samples) / elapsed_sec) if elapsed_sec > 0.0 else None,
        "sample_count": len(samples),
        "success_count": len(successes),
        "success_rate": len(successes) / len(samples) if samples else 0.0,
        "command_success_count": len(successes) if args.mode == "command_ack_no_motion" else 0,
        "command_error_count": len(errors) if args.mode == "command_ack_no_motion" else 0,
        "command_timeout_count": timeout_count if args.mode == "command_ack_no_motion" else 0,
        "connect_latency_us": metric_block(connect_values),
        "read_duration_us": metric_block(read_values),
        "command_ack_latency_us": metric_block(ack_values),
        "loop_interval_ms": metric_block(intervals),
        "parse_error_count": parse_error_count,
        "response_error_names": response_error_names,
        "m_code_counts": {code: response_error_names.get(code, 0) for code in M_CODES},
        "state_valid_count": sum(1 for sample in state_samples if sample.state_valid),
        "state_invalid_count": sum(1 for sample in state_samples if sample.state_valid is False),
        "q_actual_finite_ratio": finite_count / len(state_samples) if state_samples else None,
        "data_port_timeout_count": timeout_count if args.mode == "read_state" else 0,
        "reconnect_count": max(len(samples) - 1, 0) if args.mode in {"connect_only", "read_state", "command_ack_no_motion"} else 0,
        "ack_disabled": False,
        "safety_preflight": safety,
        "result": "completed" if samples else "error",
    }


def sample_row(sample: Sample) -> dict[str, Any]:
    return {
        "index": sample.index,
        "mode": sample.mode,
        "backend": sample.backend,
        "success": sample.success,
        "duration_us": sample.duration_us,
        "start_ns": sample.start_ns,
        "end_ns": sample.end_ns,
        "error_name": sample.error_name,
        "error_message": sample.error_message,
        "state_valid": sample.state_valid,
        "q_actual_finite": sample.q_actual_finite,
        "state_age_ms": sample.state_age_ms,
    }


def write_artifacts(args: argparse.Namespace, safety: dict[str, Any], samples: list[Sample], summary: dict[str, Any]) -> None:
    artifact_dir = args.artifact_dir.resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    summary["artifact_dir"] = str(artifact_dir)
    summary_path = artifact_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    with (artifact_dir / "samples.csv").open("w", newline="", encoding="utf-8") as handle:
        fieldnames = list(sample_row(samples[0]).keys()) if samples else [
            "index", "mode", "backend", "success", "duration_us", "start_ns", "end_ns",
            "error_name", "error_message", "state_valid", "q_actual_finite", "state_age_ms",
        ]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for sample in samples:
            writer.writerow(sample_row(sample))

    with (artifact_dir / "responses.jsonl").open("w", encoding="utf-8") as handle:
        for sample in samples:
            if sample.response:
                handle.write(json.dumps({"index": sample.index, "response": sample.response}) + "\n")

    with (artifact_dir / "errors.jsonl").open("w", encoding="utf-8") as handle:
        for sample in samples:
            if not sample.success:
                handle.write(json.dumps({
                    "index": sample.index,
                    "error_name": sample.error_name,
                    "error_message": sample.error_message,
                    "response_error_names": sample.response_error_names,
                }) + "\n")

    command = " ".join(sys.argv)
    (artifact_dir / "README.txt").write_text(
        "\n".join([
            "rb_backend_ablation artifacts",
            "",
            f"command: {command}",
            f"backend: {args.backend}",
            f"mode: {args.mode}",
            f"safety_mode: {safety['safety_mode']}",
            "No real motion is commanded by this task.",
            "",
        ]),
        encoding="utf-8",
    )

    if not args.skip_plots:
        write_plots(artifact_dir, samples)


def write_plots(artifact_dir: Path, samples: list[Sample]) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        (artifact_dir / "plot_skip_reason.txt").write_text(f"matplotlib unavailable: {exc}\n", encoding="utf-8")
        return

    durations = [sample.duration_us for sample in samples]
    if durations:
        plt.figure(figsize=(8, 4))
        plt.hist(durations, bins=min(40, max(5, len(durations) // 2)))
        plt.xlabel("duration_us")
        plt.ylabel("count")
        plt.tight_layout()
        plt.savefig(artifact_dir / "timing_histogram.png")
        plt.close()

    intervals = loop_intervals_ms(samples)
    if intervals:
        plt.figure(figsize=(8, 4))
        plt.plot(intervals)
        plt.xlabel("sample")
        plt.ylabel("loop interval ms")
        plt.tight_layout()
        plt.savefig(artifact_dir / "loop_interval.png")
        plt.close()

    ages = [sample.state_age_ms for sample in samples if sample.state_age_ms is not None]
    if ages:
        plt.figure(figsize=(8, 4))
        plt.plot(ages)
        plt.xlabel("sample")
        plt.ylabel("state age ms")
        plt.tight_layout()
        plt.savefig(artifact_dir / "state_age.png")
        plt.close()


def main() -> int:
    args = parse_args()
    try:
        safety = preflight(args)
        print(json.dumps({"event": "safety_preflight", **safety}, indent=2, sort_keys=True))
        samples = run_samples(args)
        summary = summarize(args, safety, samples)
        write_artifacts(args, safety, samples, summary)
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 0 if summary["result"] == "completed" else 1
    except AblationError as exc:
        print(f"rb_backend_ablation: FAIL: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
