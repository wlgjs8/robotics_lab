#!/usr/bin/env python3
"""Explicit Rainbow external command/read-state rate probe."""

from __future__ import annotations

import argparse
import csv
import importlib
import json
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import rb_backend_ablation as backend_probe


DEFAULT_RATES = "50,75,100,125,150,200"
SERVO_J_DEFAULT_RATES = "100,500"
RBPODO_SERVO_J_ENV = (
    "RB_ALLOW_REAL_ROBOT",
    "RB_ALLOW_REAL_MOTION",
    "RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION",
)
RBPODO_SERVO_J_DIAGNOSTIC_FIELDS = (
    "real_vs_simulation_mode",
    "init_state_info",
    "init_error",
    "op_stat_sos_flag",
    "op_stat_ems_flag",
    "op_stat_soft_estop_occur",
    "op_stat_collision_occur",
    "op_stat_self_collision",
)
SERVO_J_SAMPLE_FIELDS = [
    "index", "mode", "backend", "requested_rate_hz", "feedback_rate_hz",
    "loop_start_ns", "loop_end_ns", "loop_interval_ms", "read_duration_us",
    "state_valid", "controller_mode", "raw_controller_mode",
    "q_actual_drift_max_deg", "send_requested", "send_success",
    "send_start_ns", "send_end_ns", "send_duration_us", "servo_t1_sec",
    "servo_t2_sec", "servo_gain", "servo_alpha", "error_name",
    "error_message", "response", "response_error_names",
    "q_actual_0", "q_actual_1", "q_actual_2", "q_actual_3", "q_actual_4", "q_actual_5",
    "q_ref_0", "q_ref_1", "q_ref_2", "q_ref_3", "q_ref_4", "q_ref_5",
]


class RateProbeError(RuntimeError):
    pass


@dataclass
class RbpodoStateRead:
    success: bool
    duration_us: float
    q_actual_deg: list[float] | None
    q_ref_deg: list[float] | None
    controller_mode: str
    raw_mode: Any
    diagnostics: dict[str, Any]
    error_name: str = ""
    error_message: str = ""


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
    parser.add_argument(
        "--rates",
        default=DEFAULT_RATES,
        help=(
            f"Comma-separated rates. Default is {DEFAULT_RATES}; "
            f"servo_j_simulation_only uses and requires {SERVO_J_DEFAULT_RATES}."
        ),
    )
    parser.add_argument("--duration-sec", type=float, default=10.0)
    parser.add_argument("--artifact-dir", type=Path, required=True)
    parser.add_argument("--command-port", type=int, default=5000)
    parser.add_argument("--data-port", type=int, default=5001)
    parser.add_argument("--connect-timeout-sec", type=float, default=1.0)
    parser.add_argument("--read-timeout-sec", type=float, default=0.2)
    parser.add_argument("--command-timeout-sec", type=float, default=0.2)
    parser.add_argument(
        "--persistent-socket",
        action="store_true",
        help=(
            "Keep one rbscript_tcp socket open per rate run and reconnect only "
            "after transport errors. Recommended for C++ backend comparison."
        ),
    )
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
        help="Allow rbpodo pgmode-simulation Servo J no-op rate probing.",
    )
    parser.add_argument(
        "--disable-waiting-ack",
        action="store_true",
        help="Rejected by default; included to document that no-ACK rate probes lose immediate ACK/error data.",
    )
    parser.add_argument(
        "--capture-raw-data-port",
        action="store_true",
        help=(
            "Read-only rbscript_tcp data-port capture mode: send reqdata to TCP 5001 "
            "and store raw bytes/text without parsing or marking state valid."
        ),
    )
    parser.add_argument(
        "--feedback-rate-hz",
        type=float,
        default=500.0,
        help="Feedback CobotData read loop rate for servo_j_simulation_only.",
    )
    parser.add_argument("--servo-t2-sec", type=float, default=0.05)
    parser.add_argument("--servo-gain", type=float, default=1.0)
    parser.add_argument("--servo-alpha", type=float, default=0.5)
    parser.add_argument("--max-q-actual-drift-deg", type=float, default=0.05)
    parser.add_argument(
        "--set-pgmode-simulation",
        action="store_true",
        help="Send pgmode simulation before servo_j_simulation_only and verify it.",
    )
    parser.add_argument(
        "--verify-pgmode-simulation",
        action="store_true",
        help="Verify pgmode simulation before servo_j_simulation_only without sending pgmode.",
    )
    parser.add_argument("--pgmode-timeout-sec", type=float, default=1.0)
    parser.add_argument("--pgmode-command-port", type=int, default=5000)
    parser.add_argument("--skip-plots", action="store_true")
    return parser.parse_args()


def mode_for_backend_probe(mode: str) -> str:
    if mode == "ack_no_motion":
        return "command_ack_no_motion"
    if mode == "read_state":
        return "read_state"
    return "servo_j_dry_run"


def env_enabled(name: str) -> bool:
    return os.environ.get(name) == "1"


def servo_j_env_snapshot() -> dict[str, str | None]:
    return {key: os.environ.get(key) for key in RBPODO_SERVO_J_ENV + ("RB_ALLOW_REAL_CARTESIAN",)}


def finite_positive(value: float, name: str) -> None:
    if not math.isfinite(value) or value <= 0.0:
        raise RateProbeError(f"{name} must be finite and positive")


def validate_servo_j_rate_schedule(rates: list[float], feedback_rate_hz: float) -> list[dict[str, Any]]:
    schedule: list[dict[str, Any]] = []
    for rate in rates:
        if rate > feedback_rate_hz:
            raise RateProbeError("servo_j command rates must be <= --feedback-rate-hz")
        ratio = feedback_rate_hz / rate
        interval = int(round(ratio))
        if interval < 1 or abs(ratio - interval) > 1e-9:
            raise RateProbeError(
                "servo_j command rates must divide --feedback-rate-hz exactly "
                f"for deterministic tick scheduling: rate={rate}, feedback={feedback_rate_hz}"
            )
        schedule.append({
            "command_rate_hz": rate,
            "send_interval_feedback_ticks": interval,
            "servo_t1_sec": 1.0 / rate,
        })
    return schedule


def validate_servo_j_requested_rates(rates: list[float]) -> None:
    if len(rates) != 2 or any(abs(actual - expected) > 1e-9 for actual, expected in zip(rates, [100.0, 500.0])):
        raise RateProbeError("servo_j_simulation_only is scoped to exactly --rates 100,500")


def write_pgmode_preflight_summary(args: argparse.Namespace, summary: dict[str, Any]) -> Path:
    path = args.artifact_dir.resolve() / "pgmode_preflight.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path


def compact_text(value: Any, limit: int = 160) -> str:
    text = str(value).strip().replace("\r", "\\r").replace("\n", "\\n")
    if len(text) <= limit:
        return text
    return text[: limit - 3] + "..."


def pgmode_failure_details(summary: dict[str, Any]) -> str:
    parts: list[str] = []
    for item in summary.get("results", []):
        if not isinstance(item, dict):
            continue
        fields = [
            ("ip", item.get("ip")),
            ("action", item.get("action")),
            ("pgmode_command_sent", item.get("pgmode_command_sent")),
            ("command_ok", item.get("command_ok")),
            ("response_classification", item.get("response_classification")),
            ("response_raw", compact_text(item.get("response_raw", ""))),
            ("verification_available", item.get("verification_available")),
            ("real_vs_simulation_mode", item.get("real_vs_simulation_mode")),
            ("controller_mode", item.get("controller_mode")),
            ("verification_error_name", item.get("verification_error_name")),
            ("verification_error_message", compact_text(item.get("verification_error_message", ""))),
            ("error_name", item.get("error_name")),
            ("error_message", compact_text(item.get("error_message", ""))),
        ]
        detail = ", ".join(
            f"{name}={value}"
            for name, value in fields
            if value not in (None, "")
        )
        if detail:
            parts.append(detail)
    return "; ".join(parts) if parts else "no per-controller details"


def ensure_controller_simulation_for_servo_j(args: argparse.Namespace) -> dict[str, Any]:
    try:
        from rainbow_pgmode import RainbowPgmodeError, run_pgmode
    except Exception as exc:
        raise RateProbeError("scripts/rainbow_pgmode.py helper is unavailable") from exc
    try:
        summary = run_pgmode(
            [args.ip],
            args.pgmode_timeout_sec,
            port=args.pgmode_command_port,
            confirmation=args.i_understand_this_connects_to_real_controller,
            set_simulation=args.set_pgmode_simulation,
            verify_only=args.verify_pgmode_simulation,
        )
    except RainbowPgmodeError as exc:
        raise RateProbeError(
            f"controller not confirmed in pgmode simulation; refusing Servo J probe: {exc}"
        ) from exc
    path = write_pgmode_preflight_summary(args, summary)
    if summary.get("overall_result") != "ok":
        raise RateProbeError(
            "controller not confirmed in pgmode simulation; refusing Servo J probe; "
            f"pgmode_summary={path}; details: {pgmode_failure_details(summary)}"
        )
    return summary


def servo_j_preflight(args: argparse.Namespace, rates: list[float]) -> dict[str, Any]:
    if args.backend != "rbpodo":
        raise RateProbeError("servo_j_simulation_only is implemented only for --backend rbpodo")
    if not args.allow_simulation_servo_j:
        raise RateProbeError("servo_j_simulation_only requires --allow-simulation-servo-j")
    if args.capture_raw_data_port:
        raise RateProbeError("--capture-raw-data-port is not valid for servo_j_simulation_only")
    if getattr(args, "persistent_socket", False):
        raise RateProbeError("--persistent-socket is rbscript_tcp-only")
    if args.rbscript_no_motion_command:
        raise RateProbeError("--rbscript-no-motion-command is rbscript_tcp-only")
    if args.set_pgmode_simulation and args.verify_pgmode_simulation:
        raise RateProbeError("--set-pgmode-simulation and --verify-pgmode-simulation are mutually exclusive")
    if not args.set_pgmode_simulation and not args.verify_pgmode_simulation:
        raise RateProbeError(
            "servo_j_simulation_only requires --set-pgmode-simulation or --verify-pgmode-simulation"
        )
    if args.ip in backend_probe.REAL_ROBOT_IPS and not args.i_understand_this_connects_to_real_controller:
        raise RateProbeError("refusing known real controller IP without explicit confirmation flag")
    for name in RBPODO_SERVO_J_ENV:
        if not env_enabled(name):
            raise RateProbeError(f"servo_j_simulation_only requires {name}=1")
    if env_enabled("RB_ALLOW_REAL_CARTESIAN"):
        raise RateProbeError("RB_ALLOW_REAL_CARTESIAN must not be set for Servo J rate probing")

    finite_positive(args.feedback_rate_hz, "--feedback-rate-hz")
    finite_positive(args.command_timeout_sec, "--command-timeout-sec")
    finite_positive(args.read_timeout_sec, "--read-timeout-sec")
    finite_positive(args.connect_timeout_sec, "--connect-timeout-sec")
    finite_positive(args.pgmode_timeout_sec, "--pgmode-timeout-sec")
    finite_positive(args.servo_t2_sec, "--servo-t2-sec")
    finite_positive(args.servo_gain, "--servo-gain")
    if not math.isfinite(args.servo_alpha) or not (0.0 < args.servo_alpha < 1.0):
        raise RateProbeError("--servo-alpha must be finite and in (0, 1)")
    if not math.isfinite(args.max_q_actual_drift_deg) or not (0.0 < args.max_q_actual_drift_deg <= 0.2):
        raise RateProbeError("--max-q-actual-drift-deg must be finite, positive, and <= 0.2")
    if args.command_port < 1 or args.command_port > 65535:
        raise RateProbeError("--command-port must be in [1, 65535]")
    if args.data_port < 1 or args.data_port > 65535:
        raise RateProbeError("--data-port must be in [1, 65535]")
    if args.pgmode_command_port < 1 or args.pgmode_command_port > 65535:
        raise RateProbeError("--pgmode-command-port must be in [1, 65535]")

    validate_servo_j_requested_rates(rates)
    schedule = validate_servo_j_rate_schedule(rates, args.feedback_rate_hz)
    pgmode_summary = ensure_controller_simulation_for_servo_j(args)
    return {
        "backend": args.backend,
        "target_ip": args.ip,
        "env": servo_j_env_snapshot(),
        "rates": rates,
        "feedback_rate_hz": args.feedback_rate_hz,
        "send_rate_schedule": schedule,
        "rate_probe_mode": args.mode,
        "safety_mode": "rbpodo_controller_simulation_servo_j_noop",
        "physical_motion_expected": False,
        "allow_simulation_servo_j": True,
        "disable_waiting_ack": False,
        "target_is_known_real_robot_ip": args.ip in backend_probe.REAL_ROBOT_IPS,
        "pgmode_simulation_preflight": pgmode_summary,
        "pgmode_simulation_confirmed": pgmode_summary.get("overall_result") == "ok",
        "servo_params": {
            "servo_t2_sec": args.servo_t2_sec,
            "servo_gain": args.servo_gain,
            "servo_alpha": args.servo_alpha,
            "ack_policy": "wait",
        },
        "max_q_actual_drift_deg": args.max_q_actual_drift_deg,
        "caveat": "Connects to real Rainbow controller boxes but requires pgmode simulation; physical motion is not approved.",
    }


def preflight(args: argparse.Namespace) -> dict[str, Any]:
    rate_text = SERVO_J_DEFAULT_RATES if args.mode == "servo_j_simulation_only" and args.rates == DEFAULT_RATES else args.rates
    rates = parse_rates(rate_text)
    if not math.isfinite(args.duration_sec) or args.duration_sec <= 0.0:
        raise RateProbeError("--duration-sec must be finite and positive")
    if args.disable_waiting_ack:
        raise RateProbeError("--disable-waiting-ack is not supported in RBSCRIPT-RATE-PROBE-01")
    if args.mode == "servo_j_simulation_only":
        return servo_j_preflight(args, rates)
    if args.mode == "ack_no_motion":
        if args.backend != "rbscript_tcp":
            raise RateProbeError("ack_no_motion is currently implemented only for rbscript_tcp")
        if not args.rbscript_no_motion_command:
            raise RateProbeError("ack_no_motion requires explicit --rbscript-no-motion-command")
        if backend_probe.command_text_looks_motion_capable(args.rbscript_no_motion_command):
            raise RateProbeError("refusing ack_no_motion command text with motion-capable token")
    if args.capture_raw_data_port:
        if args.backend != "rbscript_tcp" or args.mode != "read_state":
            raise RateProbeError("--capture-raw-data-port requires --backend rbscript_tcp --mode read_state")

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
        persistent_socket=getattr(args, "persistent_socket", False),
        rbscript_no_motion_command=args.rbscript_no_motion_command,
        capture_raw_data_port=getattr(args, "capture_raw_data_port", False),
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
    safety["persistent_socket"] = bool(getattr(args, "persistent_socket", False) and args.backend == "rbscript_tcp")
    safety["capture_raw_data_port"] = bool(getattr(args, "capture_raw_data_port", False))
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
        persistent_socket=getattr(args, "persistent_socket", False),
        rbscript_no_motion_command=args.rbscript_no_motion_command,
        capture_raw_data_port=getattr(args, "capture_raw_data_port", False),
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


def integer(value: Any) -> int | None:
    if isinstance(value, bool):
        return int(value)
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def controller_mode_name(value: Any) -> str:
    mode = integer(value)
    if mode == 1:
        return "simulation"
    if mode == 0:
        return "real"
    return "unknown"


def finite_joint_list(value: Any) -> list[float] | None:
    try:
        items = list(value)
    except TypeError:
        return None
    if len(items) != 6:
        return None
    joints: list[float] = []
    for item in items:
        if isinstance(item, bool):
            return None
        try:
            number = float(item)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(number):
            return None
        joints.append(number)
    return joints


def diagnostics_from_state(sdata: Any) -> dict[str, Any]:
    return {field: getattr(sdata, field, None) for field in RBPODO_SERVO_J_DIAGNOSTIC_FIELDS}


def read_rbpodo_state(data_channel: Any, timeout_sec: float) -> RbpodoStateRead:
    started_ns = backend_probe.now_ns()
    try:
        state = data_channel.request_data(timeout_sec)
        ended_ns = backend_probe.now_ns()
        if state is None:
            raise RateProbeError("CobotData.request_data returned no state")
        sdata = getattr(state, "sdata", state)
        diagnostics = diagnostics_from_state(sdata)
        raw_mode = diagnostics.get("real_vs_simulation_mode")
        return RbpodoStateRead(
            success=True,
            duration_us=(ended_ns - started_ns) / 1000.0,
            q_actual_deg=finite_joint_list(getattr(sdata, "jnt_ang", [])),
            q_ref_deg=finite_joint_list(getattr(sdata, "jnt_ref", [])),
            controller_mode=controller_mode_name(raw_mode),
            raw_mode=raw_mode,
            diagnostics=diagnostics,
        )
    except Exception as exc:
        ended_ns = backend_probe.now_ns()
        return RbpodoStateRead(
            success=False,
            duration_us=(ended_ns - started_ns) / 1000.0,
            q_actual_deg=None,
            q_ref_deg=None,
            controller_mode="unknown",
            raw_mode=None,
            diagnostics={},
            error_name=type(exc).__name__,
            error_message=str(exc),
        )


def load_rbpodo_module() -> Any:
    try:
        rbpodo = importlib.import_module("rbpodo")
    except Exception as exc:
        raise RateProbeError("Python rbpodo module is unavailable") from exc
    required = ("Cobot", "CobotData", "ResponseCollector")
    missing = [name for name in required if not hasattr(rbpodo, name)]
    if missing:
        raise RateProbeError("Python rbpodo module is missing: " + ", ".join(missing))
    return rbpodo


def collector_has_error(response_collector: Any) -> bool:
    has_error = getattr(response_collector, "has_error", None)
    return bool(has_error()) if callable(has_error) else False


def return_success(ret: Any) -> bool:
    is_success = getattr(ret, "is_success", None)
    if callable(is_success):
        return bool(is_success())
    return bool(ret)


def return_timeout(ret: Any) -> bool:
    is_timeout = getattr(ret, "is_timeout", None)
    return bool(is_timeout()) if callable(is_timeout) else False


def response_text(response_collector: Any, ret: Any) -> str:
    parts: list[str] = []
    for value in (response_collector, ret):
        text = str(value).strip()
        if text and not text.startswith("<") and text not in {"True", "False"}:
            parts.append(text)
    return "\n".join(parts)


def response_error_names(text: str) -> list[str]:
    lowered = text.lower()
    names = [code for code in backend_probe.M_CODES if code.lower() in lowered]
    if ("not allowed" in lowered or "error" in lowered or "fail" in lowered) and not names:
        names.append("controller_error")
    return names


def configure_waiting_ack(control: Any, rbpodo: Any) -> None:
    responses = rbpodo.ResponseCollector()
    configured = control.enable_waiting_ack(responses)
    if not configured or collector_has_error(responses):
        details = str(responses).strip()
        suffix = f": {details}" if details else ""
        raise RateProbeError("rbpodo enable_waiting_ack was not accepted" + suffix)


def joint_payload(q_target_deg: list[float]) -> Any:
    try:
        import numpy as np

        return np.asarray(q_target_deg, dtype=float)
    except Exception:
        return list(q_target_deg)


def servo_j_schedule_entry(safety: dict[str, Any], rate: float) -> dict[str, Any]:
    for entry in safety.get("send_rate_schedule", []):
        if abs(float(entry.get("command_rate_hz")) - rate) < 1e-9:
            return entry
    raise RateProbeError(f"missing Servo J schedule entry for {rate} Hz")


def q_drift_max_deg(baseline: list[float], q_actual: list[float] | None) -> float | None:
    if q_actual is None:
        return None
    return max(abs(current - start) for current, start in zip(q_actual, baseline))


def servo_j_sample_row(
    *,
    index: int,
    args: argparse.Namespace,
    rate: float,
    read: RbpodoStateRead,
    loop_start_ns: int,
    loop_end_ns: int,
    previous_loop_start_ns: int | None,
    drift_deg: float | None,
    send_requested: bool,
    send_success: bool | None,
    send_start_ns: int | None,
    send_end_ns: int | None,
    send_duration_us: float | None,
    servo_t1_sec: float,
    error_name: str,
    error_message: str,
    response: str,
    names: list[str],
) -> dict[str, Any]:
    q_actual = read.q_actual_deg or [None] * 6
    q_ref = read.q_ref_deg or [None] * 6
    row = {
        "index": index,
        "mode": args.mode,
        "backend": args.backend,
        "requested_rate_hz": rate,
        "feedback_rate_hz": args.feedback_rate_hz,
        "loop_start_ns": loop_start_ns,
        "loop_end_ns": loop_end_ns,
        "loop_interval_ms": (
            (loop_start_ns - previous_loop_start_ns) / 1e6
            if previous_loop_start_ns is not None and loop_start_ns >= previous_loop_start_ns
            else None
        ),
        "read_duration_us": read.duration_us,
        "state_valid": read.success and read.q_actual_deg is not None,
        "controller_mode": read.controller_mode,
        "raw_controller_mode": read.raw_mode,
        "q_actual_drift_max_deg": drift_deg,
        "send_requested": send_requested,
        "send_success": send_success,
        "send_start_ns": send_start_ns,
        "send_end_ns": send_end_ns,
        "send_duration_us": send_duration_us,
        "servo_t1_sec": servo_t1_sec,
        "servo_t2_sec": args.servo_t2_sec,
        "servo_gain": args.servo_gain,
        "servo_alpha": args.servo_alpha,
        "error_name": error_name,
        "error_message": error_message,
        "response": response,
        "response_error_names": names,
    }
    for joint_index in range(6):
        row[f"q_actual_{joint_index}"] = q_actual[joint_index]
        row[f"q_ref_{joint_index}"] = q_ref[joint_index]
    return row


def summarize_servo_j_samples(
    args: argparse.Namespace,
    safety: dict[str, Any],
    rate: float,
    samples: list[dict[str, Any]],
    result: str,
    result_reason: str,
) -> dict[str, Any]:
    elapsed_sec = (
        (samples[-1]["loop_end_ns"] - samples[0]["loop_start_ns"]) / 1e9
        if len(samples) >= 2
        else 0.0
    )
    send_samples = [sample for sample in samples if sample["send_requested"]]
    send_successes = [sample for sample in send_samples if sample["send_success"] is True]
    m_counts: dict[str, int] = {code: 0 for code in backend_probe.M_CODES}
    error_counts: dict[str, int] = {}
    for sample in samples:
        names = set(sample.get("response_error_names") or [])
        if sample.get("error_name"):
            names.add(sample["error_name"])
        for name in names:
            if name in m_counts:
                m_counts[name] += 1
            error_counts[name] = error_counts.get(name, 0) + 1
    loop_intervals = [sample["loop_interval_ms"] for sample in samples if sample["loop_interval_ms"] is not None]
    read_durations = [sample["read_duration_us"] for sample in samples]
    send_durations = [
        sample["send_duration_us"] for sample in send_samples if sample["send_duration_us"] is not None
    ]
    drift_values = [
        sample["q_actual_drift_max_deg"]
        for sample in samples
        if sample["q_actual_drift_max_deg"] is not None
    ]
    return {
        "backend": args.backend,
        "mode": args.mode,
        "target_ip": args.ip,
        "requested_rate_hz": rate,
        "feedback_rate_hz": args.feedback_rate_hz,
        "duration_sec": args.duration_sec,
        "achieved_feedback_rate_hz": (len(samples) / elapsed_sec) if elapsed_sec > 0.0 else None,
        "achieved_send_rate_hz": (len(send_samples) / elapsed_sec) if elapsed_sec > 0.0 else None,
        "feedback_sample_count": len(samples),
        "send_count": len(send_samples),
        "send_success_count": len(send_successes),
        "send_failure_count": len(send_samples) - len(send_successes),
        "send_success_rate": len(send_successes) / len(send_samples) if send_samples else 0.0,
        "loop_interval_ms": backend_probe.metric_block(loop_intervals),
        "read_duration_us": backend_probe.metric_block(read_durations),
        "send_duration_us": backend_probe.metric_block(send_durations),
        "q_actual_drift_max_deg": max(drift_values) if drift_values else None,
        "m_code_counts": m_counts,
        "response_error_names": error_counts,
        "result": result,
        "result_reason": result_reason,
        "safety_preflight": safety,
    }


def run_servo_j_simulation_rate(
    args: argparse.Namespace,
    safety: dict[str, Any],
    rate: float,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    rbpodo = load_rbpodo_module()
    data_channel = rbpodo.CobotData(args.ip)
    control = rbpodo.Cobot(args.ip)
    configure_waiting_ack(control, rbpodo)

    initial_read: RbpodoStateRead | None = None
    for _ in range(5):
        candidate = read_rbpodo_state(data_channel, args.read_timeout_sec)
        if candidate.success and candidate.q_actual_deg is not None:
            initial_read = candidate
            break
        time.sleep(min(args.read_timeout_sec, 0.01))
    if initial_read is None:
        raise RateProbeError("unable to read finite initial rbpodo joint state")
    if initial_read.controller_mode != "simulation":
        raise RateProbeError(
            "controller not confirmed in pgmode simulation at Servo J start: "
            f"real_vs_simulation_mode={initial_read.raw_mode}"
        )

    q_target = list(initial_read.q_actual_deg)
    baseline = list(initial_read.q_actual_deg)
    schedule = servo_j_schedule_entry(safety, rate)
    send_interval_ticks = int(schedule["send_interval_feedback_ticks"])
    servo_t1_sec = float(schedule["servo_t1_sec"])
    feedback_period_sec = 1.0 / args.feedback_rate_hz
    samples: list[dict[str, Any]] = []
    result = "completed"
    result_reason = ""
    deadline = time.monotonic() + args.duration_sec
    next_time = time.monotonic()
    previous_loop_start_ns: int | None = None
    index = 0
    while time.monotonic() < deadline:
        loop_start_ns = backend_probe.now_ns()
        read = read_rbpodo_state(data_channel, args.read_timeout_sec)
        drift = q_drift_max_deg(baseline, read.q_actual_deg)
        send_requested = index % send_interval_ticks == 0
        send_success: bool | None = None
        send_start_ns: int | None = None
        send_end_ns: int | None = None
        send_duration_us: float | None = None
        error_name = read.error_name
        error_message = read.error_message
        response = ""
        names: list[str] = []

        if not read.success or read.q_actual_deg is None:
            result = "stopped"
            result_reason = f"state read failed: {read.error_name or 'invalid_joint_state'}"
            if not error_name:
                error_name = "invalid_joint_state"
                error_message = "CobotData.request_data did not return six finite joints"
        elif read.controller_mode != "simulation":
            result = "stopped"
            result_reason = f"controller left pgmode simulation: real_vs_simulation_mode={read.raw_mode}"
            error_name = "controller_not_simulation"
            error_message = result_reason
        elif drift is not None and drift > args.max_q_actual_drift_deg:
            result = "stopped"
            result_reason = (
                f"q_actual drift {drift:.6f} deg exceeded limit "
                f"{args.max_q_actual_drift_deg:.6f} deg"
            )
            error_name = "q_actual_drift_exceeded"
            error_message = result_reason
        elif send_requested:
            responses = rbpodo.ResponseCollector()
            send_start_ns = backend_probe.now_ns()
            try:
                ret = control.move_servo_j(
                    responses,
                    joint_payload(q_target),
                    servo_t1_sec,
                    args.servo_t2_sec,
                    args.servo_gain,
                    args.servo_alpha,
                    args.command_timeout_sec,
                    True,
                )
                send_end_ns = backend_probe.now_ns()
                send_duration_us = (send_end_ns - send_start_ns) / 1000.0
                response = response_text(responses, ret)
                names = response_error_names(response)
                send_success = return_success(ret) and not collector_has_error(responses) and not names
                if not send_success:
                    result = "stopped"
                    if return_timeout(ret):
                        error_name = "TransportTimeout"
                    elif collector_has_error(responses) or names:
                        error_name = names[0] if names else "controller_response_error"
                    else:
                        error_name = "ControllerRejected"
                    result_reason = f"move_servo_j failed at sample {index}: {error_name}"
                    error_message = response
            except Exception as exc:
                send_end_ns = backend_probe.now_ns()
                send_duration_us = (send_end_ns - send_start_ns) / 1000.0
                send_success = False
                error_name = type(exc).__name__
                error_message = str(exc)
                result = "stopped"
                result_reason = f"move_servo_j exception at sample {index}: {error_name}"

        loop_end_ns = backend_probe.now_ns()
        samples.append(servo_j_sample_row(
            index=index,
            args=args,
            rate=rate,
            read=read,
            loop_start_ns=loop_start_ns,
            loop_end_ns=loop_end_ns,
            previous_loop_start_ns=previous_loop_start_ns,
            drift_deg=drift,
            send_requested=send_requested,
            send_success=send_success,
            send_start_ns=send_start_ns,
            send_end_ns=send_end_ns,
            send_duration_us=send_duration_us,
            servo_t1_sec=servo_t1_sec,
            error_name=error_name,
            error_message=error_message,
            response=response,
            names=names,
        ))
        previous_loop_start_ns = loop_start_ns
        if result != "completed":
            break
        index += 1
        next_time += feedback_period_sec
        sleep_sec = next_time - time.monotonic()
        if sleep_sec > 0.0:
            time.sleep(sleep_sec)

    summary = summarize_servo_j_samples(args, safety, rate, samples, result, result_reason)
    return samples, summary


def capture_raw_data_port(args: argparse.Namespace, artifact_dir: Path) -> dict[str, Any]:
    artifact_dir.mkdir(parents=True, exist_ok=True)
    started_ns = backend_probe.now_ns()
    raw = b""
    error_name = ""
    error_message = ""
    try:
        with backend_probe.tcp_connect(args.ip, args.data_port, args.connect_timeout_sec) as sock:
            sock.settimeout(args.read_timeout_sec)
            sock.sendall(b"reqdata\n")
            while len(raw) < 65536:
                try:
                    chunk = sock.recv(min(4096, 65536 - len(raw)))
                except TimeoutError:
                    if raw:
                        break
                    raise
                if not chunk:
                    break
                raw += chunk
                if b"\n" in chunk:
                    break
    except TimeoutError as exc:
        error_name = "TransportTimeout"
        error_message = str(exc)
    except OSError as exc:
        error_name = type(exc).__name__
        error_message = str(exc)
    ended_ns = backend_probe.now_ns()

    binary_path = artifact_dir / "raw_data_port_capture.bin"
    text_path = artifact_dir / "raw_data_port_capture.txt"
    metadata_path = artifact_dir / "raw_data_port_capture.json"
    binary_path.write_bytes(raw)
    text_path.write_text(raw.decode("utf-8", errors="replace"), encoding="utf-8")
    metadata = {
        "backend": args.backend,
        "mode": args.mode,
        "ip": args.ip,
        "data_port": args.data_port,
        "request": "reqdata",
        "byte_count": len(raw),
        "started_ns": started_ns,
        "ended_ns": ended_ns,
        "duration_us": (ended_ns - started_ns) / 1000.0,
        "error_name": error_name,
        "error_message": error_message,
        "state_valid": False,
        "read_state_capability": backend_probe.READ_STATE_CAPABILITY_UNSUPPORTED,
        "rbscript_tcp_data_port_mode": backend_probe.RBSCRIPT_DATA_PORT_MODE_REAL_UNSUPPORTED,
        "comparable": False,
        "not_comparable_reason": backend_probe.RBSCRIPT_READ_STATE_NOT_COMPARABLE_REASON,
        "artifacts": {
            "raw_bytes": str(binary_path),
            "raw_text": str(text_path),
        },
    }
    metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return metadata


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
    response_lines = summary.get("response_lines_per_command") if isinstance(summary.get("response_lines_per_command"), dict) else {}
    write_duration = summary.get("command_write_duration_us") if isinstance(summary.get("command_write_duration_us"), dict) else {}
    ack_read_duration = summary.get("ack_read_duration_us") if isinstance(summary.get("ack_read_duration_us"), dict) else {}
    return {
        "requested_rate_hz": summary.get("requested_rate_hz"),
        "achieved_rate_hz": summary.get("achieved_rate_hz"),
        "persistent_socket": summary.get("persistent_socket"),
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
        "stale_response_count": summary.get("stale_response_count"),
        "extra_response_count": summary.get("extra_response_count"),
        "unrecognized_response_count": summary.get("unrecognized_response_count"),
        "response_lines_per_command_p50": response_lines.get("p50"),
        "response_lines_per_command_p95": response_lines.get("p95"),
        "response_lines_per_command_max": response_lines.get("max"),
        "command_write_duration_us_p50": write_duration.get("p50"),
        "command_write_duration_us_p95": write_duration.get("p95"),
        "command_write_duration_us_max": write_duration.get("max"),
        "ack_read_duration_us_p50": ack_read_duration.get("p50"),
        "ack_read_duration_us_p95": ack_read_duration.get("p95"),
        "ack_read_duration_us_max": ack_read_duration.get("max"),
        "data_success_count": summary.get("state_valid_count") if summary.get("mode") == "read_state" else 0,
        "data_timeout_count": summary.get("data_port_timeout_count") if summary.get("mode") == "read_state" else 0,
        "success_rate": summary.get("success_rate"),
        "read_state_capability": summary.get("read_state_capability"),
        "rbscript_tcp_data_port_mode": summary.get("rbscript_tcp_data_port_mode"),
        "comparable": summary.get("comparable"),
        "not_comparable_reason": summary.get("not_comparable_reason"),
    }


def write_samples_csv(path: Path, samples: list[backend_probe.Sample]) -> None:
    fieldnames = [
        "index", "mode", "backend", "success", "duration_us", "start_ns", "end_ns",
        "error_name", "error_message", "state_valid", "q_actual_finite", "state_age_ms",
        "response_line_count", "extra_response_count", "stale_response",
        "unrecognized_response_count", "command_write_duration_us", "ack_read_duration_us",
        "reconnect_count", "persistent_socket", "rbscript_tcp_data_port_mode",
        "read_state_capability", "comparable", "not_comparable_reason",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for sample in samples:
            writer.writerow(backend_probe.sample_row(sample))


def write_responses_jsonl(path: Path, samples: list[backend_probe.Sample]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        for sample in samples:
            if sample.response or sample.response_lines or sample.extra_response_lines or sample.stale_response_lines or sample.error_name:
                handle.write(json.dumps({
                    "index": sample.index,
                    "success": sample.success,
                    "response": sample.response,
                    "response_lines": sample.response_lines,
                    "extra_response_lines": sample.extra_response_lines,
                    "stale_response_lines": sample.stale_response_lines,
                    "stale_response": sample.stale_response,
                    "error_name": sample.error_name,
                    "error_message": sample.error_message,
                    "response_error_names": sample.response_error_names,
                    "rbscript_tcp_data_port_mode": sample.rbscript_tcp_data_port_mode,
                    "read_state_capability": sample.read_state_capability,
                    "comparable": sample.comparable,
                    "not_comparable_reason": sample.not_comparable_reason,
                }) + "\n")


def write_summary_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fieldnames = [
        "requested_rate_hz", "achieved_rate_hz", "persistent_socket", "send_count", "ack_success_count",
        "ack_timeout_count", "ack_error_count", "p50_ack_us", "p95_ack_us",
        "p99_ack_us", "max_ack_us", "loop_interval_p50_ms", "loop_interval_p95_ms",
        "loop_interval_max_ms", "m561_count", "m568_count", "m569_count",
        "m570_count", "other_error_counts", "reconnect_count", "stale_response_count",
        "extra_response_count", "unrecognized_response_count", "response_lines_per_command_p50",
        "response_lines_per_command_p95", "response_lines_per_command_max",
        "command_write_duration_us_p50", "command_write_duration_us_p95",
        "command_write_duration_us_max", "ack_read_duration_us_p50",
        "ack_read_duration_us_p95", "ack_read_duration_us_max", "data_success_count",
        "data_timeout_count", "success_rate", "read_state_capability",
        "rbscript_tcp_data_port_mode", "comparable", "not_comparable_reason",
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
    raw_capture: dict[str, Any] | None = None,
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
        "raw_data_port_capture": raw_capture,
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
            f"raw_data_port_capture: {bool(raw_capture)}",
            "No motion is commanded by default. ACK/read success is not motion readiness.",
            "",
        ]),
        encoding="utf-8",
    )
    if not args.skip_plots and rows:
        write_plots(artifact_dir, rows)
    return summary


def servo_j_rate_label(rate: float) -> str:
    return str(int(rate)) if float(rate).is_integer() else str(rate).replace(".", "p")


def servo_j_summary_row(summary: dict[str, Any]) -> dict[str, Any]:
    loop = summary.get("loop_interval_ms") if isinstance(summary.get("loop_interval_ms"), dict) else {}
    read = summary.get("read_duration_us") if isinstance(summary.get("read_duration_us"), dict) else {}
    send = summary.get("send_duration_us") if isinstance(summary.get("send_duration_us"), dict) else {}
    m_counts = summary.get("m_code_counts") if isinstance(summary.get("m_code_counts"), dict) else {}
    errors = summary.get("response_error_names") if isinstance(summary.get("response_error_names"), dict) else {}
    return {
        "requested_rate_hz": summary.get("requested_rate_hz"),
        "feedback_rate_hz": summary.get("feedback_rate_hz"),
        "achieved_feedback_rate_hz": summary.get("achieved_feedback_rate_hz"),
        "achieved_send_rate_hz": summary.get("achieved_send_rate_hz"),
        "feedback_sample_count": summary.get("feedback_sample_count"),
        "send_count": summary.get("send_count"),
        "send_success_count": summary.get("send_success_count"),
        "send_failure_count": summary.get("send_failure_count"),
        "send_success_rate": summary.get("send_success_rate"),
        "loop_interval_p50_ms": loop.get("p50"),
        "loop_interval_p95_ms": loop.get("p95"),
        "loop_interval_p99_ms": loop.get("p99"),
        "loop_interval_max_ms": loop.get("max"),
        "read_duration_p50_us": read.get("p50"),
        "read_duration_p95_us": read.get("p95"),
        "read_duration_p99_us": read.get("p99"),
        "read_duration_max_us": read.get("max"),
        "send_duration_p50_us": send.get("p50"),
        "send_duration_p95_us": send.get("p95"),
        "send_duration_p99_us": send.get("p99"),
        "send_duration_max_us": send.get("max"),
        "q_actual_drift_max_deg": summary.get("q_actual_drift_max_deg"),
        "m561_count": m_counts.get("M561", 0),
        "m568_count": m_counts.get("M568", 0),
        "m569_count": m_counts.get("M569", 0),
        "m570_count": m_counts.get("M570", 0),
        "other_error_counts": {
            key: value
            for key, value in errors.items()
            if key not in set(backend_probe.M_CODES)
        },
        "result": summary.get("result"),
        "result_reason": summary.get("result_reason"),
    }


def write_servo_j_samples_csv(path: Path, samples: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=SERVO_J_SAMPLE_FIELDS)
        writer.writeheader()
        for sample in samples:
            row = dict(sample)
            row["response_error_names"] = json.dumps(row.get("response_error_names") or [])
            writer.writerow({field: row.get(field) for field in SERVO_J_SAMPLE_FIELDS})


def write_servo_j_responses_jsonl(path: Path, samples: list[dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        for sample in samples:
            if sample.get("send_requested") or sample.get("error_name") or sample.get("response"):
                handle.write(json.dumps({
                    "index": sample.get("index"),
                    "requested_rate_hz": sample.get("requested_rate_hz"),
                    "feedback_rate_hz": sample.get("feedback_rate_hz"),
                    "send_requested": sample.get("send_requested"),
                    "send_success": sample.get("send_success"),
                    "send_duration_us": sample.get("send_duration_us"),
                    "error_name": sample.get("error_name"),
                    "error_message": sample.get("error_message"),
                    "response": sample.get("response"),
                    "response_error_names": sample.get("response_error_names"),
                    "controller_mode": sample.get("controller_mode"),
                    "raw_controller_mode": sample.get("raw_controller_mode"),
                }, sort_keys=True) + "\n")


def write_servo_j_summary_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fieldnames = [
        "requested_rate_hz", "feedback_rate_hz", "achieved_feedback_rate_hz",
        "achieved_send_rate_hz", "feedback_sample_count", "send_count",
        "send_success_count", "send_failure_count", "send_success_rate",
        "loop_interval_p50_ms", "loop_interval_p95_ms", "loop_interval_p99_ms",
        "loop_interval_max_ms", "read_duration_p50_us", "read_duration_p95_us",
        "read_duration_p99_us", "read_duration_max_us", "send_duration_p50_us",
        "send_duration_p95_us", "send_duration_p99_us", "send_duration_max_us",
        "q_actual_drift_max_deg", "m561_count", "m568_count", "m569_count",
        "m570_count", "other_error_counts", "result", "result_reason",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            flat = dict(row)
            flat["other_error_counts"] = json.dumps(flat.get("other_error_counts") or {}, sort_keys=True)
            writer.writerow(flat)


def write_servo_j_plots(
    artifact_dir: Path,
    rows: list[dict[str, Any]],
    per_rate: list[dict[str, Any]],
) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        (artifact_dir / "plot_skip_reason.txt").write_text(f"matplotlib unavailable: {exc}\n", encoding="utf-8")
        return

    rates = [float(row["requested_rate_hz"]) for row in rows]
    loop_p95 = [row.get("loop_interval_p95_ms") for row in rows]
    loop_max = [row.get("loop_interval_max_ms") for row in rows]
    send_p95 = [row.get("send_duration_p95_us") for row in rows]
    send_max = [row.get("send_duration_max_us") for row in rows]
    success = [row.get("send_success_rate") for row in rows]

    plt.figure(figsize=(8, 4))
    plt.plot(rates, loop_p95, marker="o", label="p95")
    plt.plot(rates, loop_max, marker="o", label="max")
    plt.xlabel("Servo J requested rate Hz")
    plt.ylabel("500 Hz feedback loop interval ms")
    plt.legend()
    plt.tight_layout()
    plt.savefig(artifact_dir / "feedback_loop_interval_by_rate.png")
    plt.close()

    plt.figure(figsize=(8, 4))
    plt.plot(rates, send_p95, marker="o", label="p95")
    plt.plot(rates, send_max, marker="o", label="max")
    plt.xlabel("Servo J requested rate Hz")
    plt.ylabel("move_servo_j duration us")
    plt.legend()
    plt.tight_layout()
    plt.savefig(artifact_dir / "send_duration_by_rate.png")
    plt.close()

    plt.figure(figsize=(8, 4))
    plt.plot(rates, success, marker="o")
    plt.xlabel("Servo J requested rate Hz")
    plt.ylabel("send success rate")
    plt.ylim(-0.05, 1.05)
    plt.tight_layout()
    plt.savefig(artifact_dir / "send_success_by_rate.png")
    plt.close()

    for item in per_rate:
        samples = item["samples"]
        if not samples:
            continue
        x = [sample["index"] for sample in samples]
        actual = [sample.get("q_actual_0") for sample in samples]
        ref = [sample.get("q_ref_0") for sample in samples]
        plt.figure(figsize=(8, 4))
        plt.plot(x, actual, label="q_actual_j0")
        plt.plot(x, ref, label="q_ref_j0")
        plt.xlabel("feedback sample index")
        plt.ylabel("joint 0 deg")
        plt.legend()
        plt.tight_layout()
        plt.savefig(artifact_dir / f"q_ref_actual_j0_{servo_j_rate_label(float(item['rate']))}.png")
        plt.close()


def write_servo_j_artifacts(
    args: argparse.Namespace,
    safety: dict[str, Any],
    per_rate: list[dict[str, Any]],
    rows: list[dict[str, Any]],
) -> dict[str, Any]:
    artifact_dir = args.artifact_dir.resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    for item in per_rate:
        rate = float(item["rate"])
        label = servo_j_rate_label(rate)
        write_servo_j_samples_csv(artifact_dir / f"samples_servo_j_{label}.csv", item["samples"])
        write_servo_j_responses_jsonl(artifact_dir / f"responses_servo_j_{label}.jsonl", item["samples"])

    result = "completed" if rows and all(row.get("result") == "completed" for row in rows) else "stopped"
    summary = {
        "backend": args.backend,
        "mode": args.mode,
        "ip": args.ip,
        "rates": safety["rates"],
        "feedback_rate_hz": args.feedback_rate_hz,
        "duration_sec": args.duration_sec,
        "artifact_dir": str(artifact_dir),
        "safety_preflight": safety,
        "rate_results": rows,
        "result": result,
        "caveat": (
            "Servo J no-op controller-simulation rate evidence only. This connects "
            "to Rainbow controller boxes in pgmode simulation and is not physical "
            "motion readiness."
        ),
    }
    (artifact_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    write_servo_j_summary_csv(artifact_dir / "summary.csv", rows)
    (artifact_dir / "README.txt").write_text(
        "\n".join([
            "rainbow_rate_probe Servo J controller-simulation artifacts",
            "",
            "command: " + " ".join(sys.argv),
            f"backend: {args.backend}",
            f"mode: {args.mode}",
            f"feedback_rate_hz: {args.feedback_rate_hz}",
            "Servo J target is the initial measured joint position.",
            "Controller pgmode simulation is required before any command is sent.",
            "Physical motion is not approved or expected from this artifact set.",
            "",
        ]),
        encoding="utf-8",
    )
    if not args.skip_plots and rows:
        write_servo_j_plots(artifact_dir, rows, per_rate)
    return summary


def main() -> int:
    args = parse_args()
    try:
        safety = preflight(args)
        print(json.dumps({"event": "safety_preflight", **safety}, indent=2, sort_keys=True))
        if safety.get("capture_raw_data_port"):
            raw_capture = capture_raw_data_port(args, args.artifact_dir.resolve())
            summary = write_artifacts(args, safety, [], [], raw_capture=raw_capture)
            print(json.dumps({"event": "raw_data_port_capture", **raw_capture}, sort_keys=True))
            print(json.dumps(summary, indent=2, sort_keys=True))
            return 0 if not raw_capture.get("error_name") else 1
        if safety.get("rate_probe_mode") == "servo_j_simulation_only":
            per_rate: list[dict[str, Any]] = []
            rows: list[dict[str, Any]] = []
            for rate in safety["rates"]:
                samples, rate_summary = run_servo_j_simulation_rate(args, safety, rate)
                row = servo_j_summary_row(rate_summary)
                per_rate.append({"rate": rate, "samples": samples, "summary": rate_summary})
                rows.append(row)
                print(json.dumps({"event": "rate_complete", **row}, sort_keys=True))
                if row.get("result") != "completed":
                    break
            summary = write_servo_j_artifacts(args, safety, per_rate, rows)
            print(json.dumps(summary, indent=2, sort_keys=True))
            return 0 if summary.get("result") == "completed" else 1
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
