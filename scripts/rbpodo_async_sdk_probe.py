#!/usr/bin/env python3
"""Probe Python rbpodo SDK assumptions for async ACK-supervised streaming."""

from __future__ import annotations

import argparse
import csv
import importlib
import inspect
import json
import math
import os
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


SCHEMA = "robotics_lab.rbpodo_async_sdk_probe.v1"
SDK_CAPABILITIES_SCHEMA = "robotics_lab.rbpodo_async_sdk_capabilities.v1"
REAL_ROBOT_IPS = {"172.28.60.200", "172.28.60.201"}
REQUIRED_ENV = (
    "RB_ALLOW_REAL_ROBOT",
    "RB_ALLOW_REAL_MOTION",
    "RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION",
)
ACK_DISABLED_ENV = "RB_ALLOW_RBPODO_ACK_DISABLED_MOTION"
M_CODES = ("M561", "M568", "M569", "M570")
DIAGNOSTIC_FIELDS = (
    "time",
    "real_vs_simulation_mode",
    "init_state_info",
    "init_error",
    "op_stat_sos_flag",
    "op_stat_ems_flag",
    "op_stat_soft_estop_occur",
    "op_stat_collision_occur",
    "op_stat_self_collision",
)
SEND_FIELDNAMES = [
    "index",
    "mode",
    "requested_rate_hz",
    "send_start_ns",
    "send_end_ns",
    "send_duration_us",
    "send_success",
    "ack_policy",
    "ack_observed",
    "socket_send_only",
    "target_source",
    "servo_t1_sec",
    "servo_t2_sec",
    "servo_gain",
    "servo_alpha",
    "command_timeout_sec",
    "error_name",
    "error_message",
    "response",
    "response_error_names",
]
STATE_FIELDNAMES = [
    "index",
    "mode",
    "sample_role",
    "host_time_ns",
    "read_duration_us",
    "state_valid",
    "controller_mode",
    "raw_controller_mode",
    "q_actual_drift_deg",
    "q_ref_drift_deg",
    "q_actual_target_error_deg",
    "q_ref_target_error_deg",
    "error_name",
    "error_message",
    "q_actual_0",
    "q_actual_1",
    "q_actual_2",
    "q_actual_3",
    "q_actual_4",
    "q_actual_5",
    "q_ref_0",
    "q_ref_1",
    "q_ref_2",
    "q_ref_3",
    "q_ref_4",
    "q_ref_5",
]


class AsyncSdkProbeError(RuntimeError):
    pass


@dataclass
class ProbeRun:
    send_samples: list[dict[str, Any]]
    state_samples: list[dict[str, Any]]
    response_events: list[dict[str, Any]]
    sdk_capabilities: dict[str, Any]
    run_notes: list[str]
    deadlock_observed: bool = False
    late_ack_observed_after_ack_off: bool | str = "unknown"
    sdk_thread_safety_observed: bool | str = "unknown"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Characterize Python rbpodo SDK behavior relevant to future async "
            "ACK-supervised streaming. The probe connects to real Rainbow "
            "controller boxes only in pgmode simulation and sends no-op Servo J "
            "targets equal to the current controller/actual joint state."
        )
    )
    parser.add_argument("--ip", required=True)
    parser.add_argument("--duration-sec", type=float, default=5.0)
    parser.add_argument("--rate-hz", type=float, default=500.0)
    parser.add_argument(
        "--mode",
        choices=("ack_on", "ack_off", "concurrent_read_send"),
        required=True,
    )
    parser.add_argument("--artifact-dir", type=Path, required=True)
    parser.add_argument("--operation-mode", default="simulation")
    parser.add_argument(
        "--config",
        type=Path,
        help="Optional YAML config to sanity-check operation_mode fields; no config is modified.",
    )
    parser.add_argument("--read-timeout-sec", type=float, default=0.1)
    parser.add_argument("--command-timeout-sec", type=float, default=0.02)
    parser.add_argument("--servo-t2-sec", type=float, default=0.03)
    parser.add_argument("--servo-gain", type=float, default=1.0)
    parser.add_argument("--servo-alpha", type=float, default=0.5)
    parser.add_argument("--max-noop-target-delta-deg", type=float, default=0.05)
    parser.add_argument("--max-q-actual-drift-deg", type=float, default=0.05)
    parser.add_argument("--late-ack-poll-sec", type=float, default=0.02)
    parser.add_argument("--set-pgmode-simulation", action="store_true")
    parser.add_argument("--verify-pgmode-simulation", action="store_true")
    parser.add_argument("--pgmode-timeout-sec", type=float, default=1.0)
    parser.add_argument("--pgmode-command-port", type=int, default=5000)
    parser.add_argument(
        "--i-understand-this-connects-to-real-controller",
        action="store_true",
        help="Required before any controller connection.",
    )
    parser.add_argument(
        "--allow-simulation-servo-j",
        action="store_true",
        help="Required before no-op Servo J commands in controller pgmode simulation.",
    )
    return parser.parse_args()


def now_ns() -> int:
    return time.monotonic_ns()


def env_enabled(name: str) -> bool:
    return os.environ.get(name) == "1"


def env_snapshot() -> dict[str, str | None]:
    keys = list(REQUIRED_ENV) + [
        ACK_DISABLED_ENV,
        "RB_ALLOW_REAL_CARTESIAN",
        "RB_RBPODO_PGMODE_SIMULATION_CONFIRMED",
    ]
    return {key: os.environ.get(key) for key in keys}


def finite_positive(value: float, name: str) -> None:
    if not math.isfinite(value) or value <= 0.0:
        raise AsyncSdkProbeError(f"{name} must be finite and positive")


def finite_port(value: int, name: str) -> None:
    if value < 1 or value > 65535:
        raise AsyncSdkProbeError(f"{name} must be in [1, 65535]")


def operation_mode_is_simulation(value: Any) -> bool:
    return str(value).strip().lower() in {"simulation", "sim"}


def operation_mode_is_real(value: Any) -> bool:
    return str(value).strip().lower() == "real"


def operation_modes_from_config(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        raise AsyncSdkProbeError(f"config not found: {path}")
    try:
        import yaml  # type: ignore
    except Exception:
        return operation_modes_from_config_fallback(path)
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise AsyncSdkProbeError(f"failed to parse config {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise AsyncSdkProbeError("config must be a YAML object")
    modes: list[dict[str, str]] = []
    for key, value in data.items():
        if key == "operation_mode" and value is not None:
            modes.append({"scope": key, "operation_mode": str(value)})
        if isinstance(value, dict) and "operation_mode" in value:
            modes.append({"scope": str(key), "operation_mode": str(value.get("operation_mode"))})
    return modes


def operation_modes_from_config_fallback(path: Path) -> list[dict[str, str]]:
    modes: list[dict[str, str]] = []
    current = "<root>"
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip(" "))
        text = line.strip()
        if indent == 0 and text.endswith(":"):
            current = text[:-1]
            continue
        if ":" not in text:
            continue
        key, value = text.split(":", 1)
        if key.strip() == "operation_mode":
            modes.append({"scope": current, "operation_mode": value.strip().strip("'\"")})
    return modes


def validate_operation_modes(args: argparse.Namespace) -> list[dict[str, str]]:
    requested = str(args.operation_mode)
    if operation_mode_is_real(requested):
        raise AsyncSdkProbeError("operation_mode real is refused; this probe is pgmode simulation only")
    if not operation_mode_is_simulation(requested):
        raise AsyncSdkProbeError("--operation-mode must be simulation")
    modes: list[dict[str, str]] = [{"scope": "--operation-mode", "operation_mode": requested}]
    if args.config:
        modes.extend(operation_modes_from_config(args.config))
    for item in modes:
        mode = item.get("operation_mode", "")
        if operation_mode_is_real(mode):
            raise AsyncSdkProbeError(
                f"config operation_mode is real for {item.get('scope')}; refusing physical real operation_mode"
            )
        if mode and not operation_mode_is_simulation(mode):
            raise AsyncSdkProbeError(
                f"operation_mode must be simulation for {item.get('scope')}; got {mode!r}"
            )
    return modes


def write_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")


def ensure_pgmode(args: argparse.Namespace, artifact_dir: Path) -> dict[str, Any]:
    if args.set_pgmode_simulation and args.verify_pgmode_simulation:
        raise AsyncSdkProbeError("--set-pgmode-simulation and --verify-pgmode-simulation are mutually exclusive")
    if not args.set_pgmode_simulation and not args.verify_pgmode_simulation:
        raise AsyncSdkProbeError(
            "rbpodo async SDK probe requires --set-pgmode-simulation or --verify-pgmode-simulation"
        )
    try:
        from rainbow_pgmode import RainbowPgmodeError, run_pgmode
    except Exception as exc:
        raise AsyncSdkProbeError("scripts/rainbow_pgmode.py helper is unavailable") from exc
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
        raise AsyncSdkProbeError(f"controller not confirmed in pgmode simulation: {exc}") from exc
    write_json(artifact_dir / "pgmode_summary.json", summary)
    if summary.get("overall_result") != "ok":
        raise AsyncSdkProbeError("controller not confirmed in pgmode simulation; refusing Servo J probe")
    return summary


def preflight(args: argparse.Namespace, *, run_pgmode: bool = True) -> dict[str, Any]:
    finite_positive(args.duration_sec, "--duration-sec")
    finite_positive(args.rate_hz, "--rate-hz")
    finite_positive(args.read_timeout_sec, "--read-timeout-sec")
    finite_positive(args.command_timeout_sec, "--command-timeout-sec")
    finite_positive(args.servo_t2_sec, "--servo-t2-sec")
    finite_positive(args.servo_gain, "--servo-gain")
    finite_positive(args.pgmode_timeout_sec, "--pgmode-timeout-sec")
    finite_port(args.pgmode_command_port, "--pgmode-command-port")
    if not math.isfinite(args.servo_alpha) or not (0.0 < args.servo_alpha < 1.0):
        raise AsyncSdkProbeError("--servo-alpha must be finite and in (0, 1)")
    if (
        not math.isfinite(args.max_noop_target_delta_deg)
        or args.max_noop_target_delta_deg <= 0.0
        or args.max_noop_target_delta_deg > 0.2
    ):
        raise AsyncSdkProbeError("--max-noop-target-delta-deg must be finite, positive, and <= 0.2")
    if (
        not math.isfinite(args.max_q_actual_drift_deg)
        or args.max_q_actual_drift_deg <= 0.0
        or args.max_q_actual_drift_deg > 0.2
    ):
        raise AsyncSdkProbeError("--max-q-actual-drift-deg must be finite, positive, and <= 0.2")
    if args.late_ack_poll_sec < 0.0 or not math.isfinite(args.late_ack_poll_sec):
        raise AsyncSdkProbeError("--late-ack-poll-sec must be finite and non-negative")
    if not args.i_understand_this_connects_to_real_controller:
        raise AsyncSdkProbeError("missing --i-understand-this-connects-to-real-controller")
    if not args.allow_simulation_servo_j:
        raise AsyncSdkProbeError("missing --allow-simulation-servo-j")
    operation_modes = validate_operation_modes(args)
    for name in REQUIRED_ENV:
        if not env_enabled(name):
            raise AsyncSdkProbeError(f"rbpodo async SDK probe requires {name}=1")
    if env_enabled("RB_ALLOW_REAL_CARTESIAN"):
        raise AsyncSdkProbeError("RB_ALLOW_REAL_CARTESIAN must not be set for this Servo J SDK probe")
    if args.mode == "ack_off" and not env_enabled(ACK_DISABLED_ENV):
        raise AsyncSdkProbeError(f"ack_off mode requires {ACK_DISABLED_ENV}=1")

    artifact_dir = args.artifact_dir.resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    pgmode_summary = ensure_pgmode(args, artifact_dir) if run_pgmode else {"overall_result": "not_run_for_test"}
    return {
        "schema": SCHEMA,
        "target_ip": args.ip,
        "target_is_known_real_robot_ip": args.ip in REAL_ROBOT_IPS,
        "mode": args.mode,
        "duration_sec": args.duration_sec,
        "rate_hz": args.rate_hz,
        "operation_modes": operation_modes,
        "controller_simulation_only": True,
        "physical_motion_expected": False,
        "physical_real_motion_refused": True,
        "user_confirmation_flag": True,
        "allow_simulation_servo_j": True,
        "required_env": list(REQUIRED_ENV) + ([ACK_DISABLED_ENV] if args.mode == "ack_off" else []),
        "env": env_snapshot(),
        "pgmode_simulation_confirmed": pgmode_summary.get("overall_result") == "ok",
        "pgmode_summary": pgmode_summary,
        "safety_note": (
            "This probe sends only no-op Servo J targets after confirming Rainbow "
            "controller pgmode simulation. It never uses operation_mode real."
        ),
    }


def load_rbpodo_module() -> Any:
    try:
        rbpodo = importlib.import_module("rbpodo")
    except Exception as exc:
        raise AsyncSdkProbeError("Python rbpodo module is unavailable") from exc
    missing = [name for name in ("Cobot", "CobotData", "ResponseCollector") if not hasattr(rbpodo, name)]
    if missing:
        raise AsyncSdkProbeError("Python rbpodo module is missing: " + ", ".join(missing))
    return rbpodo


def safe_method_names(obj: Any) -> list[str]:
    names: list[str] = []
    for name in dir(obj):
        if name.startswith("_"):
            continue
        try:
            attr = getattr(obj, name)
        except Exception:
            continue
        if callable(attr):
            names.append(name)
    return sorted(set(names))


def method_signature(obj: Any, name: str) -> str | None:
    try:
        attr = getattr(obj, name)
    except Exception:
        return None
    try:
        return str(inspect.signature(attr))
    except Exception:
        return None


def rbpodo_sdk_capabilities(rbpodo: Any, control: Any | None = None, data_channel: Any | None = None) -> dict[str, Any]:
    module_version = None
    for attr in ("__version__", "VERSION", "version"):
        value = getattr(rbpodo, attr, None)
        if value is not None:
            module_version = str(value)
            break
    target = control if control is not None else getattr(rbpodo, "Cobot", None)
    data_target = data_channel if data_channel is not None else getattr(rbpodo, "CobotData", None)
    control_methods = safe_method_names(target) if target is not None else []
    data_methods = safe_method_names(data_target) if data_target is not None else []
    ack_related = sorted(name for name in control_methods if "ack" in name.lower() or "response" in name.lower())
    has_disable = "disable_waiting_ack" in control_methods or hasattr(target, "disable_waiting_ack")
    has_enable = "enable_waiting_ack" in control_methods or hasattr(target, "enable_waiting_ack")
    has_move = "move_servo_j" in control_methods or hasattr(target, "move_servo_j")
    has_request = "request_data" in data_methods or hasattr(data_target, "request_data")
    late_ack_candidates = [
        name for name in ack_related
        if name not in {"enable_waiting_ack", "disable_waiting_ack"}
    ]
    return {
        "schema": SDK_CAPABILITIES_SCHEMA,
        "timestamp_unix_sec": time.time(),
        "python_executable": sys.executable,
        "python_version": sys.version.split()[0],
        "module_available": True,
        "module_file": getattr(rbpodo, "__file__", None),
        "module_version": module_version,
        "has_cobot": hasattr(rbpodo, "Cobot"),
        "has_cobot_data": hasattr(rbpodo, "CobotData"),
        "has_response_collector": hasattr(rbpodo, "ResponseCollector"),
        "has_move_servo_j": has_move,
        "has_enable_waiting_ack": has_enable,
        "has_disable_waiting_ack": has_disable,
        "has_request_data": has_request,
        "control_methods_ack_related": ack_related,
        "data_methods_read_related": sorted(name for name in data_methods if "data" in name.lower() or "read" in name.lower()),
        "move_servo_j_signature": method_signature(target, "move_servo_j") if target is not None else None,
        "enable_waiting_ack_signature": method_signature(target, "enable_waiting_ack") if target is not None else None,
        "disable_waiting_ack_signature": method_signature(target, "disable_waiting_ack") if target is not None else None,
        "request_data_signature": method_signature(data_target, "request_data") if data_target is not None else None,
        "send_ack_separation_exposed": has_disable,
        "late_ack_observation_methods": late_ack_candidates,
        "late_ack_observation_exposed": bool(late_ack_candidates),
        "thread_safety_documented_by_probe": "unknown",
        "concurrent_read_send_probe_uses_separate_objects": True,
    }


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


def numeric(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def finite_joint_list(value: Any) -> list[float] | None:
    try:
        items = list(value)
    except TypeError:
        return None
    if len(items) != 6:
        return None
    out: list[float] = []
    for item in items:
        number = numeric(item)
        if number is None:
            return None
        out.append(number)
    return out


def max_abs_delta(a_values: list[float] | None, b_values: list[float] | None) -> float | None:
    if a_values is None or b_values is None:
        return None
    if len(a_values) != 6 or len(b_values) != 6:
        return None
    return max(abs(a - b) for a, b in zip(a_values, b_values))


def diagnostics_from_state(sdata: Any) -> dict[str, Any]:
    return {field: getattr(sdata, field, None) for field in DIAGNOSTIC_FIELDS}


def read_rbpodo_state(
    data_channel: Any,
    timeout_sec: float,
    *,
    index: int,
    mode: str,
    sample_role: str,
    q_baseline: list[float] | None = None,
    q_target: list[float] | None = None,
) -> dict[str, Any]:
    started_ns = now_ns()
    try:
        state = data_channel.request_data(timeout_sec)
        ended_ns = now_ns()
        if state is None:
            raise AsyncSdkProbeError("CobotData.request_data returned no state")
        sdata = getattr(state, "sdata", state)
        raw_mode = getattr(sdata, "real_vs_simulation_mode", None)
        q_actual = finite_joint_list(getattr(sdata, "jnt_ang", []))
        q_ref = finite_joint_list(getattr(sdata, "jnt_ref", []))
        valid = q_actual is not None and q_ref is not None
        return {
            "index": index,
            "mode": mode,
            "sample_role": sample_role,
            "host_time_ns": ended_ns,
            "read_duration_us": (ended_ns - started_ns) / 1000.0,
            "state_valid": valid,
            "controller_mode": controller_mode_name(raw_mode),
            "raw_controller_mode": raw_mode,
            "q_actual_deg": q_actual,
            "q_ref_deg": q_ref,
            "q_actual_drift_deg": max_abs_delta(q_baseline, q_actual),
            "q_ref_drift_deg": max_abs_delta(q_baseline, q_ref),
            "q_actual_target_error_deg": max_abs_delta(q_target, q_actual),
            "q_ref_target_error_deg": max_abs_delta(q_target, q_ref),
            "diagnostics": diagnostics_from_state(sdata),
            "error_name": "" if valid else "invalid_joint_state",
            "error_message": "" if valid else "CobotData.request_data did not return finite q_actual and q_ref",
        }
    except Exception as exc:
        ended_ns = now_ns()
        return {
            "index": index,
            "mode": mode,
            "sample_role": sample_role,
            "host_time_ns": ended_ns,
            "read_duration_us": (ended_ns - started_ns) / 1000.0,
            "state_valid": False,
            "controller_mode": "unknown",
            "raw_controller_mode": None,
            "q_actual_deg": None,
            "q_ref_deg": None,
            "q_actual_drift_deg": None,
            "q_ref_drift_deg": None,
            "q_actual_target_error_deg": None,
            "q_ref_target_error_deg": None,
            "diagnostics": {},
            "error_name": type(exc).__name__,
            "error_message": str(exc),
        }


def wait_initial_state(args: argparse.Namespace, data_channel: Any) -> dict[str, Any]:
    last: dict[str, Any] | None = None
    for attempt in range(10):
        sample = read_rbpodo_state(
            data_channel,
            args.read_timeout_sec,
            index=-1 - attempt,
            mode=args.mode,
            sample_role="initial",
        )
        last = sample
        if sample["state_valid"] and sample["controller_mode"] == "simulation":
            return sample
        time.sleep(min(args.read_timeout_sec, 0.02))
    detail = last or {}
    raise AsyncSdkProbeError(
        "unable to read finite initial rbpodo state in pgmode simulation: "
        f"{detail.get('error_name') or detail.get('controller_mode')}"
    )


def noop_target_from_initial(args: argparse.Namespace, initial: dict[str, Any]) -> tuple[list[float], str]:
    q_actual = initial.get("q_actual_deg")
    q_ref = initial.get("q_ref_deg")
    if q_actual is None or q_ref is None:
        raise AsyncSdkProbeError("initial state must contain finite q_actual and q_ref")
    delta = max_abs_delta(q_actual, q_ref)
    if delta is None or delta > args.max_noop_target_delta_deg:
        raise AsyncSdkProbeError(
            "refusing no-op Servo J target because q_ref and q_actual differ by "
            f"{delta} deg; limit is {args.max_noop_target_delta_deg} deg"
        )
    return list(q_ref), "initial_q_ref_deg"


def state_allows_noop_send(args: argparse.Namespace, sample: dict[str, Any], q_target: list[float]) -> tuple[bool, str]:
    if not sample.get("state_valid"):
        return False, sample.get("error_message") or "invalid state"
    if sample.get("controller_mode") != "simulation":
        return False, f"controller left pgmode simulation: real_vs_simulation_mode={sample.get('raw_controller_mode')}"
    actual_error = max_abs_delta(q_target, sample.get("q_actual_deg"))
    ref_error = max_abs_delta(q_target, sample.get("q_ref_deg"))
    if actual_error is None or ref_error is None:
        return False, "state did not contain finite q_actual/q_ref for no-op check"
    if actual_error > args.max_noop_target_delta_deg:
        return False, (
            f"target no longer equals q_actual within tolerance: {actual_error:.6f} deg > "
            f"{args.max_noop_target_delta_deg:.6f} deg"
        )
    if ref_error > args.max_noop_target_delta_deg:
        return False, (
            f"target no longer equals q_ref within tolerance: {ref_error:.6f} deg > "
            f"{args.max_noop_target_delta_deg:.6f} deg"
        )
    drift = sample.get("q_actual_drift_deg")
    if isinstance(drift, (int, float)) and drift > args.max_q_actual_drift_deg:
        return False, (
            f"q_actual drift {drift:.6f} deg exceeded limit "
            f"{args.max_q_actual_drift_deg:.6f} deg"
        )
    return True, ""


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


def compact_response_text(value: Any) -> str:
    text = str(value).strip()
    if text.startswith("<") and text.endswith(">"):
        return ""
    if text in {"True", "False", "None"}:
        return ""
    return text


def response_text(response_collector: Any, ret: Any | None = None) -> str:
    parts = [compact_response_text(response_collector)]
    if ret is not None:
        parts.append(compact_response_text(ret))
    return "\n".join(part for part in parts if part)


def response_error_names(text: str) -> list[str]:
    lowered = text.lower()
    names = [code for code in M_CODES if code.lower() in lowered]
    if ("not allowed" in lowered or "error" in lowered or "fail" in lowered) and not names:
        names.append("controller_error")
    return names


def joint_payload(q_target_deg: list[float]) -> Any:
    try:
        import numpy as np

        return np.asarray(q_target_deg, dtype=float)
    except Exception:
        return list(q_target_deg)


def configure_waiting_ack(control: Any, rbpodo: Any, *, disable_waiting_ack: bool) -> dict[str, Any]:
    operation = "disable_waiting_ack" if disable_waiting_ack else "enable_waiting_ack"
    method = getattr(control, operation, None)
    if not callable(method):
        raise AsyncSdkProbeError(f"Python rbpodo Cobot does not expose {operation}")
    responses = rbpodo.ResponseCollector()
    started_ns = now_ns()
    configured = method(responses)
    ended_ns = now_ns()
    text = response_text(responses)
    if not configured or collector_has_error(responses):
        suffix = f": {text}" if text else ""
        raise AsyncSdkProbeError(f"rbpodo {operation} was not accepted{suffix}")
    return {
        "operation": operation,
        "duration_us": (ended_ns - started_ns) / 1000.0,
        "response": text,
        "ack_policy": "disabled" if disable_waiting_ack else "wait",
    }


def send_servo_j_sample(
    *,
    args: argparse.Namespace,
    rbpodo: Any,
    control: Any,
    q_target: list[float],
    target_source: str,
    index: int,
    ack_disabled: bool,
) -> tuple[dict[str, Any], dict[str, Any]]:
    responses = rbpodo.ResponseCollector()
    ret: Any | None = None
    started_ns = now_ns()
    error_name = ""
    error_message = ""
    try:
        ret = control.move_servo_j(
            responses,
            joint_payload(q_target),
            1.0 / args.rate_hz,
            args.servo_t2_sec,
            args.servo_gain,
            args.servo_alpha,
            args.command_timeout_sec,
            True,
        )
        ended_ns = now_ns()
        text = response_text(responses, ret)
        names = response_error_names(text)
        success = return_success(ret) and not collector_has_error(responses) and not names
        if not success:
            if return_timeout(ret):
                error_name = "TransportTimeout"
            elif names:
                error_name = names[0]
            elif collector_has_error(responses):
                error_name = "controller_response_error"
            else:
                error_name = "ControllerRejected"
            error_message = text
    except Exception as exc:
        ended_ns = now_ns()
        text = response_text(responses, ret)
        names = response_error_names(text)
        success = False
        error_name = type(exc).__name__
        error_message = str(exc)
    sample = {
        "index": index,
        "mode": args.mode,
        "requested_rate_hz": args.rate_hz,
        "send_start_ns": started_ns,
        "send_end_ns": ended_ns,
        "send_duration_us": (ended_ns - started_ns) / 1000.0,
        "send_success": success,
        "ack_policy": "disabled" if ack_disabled else "wait",
        "ack_observed": bool(success and not ack_disabled),
        "socket_send_only": bool(success and ack_disabled),
        "target_source": target_source,
        "servo_t1_sec": 1.0 / args.rate_hz,
        "servo_t2_sec": args.servo_t2_sec,
        "servo_gain": args.servo_gain,
        "servo_alpha": args.servo_alpha,
        "command_timeout_sec": args.command_timeout_sec,
        "error_name": error_name,
        "error_message": error_message,
        "response": text,
        "response_error_names": names,
    }
    event = {
        "index": index,
        "mode": args.mode,
        "ack_policy": sample["ack_policy"],
        "send_success": success,
        "send_duration_us": sample["send_duration_us"],
        "ack_observed": sample["ack_observed"],
        "socket_send_only": sample["socket_send_only"],
        "response": text,
        "response_error_names": names,
        "error_name": error_name,
        "error_message": error_message,
        "_collector": responses,
        "_response_initial": text,
    }
    return sample, event


def run_ack_mode(args: argparse.Namespace, rbpodo: Any, *, ack_disabled: bool) -> ProbeRun:
    data_channel = rbpodo.CobotData(args.ip)
    control = rbpodo.Cobot(args.ip)
    sdk_capabilities = rbpodo_sdk_capabilities(rbpodo, control, data_channel)
    try:
        config_event = configure_waiting_ack(control, rbpodo, disable_waiting_ack=ack_disabled)
        if ack_disabled:
            sdk_capabilities["disable_waiting_ack_configured"] = True
        else:
            sdk_capabilities["enable_waiting_ack_configured"] = True
    except AsyncSdkProbeError as exc:
        if ack_disabled:
            sdk_capabilities["disable_waiting_ack_configured"] = False
        else:
            sdk_capabilities["enable_waiting_ack_configured"] = False
        sdk_capabilities["waiting_ack_configuration_error"] = str(exc)
        return ProbeRun(
            send_samples=[],
            state_samples=[],
            response_events=[{
                "event": "configure_waiting_ack_failed",
                "ack_policy": "disabled" if ack_disabled else "wait",
                "error_name": "waiting_ack_configuration_failed",
                "error_message": str(exc),
            }],
            sdk_capabilities=sdk_capabilities,
            run_notes=[str(exc)],
        )
    initial = wait_initial_state(args, data_channel)
    q_target, target_source = noop_target_from_initial(args, initial)
    initial = read_rbpodo_state(
        data_channel,
        args.read_timeout_sec,
        index=-1,
        mode=args.mode,
        sample_role="initial",
        q_baseline=q_target,
        q_target=q_target,
    )

    state_samples = [initial]
    send_samples: list[dict[str, Any]] = []
    response_events: list[dict[str, Any]] = [{"event": "configure_waiting_ack", **config_event}]
    period_sec = 1.0 / args.rate_hz
    expected_count = max(1, int(round(args.duration_sec * args.rate_hz)))
    next_time = time.monotonic()
    for index in range(expected_count):
        sample = read_rbpodo_state(
            data_channel,
            args.read_timeout_sec,
            index=index,
            mode=args.mode,
            sample_role="pre_send",
            q_baseline=q_target,
            q_target=q_target,
        )
        state_samples.append(sample)
        allowed, reason = state_allows_noop_send(args, sample, q_target)
        if not allowed:
            response_events.append({
                "index": index,
                "mode": args.mode,
                "event": "send_suppressed_by_probe_noop_guard",
                "error_name": "noop_guard_refused_send",
                "error_message": reason,
            })
            break
        send_sample, event = send_servo_j_sample(
            args=args,
            rbpodo=rbpodo,
            control=control,
            q_target=q_target,
            target_source=target_source,
            index=index,
            ack_disabled=ack_disabled,
        )
        send_samples.append(send_sample)
        response_events.append(event)
        if not send_sample["send_success"]:
            break
        next_time += period_sec
        sleep_sec = next_time - time.monotonic()
        if sleep_sec > 0.0:
            time.sleep(sleep_sec)

    late_ack: bool | str = "unknown"
    if ack_disabled and response_events:
        if args.late_ack_poll_sec > 0.0:
            time.sleep(args.late_ack_poll_sec)
        observed = False
        for event in response_events:
            collector = event.get("_collector")
            if collector is None:
                continue
            later = response_text(collector)
            if later and later != event.get("_response_initial"):
                event["late_response"] = later
                observed = True
        late_ack = True if observed else "unknown"

    for event in response_events:
        event.pop("_collector", None)
        event.pop("_response_initial", None)

    return ProbeRun(
        send_samples=send_samples,
        state_samples=state_samples,
        response_events=response_events,
        sdk_capabilities=sdk_capabilities,
        run_notes=[
            "ACK-off late ACK observation is reported as unknown unless a response collector changed after send return.",
        ] if ack_disabled else [],
        late_ack_observed_after_ack_off=late_ack,
    )


def run_concurrent_read_send(args: argparse.Namespace, rbpodo: Any) -> ProbeRun:
    data_channel = rbpodo.CobotData(args.ip)
    control = rbpodo.Cobot(args.ip)
    sdk_capabilities = rbpodo_sdk_capabilities(rbpodo, control, data_channel)
    config_event = configure_waiting_ack(control, rbpodo, disable_waiting_ack=False)
    initial = wait_initial_state(args, data_channel)
    q_target, target_source = noop_target_from_initial(args, initial)
    initial = read_rbpodo_state(
        data_channel,
        args.read_timeout_sec,
        index=-1,
        mode=args.mode,
        sample_role="initial",
        q_baseline=q_target,
        q_target=q_target,
    )

    send_samples: list[dict[str, Any]] = []
    state_samples: list[dict[str, Any]] = [initial]
    response_events: list[dict[str, Any]] = [{"event": "configure_waiting_ack", **config_event}]
    thread_errors: list[dict[str, Any]] = []
    lock = threading.Lock()
    stop_event = threading.Event()
    latest_state = {"sample": initial}
    period_sec = 1.0 / args.rate_hz
    deadline = time.monotonic() + args.duration_sec
    expected_count = max(1, int(round(args.duration_sec * args.rate_hz)))

    def record_thread_error(thread_name: str, exc: BaseException) -> None:
        with lock:
            thread_errors.append({
                "thread": thread_name,
                "error_name": type(exc).__name__,
                "error_message": str(exc),
            })
        stop_event.set()

    def read_worker() -> None:
        try:
            next_read = time.monotonic()
            index = 0
            while not stop_event.is_set() and time.monotonic() < deadline:
                sample = read_rbpodo_state(
                    data_channel,
                    args.read_timeout_sec,
                    index=index,
                    mode=args.mode,
                    sample_role="concurrent_read",
                    q_baseline=q_target,
                    q_target=q_target,
                )
                with lock:
                    state_samples.append(sample)
                    latest_state["sample"] = sample
                allowed, reason = state_allows_noop_send(args, sample, q_target)
                if not allowed:
                    with lock:
                        thread_errors.append({
                            "thread": "read",
                            "error_name": "noop_guard_refused_send",
                            "error_message": reason,
                        })
                    stop_event.set()
                    break
                index += 1
                next_read += period_sec
                sleep_sec = next_read - time.monotonic()
                if sleep_sec > 0.0:
                    time.sleep(sleep_sec)
        except BaseException as exc:
            record_thread_error("read", exc)

    def send_worker() -> None:
        try:
            next_send = time.monotonic()
            for index in range(expected_count):
                if stop_event.is_set() or time.monotonic() >= deadline:
                    break
                with lock:
                    sample = latest_state["sample"]
                allowed, reason = state_allows_noop_send(args, sample, q_target)
                if not allowed:
                    with lock:
                        thread_errors.append({
                            "thread": "send",
                            "error_name": "noop_guard_refused_send",
                            "error_message": reason,
                        })
                    stop_event.set()
                    break
                send_sample, event = send_servo_j_sample(
                    args=args,
                    rbpodo=rbpodo,
                    control=control,
                    q_target=q_target,
                    target_source=target_source,
                    index=index,
                    ack_disabled=False,
                )
                with lock:
                    send_samples.append(send_sample)
                    response_events.append(event)
                if not send_sample["send_success"]:
                    stop_event.set()
                    break
                next_send += period_sec
                sleep_sec = next_send - time.monotonic()
                if sleep_sec > 0.0:
                    time.sleep(sleep_sec)
        except BaseException as exc:
            record_thread_error("send", exc)

    read_thread = threading.Thread(target=read_worker, name="rbpodo-data-read")
    send_thread = threading.Thread(target=send_worker, name="rbpodo-servo-send")
    read_thread.start()
    send_thread.start()
    join_timeout = args.duration_sec + max(args.command_timeout_sec, args.read_timeout_sec) + 2.0
    read_thread.join(join_timeout)
    send_thread.join(join_timeout)
    deadlock = read_thread.is_alive() or send_thread.is_alive()
    if deadlock:
        stop_event.set()
        thread_errors.append({
            "thread": "join",
            "error_name": "thread_deadlock_or_hung_sdk_call",
            "error_message": "read or send worker did not exit before join timeout",
        })
    read_thread.join(0.2)
    send_thread.join(0.2)

    for event in response_events:
        event.pop("_collector", None)
        event.pop("_response_initial", None)
    for error in thread_errors:
        response_events.append({"event": "thread_error", **error})

    return ProbeRun(
        send_samples=send_samples,
        state_samples=state_samples,
        response_events=response_events,
        sdk_capabilities=sdk_capabilities,
        run_notes=[
            "concurrent_read_send uses separate Cobot and CobotData objects; shared Cobot thread safety remains unknown.",
        ],
        deadlock_observed=deadlock,
        sdk_thread_safety_observed="unknown",
    )


def percentile(values: list[float], pct: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    pos = (len(ordered) - 1) * pct
    lo = math.floor(pos)
    hi = math.ceil(pos)
    if lo == hi:
        return ordered[lo]
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (pos - lo)


def metric_block(values: list[float]) -> dict[str, float | None]:
    return {
        "p50": percentile(values, 0.50),
        "p95": percentile(values, 0.95),
        "p99": percentile(values, 0.99),
        "max": max(values) if values else None,
    }


def elapsed_sec_from_samples(samples: list[dict[str, Any]], start_key: str, end_key: str) -> float | None:
    starts = [numeric(sample.get(start_key)) for sample in samples]
    ends = [numeric(sample.get(end_key)) for sample in samples]
    starts = [value for value in starts if value is not None]
    ends = [value for value in ends if value is not None]
    if not starts or not ends:
        return None
    elapsed_ns = max(ends) - min(starts)
    return elapsed_ns / 1e9 if elapsed_ns > 0 else None


def build_metrics(args: argparse.Namespace, run: ProbeRun) -> dict[str, Any]:
    send_durations = [
        float(sample["send_duration_us"])
        for sample in run.send_samples
        if numeric(sample.get("send_duration_us")) is not None
    ]
    send_success_count = sum(1 for sample in run.send_samples if sample.get("send_success") is True)
    ack_observed_count = sum(1 for sample in run.send_samples if sample.get("ack_observed") is True)
    socket_send_only_count = sum(1 for sample in run.send_samples if sample.get("socket_send_only") is True)
    state_valid_samples = [sample for sample in run.state_samples if sample.get("state_valid") is True]
    q_ref_samples = [sample for sample in state_valid_samples if sample.get("q_ref_deg") is not None]
    q_ref_drifts = [
        float(sample["q_ref_drift_deg"])
        for sample in run.state_samples
        if numeric(sample.get("q_ref_drift_deg")) is not None
    ]
    q_actual_drifts = [
        float(sample["q_actual_drift_deg"])
        for sample in run.state_samples
        if numeric(sample.get("q_actual_drift_deg")) is not None
    ]
    state_elapsed = elapsed_sec_from_samples(run.state_samples, "host_time_ns", "host_time_ns")
    send_elapsed = elapsed_sec_from_samples(run.send_samples, "send_start_ns", "send_end_ns")
    concurrent_read_error_count = 0
    concurrent_send_error_count = 0
    if args.mode == "concurrent_read_send":
        concurrent_read_error_count = sum(1 for sample in run.state_samples if sample.get("state_valid") is not True)
        concurrent_send_error_count = sum(1 for sample in run.send_samples if sample.get("send_success") is not True)
        concurrent_read_error_count += sum(
            1 for event in run.response_events
            if event.get("event") == "thread_error" and event.get("thread") == "read"
        )
        concurrent_send_error_count += sum(
            1 for event in run.response_events
            if event.get("event") == "thread_error" and event.get("thread") == "send"
        )
    return {
        "send_count": len(run.send_samples),
        "send_success_count": send_success_count,
        "send_failure_count": len(run.send_samples) - send_success_count,
        "send_success_ratio": send_success_count / len(run.send_samples) if run.send_samples else 0.0,
        "send_duration": metric_block(send_durations),
        "send_duration_us": metric_block(send_durations),
        "ack_observed_count": ack_observed_count,
        "socket_send_only_count": socket_send_only_count,
        "state_sample_count": len(run.state_samples),
        "state_valid_sample_count": len(state_valid_samples),
        "q_ref_finite_sample_count": len(q_ref_samples),
        "q_ref_update_rate_hz": (len(q_ref_samples) / state_elapsed) if state_elapsed else None,
        "q_ref_drift_deg": max(q_ref_drifts) if q_ref_drifts else None,
        "q_actual_drift_deg": max(q_actual_drifts) if q_actual_drifts else None,
        "achieved_send_rate_hz": (len(run.send_samples) / send_elapsed) if send_elapsed else None,
        "concurrent_read_error_count": concurrent_read_error_count,
        "concurrent_send_error_count": concurrent_send_error_count,
        "sdk_thread_safety_observed": run.sdk_thread_safety_observed,
        "late_ack_observed_after_ack_off": run.late_ack_observed_after_ack_off,
    }


def classify_summary(summary: dict[str, Any]) -> tuple[str, list[str]]:
    mode = summary.get("mode")
    rate_hz = numeric(summary.get("rate_hz")) or 0.0
    period_us = 1_000_000.0 / rate_hz if rate_hz > 0 else None
    metrics = summary.get("metrics") if isinstance(summary.get("metrics"), dict) else {}
    caps = summary.get("sdk_capabilities") if isinstance(summary.get("sdk_capabilities"), dict) else {}
    reasons: list[str] = []

    for key in ("module_available", "has_cobot", "has_cobot_data", "has_response_collector", "has_move_servo_j"):
        if caps.get(key) is False:
            return "sdk_async_ack_not_supported", [f"SDK capability {key} is false"]
    if mode == "ack_off" and caps.get("has_disable_waiting_ack") is False:
        return "sdk_async_ack_not_supported", ["SDK does not expose disable_waiting_ack"]
    if mode == "ack_off" and caps.get("disable_waiting_ack_configured") is False:
        return "sdk_async_ack_not_supported", ["SDK did not accept disable_waiting_ack configuration"]
    if mode == "ack_on" and caps.get("enable_waiting_ack_configured") is False:
        return "sdk_async_ack_not_supported", ["SDK did not accept enable_waiting_ack configuration"]

    send_count = int(metrics.get("send_count") or 0)
    send_success_count = int(metrics.get("send_success_count") or 0)
    success_ratio = float(metrics.get("send_success_ratio") or 0.0)
    if send_count <= 0 or send_success_count <= 0:
        return "insufficient_evidence", ["no successful send samples were recorded"]

    send_duration = metrics.get("send_duration_us") if isinstance(metrics.get("send_duration_us"), dict) else {}
    if not send_duration and isinstance(metrics.get("send_duration"), dict):
        send_duration = metrics["send_duration"]
    p95 = numeric(send_duration.get("p95"))
    p99 = numeric(send_duration.get("p99"))
    max_value = numeric(send_duration.get("max"))
    ack_observed_count = int(metrics.get("ack_observed_count") or 0)
    socket_send_only_count = int(metrics.get("socket_send_only_count") or 0)

    if mode == "ack_on":
        if success_ratio < 0.98:
            return "insufficient_evidence", [f"send success ratio {success_ratio:.3f} is below 0.98"]
        if ack_observed_count < max(1, int(0.98 * send_count)):
            return "insufficient_evidence", ["ACK-on sends did not observe controller ACKs for enough samples"]
        if period_us is not None and p99 is not None and p99 <= period_us:
            return "ack_on_fast_enough", [f"p99 send duration {p99:.3f} us <= period {period_us:.3f} us"]
        if (
            period_us is not None
            and p95 is not None
            and p95 <= period_us
            and max_value is not None
            and max_value <= 5.0 * period_us
        ):
            return "ack_on_outlier_limited", [
                f"p95 send duration {p95:.3f} us <= period {period_us:.3f} us with bounded outliers"
            ]
        return "insufficient_evidence", ["ACK-on send durations were not fast enough for the requested rate"]

    if mode == "ack_off":
        q_ref_count = int(metrics.get("q_ref_finite_sample_count") or 0)
        state_count = int(metrics.get("state_sample_count") or 0)
        q_ref_drift = numeric(metrics.get("q_ref_drift_deg"))
        if success_ratio >= 0.98 and socket_send_only_count >= max(1, int(0.98 * send_count)):
            if state_count > 0 and q_ref_count / state_count >= 0.8 and (q_ref_drift is None or q_ref_drift <= 0.2):
                return "ack_off_state_supervision_viable", [
                    "ACK-off sends were socket-send-only and q_ref samples remained finite for supervision"
                ]
        reasons.append("ACK-off did not produce enough socket-send and q_ref supervision evidence")
        return "insufficient_evidence", reasons

    if mode == "concurrent_read_send":
        if summary.get("deadlock_observed"):
            return "insufficient_evidence", ["read/send worker did not exit before timeout"]
        if int(metrics.get("concurrent_read_error_count") or 0) != 0:
            return "insufficient_evidence", ["concurrent read errors were observed"]
        if int(metrics.get("concurrent_send_error_count") or 0) != 0:
            return "insufficient_evidence", ["concurrent send errors were observed"]
        if int(metrics.get("state_valid_sample_count") or 0) > 0 and success_ratio > 0.0:
            return "concurrent_read_send_viable", [
                "separate CobotData read and Cobot send objects ran concurrently without observed errors"
            ]
        return "insufficient_evidence", ["concurrent mode did not collect both read and send evidence"]

    return "insufficient_evidence", [f"unknown mode {mode!r}"]


def state_row(sample: dict[str, Any]) -> dict[str, Any]:
    row = {field: sample.get(field) for field in STATE_FIELDNAMES}
    q_actual = sample.get("q_actual_deg") or [None] * 6
    q_ref = sample.get("q_ref_deg") or [None] * 6
    for index in range(6):
        row[f"q_actual_{index}"] = q_actual[index] if index < len(q_actual) else None
        row[f"q_ref_{index}"] = q_ref[index] if index < len(q_ref) else None
    return row


def write_send_samples_csv(path: Path, samples: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=SEND_FIELDNAMES)
        writer.writeheader()
        for sample in samples:
            row = dict(sample)
            row["response_error_names"] = json.dumps(row.get("response_error_names") or [])
            writer.writerow({field: row.get(field) for field in SEND_FIELDNAMES})


def write_state_samples_csv(path: Path, samples: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=STATE_FIELDNAMES)
        writer.writeheader()
        for sample in samples:
            writer.writerow(state_row(sample))


def write_responses_jsonl(path: Path, events: list[dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        for event in events:
            handle.write(json.dumps(event, sort_keys=True, allow_nan=False) + "\n")


def write_report(path: Path, summary: dict[str, Any]) -> None:
    metrics = summary.get("metrics", {})
    send_duration = metrics.get("send_duration_us", {}) if isinstance(metrics, dict) else {}
    lines = [
        "# rbpodo Async SDK Probe Report",
        "",
        f"- result: `{summary.get('result_classification')}`",
        f"- mode: `{summary.get('mode')}`",
        f"- requested rate: `{summary.get('rate_hz')}` Hz",
        f"- send_success_count: `{metrics.get('send_success_count')}`",
        f"- ack_observed_count: `{metrics.get('ack_observed_count')}`",
        f"- socket_send_only_count: `{metrics.get('socket_send_only_count')}`",
        f"- send_duration_us p50/p95/p99/max: `{send_duration.get('p50')}` / `{send_duration.get('p95')}` / `{send_duration.get('p99')}` / `{send_duration.get('max')}`",
        f"- q_ref_update_rate_hz: `{metrics.get('q_ref_update_rate_hz')}`",
        f"- q_ref_drift_deg: `{metrics.get('q_ref_drift_deg')}`",
        f"- concurrent_read_error_count: `{metrics.get('concurrent_read_error_count')}`",
        f"- concurrent_send_error_count: `{metrics.get('concurrent_send_error_count')}`",
        f"- sdk_thread_safety_observed: `{metrics.get('sdk_thread_safety_observed')}`",
        f"- late_ack_observed_after_ack_off: `{metrics.get('late_ack_observed_after_ack_off')}`",
        "",
        "## Interpretation",
    ]
    for reason in summary.get("classification_reasons", []):
        lines.append(f"- {reason}")
    lines.extend([
        "",
        "## Caveats",
        "- This probe connects to real Rainbow controller boxes only after pgmode simulation confirmation.",
        "- It sends no-op Servo J targets only; target, q_ref, and q_actual must remain within tolerance.",
        "- It does not prove dual-arm rb_servo_server 500 Hz behavior.",
        "- It is intended to choose between `sdk_ack_worker` and `socket_send_supervised` follow-up designs.",
    ])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_artifacts(args: argparse.Namespace, safety: dict[str, Any], run: ProbeRun) -> dict[str, Any]:
    artifact_dir = args.artifact_dir.resolve()
    metrics = build_metrics(args, run)
    summary = {
        "schema": SCHEMA,
        "timestamp_unix_sec": time.time(),
        "mode": args.mode,
        "ip": args.ip,
        "duration_sec": args.duration_sec,
        "rate_hz": args.rate_hz,
        "artifact_dir": str(artifact_dir),
        "controller_simulation_only": True,
        "physical_motion_expected": False,
        "physical_real_motion_refused": True,
        "safety_preflight": safety,
        "sdk_capabilities": run.sdk_capabilities,
        "metrics": metrics,
        "deadlock_observed": run.deadlock_observed,
        "run_notes": run.run_notes,
        "artifacts": {
            "summary": str((artifact_dir / "summary.json").resolve()),
            "send_samples": str((artifact_dir / "send_samples.csv").resolve()),
            "state_samples": str((artifact_dir / "state_samples.csv").resolve()),
            "responses": str((artifact_dir / "responses.jsonl").resolve()),
            "sdk_capabilities": str((artifact_dir / "sdk_capabilities.json").resolve()),
            "report": str((artifact_dir / "report.md").resolve()),
        },
        "caveat": (
            "Controller pgmode simulation SDK capability evidence only; not "
            "dual-arm rb_servo_server 500 Hz acceptance or physical real-motion readiness."
        ),
    }
    classification, reasons = classify_summary(summary)
    summary["result_classification"] = classification
    summary["classification_reasons"] = reasons
    summary["result"] = "completed" if classification != "insufficient_evidence" else "inconclusive"

    write_send_samples_csv(artifact_dir / "send_samples.csv", run.send_samples)
    write_state_samples_csv(artifact_dir / "state_samples.csv", run.state_samples)
    write_responses_jsonl(artifact_dir / "responses.jsonl", run.response_events)
    write_json(artifact_dir / "sdk_capabilities.json", run.sdk_capabilities)
    write_json(artifact_dir / "summary.json", summary)
    write_report(artifact_dir / "report.md", summary)
    return summary


def run_probe(args: argparse.Namespace, safety: dict[str, Any]) -> ProbeRun:
    rbpodo = load_rbpodo_module()
    if args.mode == "ack_on":
        return run_ack_mode(args, rbpodo, ack_disabled=False)
    if args.mode == "ack_off":
        return run_ack_mode(args, rbpodo, ack_disabled=True)
    return run_concurrent_read_send(args, rbpodo)


def main() -> int:
    args = parse_args()
    try:
        safety = preflight(args)
        print(json.dumps({"event": "safety_preflight", **safety}, indent=2, sort_keys=True))
        run = run_probe(args, safety)
        summary = write_artifacts(args, safety, run)
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 0 if summary.get("result_classification") != "insufficient_evidence" else 1
    except AsyncSdkProbeError as exc:
        print(f"rbpodo_async_sdk_probe: FAIL: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
