#!/usr/bin/env python3
"""rb_servo_server-level rbpodo 500 Hz controller-simulation no-op acceptance."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shutil
import signal
import socket
import subprocess
import sys
import time
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from cartesian_acceptance import StateCapture
from rbpodo_servo_acceptance import (
    REAL_ROBOT_IPS,
    ParsedConfig,
    as_bool,
    as_float,
    env_enabled,
    env_snapshot,
    load_config,
    parse_udp_endpoint,
    scalar_value,
    state_stream_timeout_message,
)


SCHEMA = "robotics_lab.rbpodo_500hz_acceptance.v1"
MODE = "servo_j_noop_500hz"
COMMAND_RATE_HZ = 500.0
COMMAND_PERIOD_SEC = 1.0 / COMMAND_RATE_HZ
REQUIRED_ENV = (
    "RB_ALLOW_REAL_ROBOT",
    "RB_ALLOW_REAL_MOTION",
    "RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION",
    "RB_RBPODO_PGMODE_SIMULATION_CONFIRMED",
)


class Acceptance500HzError(RuntimeError):
    pass


@dataclass
class CommandRunMetrics:
    command_count: int
    expected_command_count: int
    start_host_time_ns: int
    end_host_time_ns: int
    elapsed_sec: float
    sender_deadline_missed_count: int
    max_sender_lateness_us: float
    hold_sent: bool


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run a rb_servo_server full-path 500 Hz rbpodo Servo J no-op "
            "acceptance against Rainbow controllers in pgmode simulation. "
            "Physical robot motion is refused."
        )
    )
    parser.add_argument("--root", type=Path, default=Path("."))
    parser.add_argument("--server", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--arm", choices=("left", "right"), required=True)
    parser.add_argument("--mode", choices=(MODE,), default=MODE)
    parser.add_argument("--duration-sec", type=float, default=10.0)
    parser.add_argument("--artifact-dir", type=Path, required=True)
    parser.add_argument("--startup-timeout-sec", type=float, default=12.0)
    parser.add_argument("--settle-sec", type=float, default=0.25)
    parser.add_argument("--max-state-age-us", type=float, default=250_000.0)
    parser.add_argument("--max-physical-motion-deg", type=float, default=0.05)
    parser.add_argument("--max-reference-drift-deg", type=float, default=0.05)
    parser.add_argument("--min-send-count-ratio", type=float, default=0.98)
    parser.add_argument("--min-controller-acceptance-ratio", type=float, default=0.98)
    parser.add_argument("--max-send-duration-p99-us", type=float, default=1000.0)
    parser.add_argument("--max-servo-jitter-p99-ms", type=float, default=2.5)
    parser.add_argument("--max-deadline-miss-count", type=int, default=0)
    parser.add_argument("--max-worker-drop-count", type=int, default=0)
    parser.add_argument("--set-pgmode-simulation", action="store_true")
    parser.add_argument("--verify-pgmode-simulation", action="store_true")
    parser.add_argument("--pgmode-timeout-sec", type=float, default=1.0)
    parser.add_argument("--pgmode-command-port", type=int, default=5000)
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--skip-plots", action="store_true")
    parser.add_argument(
        "--i-understand-this-connects-to-real-controller",
        action="store_true",
        help="Required before connecting to real Rainbow controller boxes.",
    )
    parser.add_argument(
        "--i-confirm-controller-is-in-pgmode-simulation",
        action="store_true",
        help="Required acknowledgement before controller-simulation acceptance.",
    )
    return parser.parse_args()


def finite_number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        number = float(value)
        return number if math.isfinite(number) else None
    if isinstance(value, str):
        try:
            number = float(value)
        except ValueError:
            return None
        return number if math.isfinite(number) else None
    return None


def bool_value(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return False


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


def fallback_yaml_sections(path: Path) -> dict[str, dict[str, Any]]:
    sections: dict[str, dict[str, Any]] = {}
    current: str | None = None
    pending_list_key: str | None = None
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip(" "))
        text = line.strip()
        if indent == 0 and text.endswith(":"):
            current = text[:-1]
            sections[current] = {}
            pending_list_key = None
            continue
        if current is None:
            continue
        if indent == 2 and ":" in text:
            key, value = text.split(":", 1)
            key = key.strip()
            if value.strip():
                sections[current][key] = scalar_value(value)
                pending_list_key = None
            else:
                sections[current][key] = []
                pending_list_key = key
            continue
        if indent == 4 and pending_list_key is not None and text.startswith("- "):
            items = sections[current].setdefault(pending_list_key, [])
            if isinstance(items, list):
                items.append(scalar_value(text[2:].strip()))
    return sections


def yaml_sections(path: Path) -> dict[str, dict[str, Any]]:
    try:
        import yaml  # type: ignore
    except Exception:
        return fallback_yaml_sections(path)
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise Acceptance500HzError(f"failed to parse YAML config {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise Acceptance500HzError("server config must be a YAML object")
    return {str(key): value for key, value in data.items() if isinstance(value, dict)}


def selected_arm(config: ParsedConfig, arm: str) -> Any:
    return config.left if arm == "left" else config.right


def all_arms(config: ParsedConfig) -> tuple[tuple[str, Any], tuple[str, Any]]:
    return (("left", config.left), ("right", config.right))


def endpoint_from_config(config: ParsedConfig, key: str, default: str) -> tuple[str, int, str]:
    value = config.network.get(key)
    if value is None and key == "state_pub_endpoint":
        endpoints = config.network.get("state_pub_endpoints")
        if endpoints is not None:
            if not isinstance(endpoints, list) or not endpoints:
                raise Acceptance500HzError("network.state_pub_endpoints must be a non-empty list")
            value = endpoints[0]
    endpoint = str(value if value is not None else default)
    host, port = parse_udp_endpoint(endpoint)
    bind_host = "127.0.0.1" if host == "localhost" else host
    return bind_host, port, endpoint


def finite_joint_values(value: Any) -> list[float] | None:
    if not isinstance(value, list) or len(value) != 6:
        return None
    out: list[float] = []
    for item in value:
        number = finite_number(item)
        if number is None:
            return None
        out.append(number)
    return out


def arm_state(snapshot: dict[str, Any], arm: str) -> dict[str, Any] | None:
    value = snapshot.get(arm)
    return value if isinstance(value, dict) else None


def noop_target_from_state(snapshot: dict[str, Any], arm: str) -> tuple[list[float], str]:
    state = arm_state(snapshot, arm)
    if state is None:
        raise Acceptance500HzError(f"{arm} state is missing")
    for key in ("q_ref_deg", "q_target_deg"):
        joints = finite_joint_values(state.get(key))
        if joints is not None:
            return joints, key
    raise Acceptance500HzError(f"{arm} state did not publish finite q_ref_deg or q_target_deg")


def q_actual_from_state(snapshot: dict[str, Any], arm: str) -> list[float]:
    state = arm_state(snapshot, arm)
    joints = finite_joint_values(state.get("q_actual_deg") if state else None)
    if joints is None:
        raise Acceptance500HzError(f"{arm} state did not publish finite q_actual_deg")
    return joints


def validate_numeric_args(args: argparse.Namespace) -> None:
    positive = (
        ("--duration-sec", args.duration_sec),
        ("--startup-timeout-sec", args.startup_timeout_sec),
        ("--pgmode-timeout-sec", args.pgmode_timeout_sec),
        ("--max-state-age-us", args.max_state_age_us),
        ("--max-physical-motion-deg", args.max_physical_motion_deg),
        ("--max-reference-drift-deg", args.max_reference_drift_deg),
        ("--max-send-duration-p99-us", args.max_send_duration_p99_us),
        ("--max-servo-jitter-p99-ms", args.max_servo_jitter_p99_ms),
    )
    for name, value in positive:
        if not math.isfinite(value) or value <= 0.0:
            raise Acceptance500HzError(f"{name} must be finite and positive")
    for name, value in (
        ("--min-send-count-ratio", args.min_send_count_ratio),
        ("--min-controller-acceptance-ratio", args.min_controller_acceptance_ratio),
    ):
        if not math.isfinite(value) or value <= 0.0 or value > 1.0:
            raise Acceptance500HzError(f"{name} must be finite and in (0, 1]")
    if args.max_deadline_miss_count < 0:
        raise Acceptance500HzError("--max-deadline-miss-count must be >= 0")
    if args.max_worker_drop_count < 0:
        raise Acceptance500HzError("--max-worker-drop-count must be >= 0")
    if args.pgmode_command_port < 1 or args.pgmode_command_port > 65535:
        raise Acceptance500HzError("--pgmode-command-port must be in [1, 65535]")
    if args.settle_sec < 0.0 or not math.isfinite(args.settle_sec):
        raise Acceptance500HzError("--settle-sec must be finite and non-negative")


def ensure_pgmode(args: argparse.Namespace, config: ParsedConfig) -> dict[str, Any]:
    if args.set_pgmode_simulation and args.verify_pgmode_simulation:
        raise Acceptance500HzError("--set-pgmode-simulation and --verify-pgmode-simulation are mutually exclusive")
    if not args.set_pgmode_simulation and not args.verify_pgmode_simulation:
        raise Acceptance500HzError(
            "500 Hz controller-simulation acceptance requires --set-pgmode-simulation "
            "or --verify-pgmode-simulation"
        )
    if not args.i_confirm_controller_is_in_pgmode_simulation:
        raise Acceptance500HzError("missing --i-confirm-controller-is-in-pgmode-simulation")
    try:
        from rainbow_pgmode import RainbowPgmodeError, ensure_controller_simulation_mode
    except Exception as exc:
        raise Acceptance500HzError("scripts/rainbow_pgmode.py helper is unavailable") from exc
    try:
        return ensure_controller_simulation_mode(
            [config.left.ip, config.right.ip],
            args.pgmode_timeout_sec,
            port=args.pgmode_command_port,
            confirmation=args.i_understand_this_connects_to_real_controller,
            set_simulation=args.set_pgmode_simulation,
            verify_only=args.verify_pgmode_simulation,
        )
    except RainbowPgmodeError as exc:
        raise Acceptance500HzError(
            f"controller not confirmed in pgmode simulation; refusing 500 Hz no-op acceptance: {exc}"
        ) from exc


def validate_config_and_env(
    args: argparse.Namespace,
    config: ParsedConfig,
    sections: dict[str, dict[str, Any]],
    *,
    run_pgmode: bool = True,
) -> dict[str, Any]:
    validate_numeric_args(args)
    if args.mode != MODE:
        raise Acceptance500HzError(f"unsupported mode {args.mode}")
    for label, cfg in all_arms(config):
        if cfg.backend_type != "rbpodo":
            raise Acceptance500HzError(f"{label}_robot.backend_type must be rbpodo")
        if cfg.run_mode != "real":
            raise Acceptance500HzError(f"{label}_robot.run_mode must be real for rbpodo controller-simulation")
        if cfg.operation_mode not in {"simulation", "sim"}:
            actual = cfg.operation_mode or "<missing>"
            raise Acceptance500HzError(
                f"config operation_mode is {actual} for {label}_robot; refusing physical real operation_mode"
            )
        if not cfg.ip:
            raise Acceptance500HzError(f"{label}_robot.ip is required")
        if cfg.disable_waiting_ack:
            raise Acceptance500HzError("500 Hz no-op acceptance requires ACK-on rbpodo settings")

    known_ips = {config.left.ip, config.right.ip} & REAL_ROBOT_IPS
    if known_ips and not args.i_understand_this_connects_to_real_controller:
        raise Acceptance500HzError("refusing known real controller IP without explicit confirmation flag")
    if not args.i_understand_this_connects_to_real_controller:
        raise Acceptance500HzError("missing --i-understand-this-connects-to-real-controller")
    for name in REQUIRED_ENV:
        if not env_enabled(name):
            raise Acceptance500HzError(f"500 Hz controller-simulation no-op requires {name}=1")
    if env_enabled("RB_ALLOW_REAL_CARTESIAN"):
        raise Acceptance500HzError("RB_ALLOW_REAL_CARTESIAN must not be set for 500 Hz Servo J no-op acceptance")

    send_servo_commands = as_bool(config.servo.get("send_servo_commands"), False)
    if not send_servo_commands:
        raise Acceptance500HzError("500 Hz no-op acceptance requires servo.send_servo_commands=true")
    if not as_bool(config.servo.get("allow_controller_simulation_motion"), False):
        raise Acceptance500HzError(
            "500 Hz no-op acceptance requires servo.allow_controller_simulation_motion=true"
        )
    if as_bool(config.servo.get("allow_controller_simulation_diagnostics_suspect"), False) and not env_enabled(
        "RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM"
    ):
        raise Acceptance500HzError(
            "diagnostics-suspect controller-simulation override requires "
            "RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1"
        )

    servo_rate_hz = as_float(config.servo.get("rate_hz"))
    if servo_rate_hz is None or abs(servo_rate_hz - COMMAND_RATE_HZ) > 1e-9:
        raise Acceptance500HzError(f"servo.rate_hz must be 500 for {MODE}")
    tolerance_ratio = as_float(config.servo.get("servo_t1_rate_match_tolerance_ratio"), 0.2) or 0.2
    expected_t1 = COMMAND_PERIOD_SEC
    for label, cfg in all_arms(config):
        if cfg.servo_t1_sec is None:
            raise Acceptance500HzError(f"{label}_robot.servo_t1_sec is required")
        if abs(float(cfg.servo_t1_sec) - expected_t1) > tolerance_ratio * expected_t1:
            raise Acceptance500HzError(
                f"{label}_robot.servo_t1_sec={cfg.servo_t1_sec} does not match 500 Hz period {expected_t1:.6f}"
            )

    command_host, command_port, command_endpoint = endpoint_from_config(config, "command_bind", "udp://127.0.0.1:50010")
    state_host, state_port, state_endpoint = endpoint_from_config(config, "state_pub_endpoint", "udp://127.0.0.1:50110")
    pgmode_summary = ensure_pgmode(args, config) if run_pgmode else {"overall_result": "not_run_for_test"}
    if run_pgmode and pgmode_summary.get("overall_result") != "ok":
        raise Acceptance500HzError("pgmode simulation preflight did not return overall_result=ok")

    selected = selected_arm(config, args.arm)
    return {
        "passed": True,
        "schema": SCHEMA,
        "mode": MODE,
        "backend": "rbpodo",
        "controller_simulation_only": True,
        "physical_motion_expected": False,
        "physical_real_motion_refused": True,
        "arm": args.arm,
        "selected_ip": selected.ip,
        "configured_ips": [config.left.ip, config.right.ip],
        "known_real_ips": sorted(known_ips),
        "config": str(config.path),
        "servo_rate_hz": servo_rate_hz,
        "command_rate_hz": COMMAND_RATE_HZ,
        "servo_t1_sec": selected.servo_t1_sec,
        "disable_waiting_ack": False,
        "ack_semantics": "controller_ack_observed",
        "send_servo_commands": send_servo_commands,
        "allow_controller_simulation_motion": True,
        "allow_controller_simulation_diagnostics_suspect": as_bool(
            config.servo.get("allow_controller_simulation_diagnostics_suspect"), False
        ),
        "operation_modes": {"left": config.left.operation_mode, "right": config.right.operation_mode},
        "command_endpoint": command_endpoint,
        "state_endpoint": state_endpoint,
        "command_host": command_host,
        "command_port": command_port,
        "state_host": state_host,
        "state_port": state_port,
        "required_env": list(REQUIRED_ENV),
        "env": env_snapshot(),
        "confirmation_flag": args.i_understand_this_connects_to_real_controller,
        "pgmode_confirmation_flag": args.i_confirm_controller_is_in_pgmode_simulation,
        "pgmode_simulation_confirmed": pgmode_summary.get("overall_result") == "ok",
        "pgmode_summary": pgmode_summary,
        "server_env_overrides": {"RB_RBPODO_PGMODE_SIMULATION_CONFIRMED": "1"},
        "cartesian_env_required": False,
        "cartesian_env_forbidden": "RB_ALLOW_REAL_CARTESIAN",
        "network_state_pub_rate_hz": as_float(config.network.get("state_pub_rate_hz")),
        "logging_directory": sections.get("logging", {}).get("directory"),
    }


def preflight(args: argparse.Namespace, *, run_pgmode: bool = True) -> tuple[ParsedConfig, dict[str, Any]]:
    config_path = (args.root / args.config).resolve() if not args.config.is_absolute() else args.config.resolve()
    if not config_path.is_file():
        raise Acceptance500HzError(f"config not found: {config_path}")
    config = load_config(config_path)
    sections = yaml_sections(config_path)
    if "servo" in sections:
        config.servo = sections["servo"]
    if "network" in sections:
        config.network = sections["network"]
    if "logging" in sections:
        config.logging = sections["logging"]
    return config, validate_config_and_env(args, config, sections, run_pgmode=run_pgmode)


def now_ns() -> int:
    return time.monotonic_ns()


def send_udp(host: str, port: int, packet: dict[str, Any], handle: Any) -> None:
    packet.setdefault("schema_version", 1)
    packet.setdefault("host_time_ns", now_ns())
    packet.setdefault("timeout_sec", 0.2)
    packet.setdefault("coupled_timeout", True)
    payload = json.dumps(packet, separators=(",", ":"), allow_nan=False).encode("utf-8")
    handle.write(json.dumps(packet, sort_keys=True, separators=(",", ":")) + "\n")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.sendto(payload, (host, port))


def send_udp_with_socket(sock: socket.socket, host: str, port: int, packet: dict[str, Any], handle: Any) -> None:
    packet.setdefault("schema_version", 1)
    packet.setdefault("host_time_ns", now_ns())
    packet.setdefault("timeout_sec", 0.2)
    packet.setdefault("coupled_timeout", True)
    payload = json.dumps(packet, separators=(",", ":"), allow_nan=False).encode("utf-8")
    handle.write(json.dumps(packet, sort_keys=True, separators=(",", ":")) + "\n")
    sock.sendto(payload, (host, port))


def lifecycle_packet(mode: str, seq: int, session_id: str) -> dict[str, Any]:
    stamp = now_ns()
    return {
        "schema_version": 1,
        "seq": seq,
        "mode": mode,
        "host_time_ns": stamp,
        "timeout_sec": 0.2,
        "coupled_timeout": True,
        "source_id": "rbpodo_500hz_acceptance",
        "session_id": session_id,
        "left": {},
        "right": {},
    }


def hold_packet(seq: int, session_id: str) -> dict[str, Any]:
    stamp = now_ns()
    return {
        "schema_version": 1,
        "seq": seq,
        "mode": "Hold",
        "host_time_ns": stamp,
        "timeout_sec": 0.2,
        "coupled_timeout": True,
        "source_id": "rbpodo_500hz_acceptance",
        "session_id": session_id,
        "post_run_hold": True,
        "left": {"mode": "Hold"},
        "right": {"mode": "Hold"},
    }


def joint_target_packet(seq: int, session_id: str, arm: str, q_target_deg: list[float]) -> dict[str, Any]:
    payload = {"q_target_deg": q_target_deg}
    stamp = now_ns()
    return {
        "schema_version": 1,
        "seq": seq,
        "mode": "JointTarget",
        "host_time_ns": stamp,
        "timeout_sec": 0.2,
        "coupled_timeout": True,
        "source_id": "rbpodo_500hz_acceptance",
        "session_id": session_id,
        "left": payload if arm == "left" else {"mode": "Hold"},
        "right": payload if arm == "right" else {"mode": "Hold"},
    }


def start_server(args: argparse.Namespace, log_path: Path, preflight_result: dict[str, Any]) -> subprocess.Popen[str]:
    server = (args.root / args.server).resolve() if not args.server.is_absolute() else args.server.resolve()
    if not server.is_file():
        raise Acceptance500HzError(f"server binary not found: {server}")
    config_path = (args.root / args.config).resolve() if not args.config.is_absolute() else args.config.resolve()
    command = [str(server), "--config", str(config_path)]
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log = log_path.open("w", encoding="utf-8")
    env = os.environ.copy()
    env.update(preflight_result.get("server_env_overrides") or {})
    return subprocess.Popen(
        command,
        cwd=str(args.root.resolve()),
        stdout=log,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
        start_new_session=True,
    )


def stop_server(proc: subprocess.Popen[str] | None) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=5)
    except Exception:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except Exception:
            proc.kill()
            proc.wait(timeout=3)


def state_excerpt(snapshot: dict[str, Any], arm: str) -> dict[str, Any]:
    top_keys = (
        "schema_version",
        "host_time_ns",
        "motion_state",
        "safety_verdict",
        "fault_latched",
        "latched_fault_reason",
        "fault_reason",
        "send_policy",
        "send_suppressed",
    )
    arm_keys = (
        "has_valid_joint_state",
        "q_actual_deg",
        "q_ref_deg",
        "q_target_deg",
        "q_sent_deg",
        "state_age_us",
        "send_duration_us",
        "send_within_period",
        "send_period_overrun",
        "send_command_deadline_missed",
        "last_send",
        "worker",
    )
    out = {key: snapshot.get(key) for key in top_keys if key in snapshot}
    state = arm_state(snapshot, arm)
    if state is not None:
        out[arm] = {key: state.get(key) for key in arm_keys if key in state}
    return out


def start_state_ready(snapshot: dict[str, Any], arm: str) -> bool:
    if snapshot.get("fault_latched") is True:
        return False
    state = arm_state(snapshot, arm)
    if state is None or state.get("has_valid_joint_state") is not True:
        return False
    return finite_joint_values(state.get("q_actual_deg")) is not None and any(
        finite_joint_values(state.get(key)) is not None for key in ("q_ref_deg", "q_target_deg")
    )


def wait_for_start_state(
    capture: StateCapture,
    args: argparse.Namespace,
    proc: subprocess.Popen[str] | None,
    log_path: Path,
    state_endpoint: str,
) -> dict[str, Any]:
    deadline = time.monotonic() + args.startup_timeout_sec
    scanned = 0
    latest: dict[str, Any] | None = None
    while time.monotonic() < deadline:
        snapshots = list(capture.snapshots)
        for snapshot in snapshots[scanned:]:
            latest = snapshot
            if start_state_ready(snapshot, args.arm):
                return snapshot
            if snapshot.get("fault_latched") is True:
                raise Acceptance500HzError(
                    "rb_servo_server published a fault-latched startup state:\n"
                    + json.dumps(state_excerpt(snapshot, args.arm), indent=2, sort_keys=True)
                )
        scanned = len(snapshots)
        if proc is not None and proc.poll() is not None:
            break
        time.sleep(0.02)
    if latest is not None:
        raise Acceptance500HzError(
            "received rb_servo_server state packets, but no valid rbpodo no-op start state was observed:\n"
            + json.dumps(state_excerpt(latest, args.arm), indent=2, sort_keys=True)
        )
    raise Acceptance500HzError(state_stream_timeout_message(proc, log_path, state_endpoint))


def wait_for_armed(
    capture: StateCapture,
    args: argparse.Namespace,
    after_ns: int,
) -> dict[str, Any]:
    deadline = time.monotonic() + args.startup_timeout_sec
    scanned = 0
    latest: dict[str, Any] | None = None
    while time.monotonic() < deadline:
        snapshots = list(capture.snapshots)
        for snapshot in snapshots[scanned:]:
            latest = snapshot
            if int(snapshot.get("host_time_ns", -1)) < after_ns:
                continue
            if snapshot.get("fault_latched") is True:
                raise Acceptance500HzError(
                    "fault latched while arming rb_servo_server:\n"
                    + json.dumps(state_excerpt(snapshot, args.arm), indent=2, sort_keys=True)
                )
            if snapshot.get("motion_state") in {"ArmedHold", "Running"} and start_state_ready(snapshot, args.arm):
                return snapshot
        scanned = len(snapshots)
        time.sleep(0.02)
    detail = json.dumps(state_excerpt(latest or {}, args.arm), indent=2, sort_keys=True)
    raise Acceptance500HzError(f"timed out waiting for rb_servo_server ArmedHold state:\n{detail}")


def send_noop_stream(
    args: argparse.Namespace,
    preflight_result: dict[str, Any],
    q_noop: list[float],
    artifact_path: Path,
    session_id: str,
) -> CommandRunMetrics:
    host = str(preflight_result["command_host"])
    port = int(preflight_result["command_port"])
    expected_count = max(1, int(round(float(args.duration_sec) * COMMAND_RATE_HZ)))
    seq = 2
    start_host_ns = 0
    end_host_ns = 0
    deadline_misses = 0
    max_lateness_us = 0.0
    artifact_path.parent.mkdir(parents=True, exist_ok=True)
    start_wall = time.monotonic()
    next_send = start_wall
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock, artifact_path.open("a", encoding="utf-8") as handle:
        start_wall = time.monotonic()
        next_send = start_wall
        for _index in range(expected_count):
            now = time.monotonic()
            lateness = now - next_send
            if lateness > COMMAND_PERIOD_SEC:
                deadline_misses += 1
                max_lateness_us = max(max_lateness_us, lateness * 1e6)
            packet = joint_target_packet(seq, session_id, args.arm, q_noop)
            if start_host_ns == 0:
                start_host_ns = int(packet["host_time_ns"])
            end_host_ns = int(packet["host_time_ns"])
            send_udp_with_socket(sock, host, port, packet, handle)
            seq += 1
            next_send += COMMAND_PERIOD_SEC
            sleep_sec = next_send - time.monotonic()
            if sleep_sec > 0.0:
                time.sleep(sleep_sec)
        hold = hold_packet(seq, session_id)
        send_udp_with_socket(sock, host, port, hold, handle)
    elapsed = max(time.monotonic() - start_wall, 1e-9)
    return CommandRunMetrics(
        command_count=expected_count,
        expected_command_count=expected_count,
        start_host_time_ns=start_host_ns,
        end_host_time_ns=end_host_ns,
        elapsed_sec=elapsed,
        sender_deadline_missed_count=deadline_misses,
        max_sender_lateness_us=max_lateness_us,
        hold_sent=True,
    )


def copy_servo_log(root: Path, config: ParsedConfig, artifact_dir: Path, started_at: float) -> dict[str, Any] | None:
    candidates: list[Path] = []
    log_dir_value = config.logging.get("directory")
    if isinstance(log_dir_value, str) and log_dir_value:
        log_dir = Path(log_dir_value)
        candidates.append((root / log_dir).resolve() if not log_dir.is_absolute() else log_dir)
    candidates.append((root / "logs").resolve())
    csv_candidates: list[Path] = []
    for directory in dict.fromkeys(candidates):
        if directory.is_dir():
            csv_candidates.extend(directory.glob("*.csv"))
    csv_candidates = [
        path for path in csv_candidates if path.is_file() and path.stat().st_mtime >= started_at - 1.0
    ]
    if not csv_candidates:
        return None
    source = max(csv_candidates, key=lambda path: path.stat().st_mtime)
    target = artifact_dir / "servo_log.csv"
    shutil.copy2(source, target)
    rows = 0
    with target.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for _row in reader:
            rows += 1
    return {"path": str(target.resolve()), "rows": rows, "source": str(source.resolve())}


def parse_servo_log_metrics(
    path: Path,
    arm: str,
    start_ns: int,
    end_ns: int,
) -> dict[str, Any]:
    prefix = f"{arm}_"
    mode_key = f"{prefix}mode"
    send_ok_key = f"{prefix}send_ok"
    acceptance_key = f"{prefix}controller_acceptance_observed"
    ack_key = f"{prefix}ack_observed"
    duration_key = f"{prefix}send_duration_us"
    deadline_key = f"{prefix}send_command_deadline_missed"
    period_overrun_key = f"{prefix}send_period_overrun"
    within_period_key = f"{prefix}send_within_period"
    rows = 0
    selected_rows = 0
    send_success_count = 0
    send_failure_count = 0
    acceptance_count = 0
    ack_count = 0
    deadline_missed_count = 0
    period_overrun_count = 0
    within_period_count = 0
    send_durations: list[float] = []
    jitter_values: list[float] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rows += 1
            loop_ns = int(finite_number(row.get("loop_start_time_ns")) or -1)
            if start_ns > 0 and loop_ns < start_ns:
                continue
            if end_ns > 0 and loop_ns > end_ns + int(4 * COMMAND_PERIOD_SEC * 1e9):
                continue
            if row.get(mode_key) != "JointTarget":
                continue
            selected_rows += 1
            if bool_value(row.get(send_ok_key)):
                send_success_count += 1
            else:
                send_failure_count += 1
            if bool_value(row.get(acceptance_key)):
                acceptance_count += 1
            if bool_value(row.get(ack_key)):
                ack_count += 1
            if bool_value(row.get(deadline_key)):
                deadline_missed_count += 1
            if bool_value(row.get(period_overrun_key)):
                period_overrun_count += 1
            if bool_value(row.get(within_period_key)):
                within_period_count += 1
            duration = finite_number(row.get(duration_key))
            if duration is not None:
                send_durations.append(duration)
            jitter = finite_number(row.get("jitter_ms"))
            if jitter is not None:
                jitter_values.append(abs(jitter))
    return {
        "source": "servo_log",
        "servo_log_rows": rows,
        "send_count": selected_rows,
        "send_success_count": send_success_count,
        "send_failure_count": send_failure_count,
        "controller_acceptance_observed_count": acceptance_count,
        "ack_observed_count": ack_count,
        "send_deadline_missed_count": deadline_missed_count,
        "send_period_overrun_count": period_overrun_count,
        "send_within_period_count": within_period_count,
        "send_duration_us": metric_block(send_durations),
        "servo_jitter_ms": metric_block(jitter_values),
    }


def series_for_key(states: list[dict[str, Any]], arm: str, key: str) -> list[list[float]]:
    out: list[list[float]] = []
    for snapshot in states:
        state = arm_state(snapshot, arm)
        if state is None:
            continue
        values = finite_joint_values(state.get(key))
        if values is not None:
            out.append(values)
    return out


def max_drift(series: list[list[float]]) -> float | None:
    if not series:
        return None
    first = series[0]
    return max(max(abs(a - b) for a, b in zip(values, first)) for values in series)


def state_stream_metrics(states: list[dict[str, Any]], arm: str) -> dict[str, Any]:
    arm_states = [state.get(arm) for state in states if isinstance(state.get(arm), dict)]
    state_ages: list[float] = []
    send_durations: list[float] = []
    ack_waits: list[float] = []
    host_times: list[int] = []
    send_success_count = 0
    send_failure_count = 0
    acceptance_count = 0
    ack_count = 0
    deadline_missed_count = 0
    period_overrun_count = 0
    stale_state_count = 0
    worker_enabled = False
    worker_drops = 0
    worker_pending_overwrites = 0
    valid_joint_count = 0
    for snapshot in states:
        host_time = finite_number(snapshot.get("loop_start_time_ns"))
        if host_time is None:
            host_time = finite_number(snapshot.get("host_time_ns"))
        if host_time is not None:
            host_times.append(int(host_time))
        state = arm_state(snapshot, arm)
        if state is None:
            continue
        if state.get("has_valid_joint_state") is True:
            valid_joint_count += 1
        age = finite_number(state.get("state_age_us"))
        if age is not None:
            state_ages.append(age)
        last_send = state.get("last_send") if isinstance(state.get("last_send"), dict) else {}
        duration = finite_number(last_send.get("duration_us")) or finite_number(last_send.get("send_duration_us"))
        if duration is None:
            duration = finite_number(state.get("send_duration_us"))
        if duration is not None:
            send_durations.append(duration)
        ack_wait = finite_number(last_send.get("ack_wait_duration_us"))
        if ack_wait is not None:
            ack_waits.append(ack_wait)
        accepted = last_send.get("accepted")
        if accepted is True or state.get("send_ok") is True:
            send_success_count += 1
        elif accepted is False or state.get("send_ok") is False:
            send_failure_count += 1
        if last_send.get("controller_acceptance_observed") is True:
            acceptance_count += 1
        if last_send.get("ack_observed") is True:
            ack_count += 1
        if state.get("send_command_deadline_missed") is True or snapshot.get("send_command_deadline_missed") is True:
            deadline_missed_count += 1
        if state.get("send_period_overrun") is True or snapshot.get("send_period_overrun") is True:
            period_overrun_count += 1
        worker = state.get("worker")
        if isinstance(worker, dict):
            worker_enabled = worker_enabled or worker.get("enabled") is True
            drops = finite_number(worker.get("command_drops_total"))
            if drops is not None:
                worker_drops = max(worker_drops, int(drops))
            overwrites = finite_number(worker.get("pending_overwrites_total"))
            if overwrites is not None:
                worker_pending_overwrites = max(worker_pending_overwrites, int(overwrites))
    intervals = [(b - a) / 1e6 for a, b in zip(host_times, host_times[1:]) if b >= a]
    jitter = [abs(interval - (1000.0 / COMMAND_RATE_HZ)) for interval in intervals]
    return {
        "source": "state_stream",
        "state_packet_count": len(states),
        "arm_state_packet_count": len(arm_states),
        "state_valid_ratio": valid_joint_count / len(arm_states) if arm_states else 0.0,
        "state_age_us": metric_block(state_ages),
        "state_stream_interval_ms": metric_block(intervals),
        "state_stream_jitter_ms": metric_block(jitter),
        "send_success_count": send_success_count,
        "send_failure_count": send_failure_count,
        "controller_acceptance_observed_count": acceptance_count,
        "ack_observed_count": ack_count,
        "send_deadline_missed_count": deadline_missed_count,
        "send_period_overrun_count": period_overrun_count,
        "send_duration_us": metric_block(send_durations),
        "ack_wait_duration_us": metric_block(ack_waits),
        "q_actual_drift_from_start_deg": max_drift(series_for_key(states, arm, "q_actual_deg")),
        "q_ref_drift_from_start_deg": max_drift(series_for_key(states, arm, "q_ref_deg")),
        "q_target_drift_from_start_deg": max_drift(series_for_key(states, arm, "q_target_deg")),
        "worker_path_used": worker_enabled,
        "worker_command_drops_total": worker_drops,
        "worker_pending_overwrites_total": worker_pending_overwrites,
        "fault_latched": any(snapshot.get("fault_latched") is True for snapshot in states),
        "stale_state_count": stale_state_count,
    }


def summarize_acceptance(
    args: argparse.Namespace,
    config: ParsedConfig,
    preflight_result: dict[str, Any],
    states: list[dict[str, Any]],
    command_run: CommandRunMetrics,
    artifact_dir: Path,
    server_returncode: int | None,
    servo_log: dict[str, Any] | None,
) -> dict[str, Any]:
    state_metrics = state_stream_metrics(states, args.arm)
    servo_metrics: dict[str, Any] | None = None
    if servo_log and isinstance(servo_log.get("path"), str):
        servo_metrics = parse_servo_log_metrics(
            Path(servo_log["path"]),
            args.arm,
            command_run.start_host_time_ns,
            command_run.end_host_time_ns,
        )

    timing_source = servo_metrics if servo_metrics and servo_metrics.get("send_count") else state_metrics
    q_actual_drift = state_metrics.get("q_actual_drift_from_start_deg")
    q_ref_drift = state_metrics.get("q_ref_drift_from_start_deg")
    physical_motion_detected = (
        q_actual_drift is not None and float(q_actual_drift) > args.max_physical_motion_deg
    )
    expected_send_count = command_run.expected_command_count
    send_count = int(timing_source.get("send_count") or command_run.command_count)
    acceptance_count = int(timing_source.get("controller_acceptance_observed_count") or 0)
    send_failure_count = int(timing_source.get("send_failure_count") or 0)
    send_duration = timing_source.get("send_duration_us") if isinstance(timing_source.get("send_duration_us"), dict) else {}
    servo_jitter = (
        timing_source.get("servo_jitter_ms")
        if isinstance(timing_source.get("servo_jitter_ms"), dict)
        else state_metrics.get("state_stream_jitter_ms")
    )
    if not isinstance(servo_jitter, dict):
        servo_jitter = {}
    send_deadline_missed_count = int(timing_source.get("send_deadline_missed_count") or 0)
    failures = threshold_failures(
        args,
        expected_send_count=expected_send_count,
        send_count=send_count,
        acceptance_count=acceptance_count,
        send_failure_count=send_failure_count,
        send_duration=send_duration,
        servo_jitter=servo_jitter,
        send_deadline_missed_count=send_deadline_missed_count,
        state_metrics=state_metrics,
        timing_source_name=str(timing_source.get("source")),
        physical_motion_detected=physical_motion_detected,
    )
    result = "fail" if failures else "pass"
    result_reason = "thresholds applied and failed" if failures else "500 Hz no-op thresholds satisfied"
    summary = {
        "schema": SCHEMA,
        "result": result,
        "result_reason": result_reason,
        "artifact_dir": str(artifact_dir.resolve()),
        "config": str(config.path),
        "mode": MODE,
        "arm": args.arm,
        "duration_sec": args.duration_sec,
        "command_rate_hz": COMMAND_RATE_HZ,
        "expected_send_count": expected_send_count,
        "udp_command_count": command_run.command_count,
        "send_count": send_count,
        "send_count_source": timing_source.get("source"),
        "achieved_udp_command_rate_hz": command_run.command_count / command_run.elapsed_sec,
        "controller_acceptance_observed_count": acceptance_count,
        "controller_acceptance_ratio": acceptance_count / send_count if send_count > 0 else None,
        "send_success_count": int(timing_source.get("send_success_count") or 0),
        "send_failure_count": send_failure_count,
        "ack_observed_count": int(timing_source.get("ack_observed_count") or 0),
        "send_duration_us": send_duration,
        "servo_jitter_ms": servo_jitter,
        "send_deadline_missed_count": send_deadline_missed_count,
        "send_period_overrun_count": int(timing_source.get("send_period_overrun_count") or 0),
        "command_sender_deadline_missed_count": command_run.sender_deadline_missed_count,
        "command_sender_max_lateness_us": command_run.max_sender_lateness_us,
        "state_packet_count": len(states),
        "state_valid_ratio": state_metrics.get("state_valid_ratio"),
        "state_age_us": state_metrics.get("state_age_us"),
        "q_actual_drift_from_start_deg": q_actual_drift,
        "q_ref_drift_from_start_deg": q_ref_drift,
        "q_target_drift_from_start_deg": state_metrics.get("q_target_drift_from_start_deg"),
        "physical_motion_expected": False,
        "physical_motion_detected": physical_motion_detected,
        "physical_motion_warning_threshold_deg": args.max_physical_motion_deg,
        "fault_latched": state_metrics.get("fault_latched"),
        "worker_path_used": state_metrics.get("worker_path_used"),
        "worker_command_drops_total": state_metrics.get("worker_command_drops_total"),
        "worker_pending_overwrites_total": state_metrics.get("worker_pending_overwrites_total"),
        "server_returncode": server_returncode,
        "state_stream": str((artifact_dir / "state_stream.jsonl").resolve()),
        "command_packets": str((artifact_dir / "command_packets.jsonl").resolve()),
        "rb_servo_server_log": str((artifact_dir / "rb_servo_server.log").resolve()),
        "servo_log": servo_log,
        "servo_log_metrics": servo_metrics,
        "state_stream_metrics": state_metrics,
        "safety_preflight": preflight_result,
        "thresholds": {
            "min_send_count_ratio": args.min_send_count_ratio,
            "min_controller_acceptance_ratio": args.min_controller_acceptance_ratio,
            "max_send_duration_p99_us": args.max_send_duration_p99_us,
            "max_servo_jitter_p99_ms": args.max_servo_jitter_p99_ms,
            "max_deadline_miss_count": args.max_deadline_miss_count,
            "max_worker_drop_count": args.max_worker_drop_count,
            "max_physical_motion_deg": args.max_physical_motion_deg,
            "max_reference_drift_deg": args.max_reference_drift_deg,
        },
        "threshold_failures": failures,
        "caveat": (
            "rbpodo controller-simulation no-op evidence; controller boxes are real, "
            "physical robot motion is not expected or approved"
        ),
    }
    return summary


def threshold_failures(
    args: argparse.Namespace,
    *,
    expected_send_count: int,
    send_count: int,
    acceptance_count: int,
    send_failure_count: int,
    send_duration: dict[str, Any],
    servo_jitter: dict[str, Any],
    send_deadline_missed_count: int,
    state_metrics: dict[str, Any],
    timing_source_name: str,
    physical_motion_detected: bool,
) -> list[str]:
    failures: list[str] = []
    min_send_count = math.floor(expected_send_count * args.min_send_count_ratio)
    if send_count < min_send_count:
        failures.append(f"send_count {send_count} below minimum {min_send_count} for expected {expected_send_count}")
    min_acceptance = math.floor(max(send_count, 1) * args.min_controller_acceptance_ratio)
    if acceptance_count < min_acceptance:
        failures.append(
            f"controller_acceptance_observed_count {acceptance_count} below minimum {min_acceptance} "
            f"for send_count {send_count}"
        )
    p99_send = finite_number(send_duration.get("p99"))
    if p99_send is None:
        failures.append(f"send_duration_us.p99 unavailable from {timing_source_name}")
    elif p99_send > args.max_send_duration_p99_us:
        failures.append(f"send_duration_us.p99 {p99_send:.3f} exceeds {args.max_send_duration_p99_us:.3f}")
    p99_jitter = finite_number(servo_jitter.get("p99"))
    if p99_jitter is None:
        failures.append(f"servo_jitter_ms.p99 unavailable from {timing_source_name}")
    elif p99_jitter > args.max_servo_jitter_p99_ms:
        failures.append(f"servo_jitter_ms.p99 {p99_jitter:.3f} exceeds {args.max_servo_jitter_p99_ms:.3f}")
    if send_deadline_missed_count > args.max_deadline_miss_count:
        failures.append(
            f"send_deadline_missed_count {send_deadline_missed_count} exceeds {args.max_deadline_miss_count}"
        )
    if state_metrics.get("fault_latched") is True:
        failures.append("fault_latched was true")
    if physical_motion_detected:
        failures.append(
            "physical q_actual motion exceeded threshold: "
            f"{state_metrics.get('q_actual_drift_from_start_deg')} deg"
        )
    q_ref_drift = finite_number(state_metrics.get("q_ref_drift_from_start_deg"))
    if q_ref_drift is not None and q_ref_drift > args.max_reference_drift_deg:
        failures.append(f"q_ref_drift_from_start_deg {q_ref_drift:.6f} exceeds {args.max_reference_drift_deg:.6f}")
    state_valid_ratio = finite_number(state_metrics.get("state_valid_ratio"))
    if state_valid_ratio is None or state_valid_ratio <= 0.0:
        failures.append("state stream did not contain valid selected-arm joint state")
    worker_drops = int(state_metrics.get("worker_command_drops_total") or 0)
    if state_metrics.get("worker_path_used") is True and worker_drops > args.max_worker_drop_count:
        failures.append(f"worker_command_drops_total {worker_drops} exceeds {args.max_worker_drop_count}")
    if send_failure_count > 0:
        failures.append(f"{timing_source_name} reported send failures: {send_failure_count}")
    return failures


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_summary_csv(path: Path, summary: dict[str, Any]) -> None:
    fields = [
        "result",
        "result_reason",
        "mode",
        "arm",
        "duration_sec",
        "expected_send_count",
        "send_count",
        "controller_acceptance_observed_count",
        "controller_acceptance_ratio",
        "send_duration_p99_us",
        "servo_jitter_p99_ms",
        "send_deadline_missed_count",
        "worker_command_drops_total",
        "physical_motion_detected",
        "q_actual_drift_from_start_deg",
        "q_ref_drift_from_start_deg",
        "fault_latched",
    ]
    row = {field: summary.get(field) for field in fields}
    row["send_duration_p99_us"] = (summary.get("send_duration_us") or {}).get("p99")
    row["servo_jitter_p99_ms"] = (summary.get("servo_jitter_ms") or {}).get("p99")
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerow(row)


def write_plots(artifact_dir: Path, summary: dict[str, Any]) -> tuple[list[str], list[str]]:
    if summary.get("skip_plots"):
        return [], ["plots skipped"]
    servo_log = summary.get("servo_log")
    if not isinstance(servo_log, dict) or not isinstance(servo_log.get("path"), str):
        return [], ["servo_log unavailable; timing plots skipped"]
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        return [], [f"matplotlib unavailable: {exc}"]
    path = Path(servo_log["path"])
    jitters: list[float] = []
    sends: list[float] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        duration_key = f"{summary['arm']}_send_duration_us"
        for row in reader:
            jitter = finite_number(row.get("jitter_ms"))
            if jitter is not None:
                jitters.append(abs(jitter))
            duration = finite_number(row.get(duration_key))
            if duration is not None:
                sends.append(duration)
    generated: list[str] = []
    if sends:
        plt.figure(figsize=(8, 4))
        plt.plot(sends)
        plt.xlabel("servo log sample")
        plt.ylabel("send_duration_us")
        plt.tight_layout()
        target = artifact_dir / "timing_send_duration.png"
        plt.savefig(target)
        plt.close()
        generated.append(str(target.resolve()))
    if jitters:
        plt.figure(figsize=(8, 4))
        plt.plot(jitters)
        plt.xlabel("servo log sample")
        plt.ylabel("abs(jitter_ms)")
        plt.tight_layout()
        target = artifact_dir / "timing_servo_jitter.png"
        plt.savefig(target)
        plt.close()
        generated.append(str(target.resolve()))
    return generated, []


def run_acceptance(args: argparse.Namespace) -> dict[str, Any]:
    args.root = args.root.resolve()
    config, preflight_result = preflight(args)
    artifact_dir = args.artifact_dir.resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    write_json(artifact_dir / "safety_preflight.json", preflight_result)
    write_json(artifact_dir / "pgmode_summary.json", preflight_result["pgmode_summary"])
    shutil.copy2(config.path, artifact_dir / "raw_config.yaml")
    if args.preflight_only:
        summary = {
            "schema": SCHEMA,
            "result": "pass",
            "result_reason": "preflight only",
            "artifact_dir": str(artifact_dir),
            "mode": MODE,
            "safety_preflight": preflight_result,
            "preflight_only": True,
        }
        write_json(artifact_dir / "summary.json", summary)
        write_summary_csv(artifact_dir / "summary.csv", summary)
        return summary

    capture = StateCapture(
        str(preflight_result["state_host"]),
        int(preflight_result["state_port"]),
        artifact_dir / "state_stream.jsonl",
    )
    server_proc: subprocess.Popen[str] | None = None
    server_log_path = artifact_dir / "rb_servo_server.log"
    started_at = time.time()
    command_run = CommandRunMetrics(0, 0, 0, 0, 0.0, 0, 0.0, False)
    try:
        capture.start()
        server_proc = start_server(args, server_log_path, preflight_result)
        first_state = wait_for_start_state(
            capture,
            args,
            server_proc,
            server_log_path,
            str(preflight_result["state_endpoint"]),
        )
        q_noop, target_source = noop_target_from_state(first_state, args.arm)
        q_actual_from_state(first_state, args.arm)
        with (artifact_dir / "noop_target.json").open("w", encoding="utf-8") as handle:
            json.dump(
                {
                    "arm": args.arm,
                    "target_source": target_source,
                    "q_target_deg": q_noop,
                    "startup_state_excerpt": state_excerpt(first_state, args.arm),
                },
                handle,
                indent=2,
                sort_keys=True,
            )
            handle.write("\n")
        # Send ArmMotion first, then wait for ArmedHold before streaming no-op JointTarget packets.
        host = str(preflight_result["command_host"])
        port = int(preflight_result["command_port"])
        session_id = f"rbpodo-500hz-{now_ns()}"
        with (artifact_dir / "command_packets.jsonl").open("w", encoding="utf-8") as handle:
            arm_packet = lifecycle_packet("ArmMotion", 1, session_id)
            send_udp(host, port, arm_packet, handle)
            armed_after_ns = int(arm_packet["host_time_ns"])
        wait_for_armed(capture, args, armed_after_ns)
        command_run = send_noop_stream(
            args,
            preflight_result,
            q_noop,
            artifact_dir / "command_packets.jsonl",
            session_id,
        )
        if args.settle_sec > 0.0:
            time.sleep(args.settle_sec)
    finally:
        stop_server(server_proc)
        capture.stop()
    server_returncode = server_proc.returncode if server_proc is not None else None
    servo_log = copy_servo_log(args.root, config, artifact_dir, started_at)
    summary = summarize_acceptance(
        args,
        config,
        preflight_result,
        capture.snapshots,
        command_run,
        artifact_dir,
        server_returncode,
        servo_log,
    )
    if args.skip_plots:
        summary["generated_plots"] = []
        summary["skipped_plots"] = ["plots skipped by --skip-plots"]
    else:
        generated_plots, skipped_plots = write_plots(artifact_dir, summary)
        summary["generated_plots"] = generated_plots
        summary["skipped_plots"] = skipped_plots
    write_json(artifact_dir / "summary.json", summary)
    write_summary_csv(artifact_dir / "summary.csv", summary)
    return summary


def failure_summary(args: argparse.Namespace, exc: Exception) -> dict[str, Any]:
    artifact_dir = args.artifact_dir.resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    summary = {
        "schema": SCHEMA,
        "result": "error",
        "result_reason": "500 Hz no-op acceptance could not run",
        "error": str(exc),
        "mode": MODE,
        "artifact_dir": str(artifact_dir),
        "safety_preflight": {"passed": False, "error": str(exc), "env": env_snapshot()},
        "threshold_failures": [str(exc)],
        "caveat": "No acceptance evidence was produced.",
    }
    write_json(artifact_dir / "summary.json", summary)
    try:
        write_summary_csv(artifact_dir / "summary.csv", summary)
    except Exception:
        pass
    return summary


def main() -> int:
    args = parse_args()
    try:
        summary = run_acceptance(args)
    except Exception as exc:
        failure_summary(args, exc)
        print(f"rbpodo_500hz_acceptance: FAIL: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary.get("result") == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
