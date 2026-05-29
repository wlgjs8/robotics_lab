#!/usr/bin/env python3
"""Supervised rbscript_tcp controller-simulation Servo J no-op harness.

This script talks to the experimental Rainbow script TCP command/data ports
directly. It is for controller pgmode simulation no-op comparison only; it does
not authorize physical robot motion.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shutil
import socket
import sys
import tempfile
import threading
import time
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import rb_backend_ablation as backend_probe


REAL_ROBOT_IPS = {"172.28.60.200", "172.28.60.201"}
ENV_KEYS = (
    "RB_ALLOW_REAL_ROBOT",
    "RB_ALLOW_REAL_MOTION",
    "RB_ALLOW_RBSCRIPT_TCP",
    "RB_ALLOW_RBSCRIPT_TCP_MOTION",
    "RB_ALLOW_REAL_CARTESIAN",
)
PROFILES = {
    "100hz_ack": {"rate_hz": 100, "script_t1_sec": 0.01, "disable_waiting_ack": False},
    "200hz_ack": {"rate_hz": 200, "script_t1_sec": 0.005, "disable_waiting_ack": False},
    "200hz_no_ack": {"rate_hz": 200, "script_t1_sec": 0.005, "disable_waiting_ack": True},
}
M_CODES = ("M561", "M568", "M569", "M570")
MOTION_MODES = {"servo_j_noop", "tiny_joint_motion"}


class AcceptanceError(RuntimeError):
    pass


@dataclass
class ArmConfig:
    backend_type: str = ""
    run_mode: str = ""
    ip: str = ""
    operation_mode: str = ""
    command_port: int = 5000
    data_port: int = 5001
    command_timeout_sec: float | None = None
    read_timeout_sec: float | None = None
    connect_timeout_sec: float | None = None
    disable_waiting_ack: bool = False
    script_t1_sec: float | None = None
    script_t2_sec: float | None = None
    script_gain: float | None = None
    script_alpha: float | None = None


@dataclass
class ParsedConfig:
    path: Path
    left: ArmConfig
    right: ArmConfig
    servo: dict[str, Any]
    network: dict[str, Any]


@dataclass
class CommandSample:
    index: int
    mode: str
    success: bool
    start_ns: int
    end_ns: int
    duration_us: float
    command_write_duration_us: float | None = None
    ack_wait_duration_us: float | None = None
    ack_policy: str = ""
    ack_observed: bool = False
    controller_acceptance_observed: bool = False
    send_acceptance_semantics: str = ""
    error_name: str = ""
    error_message: str = ""
    response: str = ""
    response_lines: list[str] = field(default_factory=list)
    extra_response_lines: list[str] = field(default_factory=list)
    response_error_names: list[str] = field(default_factory=list)
    reconnect_count: int = 0
    command_text: str = ""


class PersistentCommandClient:
    def __init__(self, ip: str, port: int, connect_timeout_sec: float, command_timeout_sec: float) -> None:
        self.ip = ip
        self.port = port
        self.connect_timeout_sec = connect_timeout_sec
        self.command_timeout_sec = command_timeout_sec
        self.sock: socket.socket | None = None
        self.connected_once = False

    def close(self) -> None:
        if self.sock is None:
            return
        try:
            self.sock.close()
        except OSError:
            pass
        self.sock = None

    def socket(self) -> tuple[socket.socket, int]:
        if self.sock is not None:
            return self.sock, 0
        sock = backend_probe.tcp_connect(self.ip, self.port, self.connect_timeout_sec)
        reconnect = 1 if self.connected_once else 0
        self.connected_once = True
        self.sock = sock
        return sock, reconnect

    def send(self, command: str, *, wait_ack: bool) -> CommandSample:
        start = time.monotonic_ns()
        reconnect_count = 0
        write_us: float | None = None
        ack_wait_us: float | None = None
        response = ""
        response_lines: list[str] = []
        extra_lines: list[str] = []
        names: list[str] = []
        try:
            sock, reconnect_count = self.socket()
            sock.settimeout(self.command_timeout_sec)
            write_start = time.monotonic_ns()
            sock.sendall(command.encode("utf-8"))
            write_end = time.monotonic_ns()
            write_us = (write_end - write_start) / 1000.0
            if wait_ack:
                ack_start = time.monotonic_ns()
                response = backend_probe.recv_line(sock, self.command_timeout_sec)
                ack_end = time.monotonic_ns()
                ack_wait_us = (ack_end - ack_start) / 1000.0
                response_lines = [response]
                extra_lines, closed = backend_probe.drain_response_lines(
                    sock,
                    timeout_sec=min(self.command_timeout_sec, 0.001),
                )
                if closed:
                    self.close()
                ok, names, _ = backend_probe.classify_response_lines(response_lines + extra_lines)
                end = time.monotonic_ns()
                return CommandSample(
                    index=-1,
                    mode="servo_j_noop",
                    success=ok,
                    start_ns=start,
                    end_ns=end,
                    duration_us=(end - start) / 1000.0,
                    command_write_duration_us=write_us,
                    ack_wait_duration_us=ack_wait_us,
                    ack_policy="wait",
                    ack_observed=True,
                    controller_acceptance_observed=ok,
                    send_acceptance_semantics="controller_ack_observed" if ok else "controller_rejected",
                    error_name="" if ok else (names[0] if names else "ack_error"),
                    response=response,
                    response_lines=response_lines,
                    extra_response_lines=extra_lines,
                    response_error_names=names,
                    reconnect_count=reconnect_count,
                    command_text=command,
                )
            end = time.monotonic_ns()
            return CommandSample(
                index=-1,
                mode="servo_j_noop",
                success=True,
                start_ns=start,
                end_ns=end,
                duration_us=(end - start) / 1000.0,
                command_write_duration_us=write_us,
                ack_policy="disabled",
                ack_observed=False,
                controller_acceptance_observed=False,
                send_acceptance_semantics="socket_send_only",
                reconnect_count=reconnect_count,
                command_text=command,
            )
        except socket.timeout as exc:
            self.close()
            end = time.monotonic_ns()
            return CommandSample(
                index=-1,
                mode="servo_j_noop",
                success=False,
                start_ns=start,
                end_ns=end,
                duration_us=(end - start) / 1000.0,
                command_write_duration_us=write_us,
                ack_wait_duration_us=ack_wait_us,
                ack_policy="wait" if wait_ack else "disabled",
                error_name="TransportTimeout",
                error_message=str(exc),
                response=response,
                response_lines=response_lines,
                extra_response_lines=extra_lines,
                response_error_names=names,
                reconnect_count=reconnect_count,
                command_text=command,
            )
        except Exception as exc:
            self.close()
            end = time.monotonic_ns()
            return CommandSample(
                index=-1,
                mode="servo_j_noop",
                success=False,
                start_ns=start,
                end_ns=end,
                duration_us=(end - start) / 1000.0,
                command_write_duration_us=write_us,
                ack_wait_duration_us=ack_wait_us,
                ack_policy="wait" if wait_ack else "disabled",
                error_name=type(exc).__name__,
                error_message=str(exc),
                response=response,
                response_lines=response_lines,
                extra_response_lines=extra_lines,
                response_error_names=names,
                reconnect_count=reconnect_count,
                command_text=command,
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run experimental rbscript_tcp controller-simulation Servo J no-op "
            "acceptance. Servo J sends are refused unless explicit motion, env, "
            "operation_mode=simulation, and confirmation gates are all present."
        )
    )
    parser.add_argument("--config", type=Path)
    parser.add_argument("--arm", choices=("left", "right"))
    parser.add_argument(
        "--mode",
        choices=("read_only", "servo_j_noop", "tiny_joint_motion"),
        default="read_only",
    )
    parser.add_argument("--profile", choices=tuple(PROFILES))
    parser.add_argument("--duration-sec", type=float, default=10.0)
    parser.add_argument("--command-rate-hz", type=float)
    parser.add_argument("--artifact-dir", type=Path)
    parser.add_argument("--q-current-deg", help="Six comma-separated current joint angles in degrees.")
    parser.add_argument(
        "--q-current-from-rbpodo",
        action="store_true",
        help="Use rbpodo only for q_current state acquisition; rbscript_tcp remains the command path.",
    )
    parser.add_argument("--q-current-tolerance-deg", type=float, default=1e-6)
    parser.add_argument("--allow-motion", action="store_true")
    parser.add_argument("--allow-ack-disabled", action="store_true")
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--skip-plots", action="store_true")
    parser.add_argument(
        "--i-understand-this-connects-to-real-controller",
        action="store_true",
        help="Required before connecting to known RB controller IPs.",
    )
    parser.add_argument("--self-test", action="store_true", help=argparse.SUPPRESS)
    return parser.parse_args()


def strip_comment(line: str) -> str:
    in_quote = False
    quote_char = ""
    for index, char in enumerate(line):
        if char in {"'", '"'}:
            if not in_quote:
                in_quote = True
                quote_char = char
            elif quote_char == char:
                in_quote = False
        if char == "#" and not in_quote:
            return line[:index]
    return line


def scalar_value(text: str) -> Any:
    value = text.strip()
    if not value:
        return ""
    if value in {"true", "True"}:
        return True
    if value in {"false", "False"}:
        return False
    if value in {"null", "None", "~"}:
        return None
    if (value.startswith('"') and value.endswith('"')) or (value.startswith("'") and value.endswith("'")):
        return value[1:-1]
    if value.startswith("[") and value.endswith("]"):
        items = [item.strip() for item in value[1:-1].split(",") if item.strip()]
        return [scalar_value(item) for item in items]
    try:
        if any(ch in value for ch in ".eE"):
            return float(value)
        return int(value, 10)
    except ValueError:
        return value


def simple_yaml_sections(path: Path) -> dict[str, dict[str, Any]]:
    sections: dict[str, dict[str, Any]] = {}
    current: str | None = None
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = strip_comment(raw_line).rstrip()
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip(" "))
        text = line.strip()
        if indent == 0 and text.endswith(":"):
            current = text[:-1]
            sections[current] = {}
            continue
        if indent == 0:
            current = None
            continue
        if current is None or ":" not in text:
            continue
        key, value = text.split(":", 1)
        sections.setdefault(current, {})[key.strip()] = scalar_value(value)
    return sections


def finite_float(value: Any, default: float | None = None) -> float | None:
    if value is None or isinstance(value, bool):
        return default
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    return out if math.isfinite(out) else default


def int_port(value: Any, default: int) -> int:
    try:
        port = int(value)
    except (TypeError, ValueError):
        return default
    return port if 1 <= port <= 65535 else default


def bool_value(value: Any, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in {"1", "true", "yes", "on"}
    return default


def arm_config(section: dict[str, Any]) -> ArmConfig:
    return ArmConfig(
        backend_type=str(section.get("backend_type", "")),
        run_mode=str(section.get("run_mode", "")),
        ip=str(section.get("ip", "")),
        operation_mode=str(section.get("operation_mode", "")),
        command_port=int_port(section.get("command_port"), 5000),
        data_port=int_port(section.get("data_port"), 5001),
        command_timeout_sec=finite_float(section.get("command_timeout_sec")),
        read_timeout_sec=finite_float(section.get("read_timeout_sec")),
        connect_timeout_sec=finite_float(section.get("connect_timeout_sec")),
        disable_waiting_ack=bool_value(section.get("disable_waiting_ack"), False),
        script_t1_sec=finite_float(section.get("script_t1_sec")),
        script_t2_sec=finite_float(section.get("script_t2_sec")),
        script_gain=finite_float(section.get("script_gain")),
        script_alpha=finite_float(section.get("script_alpha")),
    )


def load_config(path: Path) -> ParsedConfig:
    if not path.is_file():
        raise AcceptanceError(f"config not found: {path}")
    sections = simple_yaml_sections(path)
    return ParsedConfig(
        path=path.resolve(),
        left=arm_config(sections.get("left_robot", {})),
        right=arm_config(sections.get("right_robot", {})),
        servo=sections.get("servo", {}),
        network=sections.get("network", {}),
    )


def selected_arm(config: ParsedConfig, arm: str) -> ArmConfig:
    return config.left if arm == "left" else config.right


def env_enabled(name: str) -> bool:
    return os.environ.get(name) == "1"


def env_snapshot() -> dict[str, str | None]:
    return {key: os.environ.get(key) for key in ENV_KEYS}


def parse_q_current(text: str) -> list[float]:
    try:
        values = [float(item.strip()) for item in text.split(",")]
    except ValueError as exc:
        raise AcceptanceError("--q-current-deg must contain six finite comma-separated numbers") from exc
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise AcceptanceError("--q-current-deg must contain six finite comma-separated numbers")
    return values


def max_abs_delta(left: list[float], right: list[float]) -> float:
    return max(abs(a - b) for a, b in zip(left, right))


def validate_profile(config: ParsedConfig, args: argparse.Namespace) -> str:
    rate = int(config.servo.get("rate_hz", 0) or 0)
    tolerance_ratio = finite_float(config.servo.get("servo_t1_rate_match_tolerance_ratio"), 0.2) or 0.2
    arm = selected_arm(config, args.arm)
    if rate <= 0:
        raise AcceptanceError("servo.rate_hz must be positive")
    if arm.script_t1_sec is None:
        raise AcceptanceError(f"{args.arm}_robot.script_t1_sec is required")
    dt = 1.0 / float(rate)
    if abs(arm.script_t1_sec - dt) > tolerance_ratio * dt:
        raise AcceptanceError(
            f"profile mismatch: script_t1_sec={arm.script_t1_sec} does not match rate period {dt:.6f}"
        )
    if args.profile:
        expected = PROFILES[args.profile]
        if rate != expected["rate_hz"]:
            raise AcceptanceError(f"profile {args.profile} expects servo.rate_hz={expected['rate_hz']}, got {rate}")
        if abs(arm.script_t1_sec - expected["script_t1_sec"]) > tolerance_ratio * dt:
            raise AcceptanceError(f"profile {args.profile} expects script_t1_sec={expected['script_t1_sec']}")
        if arm.disable_waiting_ack != expected["disable_waiting_ack"]:
            raise AcceptanceError(f"profile {args.profile} ACK setting does not match disable_waiting_ack")
        return args.profile
    for name, expected in PROFILES.items():
        if (
            rate == expected["rate_hz"]
            and abs(arm.script_t1_sec - expected["script_t1_sec"]) <= tolerance_ratio * dt
            and arm.disable_waiting_ack == expected["disable_waiting_ack"]
        ):
            return name
    raise AcceptanceError("config does not match a supported rbscript acceptance profile")


def preflight(args: argparse.Namespace, config: ParsedConfig) -> dict[str, Any]:
    if not math.isfinite(args.duration_sec) or args.duration_sec <= 0.0:
        raise AcceptanceError("--duration-sec must be finite and positive")
    if not math.isfinite(args.q_current_tolerance_deg) or args.q_current_tolerance_deg < 0.0:
        raise AcceptanceError("--q-current-tolerance-deg must be finite and nonnegative")
    if args.q_current_deg and args.q_current_from_rbpodo:
        raise AcceptanceError("choose only one q source: --q-current-deg or --q-current-from-rbpodo")

    for label, arm_cfg in (("left", config.left), ("right", config.right)):
        if arm_cfg.backend_type != "rbscript_tcp":
            raise AcceptanceError(f"{label}_robot.backend_type must be rbscript_tcp")
        if arm_cfg.run_mode != "real":
            raise AcceptanceError(f"{label}_robot.run_mode must be real for rbscript_tcp controller comparison")
        if not arm_cfg.ip:
            raise AcceptanceError(f"{label}_robot.ip is required")

    profile = validate_profile(config, args)
    selected = selected_arm(config, args.arm)
    known_real_ips = {config.left.ip, config.right.ip} & REAL_ROBOT_IPS
    if known_real_ips and not args.i_understand_this_connects_to_real_controller:
        raise AcceptanceError("refusing known real controller IP without explicit confirmation flag")
    if known_real_ips:
        missing = [key for key in ("RB_ALLOW_REAL_ROBOT", "RB_ALLOW_RBSCRIPT_TCP") if not env_enabled(key)]
        if missing:
            raise AcceptanceError("rbscript_tcp real connect/read requires env gates: " + ", ".join(missing))
    if env_enabled("RB_ALLOW_REAL_CARTESIAN"):
        raise AcceptanceError("RB_ALLOW_REAL_CARTESIAN must not be set for rbscript Servo J acceptance")

    send_servo_commands = bool_value(config.servo.get("send_servo_commands"), False)
    if selected.disable_waiting_ack and not args.allow_ack_disabled:
        raise AcceptanceError("ACK-off profile requires --allow-ack-disabled")
    if args.mode == "read_only" and send_servo_commands:
        raise AcceptanceError("read_only mode requires servo.send_servo_commands=false")
    if args.mode == "tiny_joint_motion":
        raise AcceptanceError("tiny_joint_motion is not implemented in RBSCRIPT-SERVO-NOOP-01")
    if args.mode in MOTION_MODES:
        if not args.allow_motion:
            raise AcceptanceError(f"{args.mode} requires --allow-motion")
        if not args.i_understand_this_connects_to_real_controller:
            raise AcceptanceError(f"{args.mode} requires explicit real-controller confirmation")
        if selected.operation_mode != "simulation":
            raise AcceptanceError("servo_j_noop requires selected arm operation_mode=simulation")
        if not send_servo_commands:
            raise AcceptanceError("servo_j_noop requires a sim_noop config with servo.send_servo_commands=true")
        required = (
            "RB_ALLOW_REAL_ROBOT",
            "RB_ALLOW_REAL_MOTION",
            "RB_ALLOW_RBSCRIPT_TCP",
            "RB_ALLOW_RBSCRIPT_TCP_MOTION",
        )
        missing = [key for key in required if not env_enabled(key)]
        if missing:
            raise AcceptanceError("refusing rbscript Servo J send; missing env gates: " + ", ".join(missing))

    return {
        "passed": True,
        "backend": "rbscript_tcp",
        "profile": profile,
        "mode": args.mode,
        "arm": args.arm,
        "config": str(config.path),
        "send_servo_commands": send_servo_commands,
        "selected_ip": selected.ip,
        "selected_operation_mode": selected.operation_mode,
        "command_port": selected.command_port,
        "data_port": selected.data_port,
        "servo_rate_hz": int(config.servo.get("rate_hz", 0) or 0),
        "script_t1_sec": selected.script_t1_sec,
        "script_t2_sec": selected.script_t2_sec,
        "script_gain": selected.script_gain,
        "script_alpha": selected.script_alpha,
        "disable_waiting_ack": selected.disable_waiting_ack,
        "ack_semantics": "socket_send_only" if selected.disable_waiting_ack else "controller_ack_observed",
        "persistent_socket": True,
        "q_source_requested": (
            "cli" if args.q_current_deg else ("rbpodo" if args.q_current_from_rbpodo else "rbscript_data")
        ),
        "env": env_snapshot(),
        "real_robot_ips_checked": sorted(REAL_ROBOT_IPS),
        "confirmation_flag": args.i_understand_this_connects_to_real_controller,
    }


def now_ns() -> int:
    return time.monotonic_ns()


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


def count_m_codes(text: Any, counts: Counter[str]) -> None:
    if text is None:
        return
    value = str(text)
    for code in M_CODES:
        if code in value:
            counts[code] += 1


def numeric(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)) and math.isfinite(float(value)):
        return float(value)
    return None


def format_move_servo_j(q_target_deg: list[float], arm: ArmConfig) -> str:
    required = {
        "script_t1_sec": arm.script_t1_sec,
        "script_t2_sec": arm.script_t2_sec,
        "script_gain": arm.script_gain,
        "script_alpha": arm.script_alpha,
    }
    missing = [name for name, value in required.items() if value is None]
    if missing:
        raise AcceptanceError("missing rbscript Servo J config fields: " + ", ".join(missing))
    joints = ",".join(f"{value:.12g}" for value in q_target_deg)
    return (
        f"move_servo_j(jnt[{joints}],"
        f"{arm.script_t1_sec:.12g},{arm.script_t2_sec:.12g},"
        f"{arm.script_gain:.12g},{arm.script_alpha:.12g})\n"
    )


def parse_rbscript_state_payload(text: str) -> tuple[list[float] | None, dict[str, Any]]:
    try:
        payload = json.loads(text)
    except json.JSONDecodeError as exc:
        return None, {"available": False, "error_name": "rbscript_tcp_data_unrecognized_format", "error_message": str(exc)}
    if not isinstance(payload, dict) or payload.get("schema") != "rbscript_tcp_state_v1":
        return None, {"available": False, "error_name": "rbscript_tcp_data_unsupported_schema"}
    q_actual = payload.get("q_actual_deg")
    if not (
        isinstance(q_actual, list)
        and len(q_actual) == 6
        and all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in q_actual)
    ):
        return None, {"available": False, "error_name": "rbscript_tcp_data_invalid_q_actual"}
    return [float(value) for value in q_actual], {"available": True, "error_name": "", "payload": payload}


def read_rbscript_state(arm: ArmConfig) -> tuple[list[float] | None, dict[str, Any]]:
    start = now_ns()
    response = ""
    try:
        with backend_probe.tcp_connect(arm.ip, arm.data_port, arm.connect_timeout_sec or 1.0) as sock:
            sock.settimeout(arm.read_timeout_sec or 0.2)
            sock.sendall(b"reqdata\n")
            response = backend_probe.recv_line(sock, arm.read_timeout_sec or 0.2)
        q_actual, result = parse_rbscript_state_payload(response)
        end = now_ns()
        result.update({
            "response": response,
            "duration_us": (end - start) / 1000.0,
            "q_actual_deg": q_actual,
        })
        return q_actual, result
    except socket.timeout as exc:
        end = now_ns()
        return None, {
            "available": False,
            "error_name": "TransportTimeout",
            "error_message": str(exc),
            "response": response,
            "duration_us": (end - start) / 1000.0,
        }
    except Exception as exc:
        end = now_ns()
        return None, {
            "available": False,
            "error_name": type(exc).__name__,
            "error_message": str(exc),
            "response": response,
            "duration_us": (end - start) / 1000.0,
        }


def acquire_q_from_rbpodo(arm: ArmConfig, read_timeout_sec: float) -> list[float]:
    try:
        import rbpodo  # type: ignore
    except Exception as exc:
        raise AcceptanceError("--q-current-from-rbpodo requires an installed Python rbpodo module") from exc
    if not hasattr(rbpodo, "CobotData"):
        raise AcceptanceError("Python rbpodo module does not expose CobotData")
    state = rbpodo.CobotData(arm.ip).request_data(read_timeout_sec)
    sdata = getattr(state, "sdata", state)
    q_actual = list(getattr(sdata, "jnt_ang", []))
    if len(q_actual) != 6 or not all(numeric(value) is not None for value in q_actual):
        raise AcceptanceError("rbpodo state did not contain six finite jnt_ang values")
    return [float(value) for value in q_actual]


def acquire_q_current(args: argparse.Namespace, arm: ArmConfig) -> tuple[list[float], str, dict[str, Any]]:
    if args.q_current_deg:
        q = parse_q_current(args.q_current_deg)
        return q, "cli", {"available": True, "q_actual_deg": q}
    if args.q_current_from_rbpodo:
        q = acquire_q_from_rbpodo(arm, arm.read_timeout_sec or 0.2)
        return q, "rbpodo", {"available": True, "q_actual_deg": q}
    q, state = read_rbscript_state(arm)
    if q is None:
        raise AcceptanceError(
            "rbscript data-port q_current unavailable; pass --q-current-deg or --q-current-from-rbpodo"
        )
    return q, "rbscript_data", state


def read_only_probe(config: ParsedConfig, args: argparse.Namespace) -> tuple[list[CommandSample], dict[str, Any]]:
    arm = selected_arm(config, args.arm)
    result: dict[str, Any] = {
        "command_port_connected": False,
        "data_port_connected": False,
        "read_state_available": False,
        "q_actual_deg": None,
        "data_error_name": "",
        "data_error_message": "",
    }
    samples: list[CommandSample] = []
    start = now_ns()
    try:
        with backend_probe.tcp_connect(arm.ip, arm.command_port, arm.connect_timeout_sec or 1.0):
            result["command_port_connected"] = True
    except Exception as exc:
        end = now_ns()
        samples.append(CommandSample(
            0,
            "read_only",
            False,
            start,
            end,
            (end - start) / 1000.0,
            error_name=type(exc).__name__,
            error_message=str(exc),
        ))
        return samples, result

    q_actual, data = read_rbscript_state(arm)
    result["data_port_connected"] = data.get("error_name") not in {"ConnectionRefusedError", "TimeoutError"}
    result["read_state_available"] = bool(data.get("available"))
    result["q_actual_deg"] = q_actual
    result["data_error_name"] = data.get("error_name", "")
    result["data_error_message"] = data.get("error_message", "")
    end = now_ns()
    samples.append(CommandSample(
        0,
        "read_only",
        bool(result["command_port_connected"]),
        start,
        end,
        (end - start) / 1000.0,
        response=str(data.get("response", "")),
        response_lines=[str(data["response"])] if data.get("response") else [],
        error_name=str(data.get("error_name", "")),
        error_message=str(data.get("error_message", "")),
    ))
    return samples, result


def servo_j_noop(config: ParsedConfig, args: argparse.Namespace) -> tuple[list[CommandSample], dict[str, Any]]:
    arm = selected_arm(config, args.arm)
    rate = args.command_rate_hz or float(config.servo.get("rate_hz", 0) or 0)
    if not math.isfinite(rate) or rate <= 0.0:
        raise AcceptanceError("command rate must be finite and positive")
    q_current, q_source, q_state = acquire_q_current(args, arm)
    q_target = list(q_current)
    if max_abs_delta(q_target, q_current) > args.q_current_tolerance_deg:
        raise AcceptanceError("servo_j_noop target must equal current q within tolerance")
    command = format_move_servo_j(q_target, arm)
    wait_ack = not arm.disable_waiting_ack
    client = PersistentCommandClient(
        arm.ip,
        arm.command_port,
        arm.connect_timeout_sec or 1.0,
        arm.command_timeout_sec or 0.2,
    )
    samples: list[CommandSample] = []
    deadline = time.monotonic() + args.duration_sec
    period = 1.0 / rate
    next_time = time.monotonic()
    index = 0
    try:
        while time.monotonic() < deadline:
            sample = client.send(command, wait_ack=wait_ack)
            sample.index = index
            samples.append(sample)
            index += 1
            next_time += period
            sleep_sec = next_time - time.monotonic()
            if sleep_sec > 0.0:
                time.sleep(sleep_sec)
    finally:
        client.close()
    return samples, {
        "q_source": q_source,
        "q_current_deg": q_current,
        "q_state": q_state,
        "q_target_max_abs_delta_deg": max_abs_delta(q_target, q_current),
        "command_rate_hz": rate,
    }


def loop_intervals_ms(samples: list[CommandSample]) -> list[float]:
    return [
        (b.start_ns - a.start_ns) / 1e6
        for a, b in zip(samples, samples[1:])
        if b.start_ns >= a.start_ns
    ]


def q_drift_if_available(config: ParsedConfig, args: argparse.Namespace, q_current: list[float] | None) -> float | None:
    if q_current is None:
        return None
    q_after, _ = read_rbscript_state(selected_arm(config, args.arm))
    if q_after is None:
        return None
    return max_abs_delta(q_after, q_current)


def summarize(
    args: argparse.Namespace,
    config: ParsedConfig,
    preflight_result: dict[str, Any],
    samples: list[CommandSample],
    run_result: dict[str, Any],
    artifact_dir: Path,
) -> dict[str, Any]:
    successes = [sample for sample in samples if sample.success]
    failures = [sample for sample in samples if not sample.success]
    send_durations = [sample.duration_us for sample in samples if sample.mode == "servo_j_noop"]
    write_durations = [
        sample.command_write_duration_us for sample in samples if sample.command_write_duration_us is not None
    ]
    ack_waits = [sample.ack_wait_duration_us for sample in samples if sample.ack_wait_duration_us is not None]
    intervals = loop_intervals_ms(samples)
    ack_policies: Counter[str] = Counter(sample.ack_policy for sample in samples if sample.ack_policy)
    semantics: Counter[str] = Counter(
        sample.send_acceptance_semantics for sample in samples if sample.send_acceptance_semantics
    )
    response_error_names: Counter[str] = Counter()
    m_counts: Counter[str] = Counter()
    for sample in samples:
        if sample.error_name:
            response_error_names[sample.error_name] += 1
            count_m_codes(sample.error_name, m_counts)
        for name in sample.response_error_names:
            response_error_names[name] += 1
            count_m_codes(name, m_counts)
        count_m_codes(sample.response, m_counts)
        for line in sample.response_lines + sample.extra_response_lines:
            count_m_codes(line, m_counts)

    elapsed_sec = (samples[-1].end_ns - samples[0].start_ns) / 1e9 if len(samples) >= 2 else 0.0
    q_current = run_result.get("q_current_deg")
    q_actual_drift = q_drift_if_available(config, args, q_current) if args.mode == "servo_j_noop" else None
    command_timeout_count = sum(1 for sample in samples if "timeout" in sample.error_name.lower())
    controller_rejected_count = sum(
        1
        for sample in samples
        if sample.error_name in {"controller_error", "rbscript_tcp_controller_rejected"}
        or "controller_error" in sample.response_error_names
    )
    return {
        "result": "completed" if samples else "error",
        "result_reason": "samples captured" if samples else "no samples captured",
        "backend": "rbscript_tcp",
        "artifact_dir": str(artifact_dir.resolve()),
        "config": str(config.path),
        "profile": preflight_result["profile"],
        "mode": args.mode,
        "arm": args.arm,
        "duration_sec": args.duration_sec,
        "requested_rate_hz": args.command_rate_hz or float(config.servo.get("rate_hz", 0) or 0),
        "achieved_rate_hz": (len(samples) / elapsed_sec) if elapsed_sec > 0.0 else None,
        "sample_count": len(samples),
        "send_count": len(samples) if args.mode == "servo_j_noop" else 0,
        "send_success_count": len(successes) if args.mode == "servo_j_noop" else 0,
        "send_failure_count": len(failures) if args.mode == "servo_j_noop" else 0,
        "success_rate": len(successes) / len(samples) if samples else 0.0,
        "ack_policy_distribution": dict(ack_policies),
        "ack_observed_count": sum(1 for sample in samples if sample.ack_observed),
        "controller_acceptance_observed_count": sum(
            1 for sample in samples if sample.controller_acceptance_observed
        ),
        "send_acceptance_semantics_distribution": dict(semantics),
        "send_duration_us": metric_block(send_durations),
        "command_write_duration_us": metric_block(write_durations),
        "ack_wait_duration_us": metric_block(ack_waits),
        "command_timeout_count": command_timeout_count,
        "controller_rejected_count": controller_rejected_count,
        "response_error_names": dict(response_error_names),
        "m_code_counts": {code: int(m_counts.get(code, 0)) for code in M_CODES},
        "loop_interval_ms": metric_block(intervals),
        "persistent_socket": True,
        "reconnect_count": sum(sample.reconnect_count for sample in samples),
        "q_source": run_result.get("q_source"),
        "q_current_deg": q_current,
        "q_target_max_abs_delta_deg": run_result.get("q_target_max_abs_delta_deg"),
        "q_actual_drift": q_actual_drift,
        "read_only": run_result if args.mode == "read_only" else None,
        "safety_preflight": preflight_result,
    }


def sample_row(sample: CommandSample) -> dict[str, Any]:
    return {
        "index": sample.index,
        "mode": sample.mode,
        "success": sample.success,
        "duration_us": sample.duration_us,
        "start_ns": sample.start_ns,
        "end_ns": sample.end_ns,
        "command_write_duration_us": sample.command_write_duration_us,
        "ack_wait_duration_us": sample.ack_wait_duration_us,
        "ack_policy": sample.ack_policy,
        "ack_observed": sample.ack_observed,
        "controller_acceptance_observed": sample.controller_acceptance_observed,
        "send_acceptance_semantics": sample.send_acceptance_semantics,
        "error_name": sample.error_name,
        "error_message": sample.error_message,
        "reconnect_count": sample.reconnect_count,
    }


def write_summary_csv(path: Path, summary: dict[str, Any]) -> None:
    flat = {
        "result": summary.get("result"),
        "backend": summary.get("backend"),
        "profile": summary.get("profile"),
        "mode": summary.get("mode"),
        "arm": summary.get("arm"),
        "requested_rate_hz": summary.get("requested_rate_hz"),
        "achieved_rate_hz": summary.get("achieved_rate_hz"),
        "send_count": summary.get("send_count"),
        "send_success_count": summary.get("send_success_count"),
        "send_failure_count": summary.get("send_failure_count"),
        "ack_observed_count": summary.get("ack_observed_count"),
        "controller_acceptance_observed_count": summary.get("controller_acceptance_observed_count"),
        "send_duration_p95_us": (summary.get("send_duration_us") or {}).get("p95"),
        "ack_wait_p95_us": (summary.get("ack_wait_duration_us") or {}).get("p95"),
        "command_timeout_count": summary.get("command_timeout_count"),
        "controller_rejected_count": summary.get("controller_rejected_count"),
        "persistent_socket": summary.get("persistent_socket"),
        "reconnect_count": summary.get("reconnect_count"),
        "q_source": summary.get("q_source"),
        "q_actual_drift": summary.get("q_actual_drift"),
    }
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(flat.keys()))
        writer.writeheader()
        writer.writerow(flat)


def write_artifacts(
    args: argparse.Namespace,
    config: ParsedConfig,
    preflight_result: dict[str, Any],
    samples: list[CommandSample],
    run_result: dict[str, Any],
) -> dict[str, Any]:
    artifact_dir = args.artifact_dir.resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    summary = summarize(args, config, preflight_result, samples, run_result, artifact_dir)
    (artifact_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    write_summary_csv(artifact_dir / "summary.csv", summary)
    with (artifact_dir / "samples.csv").open("w", newline="", encoding="utf-8") as handle:
        fieldnames = list(sample_row(samples[0]).keys()) if samples else list(sample_row(CommandSample(0, "", False, 0, 0, 0.0)).keys())
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for sample in samples:
            writer.writerow(sample_row(sample))
    with (artifact_dir / "command_packets.jsonl").open("w", encoding="utf-8") as handle:
        for sample in samples:
            if sample.command_text:
                handle.write(json.dumps({
                    "index": sample.index,
                    "command": sample.command_text.strip(),
                    "ack_policy": sample.ack_policy,
                }, sort_keys=True) + "\n")
    with (artifact_dir / "responses.jsonl").open("w", encoding="utf-8") as handle:
        for sample in samples:
            if sample.response or sample.response_lines or sample.extra_response_lines or sample.error_name:
                handle.write(json.dumps({
                    "index": sample.index,
                    "success": sample.success,
                    "response": sample.response,
                    "response_lines": sample.response_lines,
                    "extra_response_lines": sample.extra_response_lines,
                    "error_name": sample.error_name,
                    "error_message": sample.error_message,
                    "response_error_names": sample.response_error_names,
                }, sort_keys=True) + "\n")
    shutil.copy2(config.path, artifact_dir / "raw_config.yaml")
    (artifact_dir / "safety_preflight.json").write_text(
        json.dumps(preflight_result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    if not args.skip_plots:
        write_plots(artifact_dir, samples)
    return summary


def write_plots(artifact_dir: Path, samples: list[CommandSample]) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        (artifact_dir / "plot_skip_reason.txt").write_text(f"matplotlib unavailable: {exc}\n", encoding="utf-8")
        return
    ack_waits = [sample.ack_wait_duration_us for sample in samples if sample.ack_wait_duration_us is not None]
    if ack_waits:
        plt.figure(figsize=(8, 4))
        plt.hist(ack_waits, bins=min(40, max(5, len(ack_waits) // 2)))
        plt.xlabel("ack_wait_duration_us")
        plt.ylabel("count")
        plt.tight_layout()
        plt.savefig(artifact_dir / "timing_ack_duration.png")
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


def run_acceptance(args: argparse.Namespace) -> int:
    config = load_config(args.config)
    preflight_result = preflight(args, config)
    artifact_dir = args.artifact_dir.resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    print(json.dumps({"safety_preflight": preflight_result}, indent=2, sort_keys=True), file=sys.stderr)
    if args.preflight_only:
        summary = {
            "result": "completed",
            "result_reason": "preflight only",
            "backend": "rbscript_tcp",
            "artifact_dir": str(artifact_dir),
            "safety_preflight": preflight_result,
        }
        (artifact_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        write_summary_csv(artifact_dir / "summary.csv", summary)
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 0
    if args.mode == "read_only":
        samples, run_result = read_only_probe(config, args)
    elif args.mode == "servo_j_noop":
        samples, run_result = servo_j_noop(config, args)
    else:
        raise AcceptanceError("tiny_joint_motion is not implemented in RBSCRIPT-SERVO-NOOP-01")
    summary = write_artifacts(args, config, preflight_result, samples, run_result)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["result"] == "completed" else 1


def self_test_config(path: Path, *, operation_mode: str = "simulation", disable_ack: bool = False) -> None:
    path.write_text(
        f"""schema: robotics_lab.rb_servo_server.v1
left_robot:
  backend_type: rbscript_tcp
  run_mode: real
  ip: "127.0.0.1"
  operation_mode: {operation_mode}
  command_port: 5000
  data_port: 5001
  command_timeout_sec: 0.1
  read_timeout_sec: 0.1
  connect_timeout_sec: 0.1
  disable_waiting_ack: {'true' if disable_ack else 'false'}
  script_t1_sec: 0.005
  script_t2_sec: 0.05
  script_gain: 1.0
  script_alpha: 0.5
right_robot:
  backend_type: rbscript_tcp
  run_mode: real
  ip: "127.0.0.1"
  operation_mode: {operation_mode}
  command_port: 5000
  data_port: 5001
  command_timeout_sec: 0.1
  read_timeout_sec: 0.1
  connect_timeout_sec: 0.1
  disable_waiting_ack: {'true' if disable_ack else 'false'}
  script_t1_sec: 0.005
  script_t2_sec: 0.05
  script_gain: 1.0
  script_alpha: 0.5
servo:
  rate_hz: 200
  send_servo_commands: true
  servo_t1_rate_match_tolerance_ratio: 0.2
network:
  command_bind: "udp://127.0.0.1:50042"
  state_pub_endpoint: "udp://127.0.0.1:50142"
""",
        encoding="utf-8",
    )


class FakeCommandServer:
    def __init__(self, responses: list[str]) -> None:
        self.responses = responses
        self.received: list[str] = []
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.listen(5)
        self.port = self.sock.getsockname()[1]
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self) -> None:
        try:
            conn, _ = self.sock.accept()
        except OSError:
            return
        with conn:
            while not self.stop.is_set():
                chunks = []
                while not self.stop.is_set():
                    data = conn.recv(1)
                    if not data:
                        return
                    chunks.append(data)
                    if data == b"\n":
                        break
                text = b"".join(chunks).decode("utf-8", errors="replace")
                self.received.append(text)
                response = self.responses[min(len(self.received) - 1, len(self.responses) - 1)]
                try:
                    conn.sendall((response + "\n").encode("utf-8"))
                except OSError:
                    return

    def close(self) -> None:
        self.stop.set()
        try:
            self.sock.close()
        except OSError:
            pass
        self.thread.join(timeout=1.0)


def expect_error(fn: Any, contains: str = "") -> None:
    try:
        fn()
    except AcceptanceError as exc:
        if contains and contains not in str(exc):
            raise AssertionError(f"expected error containing {contains!r}, got {exc!s}") from exc
        return
    raise AssertionError("expected AcceptanceError")


def run_self_test() -> int:
    old_env = {key: os.environ.get(key) for key in ENV_KEYS}
    try:
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            cfg_path = tmp_path / "config.yaml"
            self_test_config(cfg_path)
            config = load_config(cfg_path)
            base = argparse.Namespace(
                config=cfg_path,
                arm="left",
                mode="servo_j_noop",
                profile="200hz_ack",
                duration_sec=0.02,
                command_rate_hz=100.0,
                artifact_dir=tmp_path / "artifacts",
                q_current_deg="0,1,2,3,4,5",
                q_current_from_rbpodo=False,
                q_current_tolerance_deg=1e-6,
                allow_motion=False,
                allow_ack_disabled=False,
                preflight_only=False,
                skip_plots=True,
                i_understand_this_connects_to_real_controller=True,
            )
            expect_error(lambda: preflight(base, config), "--allow-motion")
            motion = argparse.Namespace(**vars(base))
            motion.allow_motion = True
            os.environ["RB_ALLOW_REAL_ROBOT"] = "1"
            os.environ["RB_ALLOW_REAL_MOTION"] = "1"
            os.environ["RB_ALLOW_RBSCRIPT_TCP"] = "1"
            os.environ.pop("RB_ALLOW_RBSCRIPT_TCP_MOTION", None)
            os.environ.pop("RB_ALLOW_REAL_CARTESIAN", None)
            expect_error(lambda: preflight(motion, config), "RB_ALLOW_RBSCRIPT_TCP_MOTION")
            os.environ["RB_ALLOW_RBSCRIPT_TCP_MOTION"] = "1"
            real_cfg_path = tmp_path / "real_config.yaml"
            self_test_config(real_cfg_path, operation_mode="real")
            expect_error(lambda: preflight(motion, load_config(real_cfg_path)), "operation_mode=simulation")
            ack_off_path = tmp_path / "ack_off.yaml"
            self_test_config(ack_off_path, disable_ack=True)
            ack_off_cfg = load_config(ack_off_path)
            ack_off_args = argparse.Namespace(**vars(motion))
            ack_off_args.config = ack_off_path
            ack_off_args.profile = "200hz_no_ack"
            expect_error(lambda: preflight(ack_off_args, ack_off_cfg), "--allow-ack-disabled")
            assert parse_q_current("0,1,2,3,4,5") == [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
            assert backend_probe.command_text_looks_motion_capable("move_servo_j(jnt[0,0,0,0,0,0],0.005,0.05,1,0.5)")

            server = FakeCommandServer(["The command was executed", "The command was executed"])
            try:
                config.left.command_port = server.port
                samples, result = servo_j_noop(config, motion)
            finally:
                server.close()
            assert samples
            assert "move_servo_j(jnt[0,1,2,3,4,5],0.005,0.05,1,0.5)" in server.received[0]
            assert samples[0].controller_acceptance_observed is True
            assert result["q_source"] == "cli"

            ack_off_args.allow_ack_disabled = True
            ack_off_cfg.left.command_port = server.port
            sample = CommandSample(
                0,
                "servo_j_noop",
                True,
                0,
                1000,
                1.0,
                ack_policy="disabled",
                ack_observed=False,
                controller_acceptance_observed=False,
                send_acceptance_semantics="socket_send_only",
            )
            summary = summarize(
                ack_off_args,
                ack_off_cfg,
                preflight(ack_off_args, ack_off_cfg),
                [sample],
                {"q_source": "cli", "q_current_deg": [0, 1, 2, 3, 4, 5]},
                tmp_path,
            )
            assert summary["ack_observed_count"] == 0
            assert summary["controller_acceptance_observed_count"] == 0
            assert summary["send_acceptance_semantics_distribution"]["socket_send_only"] == 1
    finally:
        for key, value in old_env.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
    print("rbscript_servo_acceptance self-test passed")
    return 0


def main() -> int:
    args = parse_args()
    if args.self_test:
        return run_self_test()
    missing = [name for name in ("config", "arm", "artifact_dir") if getattr(args, name) is None]
    if missing:
        print(
            "rbscript_servo_acceptance: FAIL: missing required arguments: "
            + ", ".join("--" + name.replace("_", "-") for name in missing),
            file=sys.stderr,
        )
        return 2
    try:
        return run_acceptance(args)
    except AcceptanceError as exc:
        print(f"rbscript_servo_acceptance: FAIL: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
