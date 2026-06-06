#!/usr/bin/env python3
"""Supervised rbpodo Servo J ACK/rate acceptance harness.

The default mode is read-only. Real motion modes are intentionally gated by
flags, environment variables, and config checks.
"""

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
import tempfile
import time
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any


REAL_ROBOT_IPS = {"172.28.60.200", "172.28.60.201"}
ENV_KEYS = (
    "RB_ALLOW_REAL_ROBOT",
    "RB_ALLOW_REAL_MOTION",
    "RB_ALLOW_RBPODO_ACK_DISABLED_MOTION",
    "RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION",
    "RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM",
    "RB_RBPODO_PGMODE_SIMULATION_CONFIRMED",
    "RB_ALLOW_REAL_CARTESIAN",
)
PROFILES = {
    "500hz_ack": {"rate_hz": 500, "servo_t1_sec": 0.002, "disable_waiting_ack": False},
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
    command_timeout_sec: float | None = None
    servo_t1_sec: float | None = None
    servo_t2_sec: float | None = None
    servo_gain: float | None = None
    servo_alpha: float | None = None
    disable_waiting_ack: bool = False


@dataclass
class ParsedConfig:
    path: Path
    left: ArmConfig
    right: ArmConfig
    servo: dict[str, Any]
    network: dict[str, Any]
    logging: dict[str, Any]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run staged rbpodo Servo J acceptance for the supported 500Hz ACK-on "
            "profile. Default mode is read-only and does not command motion."
        )
    )
    parser.add_argument("--config", type=Path)
    parser.add_argument("--arm", choices=("left", "right"))
    parser.add_argument(
        "--mode",
        choices=("read_only", "hold_no_motion", "servo_j_noop", "tiny_joint_motion"),
        default="read_only",
    )
    parser.add_argument("--profile", choices=tuple(PROFILES), help="Expected rbpodo rate/ACK profile")
    parser.add_argument("--duration-sec", type=float, default=10.0)
    parser.add_argument("--command-rate-hz", type=float)
    parser.add_argument("--artifact-dir", type=Path)
    parser.add_argument("--server", default="rb_servo_server/build/hardware_free_gate/rb_servo_server", type=Path)
    parser.add_argument("--root", default=".", type=Path)
    parser.add_argument("--startup-timeout-sec", type=float, default=10.0)
    parser.add_argument("--state-timeout-sec", type=float, default=1.0)
    parser.add_argument("--max-state-age-us", type=float, default=250_000.0)
    parser.add_argument("--max-delta-deg", type=float, default=0.2)
    parser.add_argument("--joint-index", type=int, default=0)
    parser.add_argument("--expected-left-ip", default="172.28.60.200")
    parser.add_argument("--expected-right-ip", default="172.28.60.201")
    parser.add_argument("--allow-motion", action="store_true")
    parser.add_argument("--allow-ack-disabled", action="store_true")
    parser.add_argument(
        "--set-pgmode-simulation",
        action="store_true",
        help="Send pgmode simulation to both configured controllers during preflight.",
    )
    parser.add_argument(
        "--verify-pgmode-simulation",
        action="store_true",
        help="Verify CobotData.real_vs_simulation_mode without sending pgmode.",
    )
    parser.add_argument("--pgmode-timeout-sec", type=float, default=1.0)
    parser.add_argument("--pgmode-command-port", type=int, default=5000)
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
            number = float(value)
            return number
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


def as_float(value: Any, default: float | None = None) -> float | None:
    if value is None:
        return default
    if isinstance(value, bool):
        return default
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    return out if math.isfinite(out) else default


def as_bool(value: Any, default: bool = False) -> bool:
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
        command_timeout_sec=as_float(section.get("command_timeout_sec")),
        servo_t1_sec=as_float(section.get("servo_t1_sec", section.get("servo_time_sec"))),
        servo_t2_sec=as_float(section.get("servo_t2_sec", section.get("servo_lookahead_sec"))),
        servo_gain=as_float(section.get("servo_gain")),
        servo_alpha=as_float(section.get("servo_alpha", section.get("servo_acc"))),
        disable_waiting_ack=as_bool(section.get("disable_waiting_ack"), False),
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
        logging=sections.get("logging", {}),
    )


def env_enabled(name: str) -> bool:
    return os.environ.get(name) == "1"


def env_snapshot() -> dict[str, str | None]:
    return {key: os.environ.get(key) for key in ENV_KEYS}


def parse_udp_endpoint(value: str) -> tuple[str, int]:
    prefix = "udp://"
    if not value.startswith(prefix):
        raise AcceptanceError(f"endpoint must use udp://host:port, got {value}")
    host_port = value[len(prefix):]
    if ":" not in host_port:
        raise AcceptanceError(f"endpoint must use udp://host:port, got {value}")
    host, port_text = host_port.rsplit(":", 1)
    try:
        port = int(port_text, 10)
    except ValueError as exc:
        raise AcceptanceError(f"invalid endpoint port in {value}") from exc
    if not host or port < 1 or port > 65535:
        raise AcceptanceError(f"invalid UDP endpoint {value}")
    return host, port


def selected_arm(config: ParsedConfig, arm: str) -> ArmConfig:
    return config.left if arm == "left" else config.right


def ensure_pgmode_simulation(args: argparse.Namespace, config: ParsedConfig) -> dict[str, Any] | None:
    set_pgmode = bool(getattr(args, "set_pgmode_simulation", False))
    verify_pgmode = bool(getattr(args, "verify_pgmode_simulation", False))
    if set_pgmode and verify_pgmode:
        raise AcceptanceError("--set-pgmode-simulation and --verify-pgmode-simulation are mutually exclusive")
    if not set_pgmode and not verify_pgmode:
        return None
    try:
        from rainbow_pgmode import RainbowPgmodeError, ensure_controller_simulation_mode
    except Exception as exc:
        raise AcceptanceError("scripts/rainbow_pgmode.py helper is unavailable") from exc
    try:
        return ensure_controller_simulation_mode(
            [config.left.ip, config.right.ip],
            getattr(args, "pgmode_timeout_sec", 1.0),
            port=getattr(args, "pgmode_command_port", 5000),
            confirmation=args.i_understand_this_connects_to_real_controller,
            set_simulation=set_pgmode,
            verify_only=verify_pgmode,
        )
    except RainbowPgmodeError as exc:
        raise AcceptanceError(
            f"controller not confirmed in pgmode simulation; refusing controller-simulation benchmark: {exc}"
        ) from exc


def require_controller_simulation_config(config: ParsedConfig) -> None:
    for label, arm_cfg in (("left", config.left), ("right", config.right)):
        if arm_cfg.operation_mode not in {"simulation", "sim"}:
            actual = arm_cfg.operation_mode or "<missing>"
            raise AcceptanceError(
                f"config operation_mode is {actual} for {label}_robot; refusing controller-simulation benchmark"
            )


def validate_profile(config: ParsedConfig, args: argparse.Namespace) -> str:
    rate = int(config.servo.get("rate_hz", 0) or 0)
    tolerance_ratio = as_float(config.servo.get("servo_t1_rate_match_tolerance_ratio"), 0.2) or 0.2
    arm = selected_arm(config, args.arm)
    if rate <= 0:
        raise AcceptanceError("servo.rate_hz must be positive")
    if arm.servo_t1_sec is None:
        raise AcceptanceError(f"{args.arm}_robot.servo_t1_sec is required")
    dt = 1.0 / float(rate)
    if abs(arm.servo_t1_sec - dt) > tolerance_ratio * dt:
        raise AcceptanceError(
            f"profile mismatch: servo_t1_sec={arm.servo_t1_sec} does not match rate period {dt:.6f}"
        )

    if args.profile:
        expected = PROFILES[args.profile]
        if rate != expected["rate_hz"]:
            raise AcceptanceError(f"profile {args.profile} expects servo.rate_hz={expected['rate_hz']}, got {rate}")
        if abs(arm.servo_t1_sec - expected["servo_t1_sec"]) > tolerance_ratio * dt:
            raise AcceptanceError(f"profile {args.profile} expects servo_t1_sec={expected['servo_t1_sec']}")
        if arm.disable_waiting_ack != expected["disable_waiting_ack"]:
            raise AcceptanceError(f"profile {args.profile} ACK setting does not match disable_waiting_ack")
        return args.profile

    for name, expected in PROFILES.items():
        if (
            rate == expected["rate_hz"]
            and abs(arm.servo_t1_sec - expected["servo_t1_sec"]) <= tolerance_ratio * dt
            and arm.disable_waiting_ack == expected["disable_waiting_ack"]
        ):
            return name
    raise AcceptanceError("config does not match a supported rbpodo acceptance profile")


def preflight(args: argparse.Namespace, config: ParsedConfig) -> dict[str, Any]:
    if not math.isfinite(args.duration_sec) or args.duration_sec <= 0.0:
        raise AcceptanceError("--duration-sec must be finite and positive")
    if not math.isfinite(args.max_delta_deg) or args.max_delta_deg <= 0.0 or args.max_delta_deg > 0.2:
        raise AcceptanceError("--max-delta-deg must be finite, positive, and <= 0.2")
    if args.joint_index < 0 or args.joint_index > 5:
        raise AcceptanceError("--joint-index must be in [0, 5]")

    for label, arm_cfg in (("left", config.left), ("right", config.right)):
        if arm_cfg.backend_type != "rbpodo":
            raise AcceptanceError(f"{label}_robot.backend_type must be rbpodo")
        if arm_cfg.run_mode != "real":
            raise AcceptanceError(f"{label}_robot.run_mode must be real for rbpodo acceptance")
        if not arm_cfg.ip:
            raise AcceptanceError(f"{label}_robot.ip is required")

    if config.left.ip != args.expected_left_ip:
        raise AcceptanceError(f"left_robot.ip {config.left.ip} does not match --expected-left-ip {args.expected_left_ip}")
    if config.right.ip != args.expected_right_ip:
        raise AcceptanceError(f"right_robot.ip {config.right.ip} does not match --expected-right-ip {args.expected_right_ip}")

    known_real_ips = {config.left.ip, config.right.ip} & REAL_ROBOT_IPS
    if known_real_ips and not args.i_understand_this_connects_to_real_controller:
        raise AcceptanceError("refusing known real controller IP without explicit confirmation flag")
    if not env_enabled("RB_ALLOW_REAL_ROBOT"):
        raise AcceptanceError("rbpodo real acceptance requires RB_ALLOW_REAL_ROBOT=1")
    if env_enabled("RB_ALLOW_REAL_CARTESIAN"):
        raise AcceptanceError("RB_ALLOW_REAL_CARTESIAN must not be set for rbpodo Servo J acceptance")
    pgmode_timeout_sec = getattr(args, "pgmode_timeout_sec", 1.0)
    pgmode_command_port = getattr(args, "pgmode_command_port", 5000)
    if not math.isfinite(pgmode_timeout_sec) or pgmode_timeout_sec <= 0.0:
        raise AcceptanceError("--pgmode-timeout-sec must be finite and positive")
    if pgmode_command_port < 1 or pgmode_command_port > 65535:
        raise AcceptanceError("--pgmode-command-port must be in [1, 65535]")

    send_servo_commands = as_bool(config.servo.get("send_servo_commands"), True)
    selected = selected_arm(config, args.arm)
    profile = validate_profile(config, args)
    if selected.disable_waiting_ack and not args.allow_ack_disabled:
        raise AcceptanceError("ACK-off profile requires --allow-ack-disabled")
    if args.mode == "read_only" and send_servo_commands:
        raise AcceptanceError("read_only mode requires servo.send_servo_commands=false")
    if args.mode in MOTION_MODES:
        if not args.allow_motion:
            raise AcceptanceError(f"{args.mode} requires --allow-motion")
        if not send_servo_commands:
            raise AcceptanceError(f"{args.mode} requires servo.send_servo_commands=true")
        if not env_enabled("RB_ALLOW_REAL_MOTION"):
            raise AcceptanceError(f"{args.mode} requires RB_ALLOW_REAL_MOTION=1")
        if not env_enabled("RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION"):
            raise AcceptanceError(f"{args.mode} requires RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1")
        if not as_bool(config.servo.get("allow_controller_simulation_motion"), False):
            raise AcceptanceError(
                f"{args.mode} requires servo.allow_controller_simulation_motion=true"
            )
        if as_bool(config.servo.get("allow_controller_simulation_diagnostics_suspect"), False) and not env_enabled(
            "RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM"
        ):
            raise AcceptanceError(
                "diagnostics-suspect controller-simulation override requires "
                "RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1"
            )
        if selected.disable_waiting_ack and not env_enabled("RB_ALLOW_RBPODO_ACK_DISABLED_MOTION"):
            raise AcceptanceError("ACK-off motion requires RB_ALLOW_RBPODO_ACK_DISABLED_MOTION=1")
        if not args.i_understand_this_connects_to_real_controller:
            raise AcceptanceError(f"{args.mode} requires explicit real-controller confirmation")
        require_controller_simulation_config(config)
        if not getattr(args, "set_pgmode_simulation", False) and not getattr(args, "verify_pgmode_simulation", False):
            raise AcceptanceError(
                "controller not confirmed in pgmode simulation; refusing controller-simulation benchmark"
            )
    pgmode_preflight = ensure_pgmode_simulation(args, config)
    if args.mode == "tiny_joint_motion":
        raise AcceptanceError("tiny_joint_motion is reserved for a future explicit motion runbook")

    return {
        "passed": True,
        "profile": profile,
        "mode": args.mode,
        "arm": args.arm,
        "config": str(config.path),
        "send_servo_commands": send_servo_commands,
        "selected_ip": selected.ip,
        "selected_operation_mode": selected.operation_mode,
        "servo_rate_hz": int(config.servo.get("rate_hz", 0) or 0),
        "servo_t1_sec": selected.servo_t1_sec,
        "disable_waiting_ack": selected.disable_waiting_ack,
        "ack_semantics": "socket_send_only" if selected.disable_waiting_ack else "controller_ack_observed",
        "env": env_snapshot(),
        "real_robot_ips_checked": sorted(REAL_ROBOT_IPS),
        "confirmation_flag": args.i_understand_this_connects_to_real_controller,
        "pgmode_simulation_preflight": pgmode_preflight,
        "pgmode_simulation_confirmed": (
            pgmode_preflight is not None and pgmode_preflight.get("overall_result") == "ok"
        ),
        "server_env_overrides": {
            "RB_RBPODO_PGMODE_SIMULATION_CONFIRMED": "1"
        } if (
            args.mode in MOTION_MODES and
            pgmode_preflight is not None and
            pgmode_preflight.get("overall_result") == "ok"
        ) else {},
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


def bind_state_socket(endpoint: str) -> socket.socket:
    host, port = parse_udp_endpoint(endpoint)
    bind_host = "127.0.0.1" if host in {"localhost", "127.0.0.1"} else host
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # sock.bind((bind_host, port))
    try:
        sock.bind((bind_host, port))
    except OSError as exc:
        if exc.errno == 98:
            raise AcceptanceError(
                f"state endpoint udp://{bind_host}:{port} is already in use. "
                "Stop existing rb_servo_server/GUI/policy_runner/acceptance processes "
                "or use a config with a unique network.state_pub_endpoint."
            ) from exc
        raise
    sock.settimeout(0.1)
    return sock


def recv_state(sock: socket.socket, deadline: float) -> dict[str, Any] | None:
    while time.monotonic() < deadline:
        try:
            payload, _ = sock.recvfrom(262_144)
        except socket.timeout:
            continue
        try:
            return json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            continue
    return None


def tail_lines(path: Path, line_count: int = 120) -> list[str]:
    if not path.is_file():
        return []
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return []
    return lines[-line_count:]


def likely_startup_causes(log_text: str) -> list[str]:
    lower_log = log_text.lower()
    causes: list[str] = []
    if "invalid robot startup state" in lower_log:
        causes.append(
            "invalid robot startup state; run scripts/rbpodo_state_dump.py to inspect q_actual, q_ref, status flags, range violations, and wrap diagnostics"
        )
    if "rb_servo_enable_rbpodo=off" in lower_log or ("rbpodo" in lower_log and "disabled" in lower_log):
        causes.append(
            "rbpodo backend appears disabled; use --server rb_servo_server/build/rbpodo_real_gate/rb_servo_server"
        )
    if "realtime" in lower_log and (
        "fail" in lower_log or "permission" in lower_log or "operation not permitted" in lower_log or "sched" in lower_log
    ):
        causes.append("realtime setup failure; run with appropriate privileges or setcap for realtime scheduling")
    if "failed to connect" in lower_log or "connect failed" in lower_log or "transportconnectfailed" in lower_log:
        causes.append("controller connection failed; check left/right IPs and TCP command/data ports 5000/5001")
    if "5001" in lower_log or "data port" in lower_log or "request_data" in lower_log or "cobotdata" in lower_log:
        causes.append("rbpodo data channel may be unavailable; check controller TCP data port 5001")

    defaults = [
        "server binary may not include rbpodo support",
        "realtime scheduling may lack permission",
        "robot startup state may be invalid before state publication",
        "controller data port 5001 may be unreachable or blocked",
        "network.state_pub_endpoint may not match the acceptance listener endpoint",
    ]
    for cause in defaults:
        if cause not in causes:
            causes.append(cause)
    return causes


def state_stream_timeout_message(
    proc: subprocess.Popen[str] | None,
    log_path: Path,
    endpoint: str,
) -> str:
    returncode: int | None = None
    if proc is not None:
        returncode = proc.poll()
    log_tail = tail_lines(log_path, 120)
    log_text = "\n".join(log_tail)
    causes = likely_startup_causes(log_text)
    parts = [
        "timed out waiting for rb_servo_server state stream",
        f"state_endpoint={endpoint}",
        f"server_returncode={returncode if returncode is not None else 'still running'}",
        "likely_causes:",
    ]
    parts.extend(f"- {cause}" for cause in causes)
    if log_tail:
        parts.append("last_120_lines_of_rb_servo_server.log:")
        parts.extend(log_tail)
    else:
        parts.append(f"rb_servo_server.log was empty or unavailable: {log_path}")
    return "\n".join(parts)


def wait_for_first_state(
    sock: socket.socket,
    timeout_sec: float,
    proc: subprocess.Popen[str] | None,
    log_path: Path,
    endpoint: str,
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        state = recv_state(sock, min(time.monotonic() + 0.2, deadline))
        if state is not None:
            return state
        if proc is not None and proc.poll() is not None:
            time.sleep(0.1)
            break
    raise AcceptanceError(state_stream_timeout_message(proc, log_path, endpoint))


def drain_states(sock: socket.socket, duration_sec: float, artifact_path: Path) -> list[dict[str, Any]]:
    deadline = time.monotonic() + duration_sec
    states: list[dict[str, Any]] = []
    with artifact_path.open("w", encoding="utf-8") as handle:
        while time.monotonic() < deadline:
            state = recv_state(sock, min(time.monotonic() + 0.2, deadline))
            if state is None:
                continue
            states.append(state)
            handle.write(json.dumps(state, sort_keys=True) + "\n")
    return states


def command_endpoint(config: ParsedConfig) -> tuple[str, int]:
    return parse_udp_endpoint(str(config.network.get("command_bind", "udp://127.0.0.1:50010")))


def state_endpoint(config: ParsedConfig) -> str:
    return str(config.network.get("state_pub_endpoint", config.network.get("state_pub_bind", "udp://127.0.0.1:50110")))


def hold_packet(seq: int, session_id: str) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "seq": seq,
        "mode": "Hold",
        "host_time_ns": now_ns(),
        "timeout_sec": 0.2,
        "source_id": "rbpodo_servo_acceptance",
        "session_id": session_id,
        "left": {"mode": "Hold"},
        "right": {"mode": "Hold"},
    }


def joint_target_packet(seq: int, session_id: str, arm: str, q_target_deg: list[float]) -> dict[str, Any]:
    payload = {"q_target_deg": q_target_deg}
    return {
        "schema_version": 1,
        "seq": seq,
        "mode": "JointTarget",
        "host_time_ns": now_ns(),
        "timeout_sec": 0.2,
        "coupled_timeout": True,
        "source_id": "rbpodo_servo_acceptance",
        "session_id": session_id,
        "left": payload if arm == "left" else {"mode": "Hold"},
        "right": payload if arm == "right" else {"mode": "Hold"},
    }


def send_commands(
    config: ParsedConfig,
    args: argparse.Namespace,
    states: list[dict[str, Any]],
    artifact_path: Path,
    stop_time: float,
) -> int:
    if args.mode == "read_only":
        artifact_path.write_text("", encoding="utf-8")
        return 0

    host, port = command_endpoint(config)
    rate = args.command_rate_hz or float(config.servo.get("rate_hz", 0) or 0)
    if not math.isfinite(rate) or rate <= 0.0:
        raise AcceptanceError("command rate must be finite and positive")
    session_id = f"rbpodo-accept-{now_ns()}"
    seq = 1
    period = 1.0 / rate
    next_send = time.monotonic()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock, artifact_path.open("w", encoding="utf-8") as handle:
        q_noop: list[float] | None = None
        while time.monotonic() < stop_time:
            if args.mode == "hold_no_motion":
                packet = hold_packet(seq, session_id)
            elif args.mode == "servo_j_noop":
                if q_noop is None:
                    q_noop = latest_q_actual(states, args.arm)
                    if q_noop is None:
                        time.sleep(0.01)
                        continue
                packet = joint_target_packet(seq, session_id, args.arm, q_noop)
            else:
                raise AcceptanceError(f"mode {args.mode} is not implemented")
            payload = json.dumps(packet, sort_keys=True).encode("utf-8")
            sock.sendto(payload, (host, port))
            handle.write(json.dumps(packet, sort_keys=True) + "\n")
            seq += 1
            next_send += period
            sleep_sec = next_send - time.monotonic()
            if sleep_sec > 0.0:
                time.sleep(sleep_sec)
    return seq - 1


def latest_q_actual(states: list[dict[str, Any]], arm: str) -> list[float] | None:
    for state in reversed(states):
        arm_state = state.get(arm)
        if not isinstance(arm_state, dict):
            continue
        q = arm_state.get("q_actual_deg")
        if isinstance(q, list) and len(q) == 6 and all(numeric(value) is not None for value in q):
            return [float(value) for value in q]
    return None


def start_server(
    args: argparse.Namespace,
    log_path: Path,
    preflight_result: dict[str, Any],
) -> subprocess.Popen[str]:
    server = (args.root / args.server).resolve() if not args.server.is_absolute() else args.server
    if not server.is_file():
        raise AcceptanceError(f"server binary not found: {server}")
    command = [str(server), "--config", str(args.config.resolve())]
    log = log_path.open("w", encoding="utf-8")
    server_env = os.environ.copy()
    server_env.update(preflight_result.get("server_env_overrides") or {})
    return subprocess.Popen(
        command,
        cwd=str(args.root.resolve()),
        stdout=log,
        stderr=subprocess.STDOUT,
        text=True,
        env=server_env,
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


def arm_states(states: list[dict[str, Any]], arm: str) -> list[dict[str, Any]]:
    out = []
    for state in states:
        value = state.get(arm)
        if isinstance(value, dict):
            out.append(value)
    return out


def q_drift_from_start(arms: list[dict[str, Any]]) -> float | None:
    q0 = None
    max_drift = 0.0
    for arm in arms:
        q = arm.get("q_actual_deg")
        if not isinstance(q, list) or len(q) != 6 or not all(numeric(value) is not None for value in q):
            continue
        qf = [float(value) for value in q]
        if q0 is None:
            q0 = qf
            continue
        max_drift = max(max_drift, max(abs(a - b) for a, b in zip(qf, q0)))
    return max_drift if q0 is not None else None


def summarize(
    args: argparse.Namespace,
    config: ParsedConfig,
    preflight_result: dict[str, Any],
    states: list[dict[str, Any]],
    command_count: int,
    artifact_dir: Path,
    server_returncode: int | None,
) -> dict[str, Any]:
    arms = arm_states(states, args.arm)
    state_ages = [value for value in (numeric(arm.get("state_age_us")) for arm in arms) if value is not None]
    send_durations = []
    ack_waits = []
    ack_policies: Counter[str] = Counter()
    semantics: Counter[str] = Counter()
    error_codes: Counter[str] = Counter()
    command_timeout_count = 0
    controller_rejected_count = 0
    ack_observed_count = 0
    controller_acceptance_count = 0
    send_success_count = 0
    send_failure_count = 0
    send_deadline_missed_count = 0
    stale_state_count = 0
    m_counts: Counter[str] = Counter()
    for arm in arms:
        if numeric(arm.get("state_age_us")) is not None and float(arm["state_age_us"]) > args.max_state_age_us:
            stale_state_count += 1
        code = arm.get("error_code")
        if code not in {None, 0, "0"}:
            error_codes[str(code)] += 1
            count_m_codes(code, m_counts)
        last_send = arm.get("last_send")
        if isinstance(last_send, dict):
            for key in ("backend_error_kind", "backend_error_name", "error_name", "error_message", "message"):
                count_m_codes(last_send.get(key), m_counts)
            duration = numeric(last_send.get("duration_us"))
            if duration is not None:
                send_durations.append(duration)
            ack_wait = numeric(last_send.get("ack_wait_duration_us"))
            if ack_wait is not None:
                ack_waits.append(ack_wait)
            policy = last_send.get("ack_policy")
            if isinstance(policy, str):
                ack_policies[policy] += 1
            semantic = last_send.get("send_acceptance_semantics")
            if isinstance(semantic, str):
                semantics[semantic] += 1
            if last_send.get("ack_observed") is True:
                ack_observed_count += 1
            if last_send.get("controller_acceptance_observed") is True:
                controller_acceptance_count += 1
            if last_send.get("accepted") is True:
                send_success_count += 1
            elif last_send.get("accepted") is False:
                send_failure_count += 1
            kind = str(last_send.get("backend_error_kind", ""))
            if kind in {"CommandTimeout", "TransportTimeout"}:
                command_timeout_count += 1
            if kind == "ControllerRejected":
                controller_rejected_count += 1
        if arm.get("send_command_deadline_missed") is True:
            send_deadline_missed_count += 1

    loop_times = [numeric(state.get("loop_start_time_ns")) for state in states]
    loop_ns = [int(value) for value in loop_times if value is not None]
    loop_intervals_ms = [
        (b - a) / 1e6 for a, b in zip(loop_ns, loop_ns[1:]) if b >= a
    ]
    fault_latched = any(state.get("fault_latched") is True for state in states)
    valid_count = sum(1 for arm in arms if arm.get("has_valid_joint_state") is True)
    m_code_counts = {code_name: int(m_counts.get(code_name, 0)) for code_name in M_CODES}

    return {
        "result": "completed" if states and server_returncode in {None, 0, -signal.SIGINT} else "error",
        "result_reason": "state stream captured" if states else "no state samples captured",
        "artifact_dir": str(artifact_dir.resolve()),
        "config": str(config.path),
        "profile": preflight_result["profile"],
        "mode": args.mode,
        "arm": args.arm,
        "duration_sec": args.duration_sec,
        "state_sample_count": len(states),
        "state_valid_ratio": valid_count / len(arms) if arms else 0.0,
        "state_age_us": metric_block(state_ages),
        "send_count": command_count,
        "send_success_count": send_success_count,
        "send_failure_count": send_failure_count,
        "ack_policy_distribution": dict(ack_policies),
        "ack_observed_count": ack_observed_count,
        "controller_acceptance_observed_count": controller_acceptance_count,
        "send_acceptance_semantics_distribution": dict(semantics),
        "send_duration_us": metric_block(send_durations),
        "ack_wait_duration_us": metric_block(ack_waits),
        "command_timeout_count": command_timeout_count,
        "controller_rejected_count": controller_rejected_count,
        "error_code_distribution": dict(error_codes),
        "m_code_counts": m_code_counts,
        "q_actual_drift_from_start_deg": q_drift_from_start(arms),
        "q_ref_update_rate_hz": None,
        "q_actual_tracking_error_deg": None,
        "loop_interval_ms": metric_block(loop_intervals_ms),
        "fault_latched": fault_latched,
        "send_deadline_missed_count": send_deadline_missed_count,
        "stale_state_count": stale_state_count,
        "observed_backend": states[-1].get("observed_backend") if states else None,
        "safety_preflight": preflight_result,
        "server_returncode": server_returncode,
    }


def write_summary_csv(path: Path, summary: dict[str, Any]) -> None:
    flat = {
        "result": summary.get("result"),
        "profile": summary.get("profile"),
        "mode": summary.get("mode"),
        "arm": summary.get("arm"),
        "state_valid_ratio": summary.get("state_valid_ratio"),
        "state_age_p95_us": (summary.get("state_age_us") or {}).get("p95"),
        "state_age_max_us": (summary.get("state_age_us") or {}).get("max"),
        "send_count": summary.get("send_count"),
        "send_success_count": summary.get("send_success_count"),
        "send_failure_count": summary.get("send_failure_count"),
        "ack_observed_count": summary.get("ack_observed_count"),
        "controller_acceptance_observed_count": summary.get("controller_acceptance_observed_count"),
        "send_duration_p95_us": (summary.get("send_duration_us") or {}).get("p95"),
        "ack_wait_p95_us": (summary.get("ack_wait_duration_us") or {}).get("p95"),
        "fault_latched": summary.get("fault_latched"),
        "stale_state_count": summary.get("stale_state_count"),
    }
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(flat.keys()))
        writer.writeheader()
        writer.writerow(flat)


def write_plots(artifact_dir: Path, states: list[dict[str, Any]], arm: str) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        (artifact_dir / "plot_skip_reason.txt").write_text(f"matplotlib unavailable: {exc}\n", encoding="utf-8")
        return
    arms = arm_states(states, arm)
    times = list(range(len(arms)))
    state_ages = [numeric(sample.get("state_age_us")) or 0.0 for sample in arms]
    if state_ages:
        plt.figure(figsize=(8, 4))
        plt.plot(times, state_ages)
        plt.xlabel("sample")
        plt.ylabel("state_age_us")
        plt.tight_layout()
        plt.savefig(artifact_dir / "timing_state_age.png")
        plt.close()

    send_durations = []
    ack_policies = Counter()
    for sample in arms:
        last_send = sample.get("last_send")
        if isinstance(last_send, dict):
            duration = numeric(last_send.get("duration_us"))
            if duration is not None:
                send_durations.append(duration)
            policy = last_send.get("ack_policy")
            if isinstance(policy, str):
                ack_policies[policy] += 1
    if send_durations:
        plt.figure(figsize=(8, 4))
        plt.hist(send_durations, bins=min(40, max(5, len(send_durations) // 2)))
        plt.xlabel("send_duration_us")
        plt.ylabel("count")
        plt.tight_layout()
        plt.savefig(artifact_dir / "timing_send_duration.png")
        plt.close()
    if ack_policies:
        plt.figure(figsize=(6, 4))
        plt.bar(list(ack_policies.keys()), list(ack_policies.values()))
        plt.ylabel("samples")
        plt.tight_layout()
        plt.savefig(artifact_dir / "ack_policy.png")
        plt.close()

    q_series = []
    for sample in arms:
        q = sample.get("q_actual_deg")
        if isinstance(q, list) and len(q) == 6 and all(numeric(v) is not None for v in q):
            q_series.append([float(v) for v in q])
    if q_series:
        plt.figure(figsize=(9, 5))
        for idx in range(6):
            plt.plot([row[idx] for row in q_series], label=f"j{idx}")
        plt.xlabel("sample")
        plt.ylabel("q_actual_deg")
        plt.legend(loc="best")
        plt.tight_layout()
        plt.savefig(artifact_dir / "q_actual_time.png")
        plt.close()


def copy_servo_log(root: Path, artifact_dir: Path) -> None:
    source = root / "logs" / "servo_log.csv"
    if source.is_file():
        shutil.copy2(source, artifact_dir / "servo_log.csv")


def run_acceptance(args: argparse.Namespace) -> int:
    config = load_config(args.config)
    preflight_result = preflight(args, config)
    artifact_dir = args.artifact_dir.resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    (artifact_dir / "safety_preflight.json").write_text(
        json.dumps(preflight_result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"safety_preflight": preflight_result}, indent=2, sort_keys=True), file=sys.stderr)
    if args.preflight_only:
        summary = {
            "result": "completed",
            "result_reason": "preflight only",
            "artifact_dir": str(artifact_dir),
            "safety_preflight": preflight_result,
        }
        (artifact_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        write_summary_csv(artifact_dir / "summary.csv", summary)
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 0

    endpoint = state_endpoint(config)
    state_sock = bind_state_socket(endpoint)
    server_proc: subprocess.Popen[str] | None = None
    states: list[dict[str, Any]] = []
    command_count = 0
    server_log_path = artifact_dir / "rb_servo_server.log"
    try:
        server_proc = start_server(args, server_log_path, preflight_result)
        first_state = wait_for_first_state(
            state_sock,
            args.startup_timeout_sec,
            server_proc,
            server_log_path,
            endpoint,
        )
        states.append(first_state)
        state_stream_path = artifact_dir / "state_stream.jsonl"
        with state_stream_path.open("w", encoding="utf-8") as handle:
            handle.write(json.dumps(first_state, sort_keys=True) + "\n")

        stop_time = time.monotonic() + args.duration_sec
        if args.mode == "read_only":
            more_states = drain_states(state_sock, args.duration_sec, artifact_dir / "state_stream_extra.jsonl")
            states.extend(more_states)
            # Preserve the documented artifact path as the full stream.
            with state_stream_path.open("a", encoding="utf-8") as handle:
                for state in more_states:
                    handle.write(json.dumps(state, sort_keys=True) + "\n")
            (artifact_dir / "command_packets.jsonl").write_text("", encoding="utf-8")
        else:
            command_count = send_commands(config, args, states, artifact_dir / "command_packets.jsonl", stop_time)
            remaining = max(stop_time - time.monotonic(), 0.1)
            states.extend(drain_states(state_sock, remaining, artifact_dir / "state_stream_extra.jsonl"))
            with state_stream_path.open("a", encoding="utf-8") as handle:
                for state in states[1:]:
                    handle.write(json.dumps(state, sort_keys=True) + "\n")
    finally:
        state_sock.close()
        stop_server(server_proc)

    returncode = server_proc.returncode if server_proc is not None else None
    copy_servo_log(args.root.resolve(), artifact_dir)
    summary = summarize(args, config, preflight_result, states, command_count, artifact_dir, returncode)
    (artifact_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    write_summary_csv(artifact_dir / "summary.csv", summary)
    (artifact_dir / "error_code_counts.json").write_text(
        json.dumps(summary["error_code_distribution"], indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    if not args.skip_plots:
        write_plots(artifact_dir, states, args.arm)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["result"] == "completed" else 1


def expect_error(fn: Any) -> None:
    try:
        fn()
    except AcceptanceError:
        return
    raise AssertionError("expected AcceptanceError")


def run_self_test() -> int:
    body = """schema: robotics_lab.rb_servo_server.v1
left_robot:
  backend_type: rbpodo
  run_mode: real
  ip: "172.28.60.200"
  operation_mode: real
  command_timeout_sec: 0.02
  servo_t1_sec: 0.002
  servo_t2_sec: 0.05
  servo_gain: 1.0
  servo_alpha: 0.5
  disable_waiting_ack: false
right_robot:
  backend_type: rbpodo
  run_mode: real
  ip: "172.28.60.201"
  operation_mode: real
  command_timeout_sec: 0.02
  servo_t1_sec: 0.002
  servo_t2_sec: 0.05
  servo_gain: 1.0
  servo_alpha: 0.5
  disable_waiting_ack: false
servo:
  rate_hz: 500
  send_servo_commands: false
  servo_t1_rate_match_tolerance_ratio: 0.2
network:
  command_bind: "udp://127.0.0.1:50010"
  state_pub_endpoint: "udp://127.0.0.1:50110"
logging:
  directory: "./logs"
"""
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "config.yaml"
        path.write_text(body, encoding="utf-8")
        config = load_config(path)
        base = argparse.Namespace(
            config=path,
            arm="left",
            mode="read_only",
            profile="500hz_ack",
            duration_sec=1.0,
            command_rate_hz=None,
            artifact_dir=Path(tmp) / "artifacts",
            server=Path("missing"),
            root=Path("."),
            startup_timeout_sec=1.0,
            state_timeout_sec=1.0,
            max_state_age_us=250_000.0,
            max_delta_deg=0.2,
            joint_index=0,
            expected_left_ip="172.28.60.200",
            expected_right_ip="172.28.60.201",
            allow_motion=False,
            allow_ack_disabled=False,
            set_pgmode_simulation=False,
            verify_pgmode_simulation=False,
            pgmode_timeout_sec=1.0,
            pgmode_command_port=5000,
            preflight_only=False,
            skip_plots=True,
            i_understand_this_connects_to_real_controller=True,
        )
        old_env = {key: os.environ.get(key) for key in ENV_KEYS}
        try:
            os.environ["RB_ALLOW_REAL_ROBOT"] = "1"
            os.environ.pop("RB_ALLOW_REAL_MOTION", None)
            os.environ.pop("RB_ALLOW_RBPODO_ACK_DISABLED_MOTION", None)
            os.environ.pop("RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION", None)
            os.environ.pop("RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM", None)
            os.environ.pop("RB_RBPODO_PGMODE_SIMULATION_CONFIRMED", None)
            os.environ.pop("RB_ALLOW_REAL_CARTESIAN", None)
            result = preflight(base, config)
            assert result["profile"] == "500hz_ack"

            motion = argparse.Namespace(**vars(base))
            motion.mode = "servo_j_noop"
            expect_error(lambda: preflight(motion, config))

            controller_sim_config = load_config(path)
            controller_sim_config.left.operation_mode = "simulation"
            controller_sim_config.right.operation_mode = "simulation"
            controller_sim_config.servo["send_servo_commands"] = True
            controller_sim_config.servo["allow_controller_simulation_motion"] = True
            controller_sim_motion = argparse.Namespace(**vars(base))
            controller_sim_motion.mode = "servo_j_noop"
            controller_sim_motion.allow_motion = True
            os.environ["RB_ALLOW_REAL_MOTION"] = "1"
            expect_error(lambda: preflight(controller_sim_motion, controller_sim_config))
            os.environ["RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION"] = "1"
            expect_error(lambda: preflight(controller_sim_motion, controller_sim_config))
            os.environ.pop("RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION", None)
            os.environ.pop("RB_ALLOW_REAL_MOTION", None)

            config.servo["send_servo_commands"] = True
            expect_error(lambda: preflight(base, config))
            config.servo["send_servo_commands"] = False

            bad_rate = load_config(path)
            bad_rate.left.servo_t1_sec = 0.01
            expect_error(lambda: preflight(base, bad_rate))

            no_ack = load_config(path)
            no_ack.left.disable_waiting_ack = True
            no_ack.right.disable_waiting_ack = True
            expect_error(lambda: preflight(base, no_ack))

            fake_states = [
                {
                    "loop_start_time_ns": 1000,
                    "fault_latched": False,
                    "observed_backend": "rbpodo",
                    "left": {
                        "has_valid_joint_state": True,
                        "state_age_us": 1000.0,
                        "q_actual_deg": [0, 1, 2, 3, 4, 5],
                        "error_code": 0,
                        "last_send": {
                            "accepted": True,
                            "backend_error_kind": "None",
                            "duration_us": 100.0,
                            "ack_policy": "wait",
                            "ack_observed": True,
                            "controller_acceptance_observed": True,
                            "ack_wait_duration_us": 80.0,
                            "send_acceptance_semantics": "controller_ack_observed",
                        },
                    },
                },
                {
                    "loop_start_time_ns": 2_001_000,
                    "fault_latched": False,
                    "observed_backend": "rbpodo",
                    "left": {
                        "has_valid_joint_state": True,
                        "state_age_us": 2000.0,
                        "q_actual_deg": [0, 1, 2, 3, 4, 5.1],
                        "error_code": 0,
                        "last_send": {
                            "accepted": True,
                            "backend_error_kind": "None",
                            "duration_us": 120.0,
                            "ack_policy": "wait",
                            "ack_observed": True,
                            "controller_acceptance_observed": True,
                            "ack_wait_duration_us": 90.0,
                            "send_acceptance_semantics": "controller_ack_observed",
                        },
                    },
                },
            ]
            summary = summarize(base, config, result, fake_states, 2, Path(tmp), None)
            assert summary["state_valid_ratio"] == 1.0
            assert summary["ack_observed_count"] == 2
            assert abs(summary["q_actual_drift_from_start_deg"] - 0.1) < 1e-9
            write_plots(Path(tmp), fake_states, "left")

            log_path = Path(tmp) / "rb_servo_server.log"
            log_path.write_text(
                "\n".join(
                    [
                        "[ERROR] invalid robot startup state: right",
                        "  has_valid_joint_state=true",
                        "  op_stat_self_collision=1977953904",
                        "[ERROR] failed to start servo loop",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            class ExitedProcess:
                returncode = 7

                def poll(self) -> int:
                    return self.returncode

            diagnostic = state_stream_timeout_message(
                ExitedProcess(), log_path, "udp://127.0.0.1:50110"
            )
            assert "server_returncode=7" in diagnostic
            assert "last_120_lines_of_rb_servo_server.log" in diagnostic
            assert "scripts/rbpodo_state_dump.py" in diagnostic
            assert "invalid robot startup state" in diagnostic
        finally:
            for key, value in old_env.items():
                if value is None:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = value
    print("rbpodo_servo_acceptance self-test passed")
    return 0


def main() -> int:
    args = parse_args()
    if args.self_test:
        return run_self_test()
    missing = [name for name in ("config", "arm", "artifact_dir") if getattr(args, name) is None]
    if missing:
        print(f"rbpodo_servo_acceptance: FAIL: missing required arguments: {', '.join('--' + name.replace('_', '-') for name in missing)}", file=sys.stderr)
        return 2
    try:
        return run_acceptance(args)
    except AcceptanceError as exc:
        print(f"rbpodo_servo_acceptance: FAIL: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
