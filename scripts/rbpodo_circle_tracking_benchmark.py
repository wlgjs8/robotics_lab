#!/usr/bin/env python3
"""rbpodo controller-simulation circular TCP tracking benchmark.

This runner connects to real Rainbow controller boxes through rbpodo while the
controllers are in pgmode simulation. It sends only through rb_servo_server's
UDP CommandServer and records controller-reference TCP telemetry from the state
stream. Physical robot motion is not an accepted outcome.
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
import time
from collections import Counter
from pathlib import Path
from typing import Any

import benchmark_overlay
import circle_tracking_benchmark as sim_bench
from cartesian_acceptance import CommandRecorder, StateCapture, wait_for
from rbpodo_servo_acceptance import (
    ENV_KEYS,
    REAL_ROBOT_IPS,
    ParsedConfig,
    as_bool,
    as_float,
    env_enabled,
    env_snapshot,
    load_config,
    parse_udp_endpoint,
    simple_yaml_sections,
    state_stream_timeout_message,
)


SCHEMA = "robotics_lab.rbpodo_circle_tracking_benchmark.v1"
CONTROLLERS = (
    "twist_stand",
    "twist_local",
    "twist_stand_feedback",
    "twist_local_feedback",
    "server_circle",
)
TRACKING_SOURCES = ("auto", "tcp_ref_stand", "tcp_actual_stand")
PROFILE_DEFAULTS = {
    "circle_15cm_16s": (0.15, 16.0),
    "gene_15cm_4s": (0.15, 4.0),
}
REQUIRED_ENV = (
    "RB_ALLOW_REAL_ROBOT",
    "RB_ALLOW_REAL_MOTION",
    "RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION",
    "RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN",
)
DEFAULT_PHYSICAL_MOTION_WARNING_DEG = 0.05
CARTESIAN_UNAVAILABLE_PREFIX = "cartesian_control_unavailable"
CARTESIAN_REJECTION_HINTS = [
    "check cartesian_control.allow_in_controller_simulation: true",
    "check RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1",
    "check operation_mode: simulation",
    "check same-run pgmode simulation confirmation",
]


class BenchmarkError(RuntimeError):
    pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run rbpodo controller-simulation circle tracking against real "
            "Rainbow controller boxes in pgmode simulation. Physical robot "
            "motion is refused; tracking defaults to tcp_ref_stand."
        )
    )
    parser.add_argument("--root", type=Path, default=Path("."))
    parser.add_argument("--server", type=Path, required=True)
    parser.add_argument("--server-config", type=Path, required=True)
    parser.add_argument("--arm", choices=("left", "right"), required=True)
    parser.add_argument("--controller", choices=CONTROLLERS, default="twist_stand")
    parser.add_argument("--plane", choices=("xy", "xz", "yz"), default="xy")
    parser.add_argument("--profile", choices=tuple(PROFILE_DEFAULTS), default="circle_15cm_16s")
    parser.add_argument("--diameter-m", type=float)
    parser.add_argument("--period-sec", type=float)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--command-rate-hz", type=float, default=100.0)
    parser.add_argument("--warmup-sec", type=float, default=0.5)
    parser.add_argument("--settle-sec", type=float, default=1.0)
    parser.add_argument("--startup-timeout-sec", type=float, default=12.0)
    parser.add_argument("--tracking-source", choices=TRACKING_SOURCES, default="auto")
    parser.add_argument("--feedback-kp-pos", type=float, default=2.0)
    parser.add_argument("--feedback-kp-ori", type=float, default=2.0)
    parser.add_argument("--feedback-max-linear-m-s", type=float)
    parser.add_argument("--feedback-max-angular-rad-s", type=float)
    parser.add_argument("--feedback-use-current-state-time", action="store_true")
    parser.add_argument("--physical-motion-warning-deg", type=float, default=DEFAULT_PHYSICAL_MOTION_WARNING_DEG)
    parser.add_argument("--max-state-age-us", type=float, default=250_000.0)
    parser.add_argument("--max-allowed-rms-error-m", type=float)
    parser.add_argument("--max-allowed-p95-error-m", type=float)
    parser.add_argument("--max-allowed-orientation-drift-rad", type=float)
    parser.add_argument("--max-allowed-latency-ms", type=float)
    parser.add_argument("--pgmode-summary-json", type=Path)
    parser.add_argument("--set-pgmode-simulation", action="store_true")
    parser.add_argument("--verify-pgmode-simulation", action="store_true")
    parser.add_argument("--pgmode-timeout-sec", type=float, default=1.0)
    parser.add_argument("--pgmode-command-port", type=int, default=5000)
    parser.add_argument("--overlay-pub-endpoint", default="udp://127.0.0.1:50261")
    parser.add_argument("--overlay-pub-rate-hz", type=float, default=20.0)
    parser.add_argument("--overlay-run-id")
    parser.add_argument("--overlay-disable", action="store_true")
    parser.add_argument("--artifact-dir", type=Path, required=True)
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--skip-plots", action="store_true")
    parser.add_argument(
        "--i-understand-this-connects-to-real-controller",
        action="store_true",
        help="Required before connecting to known Rainbow controller IPs.",
    )
    parser.add_argument(
        "--i-confirm-controller-is-in-pgmode-simulation",
        action="store_true",
        help="Required acknowledgement before pgmode simulation verification is accepted.",
    )
    return parser.parse_args()


def finite_number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        number = float(value)
        return number if math.isfinite(number) else None
    return None


def metric_block(values: list[float]) -> dict[str, float | None]:
    return {
        "p50": sim_bench.percentile(values, 50.0),
        "p95": sim_bench.percentile(values, 95.0),
        "p99": sim_bench.percentile(values, 99.0),
        "max": max(values) if values else None,
    }


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_csv(path: Path, rows: list[dict[str, Any]], fieldnames: list[str] | None = None) -> None:
    if fieldnames is None:
        keys: list[str] = []
        for row in rows:
            for key in row:
                if key not in keys:
                    keys.append(key)
        fieldnames = keys
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def benchmark_env_snapshot() -> dict[str, str | None]:
    snapshot = env_snapshot()
    for key in REQUIRED_ENV:
        snapshot[key] = os.environ.get(key)
    snapshot["RB_ALLOW_REAL_CARTESIAN"] = os.environ.get("RB_ALLOW_REAL_CARTESIAN")
    return snapshot


def apply_profile(args: argparse.Namespace) -> None:
    default_diameter, default_period = PROFILE_DEFAULTS[args.profile]
    if args.diameter_m is None:
        args.diameter_m = default_diameter
    if args.period_sec is None:
        args.period_sec = default_period


def apply_overlay_defaults(args: argparse.Namespace) -> None:
    if not hasattr(args, "overlay_pub_endpoint"):
        args.overlay_pub_endpoint = "udp://127.0.0.1:50261"
    if not hasattr(args, "overlay_pub_rate_hz"):
        args.overlay_pub_rate_hz = 20.0
    if not hasattr(args, "overlay_run_id"):
        args.overlay_run_id = None
    if not hasattr(args, "overlay_disable"):
        args.overlay_disable = False


def required_speed(args: argparse.Namespace) -> float:
    assert args.diameter_m is not None and args.period_sec is not None
    return math.pi * args.diameter_m / args.period_sec


def endpoint_from_config(config: ParsedConfig, key: str, default: str) -> tuple[str, int, str]:
    endpoint = str(config.network.get(key, default))
    host, port = parse_udp_endpoint(endpoint)
    return host, port, endpoint


def selected_arm_config(config: ParsedConfig, arm: str) -> Any:
    return config.left if arm == "left" else config.right


def all_arm_configs(config: ParsedConfig) -> tuple[tuple[str, Any], tuple[str, Any]]:
    return (("left", config.left), ("right", config.right))


def ensure_finite_pose(value: Any, label: str) -> dict[str, Any]:
    return sim_bench.pose(value, label)


def pose_for_source(snapshot: dict[str, Any], arm: str, source: str) -> dict[str, Any]:
    arm_state = snapshot.get(arm)
    if not isinstance(arm_state, dict):
        raise BenchmarkError(f"{arm} state is missing")
    if source == "tcp_ref_stand":
        if arm_state.get("tcp_ref_valid") is False:
            raise BenchmarkError(f"{arm}.tcp_ref_stand is marked invalid")
        return ensure_finite_pose(arm_state.get("tcp_ref_stand"), f"{arm}.tcp_ref_stand")
    if source == "tcp_actual_stand":
        if arm_state.get("tcp_actual_valid") is False:
            raise BenchmarkError(f"{arm}.tcp_actual_stand is marked invalid")
        value = arm_state.get("tcp_actual_stand")
        if value is None:
            value = arm_state.get("tcp_stand")
        return ensure_finite_pose(value, f"{arm}.tcp_actual_stand")
    raise BenchmarkError(f"unsupported tracking source {source}")


def source_valid(snapshot: dict[str, Any], arm: str, source: str) -> bool:
    try:
        pose_for_source(snapshot, arm, source)
        return True
    except Exception:
        return False


def valid_rbpodo_state(snapshot: dict[str, Any], arm: str, source: str | None = None) -> bool:
    if snapshot.get("schema_version") != 1:
        return False
    if snapshot.get("fault_latched") is True:
        return False
    arm_state = snapshot.get(arm)
    if not isinstance(arm_state, dict):
        return False
    if arm_state.get("has_valid_joint_state") is not True:
        return False
    if source is None:
        return True
    return source_valid(snapshot, arm, source)


def select_tracking_source(requested: str, snapshot: dict[str, Any], arm: str) -> tuple[str, str | None]:
    if requested == "auto":
        if source_valid(snapshot, arm, "tcp_ref_stand"):
            return "tcp_ref_stand", None
        raise BenchmarkError(
            "tracking-source auto requires valid tcp_ref_stand in rbpodo controller-simulation state stream"
        )
    if source_valid(snapshot, arm, requested):
        warning = None
        if requested == "tcp_actual_stand":
            warning = (
                "tcp_actual_stand was explicitly selected; controller pgmode "
                "simulation reports should normally use tcp_ref_stand"
            )
        return requested, warning
    raise BenchmarkError(f"tracking source {requested} is unavailable or invalid in state stream")


def load_pgmode_summary(path: Path, expected_ips: list[str]) -> dict[str, Any]:
    try:
        summary = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise BenchmarkError(f"failed to read pgmode summary {path}: {exc}") from exc
    if not isinstance(summary, dict):
        raise BenchmarkError("pgmode summary must be a JSON object")
    if summary.get("overall_result") != "ok":
        raise BenchmarkError("pgmode summary does not confirm simulation mode")
    seen_ips = set(str(item) for item in summary.get("ips", []))
    if seen_ips and not set(expected_ips).issubset(seen_ips):
        raise BenchmarkError("pgmode summary IPs do not cover configured controllers")
    return summary


def ensure_pgmode(args: argparse.Namespace, config: ParsedConfig) -> dict[str, Any]:
    ips = [config.left.ip, config.right.ip]
    if args.set_pgmode_simulation and args.verify_pgmode_simulation:
        raise BenchmarkError("--set-pgmode-simulation and --verify-pgmode-simulation are mutually exclusive")
    if args.pgmode_summary_json and (args.set_pgmode_simulation or args.verify_pgmode_simulation):
        raise BenchmarkError("--pgmode-summary-json cannot be combined with pgmode set/verify flags")
    if not args.i_confirm_controller_is_in_pgmode_simulation:
        raise BenchmarkError("missing --i-confirm-controller-is-in-pgmode-simulation")
    if args.pgmode_summary_json:
        summary = load_pgmode_summary(args.pgmode_summary_json, ips)
        summary = dict(summary)
        summary["source"] = str(args.pgmode_summary_json.resolve())
        return summary
    try:
        from rainbow_pgmode import RainbowPgmodeError, ensure_controller_simulation_mode
    except Exception as exc:
        raise BenchmarkError("scripts/rainbow_pgmode.py helper is unavailable") from exc
    try:
        return ensure_controller_simulation_mode(
            ips,
            args.pgmode_timeout_sec,
            port=args.pgmode_command_port,
            confirmation=args.i_understand_this_connects_to_real_controller,
            set_simulation=args.set_pgmode_simulation,
            verify_only=not args.set_pgmode_simulation,
        )
    except RainbowPgmodeError as exc:
        raise BenchmarkError(
            f"controller not confirmed in pgmode simulation; refusing controller-simulation benchmark: {exc}"
        ) from exc


def docker_port_check(command_port: int, state_port: int) -> dict[str, Any]:
    docker = shutil.which("docker")
    if docker is None:
        return {"status": "not_checked", "reason": "docker CLI unavailable"}
    try:
        completed = subprocess.run(
            [docker, "ps", "--format", "{{.Names}} {{.Ports}}"],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=2.0,
        )
    except Exception as exc:
        return {"status": "not_checked", "reason": str(exc)}
    if completed.returncode != 0:
        return {"status": "not_checked", "reason": completed.stderr.strip() or "docker ps failed"}
    hits = [
        line
        for line in completed.stdout.splitlines()
        if "rbsim" in line.lower()
        and (str(command_port) in line or str(state_port) in line)
    ]
    if hits:
        return {"status": "conflict", "matches": hits}
    return {"status": "ok"}


def validate_config_and_env(
    args: argparse.Namespace,
    config: ParsedConfig,
    sections: dict[str, dict[str, Any]],
) -> tuple[dict[str, Any], dict[str, Any]]:
    apply_profile(args)
    apply_overlay_defaults(args)
    assert args.diameter_m is not None and args.period_sec is not None
    if args.repeat < 1:
        raise BenchmarkError("--repeat must be >= 1")
    for name, value in (
        ("--diameter-m", args.diameter_m),
        ("--period-sec", args.period_sec),
        ("--command-rate-hz", args.command_rate_hz),
        ("--warmup-sec", args.warmup_sec),
        ("--settle-sec", args.settle_sec),
        ("--feedback-kp-pos", args.feedback_kp_pos),
        ("--feedback-kp-ori", args.feedback_kp_ori),
        ("--physical-motion-warning-deg", args.physical_motion_warning_deg),
    ):
        if name in {"--warmup-sec", "--settle-sec"}:
            if not math.isfinite(value) or value < 0.0:
                raise BenchmarkError(f"{name} must be finite and non-negative")
        elif not math.isfinite(value) or value <= 0.0:
            raise BenchmarkError(f"{name} must be finite and positive")
    if args.pgmode_command_port < 1 or args.pgmode_command_port > 65535:
        raise BenchmarkError("--pgmode-command-port must be in [1, 65535]")
    if not math.isfinite(args.pgmode_timeout_sec) or args.pgmode_timeout_sec <= 0.0:
        raise BenchmarkError("--pgmode-timeout-sec must be finite and positive")
    if not args.overlay_disable:
        if not math.isfinite(args.overlay_pub_rate_hz) or args.overlay_pub_rate_hz <= 0.0:
            raise BenchmarkError("--overlay-pub-rate-hz must be finite and positive")
        try:
            benchmark_overlay.parse_udp_endpoint(args.overlay_pub_endpoint)
        except ValueError as exc:
            raise BenchmarkError(str(exc)) from exc

    for label, arm_cfg in all_arm_configs(config):
        if arm_cfg.backend_type != "rbpodo":
            raise BenchmarkError(f"{label}_robot.backend_type must be rbpodo")
        if arm_cfg.run_mode != "real":
            raise BenchmarkError(f"{label}_robot.run_mode must be real for rbpodo controller-simulation benchmark")
        if arm_cfg.operation_mode not in {"simulation", "sim"}:
            actual = arm_cfg.operation_mode or "<missing>"
            raise BenchmarkError(
                f"config operation_mode is {actual} for {label}_robot; refusing physical real circle benchmark"
            )
        if not arm_cfg.ip:
            raise BenchmarkError(f"{label}_robot.ip is required")

    known_ips = {config.left.ip, config.right.ip} & REAL_ROBOT_IPS
    if not args.i_understand_this_connects_to_real_controller:
        raise BenchmarkError("refusing controller connection without explicit real-controller confirmation flag")
    if not args.i_confirm_controller_is_in_pgmode_simulation:
        raise BenchmarkError("missing --i-confirm-controller-is-in-pgmode-simulation")
    for name in REQUIRED_ENV:
        if not env_enabled(name):
            raise BenchmarkError(f"rbpodo controller-simulation circle benchmark requires {name}=1")
    if env_enabled("RB_ALLOW_REAL_CARTESIAN"):
        raise BenchmarkError("RB_ALLOW_REAL_CARTESIAN must not be set for controller-simulation circle benchmark")

    send_servo_commands = as_bool(config.servo.get("send_servo_commands"), False)
    if not send_servo_commands:
        raise BenchmarkError("controller-simulation circle benchmark requires servo.send_servo_commands=true")
    if not as_bool(config.servo.get("allow_controller_simulation_motion"), False):
        raise BenchmarkError("controller-simulation circle benchmark requires servo.allow_controller_simulation_motion=true")
    if as_bool(config.servo.get("allow_controller_simulation_diagnostics_suspect"), False) and not env_enabled(
        "RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM"
    ):
        raise BenchmarkError(
            "diagnostics-suspect controller-simulation override requires "
            "RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1"
        )
    if (config.left.disable_waiting_ack or config.right.disable_waiting_ack) and not env_enabled(
        "RB_ALLOW_RBPODO_ACK_DISABLED_MOTION"
    ):
        raise BenchmarkError("ACK-off controller-simulation motion requires RB_ALLOW_RBPODO_ACK_DISABLED_MOTION=1")

    cartesian = sections.get("cartesian_control", {})
    if as_bool(cartesian.get("allow_in_real"), False):
        raise BenchmarkError("cartesian_control.allow_in_real must remain false")
    if not as_bool(cartesian.get("enable"), False):
        raise BenchmarkError("cartesian_control.enable must be true for circle benchmark")
    if not as_bool(cartesian.get("allow_in_controller_simulation"), False):
        raise BenchmarkError(
            "rbpodo controller-simulation Cartesian benchmark requires "
            "cartesian_control.allow_in_controller_simulation=true"
        )
    if args.controller == "server_circle":
        if not as_bool(cartesian.get("enable_benchmark_primitives"), False):
            raise BenchmarkError("server_circle requires cartesian_control.enable_benchmark_primitives=true")
        raise BenchmarkError("server_circle is not enabled for rbpodo controller-simulation real-controller configs")

    max_twist = as_float(cartesian.get("max_twist_linear_m_s"))
    max_twist_angular = as_float(cartesian.get("max_twist_angular_rad_s"))
    if max_twist is None:
        raise BenchmarkError("config must expose cartesian_control.max_twist_linear_m_s")
    if max_twist_angular is None:
        raise BenchmarkError("config must expose cartesian_control.max_twist_angular_rad_s")
    speed = required_speed(args)
    if speed > max_twist + 1e-9:
        raise BenchmarkError(f"required tangential speed {speed:.6f} m/s exceeds max_twist_linear_m_s {max_twist:.6f}")
    if args.controller.endswith("_feedback"):
        if args.feedback_max_linear_m_s is None:
            args.feedback_max_linear_m_s = max_twist
        if args.feedback_max_angular_rad_s is None:
            args.feedback_max_angular_rad_s = max_twist_angular
        for name, value in (
            ("--feedback-max-linear-m-s", args.feedback_max_linear_m_s),
            ("--feedback-max-angular-rad-s", args.feedback_max_angular_rad_s),
        ):
            if value is None or not math.isfinite(value) or value <= 0.0:
                raise BenchmarkError(f"{name} must be finite and positive")

    command_host, command_port, command_endpoint = endpoint_from_config(config, "command_bind", "udp://127.0.0.1:50010")
    state_host, state_port, state_endpoint = endpoint_from_config(config, "state_pub_endpoint", "udp://127.0.0.1:50110")
    docker_check = docker_port_check(command_port, state_port)
    if docker_check.get("status") == "conflict":
        raise BenchmarkError("Docker rb_simulator appears to use benchmark UDP ports: " + "; ".join(docker_check["matches"]))
    pgmode_summary = ensure_pgmode(args, config)
    preflight = {
        "passed": True,
        "schema": SCHEMA,
        "backend": "rbpodo",
        "controller_simulation_only": True,
        "physical_motion_expected": False,
        "physical_real_circle_refused": True,
        "arm": args.arm,
        "server_config": str(args.server_config.resolve()),
        "profile": args.profile,
        "controller": args.controller,
        "diameter_m": args.diameter_m,
        "period_sec": args.period_sec,
        "repeat": args.repeat,
        "command_rate_hz": args.command_rate_hz,
        "required_tangential_speed_m_s": speed,
        "max_twist_linear_m_s": max_twist,
        "max_twist_angular_rad_s": max_twist_angular,
        "command_endpoint": command_endpoint,
        "state_endpoint": state_endpoint,
        "real_robot_ips_checked": sorted(REAL_ROBOT_IPS),
        "configured_ips": [config.left.ip, config.right.ip],
        "known_real_ips": sorted(known_ips),
        "confirmation_flag": args.i_understand_this_connects_to_real_controller,
        "pgmode_confirmation_flag": args.i_confirm_controller_is_in_pgmode_simulation,
        "pgmode_simulation_confirmed": pgmode_summary.get("overall_result") == "ok",
        "pgmode_summary": pgmode_summary,
        "env": benchmark_env_snapshot(),
        "required_env": list(REQUIRED_ENV),
        "send_servo_commands": send_servo_commands,
        "allow_controller_simulation_motion": as_bool(config.servo.get("allow_controller_simulation_motion"), False),
        "allow_controller_simulation_diagnostics_suspect": as_bool(
            config.servo.get("allow_controller_simulation_diagnostics_suspect"), False
        ),
        "disable_waiting_ack": bool(config.left.disable_waiting_ack or config.right.disable_waiting_ack),
        "docker_rb_simulator_port_check": docker_check,
        "state_pub_endpoint_unique_checked_by_bind": True,
        "overlay_enabled": not args.overlay_disable,
        "overlay_pub_endpoint": None if args.overlay_disable else args.overlay_pub_endpoint,
        "overlay_pub_rate_hz": None if args.overlay_disable else args.overlay_pub_rate_hz,
        "server_env_overrides": {"RB_RBPODO_PGMODE_SIMULATION_CONFIRMED": "1"},
    }
    endpoints = {"command_host": command_host, "command_port": command_port, "state_host": state_host, "state_port": state_port}
    return preflight, endpoints


def preflight(args: argparse.Namespace) -> tuple[ParsedConfig, dict[str, dict[str, Any]], dict[str, Any], dict[str, Any]]:
    if not args.server_config.is_file():
        hint = ""
        if "rb_servo_server/config/local" in args.server_config.as_posix() or args.server_config.name in {
            "dual_real_rbpodo_circle_15cm16s.yaml",
            "dual_real_rbpodo_circle_15cm4s.yaml",
        }:
            hint = (
                "; create local controller-simulation circle configs with "
                "tools/create_rbpodo_circle_local_configs.sh"
            )
        raise BenchmarkError(f"server config not found: {args.server_config}{hint}")
    if args.server_config.name.endswith(".example.yaml"):
        print(
            "WARNING: You are using an example config. Recommended: copy to "
            "rb_servo_server/config/local first with "
            "tools/create_rbpodo_circle_local_configs.sh.",
            file=sys.stderr,
        )
    config = load_config(args.server_config)
    sections = simple_yaml_sections(args.server_config)
    preflight_result, endpoints = validate_config_and_env(args, config, sections)
    return config, sections, preflight_result, endpoints


def start_server(args: argparse.Namespace, log_path: Path, preflight_result: dict[str, Any]) -> subprocess.Popen[str]:
    server = (args.root / args.server).resolve() if not args.server.is_absolute() else args.server
    if not server.is_file():
        raise BenchmarkError(f"server binary not found: {server}")
    command = [str(server), "--config", str(args.server_config.resolve())]
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


def wait_for_state_source(
    capture: StateCapture,
    args: argparse.Namespace,
    timeout_sec: float,
    proc: subprocess.Popen[str] | None,
    log_path: Path,
    state_endpoint: str,
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        for snapshot in capture.snapshots:
            if valid_rbpodo_state(snapshot, args.arm):
                return snapshot
        if proc is not None and proc.poll() is not None:
            break
        time.sleep(0.02)
    raise BenchmarkError(state_stream_timeout_message(proc, log_path, state_endpoint))


def latest_feedback_snapshot(capture: StateCapture, arm: str, source: str, stale_limit_ns: int, now_ns: int) -> dict[str, Any] | None:
    for snapshot in reversed(capture.snapshots):
        state_ns = int(snapshot.get("host_time_ns", -1))
        if state_ns < 0 or now_ns - state_ns > stale_limit_ns:
            continue
        if valid_rbpodo_state(snapshot, arm, source):
            return snapshot
    return None


def send_udp(host: str, port: int, packet: dict[str, Any], recorder: CommandRecorder) -> None:
    sim_bench.send_udp(host, port, packet, recorder)


def warn_overlay(message: str) -> None:
    print(f"WARNING: benchmark overlay: {message}", file=sys.stderr)


def desired_phase(traj: sim_bench.Trajectory, t_sec: float) -> float:
    return math.fmod(traj.omega * t_sec, 2.0 * math.pi)


def latest_overlay_snapshot(
    capture: StateCapture,
    arm: str,
    source: str,
    stale_limit_ns: int,
    now_ns: int,
) -> dict[str, Any] | None:
    return latest_feedback_snapshot(capture, arm, source, stale_limit_ns, now_ns)


def publish_overlay_status(
    args: argparse.Namespace,
    capture: StateCapture,
    publisher: benchmark_overlay.BenchmarkOverlayPublisher | None,
    metrics: benchmark_overlay.CircleOverlayMetrics | None,
    traj: sim_bench.Trajectory,
    q0: list[float],
    tracking_source: str,
    *,
    t_sec: float,
    command_count: int,
    result_so_far: str = "running",
    force: bool = False,
) -> None:
    if publisher is None or metrics is None or not publisher.enabled:
        return
    now_ns = time.monotonic_ns()
    desired_position = traj.position(t_sec)
    actual_position = None
    sample_id: int | None = None
    stale_limit_ns = int(max(1_000.0, args.max_state_age_us) * 1000.0)
    snapshot = latest_overlay_snapshot(capture, args.arm, tracking_source, stale_limit_ns, now_ns)
    if snapshot is not None:
        try:
            tracking_pose = pose_for_source(snapshot, args.arm, tracking_source)
            actual_position = sim_bench.vec(tracking_pose)
            sample_id = int(snapshot.get("host_time_ns", -1))
        except Exception:
            actual_position = None
            sample_id = None
    metrics.observe(
        t_sec=t_sec,
        desired_position=desired_position,
        actual_position=actual_position,
        sample_id=sample_id,
    )
    try:
        message = benchmark_overlay.build_circle_overlay_message(
            run_id=publisher.run_id,
            arm=args.arm,
            profile=args.profile,
            controller=args.controller,
            tracking_source=tracking_source,
            plane=args.plane,
            center_stand=traj.center,
            axis1_stand=traj.axis1,
            axis2_stand=traj.axis2,
            radius_m=traj.radius,
            period_sec=float(args.period_sec),
            repeat=int(args.repeat),
            phase_rad=desired_phase(traj, t_sec),
            desired_pose_stand=benchmark_overlay.pose_payload(desired_position, q0),
            metrics=metrics.snapshot(),
            command_count=command_count,
            physical_motion_expected=False,
            result_so_far=result_so_far,
        )
        publisher.publish(message, force=force)
    except Exception as exc:
        publisher.record_warning(f"overlay publish skipped: {exc}")


def twist_packet(args: argparse.Namespace, frame: str, twist: list[float], timeout_sec: float) -> dict[str, Any]:
    return sim_bench.twist_command(args.arm, frame, twist, timeout_sec)


def stream_twist(
    args: argparse.Namespace,
    capture: StateCapture,
    commands: CommandRecorder,
    endpoints: dict[str, Any],
    traj: sim_bench.Trajectory,
    frame: str,
    q0: list[float],
    tracking_source: str,
    duration_sec: float,
    overlay_publisher: benchmark_overlay.BenchmarkOverlayPublisher | None = None,
    overlay_metrics: benchmark_overlay.CircleOverlayMetrics | None = None,
) -> tuple[int, int, int]:
    period = 1.0 / args.command_rate_hz
    timeout = max(0.2, 3.0 * period)
    command_count = 0
    first_ns = 0
    last_ns = 0
    start_monotonic = time.monotonic()
    next_send = start_monotonic
    while True:
        now = time.monotonic()
        t = now - start_monotonic
        if t >= duration_sec:
            break
        v1, v2 = traj.velocity_components(t)
        twist = [0.0] * 6
        if args.plane == "xy":
            twist[0], twist[1] = v1, v2
        elif args.plane == "xz":
            twist[0], twist[2] = v1, v2
        else:
            twist[1], twist[2] = v1, v2
        packet = twist_packet(args, frame, twist, timeout)
        if first_ns == 0:
            first_ns = int(packet["host_time_ns"])
        last_ns = int(packet["host_time_ns"])
        send_udp(endpoints["command_host"], endpoints["command_port"], packet, commands)
        command_count += 1
        publish_overlay_status(
            args,
            capture,
            overlay_publisher,
            overlay_metrics,
            traj,
            q0,
            tracking_source,
            t_sec=t,
            command_count=command_count,
        )
        next_send += period
        time.sleep(max(0.0, next_send - time.monotonic()))
    stop = sim_bench.hold_command()
    send_udp(endpoints["command_host"], endpoints["command_port"], stop, commands)
    return first_ns, last_ns, command_count + 1


def stream_twist_feedback(
    args: argparse.Namespace,
    capture: StateCapture,
    commands: CommandRecorder,
    endpoints: dict[str, Any],
    traj: sim_bench.Trajectory,
    frame: str,
    q0: list[float],
    tracking_source: str,
    duration_sec: float,
    overlay_publisher: benchmark_overlay.BenchmarkOverlayPublisher | None = None,
    overlay_metrics: benchmark_overlay.CircleOverlayMetrics | None = None,
) -> tuple[int, int, int, list[dict[str, Any]]]:
    period = 1.0 / args.command_rate_hz
    timeout = max(0.2, 3.0 * period)
    stale_limit_ns = int(max(0.2, 3.0 * period) * 1e9)
    command_count = 0
    first_ns = 0
    last_ns = 0
    rows: list[dict[str, Any]] = []
    start_monotonic = time.monotonic()
    next_send = start_monotonic
    while True:
        now = time.monotonic()
        elapsed = now - start_monotonic
        if elapsed >= duration_sec:
            break
        now_ns = time.monotonic_ns()
        t = elapsed
        snapshot = latest_feedback_snapshot(capture, args.arm, tracking_source, stale_limit_ns, now_ns)
        if snapshot is None:
            applied_twist = [0.0] * 6
            row = sim_bench.zero_feedback_record(t, "missing/stale valid tracking source", frame)
        else:
            state_ns = int(snapshot.get("host_time_ns", -1))
            if args.feedback_use_current_state_time and first_ns:
                t = max(0.0, (state_ns - first_ns) / 1e9)
            tracking_pose = pose_for_source(snapshot, args.arm, tracking_source)
            p_actual = sim_bench.vec(tracking_pose)
            q_actual = tracking_pose["quaternion_xyzw"]
            p_ref = traj.position(t)
            position_error_stand = sim_bench.sub(p_ref, p_actual)
            orientation_error_stand = sim_bench.quat_error_vector(q0, q_actual)
            feedback = sim_bench.compute_feedback_twist_stand(
                feedforward_linear_stand=sim_bench.trajectory_velocity_stand(traj, t),
                position_error_stand=position_error_stand,
                orientation_error_stand=orientation_error_stand,
                kp_pos=args.feedback_kp_pos,
                kp_ori=args.feedback_kp_ori,
                max_linear_m_s=float(args.feedback_max_linear_m_s),
                max_angular_rad_s=float(args.feedback_max_angular_rad_s),
            )
            if frame == "local":
                rot_current = sim_bench.quat_to_matrix(q_actual)
                feedforward_linear = sim_bench.mat_transpose_vec(rot_current, sim_bench.twist_linear(feedback["feedforward_twist_stand"]))
                feedback_linear = sim_bench.mat_transpose_vec(rot_current, sim_bench.twist_linear(feedback["feedback_twist_stand"]))
                feedback_angular = sim_bench.mat_transpose_vec(rot_current, sim_bench.twist_angular(feedback["feedback_twist_stand"]))
                applied_linear = sim_bench.mat_transpose_vec(rot_current, sim_bench.twist_linear(feedback["applied_twist_stand"]))
                applied_angular = sim_bench.mat_transpose_vec(rot_current, sim_bench.twist_angular(feedback["applied_twist_stand"]))
                feedforward_twist = [*feedforward_linear, 0.0, 0.0, 0.0]
                feedback_twist = [*feedback_linear, *feedback_angular]
                applied_twist = [*applied_linear, *applied_angular]
            else:
                feedforward_twist = feedback["feedforward_twist_stand"]
                feedback_twist = feedback["feedback_twist_stand"]
                applied_twist = feedback["applied_twist_stand"]
            row = {
                "t_sec": t,
                "frame": frame,
                "tracking_source": tracking_source,
                "feedback_skip_reason": "",
                "actual_x": p_actual[0],
                "actual_y": p_actual[1],
                "actual_z": p_actual[2],
                "reference_x": p_ref[0],
                "reference_y": p_ref[1],
                "reference_z": p_ref[2],
                "position_error_vector": position_error_stand,
                "orientation_error_vector": orientation_error_stand,
                "feedforward_twist": feedforward_twist,
                "feedback_twist": feedback_twist,
                "applied_twist": applied_twist,
                "feedforward_twist_stand": feedback["feedforward_twist_stand"],
                "feedback_twist_stand": feedback["feedback_twist_stand"],
                "applied_twist_stand": feedback["applied_twist_stand"],
                "saturated": bool(feedback["saturated"]),
                "stale_or_invalid_state": False,
            }
        packet = twist_packet(args, frame, applied_twist, timeout)
        if first_ns == 0:
            first_ns = int(packet["host_time_ns"])
        last_ns = int(packet["host_time_ns"])
        row["host_time_ns"] = int(packet["host_time_ns"])
        rows.append(row)
        send_udp(endpoints["command_host"], endpoints["command_port"], packet, commands)
        command_count += 1
        publish_overlay_status(
            args,
            capture,
            overlay_publisher,
            overlay_metrics,
            traj,
            q0,
            tracking_source,
            t_sec=t,
            command_count=command_count,
        )
        next_send += period
        time.sleep(max(0.0, next_send - time.monotonic()))
    stop = sim_bench.hold_command()
    send_udp(endpoints["command_host"], endpoints["command_port"], stop, commands)
    return first_ns, last_ns, command_count + 1, rows


def send_arm_motion(args: argparse.Namespace, endpoints: dict[str, Any], commands: CommandRecorder, capture: StateCapture) -> dict[str, Any]:
    packet = sim_bench.lifecycle_command("ArmMotion")
    send_udp(endpoints["command_host"], endpoints["command_port"], packet, commands)

    def armed(snapshot: dict[str, Any]) -> bool:
        return (
            int(snapshot.get("host_time_ns", -1)) >= int(packet["host_time_ns"])
            and snapshot.get("motion_state") in {"ArmedHold", "Running"}
            and snapshot.get("safety_verdict") == "Ok"
            and snapshot.get("fault_latched") is False
            and valid_rbpodo_state(snapshot, args.arm)
        )

    return wait_for(capture, armed, args.startup_timeout_sec, "rbpodo controller-simulation ArmMotion")


def collect_samples(
    states: list[dict[str, Any]],
    arm: str,
    source: str,
    start_ns: int,
    end_ns: int,
) -> list[dict[str, Any]]:
    samples: list[dict[str, Any]] = []
    for snapshot in states:
        host_time_ns = int(snapshot.get("host_time_ns", -1))
        if host_time_ns < start_ns or host_time_ns > end_ns:
            continue
        if not valid_rbpodo_state(snapshot, arm, source):
            continue
        arm_state = snapshot.get(arm, {})
        tel = arm_state.get("cartesian_solve") if isinstance(arm_state.get("cartesian_solve"), dict) else {}
        worker = arm_state.get("worker") if isinstance(arm_state.get("worker"), dict) else {}
        samples.append(
            {
                "host_time_ns": host_time_ns,
                "pose": pose_for_source(snapshot, arm, source),
                "arm_state": arm_state,
                "telemetry": tel,
                "worker": worker,
                "snapshot": snapshot,
            }
        )
    return samples


def pose_rows(
    states: list[dict[str, Any]],
    arm: str,
    source: str,
    benchmark_start_ns: int,
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for snapshot in states:
        host_time_ns = int(snapshot.get("host_time_ns", -1))
        if not source_valid(snapshot, arm, source):
            continue
        p = pose_for_source(snapshot, arm, source)
        rows.append(
            {
                "host_time_ns": host_time_ns,
                "t_sec": (host_time_ns - benchmark_start_ns) / 1e9 if benchmark_start_ns else None,
                "x": p["x"],
                "y": p["y"],
                "z": p["z"],
                "qx": p["quaternion_xyzw"][0],
                "qy": p["quaternion_xyzw"][1],
                "qz": p["quaternion_xyzw"][2],
                "qw": p["quaternion_xyzw"][3],
            }
        )
    return rows


def q_series(states: list[dict[str, Any]], arm: str, key: str) -> list[tuple[int, list[float]]]:
    out: list[tuple[int, list[float]]] = []
    for snapshot in states:
        arm_state = snapshot.get(arm)
        if not isinstance(arm_state, dict):
            continue
        values = arm_state.get(key)
        if not isinstance(values, list) or len(values) != 6:
            continue
        numbers = [finite_number(value) for value in values]
        if any(value is None for value in numbers):
            continue
        out.append((int(snapshot.get("host_time_ns", -1)), [float(value) for value in numbers if value is not None]))
    return out


def max_q_drift(series: list[tuple[int, list[float]]]) -> float | None:
    if not series:
        return None
    q0 = series[0][1]
    return max(max(abs(a - b) for a, b in zip(q, q0)) for _time_ns, q in series)


def q_update_rate_hz(series: list[tuple[int, list[float]]], epsilon_deg: float = 1e-6) -> float | None:
    if len(series) < 2:
        return None
    start_ns, _ = series[0]
    end_ns, _ = series[-1]
    if end_ns <= start_ns:
        return None
    changes = 0
    previous = series[0][1]
    for _time_ns, q in series[1:]:
        if max(abs(a - b) for a, b in zip(q, previous)) > epsilon_deg:
            changes += 1
            previous = q
    return changes / ((end_ns - start_ns) / 1e9)


def windowed_states(states: list[dict[str, Any]], start_ns: int, end_ns: int) -> list[dict[str, Any]]:
    if start_ns <= 0 or end_ns <= 0:
        return list(states)
    return [
        state for state in states
        if start_ns <= int(state.get("host_time_ns", -1)) <= end_ns
    ]


def max_pose_displacement(
    states: list[dict[str, Any]],
    arm: str,
    source: str,
    start_ns: int,
    end_ns: int,
) -> float | None:
    positions: list[list[float]] = []
    for snapshot in windowed_states(states, start_ns, end_ns):
        if not source_valid(snapshot, arm, source):
            continue
        positions.append(sim_bench.vec(pose_for_source(snapshot, arm, source)))
    if not positions:
        return None
    first = positions[0]
    return max(sim_bench.norm(sim_bench.sub(position, first)) for position in positions)


def cartesian_runtime_diagnostics(
    states: list[dict[str, Any]],
    arm: str,
    start_ns: int,
    end_ns: int,
) -> dict[str, Any]:
    status_counts: Counter[str] = Counter()
    reason_counts: Counter[str] = Counter()
    motion_state_counts: Counter[str] = Counter()
    attempted_count = 0
    success_count = 0
    max_command_errors: list[float] = []
    selected_states = windowed_states(states, start_ns, end_ns)
    for snapshot in selected_states:
        motion_state = snapshot.get("motion_state")
        if isinstance(motion_state, str) and motion_state:
            motion_state_counts[motion_state] += 1
        arm_state = snapshot.get(arm)
        if not isinstance(arm_state, dict):
            continue
        solve = arm_state.get("cartesian_solve")
        if not isinstance(solve, dict):
            continue
        status = str(solve.get("status") or solve.get("ik_status") or "")
        if status:
            status_counts[status] += 1
        reason = str(solve.get("reason") or solve.get("ik_reason") or "")
        if status == "unavailable":
            reason_counts[reason or "unknown"] += 1
        if solve.get("attempted") is True:
            attempted_count += 1
        if solve.get("success") is True:
            success_count += 1
        value = finite_number(solve.get("max_command_actual_error_deg_observed"))
        if value is not None:
            max_command_errors.append(value)
    return {
        "cartesian_status_counts": dict(status_counts),
        "cartesian_unavailable_count": int(status_counts.get("unavailable", 0)),
        "cartesian_unavailable_reason_counts": dict(reason_counts),
        "cartesian_attempted_count": attempted_count,
        "cartesian_success_count": success_count,
        "motion_state_counts": dict(motion_state_counts),
        "armed_hold_count": int(motion_state_counts.get("ArmedHold", 0)),
        "max_command_actual_error_deg_observed": max(max_command_errors) if max_command_errors else None,
        "tcp_ref_displacement_m": max_pose_displacement(states, arm, "tcp_ref_stand", start_ns, end_ns),
        "tcp_actual_displacement_m": max_pose_displacement(states, arm, "tcp_actual_stand", start_ns, end_ns),
    }


def telemetry_metrics(states: list[dict[str, Any]], arm: str) -> dict[str, Any]:
    arm_states = [state.get(arm) for state in states if isinstance(state.get(arm), dict)]
    state_ages: list[float] = []
    send_durations: list[float] = []
    ack_waits: list[float] = []
    ack_policies: Counter[str] = Counter()
    semantics: Counter[str] = Counter()
    error_names: Counter[str] = Counter()
    command_timeout_count = 0
    controller_rejected_count = 0
    ack_observed_count = 0
    controller_acceptance_count = 0
    diagnostics_suspect_count = 0
    override_active_count = 0
    for arm_state in arm_states:
        age = finite_number(arm_state.get("state_age_us"))
        if age is not None:
            state_ages.append(age)
        if arm_state.get("diagnostic_error_source") == "rbpodo_diagnostics_suspect":
            diagnostics_suspect_count += 1
        if arm_state.get("controller_simulation_diagnostic_override_active") is True:
            override_active_count += 1
        last_send = arm_state.get("last_send")
        if not isinstance(last_send, dict):
            continue
        duration = finite_number(last_send.get("duration_us"))
        if duration is None:
            duration = finite_number(last_send.get("send_duration_us"))
        if duration is not None:
            send_durations.append(duration)
        ack_wait = finite_number(last_send.get("ack_wait_duration_us"))
        if ack_wait is not None:
            ack_waits.append(ack_wait)
        if isinstance(last_send.get("ack_policy"), str):
            ack_policies[last_send["ack_policy"]] += 1
        if isinstance(last_send.get("send_acceptance_semantics"), str):
            semantics[last_send["send_acceptance_semantics"]] += 1
        for key in ("backend_error_name", "error_name"):
            if isinstance(last_send.get(key), str) and last_send[key] not in {"", "None"}:
                error_names[last_send[key]] += 1
        if last_send.get("ack_observed") is True:
            ack_observed_count += 1
        if last_send.get("controller_acceptance_observed") is True:
            controller_acceptance_count += 1
        kind = str(last_send.get("backend_error_kind", ""))
        if kind in {"CommandTimeout", "TransportTimeout"}:
            command_timeout_count += 1
        if kind == "ControllerRejected":
            controller_rejected_count += 1
    return {
        "state_age_us": metric_block(state_ages),
        "ack_policy_distribution": dict(ack_policies),
        "ack_observed_count": ack_observed_count,
        "controller_acceptance_observed_count": controller_acceptance_count,
        "send_acceptance_semantics_distribution": dict(semantics),
        "send_duration_us": metric_block(send_durations),
        "ack_wait_duration_us": metric_block(ack_waits),
        "command_timeout_count": command_timeout_count,
        "controller_rejected_count": controller_rejected_count,
        "response_error_names": dict(error_names),
        "diagnostics_suspect_count": diagnostics_suspect_count,
        "controller_simulation_diagnostic_override_active_count": override_active_count,
    }


def tcp_valid_ratio(states: list[dict[str, Any]], arm: str, source: str) -> float:
    arm_states = [state.get(arm) for state in states if isinstance(state.get(arm), dict)]
    if not arm_states:
        return 0.0
    valid_count = sum(1 for state in states if source_valid(state, arm, source))
    return valid_count / len(arm_states)


def command_interval_metrics(command_path: Path) -> dict[str, Any]:
    host_times: list[int] = []
    if not command_path.is_file():
        return {"command_interval_ms": metric_block([])}
    with command_path.open(encoding="utf-8") as handle:
        for line in handle:
            try:
                packet = json.loads(line)
            except json.JSONDecodeError:
                continue
            host_time_ns = packet.get("host_time_ns")
            if isinstance(host_time_ns, int):
                host_times.append(host_time_ns)
    intervals = [(b - a) / 1e6 for a, b in zip(host_times, host_times[1:]) if b >= a]
    return {"command_interval_ms": metric_block(intervals)}


def threshold_failures(args: argparse.Namespace, summary: dict[str, Any]) -> list[str]:
    return sim_bench.threshold_failures(args, summary)


def thresholds_requested(args: argparse.Namespace) -> bool:
    return sim_bench.thresholds_requested(args)


def result_from_thresholds(args: argparse.Namespace, summary: dict[str, Any]) -> tuple[str, str, list[str]]:
    failures = threshold_failures(args, summary)
    if summary.get("fault_latched") is True and thresholds_requested(args):
        failures.append("fault_latched was true during benchmark")
    result, reason = sim_bench.benchmark_result(thresholds_requested(args), failures)
    return result, reason, failures


def classify_cartesian_runtime(summary: dict[str, Any], command_count: int) -> dict[str, Any]:
    sample_count = int(summary.get("sample_count") or 0)
    unavailable_count = int(summary.get("cartesian_unavailable_count") or 0)
    high_unavailable = unavailable_count >= max(3, int(sample_count * 0.5))
    fit_reason = str(summary.get("circle_fit_reason") or "").lower()
    fit_singular = "singular" in fit_reason
    reason_counts = summary.get("cartesian_unavailable_reason_counts")
    has_cartesian_unavailable_reason = (
        isinstance(reason_counts, dict)
        and any(str(reason).startswith(CARTESIAN_UNAVAILABLE_PREFIX) for reason in reason_counts)
    )
    max_command_error = finite_number(summary.get("max_command_actual_error_deg_observed"))
    target_static = (
        command_count > 0
        and int(summary.get("controller_acceptance_observed_count") or 0) > 0
        and summary.get("q_ref_moved") is False
        and summary.get("tcp_ref_moved") is False
        and (max_command_error is None or max_command_error <= 1e-9)
    )
    server_rejected = (
        command_count > 0
        and high_unavailable
        and fit_singular
        and has_cartesian_unavailable_reason
    )
    return {
        "cartesian_unavailable_high": high_unavailable,
        "command_accepted_but_target_static": target_static,
        "server_rejected_cartesian": server_rejected,
        "cartesian_block_hint": CARTESIAN_REJECTION_HINTS if server_rejected else [],
    }


def write_summary_csv(path: Path, summary: dict[str, Any]) -> None:
    fields = [
        "result",
        "result_reason",
        "controller",
        "arm",
        "profile",
        "tracking_source_used",
        "diameter_m",
        "period_sec",
        "repeat",
        "command_rate_hz",
        "mean_error_m",
        "rms_error_m",
        "p95_error_m",
        "max_error_m",
        "radius_gain",
        "fit_center_error_m",
        "estimated_latency_ms",
        "max_orientation_drift_rad",
        "tcp_ref_valid_ratio",
        "tcp_actual_valid_ratio",
        "physical_motion_detected",
        "q_ref_update_rate_hz",
        "q_actual_update_rate_hz",
        "ack_observed_count",
        "controller_acceptance_observed_count",
        "command_timeout_count",
        "controller_rejected_count",
        "diagnostics_suspect_count",
        "cartesian_unavailable_count",
        "armed_hold_count",
        "command_accepted_but_target_static",
        "q_ref_moved",
        "tcp_ref_moved",
        "server_rejected_cartesian",
        "overlay_pub_endpoint",
        "overlay_messages_sent",
    ]
    write_csv(path, [{field: summary.get(field) for field in fields}], fields)


def plot_artifacts(
    artifact_dir: Path,
    args: argparse.Namespace,
    traj: sim_bench.Trajectory,
    merged: list[dict[str, Any]],
    controller_rows: list[dict[str, Any]],
    physical_rows: list[dict[str, Any]],
    no_circle_reason: str | None = None,
) -> list[str]:
    if args.skip_plots:
        return ["plots skipped by --skip-plots"]
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        return [f"matplotlib unavailable: {exc}"]
    skipped: list[str] = []
    if not merged:
        return ["no merged samples available for plotting"]

    def plane_xy(row: dict[str, Any]) -> tuple[float, float]:
        p = [float(row["x"]), float(row["y"]), float(row["z"])]
        rel = sim_bench.sub(p, traj.center)
        return sim_bench.dot(rel, traj.axis1), sim_bench.dot(rel, traj.axis2)

    desired = [(row["reference_x"], row["reference_y"], row["reference_z"]) for row in merged]
    desired_plane = [
        (sim_bench.dot(sim_bench.sub(list(p), traj.center), traj.axis1), sim_bench.dot(sim_bench.sub(list(p), traj.center), traj.axis2))
        for p in desired
    ]
    if controller_rows:
        controller_plane = [plane_xy(row) for row in controller_rows]
        plt.figure()
        plt.plot([p[0] for p in desired_plane], [p[1] for p in desired_plane], label="desired")
        plt.plot([p[0] for p in controller_plane], [p[1] for p in controller_plane], label="controller_reference")
        if no_circle_reason:
            plt.title(no_circle_reason)
        plt.gca().set_aspect("equal", adjustable="box")
        plt.legend()
        plt.tight_layout()
        plt.savefig(artifact_dir / "circle_trajectory_controller_reference.png")
        plt.close()
    if physical_rows:
        physical_plane = [plane_xy(row) for row in physical_rows]
        plt.figure()
        plt.plot([p[0] for p in desired_plane], [p[1] for p in desired_plane], label="desired")
        plt.plot([p[0] for p in physical_plane], [p[1] for p in physical_plane], label="physical_actual")
        if no_circle_reason:
            plt.title(no_circle_reason)
        plt.gca().set_aspect("equal", adjustable="box")
        plt.legend()
        plt.tight_layout()
        plt.savefig(artifact_dir / "circle_trajectory_physical_actual.png")
        plt.close()
    else:
        skipped.append("circle_trajectory_physical_actual.png skipped because tcp_actual_stand was unavailable")

    def line_plot(filename: str, key: str, ylabel: str) -> None:
        plt.figure()
        plt.plot([row["t_sec"] for row in merged], [row.get(key) for row in merged])
        if no_circle_reason:
            plt.title(no_circle_reason)
        plt.xlabel("time (s)")
        plt.ylabel(ylabel)
        plt.tight_layout()
        plt.savefig(artifact_dir / filename)
        plt.close()

    line_plot("tracking_error_time.png", "position_error_m", "position error (m)")
    line_plot("orientation_drift_time.png", "orientation_drift_rad", "orientation drift (rad)")
    if any("phase_lag_ms" in row for row in merged):
        line_plot("phase_lag_time.png", "phase_lag_ms", "phase lag (ms)")
    else:
        skipped.append("phase_lag_time.png skipped because phase estimate was unreliable")

    plt.figure()
    for axis in ("x", "y", "z"):
        plt.plot([row["t_sec"] for row in merged], [row[f"actual_{axis}"] for row in merged], label=f"tracking {axis}")
        plt.plot([row["t_sec"] for row in merged], [row[f"reference_{axis}"] for row in merged], linestyle="--", label=f"desired {axis}")
    if no_circle_reason:
        plt.title(no_circle_reason)
    plt.xlabel("time (s)")
    plt.ylabel("position (m)")
    plt.legend(ncol=2)
    plt.tight_layout()
    plt.savefig(artifact_dir / "axis_positions_time.png")
    plt.close()
    return skipped


def generated_plot_paths(artifact_dir: Path) -> list[str]:
    names = [
        "circle_trajectory_controller_reference.png",
        "circle_trajectory_physical_actual.png",
        "tracking_error_time.png",
        "orientation_drift_time.png",
        "phase_lag_time.png",
        "axis_positions_time.png",
    ]
    return [str((artifact_dir / name).resolve()) for name in names if (artifact_dir / name).is_file()]


def build_trajectory(args: argparse.Namespace, start_pose: dict[str, Any]) -> sim_bench.Trajectory:
    p0 = sim_bench.vec(start_pose)
    q0 = start_pose["quaternion_xyzw"]
    local_axis1, local_axis2 = sim_bench.axes_for_plane(args.plane)
    if args.controller in {"twist_local", "twist_local_feedback"}:
        rot = sim_bench.quat_to_matrix(q0)
        axis1 = sim_bench.mat_vec(rot, local_axis1)
        axis2 = sim_bench.mat_vec(rot, local_axis2)
    else:
        axis1, axis2 = local_axis1, local_axis2
    return sim_bench.Trajectory(
        start=p0,
        axis1=axis1,
        axis2=axis2,
        radius=float(args.diameter_m) * 0.5,
        period_sec=float(args.period_sec),
    )


def run_tracking_commands(
    args: argparse.Namespace,
    capture: StateCapture,
    commands: CommandRecorder,
    endpoints: dict[str, Any],
    traj: sim_bench.Trajectory,
    q0: list[float],
    tracking_source: str,
    duration_sec: float,
    overlay_publisher: benchmark_overlay.BenchmarkOverlayPublisher | None = None,
    overlay_metrics: benchmark_overlay.CircleOverlayMetrics | None = None,
) -> tuple[int, int, int, list[dict[str, Any]]]:
    if args.controller == "twist_stand":
        start_ns, end_ns, count = stream_twist(
            args,
            capture,
            commands,
            endpoints,
            traj,
            "stand",
            q0,
            tracking_source,
            duration_sec,
            overlay_publisher,
            overlay_metrics,
        )
        return start_ns, end_ns, count, []
    if args.controller == "twist_local":
        start_ns, end_ns, count = stream_twist(
            args,
            capture,
            commands,
            endpoints,
            traj,
            "local",
            q0,
            tracking_source,
            duration_sec,
            overlay_publisher,
            overlay_metrics,
        )
        return start_ns, end_ns, count, []
    if args.controller == "twist_stand_feedback":
        return stream_twist_feedback(
            args,
            capture,
            commands,
            endpoints,
            traj,
            "stand",
            q0,
            tracking_source,
            duration_sec,
            overlay_publisher,
            overlay_metrics,
        )
    if args.controller == "twist_local_feedback":
        return stream_twist_feedback(
            args,
            capture,
            commands,
            endpoints,
            traj,
            "local",
            q0,
            tracking_source,
            duration_sec,
            overlay_publisher,
            overlay_metrics,
        )
    raise BenchmarkError(f"controller {args.controller} is not implemented for rbpodo controller-simulation benchmark")


def summarize_run(
    args: argparse.Namespace,
    config: ParsedConfig,
    preflight_result: dict[str, Any],
    states: list[dict[str, Any]],
    traj: sim_bench.Trajectory,
    q0: list[float],
    tracking_source: str,
    tracking_source_warning: str | None,
    benchmark_start_ns: int,
    benchmark_end_ns: int,
    command_count: int,
    feedback_rows: list[dict[str, Any]],
    artifact_dir: Path,
    server_returncode: int | None,
    overlay_stats: dict[str, Any] | None = None,
) -> dict[str, Any]:
    overlay_stats = overlay_stats or {
        "overlay_enabled": False,
        "overlay_pub_endpoint": None,
        "overlay_messages_sent": 0,
        "overlay_messages_recorded": 0,
    }
    duration_sec = float(args.period_sec) * int(args.repeat)
    tracking_samples = collect_samples(states, args.arm, tracking_source, benchmark_start_ns, benchmark_start_ns + int(duration_sec * 1e9))
    if not tracking_samples:
        raise BenchmarkError(f"no valid {tracking_source} samples captured during benchmark")
    metrics, merged = sim_bench.compute_metrics(
        args=args,
        traj=traj,
        q0=q0,
        samples=tracking_samples,
        benchmark_start_ns=benchmark_start_ns,
    )
    feedback_artifacts = sim_bench.write_feedback_artifacts(artifact_dir, feedback_rows)
    if feedback_rows:
        metrics.update(sim_bench.feedback_metrics(feedback_rows))
    controller_rows = pose_rows(states, args.arm, "tcp_ref_stand", benchmark_start_ns)
    physical_rows = pose_rows(states, args.arm, "tcp_actual_stand", benchmark_start_ns)
    q_actual = q_series(states, args.arm, "q_actual_deg")
    q_ref = q_series(states, args.arm, "q_ref_deg")
    if not q_ref:
        q_ref = q_series(states, args.arm, "q_target_deg")
    q_ref_drift = max_q_drift(q_ref)
    q_actual_drift = max_q_drift(q_actual)
    runtime_diagnostics = cartesian_runtime_diagnostics(
        states,
        args.arm,
        benchmark_start_ns,
        benchmark_start_ns + int(duration_sec * 1e9),
    )
    q_ref_moved = q_ref_drift is not None and q_ref_drift > 1e-5
    tcp_ref_displacement = finite_number(runtime_diagnostics.get("tcp_ref_displacement_m"))
    tcp_ref_moved = tcp_ref_displacement is not None and tcp_ref_displacement > 1e-5
    physical_motion_detected = (
        q_actual_drift is not None and q_actual_drift > args.physical_motion_warning_deg
    )
    performance_warnings = sim_bench.performance_warnings(metrics)
    if tracking_source_warning:
        performance_warnings.append(tracking_source_warning)
    if physical_motion_detected:
        performance_warnings.append(
            f"physical q_actual drift {q_actual_drift:.6f} deg exceeded pgmode simulation warning threshold "
            f"{args.physical_motion_warning_deg:.6f} deg"
        )

    summary: dict[str, Any] = {
        "schema": SCHEMA,
        "repo_git_commit": sim_bench.git_commit(args.root),
        "result": "completed",
        "result_reason": "run completed without thresholds; performance pass/fail was not evaluated",
        "artifact_dir": str(artifact_dir.resolve()),
        "server_config": str(args.server_config.resolve()),
        "raw_config": str((artifact_dir / "raw_config.yaml").resolve()),
        "controller": args.controller,
        "arm": args.arm,
        "plane": args.plane,
        "profile": args.profile,
        "diameter_m": args.diameter_m,
        "radius_m": float(args.diameter_m) * 0.5,
        "reference_radius_m": float(args.diameter_m) * 0.5,
        "period_sec": args.period_sec,
        "repeat": args.repeat,
        "command_rate_hz": args.command_rate_hz,
        "required_tangential_speed_m_s": preflight_result["required_tangential_speed_m_s"],
        "duration_sec": duration_sec,
        "command_count": command_count,
        "benchmark_start_ns": benchmark_start_ns,
        "benchmark_end_ns": benchmark_end_ns,
        "tracking_source_requested": args.tracking_source,
        "tracking_source_used": tracking_source,
        "tracking_source_warning": tracking_source_warning,
        "state_packet_count": len(states),
        "invalid_state_packets": 0,
        "tcp_ref_valid_ratio": tcp_valid_ratio(states, args.arm, "tcp_ref_stand"),
        "tcp_actual_valid_ratio": tcp_valid_ratio(states, args.arm, "tcp_actual_stand"),
        "q_ref_update_rate_hz": q_update_rate_hz(q_ref),
        "q_actual_update_rate_hz": q_update_rate_hz(q_actual),
        "q_ref_drift_from_start_deg": q_ref_drift,
        "q_ref_moved": q_ref_moved,
        "q_actual_drift_from_start_deg": q_actual_drift,
        "tcp_ref_moved": tcp_ref_moved,
        "physical_motion_detected": physical_motion_detected,
        "physical_motion_warning_threshold_deg": args.physical_motion_warning_deg,
        "safety_preflight": preflight_result,
        "pgmode_summary": str((artifact_dir / "pgmode_summary.json").resolve()),
        "state_stream": str((artifact_dir / "state_stream.jsonl").resolve()),
        "command_packets": str((artifact_dir / "command_packets.jsonl").resolve()),
        "overlay_stream": overlay_stats.get("overlay_stream"),
        "overlay_pub_endpoint": overlay_stats.get("overlay_pub_endpoint"),
        "overlay_pub_rate_hz": overlay_stats.get("overlay_pub_rate_hz"),
        "overlay_run_id": overlay_stats.get("overlay_run_id"),
        "overlay_messages_sent": overlay_stats.get("overlay_messages_sent", 0),
        "overlay_messages_recorded": overlay_stats.get("overlay_messages_recorded", 0),
        "overlay_warning_count": overlay_stats.get("overlay_warning_count", 0),
        "overlay_last_warning": overlay_stats.get("overlay_last_warning"),
        "rb_servo_server_log": str((artifact_dir / "rb_servo_server.log").resolve()),
        "desired_reference_csv": str((artifact_dir / "desired_reference.csv").resolve()),
        "controller_reference_actual_csv": str((artifact_dir / "controller_reference_actual.csv").resolve()),
        "physical_actual_csv": str((artifact_dir / "physical_actual.csv").resolve()) if physical_rows else None,
        "feedback_terms": feedback_artifacts,
        "server_returncode": server_returncode,
        "caveat": (
            "rbpodo controller-simulation benchmark evidence; controller boxes are real, "
            "physical robot motion is not expected or approved"
        ),
    }
    summary.update(metrics)
    summary.update(telemetry_metrics(states, args.arm))
    summary.update(runtime_diagnostics)
    summary.update(command_interval_metrics(artifact_dir / "command_packets.jsonl"))
    summary.update(classify_cartesian_runtime(summary, command_count))
    no_circle_reason = (
        "No circle attempted: Cartesian unavailable"
        if summary.get("server_rejected_cartesian") is True
        else None
    )
    write_csv(artifact_dir / "desired_reference.csv", sim_bench.reference_rows(traj, q0, duration_sec, args.command_rate_hz))
    write_csv(artifact_dir / "controller_reference_actual.csv", controller_rows)
    if physical_rows:
        write_csv(artifact_dir / "physical_actual.csv", physical_rows)
    write_csv(artifact_dir / "samples.csv", merged)
    skipped_plots = plot_artifacts(artifact_dir, args, traj, merged, controller_rows, physical_rows, no_circle_reason)
    if summary.get("server_rejected_cartesian") is True:
        result = "blocked"
        result_reason = "cartesian_commands_rejected_by_server"
        failures = [
            "server rejected Cartesian command before attempting path; "
            "ServoJ ACKs only show hold-target sends, not circle tracking"
        ]
        performance_warnings.append(
            "server rejected Cartesian command before attempting path; check "
            "cartesian_control.allow_in_controller_simulation, "
            "RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN, operation_mode=simulation, "
            "and same-run pgmode confirmation"
        )
    else:
        result, result_reason, failures = result_from_thresholds(args, summary)
    summary.update(
        {
            "result": result,
            "result_reason": result_reason,
            "threshold_failures": failures,
            "performance_warnings": performance_warnings,
            "generated_plots": generated_plot_paths(artifact_dir),
            "skipped_plots": skipped_plots,
            "fault_latched": any(state.get("fault_latched") is True for state in states),
        }
    )
    write_json(artifact_dir / "summary.json", summary)
    write_summary_csv(artifact_dir / "summary.csv", summary)
    return summary


def run_benchmark(args: argparse.Namespace) -> dict[str, Any]:
    config, _sections, preflight_result, endpoints = preflight(args)
    artifact_dir = args.artifact_dir.resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    write_json(artifact_dir / "safety_preflight.json", preflight_result)
    write_json(artifact_dir / "pgmode_summary.json", preflight_result["pgmode_summary"])
    shutil.copy2(args.server_config, artifact_dir / "raw_config.yaml")
    if args.preflight_only:
        summary = {
            "schema": SCHEMA,
            "result": "completed",
            "result_reason": "preflight only; no tracking run or performance thresholds were evaluated",
            "artifact_dir": str(artifact_dir),
            "safety_preflight": preflight_result,
            "preflight_only": True,
            "overlay_enabled": not args.overlay_disable,
            "overlay_pub_endpoint": None if args.overlay_disable else args.overlay_pub_endpoint,
            "overlay_messages_sent": 0,
        }
        write_json(artifact_dir / "summary.json", summary)
        write_summary_csv(artifact_dir / "summary.csv", summary)
        return summary

    state_endpoint = preflight_result["state_endpoint"]
    capture = StateCapture(endpoints["state_host"], endpoints["state_port"], artifact_dir / "state_stream.jsonl")
    commands = CommandRecorder(artifact_dir / "command_packets.jsonl")
    server_proc: subprocess.Popen[str] | None = None
    benchmark_start_ns = 0
    benchmark_end_ns = 0
    command_count = 0
    feedback_rows: list[dict[str, Any]] = []
    tracking_source = "tcp_ref_stand"
    tracking_source_warning: str | None = None
    traj: sim_bench.Trajectory | None = None
    q0: list[float] | None = None
    overlay_publisher: benchmark_overlay.BenchmarkOverlayPublisher | None = None
    overlay_metrics: benchmark_overlay.CircleOverlayMetrics | None = None
    server_log_path = artifact_dir / "rb_servo_server.log"
    try:
        capture.start()
        server_proc = start_server(args, server_log_path, preflight_result)
        first_state = wait_for_state_source(capture, args, args.startup_timeout_sec, server_proc, server_log_path, state_endpoint)
        tracking_source, tracking_source_warning = select_tracking_source(args.tracking_source, first_state, args.arm)
        armed = send_arm_motion(args, endpoints, commands, capture)
        start_pose = pose_for_source(armed, args.arm, tracking_source)
        q0 = start_pose["quaternion_xyzw"]
        traj = build_trajectory(args, start_pose)
        overlay_publisher = benchmark_overlay.BenchmarkOverlayPublisher(
            endpoint=args.overlay_pub_endpoint,
            rate_hz=args.overlay_pub_rate_hz,
            run_id=args.overlay_run_id,
            artifact_path=artifact_dir / "overlay_stream.jsonl",
            enabled=not args.overlay_disable,
            warn=warn_overlay,
        )
        overlay_metrics = benchmark_overlay.CircleOverlayMetrics(
            center=traj.center,
            axis1=traj.axis1,
            axis2=traj.axis2,
            radius_m=traj.radius,
            omega_rad_s=traj.omega,
        )
        duration_sec = float(args.period_sec) * int(args.repeat)
        if args.warmup_sec > 0.0:
            time.sleep(args.warmup_sec)
        benchmark_start_ns, benchmark_end_ns, command_count, feedback_rows = run_tracking_commands(
            args,
            capture,
            commands,
            endpoints,
            traj,
            q0,
            tracking_source,
            duration_sec,
            overlay_publisher,
            overlay_metrics,
        )
        publish_overlay_status(
            args,
            capture,
            overlay_publisher,
            overlay_metrics,
            traj,
            q0,
            tracking_source,
            t_sec=duration_sec,
            command_count=command_count,
            result_so_far="completed",
            force=True,
        )
        if args.settle_sec > 0.0:
            time.sleep(args.settle_sec)
        benchmark_end_ns = max(benchmark_end_ns, sim_bench.seq())
    finally:
        commands.close()
        if overlay_publisher is not None:
            overlay_publisher.close()
        stop_server(server_proc)
        capture.stop()
    if benchmark_start_ns == 0 or traj is None or q0 is None:
        raise BenchmarkError("benchmark did not initialize tracking commands")
    server_returncode = server_proc.returncode if server_proc is not None else None
    return summarize_run(
        args,
        config,
        preflight_result,
        capture.snapshots,
        traj,
        q0,
        tracking_source,
        tracking_source_warning,
        benchmark_start_ns,
        benchmark_end_ns,
        command_count,
        feedback_rows,
        artifact_dir,
        server_returncode,
        overlay_publisher.summary() if overlay_publisher is not None else {
            "overlay_enabled": False,
            "overlay_pub_endpoint": None,
            "overlay_messages_sent": 0,
            "overlay_messages_recorded": 0,
        },
    )


def failure_summary(args: argparse.Namespace, exc: Exception) -> dict[str, Any]:
    artifact_dir = args.artifact_dir.resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    failure = {
        "schema": SCHEMA,
        "result": "error",
        "result_reason": "benchmark could not run",
        "error": str(exc),
        "server_config": str(args.server_config.resolve()) if args.server_config else None,
        "safety_preflight": {"passed": False, "error": str(exc), "env": benchmark_env_snapshot()},
        "threshold_failures": [str(exc)],
        "performance_warnings": [],
        "caveat": "rbpodo controller-simulation benchmark did not complete",
    }
    write_json(artifact_dir / "summary.json", failure)
    write_summary_csv(artifact_dir / "summary.csv", failure)
    return failure


def main() -> int:
    args = parse_args()
    try:
        summary = run_benchmark(args)
    except Exception as exc:
        failure_summary(args, exc)
        print(f"rbpodo_circle_tracking_benchmark: FAIL: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary.get("result") in {"completed", "pass"} else 2


if __name__ == "__main__":
    raise SystemExit(main())
