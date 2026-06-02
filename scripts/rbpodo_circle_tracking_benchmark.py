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
import error_decomposition
import generate_rbpodo_measurement_reliability_report as reliability_report
import timestamp_alignment_audit
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
    scalar_value,
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
PROFILE_DEFAULTS = sim_bench.PROFILE_DEFAULTS
REQUIRED_ENV = (
    "RB_ALLOW_REAL_ROBOT",
    "RB_ALLOW_REAL_MOTION",
    "RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION",
    "RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN",
)
DEFAULT_PHYSICAL_MOTION_WARNING_DEG = 0.05
MOTION_EPSILON_DEG = 1e-5
MOTION_EPSILON_M = 1e-5
INTEGRATOR_DIVERGENCE_WARNING_MIN = 10.0
CARTESIAN_UNAVAILABLE_PREFIX = "cartesian_control_unavailable"
CARTESIAN_REJECTION_HINTS = [
    "check cartesian_control.allow_in_controller_simulation: true",
    "check RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1",
    "check operation_mode: simulation",
    "check same-run pgmode simulation confirmation",
]
ACKON500_GOAL_THRESHOLDS = {
    "min_repeat": 5,
    "servo_rate_hz": 500.0,
    "servo_t1_sec": 0.002,
    "min_ack_ratio": 0.98,
    "min_effective_goal_command_rate_hz": 490.0,
    "max_effective_goal_command_rate_hz": 510.0,
    "max_saturation_ratio": 0.01,
    "max_rms_error_m": 0.003,
    "max_p95_error_m": 0.006,
    "max_fit_center_error_m": 0.003,
    "min_radius_gain": 0.98,
    "max_radius_gain": 1.02,
    "max_p95_orientation_drift_rad": 0.02,
    "max_effective_phase_latency_abs_ms": 5.0,
    "max_state_age_p95_us": 5000.0,
}
ACKON500_ACK_SEMANTICS = {"controller_ack_observed", "sdk_worker_ack_observed"}
RUN_FAILURE_STATUSES = {"error", "blocked", "faulted", "startup_fault"}


class BenchmarkError(RuntimeError):
    pass


class StartupFaultError(BenchmarkError):
    def __init__(self, message: str, details: dict[str, Any]) -> None:
        super().__init__(message)
        self.details = details


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
    parser.add_argument("--allow-fast-stress", action="store_true")
    parser.add_argument("--diameter-m", type=float)
    parser.add_argument("--period-sec", type=float)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--command-rate-hz", type=float, default=100.0)
    parser.add_argument("--phase-advance-sec", type=float, default=0.0)
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
    parser.add_argument("--tool-offset-m", default="0.03,0.05,0.10")
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


def finite_float_list(value: Any, length: int) -> list[float] | None:
    if not isinstance(value, list) or len(value) < length:
        return None
    numbers = [finite_number(item) for item in value[:length]]
    if any(item is None for item in numbers):
        return None
    return [float(item) for item in numbers if item is not None]


def angular_norm_from_twist(value: Any) -> float | None:
    twist = finite_float_list(value, 6)
    if twist is None:
        return None
    return sim_bench.norm(twist[3:6])


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


def tool_offsets_from_args(args: argparse.Namespace) -> list[float]:
    return error_decomposition.parse_tool_offsets(getattr(args, "tool_offset_m", "0.03,0.05,0.10"))


def trajectory_geometry(traj: sim_bench.Trajectory) -> dict[str, Any]:
    return {
        "center": list(traj.center),
        "axis1": list(traj.axis1),
        "axis2": list(traj.axis2),
        "radius": float(traj.radius),
    }


def required_speed(args: argparse.Namespace) -> float:
    assert args.diameter_m is not None and args.period_sec is not None
    return math.pi * args.diameter_m / args.period_sec


def endpoint_from_config(config: ParsedConfig, key: str, default: str) -> tuple[str, int, str]:
    value = config.network.get(key)
    if value is None and key == "state_pub_endpoint":
        endpoints = config.network.get("state_pub_endpoints")
        if endpoints is not None:
            if not isinstance(endpoints, list) or not endpoints:
                raise BenchmarkError("network.state_pub_endpoints must be a non-empty list")
            value = endpoints[0]
    endpoint = str(value if value is not None else default)
    host, port = parse_udp_endpoint(endpoint)
    return host, port, endpoint


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
                sections[current][key] = sim_bench.scalar_value(value)
                pending_list_key = None
            else:
                sections[current][key] = []
                pending_list_key = key
            continue
        if indent == 4 and pending_list_key is not None and text.startswith("- "):
            item = text[2:].strip()
            items = sections[current].setdefault(pending_list_key, [])
            if isinstance(items, list):
                items.append(scalar_value(item))
    return sections


def yaml_sections(path: Path) -> dict[str, dict[str, Any]]:
    try:
        import yaml  # type: ignore
    except Exception:
        return fallback_yaml_sections(path)
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise BenchmarkError(f"failed to parse YAML config {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise BenchmarkError("server config must be a YAML object")
    sections: dict[str, dict[str, Any]] = {}
    for key, value in data.items():
        if isinstance(value, dict):
            sections[str(key)] = value
    return sections


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


def finite_joint_values(value: Any) -> list[float] | None:
    if not isinstance(value, list) or len(value) != 6:
        return None
    numbers = [finite_number(item) for item in value]
    if any(item is None for item in numbers):
        return None
    return [float(item) for item in numbers if item is not None]


def state_excerpt(snapshot: dict[str, Any], arm: str) -> dict[str, Any]:
    top_keys = (
        "schema_version",
        "host_time_ns",
        "motion_state",
        "safety_verdict",
        "fault_latched",
        "latched_fault_reason",
        "fault_reason",
        "fault_context",
    )
    arm_keys = (
        "has_valid_joint_state",
        "diagnostic_error_source",
        "lifecycle_state",
        "q_actual_deg",
        "q_target_deg",
        "q_ref_deg",
        "q_sent_deg",
        "tcp_ref_valid",
        "tcp_actual_valid",
        "tracking_error_source",
        "tracking_error_source_valid",
        "command_reference_tracking_error_deg",
        "physical_command_actual_error_deg",
        "controller_simulation_physical_motion_detected",
        "controller_simulation_diagnostic_override_active",
    )
    excerpt = {key: snapshot.get(key) for key in top_keys if key in snapshot}
    arm_state = snapshot.get(arm)
    if isinstance(arm_state, dict):
        arm_excerpt = {key: arm_state.get(key) for key in arm_keys if key in arm_state}
        solve = arm_state.get("cartesian_solve")
        if isinstance(solve, dict):
            arm_excerpt["cartesian_solve"] = {
                key: solve.get(key)
                for key in (
                    "status",
                    "reason",
                    "cartesian_servo_state_source",
                    "cartesian_divergence_source",
                    "q_reference_for_servo_valid",
                    "command_reference_error_deg_observed",
                    "physical_command_actual_error_deg_observed",
                    "max_command_actual_error_deg_observed",
                    "integrator_resets_total",
                    "integrator_divergence_total",
                )
                if key in solve
            }
        worker = arm_state.get("worker")
        if isinstance(worker, dict):
            arm_excerpt["worker"] = {
                key: worker.get(key)
                for key in ("state", "last_error", "last_error_kind")
                if key in worker
            }
        excerpt[arm] = arm_excerpt
    return excerpt


def q_actual_target_error_summary(snapshot: dict[str, Any], arm: str) -> dict[str, Any]:
    arm_state = snapshot.get(arm)
    if not isinstance(arm_state, dict):
        return {"arm_state_available": False}
    q_actual = finite_joint_values(arm_state.get("q_actual_deg"))
    summary: dict[str, Any] = {
        "arm_state_available": True,
        "q_actual_available": q_actual is not None,
    }
    for key, label in (
        ("q_target_deg", "q_target"),
        ("q_ref_deg", "q_ref"),
        ("q_sent_deg", "q_sent"),
    ):
        q_other = finite_joint_values(arm_state.get(key))
        summary[f"{label}_available"] = q_other is not None
        if q_actual is not None and q_other is not None:
            errors = [abs(actual - target) for actual, target in zip(q_actual, q_other)]
            summary[f"q_actual_to_{label}_max_abs_error_deg"] = max(errors)
            summary[f"q_actual_to_{label}_error_deg"] = errors
        else:
            summary[f"q_actual_to_{label}_max_abs_error_deg"] = None
    return summary


def safety_tracking_excerpt(snapshot: dict[str, Any], arm: str) -> dict[str, Any]:
    arm_state = snapshot.get(arm)
    if not isinstance(arm_state, dict):
        return {}
    keys = (
        "tracking_error_source",
        "tracking_error_source_valid",
        "command_reference_tracking_error_deg",
        "physical_command_actual_error_deg",
        "controller_simulation_physical_motion_detected",
    )
    out = {key: arm_state.get(key) for key in keys if key in arm_state}
    solve = arm_state.get("cartesian_solve")
    if isinstance(solve, dict):
        for key in (
            "cartesian_servo_state_source",
            "cartesian_divergence_source",
            "command_reference_error_deg_observed",
            "physical_command_actual_error_deg_observed",
            "q_reference_for_servo_valid",
        ):
            if key in solve:
                out[key] = solve.get(key)
    return out


def tracking_error_source_hint(snapshot: dict[str, Any], arm: str) -> str | None:
    safety = safety_tracking_excerpt(snapshot, arm)
    source = safety.get("tracking_error_source")
    if isinstance(source, str) and source:
        return source
    solve_source = safety.get("cartesian_divergence_source")
    if isinstance(solve_source, str) and solve_source:
        return solve_source
    return None


def startup_fault_hints(snapshot: dict[str, Any], arm: str) -> list[str]:
    latched = str(snapshot.get("latched_fault_reason") or "")
    source = tracking_error_source_hint(snapshot, arm)
    hints: list[str] = []
    if latched == "TrackingError" and source == "reference":
        hints.extend(
            [
                "q_target/q_ref may differ from the startup previous target",
                "initialize prev_sent from reference before starting controller-simulation tracking",
                "reset pgmode simulation reference to q_actual before running",
            ]
        )
    elif latched == "TrackingError":
        hints.append("inspect q_actual/q_target startup tracking error before retrying")
    return hints


def server_returncode_text(proc: subprocess.Popen[str] | None) -> int | str | None:
    if proc is None:
        return None
    returncode = proc.poll()
    return returncode if returncode is not None else "still running"


def build_startup_fault_details(
    snapshot: dict[str, Any],
    args: argparse.Namespace,
    *,
    state_packets_received: int,
    proc: subprocess.Popen[str] | None,
    state_endpoint: str,
) -> dict[str, Any]:
    latched = snapshot.get("latched_fault_reason") or "unknown"
    fault_reason = snapshot.get("fault_reason") or "no fault reason reported"
    details = {
        "category": "startup_fault",
        "result": "startup_fault",
        "result_reason": f"startup fault latched: {latched}: {fault_reason}",
        "state_packets_received": state_packets_received,
        "state_endpoint": state_endpoint,
        "server_returncode": server_returncode_text(proc),
        "latched_fault_reason": snapshot.get("latched_fault_reason"),
        "fault_reason": snapshot.get("fault_reason"),
        "fault_context": snapshot.get("fault_context"),
        "safety_tracking": safety_tracking_excerpt(snapshot, args.arm),
        "q_actual_target_error_summary": q_actual_target_error_summary(snapshot, args.arm),
        "latest_state_excerpt": state_excerpt(snapshot, args.arm),
    }
    details["startup_fault_hints"] = startup_fault_hints(snapshot, args.arm)
    return details


def startup_fault_message(details: dict[str, Any]) -> str:
    parts = [
        "rb_servo_server published a fault-latched startup state",
        "state packets were received; this is not a state stream timeout",
        f"state_endpoint={details.get('state_endpoint')}",
        f"server_returncode={details.get('server_returncode')}",
        f"latched_fault_reason={details.get('latched_fault_reason')}",
        f"fault_reason={details.get('fault_reason')}",
    ]
    hints = details.get("startup_fault_hints")
    if isinstance(hints, list) and hints:
        parts.append("startup_fault_hints:")
        parts.extend(f"- {hint}" for hint in hints)
    return "\n".join(parts)


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
    sim_bench.validate_phase_advance(args, BenchmarkError)
    if sim_bench.profile_requires_fast_stress(args.profile) and not getattr(args, "allow_fast_stress", False):
        raise BenchmarkError("GENE-style 15 cm / 4 s stress requires --allow-fast-stress")
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
        if name in {"--warmup-sec", "--settle-sec", "--feedback-kp-pos", "--feedback-kp-ori"}:
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

    max_twist = as_float(cartesian.get("max_twist_linear_m_s"))
    max_twist_angular = as_float(cartesian.get("max_twist_angular_rad_s"))
    if max_twist is None:
        raise BenchmarkError("config must expose cartesian_control.max_twist_linear_m_s")
    if max_twist_angular is None:
        raise BenchmarkError("config must expose cartesian_control.max_twist_angular_rad_s")
    speed = required_speed(args)
    profile_metadata = sim_bench.benchmark_profile_metadata(args.profile)
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
        "profile_purpose": profile_metadata["purpose"],
        "profile_catalog_entry": profile_metadata,
        "stress_level": profile_metadata["stress_level"],
        "angular_frequency_rad_s": 2.0 * math.pi / float(args.period_sec),
        "recommended_controller": profile_metadata["recommended_controller"],
        "recommended_controllers": profile_metadata["recommended_controllers"],
        "controller": args.controller,
        "diameter_m": args.diameter_m,
        "period_sec": args.period_sec,
        "repeat": args.repeat,
        "command_rate_hz": args.command_rate_hz,
        **sim_bench.phase_advance_summary(args),
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
    sections = yaml_sections(args.server_config)
    if "servo" in sections:
        config.servo = sections["servo"]
    if "network" in sections:
        config.network = sections["network"]
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
    scanned_count = 0
    latest_snapshot: dict[str, Any] | None = None
    while True:
        snapshots = list(capture.snapshots)
        for snapshot in snapshots[scanned_count:]:
            latest_snapshot = snapshot
            if valid_rbpodo_state(snapshot, args.arm):
                return snapshot
        scanned_count = len(snapshots)
        if proc is not None and proc.poll() is not None:
            break
        if time.monotonic() >= deadline:
            break
        time.sleep(0.02)
    if latest_snapshot is not None:
        if latest_snapshot.get("fault_latched") is True:
            details = build_startup_fault_details(
                latest_snapshot,
                args,
                state_packets_received=scanned_count,
                proc=proc,
                state_endpoint=state_endpoint,
            )
            raise StartupFaultError(startup_fault_message(details), details)
        raise BenchmarkError(
            "\n".join(
                [
                    "received rb_servo_server state packets, but none were valid for rbpodo circle startup",
                    f"state_endpoint={state_endpoint}",
                    f"server_returncode={server_returncode_text(proc)}",
                    f"state_packets_received={scanned_count}",
                    "latest_state_excerpt:",
                    json.dumps(state_excerpt(latest_snapshot, args.arm), indent=2, sort_keys=True),
                ]
            )
        )
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
        command_t = sim_bench.command_sample_time(args, t)
        v1, v2 = traj.velocity_components(command_t)
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
            command_t = sim_bench.command_sample_time(args, t)
            command_ref = traj.position(command_t)
            measurement_ref = traj.position(t)
            row.update(
                {
                    "command_t_sec": command_t,
                    "phase_advance_sec": sim_bench.phase_advance_sec(args),
                    "command_reference_x": command_ref[0],
                    "command_reference_y": command_ref[1],
                    "command_reference_z": command_ref[2],
                    "measurement_reference_x": measurement_ref[0],
                    "measurement_reference_y": measurement_ref[1],
                    "measurement_reference_z": measurement_ref[2],
                }
            )
            row["orientation_error_norm_rad"] = None
            row["angular_feedback_norm_rad_s"] = None
            row["angular_applied_norm_rad_s"] = None
            row["angular_saturated"] = False
            row["feedback_state_age_us"] = None
            row["feedback_use_current_state_time"] = args.feedback_use_current_state_time
        else:
            state_ns = int(snapshot.get("host_time_ns", -1))
            if args.feedback_use_current_state_time and first_ns:
                t = max(0.0, (state_ns - first_ns) / 1e9)
            feedback_state_age_us = (now_ns - state_ns) / 1000.0 if state_ns >= 0 and now_ns >= state_ns else None
            tracking_pose = pose_for_source(snapshot, args.arm, tracking_source)
            p_actual = sim_bench.vec(tracking_pose)
            q_actual = tracking_pose["quaternion_xyzw"]
            command_t = sim_bench.command_sample_time(args, t)
            p_ref = traj.position(command_t)
            measurement_ref = traj.position(t)
            position_error_stand = sim_bench.sub(p_ref, p_actual)
            orientation_error_stand = sim_bench.quat_error_vector(q0, q_actual)
            feedback = sim_bench.compute_feedback_twist_stand(
                feedforward_linear_stand=sim_bench.trajectory_velocity_stand(traj, command_t),
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
            orientation_error_norm = sim_bench.norm(orientation_error_stand)
            angular_feedback_norm = angular_norm_from_twist(feedback["feedback_twist_stand"])
            angular_applied_norm = angular_norm_from_twist(feedback["applied_twist_stand"])
            angular_saturated = (
                angular_feedback_norm is not None
                and args.feedback_max_angular_rad_s is not None
                and angular_feedback_norm > float(args.feedback_max_angular_rad_s) + 1e-12
            )
            row = {
                "t_sec": t,
                "command_t_sec": command_t,
                "phase_advance_sec": sim_bench.phase_advance_sec(args),
                "frame": frame,
                "tracking_source": tracking_source,
                "feedback_skip_reason": "",
                "actual_x": p_actual[0],
                "actual_y": p_actual[1],
                "actual_z": p_actual[2],
                "reference_x": p_ref[0],
                "reference_y": p_ref[1],
                "reference_z": p_ref[2],
                "command_reference_x": p_ref[0],
                "command_reference_y": p_ref[1],
                "command_reference_z": p_ref[2],
                "measurement_reference_x": measurement_ref[0],
                "measurement_reference_y": measurement_ref[1],
                "measurement_reference_z": measurement_ref[2],
                "position_error_vector": position_error_stand,
                "orientation_error_vector": orientation_error_stand,
                "orientation_error_norm_rad": orientation_error_norm,
                "feedforward_twist": feedforward_twist,
                "feedback_twist": feedback_twist,
                "applied_twist": applied_twist,
                "feedforward_twist_stand": feedback["feedforward_twist_stand"],
                "feedback_twist_stand": feedback["feedback_twist_stand"],
                "applied_twist_stand": feedback["applied_twist_stand"],
                "angular_feedback_norm_rad_s": angular_feedback_norm,
                "angular_applied_norm_rad_s": angular_applied_norm,
                "angular_saturated": angular_saturated,
                "saturated": bool(feedback["saturated"]),
                "stale_or_invalid_state": False,
                "feedback_state_age_us": feedback_state_age_us,
                "feedback_use_current_state_time": args.feedback_use_current_state_time,
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


def run_server_circle(
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
    packet = sim_bench.circle_move_command(args, duration_sec)
    first_ns = int(packet["host_time_ns"])
    send_udp(endpoints["command_host"], endpoints["command_port"], packet, commands)
    start_monotonic = time.monotonic()
    next_overlay = start_monotonic
    overlay_period = 1.0 / max(1.0, float(args.overlay_pub_rate_hz))
    while True:
        now = time.monotonic()
        elapsed = now - start_monotonic
        if elapsed >= duration_sec:
            break
        if overlay_publisher is not None and overlay_metrics is not None and now >= next_overlay:
            publish_overlay_status(
                args,
                capture,
                overlay_publisher,
                overlay_metrics,
                traj,
                q0,
                tracking_source,
                t_sec=elapsed,
                command_count=1,
            )
            next_overlay = now + overlay_period
        if overlay_publisher is None or overlay_metrics is None:
            time.sleep(max(0.0, min(0.05, duration_sec - elapsed)))
            continue
        sleep_until = min(duration_sec, max(elapsed, next_overlay - start_monotonic))
        time.sleep(max(0.0, min(0.05, sleep_until - elapsed)))
    stop = sim_bench.hold_command()
    send_udp(endpoints["command_host"], endpoints["command_port"], stop, commands)
    return first_ns, int(stop["host_time_ns"]), 2, []


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


def arm_field_observed(states: list[dict[str, Any]], arm: str, key: str) -> bool:
    for snapshot in states:
        arm_state = snapshot.get(arm)
        if isinstance(arm_state, dict) and key in arm_state:
            return True
    return False


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


def q_motion_metrics(
    states: list[dict[str, Any]],
    arm: str,
    key: str,
    *,
    moved_epsilon_deg: float = MOTION_EPSILON_DEG,
) -> dict[str, Any]:
    observed = arm_field_observed(states, arm, key)
    series = q_series(states, arm, key)
    drift = max_q_drift(series) if observed else None
    moved = (drift > moved_epsilon_deg) if drift is not None and len(series) >= 2 else None
    if not observed:
        reason = f"{key} not published"
    elif not series:
        reason = f"{key} not valid"
    elif len(series) < 2:
        reason = f"{key} has fewer than two valid samples"
    else:
        reason = f"{key} published"
    prefix = key[:-4] if key.endswith("_deg") else key
    return {
        f"{prefix}_series": series,
        f"{prefix}_drift_from_start_deg": drift,
        f"{prefix}_moved": moved,
        f"{prefix}_update_rate_hz": q_update_rate_hz(series) if observed else None,
        f"{prefix}_reason": reason,
    }


def q_valid_ratio(states: list[dict[str, Any]], arm: str, key: str) -> float:
    arm_states = [state.get(arm) for state in states if isinstance(state.get(arm), dict)]
    if not arm_states:
        return 0.0
    valid_count = 0
    for arm_state in arm_states:
        value = arm_state.get(key)
        if isinstance(value, list) and len(value) == 6:
            parsed = [finite_number(item) for item in value]
            if all(item is not None for item in parsed):
                valid_count += 1
    return valid_count / len(arm_states)


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


def pose_moved(displacement_m: Any, epsilon_m: float = MOTION_EPSILON_M) -> bool | None:
    displacement = finite_number(displacement_m)
    if displacement is None:
        return None
    return displacement > epsilon_m


def cartesian_runtime_diagnostics(
    states: list[dict[str, Any]],
    arm: str,
    start_ns: int,
    end_ns: int,
) -> dict[str, Any]:
    status_counts: Counter[str] = Counter()
    reason_counts: Counter[str] = Counter()
    motion_state_counts: Counter[str] = Counter()
    servo_state_source_counts: Counter[str] = Counter()
    divergence_source_counts: Counter[str] = Counter()
    attempted_count = 0
    success_count = 0
    max_command_errors: list[float] = []
    command_reference_errors: list[float] = []
    physical_command_actual_errors: list[float] = []
    q_reference_valid_count = 0
    q_reference_observed_count = 0
    twist_clamp_count = 0
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
        if solve.get("twist_clamped") is True:
            twist_clamp_count += 1
        servo_source = solve.get("cartesian_servo_state_source")
        if isinstance(servo_source, str) and servo_source:
            servo_state_source_counts[servo_source] += 1
        divergence_source = solve.get("cartesian_divergence_source")
        if isinstance(divergence_source, str) and divergence_source:
            divergence_source_counts[divergence_source] += 1
        if "q_reference_for_servo_valid" in solve:
            q_reference_observed_count += 1
            if solve.get("q_reference_for_servo_valid") is True:
                q_reference_valid_count += 1
        value = finite_number(solve.get("max_command_actual_error_deg_observed"))
        if value is not None:
            max_command_errors.append(value)
        value = finite_number(solve.get("command_reference_error_deg_observed"))
        if value is not None:
            command_reference_errors.append(value)
        value = finite_number(solve.get("physical_command_actual_error_deg_observed"))
        if value is not None:
            physical_command_actual_errors.append(value)
    return {
        "cartesian_status_counts": dict(status_counts),
        "cartesian_unavailable_count": int(status_counts.get("unavailable", 0)),
        "cartesian_unavailable_reason_counts": dict(reason_counts),
        "cartesian_attempted_count": attempted_count,
        "cartesian_success_count": success_count,
        "cartesian_twist_clamp_count": twist_clamp_count,
        "cartesian_servo_state_source_counts": dict(servo_state_source_counts),
        "cartesian_divergence_source_counts": dict(divergence_source_counts),
        "q_reference_for_servo_valid_ratio": (
            q_reference_valid_count / q_reference_observed_count
            if q_reference_observed_count
            else None
        ),
        "motion_state_counts": dict(motion_state_counts),
        "armed_hold_count": int(motion_state_counts.get("ArmedHold", 0)),
        "max_command_actual_error_deg_observed": max(max_command_errors) if max_command_errors else None,
        "command_reference_error_deg_observed": max(command_reference_errors) if command_reference_errors else None,
        "physical_command_actual_error_deg_observed": max(physical_command_actual_errors) if physical_command_actual_errors else None,
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
    async_modes: Counter[str] = Counter()
    error_names: Counter[str] = Counter()
    command_timeout_count = 0
    controller_rejected_count = 0
    ack_observed_count = 0
    controller_acceptance_count = 0
    diagnostics_suspect_count = 0
    override_active_count = 0
    async_sample_count = 0
    async_enabled_observed = False
    async_counters = {
        "commands_enqueued_total": 0,
        "commands_sent_total": 0,
        "commands_acked_total": 0,
        "commands_socket_sent_total": 0,
        "commands_dropped_total": 0,
        "commands_overwritten_total": 0,
        "goal_window_commands_sent": 0,
        "goal_window_commands_acked": 0,
        "ack_timeout_count": 0,
        "missing_ack_count": 0,
        "reference_supervision_fault_count": 0,
    }
    async_timestamps = {
        "first_goal_command_send_ns": 0,
        "last_goal_command_send_ns": 0,
        "first_worker_send_ns": 0,
        "last_worker_send_ns": 0,
    }
    command_phases: Counter[str] = Counter()
    for arm_state in arm_states:
        age = finite_number(arm_state.get("state_age_us"))
        if age is not None:
            state_ages.append(age)
        if arm_state.get("diagnostic_error_source") == "rbpodo_diagnostics_suspect":
            diagnostics_suspect_count += 1
        if arm_state.get("controller_simulation_diagnostic_override_active") is True:
            override_active_count += 1
        async_streaming = arm_state.get("async_streaming")
        if isinstance(async_streaming, dict):
            async_sample_count += 1
            if async_streaming.get("enabled") is True:
                async_enabled_observed = True
            mode = async_streaming.get("mode")
            if isinstance(mode, str) and mode:
                async_modes[mode] += 1
            for key in async_counters:
                value = finite_number(async_streaming.get(key))
                if value is not None and value >= 0.0:
                    async_counters[key] = max(async_counters[key], int(value))
            for key in async_timestamps:
                value = finite_number(async_streaming.get(key))
                if value is None or value <= 0.0:
                    continue
                as_int = int(value)
                if key.startswith("first_"):
                    async_timestamps[key] = (
                        as_int
                        if async_timestamps[key] == 0
                        else min(async_timestamps[key], as_int)
                    )
                else:
                    async_timestamps[key] = max(async_timestamps[key], as_int)
            phase = async_streaming.get("command_phase")
            if isinstance(phase, str) and phase:
                command_phases[phase] += 1
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
        "async_streaming_metrics": {
            "sample_count": async_sample_count,
            "enabled_observed": async_enabled_observed,
            "mode": async_modes.most_common(1)[0][0] if async_modes else None,
            **async_counters,
            **async_timestamps,
            "command_phase": command_phases.most_common(1)[0][0] if command_phases else None,
            "command_phase_distribution": dict(command_phases),
        },
        "async_streaming_mode": async_modes.most_common(1)[0][0] if async_modes else None,
        **async_counters,
        **async_timestamps,
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


def orientation_error_vector_sample_rows(
    merged: list[dict[str, Any]],
    q0: list[float],
    *,
    max_samples: int = 12,
) -> list[dict[str, Any]]:
    candidates: list[dict[str, Any]] = []
    for row in merged:
        q_actual = [
            finite_number(row.get("actual_qx")),
            finite_number(row.get("actual_qy")),
            finite_number(row.get("actual_qz")),
            finite_number(row.get("actual_qw")),
        ]
        if any(value is None for value in q_actual):
            continue
        q_values = [float(value) for value in q_actual if value is not None]
        vector = sim_bench.quat_error_vector(q0, q_values)
        candidates.append(
            {
                "host_time_ns": row.get("host_time_ns"),
                "t_sec": row.get("t_sec"),
                "orientation_error_vector_stand": vector,
                "orientation_error_norm_rad": sim_bench.norm(vector),
            }
        )
    if len(candidates) <= max_samples:
        return candidates
    if max_samples <= 1:
        return candidates[:1]
    indices = [
        round(index * (len(candidates) - 1) / (max_samples - 1))
        for index in range(max_samples)
    ]
    return [candidates[index] for index in indices]


def feedback_orientation_command_metrics(
    rows: list[dict[str, Any]],
    *,
    max_angular_rad_s: float | None,
) -> dict[str, Any]:
    feedback_norms: list[float] = []
    applied_norms: list[float] = []
    angular_saturation_count = 0
    for row in rows:
        if row.get("stale_or_invalid_state") is True:
            continue
        feedback_norm = finite_number(row.get("angular_feedback_norm_rad_s"))
        if feedback_norm is None:
            feedback_norm = angular_norm_from_twist(row.get("feedback_twist_stand"))
        applied_norm = finite_number(row.get("angular_applied_norm_rad_s"))
        if applied_norm is None:
            applied_norm = angular_norm_from_twist(row.get("applied_twist_stand"))
        if feedback_norm is not None:
            feedback_norms.append(feedback_norm)
        if applied_norm is not None:
            applied_norms.append(applied_norm)
        saturated = row.get("angular_saturated") is True
        if not saturated and feedback_norm is not None and max_angular_rad_s is not None:
            saturated = feedback_norm > max_angular_rad_s + 1e-12
        if saturated:
            angular_saturation_count += 1

    feedback_block = metric_block(feedback_norms)
    applied_block = metric_block(applied_norms)
    denominator = len(feedback_norms)
    return {
        "angular_feedback_norm": feedback_block,
        "angular_applied_norm": applied_block,
        "angular_feedback_norm_p50": feedback_block["p50"],
        "angular_feedback_norm_p95": feedback_block["p95"],
        "angular_feedback_norm_max": feedback_block["max"],
        "angular_applied_norm_p50": applied_block["p50"],
        "angular_applied_norm_p95": applied_block["p95"],
        "angular_applied_norm_max": applied_block["max"],
        "angular_saturation_count": angular_saturation_count,
        "angular_saturation_ratio": (
            angular_saturation_count / denominator
            if denominator
            else None
        ),
    }


def config_section(config: ParsedConfig | None, name: str) -> dict[str, Any]:
    value = getattr(config, name, None)
    return value if isinstance(value, dict) else {}


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


def near(actual: Any, expected: float, tolerance: float = 1e-9) -> bool:
    number = finite_number(actual)
    return number is not None and abs(number - expected) <= tolerance


def text_list(value: Any) -> list[str]:
    if value is None:
        return []
    if isinstance(value, list):
        return [str(item) for item in value]
    if isinstance(value, tuple):
        return [str(item) for item in value]
    if isinstance(value, str):
        return [value] if value else []
    return [str(value)]


def count_from_distribution(summary: dict[str, Any], name: str) -> int:
    distribution = summary.get("send_acceptance_semantics_distribution")
    if not isinstance(distribution, dict):
        return 0
    value = finite_number(distribution.get(name))
    return int(value) if value is not None and value >= 0.0 else 0


def most_likely_acceptance_semantics(summary: dict[str, Any]) -> str | None:
    explicit = summary.get("acceptance_semantics") or summary.get("ack_semantics")
    if isinstance(explicit, str) and explicit:
        if summary.get("async_mode") == "sdk_ack_worker" and explicit == "controller_ack_observed":
            return "sdk_worker_ack_observed"
        return explicit
    async_mode = summary.get("async_mode") or summary.get("async_streaming_mode")
    if async_mode == "sdk_ack_worker" and int(finite_number(summary.get("commands_acked_total")) or 0) > 0:
        return "sdk_worker_ack_observed"
    if int(finite_number(summary.get("socket_send_only_count")) or 0) > 0:
        return "socket_send_only"
    if count_from_distribution(summary, "controller_ack_observed") > 0:
        return "controller_ack_observed"
    if count_from_distribution(summary, "socket_send_only") > 0:
        return "socket_send_only"
    return None


def positive_int(value: Any) -> int | None:
    number = finite_number(value)
    if number is None or number <= 0.0:
        return None
    return int(round(number))


def official_tracking_window(summary: dict[str, Any]) -> tuple[int | None, int | None, float | None]:
    window_sec = finite_number(summary.get("official_tracking_window_sec"))
    start_ns = positive_int(summary.get("official_tracking_start_ns"))
    end_ns = positive_int(summary.get("official_tracking_end_ns"))
    if start_ns is None:
        start_ns = positive_int(summary.get("benchmark_start_ns"))
    if window_sec is None:
        duration_sec = finite_number(summary.get("duration_sec"))
        if duration_sec is not None and duration_sec > 0.0:
            window_sec = duration_sec
    if end_ns is None and start_ns is not None and window_sec is not None:
        end_ns = start_ns + int(round(window_sec * 1e9))
    if window_sec is None and start_ns is not None and end_ns is not None and end_ns > start_ns:
        window_sec = (end_ns - start_ns) / 1e9
    return start_ns, end_ns, window_sec


def snapshot_time_ns(snapshot: dict[str, Any]) -> int | None:
    for key in ("loop_start_time_ns", "host_time_ns", "loop_end_time_ns"):
        value = positive_int(snapshot.get(key))
        if value is not None:
            return value
    return None


def server_servo_tick_count(states: list[dict[str, Any]], start_ns: int | None, end_ns: int | None) -> int | None:
    if start_ns is None or end_ns is None or end_ns <= start_ns:
        return None
    ticks: list[int] = []
    for snapshot in states:
        time_ns = snapshot_time_ns(snapshot)
        if time_ns is None or time_ns < start_ns or time_ns >= end_ns:
            continue
        tick = positive_int(snapshot.get("tick"))
        if tick is not None:
            ticks.append(tick)
    if len(ticks) >= 2:
        return max(ticks) - min(ticks) + 1
    return None


def copy_async_rate_fields(summary: dict[str, Any], async_metrics: dict[str, Any]) -> None:
    aliases = {
        "async_commands_enqueued_total": "commands_enqueued_total",
        "async_commands_sent_total": "commands_sent_total",
        "async_commands_acked_total": "commands_acked_total",
    }
    for alias, legacy in aliases.items():
        value = summary.get(alias)
        if value is None:
            value = summary.get(legacy)
        if value is None:
            value = async_metrics.get(alias)
        if value is None:
            value = async_metrics.get(legacy)
        number = positive_int(value)
        if number is not None:
            summary[alias] = number
            if summary.get(legacy) is None:
                summary[legacy] = number


def annotate_goal_derived_fields(summary: dict[str, Any]) -> None:
    async_metrics = summary.get("async_streaming_metrics")
    if not isinstance(async_metrics, dict):
        async_metrics = {}
    if summary.get("async_mode") in (None, ""):
        summary["async_mode"] = summary.get("async_streaming_mode") or async_metrics.get("mode") or "disabled"
    for key in (
        "commands_sent_total",
        "commands_acked_total",
        "commands_socket_sent_total",
        "commands_enqueued_total",
        "commands_dropped_total",
        "commands_overwritten_total",
    ):
        if summary.get(key) is None and async_metrics.get(key) is not None:
            summary[key] = async_metrics.get(key)
    copy_async_rate_fields(summary, async_metrics)
    socket_count = finite_number(summary.get("socket_send_only_count"))
    if socket_count is None:
        socket_count = finite_number(summary.get("commands_socket_sent_total"))
    if socket_count is None:
        socket_count = float(count_from_distribution(summary, "socket_send_only"))
    summary["socket_send_only_count"] = int(socket_count or 0)
    semantics = most_likely_acceptance_semantics(summary)
    if semantics:
        summary["acceptance_semantics"] = semantics

    commands_sent = finite_number(summary.get("commands_sent_total"))
    if commands_sent is None or commands_sent <= 0.0:
        commands_sent = finite_number(summary.get("command_count"))
    commands_acked = finite_number(summary.get("commands_acked_total"))
    if commands_acked is None or commands_acked <= 0.0:
        commands_acked = finite_number(summary.get("ack_observed_count"))
    if commands_acked is None or commands_acked <= 0.0:
        commands_acked = finite_number(summary.get("controller_acceptance_observed_count"))
    if commands_sent is not None:
        summary["commands_sent_total"] = int(commands_sent)
    if commands_acked is not None:
        summary["commands_acked_total"] = int(commands_acked)
    summary["ack_observed_ratio"] = (
        commands_acked / commands_sent
        if commands_sent is not None and commands_sent > 0.0 and commands_acked is not None
        else None
    )
    start_ns, end_ns, official_window_sec = official_tracking_window(summary)
    if start_ns is not None:
        summary["official_tracking_start_ns"] = start_ns
    if end_ns is not None:
        summary["official_tracking_end_ns"] = end_ns
    if official_window_sec is not None:
        summary["official_tracking_window_sec"] = official_window_sec
    servo_rate = finite_number(summary.get("servo_rate_hz"))
    if servo_rate is not None:
        summary["official_servo_rate_hz"] = servo_rate
    if servo_rate is not None and official_window_sec is not None:
        summary["expected_servo_ticks"] = int(round(servo_rate * official_window_sec))

    for key in (
        "first_goal_command_send_ns",
        "last_goal_command_send_ns",
        "first_worker_send_ns",
        "last_worker_send_ns",
    ):
        if summary.get(key) is None and async_metrics.get(key) is not None:
            summary[key] = async_metrics.get(key)

    goal_sent = finite_number(summary.get("goal_window_commands_sent"))
    if goal_sent is None or goal_sent <= 0.0:
        goal_sent = finite_number(async_metrics.get("goal_window_commands_sent"))
    goal_acked = finite_number(summary.get("goal_window_commands_acked"))
    if goal_acked is None or goal_acked <= 0.0:
        goal_acked = finite_number(async_metrics.get("goal_window_commands_acked"))
    goal_count_source = summary.get("goal_window_count_source")
    if (
        (goal_sent is None or goal_sent <= 0.0)
        and finite_number(summary.get("server_servo_tick_count")) is not None
        and int(finite_number(summary.get("commands_dropped_total")) or 0) == 0
        and int(finite_number(summary.get("commands_overwritten_total")) or 0) == 0
    ):
        goal_sent = finite_number(summary.get("server_servo_tick_count"))
        if goal_sent is not None and commands_acked is not None and commands_acked >= goal_sent:
            goal_acked = goal_sent
        goal_count_source = "server_servo_tick_count_inferred"
    if goal_sent is not None and goal_sent > 0.0:
        summary["goal_window_commands_sent"] = int(goal_sent)
        if not goal_count_source:
            summary["goal_window_count_source"] = "async_worker_telemetry"
        else:
            summary["goal_window_count_source"] = goal_count_source
    if goal_acked is not None and goal_acked >= 0.0:
        summary["goal_window_commands_acked"] = int(goal_acked)
    summary["effective_goal_command_rate_hz"] = (
        goal_sent / official_window_sec
        if goal_sent is not None and goal_sent > 0.0 and official_window_sec is not None and official_window_sec > 0.0
        else None
    )
    summary["ack_coverage_ratio"] = (
        goal_acked / goal_sent
        if goal_sent is not None and goal_sent > 0.0 and goal_acked is not None
        else None
    )
    worker_first_ns = positive_int(summary.get("first_worker_send_ns"))
    worker_last_ns = positive_int(summary.get("last_worker_send_ns"))
    worker_window_sec = None
    if worker_first_ns is not None and worker_last_ns is not None and worker_last_ns > worker_first_ns:
        worker_window_sec = (worker_last_ns - worker_first_ns) / 1e9
    if worker_window_sec is not None:
        summary["measured_worker_window_sec"] = worker_window_sec
        summary["worker_active_window_sec"] = worker_window_sec
    async_sent_total = finite_number(summary.get("async_commands_sent_total"))
    if async_sent_total is None:
        async_sent_total = finite_number(summary.get("commands_sent_total"))
    worker_send_rate = (
        async_sent_total / worker_window_sec
        if async_sent_total is not None and async_sent_total > 0.0 and worker_window_sec is not None and worker_window_sec > 0.0
        else None
    )
    summary["worker_send_rate_hz"] = worker_send_rate
    summary["worker_lifetime_send_rate_hz"] = worker_send_rate
    if async_sent_total is not None and goal_sent is not None:
        outside = int(async_sent_total) - int(goal_sent)
        summary["worker_sends_outside_official_window"] = outside
        summary["worker_sends_outside_official_window_detected"] = outside != 0
    saturation_count = finite_number(summary.get("feedback_saturation_count"))
    command_count = finite_number(summary.get("command_count"))
    saturation_denominator = max(command_count or 0.0, commands_sent or 0.0)
    summary["feedback_saturation_ratio"] = (
        saturation_count / saturation_denominator
        if saturation_count is not None and saturation_denominator > 0.0
        else finite_number(summary.get("feedback_saturation_ratio"))
    )
    estimated_latency_ms = finite_number(summary.get("estimated_latency_ms"))
    summary["effective_phase_latency_abs_ms"] = (
        abs(estimated_latency_ms)
        if estimated_latency_ms is not None
        else finite_number(summary.get("effective_phase_latency_abs_ms"))
    )
    state_age = summary.get("state_age_us")
    if isinstance(state_age, dict):
        state_age_p95 = finite_number(state_age.get("p95"))
        if state_age_p95 is not None:
            summary["state_age_p95_us"] = state_age_p95


def safety_result(summary: dict[str, Any], run_status: str) -> dict[str, Any]:
    fault_latched = summary.get("fault_latched") is True or run_status in {"faulted", "startup_fault"}
    physical_motion = summary.get("physical_motion_detected") is True
    cartesian_unavailable = int(finite_number(summary.get("cartesian_unavailable_count")) or 0)
    status = "fail" if fault_latched or physical_motion or cartesian_unavailable > 0 or run_status in {"error", "blocked"} else "pass"
    return {
        "fault_latched": fault_latched,
        "physical_motion_detected": physical_motion,
        "cartesian_unavailable_count": cartesian_unavailable,
        "status": status,
    }


def benchmark_threshold_result(
    threshold_failures_value: list[str],
    threshold_warnings: list[str],
    thresholds_were_requested: bool,
) -> dict[str, Any]:
    if threshold_failures_value:
        status = "fail"
    elif thresholds_were_requested:
        status = "pass"
    else:
        status = "not_evaluated"
    return {
        "status": status,
        "threshold_failures": list(threshold_failures_value),
        "threshold_warnings": list(threshold_warnings),
    }


def diagnostic_warnings(summary: dict[str, Any], threshold_failures_value: list[str], threshold_warnings: list[str]) -> list[str]:
    warnings: list[str] = []
    joined = " ".join([*threshold_failures_value, *threshold_warnings])
    if "max_orientation_drift_rad" in joined or "orientation_drift_high" in joined:
        warnings.append("max_orientation_drift_spike")
    if (finite_number(summary.get("controller_simulation_diagnostic_override_active_count")) or 0.0) > 0.0:
        warnings.append("diagnostics_suspect_override_active")
    elif (finite_number(summary.get("diagnostics_suspect_count")) or 0.0) > 0.0:
        warnings.append("diagnostics_suspect_override_active")
    if summary.get("tracking_source_used") == "tcp_ref_stand" or summary.get("tracking_source") == "tcp_ref_stand":
        warnings.append("controller_reference_lower_bound")
    if (finite_number(summary.get("max_over_p95")) or 0.0) >= 3.0 or (finite_number(summary.get("tail_ratio")) or 0.0) >= 3.0:
        warnings.append("max_error_spike")
    timing = summary.get("timing_classification")
    if timing not in (None, "", "clean_timing"):
        warnings.append("timing_spike")
    for key in ("ack_spike_count_10ms", "ack_spike_count_20ms", "state_gap_count", "command_gap_count"):
        if (finite_number(summary.get(key)) or 0.0) > 0.0:
            warnings.append("timing_spike")
            break
    if (finite_number(summary.get("feedback_saturation_count")) or 0.0) > 0.0:
        warnings.append("feedback_saturation")
    return list(dict.fromkeys(warnings))


def check_min(summary: dict[str, Any], key: str, minimum: float, failures: list[str]) -> None:
    value = finite_number(summary.get(key))
    if value is None:
        failures.append(f"{key} unavailable")
    elif value < minimum:
        failures.append(f"{key} {value:.9g} < {minimum:.9g}")


def check_max(summary: dict[str, Any], key: str, maximum: float, failures: list[str]) -> None:
    value = finite_number(summary.get(key))
    if value is None:
        failures.append(f"{key} unavailable")
    elif value > maximum:
        failures.append(f"{key} {value:.9g} > {maximum:.9g}")


def ackon500_goal_result(summary: dict[str, Any], warnings: list[str]) -> dict[str, Any]:
    if summary.get("profile") != "gene_15cm_4s" or summary.get("preflight_only") is True:
        return {
            "evaluated": False,
            "status": "not_applicable",
            "failures": [],
            "warnings": list(warnings),
        }

    failures: list[str] = []
    check_min(summary, "repeat", ACKON500_GOAL_THRESHOLDS["min_repeat"], failures)
    tracking_source = summary.get("tracking_source_used") or summary.get("tracking_source")
    if tracking_source != "tcp_ref_stand":
        failures.append(f"tracking_source {tracking_source} != tcp_ref_stand")
    if not near(summary.get("servo_rate_hz"), ACKON500_GOAL_THRESHOLDS["servo_rate_hz"]):
        failures.append(f"servo_rate_hz {summary.get('servo_rate_hz')} != 500")
    if not near(summary.get("servo_t1_sec"), ACKON500_GOAL_THRESHOLDS["servo_t1_sec"]):
        failures.append(f"servo_t1_sec {summary.get('servo_t1_sec')} != 0.002")
    semantics = summary.get("acceptance_semantics")
    if semantics not in ACKON500_ACK_SEMANTICS:
        failures.append(f"acceptance_semantics {semantics} is not ACK-observed")
    check_min(summary, "official_tracking_window_sec", 1e-9, failures)
    check_min(summary, "goal_window_commands_sent", 1.0, failures)
    check_min(summary, "ack_coverage_ratio", ACKON500_GOAL_THRESHOLDS["min_ack_ratio"], failures)
    if int(finite_number(summary.get("socket_send_only_count")) or 0) != 0:
        failures.append(f"socket_send_only_count {summary.get('socket_send_only_count')} != 0")
    check_min(
        summary,
        "effective_goal_command_rate_hz",
        ACKON500_GOAL_THRESHOLDS["min_effective_goal_command_rate_hz"],
        failures,
    )
    check_max(
        summary,
        "effective_goal_command_rate_hz",
        ACKON500_GOAL_THRESHOLDS["max_effective_goal_command_rate_hz"],
        failures,
    )
    if summary.get("fault_latched") is not False:
        failures.append(f"fault_latched {summary.get('fault_latched')} is not false")
    if summary.get("physical_motion_detected") is not False:
        failures.append(f"physical_motion_detected {summary.get('physical_motion_detected')} is not false")
    if summary.get("physical_motion_expected") is not False:
        failures.append(f"physical_motion_expected {summary.get('physical_motion_expected')} is not false")
    if int(finite_number(summary.get("cartesian_unavailable_count")) or 0) != 0:
        failures.append(f"cartesian_unavailable_count {summary.get('cartesian_unavailable_count')} != 0")
    check_max(summary, "feedback_saturation_ratio", ACKON500_GOAL_THRESHOLDS["max_saturation_ratio"], failures)
    check_max(summary, "rms_error_m", ACKON500_GOAL_THRESHOLDS["max_rms_error_m"], failures)
    check_max(summary, "p95_error_m", ACKON500_GOAL_THRESHOLDS["max_p95_error_m"], failures)
    check_max(summary, "fit_center_error_m", ACKON500_GOAL_THRESHOLDS["max_fit_center_error_m"], failures)
    check_min(summary, "radius_gain", ACKON500_GOAL_THRESHOLDS["min_radius_gain"], failures)
    check_max(summary, "radius_gain", ACKON500_GOAL_THRESHOLDS["max_radius_gain"], failures)
    check_max(summary, "p95_orientation_drift_rad", ACKON500_GOAL_THRESHOLDS["max_p95_orientation_drift_rad"], failures)
    check_max(summary, "effective_phase_latency_abs_ms", ACKON500_GOAL_THRESHOLDS["max_effective_phase_latency_abs_ms"], failures)
    check_max(summary, "state_age_p95_us", ACKON500_GOAL_THRESHOLDS["max_state_age_p95_us"], failures)
    if summary.get("measurement_reliability_level") == "unreliable":
        failures.append("measurement_reliability_level is unreliable")
    return {
        "evaluated": True,
        "status": "fail" if failures else "pass",
        "failures": failures,
        "warnings": list(warnings),
    }


def apply_result_contract(
    summary: dict[str, Any],
    *,
    run_status: str,
    run_reason: str,
    threshold_failures_value: list[str] | None = None,
    threshold_warnings: list[str] | None = None,
    thresholds_were_requested: bool = False,
    legacy_threshold_failures_value: list[str] | None = None,
) -> None:
    threshold_failures_list = list(threshold_failures_value or [])
    legacy_threshold_failures = (
        list(legacy_threshold_failures_value)
        if legacy_threshold_failures_value is not None
        else threshold_failures_list
    )
    threshold_warnings_list = list(threshold_warnings or [])
    annotate_goal_derived_fields(summary)
    diagnostics = diagnostic_warnings(summary, threshold_failures_list, threshold_warnings_list)
    benchmark_result_block = benchmark_threshold_result(
        threshold_failures_list,
        threshold_warnings_list,
        thresholds_were_requested,
    )
    safety = safety_result(summary, run_status)
    goal = ackon500_goal_result(summary, diagnostics)
    summary.update(
        {
            "run_result": {"status": run_status, "reason": run_reason},
            "safety_result": safety,
            "benchmark_threshold_result": benchmark_result_block,
            "ackon500_goal_result": goal,
            "diagnostic_warnings": diagnostics,
            "run_result_status": run_status,
            "run_result_reason": run_reason,
            "safety_result_status": safety["status"],
            "benchmark_threshold_status": benchmark_result_block["status"],
            "ackon500_goal_status": goal["status"],
            "diagnostic_warning_count": len(diagnostics),
            "goal_pass": goal["status"] == "pass" if goal.get("evaluated") else None,
            "result": run_status,
            "result_reason": run_reason,
            "threshold_failures": legacy_threshold_failures,
            "threshold_warnings": threshold_warnings_list,
        }
    )


def first_fault_details(states: list[dict[str, Any]], benchmark_start_ns: int) -> dict[str, Any]:
    for snapshot in states:
        if snapshot.get("fault_latched") is not True:
            continue
        fault_time_ns = snapshot.get("host_time_ns")
        if not isinstance(fault_time_ns, int):
            fault_time_ns = snapshot.get("loop_end_time_ns")
        first_fault_time_sec = None
        if isinstance(fault_time_ns, int) and fault_time_ns >= benchmark_start_ns:
            first_fault_time_sec = (fault_time_ns - benchmark_start_ns) / 1e9
        return {
            "fault_latched": True,
            "first_fault_time_sec": first_fault_time_sec,
            "latched_fault_reason": snapshot.get("latched_fault_reason"),
            "fault_reason": snapshot.get("fault_reason"),
            "fault_context": snapshot.get("fault_context"),
        }
    return {
        "fault_latched": False,
        "first_fault_time_sec": None,
        "latched_fault_reason": None,
        "fault_reason": None,
        "fault_context": None,
    }


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
        and summary.get("tcp_ref_moved") is False
        and summary.get("q_sent_moved") is not True
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
        "run_result_status",
        "run_result_reason",
        "safety_result_status",
        "benchmark_threshold_status",
        "ackon500_goal_status",
        "goal_pass",
        "diagnostic_warning_count",
        "controller",
        "arm",
        "profile",
        "profile_purpose",
        "stress_level",
        "tracking_source_used",
        "measured_orientation_source",
        "diameter_m",
        "period_sec",
        "angular_frequency_rad_s",
        "repeat",
        "command_rate_hz",
        "servo_rate_hz",
        "servo_t1_sec",
        "acceptance_semantics",
        "async_mode",
        "udp_command_count",
        "server_servo_tick_count",
        "async_commands_enqueued_total",
        "async_commands_sent_total",
        "async_commands_acked_total",
        "commands_sent_total",
        "commands_acked_total",
        "socket_send_only_count",
        "ack_observed_ratio",
        "official_tracking_window_sec",
        "measured_worker_window_sec",
        "official_servo_rate_hz",
        "expected_servo_ticks",
        "goal_window_commands_sent",
        "goal_window_commands_acked",
        "goal_window_count_source",
        "ack_coverage_ratio",
        "effective_goal_command_rate_hz",
        "worker_send_rate_hz",
        "worker_lifetime_send_rate_hz",
        "worker_sends_outside_official_window",
        "phase_advance_sec",
        "phase_advance_fraction_of_period",
        "phase_advance_enabled",
        "commanded_phase_advance_ms",
        "feedback_kp_pos",
        "feedback_kp_ori",
        "feedback_max_linear_m_s",
        "feedback_max_angular_rad_s",
        "required_tangential_speed_m_s",
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
        "q_actual_moved",
        "q_sent_moved",
        "q_ref_moved",
        "q_ref_reason",
        "q_ref_valid_ratio",
        "q_actual_valid_ratio",
        "tcp_ref_moved",
        "tcp_actual_moved",
        "q_sent_update_rate_hz",
        "q_ref_update_rate_hz",
        "q_actual_update_rate_hz",
        "reset_rate_hz",
        "divergence_rate_hz",
        "feedback_saturation_count",
        "stale_state_feedback_skips",
        "median_error_m",
        "mad_error_m",
        "iqr_error_m",
        "tail_ratio",
        "max_over_p95",
        "phase_aligned_rms_error_m",
        "center_removed_rms_error_m",
        "center_and_phase_removed_rms_error_m",
        "orientation_p50_drift_rad",
        "orientation_p95_drift_rad",
        "orientation_max_drift_rad",
        "orientation_p50_deg",
        "orientation_p95_deg",
        "orientation_max_deg",
        "orientation_position_equiv_50mm_m",
        "orientation_position_equiv_50mm_mm",
        "angular_feedback_norm_p50",
        "angular_feedback_norm_p95",
        "angular_feedback_norm_max",
        "angular_applied_norm_p50",
        "angular_applied_norm_p95",
        "angular_applied_norm_max",
        "angular_saturation_count",
        "angular_saturation_ratio",
        "error_classification",
        "timing_classification",
        "ack_spike_count_10ms",
        "ack_spike_count_20ms",
        "state_gap_count",
        "command_gap_count",
        "p95_error_near_ack_spike_m",
        "p95_error_away_from_ack_spike_m",
        "p95_error_near_command_gap_m",
        "p95_error_away_from_command_gap_m",
        "ack_observed_count",
        "controller_acceptance_observed_count",
        "command_timeout_count",
        "controller_rejected_count",
        "diagnostics_suspect_count",
        "controller_simulation_diagnostic_override_active_count",
        "measurement_reliability_level",
        "reliability_caveats",
        "benchmark_interpretation",
        "physical_real_blockers",
        "cartesian_unavailable_count",
        "threshold_failures",
        "threshold_warnings",
        "diagnostic_warnings",
        "armed_hold_count",
        "command_accepted_but_target_static",
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
    if args.controller == "server_circle":
        return run_server_circle(
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
    official_tracking_start_ns = benchmark_start_ns
    official_tracking_end_ns = benchmark_start_ns + int(round(duration_sec * 1e9))
    profile_metadata = sim_bench.benchmark_profile_metadata(args.profile)
    tracking_samples = collect_samples(states, args.arm, tracking_source, official_tracking_start_ns, official_tracking_end_ns)
    if not tracking_samples:
        raise BenchmarkError(f"no valid {tracking_source} samples captured during benchmark")
    network_config = config_section(config, "network")
    servo_config = config_section(config, "servo")
    arm_config = (
        selected_arm_config(config, args.arm)
        if hasattr(config, "left") and hasattr(config, "right")
        else None
    )
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
    q_actual_metrics = q_motion_metrics(states, args.arm, "q_actual_deg")
    q_sent_metrics = q_motion_metrics(states, args.arm, "q_sent_deg")
    q_ref_metrics = q_motion_metrics(states, args.arm, "q_ref_deg")
    q_actual_drift = q_actual_metrics["q_actual_drift_from_start_deg"]
    runtime_diagnostics = cartesian_runtime_diagnostics(
        states,
        args.arm,
        official_tracking_start_ns,
        official_tracking_end_ns,
    )
    tcp_ref_displacement = finite_number(runtime_diagnostics.get("tcp_ref_displacement_m"))
    tcp_ref_moved = pose_moved(tcp_ref_displacement)
    tcp_actual_displacement = finite_number(runtime_diagnostics.get("tcp_actual_displacement_m"))
    tcp_actual_moved = pose_moved(tcp_actual_displacement)
    physical_motion_detected = (
        q_actual_drift is not None and q_actual_drift > args.physical_motion_warning_deg
    )
    q_actual_moved = q_actual_metrics["q_actual_moved"]
    integrator_resets_total = finite_number(metrics.get("integrator_resets_total"))
    integrator_divergence_total = finite_number(metrics.get("integrator_divergence_total"))
    reset_rate_hz = integrator_resets_total / duration_sec if integrator_resets_total is not None and duration_sec > 0 else None
    divergence_rate_hz = (
        integrator_divergence_total / duration_sec
        if integrator_divergence_total is not None and duration_sec > 0
        else None
    )
    performance_warnings = sim_bench.performance_warnings(metrics)
    if tracking_source_warning:
        performance_warnings.append(tracking_source_warning)
    if physical_motion_detected:
        performance_warnings.append(
            f"physical q_actual drift {q_actual_drift:.6f} deg exceeded pgmode simulation warning threshold "
            f"{args.physical_motion_warning_deg:.6f} deg"
        )
    if (
        integrator_divergence_total is not None
        and integrator_divergence_total >= INTEGRATOR_DIVERGENCE_WARNING_MIN
        and q_actual_moved is False
    ):
        performance_warnings.append(
            "controller-simulation q_actual is stationary; Cartesian integration may need reference-state source."
        )
    fault_details = first_fault_details(states, benchmark_start_ns)

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
        "profile_purpose": profile_metadata["purpose"],
        "profile_catalog_entry": profile_metadata,
        "stress_level": profile_metadata["stress_level"],
        "diameter_m": args.diameter_m,
        "radius_m": float(args.diameter_m) * 0.5,
        "reference_radius_m": float(args.diameter_m) * 0.5,
        "period_sec": args.period_sec,
        "angular_frequency_rad_s": 2.0 * math.pi / float(args.period_sec),
        "recommended_controller": profile_metadata["recommended_controller"],
        "recommended_controllers": profile_metadata["recommended_controllers"],
        "repeat": args.repeat,
        "command_rate_hz": args.command_rate_hz,
        **sim_bench.phase_advance_summary(args),
        "state_pub_rate_hz": finite_number(network_config.get("state_pub_rate_hz")),
        "servo_rate_hz": finite_number(servo_config.get("rate_hz")),
        "servo_t1_sec": finite_number(getattr(arm_config, "servo_t1_sec", None)),
        "servo_t2_sec": finite_number(getattr(arm_config, "servo_t2_sec", None)),
        "servo_alpha": finite_number(getattr(arm_config, "servo_alpha", None)),
        "tool_offset_m": tool_offsets_from_args(args),
        "required_tangential_speed_m_s": preflight_result["required_tangential_speed_m_s"],
        "duration_sec": duration_sec,
        "command_count": command_count,
        "udp_command_count": command_count,
        "benchmark_start_ns": benchmark_start_ns,
        "benchmark_end_ns": benchmark_end_ns,
        "official_tracking_start_ns": official_tracking_start_ns,
        "official_tracking_end_ns": official_tracking_end_ns,
        "official_tracking_window_sec": duration_sec,
        "server_servo_tick_count": server_servo_tick_count(
            states,
            official_tracking_start_ns,
            official_tracking_end_ns,
        ),
        "tracking_source_requested": args.tracking_source,
        "tracking_source_used": tracking_source,
        "tracking_source_warning": tracking_source_warning,
        "desired_orientation_stand": {
            "quaternion_xyzw": list(q0),
            "frame": "stand",
            "source": f"{tracking_source} at ArmMotion",
        },
        "measured_orientation_source": f"{tracking_source}.quaternion_xyzw",
        "state_packet_count": len(states),
        "invalid_state_packets": 0,
        "tcp_ref_valid_ratio": tcp_valid_ratio(states, args.arm, "tcp_ref_stand"),
        "tcp_actual_valid_ratio": tcp_valid_ratio(states, args.arm, "tcp_actual_stand"),
        "q_ref_valid_ratio": q_valid_ratio(states, args.arm, "q_ref_deg"),
        "q_actual_valid_ratio": q_valid_ratio(states, args.arm, "q_actual_deg"),
        "q_ref_update_rate_hz": q_ref_metrics["q_ref_update_rate_hz"],
        "q_ref_drift_from_start_deg": q_ref_metrics["q_ref_drift_from_start_deg"],
        "q_ref_moved": q_ref_metrics["q_ref_moved"],
        "q_ref_reason": q_ref_metrics["q_ref_reason"],
        "q_sent_update_rate_hz": q_sent_metrics["q_sent_update_rate_hz"],
        "q_sent_drift_from_start_deg": q_sent_metrics["q_sent_drift_from_start_deg"],
        "q_sent_moved": q_sent_metrics["q_sent_moved"],
        "q_sent_reason": q_sent_metrics["q_sent_reason"],
        "q_actual_update_rate_hz": q_actual_metrics["q_actual_update_rate_hz"],
        "q_actual_drift_from_start_deg": q_actual_drift,
        "q_actual_moved": q_actual_moved,
        "q_actual_reason": q_actual_metrics["q_actual_reason"],
        "tcp_ref_moved": tcp_ref_moved,
        "tcp_actual_moved": tcp_actual_moved,
        "controller_simulation_motion_evidence_source": "tcp_ref_stand",
        "controller_simulation_motion_detected": tcp_ref_moved,
        "reset_rate_hz": reset_rate_hz,
        "divergence_rate_hz": divergence_rate_hz,
        "physical_motion_detected": physical_motion_detected,
        "physical_motion_expected": False,
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
        "fault_latched": fault_details["fault_latched"],
        "first_fault_time_sec": fault_details["first_fault_time_sec"],
        "latched_fault_reason": fault_details["latched_fault_reason"],
        "fault_reason": fault_details["fault_reason"],
        "fault_context": fault_details["fault_context"],
        "caveat": (
            "rbpodo controller-simulation benchmark evidence; controller boxes are real, "
            "physical robot motion is not expected or approved"
        ),
    }
    if args.controller.endswith("_feedback"):
        summary.update(
            {
                "feedback_kp_pos": args.feedback_kp_pos,
                "feedback_kp_ori": args.feedback_kp_ori,
                "feedback_max_linear_m_s": args.feedback_max_linear_m_s,
                "feedback_max_angular_rad_s": args.feedback_max_angular_rad_s,
                "feedback_use_current_state_time": args.feedback_use_current_state_time,
                "feedback_mode_caveat": (
                    "closed-loop command-source benchmark compensation; not production policy or real robot readiness"
                ),
            }
        )
    summary.update(metrics)
    summary.update(
        feedback_orientation_command_metrics(
            feedback_rows,
            max_angular_rad_s=finite_number(args.feedback_max_angular_rad_s),
        )
    )
    summary["orientation_error_vector_samples"] = orientation_error_vector_sample_rows(merged, q0)
    summary.update(telemetry_metrics(states, args.arm))
    summary.update(runtime_diagnostics)
    if summary.get("feedback_saturation_count") is None:
        summary["feedback_saturation_count"] = summary.get("cartesian_twist_clamp_count")
    summary.update(command_interval_metrics(artifact_dir / "command_packets.jsonl"))
    summary.update(fault_details)
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
    alignment_report_path = artifact_dir / "alignment_report.md"
    alignment_summary_path = artifact_dir / "alignment_summary.json"
    alignment_audit = timestamp_alignment_audit.audit_artifact_dir(
        artifact_dir,
        summary=summary,
        expected_command_rate_hz=finite_number(args.command_rate_hz),
        expected_state_rate_hz=finite_number(network_config.get("state_pub_rate_hz")),
        output_md_path=alignment_report_path,
        output_json_path=alignment_summary_path,
    )
    summary["timestamp_alignment"] = timestamp_alignment_audit.benchmark_timestamp_alignment_block(alignment_audit)
    summary["tail_error_correlation"] = timestamp_alignment_audit.benchmark_tail_error_correlation_block(alignment_audit)
    summary["timestamp_alignment_report"] = str(alignment_report_path.resolve())
    summary["timestamp_alignment_summary"] = str(alignment_summary_path.resolve())
    timestamp_alignment = summary["timestamp_alignment"]
    tail_error_correlation = summary["tail_error_correlation"]
    summary["timing_classification"] = timestamp_alignment.get("timing_classification")
    summary["ack_spike_count_10ms"] = timestamp_alignment.get("ack_spike_count_10ms")
    summary["ack_spike_count_20ms"] = timestamp_alignment.get("ack_spike_count_20ms")
    summary["state_gap_count"] = timestamp_alignment.get("state_gap_count")
    summary["command_gap_count"] = timestamp_alignment.get("command_gap_count")
    summary["p95_error_near_ack_spike_m"] = tail_error_correlation.get("p95_error_near_ack_spike_m")
    summary["p95_error_away_from_ack_spike_m"] = tail_error_correlation.get("p95_error_away_from_ack_spike_m")
    summary["p95_error_near_command_gap_m"] = tail_error_correlation.get("p95_error_near_command_gap_m")
    summary["p95_error_away_from_command_gap_m"] = tail_error_correlation.get("p95_error_away_from_command_gap_m")
    if summary["timing_classification"] not in (None, "", "clean_timing"):
        performance_warnings.append(
            "timestamp_alignment timing_classification="
            f"{summary['timing_classification']}; do not treat timing-limited tails as reliable tracking error"
        )
    error_decomp = error_decomposition.decompose_circle_run(
        merged,
        summary=summary,
        geometry=trajectory_geometry(traj),
        period_sec=finite_number(args.period_sec),
        tool_offsets_m=tool_offsets_from_args(args),
        feedback_rows=feedback_rows,
    )
    error_decomposition_path = artifact_dir / "error_decomposition.json"
    write_json(error_decomposition_path, error_decomp)
    summary["error_decomposition"] = error_decomp
    summary["error_decomposition_json"] = str(error_decomposition_path.resolve())
    summary["mad_error_m"] = error_decomp.get("mad_error_m")
    summary["iqr_error_m"] = error_decomp.get("iqr_error_m")
    summary["tail_ratio"] = error_decomp.get("tail_ratio")
    summary["max_over_p95"] = error_decomp.get("max_over_p95")
    summary["phase_lag_rad"] = error_decomp.get("phase_lag_rad")
    summary["phase_aligned_rms_error_m"] = error_decomp.get("phase_aligned_rms_error_m")
    summary["center_removed_rms_error_m"] = error_decomp.get("center_removed_rms_error_m")
    summary["center_and_phase_removed_rms_error_m"] = error_decomp.get("center_and_phase_removed_rms_error_m")
    orientation_block = error_decomp.get("orientation_drift_rad") if isinstance(error_decomp.get("orientation_drift_rad"), dict) else {}
    summary["orientation_p50_drift_rad"] = orientation_block.get("p50")
    summary["orientation_p95_drift_rad"] = orientation_block.get("p95")
    summary["orientation_max_drift_rad"] = orientation_block.get("max")
    summary["orientation_p50_rad"] = error_decomp.get("orientation_p50_rad")
    summary["orientation_p95_rad"] = error_decomp.get("orientation_p95_rad")
    summary["orientation_max_rad"] = error_decomp.get("orientation_max_rad")
    summary["orientation_p50_deg"] = error_decomp.get("orientation_p50_deg")
    summary["orientation_p95_deg"] = error_decomp.get("orientation_p95_deg")
    summary["orientation_max_deg"] = error_decomp.get("orientation_max_deg")
    summary["orientation_position_equiv_50mm_m"] = error_decomp.get("orientation_position_equiv_50mm_m")
    summary["orientation_position_equiv_50mm_mm"] = error_decomp.get("orientation_position_equiv_50mm_mm")
    summary["orientation_position_equiv_mm"] = error_decomp.get("orientation_position_equiv_mm")
    summary["error_classification"] = error_decomp.get("error_classification")
    summary["error_classifications"] = error_decomp.get("error_classifications")
    summary["error_classification_reasons"] = error_decomp.get("classification_reasons")
    cycle_rows = error_decomp.get("cycles")
    if isinstance(cycle_rows, list):
        write_csv(artifact_dir / "cycle_error_decomposition.csv", cycle_rows)
    skipped_plots = plot_artifacts(artifact_dir, args, traj, merged, controller_rows, physical_rows, no_circle_reason)
    if summary.get("fault_latched") is True:
        fault_name = summary.get("latched_fault_reason") or "unknown"
        fault_text = summary.get("fault_reason") or "no fault reason reported"
        run_status = "faulted"
        run_reason = f"server fault latched: {fault_name}: {fault_text}"
        failures: list[str] = []
        legacy_failures = [run_reason]
    elif summary.get("server_rejected_cartesian") is True:
        run_status = "blocked"
        run_reason = "cartesian_commands_rejected_by_server"
        failures = []
        legacy_failures = [
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
        _threshold_result, threshold_reason, failures = result_from_thresholds(args, summary)
        run_status = "completed"
        run_reason = "run completed; " + threshold_reason
        legacy_failures = failures
    summary.update(
        {
            "result": run_status,
            "result_reason": run_reason,
            "performance_warnings": performance_warnings,
            "generated_plots": generated_plot_paths(artifact_dir),
            "skipped_plots": skipped_plots,
        }
    )
    reliability_report.annotate_row(summary)
    apply_result_contract(
        summary,
        run_status=run_status,
        run_reason=run_reason,
        threshold_failures_value=failures,
        threshold_warnings=performance_warnings,
        thresholds_were_requested=thresholds_requested(args),
        legacy_threshold_failures_value=legacy_failures,
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
            "profile": args.profile,
            "profile_purpose": preflight_result.get("profile_purpose"),
            "profile_catalog_entry": preflight_result.get("profile_catalog_entry"),
            "stress_level": preflight_result.get("stress_level"),
            "diameter_m": args.diameter_m,
            "period_sec": args.period_sec,
            "required_tangential_speed_m_s": preflight_result.get("required_tangential_speed_m_s"),
            "angular_frequency_rad_s": preflight_result.get("angular_frequency_rad_s"),
            **sim_bench.phase_advance_summary(args),
            "recommended_controller": preflight_result.get("recommended_controller"),
            "recommended_controllers": preflight_result.get("recommended_controllers"),
            "safety_preflight": preflight_result,
            "preflight_only": True,
            "fault_latched": False,
            "physical_motion_detected": False,
            "physical_motion_expected": False,
            "cartesian_unavailable_count": 0,
            "overlay_enabled": not args.overlay_disable,
            "overlay_pub_endpoint": None if args.overlay_disable else args.overlay_pub_endpoint,
            "overlay_messages_sent": 0,
        }
        reliability_report.annotate_row(summary)
        apply_result_contract(
            summary,
            run_status="completed",
            run_reason="preflight only; no tracking run or performance thresholds were evaluated",
            threshold_failures_value=[],
            threshold_warnings=[],
            thresholds_were_requested=False,
        )
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
    if isinstance(exc, StartupFaultError):
        details = dict(exc.details)
        failure = {
            "schema": SCHEMA,
            "result": "startup_fault",
            "result_reason": details.get("result_reason") or "startup fault latched",
            "error": str(exc),
            "server_config": str(args.server_config.resolve()) if args.server_config else None,
            "state_endpoint": details.get("state_endpoint"),
            "server_returncode": details.get("server_returncode"),
            "state_packets_received": details.get("state_packets_received"),
            "latched_fault_reason": details.get("latched_fault_reason"),
            "fault_reason": details.get("fault_reason"),
            "fault_context": details.get("fault_context"),
            "fault_latched": True,
            "physical_motion_detected": False,
            "physical_motion_expected": False,
            "cartesian_unavailable_count": 0,
            "safety_tracking": details.get("safety_tracking"),
            "q_actual_target_error_summary": details.get("q_actual_target_error_summary"),
            "latest_state_excerpt": details.get("latest_state_excerpt"),
            "startup_fault": details,
            "safety_preflight": {"passed": False, "error": str(exc), "env": benchmark_env_snapshot()},
            **sim_bench.phase_advance_summary(args),
            "performance_warnings": details.get("startup_fault_hints") or [],
            "caveat": "rbpodo controller-simulation benchmark did not complete",
        }
        reliability_report.annotate_row(failure)
        apply_result_contract(
            failure,
            run_status="startup_fault",
            run_reason=details.get("result_reason") or "startup fault latched",
            threshold_failures_value=[],
            threshold_warnings=text_list(details.get("startup_fault_hints")),
            thresholds_were_requested=False,
        )
        write_json(artifact_dir / "summary.json", failure)
        write_summary_csv(artifact_dir / "summary.csv", failure)
        return failure
    failure = {
        "schema": SCHEMA,
        "result": "error",
        "result_reason": "benchmark could not run",
        "error": str(exc),
        "server_config": str(args.server_config.resolve()) if args.server_config else None,
        "safety_preflight": {"passed": False, "error": str(exc), "env": benchmark_env_snapshot()},
        **sim_bench.phase_advance_summary(args),
        "performance_warnings": [],
        "fault_latched": False,
        "physical_motion_detected": False,
        "physical_motion_expected": False,
        "cartesian_unavailable_count": 0,
        "caveat": "rbpodo controller-simulation benchmark did not complete",
    }
    reliability_report.annotate_row(failure)
    apply_result_contract(
        failure,
        run_status="error",
        run_reason="benchmark could not run",
        threshold_failures_value=[],
        threshold_warnings=[],
        thresholds_were_requested=False,
    )
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
