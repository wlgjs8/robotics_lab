#!/usr/bin/env python3
"""rb_servo_server-level rbpodo 500 Hz controller-simulation no-op acceptance."""

from __future__ import annotations

import argparse
import copy
import csv
import json
import math
import os
import re
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
    load_config,
    parse_udp_endpoint,
    scalar_value,
    state_stream_timeout_message,
)


def env_enabled(name: str) -> bool:
    return os.environ.get(name, "").strip().lower() in {"1", "true", "yes", "on"}


SCHEMA = "robotics_lab.rbpodo_500hz_acceptance.v1"
MODE = "servo_j_noop_500hz"
COMMAND_RATE_HZ = 500.0
COMMAND_PERIOD_SEC = 1.0 / COMMAND_RATE_HZ
ARMS = ("left", "right")
ASYNC_STREAMING_ENV = "RB_ALLOW_RBPODO_ASYNC_STREAMING"
SOCKET_SEND_ONLY_ENV = "RB_ALLOW_RBPODO_SOCKET_SEND_ONLY_STREAMING"
ASYNC_DISABLED = "disabled"
ASYNC_SDK_ACK_WORKER = "sdk_ack_worker"
ASYNC_SOCKET_SEND_SUPERVISED = "socket_send_supervised"
ASYNC_MODES = (ASYNC_DISABLED, ASYNC_SDK_ACK_WORKER, ASYNC_SOCKET_SEND_SUPERVISED)


class Acceptance500HzError(RuntimeError):
    def __init__(
        self,
        message: str,
        *,
        failure_phase: str | None = None,
        failure_classification: str | None = None,
        snapshot: dict[str, Any] | None = None,
    ) -> None:
        super().__init__(message)
        self.failure_phase = failure_phase
        self.failure_classification = failure_classification
        self.snapshot = snapshot


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
    command_rate_hz: float = COMMAND_RATE_HZ


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
    parser.add_argument(
        "--send-arms",
        choices=("left", "right", "both"),
        help="Arm(s) that receive no-op JointTarget packets. Defaults to --arm.",
    )
    parser.add_argument("--mode", choices=(MODE,), default=MODE)
    parser.add_argument("--duration-sec", type=float, default=10.0)
    parser.add_argument("--artifact-dir", type=Path, required=True)
    parser.add_argument(
        "--command-timeout-sec",
        type=float,
        help=(
            "Override left/right rbpodo command_timeout_sec in artifact-dir/resolved_config.yaml "
            "for the measured 500 Hz phase."
        ),
    )
    parser.add_argument("--warmup-duration-sec", type=float, default=0.0)
    parser.add_argument("--warmup-rate-hz", type=float, default=100.0)
    parser.add_argument(
        "--warmup-command-timeout-sec",
        type=float,
        help="Override rbpodo command_timeout_sec for the separate warmup server run.",
    )
    parser.add_argument(
        "--ack-timeout-sweep",
        help="Comma-separated measured-phase command_timeout_sec values, e.g. 0.005,0.01,0.02,0.05.",
    )
    parser.add_argument(
        "--preserve-cartesian-control",
        action="store_true",
        help="Do not disable cartesian_control.enable in the no-op resolved config.",
    )
    parser.add_argument(
        "--disable-waiting-ack-diagnostic",
        action="store_true",
        help=(
            "Write an artifact-local resolved config with rbpodo disable_waiting_ack=true for both arms. "
            "This is diagnostic-only socket-send evidence and is not controller ACK acceptance."
        ),
    )
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
    parser.add_argument(
        "--async-mode",
        choices=ASYNC_MODES,
        default=ASYNC_DISABLED,
        help=(
            "Configure artifact-local rbpodo async streaming for the measured run. "
            "disabled preserves synchronous behavior; sdk_ack_worker expects worker "
            "controller ACK telemetry; socket_send_supervised expects socket-send-only "
            "plus q_ref/tcp_ref reference supervision."
        ),
    )
    parser.add_argument(
        "--require-reference-supervision",
        action="store_true",
        help="Require async q_ref/tcp_ref supervision telemetry to be present and healthy.",
    )
    parser.add_argument("--max-q-ref-update-age-ms", type=float, default=50.0)
    parser.add_argument("--max-tcp-ref-update-age-ms", type=float, default=50.0)
    parser.add_argument("--max-overwrite-ratio", type=float, default=0.05)
    parser.add_argument("--max-drop-ratio", type=float, default=0.0)
    parser.add_argument("--min-q-ref-update-rate-hz", type=float, default=20.0)
    parser.add_argument(
        "--allow-socket-send-only",
        action="store_true",
        help=(
            "Required with --async-mode socket_send_supervised to acknowledge that "
            "successful sends are socket-send-only evidence, not controller ACKs."
        ),
    )
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


def env_snapshot_500hz() -> dict[str, str | None]:
    return {
        ASYNC_STREAMING_ENV: os.environ.get(ASYNC_STREAMING_ENV),
        SOCKET_SEND_ONLY_ENV: os.environ.get(SOCKET_SEND_ONLY_ENV),
    }


def async_mode_value(args: argparse.Namespace | Any) -> str:
    return str(getattr(args, "async_mode", ASYNC_DISABLED) or ASYNC_DISABLED)


def reference_supervision_required(args: argparse.Namespace | Any) -> bool:
    return bool(getattr(args, "require_reference_supervision", False)) or async_mode_value(args) == ASYNC_SOCKET_SEND_SUPERVISED


def active_send_arms(args: argparse.Namespace) -> tuple[str, ...]:
    requested = getattr(args, "send_arms", None) or args.arm
    if requested == "both":
        return ARMS
    if requested not in ARMS:
        raise Acceptance500HzError(f"--send-arms must be left, right, or both; got {requested!r}")
    return (str(requested),)


def parse_ack_timeout_sweep(value: str | None) -> list[float]:
    if value is None:
        return []
    out: list[float] = []
    for raw_item in value.split(","):
        item = raw_item.strip()
        if not item:
            continue
        try:
            timeout = float(item)
        except ValueError as exc:
            raise Acceptance500HzError(f"invalid --ack-timeout-sweep value {item!r}") from exc
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise Acceptance500HzError("--ack-timeout-sweep values must be finite and positive")
        out.append(timeout)
    if not out:
        raise Acceptance500HzError("--ack-timeout-sweep must contain at least one timeout")
    return out


def timeout_sweep_label(timeout_sec: float) -> str:
    timeout_ms = timeout_sec * 1000.0
    if abs(timeout_ms - round(timeout_ms)) < 1e-9:
        return f"timeout_{int(round(timeout_ms)):03d}ms"
    safe = f"{timeout_ms:.3f}".rstrip("0").rstrip(".").replace(".", "p")
    return f"timeout_{safe}ms"


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


def yaml_document(path: Path) -> dict[str, Any]:
    try:
        import yaml  # type: ignore
    except Exception as exc:
        raise Acceptance500HzError(
            "PyYAML is required to write the artifact-local resolved 500 Hz config"
        ) from exc
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise Acceptance500HzError(f"failed to parse YAML config {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise Acceptance500HzError("server config must be a YAML object")
    return data


def write_yaml_document(path: Path, data: dict[str, Any]) -> None:
    try:
        import yaml  # type: ignore
    except Exception as exc:
        raise Acceptance500HzError(
            "PyYAML is required to write the artifact-local resolved 500 Hz config"
        ) from exc
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")


def resolve_existing_relative_path(source_config_path: Path, value: str) -> str:
    path = Path(value)
    if path.is_absolute():
        return value
    candidates = [
        source_config_path.parent / path,
        source_config_path.parent.parent / path,
        source_config_path.parent.parent.parent / path,
    ]
    for candidate in candidates:
        resolved = candidate.resolve()
        if resolved.exists():
            return str(resolved)
    return value


def nested_dict(data: dict[str, Any], key: str) -> dict[str, Any]:
    value = data.get(key)
    if isinstance(value, dict):
        return value
    value = {}
    data[key] = value
    return value


def command_timeout_from_sections(sections: dict[str, dict[str, Any]], arm: str) -> float | None:
    return finite_number(sections.get(f"{arm}_robot", {}).get("command_timeout_sec"))


def write_resolved_config(
    args: argparse.Namespace,
    source_config_path: Path,
    artifact_dir: Path,
    *,
    command_timeout_sec: float | None,
) -> tuple[Path, dict[str, Any]]:
    data = yaml_document(source_config_path)
    overrides: dict[str, Any] = {}
    async_mode = async_mode_value(args)
    if command_timeout_sec is not None:
        if not math.isfinite(command_timeout_sec) or command_timeout_sec <= 0.0:
            raise Acceptance500HzError("--command-timeout-sec must be finite and positive")
        for arm in ARMS:
            robot = nested_dict(data, f"{arm}_robot")
            robot["command_timeout_sec"] = float(command_timeout_sec)
        overrides["left_robot.command_timeout_sec"] = float(command_timeout_sec)
        overrides["right_robot.command_timeout_sec"] = float(command_timeout_sec)

    if getattr(args, "disable_waiting_ack_diagnostic", False):
        for arm in ARMS:
            robot = nested_dict(data, f"{arm}_robot")
            robot["disable_waiting_ack"] = True
        overrides["left_robot.disable_waiting_ack"] = True
        overrides["right_robot.disable_waiting_ack"] = True

    if async_mode != ASYNC_DISABLED:
        if getattr(args, "disable_waiting_ack_diagnostic", False):
            raise Acceptance500HzError("--disable-waiting-ack-diagnostic cannot be combined with --async-mode")
        servo = nested_dict(data, "servo")
        servo["worker_read_period_sec"] = 1.0 / COMMAND_RATE_HZ
        async_cfg = nested_dict(servo, "rbpodo_async_streaming")
        async_cfg["enable"] = True
        async_cfg["mode"] = async_mode
        async_cfg["rate_hz"] = int(COMMAND_RATE_HZ)
        async_cfg["queue_policy"] = "latest_wins"
        async_cfg.setdefault("max_pending_age_ms", 10)
        existing_ack_supervision = (
            async_cfg.get("ack_supervision")
            if isinstance(async_cfg.get("ack_supervision"), dict)
            else {}
        )
        async_cfg["ack_supervision"] = {
            "enable": True,
            "expected_ack_timeout_ms": 50,
            "missing_ack_fault_after_ms": 100,
            "max_consecutive_missing_ack": 10,
            **existing_ack_supervision,
        }
        async_cfg["ack_supervision"]["enable"] = True
        existing_reference_supervision = (
            async_cfg.get("reference_supervision")
            if isinstance(async_cfg.get("reference_supervision"), dict)
            else {}
        )
        async_cfg["reference_supervision"] = {
            **existing_reference_supervision,
            "enable": True,
            "q_ref_update_timeout_ms": float(getattr(args, "max_q_ref_update_age_ms", 50.0)),
            "q_ref_target_tolerance_deg": float(getattr(args, "max_reference_drift_deg", 0.05)),
            "q_ref_target_fault_after_ms": 100,
            "tcp_ref_update_timeout_ms": float(getattr(args, "max_tcp_ref_update_age_ms", 50.0)),
            "tcp_ref_target_tolerance_m": 0.02,
            "tcp_ref_target_fault_after_ms": 100,
            "policy": "fault_latch",
        }
        diagnostics = async_cfg.get("diagnostics")
        if not isinstance(diagnostics, dict):
            diagnostics = {}
        diagnostics.setdefault("publish_per_command_jsonl", False)
        async_cfg["diagnostics"] = diagnostics
        overrides["servo.rbpodo_async_streaming.enable"] = True
        overrides["servo.rbpodo_async_streaming.mode"] = async_mode
        overrides["servo.rbpodo_async_streaming.rate_hz"] = int(COMMAND_RATE_HZ)
        overrides["servo.worker_read_period_sec"] = 1.0 / COMMAND_RATE_HZ
        if async_mode == ASYNC_SOCKET_SEND_SUPERVISED:
            for arm in ARMS:
                robot = nested_dict(data, f"{arm}_robot")
                robot["disable_waiting_ack"] = True
            overrides["left_robot.disable_waiting_ack"] = True
            overrides["right_robot.disable_waiting_ack"] = True
        elif async_mode == ASYNC_SDK_ACK_WORKER:
            for arm in ARMS:
                robot = nested_dict(data, f"{arm}_robot")
                robot["disable_waiting_ack"] = False
            overrides["left_robot.disable_waiting_ack"] = False
            overrides["right_robot.disable_waiting_ack"] = False

    cartesian_control = data.get("cartesian_control")
    cartesian_disabled_for_noop = False
    if (
        isinstance(cartesian_control, dict)
        and not getattr(args, "preserve_cartesian_control", False)
        and bool_value(cartesian_control.get("enable"))
    ):
        cartesian_control["enable"] = False
        cartesian_control["allow_in_controller_simulation"] = False
        cartesian_disabled_for_noop = True
        overrides["cartesian_control.enable"] = False
        overrides["cartesian_control.allow_in_controller_simulation"] = False

    kinematics = data.get("kinematics")
    if isinstance(kinematics, dict) and isinstance(kinematics.get("urdf"), str):
        resolved_urdf = resolve_existing_relative_path(source_config_path, str(kinematics["urdf"]))
        if resolved_urdf != kinematics["urdf"]:
            kinematics["urdf"] = resolved_urdf
            overrides["kinematics.urdf"] = resolved_urdf

    resolved_config_path = artifact_dir / "resolved_config.yaml"
    write_yaml_document(resolved_config_path, data)
    sections = yaml_sections(resolved_config_path)
    timeout_by_arm = {
        arm: command_timeout_from_sections(sections, arm)
        for arm in ARMS
    }
    return resolved_config_path, {
        "source_config": str(source_config_path),
        "resolved_config": str(resolved_config_path.resolve()),
        "resolved_config_overrides": overrides,
        "cartesian_control_disabled_for_noop": cartesian_disabled_for_noop,
        "preserve_cartesian_control": bool(getattr(args, "preserve_cartesian_control", False)),
        "command_timeout_sec_left": timeout_by_arm["left"],
        "command_timeout_sec_right": timeout_by_arm["right"],
    }


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


def noop_targets_from_state(snapshot: dict[str, Any], arms: tuple[str, ...]) -> dict[str, dict[str, Any]]:
    targets: dict[str, dict[str, Any]] = {}
    for arm in arms:
        q_target, target_source = noop_target_from_state(snapshot, arm)
        targets[arm] = {"q_target_deg": q_target, "target_source": target_source}
    return targets


def q_actual_from_state(snapshot: dict[str, Any], arm: str) -> list[float]:
    state = arm_state(snapshot, arm)
    joints = finite_joint_values(state.get("q_actual_deg") if state else None)
    if joints is None:
        raise Acceptance500HzError(f"{arm} state did not publish finite q_actual_deg")
    return joints


def q_actual_for_arms(snapshot: dict[str, Any], arms: tuple[str, ...]) -> dict[str, list[float]]:
    return {arm: q_actual_from_state(snapshot, arm) for arm in arms}


def validate_numeric_args(args: argparse.Namespace) -> None:
    positive = (
        ("--duration-sec", args.duration_sec),
        ("--startup-timeout-sec", args.startup_timeout_sec),
        ("--warmup-rate-hz", getattr(args, "warmup_rate_hz", 100.0)),
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
        ("--max-overwrite-ratio", getattr(args, "max_overwrite_ratio", 0.05)),
        ("--max-drop-ratio", getattr(args, "max_drop_ratio", 0.0)),
    ):
        lower_ok = value >= 0.0 if name in {"--max-overwrite-ratio", "--max-drop-ratio"} else value > 0.0
        if not math.isfinite(value) or not lower_ok or value > 1.0:
            interval = "[0, 1]" if name in {"--max-overwrite-ratio", "--max-drop-ratio"} else "(0, 1]"
            raise Acceptance500HzError(f"{name} must be finite and in {interval}")
    if args.max_deadline_miss_count < 0:
        raise Acceptance500HzError("--max-deadline-miss-count must be >= 0")
    if args.max_worker_drop_count < 0:
        raise Acceptance500HzError("--max-worker-drop-count must be >= 0")
    if args.pgmode_command_port < 1 or args.pgmode_command_port > 65535:
        raise Acceptance500HzError("--pgmode-command-port must be in [1, 65535]")
    if args.settle_sec < 0.0 or not math.isfinite(args.settle_sec):
        raise Acceptance500HzError("--settle-sec must be finite and non-negative")
    warmup_duration = getattr(args, "warmup_duration_sec", 0.0)
    if warmup_duration < 0.0 or not math.isfinite(warmup_duration):
        raise Acceptance500HzError("--warmup-duration-sec must be finite and non-negative")
    for name, value in (
        ("--command-timeout-sec", getattr(args, "command_timeout_sec", None)),
        ("--warmup-command-timeout-sec", getattr(args, "warmup_command_timeout_sec", None)),
    ):
        if value is not None and (not math.isfinite(value) or value <= 0.0):
            raise Acceptance500HzError(f"{name} must be finite and positive")
    for name, value in (
        ("--max-q-ref-update-age-ms", getattr(args, "max_q_ref_update_age_ms", 50.0)),
        ("--max-tcp-ref-update-age-ms", getattr(args, "max_tcp_ref_update_age_ms", 50.0)),
        ("--min-q-ref-update-rate-hz", getattr(args, "min_q_ref_update_rate_hz", 20.0)),
    ):
        if not math.isfinite(value) or value <= 0.0:
            raise Acceptance500HzError(f"{name} must be finite and positive")
    if async_mode_value(args) not in ASYNC_MODES:
        raise Acceptance500HzError(f"--async-mode must be one of {', '.join(ASYNC_MODES)}")
    if async_mode_value(args) == ASYNC_DISABLED and getattr(args, "require_reference_supervision", False):
        raise Acceptance500HzError("--require-reference-supervision requires --async-mode sdk_ack_worker or socket_send_supervised")
    if async_mode_value(args) != ASYNC_SOCKET_SEND_SUPERVISED and getattr(args, "allow_socket_send_only", False):
        raise Acceptance500HzError("--allow-socket-send-only is only valid with --async-mode socket_send_supervised")
    parse_ack_timeout_sweep(getattr(args, "ack_timeout_sweep", None))


def ensure_pgmode(args: argparse.Namespace, config: ParsedConfig) -> dict[str, Any]:
    if args.set_pgmode_simulation and args.verify_pgmode_simulation:
        raise Acceptance500HzError("--set-pgmode-simulation and --verify-pgmode-simulation are mutually exclusive")
    if not args.set_pgmode_simulation and not args.verify_pgmode_simulation:
        raise Acceptance500HzError(
            "500 Hz controller-simulation acceptance requires --set-pgmode-simulation "
            "or --verify-pgmode-simulation",
            failure_phase="preflight",
            failure_classification="preflight_env_missing",
        )
    if not args.i_confirm_controller_is_in_pgmode_simulation:
        raise Acceptance500HzError(
            "missing --i-confirm-controller-is-in-pgmode-simulation",
            failure_phase="preflight",
            failure_classification="preflight_env_missing",
        )
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
    resolve_info: dict[str, Any] | None = None,
) -> dict[str, Any]:
    validate_numeric_args(args)
    if args.mode != MODE:
        raise Acceptance500HzError(f"unsupported mode {args.mode}")
    async_mode = async_mode_value(args)
    ack_disabled_diagnostic = bool(getattr(args, "disable_waiting_ack_diagnostic", False))
    ack_disabled_arms: list[str] = []
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
            ack_disabled_arms.append(label)
            if async_mode == ASYNC_SOCKET_SEND_SUPERVISED:
                pass
            elif not ack_disabled_diagnostic:
                raise Acceptance500HzError("500 Hz no-op acceptance requires ACK-on rbpodo settings")

    ack_disabled = bool(ack_disabled_arms)
    if ack_disabled_diagnostic:
        if set(ack_disabled_arms) != set(ARMS):
            raise Acceptance500HzError(
                "--disable-waiting-ack-diagnostic must resolve both arms to disable_waiting_ack=true"
            )
    if async_mode == ASYNC_SOCKET_SEND_SUPERVISED:
        if set(ack_disabled_arms) != set(ARMS):
            raise Acceptance500HzError(
                "--async-mode socket_send_supervised must resolve both arms to disable_waiting_ack=true"
            )
        if not getattr(args, "allow_socket_send_only", False):
            raise Acceptance500HzError(
                "--async-mode socket_send_supervised requires --allow-socket-send-only",
                failure_phase="preflight",
                failure_classification="preflight_env_missing",
            )
        if not env_enabled(SOCKET_SEND_ONLY_ENV):
            raise Acceptance500HzError(
                f"--async-mode socket_send_supervised requires {SOCKET_SEND_ONLY_ENV}=1",
                failure_phase="preflight",
                failure_classification="preflight_env_missing",
            )
    elif async_mode == ASYNC_SDK_ACK_WORKER and ack_disabled:
        raise Acceptance500HzError("--async-mode sdk_ack_worker requires ACK waiting enabled for both arms")

    known_ips = {config.left.ip, config.right.ip} & REAL_ROBOT_IPS
    if known_ips and not args.i_understand_this_connects_to_real_controller:
        raise Acceptance500HzError(
            "refusing known real controller IP without explicit confirmation flag",
            failure_phase="preflight",
            failure_classification="preflight_env_missing",
        )
    if not args.i_understand_this_connects_to_real_controller:
        raise Acceptance500HzError(
            "missing --i-understand-this-connects-to-real-controller",
            failure_phase="preflight",
            failure_classification="preflight_env_missing",
        )
    if async_mode != ASYNC_DISABLED and not env_enabled(ASYNC_STREAMING_ENV):
        raise Acceptance500HzError(
            f"--async-mode {async_mode} requires {ASYNC_STREAMING_ENV}=1",
            failure_phase="preflight",
            failure_classification="preflight_env_missing",
        )

    cartesian_control = sections.get("cartesian_control", {})
    cartesian_enabled = as_bool(cartesian_control.get("enable"), False)
    cartesian_controller_sim_allowed = as_bool(cartesian_control.get("allow_in_controller_simulation"), False)

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
            "RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1",
            failure_phase="preflight",
            failure_classification="preflight_env_missing",
        )

    servo_rate_hz = as_float(config.servo.get("rate_hz"))
    if servo_rate_hz is None or abs(servo_rate_hz - COMMAND_RATE_HZ) > 1e-9:
        raise Acceptance500HzError(f"servo.rate_hz must be 500 for {MODE}")
    async_cfg = sections.get("servo", {}).get("rbpodo_async_streaming")
    async_enabled = isinstance(async_cfg, dict) and as_bool(async_cfg.get("enable"), False)
    resolved_async_mode = str(async_cfg.get("mode", ASYNC_DISABLED)) if isinstance(async_cfg, dict) else ASYNC_DISABLED
    if async_mode == ASYNC_DISABLED:
        if async_enabled or resolved_async_mode != ASYNC_DISABLED:
            raise Acceptance500HzError("resolved config has rbpodo async streaming enabled but --async-mode is disabled")
    else:
        if not async_enabled or resolved_async_mode != async_mode:
            raise Acceptance500HzError(
                f"resolved config must enable servo.rbpodo_async_streaming.mode={async_mode}"
            )
        reference_cfg = async_cfg.get("reference_supervision") if isinstance(async_cfg, dict) else None
        if reference_supervision_required(args) and not (
            isinstance(reference_cfg, dict) and as_bool(reference_cfg.get("enable"), False)
        ):
            raise Acceptance500HzError(
                f"--async-mode {async_mode} requires servo.rbpodo_async_streaming.reference_supervision.enable=true"
            )
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
    result = {
        "passed": True,
        "schema": SCHEMA,
        "mode": MODE,
        "backend": "rbpodo",
        "controller_simulation_only": True,
        "physical_motion_expected": False,
        "physical_real_motion_refused": True,
        "arm": args.arm,
        "send_arms": list(active_send_arms(args)),
        "selected_ip": selected.ip,
        "configured_ips": [config.left.ip, config.right.ip],
        "known_real_ips": sorted(known_ips),
        "config": str(config.path),
        "servo_rate_hz": servo_rate_hz,
        "command_rate_hz": COMMAND_RATE_HZ,
        "servo_t1_sec": selected.servo_t1_sec,
        "async_mode": async_mode,
        "rbpodo_async_streaming_enabled": async_mode != ASYNC_DISABLED,
        "reference_supervision_required": reference_supervision_required(args),
        "allow_socket_send_only": bool(getattr(args, "allow_socket_send_only", False)),
        "disable_waiting_ack": ack_disabled,
        "disable_waiting_ack_diagnostic": ack_disabled_diagnostic,
        "ack_semantics": "socket_send_only" if ack_disabled else "controller_ack_observed",
        "controller_acceptance_measured": async_mode != ASYNC_SOCKET_SEND_SUPERVISED and not ack_disabled,
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
        "required_env": (
            ([ASYNC_STREAMING_ENV] if async_mode != ASYNC_DISABLED else [])
            + ([SOCKET_SEND_ONLY_ENV] if async_mode == ASYNC_SOCKET_SEND_SUPERVISED else [])
        ),
        "env": env_snapshot_500hz(),
        "confirmation_flag": args.i_understand_this_connects_to_real_controller,
        "pgmode_confirmation_flag": args.i_confirm_controller_is_in_pgmode_simulation,
        "pgmode_simulation_confirmed": pgmode_summary.get("overall_result") == "ok",
        "pgmode_summary": pgmode_summary,
        "server_env_overrides": {},
        "cartesian_control_enable": cartesian_enabled,
        "cartesian_control_allow_in_controller_simulation": cartesian_controller_sim_allowed,
        "network_state_pub_rate_hz": as_float(config.network.get("state_pub_rate_hz")),
        "logging_directory": sections.get("logging", {}).get("directory"),
    }
    if resolve_info:
        result.update(resolve_info)
    return result


def preflight(args: argparse.Namespace, *, run_pgmode: bool = True) -> tuple[ParsedConfig, dict[str, Any]]:
    config_path = (args.root / args.config).resolve() if not args.config.is_absolute() else args.config.resolve()
    if not config_path.is_file():
        raise Acceptance500HzError(f"config not found: {config_path}")
    artifact_dir = args.artifact_dir.resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    resolved_config_path, resolve_info = write_resolved_config(
        args,
        config_path,
        artifact_dir,
        command_timeout_sec=getattr(args, "command_timeout_sec", None),
    )
    config = load_config(resolved_config_path)
    sections = yaml_sections(resolved_config_path)
    if "servo" in sections:
        config.servo = sections["servo"]
    if "network" in sections:
        config.network = sections["network"]
    if "logging" in sections:
        config.logging = sections["logging"]
    return config, validate_config_and_env(
        args,
        config,
        sections,
        run_pgmode=run_pgmode,
        resolve_info=resolve_info,
    )


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


def joint_target_packet(seq: int, session_id: str, q_targets_by_arm: dict[str, list[float]]) -> dict[str, Any]:
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
        "left": {"q_target_deg": q_targets_by_arm["left"]} if "left" in q_targets_by_arm else {"mode": "Hold"},
        "right": {"q_target_deg": q_targets_by_arm["right"]} if "right" in q_targets_by_arm else {"mode": "Hold"},
    }


def start_server(args: argparse.Namespace, log_path: Path, preflight_result: dict[str, Any]) -> subprocess.Popen[str]:
    server = (args.root / args.server).resolve() if not args.server.is_absolute() else args.server.resolve()
    if not server.is_file():
        raise Acceptance500HzError(f"server binary not found: {server}")
    resolved_config = preflight_result.get("resolved_config")
    config_path = Path(str(resolved_config)) if resolved_config else (
        (args.root / args.config).resolve() if not args.config.is_absolute() else args.config.resolve()
    )
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
        "async_streaming_enabled",
        "async_streaming_mode",
        "async_streaming_policy",
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
        "async_streaming",
    )
    out = {key: snapshot.get(key) for key in top_keys if key in snapshot}
    state = arm_state(snapshot, arm)
    if state is not None:
        out[arm] = {key: state.get(key) for key in arm_keys if key in state}
    return out


def state_excerpt_for_arms(snapshot: dict[str, Any], arms: tuple[str, ...]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for arm in arms:
        arm_excerpt = state_excerpt(snapshot, arm)
        for key, value in arm_excerpt.items():
            if key not in out:
                out[key] = value
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


def start_state_ready_for_arms(snapshot: dict[str, Any], arms: tuple[str, ...]) -> bool:
    return all(start_state_ready(snapshot, arm) for arm in arms)


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
    arms = active_send_arms(args)
    while time.monotonic() < deadline:
        snapshots = list(capture.snapshots)
        for snapshot in snapshots[scanned:]:
            latest = snapshot
            if start_state_ready_for_arms(snapshot, arms):
                return snapshot
            if snapshot.get("fault_latched") is True:
                raise Acceptance500HzError(
                    "rb_servo_server published a fault-latched startup state:\n"
                    + json.dumps(state_excerpt_for_arms(snapshot, arms), indent=2, sort_keys=True),
                    failure_phase="startup",
                    failure_classification=classify_snapshot_failure(snapshot, "startup"),
                    snapshot=snapshot,
                )
        scanned = len(snapshots)
        if proc is not None and proc.poll() is not None:
            break
        time.sleep(0.02)
    if latest is not None:
        raise Acceptance500HzError(
            "received rb_servo_server state packets, but no valid rbpodo no-op start state was observed:\n"
            + json.dumps(state_excerpt_for_arms(latest, arms), indent=2, sort_keys=True),
            failure_phase="startup",
            snapshot=latest,
        )
    raise Acceptance500HzError(
        state_stream_timeout_message(proc, log_path, state_endpoint),
        failure_phase="startup",
        failure_classification=classify_error_text(state_stream_timeout_message(proc, log_path, state_endpoint), "startup"),
    )


def wait_for_armed(
    capture: StateCapture,
    args: argparse.Namespace,
    after_ns: int,
) -> dict[str, Any]:
    deadline = time.monotonic() + args.startup_timeout_sec
    scanned = 0
    latest: dict[str, Any] | None = None
    arms = active_send_arms(args)
    while time.monotonic() < deadline:
        snapshots = list(capture.snapshots)
        for snapshot in snapshots[scanned:]:
            latest = snapshot
            if int(snapshot.get("host_time_ns", -1)) < after_ns:
                continue
            if snapshot.get("fault_latched") is True:
                raise Acceptance500HzError(
                    "fault latched while arming rb_servo_server:\n"
                    + json.dumps(state_excerpt_for_arms(snapshot, arms), indent=2, sort_keys=True),
                    failure_phase="startup",
                    failure_classification=classify_snapshot_failure(snapshot, "startup"),
                    snapshot=snapshot,
                )
            if snapshot.get("motion_state") in {"ArmedHold", "Running"} and start_state_ready_for_arms(snapshot, arms):
                return snapshot
        scanned = len(snapshots)
        time.sleep(0.02)
    detail = json.dumps(state_excerpt_for_arms(latest or {}, arms), indent=2, sort_keys=True)
    raise Acceptance500HzError(
        f"timed out waiting for rb_servo_server ArmedHold state:\n{detail}",
        failure_phase="startup",
        snapshot=latest,
    )


def send_noop_stream(
    args: argparse.Namespace,
    preflight_result: dict[str, Any],
    q_noop_by_arm: dict[str, list[float]],
    artifact_path: Path,
    session_id: str,
    *,
    command_rate_hz: float = COMMAND_RATE_HZ,
) -> CommandRunMetrics:
    host = str(preflight_result["command_host"])
    port = int(preflight_result["command_port"])
    if not math.isfinite(command_rate_hz) or command_rate_hz <= 0.0:
        raise Acceptance500HzError("command_rate_hz must be finite and positive")
    command_period_sec = 1.0 / command_rate_hz
    expected_count = max(1, int(round(float(args.duration_sec) * command_rate_hz)))
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
            if lateness > command_period_sec:
                deadline_misses += 1
                max_lateness_us = max(max_lateness_us, lateness * 1e6)
            packet = joint_target_packet(seq, session_id, q_noop_by_arm)
            if start_host_ns == 0:
                start_host_ns = int(packet["host_time_ns"])
            end_host_ns = int(packet["host_time_ns"])
            send_udp_with_socket(sock, host, port, packet, handle)
            seq += 1
            next_send += command_period_sec
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
        command_rate_hz=command_rate_hz,
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
    socket_send_only_count = 0
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
        semantics = str(
            last_send.get("send_acceptance_semantics")
            or last_send.get("last_controller_acceptance_semantics")
            or ""
        )
        if semantics == "socket_send_only":
            socket_send_only_count += 1
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
        async_streaming = state.get("async_streaming")
        if isinstance(async_streaming, dict):
            async_duration = finite_number(async_streaming.get("last_async_send_duration_us"))
            if async_duration is not None:
                send_durations.append(async_duration)
            if async_streaming.get("last_controller_acceptance_semantics") == "controller_ack_observed":
                acceptance_count += 1
            if async_streaming.get("last_async_acceptance_semantics") == "socket_send_only":
                socket_send_only_count += 1
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
        "socket_send_only_count": socket_send_only_count,
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


def async_arm_state(snapshot: dict[str, Any], arm: str) -> dict[str, Any] | None:
    state = arm_state(snapshot, arm)
    if state is None:
        return None
    value = state.get("async_streaming")
    return value if isinstance(value, dict) else None


def nonnegative_int(value: Any) -> int | None:
    number = finite_number(value)
    if number is None or number < 0:
        return None
    return int(number)


def max_counter(samples: list[dict[str, Any]], key: str) -> int:
    values = [nonnegative_int(sample.get(key)) for sample in samples]
    return max((value for value in values if value is not None), default=0)


def supervision_state_from_samples(samples: list[dict[str, Any]], key: str) -> str:
    if not samples:
        return "unknown"
    rank = {"ok": 0, "warning": 1, "fault": 2}
    worst = "unknown"
    worst_rank = -1
    for sample in samples:
        value = str(sample.get(key) or "").strip().lower()
        if value not in rank:
            continue
        if rank[value] > worst_rank:
            worst = value
            worst_rank = rank[value]
    return worst


def update_rate_from_timestamps(samples: list[dict[str, Any]], key: str) -> float | None:
    timestamps: list[int] = []
    previous: int | None = None
    for sample in samples:
        value = nonnegative_int(sample.get(key))
        if value is None or value <= 0:
            continue
        if previous is None or value != previous:
            timestamps.append(value)
            previous = value
    if len(timestamps) < 2:
        return None
    elapsed_sec = (timestamps[-1] - timestamps[0]) / 1e9
    if elapsed_sec <= 0.0:
        return None
    return (len(timestamps) - 1) / elapsed_sec


def async_streaming_metrics(states: list[dict[str, Any]], arm: str) -> dict[str, Any]:
    samples = [
        sample
        for snapshot in states
        for sample in [async_arm_state(snapshot, arm)]
        if sample is not None
    ]
    q_ref_update_ages = [
        value
        for value in (finite_number(sample.get("q_ref_update_age_ms")) for sample in samples)
        if value is not None
    ]
    tcp_ref_update_ages = [
        value
        for value in (finite_number(sample.get("tcp_ref_update_age_ms")) for sample in samples)
        if value is not None
    ]
    q_ref_target_errors = [
        value
        for value in (finite_number(sample.get("q_ref_target_error_deg_max")) for sample in samples)
        if value is not None
    ]
    tcp_ref_target_errors = [
        value
        for value in (finite_number(sample.get("tcp_ref_target_error_m")) for sample in samples)
        if value is not None
    ]
    backlogs = [
        value
        for value in (
            nonnegative_int(sample.get("async_worker_backlog") if "async_worker_backlog" in sample else sample.get("worker_backlog"))
            for sample in samples
        )
        if value is not None
    ]
    enabled_observed = any(sample.get("enabled") is True for sample in samples)
    modes = [str(sample.get("mode")) for sample in samples if sample.get("mode") not in (None, "")]
    mode_counts = Counter(modes)
    mode = mode_counts.most_common(1)[0][0] if mode_counts else None
    commands_enqueued = max_counter(samples, "commands_enqueued_total")
    commands_sent = max_counter(samples, "commands_sent_total")
    commands_acked = max_counter(samples, "commands_acked_total")
    commands_socket_sent = max_counter(samples, "commands_socket_sent_total")
    commands_overwritten = max_counter(samples, "commands_overwritten_total")
    commands_dropped = max_counter(samples, "commands_dropped_total")
    ack_timeout_count = max_counter(samples, "ack_timeout_count")
    q_ref_watchdog_miss_count = max_counter(samples, "q_ref_watchdog_miss_count")
    tcp_ref_watchdog_miss_count = max_counter(samples, "tcp_ref_watchdog_miss_count")
    reference_supervision_fault_count = max_counter(samples, "reference_supervision_fault_count")
    denominator = max(commands_enqueued, commands_sent, 1)
    return {
        "source": "async_streaming",
        "sample_count": len(samples),
        "enabled_observed": enabled_observed,
        "mode": mode,
        "commands_enqueued_total": commands_enqueued,
        "commands_sent_total": commands_sent,
        "commands_acked_total": commands_acked,
        "commands_socket_sent_total": commands_socket_sent,
        "socket_send_only_count": commands_socket_sent,
        "controller_ack_observed_count": commands_acked,
        "commands_overwritten_total": commands_overwritten,
        "commands_dropped_total": commands_dropped,
        "commands_overwritten_ratio": commands_overwritten / denominator,
        "commands_dropped_ratio": commands_dropped / denominator,
        "ack_timeout_count": ack_timeout_count,
        "q_ref_watchdog_miss_count": q_ref_watchdog_miss_count,
        "tcp_ref_watchdog_miss_count": tcp_ref_watchdog_miss_count,
        "q_ref_update_rate_hz": update_rate_from_timestamps(samples, "last_q_ref_update_host_time_ns"),
        "tcp_ref_update_rate_hz": update_rate_from_timestamps(samples, "last_tcp_ref_update_host_time_ns"),
        "q_ref_update_age_ms": metric_block(q_ref_update_ages),
        "tcp_ref_update_age_ms": metric_block(tcp_ref_update_ages),
        "q_ref_update_age_p95_ms": percentile(q_ref_update_ages, 0.95),
        "tcp_ref_update_age_p95_ms": percentile(tcp_ref_update_ages, 0.95),
        "q_ref_target_error_deg_max": max(q_ref_target_errors) if q_ref_target_errors else None,
        "tcp_ref_target_error_m_max": max(tcp_ref_target_errors) if tcp_ref_target_errors else None,
        "async_worker_backlog_max": max(backlogs) if backlogs else 0,
        "supervision_state": supervision_state_from_samples(samples, "supervision_state"),
        "async_supervision_state": supervision_state_from_samples(samples, "async_supervision_state"),
        "reference_supervision_state": supervision_state_from_samples(samples, "reference_supervision_state"),
        "reference_supervision_reason": next(
            (
                str(sample.get("reference_supervision_reason"))
                for sample in reversed(samples)
                if sample.get("reference_supervision_reason") not in (None, "")
            ),
            None,
        ),
        "reference_supervision_fault_count": reference_supervision_fault_count,
        "supervision_fault_count": max(reference_supervision_fault_count, ack_timeout_count),
    }


def error_text_from_mapping(value: Any) -> str:
    if not isinstance(value, dict):
        return ""
    keys = (
        "backend_error_kind",
        "backend_error_name",
        "backend_error_code",
        "backend_op",
        "error_kind",
        "error_name",
        "error_code",
        "send_error_kind",
        "send_error_name",
        "send_error_message",
        "reason",
        "message",
        "verdict",
    )
    parts = [str(value.get(key)) for key in keys if value.get(key) not in (None, "")]
    return " ".join(parts)


def is_ack_timeout_text(text: str) -> bool:
    lower = text.lower()
    has_timeout = "timeout" in lower or "timed out" in lower
    has_ack_context = (
        re.search(r"\back\b|ack_|_ack|ack-|acknowledg", lower) is not None
        or "acknowledg" in lower
        or "move_servo_j" in lower
        or "servolj" in lower
        or "sendservoj" in lower
        or "transporttimeout" in lower
    )
    return has_timeout and has_ack_context


def arm_from_error_text(text: str) -> str | None:
    lower = text.lower()
    if " on left" in lower or "left_" in lower:
        return "left"
    if " on right" in lower or "right_" in lower:
        return "right"
    return None


def observation_from_error(
    snapshot: dict[str, Any],
    index: int,
    arm: str | None,
    source: str,
    fields: dict[str, Any],
) -> dict[str, Any] | None:
    text = error_text_from_mapping(fields)
    if not text:
        return None
    backend_kind = str(fields.get("backend_error_kind") or fields.get("send_error_kind") or "")
    error_name = str(fields.get("backend_error_name") or fields.get("error_name") or fields.get("send_error_name") or "")
    if backend_kind == "SuppressedByPolicy" or error_name == "fault_latched":
        return None
    accepted = fields.get("accepted")
    send_ok = fields.get("send_ok")
    verdict = str(fields.get("verdict") or "")
    backend_op = str(fields.get("backend_op") or "")
    is_failure = (
        accepted is False
        or send_ok is False
        or verdict == "SendFailure"
        or bool(fields.get("transport_fault"))
        or backend_kind not in {"", "None"}
        or (backend_op == "SendServoJ" and backend_kind)
    )
    if not is_failure:
        return None
    resolved_arm = arm or arm_from_error_text(text)
    if resolved_arm not in ARMS:
        resolved_arm = "unknown"
    host_time_ns = int(finite_number(snapshot.get("loop_start_time_ns")) or finite_number(snapshot.get("host_time_ns")) or 0)
    return {
        "index": index,
        "host_time_ns": host_time_ns,
        "arm": resolved_arm,
        "source": source,
        "ack_timeout": is_ack_timeout_text(text),
        "text": text,
    }


def send_failure_observations(states: list[dict[str, Any]]) -> list[dict[str, Any]]:
    observations: list[dict[str, Any]] = []
    seen: set[tuple[int, str, str]] = set()
    for index, snapshot in enumerate(states):
        fault_context = snapshot.get("fault_context")
        if isinstance(fault_context, dict):
            for key in ("top_level", "left", "right"):
                context = fault_context.get(key)
                if not isinstance(context, dict):
                    continue
                arm = str(context.get("arm") or key)
                obs = observation_from_error(snapshot, index, arm, f"fault_context.{key}", context)
                if obs is not None:
                    token = (index, str(obs["arm"]), str(obs["source"]))
                    if token not in seen:
                        seen.add(token)
                        observations.append(obs)
        for arm in ARMS:
            state = arm_state(snapshot, arm)
            if state is None:
                continue
            last_send = state.get("last_send")
            if isinstance(last_send, dict):
                obs = observation_from_error(snapshot, index, arm, f"{arm}.last_send", last_send)
                if obs is not None:
                    token = (index, str(obs["arm"]), str(obs["source"]))
                    if token not in seen:
                        seen.add(token)
                        observations.append(obs)
            send_fields = {
                "send_ok": state.get("send_ok"),
                "send_error_kind": state.get("send_error_kind"),
                "send_error_name": state.get("send_error_name"),
                "send_error_message": state.get("send_error_message"),
            }
            obs = observation_from_error(snapshot, index, arm, f"{arm}.send_result", send_fields)
            if obs is not None:
                token = (index, str(obs["arm"]), str(obs["source"]))
                if token not in seen:
                    seen.add(token)
                    observations.append(obs)
    observations.sort(key=lambda item: (int(item.get("host_time_ns") or 0), int(item.get("index") or 0)))
    return observations


def send_failure_summary(
    states: list[dict[str, Any]],
    *,
    phase: str,
    start_host_time_ns: int = 0,
) -> dict[str, Any]:
    observations = send_failure_observations(states)
    ack_timeout_count_by_arm = {arm: 0 for arm in ARMS}
    ack_timeout_count_by_arm["unknown"] = 0
    for observation in observations:
        if not observation.get("ack_timeout"):
            continue
        arm = str(observation.get("arm") or "unknown")
        ack_timeout_count_by_arm[arm] = ack_timeout_count_by_arm.get(arm, 0) + 1
    first = observations[0] if observations else None
    first_elapsed_sec: float | None = None
    if first is not None:
        host_ns = int(first.get("host_time_ns") or 0)
        base_ns = start_host_time_ns if start_host_time_ns > 0 else host_ns
        first_elapsed_sec = max(0.0, (host_ns - base_ns) / 1e9) if host_ns > 0 else None
    timeout_count = sum(ack_timeout_count_by_arm.values())
    return {
        "ack_timeout_count_by_arm": {arm: ack_timeout_count_by_arm.get(arm, 0) for arm in ARMS},
        "ack_timeout_count_unknown_arm": ack_timeout_count_by_arm.get("unknown", 0),
        "warmup_timeout_count": timeout_count if phase == "warmup" else 0,
        "measurement_timeout_count": timeout_count if phase == "measurement" else 0,
        "first_send_failure_arm": first.get("arm") if first else None,
        "first_send_failure_index": first.get("index") if first else None,
        "first_send_failure_elapsed_sec": first_elapsed_sec,
        "first_send_failure_source": first.get("source") if first else None,
        "first_send_failure_ack_timeout": bool(first.get("ack_timeout")) if first else False,
        "first_send_failure_text": first.get("text") if first else None,
    }


def classify_error_text(text: str, phase: str | None) -> str | None:
    if not text:
        return None
    lower = text.lower()
    if "requires rb_allow" in lower or "missing --" in lower:
        return "preflight_env_missing"
    if is_ack_timeout_text(text):
        if phase == "warmup":
            return "warmup_ack_timeout"
        if phase == "measurement":
            return "measurement_ack_timeout"
        if phase == "startup":
            return "startup_ack_timeout"
        return "ack_timeout"
    if "deadline" in lower:
        return "deadline_limited"
    return None


def classify_snapshot_failure(snapshot: dict[str, Any] | None, phase: str) -> str | None:
    if not snapshot:
        return None
    failures = send_failure_summary([snapshot], phase=phase)
    if sum(failures.get("ack_timeout_count_by_arm", {}).values()) > 0:
        return classify_error_text(str(failures.get("first_send_failure_text") or ""), phase)
    if snapshot.get("send_command_deadline_missed") is True:
        return "deadline_limited"
    return f"{phase}_fault_latched" if snapshot.get("fault_latched") is True else None


def classify_acceptance_summary(summary: dict[str, Any], *, phase: str) -> str:
    if summary.get("result") == "pass":
        return "pass"
    timeout_count = int(summary.get("warmup_timeout_count") or 0) + int(summary.get("measurement_timeout_count") or 0)
    if timeout_count > 0:
        if phase == "warmup":
            return "warmup_ack_timeout"
        if phase == "measurement":
            return "measurement_ack_timeout"
        return "ack_timeout"
    deadline_counts = summary.get("deadline_miss_count_by_arm")
    if isinstance(deadline_counts, dict) and sum(int(value or 0) for value in deadline_counts.values()) > 0:
        return "deadline_limited"
    if int(summary.get("send_deadline_missed_count") or 0) > 0:
        return "deadline_limited"
    failures = summary.get("threshold_failures")
    if isinstance(failures, list):
        joined = "\n".join(str(item) for item in failures)
        if "controller_ack_observed_count" in joined and "sdk_ack_worker" in joined:
            return "async_ack_missing"
        if "commands_overwritten_ratio" in joined:
            return "async_overwrite_limited"
        if "commands_dropped_ratio" in joined:
            return "async_drop_limited"
        if "reference_supervision_state" in joined or "q_ref_update" in joined or "tcp_ref_update" in joined:
            return "reference_supervision_failed"
        if "servo_loop_blocked_by_ack" in joined:
            return "async_servo_loop_blocked"
    return "threshold_failure"


def classify_servo_loop_blocked_by_ack(
    args: argparse.Namespace,
    async_mode: str,
    async_metrics: dict[str, Any],
    *,
    send_deadline_missed_count: int,
    servo_jitter: dict[str, Any],
) -> bool | str:
    if async_mode == ASYNC_DISABLED:
        return "unknown"
    if not async_metrics.get("enabled_observed"):
        return "unknown"
    if send_deadline_missed_count > args.max_deadline_miss_count:
        return True
    p99_jitter = finite_number(servo_jitter.get("p99")) if isinstance(servo_jitter, dict) else None
    if p99_jitter is not None and p99_jitter > args.max_servo_jitter_p99_ms:
        return True
    return False


def summarize_acceptance(
    args: argparse.Namespace,
    config: ParsedConfig,
    preflight_result: dict[str, Any],
    states: list[dict[str, Any]],
    command_run: CommandRunMetrics,
    artifact_dir: Path,
    server_returncode: int | None,
    servo_log: dict[str, Any] | None,
    *,
    phase: str = "measurement",
) -> dict[str, Any]:
    async_mode = str(preflight_result.get("async_mode") or async_mode_value(args))
    state_metrics = state_stream_metrics(states, args.arm)
    state_metrics_by_arm = {arm: state_stream_metrics(states, arm) for arm in ARMS}
    async_metrics = async_streaming_metrics(states, args.arm)
    async_metrics_by_arm = {arm: async_streaming_metrics(states, arm) for arm in ARMS}
    servo_metrics: dict[str, Any] | None = None
    servo_metrics_by_arm: dict[str, dict[str, Any]] = {}
    if servo_log and isinstance(servo_log.get("path"), str):
        servo_log_path = Path(servo_log["path"])
        for arm in ARMS:
            servo_metrics_by_arm[arm] = parse_servo_log_metrics(
                servo_log_path,
                arm,
                command_run.start_host_time_ns,
                command_run.end_host_time_ns,
            )
        servo_metrics = servo_metrics_by_arm.get(args.arm)

    timing_source = servo_metrics if servo_metrics and servo_metrics.get("send_count") else state_metrics
    timing_source_by_arm: dict[str, dict[str, Any]] = {}
    for arm in ARMS:
        servo_arm_metrics = servo_metrics_by_arm.get(arm)
        timing_source_by_arm[arm] = (
            servo_arm_metrics
            if servo_arm_metrics and servo_arm_metrics.get("send_count")
            else state_metrics_by_arm[arm]
        )
    q_actual_drift = state_metrics.get("q_actual_drift_from_start_deg")
    q_actual_drift_by_arm = {
        arm: state_metrics_by_arm[arm].get("q_actual_drift_from_start_deg")
        for arm in ARMS
    }
    q_ref_drift = state_metrics.get("q_ref_drift_from_start_deg")
    physical_motion_detected = any(
        finite_number(q_actual_drift_by_arm.get(arm)) is not None
        and float(q_actual_drift_by_arm[arm]) > args.max_physical_motion_deg
        for arm in active_send_arms(args)
    )
    expected_send_count = command_run.expected_command_count
    send_count = int(timing_source.get("send_count") or command_run.command_count)
    timing_acceptance_count = int(timing_source.get("controller_acceptance_observed_count") or 0)
    async_ack_count = int(async_metrics.get("controller_ack_observed_count") or 0)
    if async_mode == ASYNC_SDK_ACK_WORKER:
        acceptance_count = async_ack_count
    elif async_mode == ASYNC_SOCKET_SEND_SUPERVISED:
        acceptance_count = 0
    else:
        acceptance_count = max(timing_acceptance_count, async_ack_count)
    socket_send_only_count = max(
        int(timing_source.get("socket_send_only_count") or 0),
        int(async_metrics.get("socket_send_only_count") or 0),
    )
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
    servo_loop_blocked_by_ack = classify_servo_loop_blocked_by_ack(
        args,
        async_mode,
        async_metrics,
        send_deadline_missed_count=send_deadline_missed_count,
        servo_jitter=servo_jitter,
    )
    send_duration_p99_us_by_arm = {
        arm: (
            timing_source_by_arm[arm].get("send_duration_us") or {}
        ).get("p99")
        if isinstance(timing_source_by_arm[arm].get("send_duration_us"), dict)
        else None
        for arm in ARMS
    }
    deadline_miss_count_by_arm = {
        arm: int(timing_source_by_arm[arm].get("send_deadline_missed_count") or 0)
        for arm in ARMS
    }
    send_failure_details = send_failure_summary(
        states,
        phase=phase,
        start_host_time_ns=command_run.start_host_time_ns,
    )
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
        async_mode=async_mode,
        async_metrics=async_metrics,
        controller_ack_observed_count=acceptance_count,
        socket_send_only_count=socket_send_only_count,
        servo_loop_blocked_by_ack=servo_loop_blocked_by_ack,
        timing_source_name=str(timing_source.get("source")),
        physical_motion_detected=physical_motion_detected,
    )
    ack_timeout_total = int(send_failure_details.get("warmup_timeout_count") or 0) + int(
        send_failure_details.get("measurement_timeout_count") or 0
    )
    if ack_timeout_total > 0:
        failures.append(f"{phase} observed rbpodo Servo J ACK timeout count {ack_timeout_total}")
    if sum(deadline_miss_count_by_arm.values()) > args.max_deadline_miss_count and ack_timeout_total == 0:
        failures.append(
            f"deadline_miss_count_by_arm total {sum(deadline_miss_count_by_arm.values())} "
            f"exceeds {args.max_deadline_miss_count}"
        )
    result = "fail" if failures else "pass"
    diagnostic_only = bool(preflight_result.get("disable_waiting_ack_diagnostic"))
    if failures:
        result_reason = "thresholds applied and failed"
    elif async_mode == ASYNC_SOCKET_SEND_SUPERVISED:
        result_reason = "500 Hz socket-send supervised async thresholds satisfied"
    elif async_mode == ASYNC_SDK_ACK_WORKER:
        result_reason = "500 Hz SDK ACK-worker async thresholds satisfied"
    elif diagnostic_only:
        result_reason = "500 Hz ACK-off diagnostic thresholds satisfied; controller ACK acceptance not measured"
    else:
        result_reason = "500 Hz no-op thresholds satisfied"
    failure_classification = None if result == "pass" else classify_acceptance_summary(
        {
            "result": result,
            **send_failure_details,
            "deadline_miss_count_by_arm": deadline_miss_count_by_arm,
            "send_deadline_missed_count": send_deadline_missed_count,
            "threshold_failures": failures,
        },
        phase=phase,
    )
    ack_disabled = bool(preflight_result.get("disable_waiting_ack"))
    caveat = (
        "ACK-off diagnostic only: socket send success is not controller ACK acceptance; "
        "controller boxes are real, physical robot motion is not expected or approved"
        if diagnostic_only
        else (
            (
                "Async socket-send supervised controller-simulation evidence; q_ref/tcp_ref "
                "supervision is required because socket sends are not controller ACK acceptance; "
                "physical robot motion is not expected or approved"
            )
            if async_mode == ASYNC_SOCKET_SEND_SUPERVISED
            else (
                "Async SDK ACK-worker controller-simulation evidence; ACK waiting runs outside "
                "the servo loop; physical robot motion is not expected or approved"
            )
            if async_mode == ASYNC_SDK_ACK_WORKER
            else (
                "rbpodo controller-simulation no-op evidence; controller boxes are real, "
                "physical robot motion is not expected or approved"
            )
        )
    )
    summary = {
        "schema": SCHEMA,
        "result": result,
        "result_reason": result_reason,
        "failure_phase": phase if result != "pass" else None,
        "failure_classification": failure_classification,
        "artifact_dir": str(artifact_dir.resolve()),
        "config": str(config.path),
        "source_config": preflight_result.get("source_config"),
        "resolved_config": preflight_result.get("resolved_config"),
        "mode": MODE,
        "acceptance_stage": (
            f"noop_500hz_{async_mode}"
            if async_mode != ASYNC_DISABLED
            else MODE
        ),
        "arm": args.arm,
        "send_arms": list(active_send_arms(args)),
        "duration_sec": args.duration_sec,
        "command_rate_hz": command_run.command_rate_hz,
        "servo_rate_hz": COMMAND_RATE_HZ,
        "async_mode": async_mode,
        "rbpodo_async_streaming_enabled": async_mode != ASYNC_DISABLED,
        "reference_supervision_required": reference_supervision_required(args),
        "disable_waiting_ack": ack_disabled,
        "disable_waiting_ack_diagnostic": diagnostic_only,
        "ack_semantics": preflight_result.get("ack_semantics"),
        "controller_acceptance_measured": bool(preflight_result.get("controller_acceptance_measured", True)),
        "command_timeout_sec_left": preflight_result.get("command_timeout_sec_left"),
        "command_timeout_sec_right": preflight_result.get("command_timeout_sec_right"),
        "warmup_enabled": phase == "warmup",
        "warmup_result": result if phase == "warmup" else "not_run",
        "expected_send_count": expected_send_count,
        "udp_command_count": command_run.command_count,
        "send_count": send_count,
        "send_count_source": timing_source.get("source"),
        "achieved_udp_command_rate_hz": command_run.command_count / command_run.elapsed_sec,
        "controller_acceptance_observed_count": acceptance_count,
        "controller_ack_observed_count": acceptance_count,
        "controller_acceptance_ratio": acceptance_count / send_count if send_count > 0 else None,
        "socket_send_only_count": socket_send_only_count,
        "send_success_count": int(timing_source.get("send_success_count") or 0),
        "send_failure_count": send_failure_count,
        "ack_observed_count": int(timing_source.get("ack_observed_count") or 0),
        "send_duration_us": send_duration,
        "send_duration_p99_us_by_arm": send_duration_p99_us_by_arm,
        "servo_jitter_ms": servo_jitter,
        "send_deadline_missed_count": send_deadline_missed_count,
        "deadline_miss_count_by_arm": deadline_miss_count_by_arm,
        "send_period_overrun_count": int(timing_source.get("send_period_overrun_count") or 0),
        "command_sender_deadline_missed_count": command_run.sender_deadline_missed_count,
        "command_sender_max_lateness_us": command_run.max_sender_lateness_us,
        "state_packet_count": len(states),
        "state_valid_ratio": state_metrics.get("state_valid_ratio"),
        "state_age_us": state_metrics.get("state_age_us"),
        "q_actual_drift_from_start_deg": q_actual_drift,
        "q_actual_drift_from_start_deg_by_arm": q_actual_drift_by_arm,
        "q_ref_drift_from_start_deg": q_ref_drift,
        "q_target_drift_from_start_deg": state_metrics.get("q_target_drift_from_start_deg"),
        "physical_motion_expected": False,
        "physical_motion_detected": physical_motion_detected,
        "physical_motion_warning_threshold_deg": args.max_physical_motion_deg,
        "fault_latched": state_metrics.get("fault_latched"),
        "worker_path_used": state_metrics.get("worker_path_used"),
        "worker_command_drops_total": state_metrics.get("worker_command_drops_total"),
        "worker_pending_overwrites_total": state_metrics.get("worker_pending_overwrites_total"),
        "commands_overwritten_total": async_metrics.get("commands_overwritten_total"),
        "commands_dropped_total": async_metrics.get("commands_dropped_total"),
        "commands_overwritten_ratio": async_metrics.get("commands_overwritten_ratio"),
        "commands_dropped_ratio": async_metrics.get("commands_dropped_ratio"),
        "async_worker_backlog_max": async_metrics.get("async_worker_backlog_max"),
        "reference_supervision_state": async_metrics.get("reference_supervision_state"),
        "reference_supervision_reason": async_metrics.get("reference_supervision_reason"),
        "q_ref_update_rate_hz": async_metrics.get("q_ref_update_rate_hz"),
        "tcp_ref_update_rate_hz": async_metrics.get("tcp_ref_update_rate_hz"),
        "q_ref_update_age_ms": async_metrics.get("q_ref_update_age_ms"),
        "q_ref_update_age_p95_ms": async_metrics.get("q_ref_update_age_p95_ms"),
        "tcp_ref_update_age_ms": async_metrics.get("tcp_ref_update_age_ms"),
        "tcp_ref_update_age_p95_ms": async_metrics.get("tcp_ref_update_age_p95_ms"),
        "q_ref_target_error_deg_max": async_metrics.get("q_ref_target_error_deg_max"),
        "tcp_ref_target_error_m_max": async_metrics.get("tcp_ref_target_error_m_max"),
        "supervision_fault_count": async_metrics.get("supervision_fault_count"),
        "servo_loop_blocked_by_ack": servo_loop_blocked_by_ack,
        "server_returncode": server_returncode,
        "state_stream": str((artifact_dir / "state_stream.jsonl").resolve()),
        "command_packets": str((artifact_dir / "command_packets.jsonl").resolve()),
        "rb_servo_server_log": str((artifact_dir / "rb_servo_server.log").resolve()),
        "servo_log": servo_log,
        "servo_log_metrics": servo_metrics,
        "servo_log_metrics_by_arm": servo_metrics_by_arm,
        "state_stream_metrics": state_metrics,
        "state_stream_metrics_by_arm": state_metrics_by_arm,
        "async_streaming_metrics": async_metrics,
        "async_streaming_metrics_by_arm": async_metrics_by_arm,
        "safety_preflight": preflight_result,
        "thresholds": {
            "min_send_count_ratio": args.min_send_count_ratio,
            "min_controller_acceptance_ratio": args.min_controller_acceptance_ratio,
            "max_send_duration_p99_us": args.max_send_duration_p99_us,
            "max_servo_jitter_p99_ms": args.max_servo_jitter_p99_ms,
            "max_deadline_miss_count": args.max_deadline_miss_count,
            "max_worker_drop_count": args.max_worker_drop_count,
            "max_overwrite_ratio": getattr(args, "max_overwrite_ratio", 0.05),
            "max_drop_ratio": getattr(args, "max_drop_ratio", 0.0),
            "max_q_ref_update_age_ms": getattr(args, "max_q_ref_update_age_ms", 50.0),
            "max_tcp_ref_update_age_ms": getattr(args, "max_tcp_ref_update_age_ms", 50.0),
            "min_q_ref_update_rate_hz": getattr(args, "min_q_ref_update_rate_hz", 20.0),
            "max_physical_motion_deg": args.max_physical_motion_deg,
            "max_reference_drift_deg": args.max_reference_drift_deg,
        },
        "threshold_failures": failures,
        "caveat": caveat,
    }
    summary.update(send_failure_details)
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
    async_mode: str,
    async_metrics: dict[str, Any],
    controller_ack_observed_count: int,
    socket_send_only_count: int,
    servo_loop_blocked_by_ack: bool | str,
    timing_source_name: str,
    physical_motion_detected: bool,
) -> list[str]:
    failures: list[str] = []
    min_send_count = math.floor(expected_send_count * args.min_send_count_ratio)
    if send_count < min_send_count:
        failures.append(f"send_count {send_count} below minimum {min_send_count} for expected {expected_send_count}")
    if async_mode == ASYNC_SDK_ACK_WORKER:
        if controller_ack_observed_count <= 0:
            failures.append("sdk_ack_worker controller_ack_observed_count must be > 0")
    elif async_mode == ASYNC_SOCKET_SEND_SUPERVISED:
        if socket_send_only_count <= 0:
            failures.append("socket_send_supervised socket_send_only_count must be > 0")
    elif not getattr(args, "disable_waiting_ack_diagnostic", False):
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
    elif async_mode != ASYNC_DISABLED and state_valid_ratio < 1.0:
        failures.append(f"state_valid_ratio {state_valid_ratio:.6f} below 1.0 for async acceptance")
    worker_drops = int(state_metrics.get("worker_command_drops_total") or 0)
    if state_metrics.get("worker_path_used") is True and worker_drops > args.max_worker_drop_count:
        failures.append(f"worker_command_drops_total {worker_drops} exceeds {args.max_worker_drop_count}")
    if async_mode != ASYNC_DISABLED:
        if async_metrics.get("enabled_observed") is not True:
            failures.append(f"{async_mode} async_streaming telemetry was not observed")
        if async_metrics.get("mode") not in {None, async_mode}:
            failures.append(f"async_streaming mode {async_metrics.get('mode')} did not match {async_mode}")
        if servo_loop_blocked_by_ack is not False:
            failures.append(f"servo_loop_blocked_by_ack was {servo_loop_blocked_by_ack}")
        overwrite_ratio = finite_number(async_metrics.get("commands_overwritten_ratio"))
        if overwrite_ratio is None:
            failures.append("commands_overwritten_ratio unavailable from async telemetry")
        elif overwrite_ratio > getattr(args, "max_overwrite_ratio", 0.05):
            failures.append(
                f"commands_overwritten_ratio {overwrite_ratio:.6f} exceeds {getattr(args, 'max_overwrite_ratio', 0.05):.6f}"
            )
        drop_ratio = finite_number(async_metrics.get("commands_dropped_ratio"))
        if drop_ratio is None:
            failures.append("commands_dropped_ratio unavailable from async telemetry")
        elif drop_ratio > getattr(args, "max_drop_ratio", 0.0):
            failures.append(f"commands_dropped_ratio {drop_ratio:.6f} exceeds {getattr(args, 'max_drop_ratio', 0.0):.6f}")
    if reference_supervision_required(args):
        reference_state = str(async_metrics.get("reference_supervision_state") or "unknown")
        if reference_state != "ok":
            failures.append(f"reference_supervision_state {reference_state} is not ok")
        q_ref_age_p95 = finite_number(async_metrics.get("q_ref_update_age_p95_ms"))
        if q_ref_age_p95 is None:
            failures.append("q_ref_update_age_p95_ms unavailable from async telemetry")
        elif q_ref_age_p95 > getattr(args, "max_q_ref_update_age_ms", 50.0):
            failures.append(
                f"q_ref_update_age_p95_ms {q_ref_age_p95:.3f} exceeds {getattr(args, 'max_q_ref_update_age_ms', 50.0):.3f}"
            )
        tcp_ref_age_p95 = finite_number(async_metrics.get("tcp_ref_update_age_p95_ms"))
        if tcp_ref_age_p95 is None:
            failures.append("tcp_ref_update_age_p95_ms unavailable from async telemetry")
        elif tcp_ref_age_p95 > getattr(args, "max_tcp_ref_update_age_ms", 50.0):
            failures.append(
                f"tcp_ref_update_age_p95_ms {tcp_ref_age_p95:.3f} exceeds {getattr(args, 'max_tcp_ref_update_age_ms', 50.0):.3f}"
            )
        q_ref_rate = finite_number(async_metrics.get("q_ref_update_rate_hz"))
        if q_ref_rate is None:
            failures.append("q_ref_update_rate_hz unavailable from async telemetry")
        elif q_ref_rate < getattr(args, "min_q_ref_update_rate_hz", 20.0):
            failures.append(
                f"q_ref_update_rate_hz {q_ref_rate:.3f} below {getattr(args, 'min_q_ref_update_rate_hz', 20.0):.3f}"
            )
        q_ref_target_error = finite_number(async_metrics.get("q_ref_target_error_deg_max"))
        if q_ref_target_error is None:
            failures.append("q_ref_target_error_deg_max unavailable from async telemetry")
        elif q_ref_target_error > args.max_reference_drift_deg:
            failures.append(
                f"q_ref_target_error_deg_max {q_ref_target_error:.6f} exceeds {args.max_reference_drift_deg:.6f}"
            )
        if int(async_metrics.get("supervision_fault_count") or 0) > 0:
            failures.append(f"supervision_fault_count {async_metrics.get('supervision_fault_count')} must be 0")
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
        "failure_phase",
        "failure_classification",
        "mode",
        "acceptance_stage",
        "arm",
        "send_arms",
        "async_mode",
        "rbpodo_async_streaming_enabled",
        "reference_supervision_required",
        "disable_waiting_ack",
        "disable_waiting_ack_diagnostic",
        "ack_semantics",
        "controller_acceptance_measured",
        "duration_sec",
        "command_timeout_sec_left",
        "command_timeout_sec_right",
        "warmup_enabled",
        "warmup_result",
        "warmup_timeout_count",
        "measurement_timeout_count",
        "first_send_failure_arm",
        "first_send_failure_index",
        "first_send_failure_elapsed_sec",
        "expected_send_count",
        "send_count",
        "controller_acceptance_observed_count",
        "controller_ack_observed_count",
        "controller_acceptance_ratio",
        "socket_send_only_count",
        "send_duration_p99_us",
        "send_duration_p99_us_by_arm",
        "servo_jitter_p99_ms",
        "send_deadline_missed_count",
        "deadline_miss_count_by_arm",
        "ack_timeout_count_by_arm",
        "worker_command_drops_total",
        "commands_overwritten_total",
        "commands_dropped_total",
        "async_worker_backlog_max",
        "reference_supervision_state",
        "q_ref_update_rate_hz",
        "q_ref_update_age_p95_ms",
        "tcp_ref_update_age_p95_ms",
        "supervision_fault_count",
        "servo_loop_blocked_by_ack",
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


def run_acceptance_once(
    args: argparse.Namespace,
    *,
    phase: str = "measurement",
    command_rate_hz: float = COMMAND_RATE_HZ,
) -> dict[str, Any]:
    args.root = args.root.resolve()
    config, preflight_result = preflight(args)
    artifact_dir = args.artifact_dir.resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    write_json(artifact_dir / "safety_preflight.json", preflight_result)
    write_json(artifact_dir / "pgmode_summary.json", preflight_result["pgmode_summary"])
    source_config = preflight_result.get("source_config")
    if isinstance(source_config, str) and Path(source_config).is_file():
        shutil.copy2(source_config, artifact_dir / "raw_config.yaml")
    if args.preflight_only:
        summary = {
            "schema": SCHEMA,
            "result": "pass",
            "result_reason": "preflight only",
            "artifact_dir": str(artifact_dir),
            "mode": MODE,
            "acceptance_stage": (
                f"noop_500hz_{preflight_result.get('async_mode')}"
                if preflight_result.get("async_mode") not in (None, ASYNC_DISABLED)
                else MODE
            ),
            "failure_phase": None,
            "failure_classification": None,
            "source_config": preflight_result.get("source_config"),
            "resolved_config": preflight_result.get("resolved_config"),
            "async_mode": preflight_result.get("async_mode", ASYNC_DISABLED),
            "rbpodo_async_streaming_enabled": bool(preflight_result.get("rbpodo_async_streaming_enabled")),
            "reference_supervision_required": bool(preflight_result.get("reference_supervision_required")),
            "disable_waiting_ack": bool(preflight_result.get("disable_waiting_ack")),
            "disable_waiting_ack_diagnostic": bool(preflight_result.get("disable_waiting_ack_diagnostic")),
            "ack_semantics": preflight_result.get("ack_semantics"),
            "controller_acceptance_measured": bool(preflight_result.get("controller_acceptance_measured", True)),
            "command_timeout_sec_left": preflight_result.get("command_timeout_sec_left"),
            "command_timeout_sec_right": preflight_result.get("command_timeout_sec_right"),
            "warmup_enabled": False,
            "warmup_result": "not_run",
            "warmup_timeout_count": 0,
            "measurement_timeout_count": 0,
            "first_send_failure_arm": None,
            "first_send_failure_index": None,
            "first_send_failure_elapsed_sec": None,
            "ack_timeout_count_by_arm": {arm: 0 for arm in ARMS},
            "send_duration_p99_us_by_arm": {arm: None for arm in ARMS},
            "deadline_miss_count_by_arm": {arm: 0 for arm in ARMS},
            "controller_ack_observed_count": 0,
            "socket_send_only_count": 0,
            "reference_supervision_state": "unknown",
            "q_ref_update_rate_hz": None,
            "q_ref_update_age_p95_ms": None,
            "tcp_ref_update_age_p95_ms": None,
            "commands_overwritten_total": 0,
            "commands_dropped_total": 0,
            "async_worker_backlog_max": 0,
            "supervision_fault_count": 0,
            "servo_loop_blocked_by_ack": "unknown",
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
        arms = active_send_arms(args)
        noop_targets = noop_targets_from_state(first_state, arms)
        q_actual_for_arms(first_state, arms)
        q_noop_by_arm = {arm: list(noop_targets[arm]["q_target_deg"]) for arm in arms}
        with (artifact_dir / "noop_target.json").open("w", encoding="utf-8") as handle:
            json.dump(
                {
                    "arm": args.arm,
                    "send_arms": list(arms),
                    "targets_by_arm": noop_targets,
                    "startup_state_excerpt": state_excerpt_for_arms(first_state, arms),
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
            q_noop_by_arm,
            artifact_dir / "command_packets.jsonl",
            session_id,
            command_rate_hz=command_rate_hz,
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
        phase=phase,
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


def run_acceptance_with_warmup(args: argparse.Namespace) -> dict[str, Any]:
    root_artifact_dir = args.artifact_dir.resolve()
    root_artifact_dir.mkdir(parents=True, exist_ok=True)

    warmup_args = copy.copy(args)
    warmup_args.artifact_dir = root_artifact_dir / "warmup"
    warmup_args.duration_sec = args.warmup_duration_sec
    warmup_args.command_timeout_sec = (
        args.warmup_command_timeout_sec
        if args.warmup_command_timeout_sec is not None
        else args.command_timeout_sec
    )
    warmup_args.warmup_duration_sec = 0.0
    warmup_args.ack_timeout_sweep = None
    try:
        warmup_summary = run_acceptance_once(
            warmup_args,
            phase="warmup",
            command_rate_hz=args.warmup_rate_hz,
        )
    except Exception as exc:
        warmup_summary = failure_summary(warmup_args, exc, default_phase="warmup")

    if warmup_summary.get("result") != "pass":
        summary = dict(warmup_summary)
        summary.update(
            {
                "artifact_dir": str(root_artifact_dir),
                "warmup_enabled": True,
                "warmup_result": warmup_summary.get("result"),
                "warmup_summary": str((warmup_args.artifact_dir / "summary.json").resolve()),
                "measurement_summary": None,
                "failure_phase": "warmup",
                "failure_classification": warmup_summary.get("failure_classification") or "warmup_failed",
            }
        )
        write_json(root_artifact_dir / "summary.json", summary)
        write_summary_csv(root_artifact_dir / "summary.csv", summary)
        return summary

    measurement_args = copy.copy(args)
    measurement_args.artifact_dir = root_artifact_dir / "measurement"
    measurement_args.warmup_duration_sec = 0.0
    measurement_args.ack_timeout_sweep = None
    try:
        measurement_summary = run_acceptance_once(measurement_args, phase="measurement")
    except Exception as exc:
        measurement_summary = failure_summary(measurement_args, exc, default_phase="measurement")

    summary = dict(measurement_summary)
    summary.update(
        {
            "artifact_dir": str(root_artifact_dir),
            "warmup_enabled": True,
            "warmup_result": warmup_summary.get("result"),
            "warmup_timeout_count": warmup_summary.get("warmup_timeout_count", 0),
            "warmup_summary": str((warmup_args.artifact_dir / "summary.json").resolve()),
            "measurement_summary": str((measurement_args.artifact_dir / "summary.json").resolve()),
        }
    )
    write_json(root_artifact_dir / "summary.json", summary)
    write_summary_csv(root_artifact_dir / "summary.csv", summary)
    return summary


def run_ack_timeout_sweep(args: argparse.Namespace) -> dict[str, Any]:
    timeouts = parse_ack_timeout_sweep(args.ack_timeout_sweep)
    root_artifact_dir = args.artifact_dir.resolve()
    root_artifact_dir.mkdir(parents=True, exist_ok=True)
    sub_experiments: list[dict[str, Any]] = []
    for timeout_sec in timeouts:
        sub_args = copy.copy(args)
        sub_args.artifact_dir = root_artifact_dir / timeout_sweep_label(timeout_sec)
        sub_args.command_timeout_sec = timeout_sec
        sub_args.ack_timeout_sweep = None
        try:
            summary = run_acceptance(sub_args)
        except Exception as exc:
            summary = failure_summary(sub_args, exc)
        sub_experiments.append(
            {
                "command_timeout_sec": timeout_sec,
                "artifact_dir": str(sub_args.artifact_dir.resolve()),
                "summary": str((sub_args.artifact_dir / "summary.json").resolve()),
                "result": summary.get("result"),
                "failure_phase": summary.get("failure_phase"),
                "failure_classification": summary.get("failure_classification"),
                "warmup_result": summary.get("warmup_result"),
                "warmup_timeout_count": summary.get("warmup_timeout_count"),
                "measurement_timeout_count": summary.get("measurement_timeout_count"),
                "ack_timeout_count_by_arm": summary.get("ack_timeout_count_by_arm"),
            }
        )
    result = "pass" if all(item.get("result") == "pass" for item in sub_experiments) else "fail"
    first_failed = next((item for item in sub_experiments if item.get("result") != "pass"), None)
    ack_timeout_count_by_arm = {
        arm: sum(
            int(((item.get("ack_timeout_count_by_arm") or {}).get(arm) or 0))
            for item in sub_experiments
        )
        for arm in ARMS
    }
    warmup_timeout_count = sum(int(item.get("warmup_timeout_count") or 0) for item in sub_experiments)
    measurement_timeout_count = sum(int(item.get("measurement_timeout_count") or 0) for item in sub_experiments)
    summary = {
        "schema": SCHEMA,
        "result": result,
        "result_reason": "ACK timeout sweep completed",
        "mode": MODE,
        "acceptance_stage": (
            f"noop_500hz_{async_mode_value(args)}"
            if async_mode_value(args) != ASYNC_DISABLED
            else MODE
        ),
        "async_mode": async_mode_value(args),
        "artifact_dir": str(root_artifact_dir),
        "ack_timeout_sweep": timeouts,
        "command_timeout_sec_left": None,
        "command_timeout_sec_right": None,
        "warmup_enabled": getattr(args, "warmup_duration_sec", 0.0) > 0.0,
        "warmup_result": "pass"
        if sub_experiments and all(item.get("warmup_result") in {"pass", "not_run"} for item in sub_experiments)
        else "mixed",
        "warmup_timeout_count": warmup_timeout_count,
        "measurement_timeout_count": measurement_timeout_count,
        "first_send_failure_arm": None,
        "first_send_failure_index": None,
        "first_send_failure_elapsed_sec": None,
        "ack_timeout_count_by_arm": ack_timeout_count_by_arm,
        "send_duration_p99_us_by_arm": {arm: None for arm in ARMS},
        "deadline_miss_count_by_arm": {arm: 0 for arm in ARMS},
        "controller_ack_observed_count": None,
        "socket_send_only_count": None,
        "reference_supervision_state": None,
        "q_ref_update_rate_hz": None,
        "q_ref_update_age_p95_ms": None,
        "tcp_ref_update_age_p95_ms": None,
        "commands_overwritten_total": None,
        "commands_dropped_total": None,
        "async_worker_backlog_max": None,
        "supervision_fault_count": None,
        "servo_loop_blocked_by_ack": "unknown",
        "sub_experiments": sub_experiments,
        "failure_phase": None if result == "pass" else (first_failed or {}).get("failure_phase"),
        "failure_classification": None
        if result == "pass"
        else (first_failed or {}).get("failure_classification") or "ack_timeout_sweep_failed",
        "caveat": (
            "Sweep results separate ACK timeout tightness from the 500 Hz no-op control path; "
            "ACK failures remain failures."
        ),
    }
    write_json(root_artifact_dir / "summary.json", summary)
    write_json(root_artifact_dir / "ack_timeout_sweep_summary.json", summary)
    write_summary_csv(root_artifact_dir / "summary.csv", summary)
    return summary


def run_acceptance(args: argparse.Namespace) -> dict[str, Any]:
    if getattr(args, "ack_timeout_sweep", None):
        return run_ack_timeout_sweep(args)
    if getattr(args, "warmup_duration_sec", 0.0) > 0.0:
        return run_acceptance_with_warmup(args)
    return run_acceptance_once(args, phase="measurement")


def failure_summary(
    args: argparse.Namespace,
    exc: Exception,
    *,
    default_phase: str = "preflight",
) -> dict[str, Any]:
    artifact_dir = args.artifact_dir.resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    phase = getattr(exc, "failure_phase", None) or default_phase
    classification = getattr(exc, "failure_classification", None) or classify_error_text(str(exc), phase)
    snapshot = getattr(exc, "snapshot", None)
    failure_details = send_failure_summary([snapshot], phase=phase) if isinstance(snapshot, dict) else {
        "ack_timeout_count_by_arm": {arm: 0 for arm in ARMS},
        "ack_timeout_count_unknown_arm": 0,
        "warmup_timeout_count": 0,
        "measurement_timeout_count": 0,
        "first_send_failure_arm": None,
        "first_send_failure_index": None,
        "first_send_failure_elapsed_sec": None,
        "first_send_failure_source": None,
        "first_send_failure_ack_timeout": False,
        "first_send_failure_text": None,
    }
    if classification is None:
        classification = "preflight_error" if phase == "preflight" else f"{phase}_error"
    summary = {
        "schema": SCHEMA,
        "result": "error",
        "result_reason": "500 Hz no-op acceptance could not run",
        "error": str(exc),
        "mode": MODE,
        "acceptance_stage": (
            f"noop_500hz_{async_mode_value(args)}"
            if async_mode_value(args) != ASYNC_DISABLED
            else MODE
        ),
        "async_mode": async_mode_value(args),
        "artifact_dir": str(artifact_dir),
        "failure_phase": phase,
        "failure_classification": classification,
        "command_timeout_sec_left": getattr(args, "command_timeout_sec", None),
        "command_timeout_sec_right": getattr(args, "command_timeout_sec", None),
        "warmup_enabled": getattr(args, "warmup_duration_sec", 0.0) > 0.0 or phase == "warmup",
        "warmup_result": "error" if phase == "warmup" else "not_run",
        "send_duration_p99_us_by_arm": {arm: None for arm in ARMS},
        "deadline_miss_count_by_arm": {arm: 0 for arm in ARMS},
        "controller_ack_observed_count": 0,
        "socket_send_only_count": 0,
        "reference_supervision_state": "unknown",
        "q_ref_update_rate_hz": None,
        "q_ref_update_age_p95_ms": None,
        "tcp_ref_update_age_p95_ms": None,
        "commands_overwritten_total": 0,
        "commands_dropped_total": 0,
        "async_worker_backlog_max": 0,
        "supervision_fault_count": 0,
        "servo_loop_blocked_by_ack": "unknown",
        "safety_preflight": {"passed": False, "error": str(exc), "env": env_snapshot_500hz()},
        "threshold_failures": [str(exc)],
        "caveat": "No acceptance evidence was produced.",
    }
    summary.update(failure_details)
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
