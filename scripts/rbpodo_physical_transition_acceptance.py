#!/usr/bin/env python3
"""Conservative rbpodo pgmode-simulation to physical-real transition gate.

This script is a stage-aware preflight and artifact writer. Dry-run is the
default and never connects to hardware or sends motion commands. The non-dry-run
path validates local config, then records a supervised preflight artifact; it
intentionally does not launch a servo server or command motion by itself.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


SCHEMA = "robotics_lab.rbpodo_physical_transition.stage.v1"
DEFAULT_ROOT = Path(__file__).resolve().parents[1]
ENV_KEYS: tuple[str, ...] = ()


class AcceptanceError(RuntimeError):
    pass


@dataclass(frozen=True)
class Stage:
    stage_id: str
    cli_name: str
    ladder_name: str
    category: str
    description: str
    required_env: tuple[str, ...]
    default_parameters: dict[str, Any]
    prerequisite_stage_ids: tuple[str, ...] = ()

    @property
    def is_motion(self) -> bool:
        return self.category in {"joint_motion", "cartesian_motion", "fast_cartesian_motion"}

    @property
    def is_cartesian(self) -> bool:
        return self.category in {"cartesian_motion", "fast_cartesian_motion"}


STAGE_LIST = (
    Stage(
        "P0",
        "controller_sim_repeatability",
        "controller_sim_repeatability_done",
        "controller_simulation",
        "Recorded rbpodo controller pgmode repeatability evidence; no physical motion.",
        (),
        {
            "evidence_category": "rbpodo_controller_simulation",
            "tracking_source": "tcp_ref_stand",
            "physical_motion_expected": False,
        },
    ),
    Stage(
        "P1",
        "read_only",
        "real_readonly_diagnostics_parity",
        "read_only",
        "Physical operation_mode real state diagnostics only.",
        (),
        {
            "servo_send_servo_commands": False,
            "expected_tracking_source": "tcp_actual_stand",
            "physical_motion_expected": False,
        },
        ("P0",),
    ),
    Stage(
        "P2",
        "stop_policy",
        "stop_resetFault_or_operator_stop_policy_verified",
        "read_only",
        "Verify stop/resetFault API behavior or record unresolved operator-stop policy.",
        (),
        {
            "servo_send_servo_commands": False,
            "stop_reset_behavior_result": "unresolved_until_operator_artifact",
            "physical_motion_expected": False,
        },
        ("P0", "P1"),
    ),
    Stage(
        "P3",
        "hold_no_motion",
        "real_hold_no_motion",
        "read_only",
        "Hold startup with no Servo J sends and no physical motion expected.",
        (),
        {
            "servo_send_servo_commands": False,
            "hold_duration_sec": 10.0,
            "physical_motion_expected": False,
        },
        ("P0", "P1", "P2"),
    ),
    Stage(
        "P4",
        "tiny_joint",
        "tiny_joint_noop_or_tiny_joint_motion",
        "joint_motion",
        "Tiny joint no-op or explicitly tiny supervised joint motion.",
        (),
        {
            "rate_hz_policy": "accepted_real_servo_j_rate",
            "max_joint_delta_deg": 0.02,
            "speed_bar_policy": "conservative_local_value",
            "physical_motion_expected": "operator_selected_noop_or_tiny_motion",
        },
        ("P0", "P1", "P2", "P3"),
    ),
    Stage(
        "P5",
        "tiny_cartesian",
        "tiny_cartesian_delta",
        "cartesian_motion",
        "Tiny Cartesian delta using physical actual TCP state only.",
        (),
        {
            "rate_hz_policy": "accepted_real_servo_j_rate",
            "max_cartesian_step_m": 0.001,
            "max_cartesian_step_rad": 0.005,
            "tracking_source": "tcp_actual_stand",
            "physical_motion_expected": True,
        },
        ("P0", "P1", "P2", "P3", "P4"),
    ),
    Stage(
        "P6",
        "circle_5cm_10s",
        "slow_physical_circle_5cm_10s",
        "cartesian_motion",
        "First slow physical circle candidate.",
        (),
        {
            "rate_hz_policy": "accepted_real_servo_j_rate",
            "circle_diameter_m": 0.05,
            "circle_period_sec": 10.0,
            "tracking_source": "tcp_actual_stand",
            "physical_motion_expected": True,
        },
        ("P0", "P1", "P2", "P3", "P4", "P5"),
    ),
    Stage(
        "P7",
        "circle_15cm_16s",
        "stable_physical_circle_15cm_16s",
        "cartesian_motion",
        "Stable physical 15 cm circle after 5 cm / 10 s passes.",
        (),
        {
            "rate_hz_policy": "accepted_real_servo_j_rate",
            "circle_diameter_m": 0.15,
            "circle_period_sec": 16.0,
            "tracking_source": "tcp_actual_stand",
            "physical_motion_expected": True,
        },
        ("P0", "P1", "P2", "P3", "P4", "P5", "P6"),
    ),
    Stage(
        "P8",
        "circle_15cm_8s",
        "medium_physical_circle_15cm_8s",
        "cartesian_motion",
        "Medium physical 15 cm circle after stable 16 s evidence.",
        (),
        {
            "rate_hz_policy": "accepted_real_servo_j_rate",
            "circle_diameter_m": 0.15,
            "circle_period_sec": 8.0,
            "tracking_source": "tcp_actual_stand",
            "physical_motion_expected": True,
        },
        ("P0", "P1", "P2", "P3", "P4", "P5", "P6", "P7"),
    ),
    Stage(
        "P9",
        "circle_15cm_4s_fast",
        "fast_physical_circle_15cm_4s_only_after_explicit_approval",
        "fast_cartesian_motion",
        "Fast 15 cm / 4 s physical circle only after explicit approval.",
        (),
        {
            "rate_hz_policy": "accepted_real_servo_j_rate",
            "circle_diameter_m": 0.15,
            "circle_period_sec": 4.0,
            "tracking_source": "tcp_actual_stand",
            "physical_motion_expected": True,
            "controller_sim_best_profile_usage": "seed_record_only_not_default_envelope",
        },
        ("P0", "P1", "P2", "P3", "P4", "P5", "P6", "P7", "P8"),
    ),
)
STAGES = {stage.cli_name: stage for stage in STAGE_LIST}
STAGES_BY_ID = {stage.stage_id: stage for stage in STAGE_LIST}


@dataclass
class ArmConfig:
    backend_type: str = ""
    run_mode: str = ""
    ip: str = ""
    operation_mode: str = ""
    speed_bar: float | None = None
    servo_t1_sec: float | None = None
    servo_t2_sec: float | None = None
    servo_gain: float | None = None
    servo_alpha: float | None = None
    disable_waiting_ack: bool = False


@dataclass
class ParsedConfig:
    path: Path
    root: Path
    left: ArmConfig
    right: ArmConfig
    servo: dict[str, Any]
    has_force_control_section: bool
    cartesian_control: dict[str, Any]
    network: dict[str, Any]


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Gate the rbpodo controller pgmode-simulation to physical-real "
            "transition ladder. Dry-run is the default and starts no hardware."
        )
    )
    parser.add_argument("--stage", choices=tuple(STAGES), default="read_only")
    parser.add_argument("--config", type=Path, help="Site-local config for non-dry-run stages.")
    parser.add_argument("--artifact-dir", type=Path, help="Write summary.json under this directory.")
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--arm", choices=("left", "right", "both"), default="left")
    parser.add_argument("--duration-sec", type=float)
    parser.add_argument("--max-joint-delta-deg", type=float, default=0.02)
    parser.add_argument("--max-cartesian-step-m", type=float, default=0.001)
    parser.add_argument("--circle-diameter-m", type=float)
    parser.add_argument("--circle-period-sec", type=float)
    parser.add_argument(
        "--operator-stop-policy-verified",
        action="store_true",
        help="For P2 only, record that the physical operator-stop policy was verified.",
    )
    parser.add_argument(
        "--operator-stop-note",
        default="",
        help="Required non-empty operator note when --operator-stop-policy-verified is set.",
    )
    parser.add_argument(
        "--prereq-artifact",
        action="append",
        default=[],
        metavar="P0=PATH",
        help="Record prerequisite artifact references for report readiness checks.",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", dest="dry_run", action="store_true", help="Do not require env gates or run hardware.")
    mode.add_argument(
        "--execute",
        dest="dry_run",
        action="store_false",
        help="Validate live gates and write a supervised preflight artifact; this script still does not launch motion.",
    )
    parser.set_defaults(dry_run=True)
    return parser.parse_args(argv)


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
        if key.startswith("- "):
            continue
        sections.setdefault(current, {})[key.strip()] = scalar_value(value)
    return sections


def as_bool(value: Any, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in {"1", "true", "yes", "on"}
    return default


def as_float(value: Any, default: float | None = None) -> float | None:
    if value is None or isinstance(value, bool):
        return default
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def arm_config(section: dict[str, Any]) -> ArmConfig:
    return ArmConfig(
        backend_type=str(section.get("backend_type", "")),
        run_mode=str(section.get("run_mode", "")),
        ip=str(section.get("ip", "")),
        operation_mode=str(section.get("operation_mode", "")),
        speed_bar=as_float(section.get("speed_bar")),
        servo_t1_sec=as_float(section.get("servo_t1_sec", section.get("servo_time_sec"))),
        servo_t2_sec=as_float(section.get("servo_t2_sec", section.get("servo_lookahead_sec"))),
        servo_gain=as_float(section.get("servo_gain")),
        servo_alpha=as_float(section.get("servo_alpha", section.get("servo_acc"))),
        disable_waiting_ack=as_bool(section.get("disable_waiting_ack"), False),
    )


def load_config(path: Path, root: Path) -> ParsedConfig:
    if not path.is_file():
        raise AcceptanceError(f"config not found: {path}")
    sections = simple_yaml_sections(path)
    return ParsedConfig(
        path=path.resolve(),
        root=root.resolve(),
        left=arm_config(sections.get("left_robot", {})),
        right=arm_config(sections.get("right_robot", {})),
        servo=sections.get("servo", {}),
        has_force_control_section="force_control" in sections,
        cartesian_control=sections.get("cartesian_control", {}),
        network=sections.get("network", {}),
    )


def path_is_under(path: Path, parent: Path) -> bool:
    try:
        path.resolve().relative_to(parent.resolve())
    except ValueError:
        return False
    return True


def is_site_local_config(config: ParsedConfig) -> bool:
    return path_is_under(config.path, config.root / "rb_servo_server" / "config" / "local")


def is_tracked_example_config(config: ParsedConfig) -> bool:
    config_root = config.root / "rb_servo_server" / "config"
    return path_is_under(config.path, config_root) and not is_site_local_config(config) and config.path.name.endswith(
        ".example.yaml"
    )


def env_enabled(name: str) -> bool:
    return os.environ.get(name) == "1"


def env_snapshot() -> dict[str, str | None]:
    return {key: os.environ.get(key) for key in ENV_KEYS}


def stage_required_env(stage: Stage, config: ParsedConfig | None) -> tuple[str, ...]:
    return tuple(stage.required_env)


def missing_env(required_env: tuple[str, ...]) -> list[str]:
    return [name for name in required_env if not env_enabled(name)]


def validate_numeric_args(args: argparse.Namespace, stage: Stage) -> list[str]:
    blockers: list[str] = []
    if args.duration_sec is not None and (not math.isfinite(args.duration_sec) or args.duration_sec <= 0.0):
        blockers.append("--duration-sec must be finite and positive")
    if stage.category == "joint_motion":
        if (
            not math.isfinite(args.max_joint_delta_deg)
            or args.max_joint_delta_deg < 0.0
            or args.max_joint_delta_deg > 0.05
        ):
            blockers.append("--max-joint-delta-deg must be finite, non-negative, and <= 0.05 for P4")
    if stage.is_cartesian:
        if (
            not math.isfinite(args.max_cartesian_step_m)
            or args.max_cartesian_step_m <= 0.0
            or args.max_cartesian_step_m > 0.005
        ):
            blockers.append("--max-cartesian-step-m must be finite, positive, and <= 0.005 for physical Cartesian stages")
    if args.circle_diameter_m is not None:
        expected = stage.default_parameters.get("circle_diameter_m")
        if expected is not None and abs(args.circle_diameter_m - float(expected)) > 1e-9:
            blockers.append(f"{stage.stage_id} requires circle diameter {expected} m")
    if args.circle_period_sec is not None:
        expected = stage.default_parameters.get("circle_period_sec")
        if expected is not None and abs(args.circle_period_sec - float(expected)) > 1e-9:
            blockers.append(f"{stage.stage_id} requires circle period {expected} s")
    return blockers


def validate_operator_stop_policy_args(args: argparse.Namespace, stage: Stage) -> list[str]:
    if not getattr(args, "operator_stop_policy_verified", False):
        return []
    blockers: list[str] = []
    if stage.stage_id != "P2":
        blockers.append("--operator-stop-policy-verified is valid only for P2 stop_policy")
    if not str(getattr(args, "operator_stop_note", "") or "").strip():
        blockers.append("--operator-stop-note is required when --operator-stop-policy-verified is set")
    return blockers


def validate_config_policy(args: argparse.Namespace, stage: Stage, config: ParsedConfig | None) -> list[str]:
    if config is None:
        if args.dry_run:
            return []
        return ["--config is required for non-dry-run physical transition stages"]

    blockers: list[str] = []
    send_servo_commands = as_bool(config.servo.get("send_servo_commands"), True)
    cartesian_allow_in_real = as_bool(config.cartesian_control.get("allow_in_real"), False)

    if is_tracked_example_config(config):
        if send_servo_commands:
            blockers.append("tracked example config must keep servo.send_servo_commands=false")
        if cartesian_allow_in_real:
            blockers.append("tracked example config must keep cartesian_control.allow_in_real=false")

    # force_control was removed from the server config schema; a config that
    # still carries the section will not load at all, so refuse it here with a
    # message that says why rather than letting the server fail later.
    if config.has_force_control_section:
        blockers.append(
            "force_control is no longer a server config section; remove it"
        )

    if not args.dry_run:
        if not is_site_local_config(config):
            blockers.append("non-dry-run stages require a site-local config under rb_servo_server/config/local/")
        for label, arm in (("left", config.left), ("right", config.right)):
            if arm.backend_type != "rbpodo":
                blockers.append(f"{label}_robot.backend_type must be rbpodo")
            if arm.run_mode != "real":
                blockers.append(f"{label}_robot.run_mode must be real")
            if arm.operation_mode != "real":
                blockers.append(f"{label}_robot.operation_mode must be real for physical transition stages")
        if stage.is_motion and not send_servo_commands:
            blockers.append(f"{stage.stage_id} {stage.cli_name} requires servo.send_servo_commands=true in the local config")
        if not stage.is_motion and send_servo_commands:
            blockers.append(f"{stage.stage_id} {stage.cli_name} requires servo.send_servo_commands=false")
        if stage.is_cartesian:
            if not as_bool(config.cartesian_control.get("enable"), False):
                blockers.append(f"{stage.stage_id} {stage.cli_name} requires cartesian_control.enable=true")
            if not cartesian_allow_in_real:
                blockers.append(
                    f"{stage.stage_id} {stage.cli_name} requires a site-local config with cartesian_control.allow_in_real=true"
                )
    return blockers


def parse_prereq_artifacts(values: list[str]) -> dict[str, str]:
    parsed: dict[str, str] = {}
    for raw in values:
        if "=" not in raw:
            raise AcceptanceError(f"--prereq-artifact must use STAGE=PATH, got {raw}")
        stage_id, path = raw.split("=", 1)
        stage_id = stage_id.strip().upper()
        if stage_id not in STAGES_BY_ID:
            raise AcceptanceError(f"unknown prerequisite stage id {stage_id}")
        if not path.strip():
            raise AcceptanceError(f"empty artifact path for {stage_id}")
        parsed[stage_id] = path.strip()
    return parsed


def read_calibration(root: Path) -> dict[str, Any]:
    path = root / "calibration" / "active_calibration.yaml"
    if not path.is_file():
        return {
            "path": str(path),
            "status": "unknown",
            "geometry_valid_for_real_policy": False,
            "measured": False,
        }
    sections = simple_yaml_sections(path)
    top: dict[str, Any] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = strip_comment(raw_line).rstrip()
        if not line.strip() or line.startswith(" "):
            continue
        if ":" in line:
            key, value = line.split(":", 1)
            top[key.strip()] = scalar_value(value)
    status = str(top.get("status", "unknown"))
    return {
        "path": str(path),
        "calibration_id": top.get("calibration_id"),
        "status": status,
        "geometry_valid_for_real_policy": as_bool(top.get("geometry_valid_for_real_policy"), False),
        "measured": status == "measured",
        "robot_sections_seen": sorted(key for key in sections if key.startswith("T_")),
    }


def stage_parameters(args: argparse.Namespace, stage: Stage, config: ParsedConfig | None) -> dict[str, Any]:
    params = dict(stage.default_parameters)
    if config is not None:
        params["config_servo_rate_hz"] = config.servo.get("rate_hz")
        params["left_speed_bar"] = config.left.speed_bar
        params["right_speed_bar"] = config.right.speed_bar
        params["left_servo_t1_sec"] = config.left.servo_t1_sec
        params["right_servo_t1_sec"] = config.right.servo_t1_sec
    if args.duration_sec is not None:
        params["duration_sec"] = args.duration_sec
    if stage.category == "joint_motion":
        params["max_joint_delta_deg"] = args.max_joint_delta_deg
    if stage.is_cartesian:
        params["max_cartesian_step_m"] = args.max_cartesian_step_m
    if args.circle_diameter_m is not None:
        params["circle_diameter_m"] = args.circle_diameter_m
    if args.circle_period_sec is not None:
        params["circle_period_sec"] = args.circle_period_sec
    return params


def config_summary(config: ParsedConfig | None) -> dict[str, Any] | None:
    if config is None:
        return None
    return {
        "path": str(config.path),
        "is_site_local": is_site_local_config(config),
        "is_tracked_example": is_tracked_example_config(config),
        "servo_send_servo_commands": as_bool(config.servo.get("send_servo_commands"), True),
        "servo_rate_hz": config.servo.get("rate_hz"),
        "has_force_control_section": config.has_force_control_section,
        "cartesian_allow_in_real": as_bool(config.cartesian_control.get("allow_in_real"), False),
        "cartesian_enable": as_bool(config.cartesian_control.get("enable"), False),
        "left": {
            "backend_type": config.left.backend_type,
            "run_mode": config.left.run_mode,
            "ip": config.left.ip,
            "operation_mode": config.left.operation_mode,
            "speed_bar": config.left.speed_bar,
            "disable_waiting_ack": config.left.disable_waiting_ack,
        },
        "right": {
            "backend_type": config.right.backend_type,
            "run_mode": config.right.run_mode,
            "ip": config.right.ip,
            "operation_mode": config.right.operation_mode,
            "speed_bar": config.right.speed_bar,
            "disable_waiting_ack": config.right.disable_waiting_ack,
        },
    }


def build_summary(
    args: argparse.Namespace,
    stage: Stage,
    config: ParsedConfig | None,
    blockers: list[str],
    prereq_artifacts: dict[str, str],
    required_env: tuple[str, ...],
) -> dict[str, Any]:
    dry_run = bool(args.dry_run)
    result_status = "dry_run" if dry_run and not blockers else "blocked" if blockers else "preflight_pass"
    physical_status = "not_run"
    operator_stop_note = str(getattr(args, "operator_stop_note", "") or "").strip()
    operator_stop_verified = bool(
        getattr(args, "operator_stop_policy_verified", False) and stage.stage_id == "P2" and operator_stop_note
    )
    stop_reset_behavior_result = (
        "operator_stop_policy_verified"
        if operator_stop_verified
        else "unresolved"
        if stage.stage_id in {"P2", "P3"} or stage.is_motion
        else "not_applicable"
    )
    return {
        "schema": SCHEMA,
        "generated_at_unix": time.time(),
        "stage": {
            "id": stage.stage_id,
            "cli_name": stage.cli_name,
            "ladder_name": stage.ladder_name,
            "category": stage.category,
            "description": stage.description,
            "prerequisite_stage_ids": list(stage.prerequisite_stage_ids),
        },
        "result": {
            "status": result_status,
            "dry_run": dry_run,
            "hardware_process_started": False,
            "motion_command_sent": False,
            "blockers": blockers,
        },
        "gates": {
            "required_env": [f"{name}=1" for name in required_env],
            "missing_env": [f"{name}=1" for name in missing_env(required_env)],
            "env_snapshot": env_snapshot(),
        },
        "config": config_summary(config),
        "parameters": stage_parameters(args, stage, config),
        "prerequisite_artifacts": prereq_artifacts,
        "physical_tracking_result": {
            "status": physical_status,
            "tracking_source": "tcp_actual_stand",
            "rms_error_m": None,
            "p95_error_m": None,
            "max_error_m": None,
        },
        "controller_reference_result": {
            "status": "informational_only",
            "tracking_source": "tcp_ref_stand",
            "rms_error_m": None,
            "p95_error_m": None,
            "max_error_m": None,
        },
        "telemetry_requirements": {
            "state_age_us": {"p95": None, "max": None},
            "state_jitter_us": {"p95": None, "max": None},
            "q_actual_update_rate_hz": None,
            "q_ref_update_rate_hz": None,
            "fault_latch_status": "not_checked" if dry_run else "unresolved",
            "cartesian_availability": "not_checked" if dry_run else "unresolved",
            "stop_reset_behavior_result": stop_reset_behavior_result,
            "stop_reset_behavior_note": operator_stop_note if operator_stop_verified else None,
            "physical_motion_expected": False if dry_run else bool(stage.is_motion),
            "physical_motion_detected": None,
        },
        "calibration": read_calibration(args.root.resolve()),
        "safety_notes": [
            "Dry-run does not require env gates and starts no hardware." if dry_run else "Non-dry-run preflight passed only if blockers is empty.",
            "Physical pass evidence must use tcp_actual_stand.",
            "tcp_ref_stand is retained only as controller-reference informational evidence.",
        ],
    }


def write_summary(artifact_dir: Path, summary: dict[str, Any]) -> Path:
    artifact_dir.mkdir(parents=True, exist_ok=True)
    path = artifact_dir / "summary.json"
    path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    return path


def print_gate_summary(stage: Stage, summary: dict[str, Any]) -> None:
    print(f"Stage {stage.stage_id} {stage.ladder_name} ({stage.cli_name})")
    print(f"Dry run: {summary['result']['dry_run']}")
    print("Hardware process started: false")
    print("Motion command sent: false")
    required_env = summary["gates"]["required_env"]
    if required_env:
        print("Required env gates for non-dry-run: " + ", ".join(required_env))
    else:
        print("Required env gates for non-dry-run: none")
    blockers = summary["result"]["blockers"]
    if blockers:
        print("Refusal blockers:")
        for blocker in blockers:
            print(f"  - {blocker}")
    else:
        print("Refusal blockers: none")
    print("Physical tracking source for pass evidence: tcp_actual_stand")
    print("Controller reference source is informational only: tcp_ref_stand")


def live_preflight_placeholder(summary: dict[str, Any]) -> dict[str, Any]:
    """Return explicit non-launch evidence for the supervised live path."""
    summary = dict(summary)
    result = dict(summary["result"])
    result["status"] = "preflight_pass"
    result["hardware_process_started"] = False
    result["motion_command_sent"] = False
    result["operator_next_step"] = (
        "Use the runbook stage command under local supervision; this script has "
        "validated gates but does not launch physical motion."
    )
    summary["result"] = result
    return summary


def main(argv: list[str] | None = None) -> int:
    try:
        args = parse_args(argv)
        args.root = args.root.resolve()
        stage = STAGES[args.stage]
        config = load_config(args.config, args.root) if args.config else None
        prereq_artifacts = parse_prereq_artifacts(args.prereq_artifact)
        required_env = stage_required_env(stage, config)
        blockers = validate_numeric_args(args, stage)
        blockers.extend(validate_operator_stop_policy_args(args, stage))
        blockers.extend(validate_config_policy(args, stage, config))
        if not args.dry_run:
            blockers.extend(f"missing env gate {gate}=1" for gate in missing_env(required_env))
        summary = build_summary(args, stage, config, blockers, prereq_artifacts, required_env)
        if blockers:
            print_gate_summary(stage, summary)
            if args.artifact_dir:
                path = write_summary(args.artifact_dir, summary)
                print(f"Artifact written: {path}")
            return 2
        if not args.dry_run:
            summary = live_preflight_placeholder(summary)
        print_gate_summary(stage, summary)
        if args.artifact_dir:
            path = write_summary(args.artifact_dir, summary)
            print(f"Artifact written: {path}")
        return 0
    except AcceptanceError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
