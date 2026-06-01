#!/usr/bin/env python3
"""Run rbpodo controller-simulation circle benchmark ablation matrices."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shlex
import subprocess
import sys
from pathlib import Path
from typing import Any

import circle_tracking_benchmark as sim_bench
import generate_rbpodo_measurement_reliability_report as reliability_report
import rbpodo_circle_tracking_benchmark as circle_bench
import run_circle_ablation as sim_ablation
from rbpodo_servo_acceptance import (
    REAL_ROBOT_IPS,
    as_bool,
    as_float,
    env_enabled,
    env_snapshot,
    load_config,
    simple_yaml_sections,
)


SCHEMA = "robotics_lab.rbpodo_circle_ablation.v1"
REQUIRED_ENV = (
    "RB_ALLOW_REAL_ROBOT",
    "RB_ALLOW_REAL_MOTION",
    "RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION",
    "RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN",
)
OPTIONAL_ENV_REQUIREMENTS = {
    "RB_ALLOW_RBPODO_ACK_DISABLED_MOTION",
    "RB_ALLOW_RBPODO_ASYNC_STREAMING",
    "RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM",
    "RB_ALLOW_RBPODO_SOCKET_SEND_ONLY_STREAMING",
}
PROFILES = set(circle_bench.PROFILE_DEFAULTS)
CONTROLLERS = {
    "twist_stand",
    "twist_local",
    "twist_stand_feedback",
    "twist_local_feedback",
    "server_circle",
}
TRACKING_SOURCES = set(circle_bench.TRACKING_SOURCES)
ARMS = {"left", "right"}
SERVO_T2_MIN_SEC = 0.02
SERVO_T2_MAX_SEC = 0.2
SERVO_ALPHA_MIN = 0.0
SERVO_ALPHA_MAX = 1.0
MAX_STATE_PUB_RATE_HZ = 500.0
EXPERIMENT_KEYS = {
    "name",
    "enabled",
    "config",
    "profile",
    "controller",
    "arm",
    "plane",
    "command_rate_hz",
    "repeat",
    "tracking_source",
    "feedback_kp_pos",
    "feedback_kp_ori",
    "feedback_max_linear_m_s",
    "feedback_max_angular_rad_s",
    "feedback_use_current_state_time",
    "phase_advance_sec",
    "warmup_sec",
    "settle_sec",
    "startup_timeout_sec",
    "max_state_age_us",
    "physical_motion_warning_deg",
    "max_allowed_rms_error_m",
    "max_allowed_p95_error_m",
    "max_allowed_orientation_drift_rad",
    "max_allowed_latency_ms",
    "skip_plots",
    "env_requirements",
    "config_overrides",
}
ALLOWED_CONFIG_OVERRIDES = {
    "network.state_pub_rate_hz",
    "servo.rate_hz",
    "servo.worker_read_period_sec",
    "left_robot.speed_bar",
    "right_robot.speed_bar",
    "left_robot.servo_t1_sec",
    "right_robot.servo_t1_sec",
    "left_robot.disable_waiting_ack",
    "right_robot.disable_waiting_ack",
    "left_robot.servo_t2_sec",
    "right_robot.servo_t2_sec",
    "left_robot.servo_alpha",
    "right_robot.servo_alpha",
    "left_robot.command_timeout_sec",
    "right_robot.command_timeout_sec",
    "cartesian_control.max_twist_linear_m_s",
    "cartesian_control.max_twist_angular_rad_s",
    "cartesian_control.max_linear_move_speed_m_s",
    "cartesian_control.enable_benchmark_primitives",
    "cartesian_control.path_kp_pos",
    "cartesian_control.path_kp_ori",
    "cartesian_control.twist_angular_deadband_rad_s",
    "cartesian_control.velocity_target_integration",
    "cartesian_control.velocity_target_lookahead_sec",
    "servo.rbpodo_async_streaming.enable",
    "servo.rbpodo_async_streaming.mode",
    "servo.rbpodo_async_streaming.rate_hz",
    "servo.rbpodo_async_streaming.queue_policy",
    "servo.rbpodo_async_streaming.max_pending_age_ms",
    "servo.rbpodo_async_streaming.ack_supervision.enable",
    "servo.rbpodo_async_streaming.ack_supervision.expected_ack_timeout_ms",
    "servo.rbpodo_async_streaming.ack_supervision.missing_ack_fault_after_ms",
    "servo.rbpodo_async_streaming.ack_supervision.max_consecutive_missing_ack",
    "servo.rbpodo_async_streaming.reference_supervision.enable",
    "servo.rbpodo_async_streaming.reference_supervision.q_ref_update_timeout_ms",
    "servo.rbpodo_async_streaming.reference_supervision.q_ref_target_tolerance_deg",
    "servo.rbpodo_async_streaming.reference_supervision.q_ref_target_fault_after_ms",
    "servo.rbpodo_async_streaming.reference_supervision.tcp_ref_update_timeout_ms",
    "servo.rbpodo_async_streaming.reference_supervision.tcp_ref_target_tolerance_m",
    "servo.rbpodo_async_streaming.reference_supervision.tcp_ref_target_fault_after_ms",
    "servo.rbpodo_async_streaming.reference_supervision.policy",
    "servo.rbpodo_async_streaming.diagnostics.publish_per_command_jsonl",
}
BOOLEAN_CONFIG_OVERRIDES = {
    "left_robot.disable_waiting_ack",
    "right_robot.disable_waiting_ack",
    "servo.rbpodo_async_streaming.enable",
    "cartesian_control.enable_benchmark_primitives",
    "servo.rbpodo_async_streaming.ack_supervision.enable",
    "servo.rbpodo_async_streaming.reference_supervision.enable",
    "servo.rbpodo_async_streaming.diagnostics.publish_per_command_jsonl",
}
STRING_CONFIG_OVERRIDE_VALUES = {
    "servo.rbpodo_async_streaming.mode": {
        "disabled",
        "sdk_ack_worker",
        "socket_send_supervised",
    },
    "servo.rbpodo_async_streaming.queue_policy": {"latest_wins"},
    "servo.rbpodo_async_streaming.reference_supervision.policy": {
        "warn_only",
        "fault_latch",
    },
    "cartesian_control.velocity_target_integration": {
        "measured_actual",
        "measured_actual_lookahead",
        "previous_command",
    },
}
INTEGER_CONFIG_OVERRIDES = {
    "servo.rbpodo_async_streaming.rate_hz",
    "servo.rbpodo_async_streaming.ack_supervision.max_consecutive_missing_ack",
}
ASYNC_CONFIG_OVERRIDE_PREFIX = "servo.rbpodo_async_streaming."
UNSAFE_OVERRIDE_KEY_PARTS = (
    "operation_mode",
    "allow_in_real",
    "allow_in_controller_simulation",
    "allow_controller_simulation_motion",
    "backend_type",
    "run_mode",
)
RATE_T1_OVERRIDE_KEYS = {
    "servo.rate_hz",
    "left_robot.servo_t1_sec",
    "right_robot.servo_t1_sec",
}
SUMMARY_COLUMNS = [
    "name",
    "controller",
    "profile",
    "arm",
    "ack_policy",
    "async_mode",
    "acceptance_semantics",
    "state_pub_rate_hz",
    "speed_bar_left",
    "speed_bar_right",
    "speed_bar",
    "servo_rate_hz",
    "servo_t1_sec",
    "servo_t2_sec",
    "servo_t2_sec_left",
    "servo_t2_sec_right",
    "servo_alpha",
    "servo_alpha_left",
    "servo_alpha_right",
    "command_rate_hz",
    "phase_advance_sec",
    "phase_advance_fraction_of_period",
    "phase_advance_enabled",
    "commanded_phase_advance_ms",
    "phase_advance_effect",
    "phase_aligned_rms_delta_mm",
    "saturation_ratio_delta",
    "command_count",
    "tracking_source",
    "kp_pos",
    "kp_ori",
    "feedback_kp_pos",
    "feedback_kp_ori",
    "feedback_max_linear_m_s",
    "feedback_max_angular_rad_s",
    "feedback_saturation_count",
    "saturation_ratio",
    "p95_orientation_drift_rad",
    "orientation_p95_deg",
    "fit_center_error_m",
    "center_error_mm",
    "physical_motion_detected",
    "fault_latched",
    "cartesian_unavailable_count",
    "radius_gain",
    "rms_error_mm",
    "median_error_mm",
    "p95_error_mm",
    "max_error_mm",
    "tail_ratio",
    "center_removed_rms_mm",
    "phase_aligned_rms_mm",
    "orientation_position_equiv_50mm_mm",
    "error_classification",
    "p95_orientation_drift_mrad",
    "fit_center_error_mm",
    "estimated_latency_ms",
    "q_ref_update_rate_hz",
    "q_ref_valid_ratio",
    "send_success_rate",
    "controller_acceptance_observed_rate",
    "send_duration_p95_us",
    "send_duration_p99_us",
    "send_duration_max_us",
    "servo_jitter_p99_ms",
    "deadline_miss_count",
    "command_interval_max_ms",
    "timing_classification",
    "ack_spike_count_10ms",
    "ack_spike_count_20ms",
    "state_gap_count",
    "command_gap_count",
    "p95_error_near_ack_spike_mm",
    "p95_error_away_from_ack_spike_mm",
    "p95_error_near_command_gap_mm",
    "p95_error_away_from_command_gap_mm",
    "ack_observed_count",
    "controller_ack_observed_count",
    "controller_acceptance_observed_count",
    "socket_send_only_count",
    "reference_supervision_state",
    "diagnostics_suspect_count",
    "controller_simulation_diagnostic_override_active_count",
    "score",
    "classification",
    "result",
    "run_result_status",
    "benchmark_threshold_status",
    "ackon500_goal_status",
    "diagnostic_warning_count",
    "measurement_reliability_level",
    "reliability_caveats",
    "benchmark_interpretation",
    "physical_real_blockers",
]


class AblationError(RuntimeError):
    pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run a rbpodo-only controller-simulation circle ablation matrix. "
            "Each enabled experiment invokes rbpodo_circle_tracking_benchmark.py "
            "against real Rainbow controller boxes in pgmode simulation."
        )
    )
    parser.add_argument("--root", type=Path, default=Path("."))
    parser.add_argument("--matrix", type=Path, required=True)
    parser.add_argument("--artifact-root", type=Path, required=True)
    parser.add_argument("--max-workers", type=int, default=1)
    parser.add_argument(
        "--server",
        type=Path,
        default=Path("rb_servo_server/build/rbpodo_real_gate/rb_servo_server"),
    )
    parser.add_argument("--dry-run", action="store_true", help="Validate and print benchmark commands without running them.")
    parser.add_argument("--skip-plots", action="store_true", help="Forward --skip-plots to every benchmark run.")
    parser.add_argument("--set-pgmode-simulation", action="store_true")
    parser.add_argument("--verify-pgmode-simulation", action="store_true")
    parser.add_argument("--pgmode-summary-json", type=Path)
    parser.add_argument("--pgmode-timeout-sec", type=float, default=1.0)
    parser.add_argument("--pgmode-command-port", type=int, default=5000)
    parser.add_argument(
        "--i-understand-this-connects-to-real-controller",
        action="store_true",
        help="Required before any real controller connection can be attempted.",
    )
    parser.add_argument(
        "--i-confirm-controller-is-in-pgmode-simulation",
        action="store_true",
        help="Required acknowledgement before controller-simulation ablation is accepted.",
    )
    return parser.parse_args()


def load_matrix(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        raise AblationError(f"matrix not found: {path}")
    return sim_ablation.parse_matrix_text(path.read_text(encoding="utf-8"))


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


def yaml_scalar(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if value is None:
        return "null"
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        number = float(value)
        if not math.isfinite(number):
            raise AblationError(f"non-finite override value: {value}")
        if isinstance(value, int):
            return str(value)
        return f"{number:.12g}"
    if isinstance(value, str):
        return value
    raise AblationError(f"unsupported override value type: {type(value).__name__}")


def config_overrides(exp: dict[str, Any]) -> dict[str, Any]:
    value = exp.get("config_overrides", {})
    if value in (None, ""):
        return {}
    if not isinstance(value, dict):
        raise AblationError(f"experiment {exp.get('name', '<unknown>')} config_overrides must be a mapping")
    out: dict[str, Any] = {}
    for key, override_value in value.items():
        dotted = str(key)
        if dotted not in ALLOWED_CONFIG_OVERRIDES:
            if any(part in dotted for part in UNSAFE_OVERRIDE_KEY_PARTS):
                raise AblationError(f"experiment {exp.get('name', '<unknown>')} unsafe config override is not allowed: {dotted}")
            raise AblationError(f"experiment {exp.get('name', '<unknown>')} has unknown config override: {dotted}")
        validate_override_value(str(exp.get("name", "<unknown>")), dotted, override_value)
        out[dotted] = override_value
    return out


def validate_override_value(name: str, key: str, value: Any) -> None:
    if key in BOOLEAN_CONFIG_OVERRIDES:
        if not isinstance(value, bool):
            raise AblationError(f"experiment {name} override {key} must be true or false")
        return
    if key in STRING_CONFIG_OVERRIDE_VALUES:
        if not isinstance(value, str) or value not in STRING_CONFIG_OVERRIDE_VALUES[key]:
            allowed = ", ".join(sorted(STRING_CONFIG_OVERRIDE_VALUES[key]))
            raise AblationError(f"experiment {name} override {key} must be one of: {allowed}")
        return
    number = finite_number(value)
    if number is None:
        raise AblationError(f"experiment {name} override {key} must be a finite number")
    if key in INTEGER_CONFIG_OVERRIDES:
        if abs(number - int(number)) > 1e-12 or int(number) <= 0:
            raise AblationError(f"experiment {name} override {key} must be a positive integer")
    if key == "network.state_pub_rate_hz" and not (0.0 < number <= MAX_STATE_PUB_RATE_HZ):
        raise AblationError(
            f"experiment {name} override {key} must be > 0 and <= {MAX_STATE_PUB_RATE_HZ:g}"
        )
    elif key == "servo.rate_hz" and number <= 0.0:
        raise AblationError(f"experiment {name} override {key} must be > 0")
    elif key.startswith(ASYNC_CONFIG_OVERRIDE_PREFIX) and number <= 0.0:
        raise AblationError(f"experiment {name} override {key} must be > 0")
    elif key.endswith(".speed_bar") and not (0.0 < number <= 1.0):
        raise AblationError(f"experiment {name} override {key} must be > 0 and <= 1.0")
    elif key.endswith(".servo_t1_sec") and number <= 0.0:
        raise AblationError(f"experiment {name} override {key} must be > 0")
    elif key.endswith(".servo_t2_sec") and not (SERVO_T2_MIN_SEC < number < SERVO_T2_MAX_SEC):
        raise AblationError(f"experiment {name} override {key} must be > 0.02 and < 0.2")
    elif key.endswith(".servo_alpha") and not (SERVO_ALPHA_MIN < number < SERVO_ALPHA_MAX):
        raise AblationError(f"experiment {name} override {key} must be > 0 and < 1.0")
    elif key.endswith(".command_timeout_sec") and number <= 0.0:
        raise AblationError(f"experiment {name} override {key} must be > 0")
    elif key in {
        "cartesian_control.max_twist_linear_m_s",
        "cartesian_control.max_twist_angular_rad_s",
        "cartesian_control.max_linear_move_speed_m_s",
    } and number <= 0.0:
        raise AblationError(f"experiment {name} override {key} must be > 0")
    elif key == "cartesian_control.twist_angular_deadband_rad_s" and number < 0.0:
        raise AblationError(f"experiment {name} override {key} must be >= 0")
    elif key in {"cartesian_control.path_kp_pos", "cartesian_control.path_kp_ori"} and number < 0.0:
        raise AblationError(f"experiment {name} override {key} must be >= 0")


def has_nested_config_overrides(overrides: dict[str, Any]) -> bool:
    return any("." in dotted.split(".", 1)[1] for dotted in overrides)


def set_nested_override(data: dict[str, Any], dotted: str, value: Any) -> None:
    parts = dotted.split(".")
    current: dict[str, Any] = data
    for part in parts[:-1]:
        existing = current.get(part)
        if existing is None:
            existing = {}
            current[part] = existing
        if not isinstance(existing, dict):
            raise AblationError(f"cannot apply nested override through non-mapping key: {dotted}")
        current = existing
    current[parts[-1]] = value


def apply_nested_config_overrides_text(text: str, overrides: dict[str, Any]) -> str:
    try:
        import yaml  # type: ignore
    except Exception as exc:
        raise AblationError("nested async config overrides require PyYAML") from exc
    try:
        data = yaml.safe_load(text)
    except Exception as exc:
        raise AblationError(f"failed to parse config YAML for nested overrides: {exc}") from exc
    if not isinstance(data, dict):
        raise AblationError("config YAML must be a mapping for nested overrides")
    for dotted, value in overrides.items():
        set_nested_override(data, dotted, value)
    return yaml.safe_dump(data, sort_keys=False)


def apply_config_overrides_text(text: str, overrides: dict[str, Any]) -> str:
    if not overrides:
        return text
    if has_nested_config_overrides(overrides):
        return apply_nested_config_overrides_text(text, overrides)
    grouped: dict[str, dict[str, str]] = {}
    for dotted, value in overrides.items():
        section, field = dotted.split(".", 1)
        grouped.setdefault(section, {})[field] = yaml_scalar(value)

    lines = text.splitlines()
    output: list[str] = []
    current_section: str | None = None
    seen_sections: set[str] = set()
    written: set[tuple[str, str]] = set()

    def append_missing(section: str | None) -> None:
        if section not in grouped:
            return
        assert section is not None
        for field, rendered in grouped[section].items():
            marker = (section, field)
            if marker not in written:
                output.append(f"  {field}: {rendered}")
                written.add(marker)

    for raw_line in lines:
        stripped_without_comment = sim_ablation.strip_comment(raw_line).rstrip()
        stripped = stripped_without_comment.strip()
        indent = len(raw_line) - len(raw_line.lstrip(" "))
        if indent == 0 and stripped.endswith(":") and ":" not in stripped[:-1]:
            append_missing(current_section)
            current_section = stripped[:-1]
            seen_sections.add(current_section)
            output.append(raw_line)
            continue
        if current_section in grouped and indent == 2 and ":" in stripped:
            field = stripped.split(":", 1)[0].strip()
            section_overrides = grouped[current_section]
            if field in section_overrides:
                output.append(f"  {field}: {section_overrides[field]}")
                written.add((current_section, field))
                continue
        output.append(raw_line)
    append_missing(current_section)

    missing_sections = sorted(set(grouped) - seen_sections)
    if missing_sections:
        raise AblationError(f"config missing sections for overrides: {', '.join(missing_sections)}")
    return "\n".join(output) + ("\n" if text.endswith("\n") else "")


def yaml_unquote_scalar(value: str) -> str:
    value = sim_ablation.strip_comment(value).strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in {"'", '"'}:
        return value[1:-1]
    return value


def resolve_config_relative_path(raw_path: str, source_config: Path) -> Path:
    path = Path(raw_path)
    if path.is_absolute():
        return path
    candidates = [
        source_config.parent / path,
        source_config.parent.parent / path,
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()
    return candidates[0].resolve()


def resolve_config_relative_paths_text(text: str, source_config: Path) -> str:
    """Make copied configs independent of the artifact directory location."""
    lines = text.splitlines()
    output: list[str] = []
    current_section: str | None = None

    for raw_line in lines:
        stripped_without_comment = sim_ablation.strip_comment(raw_line).rstrip()
        stripped = stripped_without_comment.strip()
        indent = len(raw_line) - len(raw_line.lstrip(" "))
        if indent == 0 and stripped.endswith(":") and ":" not in stripped[:-1]:
            current_section = stripped[:-1]
            output.append(raw_line)
            continue
        if current_section == "kinematics" and indent == 2 and stripped.startswith("urdf:"):
            raw_value = stripped.split(":", 1)[1]
            urdf_path = yaml_unquote_scalar(raw_value)
            if urdf_path and "://" not in urdf_path:
                resolved = resolve_config_relative_path(urdf_path, source_config)
                output.append(f"{' ' * indent}urdf: {json.dumps(str(resolved))}")
                continue
        output.append(raw_line)
    return "\n".join(output) + ("\n" if text.endswith("\n") else "")


def scaled(summary: dict[str, Any], key: str, factor: float) -> float | None:
    value = finite_number(summary.get(key))
    return value * factor if value is not None else None


def scaled_value(value: Any, factor: float) -> float | None:
    number = finite_number(value)
    return number * factor if number is not None else None


def nested_dict(summary: dict[str, Any], key: str) -> dict[str, Any]:
    value = summary.get(key)
    return value if isinstance(value, dict) else {}


def summary_or_nested(summary: dict[str, Any], nested_key: str, key: str) -> Any:
    if key in summary and summary.get(key) is not None:
        return summary.get(key)
    return nested_dict(summary, nested_key).get(key)


def infer_run_result_status(summary: dict[str, Any]) -> str:
    if summary.get("run_result_status") not in (None, ""):
        return str(summary.get("run_result_status"))
    status = summary_or_nested(summary, "run_result", "status")
    if status not in (None, ""):
        return str(status)
    result = str(summary.get("result") or "")
    reason = str(summary.get("result_reason") or "")
    if result in {"completed", "pass"}:
        return "completed"
    if result == "fail" and "threshold" in reason:
        return "completed"
    if result in {"error", "blocked", "faulted", "startup_fault"}:
        return result
    return result


def infer_benchmark_threshold_status(summary: dict[str, Any]) -> str:
    if summary.get("benchmark_threshold_status") not in (None, ""):
        return str(summary.get("benchmark_threshold_status"))
    status = summary_or_nested(summary, "benchmark_threshold_result", "status")
    if status not in (None, ""):
        return str(status)
    failures = text_list(summary.get("threshold_failures"))
    result = str(summary.get("result") or "")
    reason = str(summary.get("result_reason") or "")
    if failures:
        return "fail"
    if "threshold" in reason and result in {"pass", "fail"}:
        return result
    return "not_evaluated"


def infer_ackon500_goal_status(summary: dict[str, Any]) -> str:
    if summary.get("ackon500_goal_status") not in (None, ""):
        return str(summary.get("ackon500_goal_status"))
    status = summary_or_nested(summary, "ackon500_goal_result", "status")
    if status not in (None, ""):
        return str(status)
    goal_pass = summary.get("goal_pass")
    if isinstance(goal_pass, bool):
        return "pass" if goal_pass else "fail"
    return "not_applicable"


def text_list(value: Any) -> list[str]:
    if value is None:
        return []
    if isinstance(value, list):
        return [str(item) for item in value if str(item)]
    if isinstance(value, str):
        if not value:
            return []
        if ";" in value:
            return [item.strip() for item in value.split(";") if item.strip()]
        return [value]
    return [str(value)]


def diagnostic_warning_count(summary: dict[str, Any]) -> int:
    explicit = finite_number(summary.get("diagnostic_warning_count"))
    if explicit is not None:
        return int(explicit)
    return len(text_list(summary.get("diagnostic_warnings")))


def nested_metric(summary: dict[str, Any], key: str, metric: str) -> float | None:
    value = summary.get(key)
    if not isinstance(value, dict):
        return None
    return finite_number(value.get(metric))


def first_present(*values: Any) -> Any:
    for value in values:
        if value is not None:
            return value
    return None


def common_value_or_pair(left: Any, right: Any) -> Any:
    if left == right:
        return left
    if left is None:
        return right
    if right is None:
        return left
    return f"{left}/{right}"


def ratio_or_none(numerator: Any, denominator: Any) -> float | None:
    top = finite_number(numerator)
    bottom = finite_number(denominator)
    if top is None or bottom is None or bottom <= 0.0:
        return None
    return top / bottom


def success_rate(summary: dict[str, Any], denominator: Any) -> float | None:
    existing = finite_number(summary.get("send_success_rate"))
    if existing is not None:
        return existing
    rate = ratio_or_none(summary.get("send_success_count"), denominator)
    if rate is not None:
        return rate
    failures = finite_number(summary.get("send_failure_count"))
    count = finite_number(denominator)
    if failures is None or count is None or count <= 0.0:
        return None
    return max(0.0, 1.0 - failures / count)


def semantics_distribution_count(summary: dict[str, Any], semantics: str) -> Any:
    distribution = summary.get("send_acceptance_semantics_distribution")
    if isinstance(distribution, dict):
        return distribution.get(semantics)
    return None


def append_cell_value(row: dict[str, Any], key: str, value: str) -> None:
    existing = row.get(key)
    parts: list[str] = []
    if isinstance(existing, str):
        parts = [part.strip() for part in existing.split(";") if part.strip()]
    elif isinstance(existing, list):
        parts = [str(part).strip() for part in existing if str(part).strip()]
    if value not in parts:
        parts.append(value)
    row[key] = "; ".join(parts)


def command_interval_max_ms(summary: dict[str, Any], timestamp_alignment: dict[str, Any]) -> float | None:
    value = finite_number(summary.get("command_interval_max_ms"))
    if value is not None:
        return value
    nested = timestamp_alignment.get("command_interval_ms")
    if isinstance(nested, dict):
        return finite_number(nested.get("max"))
    return None


def rate_t1_overridden(overrides: dict[str, Any]) -> bool:
    return any(key in overrides for key in RATE_T1_OVERRIDE_KEYS)


def expected_servo_t1_sec(rate_hz: float) -> float:
    if abs(rate_hz - 100.0) <= 1e-9:
        return 0.01
    if abs(rate_hz - 200.0) <= 1e-9:
        return 0.005
    return 1.0 / rate_hz


def ablation_env_snapshot() -> dict[str, str | None]:
    snapshot = env_snapshot()
    for name in sorted(OPTIONAL_ENV_REQUIREMENTS):
        snapshot.setdefault(name, os.environ.get(name))
    return snapshot


def override_bool(overrides: dict[str, Any], key: str, default: bool = False) -> bool:
    value = overrides.get(key)
    return value if isinstance(value, bool) else default


def async_mode_from_overrides(overrides: dict[str, Any]) -> str:
    mode = overrides.get("servo.rbpodo_async_streaming.mode")
    return str(mode) if mode is not None else "disabled"


def async_enabled_from_overrides(overrides: dict[str, Any]) -> bool:
    return override_bool(overrides, "servo.rbpodo_async_streaming.enable", False)


def validate_servo_rate_t1_alignment(
    name: str,
    config: Any,
    overrides: dict[str, Any],
) -> tuple[bool | None, str]:
    servo_rate_hz = as_float(config.servo.get("rate_hz"))
    if servo_rate_hz is None or servo_rate_hz <= 0.0:
        return None, ""
    expected_t1 = expected_servo_t1_sec(servo_rate_hz)
    mismatches: list[str] = []
    observed = False
    for label, arm_cfg in (("left_robot", config.left), ("right_robot", config.right)):
        servo_t1_sec = arm_cfg.servo_t1_sec
        if servo_t1_sec is None:
            continue
        observed = True
        if abs(float(servo_t1_sec) - expected_t1) > 1e-6:
            mismatches.append(f"{label}.servo_t1_sec {float(servo_t1_sec):.6f} != expected {expected_t1:.6f}")
    if not observed:
        return None, ""
    if not mismatches:
        return True, ""
    warning = f"servo.rate_hz {servo_rate_hz:.6f}: " + "; ".join(mismatches)
    if rate_t1_overridden(overrides) and not as_bool(config.servo.get("allow_servo_t1_rate_mismatch"), False):
        raise AblationError(f"experiment {name} resolved config has unsupported servo rate/t1 mismatch: {warning}")
    return False, warning


def validate_servo_sweep_ranges(name: str, label: str, arm_cfg: Any) -> None:
    servo_t2_sec = arm_cfg.servo_t2_sec
    if servo_t2_sec is None or not (
        SERVO_T2_MIN_SEC < float(servo_t2_sec) < SERVO_T2_MAX_SEC
    ):
        raise AblationError(
            f"experiment {name} {label}.servo_t2_sec must be finite and in (0.02, 0.2)"
        )
    servo_alpha = arm_cfg.servo_alpha
    if servo_alpha is None or not (
        SERVO_ALPHA_MIN < float(servo_alpha) < SERVO_ALPHA_MAX
    ):
        raise AblationError(
            f"experiment {name} {label}.servo_alpha must be finite and in (0, 1)"
        )


def root_path(root: Path, path_value: Any) -> Path:
    path = Path(str(path_value))
    return path if path.is_absolute() else root / path


def list_value(value: Any) -> list[str]:
    if value is None:
        return []
    if isinstance(value, list):
        return [str(item) for item in value]
    if isinstance(value, str):
        if "," in value:
            return [item.strip() for item in value.split(",") if item.strip()]
        return [value]
    raise AblationError("env_requirements must be a string or list")


def validate_experiment(exp: dict[str, Any], index: int) -> None:
    unknown = sorted(set(exp) - EXPERIMENT_KEYS)
    if unknown:
        raise AblationError(f"experiment {index} has unknown keys: {', '.join(unknown)}")
    for required in ("name", "config", "profile", "controller", "arm"):
        if required not in exp:
            raise AblationError(f"experiment {index} missing required key: {required}")
    if str(exp["profile"]) not in PROFILES:
        raise AblationError(f"experiment {exp['name']} has unsupported profile: {exp['profile']}")
    if str(exp["controller"]) not in CONTROLLERS:
        raise AblationError(f"experiment {exp['name']} has unsupported controller: {exp['controller']}")
    if str(exp["arm"]) not in ARMS:
        raise AblationError(f"experiment {exp['name']} has unsupported arm: {exp['arm']}")
    if str(exp.get("plane", "xy")) not in {"xy", "xz", "yz"}:
        raise AblationError(f"experiment {exp['name']} has unsupported plane: {exp['plane']}")
    if str(exp.get("tracking_source", "auto")) not in TRACKING_SOURCES:
        raise AblationError(f"experiment {exp['name']} has unsupported tracking_source: {exp['tracking_source']}")
    for key in ("command_rate_hz", "feedback_kp_pos", "feedback_kp_ori", "warmup_sec", "settle_sec"):
        if key in exp:
            value = finite_number(exp[key])
            nonnegative = key in {"feedback_kp_pos", "feedback_kp_ori", "warmup_sec", "settle_sec"}
            if value is None or (nonnegative and value < 0.0) or (not nonnegative and value <= 0.0):
                raise AblationError(f"experiment {exp['name']} has invalid {key}: {exp[key]}")
    if "phase_advance_sec" in exp:
        value = finite_number(exp["phase_advance_sec"])
        if value is None or value < 0.0:
            raise AblationError(f"experiment {exp['name']} has invalid phase_advance_sec: {exp['phase_advance_sec']}")
        period_sec = circle_bench.PROFILE_DEFAULTS[str(exp["profile"])][1]
        limit = sim_bench.MAX_PHASE_ADVANCE_FRACTION_OF_PERIOD * period_sec
        if value > limit + 1e-12:
            raise AblationError(
                f"experiment {exp['name']} phase_advance_sec exceeds "
                f"{sim_bench.MAX_PHASE_ADVANCE_FRACTION_OF_PERIOD:.2f} * period ({limit:.6g} sec)"
            )
    if "repeat" in exp:
        repeat = finite_number(exp["repeat"])
        if repeat is None or int(repeat) < 1:
            raise AblationError(f"experiment {exp['name']} has invalid repeat: {exp['repeat']}")
    for env_name in list_value(exp.get("env_requirements")):
        if env_name == "RB_ALLOW_REAL_CARTESIAN":
            raise AblationError(f"experiment {exp['name']} may not require RB_ALLOW_REAL_CARTESIAN")
        if env_name not in set(REQUIRED_ENV) | OPTIONAL_ENV_REQUIREMENTS:
            raise AblationError(f"experiment {exp['name']} has unsupported env requirement: {env_name}")
    config_overrides(exp)


def selected_arm(config: Any, arm: str) -> Any:
    return config.left if arm == "left" else config.right


def validate_config(root: Path, exp: dict[str, Any], config_path_override: Path | None = None) -> dict[str, Any]:
    config_path = (config_path_override if config_path_override is not None else root_path(root, exp["config"])).resolve()
    if not config_path.is_file():
        raise AblationError(f"experiment {exp['name']} config not found: {config_path}")
    config = load_config(config_path)
    sections = simple_yaml_sections(config_path)
    for label, arm_cfg in (("left", config.left), ("right", config.right)):
        if arm_cfg.backend_type != "rbpodo":
            raise AblationError(f"experiment {exp['name']} {label}_robot.backend_type must be rbpodo")
        if arm_cfg.run_mode != "real":
            raise AblationError(f"experiment {exp['name']} {label}_robot.run_mode must be real")
        if arm_cfg.operation_mode not in {"simulation", "sim"}:
            actual = arm_cfg.operation_mode or "<missing>"
            raise AblationError(
                f"experiment {exp['name']} config operation_mode is {actual}; refusing physical real circle benchmark"
            )
        if not arm_cfg.ip:
            raise AblationError(f"experiment {exp['name']} {label}_robot.ip is required")
        validate_servo_sweep_ranges(str(exp["name"]), f"{label}_robot", arm_cfg)
    if config.left.disable_waiting_ack != config.right.disable_waiting_ack:
        raise AblationError(f"experiment {exp['name']} has mismatched left/right ACK policy")
    send_servo_commands = as_bool(config.servo.get("send_servo_commands"), False)
    if not send_servo_commands:
        raise AblationError(f"experiment {exp['name']} requires servo.send_servo_commands=true")
    if not as_bool(config.servo.get("allow_controller_simulation_motion"), False):
        raise AblationError(
            f"experiment {exp['name']} requires servo.allow_controller_simulation_motion=true"
        )
    cartesian = sections.get("cartesian_control", {})
    if not as_bool(cartesian.get("enable"), False):
        raise AblationError(f"experiment {exp['name']} requires cartesian_control.enable=true")
    if as_bool(cartesian.get("allow_in_real"), False):
        raise AblationError(f"experiment {exp['name']} must keep cartesian_control.allow_in_real=false")
    if not as_bool(cartesian.get("allow_in_controller_simulation"), False):
        raise AblationError(
            f"experiment {exp['name']} must keep cartesian_control.allow_in_controller_simulation=true"
        )

    overrides = config_overrides(exp)
    async_enabled = async_enabled_from_overrides(overrides)
    async_mode = async_mode_from_overrides(overrides)
    async_reference_supervision_enabled = override_bool(
        overrides,
        "servo.rbpodo_async_streaming.reference_supervision.enable",
        False,
    )
    if not async_enabled and async_mode != "disabled":
        raise AblationError(
            f"experiment {exp['name']} has async mode {async_mode} while async streaming is disabled"
        )
    if async_enabled and async_mode == "disabled":
        raise AblationError(
            f"experiment {exp['name']} enables async streaming but leaves mode disabled"
        )
    if async_mode == "socket_send_supervised" and not (
        config.left.disable_waiting_ack and config.right.disable_waiting_ack
    ):
        raise AblationError(
            f"experiment {exp['name']} socket_send_supervised requires both disable_waiting_ack fields true"
        )
    if async_mode == "sdk_ack_worker" and (config.left.disable_waiting_ack or config.right.disable_waiting_ack):
        raise AblationError(
            f"experiment {exp['name']} sdk_ack_worker must keep both disable_waiting_ack fields false"
        )
    arm_cfg = selected_arm(config, str(exp["arm"]))
    servo_rate_hz = as_float(config.servo.get("rate_hz"))
    servo_t1_sec = arm_cfg.servo_t1_sec
    t1_aligned, alignment_warning = validate_servo_rate_t1_alignment(str(exp["name"]), config, overrides)
    state_pub_rate_hz = as_float(config.network.get("state_pub_rate_hz"))
    if state_pub_rate_hz is not None and not (0.0 < state_pub_rate_hz <= MAX_STATE_PUB_RATE_HZ):
        raise AblationError(
            f"experiment {exp['name']} network.state_pub_rate_hz must be > 0 and <= {MAX_STATE_PUB_RATE_HZ:g}"
        )
    left_speed_bar = as_float(sections.get("left_robot", {}).get("speed_bar"))
    right_speed_bar = as_float(sections.get("right_robot", {}).get("speed_bar"))
    for label, value in (("left_robot.speed_bar", left_speed_bar), ("right_robot.speed_bar", right_speed_bar)):
        if value is not None and not (0.0 < value <= 1.0):
            raise AblationError(f"experiment {exp['name']} {label} must be > 0 and <= 1.0")
    max_twist_linear = as_float(cartesian.get("max_twist_linear_m_s"))
    max_twist_angular = as_float(cartesian.get("max_twist_angular_rad_s"))
    max_linear_move_speed = as_float(cartesian.get("max_linear_move_speed_m_s"))
    twist_angular_deadband = as_float(cartesian.get("twist_angular_deadband_rad_s"))
    for label, value in (
        ("cartesian_control.max_twist_linear_m_s", max_twist_linear),
        ("cartesian_control.max_twist_angular_rad_s", max_twist_angular),
        ("cartesian_control.max_linear_move_speed_m_s", max_linear_move_speed),
    ):
        if value is not None and value <= 0.0:
            raise AblationError(f"experiment {exp['name']} {label} must be > 0")
    velocity_target_lookahead = as_float(cartesian.get("velocity_target_lookahead_sec"))
    if twist_angular_deadband is not None and twist_angular_deadband < 0.0:
        raise AblationError(f"experiment {exp['name']} cartesian_control.twist_angular_deadband_rad_s must be >= 0")
    if velocity_target_lookahead is not None and velocity_target_lookahead < 0.0:
        raise AblationError(
            f"experiment {exp['name']} cartesian_control.velocity_target_lookahead_sec must be >= 0"
        )
    return {
        "config_path": str(config_path),
        "base_config_path": str(root_path(root, exp["config"]).resolve()),
        "config_overrides": overrides,
        "configured_ips": [config.left.ip, config.right.ip],
        "known_real_ips": sorted({config.left.ip, config.right.ip} & REAL_ROBOT_IPS),
        "ack_policy": "ack_off" if config.left.disable_waiting_ack else "ack_on",
        "disable_waiting_ack": bool(config.left.disable_waiting_ack),
        "async_streaming_enabled": async_enabled,
        "async_mode": async_mode,
        "async_reference_supervision_enabled": async_reference_supervision_enabled,
        "acceptance_semantics": "socket_send_only"
        if async_mode == "socket_send_supervised" or config.left.disable_waiting_ack
        else "controller_ack_observed",
        "state_pub_rate_hz": state_pub_rate_hz,
        "speed_bar_left": left_speed_bar,
        "speed_bar_right": right_speed_bar,
        "servo_rate_hz": servo_rate_hz,
        "servo_t1_sec": servo_t1_sec,
        "servo_t2_sec": common_value_or_pair(config.left.servo_t2_sec, config.right.servo_t2_sec),
        "servo_t2_sec_left": config.left.servo_t2_sec,
        "servo_t2_sec_right": config.right.servo_t2_sec,
        "servo_alpha": common_value_or_pair(config.left.servo_alpha, config.right.servo_alpha),
        "servo_alpha_left": config.left.servo_alpha,
        "servo_alpha_right": config.right.servo_alpha,
        "servo_t1_rate_aligned": t1_aligned,
        "alignment_warning": alignment_warning,
        "cartesian_max_twist_linear_m_s": max_twist_linear,
        "cartesian_max_twist_angular_rad_s": max_twist_angular,
        "cartesian_max_linear_move_speed_m_s": max_linear_move_speed,
        "cartesian_path_kp_pos": as_float(cartesian.get("path_kp_pos")),
        "cartesian_path_kp_ori": as_float(cartesian.get("path_kp_ori")),
        "cartesian_twist_angular_deadband_rad_s": twist_angular_deadband,
        "cartesian_velocity_target_integration": cartesian.get("velocity_target_integration"),
        "cartesian_velocity_target_lookahead_sec": velocity_target_lookahead,
        "allow_controller_simulation_diagnostics_suspect": as_bool(
            config.servo.get("allow_controller_simulation_diagnostics_suspect"), False
        ),
        "command_bind": config.network.get("command_bind"),
        "state_pub_endpoint": config.network.get("state_pub_endpoint"),
    }


def experiment_dir(artifact_root: Path, index: int, exp: dict[str, Any]) -> Path:
    return artifact_root / f"{index:02d}_{sim_ablation.safe_name(str(exp['name']))}"


def prepare_experiment_config(root: Path, exp: dict[str, Any], exp_dir: Path) -> dict[str, Any]:
    base_config = root_path(root, exp["config"]).resolve()
    if not base_config.is_file():
        raise AblationError(f"experiment {exp['name']} config not found: {base_config}")
    overrides = config_overrides(exp)
    text = base_config.read_text(encoding="utf-8")
    resolved_text = apply_config_overrides_text(text, overrides)
    resolved_text = resolve_config_relative_paths_text(resolved_text, base_config)
    exp_dir.mkdir(parents=True, exist_ok=True)
    resolved_config = exp_dir / "resolved_server_config.yaml"
    resolved_config.write_text(resolved_text, encoding="utf-8")
    metadata = validate_config(root, exp, resolved_config)
    metadata["resolved_config_path"] = str(resolved_config.resolve())
    metadata["source_config_unchanged"] = base_config.read_text(encoding="utf-8") == text
    return metadata


def validate_matrix_safety(args: argparse.Namespace, metadata: list[dict[str, Any]], experiments: list[dict[str, Any]]) -> None:
    if args.max_workers < 1:
        raise AblationError("--max-workers must be >= 1")
    if args.set_pgmode_simulation and args.verify_pgmode_simulation:
        raise AblationError("--set-pgmode-simulation and --verify-pgmode-simulation are mutually exclusive")
    if args.pgmode_summary_json and (args.set_pgmode_simulation or args.verify_pgmode_simulation):
        raise AblationError("--pgmode-summary-json cannot be combined with pgmode set/verify flags")
    if not (args.set_pgmode_simulation or args.verify_pgmode_simulation or args.pgmode_summary_json):
        raise AblationError(
            "controller-simulation ablation requires --set-pgmode-simulation, "
            "--verify-pgmode-simulation, or --pgmode-summary-json"
        )
    if args.pgmode_summary_json and not root_path(args.root, args.pgmode_summary_json).is_file():
        raise AblationError(f"pgmode summary not found: {root_path(args.root, args.pgmode_summary_json)}")
    if not args.i_understand_this_connects_to_real_controller:
        raise AblationError("missing --i-understand-this-connects-to-real-controller")
    if not args.i_confirm_controller_is_in_pgmode_simulation:
        raise AblationError("missing --i-confirm-controller-is-in-pgmode-simulation")
    for name in REQUIRED_ENV:
        if not env_enabled(name):
            raise AblationError(f"rbpodo circle ablation requires {name}=1")
    if env_enabled("RB_ALLOW_REAL_CARTESIAN"):
        raise AblationError("RB_ALLOW_REAL_CARTESIAN must not be set for controller-simulation circle ablation")
    for exp, meta in zip(experiments, metadata):
        for env_name in list_value(exp.get("env_requirements")):
            if not env_enabled(env_name):
                raise AblationError(f"experiment {exp['name']} requires {env_name}=1")
        if meta["disable_waiting_ack"] and not env_enabled("RB_ALLOW_RBPODO_ACK_DISABLED_MOTION"):
            raise AblationError(
                f"experiment {exp['name']} is ACK-off and requires RB_ALLOW_RBPODO_ACK_DISABLED_MOTION=1"
            )
        if meta.get("async_streaming_enabled") and not env_enabled("RB_ALLOW_RBPODO_ASYNC_STREAMING"):
            raise AblationError(
                f"experiment {exp['name']} enables rbpodo async streaming and requires "
                "RB_ALLOW_RBPODO_ASYNC_STREAMING=1"
            )
        if meta.get("async_mode") == "socket_send_supervised":
            if meta.get("acceptance_semantics") == "controller_ack_observed":
                raise AblationError(f"experiment {exp['name']} mislabels socket_send_supervised as controller ACK")
            if not meta.get("async_reference_supervision_enabled"):
                raise AblationError(
                    f"experiment {exp['name']} socket_send_supervised requires reference supervision"
                )
        if meta["allow_controller_simulation_diagnostics_suspect"] and not env_enabled(
            "RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM"
        ):
            raise AblationError(
                f"experiment {exp['name']} enables diagnostics-suspect override and requires "
                "RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1"
            )


def benchmark_command(args: argparse.Namespace, exp: dict[str, Any], meta: dict[str, Any], exp_dir: Path) -> list[str]:
    command = [
        sys.executable,
        "scripts/rbpodo_circle_tracking_benchmark.py",
        "--root",
        str(args.root),
        "--server",
        str(args.server),
        "--server-config",
        meta["config_path"],
        "--arm",
        str(exp["arm"]),
        "--controller",
        str(exp["controller"]),
        "--plane",
        str(exp.get("plane", "xy")),
        "--profile",
        str(exp["profile"]),
        "--repeat",
        str(exp.get("repeat", 1)),
        "--command-rate-hz",
        str(exp.get("command_rate_hz", 100)),
        "--tracking-source",
        str(exp.get("tracking_source", "auto")),
        "--artifact-dir",
        str(exp_dir),
        "--i-understand-this-connects-to-real-controller",
        "--i-confirm-controller-is-in-pgmode-simulation",
    ]
    if sim_bench.profile_requires_fast_stress(str(exp["profile"])):
        command.append("--allow-fast-stress")
    for matrix_key, cli_key in (
        ("feedback_kp_pos", "--feedback-kp-pos"),
        ("feedback_kp_ori", "--feedback-kp-ori"),
        ("feedback_max_linear_m_s", "--feedback-max-linear-m-s"),
        ("feedback_max_angular_rad_s", "--feedback-max-angular-rad-s"),
        ("phase_advance_sec", "--phase-advance-sec"),
        ("warmup_sec", "--warmup-sec"),
        ("settle_sec", "--settle-sec"),
        ("startup_timeout_sec", "--startup-timeout-sec"),
        ("max_state_age_us", "--max-state-age-us"),
        ("physical_motion_warning_deg", "--physical-motion-warning-deg"),
        ("max_allowed_rms_error_m", "--max-allowed-rms-error-m"),
        ("max_allowed_p95_error_m", "--max-allowed-p95-error-m"),
        ("max_allowed_orientation_drift_rad", "--max-allowed-orientation-drift-rad"),
        ("max_allowed_latency_ms", "--max-allowed-latency-ms"),
    ):
        if matrix_key in exp:
            command.extend([cli_key, str(exp[matrix_key])])
    if as_bool(exp.get("feedback_use_current_state_time"), False):
        command.append("--feedback-use-current-state-time")
    if args.skip_plots or as_bool(exp.get("skip_plots"), False):
        command.append("--skip-plots")
    if args.set_pgmode_simulation:
        command.append("--set-pgmode-simulation")
    elif args.verify_pgmode_simulation:
        command.append("--verify-pgmode-simulation")
    elif args.pgmode_summary_json:
        command.extend(["--pgmode-summary-json", str(root_path(args.root, args.pgmode_summary_json).resolve())])
    command.extend(["--pgmode-timeout-sec", str(args.pgmode_timeout_sec)])
    command.extend(["--pgmode-command-port", str(args.pgmode_command_port)])
    return command


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise AblationError(f"failed to read {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise AblationError(f"{path} does not contain a JSON object")
    return value


def run_experiment(
    args: argparse.Namespace,
    exp: dict[str, Any],
    meta: dict[str, Any],
    index: int,
    artifact_root: Path,
) -> dict[str, Any]:
    exp_dir = experiment_dir(artifact_root, index, exp)
    exp_dir.mkdir(parents=True, exist_ok=True)
    command = benchmark_command(args, exp, meta, exp_dir)
    command_text = shlex.join(command)
    (exp_dir / "experiment_command.txt").write_text(command_text + "\n", encoding="utf-8")
    write_json(exp_dir / "ablation_command.json", command)
    if args.dry_run:
        print(f"resolved_config: {meta['resolved_config_path']}")
        print(command_text)
        return {
            "schema": circle_bench.SCHEMA,
            "result": "dry_run",
            "result_reason": "dry-run; benchmark command was not executed",
            "artifact_dir": str(exp_dir.resolve()),
            "controller": exp.get("controller"),
            "arm": exp.get("arm"),
            "profile": exp.get("profile"),
            "command_rate_hz": exp.get("command_rate_hz", 100),
            "phase_advance_sec": exp.get("phase_advance_sec", 0.0),
            "phase_advance_fraction_of_period": (
                finite_number(exp.get("phase_advance_sec", 0.0)) / circle_bench.PROFILE_DEFAULTS[str(exp["profile"])][1]
                if finite_number(exp.get("phase_advance_sec", 0.0)) is not None
                else None
            ),
            "phase_advance_enabled": bool(finite_number(exp.get("phase_advance_sec", 0.0)) or 0.0),
            "tracking_source_used": exp.get("tracking_source", "auto"),
            "server_config": meta["config_path"],
            "physical_motion_expected": False,
        }
    completed = subprocess.run(
        command,
        cwd=str(args.root.resolve()),
        env=os.environ.copy(),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    (exp_dir / "ablation_runner.log").write_text(completed.stdout, encoding="utf-8")
    summary_path = exp_dir / "summary.json"
    if summary_path.is_file():
        summary = load_json(summary_path)
    else:
        summary = {
            "schema": circle_bench.SCHEMA,
            "result": "error",
            "result_reason": "rbpodo circle benchmark did not produce summary.json",
            "error": completed.stdout[-4000:],
            "artifact_dir": str(exp_dir.resolve()),
        }
    if completed.returncode != 0 and summary.get("result") != "error":
        summary["ablation_runner_warning"] = f"rbpodo circle benchmark exited with {completed.returncode}"
    return summary


def row_from_summary(summary: dict[str, Any], exp: dict[str, Any], meta: dict[str, Any]) -> dict[str, Any]:
    command_count = first_present(
        summary.get("command_count"),
        summary.get("ack_observed_count"),
        summary.get("controller_acceptance_observed_count"),
    )
    feedback_kp_pos = first_present(summary.get("feedback_kp_pos"), exp.get("feedback_kp_pos"))
    feedback_kp_ori = first_present(summary.get("feedback_kp_ori"), exp.get("feedback_kp_ori"))
    timestamp_alignment = nested_dict(summary, "timestamp_alignment")
    tail_error_correlation = nested_dict(summary, "tail_error_correlation")
    async_mode = first_present(summary.get("async_mode"), summary.get("async_streaming_mode"), meta.get("async_mode"))
    acceptance_semantics = first_present(
        summary.get("acceptance_semantics"),
        summary.get("ack_semantics"),
        meta.get("acceptance_semantics"),
    )
    socket_send_only_count = first_present(
        summary.get("socket_send_only_count"),
        semantics_distribution_count(summary, "socket_send_only"),
        0 if acceptance_semantics == "controller_ack_observed" else None,
    )
    if acceptance_semantics == "socket_send_only":
        controller_ack_observed_count = 0
    else:
        controller_ack_observed_count = first_present(
            summary.get("controller_ack_observed_count"),
            summary.get("controller_acceptance_observed_count"),
            semantics_distribution_count(summary, "controller_ack_observed"),
        )
    row = {
        "name": exp.get("name"),
        "controller": summary.get("controller") or exp.get("controller"),
        "profile": summary.get("profile") or exp.get("profile"),
        "arm": summary.get("arm") or exp.get("arm"),
        "ack_policy": meta.get("ack_policy"),
        "async_mode": async_mode,
        "acceptance_semantics": acceptance_semantics,
        "state_pub_rate_hz": meta.get("state_pub_rate_hz"),
        "speed_bar_left": meta.get("speed_bar_left"),
        "speed_bar_right": meta.get("speed_bar_right"),
        "speed_bar": common_value_or_pair(meta.get("speed_bar_left"), meta.get("speed_bar_right")),
        "servo_rate_hz": first_present(summary.get("servo_rate_hz"), meta.get("servo_rate_hz")),
        "servo_t1_sec": meta.get("servo_t1_sec"),
        "servo_t2_sec": first_present(summary.get("servo_t2_sec"), meta.get("servo_t2_sec")),
        "servo_t2_sec_left": first_present(summary.get("servo_t2_sec_left"), meta.get("servo_t2_sec_left")),
        "servo_t2_sec_right": first_present(summary.get("servo_t2_sec_right"), meta.get("servo_t2_sec_right")),
        "servo_alpha": first_present(summary.get("servo_alpha"), meta.get("servo_alpha")),
        "servo_alpha_left": first_present(summary.get("servo_alpha_left"), meta.get("servo_alpha_left")),
        "servo_alpha_right": first_present(summary.get("servo_alpha_right"), meta.get("servo_alpha_right")),
        "command_rate_hz": first_present(summary.get("command_rate_hz"), exp.get("command_rate_hz")),
        "phase_advance_sec": first_present(summary.get("phase_advance_sec"), exp.get("phase_advance_sec")),
        "phase_advance_fraction_of_period": first_present(
            summary.get("phase_advance_fraction_of_period"),
            (
                finite_number(exp.get("phase_advance_sec")) / circle_bench.PROFILE_DEFAULTS[str(exp["profile"])][1]
                if finite_number(exp.get("phase_advance_sec")) is not None
                else None
            ),
        ),
        "phase_advance_enabled": first_present(
            summary.get("phase_advance_enabled"),
            bool(finite_number(exp.get("phase_advance_sec")) or 0.0),
        ),
        "commanded_phase_advance_ms": first_present(
            summary.get("commanded_phase_advance_ms"),
            scaled_value(exp.get("phase_advance_sec"), 1000.0),
        ),
        "command_count": command_count,
        "tracking_source": first_present(summary.get("tracking_source_used"), exp.get("tracking_source", "auto")),
        "kp_pos": feedback_kp_pos,
        "kp_ori": feedback_kp_ori,
        "feedback_kp_pos": feedback_kp_pos,
        "feedback_kp_ori": feedback_kp_ori,
        "feedback_max_linear_m_s": first_present(
            summary.get("feedback_max_linear_m_s"), exp.get("feedback_max_linear_m_s")
        ),
        "feedback_max_angular_rad_s": first_present(
            summary.get("feedback_max_angular_rad_s"), exp.get("feedback_max_angular_rad_s")
        ),
        "feedback_saturation_count": summary.get("feedback_saturation_count"),
        "saturation_ratio": ratio_or_none(summary.get("feedback_saturation_count"), command_count),
        "p95_orientation_drift_rad": summary.get("p95_orientation_drift_rad"),
        "orientation_p95_deg": scaled(summary, "p95_orientation_drift_rad", 180.0 / math.pi),
        "fit_center_error_m": summary.get("fit_center_error_m"),
        "center_error_mm": scaled(summary, "fit_center_error_m", 1000.0),
        "physical_motion_detected": summary.get("physical_motion_detected"),
        "fault_latched": summary.get("fault_latched"),
        "cartesian_unavailable_count": summary.get("cartesian_unavailable_count"),
        "radius_gain": summary.get("radius_gain"),
        "rms_error_mm": scaled(summary, "rms_error_m", 1000.0),
        "median_error_mm": scaled_value(summary_or_nested(summary, "error_decomposition", "median_error_m"), 1000.0),
        "p95_error_mm": scaled(summary, "p95_error_m", 1000.0),
        "max_error_mm": scaled(summary, "max_error_m", 1000.0),
        "tail_ratio": summary_or_nested(summary, "error_decomposition", "tail_ratio"),
        "center_removed_rms_mm": scaled_value(
            summary_or_nested(summary, "error_decomposition", "center_removed_rms_error_m"),
            1000.0,
        ),
        "phase_aligned_rms_mm": scaled_value(
            summary_or_nested(summary, "error_decomposition", "phase_aligned_rms_error_m"),
            1000.0,
        ),
        "orientation_position_equiv_50mm_mm": scaled_value(
            summary_or_nested(summary, "error_decomposition", "orientation_position_equiv_50mm_m"),
            1000.0,
        ),
        "error_classification": summary_or_nested(summary, "error_decomposition", "error_classification"),
        "p95_orientation_drift_mrad": scaled(summary, "p95_orientation_drift_rad", 1000.0),
        "fit_center_error_mm": scaled(summary, "fit_center_error_m", 1000.0),
        "estimated_latency_ms": summary.get("estimated_latency_ms"),
        "q_ref_update_rate_hz": summary.get("q_ref_update_rate_hz"),
        "q_ref_valid_ratio": summary.get("q_ref_valid_ratio"),
        "send_success_rate": success_rate(summary, command_count),
        "controller_acceptance_observed_rate": first_present(
            summary.get("controller_acceptance_observed_rate"),
            summary.get("controller_acceptance_ratio"),
            ratio_or_none(summary.get("controller_acceptance_observed_count"), command_count),
        ),
        "send_duration_p95_us": nested_metric(summary, "send_duration_us", "p95"),
        "send_duration_p99_us": nested_metric(summary, "send_duration_us", "p99"),
        "send_duration_max_us": nested_metric(summary, "send_duration_us", "max"),
        "servo_jitter_p99_ms": nested_metric(summary, "servo_jitter_ms", "p99"),
        "deadline_miss_count": first_present(
            summary.get("deadline_miss_count"),
            summary.get("send_deadline_missed_count"),
            summary.get("send_command_deadline_missed_count"),
            summary.get("command_sender_deadline_missed_count"),
        ),
        "command_interval_max_ms": command_interval_max_ms(summary, timestamp_alignment),
        "timing_classification": first_present(
            summary.get("timing_classification"),
            timestamp_alignment.get("timing_classification"),
        ),
        "ack_spike_count_10ms": first_present(
            summary.get("ack_spike_count_10ms"),
            timestamp_alignment.get("ack_spike_count_10ms"),
        ),
        "ack_spike_count_20ms": first_present(
            summary.get("ack_spike_count_20ms"),
            timestamp_alignment.get("ack_spike_count_20ms"),
        ),
        "state_gap_count": first_present(summary.get("state_gap_count"), timestamp_alignment.get("state_gap_count")),
        "command_gap_count": first_present(summary.get("command_gap_count"), timestamp_alignment.get("command_gap_count")),
        "p95_error_near_ack_spike_mm": scaled_value(
            first_present(
                summary.get("p95_error_near_ack_spike_m"),
                tail_error_correlation.get("p95_error_near_ack_spike_m"),
            ),
            1000.0,
        ),
        "p95_error_away_from_ack_spike_mm": scaled_value(
            first_present(
                summary.get("p95_error_away_from_ack_spike_m"),
                tail_error_correlation.get("p95_error_away_from_ack_spike_m"),
            ),
            1000.0,
        ),
        "p95_error_near_command_gap_mm": scaled_value(
            first_present(
                summary.get("p95_error_near_command_gap_m"),
                tail_error_correlation.get("p95_error_near_command_gap_m"),
            ),
            1000.0,
        ),
        "p95_error_away_from_command_gap_mm": scaled_value(
            first_present(
                summary.get("p95_error_away_from_command_gap_m"),
                tail_error_correlation.get("p95_error_away_from_command_gap_m"),
            ),
            1000.0,
        ),
        "ack_observed_count": summary.get("ack_observed_count"),
        "controller_ack_observed_count": controller_ack_observed_count,
        "controller_acceptance_observed_count": summary.get("controller_acceptance_observed_count"),
        "socket_send_only_count": socket_send_only_count,
        "reference_supervision_state": first_present(
            summary.get("reference_supervision_state"),
            summary.get("async_reference_supervision_state"),
            "configured"
            if meta.get("async_reference_supervision_enabled")
            else "not_applicable",
        ),
        "diagnostics_suspect_count": summary.get("diagnostics_suspect_count"),
        "controller_simulation_diagnostic_override_active_count": summary.get(
            "controller_simulation_diagnostic_override_active_count"
        ),
        "score": summary.get("score"),
        "classification": summary.get("classification"),
        "result": summary.get("result"),
        "run_result_status": infer_run_result_status(summary),
        "benchmark_threshold_status": infer_benchmark_threshold_status(summary),
        "ackon500_goal_status": infer_ackon500_goal_status(summary),
        "diagnostic_warning_count": diagnostic_warning_count(summary),
        "artifact_dir": summary.get("artifact_dir"),
        "warnings": warning_text(summary, meta),
    }
    row.setdefault("benchmark_category", "rbpodo_controller_simulation")
    row.setdefault("backend", "rbpodo")
    row.setdefault("controller_mode", "pgmode_simulation")
    row.setdefault("physical_motion_expected", False)
    reliability_report.annotate_row(row)
    if acceptance_semantics == "socket_send_only" or async_mode == "socket_send_supervised":
        append_cell_value(row, "reliability_caveats", "socket_send_only_not_controller_ack")
        append_cell_value(row, "benchmark_interpretation", "reference_supervision_required")
    return row


def warning_text(summary: dict[str, Any], meta: dict[str, Any]) -> str:
    warnings: list[str] = []
    if meta.get("alignment_warning"):
        warnings.append(str(meta["alignment_warning"]))
    value = summary.get("performance_warnings")
    if isinstance(value, list):
        warnings.extend(str(item) for item in value)
    elif isinstance(value, str) and value:
        warnings.append(value)
    for item in text_list(summary.get("diagnostic_warnings")):
        warnings.append(f"diagnostic_warning={item}")
    if summary.get("physical_motion_detected") is True:
        warnings.append("physical_motion_detected true in pgmode simulation")
    if infer_run_result_status(summary) == "error":
        reason = summary.get("result_reason") or summary.get("error") or "error"
        warnings.append(str(reason))
    if meta.get("ack_policy") == "ack_off":
        warnings.append("ACK-off controller-simulation evidence is experimental")
    if meta.get("acceptance_semantics") == "socket_send_only":
        warnings.append("socket_send_only evidence is not controller ACK acceptance")
    timing = summary.get("timing_classification")
    if timing is None:
        timestamp_alignment = summary.get("timestamp_alignment")
        if isinstance(timestamp_alignment, dict):
            timing = timestamp_alignment.get("timing_classification")
    if timing not in (None, "", "clean_timing"):
        warnings.append(f"timestamp_alignment timing_classification={timing}")
    error_classification = summary_or_nested(summary, "error_decomposition", "error_classification")
    if error_classification:
        warnings.append(f"error_classification={error_classification}")
    return "; ".join(dict.fromkeys(warnings))


def rows_from_summaries(
    summaries: list[dict[str, Any]],
    experiments: list[dict[str, Any]],
    metadata: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    return [
        row_from_summary(summary, exp, meta)
        for summary, exp, meta in zip(summaries, experiments, metadata)
    ]


def phase_compare_value(value: Any) -> Any:
    number = finite_number(value)
    if number is not None:
        return round(number, 9)
    return value


def phase_advance_compare_key(row: dict[str, Any]) -> tuple[Any, ...]:
    keys = (
        "controller",
        "profile",
        "arm",
        "ack_policy",
        "state_pub_rate_hz",
        "speed_bar_left",
        "speed_bar_right",
        "servo_rate_hz",
        "servo_t1_sec",
        "servo_t2_sec",
        "servo_alpha",
        "command_rate_hz",
        "tracking_source",
        "feedback_kp_pos",
        "feedback_kp_ori",
        "feedback_max_linear_m_s",
        "feedback_max_angular_rad_s",
    )
    return tuple(phase_compare_value(row.get(key)) for key in keys)


def phase_saturation_metric(row: dict[str, Any]) -> float | None:
    value = finite_number(row.get("saturation_ratio"))
    if value is not None:
        return value
    return finite_number(row.get("feedback_saturation_count"))


def annotate_phase_advance_effects(rows: list[dict[str, Any]]) -> None:
    baselines: dict[tuple[Any, ...], dict[str, Any]] = {}
    for row in rows:
        phase = finite_number(row.get("phase_advance_sec")) or 0.0
        if abs(phase) > 1e-12:
            continue
        key = phase_advance_compare_key(row)
        current = baselines.get(key)
        current_rms = finite_number(current.get("phase_aligned_rms_mm")) if current else None
        row_rms = finite_number(row.get("phase_aligned_rms_mm"))
        if current is None or (row_rms is not None and (current_rms is None or row_rms < current_rms)):
            baselines[key] = row
        row["phase_advance_effect"] = "phase_advance_baseline"
        row["phase_aligned_rms_delta_mm"] = None
        row["saturation_ratio_delta"] = None

    for row in rows:
        phase = finite_number(row.get("phase_advance_sec")) or 0.0
        if abs(phase) <= 1e-12:
            continue
        baseline = baselines.get(phase_advance_compare_key(row))
        if baseline is None:
            row["phase_advance_effect"] = "phase_advance_no_zero_baseline"
            row["phase_aligned_rms_delta_mm"] = None
            row["saturation_ratio_delta"] = None
            continue

        baseline_rms = finite_number(baseline.get("phase_aligned_rms_mm"))
        row_rms = finite_number(row.get("phase_aligned_rms_mm"))
        baseline_saturation = phase_saturation_metric(baseline)
        row_saturation = phase_saturation_metric(row)
        rms_delta = row_rms - baseline_rms if row_rms is not None and baseline_rms is not None else None
        saturation_delta = (
            row_saturation - baseline_saturation
            if row_saturation is not None and baseline_saturation is not None
            else None
        )
        row["phase_aligned_rms_delta_mm"] = rms_delta
        row["saturation_ratio_delta"] = saturation_delta
        rms_reduced = rms_delta is not None and rms_delta < -1e-9
        saturation_reduced = saturation_delta is not None and saturation_delta < -1e-12
        if rms_reduced and saturation_reduced:
            effect = "phase_advance_reduces_phase_aligned_rms_and_saturation"
        elif rms_reduced:
            effect = "phase_advance_reduces_phase_aligned_rms"
        elif saturation_reduced:
            effect = "phase_advance_reduces_saturation"
        else:
            effect = "phase_advance_no_measured_reduction"
        row["phase_advance_effect"] = effect


def write_csv_rows(path: Path, rows: list[dict[str, Any]]) -> None:
    fieldnames = SUMMARY_COLUMNS + ["artifact_dir", "warnings"]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in fieldnames})


def format_cell(value: Any) -> str:
    return sim_ablation.format_cell(value)


def markdown_table(rows: list[dict[str, Any]], columns: list[str] | None = None) -> str:
    columns = columns or SUMMARY_COLUMNS + ["artifact_dir", "warnings"]
    lines = [
        "| " + " | ".join(columns) + " |",
        "| " + " | ".join("---" for _ in columns) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(format_cell(row.get(key)) for key in columns) + " |")
    return "\n".join(lines)


def rejected(row: dict[str, Any]) -> bool:
    if child_result_failed(row):
        return True
    if row.get("physical_motion_detected") is True:
        return True
    if row.get("fault_latched") is True:
        return True
    if finite_number(row.get("diagnostics_suspect_count")) not in (None, 0.0):
        return True
    return False


def child_result_failed(value: dict[str, Any]) -> bool:
    status = value.get("run_result_status")
    if status not in (None, ""):
        return str(status) in {"error", "blocked", "faulted", "startup_fault"}
    result = str(value.get("result") or "")
    reason = str(value.get("result_reason") or "")
    if result == "fail" and "threshold" in reason:
        return False
    return result in {"error", "blocked", "faulted", "startup_fault"}


def decision_split_markdown(rows: list[dict[str, Any]]) -> str:
    columns = [
        "name",
        "classification",
        "score",
        "error_classification",
        "timing_classification",
        "measurement_reliability_level",
        "reliability_caveats",
        "physical_real_blockers",
    ]
    if not rows:
        return "_None._"
    lines = [
        "| " + " | ".join(columns) + " |",
        "| " + " | ".join("---" for _ in columns) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(format_cell(row.get(key)) for key in columns) + " |")
    return "\n".join(lines)


def write_report(path: Path, rows: list[dict[str, Any]], skipped_plots: list[str]) -> None:
    stable = [row for row in rows if row.get("profile") == "circle_15cm_16s" and not rejected(row)]
    stress = [row for row in rows if row.get("profile") == "gene_15cm_4s" and not rejected(row)]
    rejected_rows = [row for row in rows if rejected(row)]
    parts = [
        "# rbpodo Controller-Simulation Circle Ablation Report",
        "",
        "This report is rbpodo controller-simulation evidence. It connects to real Rainbow controller boxes in pgmode simulation; physical robot motion is not approved.",
        "",
        "## Measurement reliability and caveats",
        "",
        reliability_report.markdown_table(rows) if rows else "_None._",
        "",
        "## Tuning result vs measurement reliability",
        "",
        "This table separates tuning classification, error/timing classification, measurement reliability, and physical-readiness blockers.",
        "",
        decision_split_markdown(rows),
        "",
        "## All Experiments",
        "",
        markdown_table(rows),
        "",
        "## Stable 15cm/16s Candidates",
        "",
        markdown_table(stable) if stable else "_None._",
        "",
        "## GENE-Style 15cm/4s Stress Evidence",
        "",
        markdown_table(stress) if stress else "_None._",
        "",
        "## Rejected Or Safety-Flagged",
        "",
        markdown_table(rejected_rows) if rejected_rows else "_None._",
    ]
    if skipped_plots:
        parts.extend(["", "## Skipped Plots", "", "\n".join(f"- {item}" for item in skipped_plots)])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(parts) + "\n", encoding="utf-8")


def plot_rows(artifact_root: Path, rows: list[dict[str, Any]]) -> list[str]:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        return [f"matplotlib unavailable: {exc}"]
    skipped: list[str] = []
    specs = [
        ("rms_error_by_experiment.png", "rms_error_mm", "RMS error (mm)", True),
        ("p95_error_by_experiment.png", "p95_error_mm", "p95 error (mm)", True),
        ("radius_gain_by_experiment.png", "radius_gain", "Radius gain", True),
        ("latency_by_experiment.png", "estimated_latency_ms", "Estimated latency (ms)", True),
        ("q_ref_update_rate_by_experiment.png", "q_ref_update_rate_hz", "q_ref update rate (Hz)", True),
        ("physical_motion_detected_by_experiment.png", "physical_motion_detected", "Physical motion detected", True),
    ]
    labels = [str(row.get("name")) for row in rows]
    for filename, key, ylabel, required in specs:
        values: list[float | None] = []
        for row in rows:
            if key == "physical_motion_detected":
                value = row.get(key)
                values.append(1.0 if value is True else 0.0 if value is False else None)
            else:
                values.append(finite_number(row.get(key)))
        if not any(value is not None for value in values):
            if required:
                skipped.append(f"{filename} skipped; {key} unavailable")
            continue
        plt.figure(figsize=(max(6.0, 0.5 * len(rows)), 4.0))
        plt.bar(range(len(rows)), [value if value is not None else 0.0 for value in values])
        plt.xticks(range(len(rows)), labels, rotation=45, ha="right")
        plt.ylabel(ylabel)
        plt.tight_layout()
        plt.savefig(artifact_root / filename)
        plt.close()
    return skipped


def write_resolved_matrix(path: Path, args: argparse.Namespace, experiments: list[dict[str, Any]], metadata: list[dict[str, Any]]) -> None:
    value = {
        "schema": SCHEMA,
        "source_matrix": str(root_path(args.root, args.matrix).resolve()),
        "server": str(args.server),
        "max_workers_requested": args.max_workers,
        "max_workers_effective": 1,
        "dry_run": args.dry_run,
        "experiments": [
            {"experiment": exp, "metadata": meta}
            for exp, meta in zip(experiments, metadata)
        ],
    }
    write_json(path, value)


def run_matrix(args: argparse.Namespace) -> dict[str, Any]:
    root = args.root.resolve()
    args.root = root
    experiments_all = load_matrix(root_path(root, args.matrix))
    for index, exp in enumerate(experiments_all, start=1):
        validate_experiment(exp, index)
    enabled_pairs = [
        (index, exp)
        for index, exp in enumerate(experiments_all, start=1)
        if as_bool(exp.get("enabled", True), True)
    ]
    if not enabled_pairs:
        raise AblationError("matrix has no enabled experiments")
    enabled_experiments = [exp for _index, exp in enabled_pairs]
    artifact_root = root_path(root, args.artifact_root).resolve()
    artifact_root.mkdir(parents=True, exist_ok=True)
    metadata = [
        prepare_experiment_config(root, exp, experiment_dir(artifact_root, output_index, exp))
        for output_index, (_matrix_index, exp) in enumerate(enabled_pairs, start=1)
    ]
    validate_matrix_safety(args, metadata, enabled_experiments)
    write_resolved_matrix(artifact_root / "matrix_resolved.json", args, enabled_experiments, metadata)

    summaries: list[dict[str, Any]] = []
    had_errors = False
    for output_index, (_matrix_index, exp) in enumerate(enabled_pairs, start=1):
        meta = metadata[output_index - 1]
        summary = run_experiment(args, exp, meta, output_index, artifact_root)
        summary["_experiment"] = dict(exp)
        summary["_config_metadata"] = dict(meta)
        summaries.append(summary)
        if child_result_failed(summary):
            had_errors = True
            break

    rows = rows_from_summaries(summaries, enabled_experiments[: len(summaries)], metadata[: len(summaries)])
    annotate_phase_advance_effects(rows)
    skipped_plots = ["plots skipped by --dry-run"] if args.dry_run else plot_rows(artifact_root, rows)
    write_csv_rows(artifact_root / "ablation_summary.csv", rows)
    reliability_artifacts = reliability_report.write_artifacts(artifact_root, rows)
    summary = {
        "schema": SCHEMA,
        "matrix": str(root_path(root, args.matrix).resolve()),
        "artifact_root": str(artifact_root),
        "backend": "rbpodo",
        "controller_simulation_only": True,
        "physical_motion_expected": False,
        "dry_run": args.dry_run,
        "max_workers_requested": args.max_workers,
        "max_workers_effective": 1,
        "required_env": list(REQUIRED_ENV),
        "env": ablation_env_snapshot(),
        "rows": rows,
        "measurement_reliability_artifacts": reliability_artifacts,
        "skipped_plots": skipped_plots,
        "had_errors": had_errors,
        "stopped_on_error": had_errors,
    }
    write_json(artifact_root / "ablation_summary.json", summary)
    write_report(artifact_root / "ablation_report.md", rows, skipped_plots)
    return summary


def main() -> int:
    args = parse_args()
    try:
        result = run_matrix(args)
    except Exception as exc:
        print(f"run_rbpodo_circle_ablation: FAIL: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, sort_keys=True))
    return 2 if result.get("had_errors") else 0


if __name__ == "__main__":
    raise SystemExit(main())
