#!/usr/bin/env python3
"""Validate explicit control-default profile registries."""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any


SCHEMA = "robotics_lab.control_defaults.v1"
CONTROLLER_SIM_PROFILE = "controller_sim_high_performance_gene_26_5"
PHYSICAL_REAL_PROFILE = "physical_real_conservative_seed"
PHYSICAL_WARNING = (
    "The GENE 26.5 / ACKON500 default is a controller-simulation high-performance default only. "
    "It is not the physical-real default until the physical promotion ladder produces actual TCP tracking evidence."
)
REQUIRED_PHYSICAL_PROMOTION_ARTIFACTS = (
    "read_only_diagnostics_parity",
    "stop_resetFault_verified",
    "tiny_joint_motion_pass",
    "tiny_cartesian_motion_pass",
    "slow_physical_circle_pass",
)


class ValidationError(RuntimeError):
    """Raised when a defaults registry is unsafe or inconsistent."""


@dataclass(frozen=True)
class ValidationReport:
    defaults_path: Path
    profile_name: str
    source_matrix: Path
    source_server_config: Path
    parameters: dict[str, Any]
    physical_real_status: str
    physical_promotion_required: list[str]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Validate robotics_lab control-default profiles against their tracked "
            "controller-simulation evidence sources."
        )
    )
    parser.add_argument(
        "--defaults",
        type=Path,
        default=Path("configs/control_defaults/gene_26_5_ackon500_controller_sim.yaml"),
        help="Control-default registry YAML to validate.",
    )
    parser.add_argument(
        "--write-report",
        type=Path,
        help="Optional Markdown report path to write after validation succeeds.",
    )
    return parser.parse_args()


def load_yaml(path: Path) -> dict[str, Any]:
    try:
        import yaml  # type: ignore[import-not-found]
    except ImportError as exc:  # pragma: no cover - depends on developer env
        raise ValidationError("PyYAML is required to validate control defaults") from exc
    try:
        loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise ValidationError(f"failed to read YAML {path}: {exc}") from exc
    if not isinstance(loaded, dict):
        raise ValidationError(f"expected YAML mapping: {path}")
    return loaded


def resolve_source(root: Path, source_path: Any, label: str) -> Path:
    if not isinstance(source_path, str) or not source_path:
        raise ValidationError(f"{label} must be a non-empty path string")
    path = Path(source_path)
    if not path.is_absolute():
        path = root / path
    if not path.is_file():
        raise ValidationError(f"{label} does not exist: {path}")
    return path


def nested(data: dict[str, Any], dotted: str) -> Any:
    value: Any = data
    for key in dotted.split("."):
        if not isinstance(value, dict) or key not in value:
            raise ValidationError(f"missing required field: {dotted}")
        value = value[key]
    return value


def finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValidationError(f"{label} must be numeric")
    number = float(value)
    if not math.isfinite(number):
        raise ValidationError(f"{label} must be finite")
    return number


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if isinstance(expected, float) or isinstance(actual, float):
        actual_number = finite_number(actual, label)
        expected_number = finite_number(expected, label)
        if not math.isclose(actual_number, expected_number, rel_tol=1e-9, abs_tol=1e-12):
            raise ValidationError(f"{label} mismatch: expected {expected!r}, got {actual!r}")
        return
    if actual != expected:
        raise ValidationError(f"{label} mismatch: expected {expected!r}, got {actual!r}")


def first_enabled_experiment(matrix: dict[str, Any]) -> dict[str, Any]:
    experiments = matrix.get("experiments")
    if not isinstance(experiments, list) or not experiments:
        raise ValidationError("source matrix must contain at least one experiment")
    for index, experiment in enumerate(experiments, start=1):
        if not isinstance(experiment, dict):
            raise ValidationError(f"source matrix experiment {index} must be a mapping")
        if experiment.get("enabled", True):
            return experiment
    raise ValidationError("source matrix has no enabled experiments")


def matrix_override(experiment: dict[str, Any], key: str) -> Any:
    overrides = experiment.get("config_overrides")
    if not isinstance(overrides, dict):
        raise ValidationError("source matrix experiment must contain config_overrides")
    if key not in overrides:
        raise ValidationError(f"source matrix missing config_overrides.{key}")
    return overrides[key]


def optional_matrix_override(experiment: dict[str, Any], key: str) -> Any:
    overrides = experiment.get("config_overrides")
    if not isinstance(overrides, dict):
        raise ValidationError("source matrix experiment must contain config_overrides")
    return overrides.get(key)


def assert_both_arms(
    server_config: dict[str, Any],
    experiment: dict[str, Any],
    params: dict[str, Any],
    *,
    param_name: str,
    arm_field: str,
) -> None:
    expected = params[param_name]
    for arm in ("left_robot", "right_robot"):
        assert_equal(nested(server_config, f"{arm}.{arm_field}"), expected, f"server_config {arm}.{arm_field}")
        assert_equal(matrix_override(experiment, f"{arm}.{arm_field}"), expected, f"matrix override {arm}.{arm_field}")


def validate_controller_sim_profile(
    *,
    root: Path,
    defaults_path: Path,
    profile: dict[str, Any],
) -> ValidationReport:
    source = profile.get("source")
    if not isinstance(source, dict):
        raise ValidationError(f"{CONTROLLER_SIM_PROFILE}.source must be a mapping")
    matrix_path = resolve_source(root, source.get("matrix"), "source.matrix")
    server_config_path = resolve_source(root, source.get("server_config"), "source.server_config")
    assert_equal(source.get("evidence_lane"), "rbpodo_server_side_circle_ackon500_sdk_worker", "source.evidence_lane")
    assert_equal(source.get("tracking_source"), "tcp_ref_stand", "source.tracking_source")
    if profile.get("allowed_use") != ["rbpodo_controller_pgmode_simulation"]:
        raise ValidationError(
            f"{CONTROLLER_SIM_PROFILE}.allowed_use must be exactly rbpodo_controller_pgmode_simulation"
        )
    forbidden = profile.get("forbidden_use", [])
    for forbidden_use in ("physical_real_default", "real_cartesian_default_without_acceptance"):
        if forbidden_use not in forbidden:
            raise ValidationError(f"{CONTROLLER_SIM_PROFILE}.forbidden_use must include {forbidden_use}")
    physical_readiness = profile.get("physical_readiness")
    if not isinstance(physical_readiness, dict) or physical_readiness.get("status") != "blocked":
        raise ValidationError(f"{CONTROLLER_SIM_PROFILE}.physical_readiness.status must be blocked")
    params = profile.get("parameters")
    if not isinstance(params, dict):
        raise ValidationError(f"{CONTROLLER_SIM_PROFILE}.parameters must be a mapping")

    matrix = load_yaml(matrix_path)
    server_config = load_yaml(server_config_path)
    experiment = first_enabled_experiment(matrix)
    assert_equal(experiment.get("config"), str(Path(source["server_config"])), "matrix experiment config")
    assert_equal(experiment.get("tracking_source"), source["tracking_source"], "matrix experiment tracking_source")
    assert_equal(experiment.get("phase_advance_sec"), params.get("phase_advance_sec"), "matrix experiment phase_advance_sec")

    for arm in ("left_robot", "right_robot"):
        assert_equal(nested(server_config, f"{arm}.backend_type"), "rbpodo", f"server_config {arm}.backend_type")
        assert_equal(nested(server_config, f"{arm}.run_mode"), "real", f"server_config {arm}.run_mode")
        assert_equal(nested(server_config, f"{arm}.operation_mode"), "simulation", f"server_config {arm}.operation_mode")
        matrix_operation_mode = optional_matrix_override(experiment, f"{arm}.operation_mode")
        if matrix_operation_mode is not None:
            assert_equal(matrix_operation_mode, "simulation", f"matrix override {arm}.operation_mode")

    assert_equal(nested(server_config, "cartesian_control.allow_in_real"), False, "server_config cartesian_control.allow_in_real")
    assert_equal(
        matrix_override(experiment, "cartesian_control.allow_in_real"),
        False,
        "matrix override cartesian_control.allow_in_real",
    )
    assert_equal(
        nested(server_config, "cartesian_control.allow_in_controller_simulation"),
        True,
        "server_config cartesian_control.allow_in_controller_simulation",
    )
    assert_equal(
        matrix_override(experiment, "cartesian_control.allow_in_controller_simulation"),
        True,
        "matrix override cartesian_control.allow_in_controller_simulation",
    )

    assert_equal(nested(server_config, "servo.rate_hz"), params.get("servo_rate_hz"), "server_config servo.rate_hz")
    assert_equal(matrix_override(experiment, "servo.rate_hz"), params.get("servo_rate_hz"), "matrix override servo.rate_hz")
    assert_equal(
        nested(server_config, "servo.rbpodo_async_streaming.enable"),
        True,
        "server_config servo.rbpodo_async_streaming.enable",
    )
    assert_equal(
        matrix_override(experiment, "servo.rbpodo_async_streaming.enable"),
        True,
        "matrix override servo.rbpodo_async_streaming.enable",
    )
    assert_equal(
        nested(server_config, "servo.rbpodo_async_streaming.mode"),
        params.get("async_mode"),
        "server_config servo.rbpodo_async_streaming.mode",
    )
    assert_equal(
        matrix_override(experiment, "servo.rbpodo_async_streaming.mode"),
        params.get("async_mode"),
        "matrix override servo.rbpodo_async_streaming.mode",
    )
    assert_equal(
        nested(server_config, "servo.rbpodo_async_streaming.rate_hz"),
        params.get("servo_rate_hz"),
        "server_config servo.rbpodo_async_streaming.rate_hz",
    )
    assert_equal(
        matrix_override(experiment, "servo.rbpodo_async_streaming.rate_hz"),
        params.get("servo_rate_hz"),
        "matrix override servo.rbpodo_async_streaming.rate_hz",
    )

    assert_both_arms(server_config, experiment, params, param_name="speed_bar", arm_field="speed_bar")
    assert_both_arms(server_config, experiment, params, param_name="servo_t1_sec", arm_field="servo_t1_sec")
    assert_both_arms(server_config, experiment, params, param_name="servo_t2_sec", arm_field="servo_t2_sec")
    assert_both_arms(server_config, experiment, params, param_name="servo_alpha", arm_field="servo_alpha")
    assert_both_arms(
        server_config,
        experiment,
        params,
        param_name="disable_waiting_ack",
        arm_field="disable_waiting_ack",
    )
    assert_both_arms(server_config, experiment, params, param_name="command_timeout_sec", arm_field="command_timeout_sec")

    scalar_fields = {
        "path_kp_pos": "cartesian_control.path_kp_pos",
        "path_kp_ori": "cartesian_control.path_kp_ori",
        "max_twist_linear_m_s": "cartesian_control.max_twist_linear_m_s",
        "max_twist_angular_rad_s": "cartesian_control.max_twist_angular_rad_s",
        "max_linear_move_speed_m_s": "cartesian_control.max_linear_move_speed_m_s",
        "velocity_target_integration": "cartesian_control.velocity_target_integration",
        "state_pub_rate_hz": "network.state_pub_rate_hz",
    }
    for param_name, config_field in scalar_fields.items():
        assert_equal(nested(server_config, config_field), params.get(param_name), f"server_config {config_field}")
        assert_equal(matrix_override(experiment, config_field), params.get(param_name), f"matrix override {config_field}")

    if params.get("disable_waiting_ack") is not False:
        raise ValidationError(f"{CONTROLLER_SIM_PROFILE}.parameters.disable_waiting_ack must be false")
    servo_rate_hz = finite_number(params.get("servo_rate_hz"), f"{CONTROLLER_SIM_PROFILE}.parameters.servo_rate_hz")
    servo_t1_sec = finite_number(params.get("servo_t1_sec"), f"{CONTROLLER_SIM_PROFILE}.parameters.servo_t1_sec")
    if not math.isclose(servo_t1_sec, 1.0 / servo_rate_hz, rel_tol=1e-9, abs_tol=1e-12):
        raise ValidationError(f"{CONTROLLER_SIM_PROFILE}.parameters.servo_t1_sec must match servo_rate_hz period")

    return ValidationReport(
        defaults_path=defaults_path,
        profile_name=CONTROLLER_SIM_PROFILE,
        source_matrix=matrix_path,
        source_server_config=server_config_path,
        parameters=dict(params),
        physical_real_status="unknown",
        physical_promotion_required=[],
    )


def validate_physical_profile(profile: dict[str, Any]) -> tuple[str, list[str]]:
    source = profile.get("source")
    if not isinstance(source, dict):
        raise ValidationError(f"{PHYSICAL_REAL_PROFILE}.source must be a mapping")
    assert_equal(source.get("derived_from"), CONTROLLER_SIM_PROFILE, f"{PHYSICAL_REAL_PROFILE}.source.derived_from")
    if profile.get("allowed_use") != []:
        raise ValidationError(f"{PHYSICAL_REAL_PROFILE}.allowed_use must be empty until promotion")
    promotion_required = profile.get("promotion_required")
    if not isinstance(promotion_required, list):
        raise ValidationError(f"{PHYSICAL_REAL_PROFILE}.promotion_required must be a list")
    for item in REQUIRED_PHYSICAL_PROMOTION_ARTIFACTS:
        if item not in promotion_required:
            raise ValidationError(f"{PHYSICAL_REAL_PROFILE}.promotion_required missing {item}")
    status = profile.get("status")
    if status == "promoted":
        evidence = profile.get("promotion_evidence")
        if not isinstance(evidence, dict):
            raise ValidationError(f"{PHYSICAL_REAL_PROFILE} cannot be promoted without promotion_evidence")
        artifacts = evidence.get("artifact_references")
        if not isinstance(artifacts, dict):
            raise ValidationError(f"{PHYSICAL_REAL_PROFILE} cannot be promoted without artifact_references")
        for item in REQUIRED_PHYSICAL_PROMOTION_ARTIFACTS:
            artifact = artifacts.get(item)
            if not isinstance(artifact, str) or not artifact:
                raise ValidationError(f"{PHYSICAL_REAL_PROFILE} promoted without artifact reference for {item}")
    elif status != "not_promoted":
        raise ValidationError(f"{PHYSICAL_REAL_PROFILE}.status must be not_promoted or promoted")
    return str(status), [str(item) for item in promotion_required]


def validate_defaults(defaults_path: Path, *, root: Path | None = None) -> ValidationReport:
    root = root or Path.cwd()
    defaults_path = defaults_path if defaults_path.is_absolute() else root / defaults_path
    defaults = load_yaml(defaults_path)
    assert_equal(defaults.get("schema"), SCHEMA, "schema")
    profiles = defaults.get("profiles")
    if not isinstance(profiles, dict):
        raise ValidationError("profiles must be a mapping")
    controller_profile = profiles.get(CONTROLLER_SIM_PROFILE)
    if not isinstance(controller_profile, dict):
        raise ValidationError(f"missing profile {CONTROLLER_SIM_PROFILE}")
    physical_profile = profiles.get(PHYSICAL_REAL_PROFILE)
    if not isinstance(physical_profile, dict):
        raise ValidationError(f"missing profile {PHYSICAL_REAL_PROFILE}")
    report = validate_controller_sim_profile(
        root=root,
        defaults_path=defaults_path,
        profile=controller_profile,
    )
    physical_status, promotion_required = validate_physical_profile(physical_profile)
    return ValidationReport(
        defaults_path=report.defaults_path,
        profile_name=report.profile_name,
        source_matrix=report.source_matrix,
        source_server_config=report.source_server_config,
        parameters=report.parameters,
        physical_real_status=physical_status,
        physical_promotion_required=promotion_required,
    )


def markdown_table(rows: list[tuple[str, Any]]) -> str:
    lines = ["| Parameter | Value |", "| --- | --- |"]
    for key, value in rows:
        lines.append(f"| `{key}` | `{value}` |")
    return "\n".join(lines)


def report_markdown(report: ValidationReport) -> str:
    rows = [(key, report.parameters[key]) for key in sorted(report.parameters)]
    promotion_rows = "\n".join(f"- `{item}`" for item in report.physical_promotion_required)
    return "\n".join(
        [
            "# GENE 26.5 / ACKON500 Control Defaults",
            "",
            PHYSICAL_WARNING,
            "",
            "## Controller-Simulation Default",
            "",
            f"- Profile: `{report.profile_name}`",
            f"- Source matrix: `{report.source_matrix}`",
            f"- Source server config: `{report.source_server_config}`",
            "- Allowed use: `rbpodo_controller_pgmode_simulation`",
            "- Forbidden use: `physical_real_default`, `real_cartesian_default_without_acceptance`",
            "",
            markdown_table(rows),
            "",
            "## Physical Real Tier",
            "",
            f"- Profile: `{PHYSICAL_REAL_PROFILE}`",
            f"- Status: `{report.physical_real_status}`",
            "- Physical real remains blocked until all promotion artifacts exist.",
            "",
            promotion_rows,
            "",
        ]
    )


def main() -> int:
    args = parse_args()
    try:
        report = validate_defaults(args.defaults)
        if args.write_report:
            args.write_report.parent.mkdir(parents=True, exist_ok=True)
            args.write_report.write_text(report_markdown(report), encoding="utf-8")
        print(
            f"validated {report.profile_name}: physical real status={report.physical_real_status}",
            file=sys.stdout,
        )
        return 0
    except ValidationError as exc:
        print(f"control defaults validation failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
