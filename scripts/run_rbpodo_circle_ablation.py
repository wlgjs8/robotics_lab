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
)
OPTIONAL_ENV_REQUIREMENTS = {
    "RB_ALLOW_RBPODO_ACK_DISABLED_MOTION",
    "RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM",
}
PROFILES = set(circle_bench.PROFILE_DEFAULTS)
CONTROLLERS = {
    "twist_stand",
    "twist_local",
    "twist_stand_feedback",
    "twist_local_feedback",
}
TRACKING_SOURCES = set(circle_bench.TRACKING_SOURCES)
ARMS = {"left", "right"}
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
}
SUMMARY_COLUMNS = [
    "name",
    "controller",
    "profile",
    "ack_policy",
    "servo_rate_hz",
    "command_rate_hz",
    "tracking_source",
    "radius_gain",
    "rms_error_mm",
    "p95_error_mm",
    "max_error_mm",
    "p95_orientation_drift_mrad",
    "estimated_latency_ms",
    "q_ref_update_rate_hz",
    "send_duration_p95_us",
    "ack_observed_count",
    "controller_acceptance_observed_count",
    "diagnostics_suspect_count",
    "physical_motion_detected",
    "result",
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


def scaled(summary: dict[str, Any], key: str, factor: float) -> float | None:
    value = finite_number(summary.get(key))
    return value * factor if value is not None else None


def nested_metric(summary: dict[str, Any], key: str, metric: str) -> float | None:
    value = summary.get(key)
    if not isinstance(value, dict):
        return None
    return finite_number(value.get(metric))


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
            if value is None or (key in {"warmup_sec", "settle_sec"} and value < 0.0) or (
                key not in {"warmup_sec", "settle_sec"} and value <= 0.0
            ):
                raise AblationError(f"experiment {exp['name']} has invalid {key}: {exp[key]}")
    if "repeat" in exp:
        repeat = finite_number(exp["repeat"])
        if repeat is None or int(repeat) < 1:
            raise AblationError(f"experiment {exp['name']} has invalid repeat: {exp['repeat']}")
    for env_name in list_value(exp.get("env_requirements")):
        if env_name == "RB_ALLOW_REAL_CARTESIAN":
            raise AblationError(f"experiment {exp['name']} may not require RB_ALLOW_REAL_CARTESIAN")
        if env_name not in set(REQUIRED_ENV) | OPTIONAL_ENV_REQUIREMENTS:
            raise AblationError(f"experiment {exp['name']} has unsupported env requirement: {env_name}")


def selected_arm(config: Any, arm: str) -> Any:
    return config.left if arm == "left" else config.right


def validate_config(root: Path, exp: dict[str, Any]) -> dict[str, Any]:
    config_path = root_path(root, exp["config"]).resolve()
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

    arm_cfg = selected_arm(config, str(exp["arm"]))
    servo_rate_hz = as_float(config.servo.get("rate_hz"))
    servo_t1_sec = arm_cfg.servo_t1_sec
    t1_aligned = None
    alignment_warning = ""
    if servo_rate_hz and servo_t1_sec:
        t1_aligned = abs(servo_t1_sec - (1.0 / servo_rate_hz)) <= 1e-6
        if not t1_aligned:
            alignment_warning = (
                f"servo_t1_sec {servo_t1_sec:.6f} does not match 1/rate_hz "
                f"{1.0 / servo_rate_hz:.6f}"
            )
    return {
        "config_path": str(config_path),
        "configured_ips": [config.left.ip, config.right.ip],
        "known_real_ips": sorted({config.left.ip, config.right.ip} & REAL_ROBOT_IPS),
        "ack_policy": "ack_off" if config.left.disable_waiting_ack else "ack_on",
        "disable_waiting_ack": bool(config.left.disable_waiting_ack),
        "servo_rate_hz": servo_rate_hz,
        "servo_t1_sec": servo_t1_sec,
        "servo_t1_rate_aligned": t1_aligned,
        "alignment_warning": alignment_warning,
        "allow_controller_simulation_diagnostics_suspect": as_bool(
            config.servo.get("allow_controller_simulation_diagnostics_suspect"), False
        ),
        "command_bind": config.network.get("command_bind"),
        "state_pub_endpoint": config.network.get("state_pub_endpoint"),
    }


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
    for matrix_key, cli_key in (
        ("feedback_kp_pos", "--feedback-kp-pos"),
        ("feedback_kp_ori", "--feedback-kp-ori"),
        ("feedback_max_linear_m_s", "--feedback-max-linear-m-s"),
        ("feedback_max_angular_rad_s", "--feedback-max-angular-rad-s"),
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
    exp_dir = artifact_root / f"{index:02d}_{sim_ablation.safe_name(str(exp['name']))}"
    exp_dir.mkdir(parents=True, exist_ok=True)
    command = benchmark_command(args, exp, meta, exp_dir)
    command_text = shlex.join(command)
    (exp_dir / "experiment_command.txt").write_text(command_text + "\n", encoding="utf-8")
    write_json(exp_dir / "ablation_command.json", command)
    if args.dry_run:
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
            "tracking_source_used": exp.get("tracking_source", "auto"),
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
    return {
        "name": exp.get("name"),
        "controller": summary.get("controller") or exp.get("controller"),
        "profile": summary.get("profile") or exp.get("profile"),
        "ack_policy": meta.get("ack_policy"),
        "servo_rate_hz": summary.get("servo_rate_hz") or meta.get("servo_rate_hz"),
        "command_rate_hz": summary.get("command_rate_hz") or exp.get("command_rate_hz"),
        "tracking_source": summary.get("tracking_source_used") or exp.get("tracking_source", "auto"),
        "radius_gain": summary.get("radius_gain"),
        "rms_error_mm": scaled(summary, "rms_error_m", 1000.0),
        "p95_error_mm": scaled(summary, "p95_error_m", 1000.0),
        "max_error_mm": scaled(summary, "max_error_m", 1000.0),
        "p95_orientation_drift_mrad": scaled(summary, "p95_orientation_drift_rad", 1000.0),
        "estimated_latency_ms": summary.get("estimated_latency_ms"),
        "q_ref_update_rate_hz": summary.get("q_ref_update_rate_hz"),
        "send_duration_p95_us": nested_metric(summary, "send_duration_us", "p95"),
        "ack_observed_count": summary.get("ack_observed_count"),
        "controller_acceptance_observed_count": summary.get("controller_acceptance_observed_count"),
        "diagnostics_suspect_count": summary.get("diagnostics_suspect_count"),
        "physical_motion_detected": summary.get("physical_motion_detected"),
        "result": summary.get("result"),
        "artifact_dir": summary.get("artifact_dir"),
        "warnings": warning_text(summary, meta),
    }


def warning_text(summary: dict[str, Any], meta: dict[str, Any]) -> str:
    warnings: list[str] = []
    if meta.get("alignment_warning"):
        warnings.append(str(meta["alignment_warning"]))
    value = summary.get("performance_warnings")
    if isinstance(value, list):
        warnings.extend(str(item) for item in value)
    elif isinstance(value, str) and value:
        warnings.append(value)
    if summary.get("physical_motion_detected") is True:
        warnings.append("physical_motion_detected true in pgmode simulation")
    if summary.get("result") == "error":
        reason = summary.get("result_reason") or summary.get("error") or "error"
        warnings.append(str(reason))
    if meta.get("ack_policy") == "ack_off":
        warnings.append("ACK-off controller-simulation evidence is experimental")
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
    if row.get("result") == "error":
        return True
    if row.get("physical_motion_detected") is True:
        return True
    if finite_number(row.get("diagnostics_suspect_count")) not in (None, 0.0):
        return True
    return False


def write_report(path: Path, rows: list[dict[str, Any]], skipped_plots: list[str]) -> None:
    stable = [row for row in rows if row.get("profile") == "circle_15cm_16s" and not rejected(row)]
    stress = [row for row in rows if row.get("profile") == "gene_15cm_4s" and not rejected(row)]
    rejected_rows = [row for row in rows if rejected(row)]
    parts = [
        "# rbpodo Controller-Simulation Circle Ablation Report",
        "",
        "This report is rbpodo controller-simulation evidence. It connects to real Rainbow controller boxes in pgmode simulation; physical robot motion is not approved.",
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
    metadata = [validate_config(root, exp) for exp in enabled_experiments]
    validate_matrix_safety(args, metadata, enabled_experiments)

    artifact_root = root_path(root, args.artifact_root).resolve()
    artifact_root.mkdir(parents=True, exist_ok=True)
    write_resolved_matrix(artifact_root / "matrix_resolved.json", args, enabled_experiments, metadata)

    summaries: list[dict[str, Any]] = []
    had_errors = False
    for output_index, (_matrix_index, exp) in enumerate(enabled_pairs, start=1):
        meta = metadata[output_index - 1]
        summary = run_experiment(args, exp, meta, output_index, artifact_root)
        summary["_experiment"] = dict(exp)
        summary["_config_metadata"] = dict(meta)
        summaries.append(summary)
        if summary.get("result") == "error":
            had_errors = True
            break

    rows = rows_from_summaries(summaries, enabled_experiments[: len(summaries)], metadata[: len(summaries)])
    skipped_plots = ["plots skipped by --dry-run"] if args.dry_run else plot_rows(artifact_root, rows)
    write_csv_rows(artifact_root / "ablation_summary.csv", rows)
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
        "env": env_snapshot(),
        "rows": rows,
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
