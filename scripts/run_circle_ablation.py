#!/usr/bin/env python3
"""Run matrix-driven simulator-only circle tracking ablations."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import subprocess
import sys
from pathlib import Path
from typing import Any


REAL_ROBOT_IPS = ("172.28.60.200", "172.28.60.201")
REAL_GATE_ENV = ("RB_ALLOW_REAL_ROBOT", "RB_ALLOW_REAL_MOTION", "RB_ALLOW_REAL_CARTESIAN")
PROFILES = {"safe_5cm_10s", "circle_15cm_16s", "circle_15cm_8s", "gene_15cm_4s"}
STRESS_PROFILES = {"circle_15cm_8s", "gene_15cm_4s"}
CONTROLLERS = {"twist_stand", "twist_local", "linear_segments"}
ARMS = {"left", "right"}
EXPERIMENT_KEYS = {
    "name",
    "profile",
    "controller",
    "arm",
    "plane",
    "command_rate_hz",
    "repeat",
    "server_config",
    "left_config",
    "right_config",
    "server_config_overrides",
    "left_config_overrides",
    "right_config_overrides",
    "allow_fast_stress",
    "orientation_mode",
    "warmup_sec",
    "settle_sec",
    "skip_plots",
    "max_allowed_rms_error_m",
    "max_allowed_p95_error_m",
    "max_allowed_orientation_drift_rad",
    "max_allowed_latency_ms",
}
SERVER_OVERRIDE_KEYS = {
    "servo.rate_hz",
    "servo.worker_read_rate_hz",
    "servo.worker_read_period",
    "servo.worker_read_period_sec",
    "network.state_pub_rate_hz",
    "cartesian_control.velocity_target_integration",
    "cartesian_control.velocity_target_lookahead_sec",
    "cartesian_control.path_kp_pos",
    "cartesian_control.path_kp_ori",
    "cartesian_control.twist_orientation_hold_kp",
    "cartesian_control.twist_angular_deadband_rad_s",
    "cartesian_control.velocity_damping",
    "cartesian_control.max_twist_linear_m_s",
    "cartesian_control.max_twist_angular_rad_s",
    "cartesian_control.max_linear_move_speed_m_s",
    "cartesian_control.max_command_actual_error_deg",
}
SIMULATOR_OVERRIDE_KEYS = {
    "simulator.update_rate_hz",
    "simulator.motion_time_constant_sec",
    "simulator.max_joint_velocity_deg_s",
}
SUMMARY_COLUMNS = [
    "name",
    "controller",
    "profile",
    "diameter_m",
    "period_sec",
    "command_rate_hz",
    "servo_rate_hz",
    "velocity_target_integration",
    "path_kp_pos",
    "path_kp_ori",
    "velocity_damping",
    "max_twist_linear_m_s",
    "radius_gain",
    "rms_error_mm",
    "p95_error_mm",
    "max_error_mm",
    "max_orientation_drift_mrad",
    "fit_center_error_mm",
    "estimated_latency_ms",
    "worker_command_drops_total",
    "integrator_clamps_total",
    "integrator_divergence_total",
    "send_command_deadline_missed_count",
    "command_interval_max_ms",
    "servo_jitter_max_ms",
    "result",
]


class AblationError(RuntimeError):
    pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run a simulator-only circle tracking ablation matrix. Each experiment "
            "writes normal circle_tracking_benchmark artifacts plus aggregated CSV, "
            "JSON, markdown, and comparison plots."
        )
    )
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--matrix", type=Path, required=True)
    parser.add_argument("--artifact-root", type=Path, required=True)
    parser.add_argument("--max-workers", type=int, default=1)
    parser.add_argument(
        "--server",
        type=Path,
        default=Path("rb_servo_server/build/hardware_free_gate/rb_servo_server"),
    )
    parser.add_argument("--left-config", type=Path, default=Path("rb_simulator/config/left_rb3_730e.yaml"))
    parser.add_argument("--right-config", type=Path, default=Path("rb_simulator/config/right_rb3_730e.yaml"))
    parser.add_argument("--rbsim-command", default="python3 -m rbsim")
    return parser.parse_args()


def strip_comment(line: str) -> str:
    in_quote: str | None = None
    out: list[str] = []
    for char in line:
        if char in {"'", '"'}:
            in_quote = None if in_quote == char else char if in_quote is None else in_quote
        if char == "#" and in_quote is None:
            break
        out.append(char)
    return "".join(out).rstrip()


def parse_scalar(text: str) -> Any:
    value = text.strip()
    if value == "":
        return ""
    if value[0:1] in {"'", '"'} and value[-1:] == value[0]:
        return value[1:-1]
    lowered = value.lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    if lowered in {"null", "none", "~"}:
        return None
    if value.startswith("[") and value.endswith("]"):
        inner = value[1:-1].strip()
        if not inner:
            return []
        return [parse_scalar(item) for item in inner.split(",")]
    try:
        if re.search(r"[.eE]", value):
            return float(value)
        return int(value)
    except ValueError:
        return value


def parse_key_value(text: str) -> tuple[str, Any]:
    if ":" not in text:
        raise AblationError(f"expected key: value entry, got: {text}")
    key, value = text.split(":", 1)
    key = key.strip()
    if not key:
        raise AblationError(f"empty key in entry: {text}")
    return key, parse_scalar(value)


def parse_matrix_text(text: str) -> list[dict[str, Any]]:
    experiments: list[dict[str, Any]] = []
    current: dict[str, Any] | None = None
    nested_key: str | None = None
    in_experiments = False
    for raw in text.splitlines():
        line = strip_comment(raw)
        if not line.strip():
            continue
        if line.strip() == "experiments:" and not raw.startswith(" "):
            in_experiments = True
            continue
        if not in_experiments:
            raise AblationError("matrix must contain a top-level experiments: list")
        indent = len(raw) - len(raw.lstrip(" "))
        stripped = line.strip()
        if indent == 2 and stripped.startswith("- "):
            current = {}
            experiments.append(current)
            nested_key = None
            rest = stripped[2:].strip()
            if rest:
                key, value = parse_key_value(rest)
                current[key] = value
            continue
        if current is None:
            raise AblationError("matrix entry found before experiments list item")
        if indent == 4:
            if stripped.endswith(":") and ":" not in stripped[:-1]:
                nested_key = stripped[:-1].strip()
                current[nested_key] = {}
                continue
            key, value = parse_key_value(stripped)
            current[key] = value
            nested_key = None
            continue
        if indent == 6 and nested_key:
            key, value = parse_key_value(stripped)
            nested = current.get(nested_key)
            if not isinstance(nested, dict):
                raise AblationError(f"matrix key {nested_key} is not a mapping")
            nested[key] = value
            continue
        raise AblationError(f"unsupported matrix indentation or syntax: {raw}")
    if not experiments:
        raise AblationError("matrix contains no experiments")
    return experiments


def load_matrix(path: Path) -> list[dict[str, Any]]:
    return parse_matrix_text(path.read_text(encoding="utf-8"))


def as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in {"1", "true", "yes", "on"}
    return bool(value)


def finite_number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        number = float(value)
        return number if math.isfinite(number) else None
    return None


def validate_experiment(exp: dict[str, Any], index: int) -> None:
    unknown = sorted(set(exp) - EXPERIMENT_KEYS)
    if unknown:
        raise AblationError(f"experiment {index} has unknown keys: {', '.join(unknown)}")
    for required in ("name", "profile", "controller", "arm"):
        if required not in exp:
            raise AblationError(f"experiment {index} missing required key: {required}")
    if str(exp["profile"]) not in PROFILES:
        raise AblationError(f"experiment {exp['name']} has unsupported profile: {exp['profile']}")
    if str(exp["controller"]) not in CONTROLLERS:
        raise AblationError(f"experiment {exp['name']} has unsupported controller: {exp['controller']}")
    if str(exp["arm"]) not in ARMS:
        raise AblationError(f"experiment {exp['name']} has unsupported arm: {exp['arm']}")
    if exp.get("plane", "xy") not in {"xy", "xz", "yz"}:
        raise AblationError(f"experiment {exp['name']} has unsupported plane: {exp['plane']}")
    if exp["profile"] == "gene_15cm_4s" and not as_bool(exp.get("allow_fast_stress", False)):
        raise AblationError(f"experiment {exp['name']} uses gene_15cm_4s without allow_fast_stress: true")
    for map_key in ("server_config_overrides", "left_config_overrides", "right_config_overrides"):
        if map_key in exp and not isinstance(exp[map_key], dict):
            raise AblationError(f"experiment {exp['name']} key {map_key} must be a mapping")


def validate_real_gate_env() -> None:
    configured = [name for name in REAL_GATE_ENV if os.environ.get(name)]
    if configured:
        raise AblationError("real robot environment gates must not be set: " + ", ".join(configured))


def reject_unsafe_text(text: str, label: str) -> None:
    unsafe: list[str] = []
    for ip in REAL_ROBOT_IPS:
        if ip in text:
            unsafe.append(ip)
    marker_patterns = (
        ("run_mode: real", r"^\s*run_mode\s*:\s*real\s*(?:#.*)?$"),
        ("backend_type: rbpodo", r"^\s*backend_type\s*:\s*rbpodo\s*(?:#.*)?$"),
        ("allow_in_real: true", r"^\s*allow_in_real\s*:\s*true\s*(?:#.*)?$"),
    )
    for label_text, pattern in marker_patterns:
        if re.search(pattern, text, flags=re.MULTILINE):
            unsafe.append(label_text)
    if unsafe:
        raise AblationError(f"{label} is not simulator-only: " + ", ".join(unsafe))


def validate_override_safety(key: str, value: Any) -> None:
    key_lower = key.lower()
    value_text = str(value).lower()
    if key_lower.endswith("run_mode") and value_text == "real":
        raise AblationError(f"overlay would enable real run mode: {key}")
    if key_lower.endswith("backend_type") and value_text == "rbpodo":
        raise AblationError(f"overlay would enable rbpodo backend: {key}")
    if key_lower.endswith("allow_in_real") and as_bool(value):
        raise AblationError(f"overlay would allow real Cartesian motion: {key}")
    if any(ip in str(value) for ip in REAL_ROBOT_IPS):
        raise AblationError(f"overlay contains real robot IP in {key}")


def validate_overrides(overrides: dict[str, Any], allowed: set[str], label: str) -> None:
    for key, value in overrides.items():
        validate_override_safety(key, value)
        if key not in allowed:
            raise AblationError(f"unsupported {label} override: {key}")


def format_yaml_value(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if value is None:
        return "null"
    if isinstance(value, list):
        return "[" + ", ".join(format_yaml_value(item) for item in value) + "]"
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return str(value)
    text = str(value)
    if text == "" or any(char in text for char in ":#[]{}"):
        return json.dumps(text)
    return text


def yaml_line_key(line: str) -> str | None:
    stripped = strip_comment(line).strip()
    if not stripped or stripped.startswith("- ") or ":" not in stripped:
        return None
    return stripped.split(":", 1)[0].strip()


def yaml_indent(line: str) -> int:
    return len(line) - len(line.lstrip(" "))


def section_end(lines: list[str], start: int, indent: int) -> int:
    end = start + 1
    while end < len(lines):
        stripped = lines[end].strip()
        if stripped and yaml_indent(lines[end]) <= indent:
            break
        end += 1
    return end


def find_direct_child(lines: list[str], start: int, end: int, indent: int, key: str) -> int | None:
    for index in range(start, end):
        if lines[index].strip() and yaml_indent(lines[index]) == indent and yaml_line_key(lines[index]) == key:
            return index
    return None


def apply_yaml_override(text: str, dotted_key: str, value: Any) -> str:
    lines = text.splitlines()
    parts = dotted_key.split(".")
    start = 0
    end = len(lines)
    parent_indent = -2
    for part in parts[:-1]:
        child_indent = parent_indent + 2
        found = find_direct_child(lines, start, end, child_indent, part)
        if found is None:
            raise AblationError(f"cannot apply override {dotted_key}; missing section {part}")
        parent_indent = yaml_indent(lines[found])
        start = found + 1
        end = section_end(lines, found, parent_indent)
    leaf = parts[-1]
    leaf_indent = parent_indent + 2
    rendered = " " * leaf_indent + f"{leaf}: {format_yaml_value(value)}"
    found_leaf = find_direct_child(lines, start, end, leaf_indent, leaf)
    if found_leaf is not None:
        lines[found_leaf] = rendered
    else:
        lines.insert(end, rendered)
    return "\n".join(lines) + "\n"


def absolutize_urdf(text: str, source_parent: Path) -> str:
    lines = text.splitlines()
    for index, line in enumerate(lines):
        if yaml_line_key(line) != "urdf":
            continue
        stripped = strip_comment(line).strip()
        if ":" not in stripped:
            continue
        value = parse_scalar(stripped.split(":", 1)[1])
        if not isinstance(value, str) or not value:
            continue
        path = Path(value)
        if not path.is_absolute():
            path = (source_parent / path).resolve()
        lines[index] = " " * yaml_indent(line) + f"urdf: {json.dumps(str(path))}"
    return "\n".join(lines) + "\n"


def read_config(path: Path, label: str) -> str:
    if not path.is_file():
        raise AblationError(f"missing {label}: {path}")
    text = path.read_text(encoding="utf-8")
    reject_unsafe_text(text, str(path))
    return text


def prepare_config(
    *,
    source: Path,
    target: Path,
    overrides: dict[str, Any],
    allowed_overrides: set[str],
    label: str,
) -> Path:
    validate_overrides(overrides, allowed_overrides, label)
    text = read_config(source, label)
    for key, value in overrides.items():
        text = apply_yaml_override(text, key, value)
    if label == "server config":
        text = absolutize_urdf(text, source.parent)
    reject_unsafe_text(text, f"generated {label}")
    target.write_text(text, encoding="utf-8")
    return target


def parse_config_value(text: str, dotted_key: str) -> Any:
    lines = text.splitlines()
    parts = dotted_key.split(".")
    start = 0
    end = len(lines)
    parent_indent = -2
    for part in parts[:-1]:
        found = find_direct_child(lines, start, end, parent_indent + 2, part)
        if found is None:
            return None
        parent_indent = yaml_indent(lines[found])
        start = found + 1
        end = section_end(lines, found, parent_indent)
    found_leaf = find_direct_child(lines, start, end, parent_indent + 2, parts[-1])
    if found_leaf is None:
        return None
    stripped = strip_comment(lines[found_leaf]).strip()
    if ":" not in stripped:
        return None
    return parse_scalar(stripped.split(":", 1)[1])


def safe_name(name: str) -> str:
    sanitized = re.sub(r"[^A-Za-z0-9_.-]+", "_", name.strip())
    return sanitized.strip("._") or "experiment"


def prepare_experiment_configs(args: argparse.Namespace, exp: dict[str, Any], exp_dir: Path) -> dict[str, Path]:
    root = args.root.resolve()
    server_source = root / Path(str(exp.get("server_config", "rb_servo_server/config/dual_simulator_tcp_acceptance.yaml")))
    left_source = root / Path(str(exp.get("left_config", args.left_config)))
    right_source = root / Path(str(exp.get("right_config", args.right_config)))
    return {
        "server_config": prepare_config(
            source=server_source,
            target=exp_dir / "generated_server_config.yaml",
            overrides=dict(exp.get("server_config_overrides", {})),
            allowed_overrides=SERVER_OVERRIDE_KEYS,
            label="server config",
        ),
        "left_config": prepare_config(
            source=left_source,
            target=exp_dir / "generated_left_config.yaml",
            overrides=dict(exp.get("left_config_overrides", {})),
            allowed_overrides=SIMULATOR_OVERRIDE_KEYS,
            label="left simulator config",
        ),
        "right_config": prepare_config(
            source=right_source,
            target=exp_dir / "generated_right_config.yaml",
            overrides=dict(exp.get("right_config_overrides", {})),
            allowed_overrides=SIMULATOR_OVERRIDE_KEYS,
            label="right simulator config",
        ),
    }


def benchmark_command(args: argparse.Namespace, exp: dict[str, Any], configs: dict[str, Path], exp_dir: Path) -> list[str]:
    command = [
        sys.executable,
        "scripts/circle_tracking_benchmark.py",
        "--root",
        str(args.root),
        "--mode",
        "start-local",
        "--server",
        str(args.server),
        "--server-config",
        str(configs["server_config"]),
        "--left-config",
        str(configs["left_config"]),
        "--right-config",
        str(configs["right_config"]),
        "--rbsim-command",
        args.rbsim_command,
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
        "--artifact-dir",
        str(exp_dir),
    ]
    if as_bool(exp.get("allow_fast_stress", False)):
        command.append("--allow-fast-stress")
    for matrix_key, cli_key in (
        ("orientation_mode", "--orientation-mode"),
        ("warmup_sec", "--warmup-sec"),
        ("settle_sec", "--settle-sec"),
        ("max_allowed_rms_error_m", "--max-allowed-rms-error-m"),
        ("max_allowed_p95_error_m", "--max-allowed-p95-error-m"),
        ("max_allowed_orientation_drift_rad", "--max-allowed-orientation-drift-rad"),
        ("max_allowed_latency_ms", "--max-allowed-latency-ms"),
    ):
        if matrix_key in exp:
            command.extend([cli_key, str(exp[matrix_key])])
    if as_bool(exp.get("skip_plots", False)):
        command.append("--skip-plots")
    return command


def subprocess_env(root: Path) -> dict[str, str]:
    env = os.environ.copy()
    for gate in REAL_GATE_ENV:
        env.pop(gate, None)
    sim_src = str((root / "rb_simulator" / "src").resolve())
    env["PYTHONPATH"] = sim_src + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
    return env


def load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise AblationError(f"failed to read {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise AblationError(f"{path} does not contain a JSON object")
    return value


def run_experiment(args: argparse.Namespace, exp: dict[str, Any], index: int, artifact_root: Path) -> dict[str, Any]:
    exp_dir = artifact_root / f"{index:02d}_{safe_name(str(exp['name']))}"
    exp_dir.mkdir(parents=True, exist_ok=True)
    configs = prepare_experiment_configs(args, exp, exp_dir)
    command = benchmark_command(args, exp, configs, exp_dir)
    (exp_dir / "ablation_command.json").write_text(json.dumps(command, indent=2) + "\n", encoding="utf-8")
    completed = subprocess.run(
        command,
        cwd=str(args.root),
        env=subprocess_env(args.root.resolve()),
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
            "result": "error",
            "result_reason": "circle benchmark did not produce summary.json",
            "error": completed.stdout[-4000:],
            "artifact_dir": str(exp_dir),
        }
    if completed.returncode != 0 and summary.get("result") != "error":
        summary["ablation_runner_warning"] = f"circle benchmark exited with {completed.returncode}"
    summary["_experiment"] = dict(exp)
    summary["_experiment_dir"] = str(exp_dir)
    summary["_generated_server_config"] = str(configs["server_config"])
    return summary


def scaled(summary: dict[str, Any], key: str, factor: float) -> float | None:
    value = finite_number(summary.get(key))
    return value * factor if value is not None else None


def command_interval_max_ms(command_packets: Path) -> float | None:
    host_times: list[int] = []
    if not command_packets.is_file():
        return None
    with command_packets.open(encoding="utf-8") as handle:
        for line in handle:
            try:
                packet = json.loads(line)
            except json.JSONDecodeError:
                continue
            mode = (
                packet.get("left", {}).get("mode")
                if isinstance(packet.get("left"), dict)
                else None
            ) or (
                packet.get("right", {}).get("mode")
                if isinstance(packet.get("right"), dict)
                else None
            )
            if mode not in {"TcpTwistStand", "TcpTwistLocal", "TcpLinearMove"}:
                continue
            host_time_ns = packet.get("host_time_ns")
            if isinstance(host_time_ns, int):
                host_times.append(host_time_ns)
    if len(host_times) < 2:
        return None
    return max((b - a) / 1e6 for a, b in zip(host_times, host_times[1:]))


def csv_max(path: Path, field: str) -> float | None:
    if not path.is_file():
        return None
    values: list[float] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            try:
                value = float(row.get(field, ""))
            except ValueError:
                continue
            if math.isfinite(value):
                values.append(value)
    return max(values) if values else None


def row_from_summary(summary: dict[str, Any]) -> dict[str, Any]:
    exp = summary.get("_experiment") if isinstance(summary.get("_experiment"), dict) else {}
    exp_dir = Path(str(summary.get("_experiment_dir", summary.get("artifact_dir", "."))))
    server_config = Path(str(summary.get("_generated_server_config", summary.get("server_config", ""))))
    server_text = server_config.read_text(encoding="utf-8") if server_config.is_file() else ""
    row = {
        "name": exp.get("name") or exp_dir.name,
        "controller": summary.get("controller") or exp.get("controller"),
        "profile": summary.get("profile") or exp.get("profile"),
        "diameter_m": summary.get("diameter_m"),
        "period_sec": summary.get("period_sec"),
        "command_rate_hz": summary.get("command_rate_hz") or exp.get("command_rate_hz"),
        "servo_rate_hz": summary.get("servo_rate_hz") or parse_config_value(server_text, "servo.rate_hz"),
        "velocity_target_integration": parse_config_value(server_text, "cartesian_control.velocity_target_integration"),
        "path_kp_pos": parse_config_value(server_text, "cartesian_control.path_kp_pos"),
        "path_kp_ori": parse_config_value(server_text, "cartesian_control.path_kp_ori"),
        "velocity_damping": parse_config_value(server_text, "cartesian_control.velocity_damping"),
        "max_twist_linear_m_s": summary.get("configured_max_twist_linear_m_s")
        or parse_config_value(server_text, "cartesian_control.max_twist_linear_m_s"),
        "radius_gain": summary.get("radius_gain"),
        "rms_error_mm": scaled(summary, "rms_error_m", 1000.0),
        "p95_error_mm": scaled(summary, "p95_error_m", 1000.0),
        "max_error_mm": scaled(summary, "max_error_m", 1000.0),
        "max_orientation_drift_mrad": scaled(summary, "max_orientation_drift_rad", 1000.0),
        "fit_center_error_mm": scaled(summary, "fit_center_error_m", 1000.0),
        "estimated_latency_ms": summary.get("estimated_latency_ms"),
        "worker_command_drops_total": summary.get("worker_command_drops_total"),
        "integrator_clamps_total": summary.get("integrator_clamps_total"),
        "integrator_divergence_total": summary.get("integrator_divergence_total"),
        "send_command_deadline_missed_count": summary.get("send_command_deadline_missed_count"),
        "command_interval_max_ms": command_interval_max_ms(exp_dir / "command_packets.jsonl"),
        "servo_jitter_max_ms": csv_max(exp_dir / "servo_log.csv", "jitter_ms"),
        "result": summary.get("result"),
        "fault_latched": summary.get("fault_latched"),
    }
    return row


def add_row_warnings(row: dict[str, Any]) -> list[str]:
    warnings: list[str] = []
    radius_gain = finite_number(row.get("radius_gain"))
    if radius_gain is not None and radius_gain < 0.95:
        warnings.append(f"radius_gain {radius_gain:.3f} < 0.95")
    profile = row.get("profile")
    rms_error_mm = finite_number(row.get("rms_error_mm"))
    p95_error_mm = finite_number(row.get("p95_error_mm"))
    if profile == "gene_15cm_4s" and rms_error_mm is not None and rms_error_mm > 10.0:
        warnings.append(f"stress rms_error_mm {rms_error_mm:.3f} > 10")
    if profile == "gene_15cm_4s" and p95_error_mm is not None and p95_error_mm > 20.0:
        warnings.append(f"stress p95_error_mm {p95_error_mm:.3f} > 20")
    if finite_number(row.get("integrator_clamps_total")) not in (None, 0.0):
        warnings.append("integrator_clamps_total > 0")
    drops = finite_number(row.get("worker_command_drops_total"))
    if drops is not None and drops > 0.0:
        warnings.append("worker_command_drops_total > 0")
    missed = finite_number(row.get("send_command_deadline_missed_count"))
    if missed is not None and missed > 0.0:
        warnings.append("send_command_deadline_missed_count > 0")
    if row.get("fault_latched") is True:
        warnings.append("fault_latched true")
    command_rate = finite_number(row.get("command_rate_hz"))
    command_interval = finite_number(row.get("command_interval_max_ms"))
    if command_rate and command_interval is not None and command_interval > 2.0 * (1000.0 / command_rate):
        warnings.append("command_interval_max_ms > 2x nominal interval")
    servo_rate = finite_number(row.get("servo_rate_hz"))
    servo_jitter = finite_number(row.get("servo_jitter_max_ms"))
    if servo_rate and servo_jitter is not None and servo_jitter > 2.0 * (1000.0 / servo_rate):
        warnings.append("servo_jitter_max_ms > 2x servo period")
    return warnings


def rows_from_summaries(summaries: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows = []
    for summary in summaries:
        row = row_from_summary(summary)
        row["warnings"] = "; ".join(add_row_warnings(row))
        rows.append(row)
    return rows


def write_csv_rows(path: Path, rows: list[dict[str, Any]]) -> None:
    fieldnames = SUMMARY_COLUMNS + ["warnings"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in fieldnames})


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def format_cell(value: Any) -> str:
    if value is None:
        return ""
    number = finite_number(value)
    if number is not None:
        if abs(number) >= 100.0:
            return f"{number:.3f}"
        if abs(number) >= 1.0:
            return f"{number:.4f}"
        return f"{number:.6f}"
    return str(value).replace("|", "\\|").replace("\n", " ")


def markdown_table(rows: list[dict[str, Any]]) -> str:
    headers = SUMMARY_COLUMNS + ["warnings"]
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(format_cell(row.get(key)) for key in headers) + " |")
    return "\n".join(lines)


def rejected(row: dict[str, Any]) -> bool:
    if row.get("result") == "error":
        return True
    if row.get("fault_latched") is True:
        return True
    for key in ("worker_command_drops_total", "integrator_clamps_total", "send_command_deadline_missed_count"):
        value = finite_number(row.get(key))
        if value is not None and value > 0.0:
            return True
    warnings = str(row.get("warnings") or "")
    return "jitter" in warnings or "interval" in warnings


def write_report(path: Path, rows: list[dict[str, Any]], skipped_plots: list[str]) -> None:
    stable = [row for row in rows if row.get("profile") not in STRESS_PROFILES and not rejected(row)]
    stress = [row for row in rows if row.get("profile") in STRESS_PROFILES and not rejected(row)]
    rejected_rows = [row for row in rows if rejected(row)]
    parts = [
        "# Circle Tracking Ablation Report",
        "",
        "This report is simulator-only benchmark evidence. It is not real robot readiness.",
        "",
        "## All Experiments",
        "",
        markdown_table(rows),
        "",
        "## Stable Baseline Candidates",
        "",
        markdown_table(stable) if stable else "_None._",
        "",
        "## Stress Candidates",
        "",
        markdown_table(stress) if stress else "_None._",
        "",
        "## Rejected Due To Clamp, Jitter, Drop, Fault, Or Error",
        "",
        markdown_table(rejected_rows) if rejected_rows else "_None._",
    ]
    if skipped_plots:
        parts.extend(["", "## Skipped Plots", "", "\n".join(f"- {item}" for item in skipped_plots)])
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
        ("radius_gain_by_experiment.png", "radius_gain", "Radius gain", True),
        ("p95_error_by_experiment.png", "p95_error_mm", "p95 error (mm)", True),
        ("latency_by_experiment.png", "estimated_latency_ms", "Estimated latency (ms)", True),
        ("jitter_by_experiment.png", "servo_jitter_max_ms", "Servo jitter max (ms)", False),
        ("center_drift_by_experiment.png", "fit_center_error_mm", "Fit center error (mm)", False),
    ]
    labels = [str(row.get("name")) for row in rows]
    for filename, key, ylabel, required in specs:
        values = [finite_number(row.get(key)) for row in rows]
        if not any(value is not None for value in values):
            if not required:
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


def run_matrix(args: argparse.Namespace) -> dict[str, Any]:
    validate_real_gate_env()
    if args.max_workers < 1:
        raise AblationError("--max-workers must be >= 1")
    experiments = load_matrix(args.matrix)
    for index, exp in enumerate(experiments, start=1):
        validate_experiment(exp, index)
    artifact_root = args.artifact_root.resolve()
    artifact_root.mkdir(parents=True, exist_ok=True)
    summaries = []
    for index, exp in enumerate(experiments, start=1):
        summaries.append(run_experiment(args, exp, index, artifact_root))
    rows = rows_from_summaries(summaries)
    skipped_plots = plot_rows(artifact_root, rows)
    write_csv_rows(artifact_root / "ablation_summary.csv", rows)
    write_json(
        artifact_root / "ablation_summary.json",
        {
            "matrix": str(args.matrix.resolve()),
            "artifact_root": str(artifact_root),
            "max_workers_requested": args.max_workers,
            "max_workers_effective": 1,
            "simulator_only": True,
            "real_gate_env_checked": list(REAL_GATE_ENV),
            "rows": rows,
            "skipped_plots": skipped_plots,
        },
    )
    write_report(artifact_root / "ablation_report.md", rows, skipped_plots)
    return {
        "rows": rows,
        "artifact_root": str(artifact_root),
        "skipped_plots": skipped_plots,
        "had_errors": any(row.get("result") == "error" for row in rows),
    }


def main() -> int:
    args = parse_args()
    try:
        result = run_matrix(args)
    except Exception as exc:
        print(f"run_circle_ablation: FAIL: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, sort_keys=True))
    return 2 if result.get("had_errors") else 0


if __name__ == "__main__":
    raise SystemExit(main())
