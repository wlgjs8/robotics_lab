#!/usr/bin/env python3
"""Run a gated rbpodo vs rbscript_tcp backend comparison matrix."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import shlex
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import compare_backend_ablation as compare_backend


ROOT = Path(__file__).resolve().parents[1]
REAL_ROBOT_IPS = {"172.28.60.200", "172.28.60.201"}
ALLOWED_BACKENDS = {"rbpodo", "rbscript_tcp"}
ALLOWED_SCRIPTS = {
    "rb_backend_ablation",
    "rainbow_rate_probe",
    "rbpodo_servo_acceptance",
    "rbscript_servo_acceptance",
}
NO_MOTION_MODES = {"connect_only", "read_state", "command_ack_no_motion", "ack_no_motion"}
SERVO_NOOP_MODES = {"servo_j_noop", "servo_j_noop_controller_simulation"}
SUMMARY_FIELDS = [
    "experiment",
    "status",
    "reason",
    "backend",
    "script",
    "mode",
    "profile",
    "artifact_dir",
    "requested_rate_hz",
    "achieved_rate_hz",
    "persistent_socket",
    "reconnect_count",
    "read_state_capability",
    "comparable",
    "success_rate",
    "p50_ack_us",
    "p95_ack_us",
    "p99_ack_us",
    "timeout_count",
    "error_count",
    "send_count",
    "send_success_count",
    "send_failure_count",
    "ack_observed_count",
    "controller_acceptance_observed_count",
    "not_comparable_reason",
]


class MatrixError(RuntimeError):
    pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run a unified rbpodo vs rbscript_tcp comparison matrix. "
            "Default execution is read-only/no-motion; ServoJ no-op controller "
            "simulation requires an explicit matrix flag and child-script gates."
        )
    )
    parser.add_argument("--matrix", type=Path, required=True)
    parser.add_argument("--artifact-root", type=Path, required=True)
    parser.add_argument("--max-workers", type=int, default=1)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--include-disabled", action="store_true")
    parser.add_argument(
        "--allow-servo-j-noop-simulation",
        action="store_true",
        help="Allow enabled ServoJ no-op controller-simulation matrix entries to run child preflight gates.",
    )
    parser.add_argument(
        "--i-understand-this-connects-to-real-controller",
        action="store_true",
        help="Required when matrix controller IPs include known RB controller addresses.",
    )
    parser.add_argument("--skip-plots", action="store_true")
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


def split_inline_list(text: str) -> list[str]:
    items: list[str] = []
    current = []
    in_quote = False
    quote_char = ""
    for char in text:
        if char in {"'", '"'}:
            if not in_quote:
                in_quote = True
                quote_char = char
            elif quote_char == char:
                in_quote = False
        if char == "," and not in_quote:
            item = "".join(current).strip()
            if item:
                items.append(item)
            current = []
            continue
        current.append(char)
    item = "".join(current).strip()
    if item:
        items.append(item)
    return items


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
        return [scalar_value(item) for item in split_inline_list(value[1:-1])]
    try:
        if any(ch in value for ch in ".eE"):
            number = float(value)
            return number if math.isfinite(number) else value
        return int(value, 10)
    except ValueError:
        return value


def parse_simple_matrix_yaml(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise MatrixError(f"matrix file not found: {path}")
    data: dict[str, Any] = {"controllers": {}, "experiments": []}
    section: str | None = None
    current_experiment: dict[str, Any] | None = None
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = strip_comment(raw_line).rstrip()
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip(" "))
        text = line.strip()
        if indent == 0:
            if not text.endswith(":"):
                raise MatrixError(f"unsupported top-level matrix line: {raw_line}")
            section = text[:-1]
            if section not in {"controllers", "experiments"}:
                raise MatrixError(f"unsupported matrix section: {section}")
            current_experiment = None
            continue
        if section == "controllers":
            if indent != 2 or ":" not in text:
                raise MatrixError(f"unsupported controllers line: {raw_line}")
            key, value = text.split(":", 1)
            data["controllers"][key.strip()] = scalar_value(value)
            continue
        if section == "experiments":
            if indent == 2 and text.startswith("- "):
                current_experiment = {}
                data["experiments"].append(current_experiment)
                item = text[2:].strip()
                if item:
                    if ":" not in item:
                        raise MatrixError(f"unsupported experiment item: {raw_line}")
                    key, value = item.split(":", 1)
                    current_experiment[key.strip()] = scalar_value(value)
                continue
            if indent == 4 and current_experiment is not None and ":" in text:
                key, value = text.split(":", 1)
                current_experiment[key.strip()] = scalar_value(value)
                continue
            raise MatrixError(f"unsupported experiments line: {raw_line}")
        raise MatrixError(f"matrix key outside supported section: {raw_line}")
    return data


def format_yaml_value(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if value is None:
        return "null"
    if isinstance(value, (int, float)):
        return str(value)
    if isinstance(value, list):
        return "[" + ", ".join(format_yaml_value(item) for item in value) + "]"
    text = str(value)
    if not text or any(ch in text for ch in " #:[]{}"):
        return json.dumps(text)
    return text


def dump_matrix_yaml(matrix: dict[str, Any]) -> str:
    lines = ["controllers:"]
    for key, value in matrix.get("controllers", {}).items():
        lines.append(f"  {key}: {format_yaml_value(value)}")
    lines.append("")
    lines.append("experiments:")
    for exp in matrix.get("experiments", []):
        lines.append(f"  - name: {format_yaml_value(exp.get('name', ''))}")
        for key, value in exp.items():
            if key == "name":
                continue
            lines.append(f"    {key}: {format_yaml_value(value)}")
    lines.append("")
    return "\n".join(lines)


def load_matrix(path: Path) -> dict[str, Any]:
    try:
        import yaml  # type: ignore
    except Exception:
        return parse_simple_matrix_yaml(path)
    with path.open(encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise MatrixError("matrix must be a YAML object")
    return data


def bool_value(value: Any, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in {"1", "true", "yes", "on"}
    return default


def number_value(value: Any, default: float | None = None) -> float | None:
    if value is None or isinstance(value, bool):
        return default
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    return out if math.isfinite(out) else default


def list_value(value: Any) -> list[Any]:
    if isinstance(value, list):
        return value
    if value is None:
        return []
    return [value]


def slug(value: str) -> str:
    out = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip())
    return out.strip("._-") or "experiment"


def selected_ip(controllers: dict[str, Any]) -> str:
    arm = str(controllers.get("arm", "left"))
    if arm == "right":
        return str(controllers.get("right_ip", ""))
    return str(controllers.get("left_ip", ""))


def normalize_mode(exp: dict[str, Any]) -> str:
    mode = str(exp.get("mode", ""))
    if mode == "ack_no_motion":
        return "command_ack_no_motion"
    if mode == "servo_j_noop_controller_simulation":
        return "servo_j_noop"
    return mode


def validate_matrix(matrix: dict[str, Any]) -> None:
    controllers = matrix.get("controllers")
    experiments = matrix.get("experiments")
    if not isinstance(controllers, dict):
        raise MatrixError("matrix.controllers must be a mapping")
    if not isinstance(experiments, list):
        raise MatrixError("matrix.experiments must be a list")
    if controllers.get("arm") not in {"left", "right"}:
        raise MatrixError("controllers.arm must be left or right")
    for key in ("left_ip", "right_ip"):
        if not controllers.get(key):
            raise MatrixError(f"controllers.{key} is required")
    names: set[str] = set()
    for index, exp in enumerate(experiments):
        if not isinstance(exp, dict):
            raise MatrixError(f"experiment {index} must be a mapping")
        name = str(exp.get("name", "")).strip()
        if not name:
            raise MatrixError(f"experiment {index} missing name")
        if name in names:
            raise MatrixError(f"duplicate experiment name: {name}")
        names.add(name)
        backend = str(exp.get("backend", ""))
        script = str(exp.get("script", ""))
        mode = normalize_mode(exp)
        if backend not in ALLOWED_BACKENDS:
            raise MatrixError(f"experiment {name} has unsupported backend: {backend}")
        if script not in ALLOWED_SCRIPTS:
            raise MatrixError(f"experiment {name} has unsupported script: {script}")
        if mode not in NO_MOTION_MODES and mode not in SERVO_NOOP_MODES:
            raise MatrixError(f"experiment {name} has unsupported mode: {mode}")


def script_path(script: str) -> Path:
    return ROOT / "scripts" / f"{script}.py"


def unsupported_reason(exp: dict[str, Any]) -> str:
    backend = str(exp.get("backend"))
    script = str(exp.get("script"))
    mode = normalize_mode(exp)
    if script == "rainbow_rate_probe":
        if mode == "connect_only":
            return "rainbow_rate_probe does not implement connect_only; use rb_backend_ablation"
        if mode == "command_ack_no_motion" and backend != "rbscript_tcp":
            return "rainbow_rate_probe ack_no_motion is implemented only for rbscript_tcp"
        if mode in SERVO_NOOP_MODES:
            return "rainbow_rate_probe does not implement ServoJ no-op acceptance"
    if script == "rb_backend_ablation":
        if mode in SERVO_NOOP_MODES:
            return "rb_backend_ablation does not implement ServoJ no-op acceptance"
        if mode == "command_ack_no_motion" and backend != "rbscript_tcp":
            return "rb_backend_ablation command_ack_no_motion is implemented only for rbscript_tcp"
    if script == "rbpodo_servo_acceptance":
        if backend != "rbpodo":
            return "rbpodo_servo_acceptance supports only rbpodo"
        if mode not in {"read_only", "servo_j_noop"} and mode not in SERVO_NOOP_MODES:
            return "rbpodo_servo_acceptance supports read_only and servo_j_noop"
    if script == "rbscript_servo_acceptance":
        if backend != "rbscript_tcp":
            return "rbscript_servo_acceptance supports only rbscript_tcp"
        if mode not in {"read_only", "servo_j_noop"} and mode not in SERVO_NOOP_MODES:
            return "rbscript_servo_acceptance supports read_only and servo_j_noop"
    return ""


def rates_arg(exp: dict[str, Any]) -> str:
    rates = list_value(exp.get("rates"))
    if not rates:
        rate = exp.get("rate_hz")
        return str(rate if rate is not None else 100)
    return ",".join(str(rate) for rate in rates)


def single_rate(exp: dict[str, Any]) -> str:
    rates = list_value(exp.get("rates"))
    if len(rates) > 1:
        raise MatrixError(f"experiment {exp.get('name')} uses multiple rates with rb_backend_ablation; use rainbow_rate_probe")
    if len(rates) == 1:
        return str(rates[0])
    return str(exp.get("rate_hz", 100))


def command_for_experiment(exp: dict[str, Any], controllers: dict[str, Any], artifact_dir: Path, args: argparse.Namespace) -> list[str]:
    script = str(exp["script"])
    backend = str(exp["backend"])
    mode = normalize_mode(exp)
    command = [sys.executable, str(script_path(script))]
    duration = str(exp.get("duration_sec", 10))
    arm = str(exp.get("arm", controllers.get("arm", "left")))

    if script == "rainbow_rate_probe":
        child_mode = "ack_no_motion" if mode == "command_ack_no_motion" else mode
        command.extend([
            "--ip", selected_ip(controllers),
            "--backend", backend,
            "--mode", child_mode,
            "--rates", rates_arg(exp),
            "--duration-sec", duration,
            "--artifact-dir", str(artifact_dir),
        ])
        if exp.get("command_port") is not None:
            command.extend(["--command-port", str(exp["command_port"])])
        if exp.get("data_port") is not None:
            command.extend(["--data-port", str(exp["data_port"])])
        if bool_value(exp.get("persistent_socket")):
            command.append("--persistent-socket")
        if exp.get("rbscript_no_motion_command") is not None:
            command.extend(["--rbscript-no-motion-command", str(exp["rbscript_no_motion_command"])])
    elif script == "rb_backend_ablation":
        command.extend([
            "--left-ip", str(controllers["left_ip"]),
            "--right-ip", str(controllers["right_ip"]),
            "--arm", arm,
            "--backend", backend,
            "--mode", mode,
            "--rate-hz", single_rate(exp),
            "--duration-sec", duration,
            "--artifact-dir", str(artifact_dir),
        ])
        if exp.get("command_port") is not None:
            command.extend(["--command-port", str(exp["command_port"])])
        if exp.get("data_port") is not None:
            command.extend(["--data-port", str(exp["data_port"])])
        if bool_value(exp.get("persistent_socket")):
            command.append("--persistent-socket")
        if exp.get("rbscript_no_motion_command") is not None:
            command.extend(["--rbscript-no-motion-command", str(exp["rbscript_no_motion_command"])])
    elif script in {"rbpodo_servo_acceptance", "rbscript_servo_acceptance"}:
        if not exp.get("config"):
            raise MatrixError(f"experiment {exp.get('name')} requires config")
        command.extend([
            "--config", str(exp["config"]),
            "--arm", arm,
            "--mode", mode,
            "--duration-sec", duration,
            "--artifact-dir", str(artifact_dir),
        ])
        if exp.get("profile") is not None:
            command.extend(["--profile", str(exp["profile"])])
        if exp.get("command_rate_hz") is not None:
            command.extend(["--command-rate-hz", str(exp["command_rate_hz"])])
        if bool_value(exp.get("allow_motion")):
            command.append("--allow-motion")
        if bool_value(exp.get("allow_ack_disabled")):
            command.append("--allow-ack-disabled")
        if script == "rbscript_servo_acceptance":
            if exp.get("q_current_deg") is not None:
                command.extend(["--q-current-deg", str(exp["q_current_deg"])])
            if bool_value(exp.get("q_current_from_rbpodo")):
                command.append("--q-current-from-rbpodo")
    else:
        raise MatrixError(f"unsupported script: {script}")

    if args.i_understand_this_connects_to_real_controller:
        command.append("--i-understand-this-connects-to-real-controller")
    if args.skip_plots:
        command.append("--skip-plots")
    return command


def preflight_runner(matrix: dict[str, Any], args: argparse.Namespace) -> None:
    if args.max_workers != 1:
        raise MatrixError("--max-workers must be 1 for real-controller-safe serial execution")
    controllers = matrix["controllers"]
    known_real_ips = {str(controllers.get("left_ip")), str(controllers.get("right_ip"))} & REAL_ROBOT_IPS
    if known_real_ips and not args.i_understand_this_connects_to_real_controller:
        raise MatrixError("refusing known real controller IP without explicit matrix confirmation flag")


def enabled_experiments(matrix: dict[str, Any], include_disabled: bool) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for exp in matrix["experiments"]:
        if bool_value(exp.get("enabled"), True):
            out.append(exp)
        elif include_disabled:
            out.append(exp)
    return out


def base_status(exp: dict[str, Any], artifact_dir: Path, command: list[str] | None = None) -> dict[str, Any]:
    return {
        "name": exp.get("name"),
        "backend": exp.get("backend"),
        "script": exp.get("script"),
        "mode": normalize_mode(exp),
        "profile": exp.get("profile", ""),
        "artifact_dir": str(artifact_dir),
        "command": command or [],
        "started_ns": None,
        "ended_ns": None,
        "duration_sec": None,
        "status": "pending",
        "reason": "",
        "returncode": None,
        "summary_path": "",
    }


def write_status(path: Path, status: dict[str, Any]) -> None:
    path.write_text(json.dumps(status, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def status_summary_row(status: dict[str, Any], summary: dict[str, Any] | None = None) -> dict[str, Any]:
    row = {
        "experiment": status.get("name"),
        "status": status.get("status"),
        "reason": status.get("reason"),
        "backend": status.get("backend"),
        "script": status.get("script"),
        "mode": status.get("mode"),
        "profile": status.get("profile"),
        "artifact_dir": status.get("artifact_dir"),
        "requested_rate_hz": None,
        "achieved_rate_hz": None,
        "persistent_socket": None,
        "reconnect_count": None,
        "read_state_capability": "",
        "comparable": None,
        "success_rate": None,
        "p50_ack_us": None,
        "p95_ack_us": None,
        "p99_ack_us": None,
        "timeout_count": None,
        "error_count": None,
        "send_count": None,
        "send_success_count": None,
        "send_failure_count": None,
        "ack_observed_count": None,
        "controller_acceptance_observed_count": None,
        "not_comparable_reason": "",
    }
    if not summary:
        return row
    comparison = compare_backend.comparison_rows(summary)
    if comparison:
        first = comparison[0]
        for key in (
            "requested_rate_hz",
            "achieved_rate_hz",
            "persistent_socket",
            "reconnect_count",
            "read_state_capability",
            "comparable",
            "success_rate",
            "p50_ack_us",
            "p95_ack_us",
            "p99_ack_us",
            "timeout_count",
            "error_count",
            "not_comparable_reason",
        ):
            row[key] = first.get(key)
    for key in (
        "send_count",
        "send_success_count",
        "send_failure_count",
        "ack_observed_count",
        "controller_acceptance_observed_count",
    ):
        row[key] = summary.get(key, row[key])
    if summary.get("profile") is not None:
        row["profile"] = summary.get("profile")
    return row


def load_summary(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as handle:
        value = json.load(handle)
    if not isinstance(value, dict):
        raise MatrixError(f"summary is not a JSON object: {path}")
    return value


def write_summary_outputs(artifact_root: Path, rows: list[dict[str, Any]], statuses: list[dict[str, Any]]) -> None:
    (artifact_root / "backend_comparison_summary.json").write_text(
        json.dumps({"experiments": rows, "statuses": statuses}, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    with (artifact_root / "backend_comparison_summary.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=SUMMARY_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in SUMMARY_FIELDS})
    (artifact_root / "backend_comparison_report.md").write_text(report_markdown(rows), encoding="utf-8")


def fmt(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.3f}" if math.isfinite(value) else ""
    return str(value)


def table(rows: list[dict[str, Any]], fields: list[str]) -> str:
    if not rows:
        return "_No rows._\n"
    lines = ["| " + " | ".join(fields) + " |", "| " + " | ".join("---" for _ in fields) + " |"]
    for row in rows:
        lines.append("| " + " | ".join(fmt(row.get(field)) for field in fields) + " |")
    return "\n".join(lines) + "\n"


def mode_rows(rows: list[dict[str, Any]], modes: set[str]) -> list[dict[str, Any]]:
    return [row for row in rows if row.get("mode") in modes]


def recommendation(rows: list[dict[str, Any]]) -> str:
    notes = [
        "- Current primary backend: `rbpodo` remains the primary real-controller backend.",
        "- Experimental backend gaps: `rbscript_tcp` remains comparison-only until read_state and ServoJ no-op evidence are both comparable.",
    ]
    if any(row.get("read_state_capability") == "unsupported" for row in rows if row.get("backend") == "rbscript_tcp"):
        notes.append("- `rbscript_tcp` read_state is not comparable while Rainbow 5001 parsing is unsupported.")
    if not any(row.get("backend") == "rbscript_tcp" and row.get("mode") == "servo_j_noop" and row.get("status") == "completed" for row in rows):
        notes.append("- `rbscript_tcp` ServoJ no-op controller-simulation evidence is still missing or not run.")
    return "\n".join(notes) + "\n"


def report_markdown(rows: list[dict[str, Any]]) -> str:
    fields = ["experiment", "backend", "status", "requested_rate_hz", "success_rate", "p95_ack_us", "reason"]
    sections = [
        "# Backend Comparison Report",
        "",
        "## Connect Latency",
        table(mode_rows(rows, {"connect_only"}), fields),
        "## State Read Performance",
        table(mode_rows(rows, {"read_state"}), fields + ["read_state_capability", "comparable", "not_comparable_reason"]),
        "## Command ACK/No-Motion Performance",
        table(mode_rows(rows, {"command_ack_no_motion"}), fields + ["persistent_socket", "reconnect_count"]),
        "## ServoJ No-Op Controller Simulation Performance",
        table(mode_rows(rows, {"servo_j_noop"}), fields + ["send_count", "controller_acceptance_observed_count"]),
        "## Unsupported Capability Notes",
        table([row for row in rows if row.get("status") in {"unsupported", "skipped"} or row.get("not_comparable_reason")], ["experiment", "backend", "mode", "status", "reason", "not_comparable_reason"]),
        "## Recommendation",
        recommendation(rows),
    ]
    return "\n".join(sections)


def run_matrix(args: argparse.Namespace) -> dict[str, Any]:
    matrix = load_matrix(args.matrix)
    validate_matrix(matrix)
    preflight_runner(matrix, args)

    artifact_root = args.artifact_root.resolve()
    artifact_root.mkdir(parents=True, exist_ok=True)
    (artifact_root / "matrix_resolved.yaml").write_text(dump_matrix_yaml(matrix), encoding="utf-8")

    rows: list[dict[str, Any]] = []
    statuses: list[dict[str, Any]] = []
    selected = enabled_experiments(matrix, args.include_disabled)

    for exp in matrix["experiments"]:
        name = str(exp["name"])
        exp_dir = artifact_root / slug(name)
        exp_dir.mkdir(parents=True, exist_ok=True)
        if exp not in selected:
            status = base_status(exp, exp_dir)
            status.update({"status": "skipped", "reason": "experiment disabled"})
            write_status(exp_dir / "experiment_status.json", status)
            statuses.append(status)
            rows.append(status_summary_row(status))
            continue

        reason = unsupported_reason(exp)
        command = command_for_experiment(exp, matrix["controllers"], exp_dir, args) if not reason else []
        status = base_status(exp, exp_dir, command)
        if reason:
            status.update({"status": "unsupported", "reason": reason})
            write_status(exp_dir / "experiment_status.json", status)
            statuses.append(status)
            rows.append(status_summary_row(status))
            continue

        if normalize_mode(exp) in SERVO_NOOP_MODES:
            if not args.allow_servo_j_noop_simulation:
                raise MatrixError(f"experiment {name} requires --allow-servo-j-noop-simulation")
            if not bool_value(exp.get("allow_motion")):
                raise MatrixError(f"experiment {name} requires allow_motion: true")

        path = script_path(str(exp["script"]))
        if not path.is_file():
            raise MatrixError(f"missing required tool for experiment {name}: {path}")

        (exp_dir / "experiment_command.txt").write_text(shlex.join(command) + "\n", encoding="utf-8")
        if args.dry_run:
            print(shlex.join(command))
            status.update({"status": "dry_run", "reason": "command not executed"})
            write_status(exp_dir / "experiment_status.json", status)
            statuses.append(status)
            rows.append(status_summary_row(status))
            continue

        start = time.monotonic_ns()
        status["started_ns"] = start
        write_status(exp_dir / "experiment_status.json", status)
        completed = subprocess.run(
            command,
            cwd=str(ROOT),
            text=True,
            capture_output=True,
            env=os.environ.copy(),
        )
        end = time.monotonic_ns()
        (exp_dir / "stdout.log").write_text(completed.stdout, encoding="utf-8")
        (exp_dir / "stderr.log").write_text(completed.stderr, encoding="utf-8")
        status["ended_ns"] = end
        status["duration_sec"] = (end - start) / 1e9
        status["returncode"] = completed.returncode
        if completed.returncode != 0:
            status.update({"status": "failed", "reason": f"script exited with {completed.returncode}"})
            write_status(exp_dir / "experiment_status.json", status)
            statuses.append(status)
            rows.append(status_summary_row(status))
            write_summary_outputs(artifact_root, rows, statuses)
            raise MatrixError(f"experiment {name} failed: script exited with {completed.returncode}")

        summary_path = exp_dir / "summary.json"
        if not summary_path.is_file():
            status.update({"status": "failed", "reason": "summary.json missing"})
            write_status(exp_dir / "experiment_status.json", status)
            statuses.append(status)
            rows.append(status_summary_row(status))
            write_summary_outputs(artifact_root, rows, statuses)
            raise MatrixError(f"experiment {name} failed: summary.json missing")
        summary = load_summary(summary_path)
        if summary.get("result") not in {None, "completed", "pass"}:
            status.update({"status": "failed", "reason": f"summary result={summary.get('result')}"})
            write_status(exp_dir / "experiment_status.json", status)
            statuses.append(status)
            rows.append(status_summary_row(status, summary))
            write_summary_outputs(artifact_root, rows, statuses)
            raise MatrixError(f"experiment {name} failed: summary result={summary.get('result')}")

        status.update({"status": "completed", "reason": "", "summary_path": str(summary_path)})
        write_status(exp_dir / "experiment_status.json", status)
        statuses.append(status)
        rows.append(status_summary_row(status, summary))

    write_summary_outputs(artifact_root, rows, statuses)
    return {"result": "completed", "artifact_root": str(artifact_root), "experiments": rows}


def main() -> int:
    args = parse_args()
    try:
        result = run_matrix(args)
    except MatrixError as exc:
        print(f"run_backend_comparison_matrix: FAIL: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
