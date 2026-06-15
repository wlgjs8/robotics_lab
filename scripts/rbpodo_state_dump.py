#!/usr/bin/env python3
"""Read-only rbpodo controller state dump for bring-up diagnostics."""

from __future__ import annotations

import argparse
import importlib
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


REAL_ROBOT_IPS = {"172.28.60.200", "172.28.60.201"}
DIAGNOSTIC_FIELDS = (
    "time",
    "real_vs_simulation_mode",
    "init_state_info",
    "init_error",
    "op_stat_sos_flag",
    "op_stat_ems_flag",
    "op_stat_soft_estop_occur",
    "op_stat_collision_occur",
    "op_stat_self_collision",
)
ERROR_FIELD_ORDER = (
    "init_error",
    "op_stat_sos_flag",
    "op_stat_ems_flag",
    "op_stat_soft_estop_occur",
    "op_stat_collision_occur",
    "op_stat_self_collision",
)
BOOLEAN_STATUS_FIELDS = (
    "op_stat_soft_estop_occur",
    "op_stat_collision_occur",
    "op_stat_self_collision",
)
ENUM_STATUS_RANGES = {
    "real_vs_simulation_mode": (0, 1),
    "init_state_info": (0, 6),
    "op_stat_sos_flag": (0, 12),
    "op_stat_ems_flag": (0, 4),
}


class StateDumpError(RuntimeError):
    pass


@dataclass
class JointNormalization:
    normalized_value_deg: float
    was_wrapped: bool
    equivalent_in_range: bool


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read rbpodo CobotData from one or more controllers and dump raw "
            "joint/status diagnostics. This tool is read-only and never sends "
            "motion, pgmode, reset, or controller-state-changing commands."
        )
    )
    parser.add_argument("--ips", nargs="+", help="Controller IPs to read.")
    parser.add_argument("--timeout-sec", type=float, default=1.0)
    parser.add_argument("--q-min", help="Comma-separated six joint lower limits in degrees.")
    parser.add_argument("--q-max", help="Comma-separated six joint upper limits in degrees.")
    parser.add_argument("--wrap-period-deg", help="Optional comma-separated six joint wrap periods in degrees; 0 disables wrapping.")
    parser.add_argument("--output", type=Path, help="Optional JSON artifact path.")
    parser.add_argument("--json", action="store_true", help="Print JSON to stdout instead of a human summary.")
    parser.add_argument("--pretty", action="store_true", help="Pretty-print JSON or the human summary.")
    parser.add_argument(
        "--print-sdk-info",
        action="store_true",
        help="Print rbpodo Python module file/version metadata and exit without connecting.",
    )
    parser.add_argument("--self-test", action="store_true", help=argparse.SUPPRESS)
    return parser.parse_args()


def parse_joint_array(text: str | None, name: str) -> list[float] | None:
    if text is None:
        return None
    values: list[float] = []
    for raw in text.split(","):
        item = raw.strip()
        if not item:
            raise StateDumpError(f"{name} contains an empty item")
        try:
            value = float(item)
        except ValueError as exc:
            raise StateDumpError(f"{name} contains a non-numeric value: {item}") from exc
        if not math.isfinite(value):
            raise StateDumpError(f"{name} values must be finite: {item}")
        values.append(value)
    if len(values) != 6:
        raise StateDumpError(f"{name} must contain exactly 6 comma-separated values")
    return values


def rbpodo_sdk_info() -> dict[str, Any]:
    info: dict[str, Any] = {
        "schema": "robotics_lab.rbpodo_sdk_info.v1",
        "python_executable": sys.executable,
        "python_version": sys.version.split()[0],
        "module_available": False,
        "module_file": None,
        "module_version": None,
        "module_error_name": None,
        "module_error_message": None,
        "cobot_data_available": False,
    }
    try:
        rbpodo = importlib.import_module("rbpodo")
    except Exception as exc:
        info["module_error_name"] = type(exc).__name__
        info["module_error_message"] = str(exc)
        return info

    version = None
    for attr in ("__version__", "VERSION", "version"):
        value = getattr(rbpodo, attr, None)
        if value is not None:
            version = str(value)
            break
    info.update({
        "module_available": True,
        "module_file": getattr(rbpodo, "__file__", None),
        "module_version": version,
        "cobot_data_available": hasattr(rbpodo, "CobotData"),
    })
    return info


def numeric(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def integer(value: Any) -> int | None:
    if isinstance(value, bool):
        return int(value)
    try:
        number = int(value)
    except (TypeError, ValueError):
        return None
    return number


def json_scalar(value: Any) -> Any:
    if isinstance(value, bool) or value is None:
        return value
    ivalue = integer(value)
    fvalue = numeric(value)
    if ivalue is not None and fvalue is not None and abs(float(ivalue) - fvalue) < 1e-12:
        return ivalue
    if fvalue is not None:
        return fvalue
    return str(value)


def finite_joint_array(value: Any) -> tuple[list[float | None], list[bool], bool]:
    values: list[float | None] = []
    finite: list[bool] = []
    try:
        items = list(value)
    except TypeError:
        items = []
    for index in range(6):
        item = items[index] if index < len(items) else None
        number = numeric(item)
        values.append(number)
        finite.append(number is not None)
    return values, finite, len(items) == 6 and all(finite)


def joint_delta_deg(
    reference_deg: list[float | None],
    actual_deg: list[float | None],
) -> tuple[list[float | None], float | None]:
    deltas: list[float | None] = []
    finite_abs: list[float] = []
    for ref, actual in zip(reference_deg, actual_deg):
        if ref is None or actual is None:
            deltas.append(None)
            continue
        delta = ref - actual
        deltas.append(delta)
        finite_abs.append(abs(delta))
    if len(deltas) < 6:
        deltas.extend([None] * (6 - len(deltas)))
    return deltas[:6], max(finite_abs) if finite_abs else None


def joint_value_in_range(value_deg: float, min_deg: float, max_deg: float) -> bool:
    return math.isfinite(value_deg) and math.isfinite(min_deg) and math.isfinite(max_deg) and min_deg <= value_deg <= max_deg


def normalize_joint_for_range(
    value_deg: float,
    min_deg: float,
    max_deg: float,
    wrap_period_deg: float,
) -> JointNormalization:
    if joint_value_in_range(value_deg, min_deg, max_deg):
        return JointNormalization(value_deg, False, True)
    if (
        not math.isfinite(value_deg)
        or not math.isfinite(min_deg)
        or not math.isfinite(max_deg)
        or not math.isfinite(wrap_period_deg)
        or wrap_period_deg <= 0.0
        or max_deg < min_deg
    ):
        return JointNormalization(value_deg, False, False)
    normalized = value_deg - math.floor((value_deg - min_deg) / wrap_period_deg) * wrap_period_deg
    if normalized < min_deg:
        normalized += wrap_period_deg
    if normalized >= min_deg + wrap_period_deg:
        normalized -= wrap_period_deg
    was_wrapped = abs(normalized - value_deg) > 1e-9
    equivalent = min_deg - 1e-9 <= normalized <= max_deg + 1e-9
    return JointNormalization(normalized, was_wrapped, equivalent)


def q_range_diagnostics(
    q_actual_deg: list[float | None],
    q_min_deg: list[float] | None,
    q_max_deg: list[float] | None,
    wrap_period_deg: list[float] | None,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], bool]:
    if q_min_deg is None or q_max_deg is None:
        return [], [], False
    violations: list[dict[str, Any]] = []
    wrapped: list[dict[str, Any]] = []
    wrap_cleared_any_violation = False
    for index, value in enumerate(q_actual_deg):
        if value is None:
            continue
        min_value = q_min_deg[index]
        max_value = q_max_deg[index]
        raw_in_range = joint_value_in_range(value, min_value, max_value)
        if not raw_in_range:
            violations.append({
                "joint": index + 1,
                "value_deg": value,
                "min_deg": min_value,
                "max_deg": max_value,
            })
        if wrap_period_deg is None:
            continue
        period = wrap_period_deg[index]
        normalized = normalize_joint_for_range(value, min_value, max_value, period)
        if period > 0.0 and (normalized.was_wrapped or not raw_in_range):
            wrapped.append({
                "joint": index + 1,
                "raw_deg": value,
                "normalized_deg": normalized.normalized_value_deg,
                "period_deg": period,
                "was_wrapped": normalized.was_wrapped,
                "equivalent_in_range": normalized.equivalent_in_range,
            })
        if not raw_in_range and normalized.equivalent_in_range:
            wrap_cleared_any_violation = True
    return violations, wrapped, wrap_cleared_any_violation


def extract_raw_fields(sdata: Any) -> dict[str, Any]:
    return {field: json_scalar(getattr(sdata, field, None)) for field in DIAGNOSTIC_FIELDS}


def diagnostic_interpretation(raw: dict[str, Any]) -> tuple[bool, list[str], list[str]]:
    suspect_reasons: list[str] = []
    clear_errors: list[str] = []
    time_value = numeric(raw.get("time"))
    if time_value is None:
        suspect_reasons.append("time is non-finite or missing")
    elif time_value < 0.0 or (0.0 < abs(time_value) < 1e-6):
        suspect_reasons.append(f"time is implausible: {time_value}")

    for field, (minimum, maximum) in ENUM_STATUS_RANGES.items():
        value = integer(raw.get(field))
        if value is None or value < minimum or value > maximum:
            suspect_reasons.append(f"{field} is outside expected range [{minimum}, {maximum}]: {raw.get(field)}")
        elif value != 0 and field in ERROR_FIELD_ORDER:
            clear_errors.append(field)

    for field in BOOLEAN_STATUS_FIELDS:
        value = integer(raw.get(field))
        if value not in {0, 1}:
            suspect_reasons.append(f"{field} expected 0/1, got {raw.get(field)}")
        elif value == 1:
            clear_errors.append(field)

    init_error = integer(raw.get("init_error"))
    if init_error is not None and init_error != 0:
        clear_errors.append("init_error")

    for field in ERROR_FIELD_ORDER:
        value = integer(raw.get(field))
        if value is not None and abs(value) >= 1_000_000:
            suspect_reasons.append(f"{field} has huge value: {value}")

    return bool(suspect_reasons), suspect_reasons, sorted(set(clear_errors))


def controller_mode_name(raw_mode: Any) -> str:
    value = integer(raw_mode)
    if value == 1:
        return "simulation"
    if value == 0:
        return "real"
    return "unknown"


def controller_mode_warning(raw_mode: Any) -> str | None:
    name = controller_mode_name(raw_mode)
    if name == "simulation":
        return None
    if name == "real":
        return "controller not confirmed in pgmode simulation: real_vs_simulation_mode=0"
    return f"controller not confirmed in pgmode simulation: real_vs_simulation_mode={raw_mode}"


def first_nonzero_error(raw: dict[str, Any]) -> tuple[str | None, int | None]:
    for field in ERROR_FIELD_ORDER:
        value = integer(raw.get(field))
        if value not in {None, 0}:
            return field, value
    return None, None


def recommendations(
    q_actual_all_finite: bool,
    diagnostics_suspect: bool,
    clear_errors: list[str],
    q_range_violations: list[dict[str, Any]],
    wrap_cleared_any_violation: bool,
    read_ok: bool,
) -> list[str]:
    steps: list[str] = []
    if not read_ok:
        steps.append("Check controller data port 5001, network reachability, and rbpodo Python SDK installation.")
        return steps
    if not q_actual_all_finite:
        steps.append("Treat this as state acquisition failure; q_actual must be six finite values before startup diagnostics can proceed.")
    if q_actual_all_finite and diagnostics_suspect and not clear_errors:
        steps.append("Use read-only diagnostic startup to publish raw controller diagnostics; do not promote motion while diagnostics_suspect is true.")
    if wrap_cleared_any_violation:
        steps.append("Enable safety.joint_wrap_for_startup_validation with matching joint_wrap_period_deg for startup diagnostics.")
    elif q_range_violations:
        steps.append("Review safety.q_min_deg/q_max_deg against the raw controller joint representation before startup.")
    if clear_errors:
        steps.append("Clear or investigate controller/pendant error state before any motion-capable acceptance.")
    if not steps:
        steps.append("State acquisition looks finite; compare this dump with rb_servo_server startup_validation JSON.")
    return steps


def build_report_for_sdata(
    ip: str,
    sdata: Any,
    q_min_deg: list[float] | None,
    q_max_deg: list[float] | None,
    wrap_period_deg: list[float] | None,
) -> dict[str, Any]:
    raw = extract_raw_fields(sdata)
    raw_mode = raw.get("real_vs_simulation_mode")
    mode_warning = controller_mode_warning(raw_mode)
    q_actual, q_actual_finite, q_actual_all_finite = finite_joint_array(getattr(sdata, "jnt_ang", []))
    q_ref, q_ref_finite, q_ref_all_finite = finite_joint_array(getattr(sdata, "jnt_ref", []))
    q_ref_actual_delta, max_abs_q_ref_actual_delta = joint_delta_deg(q_ref, q_actual)
    violations, wrapped, wrap_cleared = q_range_diagnostics(q_actual, q_min_deg, q_max_deg, wrap_period_deg)
    diagnostics_suspect, suspect_reasons, clear_errors = diagnostic_interpretation(raw)
    first_name, first_code = first_nonzero_error(raw)
    next_steps = recommendations(
        q_actual_all_finite,
        diagnostics_suspect,
        clear_errors,
        violations,
        wrap_cleared,
        True,
    )
    if mode_warning:
        next_steps.append(
            "Verify controller pgmode simulation with scripts/rainbow_pgmode.py before controller-simulation benchmarks."
        )
    return {
        "ip": ip,
        "ok": True,
        "raw": raw,
        "real_vs_simulation_mode": raw_mode,
        "controller_mode": controller_mode_name(raw_mode),
        "controller_mode_is_simulation": controller_mode_name(raw_mode) == "simulation",
        "controller_mode_warning": mode_warning,
        "first_nonzero_error_name": first_name,
        "first_nonzero_error_code": first_code,
        "diagnostics_suspect": diagnostics_suspect,
        "diagnostics_suspect_reasons": suspect_reasons,
        "clear_error_flags": clear_errors,
        "q_actual_deg": q_actual,
        "q_ref": q_ref,
        "jnt_ref": q_ref,
        "q_ref_deg": q_ref,
        "jnt_ref_deg": q_ref,
        "q_ref_source": "python_rbpodo.sdata.jnt_ref",
        "q_ref_actual_delta_deg": q_ref_actual_delta,
        "q_actual_vs_q_ref_max_abs_error_deg": max_abs_q_ref_actual_delta,
        "q_actual_finite": q_actual_finite,
        "q_ref_finite": q_ref_finite,
        "q_actual_all_finite": q_actual_all_finite,
        "q_ref_all_finite": q_ref_all_finite,
        "q_range_violations": violations,
        "q_range_wrapped": wrapped,
        "recommended_next_steps": next_steps,
    }


def read_controller(ip: str, timeout_sec: float) -> Any:
    try:
        rbpodo = importlib.import_module("rbpodo")
    except Exception as exc:
        raise StateDumpError("Python rbpodo module is unavailable") from exc
    if not hasattr(rbpodo, "CobotData"):
        raise StateDumpError("Python rbpodo module does not expose CobotData")
    data = rbpodo.CobotData(ip)
    state = data.request_data(timeout_sec)
    if state is None:
        raise StateDumpError("CobotData.request_data returned no state")
    return getattr(state, "sdata", state)


def dump_states(args: argparse.Namespace) -> dict[str, Any]:
    if not args.ips:
        raise StateDumpError("--ips is required")
    if not math.isfinite(args.timeout_sec) or args.timeout_sec <= 0.0:
        raise StateDumpError("--timeout-sec must be finite and positive")

    q_min = parse_joint_array(args.q_min, "--q-min")
    q_max = parse_joint_array(args.q_max, "--q-max")
    if (q_min is None) != (q_max is None):
        raise StateDumpError("--q-min and --q-max must be provided together")
    wrap = parse_joint_array(args.wrap_period_deg, "--wrap-period-deg")
    if wrap is not None and any(value < 0.0 for value in wrap):
        raise StateDumpError("--wrap-period-deg values must be non-negative")

    results: list[dict[str, Any]] = []
    for ip in args.ips:
        try:
            sdata = read_controller(ip, args.timeout_sec)
            results.append(build_report_for_sdata(ip, sdata, q_min, q_max, wrap))
        except Exception as exc:
            results.append({
                "ip": ip,
                "ok": False,
                "error_name": type(exc).__name__,
                "error_message": str(exc),
                "recommended_next_steps": recommendations(False, False, [], [], False, False),
            })
    return {
        "schema": "robotics_lab.rbpodo_state_dump.v1",
        "read_only": True,
        "safety_note": "This tool only reads rbpodo CobotData and never sends motion, pgmode, reset, or state-changing commands.",
        "timestamp_unix_sec": time.time(),
        "sdk_info": rbpodo_sdk_info(),
        "ips": list(args.ips),
        "known_real_controller_ips": sorted(REAL_ROBOT_IPS),
        "q_min_deg": q_min,
        "q_max_deg": q_max,
        "wrap_period_deg": wrap,
        "results": results,
    }


def human_summary(report: dict[str, Any]) -> str:
    lines = [
        "rbpodo_state_dump: read-only controller state",
        f"schema: {report['schema']}",
    ]
    for item in report.get("results", []):
        lines.append(f"\n[{item.get('ip')}] ok={item.get('ok')}")
        if not item.get("ok"):
            lines.append(f"  error: {item.get('error_name')}: {item.get('error_message')}")
        else:
            lines.append(f"  q_actual_deg: {item.get('q_actual_deg')}")
            lines.append(f"  q_ref_deg: {item.get('q_ref_deg')}")
            lines.append(f"  q_ref_source: {item.get('q_ref_source')}")
            lines.append(f"  q_ref_actual_delta_deg: {item.get('q_ref_actual_delta_deg')}")
            lines.append(
                "  q_actual_vs_q_ref_max_abs_error_deg: "
                f"{item.get('q_actual_vs_q_ref_max_abs_error_deg')}"
            )
            lines.append(f"  real_vs_simulation_mode: {item.get('real_vs_simulation_mode')}")
            lines.append(f"  controller_mode: {item.get('controller_mode')}")
            if item.get("controller_mode_warning"):
                lines.append(f"  WARNING: {item.get('controller_mode_warning')}")
            lines.append(f"  q_actual_all_finite: {item.get('q_actual_all_finite')}")
            lines.append(f"  diagnostics_suspect: {item.get('diagnostics_suspect')}")
            if item.get("diagnostics_suspect_reasons"):
                lines.append(f"  diagnostics_suspect_reasons: {item.get('diagnostics_suspect_reasons')}")
            lines.append(f"  first_nonzero_error: {item.get('first_nonzero_error_name')}={item.get('first_nonzero_error_code')}")
            lines.append(f"  q_range_violations: {item.get('q_range_violations')}")
            lines.append(f"  q_range_wrapped: {item.get('q_range_wrapped')}")
        for step in item.get("recommended_next_steps", []):
            lines.append(f"  next: {step}")
    return "\n".join(lines) + "\n"


def run_self_test() -> int:
    parsed = parse_joint_array("-170,-120,-170,-360,-120,-360", "--q-min")
    assert parsed == [-170.0, -120.0, -170.0, -360.0, -120.0, -360.0]
    try:
        parse_joint_array("1,2,3", "--bad")
    except StateDumpError:
        pass
    else:
        raise AssertionError("expected bad joint array to fail")

    raw = {
        "time": 3.096e-41,
        "real_vs_simulation_mode": 1,
        "init_state_info": 6,
        "init_error": 0,
        "op_stat_sos_flag": 0,
        "op_stat_ems_flag": 0,
        "op_stat_soft_estop_occur": 0,
        "op_stat_collision_occur": 0,
        "op_stat_self_collision": 1977953904,
    }
    suspect, reasons, clear = diagnostic_interpretation(raw)
    assert suspect
    assert not clear
    assert any("op_stat_self_collision" in reason for reason in reasons)
    assert controller_mode_name(1) == "simulation"
    assert controller_mode_name(0) == "real"
    assert controller_mode_warning(0) == "controller not confirmed in pgmode simulation: real_vs_simulation_mode=0"
    assert controller_mode_warning(1) is None

    normalized = normalize_joint_for_range(-317.0, -190.0, 190.0, 360.0)
    assert normalized.was_wrapped
    assert normalized.equivalent_in_range
    assert abs(normalized.normalized_value_deg - 43.0) < 1e-9

    q = [0.0, -30.0, -317.0, 0.0, 60.0, 0.0]
    violations, wrapped, cleared = q_range_diagnostics(
        q,
        [-170.0, -120.0, -190.0, -190.0, -120.0, -360.0],
        [170.0, 120.0, 190.0, 190.0, 120.0, 360.0],
        [0.0, 0.0, 360.0, 0.0, 0.0, 360.0],
    )
    assert violations and wrapped and cleared

    class FakeSData:
        time = 1.0
        real_vs_simulation_mode = 1
        init_state_info = 6
        init_error = 0
        op_stat_sos_flag = 0
        op_stat_ems_flag = 0
        op_stat_soft_estop_occur = 0
        op_stat_collision_occur = 0
        op_stat_self_collision = 0
        jnt_ang = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
        jnt_ref = [0.5, 1.0, 1.5, 3.0, 4.5, 5.0]

    fake_report = build_report_for_sdata("127.0.0.1", FakeSData(), None, None, None)
    assert fake_report["q_ref_source"] == "python_rbpodo.sdata.jnt_ref"
    assert fake_report["q_ref"] == fake_report["q_ref_deg"]
    assert fake_report["jnt_ref"] == fake_report["q_ref_deg"]
    assert fake_report["q_ref_actual_delta_deg"] == [0.5, 0.0, -0.5, 0.0, 0.5, 0.0]
    assert fake_report["q_actual_vs_q_ref_max_abs_error_deg"] == 0.5
    print("rbpodo_state_dump self-test passed")
    return 0


def main() -> int:
    args = parse_args()
    if args.self_test:
        return run_self_test()
    if args.print_sdk_info:
        print(json.dumps(rbpodo_sdk_info(), indent=2, sort_keys=True))
        return 0
    try:
        report = dump_states(args)
    except StateDumpError as exc:
        print(f"rbpodo_state_dump: FAIL: {exc}", file=sys.stderr)
        return 2
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.json:
        indent = 2 if args.pretty else None
        print(json.dumps(report, indent=indent, sort_keys=True))
    else:
        print(human_summary(report), end="")
    return 0 if all(item.get("ok") for item in report.get("results", [])) else 1


if __name__ == "__main__":
    raise SystemExit(main())
