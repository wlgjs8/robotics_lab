#!/usr/bin/env python3
"""Reduce recorded rbpodo physical state captures into stage artifacts.

This tool is intentionally hardware-free. It reads a JSON-lines capture of
robotics_lab.servo_state.v1 packets and emits an external physical-transition
stage summary. It never opens a socket and never commands motion.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


SCHEMA = "robotics_lab.rbpodo_physical_transition.external_stage.v1"
DEFAULT_ROOT = Path(__file__).resolve().parents[1]
MOTION_THRESHOLD_DEG = 1.0e-3
STAGE_NAMES = {
    "P1": "real_readonly_diagnostics_parity",
    "P2": "stop_resetFault_or_operator_stop_policy_verified",
    "P3": "real_hold_no_motion",
    "P4": "tiny_joint_noop_or_tiny_joint_motion",
    "P5": "tiny_cartesian_delta",
    "P6": "slow_physical_circle_5cm_10s",
    "P7": "stable_physical_circle_15cm_16s",
    "P8": "medium_physical_circle_15cm_8s",
    "P9": "fast_physical_circle_15cm_4s_only_after_explicit_approval",
}
CIRCLE_STAGES = {"P6", "P7", "P8", "P9"}
TRACKING_STAGES = {"P5", *CIRCLE_STAGES}


class MeasureError(RuntimeError):
    pass


@dataclass(frozen=True)
class StateSample:
    index: int
    timestamp_ns: int
    snapshot: dict[str, Any]
    arm_state: dict[str, Any]


@dataclass(frozen=True)
class DesiredSample:
    t_sec: float | None
    pose: dict[str, float]


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Measure a recorded rbpodo physical stage capture. Input is a "
            "JSON-lines state capture, not a live socket."
        )
    )
    parser.add_argument("--stage", required=True, choices=tuple(STAGE_NAMES))
    parser.add_argument("--capture", required=True, type=Path, help="Recorded servo_state JSON-lines capture.")
    parser.add_argument(
        "--desired-trajectory",
        type=Path,
        help="Desired trajectory file for circle stages, or commanded delta evidence for P5.",
    )
    parser.add_argument("--arm", required=True, choices=("left", "right"))
    parser.add_argument("--window-sec", type=float, help="Use only the trailing capture window.")
    parser.add_argument("--artifact-dir", required=True, type=Path, help="Directory that will receive summary.json.")
    return parser.parse_args(argv)


def finite_number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def numeric_ns(value: Any) -> int | None:
    number = finite_number(value)
    if number is None:
        return None
    return int(round(number))


def finite_array(value: Any, length: int) -> list[float] | None:
    if not isinstance(value, list) or len(value) != length:
        return None
    out = [finite_number(item) for item in value]
    if any(item is None for item in out):
        return None
    return [float(item) for item in out if item is not None]


def pose(value: Any, label: str) -> dict[str, float]:
    if not isinstance(value, dict):
        raise MeasureError(f"{label} must be an object")
    out: dict[str, float] = {}
    for key in ("x", "y", "z"):
        number = finite_number(value.get(key))
        if number is None:
            raise MeasureError(f"{label}.{key} must be finite")
        out[key] = number
    return out


def pose_for_source(snapshot: dict[str, Any], arm: str, source: str) -> dict[str, float]:
    """Mirror the benchmark's published-pose reduction; do not re-derive FK."""
    arm_state = snapshot.get(arm)
    if not isinstance(arm_state, dict):
        raise MeasureError(f"{arm} state is missing")
    if source == "tcp_actual_stand":
        if arm_state.get("tcp_actual_valid") is not True:
            raise MeasureError(f"{arm}.tcp_actual_stand is marked invalid")
        return pose(arm_state.get("tcp_actual_stand"), f"{arm}.tcp_actual_stand")
    if source == "tcp_ref_stand":
        if arm_state.get("tcp_ref_valid") is not True:
            raise MeasureError(f"{arm}.tcp_ref_stand is marked invalid")
        return pose(arm_state.get("tcp_ref_stand"), f"{arm}.tcp_ref_stand")
    raise MeasureError(f"unsupported tracking source {source}")


def position(p: dict[str, float]) -> list[float]:
    return [float(p["x"]), float(p["y"]), float(p["z"])]


def vector_sub(a: list[float], b: list[float]) -> list[float]:
    return [a[index] - b[index] for index in range(3)]


def norm(v: list[float]) -> float:
    return math.sqrt(sum(item * item for item in v))


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


def error_metrics(errors: list[float]) -> dict[str, float | None]:
    if not errors:
        return {"rms_error_m": None, "p95_error_m": None, "max_error_m": None}
    return {
        "rms_error_m": math.sqrt(sum(item * item for item in errors) / len(errors)),
        "p95_error_m": percentile(errors, 0.95),
        "max_error_m": max(errors),
    }


def load_capture(path: Path, arm: str) -> list[StateSample]:
    if not path.is_file():
        raise MeasureError(f"capture not found: {path}")
    samples: list[StateSample] = []
    for index, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if not raw_line.strip():
            continue
        try:
            snapshot = json.loads(raw_line)
        except json.JSONDecodeError as exc:
            raise MeasureError(f"{path}:{index}: invalid JSON: {exc}") from exc
        if not isinstance(snapshot, dict):
            raise MeasureError(f"{path}:{index}: state row must be an object")
        arm_state = snapshot.get(arm)
        if not isinstance(arm_state, dict):
            raise MeasureError(f"{path}:{index}: {arm} state is missing")
        timestamp_ns = numeric_ns(snapshot.get("host_time_ns"))
        if timestamp_ns is None:
            timestamp_ns = numeric_ns(arm_state.get("host_time_ns"))
        if timestamp_ns is None:
            raise MeasureError(f"{path}:{index}: host_time_ns is required")
        samples.append(StateSample(index=index, timestamp_ns=timestamp_ns, snapshot=snapshot, arm_state=arm_state))
    return sorted(samples, key=lambda sample: sample.timestamp_ns)


def apply_window(samples: list[StateSample], window_sec: float | None) -> list[StateSample]:
    if not samples or window_sec is None:
        return samples
    if not math.isfinite(window_sec) or window_sec <= 0.0:
        raise MeasureError("--window-sec must be finite and positive")
    cutoff = samples[-1].timestamp_ns - int(round(window_sec * 1_000_000_000.0))
    return [sample for sample in samples if sample.timestamp_ns >= cutoff]


def first_present(mapping: dict[str, Any], keys: tuple[str, ...]) -> Any:
    for key in keys:
        value = mapping.get(key)
        if value is not None:
            return value
    return None


def desired_pose_from_object(value: Any, label: str) -> dict[str, float]:
    if not isinstance(value, dict):
        raise MeasureError(f"{label} must be an object")
    candidate = first_present(
        value,
        ("desired_pose_stand", "target_tcp_stand", "tcp_stand", "pose", "position"),
    )
    if isinstance(candidate, dict):
        return pose(candidate, label)
    return pose(value, label)


def desired_time(value: dict[str, Any]) -> float | None:
    for key in ("t_sec", "time_sec", "timestamp_sec", "time"):
        number = finite_number(value.get(key))
        if number is not None:
            return number
    return None


def load_desired_trajectory(path: Path | None) -> tuple[list[DesiredSample], list[float] | None]:
    if path is None:
        return [], None
    if not path.is_file():
        raise MeasureError(f"desired trajectory not found: {path}")
    commanded_delta: list[float] | None = None
    samples: list[DesiredSample] = []
    if path.suffix.lower() == ".csv":
        with path.open(newline="", encoding="utf-8") as stream:
            for index, row in enumerate(csv.DictReader(stream), start=2):
                samples.append(DesiredSample(t_sec=desired_time(row), pose=desired_pose_from_object(row, f"{path}:{index}")))
        return samples, commanded_delta
    text = path.read_text(encoding="utf-8")
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        data = [json.loads(line) for line in text.splitlines() if line.strip()]
    if isinstance(data, dict):
        delta = data.get("commanded_delta_m")
        if delta is None:
            delta = data.get("commanded_delta_stand_m")
        commanded_delta = finite_array(delta, 3) if delta is not None else None
        rows = first_present(data, ("samples", "trajectory", "desired_trajectory", "poses"))
        if rows is None:
            rows = [data] if any(key in data for key in ("x", "y", "z", "pose", "desired_pose_stand")) else []
    else:
        rows = data
    if not isinstance(rows, list):
        raise MeasureError(f"{path}: desired trajectory must be a list or object with samples")
    for index, row in enumerate(rows, start=1):
        if not isinstance(row, dict):
            raise MeasureError(f"{path}:{index}: desired sample must be an object")
        samples.append(DesiredSample(t_sec=desired_time(row), pose=desired_pose_from_object(row, f"{path}:{index}")))
    return samples, commanded_delta


def nearest_desired(
    desired: list[DesiredSample],
    sample_index: int,
    t_sec: float,
) -> dict[str, float]:
    if not desired:
        raise MeasureError("desired trajectory is empty")
    timed = [item for item in desired if item.t_sec is not None]
    if timed:
        return min(timed, key=lambda item: abs(float(item.t_sec) - t_sec)).pose
    if sample_index >= len(desired):
        raise MeasureError("desired trajectory without timestamps must match capture sample count")
    return desired[sample_index].pose


def pose_errors(
    samples: list[StateSample],
    arm: str,
    desired: list[DesiredSample],
    source: str,
) -> list[float]:
    start_ns = samples[0].timestamp_ns
    errors: list[float] = []
    for index, sample in enumerate(samples):
        actual = pose_for_source(sample.snapshot, arm, source)
        target = nearest_desired(desired, index, (sample.timestamp_ns - start_ns) / 1_000_000_000.0)
        errors.append(norm(vector_sub(position(target), position(actual))))
    return errors


def commanded_delta_from_capture(samples: list[StateSample]) -> list[float] | None:
    poses: list[dict[str, float]] = []
    for sample in samples:
        try:
            poses.append(pose(sample.arm_state.get("tcp_command_stand"), "tcp_command_stand"))
        except MeasureError:
            continue
    if len(poses) < 2:
        return None
    return vector_sub(position(poses[-1]), position(poses[0]))


def commanded_delta_from_desired(desired: list[DesiredSample]) -> list[float] | None:
    if len(desired) < 2:
        return None
    return vector_sub(position(desired[-1].pose), position(desired[0].pose))


def delta_error(samples: list[StateSample], arm: str, source: str, commanded_delta: list[float]) -> list[float]:
    start = position(pose_for_source(samples[0].snapshot, arm, source))
    end = position(pose_for_source(samples[-1].snapshot, arm, source))
    achieved = vector_sub(end, start)
    return [norm(vector_sub(commanded_delta, achieved))]


def finite_joint_array(sample: StateSample, key: str) -> list[float] | None:
    return finite_array(sample.arm_state.get(key), 6)


def update_rate(samples: list[StateSample], key: str) -> float | None:
    valid = [sample for sample in samples if finite_joint_array(sample, key) is not None]
    if len(valid) < 2:
        return None
    duration_sec = (valid[-1].timestamp_ns - valid[0].timestamp_ns) / 1_000_000_000.0
    if duration_sec <= 0.0:
        return None
    return (len(valid) - 1) / duration_sec


def state_age_block(samples: list[StateSample]) -> dict[str, float | None]:
    ages = [finite_number(sample.arm_state.get("state_age_us")) for sample in samples]
    values = [float(item) for item in ages if item is not None]
    return {"p95": percentile(values, 0.95), "max": max(values) if values else None}


def state_jitter_block(samples: list[StateSample]) -> dict[str, float | None]:
    intervals = [
        (samples[index].timestamp_ns - samples[index - 1].timestamp_ns) / 1000.0
        for index in range(1, len(samples))
    ]
    if not intervals:
        return {"p95": None, "max": None}
    median = percentile(intervals, 0.50)
    assert median is not None
    jitter = [abs(item - median) for item in intervals]
    return {"p95": percentile(jitter, 0.95), "max": max(jitter)}


def fault_latch_status(samples: list[StateSample]) -> str:
    for sample in samples:
        if sample.snapshot.get("fault_latched") is True or sample.arm_state.get("has_error") is True:
            return "fail"
    return "pass"


def cartesian_availability(samples: list[StateSample]) -> str:
    values = [sample.arm_state.get("cartesian_available") for sample in samples]
    if any(value is False for value in values):
        return "unavailable"
    if any(value is True for value in values):
        return "available"
    return "not_checked"


def physical_motion_detected(samples: list[StateSample]) -> bool | None:
    first = finite_joint_array(samples[0], "q_actual_deg")
    if first is None:
        return None
    max_delta = 0.0
    for sample in samples[1:]:
        current = finite_joint_array(sample, "q_actual_deg")
        if current is None:
            continue
        max_delta = max(max_delta, max(abs(current[index] - first[index]) for index in range(6)))
    return max_delta > MOTION_THRESHOLD_DEG


def read_calibration(root: Path) -> dict[str, Any]:
    path = root / "calibration" / "active_calibration.yaml"
    if not path.is_file():
        return {
            "path": str(path),
            "status": "unknown",
            "measured": False,
            "geometry_valid_for_real_policy": False,
        }
    top: dict[str, Any] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].rstrip()
        if not line.strip() or line.startswith(" "):
            continue
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        text = value.strip().strip('"').strip("'")
        if text.lower() == "true":
            parsed: Any = True
        elif text.lower() == "false":
            parsed = False
        else:
            parsed = text
        top[key.strip()] = parsed
    status = str(top.get("status") or "unknown")
    return {
        "path": str(path),
        "calibration_id": top.get("calibration_id"),
        "status": status,
        "measured": status == "measured",
        "geometry_valid_for_real_policy": bool(top.get("geometry_valid_for_real_policy") is True),
    }


def physical_expected(stage: str) -> bool:
    return stage in TRACKING_STAGES


def tracking_result(
    *,
    stage: str,
    samples: list[StateSample],
    arm: str,
    desired: list[DesiredSample],
    commanded_delta: list[float] | None,
    source: str,
    physical: bool,
) -> dict[str, Any]:
    result: dict[str, Any] = {
        "status": "not_run" if physical else "informational_only",
        "tracking_source": source,
        "rms_error_m": None,
        "p95_error_m": None,
        "max_error_m": None,
    }
    try:
        if stage in CIRCLE_STAGES:
            if not desired:
                raise MeasureError(f"{stage} requires --desired-trajectory")
            result.update(error_metrics(pose_errors(samples, arm, desired, source)))
            if physical:
                result["status"] = "pass"
        elif stage == "P5":
            if commanded_delta is None:
                raise MeasureError("P5 requires commanded delta from tcp_command_stand or --desired-trajectory")
            result.update(error_metrics(delta_error(samples, arm, source, commanded_delta)))
            if physical:
                result["status"] = "pass"
        elif physical:
            result["status"] = "not_run"
            result["reason"] = "no Cartesian tracking expected for this stage"
        return result
    except MeasureError as exc:
        if physical:
            result["status"] = "fail"
        result["reason"] = str(exc)
        return result


def validate_actual_window(samples: list[StateSample]) -> str | None:
    if not samples:
        return "capture window is empty"
    invalid_rows = [
        str(sample.index)
        for sample in samples
        if sample.arm_state.get("tcp_actual_valid") is not True or not isinstance(sample.arm_state.get("tcp_actual_stand"), dict)
    ]
    if invalid_rows:
        return "tcp_actual_stand is invalid for capture row(s): " + ", ".join(invalid_rows[:10])
    return None


def build_summary(args: argparse.Namespace) -> dict[str, Any]:
    all_samples = load_capture(args.capture, args.arm)
    samples = apply_window(all_samples, args.window_sec)
    desired, explicit_delta = load_desired_trajectory(args.desired_trajectory)
    commanded_delta = explicit_delta or commanded_delta_from_desired(desired) or commanded_delta_from_capture(samples)

    fail_closed_reason = validate_actual_window(samples)
    if fail_closed_reason is not None:
        physical = {
            "status": "fail",
            "tracking_source": "tcp_actual_stand",
            "rms_error_m": None,
            "p95_error_m": None,
            "max_error_m": None,
            "reason": fail_closed_reason,
        }
        if samples:
            controller = tracking_result(
                stage=args.stage,
                samples=samples,
                arm=args.arm,
                desired=desired,
                commanded_delta=commanded_delta,
                source="tcp_ref_stand",
                physical=False,
            )
        else:
            controller = {
                "status": "informational_only",
                "tracking_source": "tcp_ref_stand",
                "rms_error_m": None,
                "p95_error_m": None,
                "max_error_m": None,
                "reason": "capture window is empty",
            }
        result_status = "fail"
    else:
        physical = tracking_result(
            stage=args.stage,
            samples=samples,
            arm=args.arm,
            desired=desired,
            commanded_delta=commanded_delta,
            source="tcp_actual_stand",
            physical=True,
        )
        controller = tracking_result(
            stage=args.stage,
            samples=samples,
            arm=args.arm,
            desired=desired,
            commanded_delta=commanded_delta,
            source="tcp_ref_stand",
            physical=False,
        )
        result_status = "fail" if physical["status"] == "fail" else "pass"

    return {
        "schema": SCHEMA,
        "generated_at_unix": time.time(),
        "stage": {
            "id": args.stage,
            "ladder_name": STAGE_NAMES[args.stage],
        },
        "result": {
            "status": result_status,
            "dry_run": False,
            "hardware_process_started": False,
            "motion_command_sent": False,
            "blockers": [physical["reason"]] if physical.get("status") == "fail" and physical.get("reason") else [],
        },
        "source_capture": {
            "path": str(args.capture),
            "arm": args.arm,
            "total_samples": len(all_samples),
            "window_samples": len(samples),
            "window_sec": args.window_sec,
            "desired_trajectory": str(args.desired_trajectory) if args.desired_trajectory else None,
        },
        "physical_tracking_result": physical,
        "controller_reference_result": controller,
        "telemetry_requirements": {
            "state_age_us": state_age_block(samples) if samples else {"p95": None, "max": None},
            "state_jitter_us": state_jitter_block(samples) if samples else {"p95": None, "max": None},
            "q_actual_update_rate_hz": update_rate(samples, "q_actual_deg") if samples else None,
            "q_ref_update_rate_hz": update_rate(samples, "q_ref_deg") if samples else None,
            "fault_latch_status": fault_latch_status(samples) if samples else "not_checked",
            "cartesian_availability": cartesian_availability(samples) if samples else "not_checked",
            "stop_reset_behavior_result": "unresolved" if args.stage == "P2" else "not_applicable",
            "physical_motion_expected": physical_expected(args.stage),
            "physical_motion_detected": physical_motion_detected(samples) if samples else None,
        },
        "calibration": read_calibration(DEFAULT_ROOT),
        "safety_notes": [
            "Input was a recorded JSON-lines state capture; no live UDP capture was opened.",
            "Physical tracking evidence is gated on tcp_actual_valid and uses tcp_actual_stand only.",
            "tcp_ref_stand is informational controller-reference evidence only.",
        ],
    }


def write_summary(artifact_dir: Path, summary: dict[str, Any]) -> Path:
    artifact_dir.mkdir(parents=True, exist_ok=True)
    path = artifact_dir / "summary.json"
    path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    return path


def main(argv: list[str] | None = None) -> int:
    try:
        args = parse_args(argv)
        summary = build_summary(args)
        path = write_summary(args.artifact_dir, summary)
        print(f"Artifact written: {path}")
        print(f"Stage {summary['stage']['id']} result: {summary['result']['status']}")
        print("Physical tracking source: tcp_actual_stand")
        print("Controller reference source: tcp_ref_stand (informational only)")
        return 2 if summary["result"]["status"] == "fail" else 0
    except MeasureError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
