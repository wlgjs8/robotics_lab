#!/usr/bin/env python3
"""Fail-closed F/T monitor-log acceptance analysis.

This tool never changes configuration. It extracts a static-window incremental
residual-tare candidate and blocks promotion when freshness, frame projection,
or the configured hard-limit margin is not demonstrated by the CSV.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path
from typing import Iterable


class AnalysisError(RuntimeError):
    pass


def _percentile(values: list[float], quantile: float) -> float:
    ordered = sorted(values)
    if not ordered:
        raise AnalysisError("cannot compute a percentile from an empty sample set")
    index = round((len(ordered) - 1) * min(1.0, max(0.0, quantile)))
    return ordered[index]


def _rotate_xyzw(q: list[float], v: list[float]) -> list[float]:
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 0.0:
        raise AnalysisError("TCP quaternion is non-finite or zero length")
    x, y, z, w = (value / norm for value in (x, y, z, w))
    return [
        (1 - 2 * (y * y + z * z)) * v[0] + 2 * (x * y - z * w) * v[1] + 2 * (x * z + y * w) * v[2],
        2 * (x * y + z * w) * v[0] + (1 - 2 * (x * x + z * z)) * v[1] + 2 * (y * z - x * w) * v[2],
        2 * (x * z - y * w) * v[0] + 2 * (y * z + x * w) * v[1] + (1 - 2 * (x * x + y * y)) * v[2],
    ]


def _norm(values: Iterable[float]) -> float:
    return math.sqrt(sum(value * value for value in values))


def _required_columns(arm: str) -> list[str]:
    columns = [
        "loop_start_time_ns",
        f"{arm}_ft_healthy",
        f"{arm}_ft_stale",
        f"{arm}_ft_freshness_value",
        f"{arm}_ft_freshness_advanced",
        f"{arm}_ft_reason",
        f"{arm}_ft_auto_tare_enabled",
        f"{arm}_ft_tare_valid",
        f"{arm}_ft_tare_state",
        f"{arm}_force_control_measured_normal_force_n",
        f"{arm}_force_control_fast_normal_force_n",
        f"{arm}_force_control_fast_force_norm_n",
        f"{arm}_force_control_fast_torque_norm_nm",
    ]
    for prefix, unit in (("fast_external_f", "n"), ("fast_external_t", "nm"), ("control_external_f", "n")):
        columns.extend(f"{arm}_ft_{prefix}{axis}_{unit}" for axis in "xyz")
    columns.extend(f"{arm}_force_control_normal_stand_{axis}" for axis in "xyz")
    columns.extend(f"{arm}_tcp_actual_stand_q{axis}" for axis in "xyzw")
    return columns


def analyze_arm(
    path: Path,
    arm: str,
    static_start_sec: float,
    static_end_sec: float,
    hard_force_norm_n: float,
    hard_torque_norm_nm: float,
    projection_tolerance_n: float,
) -> dict[str, object]:
    required = _required_columns(arm)
    samples: list[dict[str, object]] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.reader(handle)
        try:
            header = next(reader)
        except StopIteration as exc:
            raise AnalysisError("CSV is empty") from exc
        missing = sorted(set(required) - set(header))
        if missing:
            raise AnalysisError("missing required CSV columns: " + ", ".join(missing))
        indices = {name: header.index(name) for name in required}
        first_time_ns: int | None = None
        previous_sequence: int | None = None
        regressions = duplicates = unhealthy = stale = not_advanced = invalid_tare = 0
        reasons: dict[str, int] = {}
        for row in reader:
            time_ns = int(row[indices["loop_start_time_ns"]])
            if first_time_ns is None:
                first_time_ns = time_ns
            elapsed = (time_ns - first_time_ns) * 1e-9
            sequence = int(float(row[indices[f"{arm}_ft_freshness_value"]]))
            if previous_sequence is not None:
                regressions += sequence < previous_sequence
                duplicates += sequence == previous_sequence
            previous_sequence = sequence
            unhealthy += row[indices[f"{arm}_ft_healthy"]] not in {"1", "true", "True"}
            stale += row[indices[f"{arm}_ft_stale"]] in {"1", "true", "True"}
            not_advanced += row[indices[f"{arm}_ft_freshness_advanced"]] not in {"1", "true", "True"}
            reason = row[indices[f"{arm}_ft_reason"]]
            reasons[reason] = reasons.get(reason, 0) + 1
            if not static_start_sec <= elapsed <= static_end_sec:
                continue
            auto_tare_enabled = row[indices[f"{arm}_ft_auto_tare_enabled"]] in {
                "1", "true", "True"
            }
            if auto_tare_enabled:
                tare_valid = row[indices[f"{arm}_ft_tare_valid"]] in {"1", "true", "True"}
                tare_state = row[indices[f"{arm}_ft_tare_state"]]
                invalid_tare += not tare_valid or tare_state != "accepted"
            fast_force = [float(row[indices[f"{arm}_ft_fast_external_f{axis}_n"]]) for axis in "xyz"]
            fast_torque = [float(row[indices[f"{arm}_ft_fast_external_t{axis}_nm"]]) for axis in "xyz"]
            control_force = [float(row[indices[f"{arm}_ft_control_external_f{axis}_n"]]) for axis in "xyz"]
            normal = [float(row[indices[f"{arm}_force_control_normal_stand_{axis}"]]) for axis in "xyz"]
            quaternion = [float(row[indices[f"{arm}_tcp_actual_stand_q{axis}"]]) for axis in "xyzw"]
            # The surface normal is geometric/outward.  Sensor reaction force
            # points against it during compression, while force-control
            # telemetry intentionally reports compression as positive.
            expected_normal = -sum(a * b for a, b in zip(normal, _rotate_xyzw(quaternion, control_force)))
            expected_fast_normal = -sum(a * b for a, b in zip(normal, _rotate_xyzw(quaternion, fast_force)))
            measured_normal = float(row[indices[f"{arm}_force_control_measured_normal_force_n"]])
            logged_fast_normal = float(row[indices[f"{arm}_force_control_fast_normal_force_n"]])
            logged_force_norm = float(row[indices[f"{arm}_force_control_fast_force_norm_n"]])
            logged_torque_norm = float(row[indices[f"{arm}_force_control_fast_torque_norm_nm"]])
            samples.append({
                "fast_force": fast_force,
                "fast_torque": fast_torque,
                "force_norm": _norm(fast_force),
                "torque_norm": _norm(fast_torque),
                "projection_error": measured_normal - expected_normal,
                "fast_projection_error": logged_fast_normal - expected_fast_normal,
                "force_norm_error": logged_force_norm - _norm(fast_force),
                "torque_norm_error": logged_torque_norm - _norm(fast_torque),
            })
    if not samples:
        raise AnalysisError(
            f"{arm}: no samples in static window {static_start_sec:.3f}..{static_end_sec:.3f}s"
        )

    force_axes = [[sample["fast_force"][axis] for sample in samples] for axis in range(3)]
    torque_axes = [[sample["fast_torque"][axis] for sample in samples] for axis in range(3)]
    force_norms = [float(sample["force_norm"]) for sample in samples]
    torque_norms = [float(sample["torque_norm"]) for sample in samples]
    projection_errors = [abs(float(sample["projection_error"])) for sample in samples]
    fast_projection_errors = [abs(float(sample["fast_projection_error"])) for sample in samples]
    force_norm_errors = [abs(float(sample["force_norm_error"])) for sample in samples]
    torque_norm_errors = [abs(float(sample["torque_norm_error"])) for sample in samples]
    tare = [statistics.median(values) for values in force_axes + torque_axes]
    force_p99 = _percentile(force_norms, 0.99)
    torque_p99 = _percentile(torque_norms, 0.99)
    projection_error_max = max(projection_errors)
    fast_projection_error_max = max(fast_projection_errors)
    force_norm_error_max = max(force_norm_errors)
    torque_norm_error_max = max(torque_norm_errors)
    blockers: list[str] = []
    if unhealthy or stale or not_advanced or regressions or duplicates:
        blockers.append("FT freshness/health was not continuously valid")
    if invalid_tare:
        blockers.append(
            f"software zero was not accepted for {invalid_tare} static-window rows"
        )
    if force_p99 >= hard_force_norm_n:
        blockers.append(
            f"static force norm p99 {force_p99:.3f}N is not below hard limit {hard_force_norm_n:.3f}N"
        )
    if torque_p99 >= hard_torque_norm_nm:
        blockers.append(
            f"static torque norm p99 {torque_p99:.3f}Nm is not below hard limit {hard_torque_norm_nm:.3f}Nm"
        )
    if projection_error_max > projection_tolerance_n:
        blockers.append(
            f"logged normal-force projection disagrees with CSV pose/wrench by up to "
            f"{projection_error_max:.3f}N (limit {projection_tolerance_n:.3f}N)"
        )
    if fast_projection_error_max > projection_tolerance_n:
        blockers.append(
            f"logged fast normal-force projection disagrees with CSV pose/wrench by up to "
            f"{fast_projection_error_max:.3f}N (limit {projection_tolerance_n:.3f}N)"
        )
    # CSV wrench components and derived norms are decimal text; allow their
    # bounded serialization round-off while still catching computation errors.
    if force_norm_error_max > 1e-3 or torque_norm_error_max > 1e-3:
        blockers.append(
            "logged fast wrench norms disagree with the CSV wrench "
            f"(force {force_norm_error_max:.6g}N, torque {torque_norm_error_max:.6g}Nm)"
        )
    return {
        "arm": arm,
        "static_sample_count": len(samples),
        "freshness": {
            "unhealthy_rows": unhealthy,
            "stale_rows": stale,
            "not_advanced_rows": not_advanced,
            "invalid_tare_rows_in_static_window": invalid_tare,
            "sequence_regressions": regressions,
            "sequence_duplicates": duplicates,
            "reasons": reasons,
        },
        "incremental_residual_tare_tcp_candidate": tare,
        "static_force_axis_stddev_n": [statistics.pstdev(values) for values in force_axes],
        "static_torque_axis_stddev_nm": [statistics.pstdev(values) for values in torque_axes],
        "static_force_norm_p99_n": force_p99,
        "static_torque_norm_p99_nm": torque_p99,
        "normal_projection_error_max_n": projection_error_max,
        "fast_normal_projection_error_max_n": fast_projection_error_max,
        "fast_force_norm_error_max_n": force_norm_error_max,
        "fast_torque_norm_error_max_nm": torque_norm_error_max,
        "promotion_ready": not blockers,
        "blockers": blockers,
    }


def analyze_csv(
    path: Path,
    static_start_sec: float = 2.0,
    static_end_sec: float = 10.0,
    hard_force_norm_n: float = 20.0,
    hard_torque_norm_nm: float = 3.0,
    projection_tolerance_n: float = 0.25,
) -> dict[str, object]:
    if static_start_sec < 0.0 or static_end_sec <= static_start_sec:
        raise AnalysisError("static window must satisfy 0 <= start < end")
    arms = [
        analyze_arm(
            path, arm, static_start_sec, static_end_sec,
            hard_force_norm_n, hard_torque_norm_nm, projection_tolerance_n,
        )
        for arm in ("left", "right")
    ]
    return {
        "csv": str(path),
        "static_window_sec": [static_start_sec, static_end_sec],
        "promotion_ready": all(bool(arm["promotion_ready"]) for arm in arms),
        "arms": arms,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--static-start-sec", type=float, default=2.0)
    parser.add_argument("--static-end-sec", type=float, default=10.0)
    parser.add_argument("--hard-force-norm-n", type=float, default=20.0)
    parser.add_argument("--hard-torque-norm-nm", type=float, default=3.0)
    parser.add_argument("--projection-tolerance-n", type=float, default=0.25)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    try:
        result = analyze_csv(
            args.csv,
            args.static_start_sec,
            args.static_end_sec,
            args.hard_force_norm_n,
            args.hard_torque_norm_nm,
            args.projection_tolerance_n,
        )
    except (AnalysisError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}")
        return 2
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print(f"F/T promotion ready: {result['promotion_ready']}")
        for arm in result["arms"]:
            print(f"\n[{arm['arm']}] ready={arm['promotion_ready']}")
            print("  incremental residual_tare_tcp candidate: " +
                  "[" + ", ".join(f"{value:.6f}" for value in arm["incremental_residual_tare_tcp_candidate"]) + "]")
            print(f"  static force norm p99: {arm['static_force_norm_p99_n']:.3f} N")
            print(f"  static torque norm p99: {arm['static_torque_norm_p99_nm']:.3f} Nm")
            print(f"  normal projection max error: {arm['normal_projection_error_max_n']:.3f} N")
            for blocker in arm["blockers"]:
                print(f"  BLOCK: {blocker}")
    return 0 if result["promotion_ready"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
