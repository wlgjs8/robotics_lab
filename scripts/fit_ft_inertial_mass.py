#!/usr/bin/env python3
"""Fit effective translational F/T inertial mass from a servo-log CSV.

The fit uses no-contact samples only, double-differences the sent TCP command,
rotates the resulting stand-frame acceleration into the commanded TCP frame,
and scans force/acceleration lag. Positive lag means the measured force occurs
that many log ticks after the command acceleration. Translation only: torque,
CoM, and rotational inertia identification are intentionally out of scope.

Pure stdlib so the script can run on the operator PC.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics


AXES = ("x", "y", "z")


def _quaternion_rotation_transpose(qx, qy, qz, qw, vector):
    """Return R(q)^T * vector for an xyzw unit quaternion."""
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("non-finite or zero commanded TCP quaternion")
    x, y, z, w = qx / norm, qy / norm, qz / norm, qw / norm
    # Columns of R are rows of R^T.
    rows = (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + w * z),
         2.0 * (x * z - w * y)),
        (2.0 * (x * y - w * z), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z + w * x)),
        (2.0 * (x * z + w * y), 2.0 * (y * z - w * x),
         1.0 - 2.0 * (x * x + y * y)),
    )
    return tuple(sum(row[k] * vector[k] for k in range(3)) for row in rows)


def load_rows(path, arm):
    """Load timestamp, sent TCP pose, fast force, and optional health flag."""
    pose = [f"{arm}_tcp_command_stand_{axis}_m" for axis in AXES]
    quat = [f"{arm}_tcp_command_stand_q{axis}" for axis in ("x", "y", "z")]
    quat.append(f"{arm}_tcp_command_stand_qw")
    force = [f"{arm}_ft_fast_external_f{axis}_n" for axis in AXES]
    wanted = ["loop_start_time_ns", *pose, *quat, *force]
    rows = []
    with open(path, newline="") as stream:
        reader = csv.DictReader(stream)
        missing = [name for name in wanted if name not in (reader.fieldnames or [])]
        if missing:
            raise KeyError(f"{path}: missing columns {missing}")
        healthy_name = f"{arm}_ft_healthy"
        for row_index, row in enumerate(reader):
            if healthy_name in row and row[healthy_name] not in ("", "1", "true", "True"):
                continue
            try:
                values = [float(row[name]) for name in wanted]
            except (TypeError, ValueError):
                continue
            if not all(math.isfinite(value) for value in values):
                continue
            rows.append({
                "row_index": row_index,
                "time_s": values[0] * 1e-9,
                "position": tuple(values[1:4]),
                "quaternion": tuple(values[4:8]),
                "force": tuple(values[8:11]),
            })
    if len(rows) < 6:
        raise ValueError("fewer than six usable servo-log rows")
    return rows


def commanded_accelerations(rows):
    """Variable-period three-point double difference, expressed in TCP."""
    output = {}
    for i in range(2, len(rows)):
        older, newer, current = rows[i - 2], rows[i - 1], rows[i]
        older_dt = newer["time_s"] - older["time_s"]
        newer_dt = current["time_s"] - newer["time_s"]
        if older_dt <= 0.0 or newer_dt <= 0.0:
            continue
        old_velocity = tuple(
            (newer["position"][axis] - older["position"][axis]) / older_dt
            for axis in range(3)
        )
        new_velocity = tuple(
            (current["position"][axis] - newer["position"][axis]) / newer_dt
            for axis in range(3)
        )
        accel_stand = tuple(
            2.0 * (new_velocity[axis] - old_velocity[axis]) /
            (older_dt + newer_dt)
            for axis in range(3)
        )
        output[current["row_index"]] = _quaternion_rotation_transpose(
            *current["quaternion"], accel_stand)
    return output


def _linear_fit(xs, ys):
    if len(xs) < 3:
        return None
    xmean = statistics.fmean(xs)
    ymean = statistics.fmean(ys)
    variance = sum((x - xmean) ** 2 for x in xs)
    if variance <= 1e-12:
        return None
    slope = sum((x - xmean) * (y - ymean) for x, y in zip(xs, ys)) / variance
    intercept = ymean - slope * xmean
    residuals = [y - (intercept + slope * x) for x, y in zip(xs, ys)]
    return {
        "mass_kg": slope,
        "intercept_n": intercept,
        "sum_squared_residual_n2": sum(value * value for value in residuals),
        "samples": len(xs),
    }


def fit_inertial_mass(rows, force_threshold_n, max_lag_ticks=40):
    accelerations = commanded_accelerations(rows)
    forces = {row["row_index"]: row["force"] for row in rows}
    best = None
    for lag in range(-max_lag_ticks, max_lag_ticks + 1):
        pairs = []
        for index, accel in accelerations.items():
            force = forces.get(index + lag)
            if force is None:
                continue
            if math.sqrt(sum(component * component for component in force)) >= force_threshold_n:
                continue
            pairs.append((accel, force))
        fits = []
        for axis in range(3):
            fit = _linear_fit(
                [pair[0][axis] for pair in pairs],
                [pair[1][axis] for pair in pairs],
            )
            if fit is None:
                break
            fits.append(fit)
        if len(fits) != 3:
            continue
        residual_count = sum(fit["samples"] for fit in fits)
        residual_rms = math.sqrt(
            sum(fit["sum_squared_residual_n2"] for fit in fits) / residual_count)
        candidate = {
            "best_lag_ticks": lag,
            "axis_fits": {axis: fits[i] for i, axis in enumerate(AXES)},
            "recommended_inertial_effective_mass_kg": statistics.fmean(
                fit["mass_kg"] for fit in fits),
            "residual_rms_n": residual_rms,
            "matched_rows": len(pairs),
            "force_threshold_n": force_threshold_n,
        }
        if best is None or candidate["residual_rms_n"] < best["residual_rms_n"]:
            best = candidate
    if best is None:
        raise ValueError("no lag had sufficient per-axis acceleration excitation")
    return best


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path")
    parser.add_argument("--arm", choices=("left", "right"), required=True)
    parser.add_argument("--force-threshold-n", type=float, default=10.0)
    parser.add_argument("--max-lag-ticks", type=int, default=40)
    parser.add_argument("--json", dest="json_path")
    args = parser.parse_args(argv)
    if args.force_threshold_n <= 0.0:
        parser.error("--force-threshold-n must be positive")
    if args.max_lag_ticks < 0:
        parser.error("--max-lag-ticks must be non-negative")

    result = fit_inertial_mass(
        load_rows(args.csv_path, args.arm),
        args.force_threshold_n,
        args.max_lag_ticks,
    )
    print(f"recommended inertial_effective_mass_kg: "
          f"{result['recommended_inertial_effective_mass_kg']:.6g}")
    print(f"best-fit lag: {result['best_lag_ticks']} ticks "
          "(positive = force trails command acceleration)")
    print(f"residual RMS: {result['residual_rms_n']:.6g} N")
    for axis in AXES:
        fit = result["axis_fits"][axis]
        print(f"  {axis}: mass={fit['mass_kg']:.6g} kg, "
              f"intercept={fit['intercept_n']:.6g} N, n={fit['samples']}")
    if args.json_path:
        with open(args.json_path, "w") as stream:
            json.dump(result, stream, indent=2, sort_keys=True)
            stream.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
