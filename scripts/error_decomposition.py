#!/usr/bin/env python3
"""Circle tracking error decomposition helpers.

This module is offline/diagnostic only. It consumes benchmark artifact rows and
summary fields; it does not connect to controllers or send commands.
"""

from __future__ import annotations

import math
from typing import Any


DEFAULT_TOOL_OFFSETS_M = (0.03, 0.05, 0.10)
EPSILON = 1e-12


def finite_number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        parsed = float(value)
        return parsed if math.isfinite(parsed) else None
    if isinstance(value, str):
        try:
            parsed = float(value)
        except ValueError:
            return None
        return parsed if math.isfinite(parsed) else None
    return None


def parse_tool_offsets(value: Any) -> list[float]:
    if value is None:
        return list(DEFAULT_TOOL_OFFSETS_M)
    if isinstance(value, str):
        pieces = [piece.strip() for piece in value.split(",") if piece.strip()]
    elif isinstance(value, (list, tuple)):
        pieces = list(value)
    else:
        pieces = [value]
    offsets: list[float] = []
    for piece in pieces:
        number = finite_number(piece)
        if number is None or number <= 0.0:
            raise ValueError(f"invalid --tool-offset-m value: {piece!r}")
        offsets.append(number)
    if not offsets:
        raise ValueError("--tool-offset-m must contain at least one positive value")
    return offsets


def percentile(values: list[float], pct: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * pct / 100.0
    lo = math.floor(index)
    hi = math.ceil(index)
    if lo == hi:
        return ordered[int(index)]
    return ordered[lo] * (hi - index) + ordered[hi] * (index - lo)


def rms(values: list[float]) -> float | None:
    if not values:
        return None
    return math.sqrt(sum(value * value for value in values) / len(values))


def norm3(value: list[float]) -> float:
    return math.sqrt(sum(component * component for component in value))


def sub3(a: list[float], b: list[float]) -> list[float]:
    return [a[i] - b[i] for i in range(3)]


def add3(a: list[float], b: list[float]) -> list[float]:
    return [a[i] + b[i] for i in range(3)]


def scale3(a: list[float], scale: float) -> list[float]:
    return [a[i] * scale for i in range(3)]


def finite_vec3(value: Any) -> list[float] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    parsed = [finite_number(item) for item in value]
    if any(item is None for item in parsed):
        return None
    return [float(item) for item in parsed if item is not None]


def geometry_vector(geometry: dict[str, Any] | None, key: str) -> list[float] | None:
    if not isinstance(geometry, dict):
        return None
    return finite_vec3(geometry.get(key))


def geometry_radius(geometry: dict[str, Any] | None, summary: dict[str, Any]) -> float | None:
    if isinstance(geometry, dict):
        value = finite_number(geometry.get("radius"))
        if value is not None and value > 0.0:
            return value
    value = finite_number(summary.get("reference_radius_m"))
    return value if value is not None and value > 0.0 else None


def row_point(row: dict[str, Any], prefix: str) -> list[float] | None:
    values = [finite_number(row.get(f"{prefix}_{axis}")) for axis in ("x", "y", "z")]
    if any(value is None for value in values):
        return None
    return [float(value) for value in values if value is not None]


def reference_point(
    geometry: dict[str, Any] | None,
    phase_rad: float,
    summary: dict[str, Any],
) -> list[float] | None:
    center = geometry_vector(geometry, "center")
    axis1 = geometry_vector(geometry, "axis1")
    axis2 = geometry_vector(geometry, "axis2")
    radius = geometry_radius(geometry, summary)
    if center is None or axis1 is None or axis2 is None or radius is None:
        return None
    return add3(center, add3(scale3(axis1, radius * math.cos(phase_rad)), scale3(axis2, radius * math.sin(phase_rad))))


def center_offset_vector(geometry: dict[str, Any] | None, summary: dict[str, Any]) -> list[float] | None:
    fit_center = summary.get("fit_center_plane_m")
    if not isinstance(fit_center, list) or len(fit_center) != 2:
        return None
    c1 = finite_number(fit_center[0])
    c2 = finite_number(fit_center[1])
    axis1 = geometry_vector(geometry, "axis1")
    axis2 = geometry_vector(geometry, "axis2")
    if c1 is None or c2 is None or axis1 is None or axis2 is None:
        return None
    return add3(scale3(axis1, c1), scale3(axis2, c2))


def circle_fit(points: list[tuple[float, float]]) -> dict[str, Any]:
    if len(points) < 3:
        return {"fit_radius_m": None, "fit_center": None, "fit_reason": "fewer than 3 samples"}
    n = float(len(points))
    sx = sum(x for x, _y in points)
    sy = sum(y for _x, y in points)
    sxx = sum(x * x for x, _y in points)
    syy = sum(y * y for _x, y in points)
    sxy = sum(x * y for x, y in points)
    rhs_x = -sum(x * (x * x + y * y) for x, y in points)
    rhs_y = -sum(y * (x * x + y * y) for x, y in points)
    rhs_1 = -sum(x * x + y * y for x, y in points)
    matrix = [[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, n]]
    rhs = [rhs_x, rhs_y, rhs_1]
    try:
        solution = solve_3x3(matrix, rhs)
    except ValueError:
        return {"fit_radius_m": None, "fit_center": None, "fit_reason": "least-squares system was singular"}
    a, b, c = solution
    cx = -a / 2.0
    cy = -b / 2.0
    radius_sq = cx * cx + cy * cy - c
    if radius_sq <= 0.0 or not math.isfinite(radius_sq):
        return {"fit_radius_m": None, "fit_center": None, "fit_reason": "fit radius was invalid"}
    return {"fit_radius_m": math.sqrt(radius_sq), "fit_center": [cx, cy], "fit_reason": None}


def solve_3x3(matrix: list[list[float]], rhs: list[float]) -> list[float]:
    rows = [list(row) + [rhs_value] for row, rhs_value in zip(matrix, rhs)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda row: abs(rows[row][col]))
        if abs(rows[pivot][col]) < EPSILON:
            raise ValueError("singular")
        rows[col], rows[pivot] = rows[pivot], rows[col]
        pivot_value = rows[col][col]
        rows[col] = [value / pivot_value for value in rows[col]]
        for row in range(3):
            if row == col:
                continue
            factor = rows[row][col]
            rows[row] = [value - factor * col_value for value, col_value in zip(rows[row], rows[col])]
    return [rows[row][3] for row in range(3)]


def plane_coords(point: list[float], geometry: dict[str, Any] | None) -> tuple[float, float] | None:
    center = geometry_vector(geometry, "center")
    axis1 = geometry_vector(geometry, "axis1")
    axis2 = geometry_vector(geometry, "axis2")
    if center is None or axis1 is None or axis2 is None:
        return None
    rel = sub3(point, center)
    return sum(rel[i] * axis1[i] for i in range(3)), sum(rel[i] * axis2[i] for i in range(3))


def orientation_equivalent(orientation_values: list[float], offsets: list[float]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for offset in offsets:
        equivalents = [value * offset for value in orientation_values]
        key = f"{offset:.3f}".rstrip("0").rstrip(".")
        out[key] = {
            "offset_m": offset,
            "p50": percentile(equivalents, 50.0),
            "p95": percentile(equivalents, 95.0),
            "max": max(equivalents) if equivalents else None,
        }
    return out


def classify_error_source(
    *,
    summary: dict[str, Any],
    rms_error_m: float | None,
    phase_aligned_rms_error_m: float | None,
    center_removed_rms_error_m: float | None,
    tail_ratio: float | None,
    orientation_equiv_50mm_p95_m: float | None,
) -> tuple[str, list[str], list[str]]:
    classifications: list[str] = []
    reasons: list[str] = []

    timing = str(summary.get("timing_classification") or "")
    if timing and timing != "clean_timing":
        classifications.append("timing_jitter_limited")
        reasons.append(f"timestamp timing_classification={timing}")

    saturation_count = finite_number(summary.get("feedback_saturation_count"))
    command_count = finite_number(summary.get("command_count"))
    saturation_ratio = saturation_count / command_count if saturation_count is not None and command_count and command_count > 0 else None
    if saturation_count is not None and saturation_count > 0 and (saturation_ratio is None or saturation_ratio >= 0.05):
        classifications.append("saturation_limited")
        if saturation_ratio is None:
            reasons.append(f"feedback_saturation_count={saturation_count:g}")
        else:
            reasons.append(f"feedback saturation ratio={saturation_ratio:.3f}")

    center_error = finite_number(summary.get("fit_center_error_m"))
    center_improvement = improvement_ratio(rms_error_m, center_removed_rms_error_m)
    if center_improvement is not None and center_improvement >= 0.25 and center_error is not None and center_error >= 0.005:
        classifications.append("center_drift_limited")
        reasons.append(f"center removal improves RMS by {center_improvement:.1%}")

    phase_improvement = improvement_ratio(rms_error_m, phase_aligned_rms_error_m)
    phase_lag = finite_number(summary.get("estimated_phase_lag_rad"))
    if phase_improvement is not None and phase_improvement >= 0.25 and phase_lag is not None:
        classifications.append("phase_lag_limited")
        reasons.append(f"phase alignment improves RMS by {phase_improvement:.1%}")

    if tail_ratio is not None and tail_ratio >= 3.0:
        classifications.append("tail_spike_limited")
        reasons.append(f"p95/median tail ratio={tail_ratio:.3f}")

    orientation_p95 = finite_number(summary.get("p95_orientation_drift_rad"))
    if (
        (orientation_equiv_50mm_p95_m is not None and orientation_equiv_50mm_p95_m >= 0.005)
        or (orientation_p95 is not None and orientation_p95 >= 0.10)
    ):
        classifications.append("orientation_limited")
        if orientation_equiv_50mm_p95_m is not None:
            reasons.append(f"50mm orientation equivalent p95={orientation_equiv_50mm_p95_m:.6f} m")
        elif orientation_p95 is not None:
            reasons.append(f"p95_orientation_drift_rad={orientation_p95:.6f}")

    deduped = list(dict.fromkeys(classifications))
    if not deduped:
        return "balanced_or_unclassified", [], ["no single error source exceeded decomposition thresholds"]
    return deduped[0], deduped, reasons


def improvement_ratio(original: float | None, improved: float | None) -> float | None:
    if original is None or improved is None or original <= EPSILON:
        return None
    return max(0.0, (original - improved) / original)


def cycle_metrics(
    rows: list[dict[str, Any]],
    *,
    period_sec: float | None,
    geometry: dict[str, Any] | None,
    feedback_rows: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    if period_sec is None or period_sec <= 0.0:
        return []
    by_cycle: dict[int, list[dict[str, Any]]] = {}
    for row in rows:
        t_sec = finite_number(row.get("t_sec"))
        if t_sec is None or t_sec < 0.0:
            continue
        cycle_index = int(t_sec // period_sec)
        by_cycle.setdefault(cycle_index, []).append(row)
    saturation_by_cycle: dict[int, int] = {}
    for row in feedback_rows:
        t_sec = finite_number(row.get("t_sec"))
        if t_sec is None or t_sec < 0.0:
            continue
        cycle_index = int(t_sec // period_sec)
        if row.get("saturated") is True:
            saturation_by_cycle[cycle_index] = saturation_by_cycle.get(cycle_index, 0) + 1
    out: list[dict[str, Any]] = []
    reference_radius = geometry_radius(geometry, {})
    for cycle_index in sorted(by_cycle):
        cycle_rows = by_cycle[cycle_index]
        errors = [value for value in (finite_number(row.get("position_error_m")) for row in cycle_rows) if value is not None]
        orientations = [value for value in (finite_number(row.get("orientation_drift_rad")) for row in cycle_rows) if value is not None]
        points: list[tuple[float, float]] = []
        for row in cycle_rows:
            point = row_point(row, "actual")
            coords = plane_coords(point, geometry) if point is not None else None
            if coords is not None:
                points.append(coords)
        fit = circle_fit(points)
        fit_center = fit.get("fit_center")
        fit_radius = finite_number(fit.get("fit_radius_m"))
        center_error = math.sqrt(fit_center[0] * fit_center[0] + fit_center[1] * fit_center[1]) if isinstance(fit_center, list) else None
        out.append({
            "cycle_index": cycle_index,
            "cycle_start_sec": cycle_index * period_sec,
            "cycle_end_sec": (cycle_index + 1) * period_sec,
            "cycle_sample_count": len(cycle_rows),
            "cycle_rms_error_m": rms(errors),
            "cycle_p95_error_m": percentile(errors, 95.0),
            "cycle_fit_center_error_m": center_error,
            "cycle_radius_gain": fit_radius / reference_radius if fit_radius is not None and reference_radius else None,
            "cycle_orientation_p95_rad": percentile(orientations, 95.0),
            "cycle_saturation_count": saturation_by_cycle.get(cycle_index, 0),
        })
    return out


def decompose_circle_run(
    rows: list[dict[str, Any]],
    *,
    summary: dict[str, Any],
    geometry: dict[str, Any] | None = None,
    period_sec: float | None = None,
    tool_offsets_m: list[float] | tuple[float, ...] | None = None,
    feedback_rows: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    offsets = list(tool_offsets_m or DEFAULT_TOOL_OFFSETS_M)
    errors = [value for value in (finite_number(row.get("position_error_m")) for row in rows) if value is not None]
    orientations = [value for value in (finite_number(row.get("orientation_drift_rad")) for row in rows) if value is not None]
    median_error = percentile(errors, 50.0)
    p95_error = percentile(errors, 95.0)
    max_error = max(errors) if errors else None
    mad = percentile([abs(value - median_error) for value in errors], 50.0) if median_error is not None else None
    q25 = percentile(errors, 25.0)
    q75 = percentile(errors, 75.0)
    iqr = q75 - q25 if q25 is not None and q75 is not None else None
    if p95_error is not None and median_error is not None:
        if median_error > EPSILON:
            tail_ratio = p95_error / median_error
        else:
            tail_ratio = math.inf if p95_error > EPSILON else 1.0
    else:
        tail_ratio = None
    max_over_p95 = max_error / p95_error if max_error is not None and p95_error is not None and p95_error > EPSILON else None
    base_rms = finite_number(summary.get("rms_error_m"))
    if base_rms is None:
        base_rms = rms(errors)
    center_offset = center_offset_vector(geometry, summary)
    phase_lag = finite_number(summary.get("estimated_phase_lag_rad"))

    phase_errors: list[float] = []
    center_errors: list[float] = []
    center_phase_errors: list[float] = []
    for row in rows:
        actual = row_point(row, "actual")
        if actual is None:
            continue
        reference = row_point(row, "reference")
        reference_phase = finite_number(row.get("reference_phase_rad"))
        phase_reference = None
        if phase_lag is not None and reference_phase is not None:
            phase_reference = reference_point(geometry, reference_phase - phase_lag, summary)
        if phase_reference is not None:
            phase_errors.append(norm3(sub3(actual, phase_reference)))
        if center_offset is not None and reference is not None:
            center_actual = sub3(actual, center_offset)
            center_errors.append(norm3(sub3(center_actual, reference)))
            if phase_reference is not None:
                center_phase_errors.append(norm3(sub3(center_actual, phase_reference)))

    orientation_block = {
        "p50": percentile(orientations, 50.0),
        "p95": percentile(orientations, 95.0),
        "max": max(orientations) if orientations else None,
    }
    orientation_equiv = orientation_equivalent(orientations, offsets)
    equiv_50mm = orientation_equiv.get("0.05") or orientation_equiv.get("0.050")
    orientation_50mm_p95 = finite_number(equiv_50mm.get("p95")) if isinstance(equiv_50mm, dict) else None
    phase_aligned_rms = rms(phase_errors)
    center_removed_rms = rms(center_errors)
    center_phase_removed_rms = rms(center_phase_errors)
    error_classification, classifications, reasons = classify_error_source(
        summary=summary,
        rms_error_m=base_rms,
        phase_aligned_rms_error_m=phase_aligned_rms,
        center_removed_rms_error_m=center_removed_rms,
        tail_ratio=tail_ratio,
        orientation_equiv_50mm_p95_m=orientation_50mm_p95,
    )
    return {
        "schema": "robotics_lab.circle_error_decomposition.v1",
        "sample_count": len(rows),
        "tool_offsets_m": offsets,
        "median_error_m": median_error,
        "mad_error_m": mad,
        "iqr_error_m": iqr,
        "tail_ratio": tail_ratio,
        "max_over_p95": max_over_p95,
        "center_error_m": finite_number(summary.get("fit_center_error_m")),
        "radius_error_m": finite_number(summary.get("radius_error_m")),
        "radius_gain": finite_number(summary.get("radius_gain")),
        "phase_lag_rad": phase_lag,
        "estimated_latency_ms": finite_number(summary.get("estimated_latency_ms")),
        "phase_aligned_rms_error_m": phase_aligned_rms,
        "center_removed_rms_error_m": center_removed_rms,
        "center_and_phase_removed_rms_error_m": center_phase_removed_rms,
        "phase_alignment_improvement_ratio": improvement_ratio(base_rms, phase_aligned_rms),
        "center_removal_improvement_ratio": improvement_ratio(base_rms, center_removed_rms),
        "center_and_phase_removal_improvement_ratio": improvement_ratio(base_rms, center_phase_removed_rms),
        "orientation_drift_rad": orientation_block,
        "orientation_error_position_equivalent_m": orientation_equiv,
        "orientation_position_equiv_50mm_m": orientation_50mm_p95,
        "error_classification": error_classification,
        "error_classifications": classifications,
        "classification_reasons": reasons,
        "cycles": cycle_metrics(
            rows,
            period_sec=period_sec,
            geometry=geometry,
            feedback_rows=feedback_rows or [],
        ),
    }
