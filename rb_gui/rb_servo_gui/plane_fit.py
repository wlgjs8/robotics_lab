"""Least-squares plane fitting for the User Safety Floor.

The viser GUI captures >= 3 stand-frame floor-contact points (alternating arms) and
fits a best-fit plane through them; the plane's point + upward unit normal are pushed
to the server as a SetUserSafetyFloorPlane command. Kept as a pure, numpy-only helper
so it is unit-testable without viser or a live state stream.
"""

from __future__ import annotations

from typing import Sequence

import numpy as np


def fit_plane(
    points: Sequence[Sequence[float]],
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Best-fit floor plane through >= 3 stand-frame points.

    Returns ``(point, normal)`` where ``point`` is the centroid (which the fitted
    plane passes through) and ``normal`` is the unit normal oriented UPWARD
    (``normal.z > 0``), matching the server's requirement that the allowed half-space
    opens upward.

    The floor is modelled as a HEIGHT FIELD ``z = a*x + b*y + c`` and fit by ordinary
    least squares (minimising vertical distance), not by total-least-squares/SVD. This
    matters for robustness: ``np.linalg.lstsq`` returns the minimum-norm solution even
    when the points do not span both in-plane directions (e.g. captured roughly along a
    line), assuming ZERO tilt in any unconstrained direction instead of failing. A
    near-vertical normal can never result. Only genuinely useless input is rejected:
    fewer than 3 points, non-finite coordinates, or all points at the same (x, y)
    (no spatial spread at all — move the arm between captures).
    """
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError("points must be an (N, 3) array of [x, y, z]")
    if pts.shape[0] < 3:
        raise ValueError("need at least 3 points to fit a plane")
    if not np.all(np.isfinite(pts)):
        raise ValueError("points must be finite")
    centroid = pts.mean(axis=0)
    centered = pts - centroid
    a_mat = centered[:, :2]   # x, y relative to centroid
    bz = centered[:, 2]       # z relative to centroid
    # The only truly unfittable case is no spatial footprint at all: every point at
    # the same (x, y), so the height field z=f(x,y) is a single column. Decide this on
    # the PHYSICAL (x, y) extent (not matrix_rank, whose tolerance is brittle) and
    # report the measured span so a duplicate-capture bug is obvious vs a real spread.
    # A colinear capture (spread along one axis only) is fine — lstsq returns the
    # minimum-norm fit (tilt along the captured axis, zero perpendicular).
    xy_span = pts[:, :2].max(axis=0) - pts[:, :2].min(axis=0)
    if xy_span[0] < 1e-4 and xy_span[1] < 1e-4:
        raise ValueError(
            f"captured points span only {xy_span[0] * 1000:.2f}x{xy_span[1] * 1000:.2f} mm "
            "in (x, y); move the arm between captures"
        )
    # Minimum-norm least-squares slopes a, b (rcond=None -> rank-aware: a colinear
    # capture yields tilt only along the captured direction, zero perpendicular).
    coef, _res, _rank, _sv = np.linalg.lstsq(a_mat, bz, rcond=None)
    a, b = float(coef[0]), float(coef[1])
    normal = np.array([-a, -b, 1.0], dtype=float)
    normal = normal / float(np.linalg.norm(normal))  # n.z > 0 by construction
    return (
        (float(centroid[0]), float(centroid[1]), float(centroid[2])),
        (float(normal[0]), float(normal[1]), float(normal[2])),
    )


def tilt_deg(normal: Sequence[float]) -> float:
    """Angle (degrees) of a normal from vertical (+z). Mirrors the server's
    max_tilt_deg check so the GUI can warn before sending a too-tilted plane."""
    n = np.asarray(normal, dtype=float)
    nrm = float(np.linalg.norm(n))
    if nrm < 1e-9:
        raise ValueError("degenerate normal")
    cos = float(np.clip(n[2] / nrm, -1.0, 1.0))
    return float(np.degrees(np.arccos(cos)))
