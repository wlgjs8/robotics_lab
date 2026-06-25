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
    """Best-fit plane through >= 3 stand-frame points.

    Returns ``(point, normal)`` where ``point`` is the centroid (a point on the plane)
    and ``normal`` is the unit normal oriented UPWARD (``normal.z > 0``), matching the
    server's requirement that the allowed half-space opens upward.

    Raises ``ValueError`` when there are fewer than 3 points, any coordinate is
    non-finite, or the points are colinear / coincident (normal ill-defined).
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
    # SVD: the right-singular vector with the smallest singular value is the plane
    # normal; the two larger singular values measure the in-plane spread.
    _u, sv, vh = np.linalg.svd(centered, full_matrices=False)
    # Degeneracy guard: the second singular value must be well above zero, otherwise
    # the points are colinear or coincident and the plane normal is undefined.
    if sv[1] < 1e-6:
        raise ValueError("points are colinear or coincident; cannot fit a plane")
    normal = vh[-1]
    nrm = float(np.linalg.norm(normal))
    if nrm < 1e-9:
        raise ValueError("degenerate plane normal")
    normal = normal / nrm
    if normal[2] < 0.0:
        normal = -normal  # orient the normal upward (into the allowed half-space)
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
