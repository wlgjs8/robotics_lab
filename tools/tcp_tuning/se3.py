from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


QUAT_NORM_EPS = 1e-12


def quat_to_rotation(q_xyzw) -> Rotation:
    """Convert a scipy-order xyzw quaternion to a Rotation after normalization."""

    return Rotation.from_quat(_normalize_quat(q_xyzw))


def rotation_to_quat(rotation: Rotation, ref=None) -> np.ndarray:
    """Convert a Rotation to a sign-continuous scipy-order xyzw quaternion."""

    return quat_canonical(rotation.as_quat(), ref=ref)


def quat_canonical(q_xyzw, ref=None) -> np.ndarray:
    """Return a normalized xyzw quaternion, flipped so dot(ref, q) is non-negative."""

    q = _normalize_quat(q_xyzw)
    if ref is not None:
        ref_q = _normalize_quat(ref)
        if float(np.dot(ref_q, q)) < 0.0:
            q = -q
    elif q[3] < 0.0:
        q = -q
    return q


def slerp(q0_xyzw, q1_xyzw, u: float) -> np.ndarray:
    """Slerp between xyzw quaternions, enforcing shortest-path sign continuity."""

    u_clamped = float(np.clip(u, 0.0, 1.0))
    q0 = quat_canonical(q0_xyzw)
    q1 = quat_canonical(q1_xyzw, ref=q0)
    if u_clamped == 0.0:
        return q0.copy()
    if u_clamped == 1.0:
        return q1.copy()
    interp = Slerp([0.0, 1.0], Rotation.from_quat(np.stack([q0, q1], axis=0)))
    return quat_canonical(interp([u_clamped]).as_quat()[0], ref=q0)


def so3_log(R) -> np.ndarray:
    """Return the SO(3) log vector for a Rotation or rotation-like input."""

    rotation = R if isinstance(R, Rotation) else Rotation.from_matrix(np.asarray(R, dtype=np.float64))
    return rotation.as_rotvec().astype(np.float64)


def so3_exp(rotvec) -> Rotation:
    """Return the Rotation represented by an SO(3) rotation vector."""

    return Rotation.from_rotvec(np.asarray(rotvec, dtype=np.float64).reshape(3))


def foh_pose(t, t0, p0, q0, t1, p1, q1) -> tuple[np.ndarray, np.ndarray]:
    """Timestamp-aware SE(3) first-order hold: lerp position and slerp orientation."""

    start = float(t0)
    stop = float(t1)
    if stop <= start:
        u = 0.0
    else:
        u = (float(t) - start) / (stop - start)
    u = float(np.clip(u, 0.0, 1.0))
    p = (1.0 - u) * np.asarray(p0, dtype=np.float64).reshape(3) + u * np.asarray(p1, dtype=np.float64).reshape(3)
    return p.astype(np.float64), slerp(q0, q1, u)


def twist_from_poses(p0, q0, p1, q1, dt) -> tuple[np.ndarray, np.ndarray]:
    """Finite-difference twist from pose0 to pose1.

    Linear velocity is expressed in the episode pose frame. Angular velocity is
    the rotation-vector rate of R1 * R0.inv(), also expressed in that pose frame.
    Quaternions are scipy xyzw and are sign-continuized before differencing.
    """

    dt_sec = float(dt)
    if dt_sec <= 0.0 or not np.isfinite(dt_sec):
        raise ValueError("dt must be positive and finite")
    p0_arr = np.asarray(p0, dtype=np.float64).reshape(3)
    p1_arr = np.asarray(p1, dtype=np.float64).reshape(3)
    q0_arr = quat_canonical(q0)
    q1_arr = quat_canonical(q1, ref=q0_arr)
    v = (p1_arr - p0_arr) / dt_sec
    delta = quat_to_rotation(q1_arr) * quat_to_rotation(q0_arr).inv()
    w = delta.as_rotvec() / dt_sec
    return v.astype(np.float64), w.astype(np.float64)


def _normalize_quat(q_xyzw) -> np.ndarray:
    q = np.asarray(q_xyzw, dtype=np.float64).reshape(4)
    norm = float(np.linalg.norm(q))
    if not np.isfinite(norm) or norm <= QUAT_NORM_EPS:
        raise ValueError("quaternion must have positive finite norm")
    return (q / norm).astype(np.float64)
