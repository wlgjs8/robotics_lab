from __future__ import annotations

import math
from typing import Any

from .models import Pose6D


def _linear_step_meters(step_mm: float) -> float:
    return float(step_mm) * 0.001


def _angular_step_radians(step_deg: float) -> float:
    return math.radians(float(step_deg))


def _mount_pose_from_mounts(mounts: dict[str, Any], arm: str, fallback: tuple[float, float, float, float, float, float]) -> Pose6D:
    try:
        parsed = Pose6D.parse(mounts[arm]["base_pose_in_stand"])
        if parsed is not None:
            return parsed
        pose = mounts[arm]["base_pose_in_stand"]
        return Pose6D(
            float(pose.get("x", fallback[0])),
            float(pose.get("y", fallback[1])),
            float(pose.get("z", fallback[2])),
            float(pose.get("rx", fallback[3])),
            float(pose.get("ry", fallback[4])),
            float(pose.get("rz", fallback[5])),
        )
    except Exception:
        return Pose6D(*fallback)


def _pose6_from_mounts(mounts: dict[str, Any], arm: str, fallback: tuple[float, float, float, float, float, float]) -> tuple[float, float, float, float, float, float]:
    return _mount_pose_from_mounts(mounts, arm, fallback).as_tuple()


def _mount_position(mounts: dict[str, Any], arm: str, fallback: tuple[float, float, float]) -> tuple[float, float, float]:
    pose = _pose6_from_mounts(mounts, arm, (fallback[0], fallback[1], fallback[2], 0.0, 0.0, 0.0))
    return (pose[0], pose[1], pose[2])


def _pose_position(pose6: Pose6D | tuple[float, float, float, float, float, float]) -> tuple[float, float, float]:
    if isinstance(pose6, Pose6D):
        return (pose6.x, pose6.y, pose6.z)
    return (pose6[0], pose6[1], pose6[2])


def _rpy_to_wxyz(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def _xyzw_to_wxyz(quaternion_xyzw: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    qx, qy, qz, qw = quaternion_xyzw
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(norm) or norm <= 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    return (qw / norm, qx / norm, qy / norm, qz / norm)


def _wxyz_to_xyzw(wxyz: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    w, x, y, z = _normalize_wxyz(wxyz)
    return (x, y, z, w)


def _normalize_wxyz(wxyz: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    w, x, y, z = (float(wxyz[0]), float(wxyz[1]), float(wxyz[2]), float(wxyz[3]))
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if not math.isfinite(norm) or norm <= 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    return (w / norm, x / norm, y / norm, z / norm)


def _pose_orientation_wxyz(pose6: Pose6D | tuple[float, float, float, float, float, float]) -> tuple[float, float, float, float]:
    if isinstance(pose6, Pose6D):
        if pose6.quaternion_xyzw is not None:
            return _xyzw_to_wxyz(pose6.quaternion_xyzw)
        pose_tuple = pose6.as_tuple()
    else:
        pose_tuple = pose6
    return _rpy_to_wxyz(pose_tuple[3], pose_tuple[4], pose_tuple[5])


def _pose_wxyz(pose6: Pose6D | tuple[float, float, float, float, float, float]) -> tuple[float, float, float, float]:
    return _pose_orientation_wxyz(pose6)


def _wxyz_to_rpy(wxyz: tuple[float, float, float, float]) -> tuple[float, float, float]:
    w, x, y, z = wxyz
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (roll, pitch, yaw)


def _matrix_to_wxyz(matrix: Any) -> tuple[float, float, float, float]:
    m00 = float(matrix[0][0])
    m01 = float(matrix[0][1])
    m02 = float(matrix[0][2])
    m10 = float(matrix[1][0])
    m11 = float(matrix[1][1])
    m12 = float(matrix[1][2])
    m20 = float(matrix[2][0])
    m21 = float(matrix[2][1])
    m22 = float(matrix[2][2])
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (m21 - m12) / scale
        y = (m02 - m20) / scale
        z = (m10 - m01) / scale
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / scale
        x = 0.25 * scale
        y = (m01 + m10) / scale
        z = (m02 + m20) / scale
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / scale
        x = (m01 + m10) / scale
        y = 0.25 * scale
        z = (m12 + m21) / scale
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / scale
        x = (m02 + m20) / scale
        y = (m12 + m21) / scale
        z = 0.25 * scale
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 0.0 or not math.isfinite(norm):
        return (1.0, 0.0, 0.0, 0.0)
    return (w / norm, x / norm, y / norm, z / norm)


def _quat_to_matrix(wxyz: tuple[float, float, float, float]) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    w, x, y, z = _normalize_wxyz(wxyz)
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def _matmul3(
    left: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
    right: Any,
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    return tuple(
        tuple(sum(left[row][k] * float(right[k][col]) for k in range(3)) for col in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def _identity3() -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def _transpose3(
    matrix: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    return tuple(tuple(matrix[col][row] for col in range(3)) for row in range(3))  # type: ignore[return-value]


def _add_matrix3(
    left: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
    right: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    return tuple(tuple(left[row][col] + right[row][col] for col in range(3)) for row in range(3))  # type: ignore[return-value]


def _scale_matrix3(
    matrix: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
    scale: float,
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    return tuple(tuple(matrix[row][col] * scale for col in range(3)) for row in range(3))  # type: ignore[return-value]


def _skew3(vector: tuple[float, float, float]) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    x, y, z = vector
    return ((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0))


def _rotate_vec(
    matrix: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    return tuple(sum(matrix[row][k] * vector[k] for k in range(3)) for row in range(3))  # type: ignore[return-value]


def _add_vec3(left: tuple[float, float, float], right: tuple[float, float, float]) -> tuple[float, float, float]:
    return (left[0] + right[0], left[1] + right[1], left[2] + right[2])


def _delta_transform(
    delta: tuple[float, float, float, float, float, float],
) -> tuple[
    tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
    tuple[float, float, float],
]:
    v = (float(delta[0]), float(delta[1]), float(delta[2]))
    omega = (float(delta[3]), float(delta[4]), float(delta[5]))
    theta2 = omega[0] * omega[0] + omega[1] * omega[1] + omega[2] * omega[2]
    theta = math.sqrt(theta2)
    omega_hat = _skew3(omega)
    omega_hat2 = _matmul3(omega_hat, omega_hat)
    a = 1.0
    b = 0.5
    c = 1.0 / 6.0
    if theta > 1e-9:
        a = math.sin(theta) / theta
        b = (1.0 - math.cos(theta)) / theta2
        c = (theta - math.sin(theta)) / (theta2 * theta)
    identity = _identity3()
    rotation = _add_matrix3(_add_matrix3(identity, _scale_matrix3(omega_hat, a)), _scale_matrix3(omega_hat2, b))
    v_matrix = _add_matrix3(_add_matrix3(identity, _scale_matrix3(omega_hat, b)), _scale_matrix3(omega_hat2, c))
    return rotation, _rotate_vec(v_matrix, v)


def _multiply_transform(
    left: tuple[
        tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
        tuple[float, float, float],
    ],
    right: tuple[
        tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
        tuple[float, float, float],
    ],
) -> tuple[
    tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
    tuple[float, float, float],
]:
    left_rotation, left_translation = left
    right_rotation, right_translation = right
    return (
        _matmul3(left_rotation, right_rotation),
        _add_vec3(_rotate_vec(left_rotation, right_translation), left_translation),
    )


def _pose_transform(
    position: tuple[float, float, float],
    wxyz: tuple[float, float, float, float],
) -> tuple[
    tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
    tuple[float, float, float],
]:
    return _quat_to_matrix(wxyz), (float(position[0]), float(position[1]), float(position[2]))


def _transform_to_pose6(
    transform: tuple[
        tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
        tuple[float, float, float],
    ],
) -> tuple[tuple[float, float, float, float, float, float], tuple[float, float, float, float]]:
    rotation, position = transform
    wxyz = _matrix_to_wxyz(rotation)
    return _pose6_from_transform(position, wxyz), wxyz


def _rotation_vector_from_matrix(
    matrix: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
) -> tuple[float, float, float]:
    w, x, y, z = _matrix_to_wxyz(matrix)
    if w < 0.0:
        w, x, y, z = (-w, -x, -y, -z)
    sin_half = math.sqrt(x * x + y * y + z * z)
    if sin_half < 1e-12:
        return (0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(sin_half, w)
    scale = angle / sin_half
    return (x * scale, y * scale, z * scale)


def _se3_log_translation(
    relative_translation: tuple[float, float, float],
    rotation_vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    theta2 = rotation_vector[0] * rotation_vector[0] + rotation_vector[1] * rotation_vector[1] + rotation_vector[2] * rotation_vector[2]
    theta = math.sqrt(theta2)
    omega_hat = _skew3(rotation_vector)
    omega_hat2 = _matmul3(omega_hat, omega_hat)
    coefficient = 1.0 / 12.0
    if theta > 1e-9:
        coefficient = (1.0 / theta2) - ((1.0 + math.cos(theta)) / (2.0 * theta * math.sin(theta)))
    inverse_v = _add_matrix3(_add_matrix3(_identity3(), _scale_matrix3(omega_hat, -0.5)), _scale_matrix3(omega_hat2, coefficient))
    return _rotate_vec(inverse_v, relative_translation)


def _tcp_local_delta_from_target(
    current_tcp_stand: Pose6D,
    target_position: tuple[float, float, float],
    target_wxyz: tuple[float, float, float, float],
) -> tuple[float, float, float, float, float, float]:
    current_position = _pose_position(current_tcp_stand)
    current_rotation = _quat_to_matrix(_pose_orientation_wxyz(current_tcp_stand))
    target_rotation = _quat_to_matrix(target_wxyz)
    current_rotation_inv = _transpose3(current_rotation)
    translation_stand = (
        float(target_position[0]) - current_position[0],
        float(target_position[1]) - current_position[1],
        float(target_position[2]) - current_position[2],
    )
    translation_local = _rotate_vec(current_rotation_inv, translation_stand)
    rotation_local = _matmul3(current_rotation_inv, target_rotation)
    rotation_vector = _rotation_vector_from_matrix(rotation_local)
    return _se3_log_translation(translation_local, rotation_vector) + rotation_vector


def _pose6_from_state_arm(arm_raw: Any, key: str) -> tuple[float, float, float, float, float, float] | None:
    if not isinstance(arm_raw, dict):
        return None
    pose = arm_raw.get(key)
    try:
        if isinstance(pose, dict):
            values = (pose["x"], pose["y"], pose["z"], pose["rx"], pose["ry"], pose["rz"])
        elif isinstance(pose, list | tuple) and len(pose) == 6:
            values = tuple(pose)
        else:
            return None
        parsed = tuple(float(value) for value in values)
    except Exception:
        return None
    if not all(math.isfinite(value) for value in parsed):
        return None
    return parsed  # type: ignore[return-value]


def _tcp_pose_from_urdf(
    urdf_handle: Any,
    base_pose: Pose6D | tuple[float, float, float, float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]] | None:
    try:
        urdf = urdf_handle._urdf
        base_frame = urdf.scene.graph.base_frame
        transform = urdf.get_transform("tcp", base_frame)
        scale = float(getattr(urdf_handle, "_scale", 1.0))
        local_position = (
            float(transform[0][3]) * scale,
            float(transform[1][3]) * scale,
            float(transform[2][3]) * scale,
        )
        base_rotation = _quat_to_matrix(_pose_wxyz(base_pose))
        rotated_position = _rotate_vec(base_rotation, local_position)
        base_position = _pose_position(base_pose)
        position = (
            base_position[0] + rotated_position[0],
            base_position[1] + rotated_position[1],
            base_position[2] + rotated_position[2],
        )
        rotation = _matrix_to_wxyz(_matmul3(base_rotation, transform[:3, :3]))
        return position, rotation
    except Exception:
        return None


def _pose6_from_transform(
    position: tuple[float, float, float],
    wxyz: tuple[float, float, float, float],
) -> tuple[float, float, float, float, float, float]:
    roll, pitch, yaw = _wxyz_to_rpy(wxyz)
    return (float(position[0]), float(position[1]), float(position[2]), roll, pitch, yaw)
