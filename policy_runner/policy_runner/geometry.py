from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from .config import _load_mapping


@dataclass(frozen=True)
class GeometryStatus:
    calibration_id: str | None = None
    setup_id: str | None = None
    status: str = "unavailable"
    geometry_valid_for_real_policy: bool = False
    left_robot_mount_available: bool = False
    right_robot_mount_available: bool = False
    left_robot_mount_status: str = "unavailable"
    right_robot_mount_status: str = "unavailable"
    camera_intrinsics_status: dict[str, str] = field(default_factory=dict)
    camera_extrinsics_status: dict[str, str] = field(default_factory=dict)
    load_error: str | None = None

    @property
    def robot_mounts_available(self) -> bool:
        return self.left_robot_mount_available and self.right_robot_mount_available

    @property
    def camera_geometry_available(self) -> bool:
        if not self.camera_intrinsics_status or not self.camera_extrinsics_status:
            return False
        return all(_is_measured_status(value) for value in self.camera_intrinsics_status.values()) and all(
            _is_measured_status(value) for value in self.camera_extrinsics_status.values()
        )

    @classmethod
    def unavailable(cls, reason: str = "geometry_unavailable") -> "GeometryStatus":
        return cls(status="unavailable", load_error=reason)


def load_geometry_status(path: str | Path) -> GeometryStatus:
    registry_path = Path(path)
    try:
        raw = _load_mapping(registry_path)
    except FileNotFoundError:
        return GeometryStatus.unavailable("geometry_file_missing")
    except Exception as exc:
        return GeometryStatus.unavailable(f"geometry_file_invalid: {exc}")
    return geometry_status_from_mapping(raw)


def geometry_status_from_mapping(raw: dict[str, Any]) -> GeometryStatus:
    robot = _mapping(raw.get("robot"))
    left_mount = _mount_status(robot.get("T_stand_left_base"))
    right_mount = _mount_status(robot.get("T_stand_right_base"))
    cameras = _mapping(raw.get("cameras"))
    intrinsics: dict[str, str] = {}
    extrinsics: dict[str, str] = {}
    for name, value in cameras.items():
        if not isinstance(name, str):
            continue
        camera = _mapping(value)
        intrinsics[name] = _status_value(camera, "intrinsics_status", "unknown")
        if "extrinsics_status" in camera:
            extrinsics[name] = _status_value(camera, "extrinsics_status", "unknown")
        elif "hand_eye_status" in camera:
            extrinsics[name] = _status_value(camera, "hand_eye_status", "unknown")
        else:
            extrinsics[name] = "unknown"
    return GeometryStatus(
        calibration_id=_optional_string(raw.get("calibration_id")),
        setup_id=_optional_string(raw.get("setup_id")),
        status=_status_value(raw, "status", "unknown"),
        geometry_valid_for_real_policy=bool(raw.get("geometry_valid_for_real_policy", False)),
        left_robot_mount_available=left_mount[0],
        right_robot_mount_available=right_mount[0],
        left_robot_mount_status=left_mount[1],
        right_robot_mount_status=right_mount[1],
        camera_intrinsics_status=intrinsics,
        camera_extrinsics_status=extrinsics,
    )


def _mount_status(value: Any) -> tuple[bool, str]:
    mount = _mapping(value)
    status = _status_value(mount, "status", "unknown")
    xyz_rpy = mount.get("xyz_rpy")
    available = (
        isinstance(mount.get("parent"), str)
        and isinstance(mount.get("child"), str)
        and isinstance(xyz_rpy, list)
        and len(xyz_rpy) == 6
        and status not in {"missing", "unavailable", "unknown", "unmeasured", "invalid"}
    )
    return available, status


def _mapping(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _optional_string(value: Any) -> str | None:
    return value if isinstance(value, str) else None


def _status_value(raw: dict[str, Any], key: str, default: str) -> str:
    value = raw.get(key, default)
    return str(value).strip().lower() if value is not None else default


def _is_measured_status(status: str) -> bool:
    return status.strip().lower() in {"measured", "calibrated", "accepted"}
