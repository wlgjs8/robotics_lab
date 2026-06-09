from __future__ import annotations

import json
import math
import socket
import time
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from typing import Any, Protocol

from policy_runner.action_sources.tcp_delta import (
    cartesian_action_requirements,
    clamp_tcp_delta,
    tcp_pose_target_stand_intent,
)
from policy_runner.robot_state_client import StateSnapshot, parse_udp_endpoint
from policy_runner.servo_command_client import CommandIntent


GRIPPER_OFFSET = (0.172, 0.0, -0.076)
IDENTITY_R_ALIGN = (
    1.0,
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    0.0,
    1.0,
)


@dataclass(frozen=True)
class UmiSample:
    pose_xyzw: tuple[float, float, float, float, float, float, float]
    gripper: float
    deadman: bool
    monotonic: float


class UmiPoseReader(Protocol):
    def read(self) -> UmiSample | None:
        ...

    def close(self) -> None:
        ...


@dataclass(frozen=True)
class _Transform:
    translation: tuple[float, float, float]
    rotation: tuple[float, ...]


@dataclass
class _ArmTeleopState:
    arm_init: _Transform | None = None
    pika_init: _Transform | None = None
    previous_target: tuple[float, ...] | None = None
    last_sample: UmiSample | None = None
    was_armed: bool = False


class UmiDualCartesianActionSource:
    requirements = cartesian_action_requirements(allow_rbpodo_controller_simulation=True)

    def __init__(
        self,
        left_reader: UmiPoseReader,
        right_reader: UmiPoseReader,
        *,
        max_linear_step_m: float = 0.005,
        max_angular_step_rad: float = 0.04,
        gripper_offset: Sequence[float] = GRIPPER_OFFSET,
        r_align: Sequence[float] = IDENTITY_R_ALIGN,
        workspace_bounds: Mapping[str, Sequence[float]] | Sequence[float] | None = None,
        sample_hold_timeout_sec: float = 0.05,
        timeout_sec: float = 0.05,
    ):
        if max_linear_step_m < 0.0:
            raise ValueError("max_linear_step_m must be non-negative")
        if max_angular_step_rad < 0.0:
            raise ValueError("max_angular_step_rad must be non-negative")
        if sample_hold_timeout_sec <= 0.0:
            raise ValueError("sample_hold_timeout_sec must be positive")
        self.left_reader = left_reader
        self.right_reader = right_reader
        self.max_linear_step_m = float(max_linear_step_m)
        self.max_angular_step_rad = float(max_angular_step_rad)
        self.gripper_offset = _tuple3(gripper_offset, "gripper_offset")
        self.r_align = _matrix3(r_align, "r_align")
        self.workspace_bounds = _workspace_bounds(workspace_bounds)
        self.sample_hold_timeout_sec = float(sample_hold_timeout_sec)
        self.timeout_sec = float(timeout_sec)
        self._left = _ArmTeleopState()
        self._right = _ArmTeleopState()

    def next_intent(self, snapshot: StateSnapshot, now_monotonic: float) -> CommandIntent | None:
        left_pose, left_gripper, left_changed = self._target_for_side(
            "left",
            self.left_reader,
            self._left,
            snapshot,
            now_monotonic,
        )
        right_pose, right_gripper, right_changed = self._target_for_side(
            "right",
            self.right_reader,
            self._right,
            snapshot,
            now_monotonic,
        )
        if not left_changed and not right_changed:
            return None
        return tcp_pose_target_stand_intent(
            left=left_pose,
            right=right_pose,
            left_gripper=left_gripper,
            right_gripper=right_gripper,
            timeout_sec=self.timeout_sec,
        )

    def close(self) -> None:
        self.left_reader.close()
        self.right_reader.close()

    def _target_for_side(
        self,
        side: str,
        reader: UmiPoseReader,
        state: _ArmTeleopState,
        snapshot: StateSnapshot,
        now_monotonic: float,
    ) -> tuple[tuple[float, ...] | None, float | None, bool]:
        sample = reader.read()
        if sample is None:
            sample = state.last_sample
            if sample is None or now_monotonic - sample.monotonic > self.sample_hold_timeout_sec:
                if state.was_armed:
                    _clear_latches(state)
                    return None, None, True
                _clear_latches(state)
                return None, None, False
        else:
            state.last_sample = sample

        if now_monotonic - sample.monotonic > self.sample_hold_timeout_sec:
            if state.was_armed:
                _clear_latches(state)
                return None, None, True
            _clear_latches(state)
            return None, None, False

        if not sample.deadman:
            if state.was_armed:
                _clear_latches(state)
                return None, _gripper_percent(sample.gripper), True
            _clear_latches(state)
            return None, None, False

        pika_now = _tracker_transform(sample, self.gripper_offset, self.r_align)
        if not state.was_armed:
            arm_init = _tcp_stand_transform(snapshot, side)
            if arm_init is None:
                _clear_latches(state)
                return None, None, False
            state.arm_init = arm_init
            state.pika_init = pika_now
            state.previous_target = _transform_to_pose6(arm_init)
            state.was_armed = True

        assert state.arm_init is not None
        assert state.pika_init is not None
        target = _compose(state.arm_init, _compose(_inverse(state.pika_init), pika_now))
        pose6 = _clamp_workspace(_transform_to_pose6(target), self.workspace_bounds)
        pose6 = self._clamp_against_previous(state, pose6)
        state.previous_target = pose6
        return pose6, _gripper_percent(sample.gripper), True

    def _clamp_against_previous(
        self,
        state: _ArmTeleopState,
        pose6: tuple[float, ...],
    ) -> tuple[float, ...]:
        previous = state.previous_target
        if previous is None:
            return pose6
        delta = (
            pose6[0] - previous[0],
            pose6[1] - previous[1],
            pose6[2] - previous[2],
            _angle_diff(pose6[3], previous[3]),
            _angle_diff(pose6[4], previous[4]),
            _angle_diff(pose6[5], previous[5]),
        )
        clamped = clamp_tcp_delta(delta, self.max_linear_step_m, self.max_angular_step_rad)
        return (
            previous[0] + clamped[0],
            previous[1] + clamped[1],
            previous[2] + clamped[2],
            _wrap_pi(previous[3] + clamped[3]),
            _wrap_pi(previous[4] + clamped[4]),
            _wrap_pi(previous[5] + clamped[5]),
        )


class MockUmiPoseReader:
    """Deterministic UMI reader for hardware-free policy_runner tests and demos."""

    def __init__(self, script: str | Iterable[Mapping[str, Any] | UmiSample | None]):
        self._samples = _script_to_samples(script)
        self.closed = False

    def read(self) -> UmiSample | None:
        if not self._samples:
            return None
        return self._samples.pop(0)

    def close(self) -> None:
        self.closed = True


class UdpUmiPoseReader:
    """UDP JSON reader for one side of the Windows SteamVR UMI publisher schema."""

    _MAX_DRAIN_PACKETS = 64

    def __init__(
        self,
        endpoint: str,
        side: str,
        *,
        socket_factory: Any = socket.socket,
        monotonic_fn: Any = time.monotonic,
    ):
        if side not in {"left", "right"}:
            raise ValueError("side must be left or right")
        self.endpoint = endpoint
        self.side = side
        self._socket_factory = socket_factory
        self._monotonic_fn = monotonic_fn
        self._socket: socket.socket | None = None
        self._latest: UmiSample | None = None

    def read(self) -> UmiSample | None:
        self._open()
        assert self._socket is not None
        for _ in range(self._MAX_DRAIN_PACKETS):
            try:
                data, _address = self._socket.recvfrom(65536)
            except BlockingIOError:
                break
            self._latest = _sample_from_udp_packet(data, self.side, self._monotonic_fn)
        return self._latest

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None

    def _open(self) -> None:
        if self._socket is not None:
            return
        endpoint = parse_udp_endpoint(self.endpoint)
        sock = self._socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((endpoint.host, endpoint.port))
        sock.setblocking(False)
        self._socket = sock


_NAMED_SCRIPTS: dict[str, tuple[Mapping[str, Any] | None, ...]] = {
    "pgmode_umi_smoke": (
        {
            "pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            "gripper": 0.0,
            "deadman": False,
            "monotonic": 0.00,
        },
        {
            "pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            "gripper": 0.25,
            "deadman": True,
            "monotonic": 0.01,
        },
        {
            "pose": [0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            "gripper": 0.50,
            "deadman": True,
            "monotonic": 0.02,
        },
        {
            "pose": [0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            "gripper": 50.0,
            "deadman": False,
            "monotonic": 0.03,
        },
        None,
    ),
}


def _clear_latches(state: _ArmTeleopState) -> None:
    state.arm_init = None
    state.pika_init = None
    state.previous_target = None
    state.last_sample = None
    state.was_armed = False


def _script_to_samples(script: str | Iterable[Mapping[str, Any] | UmiSample | None]) -> list[UmiSample | None]:
    if isinstance(script, str):
        try:
            entries = _NAMED_SCRIPTS[script]
        except KeyError as exc:
            known = ", ".join(sorted(_NAMED_SCRIPTS))
            raise ValueError(f"unknown UMI mock_script {script!r}; known scripts: {known}") from exc
    else:
        entries = tuple(script)
    return [_sample_from_mapping(entry, index) for index, entry in enumerate(entries)]


def _sample_from_udp_packet(data: bytes, side: str, monotonic_fn: Any) -> UmiSample | None:
    raw = json.loads(data.decode("utf-8"))
    if not isinstance(raw, Mapping):
        raise ValueError("UMI UDP packet must be a JSON object")
    side_raw = raw.get(side)
    if side_raw is None:
        return None
    if not isinstance(side_raw, Mapping):
        raise ValueError(f"UMI UDP packet {side} field must be an object")
    entry = dict(side_raw)
    entry.setdefault("monotonic", raw.get("t", monotonic_fn()))
    return _sample_from_mapping(entry, 0)


def _sample_from_mapping(entry: Mapping[str, Any] | UmiSample | None, index: int) -> UmiSample | None:
    if entry is None:
        return None
    if isinstance(entry, UmiSample):
        return entry
    pose = entry.get("pose", entry.get("pose_xyzw"))
    if not isinstance(pose, (list, tuple)) or len(pose) != 7:
        raise ValueError("UMI sample pose must be [x,y,z,qx,qy,qz,qw]")
    return UmiSample(
        pose_xyzw=tuple(float(value) for value in pose),  # type: ignore[arg-type]
        gripper=float(entry.get("gripper", 0.0)),
        deadman=bool(entry.get("deadman", False)),
        monotonic=float(entry.get("monotonic", entry.get("timestamp_monotonic", index * 0.01))),
    )


def _tracker_transform(
    sample: UmiSample,
    gripper_offset: tuple[float, float, float],
    r_align: tuple[float, ...],
) -> _Transform:
    x, y, z, qx, qy, qz, qw = sample.pose_xyzw
    rotation = _mat_mul(_quat_to_matrix((qx, qy, qz, qw)), r_align)
    offset_world = _mat_vec(rotation, gripper_offset)
    return _Transform(
        (x + offset_world[0], y + offset_world[1], z + offset_world[2]),
        rotation,
    )


def _tcp_stand_transform(snapshot: StateSnapshot, side: str) -> _Transform | None:
    arm = snapshot.payload.get(side)
    if not isinstance(arm, Mapping):
        return None
    raw = arm.get("tcp_stand") or arm.get("tcp_actual_stand")
    if not isinstance(raw, Mapping):
        return None
    translation = (
        float(raw.get("x", 0.0)),
        float(raw.get("y", 0.0)),
        float(raw.get("z", 0.0)),
    )
    quat = raw.get("quaternion_xyzw")
    if isinstance(quat, (list, tuple)) and len(quat) == 4:
        rotation = _quat_to_matrix(tuple(float(value) for value in quat))
    elif all(key in raw for key in ("qx", "qy", "qz", "qw")):
        rotation = _quat_to_matrix(
            (
                float(raw.get("qx", 0.0)),
                float(raw.get("qy", 0.0)),
                float(raw.get("qz", 0.0)),
                float(raw.get("qw", 1.0)),
            )
        )
    else:
        rotation = _rpy_to_matrix(
            float(raw.get("rx", 0.0)),
            float(raw.get("ry", 0.0)),
            float(raw.get("rz", 0.0)),
        )
    return _Transform(translation, rotation)


def _compose(a: _Transform, b: _Transform) -> _Transform:
    rotated = _mat_vec(a.rotation, b.translation)
    return _Transform(
        (
            a.translation[0] + rotated[0],
            a.translation[1] + rotated[1],
            a.translation[2] + rotated[2],
        ),
        _mat_mul(a.rotation, b.rotation),
    )


def _inverse(transform: _Transform) -> _Transform:
    rotation_t = _transpose(transform.rotation)
    inv_t = _mat_vec(rotation_t, (-transform.translation[0], -transform.translation[1], -transform.translation[2]))
    return _Transform(inv_t, rotation_t)


def _transform_to_pose6(transform: _Transform) -> tuple[float, ...]:
    roll, pitch, yaw = _matrix_to_rpy(transform.rotation)
    return (
        transform.translation[0],
        transform.translation[1],
        transform.translation[2],
        roll,
        pitch,
        yaw,
    )


def _quat_to_matrix(q: Sequence[float]) -> tuple[float, ...]:
    if len(q) != 4:
        raise ValueError("quaternion must contain 4 values")
    qx, qy, qz, qw = (float(value) for value in q)
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        raise ValueError("quaternion must be nonzero")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    return (
        1.0 - 2.0 * (qy * qy + qz * qz),
        2.0 * (qx * qy - qz * qw),
        2.0 * (qx * qz + qy * qw),
        2.0 * (qx * qy + qz * qw),
        1.0 - 2.0 * (qx * qx + qz * qz),
        2.0 * (qy * qz - qx * qw),
        2.0 * (qx * qz - qy * qw),
        2.0 * (qy * qz + qx * qw),
        1.0 - 2.0 * (qx * qx + qy * qy),
    )


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> tuple[float, ...]:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        cy * cp,
        cy * sp * sr - sy * cr,
        cy * sp * cr + sy * sr,
        sy * cp,
        sy * sp * sr + cy * cr,
        sy * sp * cr - cy * sr,
        -sp,
        cp * sr,
        cp * cr,
    )


def _matrix_to_rpy(m: Sequence[float]) -> tuple[float, float, float]:
    sy = -float(m[6])
    sy = max(-1.0, min(1.0, sy))
    pitch = math.asin(sy)
    cp = math.cos(pitch)
    if abs(cp) > 1e-9:
        roll = math.atan2(float(m[7]), float(m[8]))
        yaw = math.atan2(float(m[3]), float(m[0]))
    else:
        roll = 0.0
        yaw = math.atan2(-float(m[1]), float(m[4]))
    return (_wrap_pi(roll), _wrap_pi(pitch), _wrap_pi(yaw))


def _mat_mul(a: Sequence[float], b: Sequence[float]) -> tuple[float, ...]:
    return tuple(
        sum(float(a[row * 3 + k]) * float(b[k * 3 + col]) for k in range(3))
        for row in range(3)
        for col in range(3)
    )


def _mat_vec(m: Sequence[float], v: Sequence[float]) -> tuple[float, float, float]:
    return (
        float(m[0]) * float(v[0]) + float(m[1]) * float(v[1]) + float(m[2]) * float(v[2]),
        float(m[3]) * float(v[0]) + float(m[4]) * float(v[1]) + float(m[5]) * float(v[2]),
        float(m[6]) * float(v[0]) + float(m[7]) * float(v[1]) + float(m[8]) * float(v[2]),
    )


def _transpose(m: Sequence[float]) -> tuple[float, ...]:
    return (
        float(m[0]),
        float(m[3]),
        float(m[6]),
        float(m[1]),
        float(m[4]),
        float(m[7]),
        float(m[2]),
        float(m[5]),
        float(m[8]),
    )


def _tuple3(value: Sequence[float], label: str) -> tuple[float, float, float]:
    if len(value) != 3:
        raise ValueError(f"{label} must contain 3 values")
    return (float(value[0]), float(value[1]), float(value[2]))


def _matrix3(value: Sequence[float], label: str) -> tuple[float, ...]:
    if len(value) == 9:
        return tuple(float(v) for v in value)
    if len(value) == 3:
        return _rpy_to_matrix(float(value[0]), float(value[1]), float(value[2]))
    raise ValueError(f"{label} must contain 3 RPY values or 9 matrix values")


def _workspace_bounds(
    raw: Mapping[str, Sequence[float]] | Sequence[float] | None,
) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float]] | None:
    if raw is None:
        return None
    if isinstance(raw, Mapping):
        return (
            _bounds_pair(raw.get("x", (-math.inf, math.inf)), "workspace_bounds.x"),
            _bounds_pair(raw.get("y", (-math.inf, math.inf)), "workspace_bounds.y"),
            _bounds_pair(raw.get("z", (-math.inf, math.inf)), "workspace_bounds.z"),
        )
    if len(raw) != 6:
        raise ValueError("workspace_bounds must be [xmin,xmax,ymin,ymax,zmin,zmax]")
    return (
        (float(raw[0]), float(raw[1])),
        (float(raw[2]), float(raw[3])),
        (float(raw[4]), float(raw[5])),
    )


def _bounds_pair(value: Sequence[float], label: str) -> tuple[float, float]:
    if len(value) != 2:
        raise ValueError(f"{label} must contain [min,max]")
    lower, upper = float(value[0]), float(value[1])
    if lower > upper:
        raise ValueError(f"{label} min must be <= max")
    return lower, upper


def _clamp_workspace(
    pose6: tuple[float, ...],
    bounds: tuple[tuple[float, float], tuple[float, float], tuple[float, float]] | None,
) -> tuple[float, ...]:
    if bounds is None:
        return pose6
    return (
        max(bounds[0][0], min(bounds[0][1], pose6[0])),
        max(bounds[1][0], min(bounds[1][1], pose6[1])),
        max(bounds[2][0], min(bounds[2][1], pose6[2])),
        pose6[3],
        pose6[4],
        pose6[5],
    )


def _gripper_percent(value: float) -> float:
    value = float(value)
    if 0.0 <= value <= 1.0:
        value *= 100.0
    return max(0.0, min(100.0, value))


def _angle_diff(value: float, previous: float) -> float:
    return _wrap_pi(value - previous)


def _wrap_pi(value: float) -> float:
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi
