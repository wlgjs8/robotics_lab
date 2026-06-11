from __future__ import annotations

import json
import math
import socket
import time
from collections import deque
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


# Default: no receiver-side offset — the pika publisher already streams the
# official gripper-tip pose (pika_sdk T_raw·R_corr·Trans(0.172,0,-0.076)).
# Legacy raw-tracker wire pairs with gripper_offset (0.172, 0.0, -0.076) in
# config (see rbpodo_pgmode_umi_live.example.yaml fallback block).
GRIPPER_OFFSET = (0.0, 0.0, 0.0)
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
    filtered_target: tuple[float, ...] | None = None
    last_filter_monotonic: float | None = None


class _PoseMovingAverage:
    """Moving average over the last N distinct tracker SAMPLES (not 500 Hz
    ticks): position is the arithmetic mean, rotation is the hemisphere-aligned
    normalized quaternion mean. The buffer keeps filling while the deadman is
    released so the window is already warm at clutch engage; pika_init latches
    from the same filtered stream, so there is no engage offset."""

    def __init__(self, window: int):
        self.window = int(window)
        self._positions: deque[tuple[float, float, float]] = deque(maxlen=max(1, self.window))
        self._quats: deque[tuple[float, float, float, float]] = deque(maxlen=max(1, self.window))
        self._last_monotonic: float | None = None

    def filter(self, sample: UmiSample) -> UmiSample:
        if self.window <= 1:
            return sample
        if self._last_monotonic is None or sample.monotonic != self._last_monotonic:
            self._last_monotonic = sample.monotonic
            x, y, z, qx, qy, qz, qw = sample.pose_xyzw
            self._positions.append((x, y, z))
            self._quats.append((qx, qy, qz, qw))
        n = len(self._positions)
        mean_position = (
            sum(p[0] for p in self._positions) / n,
            sum(p[1] for p in self._positions) / n,
            sum(p[2] for p in self._positions) / n,
        )
        mean_quat = _average_quaternions(self._quats)
        return UmiSample(
            (*mean_position, *mean_quat),
            sample.gripper,
            sample.deadman,
            sample.monotonic,
        )


class UmiDualCartesianActionSource:
    requirements = cartesian_action_requirements(allow_rbpodo_controller_simulation=True)

    def __init__(
        self,
        left_reader: UmiPoseReader,
        right_reader: UmiPoseReader,
        *,
        max_linear_step_m: float = 0.005,
        max_angular_step_rad: float = 0.04,
        input_moving_average_window: int = 1,
        target_lpf_tau_sec: float = 0.0,
        deadband_linear_m: float = 0.0,
        deadband_angular_rad: float = 0.0,
        linear_axis_signs: Sequence[float] = (1.0, 1.0, 1.0),
        angular_axis_signs: Sequence[float] = (1.0, 1.0, 1.0),
        delta_frame: str = "tool",
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
        if int(input_moving_average_window) < 0:
            raise ValueError("input_moving_average_window must be non-negative")
        if target_lpf_tau_sec < 0.0:
            raise ValueError("target_lpf_tau_sec must be non-negative")
        if deadband_linear_m < 0.0:
            raise ValueError("deadband_linear_m must be non-negative")
        if deadband_angular_rad < 0.0:
            raise ValueError("deadband_angular_rad must be non-negative")
        if sample_hold_timeout_sec <= 0.0:
            raise ValueError("sample_hold_timeout_sec must be positive")
        self.left_reader = left_reader
        self.right_reader = right_reader
        self.max_linear_step_m = float(max_linear_step_m)
        self.max_angular_step_rad = float(max_angular_step_rad)
        self.target_lpf_tau_sec = float(target_lpf_tau_sec)
        self.deadband_linear_m = float(deadband_linear_m)
        self.deadband_angular_rad = float(deadband_angular_rad)
        self.linear_axis_signs = _signs3(linear_axis_signs, "linear_axis_signs")
        self.angular_axis_signs = _signs3(angular_axis_signs, "angular_axis_signs")
        if delta_frame not in ("tool", "world"):
            raise ValueError("delta_frame must be 'tool' or 'world'")
        self.delta_frame = delta_frame
        self.gripper_offset = _tuple3(gripper_offset, "gripper_offset")
        self.r_align = _matrix3(r_align, "r_align")
        self.workspace_bounds = _workspace_bounds(workspace_bounds)
        self.sample_hold_timeout_sec = float(sample_hold_timeout_sec)
        self.timeout_sec = float(timeout_sec)
        self.input_moving_average_window = int(input_moving_average_window)
        self._left = _ArmTeleopState()
        self._right = _ArmTeleopState()
        self._left_ma = _PoseMovingAverage(self.input_moving_average_window)
        self._right_ma = _PoseMovingAverage(self.input_moving_average_window)

    def next_intent(self, snapshot: StateSnapshot, now_monotonic: float) -> CommandIntent | None:
        left_pose, left_gripper, left_changed = self._target_for_side(
            "left",
            self.left_reader,
            self._left,
            self._left_ma,
            snapshot,
            now_monotonic,
        )
        right_pose, right_gripper, right_changed = self._target_for_side(
            "right",
            self.right_reader,
            self._right,
            self._right_ma,
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
        moving_average: _PoseMovingAverage,
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

        # Input conditioning before any latch/compose math: the moving average
        # buffer also fills while the deadman is released (warm at engage).
        sample = moving_average.filter(sample)

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
        if self.delta_frame == "world":
            target = self._world_frame_target(state.arm_init, state.pika_init, pika_now)
        else:
            delta = self._apply_axis_signs(_compose(_inverse(state.pika_init), pika_now))
            target = _compose(state.arm_init, delta)
        pose6 = _clamp_workspace(_transform_to_pose6(target), self.workspace_bounds)
        pose6 = self._filter_target(state, pose6, now_monotonic)
        pose6 = self._clamp_against_previous(state, pose6)
        state.previous_target = pose6
        return pose6, _gripper_percent(sample.gripper), True

    def _world_frame_target(
        self,
        arm_init: _Transform,
        pika_init: _Transform,
        pika_now: _Transform,
    ) -> _Transform:
        """Map the tracker delta in the tracker WORLD frame onto the stand frame.

        Unlike the 'tool' frame composition (arm_init ∘ pika_init⁻¹ ∘ pika_now),
        the world delta does not pass through the latched tracker/TCP body
        orientations, so a horizontal hand motion stays horizontal at the robot
        regardless of how the tool or the TCP was oriented at latch time.
        Assumes the tracker world vertical matches the stand vertical; in-plane
        yaw alignment is absorbed by linear_axis_signs (or operator placement).
        Axis signs are applied along world/stand axes. r_align cancels out of
        the rotation delta in this mode (it only shifts the tip via
        gripper_offset).
        """
        sx, sy, sz = self.linear_axis_signs
        translation = (
            arm_init.translation[0] + sx * (pika_now.translation[0] - pika_init.translation[0]),
            arm_init.translation[1] + sy * (pika_now.translation[1] - pika_init.translation[1]),
            arm_init.translation[2] + sz * (pika_now.translation[2] - pika_init.translation[2]),
        )
        # World-frame relative rotation, mirrored per axis via the quaternion
        # vector part, then applied as a world-frame (left) rotation offset.
        delta_rotation = _mat_mul(pika_now.rotation, _transpose(pika_init.rotation))
        ax, ay, az = self.angular_axis_signs
        qx, qy, qz, qw = _matrix_to_quat(delta_rotation)
        mirrored = _quat_to_matrix((ax * qx, ay * qy, az * qz, qw))
        return _Transform(translation, _mat_mul(mirrored, arm_init.rotation))

    def _apply_axis_signs(self, delta: _Transform) -> _Transform:
        """Mirror the latched relative delta per axis.

        Linear signs flip the delta translation componentwise; angular signs
        flip the rotation-axis components (via the quaternion vector part, so
        the angle is preserved and there is no RPY singularity). This is more
        general than an r_align conjugation: e.g. flipping x/y translation
        while flipping all of roll/pitch/yaw is not expressible as any rigid
        tool-frame alignment.
        """
        if self.linear_axis_signs == (1.0, 1.0, 1.0) and self.angular_axis_signs == (1.0, 1.0, 1.0):
            return delta
        sx, sy, sz = self.linear_axis_signs
        ax, ay, az = self.angular_axis_signs
        qx, qy, qz, qw = _matrix_to_quat(delta.rotation)
        return _Transform(
            (
                sx * delta.translation[0],
                sy * delta.translation[1],
                sz * delta.translation[2],
            ),
            _quat_to_matrix((ax * qx, ay * qy, az * qz, qw)),
        )

    def _filter_target(
        self,
        state: _ArmTeleopState,
        pose6: tuple[float, ...],
        now_monotonic: float,
    ) -> tuple[float, ...]:
        if (
            self.target_lpf_tau_sec <= 0.0
            and self.deadband_linear_m <= 0.0
            and self.deadband_angular_rad <= 0.0
        ):
            return pose6
        filtered = state.filtered_target
        if filtered is None:
            state.filtered_target = pose6
            state.last_filter_monotonic = now_monotonic
            return pose6
        # Deadband gates the filter INPUT against the current filtered output so
        # the command freezes exactly while the hand-held tracker only jitters
        # in place; a deadband on the output would let the EMA keep creeping.
        linear_dist = math.sqrt(
            (pose6[0] - filtered[0]) ** 2
            + (pose6[1] - filtered[1]) ** 2
            + (pose6[2] - filtered[2]) ** 2
        )
        angular_dist = math.sqrt(
            _angle_diff(pose6[3], filtered[3]) ** 2
            + _angle_diff(pose6[4], filtered[4]) ** 2
            + _angle_diff(pose6[5], filtered[5]) ** 2
        )
        target = pose6
        if linear_dist <= self.deadband_linear_m and angular_dist <= self.deadband_angular_rad:
            target = filtered
        if self.target_lpf_tau_sec > 0.0:
            last = state.last_filter_monotonic
            dt = now_monotonic - last if last is not None else 0.0
            if dt > 0.0:
                alpha = dt / (self.target_lpf_tau_sec + dt)
                target = (
                    filtered[0] + alpha * (target[0] - filtered[0]),
                    filtered[1] + alpha * (target[1] - filtered[1]),
                    filtered[2] + alpha * (target[2] - filtered[2]),
                    _wrap_pi(filtered[3] + alpha * _angle_diff(target[3], filtered[3])),
                    _wrap_pi(filtered[4] + alpha * _angle_diff(target[4], filtered[4])),
                    _wrap_pi(filtered[5] + alpha * _angle_diff(target[5], filtered[5])),
                )
            else:
                target = filtered
        state.filtered_target = target
        state.last_filter_monotonic = now_monotonic
        return target

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
    state.filtered_target = None
    state.last_filter_monotonic = None


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
    # Staleness MUST be measured against the local arrival clock. A remote
    # publisher's "t" (and any per-side "monotonic") is from an unrelated
    # monotonic domain on another machine and is not comparable cross-host, so
    # we always stamp the sample with the consumer's own monotonic at parse time.
    entry["monotonic"] = monotonic_fn()
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


def _average_quaternions(
    quats: Iterable[tuple[float, float, float, float]],
) -> tuple[float, float, float, float]:
    """Hemisphere-aligned normalized quaternion mean.

    Each quaternion is sign-aligned to the running sum before accumulating
    (q and -q encode the same rotation), then the sum is normalized. For the
    nearby orientations of a tracker stream this matches the geodesic mean to
    first order; for exactly two samples it is the exact slerp midpoint.
    """
    sx = sy = sz = sw = 0.0
    count = 0
    last = (0.0, 0.0, 0.0, 1.0)
    for qx, qy, qz, qw in quats:
        last = (qx, qy, qz, qw)
        if count > 0 and (sx * qx + sy * qy + sz * qz + sw * qw) < 0.0:
            qx, qy, qz, qw = -qx, -qy, -qz, -qw
        sx += qx
        sy += qy
        sz += qz
        sw += qw
        count += 1
    if count == 0:
        return last
    norm = math.sqrt(sx * sx + sy * sy + sz * sz + sw * sw)
    if norm < 1e-9:
        # Degenerate accumulation (antipodal spread) — fall back to the most
        # recent sample rather than emit a non-unit quaternion.
        return last
    return (sx / norm, sy / norm, sz / norm, sw / norm)


def _matrix_to_quat(m: Sequence[float]) -> tuple[float, float, float, float]:
    """Row-major 3x3 rotation matrix -> (qx, qy, qz, qw), Shepperd's method."""
    trace = float(m[0]) + float(m[4]) + float(m[8])
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return (
            (float(m[7]) - float(m[5])) / s,
            (float(m[2]) - float(m[6])) / s,
            (float(m[3]) - float(m[1])) / s,
            0.25 * s,
        )
    if float(m[0]) > float(m[4]) and float(m[0]) > float(m[8]):
        s = math.sqrt(1.0 + float(m[0]) - float(m[4]) - float(m[8])) * 2.0
        return (
            0.25 * s,
            (float(m[1]) + float(m[3])) / s,
            (float(m[2]) + float(m[6])) / s,
            (float(m[7]) - float(m[5])) / s,
        )
    if float(m[4]) > float(m[8]):
        s = math.sqrt(1.0 + float(m[4]) - float(m[0]) - float(m[8])) * 2.0
        return (
            (float(m[1]) + float(m[3])) / s,
            0.25 * s,
            (float(m[5]) + float(m[7])) / s,
            (float(m[2]) - float(m[6])) / s,
        )
    s = math.sqrt(1.0 + float(m[8]) - float(m[0]) - float(m[4])) * 2.0
    return (
        (float(m[2]) + float(m[6])) / s,
        (float(m[5]) + float(m[7])) / s,
        0.25 * s,
        (float(m[3]) - float(m[1])) / s,
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


def _signs3(value: Sequence[float], label: str) -> tuple[float, float, float]:
    signs = _tuple3(value, label)
    if any(sign not in (-1.0, 1.0) for sign in signs):
        raise ValueError(f"{label} entries must be -1 or 1")
    return signs


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
