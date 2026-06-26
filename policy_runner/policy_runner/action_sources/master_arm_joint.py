from __future__ import annotations

import importlib
import math
import sys
from pathlib import Path
from typing import Any, Sequence

from policy_runner.robot_state_client import StateSnapshot
from policy_runner.safety import ActionRequirements
from policy_runner.servo_command_client import CommandIntent


class MasterArmJointActionSource:
    """Joint-space teleoperation from mo_master_arm to rb_servo_server.

    mo_master_arm reports calibrated logical joint positions in radians.
    rb_servo_server accepts JointTarget commands in degrees.  The recommended
    operating mode is ``delta``: when the deadman button is pressed, the current
    master-arm pose and current robot pose are latched, and subsequent master
    joint deltas are added to the robot anchor.  This prevents a jump if the
    master and robot absolute zero conventions are not perfectly identical.
    """

    requirements = ActionRequirements(requires_valid_joint_state=True)

    def __init__(
        self,
        *,
        config_path: str,
        python_module_dir: str = "",
        module_name: str = "_mo_master_arm_py",
        selected_arm: str = "both",
        mapping_mode: str = "delta",
        robot_init_strategy: str = "current",
        deadman_side: str = "either",
        deadman_switch: int = 2,
        require_deadman: bool = True,
        use_gravity_compensation: bool = True,
        hold_master_on_release: bool = True,
        enable_gripper_readers: bool = False,
        left_scale: Sequence[float] = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
        right_scale: Sequence[float] = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
        left_sign: Sequence[float] = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
        right_sign: Sequence[float] = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
        left_offset_deg: Sequence[float] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        right_offset_deg: Sequence[float] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        left_robot_init_deg: Sequence[float] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        right_robot_init_deg: Sequence[float] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        left_joint_map: Sequence[int] = (0, 1, 2, 3, 4, 5),
        right_joint_map: Sequence[int] = (0, 1, 2, 3, 4, 5),
        max_joint_speed_deg_s: Sequence[float] = (30.0, 30.0, 30.0, 45.0, 45.0, 60.0),
        smoothing_alpha: float = 1.0,
        wrap_delta: bool = True,
        timeout_sec: float = 0.2,
    ):
        if selected_arm not in {"left", "right", "both"}:
            raise ValueError("master_arm_joint.selected_arm must be left, right, or both")
        if mapping_mode not in {"delta", "absolute"}:
            raise ValueError("master_arm_joint.mapping_mode must be delta or absolute")
        if robot_init_strategy not in {"current", "configured", "master_absolute"}:
            raise ValueError(
                "master_arm_joint.robot_init_strategy must be current, configured, or master_absolute"
            )
        if deadman_side not in {"left", "right", "either", "both", "none"}:
            raise ValueError("master_arm_joint.deadman_side must be left, right, either, both, or none")
        if deadman_switch < 0:
            raise ValueError("master_arm_joint.deadman_switch must be non-negative")
        if not 0.0 <= float(smoothing_alpha) <= 1.0:
            raise ValueError("master_arm_joint.smoothing_alpha must be in [0, 1]")
        self.config_path = config_path
        self.python_module_dir = python_module_dir
        self.module_name = module_name
        self.selected_arm = selected_arm
        self.mapping_mode = mapping_mode
        self.robot_init_strategy = robot_init_strategy
        self.deadman_side = deadman_side
        self.deadman_switch = int(deadman_switch)
        self.require_deadman = bool(require_deadman)
        self.use_gravity_compensation = bool(use_gravity_compensation)
        self.hold_master_on_release = bool(hold_master_on_release)
        self.enable_gripper_readers = bool(enable_gripper_readers)
        self.left_scale = _tuple6(left_scale, "left_scale")
        self.right_scale = _tuple6(right_scale, "right_scale")
        self.left_sign = _tuple6(left_sign, "left_sign")
        self.right_sign = _tuple6(right_sign, "right_sign")
        self.left_offset_deg = _tuple6(left_offset_deg, "left_offset_deg")
        self.right_offset_deg = _tuple6(right_offset_deg, "right_offset_deg")
        self.left_robot_init_deg = _tuple6(left_robot_init_deg, "left_robot_init_deg")
        self.right_robot_init_deg = _tuple6(right_robot_init_deg, "right_robot_init_deg")
        self.left_joint_map = _joint_map(left_joint_map, "left_joint_map")
        self.right_joint_map = _joint_map(right_joint_map, "right_joint_map")
        self.max_joint_speed_deg_s = _tuple6(max_joint_speed_deg_s, "max_joint_speed_deg_s")
        self.smoothing_alpha = float(smoothing_alpha)
        self.wrap_delta = bool(wrap_delta)
        self.timeout_sec = float(timeout_sec)

        self._session: Any | None = None
        self._robot_target_initialized = False
        self._last_deadman = False
        self._last_left_target_deg: tuple[float, ...] | None = None
        self._last_right_target_deg: tuple[float, ...] | None = None
        self._master_anchor_left_deg: tuple[float, ...] | None = None
        self._master_anchor_right_deg: tuple[float, ...] | None = None
        self._robot_anchor_left_deg: tuple[float, ...] | None = None
        self._robot_anchor_right_deg: tuple[float, ...] | None = None
        self._last_update_monotonic: float | None = None

    def next_intent(self, snapshot: StateSnapshot, now_monotonic: float) -> CommandIntent | None:
        session = self._ensure_session()
        self._read_master(session)

        if not self._robot_target_initialized:
            left, right = self._initial_robot_targets(snapshot)
            self._last_left_target_deg = left
            self._last_right_target_deg = right
            self._robot_target_initialized = True
            self._last_update_monotonic = now_monotonic
            return CommandIntent.joint_target(left=left, right=right, timeout_sec=self.timeout_sec)

        deadman = self._deadman_active(session)
        if deadman and not self._last_deadman:
            self._capture_delta_anchors(snapshot)
            self._set_master_hold(session, False)
        elif not deadman and self._last_deadman:
            self._set_master_hold(session, self.hold_master_on_release)
        self._last_deadman = deadman

        if self.use_gravity_compensation:
            session.gravity_compensation_loop()

        if deadman or not self.require_deadman:
            left, right = self._mapped_targets_from_master()
            dt_sec = None if self._last_update_monotonic is None else max(0.0, now_monotonic - self._last_update_monotonic)
            left = self._postprocess_target(left, self._last_left_target_deg, dt_sec)
            right = self._postprocess_target(right, self._last_right_target_deg, dt_sec)
            self._last_left_target_deg = left
            self._last_right_target_deg = right
            self._last_update_monotonic = now_monotonic
            return CommandIntent.joint_target(left=left, right=right, timeout_sec=self.timeout_sec)

        # Keep the robot on the last safe target while the deadman is released.
        # Repeating JointTarget avoids command-timeout Hold from stopping an
        # initial move before it reaches the target.
        self._last_update_monotonic = now_monotonic
        return CommandIntent.joint_target(
            left=self._last_left_target_deg if self.selected_arm in {"left", "both"} else None,
            right=self._last_right_target_deg if self.selected_arm in {"right", "both"} else None,
            timeout_sec=self.timeout_sec,
        )

    def close(self) -> None:
        if self._session is None:
            return
        try:
            self._set_master_hold(self._session, True)
            if self.use_gravity_compensation:
                if self.selected_arm in {"left", "both"}:
                    self._session.stop_gravity_compensation("left")
                if self.selected_arm in {"right", "both"}:
                    self._session.stop_gravity_compensation("right")
        finally:
            self._session.close()
            self._session = None

    def _ensure_session(self) -> Any:
        if self._session is not None:
            return self._session
        if self.python_module_dir:
            module_dir = str(Path(self.python_module_dir).expanduser().resolve())
            if module_dir not in sys.path:
                sys.path.insert(0, module_dir)
        module = importlib.import_module(self.module_name)
        session = module.MasterArmSession(
            self.config_path,
            True,  # enable switches
            self.enable_gripper_readers,
        )
        if self.selected_arm in {"left", "both"}:
            session.move_init_joint("left", True)
        if self.selected_arm in {"right", "both"}:
            session.move_init_joint("right", True)
        session.read_all()
        if self.use_gravity_compensation:
            if self.selected_arm in {"left", "both"}:
                session.start_gravity_compensation("left")
            if self.selected_arm in {"right", "both"}:
                session.start_gravity_compensation("right")
            self._set_master_hold(session, True)
        self._session = session
        return session

    def _read_master(self, session: Any) -> None:
        session.read_all()

    def _initial_robot_targets(
        self,
        snapshot: StateSnapshot,
    ) -> tuple[tuple[float, ...] | None, tuple[float, ...] | None]:
        left: tuple[float, ...] | None = None
        right: tuple[float, ...] | None = None
        if self.robot_init_strategy == "current":
            if self.selected_arm in {"left", "both"}:
                left = _state_joint_deg(snapshot, "left")
            if self.selected_arm in {"right", "both"}:
                right = _state_joint_deg(snapshot, "right")
        elif self.robot_init_strategy == "configured":
            left = self.left_robot_init_deg if self.selected_arm in {"left", "both"} else None
            right = self.right_robot_init_deg if self.selected_arm in {"right", "both"} else None
        else:
            left, right = self._mapped_targets_from_master(absolute_for_init=True)
        if left is not None:
            left = _tuple6(left, "left initial robot target")
        if right is not None:
            right = _tuple6(right, "right initial robot target")
        return left, right

    def _deadman_active(self, session: Any) -> bool:
        if not self.require_deadman or self.deadman_side == "none":
            return True
        if self.deadman_side == "left":
            return self._deadman_side_active(session, "left")
        if self.deadman_side == "right":
            return self._deadman_side_active(session, "right")
        if self.deadman_side == "both":
            return self._deadman_side_active(session, "left") and self._deadman_side_active(session, "right")
        return self._deadman_side_active(session, "left") or self._deadman_side_active(session, "right")

    def _deadman_side_active(self, session: Any, side: str) -> bool:
        pressed_state = session.get_pressed_state(side)
        return session.get_switch_status(side, self.deadman_switch) == pressed_state

    def _capture_delta_anchors(self, snapshot: StateSnapshot) -> None:
        if self.selected_arm in {"left", "both"}:
            self._master_anchor_left_deg = self._master_joint_deg("left")
            self._robot_anchor_left_deg = (
                self._last_left_target_deg or _state_joint_deg(snapshot, "left")
            )
        if self.selected_arm in {"right", "both"}:
            self._master_anchor_right_deg = self._master_joint_deg("right")
            self._robot_anchor_right_deg = (
                self._last_right_target_deg or _state_joint_deg(snapshot, "right")
            )

    def _set_master_hold(self, session: Any, holding: bool) -> None:
        if not self.use_gravity_compensation:
            return
        if self.selected_arm in {"left", "both"}:
            session.set_gravity_hold("left", holding)
        if self.selected_arm in {"right", "both"}:
            session.set_gravity_hold("right", holding)

    def _mapped_targets_from_master(
        self,
        *,
        absolute_for_init: bool = False,
    ) -> tuple[tuple[float, ...] | None, tuple[float, ...] | None]:
        left = None
        right = None
        if self.selected_arm in {"left", "both"}:
            left = self._map_one_side(
                side="left",
                master_now=self._master_joint_deg("left"),
                master_anchor=self._master_anchor_left_deg,
                robot_anchor=self._robot_anchor_left_deg,
                scale=self.left_scale,
                sign=self.left_sign,
                offset=self.left_offset_deg,
                joint_map=self.left_joint_map,
                absolute=absolute_for_init or self.mapping_mode == "absolute",
            )
        if self.selected_arm in {"right", "both"}:
            right = self._map_one_side(
                side="right",
                master_now=self._master_joint_deg("right"),
                master_anchor=self._master_anchor_right_deg,
                robot_anchor=self._robot_anchor_right_deg,
                scale=self.right_scale,
                sign=self.right_sign,
                offset=self.right_offset_deg,
                joint_map=self.right_joint_map,
                absolute=absolute_for_init or self.mapping_mode == "absolute",
            )
        return left, right

    def _map_one_side(
        self,
        *,
        side: str,
        master_now: tuple[float, ...],
        master_anchor: tuple[float, ...] | None,
        robot_anchor: tuple[float, ...] | None,
        scale: tuple[float, ...],
        sign: tuple[float, ...],
        offset: tuple[float, ...],
        joint_map: tuple[int, ...],
        absolute: bool,
    ) -> tuple[float, ...]:
        if absolute:
            return tuple(
                offset[i] + sign[i] * scale[i] * master_now[joint_map[i]]
                for i in range(6)
            )
        if master_anchor is None or robot_anchor is None:
            raise RuntimeError(f"{side} delta teleop anchor is missing")
        out = []
        for i in range(6):
            j = joint_map[i]
            raw_delta = master_now[j] - master_anchor[j]
            delta = _wrap_delta_deg(raw_delta) if self.wrap_delta else raw_delta
            out.append(robot_anchor[i] + sign[i] * scale[i] * delta)
        return tuple(out)

    def _master_joint_deg(self, side: str) -> tuple[float, ...]:
        assert self._session is not None
        q_rad = self._session.get_joint_positions(side)
        if not isinstance(q_rad, list) or len(q_rad) != 6:
            raise RuntimeError(f"incomplete {side} master-arm joint data")
        return tuple(math.degrees(float(v)) for v in q_rad)

    def _postprocess_target(
        self,
        target: tuple[float, ...] | None,
        previous: tuple[float, ...] | None,
        dt_sec: float | None,
    ) -> tuple[float, ...] | None:
        if target is None:
            return None
        out = tuple(float(v) for v in target)
        if previous is not None and self.smoothing_alpha < 1.0:
            alpha = self.smoothing_alpha
            out = tuple(prev + alpha * (cur - prev) for prev, cur in zip(previous, out))
        if previous is not None and dt_sec is not None and dt_sec > 0.0:
            limited = []
            for i, (prev, cur) in enumerate(zip(previous, out)):
                max_step = abs(self.max_joint_speed_deg_s[i]) * dt_sec
                if max_step > 0.0:
                    cur = max(prev - max_step, min(prev + max_step, cur))
                limited.append(cur)
            out = tuple(limited)
        return out


def _state_joint_deg(snapshot: StateSnapshot, side: str) -> tuple[float, ...]:
    arm = snapshot.payload.get(side)
    if not isinstance(arm, dict):
        raise RuntimeError(f"state stream missing {side} arm")
    for key in ("q_sent_deg", "q_target_deg", "q_actual_deg"):
        value = arm.get(key)
        if isinstance(value, list) and len(value) == 6:
            return tuple(float(v) for v in value)
    raise RuntimeError(f"state stream missing {side} q_sent/q_target/q_actual")


def _tuple6(value: Sequence[float], label: str) -> tuple[float, ...]:
    if len(value) != 6:
        raise ValueError(f"{label} must contain 6 values")
    return tuple(float(v) for v in value)


def _joint_map(value: Sequence[int], label: str) -> tuple[int, ...]:
    if len(value) != 6:
        raise ValueError(f"{label} must contain 6 values")
    out = tuple(int(v) for v in value)
    if sorted(out) != [0, 1, 2, 3, 4, 5]:
        raise ValueError(f"{label} must be a permutation of 0..5")
    return out


def _wrap_delta_deg(value: float) -> float:
    return (float(value) + 180.0) % 360.0 - 180.0
