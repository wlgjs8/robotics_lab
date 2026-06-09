from __future__ import annotations

from policy_runner.robot_state_client import StateSnapshot
from policy_runner.servo_command_client import CommandIntent
from policy_runner.safety import ActionRequirements


CARTESIAN_ACTION_REQUIREMENTS = ActionRequirements(
    requires_geometry=True,
    requires_valid_tcp_pose=True,
    simulation_only=True,
    requires_observed_simulation=True,
    requires_simulator_backend_if_available=True,
    cartesian_motion=True,
)

RBPODO_CONTROLLER_SIMULATION_CARTESIAN_ACTION_REQUIREMENTS = ActionRequirements(
    requires_geometry=True,
    requires_valid_tcp_pose=True,
    simulation_only=True,
    requires_observed_simulation=True,
    requires_simulator_backend_if_available=True,
    allow_rbpodo_controller_simulation_cartesian=True,
    cartesian_motion=True,
)


def cartesian_action_requirements(
    *,
    allow_rbpodo_controller_simulation: bool = False,
) -> ActionRequirements:
    if allow_rbpodo_controller_simulation:
        return RBPODO_CONTROLLER_SIMULATION_CARTESIAN_ACTION_REQUIREMENTS
    return CARTESIAN_ACTION_REQUIREMENTS


class CartesianCommandIntent(CommandIntent):
    @property
    def is_motion(self) -> bool:
        return True


class TcpDeltaActionSource:
    def __init__(
        self,
        delta: tuple[float, ...] = (0.001, 0.0, 0.0, 0.0, 0.0, 0.0),
        selected_arm: str = "both",
        frame: str = "stand",
        max_linear_step_m: float = 0.002,
        max_angular_step_rad: float = 0.01,
        timeout_sec: float = 0.2,
        simulation_only: bool = True,
    ):
        if len(delta) != 6:
            raise ValueError("delta must contain 6 values")
        if selected_arm not in {"left", "right", "both"}:
            raise ValueError("selected_arm must be left, right, or both")
        if frame != "stand":
            raise ValueError("only stand-frame TcpDeltaStand is enabled")
        if max_linear_step_m < 0.0:
            raise ValueError("max_linear_step_m must be non-negative")
        if max_angular_step_rad < 0.0:
            raise ValueError("max_angular_step_rad must be non-negative")
        self.delta = clamp_tcp_delta(delta, max_linear_step_m, max_angular_step_rad)
        self.selected_arm = selected_arm
        self.frame = frame
        self.timeout_sec = timeout_sec
        self.requirements = (
            CARTESIAN_ACTION_REQUIREMENTS if simulation_only else ActionRequirements(cartesian_motion=True)
        )

    def next_intent(self, snapshot: StateSnapshot, now_monotonic: float) -> CommandIntent | None:
        _ = snapshot, now_monotonic
        return tcp_delta_stand_intent(
            left=self.delta if self.selected_arm in {"left", "both"} else None,
            right=self.delta if self.selected_arm in {"right", "both"} else None,
            timeout_sec=self.timeout_sec,
        )


def tcp_delta_stand_intent(
    *,
    left: tuple[float, ...] | list[float] | None = None,
    right: tuple[float, ...] | list[float] | None = None,
    left_gripper: float | None = None,
    right_gripper: float | None = None,
    timeout_sec: float = 0.2,
) -> CommandIntent:
    return CartesianCommandIntent(
        "TcpDeltaStand",
        timeout_sec=timeout_sec,
        left=_arm_payload("TcpDeltaStand", "tcp_delta_stand", left, gripper_target=left_gripper),
        right=_arm_payload("TcpDeltaStand", "tcp_delta_stand", right, gripper_target=right_gripper),
    )


def tcp_twist_local_intent(
    *,
    left: tuple[float, ...] | list[float] | None = None,
    right: tuple[float, ...] | list[float] | None = None,
    left_gripper: float | None = None,
    right_gripper: float | None = None,
    timeout_sec: float = 0.2,
) -> CommandIntent:
    return CartesianCommandIntent(
        "TcpTwistLocal",
        timeout_sec=timeout_sec,
        left=_arm_payload("TcpTwistLocal", "tcp_twist_local", left, gripper_target=left_gripper),
        right=_arm_payload("TcpTwistLocal", "tcp_twist_local", right, gripper_target=right_gripper),
    )


def tcp_pose_target_stand_intent(
    *,
    left: tuple[float, ...] | list[float] | None = None,
    right: tuple[float, ...] | list[float] | None = None,
    left_gripper: float | None = None,
    right_gripper: float | None = None,
    timeout_sec: float = 0.2,
) -> CommandIntent:
    return CartesianCommandIntent(
        "TcpPoseTarget",
        timeout_sec=timeout_sec,
        left=_arm_payload("TcpPoseTarget", "tcp_target_stand", left, gripper_target=left_gripper),
        right=_arm_payload("TcpPoseTarget", "tcp_target_stand", right, gripper_target=right_gripper),
    )


def tcp_twist_stand_intent(
    *,
    left: tuple[float, ...] | list[float] | None = None,
    right: tuple[float, ...] | list[float] | None = None,
    left_gripper: float | None = None,
    right_gripper: float | None = None,
    timeout_sec: float = 0.2,
) -> CommandIntent:
    return CartesianCommandIntent(
        "TcpTwistStand",
        timeout_sec=timeout_sec,
        left=_arm_payload("TcpTwistStand", "tcp_twist_stand", left, gripper_target=left_gripper),
        right=_arm_payload("TcpTwistStand", "tcp_twist_stand", right, gripper_target=right_gripper),
    )


def clamp_tcp_delta(
    delta: tuple[float, ...] | list[float],
    max_linear_step_m: float,
    max_angular_step_rad: float,
) -> tuple[float, ...]:
    if len(delta) != 6:
        raise ValueError("delta must contain 6 values")
    limits = (
        abs(float(max_linear_step_m)),
        abs(float(max_linear_step_m)),
        abs(float(max_linear_step_m)),
        abs(float(max_angular_step_rad)),
        abs(float(max_angular_step_rad)),
        abs(float(max_angular_step_rad)),
    )
    return tuple(_clamp(float(value), limit) for value, limit in zip(delta, limits))


def clamp_tcp_twist(
    twist: tuple[float, ...] | list[float],
    max_linear_velocity_m_s: float,
    max_angular_velocity_rad_s: float,
) -> tuple[float, ...]:
    if len(twist) != 6:
        raise ValueError("twist must contain 6 values")
    limits = (
        abs(float(max_linear_velocity_m_s)),
        abs(float(max_linear_velocity_m_s)),
        abs(float(max_linear_velocity_m_s)),
        abs(float(max_angular_velocity_rad_s)),
        abs(float(max_angular_velocity_rad_s)),
        abs(float(max_angular_velocity_rad_s)),
    )
    return tuple(_clamp(float(value), limit) for value, limit in zip(twist, limits))


def _arm_payload(
    mode: str,
    key: str,
    delta: tuple[float, ...] | list[float] | None,
    *,
    gripper_target: float | None = None,
) -> dict:
    if delta is None:
        payload = {"mode": "Hold"}
        if gripper_target is not None:
            payload["gripper_target"] = float(gripper_target)
        return payload
    if len(delta) != 6:
        raise ValueError(f"{key} must contain 6 values")
    payload = {"mode": mode, key: [float(value) for value in delta]}
    if gripper_target is not None:
        payload["gripper_target"] = float(gripper_target)
    return payload


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))
