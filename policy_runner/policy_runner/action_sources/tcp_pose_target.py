from __future__ import annotations

from policy_runner.servo_command_client import CommandIntent
from policy_runner.safety import ActionRequirements


CARTESIAN_ACTION_REQUIREMENTS = ActionRequirements(
    requires_geometry=True,
    requires_valid_tcp_pose=True,
    simulation_only=True,
    requires_observed_simulation=True,
    cartesian_motion=True,
)

RBPODO_CONTROLLER_SIMULATION_CARTESIAN_ACTION_REQUIREMENTS = ActionRequirements(
    requires_geometry=True,
    requires_valid_tcp_pose=True,
    simulation_only=True,
    requires_observed_simulation=True,
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


def tcp_pose_target_stand_intent(
    *,
    left: tuple[float, ...] | list[float] | None = None,
    right: tuple[float, ...] | list[float] | None = None,
    left_gripper: float | None = None,
    right_gripper: float | None = None,
    timeout_sec: float = 0.2,
    tcp_target_profile: str | None = None,
    metadata: dict | None = None,
) -> CommandIntent:
    return CartesianCommandIntent(
        "TcpPoseTarget",
        timeout_sec=timeout_sec,
        left=_pose_target_arm_payload(left, gripper_target=left_gripper),
        right=_pose_target_arm_payload(right, gripper_target=right_gripper),
        tcp_target_profile=tcp_target_profile,
        metadata=metadata,
    )


def clamp_pose_delta(
    delta: tuple[float, ...] | list[float],
    max_linear_step_m: float,
    max_angular_step_rad: float,
) -> tuple[float, ...]:
    if len(delta) != 6:
        raise ValueError("pose delta must contain 6 values")
    limits = (
        abs(float(max_linear_step_m)),
        abs(float(max_linear_step_m)),
        abs(float(max_linear_step_m)),
        abs(float(max_angular_step_rad)),
        abs(float(max_angular_step_rad)),
        abs(float(max_angular_step_rad)),
    )
    return tuple(_clamp(float(value), limit) for value, limit in zip(delta, limits))


def _pose_target_arm_payload(
    pose: tuple[float, ...] | list[float] | None,
    *,
    gripper_target: float | None = None,
) -> dict:
    if pose is None:
        payload = {"mode": "Hold"}
        if gripper_target is not None:
            payload["gripper_target"] = float(gripper_target)
        return payload
    values = [float(value) for value in pose]
    if len(values) == 6:
        payload = {"mode": "TcpPoseTarget", "tcp_target_stand": values}
    elif len(values) == 7:
        payload = {
            "mode": "TcpPoseTarget",
            "tcp_target_stand": {
                "x": values[0],
                "y": values[1],
                "z": values[2],
                "rx": 0.0,
                "ry": 0.0,
                "rz": 0.0,
                "quaternion_xyzw": values[3:7],
            },
        }
    else:
        raise ValueError("tcp_target_stand must contain 6 or 7 values")
    if gripper_target is not None:
        payload["gripper_target"] = float(gripper_target)
    return payload


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))
