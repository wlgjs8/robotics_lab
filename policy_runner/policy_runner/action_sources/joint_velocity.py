from __future__ import annotations

from policy_runner.robot_state_client import StateSnapshot
from policy_runner.servo_command_client import CommandIntent
from policy_runner.safety import ActionRequirements


class JointVelocityActionSource:
    def __init__(
        self,
        velocity_deg_s: tuple[float, ...],
        selected_arm: str = "both",
        timeout_sec: float = 0.2,
        simulation_only: bool = True,
    ):
        if len(velocity_deg_s) != 6:
            raise ValueError("velocity_deg_s must contain 6 values")
        if selected_arm not in {"left", "right", "both"}:
            raise ValueError("selected_arm must be left, right, or both")
        self.velocity_deg_s = tuple(float(v) for v in velocity_deg_s)
        self.selected_arm = selected_arm
        self.timeout_sec = timeout_sec
        self.requirements = ActionRequirements(simulation_only=simulation_only)

    def next_intent(self, snapshot: StateSnapshot, now_monotonic: float) -> CommandIntent:
        _ = snapshot, now_monotonic
        return CommandIntent.joint_velocity(
            left=list(self.velocity_deg_s) if self.selected_arm in {"left", "both"} else None,
            right=list(self.velocity_deg_s) if self.selected_arm in {"right", "both"} else None,
            timeout_sec=self.timeout_sec,
        )
