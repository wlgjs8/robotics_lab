from __future__ import annotations

import math

from policy_runner.robot_state_client import StateSnapshot
from policy_runner.servo_command_client import CommandIntent
from policy_runner.safety import ActionRequirements


class JointSineActionSource:
    def __init__(
        self,
        amplitude_deg: tuple[float, ...],
        frequency_hz: float = 0.1,
        selected_arm: str = "both",
        timeout_sec: float = 0.2,
        simulation_only: bool = True,
    ):
        if len(amplitude_deg) != 6:
            raise ValueError("amplitude_deg must contain 6 values")
        if selected_arm not in {"left", "right", "both"}:
            raise ValueError("selected_arm must be left, right, or both")
        self.amplitude_deg = tuple(float(v) for v in amplitude_deg)
        self.frequency_hz = float(frequency_hz)
        self.selected_arm = selected_arm
        self.timeout_sec = timeout_sec
        self._start_monotonic: float | None = None
        self.requirements = ActionRequirements(simulation_only=simulation_only)

    def next_intent(self, snapshot: StateSnapshot, now_monotonic: float) -> CommandIntent | None:
        if self._start_monotonic is None:
            self._start_monotonic = now_monotonic
        phase = math.sin(2.0 * math.pi * self.frequency_hz * (now_monotonic - self._start_monotonic))
        left = _target_for_arm(snapshot.payload, "left", self.amplitude_deg, phase)
        right = _target_for_arm(snapshot.payload, "right", self.amplitude_deg, phase)
        return CommandIntent.joint_target(
            left=left if self.selected_arm in {"left", "both"} else None,
            right=right if self.selected_arm in {"right", "both"} else None,
            timeout_sec=self.timeout_sec,
        )


def _target_for_arm(payload: dict, arm: str, amplitude: tuple[float, ...], phase: float) -> list[float] | None:
    arm_payload = payload.get(arm, {})
    q_actual = arm_payload.get("q_actual_deg")
    if not bool(arm_payload.get("has_valid_joint_state", False)):
        return None
    if not isinstance(q_actual, list) or len(q_actual) != 6:
        return None
    return [float(q_actual[i]) + amplitude[i] * phase for i in range(6)]
