from __future__ import annotations

from policy_runner.robot_state_client import StateSnapshot
from policy_runner.servo_command_client import CommandIntent
from policy_runner.safety import ActionRequirements


class HoldActionSource:
    requirements = ActionRequirements()

    def __init__(self, send_hold: bool = False, timeout_sec: float = 0.2):
        self.send_hold = send_hold
        self.timeout_sec = timeout_sec

    def next_intent(self, snapshot: StateSnapshot, now_monotonic: float) -> CommandIntent | None:
        _ = snapshot, now_monotonic
        if not self.send_hold:
            return None
        return CommandIntent.hold(timeout_sec=self.timeout_sec)
