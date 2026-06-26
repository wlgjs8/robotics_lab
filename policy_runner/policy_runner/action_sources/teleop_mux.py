from __future__ import annotations

import os

from policy_runner.action_sources.dual_spacemouse_pose_target import (
    DualSpaceMousePoseTargetActionSource,
)
from policy_runner.action_sources.tcp_pose_target import (
    cartesian_action_requirements,
)
from policy_runner.action_sources.umi_dual_cartesian import UmiDualCartesianActionSource
from policy_runner.robot_state_client import StateSnapshot
from policy_runner.servo_command_client import CommandIntent

OWNER_IDLE = "idle"
OWNER_SPACEMOUSE = "spacemouse"
OWNER_UMI = "umi"

class TeleopMuxActionSource:
    """Run SpaceMouse and UMI teleop side by side under ONE lease/run loop.

    Both input devices are polled every tick so each stays warm (UDP backlog
    drained, moving-average filled, engagement state machines live), but only
    one source owns the WHOLE robot at a time:

    - IDLE: the first source to engage (SpaceMouse cap deflection past the
      activation gate / UMI deadman held) takes ownership. If both engage on
      the same tick, ``tie_break`` decides.
    - Owned: the other source's intents are discarded AND its engagement
      latches are reset every tick, so it can never accumulate stale
      relative-init state — when it later takes over it re-latches fresh from
      the robot's current TCP (no jump).
    - Handoff is idle-gated: ownership is released only after the owner has
      fully disengaged (its final hold / latch-clear intent is passed through
      first).

    Whole-robot exclusivity (not per-arm) keeps one operator source in charge
    of both arms at a time.

    A missing/broken SpaceMouse HID degrades to UMI-only operation: the first
    SpaceMouse error disables that side for the rest of the run (logged once;
    one safety Hold is emitted if it died while owning the robot).
    """

    def __init__(
        self,
        spacemouse_source: DualSpaceMousePoseTargetActionSource,
        umi_source: UmiDualCartesianActionSource,
        *,
        tie_break: str = "umi",
    ):
        if tie_break not in (OWNER_SPACEMOUSE, OWNER_UMI):
            raise ValueError("tie_break must be 'spacemouse' or 'umi'")
        self.spacemouse_source = spacemouse_source
        self.umi_source = umi_source
        self.tie_break = tie_break
        # Fail-closed: the mux advertises the controller-simulation carve-out
        # only if BOTH sub-sources do.
        self.requirements = cartesian_action_requirements(
            allow_rbpodo_controller_simulation=(
                spacemouse_source.requirements.allow_rbpodo_controller_simulation_cartesian
                and umi_source.requirements.allow_rbpodo_controller_simulation_cartesian
            )
        )
        self._owner = OWNER_IDLE
        self._spacemouse_failed = False
        self._debug = os.environ.get("POLICY_RUNNER_TELEOP_DEBUG", "") == "1"

    @property
    def owner(self) -> str:
        return self._owner

    def next_intent(self, snapshot: StateSnapshot, now_monotonic: float) -> CommandIntent | None:
        spacemouse_intent, spacemouse_safety_intent = self._spacemouse_intent(snapshot, now_monotonic)
        if spacemouse_safety_intent is not None:
            # SpaceMouse HID died while owning the robot: one Hold so no stale
            # motion survives, then fall back to idle next tick.
            return spacemouse_safety_intent
        umi_intent = self.umi_source.next_intent(snapshot, now_monotonic)
        spacemouse_engaged = (
            not self._spacemouse_failed
            and (
                self.spacemouse_source.engaged
                or _intent_has_gripper_target(spacemouse_intent)
            )
        )
        umi_engaged = self.umi_source.engaged

        if self._owner == OWNER_SPACEMOUSE:
            self.umi_source.reset_engagement()
            if not spacemouse_engaged and spacemouse_intent is None:
                self._set_owner(OWNER_IDLE)
            return spacemouse_intent
        if self._owner == OWNER_UMI:
            self.spacemouse_source.reset_engagement()
            if not umi_engaged and umi_intent is None:
                self._set_owner(OWNER_IDLE)
            return umi_intent

        # IDLE: first engaged wins; same-tick double engage falls to tie_break.
        if spacemouse_engaged and umi_engaged:
            winner = self.tie_break
        elif spacemouse_engaged:
            winner = OWNER_SPACEMOUSE
        elif umi_engaged:
            winner = OWNER_UMI
        else:
            return None
        self._set_owner(winner)
        if winner == OWNER_SPACEMOUSE:
            self.umi_source.reset_engagement()
            return spacemouse_intent
        self.spacemouse_source.reset_engagement()
        return umi_intent

    def close(self) -> None:
        try:
            self.spacemouse_source.close()
        finally:
            self.umi_source.close()

    def _spacemouse_intent(
        self,
        snapshot: StateSnapshot,
        now_monotonic: float,
    ) -> tuple[CommandIntent | None, CommandIntent | None]:
        """Returns (intent, safety_intent). safety_intent is non-None exactly
        once, when the HID fails while SpaceMouse owns the robot."""
        if self._spacemouse_failed:
            return None, None
        try:
            return self.spacemouse_source.next_intent(snapshot, now_monotonic), None
        except (RuntimeError, OSError) as exc:
            self._spacemouse_failed = True
            print(
                f"[mux] spacemouse disabled for this run ({exc}); continuing UMI-only",
                flush=True,
            )
            if self._owner == OWNER_SPACEMOUSE:
                self._set_owner(OWNER_IDLE)
                return None, CommandIntent.hold(timeout_sec=self.spacemouse_source.timeout_sec)
            return None, None

    def _set_owner(self, owner: str) -> None:
        if owner == self._owner:
            return
        if self._debug:
            print(f"[mux] owner {self._owner} -> {owner}", flush=True)
        self._owner = owner


def _intent_has_gripper_target(intent: CommandIntent | None) -> bool:
    if intent is None:
        return False
    for arm in (intent.left, intent.right):
        if isinstance(arm, dict) and arm.get("gripper_target") is not None:
            return True
    return False
