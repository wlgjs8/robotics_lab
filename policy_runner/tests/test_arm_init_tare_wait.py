"""After an InitMotion the override latch waits for the server's F/T auto-tare.

Measured 2026-09-04: resuming the policy 80 ms after "done" starved the tare's
0.5 s settle and the rest of the episode ran without force control.
"""

from __future__ import annotations

import unittest

from policy_runner.arm_init_control import (
    ARM_INIT_COMMAND_SCHEMA,
    ArmInitOverrideController,
    parse_arm_init_command,
)
from policy_runner.robot_state_client import StateSnapshot


def _start(controller: ArmInitOverrideController, arms: str = "right") -> None:
    payload = {"schema": ARM_INIT_COMMAND_SCHEMA, "arms": arms, "action": "start",
               "right_q_deg": [0, 1, 0, 0, 0, 0], "left_q_deg": [1, 0, 0, 0, 0, 0]}
    controller.handle_command(parse_arm_init_command(payload))
    controller.consume_transitions()


def _snapshot(status: str, *, t: float, ft: dict | None, arm: str = "right") -> StateSnapshot:
    payload: dict = {"init_motion": {arm: {"status": status}}}
    if ft is not None:
        payload[arm] = {"force_torque": ft}
    return StateSnapshot(payload, received_monotonic=t)


SETTLING = {"enabled": True, "connected": True, "auto_tare_stage": "settling", "bias_valid": False}
ARMED = {"enabled": True, "connected": True, "auto_tare_stage": "awaiting_init", "bias_valid": False}
ACCEPTED = {"enabled": True, "connected": True, "auto_tare_stage": "idle", "bias_valid": True}
DISABLED = {"enabled": False, "connected": False, "auto_tare_stage": "off", "bias_valid": False}


class ArmInitTareWaitTest(unittest.TestCase):
    def test_done_holds_the_latch_until_the_tare_lands(self):
        controller = ArmInitOverrideController(ft_tare_wait_sec=2.0)
        _start(controller)
        controller.update_from_snapshot(_snapshot("executing", t=1.0, ft=ARMED))
        self.assertEqual(controller.consume_transitions().done, ())

        controller.update_from_snapshot(_snapshot("done", t=2.0, ft=SETTLING))
        self.assertTrue(controller.right_on, "latch must stay on while the tare settles")
        self.assertEqual(controller.consume_transitions().done, ())
        self.assertEqual(controller.status_block()["right_state"], "init tare")
        # The re-emitted intent keeps the arm parked at the init pose.
        mixed = controller.compose_intent(None)
        self.assertEqual(mixed.right["mode"], "JointTarget")

        controller.update_from_snapshot(_snapshot("done", t=2.7, ft=SETTLING))
        self.assertTrue(controller.right_on)

        controller.update_from_snapshot(_snapshot("done", t=3.1, ft=ACCEPTED))
        self.assertFalse(controller.right_on)
        self.assertEqual(controller.consume_transitions().done, ("right",))
        self.assertEqual(controller.error, "")

    def test_wait_is_bounded(self):
        controller = ArmInitOverrideController(ft_tare_wait_sec=1.0)
        _start(controller)
        controller.update_from_snapshot(_snapshot("executing", t=1.0, ft=ARMED))
        controller.update_from_snapshot(_snapshot("done", t=2.0, ft=SETTLING))
        self.assertTrue(controller.right_on)
        controller.update_from_snapshot(_snapshot("done", t=2.9, ft=SETTLING))
        self.assertTrue(controller.right_on)
        controller.update_from_snapshot(_snapshot("done", t=3.05, ft=SETTLING))
        self.assertFalse(controller.right_on, "the wait must time out")
        self.assertEqual(controller.consume_transitions().done, ("right",))
        self.assertIn("did not land", controller.error)

    def test_no_wait_when_ft_is_disabled_or_absent(self):
        for ft in (DISABLED, None):
            controller = ArmInitOverrideController(ft_tare_wait_sec=2.0)
            _start(controller)
            controller.update_from_snapshot(_snapshot("executing", t=1.0, ft=ft))
            controller.update_from_snapshot(_snapshot("done", t=2.0, ft=ft))
            self.assertFalse(controller.right_on, f"ft={ft}")
            self.assertEqual(controller.consume_transitions().done, ("right",))

    def test_zero_wait_disables_the_gate(self):
        controller = ArmInitOverrideController(ft_tare_wait_sec=0.0)
        _start(controller)
        controller.update_from_snapshot(_snapshot("executing", t=1.0, ft=ARMED))
        controller.update_from_snapshot(_snapshot("done", t=2.0, ft=SETTLING))
        self.assertFalse(controller.right_on)
        self.assertEqual(controller.consume_transitions().done, ("right",))

    def test_both_arms_wait_independently(self):
        controller = ArmInitOverrideController(ft_tare_wait_sec=2.0)
        _start(controller, arms="both")
        payload = {
            "init_motion": {"left": {"status": "done"}, "right": {"status": "done"}},
            "left": {"force_torque": ACCEPTED},
            "right": {"force_torque": SETTLING},
        }
        controller.update_from_snapshot(StateSnapshot(payload, received_monotonic=5.0))
        self.assertFalse(controller.left_on)
        self.assertTrue(controller.right_on)
        self.assertEqual(controller.consume_transitions().done, ("left",))
        payload["right"] = {"force_torque": ACCEPTED}
        controller.update_from_snapshot(StateSnapshot(payload, received_monotonic=5.6))
        self.assertFalse(controller.right_on)
        self.assertEqual(controller.consume_transitions().done, ("right",))

    def test_external_init_without_latch_does_not_wait(self):
        controller = ArmInitOverrideController(ft_tare_wait_sec=2.0)
        controller.update_from_snapshot(_snapshot("executing", t=1.0, ft=ARMED))
        controller.update_from_snapshot(_snapshot("done", t=2.0, ft=SETTLING))
        self.assertFalse(controller.right_on)
        self.assertEqual(controller.consume_transitions().done, ("right",))


if __name__ == "__main__":
    unittest.main()
