"""A single-arm InitMotion must start while the OTHER arm's init is still running.

Measured 2026-09-10 (logs/servo_log_20260910_111949.csv). The operator pressed the
left arm's InitMotion, then the right arm's while the left was still moving. The
right arm did not move until the left one was completely finished:

    t=3.370  left  init planning   right_joint_target_profile_before_init_sequencer=direct
    t=5.808  left  init done       right ...                                     =direct
    t=6.810  left  F/T auto-tare accepted
    t=6.832  right init planning   <- 22 ms after the tare, 3.5 s after the press

`*_before_init_sequencer` is the RAW received command, so the right arm's request
never reached the server at all: it sat queued in policy_runner. The left press had
gone out as rb_gui's direct one-shot (`init_motion_left_request_id=0`), policy_runner
observed it as an EXTERNAL init, and `compose_intent` returned None for the whole
duration -- including the ~1 s F/T auto-tare hold after the arm had already parked.

Two things are asserted here:
  * an external init on the peer arm no longer silences OUR init request, and
  * arms selected by ONE press still share a request id, which is how the server
    tells a both-arm request (one combined 12-DOF plan) from two single-arm ones.
"""

from __future__ import annotations

import unittest

from policy_runner.action_sources.tcp_pose_target import tcp_pose_target_stand_intent
from policy_runner.arm_init_control import (
    ARM_INIT_COMMAND_SCHEMA,
    ArmInitOverrideController,
    parse_arm_init_command,
)
from policy_runner.robot_state_client import StateSnapshot

LEFT_Q = [1, 0, 0, 0, 0, 0]
RIGHT_Q = [0, 1, 0, 0, 0, 0]

SETTLING = {"enabled": True, "connected": True, "auto_tare_stage": "settling", "bias_valid": False}
ACCEPTED = {
    "enabled": True,
    "connected": True,
    "auto_tare_stage": "idle",
    "bias_valid": True,
    "tare_state": "accepted",
}


def _press(controller: ArmInitOverrideController, arms: str) -> None:
    controller.handle_command(
        parse_arm_init_command(
            {
                "schema": ARM_INIT_COMMAND_SCHEMA,
                "arms": arms,
                "action": "start",
                "left_q_deg": LEFT_Q,
                "right_q_deg": RIGHT_Q,
            }
        )
    )
    controller.consume_transitions()


def _snapshot(init_motion: dict, *, t: float, ft: dict | None = None) -> StateSnapshot:
    payload: dict = {"init_motion": init_motion}
    if ft is not None:
        payload["left"] = {"force_torque": ft}
        payload["right"] = {"force_torque": ft}
    return StateSnapshot(payload, received_monotonic=t)


class ExternalInitDoesNotQueueOurRequestTest(unittest.TestCase):
    def _controller_with_external_left(self) -> ArmInitOverrideController:
        controller = ArmInitOverrideController()
        # The init pose reaches the latch on any arm_init packet; here it arrives with a
        # settings-only packet so nothing is started locally.
        controller.handle_command(
            parse_arm_init_command(
                {
                    "schema": ARM_INIT_COMMAND_SCHEMA,
                    "arms": "both",
                    "action": "config",
                    "auto_roi_recover": False,
                    "left_q_deg": LEFT_Q,
                    "right_q_deg": RIGHT_Q,
                }
            )
        )
        # rb_gui's direct one-shot: the server reports the left arm executing an init
        # nobody told policy_runner about.
        controller.update_from_snapshot(
            _snapshot({"left": {"status": "executing"}, "right": {"status": "idle"}}, t=1.0)
        )
        controller.consume_transitions()
        self.assertTrue(controller.external_init_active)
        return controller

    def test_external_init_alone_still_sends_nothing(self):
        controller = self._controller_with_external_left()
        policy = tcp_pose_target_stand_intent(
            left=(0.1, 0.2, 0.3, 0, 0, 0),
            right=(0.4, 0.5, 0.6, 0, 0, 0),
        )
        # Nothing of ours to say: a policy packet carries no init_motion profile, and the
        # server cancels every in-flight exec on an explicit non-init command.
        self.assertIsNone(controller.compose_intent(policy))

    def test_our_init_goes_out_while_the_peer_init_is_in_flight(self):
        controller = self._controller_with_external_left()
        _press(controller, "right")

        intent = controller.compose_intent(None)

        self.assertIsNotNone(intent, "the right press must not queue behind the left init")
        self.assertEqual(intent.right["mode"], "JointTarget")
        self.assertEqual(intent.right["joint_target_profile"], "init_motion")
        self.assertEqual(intent.right["q_target_deg"], [float(v) for v in RIGHT_Q])
        self.assertEqual(intent.right["init_motion_request_id"], controller.status_block()["right_request_id"])
        # The peer arm is left to its own exec: never our init profile (that would
        # relaunch it under our request id) and never policy motion (that would cancel it).
        self.assertEqual(intent.left, {"mode": "Hold"})
        self.assertTrue(intent.is_motion)

    def test_peer_policy_motion_never_rides_along(self):
        controller = self._controller_with_external_left()
        _press(controller, "right")
        policy = tcp_pose_target_stand_intent(
            left=(0.1, 0.2, 0.3, 0, 0, 0),
            right=(0.4, 0.5, 0.6, 0, 0, 0),
            tcp_target_profile="flow_infer_smooth",
            metadata={"chunk_id": 7},
        )

        intent = controller.compose_intent(policy)

        self.assertEqual(intent.left, {"mode": "Hold"})
        self.assertEqual(intent.right["joint_target_profile"], "init_motion")
        self.assertEqual(intent.tcp_target_profile, "flow_infer_smooth")
        self.assertEqual(intent.metadata, {"chunk_id": 7})

    def test_our_arm_goes_quiet_again_once_its_own_init_is_done(self):
        controller = self._controller_with_external_left()
        _press(controller, "right")
        controller.update_from_snapshot(
            _snapshot(
                {"left": {"status": "executing"}, "right": {"status": "done"}},
                t=2.0,
                ft=SETTLING,
            )
        )
        # Our arm is parked and the peer still owns its move: anything we could send now
        # would be a cancel, so stay silent and let the server hold both.
        self.assertIsNone(controller.compose_intent(None))

    def test_the_full_measured_sequence_no_longer_waits_for_the_peer_tare(self):
        controller = self._controller_with_external_left()
        _press(controller, "right")
        self.assertIsNotNone(controller.compose_intent(None))

        # Left reaches the init pose; its auto-tare settles for ~1 s (the window that used
        # to keep the external latch up and our request queued).
        controller.update_from_snapshot(
            _snapshot({"left": {"status": "done"}, "right": {"status": "executing"}},
                      t=2.0, ft=SETTLING)
        )
        self.assertTrue(controller.external_init_active)
        self.assertIsNotNone(
            controller.compose_intent(None),
            "our init must keep streaming through the peer's tare hold",
        )

        controller.update_from_snapshot(
            _snapshot({"left": {"status": "done"}, "right": {"status": "executing"}},
                      t=3.5, ft=ACCEPTED)
        )
        self.assertFalse(controller.external_init_active)
        intent = controller.compose_intent(None)
        self.assertEqual(intent.right["joint_target_profile"], "init_motion")


class RequestIdGroupingTest(unittest.TestCase):
    def test_one_press_that_selects_both_arms_shares_one_request_id(self):
        controller = ArmInitOverrideController()
        _press(controller, "both")
        status = controller.status_block()
        self.assertEqual(status["left_request_id"], status["right_request_id"])
        self.assertNotEqual(status["left_request_id"], 0)
        intent = controller.compose_intent(None)
        self.assertEqual(
            intent.left["init_motion_request_id"], intent.right["init_motion_request_id"]
        )

    def test_two_separate_presses_get_different_request_ids(self):
        controller = ArmInitOverrideController()
        _press(controller, "left")
        _press(controller, "right")
        status = controller.status_block()
        self.assertNotEqual(status["left_request_id"], status["right_request_id"])
        intent = controller.compose_intent(None)
        # Both arms ride in one packet (policy_runner has a single command channel); the
        # differing ids are what tells the server these are two independent requests.
        self.assertEqual(intent.left["joint_target_profile"], "init_motion")
        self.assertEqual(intent.right["joint_target_profile"], "init_motion")
        self.assertNotEqual(
            intent.left["init_motion_request_id"], intent.right["init_motion_request_id"]
        )

    def test_a_new_press_on_one_arm_keeps_the_peer_id_stable(self):
        controller = ArmInitOverrideController()
        _press(controller, "both")
        both_id = controller.status_block()["left_request_id"]
        _press(controller, "right")
        status = controller.status_block()
        self.assertEqual(status["left_request_id"], both_id)
        self.assertGreater(status["right_request_id"], both_id)


if __name__ == "__main__":
    unittest.main()
