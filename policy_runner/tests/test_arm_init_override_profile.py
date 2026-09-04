"""A single-arm InitMotion override must not change the PEER arm's TcpPoseTarget profile.

Measured 2026-09-04 (servo_log_20260904_003600.csv, 148.554 s): the override
controller rebuilt the CommandIntent without the top-level ``tcp_target_profile``,
the packet fell back to the server default profile (no chunk follower), and the
streaming peer arm lost its chunk follower on the override's first tick.
"""

from __future__ import annotations

import unittest

from policy_runner.action_sources.tcp_pose_target import tcp_pose_target_stand_intent
from policy_runner.arm_init_control import (
    ARM_INIT_COMMAND_SCHEMA,
    ArmInitOverrideController,
    parse_arm_init_command,
)


def _start_right_override() -> ArmInitOverrideController:
    controller = ArmInitOverrideController()
    controller.handle_command(
        parse_arm_init_command(
            {
                "schema": ARM_INIT_COMMAND_SCHEMA,
                "arms": "right",
                "action": "start",
                "right_q_deg": [0, 1, 0, 0, 0, 0],
            }
        )
    )
    return controller


class ArmInitOverrideProfilePassthroughTest(unittest.TestCase):
    def test_single_arm_override_keeps_peer_profile_and_metadata(self):
        controller = _start_right_override()
        source = tcp_pose_target_stand_intent(
            left=(0.1, 0.2, 0.3, 0, 0, 0),
            right=(0.4, 0.5, 0.6, 0, 0, 0),
            tcp_target_profile="flow_infer_smooth",
            metadata={"chunk_id": 7},
        )

        mixed = controller.compose_intent(source)

        self.assertIsNotNone(mixed)
        self.assertEqual(mixed.right["mode"], "JointTarget")
        self.assertEqual(mixed.left["mode"], "TcpPoseTarget")
        self.assertEqual(mixed.left["tcp_target_stand"], [0.1, 0.2, 0.3, 0.0, 0.0, 0.0])
        self.assertEqual(mixed.tcp_target_profile, "flow_infer_smooth")
        self.assertEqual(mixed.metadata, {"chunk_id": 7})
        self.assertEqual(mixed.timeout_sec, source.timeout_sec)

    def test_override_without_source_intent_has_no_profile(self):
        controller = _start_right_override()

        mixed = controller.compose_intent(None)

        self.assertIsNotNone(mixed)
        self.assertEqual(mixed.right["mode"], "JointTarget")
        self.assertEqual(mixed.left["mode"], "Hold")
        self.assertIsNone(mixed.tcp_target_profile)
        self.assertIsNone(mixed.metadata)

    def test_passthrough_when_no_override_is_active(self):
        controller = ArmInitOverrideController()
        source = tcp_pose_target_stand_intent(
            left=(0.1, 0.2, 0.3, 0, 0, 0),
            tcp_target_profile="flow_infer_smooth",
        )

        self.assertIs(controller.compose_intent(source), source)


if __name__ == "__main__":
    unittest.main()
