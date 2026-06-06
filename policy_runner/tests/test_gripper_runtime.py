from __future__ import annotations

import unittest

from policy_runner.config import config_from_mapping
from policy_runner.gripper import (
    GripperCommand,
    GripperRuntime,
    NoopGripperBackend,
    REAL_GRIPPER_ENV,
    gripper_commands_from_flow_step,
)


class GripperRuntimeTest(unittest.TestCase):
    def test_real_gripper_env_absent_rejects_command(self) -> None:
        runtime = GripperRuntime(
            rollout_mode="real_policy",
            allow_real_gripper_motion=True,
            env={},
        )

        results = runtime.dispatch([GripperCommand("left", 0.25, command_type="target")])

        self.assertEqual(len(results), 1)
        self.assertTrue(results[0].dropped)
        self.assertFalse(results[0].sent_to_physical)
        self.assertEqual(results[0].reason, "real_gripper_env_missing")
        self.assertEqual(runtime.command_count, 1)
        self.assertEqual(runtime.dropped_count, 1)

    def test_config_allow_real_gripper_motion_false_rejects_even_with_env(self) -> None:
        cfg = config_from_mapping(
            {
                "schema": "robotics_lab.policy_runner.v1",
                "safety": {"allow_real_gripper_motion": False},
            }
        )
        runtime = GripperRuntime(
            rollout_mode="real_policy",
            allow_real_gripper_motion=cfg.safety.allow_real_gripper_motion,
            env={REAL_GRIPPER_ENV: "1"},
        )

        results = runtime.dispatch([GripperCommand("right", -0.1)])

        self.assertEqual(len(results), 1)
        self.assertTrue(results[0].dropped)
        self.assertFalse(results[0].sent_to_physical)
        self.assertEqual(results[0].reason, "real_gripper_config_not_allowed")

    def test_controller_sim_logs_and_drops_gripper_without_physical_command(self) -> None:
        backend = NoopGripperBackend()
        runtime = GripperRuntime(
            rollout_mode="controller_sim",
            allow_real_gripper_motion=False,
            backend=backend,
            env={REAL_GRIPPER_ENV: "1"},
        )

        results = runtime.dispatch([GripperCommand("left", 0.4)])

        self.assertEqual(len(results), 1)
        self.assertTrue(results[0].dropped)
        self.assertFalse(results[0].sent_to_physical)
        self.assertEqual(results[0].reason, "controller_sim_gripper_logged_noop")
        self.assertEqual(backend.commands, [])
        self.assertEqual(runtime.command_count, 1)
        self.assertEqual(runtime.dropped_count, 1)

    def test_noop_backend_records_dry_run_command_without_physical_send(self) -> None:
        backend = NoopGripperBackend()
        command = GripperCommand("right", 1.0, command_type="target")

        result = backend.send(command)

        self.assertTrue(result.dropped)
        self.assertFalse(result.sent_to_physical)
        self.assertEqual(backend.commands, [command])

    def test_flow_step_maps_gripper_channels_separately_from_cartesian(self) -> None:
        step = [0.0] * 14
        step[6] = 0.2
        step[13] = -0.3

        commands = gripper_commands_from_flow_step(step, arm_mask=[1.0, 1.0])

        self.assertEqual([command.arm for command in commands], ["left", "right"])
        self.assertEqual([command.value for command in commands], [0.2, -0.3])
        self.assertTrue(all(command.command_type == "delta" for command in commands))


if __name__ == "__main__":
    unittest.main()
