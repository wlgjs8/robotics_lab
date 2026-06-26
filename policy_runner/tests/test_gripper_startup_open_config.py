"""GripperConfig.startup_open parsing + default (no torch / hardware needed).

Guards the "hold current gripper position on server start" behavior: the default
must be OFF (hold), and the yaml flag must round-trip so the legacy open-at-startup
can be restored explicitly.
"""
import unittest

from policy_runner.config import GripperConfig, config_from_mapping


class GripperStartupOpenConfigTest(unittest.TestCase):
    def test_default_holds_current_position(self) -> None:
        # Default OFF: the server start must not force the asymmetric
        # right-opens/left-closes startup-open move.
        self.assertFalse(GripperConfig().startup_open)

    def test_startup_open_true_parses(self) -> None:
        cfg = config_from_mapping(
            {
                "mode": "real",
                "gripper": {"backend": "pika_serial", "startup_open": True},
            }
        )
        self.assertTrue(cfg.gripper.startup_open)

    def test_startup_open_false_parses(self) -> None:
        cfg = config_from_mapping(
            {
                "mode": "real",
                "gripper": {"backend": "pika_serial", "startup_open": False},
            }
        )
        self.assertFalse(cfg.gripper.startup_open)


if __name__ == "__main__":
    unittest.main()
