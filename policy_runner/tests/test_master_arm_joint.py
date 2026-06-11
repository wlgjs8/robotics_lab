from __future__ import annotations

import math
import sys
import time
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from policy_runner.action_sources.master_arm_joint import MasterArmJointActionSource
from policy_runner.config import load_config
from policy_runner.main import make_action_source
from policy_runner.robot_state_client import StateSnapshot
from policy_runner.safety import SafetyGate
from policy_runner.servo_command_client import CommandIntent


def sample_state(
    *,
    left_q: list[float] | None = None,
    include_right: bool = True,
) -> StateSnapshot:
    left_q = [0.0, -30.0, 80.0, 0.0, 60.0, 0.0] if left_q is None else left_q
    payload = {
        "schema_version": 1,
        "host_time_ns": 123,
        "motion_state": "ConnectedHold",
        "fault_latched": False,
        "observed_mode": "real",
        "left": {"has_valid_joint_state": True, "q_actual_deg": left_q},
    }
    if include_right:
        payload["right"] = {
            "has_valid_joint_state": True,
            "q_actual_deg": [0.0, -30.0, 80.0, 0.0, 60.0, 0.0],
        }
    return StateSnapshot(payload=payload, received_monotonic=time.monotonic())


class FakeMasterArmSession:
    instances: list["FakeMasterArmSession"] = []

    def __init__(
        self,
        config_path: str,
        enable_switches: bool,
        enable_gripper_readers: bool,
    ):
        self.config_path = config_path
        self.enable_switches = enable_switches
        self.enable_gripper_readers = enable_gripper_readers
        self.positions = {"left": [0.0] * 6}
        self.switches = {"left": False}
        self.move_init_calls: list[tuple[str, bool]] = []
        self.closed = False
        FakeMasterArmSession.instances.append(self)

    def move_init_joint(self, side: str, blocking: bool) -> None:
        self.move_init_calls.append((side, blocking))

    def read_all(self) -> None:
        return None

    def get_joint_positions(self, side: str) -> list[float]:
        return list(self.positions[side])

    def get_pressed_state(self, side: str) -> bool:
        return True

    def get_switch_status(self, side: str, switch: int) -> bool:
        _ = switch
        return self.switches[side]

    def close(self) -> None:
        self.closed = True


class MasterArmJointActionSourceTest(unittest.TestCase):
    def setUp(self) -> None:
        FakeMasterArmSession.instances.clear()
        self.module_name = "_fake_mo_master_arm_py_for_test"
        sys.modules[self.module_name] = types.SimpleNamespace(
            MasterArmSession=FakeMasterArmSession
        )

    def tearDown(self) -> None:
        sys.modules.pop(self.module_name, None)

    def test_real_example_config_loads_motion_blocked(self):
        cfg = load_config(
            Path(__file__).resolve().parents[1]
            / "config"
            / "real_master_arm_joint.yaml"
        )
        self.assertEqual(cfg.mode, "real")
        self.assertEqual(cfg.action_source, "master_arm_joint")
        self.assertFalse(cfg.safety.allow_real_motion)

        source = make_action_source(cfg)
        self.assertIsInstance(source, MasterArmJointActionSource)

        gate = SafetyGate(cfg.mode, cfg.safety, cfg.robot_state.stale_timeout_sec)
        decision = gate.evaluate(
            sample_state(),
            CommandIntent.joint_target(left=[0, 0, 0, 0, 0, 0]),
            source.requirements,
            time.monotonic(),
        )
        # Real/sim gating retired: real motion allowed without the config flag.
        self.assertTrue(decision.allowed)

    def test_left_delta_latch_does_not_require_inactive_right_state(self):
        source = MasterArmJointActionSource(
            config_path="/tmp/master.yaml",
            module_name=self.module_name,
            selected_arm="left",
            deadman_side="left",
            use_gravity_compensation=False,
            max_joint_velocity_deg_s=(100.0, 100.0, 100.0, 100.0, 100.0, 100.0),
            smoothing_alpha=1.0,
        )
        initial_left = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        intent = source.next_intent(
            sample_state(left_q=initial_left, include_right=False),
            now_monotonic=10.0,
        )
        self.assertEqual(intent.left["q_target_deg"], initial_left)
        self.assertEqual(intent.right, {"mode": "Hold"})

        session = FakeMasterArmSession.instances[-1]
        self.assertEqual(session.move_init_calls, [("left", True)])
        session.switches["left"] = True

        latched = source.next_intent(
            sample_state(left_q=initial_left, include_right=False),
            now_monotonic=11.0,
        )
        self.assertEqual(latched.left["q_target_deg"], initial_left)
        self.assertEqual(latched.right, {"mode": "Hold"})

        session.positions["left"] = [math.radians(10.0)] * 6
        moved = source.next_intent(
            sample_state(left_q=initial_left, include_right=False),
            now_monotonic=12.0,
        )
        self.assertEqual(moved.left["q_target_deg"], [11.0, 12.0, 13.0, 14.0, 15.0, 16.0])
        self.assertEqual(moved.right, {"mode": "Hold"})


if __name__ == "__main__":
    unittest.main()
