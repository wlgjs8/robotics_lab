from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

from policy_runner.action_sources.tcp_delta import tcp_pose_target_stand_intent


REPO_ROOT = Path(__file__).resolve().parents[2]


def load_tool(name: str):
    path = REPO_ROOT / "rb_servo_server" / "tools" / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


class CartesianToolsAndDocsTest(unittest.TestCase):
    def test_tcp_pose_target_intent_accepts_quaternion_pose_object(self):
        intent = tcp_pose_target_stand_intent(
            left=[0.1, -0.2, 0.3, 0.0, 0.0, 0.70710678, 0.70710678],
            right=[0.4, 0.5, 0.6, 0.1, 0.2, 0.3],
            left_gripper=42.0,
        )

        self.assertEqual(intent.left["mode"], "TcpPoseTarget")
        self.assertEqual(intent.left["tcp_target_stand"]["x"], 0.1)
        self.assertEqual(
            intent.left["tcp_target_stand"]["quaternion_xyzw"],
            [0.0, 0.0, 0.70710678, 0.70710678],
        )
        self.assertIn("rx", intent.left["tcp_target_stand"])
        self.assertEqual(intent.left["gripper_target"], 42.0)
        self.assertEqual(intent.right["tcp_target_stand"], [0.4, 0.5, 0.6, 0.1, 0.2, 0.3])

    def test_send_tcp_twist_builds_local_and_stand_packets(self):
        tool = load_tool("send_tcp_twist")

        local = tool.build_packet(
            arm="left",
            frame="local",
            twist=[0.01, 0.0, 0.0, 0.0, 0.0, 0.1],
            timeout_sec=0.2,
            seq=10,
            source_id="test",
            session_id="session",
            lease_token="lease",
            host_time_ns=123,
        )
        stand = tool.build_packet(
            arm="both",
            frame="stand",
            twist=[0.0, 0.02, 0.0, 0.0, 0.1, 0.0],
            timeout_sec=0.2,
            seq=11,
            host_time_ns=124,
        )

        self.assertEqual(local["mode"], "Hold")
        self.assertEqual(local["left"]["mode"], "TcpTwistLocal")
        self.assertEqual(local["left"]["tcp_twist_local"], [0.01, 0.0, 0.0, 0.0, 0.0, 0.1])
        self.assertEqual(local["right"]["mode"], "Hold")
        self.assertEqual(local["source_id"], "test")
        self.assertEqual(local["session_id"], "session")
        self.assertEqual(local["lease_token"], "lease")
        self.assertEqual(stand["mode"], "TcpTwistStand")
        self.assertEqual(stand["left"]["tcp_twist_stand"], [0.0, 0.02, 0.0, 0.0, 0.1, 0.0])
        self.assertEqual(stand["right"]["tcp_twist_stand"], [0.0, 0.02, 0.0, 0.0, 0.1, 0.0])

    def test_send_tcp_linear_move_builds_packet(self):
        tool = load_tool("send_tcp_linear_move")

        packet = tool.build_packet(
            arm="right",
            target=[0.35, 0.1, 0.45, 0.0, 0.0, 0.0, 1.0],
            duration_sec=2.0,
            linear_speed_m_s=0.03,
            angular_speed_rad_s=0.2,
            orientation_mode="slerp",
            timeout_sec=0.2,
            seq=20,
            host_time_ns=123,
        )

        self.assertEqual(packet["mode"], "Hold")
        self.assertEqual(packet["left"]["mode"], "Hold")
        self.assertEqual(packet["right"]["mode"], "TcpLinearMove")
        self.assertEqual(packet["right"]["duration_sec"], 2.0)
        self.assertEqual(packet["right"]["linear_speed_m_s"], 0.03)
        self.assertEqual(packet["right"]["angular_speed_rad_s"], 0.2)
        self.assertEqual(packet["right"]["orientation_mode"], "slerp")
        self.assertEqual(
            packet["right"]["target_tcp_stand"]["quaternion_xyzw"],
            [0.0, 0.0, 0.0, 1.0],
        )

    def test_protocol_docs_contain_explicit_cartesian_units(self):
        text = (REPO_ROOT / "rb_servo_server" / "docs" / "network_protocol.md").read_text()

        for expected in (
            "TcpPoseTarget",
            "TcpLinearMove",
            "TcpTwistLocal",
            "TcpTwistStand",
            "TcpDeltaLocal",
            "TcpDeltaStand",
            "meters/second",
            "radians/second",
            "one-shot",
            "m/s",
            "rad/s",
        ):
            self.assertIn(expected, text)


if __name__ == "__main__":
    unittest.main()
