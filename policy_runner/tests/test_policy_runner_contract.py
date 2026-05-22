from __future__ import annotations

import json
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from policy_runner.action_sources.hold import HoldActionSource
from policy_runner.action_sources.joint_sine import JointSineActionSource
from policy_runner.action_sources.joint_velocity import JointVelocityActionSource
from policy_runner.config import load_config
from policy_runner.robot_state_client import RobotStateClient, StateSnapshot
from policy_runner.safety import ActionRequirements, SafetyConfig, SafetyGate
from policy_runner.servo_command_client import CommandIntent, ServoCommandClient


def sample_state(**overrides):
    payload = {
        "schema_version": 1,
        "host_time_ns": 123,
        "motion_state": "ConnectedHold",
        "fault_latched": False,
        "left": {"has_valid_joint_state": True, "q_actual_deg": [0, -30, 80, 0, 60, 0]},
        "right": {"has_valid_joint_state": True, "q_actual_deg": [0, -30, 80, 0, 60, 0]},
    }
    payload.update(overrides)
    return StateSnapshot(payload=payload, received_monotonic=time.monotonic())


class FakeRecvSocket:
    def __init__(self, packets):
        self.packets = list(packets)
        self.bound = None
        self.timeout = None
        self.closed = False

    def bind(self, address):
        self.bound = address

    def getsockname(self):
        host, port = self.bound
        return (host, 50120 if port == 0 else port)

    def settimeout(self, timeout):
        self.timeout = timeout

    def recvfrom(self, _size):
        if not self.packets:
            raise TimeoutError("no fake packet queued")
        return self.packets.pop(0), ("127.0.0.1", 50000)

    def close(self):
        self.closed = True


class FakeSendSocket:
    def __init__(self):
        self.sent = []
        self.closed = False

    def sendto(self, data, address):
        self.sent.append((data, address))
        return len(data)

    def close(self):
        self.closed = True


class PolicyRunnerContractTest(unittest.TestCase):
    def test_config_example_loads_without_yaml_dependency(self):
        cfg = load_config(Path(__file__).resolve().parents[1] / "config" / "simulator_hold.yaml")
        self.assertEqual(cfg.action_source, "hold")
        self.assertFalse(cfg.safety.allow_real_motion)

    def test_udp_state_subscriber_receives_latest_snapshot(self):
        packet = json.dumps(sample_state().payload).encode("utf-8")
        fake_socket = FakeRecvSocket([packet])
        client = RobotStateClient(
            "udp://127.0.0.1:0",
            stale_timeout_sec=0.5,
            socket_factory=lambda *_args: fake_socket,
        )
        client.open()
        try:
            snapshot = client.poll_once(timeout_sec=0.5)
            self.assertIsNotNone(snapshot)
            self.assertEqual(snapshot.payload["motion_state"], "ConnectedHold")
            self.assertFalse(client.is_latest_stale())
        finally:
            client.close()

    def test_command_sender_emits_rb_servo_compatible_joint_velocity_packet(self):
        fake_socket = FakeSendSocket()
        client = ServoCommandClient(
            "udp://127.0.0.1:50010",
            socket_factory=lambda *_args: fake_socket,
        )
        try:
            seq = client.send(CommandIntent.joint_velocity(left=[1, 0, 0, 0, 0, 0], right=[-1, 0, 0, 0, 0, 0]))
            data, address = fake_socket.sent[0]
            packet = json.loads(data.decode("utf-8"))
            self.assertEqual(seq, 1)
            self.assertEqual(address, ("127.0.0.1", 50010))
            self.assertEqual(packet["seq"], 1)
            self.assertEqual(packet["mode"], "Hold")
            self.assertEqual(packet["timeout_sec"], 0.2)
            self.assertEqual(packet["left"]["mode"], "JointVelocity")
            self.assertEqual(packet["left"]["dq_target_deg_s"], [1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            self.assertEqual(packet["right"]["mode"], "JointVelocity")
        finally:
            client.close()

    def test_command_packet_schemas_match_rb_servo_parser_contract(self):
        client = ServoCommandClient("udp://127.0.0.1:50010", socket_factory=lambda *_args: FakeSendSocket())
        cases = [
            CommandIntent.hold(),
            CommandIntent.arm_motion(),
            CommandIntent.disarm_motion(),
            CommandIntent.emergency_stop(),
            CommandIntent.reset_fault(),
            CommandIntent.joint_target(left=[0, 1, 2, 3, 4, 5], right=[5, 4, 3, 2, 1, 0]),
            CommandIntent.joint_velocity(left=[1, 0, 0, 0, 0, 0], right=[0, 1, 0, 0, 0, 0]),
        ]
        for seq, intent in enumerate(cases, start=1):
            with self.subTest(mode=intent.mode):
                packet = client.build_packet(intent, seq)
                self.assertEqual(packet["seq"], seq)
                self.assertIn("mode", packet)
                self.assertIn("timeout_sec", packet)
                self.assertGreater(packet["timeout_sec"], 0.0)
        target = client.build_packet(cases[-2], 6)
        self.assertEqual(target["left"]["mode"], "JointTarget")
        self.assertEqual(target["left"]["q_target_deg"], [0.0, 1.0, 2.0, 3.0, 4.0, 5.0])
        velocity = client.build_packet(cases[-1], 7)
        self.assertEqual(velocity["right"]["mode"], "JointVelocity")
        self.assertEqual(velocity["right"]["dq_target_deg_s"], [0.0, 1.0, 0.0, 0.0, 0.0, 0.0])

    def test_hold_receives_state_and_sends_no_motion_by_default(self):
        source = HoldActionSource()
        self.assertIsNone(source.next_intent(sample_state(), time.monotonic()))

    def test_joint_sources_are_simulation_only_by_default(self):
        sine = JointSineActionSource((1, 1, 1, 1, 1, 1))
        velocity = JointVelocityActionSource((1, 0, 0, 0, 0, 0))
        self.assertTrue(sine.requirements.simulation_only)
        self.assertTrue(velocity.requirements.simulation_only)

    def test_real_mode_blocks_motion_without_explicit_allow(self):
        gate = SafetyGate("real", SafetyConfig(allow_real_motion=False), stale_timeout_sec=0.5)
        intent = CommandIntent.joint_velocity(left=[1, 0, 0, 0, 0, 0], right=[1, 0, 0, 0, 0, 0])
        decision = gate.evaluate(sample_state(), intent, now_monotonic=time.monotonic())
        self.assertFalse(decision.allowed)
        self.assertEqual(decision.reason, "real_motion_not_allowed")

    def test_observed_real_mode_blocks_motion_without_explicit_allow(self):
        gate = SafetyGate("simulation", SafetyConfig(allow_real_motion=False), stale_timeout_sec=0.5)
        intent = CommandIntent.joint_velocity(left=[1, 0, 0, 0, 0, 0], right=[1, 0, 0, 0, 0, 0])
        decision = gate.evaluate(sample_state(run_mode="real"), intent, now_monotonic=time.monotonic())
        self.assertFalse(decision.allowed)
        self.assertEqual(decision.reason, "real_motion_not_allowed")

    def test_safety_blocks_stale_fault_and_invalid_state(self):
        gate = SafetyGate("simulation", SafetyConfig(), stale_timeout_sec=0.01)
        intent = CommandIntent.joint_velocity(left=[1, 0, 0, 0, 0, 0], right=[1, 0, 0, 0, 0, 0])
        stale = StateSnapshot(sample_state().payload, time.monotonic() - 1.0)
        self.assertEqual(gate.evaluate(stale, intent).reason, "state_stream_stale")
        self.assertEqual(gate.evaluate(sample_state(fault_latched=True), intent).reason, "fault_latched")
        invalid = sample_state(left={"has_valid_joint_state": False, "q_actual_deg": [0, 0, 0, 0, 0, 0]})
        self.assertEqual(gate.evaluate(invalid, intent).reason, "invalid_joint_state")

    def test_safety_blocks_missing_camera_or_kinematics_requirements(self):
        intent = CommandIntent.joint_velocity(left=[1, 0, 0, 0, 0, 0], right=[1, 0, 0, 0, 0, 0])
        camera_gate = SafetyGate("simulation", SafetyConfig(camera_available=False), stale_timeout_sec=0.5)
        camera_decision = camera_gate.evaluate(
            sample_state(),
            intent,
            ActionRequirements(requires_camera=True),
        )
        self.assertEqual(camera_decision.reason, "camera_unavailable")
        kinematics_gate = SafetyGate("simulation", SafetyConfig(kinematics_available=False), stale_timeout_sec=0.5)
        kinematics_decision = kinematics_gate.evaluate(
            sample_state(),
            intent,
            ActionRequirements(requires_kinematics=True),
        )
        self.assertEqual(kinematics_decision.reason, "kinematics_unavailable")

    def test_joint_sine_uses_current_joint_state(self):
        source = JointSineActionSource((1, 1, 1, 1, 1, 1), frequency_hz=0.25, selected_arm="left")
        state = sample_state()
        source.next_intent(state, 10.0)
        intent = source.next_intent(state, 11.0)
        self.assertEqual(intent.left["mode"], "JointTarget")
        self.assertEqual(intent.right["mode"], "Hold")
        self.assertEqual(len(intent.left["q_target_deg"]), 6)


if __name__ == "__main__":
    unittest.main()
