from __future__ import annotations

import json
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from policy_runner.action_sources.dual_spacemouse_cartesian import DualSpaceMouseCartesianActionSource
from policy_runner.action_sources.spacemouse_cartesian import SpaceMouseCartesianActionSource
from policy_runner.action_sources.tcp_delta import TcpDeltaActionSource
from policy_runner.config import SafetyConfig, config_from_mapping
from policy_runner.geometry import geometry_status_from_mapping
from policy_runner.robot_state_client import StateSnapshot
from policy_runner.safety import SafetyGate
from policy_runner.servo_command_client import ServoCommandClient
from policy_runner.spacemouse import FakeSpaceMouseReader, SpaceMouseSample


def sample_state(**overrides):
    payload = {
        "schema_version": 1,
        "host_time_ns": 123,
        "observed_mode": "simulation",
        "observed_backend": "simulator",
        "motion_state": "ConnectedHold",
        "fault_latched": False,
        "left": {
            "has_valid_joint_state": True,
            "has_valid_tcp_pose": True,
            "q_actual_deg": [0, -30, 80, 0, 60, 0],
        },
        "right": {
            "has_valid_joint_state": True,
            "has_valid_tcp_pose": True,
            "q_actual_deg": [0, -30, 80, 0, 60, 0],
        },
    }
    payload.update(overrides)
    return StateSnapshot(payload=payload, received_monotonic=time.monotonic())


def spacemouse_sample(
    tx=0.0,
    ty=0.0,
    tz=0.0,
    rx=0.0,
    ry=0.0,
    rz=0.0,
    buttons=(True,),
):
    return SpaceMouseSample(
        tx=tx,
        ty=ty,
        tz=tz,
        rx=rx,
        ry=ry,
        rz=rz,
        buttons=tuple(buttons),
        timestamp_monotonic=time.monotonic(),
    )


def configured_estimate_geometry():
    return geometry_status_from_mapping(
        {
            "calibration_id": "CONFIG_ESTIMATE_TEST",
            "status": "configured_estimate",
            "geometry_valid_for_real_policy": False,
            "robot": {
                "T_stand_left_base": {
                    "parent": "stand",
                    "child": "left_base",
                    "xyz_rpy": [0.1, 0.2, 0.3, 0.0, 0.0, 0.0],
                    "status": "configured_estimate",
                },
                "T_stand_right_base": {
                    "parent": "stand",
                    "child": "right_base",
                    "xyz_rpy": [-0.1, 0.2, 0.3, 0.0, 0.0, 0.0],
                    "status": "configured_estimate",
                },
            },
        }
    )


class FakeSendSocket:
    def __init__(self):
        self.sent = []
        self.closed = False

    def sendto(self, data, address):
        self.sent.append((data, address))
        return len(data)

    def close(self):
        self.closed = True


class CartesianActionSourceTest(unittest.TestCase):
    def test_fake_spacemouse_input_creates_deterministic_tcp_twist_local_packet(self):
        reader = FakeSpaceMouseReader(
            [
                spacemouse_sample(
                    tx=0.5,
                    ty=-1.0,
                    tz=2.0,
                    rx=-2.0,
                    ry=0.1,
                    rz=0.0,
                )
            ]
        )
        source = SpaceMouseCartesianActionSource(reader=reader, selected_arm="left")
        snapshot = sample_state()

        intent = source.next_intent(snapshot, time.monotonic())
        gate = SafetyGate(
            "simulation",
            SafetyConfig(),
            stale_timeout_sec=0.5,
            geometry_status=configured_estimate_geometry(),
        )
        decision = gate.evaluate(snapshot, intent, source.requirements, time.monotonic())

        self.assertTrue(decision.allowed)
        self.assertIsNotNone(intent)
        assert intent is not None
        self.assertTrue(intent.is_motion)
        fake_socket = FakeSendSocket()
        client = ServoCommandClient("udp://127.0.0.1:50010", socket_factory=lambda *_args: fake_socket)
        try:
            client.send(intent)
        finally:
            client.close()
        packet = json.loads(fake_socket.sent[0][0].decode("utf-8"))
        self.assertEqual(packet["mode"], "TcpTwistLocal")
        self.assertEqual(packet["left"]["mode"], "TcpTwistLocal")
        self.assertEqual(packet["left"]["tcp_twist_local"], [0.015, -0.03, 0.03, -0.2, 0.020000000000000004, 0.0])
        self.assertEqual(packet["right"]["mode"], "Hold")

    def test_deadman_false_sends_no_command(self):
        reader = FakeSpaceMouseReader([spacemouse_sample(tx=1.0, buttons=(False,))])
        source = SpaceMouseCartesianActionSource(reader=reader, require_deadman=True)

        self.assertIsNone(source.next_intent(sample_state(), time.monotonic()))

    def test_spacemouse_cartesian_clamps_linear_and_angular_velocity(self):
        reader = FakeSpaceMouseReader([spacemouse_sample(tx=10.0, ty=-10.0, rz=10.0)])
        source = SpaceMouseCartesianActionSource(
            reader=reader,
            max_linear_velocity_m_s=0.03,
            max_angular_velocity_rad_s=0.2,
        )

        intent = source.next_intent(sample_state(), time.monotonic())

        self.assertIsNotNone(intent)
        assert intent is not None
        self.assertEqual(intent.left["tcp_twist_local"], [0.03, -0.03, 0.0, 0.0, 0.0, 0.2])

    def test_spacemouse_cartesian_deadman_cannot_be_disabled(self):
        reader = FakeSpaceMouseReader([spacemouse_sample(tx=1.0)])

        with self.assertRaisesRegex(ValueError, "requires deadman"):
            SpaceMouseCartesianActionSource(reader=reader, require_deadman=False)

    def test_dual_spacemouse_cartesian_sends_both_arms_in_one_packet(self):
        left_reader = FakeSpaceMouseReader([spacemouse_sample(tx=0.5, ry=1.0)])
        right_reader = FakeSpaceMouseReader([spacemouse_sample(ty=-1.0, rz=2.0)])
        source = DualSpaceMouseCartesianActionSource(
            left_reader=left_reader,
            right_reader=right_reader,
            max_linear_velocity_m_s=0.03,
            max_angular_velocity_rad_s=0.2,
        )

        intent = source.next_intent(sample_state(), time.monotonic())

        self.assertIsNotNone(intent)
        assert intent is not None
        self.assertEqual(intent.mode, "TcpTwistLocal")
        self.assertEqual(intent.left["mode"], "TcpTwistLocal")
        self.assertEqual(intent.right["mode"], "TcpTwistLocal")
        self.assertEqual(intent.left["tcp_twist_local"], [0.015, 0.0, 0.0, 0.0, 0.2, 0.0])
        self.assertEqual(intent.right["tcp_twist_local"], [0.0, -0.03, 0.0, 0.0, 0.0, 0.2])

    def test_dual_spacemouse_cartesian_holds_inactive_arm(self):
        left_reader = FakeSpaceMouseReader([spacemouse_sample(tx=1.0)])
        right_reader = FakeSpaceMouseReader([spacemouse_sample(ty=1.0, buttons=(False,))])
        source = DualSpaceMouseCartesianActionSource(
            left_reader=left_reader,
            right_reader=right_reader,
        )

        intent = source.next_intent(sample_state(), time.monotonic())

        self.assertIsNotNone(intent)
        assert intent is not None
        self.assertEqual(intent.left["tcp_twist_local"], [0.03, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.assertEqual(intent.right["mode"], "Hold")

    def test_dual_spacemouse_cartesian_sends_no_command_when_both_inactive(self):
        source = DualSpaceMouseCartesianActionSource(
            left_reader=FakeSpaceMouseReader([spacemouse_sample(tx=1.0, buttons=(False,))]),
            right_reader=FakeSpaceMouseReader(),
        )

        self.assertIsNone(source.next_intent(sample_state(), time.monotonic()))

    def test_dual_spacemouse_cartesian_clamps_per_arm(self):
        source = DualSpaceMouseCartesianActionSource(
            left_reader=FakeSpaceMouseReader([spacemouse_sample(tx=10.0, rx=-10.0)]),
            right_reader=FakeSpaceMouseReader([spacemouse_sample(tz=-10.0, rz=10.0)]),
            max_linear_velocity_m_s=0.03,
            max_angular_velocity_rad_s=0.2,
        )

        intent = source.next_intent(sample_state(), time.monotonic())

        self.assertIsNotNone(intent)
        assert intent is not None
        self.assertEqual(intent.left["tcp_twist_local"], [0.03, 0.0, 0.0, -0.2, 0.0, 0.0])
        self.assertEqual(intent.right["tcp_twist_local"], [0.0, 0.0, -0.03, 0.0, 0.0, 0.2])

    def test_scripted_tcp_delta_is_stand_frame_and_clamped(self):
        source = TcpDeltaActionSource(
            delta=(0.1, -0.1, 0.0, 0.1, 0.0, -0.1),
            selected_arm="both",
            max_linear_step_m=0.002,
            max_angular_step_rad=0.01,
        )

        intent = source.next_intent(sample_state(), time.monotonic())

        self.assertEqual(intent.mode, "TcpDeltaStand")
        self.assertEqual(intent.left["tcp_delta_stand"], [0.002, -0.002, 0.0, 0.01, 0.0, -0.01])
        self.assertEqual(intent.right["tcp_delta_stand"], [0.002, -0.002, 0.0, 0.01, 0.0, -0.01])

    def test_real_mode_blocks_cartesian_even_when_joint_real_motion_allowed(self):
        source = TcpDeltaActionSource()
        snapshot = sample_state(observed_mode="real", observed_backend="rbpodo")
        intent = source.next_intent(snapshot, time.monotonic())
        gate = SafetyGate(
            "real",
            SafetyConfig(allow_real_motion=True),
            stale_timeout_sec=0.5,
            geometry_status=configured_estimate_geometry(),
        )

        decision = gate.evaluate(snapshot, intent, source.requirements, time.monotonic())

        self.assertFalse(decision.allowed)
        self.assertEqual(decision.reason, "real_cartesian_not_allowed")

    def test_real_mode_blocks_spacemouse_cartesian_by_default(self):
        reader = FakeSpaceMouseReader([spacemouse_sample(tx=1.0)])
        source = SpaceMouseCartesianActionSource(reader=reader)
        snapshot = sample_state(observed_mode="real", observed_backend="rbpodo")
        intent = source.next_intent(snapshot, time.monotonic())
        gate = SafetyGate(
            "real",
            SafetyConfig(allow_real_motion=False),
            stale_timeout_sec=0.5,
            geometry_status=configured_estimate_geometry(),
        )

        decision = gate.evaluate(snapshot, intent, source.requirements, time.monotonic())

        self.assertFalse(decision.allowed)
        self.assertEqual(decision.reason, "real_cartesian_not_allowed")

    def test_stale_and_fault_state_block_cartesian_command(self):
        source = TcpDeltaActionSource()
        gate = SafetyGate(
            "simulation",
            SafetyConfig(),
            stale_timeout_sec=0.01,
            geometry_status=configured_estimate_geometry(),
        )
        stale = StateSnapshot(sample_state().payload, time.monotonic() - 1.0)
        fault = sample_state(fault_latched=True)
        intent = source.next_intent(sample_state(), time.monotonic())

        self.assertEqual(gate.evaluate(stale, intent, source.requirements).reason, "state_stream_stale")
        self.assertEqual(gate.evaluate(fault, intent, source.requirements).reason, "fault_latched")

    def test_missing_tcp_state_and_non_simulator_backend_block_cartesian_command(self):
        source = TcpDeltaActionSource()
        gate = SafetyGate(
            "simulation",
            SafetyConfig(),
            stale_timeout_sec=0.5,
            geometry_status=configured_estimate_geometry(),
        )
        intent = source.next_intent(sample_state(), time.monotonic())
        missing_tcp = sample_state(
            right={
                "has_valid_joint_state": True,
                "has_valid_tcp_pose": False,
                "q_actual_deg": [0, -30, 80, 0, 60, 0],
            }
        )
        wrong_backend = sample_state(observed_backend="rbpodo")

        self.assertEqual(gate.evaluate(missing_tcp, intent, source.requirements).reason, "invalid_tcp_pose")
        self.assertEqual(
            gate.evaluate(wrong_backend, intent, source.requirements).reason,
            "observed_backend_not_simulator",
        )

    def test_cartesian_unavailable_verdict_blocks_followup_command(self):
        source = TcpDeltaActionSource()
        gate = SafetyGate(
            "simulation",
            SafetyConfig(),
            stale_timeout_sec=0.5,
            geometry_status=configured_estimate_geometry(),
        )
        intent = source.next_intent(sample_state(), time.monotonic())
        unavailable = sample_state(safety_verdict="CartesianUnavailable")

        decision = gate.evaluate(unavailable, intent, source.requirements, time.monotonic())

        self.assertFalse(decision.allowed)
        self.assertEqual(decision.reason, "cartesian_unavailable")

    def test_spacemouse_cartesian_config_fields_parse(self):
        cfg = config_from_mapping(
            {
                "schema": "robotics_lab.policy_runner.v1",
                "action_source": "spacemouse_cartesian",
                "spacemouse_cartesian": {
                    "frame": "local",
                    "command_rate_hz": 30,
                    "max_linear_velocity_m_s": 0.03,
                    "max_angular_velocity_rad_s": 0.2,
                    "deadband": 0.08,
                    "require_deadman": True,
                    "deadman_button": 0,
                },
            }
        )

        self.assertEqual(cfg.action_source, "spacemouse_cartesian")
        self.assertEqual(cfg.spacemouse_cartesian.frame, "local")
        self.assertEqual(cfg.spacemouse_cartesian.command_rate_hz, 30.0)
        self.assertEqual(cfg.spacemouse_cartesian.max_linear_velocity_m_s, 0.03)
        self.assertEqual(cfg.spacemouse_cartesian.max_angular_velocity_rad_s, 0.2)
        self.assertTrue(cfg.spacemouse_cartesian.require_deadman)

    def test_spacemouse_cartesian_deprecated_step_aliases_warn(self):
        with self.assertWarns(DeprecationWarning):
            cfg = config_from_mapping(
                {
                    "schema": "robotics_lab.policy_runner.v1",
                    "action_source": "spacemouse_cartesian",
                    "spacemouse_cartesian": {
                        "max_linear_step_m": 0.002,
                        "max_angular_step_rad": 0.01,
                    },
                }
            )

        self.assertEqual(cfg.spacemouse_cartesian.max_linear_velocity_m_s, 0.002)
        self.assertEqual(cfg.spacemouse_cartesian.max_angular_velocity_rad_s, 0.01)

    def test_dual_spacemouse_cartesian_config_fields_parse(self):
        cfg = config_from_mapping(
            {
                "schema": "robotics_lab.policy_runner.v1",
                "action_source": "dual_spacemouse_cartesian",
                "spacemouse_cartesian_dual": {
                    "frame": "local",
                    "max_linear_velocity_m_s": 0.03,
                    "max_angular_velocity_rad_s": 0.2,
                    "deadband": 0.05,
                    "left": {
                        "path": "/dev/hidraw-left",
                        "device_number": 0,
                        "deadman_button": 0,
                    },
                    "right": {
                        "path": "/dev/hidraw-right",
                        "device_number": 2,
                        "deadman_button": 1,
                    },
                },
            }
        )

        self.assertEqual(cfg.action_source, "dual_spacemouse_cartesian")
        self.assertEqual(cfg.spacemouse_cartesian_dual.frame, "local")
        self.assertEqual(cfg.spacemouse_cartesian_dual.max_linear_velocity_m_s, 0.03)
        self.assertEqual(cfg.spacemouse_cartesian_dual.max_angular_velocity_rad_s, 0.2)
        self.assertEqual(cfg.spacemouse_cartesian_dual.deadband, 0.05)
        self.assertEqual(cfg.spacemouse_cartesian_dual.left.path, "/dev/hidraw-left")
        self.assertEqual(cfg.spacemouse_cartesian_dual.right.device_number, 2)
        self.assertEqual(cfg.spacemouse_cartesian_dual.right.deadman_button, 1)

    def test_dual_spacemouse_cartesian_invalid_device_config_fails(self):
        with self.assertRaisesRegex(ValueError, "device_number"):
            config_from_mapping(
                {
                    "schema": "robotics_lab.policy_runner.v1",
                    "spacemouse_cartesian_dual": {"left": {"device_number": -1}},
                }
            )


if __name__ == "__main__":
    unittest.main()
