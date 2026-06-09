from __future__ import annotations

import json
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from policy_runner.action_sources.umi_dual_cartesian import (  # noqa: E402
    MockUmiPoseReader,
    UdpUmiPoseReader,
    UmiDualCartesianActionSource,
    UmiSample,
)
from policy_runner.config import config_from_mapping  # noqa: E402
from policy_runner.main import make_action_source, run  # noqa: E402
from policy_runner.robot_state_client import StateSnapshot  # noqa: E402
from policy_runner.servo_command_client import ServoCommandClient  # noqa: E402


def controller_sim_cartesian_gate(**overrides):
    gate = {
        "run_mode": "real",
        "backend_type": "rbpodo",
        "operation_mode": "simulation",
        "allow_in_controller_simulation": True,
        "allow_controller_simulation_motion": True,
        "env_RB_ALLOW_REAL_ROBOT": True,
        "env_RB_ALLOW_REAL_MOTION": True,
        "env_RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION": True,
        "env_RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN": True,
        "env_RB_RBPODO_PGMODE_SIMULATION_CONFIRMED": True,
        "physical_motion_expected": False,
        "controller_simulation_cartesian_enabled": True,
        "controller_simulation_cartesian_enabled_for_current_command": True,
        "controller_simulation_streaming_cartesian_available": True,
        "streaming_cartesian_physical_real_enabled": False,
        "current_command_is_streaming_cartesian": True,
        "cartesian_available": True,
    }
    gate.update(overrides)
    return gate


def sample_state(left_pose=None, right_pose=None, **gate_overrides) -> StateSnapshot:
    left_pose = left_pose or {
        "x": 1.0,
        "y": 2.0,
        "z": 3.0,
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
    }
    right_pose = right_pose or {
        "x": -1.0,
        "y": 2.0,
        "z": 3.0,
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
    }
    arm = {
        "has_valid_joint_state": True,
        "has_valid_tcp_pose": True,
        "q_actual_deg": [0, -30, 80, 0, 60, 0],
        "cartesian_gate": controller_sim_cartesian_gate(**gate_overrides),
        "physical_motion_expected": False,
        "controller_simulation_physical_motion_detected": False,
    }
    payload = {
        "schema_version": 1,
        "host_time_ns": 123,
        "observed_mode": "real",
        "observed_backend": "rbpodo",
        "motion_state": "ConnectedHold",
        "fault_latched": False,
        "left": {**arm, "tcp_stand": left_pose},
        "right": {**arm, "tcp_stand": right_pose},
    }
    return StateSnapshot(payload=payload, received_monotonic=0.0)


def umi_sample(
    pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
    *,
    gripper=0.0,
    deadman=True,
    monotonic=0.0,
) -> UmiSample:
    return UmiSample(tuple(float(value) for value in pose), float(gripper), bool(deadman), float(monotonic))


class FakeStateClient:
    def __init__(self, snapshot: StateSnapshot):
        self._latest = snapshot
        self.started = False
        self.closed = False

    @property
    def latest(self) -> StateSnapshot:
        return self._latest

    def start(self) -> None:
        self.started = True

    def close(self) -> None:
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


class LoopClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep_until_stop_after(self, max_sleeps: int):
        sleeps = 0

        def sleep(period: float) -> None:
            nonlocal sleeps
            sleeps += 1
            self.now += period
            if sleeps >= max_sleeps:
                raise KeyboardInterrupt

        return sleep


class FakeUdpSocket:
    def __init__(self, packets):
        self.packets = list(packets)
        self.bound = None
        self.blocking = True
        self.closed = False

    def bind(self, address):
        self.bound = address

    def setblocking(self, flag):
        self.blocking = flag

    def recvfrom(self, _size):
        if not self.packets:
            raise BlockingIOError()
        return self.packets.pop(0), ("127.0.0.1", 9999)

    def close(self):
        self.closed = True


class UmiDualCartesianTest(unittest.TestCase):
    def test_relative_init_math_outputs_tcp_pose_target_stand(self):
        reader = MockUmiPoseReader(
            [
                {"pose": [0, 0, 0, 0, 0, 0, 1], "deadman": True, "monotonic": 0.0},
                {"pose": [0.02, -0.01, 0.03, 0, 0, 0, 1], "deadman": True, "monotonic": 0.01},
            ]
        )
        source = UmiDualCartesianActionSource(
            reader,
            MockUmiPoseReader([]),
            gripper_offset=(0.0, 0.0, 0.0),
            max_linear_step_m=1.0,
            max_angular_step_rad=1.0,
        )

        first = source.next_intent(sample_state(), 0.0)
        second = source.next_intent(sample_state(), 0.01)

        self.assertIsNotNone(first)
        self.assertIsNotNone(second)
        assert first is not None and second is not None
        self.assertEqual(first.left["tcp_target_stand"], [1.0, 2.0, 3.0, 0.0, 0.0, 0.0])
        self.assertEqual(second.left["mode"], "TcpPoseTarget")
        self.assertEqual(second.right["mode"], "Hold")
        self.assertAlmostEqual(second.left["tcp_target_stand"][0], 1.02)
        self.assertAlmostEqual(second.left["tcp_target_stand"][1], 1.99)
        self.assertAlmostEqual(second.left["tcp_target_stand"][2], 3.03)

    def test_relative_rotation_uses_server_rpy_convention(self):
        qz_90 = math.sin(math.pi / 4.0)
        qw_90 = math.cos(math.pi / 4.0)
        reader = MockUmiPoseReader(
            [
                {"pose": [0, 0, 0, 0, 0, 0, 1], "deadman": True, "monotonic": 0.0},
                {"pose": [0, 0, 0, 0, 0, qz_90, qw_90], "deadman": True, "monotonic": 0.01},
            ]
        )
        source = UmiDualCartesianActionSource(
            reader,
            MockUmiPoseReader([]),
            gripper_offset=(0.0, 0.0, 0.0),
            max_linear_step_m=1.0,
            max_angular_step_rad=math.pi,
        )

        _ = source.next_intent(sample_state(), 0.0)
        intent = source.next_intent(sample_state(), 0.01)

        self.assertIsNotNone(intent)
        assert intent is not None
        self.assertAlmostEqual(intent.left["tcp_target_stand"][3], 0.0, places=7)
        self.assertAlmostEqual(intent.left["tcp_target_stand"][4], 0.0, places=7)
        self.assertAlmostEqual(intent.left["tcp_target_stand"][5], math.pi / 2.0, places=7)

    def test_clutch_release_holds_and_next_press_relatches(self):
        reader = MockUmiPoseReader(
            [
                {"pose": [0, 0, 0, 0, 0, 0, 1], "deadman": True, "monotonic": 0.0},
                {"pose": [0.1, 0, 0, 0, 0, 0, 1], "deadman": False, "monotonic": 0.01},
                {"pose": [0.1, 0, 0, 0, 0, 0, 1], "deadman": True, "monotonic": 0.02},
            ]
        )
        source = UmiDualCartesianActionSource(
            reader,
            MockUmiPoseReader([]),
            gripper_offset=(0.0, 0.0, 0.0),
            max_linear_step_m=1.0,
        )

        _ = source.next_intent(sample_state(), 0.0)
        release = source.next_intent(sample_state(), 0.01)
        relatch = source.next_intent(sample_state(), 0.02)

        self.assertIsNotNone(release)
        self.assertIsNotNone(relatch)
        assert release is not None and relatch is not None
        self.assertEqual(release.left["mode"], "Hold")
        self.assertEqual(relatch.left["tcp_target_stand"], [1.0, 2.0, 3.0, 0.0, 0.0, 0.0])

    def test_step_clamp_limits_tracker_jump(self):
        reader = MockUmiPoseReader(
            [
                {"pose": [0, 0, 0, 0, 0, 0, 1], "deadman": True, "monotonic": 0.0},
                {"pose": [0.5, 0, 0, 0, 0, 0, 1], "deadman": True, "monotonic": 0.01},
            ]
        )
        source = UmiDualCartesianActionSource(
            reader,
            MockUmiPoseReader([]),
            gripper_offset=(0.0, 0.0, 0.0),
            max_linear_step_m=0.01,
        )

        _ = source.next_intent(sample_state(), 0.0)
        intent = source.next_intent(sample_state(), 0.01)

        self.assertIsNotNone(intent)
        assert intent is not None
        self.assertAlmostEqual(intent.left["tcp_target_stand"][0], 1.01)

    def test_gripper_passthrough_and_stale_sample_hold(self):
        reader = MockUmiPoseReader(
            [
                {"pose": [0, 0, 0, 0, 0, 0, 1], "gripper": 0.25, "deadman": True, "monotonic": 0.0},
                None,
            ]
        )
        source = UmiDualCartesianActionSource(
            reader,
            MockUmiPoseReader([]),
            gripper_offset=(0.0, 0.0, 0.0),
            sample_hold_timeout_sec=0.05,
        )

        first = source.next_intent(sample_state(), 0.0)
        held = source.next_intent(sample_state(), 0.02)
        stale = source.next_intent(sample_state(), 0.20)

        self.assertIsNotNone(first)
        self.assertIsNotNone(held)
        self.assertIsNotNone(stale)
        assert first is not None and held is not None and stale is not None
        self.assertEqual(first.left["gripper_target"], 25.0)
        self.assertEqual(held.left["gripper_target"], 25.0)
        self.assertEqual(stale.left["mode"], "Hold")

    def test_mock_reader_end_to_end_both_arms(self):
        left = MockUmiPoseReader([umi_sample(gripper=80.0)])
        right = MockUmiPoseReader([umi_sample(pose=(0, 0.01, 0, 0, 0, 0, 1), gripper=0.5)])
        source = UmiDualCartesianActionSource(
            left,
            right,
            gripper_offset=(0.0, 0.0, 0.0),
            max_linear_step_m=1.0,
        )

        intent = source.next_intent(sample_state(), 0.0)

        self.assertIsNotNone(intent)
        assert intent is not None
        self.assertEqual(intent.mode, "TcpPoseTarget")
        self.assertEqual(intent.left["mode"], "TcpPoseTarget")
        self.assertEqual(intent.right["mode"], "TcpPoseTarget")
        self.assertEqual(intent.left["gripper_target"], 80.0)
        self.assertEqual(intent.right["gripper_target"], 50.0)

    def test_udp_reader_parses_side_from_wire_schema(self):
        packet = json.dumps(
            {
                "t": 12.5,
                "left": {
                    "pose": [1, 2, 3, 0, 0, 0, 1],
                    "gripper": 0.2,
                    "deadman": True,
                },
                "right": {
                    "pose": [4, 5, 6, 0, 0, 0, 1],
                    "gripper": 90,
                    "deadman": False,
                },
            }
        ).encode("utf-8")
        fake_socket = FakeUdpSocket([packet])
        reader = UdpUmiPoseReader(
            "udp://127.0.0.1:49000",
            "right",
            socket_factory=lambda *_args: fake_socket,
            monotonic_fn=lambda: 777.0,
        )

        sample = reader.read()
        reader.close()

        self.assertIsNotNone(sample)
        assert sample is not None
        self.assertEqual(sample.pose_xyzw[:3], (4.0, 5.0, 6.0))
        self.assertEqual(sample.gripper, 90.0)
        self.assertFalse(sample.deadman)
        # Staleness uses the LOCAL arrival clock, not the remote packet "t" (12.5):
        # cross-machine monotonic clocks are not comparable.
        self.assertEqual(sample.monotonic, 777.0)
        self.assertEqual(fake_socket.bound, ("127.0.0.1", 49000))
        self.assertTrue(fake_socket.closed)

    def test_config_and_factory_default_to_mock_umi_source(self):
        cfg = config_from_mapping(
            {
                "schema": "robotics_lab.policy_runner.v1",
                "mode": "real",
                "action_source": "umi_dual_cartesian",
                "safety": {"allow_rbpodo_controller_simulation_cartesian": True},
                "umi_dual_cartesian": {
                    "max_linear_step_m": 0.01,
                    "left": {"mock_script": "pgmode_umi_smoke"},
                    "right": {"mock_script": "pgmode_umi_smoke"},
                },
            }
        )

        source = make_action_source(cfg)

        self.assertIsInstance(source, UmiDualCartesianActionSource)
        self.assertFalse(cfg.safety.allow_real_motion)
        source.close()

    def test_policy_runner_sends_tcp_pose_target_in_controller_simulation(self):
        cfg = config_from_mapping(
            {
                "schema": "robotics_lab.policy_runner.v1",
                "mode": "real",
                "action_source": "umi_dual_cartesian",
                "runtime": {"startup_timeout_sec": 0.1},
                "robot_state": {"bind": "udp://0.0.0.0:50376", "stale_timeout_sec": 1.0},
                "servo_command": {"endpoint": "udp://127.0.0.1:50256", "timeout_sec": 0.05},
                "safety": {
                    "allow_real_motion": False,
                    "allow_rbpodo_controller_simulation_cartesian": True,
                    "allow_configured_estimate_geometry_in_controller_simulation": True,
                },
                "command_rate_hz": 10,
                "umi_dual_cartesian": {
                    "gripper_offset": [0.0, 0.0, 0.0],
                    "left": {
                        "mock_script": [
                            {"pose": [0, 0, 0, 0, 0, 0, 1], "deadman": True, "monotonic": 0.0}
                        ]
                    },
                    "right": {
                        "mock_script": [
                            {"pose": [0, 0, 0, 0, 0, 0, 1], "deadman": True, "monotonic": 0.0}
                        ]
                    },
                },
            }
        )
        socket = FakeSendSocket()
        command_client = ServoCommandClient(
            cfg.servo_command.endpoint,
            cfg.servo_command.timeout_sec,
            socket_factory=lambda *_args: socket,
        )
        state_client = FakeStateClient(sample_state())
        clock = LoopClock()

        exit_code = run(
            cfg,
            state_client=state_client,
            command_client=command_client,
            sleep_fn=clock.sleep_until_stop_after(1),
            monotonic_fn=clock.monotonic,
        )

        packets = [json.loads(data.decode("utf-8")) for data, _address in socket.sent]
        self.assertEqual(exit_code, 0)
        self.assertEqual([packet["mode"] for packet in packets], ["ArmMotion", "TcpPoseTarget"])
        self.assertEqual(packets[1]["left"]["mode"], "TcpPoseTarget")
        self.assertEqual(packets[1]["right"]["mode"], "TcpPoseTarget")
        self.assertFalse(cfg.safety.allow_real_motion)


if __name__ == "__main__":
    unittest.main()
