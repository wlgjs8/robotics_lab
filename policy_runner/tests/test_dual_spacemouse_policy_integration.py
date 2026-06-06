from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from policy_runner.config import config_from_mapping
from policy_runner.main import run
from policy_runner.robot_state_client import StateSnapshot
from policy_runner.servo_command_client import ServoCommandClient


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
        "controller_simulation_streaming_cartesian_unavailable_reason": None,
        "streaming_cartesian_physical_real_enabled": False,
        "current_command_is_streaming_cartesian": True,
        "cartesian_available": True,
        "cartesian_unavailable_reason": None,
    }
    gate.update(overrides)
    return gate


def controller_sim_state(**gate_overrides) -> StateSnapshot:
    left_gate = controller_sim_cartesian_gate(**gate_overrides)
    right_gate = controller_sim_cartesian_gate(**gate_overrides)
    arm = {
        "has_valid_joint_state": True,
        "has_valid_tcp_pose": True,
        "q_actual_deg": [0, -30, 80, 0, 60, 0],
        "cartesian_gate": left_gate,
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
        "left": arm,
        "right": {**arm, "cartesian_gate": right_gate},
    }
    return StateSnapshot(payload=payload, received_monotonic=0.0)


def policy_config(left_script, right_script):
    return config_from_mapping(
        {
            "schema": "robotics_lab.policy_runner.v1",
            "mode": "real",
            "action_source": "dual_spacemouse_cartesian",
            "runtime": {"startup_timeout_sec": 0.1},
            "robot_state": {"bind": "udp://0.0.0.0:50376", "stale_timeout_sec": 1.0},
            "servo_command": {"endpoint": "udp://127.0.0.1:50256", "timeout_sec": 0.05},
            "safety": {
                "allow_real_motion": False,
                "allow_rbpodo_controller_simulation_cartesian": True,
                "allow_configured_estimate_geometry_in_controller_simulation": True,
            },
            "command_rate_hz": 10,
            "spacemouse_cartesian_dual": {
                "max_linear_velocity_m_s": 0.2,
                "max_angular_velocity_rad_s": 0.4,
                "deadband": 0.0,
                "response_curve_gamma": 1.0,
                "sample_hold_timeout_sec": 0.05,
                "left": {"mock_script": left_script, "deadman_button": 0},
                "right": {"mock_script": right_script, "deadman_button": 0},
            },
        }
    )


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


def run_policy_once(config, snapshot: StateSnapshot, *, max_sleeps: int):
    socket = FakeSendSocket()
    command_client = ServoCommandClient(
        config.servo_command.endpoint,
        config.servo_command.timeout_sec,
        socket_factory=lambda *_args: socket,
    )
    state_client = FakeStateClient(snapshot)
    clock = LoopClock()

    exit_code = run(
        config,
        state_client=state_client,
        command_client=command_client,
        sleep_fn=clock.sleep_until_stop_after(max_sleeps),
        monotonic_fn=clock.monotonic,
    )

    packets = [json.loads(data.decode("utf-8")) for data, _address in socket.sent]
    return exit_code, packets, state_client, socket


class DualSpaceMousePolicyIntegrationTest(unittest.TestCase):
    def test_mock_dual_spacemouse_sends_tcp_twist_local_for_both_arms(self):
        cfg = policy_config(
            [{"tx": 0.5, "buttons": [True], "timestamp_monotonic": 0.0}],
            [{"ty": -0.25, "rz": 0.5, "buttons": [True], "timestamp_monotonic": 0.0}],
        )

        exit_code, packets, state_client, socket = run_policy_once(
            cfg,
            controller_sim_state(),
            max_sleeps=1,
        )

        self.assertEqual(exit_code, 0)
        self.assertTrue(state_client.started)
        self.assertTrue(state_client.closed)
        self.assertTrue(socket.closed)
        self.assertEqual([packet["mode"] for packet in packets], ["ArmMotion", "TcpTwistLocal"])
        twist = packets[1]
        self.assertEqual(twist["left"]["mode"], "TcpTwistLocal")
        self.assertEqual(twist["right"]["mode"], "TcpTwistLocal")
        self.assertEqual(twist["left"]["tcp_twist_local"], [0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.assertEqual(twist["right"]["tcp_twist_local"], [0.0, -0.05, 0.0, 0.0, 0.0, 0.2])

    def test_release_and_stale_scripted_samples_send_zero_twist(self):
        left_script = [
            {"tx": 0.5, "buttons": [True], "timestamp_monotonic": 0.0},
            {"buttons": [False], "timestamp_monotonic": 0.1},
            {"tx": 0.25, "buttons": [True], "timestamp_monotonic": 0.2},
        ]
        right_script = [
            {"ty": -0.5, "buttons": [True], "timestamp_monotonic": 0.0},
            {"buttons": [False], "timestamp_monotonic": 0.1},
            {"ty": -0.25, "buttons": [True], "timestamp_monotonic": 0.2},
        ]
        cfg = policy_config(left_script, right_script)

        _exit_code, packets, _state_client, _socket = run_policy_once(
            cfg,
            controller_sim_state(),
            max_sleeps=4,
        )

        twist_packets = [packet for packet in packets if packet["mode"] == "TcpTwistLocal"]
        self.assertEqual(len(twist_packets), 4)
        release_zero = twist_packets[1]
        stale_zero = twist_packets[3]
        self.assertEqual(release_zero["left"]["tcp_twist_local"], [0.0] * 6)
        self.assertEqual(release_zero["right"]["tcp_twist_local"], [0.0] * 6)
        self.assertEqual(stale_zero["left"]["tcp_twist_local"], [0.0] * 6)
        self.assertEqual(stale_zero["right"]["tcp_twist_local"], [0.0] * 6)

    def test_safety_readback_blocks_when_operation_mode_is_not_simulation(self):
        cfg = policy_config(
            [{"tx": 0.5, "buttons": [True], "timestamp_monotonic": 0.0}],
            [{"ty": -0.25, "buttons": [True], "timestamp_monotonic": 0.0}],
        )

        _exit_code, packets, _state_client, _socket = run_policy_once(
            cfg,
            controller_sim_state(operation_mode="real"),
            max_sleeps=1,
        )

        self.assertEqual(packets, [])


if __name__ == "__main__":
    unittest.main()
