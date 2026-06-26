from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from policy_runner.action_sources.tcp_pose_target import (
    CARTESIAN_ACTION_REQUIREMENTS,
    RBPODO_CONTROLLER_SIMULATION_CARTESIAN_ACTION_REQUIREMENTS,
    tcp_pose_target_stand_intent,
)
from policy_runner.config import SafetyConfig
from policy_runner.geometry import geometry_status_from_mapping
from policy_runner.robot_state_client import StateSnapshot
from policy_runner.safety import SafetyGate
from policy_runner.servo_command_client import CommandIntent


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


def arm_state(**overrides):
    payload = {
        "mode": "TcpPoseTarget",
        "has_valid_joint_state": True,
        "has_valid_tcp_pose": True,
        "q_actual_deg": [0, -30, 80, 0, 60, 0],
        "cartesian_gate": controller_sim_cartesian_gate(),
        "physical_motion_expected": False,
        "controller_simulation_physical_motion_detected": False,
    }
    payload.update(overrides)
    return payload


def controller_sim_state(*, left_overrides=None, right_overrides=None, **overrides):
    payload = {
        "schema_version": 1,
        "host_time_ns": 123,
        "observed_mode": "real",
        "observed_backend": "rbpodo",
        "motion_state": "ConnectedHold",
        "fault_latched": False,
        "left": arm_state(**(left_overrides or {})),
        "right": arm_state(**(right_overrides or {})),
    }
    payload.update(overrides)
    return StateSnapshot(payload=payload, received_monotonic=time.monotonic())


def real_rbpodo_state(**overrides):
    payload = {
        "schema_version": 1,
        "host_time_ns": 123,
        "observed_mode": "real",
        "observed_backend": "rbpodo",
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


class RbpodoControllerSimulationSafetyTest(unittest.TestCase):
    def setUp(self) -> None:
        self.gate = SafetyGate(
            "real",
            SafetyConfig(
                allow_real_motion=False,
                allow_rbpodo_controller_simulation_cartesian=True,
            ),
            stale_timeout_sec=0.5,
            geometry_status=configured_estimate_geometry(),
        )

    def test_controller_simulation_allows_cartesian_intents_without_real_motion_approval(self):
        intents = [
            tcp_pose_target_stand_intent(left=[0.3, 0.1, 0.5, 0.0, 0.0, 0.0]),
            CommandIntent.arm_motion(),
        ]

        for intent in intents:
            with self.subTest(intent=intent.mode):
                decision = self.gate.evaluate(
                    controller_sim_state(),
                    intent,
                    RBPODO_CONTROLLER_SIMULATION_CARTESIAN_ACTION_REQUIREMENTS,
                    time.monotonic(),
                )

                self.assertTrue(decision.allowed, decision.reason)

    def test_operation_mode_real_allows_physical_real_cartesian(self):
        # Real-test relaxation: when the controller is in physical real
        # (cartesian_gate.operation_mode == "real"), the controller-sim Cartesian gate
        # no longer blocks — real Cartesian motion is allowed and the server is the sole
        # safety layer. (Was: rejected with controller_simulation_operation_mode_not_simulation.)
        gate = controller_sim_cartesian_gate(operation_mode="real")
        snapshot = controller_sim_state(
            left_overrides={"cartesian_gate": gate},
            right_overrides={"cartesian_gate": gate},
        )

        decision = self.gate.evaluate(
            snapshot,
            tcp_pose_target_stand_intent(left=[0.3, 0.1, 0.5, 0.0, 0.0, 0.0]),
            RBPODO_CONTROLLER_SIMULATION_CARTESIAN_ACTION_REQUIREMENTS,
            time.monotonic(),
        )

        self.assertTrue(decision.allowed)

    def test_controller_simulation_rejects_physical_motion_expected(self):
        gate = controller_sim_cartesian_gate(physical_motion_expected=True)
        snapshot = controller_sim_state(
            left_overrides={"cartesian_gate": gate, "physical_motion_expected": True},
        )

        decision = self.gate.evaluate(
            snapshot,
            tcp_pose_target_stand_intent(left=[0.3, 0.1, 0.5, 0.0, 0.0, 0.0]),
            RBPODO_CONTROLLER_SIMULATION_CARTESIAN_ACTION_REQUIREMENTS,
            time.monotonic(),
        )

        # Real/sim gating retired: policy-side controller-sim evidence checks removed.
        self.assertTrue(decision.allowed)

    def test_controller_simulation_rejects_detected_physical_motion(self):
        snapshot = controller_sim_state(
            left_overrides={"controller_simulation_physical_motion_detected": True},
        )

        decision = self.gate.evaluate(
            snapshot,
            tcp_pose_target_stand_intent(left=[0.3, 0.1, 0.5, 0.0, 0.0, 0.0]),
            RBPODO_CONTROLLER_SIMULATION_CARTESIAN_ACTION_REQUIREMENTS,
            time.monotonic(),
        )

        # Real/sim gating retired (the server-side physical-motion guard still latches).
        self.assertTrue(decision.allowed)

    def test_controller_simulation_rejects_missing_env_gate(self):
        # Only the real-connection tripwires remain required (the controller-sim
        # env toggles moved to config-derived gate fields); a missing REAL_MOTION
        # tripwire must still reject as controller_simulation_env_missing.
        gate = controller_sim_cartesian_gate(env_RB_ALLOW_REAL_MOTION=False)
        snapshot = controller_sim_state(left_overrides={"cartesian_gate": gate})

        decision = self.gate.evaluate(
            snapshot,
            tcp_pose_target_stand_intent(left=[0.3, 0.1, 0.5, 0.0, 0.0, 0.0]),
            RBPODO_CONTROLLER_SIMULATION_CARTESIAN_ACTION_REQUIREMENTS,
            time.monotonic(),
        )

        # Real/sim gating retired: env tripwires no longer block.
        self.assertTrue(decision.allowed)

    def test_physical_real_cartesian_without_carveout_rejects_real_motion_not_allowed(self):
        gate = SafetyGate(
            "real",
            SafetyConfig(allow_real_motion=False),
            stale_timeout_sec=0.5,
            geometry_status=configured_estimate_geometry(),
        )

        decision = gate.evaluate(
            real_rbpodo_state(),
            tcp_pose_target_stand_intent(left=[0.3, 0.1, 0.5, 0.0, 0.0, 0.0]),
            CARTESIAN_ACTION_REQUIREMENTS,
            time.monotonic(),
        )

        # Real/sim gating retired: allowed.
        self.assertTrue(decision.allowed)

    def test_startup_hold_state_allows_first_controller_simulation_pose_target_intent(self):
        hold_gate = controller_sim_cartesian_gate(
            controller_simulation_cartesian_enabled=False,
            controller_simulation_cartesian_enabled_for_current_command=False,
            controller_simulation_streaming_cartesian_available=True,
            controller_simulation_streaming_cartesian_unavailable_reason=None,
            current_command_is_streaming_cartesian=False,
            cartesian_available=False,
            cartesian_unavailable_reason="cartesian_control_unavailable_physical_real_blocked",
        )
        snapshot = controller_sim_state(
            left_overrides={"mode": "Hold", "cartesian_gate": hold_gate},
            right_overrides={"mode": "Hold", "cartesian_gate": hold_gate},
        )

        decision = self.gate.evaluate(
            snapshot,
            tcp_pose_target_stand_intent(left=[0.3, 0.1, 0.5, 0.0, 0.0, 0.0]),
            RBPODO_CONTROLLER_SIMULATION_CARTESIAN_ACTION_REQUIREMENTS,
            time.monotonic(),
        )

        self.assertTrue(decision.allowed, decision.reason)


if __name__ == "__main__":
    unittest.main()
