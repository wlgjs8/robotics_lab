from __future__ import annotations

import time
import unittest

from policy_runner.action_sources.tcp_pose_target import (
    RBPODO_CONTROLLER_SIMULATION_CARTESIAN_ACTION_REQUIREMENTS,
    tcp_pose_target_stand_intent,
)
from policy_runner.config import SafetyConfig, config_from_mapping
from policy_runner.geometry import geometry_status_from_mapping
from policy_runner.gripper import GripperCommand, GripperRuntime
from policy_runner.robot_state_client import StateSnapshot
from policy_runner.rollout_modes import RolloutMode, RolloutModePolicy, RolloutModeValidationError
from policy_runner.safety import SafetyGate


class DualArmPolicyGateTest(unittest.TestCase):
    def test_real_policy_rejects_missing_or_configured_estimate_collision_model(self) -> None:
        policy = RolloutModePolicy(RolloutMode.REAL_POLICY)
        for collision_model_status in ("missing", "configured_estimate"):
            with self.subTest(collision_model_status=collision_model_status):
                cfg = _real_policy_config(collision_model_status=collision_model_status)
                with self.assertRaisesRegex(
                    RolloutModeValidationError,
                    "collision_model_status",
                ):
                    policy.validate_config(
                        cfg,
                        geometry_status=_measured_geometry(),
                        checkpoint_arm_mask=(1.0, 1.0),
                    )

    def test_real_policy_rejects_selected_arm_checkpoint_arm_mask_mismatch(self) -> None:
        policy = RolloutModePolicy(RolloutMode.REAL_POLICY)
        cfg = _real_policy_config(selected_arm="left")

        with self.assertRaisesRegex(RolloutModeValidationError, "checkpoint arm_mask"):
            policy.validate_config(
                cfg,
                geometry_status=_measured_geometry(),
                checkpoint_arm_mask=(1.0, 1.0),
            )

    def test_real_policy_rejects_nonzero_gripper_checkpoint_without_gripper_gate(self) -> None:
        policy = RolloutModePolicy(RolloutMode.REAL_POLICY)
        cfg = _real_policy_config(allow_real_gripper_motion=False)

        with self.assertRaisesRegex(RolloutModeValidationError, "allow_real_gripper_motion=true"):
            policy.validate_config(
                cfg,
                geometry_status=_measured_geometry(),
                checkpoint_arm_mask=(1.0, 1.0),
                checkpoint_has_nonzero_gripper_commands=True,
            )

    def test_controller_sim_allows_cartesian_pose_target_and_logs_gripper_noop(self) -> None:
        gate = SafetyGate(
            "real",
            SafetyConfig(
                allow_real_motion=False,
                allow_rbpodo_controller_simulation_cartesian=True,
            ),
            stale_timeout_sec=0.5,
            geometry_status=_configured_estimate_geometry(),
        )

        decision = gate.evaluate(
            _controller_sim_state(),
            tcp_pose_target_stand_intent(left=[0.3, 0.1, 0.5, 0.0, 0.0, 0.0]),
            RBPODO_CONTROLLER_SIMULATION_CARTESIAN_ACTION_REQUIREMENTS,
            time.monotonic(),
        )
        gripper_runtime = GripperRuntime(rollout_mode="controller_sim", env={})
        gripper_results = gripper_runtime.dispatch([GripperCommand("left", 0.5)])

        self.assertTrue(decision.allowed, decision.reason)
        self.assertEqual(len(gripper_results), 1)
        self.assertTrue(gripper_results[0].dropped)
        self.assertFalse(gripper_results[0].sent_to_physical)
        self.assertEqual(gripper_results[0].reason, "controller_sim_gripper_logged_noop")


def _real_policy_config(
    *,
    collision_model_status: str = "measured",
    selected_arm: str = "both",
    allow_real_gripper_motion: bool = True,
):
    return config_from_mapping(
        {
            "schema": "robotics_lab.policy_runner.v1",
            "mode": "real",
            "safety": {
                "allow_real_motion": True,
                "allow_real_gripper_motion": allow_real_gripper_motion,
                "selected_arm": selected_arm,
                "retarget_status": "measured",
                "collision_model_status": collision_model_status,
                "minimum_inter_arm_distance_m": 0.05,
                "workspace_envelope_status": "measured",
                "measured_retarget_available": True,
                "measured_collision_model_available": True,
                "measured_gripper_available": True,
            },
        }
    )


def _measured_geometry():
    return geometry_status_from_mapping(
        {
            "schema": "robotics_lab.calibration.v1",
            "calibration_id": "MEASURED_TEST",
            "status": "measured",
            "geometry_valid_for_real_policy": True,
            "robot": {
                "T_stand_left_base": {
                    "parent": "stand",
                    "child": "left_base",
                    "xyz_rpy": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    "status": "measured",
                },
                "T_stand_right_base": {
                    "parent": "stand",
                    "child": "right_base",
                    "xyz_rpy": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    "status": "measured",
                },
            },
        }
    )


def _configured_estimate_geometry():
    return geometry_status_from_mapping(
        {
            "schema": "robotics_lab.calibration.v1",
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


def _controller_sim_state() -> StateSnapshot:
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
    arm = {
        "has_valid_joint_state": True,
        "has_valid_tcp_pose": True,
        "q_actual_deg": [0, -30, 80, 0, 60, 0],
        "cartesian_gate": gate,
        "physical_motion_expected": False,
        "controller_simulation_physical_motion_detected": False,
    }
    return StateSnapshot(
        payload={
            "schema_version": 1,
            "host_time_ns": 123,
            "observed_mode": "real",
            "observed_backend": "rbpodo",
            "motion_state": "ConnectedHold",
            "fault_latched": False,
            "left": dict(arm),
            "right": dict(arm),
        },
        received_monotonic=time.monotonic(),
    )


if __name__ == "__main__":
    unittest.main()
