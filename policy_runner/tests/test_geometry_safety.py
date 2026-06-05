from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from policy_runner.config import SafetyConfig, config_from_mapping
from policy_runner.geometry import geometry_status_from_mapping, load_geometry_status
from policy_runner.robot_state_client import StateSnapshot
from policy_runner.safety import ActionRequirements, CameraReadiness, SafetyGate, camera_readiness_from_snapshot
from policy_runner.servo_command_client import CommandIntent


def sample_state(**overrides):
    payload = {
        "schema_version": 1,
        "host_time_ns": 123,
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


def configured_estimate_geometry(valid_for_real: bool = False):
    return geometry_status_from_mapping(
        {
            "calibration_id": "CONFIG_ESTIMATE_TEST",
            "status": "configured_estimate",
            "geometry_valid_for_real_policy": valid_for_real,
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
            "cameras": {
                "head": {
                    "intrinsics_status": "unknown",
                    "extrinsics_status": "unmeasured",
                }
            },
        }
    )


class DummyCameraActionSource:
    requirements = ActionRequirements(requires_camera=True, camera_stale_timeout_sec=0.2)


class GeometrySafetyTest(unittest.TestCase):
    def test_config_exposes_geometry_path_and_safety_toggles(self):
        cfg = config_from_mapping(
            {
                "schema": "robotics_lab.policy_runner.v1",
                "geometry": {"path": "calibration/test.yaml"},
                "safety": {
                    "allow_configured_estimate_geometry_in_simulation": False,
                    "allow_configured_estimate_geometry_in_real": True,
                    "allow_configured_estimate_geometry_in_controller_simulation": False,
                    "camera_stale_timeout_sec": 0.75,
                },
            }
        )

        self.assertEqual(cfg.geometry.path, "calibration/test.yaml")
        self.assertFalse(cfg.safety.allow_configured_estimate_geometry_in_simulation)
        self.assertTrue(cfg.safety.allow_configured_estimate_geometry_in_real)
        self.assertFalse(cfg.safety.allow_configured_estimate_geometry_in_controller_simulation)
        self.assertEqual(cfg.safety.camera_stale_timeout_sec, 0.75)

    def test_joint_only_action_can_run_without_geometry_file(self):
        missing = load_geometry_status(Path(__file__).resolve().parent / "missing_calibration.yaml")
        gate = SafetyGate("simulation", SafetyConfig(), stale_timeout_sec=0.5, geometry_status=missing)
        intent = CommandIntent.joint_velocity(left=[1, 0, 0, 0, 0, 0], right=[1, 0, 0, 0, 0, 0])

        decision = gate.evaluate(sample_state(), intent, ActionRequirements(), time.monotonic())

        self.assertTrue(decision.allowed)

    def test_joint_only_action_can_run_without_camera_readiness(self):
        gate = SafetyGate(
            "simulation",
            SafetyConfig(camera_available=False),
            stale_timeout_sec=0.5,
            camera_readiness=CameraReadiness(available=False),
        )
        intent = CommandIntent.joint_velocity(left=[1, 0, 0, 0, 0, 0], right=[1, 0, 0, 0, 0, 0])

        decision = gate.evaluate(sample_state(), intent, ActionRequirements(), time.monotonic())

        self.assertTrue(decision.allowed)

    def test_camera_dependent_action_blocks_when_camera_readiness_is_absent(self):
        gate = SafetyGate(
            "simulation",
            SafetyConfig(camera_available=True),
            stale_timeout_sec=0.5,
            camera_readiness=CameraReadiness(available=True),
        )

        decision = gate.evaluate(
            sample_state(),
            CommandIntent.hold(),
            DummyCameraActionSource.requirements,
            now_monotonic=10.0,
        )

        self.assertFalse(decision.allowed)
        self.assertEqual(decision.reason, "camera_unavailable")

    def test_camera_dependent_action_uses_state_payload_readiness(self):
        gate = SafetyGate("simulation", SafetyConfig(camera_available=False), stale_timeout_sec=0.5)
        snapshot = sample_state(
            camera_readiness={
                "available": True,
                "required_present": True,
                "latest_bundle_age_ms": 50,
            }
        )

        decision = gate.evaluate(
            snapshot,
            CommandIntent.hold(),
            DummyCameraActionSource.requirements,
            now_monotonic=20.0,
        )

        self.assertTrue(decision.allowed)

    def test_camera_dependent_action_blocks_state_payload_missing_camera(self):
        gate = SafetyGate("simulation", SafetyConfig(camera_available=True), stale_timeout_sec=0.5)

        decision = gate.evaluate(
            sample_state(camera_readiness={"available": True, "required_missing": True}),
            CommandIntent.hold(),
            DummyCameraActionSource.requirements,
            now_monotonic=20.0,
        )

        self.assertFalse(decision.allowed)
        self.assertEqual(decision.reason, "camera_unavailable")

    def test_camera_dependent_action_blocks_when_camera_readiness_is_stale(self):
        gate = SafetyGate(
            "simulation",
            SafetyConfig(camera_available=True),
            stale_timeout_sec=0.5,
            camera_readiness=CameraReadiness(available=True, last_observed_monotonic_sec=10.0),
        )

        decision = gate.evaluate(
            sample_state(),
            CommandIntent.hold(),
            DummyCameraActionSource.requirements,
            now_monotonic=10.25,
        )

        self.assertFalse(decision.allowed)
        self.assertEqual(decision.reason, "camera_stale")

    def test_camera_dependent_action_blocks_state_payload_stale_camera(self):
        gate = SafetyGate("simulation", SafetyConfig(camera_available=True), stale_timeout_sec=0.5)

        decision = gate.evaluate(
            sample_state(camera={"available": True, "latest_observation_age_sec": 0.25}),
            CommandIntent.hold(),
            DummyCameraActionSource.requirements,
            now_monotonic=20.0,
        )

        self.assertFalse(decision.allowed)
        self.assertEqual(decision.reason, "camera_stale")

    def test_camera_readiness_parser_accepts_age_fields(self):
        snapshot = sample_state(camera_readiness={"healthy": True, "bundle_age_sec": 0.1})

        readiness = camera_readiness_from_snapshot(snapshot, now_monotonic=3.0)

        self.assertIsNotNone(readiness)
        assert readiness is not None
        self.assertTrue(readiness.available)
        self.assertEqual(readiness.last_observed_monotonic_sec, 2.9)

    def test_geometry_dependent_action_blocks_when_geometry_file_is_missing(self):
        missing = load_geometry_status(Path(__file__).resolve().parent / "missing_calibration.yaml")
        gate = SafetyGate("simulation", SafetyConfig(), stale_timeout_sec=0.5, geometry_status=missing)

        decision = gate.evaluate(
            sample_state(),
            CommandIntent.hold(),
            ActionRequirements(requires_geometry=True),
            time.monotonic(),
        )

        self.assertFalse(decision.allowed)
        self.assertEqual(decision.reason, "geometry_file_missing")

    def test_real_mode_blocks_configured_estimate_geometry_by_default(self):
        gate = SafetyGate(
            "real",
            SafetyConfig(allow_real_motion=True),
            stale_timeout_sec=0.5,
            geometry_status=configured_estimate_geometry(),
        )

        decision = gate.evaluate(
            sample_state(),
            CommandIntent.hold(),
            ActionRequirements(requires_geometry=True),
            time.monotonic(),
        )

        self.assertFalse(decision.allowed)
        self.assertEqual(decision.reason, "configured_estimate_geometry_not_allowed_in_real")

    def test_simulation_mode_can_allow_configured_estimate_geometry(self):
        gate = SafetyGate(
            "simulation",
            SafetyConfig(allow_configured_estimate_geometry_in_simulation=True),
            stale_timeout_sec=0.5,
            geometry_status=configured_estimate_geometry(),
        )

        decision = gate.evaluate(
            sample_state(),
            CommandIntent.hold(),
            ActionRequirements(requires_geometry=True),
            time.monotonic(),
        )

        self.assertTrue(decision.allowed)

    def test_camera_geometry_requires_measured_camera_status(self):
        gate = SafetyGate(
            "simulation",
            SafetyConfig(),
            stale_timeout_sec=0.5,
            geometry_status=configured_estimate_geometry(),
        )

        decision = gate.evaluate(
            sample_state(),
            CommandIntent.hold(),
            ActionRequirements(requires_camera_geometry=True),
            time.monotonic(),
        )

        self.assertFalse(decision.allowed)
        self.assertEqual(decision.reason, "camera_geometry_unavailable")

    def test_valid_tcp_pose_requirement_uses_state_validity(self):
        gate = SafetyGate("simulation", SafetyConfig(), stale_timeout_sec=0.5)
        invalid_tcp = sample_state(
            right={
                "has_valid_joint_state": True,
                "has_valid_tcp_pose": False,
                "q_actual_deg": [0, -30, 80, 0, 60, 0],
            }
        )

        decision = gate.evaluate(
            invalid_tcp,
            CommandIntent.hold(),
            ActionRequirements(requires_valid_tcp_pose=True),
            time.monotonic(),
        )

        self.assertFalse(decision.allowed)
        self.assertEqual(decision.reason, "invalid_tcp_pose")

    def test_active_calibration_registry_loads_as_configured_estimate(self):
        status = load_geometry_status(Path(__file__).resolve().parents[2] / "calibration" / "active_calibration.yaml")

        self.assertEqual(status.calibration_id, "CONFIG_ESTIMATE_001")
        self.assertEqual(status.status, "configured_estimate")
        self.assertFalse(status.geometry_valid_for_real_policy)
        self.assertTrue(status.robot_mounts_available)
        self.assertEqual(status.camera_intrinsics_status["head"], "unknown")
        self.assertEqual(status.camera_extrinsics_status["left_wrist"], "unmeasured")


if __name__ == "__main__":
    unittest.main()
