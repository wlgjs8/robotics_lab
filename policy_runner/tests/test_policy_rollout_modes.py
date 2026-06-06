from __future__ import annotations

import json
import tempfile
import time
import unittest
from pathlib import Path

from policy_runner.config import config_from_mapping
from policy_runner.geometry import geometry_status_from_mapping
from policy_runner.main import run
from policy_runner.robot_state_client import StateSnapshot
from policy_runner.rollout_modes import (
    ReadOnlyActionSource,
    RolloutMode,
    RolloutModePolicy,
    RolloutModeValidationError,
    RolloutSummaryRecorder,
    write_rollout_summary,
)
from policy_runner.safety import ActionRequirements, SafetyDecision
from policy_runner.servo_command_client import CommandIntent


def sample_state(**overrides):
    payload = {
        "schema_version": 1,
        "host_time_ns": 123,
        "motion_state": "ConnectedHold",
        "fault_latched": False,
        "observed_mode": "real",
        "observed_backend": "rbpodo",
        "operation_mode": "real",
        "left": {"has_valid_joint_state": True, "q_actual_deg": [0, -30, 80, 0, 60, 0]},
        "right": {"has_valid_joint_state": True, "q_actual_deg": [0, -30, 80, 0, 60, 0]},
    }
    payload.update(overrides)
    return StateSnapshot(payload=payload, received_monotonic=time.monotonic())


class FakeStateClient:
    def __init__(self, snapshots):
        self._snapshots = list(snapshots)
        self.started = False
        self.closed = False

    @property
    def latest(self):
        if not self._snapshots:
            return None
        return self._snapshots[0]

    def start(self):
        self.started = True

    def close(self):
        self.closed = True


class FakeCommandClient:
    def __init__(self):
        self.sent = []
        self.closed = False

    def send(self, intent):
        self.sent.append(intent)
        return len(self.sent)

    def close(self):
        self.closed = True


class NonzeroMotionSource:
    requirements = ActionRequirements(requires_valid_joint_state=True)
    camera_names = []
    command_family = "JointVelocity"
    image_decode_count = 0
    missing_camera_count = 0

    def __init__(self):
        self.closed = False

    def next_intent(self, snapshot, now_monotonic):
        _ = snapshot, now_monotonic
        return CommandIntent.joint_velocity(left=[1, 0, 0, 0, 0, 0])

    def close(self):
        self.closed = True


class PolicyRolloutModesTest(unittest.TestCase):
    def test_real_readonly_never_sends_commands_even_for_nonzero_action(self) -> None:
        cfg = config_from_mapping(
            {
                "schema": "robotics_lab.policy_runner.v1",
                "mode": "real",
                "safety": {"allow_real_motion": True},
                "geometry": {"path": ""},
                "runtime": {"startup_timeout_sec": 0.1},
            }
        )
        policy = RolloutModePolicy(RolloutMode.REAL_READONLY)
        recorder = RolloutSummaryRecorder(
            policy,
            checkpoint_path="checkpoint.pt",
            config_path="config.yaml",
            command_family="JointVelocity",
        )
        source = ReadOnlyActionSource(NonzeroMotionSource(), recorder)
        command_client = FakeCommandClient()

        result = run(
            cfg,
            state_client=FakeStateClient([sample_state()]),
            command_client=command_client,
            source=source,
            send_commands=policy.may_send_commands,
            rollout_recorder=recorder,
            monotonic_fn=lambda: time.monotonic(),
            sleep_fn=lambda _period: (_ for _ in ()).throw(KeyboardInterrupt()),
        )

        self.assertEqual(result, 0)
        self.assertEqual(command_client.sent, [])
        self.assertEqual(recorder.sent_command_count, 0)
        self.assertEqual(recorder.dropped_command_count, 1)
        self.assertEqual(recorder.dropped_reasons["rollout_mode_command_send_disabled"], 1)

    def test_real_policy_rejects_until_all_physical_gates_are_present(self) -> None:
        policy = RolloutModePolicy(RolloutMode.REAL_POLICY)
        blocked = config_from_mapping(
            {
                "schema": "robotics_lab.policy_runner.v1",
                "mode": "real",
                "geometry": {"path": ""},
            }
        )

        with self.assertRaisesRegex(RolloutModeValidationError, "allow_real_motion=true"):
            policy.validate_config(blocked, geometry_status=measured_geometry())

        missing_gates = config_from_mapping(
            {
                "schema": "robotics_lab.policy_runner.v1",
                "mode": "real",
                "safety": {"allow_real_motion": True},
            }
        )
        with self.assertRaisesRegex(RolloutModeValidationError, "retarget_status=measured or accepted"):
            policy.validate_config(missing_gates, geometry_status=measured_geometry())

        allowed = config_from_mapping(
            {
                "schema": "robotics_lab.policy_runner.v1",
                "mode": "real",
                "safety": {
                    "allow_real_motion": True,
                    "selected_arm": "both",
                    "retarget_status": "measured",
                    "collision_model_status": "measured",
                    "minimum_inter_arm_distance_m": 0.05,
                    "workspace_envelope_status": "measured",
                    "measured_retarget_available": True,
                    "measured_collision_model_available": True,
                    "measured_gripper_available": True,
                },
            }
        )

        policy.validate_config(allowed, geometry_status=measured_geometry())

        accepted = config_from_mapping(
            {
                "schema": "robotics_lab.policy_runner.v1",
                "mode": "real",
                "safety": {
                    "allow_real_motion": True,
                    "selected_arm": "both",
                    "retarget_status": "accepted",
                    "collision_model_status": "measured",
                    "minimum_inter_arm_distance_m": 0.05,
                    "workspace_envelope_status": "measured",
                    "measured_retarget_available": True,
                    "measured_collision_model_available": True,
                    "measured_gripper_available": True,
                },
            }
        )

        policy.validate_config(accepted, geometry_status=measured_geometry())

    def test_rollout_summary_json_has_rollout_summary_and_counts(self) -> None:
        recorder = RolloutSummaryRecorder(
            RolloutModePolicy(RolloutMode.SIM_DRYRUN),
            checkpoint_path="outputs/flow_policy.pt",
            config_path="policy_runner/config/simulator_hold.yaml",
            command_family="TcpDeltaStand",
            camera_names=["head"],
            selected_arms=["left", "right"],
            left_arm_mask=1.0,
            right_arm_mask=1.0,
            gripper_command_count=2,
            gripper_dropped_count=2,
            allow_real_gripper_motion=False,
            collision_model_status="configured_estimate",
        )
        recorder.record_state(
            sample_state(
                observed_mode="simulation",
                observed_backend="simulator",
                operation_mode="simulation",
                physical_motion_expected=False,
            )
        )
        recorder.image_decode_count = 2
        recorder.missing_camera_count = 1
        recorder.record_decision(SafetyDecision(True))
        recorder.record_sent(CommandIntent.hold())
        recorder.record_dropped("sim_dryrun_drop", CommandIntent.joint_velocity(left=[1, 0, 0, 0, 0, 0]))

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "rollout_summary.json"
            write_rollout_summary(recorder, path)
            document = json.loads(path.read_text(encoding="utf-8"))

        self.assertIn("rollout_summary", document)
        summary = document["rollout_summary"]
        self.assertEqual(summary["rollout_mode"], "sim_dryrun")
        self.assertEqual(summary["camera_names"], ["head"])
        self.assertEqual(summary["image_decode_count"], 2)
        self.assertEqual(summary["missing_camera_count"], 1)
        self.assertEqual(summary["sent_command_count"], 1)
        self.assertEqual(summary["dropped_command_count"], 1)
        self.assertEqual(summary["safety_decision_counts"]["ok"], 1)
        self.assertFalse(summary["physical_motion_expected"])
        self.assertEqual(summary["backend_seen"], "simulator")
        self.assertEqual(summary["run_mode_seen"], "simulation")
        self.assertEqual(summary["operation_mode_seen"], "simulation")
        self.assertEqual(summary["selected_arm"], "both")
        self.assertEqual(summary["selected_arms"], ["left", "right"])
        self.assertEqual(summary["arm_mask"], {"left": 1.0, "right": 1.0})
        self.assertEqual(summary["gripper_command_count"], 2)
        self.assertEqual(summary["gripper_dropped_count"], 2)
        self.assertFalse(summary["allow_real_gripper_motion"])
        self.assertEqual(summary["collision_model_status"], "configured_estimate")


def measured_geometry():
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
            "cameras": {
                "head": {
                    "intrinsics_status": "measured",
                    "extrinsics_status": "measured",
                }
            },
        }
    )


if __name__ == "__main__":
    unittest.main()
