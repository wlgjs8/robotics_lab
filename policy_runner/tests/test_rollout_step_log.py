from __future__ import annotations

import json
import tempfile
import unittest
from dataclasses import dataclass
from pathlib import Path

from policy_runner.rollout_step_log import (
    ROLLOUT_STEP_LOG_SCHEMA,
    RolloutStepLogger,
    build_rollout_step_record,
)
from policy_runner.servo_command_client import CommandIntent


@dataclass
class SyntheticConditionedTarget:
    pose: list[float]
    hold: bool
    stall: bool
    dropout: bool
    reanchor: bool
    chunk_id: int
    chunk_index_lo: int
    chunk_index_hi: int
    interpolation_alpha: float
    twist: list[float]
    emitted_delta_from_prev: list[float]


def _conditioned(
    pose: list[float],
    *,
    chunk_id: int = 4,
    hold: bool = False,
    stall: bool = False,
) -> SyntheticConditionedTarget:
    return SyntheticConditionedTarget(
        pose=pose,
        hold=hold,
        stall=stall,
        dropout=False,
        reanchor=False,
        chunk_id=chunk_id,
        chunk_index_lo=2,
        chunk_index_hi=3,
        interpolation_alpha=0.25,
        twist=[0.0] * 6,
        emitted_delta_from_prev=[0.0] * 6,
    )


def _state_payload() -> dict:
    return {
        "left": {
            "tcp_actual_stand": {
                "x": 0.4,
                "y": -0.1,
                "z": 0.295,
                "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
            "gripper": {
                "valid": True,
                "stale": False,
                "percent": 12.0,
                "target_percent": 77.0,
            },
            "force_control": {
                "compliance_offset_surface": [0.001, 0.002, 0.003, 0.01, 0.02, 0.03],
                "correction_m": 0.004,
            },
            "force_torque": {
                "wrench_tcp": [1.0, 2.0, 3.0, 0.1, 0.2, 0.3],
                "control_external_wrench": [4.0, 5.0, 6.0, 0.4, 0.5, 0.6],
            },
        },
        "right": {
            "tcp_actual_stand": {
                "x": 0.5,
                "y": 0.1,
                "z": 0.405,
                "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
            "gripper": {"valid": True, "stale": False, "percent": 82.0},
        },
    }


class RolloutStepLoggerTest(unittest.TestCase):
    def test_jsonl_schema_from_conditioned_targets_and_state(self) -> None:
        left = _conditioned([0.4, -0.1, 0.300, 0.0, 0.0, 0.0, 1.0])
        right = _conditioned([0.5, 0.1, 0.400, 0.0, 0.0, 0.0, 1.0])
        intent = CommandIntent(
            "TcpPoseTarget",
            left={"mode": "TcpPoseTarget", "gripper_target": 10.0},
            right={"mode": "TcpPoseTarget", "gripper_target": 80.0},
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "nested" / "steps.jsonl"
            logger = RolloutStepLogger(path)
            accepted = logger.log_step(
                state_payload=_state_payload(),
                command_intent=intent,
                conditioned_targets={"left": left, "right": right},
                raw_delta_ee_local={
                    "left": [0.001, 0.002, -0.003, 0.01, 0.02, 0.03],
                    "right": [-0.001, -0.002, 0.004, -0.01, -0.02, -0.03],
                },
                gripper_cmd_pct={"left": 9.0, "right": 79.0},
                chunk_id=4,
                chunk_step_index=3,
                inference_latency_ms=18.5,
                t_mono=123.25,
                t_wall=1_700_000_000.5,
            )
            logger.close()
            records = [
                json.loads(line)
                for line in path.read_text(encoding="utf-8").splitlines()
            ]

        self.assertTrue(accepted)
        self.assertEqual(len(records), 1)
        record = records[0]
        self.assertEqual(record["schema"], ROLLOUT_STEP_LOG_SCHEMA)
        self.assertEqual(record["chunk_id"], 4)
        self.assertEqual(record["chunk_step_index"], 3)
        self.assertFalse(record["stall"])
        self.assertFalse(record["hold"])
        self.assertEqual(record["inference_latency_ms"], 18.5)
        self.assertEqual(set(record["arms"]), {"left", "right"})

        left_record = record["arms"]["left"]
        self.assertEqual(len(left_record["cmd_pose"]), 7)
        self.assertEqual(len(left_record["meas_pose"]), 7)
        self.assertAlmostEqual(left_record["cmd_minus_meas_z_mm"], 5.0, places=5)
        self.assertEqual(
            left_record["raw_delta_ee_local"],
            [0.001, 0.002, -0.003, 0.01, 0.02, 0.03],
        )
        self.assertEqual(left_record["gripper_cmd_pct"], 9.0)
        # Measured feedback must not be replaced by the state target_percent.
        self.assertEqual(left_record["gripper_meas_pct"], 12.0)
        self.assertEqual(left_record["compliance_offset_surface"][2], 0.003)
        self.assertEqual(left_record["correction_m"], 0.004)
        self.assertEqual(left_record["wrench_tcp_fz"], 3.0)
        self.assertEqual(left_record["control_external_wrench_fz"], 6.0)
        self.assertIsNone(record["arms"]["right"]["compliance_offset_surface"])
        self.assertIsNone(record["arms"]["right"]["wrench_tcp_fz"])

    def test_gripper_feedback_age_is_recorded_and_never_fabricated(self) -> None:
        payload = _state_payload()
        payload["left"]["gripper"]["feedback_age_ms"] = 23.5
        record = build_rollout_step_record(
            state_payload=payload, command_intent=None, conditioned_targets=None,
            raw_delta_ee_local=None, gripper_cmd_pct=None, chunk_id=1,
            chunk_step_index=0, stall=False, hold=False, inference_latency_ms=None,
            t_mono=1.0, t_wall=2.0,
        )
        self.assertAlmostEqual(record["arms"]["left"]["gripper_feedback_age_ms"], 23.5)
        # No stamp -> None. A fabricated 0 would read as "feedback is instant" and
        # send the latency split the wrong way.
        self.assertIsNone(record["arms"]["right"]["gripper_feedback_age_ms"])

    def test_gripper_sample_age_is_recorded_separately_from_transport(self) -> None:
        # feedback_age_ms covers publish->receive only (~0.05 ms in practice) and
        # made the jaw feedback look instant. The pika SAMPLE age is the dominant
        # term and is logged as its own field.
        payload = _state_payload()
        payload["left"]["gripper"]["feedback_age_ms"] = 0.05
        payload["left"]["gripper"]["sample_age_ms"] = 27.0
        record = build_rollout_step_record(
            state_payload=payload, command_intent=None, conditioned_targets=None,
            raw_delta_ee_local=None, gripper_cmd_pct=None, chunk_id=1,
            chunk_step_index=0, stall=False, hold=False, inference_latency_ms=None,
            t_mono=1.0, t_wall=2.0,
        )
        self.assertAlmostEqual(record["arms"]["left"]["gripper_sample_age_ms"], 27.0)
        # Publisher too old to stamp it -> None, never a fabricated 0.
        self.assertIsNone(record["arms"]["right"]["gripper_sample_age_ms"])

    def test_gripper_proprio_value_and_source_are_recorded(self) -> None:
        # Which signal reached the policy is otherwise unrecoverable from the
        # log: measured and commanded are both present, the choice was not.
        record = build_rollout_step_record(
            state_payload=_state_payload(), command_intent=None,
            conditioned_targets=None, raw_delta_ee_local=None, gripper_cmd_pct=None,
            chunk_id=1, chunk_step_index=0, stall=False, hold=False,
            inference_latency_ms=None, t_mono=1.0, t_wall=2.0,
            gripper_proprio={
                "left": {"pct": 12.0, "source": "actual"},
                "right": {"pct": 0.0, "source": "hybrid_free"},
            },
        )
        self.assertAlmostEqual(record["arms"]["left"]["gripper_proprio_pct"], 12.0)
        self.assertEqual(record["arms"]["left"]["gripper_proprio_source"], "actual")
        self.assertAlmostEqual(record["arms"]["right"]["gripper_proprio_pct"], 0.0)
        self.assertEqual(record["arms"]["right"]["gripper_proprio_source"], "hybrid_free")

    def test_gripper_proprio_absent_records_null_not_a_guess(self) -> None:
        record = build_rollout_step_record(
            state_payload=_state_payload(), command_intent=None,
            conditioned_targets=None, raw_delta_ee_local=None, gripper_cmd_pct=None,
            chunk_id=1, chunk_step_index=0, stall=False, hold=False,
            inference_latency_ms=None, t_mono=1.0, t_wall=2.0,
        )
        self.assertIsNone(record["arms"]["left"]["gripper_proprio_pct"])
        self.assertIsNone(record["arms"]["left"]["gripper_proprio_source"])

    def test_force_control_state_is_recorded_per_arm(self) -> None:
        # An unprotected rollout (FT zero never ran because no Init Motion
        # preceded it) must be visible in the log, not inferable.
        payload = _state_payload()
        payload["left"]["force_control"] = {"enabled": True, "state": "awaiting_init_tare"}
        record = build_rollout_step_record(
            state_payload=payload,
            command_intent=None,
            conditioned_targets=None,
            raw_delta_ee_local=None,
            gripper_cmd_pct=None,
            chunk_id=1,
            chunk_step_index=0,
            stall=False,
            hold=False,
            inference_latency_ms=None,
            t_mono=1.0,
            t_wall=2.0,
        )
        self.assertEqual(record["arms"]["left"]["force_control_state"], "awaiting_init_tare")
        # Absent force_control block -> null, never a fabricated "armed".
        self.assertIsNone(record["arms"]["right"]["force_control_state"])

    def test_rtc_block_records_configured_vs_realized_delay(self) -> None:
        record = build_rollout_step_record(
            state_payload=_state_payload(),
            command_intent=None,
            conditioned_targets=None,
            raw_delta_ee_local=None,
            gripper_cmd_pct=None,
            chunk_id=1,
            chunk_step_index=0,
            stall=False,
            hold=False,
            inference_latency_ms=None,
            rtc={
                "configured_delay": 3,
                "realized_delay": 5,
                "execute_horizon": 5,
                "schedule": "exp",
                "alignment_outcome": "aligned",
            },
            t_mono=1.0,
            t_wall=2.0,
        )
        # realized > configured: the executed window ran rows the server never froze.
        self.assertEqual(record["rtc"]["configured_delay"], 3)
        self.assertEqual(record["rtc"]["realized_delay"], 5)
        self.assertEqual(record["rtc"]["delay_error"], 2)
        self.assertEqual(record["rtc"]["execute_horizon"], 5)
        self.assertEqual(record["rtc"]["schedule"], "exp")
        self.assertEqual(record["rtc"]["alignment_outcome"], "aligned")

    def test_rtc_block_absent_when_rtc_off(self) -> None:
        record = build_rollout_step_record(
            state_payload=_state_payload(),
            command_intent=None,
            conditioned_targets=None,
            raw_delta_ee_local=None,
            gripper_cmd_pct=None,
            chunk_id=1,
            chunk_step_index=0,
            stall=False,
            hold=False,
            inference_latency_ms=None,
            rtc=None,
            t_mono=1.0,
            t_wall=2.0,
        )
        self.assertNotIn("rtc", record)

    def test_hot_path_writer_exception_disables_logging_without_raising(self) -> None:
        class ExplodingWriter:
            enabled = True
            disabled_reason = None

            def submit(self, _record):
                raise OSError("synthetic write failure")

            def disable(self, reason):
                self.enabled = False
                self.disabled_reason = reason

            def close(self):
                return None

        writer = ExplodingWriter()
        logger = RolloutStepLogger(
            "unused.jsonl",
            writer_factory=lambda _path: writer,
        )
        accepted = logger.log_step(
            state_payload=_state_payload(),
            command_intent=None,
            conditioned_targets={
                "left": _conditioned(
                    [0.4, -0.1, 0.3, 0.0, 0.0, 0.0, 1.0],
                    hold=True,
                    stall=True,
                )
            },
            raw_delta_ee_local=None,
            gripper_cmd_pct=None,
            chunk_id=4,
            chunk_step_index=None,
            stall=True,
            hold=True,
            t_mono=1.0,
            t_wall=2.0,
        )

        self.assertFalse(accepted)
        self.assertFalse(logger.enabled)
        self.assertIn("hot_path_error:OSError", logger.disabled_reason)
        # Repeated calls remain no-ops and never re-raise into the rollout.
        self.assertFalse(
            logger.log_step(
                state_payload={},
                command_intent=None,
                conditioned_targets=None,
                raw_delta_ee_local=None,
                gripper_cmd_pct=None,
                chunk_id=None,
                chunk_step_index=None,
                t_mono=3.0,
                t_wall=4.0,
            )
        )

    def test_missing_measured_pose_is_null_not_zero_pose(self) -> None:
        record_writer: list[dict] = []

        class CapturingWriter:
            enabled = True

            def submit(self, record):
                record_writer.append(record)
                return True

            def close(self):
                return None

        logger = RolloutStepLogger(
            "unused.jsonl",
            writer_factory=lambda _path: CapturingWriter(),
        )
        self.assertTrue(
            logger.log_step(
                state_payload={"left": {}, "right": {}},
                command_intent=None,
                conditioned_targets=None,
                raw_delta_ee_local=None,
                gripper_cmd_pct=None,
                chunk_id=1,
                chunk_step_index=0,
                t_mono=1.0,
                t_wall=2.0,
            )
        )
        logger.close()

        self.assertIsNone(record_writer[0]["arms"]["left"]["meas_pose"])
        self.assertIsNone(record_writer[0]["arms"]["right"]["meas_pose"])

    def test_controller_simulation_uses_declared_reference_pose(self) -> None:
        payload = {
            "left": {
                "tcp_actual_stand": {
                    "x": 0.1,
                    "y": 0.0,
                    "z": 0.2,
                    "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                },
                "tcp_ref_valid": True,
                "tcp_ref_stand": {
                    "x": 0.3,
                    "y": 0.0,
                    "z": 0.4,
                    "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                },
                "physical_motion_expected": False,
                "cartesian_gate": {
                    "operation_mode": "simulation",
                    "physical_motion_expected": False,
                    "controller_simulation_servo_state_source": "reference",
                },
            },
            "right": {},
        }
        record = build_rollout_step_record(
            state_payload=payload,
            command_intent=None,
            conditioned_targets=None,
            raw_delta_ee_local=None,
            gripper_cmd_pct=None,
            chunk_id=1,
            chunk_step_index=0,
            stall=False,
            hold=False,
            inference_latency_ms=None,
            t_mono=1.0,
            t_wall=2.0,
        )

        self.assertEqual(record["arms"]["left"]["meas_pose"][0], 0.3)
        self.assertEqual(record["arms"]["left"]["meas_pose"][2], 0.4)


if __name__ == "__main__":
    unittest.main()
