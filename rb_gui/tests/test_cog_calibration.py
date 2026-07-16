from __future__ import annotations

import csv
from dataclasses import replace
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from rb_servo_gui.cog_calibration import (
    CogCalibrationError,
    CogPoseMeasurement,
    CogSample,
    CogSampleAccumulator,
    estimate_payload,
    save_blocked_calibration_report,
    save_calibration_report,
)


GRAVITY_DIRECTIONS = np.asarray(
    [
        [0.0, 0.0, -1.0],
        [0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, -1.0],
        [-1.0, 1.0, -1.0],
        [1.0, -1.0, -1.0],
    ],
    dtype=float,
)
GRAVITY_DIRECTIONS /= np.linalg.norm(GRAVITY_DIRECTIONS, axis=1)[:, None]


def synthetic_poses(
    *,
    mass_kg: float = 1.7,
    cog_tcp_m: np.ndarray | None = None,
    force_bias_n: np.ndarray | None = None,
    torque_bias_nm: np.ndarray | None = None,
    samples_per_pose: int = 100,
    force_noise_n: float = 0.0,
    torque_noise_nm: float = 0.0,
    seed: int = 7,
) -> tuple[CogPoseMeasurement, ...]:
    cog = np.asarray([0.018, -0.012, 0.145] if cog_tcp_m is None else cog_tcp_m, dtype=float)
    force_bias = np.asarray([0.4, -0.25, 0.1] if force_bias_n is None else force_bias_n, dtype=float)
    torque_bias = np.asarray(
        [0.025, -0.018, 0.011] if torque_bias_nm is None else torque_bias_nm,
        dtype=float,
    )
    rng = np.random.default_rng(seed)
    freshness = 1
    poses: list[CogPoseMeasurement] = []
    for pose_index, direction in enumerate(GRAVITY_DIRECTIONS):
        gravity = 9.80665 * direction
        samples: list[CogSample] = []
        for sample_index in range(samples_per_pose):
            force = mass_kg * gravity + force_bias
            torque = np.cross(cog, mass_kg * gravity) + torque_bias
            if force_noise_n:
                force = force + rng.normal(0.0, force_noise_n, size=3)
            if torque_noise_nm:
                torque = torque + rng.normal(0.0, torque_noise_nm, size=3)
            samples.append(
                CogSample(
                    freshness_value=freshness,
                    wrench_tcp=tuple(force) + tuple(torque),
                    gravity_tcp=tuple(gravity),
                    received_monotonic=float(pose_index * samples_per_pose + sample_index) / 100.0,
                    raw_sensor_wrench=tuple(force) + tuple(torque),
                    t_tcp_sensor=(0.0, 0.0, -0.202642, 0.0, 0.0, np.pi / 2.0),
                    q_actual_deg=tuple(float(pose_index + joint) for joint in range(6)),
                    tcp_actual_stand=(
                        0.3 + pose_index * 0.001,
                        -0.1,
                        0.45,
                        0.0,
                        0.0,
                        0.0,
                    ),
                )
            )
            freshness += 1
        poses.append(
            CogPoseMeasurement(
                name=f"joint{pose_index + 1}",
                q_target_deg=tuple(float(pose_index + joint) for joint in range(6)),
                samples=tuple(samples),
            )
        )
    return tuple(poses)


class CogCalibrationTest(unittest.TestCase):
    def test_accumulator_counts_only_strictly_fresh_samples(self) -> None:
        accumulator = CogSampleAccumulator("joint1", [0.0] * 6)
        first = CogSample(5, (0.0,) * 6, (0.0, 0.0, -9.80665), 1.0)
        duplicate = CogSample(5, (1.0,) * 6, (0.0, 0.0, -9.80665), 1.1)
        old = CogSample(4, (2.0,) * 6, (0.0, 0.0, -9.80665), 1.2)
        newer = CogSample(6, (3.0,) * 6, (0.0, 0.0, -9.80665), 1.3)

        self.assertTrue(accumulator.add(first))
        self.assertFalse(accumulator.add(duplicate))
        self.assertFalse(accumulator.add(old))
        self.assertTrue(accumulator.add(newer))
        self.assertEqual(accumulator.sample_count, 2)
        self.assertEqual(accumulator.last_freshness_value, 6)
        frozen = accumulator.freeze()
        self.assertEqual([sample.freshness_value for sample in frozen.samples], [5, 6])

    def test_recovers_mass_cog_and_bias_from_noisy_samples(self) -> None:
        expected_mass = 1.7
        expected_cog = np.asarray([0.018, -0.012, 0.145])
        expected_force_bias = np.asarray([0.4, -0.25, 0.1])
        expected_torque_bias = np.asarray([0.025, -0.018, 0.011])
        poses = synthetic_poses(force_noise_n=0.04, torque_noise_nm=0.004, samples_per_pose=200)

        estimate = estimate_payload(
            poses, wrench_convention="payload_load", min_poses=5
        )

        self.assertAlmostEqual(estimate.mass_kg, expected_mass, delta=0.002)
        np.testing.assert_allclose(estimate.cog_tcp_m, expected_cog, atol=3e-4)
        np.testing.assert_allclose(estimate.force_bias_n, expected_force_bias, atol=0.006)
        np.testing.assert_allclose(estimate.torque_bias_nm, expected_torque_bias, atol=7e-4)
        self.assertEqual(estimate.force_design_rank, 4)
        self.assertEqual(estimate.torque_design_rank, 6)
        self.assertLess(estimate.force_fit_rms_n, 0.006)
        self.assertLess(estimate.torque_fit_rms_nm, 0.0007)
        self.assertFalse(estimate.gravity_compensation_ambiguous)
        self.assertGreater(estimate.force_gravity_correlation or 0.0, 0.999)
        self.assertEqual(len(estimate.pose_residuals), len(poses))
        first_residual = estimate.pose_residuals[0]
        np.testing.assert_allclose(
            np.asarray(first_residual.observed_force_n)
            - np.asarray(first_residual.predicted_force_n),
            first_residual.force_residual_n,
        )
        np.testing.assert_allclose(
            np.asarray(first_residual.observed_torque_nm)
            - np.asarray(first_residual.predicted_torque_nm),
            first_residual.torque_residual_nm,
        )
        self.assertEqual(len(estimate.leave_one_out), len(poses))
        self.assertTrue(all(item.valid for item in estimate.leave_one_out))
        self.assertTrue(estimate.provisional)
        self.assertFalse(estimate.applied)

    def test_sensor_reaction_convention_recovers_positive_payload(self) -> None:
        load_poses = synthetic_poses(samples_per_pose=20)
        reaction_poses = tuple(
            CogPoseMeasurement(
                pose.name,
                pose.q_target_deg,
                tuple(
                    CogSample(
                        freshness_value=sample.freshness_value,
                        wrench_tcp=tuple(-value for value in sample.wrench_tcp),
                        gravity_tcp=sample.gravity_tcp,
                        received_monotonic=sample.received_monotonic,
                    )
                    for sample in pose.samples
                ),
            )
            for pose in load_poses
        )

        estimate = estimate_payload(
            reaction_poses, wrench_convention="sensor_reaction"
        )

        self.assertEqual(estimate.wrench_convention, "sensor_reaction")
        self.assertAlmostEqual(estimate.mass_kg, 1.7, places=10)
        np.testing.assert_allclose(estimate.cog_tcp_m, [0.018, -0.012, 0.145])
        np.testing.assert_allclose(estimate.force_bias_n, [-0.4, 0.25, -0.1])

    def test_latest_real_capture_signature_is_positive_but_model_mismatched(self) -> None:
        # Pose means from servo_log_20260714_153133.csv.  This compact fixture
        # prevents the old negative-mass masking bug from returning: the
        # reaction convention yields a positive candidate, but the scalar
        # gravity-wrench residual remains far beyond the tracked fit bound.
        gravity = np.asarray(
            [
                [-4.0774, -7.6627, 4.5638],
                [-1.8971, 6.4164, 7.1694],
                [-2.5747, 4.3194, 8.4193],
                [-4.0267, 8.1109, 3.7643],
                [0.3731, -1.6678, 9.6566],
                [2.5515, -6.6953, 6.6957],
                [0.4461, -4.4086, 8.7485],
            ]
        )
        wrench = np.asarray(
            [
                [-5.6031, 18.3302, -25.4884, 4.0257, 1.3740, 0.0229],
                [6.3918, 5.8654, -25.3729, 1.6701, -1.5151, 0.0419],
                [4.9918, 7.7160, -24.6019, 2.0312, -1.1574, 0.0350],
                [9.8217, 6.2791, -25.1770, 1.8774, -2.2205, 0.0326],
                [-3.2799, 9.7607, -24.7065, 2.2440, 0.6341, 0.0403],
                [-10.0140, 11.6191, -26.0943, 2.4718, 2.1000, 0.0422],
                [-6.0903, 11.8469, -24.8146, 2.6231, 1.2807, 0.0403],
            ]
        )
        poses = tuple(
            CogPoseMeasurement(
                f"joint{index + 1}",
                tuple(float(index + joint) for joint in range(6)),
                tuple(
                    CogSample(
                        freshness_value=2 * index + sample_index,
                        wrench_tcp=tuple(wrench[index]),
                        gravity_tcp=tuple(gravity[index]),
                        received_monotonic=float(2 * index + sample_index),
                    )
                    for sample_index in range(2)
                ),
            )
            for index in range(len(gravity))
        )

        estimate = estimate_payload(
            poses,
            wrench_convention="sensor_reaction",
            max_condition_number=1000.0,
        )

        self.assertGreater(estimate.mass_kg, 0.6)
        self.assertLess(estimate.mass_kg, 0.8)
        self.assertGreater(estimate.force_fit_rms_n, 3.5)

    def test_duplicate_gravity_directions_are_rank_deficient(self) -> None:
        source = synthetic_poses(samples_per_pose=2)[0]
        poses = tuple(
            CogPoseMeasurement(
                name=f"duplicate{index}",
                q_target_deg=tuple(float(index + joint) for joint in range(6)),
                samples=tuple(
                    CogSample(
                        freshness_value=100 * index + sample_index,
                        wrench_tcp=sample.wrench_tcp,
                        gravity_tcp=sample.gravity_tcp,
                        received_monotonic=float(index + sample_index),
                    )
                    for sample_index, sample in enumerate(source.samples)
                ),
            )
            for index in range(5)
        )

        with self.assertRaises(CogCalibrationError) as context:
            estimate_payload(
                poses, wrench_convention="payload_load", min_poses=5
            )

        self.assertEqual(context.exception.code, "rank_deficient")

    def test_explicit_condition_limit_rejects_without_hidden_default(self) -> None:
        poses = synthetic_poses(samples_per_pose=5)
        unconstrained = estimate_payload(poses, wrench_convention="payload_load")

        with self.assertRaises(CogCalibrationError) as context:
            estimate_payload(
                poses,
                wrench_convention="payload_load",
                max_condition_number=max(
                    unconstrained.force_design_condition,
                    unconstrained.torque_design_condition,
                )
                * 0.5,
            )

        self.assertEqual(context.exception.code, "ill_conditioned")

    def test_condition_gate_uses_the_weighted_wls_design(self) -> None:
        poses = list(
            synthetic_poses(
                samples_per_pose=20,
                force_noise_n=0.02,
                torque_noise_nm=0.002,
            )
        )
        first = poses[0]
        force = np.asarray(first.samples[0].wrench_tcp[:3])
        torque = np.asarray(first.samples[0].wrench_tcp[3:])
        poses[0] = CogPoseMeasurement(
            name=first.name,
            q_target_deg=first.q_target_deg,
            samples=tuple(
                CogSample(
                    freshness_value=sample.freshness_value,
                    wrench_tcp=tuple(force) + tuple(torque),
                    gravity_tcp=sample.gravity_tcp,
                    received_monotonic=sample.received_monotonic,
                )
                for sample in first.samples
            ),
        )

        with self.assertRaises(CogCalibrationError) as context:
            estimate_payload(
                tuple(poses),
                wrench_convention="payload_load",
                max_condition_number=1000.0,
            )

        self.assertEqual(context.exception.code, "ill_conditioned")

    def test_low_gravity_signal_relative_to_residual_is_flagged_ambiguous(self) -> None:
        base = synthetic_poses(samples_per_pose=20, mass_kg=0.02)
        poses: list[CogPoseMeasurement] = []
        for pose_index, pose in enumerate(base):
            samples: list[CogSample] = []
            for sample_index, sample in enumerate(pose.samples):
                gravity = np.asarray(sample.gravity_tcp)
                if pose_index < 3:
                    force = 0.02 * gravity + np.asarray([0.1, -0.2, 0.3])
                    force += (sample_index - 9.5) * 1e-7
                else:
                    force = -0.5 * gravity + np.asarray([0.1, -0.2, 0.3])
                    force += ((-1.0) ** sample_index) * 0.5
                samples.append(
                    CogSample(
                        freshness_value=sample.freshness_value,
                        wrench_tcp=tuple(force) + sample.wrench_tcp[3:],
                        gravity_tcp=sample.gravity_tcp,
                        received_monotonic=sample.received_monotonic,
                    )
                )
            poses.append(CogPoseMeasurement(pose.name, pose.q_target_deg, tuple(samples)))

        estimate = estimate_payload(
            tuple(poses), wrench_convention="payload_load"
        )

        self.assertTrue(estimate.gravity_compensation_ambiguous)
        self.assertTrue(
            any("gravity" in reason for reason in estimate.ambiguity_reasons)
        )

    def test_min_poses_rejects_non_integer_values(self) -> None:
        poses = synthetic_poses(samples_per_pose=2)
        for value in (5.9, "5", True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                estimate_payload(
                    poses,
                    wrench_convention="payload_load",
                    min_poses=value,
                )  # type: ignore[arg-type]

    def test_gravity_compensated_wrench_is_reported_as_ambiguous(self) -> None:
        poses = synthetic_poses(
            mass_kg=0.0,
            cog_tcp_m=np.zeros(3),
            samples_per_pose=5,
        )

        with self.assertRaises(CogCalibrationError) as context:
            estimate_payload(poses, wrench_convention="payload_load")

        self.assertEqual(context.exception.code, "gravity_compensation_ambiguous")
        self.assertIn("gravity compensated", context.exception.detail)

    def test_report_contains_provisional_result_and_raw_samples(self) -> None:
        poses = synthetic_poses(samples_per_pose=3)
        estimate = estimate_payload(poses, wrench_convention="payload_load")
        with tempfile.TemporaryDirectory() as directory:
            paths = save_calibration_report(
                directory,
                run_id="right-20260713T120000Z",
                arm="right",
                poses=poses,
                estimate=estimate,
                provenance={"server_config": "stack_real.yaml", "waypoint_hash": "abc123"},
            )

            report = json.loads(paths.report_json.read_text(encoding="utf-8"))
            self.assertEqual(report["status"], "PROVISIONAL / NOT APPLIED")
            self.assertTrue(report["provisional"])
            self.assertFalse(report["applied"])
            self.assertAlmostEqual(report["estimate"]["mass_kg"], 1.7, places=10)
            self.assertEqual(report["provenance"]["waypoint_hash"], "abc123")
            with paths.samples_csv.open(encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), len(poses) * 3)
            self.assertEqual(rows[0]["pose_name"], "joint1")
            self.assertEqual(rows[-1]["pose_name"], f"joint{len(poses)}")
            self.assertEqual(Path(paths.report_json).parent, Path(directory) / "right-20260713T120000Z")

    def test_blocked_report_preserves_samples_and_candidate_without_applying(self) -> None:
        poses = synthetic_poses(samples_per_pose=3)
        candidate = estimate_payload(poses, wrench_convention="payload_load")
        with tempfile.TemporaryDirectory() as directory:
            paths = save_blocked_calibration_report(
                directory,
                run_id="right-model-mismatch",
                arm="right",
                poses=poses,
                failure_code="force_fit_rms_exceeded",
                failure_detail="captured source does not match configured model",
                provenance={"wrench_convention": "sensor_reaction"},
                candidate_estimate=candidate,
            )

            report = json.loads(paths.report_json.read_text(encoding="utf-8"))
            self.assertEqual(report["schema_version"], 3)
            self.assertEqual(
                report["evidence_contract"]["transform_convention"],
                "point_tcp = T_tcp_sensor * point_sensor",
            )
            self.assertEqual(report["status"], "BLOCKED / NOT APPLIED")
            self.assertTrue(report["blocked"])
            self.assertFalse(report["applied"])
            self.assertEqual(report["failure"]["code"], "force_fit_rms_exceeded")
            self.assertAlmostEqual(report["candidate_estimate"]["mass_kg"], 1.7)
            with paths.samples_csv.open(encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
                self.assertEqual(len(rows), len(poses) * 3)
                self.assertEqual(rows[0]["t_tcp_sensor_z_m"], "-0.202642")
                self.assertEqual(rows[0]["raw_sensor_force_x_n"], "0.4")
                self.assertNotEqual(rows[0]["q_actual_deg"], "")
                self.assertNotEqual(rows[0]["tcp_actual_stand_x_m"], "")

    def test_report_bundle_is_all_or_nothing_on_write_failure(self) -> None:
        poses = synthetic_poses(samples_per_pose=3)
        estimate = estimate_payload(poses, wrench_convention="payload_load")
        from rb_servo_gui import cog_calibration

        real_atomic_write = cog_calibration._atomic_write
        calls = 0

        def fail_second_write(path, content):
            nonlocal calls
            calls += 1
            if calls == 2:
                raise OSError("injected CSV write failure")
            return real_atomic_write(path, content)

        with tempfile.TemporaryDirectory() as directory, mock.patch.object(
            cog_calibration, "_atomic_write", side_effect=fail_second_write
        ):
            with self.assertRaises(OSError):
                save_calibration_report(
                    directory,
                    run_id="right-write-failure",
                    arm="right",
                    poses=poses,
                    estimate=estimate,
                    provenance={},
                )
            self.assertFalse((Path(directory) / "right-write-failure").exists())
            self.assertEqual(list(Path(directory).iterdir()), [])

    def test_report_rejects_non_provisional_estimate(self) -> None:
        poses = synthetic_poses(samples_per_pose=3)
        estimate = replace(
            estimate_payload(poses, wrench_convention="payload_load"),
            provisional=False,
            applied=True,
        )
        with tempfile.TemporaryDirectory() as directory, self.assertRaises(ValueError):
            save_calibration_report(
                directory,
                run_id="right-invalid-estimate",
                arm="right",
                poses=poses,
                estimate=estimate,
                provenance={},
            )


if __name__ == "__main__":
    unittest.main()
