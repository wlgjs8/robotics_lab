import math
import unittest

import error_decomposition as decomp


GEOMETRY = {
    "center": [0.0, 0.0, 0.0],
    "axis1": [1.0, 0.0, 0.0],
    "axis2": [0.0, 1.0, 0.0],
    "radius": 1.0,
}


def circle_rows(*, phase_lag=0.0, center_offset=(0.0, 0.0), spike_indices=None, orientation_rad=0.0, count=120):
    spike_indices = set(spike_indices or [])
    rows = []
    for index in range(count):
        phase = 2.0 * math.pi * index / count
        ref = [math.cos(phase), math.sin(phase), 0.0]
        actual_phase = phase - phase_lag
        actual = [math.cos(actual_phase) + center_offset[0], math.sin(actual_phase) + center_offset[1], 0.0]
        if index in spike_indices:
            actual[0] += 0.35
        error = math.sqrt(sum((a - b) ** 2 for a, b in zip(actual, ref)))
        rows.append({
            "host_time_ns": index,
            "t_sec": index / count,
            "actual_x": actual[0],
            "actual_y": actual[1],
            "actual_z": actual[2],
            "reference_x": ref[0],
            "reference_y": ref[1],
            "reference_z": ref[2],
            "reference_phase_rad": phase,
            "position_error_m": error,
            "orientation_drift_rad": orientation_rad,
        })
    return rows


class CircleErrorDecompositionTests(unittest.TestCase):
    def test_synthetic_pure_phase_lag_classified_phase_lag_limited(self):
        rows = circle_rows(phase_lag=0.35)
        result = decomp.decompose_circle_run(
            rows,
            summary={
                "rms_error_m": 0.35,
                "estimated_phase_lag_rad": 0.35,
                "fit_center_error_m": 0.0,
                "radius_gain": 1.0,
                "radius_error_m": 0.0,
            },
            geometry=GEOMETRY,
            period_sec=1.0,
        )

        self.assertEqual(result["error_classification"], "phase_lag_limited")
        self.assertIn("phase_lag_limited", result["error_classifications"])
        self.assertLess(result["phase_aligned_rms_error_m"], 1e-9)

    def test_synthetic_center_drift_classified_center_drift_limited(self):
        rows = circle_rows(center_offset=(0.20, 0.0))
        result = decomp.decompose_circle_run(
            rows,
            summary={
                "rms_error_m": 0.20,
                "fit_center_error_m": 0.20,
                "fit_center_plane_m": [0.20, 0.0],
                "radius_gain": 1.0,
                "radius_error_m": 0.0,
            },
            geometry=GEOMETRY,
            period_sec=1.0,
        )

        self.assertEqual(result["error_classification"], "center_drift_limited")
        self.assertIn("center_drift_limited", result["error_classifications"])
        self.assertLess(result["center_removed_rms_error_m"], 1e-9)

    def test_synthetic_tail_spikes_classified_tail_spike_limited(self):
        rows = circle_rows(spike_indices={10, 30, 50, 70, 90, 110})
        result = decomp.decompose_circle_run(
            rows,
            summary={
                "rms_error_m": 0.08,
                "fit_center_error_m": 0.0,
                "radius_gain": 1.0,
                "radius_error_m": 0.0,
            },
            geometry=GEOMETRY,
            period_sec=1.0,
        )

        self.assertEqual(result["error_classification"], "tail_spike_limited")
        self.assertIn("tail_spike_limited", result["error_classifications"])
        self.assertGreater(result["tail_ratio"], 3.0)

    def test_orientation_drift_equivalent_is_computed(self):
        rows = circle_rows(orientation_rad=0.2)
        result = decomp.decompose_circle_run(
            rows,
            summary={
                "rms_error_m": 0.001,
                "fit_center_error_m": 0.0,
                "radius_gain": 1.0,
                "radius_error_m": 0.0,
                "p95_orientation_drift_rad": 0.2,
            },
            geometry=GEOMETRY,
            tool_offsets_m=[0.03, 0.05, 0.10],
            period_sec=1.0,
        )

        self.assertEqual(result["error_classification"], "orientation_limited")
        self.assertAlmostEqual(result["orientation_p50_rad"], 0.2)
        self.assertAlmostEqual(result["orientation_p95_deg"], math.degrees(0.2))
        self.assertAlmostEqual(result["orientation_position_equiv_50mm_m"], 0.01)
        self.assertAlmostEqual(result["orientation_position_equiv_50mm_mm"], 10.0)
        self.assertAlmostEqual(result["orientation_position_equiv_mm"]["50"]["p95"], 10.0)
        self.assertAlmostEqual(result["orientation_position_equiv_mm"]["100"]["max"], 20.0)
        self.assertAlmostEqual(result["orientation_error_position_equivalent_m"]["0.1"]["p95"], 0.02)

    def test_orientation_feedback_sign_diagnostic_flags_positive_kp_increase(self):
        baseline = {
            "controller": "twist_stand_feedback",
            "arm": "left",
            "profile": "gene_15cm_4s",
            "tracking_source_used": "tcp_ref_stand",
            "diameter_m": 0.15,
            "period_sec": 4.0,
            "command_rate_hz": 100.0,
            "feedback_kp_pos": 0.5,
            "feedback_kp_ori": 0.0,
            "p95_orientation_drift_rad": 0.08,
        }
        suspect = dict(baseline)
        suspect["feedback_kp_ori"] = 0.2
        suspect["p95_orientation_drift_rad"] = 0.12

        result = decomp.classify_orientation_feedback_sign([baseline, suspect])

        self.assertEqual(result["classification"], "orientation_feedback_suspect")
        self.assertEqual(result["comparisons"][0]["classification"], "orientation_feedback_suspect")
        self.assertAlmostEqual(result["comparisons"][0]["absolute_increase_rad"], 0.04)


if __name__ == "__main__":
    unittest.main()
