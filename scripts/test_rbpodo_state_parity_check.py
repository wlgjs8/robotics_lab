import unittest

import rbpodo_state_parity_check as parity


RAW_BASE = {
    "time": 1.0,
    "real_vs_simulation_mode": 1,
    "init_state_info": 6,
    "init_error": 0,
    "op_stat_sos_flag": 0,
    "op_stat_ems_flag": 0,
    "op_stat_soft_estop_occur": 0,
    "op_stat_collision_occur": 0,
    "op_stat_self_collision": 0,
}


def python_sample(raw=None, q_ref=None, suspect=False):
    raw = dict(RAW_BASE if raw is None else raw)
    q_ref = [10.0, 11.0, 12.0, 13.0, 14.0, 15.0] if q_ref is None else q_ref
    return {
        "arm": "left",
        "ip": "172.28.60.200",
        "sample_time_ns": 1_000_000_000,
        "q_actual_deg": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
        "q_ref_deg": q_ref,
        "q_target_deg": q_ref,
        "q_ref_source": "python_rbpodo.sdata.jnt_ref",
        "raw": raw,
        "diagnostics_suspect": suspect,
        "python_time_plausible": True,
    }


def cpp_sample(raw=None, q_ref=None, suspect=False):
    raw = dict(RAW_BASE if raw is None else raw)
    q_ref = [10.0, 11.0, 12.0, 13.0, 14.0, 15.0] if q_ref is None else q_ref
    return {
        "arm": "left",
        "host_time_ns": 1_000_010_000,
        "q_actual_deg": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
        "q_ref_deg": q_ref,
        "q_target_deg": q_ref,
        "q_ref_source": "rbpodo.sdata.jnt_ref",
        "rbpodo_sdk_state_source": "CobotData.request_data",
        "rbpodo_state_decode_policy": "strict_boolean_flags_with_suspect_large_values",
        "raw": raw,
        "diagnostics_suspect": suspect,
        "cpp_time_plausible": True,
    }


class RbpodoStateParityCheckTest(unittest.TestCase):
    def test_fake_python_cpp_samples_pass(self):
        summary, rows = parity.compare_samples([python_sample()], [cpp_sample()])

        self.assertEqual(summary["result"], "passed")
        self.assertEqual(len(rows), 1)
        self.assertEqual(summary["metrics"]["max_q_actual_diff_deg"], 0.0)
        self.assertEqual(summary["metrics"]["max_q_ref_diff_deg"], 0.0)
        self.assertEqual(summary["metrics"]["raw_field_match_rate"], 1.0)
        self.assertTrue(summary["metrics"]["q_ref_source_available"])

    def test_q_ref_mismatch_fails_with_field_name(self):
        summary, _ = parity.compare_samples(
            [python_sample()],
            [cpp_sample(q_ref=[10.0, 11.0, 12.5, 13.0, 14.0, 15.0])],
        )

        self.assertEqual(summary["result"], "failed")
        self.assertIn("q_ref_deg", summary["reason"])

    def test_suspect_but_consistent_for_shared_huge_diagnostic(self):
        raw = dict(RAW_BASE)
        raw["op_stat_self_collision"] = 1977953904

        summary, _ = parity.compare_samples(
            [python_sample(raw=raw, suspect=True)],
            [cpp_sample(raw=raw, suspect=True)],
        )

        self.assertEqual(summary["result"], "suspect_but_consistent")
        self.assertEqual(summary["metrics"]["raw_field_match_rate"], 1.0)
        self.assertEqual(summary["metrics"]["diagnostics_suspect_agreement_rate"], 1.0)


if __name__ == "__main__":
    unittest.main()
