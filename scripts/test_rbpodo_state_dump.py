import unittest

import rbpodo_state_dump as state_dump


class FakeSData:
    time = 1.0
    real_vs_simulation_mode = 1
    init_state_info = 6
    init_error = 0
    op_stat_sos_flag = 0
    op_stat_ems_flag = 0
    op_stat_soft_estop_occur = 0
    op_stat_collision_occur = 0
    op_stat_self_collision = 0
    jnt_ang = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    jnt_ref = [0.25, 1.0, 2.5, 3.0, 3.5, 5.0]


class RbpodoStateDumpTest(unittest.TestCase):
    def test_report_includes_q_ref_source_and_actual_ref_delta(self):
        report = state_dump.build_report_for_sdata("127.0.0.1", FakeSData(), None, None, None)

        self.assertEqual(report["q_ref_source"], "python_rbpodo.sdata.jnt_ref")
        self.assertEqual(report["q_ref"], report["q_ref_deg"])
        self.assertEqual(report["jnt_ref"], report["q_ref_deg"])
        self.assertEqual(report["jnt_ref_deg"], report["q_ref_deg"])
        self.assertEqual(
            report["q_ref_actual_delta_deg"],
            [0.25, 0.0, 0.5, 0.0, -0.5, 0.0],
        )
        self.assertEqual(report["q_actual_vs_q_ref_max_abs_error_deg"], 0.5)
        self.assertEqual(report["diagnostics_suspect_reasons"], [])


if __name__ == "__main__":
    unittest.main()
