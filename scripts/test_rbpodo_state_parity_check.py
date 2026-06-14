import socket
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

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


def python_sample(raw=None, q_actual=None, q_ref=None, suspect=False):
    raw = dict(RAW_BASE if raw is None else raw)
    q_actual = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0] if q_actual is None else q_actual
    q_ref = [10.0, 11.0, 12.0, 13.0, 14.0, 15.0] if q_ref is None else q_ref
    return {
        "arm": "left",
        "ip": "172.28.60.200",
        "sample_time_ns": 1_000_000_000,
        "q_actual_deg": q_actual,
        "q_ref_deg": q_ref,
        "q_target_deg": q_ref,
        "jnt_ref": q_ref,
        "jnt_ref_deg": q_ref,
        "q_ref_source": "python_rbpodo.sdata.jnt_ref",
        "raw": raw,
        "diagnostics_suspect": suspect,
        "python_time_plausible": True,
    }


def cpp_sample(raw=None, q_actual=None, q_ref=None, suspect=False):
    raw = dict(RAW_BASE if raw is None else raw)
    q_actual = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0] if q_actual is None else q_actual
    q_ref = [10.0, 11.0, 12.0, 13.0, 14.0, 15.0] if q_ref is None else q_ref
    return {
        "arm": "left",
        "host_time_ns": 1_000_010_000,
        "q_actual_deg": q_actual,
        "q_ref_deg": q_ref,
        "q_target_deg": q_ref,
        "jnt_ref_deg": q_ref,
        "q_ref_published": True,
        "q_target_published": True,
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

        self.assertEqual(summary["result"], "failed_parity_mismatch")
        self.assertIn("q_ref_deg", summary["reason"])

    def test_suspect_but_consistent_for_shared_huge_diagnostic(self):
        raw = dict(RAW_BASE)
        raw["op_stat_self_collision"] = 1977953904

        summary, _ = parity.compare_samples(
            [python_sample(raw=raw, suspect=True)],
            [cpp_sample(raw=raw, suspect=True)],
        )

        self.assertEqual(summary["result"], "suspect_but_consistent")
        self.assertIn("diagnostics_suspect_unresolved", summary["caveats"])
        self.assertEqual(summary["metrics"]["raw_field_match_rate"], 1.0)
        self.assertEqual(summary["metrics"]["diagnostics_suspect_agreement_rate"], 1.0)

    def test_float32_sized_q_actual_diff_is_not_decode_mismatch(self):
        raw = dict(RAW_BASE)
        raw["op_stat_self_collision"] = 1977953904

        summary, _ = parity.compare_samples(
            [python_sample(raw=raw, q_actual=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0], suspect=True)],
            [cpp_sample(raw=raw, q_actual=[1.0002, 2.0, 3.0, 4.0, 5.0, 6.0], suspect=True)],
        )

        self.assertEqual(summary["result"], "suspect_but_consistent")
        self.assertIn("diagnostics_suspect_unresolved", summary["caveats"])
        self.assertNotIn("q_actual_deg", summary["reason"])
        self.assertAlmostEqual(summary["metrics"]["max_q_actual_diff_deg"], 0.0002)

    def test_large_q_actual_diff_still_fails_decode_mismatch(self):
        summary, _ = parity.compare_samples(
            [python_sample(q_actual=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0])],
            [cpp_sample(q_actual=[1.01, 2.0, 3.0, 4.0, 5.0, 6.0])],
        )

        self.assertEqual(summary["result"], "failed_parity_mismatch")
        self.assertIn("q_actual_deg", summary["reason"])
        self.assertAlmostEqual(summary["metrics"]["max_q_actual_diff_deg"], 0.01)

    def test_send_servo_commands_true_config_rejected(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            config = Path(tmpdir) / "unsafe.yaml"
            config.write_text("servo:\n  send_servo_commands: true\n", encoding="utf-8")
            args = SimpleNamespace(
                duration_sec=1.0,
                sample_rate_hz=1.0,
                request_timeout_sec=0.1,
                startup_timeout_sec=0.1,
                tolerance_deg=0.0,
                nearest_max_delta_sec=0.1,
                ips=["172.28.60.200"],
                i_understand_this_connects_to_real_controller=True,
                use_running_server=True,
                server=None,
                server_config=config,
            )

            with self.assertRaises(parity.StateParityError) as ctx:
                parity.ensure_safe_args(args)

            self.assertIn("send_servo_commands: false", str(ctx.exception))

    def test_server_exit_before_state_is_classified_with_log_tail(self):
        class TimeoutSocket:
            def settimeout(self, _timeout):
                pass

            def recvfrom(self, _size):
                raise socket.timeout()

        class ExitedProcess:
            def poll(self):
                return 23

        with tempfile.TemporaryDirectory() as tmpdir:
            log = Path(tmpdir) / "rb_servo_server.log"
            log.write_text("line one\nfatal startup\n", encoding="utf-8")

            with self.assertRaises(parity.ParityRunFailure) as ctx:
                parity.wait_for_initial_cpp_state(
                    TimeoutSocket(),
                    0.1,
                    ExitedProcess(),
                    log,
                    "udp://127.0.0.1:50171",
                )

            self.assertEqual(ctx.exception.result, "failed_server_exit")
            self.assertIn("fatal startup", "\n".join(ctx.exception.details["server_log_tail"]))

    def test_no_state_packets_is_transport_failure(self):
        class TimeoutSocket:
            def settimeout(self, _timeout):
                pass

            def recvfrom(self, _size):
                raise socket.timeout()

        class RunningProcess:
            def poll(self):
                return None

        with tempfile.TemporaryDirectory() as tmpdir:
            log = Path(tmpdir) / "rb_servo_server.log"
            log.write_text("", encoding="utf-8")

            with self.assertRaises(parity.ParityRunFailure) as ctx:
                parity.wait_for_initial_cpp_state(
                    TimeoutSocket(),
                    0.0,
                    RunningProcess(),
                    log,
                    "udp://127.0.0.1:50171",
                )

            self.assertEqual(ctx.exception.result, "failed_transport")

    def test_missing_cpp_q_ref_fails_with_caveat(self):
        sample = cpp_sample()
        sample.pop("q_ref_deg")
        sample["q_ref_published"] = False

        summary, _ = parity.compare_samples([python_sample()], [sample])

        self.assertEqual(summary["result"], "failed_parity_mismatch")
        self.assertIn("q_ref_not_published", summary["reason"])
        self.assertIn("q_ref_not_published", summary["caveats"])

    def test_fault_latched_state_is_parity_suspect_not_transport_failure(self):
        sample = cpp_sample()
        sample["fault_latched"] = True

        summary, _ = parity.compare_samples([python_sample()], [sample])

        self.assertEqual(summary["result"], "passed")
        self.assertIn("parity_suspect", summary["caveats"])


if __name__ == "__main__":
    unittest.main()
