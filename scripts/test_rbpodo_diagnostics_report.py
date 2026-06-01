import json
import tempfile
import unittest
from pathlib import Path

import generate_rbpodo_diagnostics_report as report


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


def state_dump(raw=None, *, suspect=False, reasons=None, clear_flags=None):
    raw = dict(RAW_BASE if raw is None else raw)
    return {
        "schema": "robotics_lab.rbpodo_state_dump.v1",
        "read_only": True,
        "sdk_info": {
            "module_available": True,
            "module_file": "/tmp/rbpodo.py",
            "module_version": "test",
            "cobot_data_available": True,
        },
        "ips": ["172.28.60.200"],
        "results": [
            {
                "ip": "172.28.60.200",
                "ok": True,
                "raw": raw,
                "controller_mode": "simulation",
                "controller_mode_is_simulation": True,
                "diagnostics_suspect": suspect,
                "diagnostics_suspect_reasons": list(reasons or []),
                "clear_error_flags": list(clear_flags or []),
            }
        ],
    }


def parity_summary(result="suspect_but_consistent", *, raw_match_rate=1.0, agreement_rate=1.0):
    return {
        "schema": "robotics_lab.rbpodo_state_parity.summary.v1",
        "read_only": True,
        "result": result,
        "reason": "fixture parity result",
        "caveats": ["diagnostics_suspect_unresolved"] if result == "suspect_but_consistent" else [],
        "metrics": {
            "raw_field_match_rate": raw_match_rate,
            "diagnostics_suspect_agreement_rate": agreement_rate,
        },
        "cpp_backend_hints": {
            "sample_count": 2,
            "rbpodo_sdk_state_sources": ["CobotData.request_data"],
            "rbpodo_state_decode_policies": ["strict_boolean_flags_with_suspect_large_values"],
            "q_ref_sources": ["rbpodo.sdata.jnt_ref"],
        },
    }


def raw_capture_summary():
    return {
        "schema": "robotics_lab.rainbow_data_port_capture.summary.v1",
        "read_only": True,
        "result": "completed",
        "reason": "captured one or more raw data-port responses",
        "ips": ["172.28.60.200"],
        "sample_count": 2,
        "success_count": 2,
        "unique_payload_lengths": [1024],
        "unique_hash_count": 1,
        "stable_prefix_bytes_len": 64,
        "stable_prefix_hex": "ab",
        "fixture_comparison": {
            "payload_change_observed": False,
            "q_ref_unique_count": 1,
            "q_ref_change_observed": False,
            "q_ref_payload_pair_count": 2,
            "payload_changes_when_q_ref_changes": None,
        },
    }


class RbpodoDiagnosticsReportTest(unittest.TestCase):
    def test_huge_self_collision_with_python_cpp_agreement_is_not_healthy(self):
        raw = dict(RAW_BASE)
        raw["op_stat_self_collision"] = 1977953904

        built = report.build_report(
            state_dump(
                raw,
                suspect=True,
                reasons=["op_stat_self_collision expected 0/1, got 1977953904"],
            ),
            parity_summary("suspect_but_consistent"),
            raw_capture_summary(),
        )

        self.assertIn(
            built["likely_root_cause"],
            {"sdk_firmware_layout_mismatch", "field_semantics_unknown"},
        )
        self.assertNotEqual(built["likely_root_cause"], "healthy")
        self.assertIn("diagnostics_suspect_unresolved", built["physical_real_blockers"])

    def test_python_cpp_mismatch_classifies_decode_mismatch(self):
        built = report.build_report(
            state_dump(suspect=True, reasons=["diagnostics_suspect fixture"]),
            parity_summary("failed_parity_mismatch", raw_match_rate=0.8),
            raw_capture_summary(),
        )

        self.assertEqual(built["likely_root_cause"], "python_cpp_decode_mismatch")

    def test_clear_boolean_collision_flag_classifies_real_fault(self):
        raw = dict(RAW_BASE)
        raw["op_stat_collision_occur"] = 1

        built = report.build_report(
            state_dump(raw, clear_flags=["op_stat_collision_occur"]),
            parity_summary("passed"),
            raw_capture_summary(),
        )

        self.assertEqual(built["likely_root_cause"], "controller_reports_real_fault")
        self.assertEqual(built["clear_controller_faults"][0]["field"], "op_stat_collision_occur")

    def test_missing_raw_capture_is_insufficient_evidence(self):
        raw = dict(RAW_BASE)
        raw["op_stat_self_collision"] = 1977953904

        built = report.build_report(
            state_dump(raw, suspect=True, reasons=["op_stat_self_collision expected 0/1, got 1977953904"]),
            parity_summary("suspect_but_consistent"),
            None,
        )

        self.assertEqual(built["likely_root_cause"], "insufficient_evidence")

    def test_cli_artifacts_include_classification(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = Path(tmpdir)
            state_path = tmp / "state_dump.json"
            parity_path = tmp / "parity_summary.json"
            raw_path = tmp / "raw_summary.json"
            md_path = tmp / "diagnostics_report.md"
            json_path = tmp / "diagnostics_report.json"
            state_path.write_text(json.dumps(state_dump()), encoding="utf-8")
            parity_path.write_text(json.dumps(parity_summary("passed")), encoding="utf-8")
            raw_path.write_text(json.dumps(raw_capture_summary()), encoding="utf-8")

            loaded_state, state_status = report.load_json_artifact(state_path, "state_dump")
            loaded_parity, parity_status = report.load_json_artifact(parity_path, "parity_summary")
            loaded_raw, raw_status = report.load_json_artifact(raw_path, "raw_capture")
            built = report.build_report(
                loaded_state,
                loaded_parity,
                loaded_raw,
                {
                    "state_dump": state_status,
                    "parity_summary": parity_status,
                    "raw_capture": raw_status,
                },
            )
            report.write_outputs(built, md_path, json_path)

            written = json.loads(json_path.read_text(encoding="utf-8"))
            self.assertEqual(written["schema"], report.SCHEMA)
            self.assertTrue(md_path.read_text(encoding="utf-8").startswith("# rbpodo Diagnostics"))


if __name__ == "__main__":
    unittest.main()
