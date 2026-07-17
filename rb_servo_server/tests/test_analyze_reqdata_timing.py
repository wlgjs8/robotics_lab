import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import analyze_reqdata_timing as timing


class AnalyzeRequestDataTimingTest(unittest.TestCase):
    def test_parse_and_correlate_complete_slow_exchange(self):
        lines = [
            "1.000050000\t10.0.0.1\t41000\t172.28.60.200\t5001\t72:65:71:64:61:74:61",
            "1.010000000\t172.28.60.200\t5001\t10.0.0.1\t41000\t24:10:00:03:00:00",
        ]
        events = timing.parse_tshark_lines(
            lines, {"left": "172.28.60.200", "right": "172.28.60.201"}
        )
        call = timing.RequestDataCall(
            tick=17,
            arm="left",
            exchange_sequence=9,
            source="rbpodo_sdk_request_data",
            call_start_steady_ns=2_000_000_000,
            call_start_system_ns=1_000_000_000,
            call_return_steady_ns=2_010_100_000,
            call_return_system_ns=1_010_100_000,
            call_duration_us=10_100.0,
            backend_read_total_duration_us=10_130.0,
        )

        rows, consumed = timing.correlate_events(
            [call], events, tolerance_us=1_000.0, slow_threshold_us=5_000.0
        )

        self.assertEqual(len(rows), 1)
        self.assertEqual(consumed, {0, 1})
        row = rows[0]
        self.assertEqual(row["classification"], "complete")
        self.assertTrue(row["slow"])
        self.assertEqual(row["call_start_to_reqdata_tx_us"], 50.0)
        self.assertEqual(row["reqdata_tx_to_response_first_rx_us"], 9_950.0)
        self.assertEqual(row["response_first_rx_to_request_data_return_us"], 100.0)
        self.assertEqual(row["system_clock_call_boundary_duration_us"], 10_100.0)
        self.assertEqual(row["system_minus_steady_call_duration_us"], 0.0)
        self.assertEqual(row["backend_read_outside_request_data_us"], 30.0)
        self.assertEqual(row["dominant_phase"], "reqdata_tx_to_response_first_rx")

    def test_missing_response_is_explicit(self):
        events = [timing.PacketEvent(1_000_010_000, "right", "reqdata_tx", 7)]
        call = timing.RequestDataCall(
            tick=18,
            arm="right",
            exchange_sequence=10,
            source="rbpodo_sdk_request_data",
            call_start_steady_ns=3_000_000_000,
            call_start_system_ns=1_000_000_000,
            call_return_steady_ns=3_005_000_000,
            call_return_system_ns=1_005_000_000,
            call_duration_us=5_000.0,
            backend_read_total_duration_us=5_020.0,
        )

        rows, consumed = timing.correlate_events(
            [call], events, tolerance_us=100.0, slow_threshold_us=5_000.0
        )

        self.assertEqual(consumed, {0})
        self.assertEqual(rows[0]["classification"], "missing_response_first_rx_packet")
        self.assertIsNone(rows[0]["controller_response_first_rx_packet_system_ns"])

    def test_non_state_and_unrelated_payloads_are_ignored(self):
        lines = [
            "1.0\t172.28.60.200\t5001\t10.0.0.1\t41000\t24:10:00:02:00",
            "1.1\t10.0.0.1\t41000\t172.28.60.200\t5001\t6f:74:68:65:72",
        ]
        events = timing.parse_tshark_lines(
            lines, {"left": "172.28.60.200", "right": "172.28.60.201"}
        )
        self.assertEqual(events, [])

    def test_epoch_conversion_preserves_nanoseconds(self):
        self.assertEqual(timing.epoch_seconds_to_ns("1.000000123"), 1_000_000_123)


if __name__ == "__main__":
    unittest.main()
