import csv
import json
import tempfile
import unittest
from pathlib import Path

import timestamp_alignment_audit as audit


BASE_NS = 1_000_000_000
STEP_NS = 10_000_000


def write_json(path: Path, value):
    path.write_text(json.dumps(value, sort_keys=True) + "\n", encoding="utf-8")


def write_jsonl(path: Path, rows):
    path.write_text("".join(json.dumps(row, sort_keys=True) + "\n" for row in rows), encoding="utf-8")


def command_row(host_time_ns: int):
    return {
        "host_time_ns": host_time_ns,
        "left": {"mode": "TcpTwistStand"},
        "right": {"mode": "Hold"},
    }


def state_row(host_time_ns: int, *, ack_wait_us: float = 1000.0):
    return {
        "host_time_ns": host_time_ns,
        "left": {
            "state_age_us": 750.0,
            "last_send": {
                "duration_us": 500.0,
                "ack_wait_duration_us": ack_wait_us,
                "controller_acceptance_observed": True,
                "ack_observed": True,
            },
        },
    }


def write_samples(path: Path, samples):
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["host_time_ns", "position_error_m"])
        writer.writeheader()
        for host_time_ns, error_m in samples:
            writer.writerow({"host_time_ns": host_time_ns, "position_error_m": error_m})


def make_artifact(
    *,
    command_times=None,
    state_rows_override=None,
    samples=None,
    overlay_rows=None,
) -> Path:
    artifact_dir = Path(tempfile.mkdtemp())
    write_json(
        artifact_dir / "summary.json",
        {
            "arm": "left",
            "command_rate_hz": 100.0,
            "state_pub_rate_hz": 100.0,
            "feedback_use_current_state_time": False,
        },
    )
    if command_times is None:
        command_times = [BASE_NS + i * STEP_NS for i in range(20)]
    write_jsonl(artifact_dir / "command_packets.jsonl", [command_row(t) for t in command_times])
    if state_rows_override is None:
        state_rows_override = [state_row(BASE_NS + i * STEP_NS) for i in range(20)]
    write_jsonl(artifact_dir / "state_stream.jsonl", state_rows_override)
    if samples is None:
        samples = [(BASE_NS + i * STEP_NS, 0.001) for i in range(20)]
    write_samples(artifact_dir / "samples.csv", samples)
    if overlay_rows is not None:
        write_jsonl(artifact_dir / "overlay_stream.jsonl", overlay_rows)
    return artifact_dir


class TimestampAlignmentAuditTests(unittest.TestCase):
    def test_clean_timing_classified_clean(self):
        artifact_dir = make_artifact()

        result = audit.audit_artifact_dir(artifact_dir)

        self.assertEqual(result["classification"]["timing_classification"], "clean_timing")
        self.assertEqual(result["command_generation"]["command_gap_count"], 0)
        self.assertEqual(result["state_stream"]["state_drop_or_gap_count"], 0)
        self.assertEqual(result["server_send_ack"]["ack_spike_count_10ms"], 0)

    def test_ack_40ms_spike_classified_ack_spike_limited(self):
        states = [state_row(BASE_NS + i * STEP_NS, ack_wait_us=1000.0) for i in range(20)]
        states[10] = state_row(BASE_NS + 10 * STEP_NS, ack_wait_us=40_000.0)
        artifact_dir = make_artifact(state_rows_override=states)

        result = audit.audit_artifact_dir(artifact_dir)

        self.assertEqual(result["classification"]["timing_classification"], "ack_spike_limited")
        self.assertEqual(result["server_send_ack"]["ack_spike_count_40ms"], 1)

    def test_command_interval_560ms_classified_command_generation_limited(self):
        command_times = [BASE_NS, BASE_NS + STEP_NS, BASE_NS + STEP_NS + 560_000_000]
        artifact_dir = make_artifact(command_times=command_times)

        result = audit.audit_artifact_dir(artifact_dir)

        self.assertEqual(result["classification"]["timing_classification"], "command_generation_limited")
        self.assertEqual(result["command_generation"]["command_gap_count"], 1)

    def test_error_correlation_windows_work(self):
        states = [state_row(BASE_NS + i * STEP_NS, ack_wait_us=1000.0) for i in range(30)]
        states[10] = state_row(BASE_NS + 10 * STEP_NS, ack_wait_us=25_000.0)
        samples = [
            (BASE_NS + i * STEP_NS, 0.04 if 8 <= i <= 12 else 0.002)
            for i in range(30)
        ]
        artifact_dir = make_artifact(state_rows_override=states, samples=samples)

        result = audit.audit_artifact_dir(artifact_dir, spike_window_ms=25.0)
        correlation = result["correlation"]

        self.assertGreater(
            correlation["p95_error_near_ack_spike_m"],
            correlation["p95_error_away_from_ack_spike_m"],
        )
        self.assertGreater(correlation["error_sample_count_near_ack_spike"], 0)

    def test_missing_overlay_handled(self):
        artifact_dir = make_artifact()

        result = audit.audit_artifact_dir(
            artifact_dir,
            output_md_path=artifact_dir / "alignment_report.md",
            output_json_path=artifact_dir / "alignment_summary.json",
        )

        self.assertFalse(result["overlay"]["available"])
        self.assertTrue((artifact_dir / "alignment_report.md").is_file())
        self.assertTrue((artifact_dir / "alignment_summary.json").is_file())


if __name__ == "__main__":
    unittest.main()
