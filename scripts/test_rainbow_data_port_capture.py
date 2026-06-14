import json
import socket
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import rainbow_data_port_capture as capture
import rainbow_data_fixture_report as fixture_report


def config(**overrides):
    base = {
        "ips": ["127.0.0.1"],
        "port": 5001,
        "duration_sec": 0.001,
        "rate_hz": 1.0,
        "request_payload": "reqdata",
        "timeout_sec": 0.01,
        "include_hex": False,
        "max_bytes_per_sample": 1024,
        "also_rbpodo_python": False,
        "save_each_sample": False,
        "output_prefix": "samples",
        "artifact_dir": Path(tempfile.mkdtemp()),
        "confirmed_real_controller": False,
    }
    base.update(overrides)
    return capture.CaptureConfig(**base)


class FakeSocket:
    def __init__(self, chunks=None, *, timeout=False):
        self.chunks = list(chunks or [])
        self.timeout = timeout
        self.sent = []
        self.closed = False
        self.timeouts = []

    def settimeout(self, timeout_sec):
        self.timeouts.append(timeout_sec)

    def sendall(self, payload):
        self.sent.append(payload)

    def recv(self, _size):
        if self.timeout:
            raise socket.timeout("timed out")
        if self.chunks:
            return self.chunks.pop(0)
        return b""

    def close(self):
        self.closed = True


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


class RainbowDataPortCaptureTests(unittest.TestCase):
    def test_fake_data_server_bytes_are_stored_compact_by_default(self):
        fake_socket = FakeSocket([b"\x01RB-DATA\n"])
        addresses = []

        def connect(address, _timeout_sec):
            addresses.append(address)
            return fake_socket

        artifact_dir = Path(tempfile.mkdtemp())
        summary = capture.run_capture(config(artifact_dir=artifact_dir), connect_fn=connect)

        self.assertEqual(summary["result"], "completed")
        self.assertEqual(summary["success_count"], 1)
        self.assertEqual(addresses, [("127.0.0.1", 5001)])
        self.assertEqual(fake_socket.sent, [b"reqdata\n"])
        self.assertTrue(fake_socket.closed)
        self.assertEqual(summary["binary_storage_mode"], "compact_first_last")
        self.assertFalse(summary["save_each_sample"])
        self.assertEqual((artifact_dir / "first_payload.bin").read_bytes(), b"\x01RB-DATA\n")
        self.assertFalse((artifact_dir / "raw_response.bin").exists())
        sample_path = artifact_dir / "samples_127_0_0_1_000000.bin"
        self.assertFalse(sample_path.exists())
        samples = [
            json.loads(line)
            for line in (artifact_dir / "samples.jsonl").read_text(encoding="utf-8").splitlines()
        ]
        self.assertEqual(samples[0]["binary_path"], "first_payload.bin")
        self.assertEqual(samples[0]["payload_fixture_role"], "first,last")
        self.assertEqual(samples[0]["bytes_len"], len(b"\x01RB-DATA\n"))
        self.assertEqual(samples[0]["payload_len"], len(b"\x01RB-DATA\n"))
        self.assertEqual(samples[0]["response_prefix_hex"], b"\x01RB-DATA\n".hex())
        self.assertEqual(samples[0]["payload_prefix_hex"], b"\x01RB-DATA\n".hex())
        self.assertEqual(samples[0]["payload_suffix_hex"], b"\x01RB-DATA\n".hex())
        self.assertIn("RB-DATA", samples[0]["printable_ascii_prefix"])

    def test_save_each_sample_writes_per_sample_binary(self):
        fake_socket = FakeSocket([b"\x01RB-DATA\n"])
        artifact_dir = Path(tempfile.mkdtemp())

        summary = capture.run_capture(
            config(artifact_dir=artifact_dir, save_each_sample=True),
            connect_fn=lambda _address, _timeout_sec: fake_socket,
        )

        self.assertEqual(summary["binary_storage_mode"], "per_sample")
        sample_path = artifact_dir / "samples_127_0_0_1_000000.bin"
        self.assertEqual(sample_path.read_bytes(), b"\x01RB-DATA\n")
        samples = [
            json.loads(line)
            for line in (artifact_dir / "samples.jsonl").read_text(encoding="utf-8").splitlines()
        ]
        self.assertEqual(samples[0]["binary_path"], "samples_127_0_0_1_000000.bin")

    def test_timeout_is_recorded_without_crash(self):
        fake_socket = FakeSocket(timeout=True)

        summary = capture.run_capture(
            config(artifact_dir=Path(tempfile.mkdtemp())),
            connect_fn=lambda _address, _timeout_sec: fake_socket,
        )

        self.assertEqual(summary["result"], "unsupported_or_timeout")
        self.assertEqual(summary["timeout_count"], 1)
        self.assertEqual(summary["success_count"], 0)

    def test_confirmation_required_for_known_real_controller_ip(self):
        with self.assertRaisesRegex(capture.CaptureError, "known real controller IP"):
            capture.validate_config(config(ips=["172.28.60.200"]))

    def test_real_controller_ip_requires_artifacts_dir_after_confirmation(self):
        cfg = config(ips=["172.28.60.200"], confirmed_real_controller=True)
        with self.assertRaisesRegex(capture.CaptureError, "artifacts/"):
            capture.validate_config(cfg)

    def test_non_data_port_is_rejected(self):
        with self.assertRaisesRegex(capture.CaptureError, "--port must be 5001"):
            capture.validate_config(config(port=5000))

    def test_also_rbpodo_python_can_be_mocked(self):
        artifact_dir = Path(tempfile.mkdtemp())
        summary = capture.run_capture(
            config(artifact_dir=artifact_dir, also_rbpodo_python=True),
            connect_fn=lambda _address, _timeout_sec: FakeSocket([b"raw"]),
            read_python_sdata=lambda _ip, _timeout_sec: FakeSData(),
        )

        self.assertEqual(summary["rbpodo_python_sample_count"], 1)
        self.assertEqual(summary["rbpodo_python_diagnostics_suspect_rate"], 0.0)
        rows = [
            json.loads(line)
            for line in (artifact_dir / "python_decoded_samples.jsonl").read_text(encoding="utf-8").splitlines()
        ]
        self.assertTrue(rows[0]["ok"])
        self.assertEqual(rows[0]["q_ref_source"], "python_rbpodo.sdata.jnt_ref")
        self.assertEqual(rows[0]["q_ref_deg"], FakeSData.jnt_ref)
        self.assertIsNone(rows[0]["q_ref_delta_norm_deg"])
        self.assertEqual(rows[0]["raw"]["op_stat_self_collision"], 0)

    def test_only_data_port_connection_is_attempted(self):
        addresses = []

        def connect(address, _timeout_sec):
            addresses.append(address)
            return FakeSocket([b"ok"])

        capture.run_capture(config(artifact_dir=Path(tempfile.mkdtemp())), connect_fn=connect)

        self.assertEqual(addresses, [("127.0.0.1", 5001)])
        self.assertNotIn(("127.0.0.1", 5000), addresses)

    def test_fixture_summary_detects_payload_change_with_q_ref_change(self):
        samples = [
            {"ok": True, "sample_index": 0, "ip": "127.0.0.1", "bytes_len": 2, "response_sha256": "hash-a"},
            {"ok": True, "sample_index": 1, "ip": "127.0.0.1", "bytes_len": 2, "response_sha256": "hash-b"},
        ]
        python_samples = [
            {"ok": True, "sample_index": 0, "ip": "127.0.0.1", "q_ref_deg": [0, 0, 0, 0, 0, 0]},
            {"ok": True, "sample_index": 1, "ip": "127.0.0.1", "q_ref_deg": [1, 0, 0, 0, 0, 0]},
        ]

        summary = capture.payload_pattern_summary(samples, [b"aa", b"bb"], python_samples)

        self.assertTrue(summary["q_ref_change_observed"])
        self.assertTrue(summary["payload_changes_when_q_ref_changes"])
        self.assertEqual(summary["q_ref_payload_pair_count"], 2)
        self.assertEqual(summary["q_ref_payload_transition_count"], 1)
        self.assertEqual(summary["q_ref_changed_payload_changed_count"], 1)
        self.assertEqual(summary["q_ref_changed_payload_static_count"], 0)

    def test_fake_payload_changes_produce_offset_histogram(self):
        samples = [
            {"ok": True, "sample_index": 0, "ip": "127.0.0.1", "bytes_len": 3, "response_sha256": "hash-a"},
            {"ok": True, "sample_index": 1, "ip": "127.0.0.1", "bytes_len": 3, "response_sha256": "hash-b"},
        ]

        summary = capture.payload_pattern_summary(samples, [b"abc", b"axd"], [])

        self.assertEqual(samples[1]["changed_byte_count"], 2)
        self.assertEqual(samples[1]["first_changed_offset"], 1)
        self.assertEqual(summary["changed_offsets_histogram"], {"1": 1, "2": 1})
        self.assertEqual(summary["changed_offsets_top"][0], {"offset": 1, "count": 1})

    def test_q_ref_change_correlation_is_written_for_python_samples(self):
        payloads = [b"abc", b"axc"]
        q_refs = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]

        def connect(_address, _timeout_sec):
            return FakeSocket([payloads.pop(0)])

        def read_python_sdata(_ip, _timeout_sec):
            sdata = FakeSData()
            sdata.jnt_ref = q_refs.pop(0)
            sdata.jnt_ang = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            return sdata

        artifact_dir = Path(tempfile.mkdtemp())
        summary = capture.run_capture(
            config(
                artifact_dir=artifact_dir,
                also_rbpodo_python=True,
                duration_sec=2.0,
                rate_hz=1.0,
            ),
            connect_fn=connect,
            read_python_sdata=read_python_sdata,
        )

        self.assertEqual(summary["q_ref_changed_payload_changed_count"], 1)
        rows = [
            json.loads(line)
            for line in (artifact_dir / "python_decoded_samples.jsonl").read_text(encoding="utf-8").splitlines()
        ]
        self.assertIsNone(rows[0]["q_ref_delta_norm_deg"])
        self.assertEqual(rows[1]["q_ref_delta_norm_deg"], 1.0)
        self.assertTrue(rows[1]["payload_changed"])

    def test_output_prefix_cannot_write_outside_artifact_dir(self):
        with tempfile.TemporaryDirectory() as root:
            root_path = Path(root)
            artifact_dir = root_path / "artifacts" / "raw"
            capture.run_capture(
                config(
                    artifact_dir=artifact_dir,
                    output_prefix="../escape",
                    save_each_sample=True,
                ),
                connect_fn=lambda _address, _timeout_sec: FakeSocket([b"raw"]),
            )

            self.assertFalse((artifact_dir.parent / "escape_127_0_0_1_000000.bin").exists())
            samples = [
                json.loads(line)
                for line in (artifact_dir / "samples.jsonl").read_text(encoding="utf-8").splitlines()
            ]
            binary_path = artifact_dir / samples[0]["binary_path"]
            self.assertTrue(binary_path.exists())
            self.assertIn(artifact_dir.resolve(), binary_path.resolve().parents)

    def test_fixture_report_cli_writes_report_inside_capture_dir(self):
        payloads = [b"abc", b"axc"]

        def connect(_address, _timeout_sec):
            return FakeSocket([payloads.pop(0)])

        artifact_dir = Path(tempfile.mkdtemp())
        capture.run_capture(
            config(artifact_dir=artifact_dir, duration_sec=2.0, rate_hz=1.0),
            connect_fn=connect,
        )

        script = Path(__file__).with_name("rainbow_data_fixture_report.py")
        completed = subprocess.run(
            [
                sys.executable,
                str(script),
                "--capture-dir",
                str(artifact_dir),
                "--output-md",
                "fixture_report.md",
                "--output-json",
                "fixture_report.json",
            ],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        report = json.loads((artifact_dir / "fixture_report.json").read_text(encoding="utf-8"))
        self.assertEqual(report["unique_payload_lengths"], [3])
        self.assertEqual(report["q_ref_changed_payload_static_count"], 0)
        self.assertIn("do not infer layout yet", report["recommended_next_steps"])
        self.assertTrue((artifact_dir / "fixture_report.md").exists())

    def test_fixture_report_rejects_output_outside_capture_dir(self):
        with tempfile.TemporaryDirectory() as root:
            capture_dir = Path(root) / "capture"
            capture_dir.mkdir()
            with self.assertRaisesRegex(fixture_report.FixtureReportError, "outside capture dir"):
                fixture_report.resolve_output_path(capture_dir, Path(root) / "outside.json")

    def test_help_works(self):
        scripts_dir = Path(__file__).parent
        for script_name in ("rainbow_data_port_capture.py", "rainbow_data_fixture_report.py"):
            completed = subprocess.run(
                [sys.executable, str(scripts_dir / script_name), "--help"],
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn("usage:", completed.stdout)


if __name__ == "__main__":
    unittest.main()
