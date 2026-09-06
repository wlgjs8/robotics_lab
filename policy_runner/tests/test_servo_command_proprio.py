"""Hardware/network-free command-history and OpenPI observation regressions."""
from __future__ import annotations

import copy
import json
import unittest
from unittest.mock import Mock, patch

import numpy as np
from scipy.spatial.transform import Rotation

from policy_runner.robot_state_client import RobotStateClient, StateSnapshot
from policy_runner.servo_command_history import ServoCommandHistory
from policy_runner.openpi_remote import OpenpiRemoteActionSource


DT = 0.0334
T0 = 10_000_000_000


def payload(ms: float, *, tick: int | None = None, motion: int = 1,
            left_epoch: int = 0, right_epoch: int = 0, x: float | None = None):
    stamp = T0 + round(ms * 1e6)
    command = {"x": ms * 0.0001 if x is None else x, "y": 0.0, "z": 0.0,
               "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]}
    measured = {"x": 8.0, "y": 9.0, "z": 7.0, "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]}
    return {
        "loop_start_time_ns": stamp, "host_time_ns": stamp + 150_000,
        "tick": 100 + round(ms) if tick is None else tick, "motion_epoch": motion,
        "fault_latched": False,
        **{side: {"tcp_command_stand": copy.deepcopy(command),
                  "tcp_actual_stand": copy.deepcopy(measured),
                  "force_control": {"reference_reset_count": epoch},
                  "gripper": {"valid": True, "stale": False, "percent": 25.0}}
           for side, epoch in [("left", left_epoch), ("right", right_epoch)]},
    }


def record(history, p, *, local=True):
    return history.record(p, p["loop_start_time_ns"] * 1e-9 + 0.001, same_host_clock=local)


def populated():
    h = ServoCommandHistory(stale_timeout_sec=0.5)
    for ms in [0, 10, 20, 30, 40, 50]:
        record(h, payload(ms))
    return h


class ServoCommandHistoryTests(unittest.TestCase):
    def sample(self, history, p, **kwargs):
        return history.body_deltas(p, policy_dt_sec=DT,
                                   now_monotonic=p["loop_start_time_ns"] * 1e-9 + 0.002,
                                   **kwargs)

    def test_exact_fixed_window_interpolates_with_training_delta_units(self):
        h = populated()
        velocities, diag = self.sample(h, payload(50))
        self.assertTrue(diag["valid"])
        np.testing.assert_allclose(velocities["left"], [0.00334, 0, 0, 0, 0, 0], atol=1e-12)
        self.assertEqual(diag["window_start_time_ns"], T0 + 16_600_000)
        self.assertEqual(diag["window_end_time_ns"], T0 + 50_000_000)
        self.assertEqual(diag["arms"]["left"]["start_bracket_time_ns"], [T0+10_000_000, T0+20_000_000])
        self.assertEqual(diag["scale"], 1.0)  # Body delta, not 0.1 m/s.

    def test_delayed_worker_does_not_read_future_command_history(self):
        h = populated()
        frozen = payload(50)
        before, _ = self.sample(h, frozen)
        for ms in [60, 70, 80]:
            record(h, payload(ms, x=2.0))  # Enormous later change must not leak into this observation.
        after, diag = self.sample(h, frozen)
        np.testing.assert_array_equal(after["left"], before["left"])
        self.assertGreater(diag["history_latest_time_ns"], diag["window_end_time_ns"])
        self.assertLessEqual(max(diag["arms"]["left"]["end_bracket_time_ns"]), frozen["loop_start_time_ns"])

    def test_translation_body_frame_and_rotation_slerp(self):
        h = ServoCommandHistory(stale_timeout_sec=0.5)
        for ms in [0, 10, 20, 30, 40, 50]:
            p = payload(ms)
            # Rotation at t=16.6ms is exactly 90 degrees; translation runs along stand +Y.
            for side in ("left", "right"):
                p[side]["tcp_command_stand"] = {
                    "x": 0.0, "y": ms * 0.0001, "z": 0.0,
                    "quaternion_xyzw": Rotation.from_euler("z", np.pi/2 + (ms-16.6)*0.001).as_quat().tolist(),
                }
            record(h, p)
        velocity, diag = self.sample(h, p)
        self.assertTrue(diag["valid"])
        np.testing.assert_allclose(velocity["left"], [0.00334, 0, 0, 0, 0, 0.0334], atol=1e-12)

    def test_partial_init_epoch_cannot_bridge_but_peer_is_preserved(self):
        h = populated()
        p = payload(60, left_epoch=1, x=1.0)
        record(h, p)
        v, d = self.sample(h, p)
        self.assertFalse(d["valid"])
        self.assertFalse(d["arms"]["left"]["valid"])
        self.assertTrue(d["arms"]["right"]["valid"])
        np.testing.assert_array_equal(v["left"], np.zeros(6))
        for ms in [70, 80, 90, 100]:
            p = payload(ms, left_epoch=1, x=1.0)
            record(h, p)
        v, d = self.sample(h, p)
        self.assertTrue(d["valid"])
        np.testing.assert_array_equal(v["left"], np.zeros(6))

    def test_motion_epoch_and_restart_invalidate_old_windows(self):
        for changes in [{"motion": 2}, {"tick": 1}]:
            with self.subTest(changes=changes):
                h = populated()
                old = payload(50)
                new = payload(60, **changes)
                record(h, new)
                self.assertFalse(self.sample(h, new)[1]["valid"])
                self.assertFalse(self.sample(h, old)[1]["valid"])

    def test_delayed_udp_does_not_roll_epoch_back_or_manufacture_history(self):
        h = populated()
        self.assertFalse(record(h, payload(40, motion=0, x=8.0)))
        self.assertFalse(record(h, payload(50, x=9.0)))
        v, d = self.sample(h, payload(50))
        self.assertTrue(d["valid"])
        np.testing.assert_allclose(v["left"][0], 0.00334)

    def test_very_late_old_packet_does_not_clear_newer_fresh_history(self):
        h = ServoCommandHistory(stale_timeout_sec=0.5)
        for ms in [550, 560, 570, 580, 590, 600]:
            record(h, payload(ms))
        self.assertFalse(h.record(payload(0), 10.601, same_host_clock=True))
        self.assertTrue(self.sample(h, payload(600))[1]["valid"])

    def test_missing_epoch_timestamp_stale_and_remote_are_explicitly_invalid(self):
        for mutate, reason in [
            (lambda p: p.pop("motion_epoch"), "motion_epoch"),
            (lambda p: p.pop("loop_start_time_ns"), "loop_start_time_ns"),
            (lambda p: p["left"]["force_control"].clear(), "reference_reset_count"),
        ]:
            with self.subTest(reason=reason):
                h = populated(); p = payload(60); mutate(p)
                v, d = self.sample(h, p) if "loop_start_time_ns" in p else h.body_deltas(
                    p, policy_dt_sec=DT, now_monotonic=10.06)
                self.assertFalse(d["valid"])
                self.assertIn(reason, str(d))
        h = populated(); p = payload(50)
        _, d = h.body_deltas(p, policy_dt_sec=DT, now_monotonic=10.7)
        self.assertEqual(d["zero_reason"], "stale_observation")
        record(h, payload(60), local=False)
        _, d = self.sample(h, payload(60))
        self.assertIn("unsupported_remote_clock_domain", str(d))

    def test_sparse_bracket_and_policy_reset_are_rejected(self):
        h = ServoCommandHistory(stale_timeout_sec=0.5)
        record(h, payload(0)); record(h, payload(100))
        self.assertIn("sparse_history_bracket", str(self.sample(h, payload(100))[1]))
        _, d = self.sample(populated(), payload(50), not_before_ns=T0+30_000_000)
        self.assertEqual(d["zero_reason"], "window_crosses_policy_reset")

    def test_fault_and_nonfinite_pose_never_fall_back_to_measured(self):
        h = populated(); p = payload(60)
        p["left"]["tcp_command_stand"]["x"] = float("nan")
        record(h, p)
        v, d = self.sample(h, p)
        self.assertFalse(d["valid"])
        np.testing.assert_array_equal(v["left"], np.zeros(6))
        p = payload(70); p["fault_latched"] = True
        record(h, p)
        self.assertEqual(self.sample(h, p)[1]["zero_reason"], "server_fault_latched")


class ServoCommandSourceTests(unittest.TestCase):
    def source(self, history=None, mode="velocity_grip"):
        src = OpenpiRemoteActionSource.__new__(OpenpiRemoteActionSource)
        src.proprio_mode = mode
        src.velproprio_source = "servo_command"
        src.velproprio_sample_mode = "fixed_step"
        src.policy_dt_sec = DT
        src.ee_local_r_align = None
        src._servo_command_history = history
        src._servo_command_not_before_ns = 0
        src._live_gripper_percent = lambda side: None
        src._last_obs_camera_time_sec = 10.04
        return src

    def test_layout_source_and_camera_skew_are_explicit(self):
        src = self.source(populated())
        with patch("policy_runner.openpi_remote.time.monotonic", return_value=10.052):
            state = src._proprio_state(src._freeze_inference_payload(payload(50)))
        self.assertEqual(state.shape, (14,))
        np.testing.assert_allclose(state[[0, 7]], 0.00334)
        np.testing.assert_allclose(state[[6, 13]], 0.25)
        self.assertAlmostEqual(src._last_velproprio_diagnostics["camera_minus_state_ms"], -10.0)
        self.assertIn("not_controller_ack", src._last_velproprio_diagnostics["command_semantics"])

    def test_reset_rtc_installs_cutoff_without_erasing_shared_collector(self):
        h = populated(); src = self.source(h)
        src.reset_rtc()
        self.assertEqual(src._servo_command_not_before_ns, T0+50_000_000)
        self.assertEqual(h.latest_time_ns, T0+50_000_000)
        with patch("policy_runner.openpi_remote.time.monotonic", return_value=10.052):
            src._proprio_state(src._freeze_inference_payload(payload(50)))
        self.assertEqual(src._last_velproprio_diagnostics["zero_reason"], "window_crosses_policy_reset")

    def test_invalid_history_returns_before_any_model_client_call(self):
        src = self.source()
        src._client = Mock()
        src.prompt = "fixture"
        src._raw_camera_images = lambda: ({"left": np.zeros((1,1,3)), "right": np.zeros((1,1,3))}, 2, 0)
        self.assertIsNone(src._sample_chunk(payload(50)))
        self.assertEqual(src._last_inference_camera_diagnostics["outcome"], "servo_command_proprio_unavailable")
        self.assertEqual(src._client.mock_calls, [])

    def test_source_rejects_nonfixed_window_and_pose_input(self):
        for mode, sample in [("pose", "fixed_step"), ("velocity", "camera_frame"), ("velocity", "replan")]:
            with self.assertRaises(ValueError):
                OpenpiRemoteActionSource._validate_velproprio_source("servo_command", proprio_mode=mode, velproprio_sample_mode=sample)

    def test_full14d_stays_frozen_when_gripper_changes_before_worker(self):
        src = self.source(populated())
        src.gripper_proprio_source = "command"
        src._gripper_last_sent_by_arm = {"left": 50.0, "right": 50.0}
        with patch("policy_runner.openpi_remote.time.monotonic_ns", return_value=T0+52_000_000):
            frozen = src._freeze_inference_payload(payload(50))
        src._gripper_last_sent_by_arm = {"left": 7.0, "right": 7.0}
        self.assertIs(src._freeze_inference_payload(frozen), frozen)
        with patch("policy_runner.openpi_remote.time.monotonic", return_value=10.072):
            state = src._proprio_state(frozen)
        np.testing.assert_allclose(state[[0, 7]], 0.00334)
        np.testing.assert_allclose(state[[6, 13]], 0.5)
        self.assertTrue(src._last_velproprio_diagnostics["valid"])
        self.assertEqual(src._last_velproprio_diagnostics["gripper_observation"]["selection_time_ns"], T0+52_000_000)

    def test_unfrozen_gripper_is_explicitly_invalid(self):
        src = self.source(populated())
        with patch("policy_runner.openpi_remote.time.monotonic", return_value=10.052):
            src._proprio_state(payload(50))
        self.assertEqual(src._last_velproprio_diagnostics["zero_reason"], "frozen_gripper_observation_unavailable")

    def test_state_client_collects_packets_while_policy_is_not_ticking(self):
        packets = [payload(ms) for ms in [0, 10, 20, 30, 40, 50]]
        sock = Mock()
        sock.recvfrom.side_effect = [(json.dumps(p).encode(), ("127.0.0.1", 1234)) for p in packets]
        client = RobotStateClient("udp://127.0.0.1:0", socket_factory=lambda *_: sock)
        history = client.enable_servo_command_history()
        for p in packets:
            with patch("policy_runner.robot_state_client.time.monotonic", return_value=p["loop_start_time_ns"]*1e-9+.001):
                snapshot = client.poll_once()
        self.assertIs(snapshot.servo_command_history, history)
        src = self.source()
        src._before_policy_intent(snapshot, 10.052)
        with patch("policy_runner.openpi_remote.time.monotonic", return_value=10.052):
            state = src._proprio_state(src._freeze_inference_payload(snapshot.payload))
        np.testing.assert_allclose(state[0], 0.00334)
        client.close()


if __name__ == "__main__":
    unittest.main()
