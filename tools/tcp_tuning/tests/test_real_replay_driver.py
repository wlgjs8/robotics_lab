from __future__ import annotations

import io
import json
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest.mock import patch

import numpy as np
import h5py

from scripts import replay_episode_tcp_pose_target as replay
from policy_runner.flow_dataset import pose_compose_local, pose_delta_local


Q = np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64)


def pose(x: float, y: float = 0.0, z: float = 0.2, q: np.ndarray = Q) -> np.ndarray:
    return np.asarray([x, y, z, *q], dtype=np.float64)


class FakeCommandClient:
    source_id = "tcp_pose_replay"
    session_id = "session-1"


class RealReplayDriverTest(unittest.TestCase):
    def test_dry_run_npz_with_mock_current_pose_writes_plan_and_sends_nothing(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            npz = write_npz(root / "episode_test" / "clean_foh_se3_10hz.npz")
            config = write_server_config(root / "server.yaml")
            out = root / "out"

            class NoSocketClient:
                def __init__(self, *args, **kwargs):
                    raise AssertionError("dry-run must not construct ServoCommandClient")

            stdout = io.StringIO()
            with patch.object(replay, "ServoCommandClient", NoSocketClient), redirect_stdout(stdout):
                rc = replay.main(
                    [
                        "--npz",
                        str(npz),
                        "--server-config",
                        str(config),
                        "--mock-current-pose",
                        "first",
                        "--out-dir",
                        str(out),
                        "--max-linear-speed-m-s",
                        "1.0",
                        "--max-angular-speed-rad-s",
                        "1.0",
                    ]
                )

            self.assertEqual(rc, 0)
            text = stdout.getvalue()
            self.assertIn("DRY RUN — no motion sent", text)
            self.assertIn("planned init pre-move", text)
            self.assertIn("setpoint_min_xyz", text)
            logs = list((out / "episode_test" / "runs").glob("*/log.csv"))
            self.assertEqual(len(logs), 1)
            self.assertTrue(logs[0].read_text(encoding="utf-8").startswith("t,t_source,src_idx"))

    def test_init_delta_and_premove_interpolation_hit_endpoints(self) -> None:
        current = pose(0.0, 0.0, 0.2)
        target = pose(0.03, 0.04, 0.2)
        delta = replay.compute_init_delta(current, target, "left")
        self.assertAlmostEqual(delta.linear_m, 0.05)
        self.assertAlmostEqual(delta.angular_deg, 0.0)

        init = replay.make_init_premove(
            current,
            target,
            requested_sec=1.0,
            rate_hz=10.0,
            max_linear=0.2,
            max_angular=1.0,
        )
        np.testing.assert_allclose(init[0], current)
        np.testing.assert_allclose(init[-1], target)

    def test_refuses_out_of_roi_setpoints(self) -> None:
        server = replay.ServerRuntimeConfig(
            path=Path("server.yaml"),
            command_endpoint="udp://127.0.0.1:1",
            state_bind=None,
            servo_rate_hz=10.0,
            command_timeout_sec=0.2,
            smd_max_linear_velocity_m_s=0.5,
            smd_max_angular_velocity_rad_s=1.0,
            floor_z_min_m=0.0,
            roi_min_m=(-0.1, -0.1, 0.0),
            roi_max_m=(0.1, 0.1, 0.3),
            raw={},
        )
        goals = {"left": np.stack([pose(0.0), pose(0.2)], axis=0)}
        with self.assertRaisesRegex(replay.ReplayRefusal, "violates ROI"):
            replay.validate_bounds(goals, server)

    def test_watchdog_stops_on_fault_state(self) -> None:
        self.assertEqual(replay.watchdog_cause({"fault_latched": True}), "fault_latched")
        self.assertEqual(replay.watchdog_cause({"safety_verdict": "EmergencyStop"}), "safety_verdict=EmergencyStop")
        payload = {"left": {"cartesian_solve": {"ik_timed_out": True}}}
        self.assertEqual(replay.watchdog_cause(payload), "left.cartesian_solve.ik_timed_out")

    def test_controller_sim_not_activated_arm_error_is_flag_gated(self) -> None:
        payload = pgmode_not_activated_payload()

        self.assertEqual(replay.watchdog_cause(payload), "right.has_error")
        self.assertIsNone(replay.watchdog_cause(payload, allow_controller_sim_arm_error=True))

    def test_controller_sim_not_activated_arm_error_does_not_mask_ems_fault(self) -> None:
        payload = pgmode_not_activated_payload(raw_overrides={"op_stat_ems_flag": 1})

        self.assertFalse(replay.arm_error_is_controller_sim_not_activated(payload, "right"))
        self.assertEqual(
            replay.watchdog_cause(payload, allow_controller_sim_arm_error=True),
            "right.has_error",
        )

    def test_controller_sim_not_activated_arm_error_does_not_mask_unexpected_startup_reason(self) -> None:
        payload = pgmode_not_activated_payload(right_overrides={"startup_invalid_reasons": ["servo_disabled", "wrong_mode"]})

        self.assertFalse(replay.arm_error_is_controller_sim_not_activated(payload, "right"))
        self.assertEqual(
            replay.watchdog_cause(payload, allow_controller_sim_arm_error=True),
            "right.has_error",
        )

    def test_controller_sim_arm_error_tolerance_keeps_other_watchdogs_fail_closed(self) -> None:
        client = FakeCommandClient()
        self.assertEqual(
            replay.watchdog_cause({"fault_latched": True}, client, allow_controller_sim_arm_error=True),
            "fault_latched",
        )
        self.assertEqual(
            replay.watchdog_cause({"tracking_error_latched": True}, client, allow_controller_sim_arm_error=True),
            "tracking_error",
        )
        lease_lost = {
            "command_source": {
                "enforce_lease": True,
                "active_source_id": "other",
                "active_session_id": "session-1",
            }
        }
        self.assertEqual(
            replay.watchdog_cause(lease_lost, client, allow_controller_sim_arm_error=True),
            "command_source_lease_lost",
        )

    def test_controller_sim_arm_error_tolerance_is_not_real_mode(self) -> None:
        payload = pgmode_not_activated_payload(top_overrides={"operation_mode": "real", "observed_mode": "real"})

        self.assertFalse(replay.arm_error_is_controller_sim_not_activated(payload, "right"))
        self.assertEqual(
            replay.watchdog_cause(payload, allow_controller_sim_arm_error=True),
            "right.has_error",
        )

    def test_pose_delta_local_round_trip_preserves_trajectory_shape(self) -> None:
        trajectory = np.stack(
            [
                pose(0.0, 0.0, 0.2),
                pose(0.03, -0.01, 0.22),
                pose(0.08, -0.015, 0.24),
            ],
            axis=0,
        )
        recovered = [trajectory[0]]
        for i in range(trajectory.shape[0] - 1):
            recovered.append(pose_compose_local(recovered[-1], pose_delta_local(trajectory[i], trajectory[i + 1])))
        np.testing.assert_allclose(np.asarray(recovered), trajectory, atol=1e-6)

    def test_ee_local_anchored_composition_init_delta_is_zero(self) -> None:
        source = np.stack([pose(0.0), pose(0.02), pose(0.04)], axis=0)
        anchor = pose(-0.2, -0.4, 0.25)
        anchored, stats = replay.compose_ee_local_from_anchor(source, anchor, r_align=None, action_scale=1.0)
        np.testing.assert_allclose(anchored[0], anchor, atol=1e-7)
        delta = replay.compute_init_delta(anchor, anchored[0], "left")
        self.assertAlmostEqual(delta.linear_m, 0.0)
        self.assertAlmostEqual(delta.angular_deg, 0.0)
        self.assertEqual(stats["delta_count"], 2)
        np.testing.assert_allclose(anchored[:, 0] - anchored[0, 0], [0.0, 0.02, 0.04], atol=1e-6)

    def test_action_scale_controls_roi_envelope(self) -> None:
        source = np.stack([pose(0.0), pose(0.2), pose(0.4)], axis=0)
        anchor = pose(0.0, 0.0, 0.2)
        server = replay.ServerRuntimeConfig(
            path=Path("server.yaml"),
            command_endpoint="udp://127.0.0.1:1",
            state_bind=None,
            servo_rate_hz=10.0,
            command_timeout_sec=0.2,
            smd_max_linear_velocity_m_s=0.5,
            smd_max_angular_velocity_rad_s=1.0,
            floor_z_min_m=0.0,
            roi_min_m=(-0.01, -0.1, 0.0),
            roi_max_m=(0.25, 0.1, 0.4),
            raw={},
        )
        small, _ = replay.compose_ee_local_from_anchor(source, anchor, r_align=None, action_scale=0.5)
        large, _ = replay.compose_ee_local_from_anchor(source, anchor, r_align=None, action_scale=1.0)
        self.assertIn("ROI precheck ok", "\n".join(replay.validate_bounds({"left": small}, server)))
        with self.assertRaisesRegex(replay.ReplayRefusal, "violates ROI"):
            replay.validate_bounds({"left": large}, server)

    def test_dry_run_ee_local_data_tcp_sends_nothing(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            data_tcp = write_data_tcp(root / "data_tcp" / "episode_000.hdf5")
            config = write_server_config(root / "server.yaml")
            out = root / "out"

            class NoSocketClient:
                def __init__(self, *args, **kwargs):
                    raise AssertionError("dry-run must not construct ServoCommandClient")

            stdout = io.StringIO()
            with patch.object(replay, "ServoCommandClient", NoSocketClient), redirect_stdout(stdout):
                rc = replay.main(
                    [
                        "--source",
                        "ee_local",
                        "--data-tcp",
                        str(data_tcp),
                        "--server-config",
                        str(config),
                        "--arms",
                        "left,right",
                        "--anchor",
                        "mock",
                        "--mock-current-pose",
                        json.dumps(
                            {
                                "left": pose(-0.2, -0.4, 0.25).tolist(),
                                "right": pose(0.2, -0.4, 0.25).tolist(),
                            }
                        ),
                        "--r-align",
                        "identity",
                        "--out-dir",
                        str(out),
                        "--max-linear-speed-m-s",
                        "10.0",
                        "--max-angular-speed-rad-s",
                        "10.0",
                    ]
                )

            self.assertEqual(rc, 0)
            text = stdout.getvalue()
            self.assertIn("DRY RUN — no motion sent", text)
            self.assertIn("left init_delta: linear=0.000000 m", text)
            self.assertIn("right init_delta: linear=0.000000 m", text)
            logs = list((out / "data_tcp__episode_000" / "runs").glob("*/log.csv"))
            self.assertEqual(len(logs), 1)

    def test_ee_local_auto_largest_segment_selects_gap_free_longest_run(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            data_tcp = write_data_tcp_two_segments(root / "data_tcp" / "episode_gap.hdf5")
            config = write_server_config(root / "server.yaml", max_linear=0.5, max_angular=1.0)
            out = root / "out"

            stdout = io.StringIO()
            with redirect_stdout(stdout):
                rc = replay.main(
                    [
                        "--source",
                        "ee_local",
                        "--data-tcp",
                        str(data_tcp),
                        "--server-config",
                        str(config),
                        "--arms",
                        "left,right",
                        "--anchor",
                        "mock",
                        "--mock-current-pose",
                        "default",
                        "--r-align",
                        "identity",
                        "--segment",
                        "auto-largest",
                        "--out-dir",
                        str(out),
                    ]
                )

            self.assertEqual(rc, 0)
            text = stdout.getvalue()
            self.assertIn("segment_selection: selected segment=1 source_frames=3-6 frames=4", text)
            self.assertIn("dropped=3 (before=3, after=0)", text)
            self.assertIn("execute stream clamp precheck ok", text)
            self.assertNotIn("execute would abort", text)
            npz_paths = list((out / "data_tcp__episode_gap").glob("*segment_1_3_7_clean_foh_se3_10hz.npz"))
            self.assertEqual(len(npz_paths), 1)
            with np.load(npz_paths[0], allow_pickle=True) as data:
                meta = json.loads(str(np.asarray(data["meta_json"]).item()))
                self.assertEqual(meta["segment_selection"]["segment_index"], 1)
                self.assertEqual(meta["segment_selection"]["source_frame_range"], [3, 6])
                left = np.asarray(data["left_conditioned_goal"], dtype=np.float64)
                speeds = replay.validate_stream_speeds(
                    {"left": left},
                    np.asarray(data["t_servo"], dtype=np.float64),
                    time_scale=1.0,
                    max_linear=0.5,
                    max_angular=1.0,
                )
                self.assertLess(float(speeds["left"]["max_linear_m_s"]), 0.4)

    def test_server_config_derived_speed_clamp_defaults_are_used(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            config = write_server_config(Path(tmp) / "server.yaml", max_linear=0.42, max_angular=1.23)
            args = replay.parse_args(["--npz", "dummy.npz", "--server-config", str(config)])
            server = replay.load_server_config(config)
            replay.configure_client_speed_limits(args, server)
            self.assertAlmostEqual(args.max_linear_speed_m_s, 0.42)
            self.assertAlmostEqual(args.max_angular_speed_rad_s, 1.23)
            self.assertEqual(args._max_linear_speed_source, "server_config:cartesian_control.pose_track_smd.max_linear_velocity_m_s")
            self.assertEqual(args._max_angular_speed_source, "server_config:cartesian_control.pose_track_smd.max_angular_velocity_rad_s")


def write_npz(path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    t = np.asarray([0.0, 0.1, 0.2], dtype=np.float64)
    left = np.stack([pose(0.0), pose(0.0), pose(0.0)], axis=0)
    right = np.stack([pose(0.05), pose(0.05), pose(0.05)], axis=0)
    arrays = {
        "t_servo": t,
        "servo_rate_hz": np.asarray(10.0),
        "mode": np.asarray("clean_foh_se3"),
        "episode": np.asarray("episode_test.hdf5"),
        "seed": np.asarray(-1),
        "segments": np.asarray([[0, 3]], dtype=np.int64),
        "gaps": np.empty((0, 2), dtype=np.float64),
        "meta_json": np.asarray(json.dumps({"episode_id": "episode_test"})),
    }
    for arm, data in (("left", left), ("right", right)):
        arrays[f"{arm}_source_raw_target"] = data
        arrays[f"{arm}_conditioned_goal"] = data
        arrays[f"{arm}_conditioned_twist"] = np.zeros((3, 6), dtype=np.float64)
        arrays[f"{arm}_gripper"] = np.full(3, np.nan)
        arrays[f"{arm}_valid"] = np.ones(3, dtype=bool)
        arrays[f"{arm}_hold"] = np.zeros(3, dtype=bool)
        arrays[f"{arm}_dropout"] = np.zeros(3, dtype=bool)
        arrays[f"{arm}_gap"] = np.zeros(3, dtype=bool)
        arrays[f"{arm}_reanchor"] = np.zeros(3, dtype=bool)
        arrays[f"{arm}_src_id_lo"] = np.asarray([0, 1, 2], dtype=np.int64)
        arrays[f"{arm}_src_id_hi"] = np.asarray([0, 1, 2], dtype=np.int64)
    np.savez(path, **arrays)
    return path


def write_server_config(path: Path, *, max_linear: float = 0.5, max_angular: float = 1.0) -> Path:
    path.write_text(
        f"""
servo:
  rate_hz: 10
  command_timeout_sec: 0.2
cartesian_control:
  pose_track_smd:
    max_linear_velocity_m_s: {max_linear}
    max_angular_velocity_rad_s: {max_angular}
safety:
  floor_constraint:
    enable: true
    z_min_m: 0.0
    monitor_only: false
  roi_box:
    enable: true
    min_m: [-1.0, -1.0, 0.0]
    max_m: [1.0, 1.0, 1.0]
    monitor_only: false
network:
  command_bind: "udp://127.0.0.1:50010"
  state_pub_endpoint: "udp://127.0.0.1:50110"
""",
        encoding="utf-8",
    )
    return path


def pgmode_not_activated_payload(
    *,
    top_overrides: dict | None = None,
    right_overrides: dict | None = None,
    raw_overrides: dict | None = None,
) -> dict:
    raw = {
        "init_state_info": 0,
        "init_error": 2,
        "op_stat_ems_flag": 0,
        "op_stat_sos_flag": 0,
        "op_stat_soft_estop_occur": 0,
        "op_stat_collision_occur": 0,
        "op_stat_self_collision": 0,
    }
    if raw_overrides:
        raw.update(raw_overrides)
    right = {
        "has_error": True,
        "servo_enabled": False,
        "error_code": 2,
        "startup_invalid_reasons": ["robot_fault", "servo_disabled"],
        "diagnostic_error_source": "rbpodo_init_error",
        "rbpodo_diagnostics": {"raw": raw},
    }
    if right_overrides:
        right.update(right_overrides)
    payload = {
        "observed_mode": "simulation",
        "operation_mode": "simulation",
        "physical_motion_expected": False,
        "left": {
            "has_error": False,
            "servo_enabled": True,
            "rbpodo_diagnostics": {"raw": {"init_state_info": 6, "init_error": 0}},
        },
        "right": right,
    }
    if top_overrides:
        payload.update(top_overrides)
    return payload


def write_data_tcp(path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    left = np.stack([pose(0.0), pose(0.01), pose(0.02)], axis=0).astype(np.float32)
    right = np.stack([pose(0.0), pose(-0.01), pose(-0.02)], axis=0).astype(np.float32)
    with h5py.File(path, "w") as handle:
        action = handle.create_group("action")
        action.create_dataset("target_pose_left", data=left)
        action.create_dataset("target_pose_right", data=right)
        action.create_dataset("gripper_left", data=np.zeros(3, dtype=np.float32))
        action.create_dataset("gripper_right", data=np.zeros(3, dtype=np.float32))
        observations = handle.create_group("observations")
        observations.create_dataset("timestamp", data=np.asarray([0.0, 0.1, 0.2], dtype=np.float64))
        handle.attrs["source_pose_frame"] = "steamvr_world"
        handle.attrs["retarget_status"] = "configured_estimate"
    return path


def write_data_tcp_two_segments(path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    first = [pose(0.0), pose(0.01), pose(0.02)]
    second = [pose(1.00), pose(1.01), pose(1.02), pose(1.03)]
    left = np.stack(first + second, axis=0).astype(np.float32)
    right = np.stack([pose(-row[0]) for row in left], axis=0).astype(np.float32)
    with h5py.File(path, "w") as handle:
        action = handle.create_group("action")
        action.create_dataset("target_pose_left", data=left)
        action.create_dataset("target_pose_right", data=right)
        action.create_dataset("gripper_left", data=np.zeros(left.shape[0], dtype=np.float32))
        action.create_dataset("gripper_right", data=np.zeros(left.shape[0], dtype=np.float32))
        observations = handle.create_group("observations")
        observations.create_dataset("timestamp", data=np.asarray([0.0, 0.033, 0.066, 0.300, 0.333, 0.366, 0.399], dtype=np.float64))
        handle.attrs["source_pose_frame"] = "steamvr_world"
        handle.attrs["retarget_status"] = "configured_estimate"
    return path


if __name__ == "__main__":
    unittest.main()
