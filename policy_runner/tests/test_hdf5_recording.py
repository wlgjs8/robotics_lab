from __future__ import annotations

import shutil
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from policy_runner.camera_bundle_client import CameraBundle, CameraFrame, _MISSING_BUNDLE_AGE_US
from policy_runner.config import CameraConfig, PolicyRunnerConfig, RecordingConfig, config_from_mapping
from policy_runner.recording import (
    DATASET_METADATA_SCHEMA,
    Hdf5EpisodeRecorder,
    _hash_canonical_json,
    build_dataset_metadata,
)
from policy_runner.robot_state_client import StateSnapshot

try:
    import h5py
    import numpy as np
except ModuleNotFoundError:
    h5py = None
    np = None


def _state_snapshot(tick: int = 1, **overrides) -> StateSnapshot:
    payload = {
        "schema_version": 1,
        "tick": tick,
        "host_time_ns": 123456789 + tick,
        "command_seq": tick + 10,
        "state_age_us": 42,
        "fault_latched": False,
        "left": {
            "q_actual_deg": [1, 2, 3, 4, 5, 6],
            "q_sent_deg": [1.1, 2.1, 3.1, 4.1, 5.1, 6.1],
            "q_ref_deg": [1.2, 2.2, 3.2, 4.2, 5.2, 6.2],
            "tcp_stand": {
                "x": 0.1,
                "y": 0.2,
                "z": 0.3,
                "quaternion_xyzw": [0.0, 0.1, 0.2, 0.97],
            },
            "tcp_actual_stand": {
                "x": 0.1,
                "y": 0.2,
                "z": 0.3,
                "quaternion_xyzw": [0.0, 0.1, 0.2, 0.97],
            },
            "tcp_ref_stand": {
                "x": 0.15,
                "y": 0.25,
                "z": 0.35,
                "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
            "tcp_actual_valid": True,
            "tcp_ref_valid": True,
            "tcp_tracking_source": "tcp_ref_stand",
            "lifecycle_state": "diagnostics_suspect",
            "send_duration_us": 123,
            "ack_policy": "ACK-on",
            "controller_acceptance_observed": True,
        },
        "right": {
            "q_actual_deg": [-1, -2, -3, -4, -5, -6],
            "q_sent_deg": [-1.1, -2.1, -3.1, -4.1, -5.1, -6.1],
            "q_target_deg": [-1.2, -2.2, -3.2, -4.2, -5.2, -6.2],
            "tcp_stand": {
                "x": -0.1,
                "y": -0.2,
                "z": -0.3,
                "qx": 0.3,
                "qy": 0.2,
                "qz": 0.1,
                "qw": 0.9,
            },
            "tcp_actual_valid": True,
            "tcp_ref_valid": False,
            "tcp_tracking_source_recommendation": "tcp_actual_stand",
            "backend_timing": {"send_duration_us": 234},
            "ack_semantics": "ACK-off",
        },
    }
    payload.update(overrides)
    return StateSnapshot(payload=payload, received_monotonic=1.0)


def _pose_action(seq: int = 7) -> dict:
    return {
        "seq": seq,
        "mode": "TcpPoseTarget",
        "source_id": "policy_runner",
        "left": {
            "mode": "TcpPoseTarget",
            "tcp_target_stand": [0.31, 0.22, 0.33, 0.0, 0.1, 0.2],
            "spacemouse_raw": {"axes": [1, 0, 0, 0, 0, 0], "buttons": [True, False]},
            "deadman": True,
        },
        "right": {
            "mode": "TcpPoseTarget",
            "tcp_target_stand": [-0.31, -0.22, -0.33, 0.3, 0.2, 0.1],
            "deadman": False,
        },
    }


def _config_snapshots(path_kp_pos: float = 6.0, damping: float = 0.001) -> dict:
    return {
        "cartesian_control_snapshot": {
            "schema": "robotics_lab.cartesian_control_snapshot.v1",
            "enable": True,
            "path_kp_pos": path_kp_pos,
            "linear_move": {
                "default_linear_speed_m_s": 0.03,
                "default_orientation_mode": "constant",
            },
        },
        "kinematics_snapshot": {
            "schema": "robotics_lab.kinematics_snapshot.v1",
            "enable": True,
            "provider": "pinocchio",
            "tip_link": "tcp",
            "ik": {
                "damping": damping,
                "max_iterations": 50,
            },
        },
    }


class FakeCameraClient:
    def __init__(self, bundles_by_call: list[CameraBundle | None]):
        self._bundles = list(bundles_by_call)
        self._latest: CameraBundle | None = None

    def poll(self, timeout_ms: int = 0) -> CameraBundle | None:
        _ = timeout_ms
        if not self._bundles:
            return self._latest
        bundle = self._bundles.pop(0)
        if bundle is not None:
            self._latest = bundle
        return bundle

    def latest(self) -> CameraBundle | None:
        return self._latest

    def is_fresh(self, bundle: CameraBundle | None = None) -> bool:
        _ = bundle
        return True

    def close(self) -> None:
        pass


def _camera_bundle(
    *,
    cam_name: str = "head",
    pixels=None,
    seq: int = 1,
    frames: dict[str, CameraFrame] | None = None,
) -> CameraBundle:
    if frames is None:
        assert np is not None
        if pixels is None:
            pixels = np.arange(2 * 3 * 3, dtype=np.uint8).reshape(2, 3, 3)
        frames = {
            cam_name: CameraFrame(
                camera_name=cam_name,
                width=int(pixels.shape[1]),
                height=int(pixels.shape[0]),
                pixels=pixels,
                format="rgb8",
                frame_number=seq,
                host_arrival_time_ns=1000 + seq,
                sensor_timestamp_ns=900 + seq,
            )
        }
    return CameraBundle(
        bundle_seq=seq,
        bundle_time_ns=100_000_000_000 + seq,
        hardware_synced=False,
        complete=True,
        received_monotonic=1.0,
        frames=frames,
    )


@unittest.skipIf(h5py is None or np is None, "recording extras not installed")
class Hdf5EpisodeRecorderTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tmpdir = tempfile.mkdtemp(prefix="policy-runner-hdf5-test-")

    def tearDown(self) -> None:
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def _recorder(self, rate_hz: float = 30.0) -> Hdf5EpisodeRecorder:
        return Hdf5EpisodeRecorder(self.tmpdir, recording_rate_hz=rate_hz)

    def _start_record_end(self, recorder: Hdf5EpisodeRecorder, frame_count: int = 1) -> Path:
        recorder.start_episode(
            reset_snapshot=_state_snapshot(),
            task_description="test task",
            action_source="dual_spacemouse_pose_target",
            operator_id="operator_a",
        )
        for idx in range(frame_count):
            recorder.record_frame(
                state_snapshot=_state_snapshot(tick=idx + 1),
                action_packet=_pose_action(seq=idx + 1),
                action_host_time_ns=1000 + idx,
                action_seq=idx + 1,
            )
        return recorder.end_episode(success=True, end_reason="operator_success")

    def test_start_episode_captures_reset_qpos_from_snapshot(self) -> None:
        path = self._start_record_end(self._recorder(), frame_count=0)
        with h5py.File(path, "r") as handle:
            np.testing.assert_allclose(handle.attrs["reset_qpos_left"], [1, 2, 3, 4, 5, 6])
            np.testing.assert_allclose(handle.attrs["reset_qpos_right"], [-1, -2, -3, -4, -5, -6])

    def test_start_episode_captures_reset_tcp_stand_from_snapshot(self) -> None:
        path = self._start_record_end(self._recorder(), frame_count=0)
        with h5py.File(path, "r") as handle:
            np.testing.assert_allclose(
                handle.attrs["reset_tcp_stand_left"],
                [0.1, 0.2, 0.3, 0.0, 0.1, 0.2, 0.97],
                rtol=1e-6,
            )
            np.testing.assert_allclose(
                handle.attrs["reset_tcp_stand_right"],
                [-0.1, -0.2, -0.3, 0.3, 0.2, 0.1, 0.9],
                rtol=1e-6,
            )

    def test_record_frame_buffers_frames_at_target_rate(self) -> None:
        clock = {"now": 0.0}
        recorder = self._recorder(rate_hz=30.0)
        with mock.patch("policy_runner.recording.time.monotonic", side_effect=lambda: clock["now"]):
            recorder.start_episode(
                reset_snapshot=_state_snapshot(),
                task_description="rate test",
                action_source="dual_spacemouse_pose_target",
            )
            for idx in range(100):
                clock["now"] = idx * 0.01
                recorder.record_frame(
                    state_snapshot=_state_snapshot(tick=idx),
                    action_packet=_pose_action(seq=idx),
                    action_host_time_ns=idx,
                    action_seq=idx,
                )
            clock["now"] = 1.0
            path = recorder.end_episode(success=True, end_reason="operator_success")

        with h5py.File(path, "r") as handle:
            self.assertGreaterEqual(handle["observations/qpos_left"].shape[0], 28)
            self.assertLessEqual(handle["observations/qpos_left"].shape[0], 32)

    def test_end_episode_writes_observations_and_action_groups(self) -> None:
        clock = {"now": 0.0}
        recorder = self._recorder(rate_hz=100.0)
        with mock.patch("policy_runner.recording.time.monotonic", side_effect=lambda: clock["now"]):
            recorder.start_episode(
                reset_snapshot=_state_snapshot(),
                task_description="dataset test",
                action_source="dual_spacemouse_pose_target",
            )
            for idx in range(5):
                clock["now"] = idx * 0.02
                recorder.record_frame(
                    state_snapshot=_state_snapshot(tick=idx + 1),
                    action_packet=_pose_action(seq=idx + 1),
                    action_host_time_ns=100 + idx,
                    action_seq=idx + 1,
                )
            path = recorder.end_episode(success=True, end_reason="operator_success")

        with h5py.File(path, "r") as handle:
            for dataset in (
                "qpos_left",
                "qsent_left",
                "tcp_stand_left",
                "fault_latched",
                "state_age_us",
                "state_host_time_ns",
                "command_seq",
            ):
                self.assertEqual(handle[f"observations/{dataset}"].shape[0], 5)
            for dataset in (
                "mode",
                "tcp_target_stand_left",
                "deadman_left",
                "action_host_time_ns",
                "seq",
            ):
                self.assertEqual(handle[f"action/{dataset}"].shape[0], 5)

    def test_end_episode_attrs_contain_success_and_end_reason(self) -> None:
        path = self._start_record_end(self._recorder(), frame_count=1)
        with h5py.File(path, "r") as handle:
            self.assertTrue(bool(handle.attrs["success"]))
            self.assertEqual(handle.attrs["end_reason"], "operator_success")
            self.assertEqual(handle.attrs["schema"], Hdf5EpisodeRecorder.SCHEMA)

    def test_config_snapshots_are_hashed_and_stored(self) -> None:
        snapshots = _config_snapshots(path_kp_pos=7.0, damping=0.004)
        recorder = self._recorder()
        recorder.start_episode(
            reset_snapshot=_state_snapshot(**snapshots),
            task_description="config",
            action_source="dual_spacemouse_pose_target",
        )
        path = recorder.end_episode(success=True, end_reason="operator_success")
        with h5py.File(path, "r") as handle:
            self.assertEqual(
                handle.attrs["cartesian_control_hash"],
                _hash_canonical_json(snapshots["cartesian_control_snapshot"]),
            )
            self.assertEqual(
                handle.attrs["kinematics_hash"],
                _hash_canonical_json(snapshots["kinematics_snapshot"]),
            )
            self.assertEqual(handle["config/cartesian_control"].attrs["schema"], "robotics_lab.cartesian_control_snapshot.v1")
            self.assertEqual(handle["config/cartesian_control"].attrs["path_kp_pos"], 7.0)
            self.assertEqual(handle["config/cartesian_control/linear_move"].attrs["default_linear_speed_m_s"], 0.03)
            self.assertEqual(handle["config/kinematics"].attrs["provider"], "pinocchio")
            self.assertEqual(handle["config/kinematics/ik"].attrs["damping"], 0.004)

    def test_dataset_metadata_and_reference_state_fields_are_recorded(self) -> None:
        metadata = build_dataset_metadata(
            git_commit="abc123",
            config_hash="config-sha",
            backend_type="rbpodo",
            run_mode="real",
            operation_mode="simulation",
            physical_motion_expected=False,
            controller_pgmode="simulation",
            calibration_status="configured_estimate",
            camera_status="disabled",
            command_source_id="policy_runner",
            benchmark_linkage={
                "circle_profile": "circle_15cm_16s",
                "overlay_run_id": "run-001",
            },
        )
        recorder = self._recorder(rate_hz=100.0)
        recorder.start_episode(
            reset_snapshot=_state_snapshot(),
            task_description="metadata",
            action_source="dual_spacemouse_pose_target",
            dataset_metadata=metadata,
        )
        recorder.record_frame(
            state_snapshot=_state_snapshot(),
            action_packet=_pose_action(),
            action_host_time_ns=100,
            action_seq=1,
        )
        path = recorder.end_episode(success=True, end_reason="operator_success")

        with h5py.File(path, "r") as handle:
            dataset_meta = handle["metadata/dataset"].attrs
            self.assertEqual(dataset_meta["schema"], DATASET_METADATA_SCHEMA)
            self.assertEqual(dataset_meta["backend_type"], "rbpodo")
            self.assertFalse(bool(dataset_meta["physical_motion_expected"]))
            self.assertEqual(handle["metadata/dataset/benchmark_linkage"].attrs["circle_profile"], "circle_15cm_16s")
            np.testing.assert_allclose(handle["observations/q_ref_left"][0], [1.2, 2.2, 3.2, 4.2, 5.2, 6.2])
            np.testing.assert_allclose(handle["observations/q_ref_right"][0], [-1.2, -2.2, -3.2, -4.2, -5.2, -6.2])
            np.testing.assert_allclose(
                handle["observations/tcp_actual_stand_left"][0],
                [0.1, 0.2, 0.3, 0.0, 0.1, 0.2, 0.97],
                rtol=1e-6,
            )
            np.testing.assert_allclose(
                handle["observations/tcp_ref_stand_left"][0],
                [0.15, 0.25, 0.35, 0.0, 0.0, 0.0, 1.0],
                rtol=1e-6,
            )
            self.assertTrue(bool(handle["observations/tcp_ref_valid_left"][0]))
            self.assertFalse(bool(handle["observations/tcp_ref_valid_right"][0]))
            source = handle["observations/tcp_tracking_source_left"][0]
            self.assertEqual(source.decode("utf-8") if isinstance(source, bytes) else source, "tcp_ref_stand")
            self.assertTrue(bool(handle["observations/diagnostics_suspect_left"][0]))
            self.assertEqual(int(handle["observations/send_duration_us_left"][0]), 123)
            ack_policy = handle["observations/ack_policy_right"][0]
            self.assertEqual(ack_policy.decode("utf-8") if isinstance(ack_policy, bytes) else ack_policy, "ACK-off")
            self.assertTrue(bool(handle["observations/controller_acceptance_observed_left"][0]))
            source_id = handle["action/source_id"][0]
            self.assertEqual(source_id.decode("utf-8") if isinstance(source_id, bytes) else source_id, "policy_runner")
            np.testing.assert_allclose(handle["action/spacemouse_axes_left"][0], [1, 0, 0, 0, 0, 0])
            np.testing.assert_array_equal(
                handle["action/spacemouse_buttons_left"][0],
                [True, False, False, False, False, False, False, False],
            )

    def test_config_snapshot_hash_changes_when_snapshot_changes(self) -> None:
        first = _hash_canonical_json(_config_snapshots(path_kp_pos=6.0)["cartesian_control_snapshot"])
        second = _hash_canonical_json(_config_snapshots(path_kp_pos=9.0)["cartesian_control_snapshot"])
        self.assertNotEqual(first, second)
        self.assertRegex(first, r"^[0-9a-f]{64}$")

    def test_missing_config_snapshots_store_empty_hashes(self) -> None:
        path = self._start_record_end(self._recorder(), frame_count=0)
        with h5py.File(path, "r") as handle:
            self.assertEqual(handle.attrs["cartesian_control_hash"], "")
            self.assertEqual(handle.attrs["kinematics_hash"], "")
            self.assertIn("cartesian_control", handle["config"])
            self.assertIn("kinematics", handle["config"])

    def test_end_episode_with_invalid_end_reason_raises(self) -> None:
        recorder = self._recorder()
        recorder.start_episode(
            reset_snapshot=_state_snapshot(),
            task_description="bad reason",
            action_source="hold",
        )
        with self.assertRaises(ValueError):
            recorder.end_episode(success=False, end_reason="bogus")

    def test_start_episode_twice_without_end_raises(self) -> None:
        recorder = self._recorder()
        recorder.start_episode(
            reset_snapshot=_state_snapshot(),
            task_description="first",
            action_source="hold",
        )
        with self.assertRaises(RuntimeError):
            recorder.start_episode(
                reset_snapshot=_state_snapshot(),
                task_description="second",
                action_source="hold",
            )

    def test_action_mode_recorded_as_string(self) -> None:
        path = self._start_record_end(self._recorder(), frame_count=1)
        with h5py.File(path, "r") as handle:
            raw = handle["action/mode"][0]
            mode = raw.decode("utf-8") if isinstance(raw, bytes) else str(raw)
            self.assertEqual(mode, "TcpPoseTarget")

    def test_action_target_zero_when_mode_is_hold(self) -> None:
        recorder = self._recorder()
        recorder.start_episode(
            reset_snapshot=_state_snapshot(),
            task_description="hold",
            action_source="hold",
        )
        recorder.record_frame(
            state_snapshot=_state_snapshot(),
            action_packet={"left": {"mode": "Hold"}, "right": {"mode": "Hold"}},
            action_host_time_ns=1,
            action_seq=1,
        )
        path = recorder.end_episode(success=True, end_reason="operator_success")
        with h5py.File(path, "r") as handle:
            np.testing.assert_allclose(handle["action/tcp_target_stand_left"][0], [0, 0, 0, 0, 0, 0])

    def test_close_with_active_episode_aborts(self) -> None:
        recorder = self._recorder()
        recorder.start_episode(
            reset_snapshot=_state_snapshot(),
            task_description="abort",
            action_source="hold",
        )
        recorder.close()
        paths = list(Path(self.tmpdir).glob("*.hdf5"))
        self.assertEqual(len(paths), 1)
        with h5py.File(paths[0], "r") as handle:
            self.assertFalse(bool(handle.attrs["success"]))
            self.assertEqual(handle.attrs["end_reason"], "operator_abort")

    def test_recording_config_rate_validation(self) -> None:
        with self.assertRaises(ValueError):
            RecordingConfig(rate_hz=0.5)
        with self.assertRaises(ValueError):
            RecordingConfig(rate_hz=200.0)
        self.assertEqual(RecordingConfig(rate_hz=30.0).rate_hz, 30.0)

    def test_recording_config_default_from_dict(self) -> None:
        cfg = PolicyRunnerConfig.from_dict({})
        self.assertEqual(cfg.recording.format, "hdf5")
        self.assertEqual(cfg.recording.rate_hz, 30.0)
        mapped = config_from_mapping({"recording": {"format": "jsonl", "rate_hz": 12.5}})
        self.assertEqual(mapped.recording.format, "jsonl")
        self.assertEqual(mapped.recording.rate_hz, 12.5)

    def test_camera_config_default_from_dict(self) -> None:
        cfg = PolicyRunnerConfig.from_dict({})
        self.assertFalse(cfg.camera.enable)
        self.assertEqual(cfg.camera.bundle_topic, "camera.bundle")
        mapped = config_from_mapping(
            {
                "camera": {
                    "enable": True,
                    "zmq_endpoint": "tcp://127.0.0.1:5601",
                    "expected_cameras": ["head", "left_wrist"],
                }
            }
        )
        self.assertTrue(mapped.camera.enable)
        self.assertEqual(mapped.camera.zmq_endpoint, "tcp://127.0.0.1:5601")
        self.assertEqual(mapped.camera.expected_cameras, ["head", "left_wrist"])
        with self.assertRaises(ValueError):
            CameraConfig(max_age_ms=0.0)

    def test_recording_with_camera_client_writes_image_datasets(self) -> None:
        first = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
        second = np.full((2, 3, 3), 9, dtype=np.uint8)
        camera_client = FakeCameraClient(
            [_camera_bundle(pixels=first, seq=1), _camera_bundle(pixels=second, seq=2)]
        )
        recorder = Hdf5EpisodeRecorder(
            self.tmpdir,
            recording_rate_hz=100.0,
            camera_client=camera_client,
            expected_cameras=["head"],
        )
        recorder.start_episode(
            reset_snapshot=_state_snapshot(),
            task_description="camera",
            action_source="dual_spacemouse_pose_target",
        )
        with mock.patch("policy_runner.recording.time.monotonic", side_effect=[0.0, 0.02, 0.04]):
            recorder.record_frame(
                state_snapshot=_state_snapshot(tick=1),
                action_packet=_pose_action(seq=1),
                action_host_time_ns=1,
                action_seq=1,
            )
            recorder.record_frame(
                state_snapshot=_state_snapshot(tick=2),
                action_packet=_pose_action(seq=2),
                action_host_time_ns=2,
                action_seq=2,
            )
            path = recorder.end_episode(success=True, end_reason="operator_success")
        with h5py.File(path, "r") as handle:
            dataset = handle["observations/images/head"]
            self.assertEqual(dataset.shape, (2, 2, 3, 3))
            self.assertEqual(dataset.dtype, np.dtype("uint8"))
            self.assertEqual(dataset.compression, "gzip")
            np.testing.assert_array_equal(dataset[0], first)
            np.testing.assert_array_equal(dataset[1], second)
            np.testing.assert_array_equal(handle["observations/bundle_seq"][...], [1, 2])

    def test_recording_zero_fills_when_bundle_missing_camera(self) -> None:
        first = np.ones((2, 3, 3), dtype=np.uint8)
        missing_bundle = _camera_bundle(seq=2, frames={})
        camera_client = FakeCameraClient([_camera_bundle(pixels=first, seq=1), missing_bundle])
        recorder = Hdf5EpisodeRecorder(
            self.tmpdir,
            recording_rate_hz=100.0,
            camera_client=camera_client,
            expected_cameras=["head"],
        )
        recorder.start_episode(
            reset_snapshot=_state_snapshot(),
            task_description="zero",
            action_source="dual_spacemouse_pose_target",
        )
        with mock.patch("policy_runner.recording.time.monotonic", side_effect=[0.0, 0.02, 0.04]):
            for idx in range(2):
                recorder.record_frame(
                    state_snapshot=_state_snapshot(tick=idx + 1),
                    action_packet=_pose_action(seq=idx + 1),
                    action_host_time_ns=idx + 1,
                    action_seq=idx + 1,
                )
            path = recorder.end_episode(success=True, end_reason="operator_success")
        with h5py.File(path, "r") as handle:
            np.testing.assert_array_equal(handle["observations/images/head"][0], first)
            np.testing.assert_array_equal(handle["observations/images/head"][1], np.zeros((2, 3, 3), dtype=np.uint8))

    def test_recording_maps_dataset_camera_names_to_bundle_stream_keys(self) -> None:
        # camera_server keys frames '<camera>.<stream>'; expected_cameras may use
        # dataset names like 'left_realsense_color' and must still record pixels.
        pixels = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
        camera_client = FakeCameraClient(
            [_camera_bundle(cam_name="left_realsense.color", pixels=pixels, seq=1)]
        )
        recorder = Hdf5EpisodeRecorder(
            self.tmpdir,
            recording_rate_hz=100.0,
            camera_client=camera_client,
            expected_cameras=["left_realsense_color"],
        )
        recorder.start_episode(
            reset_snapshot=_state_snapshot(),
            task_description="mapped",
            action_source="dual_spacemouse_pose_target",
        )
        recorder.record_frame(
            state_snapshot=_state_snapshot(tick=1),
            action_packet=_pose_action(seq=1),
            action_host_time_ns=1,
            action_seq=1,
        )
        path = recorder.end_episode(success=True, end_reason="operator_success")
        with h5py.File(path, "r") as handle:
            np.testing.assert_array_equal(
                handle["observations/images/left_realsense_color"][0], pixels
            )

    def test_recording_uses_missing_marker_when_no_bundle_exists(self) -> None:
        camera_client = FakeCameraClient([None])
        recorder = Hdf5EpisodeRecorder(
            self.tmpdir,
            recording_rate_hz=30.0,
            camera_client=camera_client,
            expected_cameras=["head"],
        )
        recorder.start_episode(
            reset_snapshot=_state_snapshot(),
            task_description="missing",
            action_source="dual_spacemouse_pose_target",
        )
        recorder.record_frame(
            state_snapshot=_state_snapshot(),
            action_packet=_pose_action(),
            action_host_time_ns=1,
            action_seq=1,
        )
        path = recorder.end_episode(success=True, end_reason="operator_success")
        with h5py.File(path, "r") as handle:
            self.assertNotIn("images", handle["observations"])
            self.assertEqual(int(handle["observations/bundle_age_us"][0]), _MISSING_BUNDLE_AGE_US)

    def test_recording_camera_chunking_is_per_frame(self) -> None:
        pixels = np.zeros((4, 5, 3), dtype=np.uint8)
        camera_client = FakeCameraClient([_camera_bundle(pixels=pixels, seq=idx) for idx in range(5)])
        recorder = Hdf5EpisodeRecorder(
            self.tmpdir,
            recording_rate_hz=100.0,
            camera_client=camera_client,
            expected_cameras=["head"],
        )
        recorder.start_episode(
            reset_snapshot=_state_snapshot(),
            task_description="chunk",
            action_source="dual_spacemouse_pose_target",
        )
        with mock.patch("policy_runner.recording.time.monotonic", side_effect=[0.0, 0.02, 0.04, 0.06, 0.08, 0.1]):
            for idx in range(5):
                recorder.record_frame(
                    state_snapshot=_state_snapshot(tick=idx),
                    action_packet=_pose_action(seq=idx),
                    action_host_time_ns=idx,
                    action_seq=idx,
                )
            path = recorder.end_episode(success=True, end_reason="operator_success")
        with h5py.File(path, "r") as handle:
            self.assertEqual(handle["observations/images/head"].chunks, (1, 4, 5, 3))


if __name__ == "__main__":
    unittest.main()
