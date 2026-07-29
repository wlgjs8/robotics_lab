from __future__ import annotations

import csv
import tempfile
import unittest
from dataclasses import replace
from datetime import datetime
from pathlib import Path

import numpy as np

from rb_servo_gui.camera_quality import (
    CameraFrameInput,
    CameraQualityAnalyzer,
    CameraQualityStore,
    RobotMotionSample,
    RobotMotionTracker,
    camera_quality_html,
    compute_blur_effect,
)
from rb_servo_gui.app import (
    _operator_monitor_dynamic_html,
    _update_camera_quality,
)
from rb_servo_gui.models import StateSnapshot

try:
    import cv2
except ModuleNotFoundError:
    cv2 = None


def _textured_image(seed: int = 7) -> np.ndarray:
    rng = np.random.default_rng(seed)
    gray = rng.integers(0, 256, size=(240, 320), dtype=np.uint8)
    return np.repeat(gray[..., None], 3, axis=2)


def _frame(
    image: np.ndarray,
    *,
    index: int,
    time_sec: float,
    exposure_us: float | None = 10_000.0,
) -> CameraFrameInput:
    return CameraFrameInput(
        arm="left",
        topic="camera.bundle.wrist_left",
        bundle_seq=index,
        camera_name="left_realsense",
        serial="412622272078",
        frame_number=index,
        host_arrival_time_ns=int(time_sec * 1e9),
        received_monotonic=time_sec,
        frame_age_ms=2.0,
        actual_exposure_us=exposure_us,
        gain_level=16.0,
        auto_exposure=True,
        pixels_rgb=image,
    )


def _stationary_robot(time_sec: float) -> RobotMotionSample:
    return RobotMotionSample(
        arm="left",
        received_monotonic=time_sec,
        state_tick=int(time_sec * 100),
        tcp_linear_speed_mm_s=0.0,
        tcp_angular_speed_deg_s=0.0,
        joint_speed_rms_deg_s=0.0,
        q_tracking_rms_deg=0.0,
        motion_state="Armed",
        command_mode="Hold",
    )


def _warp(image: np.ndarray, dx: float, dy: float = 0.0) -> np.ndarray:
    assert cv2 is not None
    return cv2.warpAffine(
        image,
        np.asarray(((1.0, 0.0, dx), (0.0, 1.0, dy)), dtype=np.float32),
        (image.shape[1], image.shape[0]),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REFLECT,
    )


@unittest.skipIf(cv2 is None, "opencv-python-headless is not installed")
class CameraQualityAnalyzerTest(unittest.TestCase):
    def test_blur_effect_increases_for_gaussian_blur(self) -> None:
        assert cv2 is not None
        sharp = _textured_image()[..., 0]
        blurred = cv2.GaussianBlur(sharp, (0, 0), sigmaX=4.0)

        sharp_effect, sharp_edges = compute_blur_effect(sharp)
        blurred_effect, blurred_edges = compute_blur_effect(blurred)

        self.assertLess(sharp_effect, blurred_effect)
        self.assertGreater(sharp_edges, blurred_edges)
        self.assertGreater(blurred_effect - sharp_effect, 0.10)

    def test_global_motion_and_exposure_smear_follow_known_translation(self) -> None:
        analyzer = CameraQualityAnalyzer("left")
        source = _textured_image()
        analyzer.analyze(_frame(source, index=1, time_sec=1.0), None, None)

        sample, _ = analyzer.analyze(
            _frame(_warp(source, 3.0, 1.0), index=2, time_sec=1.0 + 1.0 / 30.0),
            _stationary_robot(1.0 + 1.0 / 30.0),
            0.0,
        )

        self.assertTrue(sample.flow_valid)
        self.assertAlmostEqual(sample.global_dx_px or 0.0, 3.0, delta=0.35)
        self.assertAlmostEqual(sample.global_dy_px or 0.0, 1.0, delta=0.35)
        self.assertAlmostEqual(sample.global_motion_px_s or 0.0, 30.0 * np.sqrt(10.0), delta=12.0)
        self.assertAlmostEqual(sample.estimated_smear_px or 0.0, np.sqrt(10.0) * 0.3, delta=0.15)
        self.assertEqual(sample.texture_confidence, "ok")

    def test_high_frequency_alternation_has_more_shake_than_smooth_motion(self) -> None:
        source = _textured_image()

        def final_shake(offsets: list[float]) -> float:
            analyzer = CameraQualityAnalyzer("left")
            result = 0.0
            for index, offset in enumerate(offsets, start=1):
                time_sec = 1.0 + (index - 1) / 30.0
                sample, _ = analyzer.analyze(
                    _frame(_warp(source, offset), index=index, time_sec=time_sec),
                    _stationary_robot(time_sec),
                    0.0,
                )
                if sample.shake_px_s_rms is not None:
                    result = sample.shake_px_s_rms
            return result

        smooth = final_shake([0.30 * index for index in range(60)])
        oscillating = final_shake([3.0 if index % 2 else 0.0 for index in range(60)])

        self.assertGreater(oscillating, smooth * 5.0)
        self.assertGreater(oscillating, 40.0)

    def test_stationary_baseline_learns_and_moving_robot_does_not(self) -> None:
        source = _textured_image()
        analyzer = CameraQualityAnalyzer("left")
        sample = None
        for index in range(150):
            time_sec = 1.0 + index / 30.0
            sample, _ = analyzer.analyze(
                _frame(source, index=index + 1, time_sec=time_sec),
                _stationary_robot(time_sec),
                0.0,
            )
        assert sample is not None
        self.assertIsNotNone(sample.baseline_blur_effect)
        self.assertEqual(sample.baseline_state, "ready")

        moving = CameraQualityAnalyzer("left")
        moving_robot = _stationary_robot(1.0)
        moving_robot = RobotMotionSample(
            **{
                **moving_robot.__dict__,
                "tcp_linear_speed_mm_s": 20.0,
            }
        )
        for index in range(150):
            time_sec = 1.0 + index / 30.0
            moving.analyze(
                _frame(source, index=index + 1, time_sec=time_sec),
                RobotMotionSample(
                    **{
                        **moving_robot.__dict__,
                        "received_monotonic": time_sec,
                    }
                ),
                0.0,
            )
        self.assertIsNone(moving.baseline_blur)

    def test_restarted_camera_frame_counter_resets_flow_and_baseline(self) -> None:
        source = _textured_image()
        analyzer = CameraQualityAnalyzer("left")
        for index in range(150):
            time_sec = 1.0 + index / 30.0
            analyzer.analyze(
                _frame(source, index=index + 100, time_sec=time_sec),
                _stationary_robot(time_sec),
                0.0,
            )
        self.assertIsNotNone(analyzer.baseline_blur)

        restarted, _ = analyzer.analyze(
            _frame(source, index=1, time_sec=7.0),
            _stationary_robot(7.0),
            0.0,
        )

        self.assertFalse(restarted.flow_valid)
        self.assertIsNone(restarted.baseline_blur_effect)
        self.assertEqual(restarted.baseline_state, "waiting_stationary")

    def test_csv_contains_metrics_only_and_html_marks_diagnostic_scope(self) -> None:
        analyzer = CameraQualityAnalyzer("left")
        sample, preview = analyzer.analyze(
            _frame(_textured_image(), index=1, time_sec=1.0),
            None,
            None,
        )
        with tempfile.TemporaryDirectory() as tmp:
            store = CameraQualityStore(
                csv_dir=tmp,
                now=datetime(2026, 7, 29, 12, 34, 56),
            )
            store.update(sample, preview)
            path = store.csv_path
            html = camera_quality_html(store, now=1.1)
            store.close()

            self.assertIsNotNone(path)
            assert path is not None
            self.assertEqual(path.name, "camera_quality_20260729_123456.csv")
            with path.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["schema"], "robotics_lab.camera_quality.v1")
            self.assertEqual(rows[0]["arm"], "left")
            self.assertNotIn("pixels_rgb", rows[0])
            self.assertIn("diagnostic only", html)

            second = CameraQualityStore(
                csv_dir=tmp,
                now=datetime(2026, 7, 29, 12, 34, 56),
            )
            try:
                self.assertEqual(
                    second.csv_path,
                    Path(tmp) / "camera_quality_20260729_123456_1.csv",
                )
            finally:
                second.close()
            with path.open(newline="", encoding="utf-8") as stream:
                self.assertEqual(len(list(csv.DictReader(stream))), 1)

    def test_gui_update_keeps_preview_off_until_operator_enables_it(self) -> None:
        class Handle:
            def __init__(self, *, value=None, content="", visible=False) -> None:
                self.value = value
                self.content = content
                self.visible = visible
                self.image = None

        analyzer = CameraQualityAnalyzer("left")
        sample, preview = analyzer.analyze(
            _frame(_textured_image(), index=1, time_sec=1.0),
            None,
            None,
        )
        store = CameraQualityStore(enable_csv=False)
        try:
            store.update(sample, preview)
            status = Handle()
            csv_status = Handle()
            toggle = Handle(value=False)
            preview_handle = Handle(visible=False)
            handles = {
                "camera_quality_store": store,
                "camera_quality_status": status,
                "camera_quality_csv": csv_status,
                "camera_quality_preview_toggle": toggle,
                "camera_quality_preview_left": preview_handle,
                "camera_quality_last_preview_monotonic": float("-inf"),
            }

            _update_camera_quality(handles)
            self.assertIn("LEFT", status.content)
            self.assertFalse(preview_handle.visible)
            self.assertIsNone(preview_handle.image)

            toggle.value = True
            _update_camera_quality(handles)
            self.assertTrue(preview_handle.visible)
            self.assertIsNotNone(preview_handle.image)
        finally:
            store.close()

    def test_fixed_monitor_shows_right_then_left_blur_and_shake(self) -> None:
        analyzer = CameraQualityAnalyzer("left")
        sample, preview = analyzer.analyze(
            _frame(_textured_image(), index=1, time_sec=10.0),
            None,
            None,
        )
        right = replace(
            sample,
            arm="right",
            blur_effect=0.3214,
            shake_px_s_rms=12.34,
        )
        left = replace(
            sample,
            arm="left",
            blur_effect=0.1234,
            shake_px_s_rms=4.56,
        )
        store = CameraQualityStore(enable_csv=False)
        try:
            store.update(left, preview)
            store.update(right, preview)
            html = _operator_monitor_dynamic_html(
                None,
                stale=True,
                camera_quality_store=store,
                now=10.1,
            )
            camera_html = html[html.index("rb-monitor-body-card rb-monitor-camera-card") :]

            self.assertLess(camera_html.index("RIGHT"), camera_html.index("LEFT"))
            self.assertIn("0.321", camera_html)
            self.assertIn("12.3", camera_html)
            self.assertIn("0.123", camera_html)
            self.assertIn("4.6", camera_html)
            self.assertIn("receiver=live", camera_html)

            stale_html = _operator_monitor_dynamic_html(
                None,
                stale=True,
                camera_quality_store=store,
                now=10.7,
            )
            stale_camera = stale_html[
                stale_html.index("rb-monitor-body-card rb-monitor-camera-card") :
            ]
            self.assertEqual(stale_camera.count("N/A"), 4)
            self.assertNotIn("0.321", stale_camera)
            self.assertNotIn("12.3", stale_camera)
        finally:
            store.close()

    def test_camera_quality_plot_and_history_are_removed(self) -> None:
        package = Path(__file__).resolve().parents[1] / "rb_servo_gui"
        camera_source = (package / "camera_quality.py").read_text(encoding="utf-8")
        app_source = (package / "app.py").read_text(encoding="utf-8")

        self.assertNotIn("seconds from now", camera_source)
        self.assertNotIn("camera_quality_figure", camera_source)
        self.assertNotIn("camera_quality_plot", app_source)
        store = CameraQualityStore(enable_csv=False)
        try:
            self.assertFalse(hasattr(store, "histories"))
        finally:
            store.close()


class RobotMotionTrackerTest(unittest.TestCase):
    @staticmethod
    def _state(time_sec: float, x_m: float, q_deg: float) -> StateSnapshot:
        arm = {
            "mode": "Hold",
            "q_actual_deg": [q_deg] * 6,
            "q_sent_deg": [q_deg + 0.5] * 6,
            "q_previous_sent_deg": [q_deg] * 6,
            "has_valid_joint_state": True,
            "connection_state": "Connected",
            "send_ok": True,
            "has_valid_tcp_pose": True,
            "tcp_actual_valid": True,
            "tcp_stand": {
                "x": x_m,
                "y": 0.0,
                "z": 0.5,
                "rx": 0.0,
                "ry": 0.0,
                "rz": 0.0,
                "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
        }
        parsed = StateSnapshot.parse(
            {
                "schema_version": 1,
                "tick": int(time_sec * 100),
                "left": arm,
                "right": arm,
                "motion_state": "Armed",
            },
            received_monotonic=time_sec,
        )
        assert parsed is not None
        return parsed

    def test_derives_tcp_joint_and_tracking_values(self) -> None:
        tracker = RobotMotionTracker()
        tracker.update(self._state(10.0, 0.0, 0.0))
        tracker.update(self._state(10.1, 0.001, 1.0))

        sample, alignment_ms = tracker.nearest("left", 10.1)

        self.assertIsNotNone(sample)
        assert sample is not None
        self.assertAlmostEqual(sample.tcp_linear_speed_mm_s or 0.0, 10.0)
        self.assertAlmostEqual(sample.tcp_angular_speed_deg_s or 0.0, 0.0)
        self.assertAlmostEqual(sample.joint_speed_rms_deg_s or 0.0, 10.0)
        self.assertAlmostEqual(sample.q_tracking_rms_deg or 0.0, 0.5)
        self.assertAlmostEqual(alignment_ms or 0.0, 0.0)


if __name__ == "__main__":
    unittest.main()
