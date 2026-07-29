"""Read-only wrist-camera blur, motion, and shake diagnostics.

The receiver consumes the existing camera_server ZMQ metadata and POSIX shared
memory rings.  It never publishes a robot command and it never writes image
payloads to disk.  Per-frame scalar diagnostics are logged to CSV so image
quality can be correlated with the already-published robot state.
"""
from __future__ import annotations

import csv
import json
import math
import mmap
import os
import queue
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from html import escape
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from .models import Pose6D, StateSnapshot


CAMERA_QUALITY_SCHEMA = "robotics_lab.camera_quality.v1"
ARMS = ("left", "right")
DEFAULT_TOPICS = {
    "left": "camera.bundle.wrist_left",
    "right": "camera.bundle.wrist_right",
}
STREAM_KEYS = {
    "left": "left_realsense.color",
    "right": "right_realsense.color",
}
ANALYSIS_WIDTH = 320
ANALYSIS_HEIGHT = 240
BLUR_FILTER_SIZE = 11
MIN_TRACKED_FEATURES = 30
MIN_RANSAC_INLIER_RATIO = 0.60
SHAKE_HIGH_PASS_HZ = 2.0
SHAKE_RMS_WINDOW_SEC = 1.0
FLOW_RESET_GAP_SEC = 0.5
BASELINE_SETTLE_SEC = 1.0
BASELINE_COLLECT_SEC = 3.0
BASELINE_TCP_LINEAR_MAX_MM_S = 2.0
BASELINE_TCP_ANGULAR_MAX_DEG_S = 2.0
BASELINE_VISUAL_MOTION_MAX_PX_FRAME = 0.5

_SLOT_HEADER = struct.Struct("<QQQQQQIIIIII")
_COLOR_FORMATS = frozenset({"rgb8", "bgr8", "rgba8", "bgra8"})


def _finite(value: Any) -> float | None:
    if not isinstance(value, int | float):
        return None
    parsed = float(value)
    return parsed if math.isfinite(parsed) else None


def _fmt(value: float | None, digits: int = 2, suffix: str = "") -> str:
    if value is None or not math.isfinite(value):
        return "N/A"
    return f"{value:.{digits}f}{suffix}"


def _quaternion_step_rad(previous: Pose6D, current: Pose6D) -> float | None:
    q0 = previous.quaternion_xyzw
    q1 = current.quaternion_xyzw
    if q0 is None or q1 is None:
        return None
    dot = abs(sum(a * b for a, b in zip(q0, q1)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


@dataclass(frozen=True)
class RobotMotionSample:
    arm: str
    received_monotonic: float
    state_tick: int
    tcp_linear_speed_mm_s: float | None
    tcp_angular_speed_deg_s: float | None
    joint_speed_rms_deg_s: float | None
    q_tracking_rms_deg: float | None
    motion_state: str
    command_mode: str


class RobotMotionTracker:
    """Derive display-only robot motion values from the 100 Hz state stream."""

    def __init__(self, history_sec: float = 2.0) -> None:
        self._lock = threading.Lock()
        self._history_sec = float(history_sec)
        self._history: dict[str, deque[RobotMotionSample]] = {
            arm: deque() for arm in ARMS
        }
        self._previous: dict[str, tuple[float, Pose6D | None, tuple[float, ...] | None]] = {}

    def update(self, snapshot: StateSnapshot) -> None:
        now = float(snapshot.received_monotonic)
        pending: list[RobotMotionSample] = []
        for arm in ARMS:
            state_arm = snapshot.left if arm == "left" else snapshot.right
            pose = state_arm.tcp_actual_stand if state_arm.tcp_actual_valid else None
            q_actual = state_arm.q_actual_deg if state_arm.has_valid_joint_state else None
            linear_speed: float | None = None
            angular_speed: float | None = None
            joint_speed: float | None = None
            previous = self._previous.get(arm)
            if previous is not None:
                previous_time, previous_pose, previous_q = previous
                dt = now - previous_time
                if 1e-4 < dt <= 0.5:
                    if pose is not None and previous_pose is not None:
                        dx = pose.x - previous_pose.x
                        dy = pose.y - previous_pose.y
                        dz = pose.z - previous_pose.z
                        linear_speed = math.sqrt(dx * dx + dy * dy + dz * dz) * 1000.0 / dt
                        angular_step = _quaternion_step_rad(previous_pose, pose)
                        if angular_step is not None:
                            angular_speed = math.degrees(angular_step) / dt
                    if q_actual is not None and previous_q is not None:
                        joint_speed = math.sqrt(
                            sum(((a - b) / dt) ** 2 for a, b in zip(q_actual, previous_q))
                            / len(q_actual)
                        )
            self._previous[arm] = (now, pose, q_actual)

            tracking = None
            if state_arm.q_actual_deg is not None and state_arm.q_sent_deg is not None:
                tracking = math.sqrt(
                    sum(
                        (sent - actual) ** 2
                        for sent, actual in zip(state_arm.q_sent_deg, state_arm.q_actual_deg)
                    )
                    / len(state_arm.q_actual_deg)
                )
            pending.append(
                RobotMotionSample(
                    arm=arm,
                    received_monotonic=now,
                    state_tick=snapshot.tick,
                    tcp_linear_speed_mm_s=linear_speed,
                    tcp_angular_speed_deg_s=angular_speed,
                    joint_speed_rms_deg_s=joint_speed,
                    q_tracking_rms_deg=tracking,
                    motion_state=snapshot.motion_state,
                    command_mode=state_arm.mode,
                )
            )

        with self._lock:
            for sample in pending:
                history = self._history[sample.arm]
                history.append(sample)
                cutoff = now - self._history_sec
                while history and history[0].received_monotonic < cutoff:
                    history.popleft()

    def nearest(
        self,
        arm: str,
        received_monotonic: float,
        *,
        max_delta_sec: float = 0.2,
    ) -> tuple[RobotMotionSample | None, float | None]:
        with self._lock:
            values = tuple(self._history.get(arm, ()))
        if not values:
            return None, None
        sample = min(values, key=lambda item: abs(item.received_monotonic - received_monotonic))
        delta = received_monotonic - sample.received_monotonic
        if abs(delta) > max_delta_sec:
            return None, delta * 1000.0
        return sample, delta * 1000.0


@dataclass(frozen=True)
class CameraFrameInput:
    arm: str
    topic: str
    bundle_seq: int
    camera_name: str
    serial: str
    frame_number: int
    host_arrival_time_ns: int
    received_monotonic: float
    frame_age_ms: float | None
    actual_exposure_us: float | None
    gain_level: float | None
    auto_exposure: bool | None
    pixels_rgb: np.ndarray


@dataclass(frozen=True)
class CameraQualitySample:
    arm: str
    received_monotonic: float
    camera_name: str
    serial: str
    topic: str
    bundle_seq: int
    frame_number: int
    host_arrival_time_ns: int
    frame_age_ms: float | None
    blur_effect: float
    edge_energy: float
    feature_count: int
    tracked_count: int
    inlier_count: int
    inlier_ratio: float | None
    flow_valid: bool
    global_dx_px: float | None
    global_dy_px: float | None
    global_rotation_deg: float | None
    global_motion_px_frame: float | None
    global_motion_px_s: float | None
    shake_px_s_rms: float | None
    baseline_blur_effect: float | None
    blur_delta_from_baseline: float | None
    baseline_state: str
    actual_exposure_us: float | None
    gain_level: float | None
    auto_exposure: bool | None
    estimated_smear_px: float | None
    texture_confidence: str
    state_tick: int | None
    state_alignment_ms: float | None
    tcp_linear_speed_mm_s: float | None
    tcp_angular_speed_deg_s: float | None
    joint_speed_rms_deg_s: float | None
    q_tracking_rms_deg: float | None
    motion_state: str
    command_mode: str
    processing_ms: float

    def csv_row(self) -> dict[str, Any]:
        return {
            "schema": CAMERA_QUALITY_SCHEMA,
            "received_monotonic": f"{self.received_monotonic:.9f}",
            "arm": self.arm,
            "camera_name": self.camera_name,
            "serial": self.serial,
            "topic": self.topic,
            "bundle_seq": self.bundle_seq,
            "frame_number": self.frame_number,
            "host_arrival_time_ns": self.host_arrival_time_ns,
            "frame_age_ms": self.frame_age_ms,
            "blur_effect": self.blur_effect,
            "edge_energy": self.edge_energy,
            "feature_count": self.feature_count,
            "tracked_count": self.tracked_count,
            "inlier_count": self.inlier_count,
            "inlier_ratio": self.inlier_ratio,
            "flow_valid": self.flow_valid,
            "global_dx_px": self.global_dx_px,
            "global_dy_px": self.global_dy_px,
            "global_rotation_deg": self.global_rotation_deg,
            "global_motion_px_frame": self.global_motion_px_frame,
            "global_motion_px_s": self.global_motion_px_s,
            "shake_px_s_rms": self.shake_px_s_rms,
            "baseline_blur_effect": self.baseline_blur_effect,
            "blur_delta_from_baseline": self.blur_delta_from_baseline,
            "baseline_state": self.baseline_state,
            "actual_exposure_us": self.actual_exposure_us,
            "gain_level": self.gain_level,
            "auto_exposure": self.auto_exposure,
            "estimated_smear_px": self.estimated_smear_px,
            "texture_confidence": self.texture_confidence,
            "state_tick": self.state_tick,
            "state_alignment_ms": self.state_alignment_ms,
            "tcp_linear_speed_mm_s": self.tcp_linear_speed_mm_s,
            "tcp_angular_speed_deg_s": self.tcp_angular_speed_deg_s,
            "joint_speed_rms_deg_s": self.joint_speed_rms_deg_s,
            "q_tracking_rms_deg": self.q_tracking_rms_deg,
            "motion_state": self.motion_state,
            "command_mode": self.command_mode,
            "processing_ms": self.processing_ms,
        }


# csv_row contains an explicit transport schema rather than relying on the
# dataclass field order.
CSV_FIELDS = tuple(
    [
        "schema",
        "received_monotonic",
        "arm",
        "camera_name",
        "serial",
        "topic",
        "bundle_seq",
        "frame_number",
        "host_arrival_time_ns",
        "frame_age_ms",
        "blur_effect",
        "edge_energy",
        "feature_count",
        "tracked_count",
        "inlier_count",
        "inlier_ratio",
        "flow_valid",
        "global_dx_px",
        "global_dy_px",
        "global_rotation_deg",
        "global_motion_px_frame",
        "global_motion_px_s",
        "shake_px_s_rms",
        "baseline_blur_effect",
        "blur_delta_from_baseline",
        "baseline_state",
        "actual_exposure_us",
        "gain_level",
        "auto_exposure",
        "estimated_smear_px",
        "texture_confidence",
        "state_tick",
        "state_alignment_ms",
        "tcp_linear_speed_mm_s",
        "tcp_angular_speed_deg_s",
        "joint_speed_rms_deg_s",
        "q_tracking_rms_deg",
        "motion_state",
        "command_mode",
        "processing_ms",
    ]
)


def compute_blur_effect(gray_u8: np.ndarray, h_size: int = BLUR_FILTER_SIZE) -> tuple[float, float]:
    """Crété-Roffet no-reference blur strength plus Sobel edge energy."""
    import cv2

    gray = np.asarray(gray_u8, dtype=np.uint8)
    if gray.ndim != 2 or min(gray.shape) < h_size + 4:
        raise ValueError("blur input must be a sufficiently large grayscale image")
    gray_f = gray.astype(np.float32) / 255.0
    slices = (slice(2, gray.shape[0] - 1), slice(2, gray.shape[1] - 1))
    blur_values: list[float] = []
    edge_terms: list[np.ndarray] = []
    for dx, dy, kernel in ((1, 0, (h_size, 1)), (0, 1, (1, h_size))):
        reblurred = cv2.blur(gray_f, kernel)
        sharp_edge = np.abs(cv2.Sobel(gray_f, cv2.CV_32F, dx, dy, ksize=3))
        blurred_edge = np.abs(cv2.Sobel(reblurred, cv2.CV_32F, dx, dy, ksize=3))
        sharp_edge = np.maximum(np.finfo(np.float32).eps, sharp_edge)
        delta = np.maximum(0.0, sharp_edge - blurred_edge)
        m1 = float(np.sum(sharp_edge[slices]))
        m2 = float(np.sum(delta[slices]))
        blur_values.append(abs(m1 - m2) / m1 if m1 > 0.0 else 1.0)
        edge_terms.append(sharp_edge)
    edge_energy = float(np.mean(edge_terms[0] ** 2 + edge_terms[1] ** 2))
    return float(max(0.0, min(1.0, max(blur_values)))), edge_energy


class CameraQualityAnalyzer:
    def __init__(self, arm: str) -> None:
        if arm not in ARMS:
            raise ValueError(f"unknown arm: {arm}")
        self.arm = arm
        self._previous_gray: np.ndarray | None = None
        self._previous_time_ns: int | None = None
        self._previous_serial: str | None = None
        self._previous_frame_number: int | None = None
        self._lowpass_velocity: np.ndarray | None = None
        self._shake_window: deque[tuple[float, float]] = deque()
        self._stationary_since: float | None = None
        self._baseline_candidates: deque[tuple[float, float]] = deque()
        self._baseline_blur: float | None = None

    def reset(self, *, reset_baseline: bool = True) -> None:
        self._previous_gray = None
        self._previous_time_ns = None
        self._previous_serial = None
        self._previous_frame_number = None
        self._lowpass_velocity = None
        self._shake_window.clear()
        self._stationary_since = None
        self._baseline_candidates.clear()
        if reset_baseline:
            self._baseline_blur = None

    @property
    def baseline_blur(self) -> float | None:
        return self._baseline_blur

    @staticmethod
    def _analysis_gray(pixels_rgb: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        import cv2

        image = np.asarray(pixels_rgb, dtype=np.uint8)
        if image.ndim != 3 or image.shape[2] < 3:
            raise ValueError("camera quality requires an RGB image")
        resized = cv2.resize(
            image[..., :3],
            (ANALYSIS_WIDTH, ANALYSIS_HEIGHT),
            interpolation=cv2.INTER_AREA,
        )
        gray = cv2.cvtColor(resized, cv2.COLOR_RGB2GRAY)
        return resized, gray

    @staticmethod
    def _stationary_robot(robot: RobotMotionSample | None) -> bool:
        if robot is None or robot.tcp_linear_speed_mm_s is None:
            return False
        if robot.tcp_linear_speed_mm_s >= BASELINE_TCP_LINEAR_MAX_MM_S:
            return False
        angular = robot.tcp_angular_speed_deg_s
        if angular is not None:
            return angular < BASELINE_TCP_ANGULAR_MAX_DEG_S
        joint_speed = robot.joint_speed_rms_deg_s
        return joint_speed is not None and joint_speed < BASELINE_TCP_ANGULAR_MAX_DEG_S

    def _update_baseline(
        self,
        now: float,
        blur_effect: float,
        *,
        stationary: bool,
    ) -> str:
        if not stationary:
            self._stationary_since = None
            self._baseline_candidates.clear()
            return "ready/frozen" if self._baseline_blur is not None else "waiting_stationary"
        if self._stationary_since is None:
            self._stationary_since = now
        elapsed = now - self._stationary_since
        if elapsed < BASELINE_SETTLE_SEC:
            return "ready/settling" if self._baseline_blur is not None else "settling"
        self._baseline_candidates.append((now, blur_effect))
        cutoff = now - BASELINE_COLLECT_SEC
        while self._baseline_candidates and self._baseline_candidates[0][0] < cutoff:
            self._baseline_candidates.popleft()
        covered = (
            self._baseline_candidates[-1][0] - self._baseline_candidates[0][0]
            if len(self._baseline_candidates) >= 2
            else 0.0
        )
        if covered >= BASELINE_COLLECT_SEC * 0.9 and len(self._baseline_candidates) >= 30:
            values = np.asarray([value for _, value in self._baseline_candidates], dtype=np.float64)
            # The sharpest robust fifth resists occasional blurred frames without
            # selecting a single noise outlier.
            self._baseline_blur = float(np.percentile(values, 20.0))
            return "ready"
        progress = min(100, int(100.0 * covered / BASELINE_COLLECT_SEC))
        return (
            f"ready/refreshing {progress}%"
            if self._baseline_blur is not None
            else f"learning {progress}%"
        )

    def analyze(
        self,
        frame: CameraFrameInput,
        robot: RobotMotionSample | None,
        state_alignment_ms: float | None,
    ) -> tuple[CameraQualitySample, np.ndarray]:
        import cv2

        started = time.perf_counter()
        preview, gray = self._analysis_gray(frame.pixels_rgb)
        blur_effect, edge_energy = compute_blur_effect(gray)
        current_features = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=200,
            qualityLevel=0.01,
            minDistance=7,
            blockSize=7,
        )
        feature_count = 0 if current_features is None else int(len(current_features))

        serial_changed = (
            self._previous_serial is not None
            and frame.serial
            and frame.serial != self._previous_serial
        )
        frame_counter_restarted = (
            self._previous_frame_number is not None
            and frame.frame_number <= self._previous_frame_number
        )
        dt: float | None = None
        if self._previous_time_ns is not None and frame.host_arrival_time_ns > 0:
            dt = (frame.host_arrival_time_ns - self._previous_time_ns) / 1e9
        if (
            serial_changed
            or frame_counter_restarted
            or dt is not None
            and (dt <= 0.0 or dt > FLOW_RESET_GAP_SEC)
        ):
            self.reset(reset_baseline=True)
            dt = None
        self._previous_serial = frame.serial or None
        self._previous_frame_number = frame.frame_number

        tracked_count = 0
        inlier_count = 0
        inlier_ratio: float | None = None
        flow_valid = False
        dx = dy = rotation_deg = motion_per_frame = motion_per_sec = shake = None
        had_previous = self._previous_gray is not None and dt is not None
        if had_previous:
            previous_points = cv2.goodFeaturesToTrack(
                self._previous_gray,
                maxCorners=200,
                qualityLevel=0.01,
                minDistance=7,
                blockSize=7,
            )
            if previous_points is not None and len(previous_points) >= MIN_TRACKED_FEATURES:
                next_points, status, _ = cv2.calcOpticalFlowPyrLK(
                    self._previous_gray,
                    gray,
                    previous_points,
                    None,
                    winSize=(21, 21),
                    maxLevel=3,
                    criteria=(
                        cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                        30,
                        0.01,
                    ),
                )
                if next_points is not None and status is not None:
                    good = status.reshape(-1) == 1
                    old_good = previous_points.reshape(-1, 2)[good]
                    new_good = next_points.reshape(-1, 2)[good]
                    tracked_count = int(len(old_good))
                    if tracked_count >= MIN_TRACKED_FEATURES:
                        transform, inliers = cv2.estimateAffinePartial2D(
                            old_good,
                            new_good,
                            method=cv2.RANSAC,
                            ransacReprojThreshold=1.5,
                            maxIters=2000,
                            confidence=0.99,
                            refineIters=10,
                        )
                        if inliers is not None:
                            inlier_count = int(np.count_nonzero(inliers))
                            inlier_ratio = inlier_count / tracked_count
                        if (
                            transform is not None
                            and inlier_count >= MIN_TRACKED_FEATURES
                            and inlier_ratio is not None
                            and inlier_ratio >= MIN_RANSAC_INLIER_RATIO
                        ):
                            flow_valid = True
                            dx = float(transform[0, 2])
                            dy = float(transform[1, 2])
                            theta = math.atan2(float(transform[1, 0]), float(transform[0, 0]))
                            rotation_deg = math.degrees(theta)
                            radius = 0.5 * math.hypot(ANALYSIS_WIDTH, ANALYSIS_HEIGHT)
                            vector = np.asarray((dx, dy, theta * radius), dtype=np.float64)
                            motion_per_frame = float(np.linalg.norm(vector))
                            velocity = vector / dt
                            motion_per_sec = float(np.linalg.norm(velocity))
                            tau = 1.0 / (2.0 * math.pi * SHAKE_HIGH_PASS_HZ)
                            alpha = 1.0 - math.exp(-dt / tau)
                            if self._lowpass_velocity is None:
                                self._lowpass_velocity = velocity.copy()
                            else:
                                self._lowpass_velocity += alpha * (
                                    velocity - self._lowpass_velocity
                                )
                            residual = velocity - self._lowpass_velocity
                            self._shake_window.append(
                                (frame.received_monotonic, float(np.dot(residual, residual)))
                            )
                            cutoff = frame.received_monotonic - SHAKE_RMS_WINDOW_SEC
                            while self._shake_window and self._shake_window[0][0] < cutoff:
                                self._shake_window.popleft()
                            if self._shake_window:
                                shake = math.sqrt(
                                    sum(value for _, value in self._shake_window)
                                    / len(self._shake_window)
                                )

        self._previous_gray = gray
        self._previous_time_ns = frame.host_arrival_time_ns

        if feature_count < MIN_TRACKED_FEATURES:
            texture_confidence = "low_texture"
        elif not had_previous:
            texture_confidence = "initializing"
        elif flow_valid:
            texture_confidence = "ok"
        else:
            texture_confidence = "flow_unreliable"
        stationary = (
            flow_valid
            and motion_per_frame is not None
            and motion_per_frame < BASELINE_VISUAL_MOTION_MAX_PX_FRAME
            and feature_count >= MIN_TRACKED_FEATURES
            and self._stationary_robot(robot)
        )
        baseline_state = self._update_baseline(
            frame.received_monotonic,
            blur_effect,
            stationary=stationary,
        )
        baseline = self._baseline_blur
        blur_delta = None if baseline is None else blur_effect - baseline
        exposure = frame.actual_exposure_us
        smear = (
            motion_per_sec * exposure / 1e6
            if motion_per_sec is not None and exposure is not None and exposure >= 0.0
            else None
        )
        processing_ms = (time.perf_counter() - started) * 1000.0
        sample = CameraQualitySample(
            arm=self.arm,
            received_monotonic=frame.received_monotonic,
            camera_name=frame.camera_name,
            serial=frame.serial,
            topic=frame.topic,
            bundle_seq=frame.bundle_seq,
            frame_number=frame.frame_number,
            host_arrival_time_ns=frame.host_arrival_time_ns,
            frame_age_ms=frame.frame_age_ms,
            blur_effect=blur_effect,
            edge_energy=edge_energy,
            feature_count=feature_count,
            tracked_count=tracked_count,
            inlier_count=inlier_count,
            inlier_ratio=inlier_ratio,
            flow_valid=flow_valid,
            global_dx_px=dx,
            global_dy_px=dy,
            global_rotation_deg=rotation_deg,
            global_motion_px_frame=motion_per_frame,
            global_motion_px_s=motion_per_sec,
            shake_px_s_rms=shake,
            baseline_blur_effect=baseline,
            blur_delta_from_baseline=blur_delta,
            baseline_state=baseline_state,
            actual_exposure_us=exposure,
            gain_level=frame.gain_level,
            auto_exposure=frame.auto_exposure,
            estimated_smear_px=smear,
            texture_confidence=texture_confidence,
            state_tick=None if robot is None else robot.state_tick,
            state_alignment_ms=state_alignment_ms,
            tcp_linear_speed_mm_s=None if robot is None else robot.tcp_linear_speed_mm_s,
            tcp_angular_speed_deg_s=None if robot is None else robot.tcp_angular_speed_deg_s,
            joint_speed_rms_deg_s=None if robot is None else robot.joint_speed_rms_deg_s,
            q_tracking_rms_deg=None if robot is None else robot.q_tracking_rms_deg,
            motion_state="" if robot is None else robot.motion_state,
            command_mode="" if robot is None else robot.command_mode,
            processing_ms=processing_ms,
        )
        return sample, preview


class CameraQualityStore:
    def __init__(
        self,
        *,
        csv_dir: str | Path = "logs/camera_quality",
        enable_csv: bool = True,
        now: datetime | None = None,
    ) -> None:
        self._lock = threading.Lock()
        self._latest: dict[str, CameraQualitySample] = {}
        self._previews: dict[str, np.ndarray] = {}
        self._receiver_status = "starting"
        self._receiver_error = ""
        self._dropped: dict[str, int] = {arm: 0 for arm in ARMS}
        self._reset_generation = 0
        self._csv_file = None
        self._csv_writer: csv.DictWriter | None = None
        self._csv_path: Path | None = None
        self._csv_error = ""
        self._last_flush = time.monotonic()
        if enable_csv:
            try:
                base = Path(csv_dir)
                base.mkdir(parents=True, exist_ok=True)
                stamp_time = now or datetime.now(timezone(timedelta(hours=9)))
                stamp = stamp_time.strftime("%Y%m%d_%H%M%S")
                path = base / f"camera_quality_{stamp}.csv"
                for duplicate_index in range(100):
                    candidate = (
                        path
                        if duplicate_index == 0
                        else path.with_name(f"{path.stem}_{duplicate_index}{path.suffix}")
                    )
                    try:
                        self._csv_file = candidate.open(
                            "x",
                            newline="",
                            encoding="utf-8",
                        )
                        path = candidate
                        break
                    except FileExistsError:
                        continue
                if self._csv_file is None:
                    raise OSError(f"no unused camera-quality CSV filename for {stamp}")
                self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=CSV_FIELDS)
                self._csv_writer.writeheader()
                self._csv_file.flush()
                self._csv_path = path
            except OSError as exc:
                self._csv_error = f"{type(exc).__name__}: {exc}"

    @property
    def csv_path(self) -> Path | None:
        with self._lock:
            return self._csv_path

    @property
    def csv_error(self) -> str:
        with self._lock:
            return self._csv_error

    @property
    def reset_generation(self) -> int:
        with self._lock:
            return self._reset_generation

    def request_baseline_reset(self) -> None:
        with self._lock:
            self._reset_generation += 1

    def set_receiver_status(self, status: str, error: str = "") -> None:
        with self._lock:
            self._receiver_status = str(status)
            self._receiver_error = str(error)

    def note_drop(self, arm: str) -> None:
        with self._lock:
            self._dropped[arm] = self._dropped.get(arm, 0) + 1

    def update(self, sample: CameraQualitySample, preview: np.ndarray) -> None:
        with self._lock:
            self._latest[sample.arm] = sample
            self._previews[sample.arm] = preview
            if self._csv_writer is not None:
                try:
                    self._csv_writer.writerow(sample.csv_row())
                    now = time.monotonic()
                    if now - self._last_flush >= 1.0:
                        assert self._csv_file is not None
                        self._csv_file.flush()
                        self._last_flush = now
                except OSError as exc:
                    self._csv_error = f"{type(exc).__name__}: {exc}"
                    self._csv_writer = None
            self._receiver_status = "live"
            self._receiver_error = ""

    def latest(self, arm: str) -> CameraQualitySample | None:
        with self._lock:
            return self._latest.get(arm)

    def preview(self, arm: str) -> np.ndarray | None:
        with self._lock:
            return self._previews.get(arm)

    def status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "receiver": self._receiver_status,
                "error": self._receiver_error,
                "dropped": dict(self._dropped),
                "csv_path": None if self._csv_path is None else str(self._csv_path),
                "csv_error": self._csv_error,
            }

    def close(self) -> None:
        with self._lock:
            if self._csv_file is not None:
                try:
                    self._csv_file.flush()
                    self._csv_file.close()
                except OSError as exc:
                    self._csv_error = f"{type(exc).__name__}: {exc}"
                self._csv_file = None
                self._csv_writer = None


class _ShmCache:
    def __init__(self) -> None:
        self._name: str | None = None
        self._file = None
        self._mmap: mmap.mmap | None = None

    def get(self, name: str) -> mmap.mmap:
        if self._name == name and self._mmap is not None:
            return self._mmap
        self.close()
        path = "/dev/shm/" + name.lstrip("/")
        self._file = open(path, "rb")
        self._mmap = mmap.mmap(self._file.fileno(), 0, access=mmap.ACCESS_READ)
        self._name = name
        return self._mmap

    def close(self) -> None:
        if self._mmap is not None:
            self._mmap.close()
            self._mmap = None
        if self._file is not None:
            self._file.close()
            self._file = None
        self._name = None


def _read_rgb_frame(cache: _ShmCache, meta: Mapping[str, Any]) -> np.ndarray:
    shm_name = str(meta["shm_name"])
    width = int(meta["width"])
    height = int(meta["height"])
    stride = int(meta["stride_bytes"])
    fmt = str(meta.get("format", "")).lower()
    offset = int(meta["shm_offset"])
    size_bytes = int(meta["size_bytes"])
    if fmt not in _COLOR_FORMATS:
        raise ValueError(f"unsupported camera quality format: {fmt}")
    mm = cache.get(shm_name)
    header_offset = offset - _SLOT_HEADER.size
    if header_offset < 0 or offset + size_bytes > len(mm):
        raise RuntimeError("camera quality shm offsets out of bounds")
    payload: bytes | None = None
    for _ in range(1000):
        start = struct.unpack_from("<Q", mm, header_offset)[0]
        if start & 1:
            continue
        values = _SLOT_HEADER.unpack_from(mm, header_offset)
        slot_size = int(values[9])
        valid = int(values[10])
        if slot_size != size_bytes:
            raise RuntimeError("camera quality shm slot size mismatch")
        copied = bytes(mm[offset:offset + size_bytes])
        end = int(values[1])
        reread = struct.unpack_from("<Q", mm, header_offset)[0]
        if valid and start == end == reread and not end & 1:
            payload = copied
            break
    if payload is None:
        raise RuntimeError("camera quality shm seqlock retry exhausted")
    channels = 4 if fmt in {"rgba8", "bgra8"} else 3
    row_bytes = width * channels
    if stride < row_bytes or len(payload) != height * stride:
        raise RuntimeError("camera quality frame dimensions do not match payload")
    array = np.frombuffer(payload, dtype=np.uint8)
    pixels = array.reshape(height, stride)[:, :row_bytes].reshape(height, width, channels)
    if fmt in {"bgr8", "bgra8"}:
        pixels = pixels[..., [2, 1, 0] + ([3] if channels == 4 else [])]
    return np.ascontiguousarray(pixels[..., :3])


class CameraQualityReceiver:
    """ZMQ/shm reader plus one latest-only analysis worker per arm."""

    def __init__(
        self,
        store: CameraQualityStore,
        robot_motion: RobotMotionTracker,
        *,
        endpoint: str = "tcp://127.0.0.1:5600",
        topics: Mapping[str, str] | None = None,
    ) -> None:
        self.store = store
        self.robot_motion = robot_motion
        self.endpoint = str(endpoint)
        self.topics = dict(DEFAULT_TOPICS if topics is None else topics)
        self._topic_to_arm = {topic: arm for arm, topic in self.topics.items()}
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._workers: list[threading.Thread] = []
        self._queues: dict[str, queue.Queue[CameraFrameInput]] = {
            arm: queue.Queue(maxsize=1) for arm in ARMS
        }

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        try:
            import cv2  # noqa: F401
            import zmq  # noqa: F401
        except ModuleNotFoundError as exc:
            self.store.set_receiver_status(
                "disabled",
                f"missing dependency {exc.name}; install rb_gui camera-quality dependencies",
            )
            return
        self._stop.clear()
        for arm in ARMS:
            worker = threading.Thread(
                target=self._worker,
                args=(arm,),
                name=f"rb-camera-quality-{arm}",
                daemon=True,
            )
            worker.start()
            self._workers.append(worker)
        self._thread = threading.Thread(
            target=self._run,
            name="rb-camera-quality-receiver",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        for worker in self._workers:
            worker.join(timeout=2.0)
        self._workers.clear()

    def _put_latest(self, frame: CameraFrameInput) -> None:
        target = self._queues[frame.arm]
        try:
            target.put_nowait(frame)
            return
        except queue.Full:
            pass
        try:
            target.get_nowait()
        except queue.Empty:
            pass
        self.store.note_drop(frame.arm)
        try:
            target.put_nowait(frame)
        except queue.Full:
            self.store.note_drop(frame.arm)

    def _worker(self, arm: str) -> None:
        analyzer = CameraQualityAnalyzer(arm)
        reset_generation = self.store.reset_generation
        while not self._stop.is_set():
            try:
                frame = self._queues[arm].get(timeout=0.2)
            except queue.Empty:
                continue
            current_generation = self.store.reset_generation
            if current_generation != reset_generation:
                analyzer.reset(reset_baseline=True)
                reset_generation = current_generation
            robot, alignment_ms = self.robot_motion.nearest(
                arm,
                frame.received_monotonic,
            )
            try:
                sample, preview = analyzer.analyze(frame, robot, alignment_ms)
            except Exception as exc:  # keep the other arm and GUI alive
                self.store.set_receiver_status(
                    "degraded",
                    f"{arm} analysis {type(exc).__name__}: {exc}",
                )
                continue
            self.store.update(sample, preview)

    def _run(self) -> None:
        import zmq

        cache = _ShmCache()
        context = zmq.Context.instance()
        sock = context.socket(zmq.SUB)
        sock.setsockopt(zmq.RCVHWM, 60)
        for topic in self._topic_to_arm:
            sock.setsockopt_string(zmq.SUBSCRIBE, topic)
        sock.connect(self.endpoint)
        self.store.set_receiver_status("waiting")
        try:
            while not self._stop.is_set():
                if not sock.poll(timeout=200, flags=zmq.POLLIN):
                    continue
                try:
                    parts = sock.recv_multipart(flags=zmq.NOBLOCK)
                except zmq.Again:
                    continue
                if len(parts) != 2:
                    continue
                topic = parts[0].decode("utf-8", errors="replace")
                arm = self._topic_to_arm.get(topic)
                if arm is None:
                    continue
                try:
                    document = json.loads(parts[1].decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    continue
                if (
                    not isinstance(document, Mapping)
                    or document.get("schema") != "camera_server.bundle.v1"
                    or document.get("complete") is not True
                ):
                    continue
                frames = document.get("frames")
                if not isinstance(frames, Mapping):
                    continue
                meta = frames.get(STREAM_KEYS[arm])
                if not isinstance(meta, Mapping) or meta.get("valid") is not True:
                    continue
                try:
                    pixels = _read_rgb_frame(cache, meta)
                    host_arrival = int(meta.get("host_arrival_time_ns", 0) or 0)
                    age_ms = None
                    if host_arrival > 0:
                        raw_now = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
                        age_ns = raw_now - host_arrival
                        if age_ns >= 0:
                            age_ms = age_ns / 1e6
                    auto_exposure_raw = meta.get("auto_exposure")
                    auto_exposure = (
                        auto_exposure_raw if isinstance(auto_exposure_raw, bool) else None
                    )
                    frame = CameraFrameInput(
                        arm=arm,
                        topic=topic,
                        bundle_seq=int(document.get("bundle_seq", 0) or 0),
                        camera_name=str(meta.get("camera_name", "")),
                        serial=str(meta.get("serial", "")),
                        frame_number=int(meta.get("frame_number", 0) or 0),
                        host_arrival_time_ns=host_arrival,
                        received_monotonic=time.monotonic(),
                        frame_age_ms=age_ms,
                        actual_exposure_us=_finite(meta.get("actual_exposure_us")),
                        gain_level=_finite(meta.get("gain_level")),
                        auto_exposure=auto_exposure,
                        pixels_rgb=pixels,
                    )
                except Exception as exc:
                    self.store.set_receiver_status(
                        "degraded",
                        f"{arm} frame read {type(exc).__name__}: {exc}",
                    )
                    continue
                self._put_latest(frame)
        except Exception as exc:
            self.store.set_receiver_status(
                "unavailable",
                f"{type(exc).__name__}: {exc}",
            )
        finally:
            sock.close(linger=0)
            cache.close()


def camera_quality_html(store: CameraQualityStore, *, now: float | None = None) -> str:
    timestamp = time.monotonic() if now is None else float(now)
    status = store.status()
    cards: list[str] = []
    for arm in ARMS:
        sample = store.latest(arm)
        if sample is None:
            headline = "데이터 대기 중"
            details = (
                f"topic {DEFAULT_TOPICS[arm]}\n"
                f"receiver={status['receiver']} {status['error']}"
            )
            tone = "#9aa4b2"
        else:
            age = timestamp - sample.received_monotonic
            stale = age > 0.5
            headline = (
                f"Blur {_fmt(sample.blur_effect, 3)} · "
                f"Shake {_fmt(sample.shake_px_s_rms, 1, ' px/s')}"
            )
            exposure_ms = (
                None if sample.actual_exposure_us is None else sample.actual_exposure_us / 1000.0
            )
            details = (
                f"baseline {_fmt(sample.baseline_blur_effect, 3)} · "
                f"Δ {_fmt(sample.blur_delta_from_baseline, 3)} · {sample.baseline_state}\n"
                f"motion {_fmt(sample.global_motion_px_s, 1, ' px/s')} · "
                f"smear {_fmt(sample.estimated_smear_px, 2, ' px')} · "
                f"exposure {_fmt(exposure_ms, 2, ' ms')} · gain {_fmt(sample.gain_level, 1)}\n"
                f"features {sample.feature_count} · inlier "
                f"{_fmt(None if sample.inlier_ratio is None else sample.inlier_ratio * 100.0, 0, '%')} · "
                f"{sample.texture_confidence} · age {_fmt(sample.frame_age_ms, 1, ' ms')}\n"
                f"TCP {_fmt(sample.tcp_linear_speed_mm_s, 1, ' mm/s')} / "
                f"{_fmt(sample.tcp_angular_speed_deg_s, 1, ' deg/s')} · "
                f"q error {_fmt(sample.q_tracking_rms_deg, 3, ' deg')}"
            )
            tone = "#dc4646" if stale else "#e1a01e" if sample.texture_confidence != "ok" else "#2563eb"
        cards.append(
            '<div style="flex:1 1 24em;min-width:21em;border-radius:0.55em;'
            'padding:0.65em 0.75em;margin:0.2em;background:#20252e;color:#dfe5ee;'
            'font-family:system-ui,-apple-system,sans-serif;">'
            f'<div style="font-weight:700;"><span style="color:{tone};">●</span> '
            f'{escape(arm.upper())} · {escape(headline)}</div>'
            f'<div style="font-size:11px;line-height:1.5;white-space:pre-line;margin-top:0.3em;">'
            f'{escape(details)}</div></div>'
        )
    csv_text = status["csv_path"] or "CSV disabled"
    if status["csv_error"]:
        csv_text += " · ERROR " + status["csv_error"]
    footer = (
        '<div style="width:100%;font-size:10px;color:#8993a1;margin:0.2em 0.45em;">'
        f'CSV: {escape(csv_text)} · analyzer drops L/R '
        f'{status["dropped"].get("left", 0)}/{status["dropped"].get("right", 0)} · '
        "diagnostic only, motion gate에 사용하지 않음</div>"
    )
    return '<div style="display:flex;flex-wrap:wrap;margin:0.2em -0.2em;">' + "".join(cards) + footer + "</div>"
