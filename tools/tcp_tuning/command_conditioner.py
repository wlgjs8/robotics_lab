from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from . import se3
from .config import Config, ConditioningConfig, SmoothingConfig, SyntheticConfig
from .smoothing import Segment, smooth_pose_segments, split_segments


MODES = {"raw_zoh", "raw_foh_se3", "clean_foh_se3", "synthetic_policy_surrogate"}
NAN_POSE = np.full(7, np.nan, dtype=np.float64)
NAN_TWIST = np.full(6, np.nan, dtype=np.float64)


@dataclass
class ConditionedCommand:
    t_servo: float
    left_pose: np.ndarray
    right_pose: np.ndarray
    left_twist: np.ndarray | None
    right_twist: np.ndarray | None
    left_gripper: float | None
    right_gripper: float | None
    valid: bool
    hold: bool
    dropout: bool
    gap: bool
    reanchor: bool
    src_ids: tuple
    meta: dict


class CommandConditioner:
    def __init__(self, mode: str, cfg: "ConditioningConfig"):
        if mode not in MODES:
            raise ValueError(f"unknown command conditioning mode: {mode}")
        self.mode = mode
        self._root_cfg = cfg
        if isinstance(cfg, Config) or hasattr(cfg, "conditioning"):
            self.cfg = cfg.conditioning
            self.smoothing_cfg = cfg.smoothing
            self.synthetic_cfg = cfg.synthetic
            self.config_dump = cfg.to_dict() if hasattr(cfg, "to_dict") else {}
        else:
            self.cfg = cfg
            self.smoothing_cfg = SmoothingConfig()
            self.synthetic_cfg = SyntheticConfig()
            self.config_dump = {}
        self.reset()

    def reset(self, *, current_left_pose=None, current_right_pose=None):
        self._samples: list[dict[str, Any]] = []
        self._prepared = False
        self._current_left_pose = _canonical_pose_or_none(current_left_pose)
        self._current_right_pose = _canonical_pose_or_none(current_right_pose)
        self.meta: dict[str, Any] = {
            "mode": self.mode,
            "servo_rate_hz": float(self.cfg.servo_rate_hz),
            "nominal_source_rate_hz": float(self.cfg.nominal_source_rate_hz),
            "nominal_rate_used": False,
        }

    def update_source_sample(
        self,
        t_source,
        pose_left,
        pose_right,
        gripper_left=None,
        gripper_right=None,
        metadata=None,
    ):
        if t_source is None:
            index = len(self._samples)
            t_value = float(index) / float(self.cfg.nominal_source_rate_hz)
            self.meta["nominal_rate_used"] = True
        else:
            t_value = float(t_source)
        self._samples.append(
            {
                "t": t_value,
                "left_pose": _canonical_pose_or_none(pose_left),
                "right_pose": _canonical_pose_or_none(pose_right),
                "left_gripper": _float_or_none(gripper_left),
                "right_gripper": _float_or_none(gripper_right),
                "metadata": dict(metadata or {}),
            }
        )
        self._prepared = False

    def sample(self, t_servo) -> ConditionedCommand:
        self._prepare()
        t_value = float(t_servo)
        if self._t.size == 0:
            return ConditionedCommand(
                t_servo=t_value,
                left_pose=NAN_POSE.copy(),
                right_pose=NAN_POSE.copy(),
                left_twist=None,
                right_twist=None,
                left_gripper=None,
                right_gripper=None,
                valid=False,
                hold=True,
                dropout=False,
                gap=False,
                reanchor=False,
                src_ids=(-1, -1),
                meta={"reason": "no_source_samples", **self.meta},
            )
        lo, hi = self._source_interval(t_value)
        gap = self._interval_crosses_gap(lo, hi, t_value)
        reanchor = self._is_reanchor_tick(hi, t_value)
        if self.mode == "raw_zoh":
            left_pose = _pose_at(self._raw_left, lo)
            right_pose = _pose_at(self._raw_right, lo)
            left_twist = None
            right_twist = None
            src_ids = (lo, lo)
            hold = True
        elif self.mode == "raw_foh_se3":
            left_pose, left_twist = self._foh_arm(self._raw_left, lo, hi, t_value, allow_gap_interp=True)
            right_pose, right_twist = self._foh_arm(self._raw_right, lo, hi, t_value, allow_gap_interp=True)
            src_ids = (lo, hi)
            hold = lo == hi
        else:
            if gap and self.cfg.reanchor_after_gap:
                left_pose = _pose_at(self._clean_left, lo)
                right_pose = _pose_at(self._clean_right, lo)
                left_twist = None
                right_twist = None
                src_ids = (lo, lo)
                hold = True
            else:
                left_pose, left_twist = self._foh_arm(self._clean_left, lo, hi, t_value, allow_gap_interp=False)
                right_pose, right_twist = self._foh_arm(self._clean_right, lo, hi, t_value, allow_gap_interp=False)
                src_ids = (lo, hi)
                hold = lo == hi
            if self.mode == "synthetic_policy_surrogate":
                synth = self._synthetic_at(t_value, left_pose, right_pose, left_twist, right_twist)
                left_pose = synth["left_pose"]
                right_pose = synth["right_pose"]
                left_twist = synth["left_twist"]
                right_twist = synth["right_twist"]
                hold = hold or synth["dropout"]
                dropout = synth["dropout"]
            else:
                dropout = False
        if self.mode != "synthetic_policy_surrogate":
            dropout = False

        left_gripper = _gripper_at(self._left_gripper, lo)
        right_gripper = _gripper_at(self._right_gripper, lo)
        valid = np.isfinite(left_pose).all() or np.isfinite(right_pose).all()
        return ConditionedCommand(
            t_servo=t_value,
            left_pose=left_pose,
            right_pose=right_pose,
            left_twist=left_twist,
            right_twist=right_twist,
            left_gripper=left_gripper,
            right_gripper=right_gripper,
            valid=bool(valid and not dropout),
            hold=bool(hold),
            dropout=bool(dropout),
            gap=bool(gap),
            reanchor=bool(reanchor),
            src_ids=tuple(int(item) for item in src_ids),
            meta={
                **self.meta,
                "gap": bool(gap),
                "segment_index": self._segment_index_for_source(lo),
                "source_t_lo": float(self._t[lo]),
                "source_t_hi": float(self._t[hi]),
            },
        )

    @property
    def segments(self) -> list[Segment]:
        self._prepare()
        return list(self._segments)

    @property
    def gaps(self) -> np.ndarray:
        self._prepare()
        return self._gaps.copy()

    @property
    def clean_source(self) -> dict[str, np.ndarray | None]:
        self._prepare()
        return {"t_source": self._t.copy(), "left_pose": _copy_or_none(self._clean_left), "right_pose": _copy_or_none(self._clean_right)}

    def _prepare(self) -> None:
        if self._prepared:
            return
        self._samples.sort(key=lambda item: item["t"])
        self._t = np.asarray([item["t"] for item in self._samples], dtype=np.float64)
        self._raw_left = _stack_poses([item["left_pose"] for item in self._samples])
        self._raw_right = _stack_poses([item["right_pose"] for item in self._samples])
        self._left_gripper = _stack_gripper([item["left_gripper"] for item in self._samples])
        self._right_gripper = _stack_gripper([item["right_gripper"] for item in self._samples])
        self._segments, self._gaps = split_segments(
            self._t,
            median_multiplier=self.cfg.gap_median_multiplier,
            absolute_threshold_sec=self.cfg.gap_absolute_threshold_sec,
        )
        self._gap_after = {stop - 1 for _, stop in self._segments[:-1]}
        self._segment_start = {start for start, _ in self._segments[1:]}
        self._clean_left = self._smooth_arm(self._raw_left)
        self._clean_right = self._smooth_arm(self._raw_right)
        self._rng = np.random.default_rng(int(self.synthetic_cfg.seed))
        self._synthetic_state = self._build_synthetic_state()
        self.meta.update(
            {
                "source_sample_count": int(self._t.size),
                "segments": [(int(start), int(stop)) for start, stop in self._segments],
                "gap_count": int(self._gaps.shape[0]),
                "smoothing": self.smoothing_cfg.__dict__.copy(),
                "synthetic": self.synthetic_cfg.__dict__.copy(),
            }
        )
        self._prepared = True

    def _smooth_arm(self, poses: np.ndarray | None) -> np.ndarray | None:
        if poses is None:
            return None
        return smooth_pose_segments(self._t, poses, self._segments, self.smoothing_cfg, endpoint_mode="preserve")

    def _source_interval(self, t_value: float) -> tuple[int, int]:
        if t_value <= self._t[0]:
            return 0, 0 if self._t.size == 1 else 1
        if t_value >= self._t[-1]:
            last = int(self._t.size - 1)
            return last, last
        lo = int(np.searchsorted(self._t, t_value, side="right") - 1)
        hi = min(lo + 1, int(self._t.size - 1))
        return lo, hi

    def _interval_crosses_gap(self, lo: int, hi: int, t_value: float) -> bool:
        if lo == hi:
            return False
        if lo in self._gap_after:
            return True
        return float(self._t[hi] - self._t[lo]) > float(self.cfg.gap_absolute_threshold_sec)

    def _is_reanchor_tick(self, hi: int, t_value: float) -> bool:
        if hi not in self._segment_start:
            return False
        return abs(float(t_value) - float(self._t[hi])) <= 0.5 / float(self.cfg.servo_rate_hz)

    def _foh_arm(self, poses: np.ndarray | None, lo: int, hi: int, t_value: float, *, allow_gap_interp: bool) -> tuple[np.ndarray, np.ndarray | None]:
        if poses is None:
            return NAN_POSE.copy(), None
        if lo == hi or (not allow_gap_interp and lo in self._gap_after):
            return _pose_at(poses, lo), None
        if not (np.isfinite(poses[lo]).all() and np.isfinite(poses[hi]).all()):
            return _pose_at(poses, lo), None
        p, q = se3.foh_pose(t_value, self._t[lo], poses[lo, :3], poses[lo, 3:7], self._t[hi], poses[hi, :3], poses[hi, 3:7])
        dt = float(self._t[hi] - self._t[lo])
        twist = None
        if dt > 0.0 and np.isfinite(dt):
            v, w = se3.twist_from_poses(poses[lo, :3], poses[lo, 3:7], poses[hi, :3], poses[hi, 3:7], dt)
            twist = np.concatenate([v, w]).astype(np.float64)
        return np.concatenate([p, q]).astype(np.float64), twist

    def _segment_index_for_source(self, index: int) -> int | None:
        for segment_index, (start, stop) in enumerate(self._segments):
            if start <= index < stop:
                return int(segment_index)
        return None

    def _build_synthetic_state(self) -> dict[str, Any]:
        if self._t.size == 0:
            return {}
        cfg = self.synthetic_cfg
        rng = np.random.default_rng(int(cfg.seed))
        state: dict[str, Any] = {
            "phase_pos": rng.uniform(0.0, 2.0 * np.pi, size=3),
            "phase_ori": rng.uniform(0.0, 2.0 * np.pi, size=3),
            "freq": rng.uniform(float(cfg.oscillation_hz_min), float(cfg.oscillation_hz_max)),
            "dropout": {},
        }
        return state

    def _synthetic_at(self, t_value: float, left_pose, right_pose, left_twist, right_twist) -> dict[str, Any]:
        left = self._apply_synthetic_pose("left", t_value, left_pose)
        right = self._apply_synthetic_pose("right", t_value, right_pose)
        dropout = bool(left["dropout"] or right["dropout"])
        if left["dropout"]:
            left_pose = left["pose"]
            left_twist = None
        else:
            left_pose = left["pose"]
        if right["dropout"]:
            right_pose = right["pose"]
            right_twist = None
        else:
            right_pose = right["pose"]
        return {
            "left_pose": left_pose,
            "right_pose": right_pose,
            "left_twist": left_twist,
            "right_twist": right_twist,
            "dropout": dropout,
        }

    def _apply_synthetic_pose(self, arm: str, t_value: float, pose: np.ndarray) -> dict[str, Any]:
        if not np.isfinite(pose).all():
            return {"pose": pose, "dropout": False}
        cfg = self.synthetic_cfg
        rng = self._rng
        tick = int(round((t_value - float(self._t[0])) * float(self.cfg.servo_rate_hz)))
        dropout_state = self._synthetic_state.setdefault("dropout", {}).setdefault(arm, {"until": -1, "last_pose": pose.copy()})
        if tick <= int(dropout_state.get("until", -1)) and self.cfg.hold_on_dropout:
            return {"pose": np.asarray(dropout_state["last_pose"], dtype=np.float64).copy(), "dropout": True}
        if float(cfg.dropout_probability) > 0.0 and rng.random() < float(cfg.dropout_probability) / float(self.cfg.servo_rate_hz):
            frames = int(rng.integers(int(cfg.dropout_min_frames), int(cfg.dropout_max_frames) + 1))
            dropout_state["until"] = tick + max(1, frames) - 1
            dropout_state["last_pose"] = pose.copy()
            return {"pose": pose.copy(), "dropout": True}

        out = pose.copy()
        pos_sigma = float(cfg.position_noise_rms_mm) / 1000.0
        ori_sigma = np.deg2rad(float(cfg.orientation_noise_rms_deg))
        if pos_sigma > 0.0:
            out[:3] += rng.normal(0.0, pos_sigma, size=3)
        if ori_sigma > 0.0:
            noise_rot = Rotation.from_rotvec(rng.normal(0.0, ori_sigma, size=3))
            out[3:7] = se3.rotation_to_quat(noise_rot * Rotation.from_quat(out[3:7]), ref=out[3:7])

        osc_pos = float(cfg.oscillation_position_rms_mm) / 1000.0
        osc_ori = np.deg2rad(float(cfg.oscillation_orientation_rms_deg))
        freq = float(self._synthetic_state.get("freq", cfg.oscillation_hz_min))
        if osc_pos > 0.0:
            phase = np.asarray(self._synthetic_state.get("phase_pos", np.zeros(3)), dtype=np.float64)
            out[:3] += osc_pos * np.sin(2.0 * np.pi * freq * t_value + phase)
        if osc_ori > 0.0:
            phase = np.asarray(self._synthetic_state.get("phase_ori", np.zeros(3)), dtype=np.float64)
            rotvec = osc_ori * np.sin(2.0 * np.pi * freq * t_value + phase)
            out[3:7] = se3.rotation_to_quat(Rotation.from_rotvec(rotvec) * Rotation.from_quat(out[3:7]), ref=out[3:7])

        chunk = max(1, int(cfg.chunk_size_source_frames))
        if chunk and self._t.size:
            idx = int(np.searchsorted(self._t, t_value, side="right") - 1)
            if idx > 0 and idx % chunk == 0 and abs(t_value - float(self._t[idx])) <= 0.5 / float(self.cfg.servo_rate_hz):
                out[:3] += rng.normal(0.0, float(cfg.chunk_boundary_position_step_mm) / 1000.0, size=3)
                step_ori = np.deg2rad(float(cfg.chunk_boundary_orientation_step_deg))
                out[3:7] = se3.rotation_to_quat(
                    Rotation.from_rotvec(rng.normal(0.0, step_ori, size=3)) * Rotation.from_quat(out[3:7]),
                    ref=out[3:7],
                )
        dropout_state["last_pose"] = out.copy()
        return {"pose": out.astype(np.float64), "dropout": False}


def _canonical_pose_or_none(pose) -> np.ndarray | None:
    if pose is None:
        return None
    arr = np.asarray(pose, dtype=np.float64).reshape(7).copy()
    arr[3:7] = se3.quat_canonical(arr[3:7])
    return arr


def _float_or_none(value) -> float | None:
    if value is None:
        return None
    return float(value)


def _stack_poses(items: list[np.ndarray | None]) -> np.ndarray | None:
    if not items or all(item is None for item in items):
        return None
    out = np.full((len(items), 7), np.nan, dtype=np.float64)
    for index, item in enumerate(items):
        if item is not None:
            out[index] = item
    return out


def _stack_gripper(items: list[float | None]) -> np.ndarray | None:
    if not items or all(item is None for item in items):
        return None
    out = np.full(len(items), np.nan, dtype=np.float64)
    for index, item in enumerate(items):
        if item is not None:
            out[index] = item
    return out


def _pose_at(poses: np.ndarray | None, index: int) -> np.ndarray:
    if poses is None:
        return NAN_POSE.copy()
    return np.asarray(poses[int(index)], dtype=np.float64).copy()


def _gripper_at(values: np.ndarray | None, index: int) -> float | None:
    if values is None:
        return None
    value = float(values[int(index)])
    return value if np.isfinite(value) else None


def _copy_or_none(values: np.ndarray | None) -> np.ndarray | None:
    return None if values is None else values.copy()

