"""Observation-aligned recursive chunk ensembling (2-chunk temporal blend).

Structure (user-specified; step = one policy_dt):

  * A new inference is kicked every R steps (window cadence); each chunk's
    timeline ORIGIN is its kick wall-step.
  * Per chunk C_k with period R and horizon H >= 3R:
      [0..R)   discarded  (covers the inference flight window)
      [R..2R)  "new" side of one blend window
      [2R..3R) "old" side of the next blend window
      [3R..H)  runway (executed pure only when the next chunk is LATE)
  * Executed window starting at wall W:
      blend( C_old[W-kick_old .. +R),  C_new[W-kick_new .. +R) )
    with per-step weight w_new(j) = j / (R - 1): j=0 is pure OLD (continuous
    with the previous window, which ended pure on the same plan) and j=R-1 is
    pure NEW (continuous with the next window's old side).
  * First chunk special case: no blend partner -> its [0..2R) executes pure.

The scheduler keeps everything in PLAN space (chain anchoring): each chunk is
integrated into absolute stand pose7 targets from the executed plan value at
its kick step, and each built window emits SYNTHETIC ee_local deltas (14-dim
rows, both arms + absolute grips) so the entire existing pipeline — FOH
conditioner (reanchor last_emitted_continuous), chain-anchored overlay /
Ruckig chunk-follower frames, gripper dispatch — consumes a window exactly
like a normal chunk with zero downstream changes.

Blending is in absolute target space: position lerp + quaternion slerp
(``interp_pose``) + scalar grip lerp.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from .flow_dataset import pose_compose_local, pose_delta_local
from .tcp_target_pose_conditioner import interp_pose

ARMS = ("left", "right")
_ARM_SLICES = {"left": slice(0, 6), "right": slice(7, 13)}
_ARM_GRIP = {"left": 6, "right": 13}

# pose7 = [x, y, z, qx, qy, qz, qw]; AnchorFn supplies the first-chunk /
# post-reset seed pose per arm (chain tail -> command pose fallback).
AnchorFn = Callable[[str], np.ndarray]


def blend_weight(j: int, r: int) -> float:
    """w_new for step j of an R-step blend window: 0 at j=0 (pure old, seam-
    continuous with the previous window) -> 1 at j=R-1 (pure new, seam-
    continuous with the next window's old side)."""
    if r <= 1:
        return 1.0
    return float(j) / float(r - 1)


@dataclass
class _Plan:
    kick_wall: int
    horizon: int = 0
    # absolute integrated targets per arm, length H (index i = wall kick_wall+i)
    abs_targets: dict[str, list[np.ndarray]] = field(default_factory=dict)
    grips: dict[str, list[float]] = field(default_factory=dict)
    anchors: dict[str, np.ndarray] = field(default_factory=dict)  # pre-pose of targets[0]

    def segment(self, arm: str, wall_start: int, length: int) -> list[np.ndarray] | None:
        i0 = wall_start - self.kick_wall
        if i0 < 0 or i0 + length > self.horizon:
            return None
        return self.abs_targets[arm][i0 : i0 + length]

    def grip_segment(self, arm: str, wall_start: int, length: int) -> list[float] | None:
        i0 = wall_start - self.kick_wall
        if i0 < 0 or i0 + length > self.horizon:
            return None
        return self.grips[arm][i0 : i0 + length]

    def available_from(self, wall_start: int, max_length: int) -> int:
        i0 = wall_start - self.kick_wall
        if i0 < 0 or i0 >= self.horizon:
            return 0
        return max(0, min(int(max_length), self.horizon - i0))


class ChunkEnsembleScheduler:
    """Owns the plan chains + window schedule. All indices are POLICY STEPS
    ("wall steps"); wall 0 = first step of the first window."""

    def __init__(self, period: int, horizon: int, blend_mode: str = "linear") -> None:
        if period < 2:
            raise ValueError("ensemble period must be >= 2")
        if horizon < 3 * period:
            raise ValueError(
                f"action horizon {horizon} must be >= 3*period ({3 * period}) "
                "(discard + new-side + old-side regions)"
            )
        if blend_mode not in ("linear", "none"):
            raise ValueError("blend_mode must be 'linear' or 'none'")
        self.period = int(period)
        self.horizon = int(horizon)
        # "linear": window = lerp(old[2R..3R), new[R..2R)) (w_new = j/(R-1)).
        # "none":   window = new[R..2R) PURE — the old plan is only the late-
        #           arrival runway; window seams may step by however far the two
        #           plans diverged over R steps (absorbed downstream by the FOH
        #           interpolation + the Ruckig follower's jerk limits).
        self.blend_mode = blend_mode
        self.reset()

    def reset(self) -> None:
        self._plans: list[_Plan] = []          # newest LAST; at most 2 kept
        self._pending_kick_wall: int | None = None
        self._window_start = 0                  # wall step of the NEXT window to build
        self._last_window_start = 0
        self._last_window_length = 0
        self._plan_tail: dict[str, np.ndarray | None] = {a: None for a in ARMS}
        self._history: dict[str, dict[int, np.ndarray]] = {a: {} for a in ARMS}
        self.first_window_built = False
        self.active_kick_at = 0                # prefetch-kick index within the ACTIVE window
        self.last_window_provenance = ""       # "pure-first" | "blend" | "pure-old-runway" | "pure-new" | "starved"

    # ------------------------------------------------------------------ kicks
    def note_kick(self, wall_step: int) -> None:
        """Record the wall step at which the in-flight inference was requested;
        it becomes the arriving chunk's timeline origin."""
        self._pending_kick_wall = int(wall_step)

    def kick_index_for_active_window(self) -> int:
        """Prefetch-kick index within the ACTIVE window: R for the first (2R)
        window — kick C2 after consuming [0..R) — then 0 (window start).
        Stored per built window (begin/advance), since first_window_built flips
        immediately after begin() while its window is still executing."""
        return self.active_kick_at

    # -------------------------------------------------------------- chunk reg
    def _register(self, raw_chunk: np.ndarray, kick_wall: int, anchor_fn: AnchorFn) -> None:
        raw = np.asarray(raw_chunk, dtype=np.float64)
        plan = _Plan(kick_wall=int(kick_wall), horizon=int(raw.shape[0]))
        for arm in ARMS:
            anchor = self._anchor_for(arm, kick_wall, anchor_fn)
            plan.anchors[arm] = anchor
            cur = anchor
            targets: list[np.ndarray] = []
            grips: list[float] = []
            sl = _ARM_SLICES[arm]
            gi = _ARM_GRIP[arm]
            for i in range(plan.horizon):
                cur = np.asarray(pose_compose_local(cur, raw[i][sl]), dtype=np.float64)
                targets.append(cur)
                grips.append(float(raw[i][gi]))
            plan.abs_targets[arm] = targets
            plan.grips[arm] = grips
        self._plans.append(plan)
        if len(self._plans) > 2:
            self._plans.pop(0)

    def _anchor_for(self, arm: str, kick_wall: int, anchor_fn: AnchorFn) -> np.ndarray:
        # The plan value AT the kick step (last executed-plan target before it);
        # falls back to the caller seed (chain tail -> command pose) on the very
        # first chunk / right after a reset.
        hist = self._history[arm]
        if kick_wall - 1 in hist:
            return np.asarray(hist[kick_wall - 1], dtype=np.float64).reshape(7)
        tail = self._plan_tail[arm]
        if tail is not None:
            return np.asarray(tail, dtype=np.float64).reshape(7)
        return np.asarray(anchor_fn(arm), dtype=np.float64).reshape(7)

    # ----------------------------------------------------------- window build
    def begin(self, raw_chunk: np.ndarray, anchor_fn: AnchorFn) -> np.ndarray:
        """First chunk: register at kick_wall=0 and return its pure [0..2R)
        window as a synthetic delta chunk."""
        self._register(raw_chunk, kick_wall=0, anchor_fn=anchor_fn)
        window = self._build_window(length=2 * self.period)
        assert window is not None  # a freshly registered plan covers [0..2R)
        self.first_window_built = True
        self.active_kick_at = self.period      # kick C2 after consuming [0..R)
        return window

    def advance(self, raw_chunk: np.ndarray | None, anchor_fn: AnchorFn) -> np.ndarray | None:
        """Boundary: optionally register the newly arrived chunk (kicked at
        note_kick()'s wall step), then build the next R-step window. Returns
        None when no plan covers the window (true starvation -> caller stalls)."""
        if raw_chunk is not None:
            kick_wall = self._pending_kick_wall
            if kick_wall is None:  # defensive: assume the nominal cadence
                kick_wall = max(0, self._window_start - self.period)
            self._pending_kick_wall = None
            self._register(raw_chunk, kick_wall=kick_wall, anchor_fn=anchor_fn)
        window = self._build_window(length=self.period)
        if window is not None:
            self.active_kick_at = 0            # steady: kick at window start
        return window

    def _build_window(self, length: int) -> np.ndarray | None:
        start = self._window_start
        new_plan = old_plan = None
        for plan in reversed(self._plans):      # newest first
            if plan.segment(ARMS[0], start, length) is not None:
                if new_plan is None:
                    new_plan = plan
                    if self.blend_mode == "none":
                        break                    # pure newest; no old side
                else:
                    # A previous plan is a blend partner only over its old-side
                    # interval [2R..3R). Past that it is runway, used pure only
                    # when no newer plan covers the window.
                    i0 = start - plan.kick_wall
                    if 2 * self.period <= i0 and i0 + length <= 3 * self.period:
                        old_plan = plan
                    break
        if new_plan is None:
            self.last_window_provenance = "starved"
            return None
        if old_plan is not None:
            self.last_window_provenance = "blend"
        elif not self.first_window_built:
            self.last_window_provenance = "pure-first"
        elif new_plan is self._plans[-1]:
            self.last_window_provenance = "pure-new"       # newest plan (blend off / no old coverage)
        else:
            self.last_window_provenance = "pure-old-runway"  # late chunk: riding the old tail

        rows = [np.zeros(14, dtype=np.float64) for _ in range(length)]
        for arm in ARMS:
            seg_new = new_plan.segment(arm, start, length)
            grip_new = new_plan.grip_segment(arm, start, length)
            seg_old = old_plan.segment(arm, start, length) if old_plan is not None else None
            grip_old = old_plan.grip_segment(arm, start, length) if old_plan is not None else None
            prev = self._plan_tail[arm]
            if prev is None:
                prev = new_plan.anchors[arm]     # first window after (re)seed
            sl = _ARM_SLICES[arm]
            gi = _ARM_GRIP[arm]
            for j in range(length):
                if seg_old is not None:
                    w = blend_weight(j, length)
                    target = np.asarray(interp_pose(seg_old[j], seg_new[j], w), dtype=np.float64)
                    grip = (1.0 - w) * grip_old[j] + w * grip_new[j]
                else:
                    target = np.asarray(seg_new[j], dtype=np.float64)
                    grip = grip_new[j]
                rows[j][sl] = np.asarray(pose_delta_local(prev, target), dtype=np.float64)
                rows[j][gi] = grip
                self._history[arm][start + j] = target
                prev = target
            self._plan_tail[arm] = prev
            # prune history beyond what a maximally-late chunk can reference
            floor = start + length - 4 * self.period
            hist = self._history[arm]
            for key in [k for k in hist if k < floor]:
                del hist[key]
        self._last_window_start = start
        self._last_window_length = length
        self._window_start = start + length
        return np.asarray(rows, dtype=np.float32)

    def runway_segment(self, length: int | None = None) -> np.ndarray:
        """Synthetic continuation rows immediately after the last built window.

        The slice is taken from the latest registered plan only. For a steady
        R-step window starting at wall W this is wall [W+R, W+2R), i.e.
        latest-plan rows [W+R-kick_wall, W+2R-kick_wall). The scheduler state is
        not advanced; these rows are for producer-side follower frames.
        """
        if not self._plans or self._last_window_length <= 0:
            return np.zeros((0, 14), dtype=np.float32)
        requested = self.period if length is None else int(length)
        if requested <= 0:
            return np.zeros((0, 14), dtype=np.float32)
        start = self._last_window_start + self._last_window_length
        latest = self._plans[-1]
        available = latest.available_from(start, requested)
        if available <= 0:
            return np.zeros((0, 14), dtype=np.float32)

        rows = [np.zeros(14, dtype=np.float64) for _ in range(available)]
        for arm in ARMS:
            i0 = start - latest.kick_wall
            seg = latest.abs_targets[arm][i0 : i0 + available]
            grip = latest.grips[arm][i0 : i0 + available]
            prev = self._plan_tail[arm]
            if prev is None:
                prev = latest.anchors[arm]
            sl = _ARM_SLICES[arm]
            gi = _ARM_GRIP[arm]
            for j in range(available):
                target = np.asarray(seg[j], dtype=np.float64)
                rows[j][sl] = np.asarray(pose_delta_local(prev, target), dtype=np.float64)
                rows[j][gi] = float(grip[j])
                prev = target
        return np.asarray(rows, dtype=np.float32)

    # ------------------------------------------------------------- telemetry
    @property
    def window_start(self) -> int:
        return self._window_start

    def plan_tail_pose(self, arm: str) -> np.ndarray | None:
        return self._plan_tail[arm]


def overlay_rows_with_runway(
    window_rows: np.ndarray,
    *,
    scheduler: ChunkEnsembleScheduler | None,
    stitch_mode: str,
) -> np.ndarray:
    rows = np.asarray(window_rows)
    if str(stitch_mode) != "ensemble" or scheduler is None:
        return rows
    if rows.shape[0] > scheduler.period:
        return rows
    runway = np.asarray(scheduler.runway_segment(), dtype=rows.dtype)
    if runway.size <= 0:
        return rows
    return np.concatenate([rows, runway], axis=0)
