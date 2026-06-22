#!/usr/bin/env python3
"""3-tier TcpTargetPose goal-metric analysis + classification for one replay run.

Consumes a single ``log.csv`` produced by ``replay_episode_tcp_pose_target.py``
(per-tick, per-arm) and emits ``pgprofile_result.json`` + ``pgprofile_summary.md``
restructured into the three goal-metric tiers requested by the supervisor:

  A. goal_conditioning_quality      (offline/pre-controller; no actual_tcp needed)
        source_raw_target -> conditioned_goal_after_A
  B. controller_reference_goal_following  (pgmode HEADLINE; measurable)
        conditioned_goal_after_A -> reference_after_B (= tcp_ref_stand)
        + IK / safety feasibility
  C. physical_goal_tracking         (status: not_measured in pgmode; q_actual frozen)
        reference_after_B -> actual_tcp (= tcp_actual_stand)

It then assigns exactly one ``primary_class`` per the campaign rules, with all
contributing risks recorded in ``risk_flags``. Soft tracking thresholds are
evaluated on tier B (controller reference), never on tier C (frozen in pgmode).
No data is manipulated to meet a threshold.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
for path in (str(REPO_ROOT), str(REPO_ROOT / "tools")):
    if path not in sys.path:
        sys.path.insert(0, path)

from tcp_tuning.config import load_config  # noqa: E402
from tcp_tuning.metrics import (  # noqa: E402
    cross_correlation_lag,
    pose_error_metrics,
    position_span,
    smoothness_metrics,
)
from tcp_tuning.trajectory_log import TrajectoryLogReader  # noqa: E402

RESULT_SCHEMA = "robotics_lab.tcp_pgprofile.run_result.v1"
ARMS = ("left", "right")

# --- soft tracking thresholds (provisional, do not game) ----------------------
POS_P95_MM = 10.0
ORI_P95_DEG = 2.0
LAG_MS = 150.0
IK_SOLVE_P95_US = 1000.0
IK_SOLVE_MAX_US = 3000.0
IK_BUDGET_US = 2000.0  # 500 Hz servo tick budget
# --- classification labels ----------------------------------------------------
REAL_READY = "REAL_READY_TS_1P0"  # legacy alias (time_scale=1.0); see real_ready_label()
REAL_READY_PREFIX = "REAL_READY_TS_"
SPEED_LIMITED = "SPEED_LIMITED"
IK_BRANCH_RISK = "IK_BRANCH_RISK"
IK_FAILURE = "IK_FAILURE"
SINGULARITY_RISK = "SINGULARITY_RISK"
WORKSPACE_ROI_RISK = "WORKSPACE_ROI_RISK"
FLOOR_RISK = "FLOOR_RISK"
SELF_COLLISION_RISK = "SELF_COLLISION_RISK"
WATCHDOG_OR_LEASE_FAILURE = "WATCHDOG_OR_LEASE_FAILURE"
TRACKING_LAG_HIGH = "TRACKING_LAG_HIGH"
SMOOTHNESS_HF_HIGH = "SMOOTHNESS_HF_HIGH"
DATA_QUALITY_BAD = "DATA_QUALITY_BAD"
NEEDS_MANUAL_REVIEW = "NEEDS_MANUAL_REVIEW"


def ts_token(time_scale: float) -> str:
    """Format a time_scale as a class-name token: 1.0->1P0, 1.25->1P25, 2.0->2P0."""
    s = f"{float(time_scale):g}"  # drop trailing zeros: 1.0->'1', 1.25->'1.25'
    if "." not in s:
        s += ".0"
    return s.replace(".", "P")


def real_ready_label(time_scale: float) -> str:
    """time_scale-aware REAL_READY label (Fix 5): REAL_READY_TS_<token>."""
    return f"{REAL_READY_PREFIX}{ts_token(time_scale)}"


# ------------------------------------------------------------------ loading ---
def _as_bool_array(values: list[Any]) -> np.ndarray:
    out = np.zeros(len(values), dtype=bool)
    for i, v in enumerate(values):
        if isinstance(v, bool):
            out[i] = v
        elif isinstance(v, (int, float)) and np.isfinite(v):
            out[i] = bool(v)
        elif isinstance(v, str):
            out[i] = v.strip().lower() == "true"
    return out


def _as_float_array(values: list[Any]) -> np.ndarray:
    out = np.full(len(values), np.nan, dtype=np.float64)
    for i, v in enumerate(values):
        try:
            f = float(v)
            out[i] = f
        except (TypeError, ValueError):
            pass
    return out


def _stack_pose(rows: list[dict[str, Any]], key: str, width: int = 7) -> np.ndarray | None:
    """TrajectoryLogReader unflattens VECTOR_COLUMNS to per-row ndarrays; fall back
    to flat <key>_<j> columns for raw CSVs."""
    arr = np.full((len(rows), width), np.nan, dtype=np.float64)
    flat_cols = [f"{key}_{j}" for j in range(width)]
    has_direct = isinstance(rows[0].get(key), (list, tuple, np.ndarray))
    has_flat = any(c in rows[0] for c in flat_cols)
    if not (has_direct or has_flat):
        return None
    for i, r in enumerate(rows):
        v = r.get(key)
        if isinstance(v, (list, tuple, np.ndarray)):
            vv = np.asarray(v, dtype=np.float64).reshape(-1)
            arr[i, : min(width, vv.size)] = vv[:width]
        else:
            for j, c in enumerate(flat_cols):
                try:
                    arr[i, j] = float(r.get(c))
                except (TypeError, ValueError):
                    pass
    if not np.isfinite(arr).any():
        return None
    return arr


def load_arms(log_path: Path) -> dict[str, dict[str, Any]]:
    reader = TrajectoryLogReader(log_path)
    rows = reader.read()
    by_arm: dict[str, list[dict[str, Any]]] = {a: [] for a in ARMS}
    for r in rows:
        a = str(r.get("arm", "")).lower()
        if a in by_arm:
            by_arm[a].append(r)
    out: dict[str, dict[str, Any]] = {}
    for arm, arm_rows in by_arm.items():
        if not arm_rows:
            continue
        d: dict[str, Any] = {"n": len(arm_rows)}
        d["t"] = _as_float_array([r.get("t") for r in arm_rows])
        for key in ("source_raw_target", "conditioned_goal_after_A", "reference_after_B", "actual_tcp"):
            d[key] = _stack_pose(arm_rows, key)
        # Optional pure-SMD reference (Patch 4/5 server telemetry); absent today.
        d["smd_ref_stand"] = _stack_pose(arm_rows, "smd_ref_stand")
        # Optional C-stage final-polish (output moving-average) joint vectors.
        d["q_target_before_output_ma_deg"] = _stack_pose(arm_rows, "q_target_before_output_ma_deg", width=6)
        d["q_target_after_output_ma_deg"] = _stack_pose(arm_rows, "q_target_after_output_ma_deg", width=6)
        d["q_target"] = _stack_pose(arm_rows, "q_target", width=6)
        d["q_actual"] = _stack_pose(arm_rows, "q_actual", width=6)
        for key in ("ik_solve_us", "ik_pos_err", "ik_ori_err", "ik_solution_jump_deg",
                    "ik_min_singular_value", "ik_applied_damping",
                    "self_collision_min_clearance_m", "roi_min_margin_m",
                    "command_reference_tracking_error_deg",
                    "smd_goal_linear_velocity_norm_m_s", "smd_goal_angular_velocity_norm_rad_s"):
            d[key] = _as_float_array([r.get(key) for r in arm_rows])
        for key in ("ik_branch_jump_clamped", "branch_jump_flag", "singular_damping_flag",
                    "safety_proj_flag", "self_collision_flag", "floor_flag", "roi_flag",
                    "smd_goal_clamped_flag",
                    # Patch 4: separated SMD clip flags (replace the lumped goal-clamped).
                    "smd_linear_velocity_clipped", "smd_linear_accel_clipped",
                    "smd_angular_velocity_clipped", "smd_angular_accel_clipped",
                    "smd_goal_linear_velocity_ff_clipped", "smd_goal_angular_velocity_ff_clipped",
                    "smd_velocity_feedforward_used"):
            d[key] = _as_bool_array([r.get(key) for r in arm_rows])
        d["smd_velocity_feedforward_source"] = [
            str(r.get("smd_velocity_feedforward_source", "")).strip().lower() for r in arm_rows
        ]
        d["ik_status"] = [str(r.get("ik_status", "")).strip().lower() for r in arm_rows]
        d["fault_latched"] = _as_bool_array([r.get("fault_latched") for r in arm_rows])
        out[arm] = d
    return out


# ------------------------------------------------------------------ tiers -----
def _pct(arr: np.ndarray, p: float) -> float | None:
    a = arr[np.isfinite(arr)]
    return float(np.percentile(a, p)) if a.size else None


def _mx(arr: np.ndarray) -> float | None:
    a = arr[np.isfinite(arr)]
    return float(a.max()) if a.size else None


def tier_a(arm: dict[str, Any], cfg) -> dict[str, Any]:
    """goal_conditioning_quality: source_raw_target -> conditioned_goal_after_A."""
    t = arm["t"]
    cg = arm.get("conditioned_goal_after_A")
    srt = arm.get("source_raw_target")
    smooth = smoothness_metrics(t, cg, cfg=cfg, policy_rate_hz=cfg.chunk_rate_hz,
                                metric_name="conditioned_goal_smoothness")
    return {
        "source_raw_target_vs_conditioned_goal": pose_error_metrics(
            srt, cg, metric_name="source_raw_target_vs_conditioned_goal"),
        "conditioned_goal_linear_velocity_p95_m_s": _spectral_speed_p95(smooth, "linear"),
        "conditioned_goal_angular_velocity_p95_rad_s": _spectral_speed_p95(smooth, "angular"),
        "conditioned_goal_linear_jerk_rms": _nested(smooth, "linear_jerk_m_s3", "rms"),
        "conditioned_goal_angular_jerk_rms": _nested(smooth, "angular_jerk_rad_s3", "rms"),
        "conditioned_goal_high_frequency_power_above_5hz": _nested(
            smooth, "linear_velocity_spectrum", "power_above_cutoff"),
        "conditioned_goal_linear_sign_reversals_per_sec": _nested(
            smooth, "linear_velocity_sign_reversals_per_sec", "per_sec"),
        "conditioned_goal_angular_sign_reversals_per_sec": _nested(
            smooth, "angular_velocity_sign_reversals_per_sec", "per_sec"),
    }


def tier_b(arm: dict[str, Any], cfg) -> dict[str, Any]:
    """B reference_generation: conditioned_goal_after_A -> reference.

    Prefers the pure-SMD reference (``smd_ref_stand``, Patch 4/5 server telemetry) when
    present; otherwise falls back to ``reference_after_B`` (= ``tcp_ref_stand``), which is
    POST IK/safety/MA and therefore NOT pure SMD output. A warning is emitted on fallback
    so B-tier numbers are never silently misattributed to the SMD alone.
    """
    t = arm["t"]
    cg = arm.get("conditioned_goal_after_A")
    smd_ref = arm.get("smd_ref_stand")
    has_smd = smd_ref is not None and np.isfinite(smd_ref).any()
    ref = smd_ref if has_smd else arm.get("reference_after_B")
    reference_source = "smd_ref_stand" if has_smd else "tcp_ref_stand"
    warning = None if has_smd else (
        "smd_ref_stand unavailable; B uses tcp_ref_stand which is post-IK/safety/MA, not pure SMD"
    )
    err = pose_error_metrics(ref, cg, metric_name="reference_vs_conditioned_goal")
    lag = cross_correlation_lag(t, cg, ref, max_lag_sec=cfg.lag_max_sec,
                                metric_name="reference_lag_vs_conditioned_goal")
    span_cg = position_span(cg)
    span_ref = position_span(ref)
    span_ratio = (span_ref / span_cg) if span_cg and span_cg > 1e-6 else None
    endpoint_err = _endpoint_error(ref, cg)
    plen_cg = _path_length(cg)
    plen_ref = _path_length(ref)
    plen_ratio = (plen_ref / plen_cg) if plen_cg and plen_cg > 1e-6 else None
    return {
        "reference_source": reference_source,
        "warning": warning,
        # back-compat alias key kept for existing campaign/report consumers
        "reference_after_B_vs_conditioned_goal": err,
        "reference_vs_conditioned_goal": err,
        "tcp_ref_lag_vs_conditioned_goal": lag,
        "reference_lag_vs_conditioned_goal": lag,
        "reference_span_ratio": span_ratio,
        "reference_endpoint_error_m": endpoint_err,
        "reference_path_length_ratio": plen_ratio,
        # Legacy lumped clamp (kept for back-compat consumers).
        "smd_goal_clamped_count": int(np.count_nonzero(arm["smd_goal_clamped_flag"])),
        # Patch 4: separated SMD clip telemetry with distinct reasons. State vel/accel
        # clips bound the SMD output; the ff clips flag a feedforward goal-velocity spike.
        "smd_clip": {
            "linear_velocity_clipped_count": int(np.count_nonzero(arm["smd_linear_velocity_clipped"])),
            "linear_accel_clipped_count": int(np.count_nonzero(arm["smd_linear_accel_clipped"])),
            "angular_velocity_clipped_count": int(np.count_nonzero(arm["smd_angular_velocity_clipped"])),
            "angular_accel_clipped_count": int(np.count_nonzero(arm["smd_angular_accel_clipped"])),
            "goal_linear_velocity_ff_clipped_count": int(np.count_nonzero(arm["smd_goal_linear_velocity_ff_clipped"])),
            "goal_angular_velocity_ff_clipped_count": int(np.count_nonzero(arm["smd_goal_angular_velocity_ff_clipped"])),
        },
        "smd_velocity_feedforward_used_ratio": (
            float(np.count_nonzero(arm["smd_velocity_feedforward_used"])) / arm["n"] if arm["n"] else None
        ),
        "smd_velocity_feedforward_source": _dominant_nonempty(arm.get("smd_velocity_feedforward_source")),
        "smd_goal_linear_velocity_p95_m_s": _pct(arm["smd_goal_linear_velocity_norm_m_s"], 95),
        "smd_goal_angular_velocity_p95_rad_s": _pct(arm["smd_goal_angular_velocity_norm_rad_s"], 95),
        "command_reference_tracking_error_deg_p95": _pct(arm["command_reference_tracking_error_deg"], 95),
    }


def c_final_polish(arm: dict[str, Any], cfg) -> dict[str, Any]:
    """C final_polish: q_target before vs after the output moving-average (Patch 4).

    Reports the HF reduction and added lag from the joint output MA. When the
    before/after MA telemetry is absent (today), returns status ``unavailable``.
    """
    before = arm.get("q_target_before_output_ma_deg")
    after = arm.get("q_target_after_output_ma_deg")
    have = (before is not None and np.isfinite(before).any()
            and after is not None and np.isfinite(after).any())
    if not have:
        return {
            "status": "unavailable",
            "reason": "q_target_before/after_output_ma_deg not in log (server output-MA telemetry absent)",
        }
    t = arm["t"]
    sm_before = smoothness_metrics(t, _pad_pose(before), cfg=cfg, policy_rate_hz=cfg.chunk_rate_hz,
                                   metric_name="q_target_before_ma")
    sm_after = smoothness_metrics(t, _pad_pose(after), cfg=cfg, policy_rate_hz=cfg.chunk_rate_hz,
                                  metric_name="q_target_after_ma")
    hf_before = _nested(sm_before, "linear_velocity_spectrum", "power_above_cutoff")
    hf_after = _nested(sm_after, "linear_velocity_spectrum", "power_above_cutoff")
    reduction = None
    if hf_before is not None and hf_after is not None and hf_before > 0:
        reduction = float(1.0 - hf_after / hf_before)
    lag = cross_correlation_lag(t, _pad_pose(before), _pad_pose(after), max_lag_sec=cfg.lag_max_sec,
                                metric_name="output_ma_added_lag")
    return {
        "status": "measured",
        "q_target_hf_power_before": hf_before,
        "q_target_hf_power_after": hf_after,
        "hf_reduction_ratio": reduction,
        "added_lag_sec": (lag.get("lag_sec") if isinstance(lag, dict) else None),
    }


def _pad_pose(joints: np.ndarray | None) -> np.ndarray | None:
    """Adapt a (N,6) joint series to the (N,7) pose interface used by metric helpers
    (first 3 cols are treated as 'position' by smoothness/lag; the rest are zero-padded)."""
    if joints is None:
        return None
    n = joints.shape[0]
    out = np.zeros((n, 7), dtype=np.float64)
    out[:, : min(6, joints.shape[1])] = joints[:, :6]
    out[:, 6] = 1.0
    return out


def ik_safety_feasibility(arm: dict[str, Any]) -> dict[str, Any]:
    n = arm["n"]
    status = arm["ik_status"]
    ik_ok = sum(1 for s in status if s in ("ok", "success", ""))
    ik_fail = sum(1 for s in status if s and s not in ("ok", "success"))
    solve = arm["ik_solve_us"]
    # ik_min_singular_value is reported as 0.0 on dead-zone ticks where the solver
    # returns the seed without iterating (more frequent with a looser position
    # tolerance). 0.0 means "not computed", NOT "exactly singular" — a genuinely
    # singular solve IkFails. Use only the nonzero finite sigma for the proximity
    # statistic so the dead-zone zeros do not masquerade as singularity.
    sigma_raw = arm["ik_min_singular_value"]
    sigma = sigma_raw[np.isfinite(sigma_raw) & (sigma_raw > 0.0)]
    branch = int(np.count_nonzero(arm["branch_jump_flag"])) + int(np.count_nonzero(arm["ik_branch_jump_clamped"]))
    return {
        "ik_success_ratio": (ik_ok / n) if n else None,
        "ik_failure_count": ik_fail,
        "ik_pos_err_p95_m": _pct(arm["ik_pos_err"], 95),
        "ik_pos_err_max_m": _mx(arm["ik_pos_err"]),
        "ik_ori_err_p95_rad": _pct(arm["ik_ori_err"], 95),
        "ik_ori_err_max_rad": _mx(arm["ik_ori_err"]),
        "ik_solve_us_p50": _pct(solve, 50),
        "ik_solve_us_p95": _pct(solve, 95),
        "ik_solve_us_max": _mx(solve),
        "ik_over_budget_2ms_count": int(np.count_nonzero(solve[np.isfinite(solve)] > IK_BUDGET_US)),
        "ik_solution_jump_deg_p95": _pct(arm["ik_solution_jump_deg"], 95),
        "ik_solution_jump_deg_max": _mx(arm["ik_solution_jump_deg"]),
        "ik_branch_jump_count": branch,
        "ik_min_singular_value_p05": (float(np.percentile(sigma, 5)) if sigma.size else None),
        "ik_min_singular_value_min": (float(sigma.min()) if sigma.size else None),
        "singular_damping_count": int(np.count_nonzero(arm["singular_damping_flag"])),
        "safety_projection_count": int(np.count_nonzero(arm["safety_proj_flag"])),
        "floor_clamp_count": int(np.count_nonzero(arm["floor_flag"])),
        "roi_clamp_count": int(np.count_nonzero(arm["roi_flag"])),
        "self_collision_count": int(np.count_nonzero(arm["self_collision_flag"])),
        "self_collision_min_clearance_m_min": (
            float(np.nanmin(arm["self_collision_min_clearance_m"]))
            if np.isfinite(arm["self_collision_min_clearance_m"]).any() else None),
        "roi_min_margin_m_min": (
            float(np.nanmin(arm["roi_min_margin_m"]))
            if np.isfinite(arm["roi_min_margin_m"]).any() else None),
        "fault_latched_any": bool(np.count_nonzero(arm["fault_latched"])),
    }


def tier_c(arm: dict[str, Any], cfg) -> dict[str, Any]:
    """physical_goal_tracking: reference_after_B -> actual_tcp. not_measured in pgmode."""
    ref = arm.get("reference_after_B")
    act = arm.get("actual_tcp")
    cg = arm.get("conditioned_goal_after_A")
    span_ref = position_span(ref)
    span_act = position_span(act)
    # q_actual / tcp_actual is frozen in pgmode -> actual span collapses.
    not_measured = span_act is None or (span_ref and span_ref > 1e-3 and span_act < max(2e-4, 0.05 * span_ref))
    status = "not_measured" if not_measured else "measured"
    out: dict[str, Any] = {
        "status": status,
        "reason": (
            "tcp_actual_stand frozen in rbpodo pgmode controller-simulation (physical q_actual stationary)"
            if not_measured else "physical motion observed"
        ),
        "actual_position_span_m": span_act,
        "reference_position_span_m": span_ref,
    }
    if not not_measured:
        out["actual_tcp_vs_reference_after_B"] = pose_error_metrics(
            act, ref, metric_name="actual_tcp_vs_reference_after_B")
        out["actual_tcp_vs_conditioned_goal"] = pose_error_metrics(
            act, cg, metric_name="actual_tcp_vs_conditioned_goal")
    return out


# --------------------------------------------------------------- classify -----
def classify(per_arm: dict[str, Any], *, speed_precheck_pass: bool | None,
             validity_class: str | None, time_scale: float = 1.0) -> dict[str, Any]:
    risks: list[str] = []
    agg: dict[str, Any] = {}
    # aggregate hard counts across arms
    def acc(key: str) -> int:
        return sum(int(per_arm[a]["ik_safety"].get(key) or 0) for a in per_arm)
    ik_fail = acc("ik_failure_count")
    branch = acc("ik_branch_jump_count")
    singd = acc("singular_damping_count")
    roi = acc("roi_clamp_count")
    floor = acc("floor_clamp_count")
    selfc = acc("self_collision_count")
    safproj = acc("safety_projection_count")
    fault = any(per_arm[a]["ik_safety"].get("fault_latched_any") for a in per_arm)
    sigma_p05 = [per_arm[a]["ik_safety"].get("ik_min_singular_value_p05") for a in per_arm]
    sigma_p05 = [s for s in sigma_p05 if s is not None]
    low_sigma = bool(sigma_p05) and min(sigma_p05) < 0.06  # singular_region_eps

    # B-tier soft tracking (controller reference following)
    track_fail = False
    track_detail = {}
    for a in per_arm:
        b = per_arm[a]["tier_b"]["reference_after_B_vs_conditioned_goal"]
        lagm = per_arm[a]["tier_b"]["tcp_ref_lag_vs_conditioned_goal"]
        pos_p95 = (b.get("position_m", {}) or {}).get("p95")
        ori_p95 = (b.get("orientation_rad", {}) or {}).get("p95")
        lag_sec = lagm.get("lag_sec") if isinstance(lagm, dict) else None
        f = False
        if pos_p95 is not None and pos_p95 * 1000.0 > POS_P95_MM:
            f = True
        if ori_p95 is not None and np.degrees(ori_p95) > ORI_P95_DEG:
            f = True
        if lag_sec is not None and abs(lag_sec) * 1000.0 > LAG_MS:
            f = True
        track_detail[a] = {
            "pos_p95_mm": (pos_p95 * 1000.0 if pos_p95 is not None else None),
            "ori_p95_deg": (np.degrees(ori_p95) if ori_p95 is not None else None),
            "lag_ms": (abs(lag_sec) * 1000.0 if lag_sec is not None else None),
            "fail": f,
        }
        track_fail = track_fail or f

    # HF smoothness (B reference reuse A conditioned goal HF as proxy if needed)
    hf_fail = False
    hf_detail = {}
    for a in per_arm:
        hf = per_arm[a]["tier_a"].get("conditioned_goal_high_frequency_power_above_5hz")
        rev = per_arm[a]["tier_a"].get("conditioned_goal_linear_sign_reversals_per_sec")
        # heuristic HF gate: power_above_5hz large or sign reversals high
        f = bool((hf is not None and hf > 1e-3) or (rev is not None and rev > 20.0))
        hf_detail[a] = {"power_above_5hz": hf, "sign_reversals_per_sec": rev, "fail": f}
        hf_fail = hf_fail or f

    agg = {
        "ik_failure_count": ik_fail, "ik_branch_jump_count": branch,
        "singular_damping_count": singd, "ik_min_singular_value_low": low_sigma,
        "roi_clamp_count": roi, "floor_clamp_count": floor,
        "self_collision_count": selfc, "safety_projection_count": safproj,
        "fault_latched": fault,
        "tracking_soft_fail": track_fail, "tracking_detail": track_detail,
        "smoothness_hf_fail": hf_fail, "hf_detail": hf_detail,
    }
    hard_counts_zero = (ik_fail == 0 and branch == 0 and roi == 0 and floor == 0
                        and selfc == 0 and not fault)
    agg["hard_counts_zero"] = hard_counts_zero

    # ---- precedence: safety/feasibility first, then speed, then soft ----------
    if validity_class and not validity_class.startswith("VALID"):
        primary = DATA_QUALITY_BAD
    elif ik_fail > 0:
        primary = IK_FAILURE
    elif branch > 0:
        primary = IK_BRANCH_RISK
    elif low_sigma or singd > 0:
        primary = SINGULARITY_RISK
    elif selfc > 0:
        primary = SELF_COLLISION_RISK
    elif floor > 0:
        primary = FLOOR_RISK
    elif roi > 0:
        primary = WORKSPACE_ROI_RISK
    elif fault:
        primary = WATCHDOG_OR_LEASE_FAILURE
    elif speed_precheck_pass is False:
        primary = SPEED_LIMITED
    elif track_fail:
        primary = TRACKING_LAG_HIGH
    elif hf_fail:
        primary = SMOOTHNESS_HF_HIGH
    else:
        primary = real_ready_label(time_scale)

    for name, cond in (
        (IK_FAILURE, ik_fail > 0), (IK_BRANCH_RISK, branch > 0),
        (SINGULARITY_RISK, low_sigma or singd > 0),
        (SELF_COLLISION_RISK, selfc > 0), (FLOOR_RISK, floor > 0),
        (WORKSPACE_ROI_RISK, roi > 0), (WATCHDOG_OR_LEASE_FAILURE, fault),
        (SPEED_LIMITED, speed_precheck_pass is False),
        (TRACKING_LAG_HIGH, track_fail), (SMOOTHNESS_HF_HIGH, hf_fail),
    ):
        if cond:
            risks.append(name)

    return {"primary_class": primary, "risk_flags": risks, "aggregate": agg}


# ------------------------------------------------------------------ helpers ---
def _dominant_nonempty(values: list[Any] | None) -> str | None:
    """Most frequent non-empty string in a per-tick list (e.g. the vff source)."""
    if not values:
        return None
    counts: dict[str, int] = {}
    for v in values:
        s = str(v).strip()
        if s:
            counts[s] = counts.get(s, 0) + 1
    if not counts:
        return None
    return max(counts.items(), key=lambda kv: kv[1])[0]


def _nested(d: dict, *keys) -> Any:
    cur = d
    for k in keys:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(k)
    return cur


def _spectral_speed_p95(smooth: dict, kind: str) -> float | None:
    key = "linear_velocity_m_s" if kind == "linear" else "angular_velocity_rad_s"
    return _nested(smooth, key, "p95")


def _endpoint_error(ref: np.ndarray | None, cg: np.ndarray | None) -> float | None:
    if ref is None or cg is None:
        return None
    rr = ref[np.isfinite(ref).all(axis=1)]
    cc = cg[np.isfinite(cg).all(axis=1)]
    if rr.size == 0 or cc.size == 0:
        return None
    return float(np.linalg.norm(rr[-1, :3] - cc[-1, :3]))


def _path_length(pose: np.ndarray | None) -> float | None:
    if pose is None:
        return None
    p = pose[np.isfinite(pose).all(axis=1)][:, :3]
    if p.shape[0] < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))


def analyze_run(log_path: Path, cfg_metrics, *, speed_precheck_pass: bool | None = None,
                validity_class: str | None = None, episode_id: str | None = None,
                time_scale: float = 1.0) -> dict[str, Any]:
    arms_raw = load_arms(log_path)
    per_arm: dict[str, Any] = {}
    for arm, data in arms_raw.items():
        per_arm[arm] = {
            "n": data["n"],
            "tier_a": tier_a(data, cfg_metrics),
            "tier_b": tier_b(data, cfg_metrics),
            "ik_safety": ik_safety_feasibility(data),
            "tier_c": tier_c(data, cfg_metrics),
            "c_final_polish": c_final_polish(data, cfg_metrics),
        }
    classification = classify(per_arm, speed_precheck_pass=speed_precheck_pass,
                              validity_class=validity_class, time_scale=time_scale)
    # A/B/C/D architecture-aligned warnings (deduped, e.g. SMD telemetry unavailable).
    warnings: list[str] = []
    for a in per_arm:
        w = per_arm[a]["tier_b"].get("warning")
        if w and w not in warnings:
            warnings.append(w)
    if all(per_arm[a]["c_final_polish"].get("status") == "unavailable" for a in per_arm):
        warnings.append("C final-polish (output-MA) telemetry unavailable; C metrics not computed")
    return {
        "schema": RESULT_SCHEMA,
        "episode_id": episode_id,
        "log_path": str(log_path),
        "speed_precheck_pass": speed_precheck_pass,
        "validity_class": validity_class,
        "time_scale": float(time_scale),
        "warnings": warnings,
        # back-compat keys (unchanged consumers)
        "goal_conditioning_quality_A": {a: per_arm[a]["tier_a"] for a in per_arm},
        "controller_reference_goal_following_B": {a: per_arm[a]["tier_b"] for a in per_arm},
        "ik_safety_feasibility": {a: per_arm[a]["ik_safety"] for a in per_arm},
        "physical_goal_tracking_C": {a: per_arm[a]["tier_c"] for a in per_arm},
        # A/B/C/D architecture-aligned grouping (Patch 6)
        "A_conditioning": {a: per_arm[a]["tier_a"] for a in per_arm},
        "B_reference_generation": {a: per_arm[a]["tier_b"] for a in per_arm},
        "C_final_polish": {a: per_arm[a]["c_final_polish"] for a in per_arm},
        "D_physical_tracking": {a: per_arm[a]["tier_c"] for a in per_arm},
        "classification": classification,
    }


def render_summary(result: dict[str, Any]) -> str:
    c = result["classification"]
    lines = [
        f"# pgprofile run — {result.get('episode_id')}",
        "",
        f"- primary_class: **{c['primary_class']}**",
        f"- risk_flags: {', '.join(c['risk_flags']) or '(none)'}",
        f"- hard_counts_zero: {c['aggregate']['hard_counts_zero']}",
        f"- speed_precheck_pass(ts=1.0): {result.get('speed_precheck_pass')}",
    ]
    warnings = result.get("warnings") or []
    if warnings:
        lines.append("- warnings: " + "; ".join(warnings))
    b_any = next(iter(result["controller_reference_goal_following_B"].values()), {})
    lines += [
        f"- B reference_source: {b_any.get('reference_source', 'tcp_ref_stand')}",
        "",
        "## B — reference generation (conditioned_goal -> reference)",
        "| arm | ref_vs_cond pos_p95(mm) | ori_p95(deg) | lag(ms) | span_ratio | endpoint_err(mm) | smd_clamp |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for a, b in result["controller_reference_goal_following_B"].items():
        td = c["aggregate"]["tracking_detail"].get(a, {})
        sr = b.get("reference_span_ratio")
        ee = b.get("reference_endpoint_error_m")
        lines.append(
            f"| {a} | {_f(td.get('pos_p95_mm'))} | {_f(td.get('ori_p95_deg'))} | {_f(td.get('lag_ms'))} | "
            f"{_f(sr)} | {_f((ee*1000.0) if ee is not None else None)} | {b.get('smd_goal_clamped_count')} |"
        )
    lines += ["", "## IK / safety feasibility", "| arm | ik_ok | branch | sigma_p05 | solve_p95(us) | over2ms | roi | floor | selfcol |",
              "|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for a, f in result["ik_safety_feasibility"].items():
        lines.append(
            f"| {a} | {_f(f.get('ik_success_ratio'))} | {f.get('ik_branch_jump_count')} | "
            f"{_f(f.get('ik_min_singular_value_p05'))} | {_f(f.get('ik_solve_us_p95'))} | "
            f"{f.get('ik_over_budget_2ms_count')} | {f.get('roi_clamp_count')} | "
            f"{f.get('floor_clamp_count')} | {f.get('self_collision_count')} |"
        )
    lines += ["", "## C — final polish (output moving-average)"]
    for a, cc in result.get("C_final_polish", {}).items():
        if cc.get("status") == "measured":
            lines.append(
                f"- {a}: hf_reduction={_f(cc.get('hf_reduction_ratio'))} "
                f"added_lag={_f(cc.get('added_lag_sec'))}s"
            )
        else:
            lines.append(f"- {a}: **{cc.get('status')}** ({cc.get('reason')})")
    lines += ["", "## D — physical tracking"]
    for a, cc in result["physical_goal_tracking_C"].items():
        lines.append(f"- {a}: **{cc['status']}** ({cc['reason']})")
    return "\n".join(lines) + "\n"


def _f(v: Any) -> str:
    if v is None:
        return "-"
    try:
        return f"{float(v):.3g}"
    except (TypeError, ValueError):
        return str(v)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", required=True, help="run log.csv path")
    parser.add_argument("--out-dir", default=None, help="defaults to the log's directory")
    parser.add_argument("--episode-id", default=None)
    parser.add_argument("--speed-precheck-pass", choices=["true", "false"], default=None)
    parser.add_argument("--validity-class", default=None)
    parser.add_argument("--time-scale", type=float, default=1.0,
                        help="Replay time_scale; sets the REAL_READY_TS_<ts> class label (Fix 5).")
    parser.add_argument("--config", default=None)
    args = parser.parse_args(argv)

    cfg = load_config(args.config)
    log_path = Path(args.log)
    out_dir = Path(args.out_dir) if args.out_dir else log_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)
    spp = None if args.speed_precheck_pass is None else (args.speed_precheck_pass == "true")
    result = analyze_run(log_path, cfg.metrics, speed_precheck_pass=spp,
                         validity_class=args.validity_class, episode_id=args.episode_id,
                         time_scale=args.time_scale)
    (out_dir / "pgprofile_result.json").write_text(
        json.dumps(result, indent=2, allow_nan=False, default=lambda o: None) + "\n", encoding="utf-8")
    (out_dir / "pgprofile_summary.md").write_text(render_summary(result), encoding="utf-8")
    print(f"primary_class={result['classification']['primary_class']} "
          f"risks={result['classification']['risk_flags']}")
    print(f"wrote {out_dir/'pgprofile_result.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
