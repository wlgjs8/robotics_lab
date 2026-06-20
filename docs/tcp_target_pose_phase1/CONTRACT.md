# TCP Target Pose Tuning — Phase 1 Shared Contract

This file is the **single source of truth** for the Phase-1 offline tooling. Every
implementation task (audit / replay-target generator / metrics) MUST read this
file first and conform to the schemas, module layout, interfaces, and constraints
below, so that independently-developed pieces interoperate.

It is authored and owned by the supervising engineer (not a task deliverable to
be rewritten). If a task finds a contradiction with the actual code, it must
STOP and report it in its summary instead of silently diverging.

## 0. Scope of Phase 1 (offline only)

Build a small, self-contained **Python** toolkit, under the robotics_lab repo,
to (a) audit recorded UMI HDF5 episodes, (b) generate 500 Hz Cartesian replay
targets from them under several command-conditioning modes, and (c) compute
tracking/smoothness/health metrics on generated trajectories.

This sprint is **offline tooling only**. It separates *Command conditioning (A)*
from *Reference generation (B, the existing SMD)* in an offline replay path. It
does **not** modify the C++ runtime, the SMD tracker, IK, safety, or teleop.

## 1. Hard constraints (do not violate)

- **Do NOT modify** any `servo_*` parameter, the SMD tracker C++ (`smd_pose_tracker.cpp`),
  IK, or any safety filter (self-collision, floor, 3D ROI, tracking-error latch,
  branch-jump guard, singular damping).
- **Do NOT** change runtime teleop or the runtime replay path in this sprint.
  All new behavior is offline and opt-in via CLI flags / config.
- **No C++ changes** in Phase 1. Python only.
- **Do NOT modify raw HDF5 episodes.** Open them read-only (`h5py.File(path, "r")`).
  All generated artifacts go under `outputs/`.
- **Do NOT commit, branch, push, or run git mutating commands.** Leave all changes
  in the working tree for human review. (Repo is on branch `dev`.)
- Preserve existing behavior; existing tests must still pass.
- No magic constants — expose tunables in config (`tools/tcp_tuning/config.py`
  defaults + optional YAML override).
- Never fake dependencies. h5py / numpy / scipy / matplotlib ARE installed
  (h5py 3.16, numpy 2.4, scipy 1.17, matplotlib 3.10). If something else is
  missing, report it; do not stub it.

## 2. Repository facts (verified by supervisor)

- Example episode (primary): `data/data_20260619_115712/episode_012.hdf5`
  (more under `../pika/data/**`; tooling must accept an arbitrary `--episode` path
  and NOT hardcode this one). Do not assume the internal HDF5 schema — sniff it.
- An HDF5 audit ALREADY exists: `policy_runner/policy_runner/hdf5_audit.py`
  (schema id `robotics_lab.policy_runner.hdf5_audit.v1`, CLI
  `python3 -m policy_runner hdf5-audit`) with `policy_runner/tests/test_hdf5_audit.py`.
  **Reuse its key-detection logic where useful; do not duplicate or break it.**
  The new `tools/audit_episode_hdf5.py` is a richer, plotting/segmentation audit
  layered on top — import helpers from `policy_runner.hdf5_audit` when reasonable.
- Existing replay utilities (read for conventions, do not break):
  `scripts/replay_episode_rollout.py`, `scripts/replay_policy_actions.py`,
  `scripts/capture_tcp_trajectory.py`, `scripts/eval_tcp_target_pose_tracking.py`,
  `scripts/eval_tcp_target_pose_500hz.py`.
- The SMD reference generator and its config keys
  (`cartesian_control.pose_track_smd.*`) are C++; Phase-1 Python does NOT call
  them. (A later sprint wires B; for now B is represented only as the documented
  reference stage in logs.) Python SE(3) math uses `scipy.spatial.transform.Rotation`,
  NOT the C++ math path.

## 3. Canonical in-memory representations

Use these everywhere (NumPy, float64):

- **Pose**: position `p` shape `(3,)` in **meters**, plus orientation quaternion
  `q` shape `(4,)` in **scipy xyzw order** (`[x, y, z, w]`). A pose array is
  `(7,)` = `[px,py,pz, qx,qy,qz,qw]`. Provide `scipy.Rotation` <-> quat helpers in
  `se3.py`. When loading from an episode whose quaternion order/units differ, the
  loader MUST detect/convert and record the detected convention in metadata.
- **Twist**: linear velocity `v` shape `(3,)` in m/s + angular velocity `w` shape
  `(3,)` in rad/s, expressed as a rotation-vector rate. Default frame = the
  episode's own pose frame (we are frame-agnostic; no frame conversion here).
- **Time**: seconds, float64, monotonic where possible. `t_source` = source-sample
  timestamp; `t_servo` = 500 Hz tick time (`dt_servo = 1/servo_rate_hz`).
- **Per-arm**: everything is per-arm `left` / `right`. Arms are independent.

Quaternion **sign continuity**: enforce a consistent hemisphere (dot with previous
≥ 0) before any slerp/log; document this in `se3.py`.

## 4. Module layout (create these)

```
tools/tcp_tuning/
  __init__.py
  config.py            # dataclasses + defaults + YAML load/merge; all tunables
  se3.py               # quat/rot helpers: slerp, so3 log/exp, quat continuity, FOH SE3, finite-diff twist
  hdf5_io.py           # read-only episode loader: sniff schema, return EpisodeData
  smoothing.py         # position & SO(3) smoothers (savgol / cubic-spline / lowpass), segment-aware
  command_conditioner.py  # CommandConditioner + ConditionedCommand (the 4 modes)
  trajectory_log.py    # TrajectoryLogWriter/Reader + the canonical column schema + run metadata
  metrics.py           # tracking / smoothness / health metric functions
  tests/
    __init__.py
    test_*.py
tools/audit_episode_hdf5.py        # CLI -> uses hdf5_io + plotting + segmentation
tools/generate_replay_target.py    # CLI -> uses command_conditioner -> writes generated npz
tools/analyze_tcp_replay_logs.py   # CLI -> uses metrics + trajectory_log -> metrics.json/summary.md/plots
```

Tests MUST run from repo root with BOTH:
`python3 -m pytest tools/tcp_tuning/tests -q` and, as stdlib fallback,
`python3 -m unittest discover -s tools/tcp_tuning/tests -t .`
(make `tools/tcp_tuning` importable: add `tools/__init__.py`-free path handling via
a `conftest.py` / sys.path insert, or run as a package — pick one and document it).

## 5. Interfaces (pin these signatures)

```python
# se3.py
def quat_canonical(q_xyzw, ref=None) -> np.ndarray          # sign-continuous
def slerp(q0_xyzw, q1_xyzw, u: float) -> np.ndarray          # u in [0,1]
def so3_log(R) -> np.ndarray   # (3,) rotvec
def so3_exp(rotvec) -> Rotation
def foh_pose(t, t0, p0, q0, t1, p1, q1) -> (p, q_xyzw)        # timestamp-aware first-order hold
def twist_from_poses(p0,q0, p1,q1, dt) -> (v(3,), w(3,))     # body/world per docstring

# hdf5_io.py
@dataclass
class EpisodeData:
    path: str
    t_source: np.ndarray            # (N,) seconds, or None if absent (then nominal_rate_hz used)
    left_pose: np.ndarray           # (N,7) canonical, or None
    right_pose: np.ndarray          # (N,7) canonical, or None
    left_gripper: np.ndarray        # (N,) or None
    right_gripper: np.ndarray       # (N,) or None
    detected: dict                  # which keys/units/quat-order were detected, and notes
def load_episode(path: str, nominal_rate_hz: float = 30.0) -> EpisodeData   # read-only

# command_conditioner.py
@dataclass
class ConditionedCommand:
    t_servo: float
    left_pose: np.ndarray; right_pose: np.ndarray            # (7,)
    left_twist: np.ndarray | None; right_twist: np.ndarray | None   # (6,) = [v(3), w(3)]
    left_gripper: float | None; right_gripper: float | None
    valid: bool; hold: bool; dropout: bool; gap: bool; reanchor: bool
    src_ids: tuple                  # source sample indices used (e.g. (i, i+1) for FOH)
    meta: dict
class CommandConditioner:
    def __init__(self, mode: str, cfg: "ConditioningConfig"): ...
    def reset(self, *, current_left_pose=None, current_right_pose=None): ...
    def update_source_sample(self, t_source, pose_left, pose_right,
                             gripper_left=None, gripper_right=None, metadata=None): ...
    def sample(self, t_servo) -> ConditionedCommand: ...
# modes: "raw_zoh" | "raw_foh_se3" | "clean_foh_se3" | "synthetic_policy_surrogate"
```

`raw_zoh` = hold last source sample (reproduces current step behavior).
`raw_foh_se3` = timestamp-aware FOH (lerp position, slerp orientation); hits exact
source poses at source timestamps; uses real source dt (mark metadata if timestamps
absent and nominal rate used). `clean_foh_se3` = segment at gaps (dt > 3*median OR
dt > 0.100 s), smooth within segments (no smoothing across gaps), then FOH to 500 Hz;
write clean trajectory to a NEW npz (never overwrite raw). `synthetic_policy_surrogate`
= clean + configurable injected noise (pos RMS mm, ori RMS deg, 5–10 Hz oscillation,
chunk-boundary discontinuities, 1–3 frame dropouts), seeded by `--seed`.

## 6. Generated-trajectory npz schema (`generate_replay_target.py` output)

One npz per (episode, mode), filename `<mode>_<rate>hz.npz`, keys:
- `t_servo` (T,), `servo_rate_hz` scalar, `mode` str, `episode` str, `seed` int|-1
- per arm `<arm>_`: `source_raw_target` (T,7), `conditioned_goal` (T,7),
  `conditioned_twist` (T,6) (NaN where unavailable), `gripper` (T,) (NaN if absent),
  flags `valid`/`hold`/`dropout`/`gap`/`reanchor` (T,) bool, `src_id_lo`/`src_id_hi` (T,)
- `segments` (S,2) int source-index ranges, `gaps` (G,2) (t_before, t_after)
- `meta_json` str (JSON: git commit, config dump, detected schema, nominal_rate flag)

`reference_after_B`, `q_target`, `q_actual`, `actual_tcp` columns are RESERVED in the
schema (present as NaN-filled or absent-with-note) — they get filled by the future
runtime/sim path. Phase-1 generated files carry A-stage data; metrics must handle
the B/actual columns being absent gracefully.

## 7. Trajectory-log column schema (`trajectory_log.py`, for future sim/replay runs)

Canonical per-arm-per-tick columns (CSV/Parquet/JSONL). Generators/loggers emit the
subset they have; readers tolerate missing columns:

`t, t_source, src_idx, chunk_id, step_id, arm,
source_raw_target[7], conditioned_goal_after_A[7], conditioned_twist_after_A[6],
reference_after_B[7], q_target[6], q_actual[6], actual_tcp[7],
smd_state*, smd_goal_vel[6], vel_clip_flag, acc_clip_flag,
ik_solve_us, ik_pos_err, ik_ori_err, ik_solution_jump_deg, branch_jump_flag,
singular_damping_flag, safety_proj_flag, self_collision_flag, floor_flag, roi_flag,
ma_in[6], ma_out[6]`

Vector columns are flattened with index suffixes (`..._0`.._N) for CSV. Run metadata
sidecar (`run_meta.json`): git commit, config path, full control params, replay mode,
episode id, segment range, seed.

## 8. Output artifact layout (pin this)

```
outputs/tcp_tuning/<episode_id>/
  audit.json
  audit_summary.md
  plots/ (audit_*.png)
  raw_zoh_500hz.npz
  raw_foh_se3_500hz.npz
  clean_foh_se3_500hz.npz
  clean_trajectory_500hz.npz        # the smoothed clean source (pre-FOH reference)
  synthetic_policy_surrogate_500hz.npz   # if generated
  runs/<run_name>/{config.yaml, log.csv, metrics.json, summary.md, plots/}
  analysis/<name>/{metrics.json, summary.md, plots/}   # for analyzing generated npz directly
```
`<episode_id>` = HDF5 parent-dir + stem, e.g. `data_20260619_115712__episode_012`.

## 9. Metrics (`analyze_tcp_replay_logs.py` / `metrics.py`)

Compute, tolerating missing columns (emit `null` + a note when a metric needs an
absent column such as `actual_tcp`):
- Tracking: actual_tcp vs reference_after_B (pos & ori RMS/p95/max); actual vs
  conditioned_goal; reference vs conditioned_goal; cross-correlation lag when actual exists.
- Smoothness: TCP vel/acc/jerk RMS/p95/max; angular vel/acc/jerk; spectral power >5 Hz;
  spectral peak near policy/chunk rate if known; velocity sign-reversals/sec.
- Health: IK solve p50/p95/max, IK failures, branch-jump count, safety/ROI/floor/
  self-collision counts, q_target-vs-q_actual lag — all from log columns if present.
- Output `metrics.json` + `summary.md` + plots (tracking error, vel/acc/jerk,
  FFT/PSD of TCP vel & wrist angular vel, raw_zoh vs raw_foh vs clean comparison).

## 10. Required tests (minimum)

- audit runs without crashing on `episode_012.hdf5`; gap detection works on a synthetic series.
- `raw_zoh` holds value between source frames; `raw_foh_se3` hits exact source poses at
  source timestamps and produces NO one-tick velocity spike for a uniform source ramp.
- quaternion interp handles sign flips; `clean_foh_se3` does not smooth across a gap boundary.
- generated npz contains all required keys; metrics script handles missing `actual_tcp`.

## 11. Experimental protocol the tooling must make easy

Baseline (raw_zoh) vs raw_foh_se3 vs clean_foh_se3 on the same episode, plus later
SMD/FF/MA sweeps. Sweep VALUES are config-driven (do not run real robot). Use the
REAL C++ config key names (verified in 01_inspection_report.md §"CONTRACT Cross-Check");
the left column is the tooling shorthand only:

| shorthand | REAL key | sweep values |
|---|---|---|
| nat_freq_lin | `cartesian_control.pose_track_smd.natural_frequency_linear_hz` | [0.8, 1.0, 1.3, 1.6] |
| nat_freq_ang | `cartesian_control.pose_track_smd.natural_frequency_angular_hz` | [0.6, 0.8, 1.0, 1.2] |
| damp_lin | `cartesian_control.pose_track_smd.damping_ratio_linear` | [1.0, 1.2, 1.5] |
| damp_ang | `cartesian_control.pose_track_smd.damping_ratio_angular` | [1.0, 1.2, 1.5] |
| vel_ff | `cartesian_control.pose_track_smd.velocity_feedforward` | [true, false] |
| max_lin_vel | `cartesian_control.pose_track_smd.max_linear_velocity_m_s` | [0.32, 0.40] |
| max_ang_vel | `cartesian_control.pose_track_smd.max_angular_velocity_rad_s` | [0.8, 1.0] |
| ma_window | `servo.output_moving_average_window` | [1, 4, 8, 12, 16] |

Provide a helper to emit a sweep config matrix (no execution). The emitted matrix
must use the REAL key names so a later sprint can apply it directly.

NOTE (from inspection): the runtime SMD goal is **delta-integrated** and anchored at
`FK(previous_sent_q)` (`smd_pose_tracker.cpp` `updateGoalFromCommand`), NOT a direct
filter of absolute samples; `velocity_feedforward` finite-differences the goal at the
2 ms servo tick. Phase-1 only produces A-stage conditioned absolute goals (reference_after_B
stays NaN); a later offline SMD reference generator must mirror this delta-integrated,
tick-finite-difference behavior to reproduce the one-tick spike.
