# Real TcpPoseTarget Replay Runbook

This runbook is for the fail-closed Python replay driver:

```bash
python3 scripts/replay_episode_tcp_pose_target.py
```

The driver streams Phase-1 conditioned `TcpPoseTarget` setpoints to an already
running `rb_servo_server`. The human operator triggers physical motion. The
script defaults to dry-run and sends no UDP motion command unless every execute
gate is present.

For recorded UMI episodes, the supported real replay path is **ee_local anchored
replay**:

```text
raw UMI HDF5 -> data_tcp action/target_pose_* -> pose_delta_local body deltas
  -> live actual TCP anchor -> absolute stand-frame TcpPoseTarget setpoints
```

Raw absolute UMI replay is invalid for these episodes. The raw/data_tcp absolute
poses are in `steamvr_world`; values such as `z ~= -2.5 m` are not stand-frame
robot targets and must never be sent as absolute `TcpPoseTarget` commands. The
ee_local path uses only adjacent body-frame deltas and composes them from the
robot's current stand-frame TCP, so the first composed setpoint is exactly the
anchor (`init_delta == 0`). Reach is controlled with `--action-scale`; `1.0` is
faithful amplitude, `0.5` halves each per-step body delta.

## Preconditions

- Physical cell is clear.
- Operator is at the E-stop and can stop motion immediately.
- Both RB3-730 controllers are powered, enabled, and in the expected pgmode
  state for the site-local real config.
- The real server is already running from the accepted site-local config, for
  example `rb_servo_server/config/local/stack_real.yaml`.
- Command-source lease is free; stop GUI/teleop/policy clients that may hold it.
- The selected trajectory has already passed dry-run bounds checks with no
  `VIOLATION:` lines.
- `RB_ALLOW_REAL_ROBOT=1`, `RB_ALLOW_REAL_MOTION=1`, and
  `RB_ALLOW_REAL_CARTESIAN=1` are set only in the terminal that starts the real
  server. The replay script itself does not bypass server gates.

## Convert

Convert raw UMI episodes to `data_tcp` before replay. Do not pass
`--require-measured-retarget`; `calibration/umi_retarget_eelocal.yaml` is a
configured-estimate tool-offset-only retarget and is valid for ee_local because
the unmeasured world rotation cancels in `pose_delta_local`.

```bash
python3 -m policy_runner umi-convert \
  --input data/data_20260619_115712/episode_012.hdf5 \
  --output data_tcp/data_20260619_115712/episode_012.hdf5 \
  --format robotics_lab_dual_arm \
  --retarget-config calibration/umi_retarget_eelocal.yaml
```

Verify the output contains:

```text
action/tcp_target_stand_left
action/tcp_target_stand_right
```

## Ee-Local Dry-Run

Run this before any execute attempt. Use a mock stand-frame pose for offline
screening, or omit `--anchor mock` only when reading a live server state is
intended.

```bash
python3 scripts/replay_episode_tcp_pose_target.py \
  --source ee_local \
  --data-tcp data_tcp/data_20260619_115712/episode_012.hdf5 \
  --mode clean_foh_se3 \
  --arms left,right \
  --anchor mock \
  --mock-current-pose default \
  --segment auto-largest \
  --action-scale 1.0 \
  --server-config rb_servo_server/config/local/stack_real.yaml \
  --out-dir outputs/tcp_tuning
```

Expected first line:

```text
DRY RUN — no motion sent
```

Read the output before any execute attempt:

- `bounds:` lines must have no `VIOLATION:` entries for a real execute run.
- `segment_selection` shows the selected source-frame range, total segments, and
  dropped frames. Use `--segment auto-largest` for the first gap-free real test;
  it selects the longest contiguous source-sample run using the Phase-1 gap
  segmentation (`dt > 3 * median_dt` or `dt > 0.100 s`). Use `--segment <int>`
  only after reviewing the segment list. `--segment all` preserves historical
  behavior and may include gap reanchors.
- Real replay MUST NOT use a full-episode `clean_foh_se3` NPZ -- it carries a
  one-tick velocity spike at each gap->next-segment boundary. Use a gap-free
  single segment: either `generate_replay_target.py --segment auto-largest|N|start:stop`,
  or the runtime path `scripts/replay_episode_tcp_pose_target.py --segment auto-largest|1`
  which segments before conditioning.
- `init_delta` is current TCP to first conditioned setpoint. In ee_local anchored
  replay it should be zero by construction; nonzero means the anchor path or
  generated target should be inspected before continuing.
- `first_pose`, `last_pose`, `setpoint_min_xyz`, and `setpoint_max_xyz` show the
  Cartesian envelope that will be sent.
- Repeat with a smaller `--action-scale` if the envelope leaves floor/ROI or
  known reach. Use the largest scale that passes dry-run bounds and operator
  review.
- `client_speed_clamp` defaults from
  `cartesian_control.pose_track_smd.max_linear_velocity_m_s` and
  `max_angular_velocity_rad_s` in `--server-config`. Explicit
  `--max-linear-speed-m-s` / `--max-angular-speed-rad-s` values still override.
  The client clamp is a belt-and-suspenders precheck and should match the server
  SMD cap, not undercut normal UMI replay speed.
- `stream_speed_max` must be within the client clamp limits, or execute refuses.
- `--time-scale <value>` slows playback uniformly; values must be `>= 1.0`.
  `--time-scale 2.0` is the cautious first-run setting when the operator wants
  extra margin or when default-speed angular motion exceeds the current server
  cap.
- `planned init pre-move` shows the slow pre-position segment endpoints.
- `would-be log written` points to `log.csv` and `run_meta.json`.

Absolute replay via `--source absolute --episode ...` or a generated absolute
NPZ is only valid for already accepted stand-frame target sets. It is invalid
for raw UMI `steamvr_world` episodes.

## Execute

Only after a clean ee_local dry-run and operator review, run with all gates. The
execute path anchors from each arm's live `tcp_actual_stand`; do not use
`--anchor mock` for execute.

```bash
python3 scripts/replay_episode_tcp_pose_target.py \
  --source ee_local \
  --data-tcp data_tcp/data_20260619_115712/episode_012.hdf5 \
  --mode clean_foh_se3 \
  --arms left,right \
  --anchor live \
  --segment auto-largest \
  --action-scale 1.0 \
  --server-config rb_servo_server/config/local/stack_real.yaml \
  --out-dir outputs/tcp_tuning \
  --execute \
  --i-am-at-the-estop
```

The script then asks for two typed confirmations:

```text
Physical motion gate 1: type left,right to confirm:
Physical motion gate 2 after init pre-move: type left,right to stream:
```

No motion command is sent before the first confirmation. The stream does not
start until after the slow init pre-move completes and the second confirmation is
typed.

For the extra-cautious first physical run, use the same human gates with
`--time-scale 2.0`:

```bash
python3 scripts/replay_episode_tcp_pose_target.py \
  --source ee_local \
  --data-tcp data_tcp/data_20260619_115712/episode_012.hdf5 \
  --mode clean_foh_se3 \
  --arms left,right \
  --anchor live \
  --segment auto-largest \
  --action-scale 1.0 \
  --time-scale 2.0 \
  --server-config rb_servo_server/config/local/stack_real.yaml \
  --out-dir outputs/tcp_tuning \
  --execute \
  --i-am-at-the-estop
```

## Telemetry To Watch

Watch the server state stream, logs, GUI, or PlotJuggler for:

- `fault_latched`
- `safety_verdict`
- `tracking_error_degraded` / tracking-error latch fields
- `command_source` active source/session
- per-arm `cartesian_solve` IK duration, timeout, and branch-jump fields
- `q_actual_deg`, `q_target_deg`, `tcp_actual_stand`, and `tcp_ref_stand`
- floor, ROI, and self-collision indicators exposed by the server

On any watchdog condition, the script sends `Hold`, releases the lease
best-effort, logs the cause, and exits non-zero.

## Abort

- Press the physical E-stop for immediate hardware stop.
- Press `Ctrl-C` in the replay terminal. The script sends `Hold` and releases
  the command-source lease best-effort.
- If a server fault latches, stop the script, inspect the server state/logs, and
  do not reset faults until the cell and commanded trajectory are understood.
