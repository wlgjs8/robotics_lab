# Training-episode teacher-forced pgmode replay

Date: 2026-07-14 KST

## Objective

Feed one exact training episode's stored RGB-D frames and recorded velocity
proprioception to the OpenPI checkpoint on port 8000, execute the resulting
`ee_local` deltas through the normal `TcpPoseTarget`/`delta_preview` path, and
measure controller tracking with both RB controllers held in pgmode simulation.
No live camera or physical gripper is in this path.

## Reproducible input

- Raw episode: `episode_002.hdf5`, 120 frames / 119 actions
- Raw episode SHA-256:
  `5a62e5bbdd36428721e3b684980a5206afa5beab34cc2b8609377ff033571fcc`
- Matched LeRobot episode: episode 5 parquet plus four final H264 RGB-D streams
- Retarget source: `calibration/umi_retarget_eelocal.yaml`
- Retarget SHA-256:
  `49790b067025177b6fee7a553e7531cd03be7e6a3f04a9015028ba2a0c773201`
- Checkpoint: `pi05_pika_umi_video_tcp_gripabs_velproprio_depth_z50_h24`,
  step 65000, served at `openpi://127.0.0.1:8000`

The replay recomputes the converter-equivalent 12-D velocity state and 14-D
action from the raw HDF5. When the parquet is supplied, both arrays must match
it within `1e-6` before a command can be emitted. The final H264 frames are used
when supplied; otherwise the raw HDF5 JPEG/PNG frames are decoded.

## Exact replay profile

- H24 checkpoint, W3 execution, 32 teacher-forced inference anchors
- 96 executed policy rows
- `speed_scale=0.1`, effective policy period about 0.334 s
- sequential boundary stitching, chain anchor, no crossfade, no runway
- final `Hold` only after the last W3 controller window has elapsed
- `stack_sim.yaml` `smoothing_window: 1`

The width-1 setting is intentional for literal delta replay. With W3 and the
previous centered width-3 pose filter, integrated poses `[p1,p2,p3]` ended at
approximately `(p2+p3+p3)/3`, losing about one third of the final delta in every
chunk. Ruckig velocity/acceleration/jerk limits and all downstream safety gates
remain enabled.

## Final pgmode result

Trial 11 completed all 32 inferences and all 96 intended rows. Every controller
frame consumed exactly steps `0,1,2`; no runway row was executed.

| Metric | Left | Right |
|---|---:|---:|
| Model predicted translation path | 284.653 mm | 289.467 mm |
| Model predicted translation net | 246.006 mm | 237.465 mm |
| Ground-truth translation net | 245.988 mm | 236.718 mm |
| Final `tcp_ref` net | 244.811 mm | 236.466 mm |
| `tcp_command` to `tcp_ref` mean error | 0.015 mm | 0.019 mm |
| `tcp_command` to `tcp_ref` p99 error | 0.216 mm | 0.210 mm |
| `tcp_command` to `tcp_ref` max error | 0.322 mm | 0.314 mm |
| Max follower actual lead | 0.330 mm | 0.319 mm |
| Max follower projection error | 0.000010 mm | 0.000010 mm |

The model-vs-training action error over all returned H24 chunks was
`pose_mae=0.0007467` per component and `gripper_mae=0.0030286` in model units.
The model output is stochastic at one medoid sample, so trial-to-trial predicted
net displacement is not bit-identical.

The bounded 75 s state monitor received 6,461 packets: every safety verdict was
`Ok`, no fault latched, no mode/physical-motion invariant failed, and physical
`q_actual` displacement was zero. Both controller boxes independently reported
`operation_mode=simulation` and `physical_motion_expected=false`.

## Why the GUI motion previously looked small

There were two separate effects:

1. `q_actual` represents the physical encoders and intentionally remains fixed
   in on-box pgmode. Controller-simulation motion must be judged from `q_ref`,
   `tcp_ref_stand`, or the server command/reference overlays.
2. Early parameter trials faulted after only a small prefix. The first fully
   completing W3 profile still used width-3 smoothing and realized only about
   88.7%/88.5% of the model's net translation. Width-1 replay recovered the
   missing displacement, from 224.0/216.2 mm to 244.8/236.5 mm in the comparable
   controller reference measurement.

This means the remaining discrepancy is not poor pgmode `q_ref` tracking. The
reference follows the server command at sub-millimetre error; the material loss
was the short-window endpoint filter.

## Monitor-only soak evidence

After workspace capacity was restored, a fail-closed one-hour soak was started
with the tracked simulation stack. It completed 22 consecutive full replays
(704 model inferences and 2,112 executed policy rows) over 1,019 seconds before
the controller endpoints went offline during replay 23. The failing packet was
not a collision/floor/ROI rejection: the right rbpodo endpoint first reported
activation stage 0 (`ServoDisabled`) and both `172.28.60.200` and
`172.28.60.201` then became unreachable. The runner stopped immediately and
the stack was shut down. The requested one-hour continuous acceptance therefore
remains incomplete pending controller/Virtual ControlBox recovery.

The read-only monitor captured 83,620 state packets before stopping on the
fault. Of those, 83,619 had `safety_verdict=Ok`; the last packet was the
controller-disconnect fault. Both arms reported `operation_mode=simulation` on
all 167,240 arm samples, with zero `physical_motion_expected` or detected
physical-motion samples.

All simulator barriers were already enabled in monitor-only mode; no further
config weakening was required. Floor, ROI, and user-floor violations were zero,
as were joint-limit and velocity clamps. Self-collision was visually flagged in
49,054 packets, reaching 4.546 mm minimum mesh clearance, but it did not change
the top-level `Ok` verdict or stop the command. During the 60,528 samples where
Cartesian IK was active and successful, simultaneous `tcp_command` to
`tcp_ref` position error remained bounded to 0.430 mm left and 0.429 mm right
(p99 0.320/0.311 mm). This rejects the hypothesis that the red Viser collision
overlay was shortening the replay trajectory. Large command/reference
differences outside active IK are reset/hold transitions and are not tracking
errors.

Sequential inference now sends an explicit `Hold` while the next teacher-forced
chunk is being computed. This disengages the strict preview follower during the
intentional gap while preserving its feed-loss fault if a producer disappears
mid-chunk. Across the 22 complete soak replays, the earlier 1.5-second
chunk-feed timeout did not recur; maximum observed per-cycle model latency was
438.9 ms and maximum inference period was 1,447.7 ms.

## Commands

```bash
make run MODE=sim
./tools/simulation_mode.sh --verify-only

FLOW_TRAINING_EPISODE_HDF5=/path/to/episode_002.hdf5 \
FLOW_TRAINING_EPISODE_VIDEO_DIR=/path/to/lerobot_episode_000005 \
FLOW_TRAINING_EPISODE_PARQUET=/path/to/episode_000005.parquet \
make flow-infer-training-replay
```

Artifacts are written under `outputs/training_episode_replay` by default:
source hashes, raw prediction chunks, matched ground-truth chunks, trajectory
metrics, action log, and rollout summary.

## Acceptance boundary

This is controller-simulation evidence only. It does not authorize physical
motion. `stack_real.yaml`, physical-real Cartesian authority, force-motion, and
real gripper authority were not changed or exercised. Before promoting the
simulation-only smoothing result to the real stack, repeat the physical
acceptance ladder under operator supervision.

The continuous one-hour gate is not claimed: the post-capacity soak stopped at
1,019 seconds when both controller endpoints became unreachable. Resume from a
fresh simulation-mode preflight after the controllers recover rather than
continuing from the faulted stack.

Regression evidence after the soak attempt:

- policy runner: 442 tests passed, 3 skipped in the OpenPI environment
- GUI: 312 tests passed
- Python compileall: passed
- C++ build: passed
- CTest: 37/38 passed; `arm_worker` remains timing-sensitive (four of five
  immediate isolated reruns passed, with the failure moving between unrelated
  asynchronous telemetry assertions)
