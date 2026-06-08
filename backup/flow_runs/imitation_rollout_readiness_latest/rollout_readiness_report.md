# Imitation Rollout Readiness Report

Date: 2026-06-07

Scope: offline policy-artifact readiness only. No robot, simulator, servo-server,
gripper, force-control, or real-mode behavior was executed or changed.

## Leaderboard Snapshot

Source: `outputs/flow_runs/imitation_combined_session_holdout_color384_h1_latest/combined_leaderboard.json`

- Rows: 100
- Source reports: 91
- Failed aggregate rows: 0 observed in the latest aggregation notes
- Split: `session_holdout`
- Dataset/evaluation family: color 384, action horizon 1

## Best Offline Metric

The best validation score is the prediction-average top-5 direct-BC ensemble:

- Model: `ensemble_direct_bc_dinov3_convnext_tiny_top5`
- Family: `direct_bc_checkpoint_ensemble`
- Normalized validation action MSE: `0.3458986436707198`
- Action MSE: `0.4460745515977639`
- Translation endpoint error: `0.0015497340351389095`
- Rotation endpoint error: `0.006470084585592774`
- Median latency: `15.917454962618649 ms`
- SHA-256: `8f5e1ffe621350e21e43e815b9d40d830ee8379014ee9cffb2c8f076b5232590`

Runtime/offline-eval support has been added for the ensemble report JSON. The
runtime loader selects a named ensemble, verifies member checkpoint SHA-256
values when present, and fails closed on mismatched action/camera/stat metadata.

Offline-eval rollout summary:

- Artifact: `outputs/flow_runs/imitation_rollout_readiness_latest/ensemble_direct_bc_dinov3_tiny_top5_offline_eval_summary.json`
- Command: `flow-infer --rollout-mode offline_eval --ensemble-name top5`
- HDF5 samples evaluated: `32`
- Proposed action chunks: `32`
- Image decode count: `64`
- Missing camera count: `0`
- Selected arms: `left`, `right`
- Arm mask: `left=1.0`, `right=1.0`
- Sent command count: `0`
- Dropped command count: `0`
- Physical real motion allowed: `false`
- Real gripper motion allowed: `false`

Interpretation: this is now the quality-first next read-only rollout candidate
when ensemble runtime cost is acceptable. The distilled student remains the
best single-checkpoint low-latency candidate.

## Best Single Experiment Checkpoint

The best single checkpoint by validation score is the cached-distilled direct-BC
student:

- Model: `distill_cached_direct_bc_dinov3_convnext_tiny_h512_hard025_e20`
- Family: `direct_bc_distilled_cached_ensemble`
- Normalized validation action MSE: `0.35817430508126186`
- Train normalized action MSE: `0.21477825340853754`
- Action MSE: `0.4619053744687731`
- Translation endpoint error: `0.0015987699511415426`
- Rotation endpoint error: `0.006623970784539561`
- Median latency: `3.5536929790396243 ms`
- SHA-256: `b5317bb6d38dabc1fe84233430816038ce904068f255a58e6c7a645f6e3f9f44`
- Checkpoint: `/outputs/flow_runs/imitation_distill_cached_direct_tiny_holdout_color384_h1_latest/h512_hard025_e20/direct_bc_dinov3_convnext_tiny_distill/checkpoint.pt`

Container load check before runtime support was added:

- Experiment direct-BC builder: loads and forwards, output shape `[1, 1, 14]`,
  finite output.
- Old `FlowMatchingActionSource` path: rejected with
  `ValueError: unsupported flow checkpoint schema: robotics_lab.policy_runner.imitation_checkpoint.v1`.

Runtime load check after adding guarded direct-BC image checkpoint support:

- `action_chunk_checkpoint_kind`: `direct_bc`
- `DirectBcImageActionSource`: loads on CPU with `--image-size 384`
- Command family: `TcpTwistStand`
- Cameras: `left_realsense_color`, `right_realsense_color`
- Selected arms: `left`, `right`
- Arm mask: `[1.0, 1.0]`
- Camera input requirement: true

Offline-eval rollout summary:

- Artifact: `outputs/flow_runs/imitation_rollout_readiness_latest/distill_direct_bc_h512_hard025_e20_offline_eval_summary.json`
- Command: `flow-infer --rollout-mode offline_eval --image-size 384`
- HDF5 samples evaluated: `32`
- Proposed action chunks: `32`
- Image decode count: `64`
- Missing camera count: `0`
- Selected arms: `left`, `right`
- Arm mask: `left=1.0`, `right=1.0`
- Sent command count: `0`
- Dropped command count: `0`
- Physical real motion allowed: `false`
- Real gripper motion allowed: `false`

Interpretation: this is now the preferred single-checkpoint candidate for the
next policy-runner read-only check. Because the checkpoint did not store
`image_size`, invocations must pass `--image-size 384` to match training.

## Existing Flow-Compatible Fallback

The best checked flow-matching fallback remains:

- Model: `flow_dinov3_convnext_tiny`
- Family: `flow_matching`
- Normalized validation action MSE: `0.3865199116943731`
- Median latency: approximately `12.8775 ms`
- SHA-256: `447a5317140b...`
- Checkpoint: `/outputs/flow_runs/imitation_flow_dinov3_tiny_holdout_color384_h1_seed9_lr3e4_steps4/flow_dinov3_convnext_tiny/checkpoint.pt`

Container load check:

- `load_flow_checkpoint_dataset_stats`: passed.
- `FlowMatchingActionSource`: passed on CPU construction with `sample_steps=1`.

Offline-eval rollout summary:

- Artifact: `outputs/flow_runs/imitation_rollout_readiness_latest/flow_dinov3_tiny_seed9_lr3e4_steps4_offline_eval_summary.json`
- Command: `flow-infer --rollout-mode offline_eval --sample-steps 4`
- HDF5 samples evaluated: `32`
- Proposed action chunks: `32`
- Image decode count: `0`
- Missing camera count: `0`
- Selected arms: `left`, `right`
- Arm mask: `left=1.0`, `right=1.0`
- Sent command count: `0`
- Dropped command count: `0`
- Physical real motion allowed: `false`
- Real gripper motion allowed: `false`

Interpretation: use this when the next check specifically needs the flow model
family. It is no longer the best runtime-compatible single checkpoint overall.

## Recommendation

1. Quality-first next check: use the top-5 direct-BC ensemble report through
   `flow-infer --rollout-mode real_readonly --ensemble-name top5`, preserving
   existing rollout gates and camera freshness requirements.
2. Single-checkpoint next check: use the h512 hard-0.25 epoch-20 cached-distill
   student through `flow-infer --rollout-mode real_readonly --image-size 384`.
3. Keep the seed-9 flow-matching checkpoint as the flow-family fallback for
   comparisons or flow-specific runtime checks.

Safety note: these results are offline artifact evidence only. They do not
authorize real robot, real Cartesian, gripper, or force-control execution.
