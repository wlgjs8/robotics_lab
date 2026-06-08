# Imitation Learning Leaderboard

## Dataset

- Snapshot hash: `532de3a1c38a4b3f12dcc875d928f723d798a1ee0edf02b99e857d2483e9f9ce`
- Episodes: 146
- Frames: 42435
- Sessions: data_20260606_134608, data_20260606_140442, data_20260606_140813, data_20260606_175002, data_20260606_175635, data_20260606_183038, data_20260606_185446, data_20260606_190156
- Cameras: left_realsense_color, left_realsense_depth, right_realsense_color, right_realsense_depth
- Split mode: `primary`
- Split: session_stratified_episode train=117 val=29
- Normalization: train-only, samples=32112

## Leaderboard

| Rank | Model | Split | Family | Backbone | Norm MSE | Action MSE | Trans End | Rot End | Grip MSE | P50 Lat ms | Checkpoint |
| ---: | --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | direct_bc_dinov3_convnext_small | primary | direct_bc_chunk | dinov3_convnext_small | 0.762513 | 1.03534 | 0.0016626 | 0.00670072 | 7.24733 | 3.131 | `f9656061172a5f09be9988c6f8499bd5e8940290fa91d6ab7a521c414781592e` |

## Recommendation

Next rollout candidate for simulator dry-run is `direct_bc_dinov3_convnext_small` from family `direct_bc_chunk`.

## Warnings

- primary split training includes episodes from session_holdout_val; use --split-mode session_holdout for strict held-session evaluation

## Safety

- Offline HDF5 training/evaluation only.
- No real robot mode, robot connection, or robot motion behavior was touched.
