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
| 1 | diffusion_resnet18 | primary | diffusion_policy_x0 | resnet18 | 0.946569 | 1.28526 | 0.0023086 | 0.00906068 | 8.99666 | 20 | `beeae9a445fa7831d967df9a6ee62b50dc89a683835b2f14ab2390368f15f082` |

## Recommendation

Next rollout candidate for simulator dry-run is `diffusion_resnet18` from family `diffusion_policy_x0`.

## Warnings

- primary split training includes episodes from session_holdout_val; use --split-mode session_holdout for strict held-session evaluation

## Safety

- Offline HDF5 training/evaluation only.
- No real robot mode, robot connection, or robot motion behavior was touched.
