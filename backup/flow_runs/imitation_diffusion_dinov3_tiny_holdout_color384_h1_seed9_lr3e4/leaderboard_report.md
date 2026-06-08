# Imitation Learning Leaderboard

## Dataset

- Snapshot hash: `532de3a1c38a4b3f12dcc875d928f723d798a1ee0edf02b99e857d2483e9f9ce`
- Episodes: 146
- Frames: 42435
- Sessions: data_20260606_134608, data_20260606_140442, data_20260606_140813, data_20260606_175002, data_20260606_175635, data_20260606_183038, data_20260606_185446, data_20260606_190156
- Cameras: left_realsense_color, left_realsense_depth, right_realsense_color, right_realsense_depth
- Split mode: `session_holdout`
- Split: session_holdout_by_folder train=96 val=50
- Normalization: train-only, samples=28254

## Leaderboard

| Rank | Model | Split | Family | Backbone | Train Norm MSE | Norm MSE | Action MSE | Trans End | Rot End | Grip MSE | P50 Lat ms | Checkpoint |
| ---: | --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | diffusion_dinov3_convnext_tiny | session_holdout | diffusion_policy_x0 | dinov3_convnext_tiny | 0.928611 | 0.932901 | 1.20308 | 0.00200599 | 0.00901308 | 8.42141 | 28.49 | `c048f5d04c631a11a3a2fcc809891a0e9282d1befb7bcba51cc4b37cd2398c8a` |

## Recommendation

Next rollout candidate for simulator dry-run is `diffusion_dinov3_convnext_tiny` from family `diffusion_policy_x0`.

## Safety

- Offline HDF5 training/evaluation only.
- No real robot mode, robot connection, or robot motion behavior was touched.
