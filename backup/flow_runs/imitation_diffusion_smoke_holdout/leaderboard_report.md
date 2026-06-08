# Imitation Learning Leaderboard

## Dataset

- Snapshot hash: `532de3a1c38a4b3f12dcc875d928f723d798a1ee0edf02b99e857d2483e9f9ce`
- Episodes: 146
- Frames: 42435
- Sessions: data_20260606_134608, data_20260606_140442, data_20260606_140813, data_20260606_175002, data_20260606_175635, data_20260606_183038, data_20260606_185446, data_20260606_190156
- Cameras: left_realsense_color, left_realsense_depth, right_realsense_color, right_realsense_depth
- Split mode: `session_holdout`
- Split: session_holdout_by_folder train=96 val=50
- Normalization: train-only, samples=64

## Leaderboard

| Rank | Model | Split | Family | Backbone | Norm MSE | Action MSE | Trans End | Rot End | Grip MSE | P50 Lat ms | Checkpoint |
| ---: | --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | diffusion_tiny | session_holdout | diffusion_policy_x0 | tiny_cnn | 0.395432 | 0.00837559 | 0.00366547 | 0.00883064 | 0.0584915 | 2.978 | `94047413d405895c9508852136c62b14ff61866b7d0ef7f3b26f71dcb628c04f` |

## Recommendation

Next rollout candidate for simulator dry-run is `diffusion_tiny` from family `diffusion_policy_x0`.

## Safety

- Offline HDF5 training/evaluation only.
- No real robot mode, robot connection, or robot motion behavior was touched.
