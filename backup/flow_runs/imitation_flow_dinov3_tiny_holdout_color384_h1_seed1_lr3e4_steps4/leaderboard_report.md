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
| 1 | flow_dinov3_convnext_tiny | session_holdout | flow_matching | dinov3_convnext_tiny | 0.442886 | 0.500946 | 0.646026 | 0.00159199 | 0.00657828 | 4.5221 | 12.85 | `9f0cea063f3af805e18c72b7b455c5b2c38308a6351a066e25aafa3b8096c88a` |

## Recommendation

Next rollout candidate for simulator dry-run is `flow_dinov3_convnext_tiny` from family `flow_matching`.

## Safety

- Offline HDF5 training/evaluation only.
- No real robot mode, robot connection, or robot motion behavior was touched.
