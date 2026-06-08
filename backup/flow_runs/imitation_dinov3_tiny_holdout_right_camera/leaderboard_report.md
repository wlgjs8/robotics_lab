# Imitation Learning Leaderboard

## Dataset

- Snapshot hash: `532de3a1c38a4b3f12dcc875d928f723d798a1ee0edf02b99e857d2483e9f9ce`
- Episodes: 146
- Frames: 42435
- Sessions: data_20260606_134608, data_20260606_140442, data_20260606_140813, data_20260606_175002, data_20260606_175635, data_20260606_183038, data_20260606_185446, data_20260606_190156
- Cameras: left_realsense_color, left_realsense_depth, right_realsense_color, right_realsense_depth
- Split mode: `session_holdout`
- Split: session_holdout_by_folder train=96 val=50
- Normalization: train-only, samples=26814

## Leaderboard

| Rank | Model | Split | Family | Backbone | Train Norm MSE | Norm MSE | Action MSE | Trans End | Rot End | Grip MSE | P50 Lat ms | Checkpoint |
| ---: | --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | direct_bc_dinov3_convnext_tiny | session_holdout | direct_bc_chunk | dinov3_convnext_tiny | 0.702859 | 0.844059 | 1.12571 | 0.00177945 | 0.00674079 | 7.87988 | 1.894 | `d51b3f4b72b4f49a897607c2f1d65dbdadfe7fb33ccd9771457d245d6e7a4877` |

## Recommendation

Next rollout candidate for simulator dry-run is `direct_bc_dinov3_convnext_tiny` from family `direct_bc_chunk`.

## Safety

- Offline HDF5 training/evaluation only.
- No real robot mode, robot connection, or robot motion behavior was touched.
