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

| Rank | Model | Split | Family | Backbone | Norm MSE | Action MSE | Trans End | Rot End | Grip MSE | P50 Lat ms | Checkpoint |
| ---: | --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | structured_dinov3_convnext_tiny | session_holdout | arm_structured_direct | dinov3_convnext_tiny | 0.906336 | 1.20877 | 0.00175969 | 0.00665772 | 8.46129 | 2.003 | `e096f22f3f68c7d61881285c57d9822e15cde9bda54dd6a4be396976d5f60801` |

## Recommendation

Next rollout candidate for simulator dry-run is `structured_dinov3_convnext_tiny` from family `arm_structured_direct`.

## Safety

- Offline HDF5 training/evaluation only.
- No real robot mode, robot connection, or robot motion behavior was touched.
