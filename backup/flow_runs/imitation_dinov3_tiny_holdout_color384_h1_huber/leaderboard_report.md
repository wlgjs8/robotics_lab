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
| 1 | direct_bc_dinov3_convnext_tiny_huber | session_holdout | direct_bc_chunk | dinov3_convnext_tiny | 0.703032 | 0.739209 | 0.953292 | 0.00163639 | 0.00658376 | 6.67296 | 3.51 | `4ffe6dff28449185d849585d06ad263e421326891ca412171e15acae7bbc845c` |

## Recommendation

Next rollout candidate for simulator dry-run is `direct_bc_dinov3_convnext_tiny_huber` from family `direct_bc_chunk`.

## Safety

- Offline HDF5 training/evaluation only.
- No real robot mode, robot connection, or robot motion behavior was touched.
