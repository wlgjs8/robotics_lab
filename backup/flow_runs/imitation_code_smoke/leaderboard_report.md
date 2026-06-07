# Imitation Learning Leaderboard

## Dataset

- Snapshot hash: `1e172aeb56f4e8e35ed4d24d6eda5cfacab7e86a56cec4329a64fdef6a4373f2`
- Episodes: 146
- Frames: 42435
- Sessions: data_20260606_134608, data_20260606_140442, data_20260606_140813, data_20260606_175002, data_20260606_175635, data_20260606_183038, data_20260606_185446, data_20260606_190156
- Cameras: left_realsense_color, left_realsense_depth, right_realsense_color, right_realsense_depth
- Split: session_stratified_episode train=117 val=29
- Normalization: train-only, samples=128

## Leaderboard

| Rank | Model | Family | Backbone | Norm MSE | Action MSE | Trans End | Rot End | Grip MSE | P50 Lat ms | Checkpoint |
| ---: | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | zero | constant_baseline |  | 0.24894 | 0.0175533 | 0.00242508 | 0.00830711 | 0.122757 | 0 | `` |
| 2 | train_mean | constant_baseline |  | 0.41874 | 0.0295262 | 0.00228661 | 0.0115342 | 0.206475 | 0 | `` |
| 3 | direct_bc_tiny | direct_bc_chunk | tiny_cnn | 0.430002 | 0.0303204 | 0.00214091 | 0.0110812 | 0.212037 | 0.3276 | `9215e00d9c2604b664c0fb1407588317f2f5fc75ed82538cba51c15ec8b38ffc` |
| 4 | state_only_mlp | state_only_mlp |  | 0.847906 | 0.0597876 | 0.00232783 | 0.01095 | 0.418293 | 0.1091 | `566348f3aa9c8df6290092489945c1fbb0e940e5fd21e991f48ddaebd1da827c` |

## Recommendation

Next rollout candidate for simulator dry-run is `zero` from family `constant_baseline`.

## Safety

- Offline HDF5 training/evaluation only.
- No real robot mode, robot connection, or robot motion behavior was touched.
