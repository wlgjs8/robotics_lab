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
| 1 | state_mlp | session_holdout | state_only_mlp |  | 0.443924 | 0.786908 | 1.0148 | 0.00182412 | 0.0068492 | 7.10355 | 0.0597 | `31669f78f3f17477b813aa6e34a28d7001eedb156aac65940aec65b5d9bb6fac` |
| 2 | train_mean | session_holdout | constant_baseline |  | 1 | 1.04494 | 1.34756 | 0.00290648 | 0.00712964 | 9.43285 | 0 | `` |
| 3 | zero | session_holdout | constant_baseline |  | 1 | 1.04494 | 1.34757 | 0.0028724 | 0.00715512 | 9.43285 | 0 | `` |

## Recommendation

Next rollout candidate for simulator dry-run is `state_mlp` from family `state_only_mlp`.

## Safety

- Offline HDF5 training/evaluation only.
- No real robot mode, robot connection, or robot motion behavior was touched.
