# Imitation Learning Leaderboard

## Dataset

- Snapshot hash: `532de3a1c38a4b3f12dcc875d928f723d798a1ee0edf02b99e857d2483e9f9ce`
- Episodes: 146
- Frames: 42435
- Sessions: data_20260606_134608, data_20260606_140442, data_20260606_140813, data_20260606_175002, data_20260606_175635, data_20260606_183038, data_20260606_185446, data_20260606_190156
- Cameras: left_realsense_color, left_realsense_depth, right_realsense_color, right_realsense_depth
- Split mode: `session_holdout`
- Split: session_holdout_by_folder train=96 val=50
- Normalization: train-only, samples=96

## Leaderboard

| Rank | Model | Split | Family | Backbone | Norm MSE | Action MSE | Trans End | Rot End | Grip MSE | P50 Lat ms | Checkpoint |
| ---: | --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | train_mean | session_holdout | constant_baseline |  | 5.90422 | 0.21731 | 0.00341824 | 0.0101991 | 1.521 | 0 | `` |
| 2 | state_mlp | session_holdout | state_only_mlp |  | 6.08804 | 0.224076 | 0.00357768 | 0.00956237 | 1.56835 | 0.07396 | `7b7ed9607536544381ce035f434c5cda5808555bca49f424317ef8d23616a294` |
| 3 | zero | session_holdout | constant_baseline |  | 6.64714 | 0.244654 | 0.00302984 | 0.00686506 | 1.71249 | 0 | `` |

## Recommendation

No learned rollout candidate is recommended yet because the best validation score is a constant baseline. Train longer or revise the model/data setup before simulator rollout.

## Safety

- Offline HDF5 training/evaluation only.
- No real robot mode, robot connection, or robot motion behavior was touched.
