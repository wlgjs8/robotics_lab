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
| 1 | state_mlp | session_holdout | state_only_mlp |  | 0.865241 | 1.15396 | 0.00178604 | 0.00644738 | 8.07764 | 0.1051 | `55433c91f0122d9638567a4d4ab03b147d21f1c125922df93b18f1f7aeae4ef2` |
| 2 | train_mean | session_holdout | constant_baseline |  | 1.04087 | 1.38819 | 0.00294744 | 0.00693174 | 9.71726 | 0 | `` |
| 3 | zero | session_holdout | constant_baseline |  | 1.0409 | 1.38823 | 0.00289302 | 0.00695194 | 9.71749 | 0 | `` |

## Recommendation

Next rollout candidate for simulator dry-run is `state_mlp` from family `state_only_mlp`.

## Safety

- Offline HDF5 training/evaluation only.
- No real robot mode, robot connection, or robot motion behavior was touched.
