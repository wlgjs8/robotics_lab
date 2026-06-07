# Imitation Learning Leaderboard

## Dataset

- Snapshot hash: `532de3a1c38a4b3f12dcc875d928f723d798a1ee0edf02b99e857d2483e9f9ce`
- Episodes: 146
- Frames: 42435
- Sessions: data_20260606_134608, data_20260606_140442, data_20260606_140813, data_20260606_175002, data_20260606_175635, data_20260606_183038, data_20260606_185446, data_20260606_190156
- Cameras: left_realsense_color, left_realsense_depth, right_realsense_color, right_realsense_depth
- Split: session_stratified_episode train=117 val=29
- Normalization: train-only, samples=256

## Leaderboard

| Rank | Model | Family | Backbone | Norm MSE | Action MSE | Trans End | Rot End | Grip MSE | P50 Lat ms | Checkpoint |
| ---: | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | train_mean | constant_baseline |  | 0.53716 | 0.203256 | 0.00211983 | 0.00737638 | 1.4227 | 0 | `` |
| 2 | state_mlp | state_only_mlp |  | 0.550772 | 0.208407 | 0.00210986 | 0.00746892 | 1.45876 | 0.1109 | `1b12992085e90ce73fec44e135b7265e424f01c7c89bf23a643df5d5170d6cc3` |
| 3 | zero | constant_baseline |  | 0.565733 | 0.214068 | 0.00222377 | 0.00658429 | 1.49839 | 0 | `` |

## Recommendation

No learned rollout candidate is recommended yet because the best validation score is a constant baseline. Train longer or revise the model/data setup before simulator rollout.

## Safety

- Offline HDF5 training/evaluation only.
- No real robot mode, robot connection, or robot motion behavior was touched.
