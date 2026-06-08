# Imitation Learning Leaderboard

## Dataset

- Snapshot hash: `532de3a1c38a4b3f12dcc875d928f723d798a1ee0edf02b99e857d2483e9f9ce`
- Episodes: 146
- Frames: 42435
- Sessions: data_20260606_134608, data_20260606_140442, data_20260606_140813, data_20260606_175002, data_20260606_175635, data_20260606_183038, data_20260606_185446, data_20260606_190156
- Cameras: left_realsense_color, left_realsense_depth, right_realsense_color, right_realsense_depth
- Split: session_stratified_episode train=117 val=29
- Normalization: train-only, samples=128

## Leaderboard

| Rank | Model | Family | Backbone | Norm MSE | Action MSE | Trans End | Rot End | Grip MSE | P50 Lat ms | Checkpoint |
| ---: | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | zero | constant_baseline |  | 0.2597 | 0.0183123 | 0.00234111 | 0.00757779 | 0.128082 | 0 | `` |
| 2 | train_mean | constant_baseline |  | 0.406507 | 0.0286641 | 0.00225149 | 0.0105973 | 0.200457 | 0 | `` |
| 3 | state_mlp | state_only_mlp |  | 0.640048 | 0.0451318 | 0.00276052 | 0.010181 | 0.315707 | 0.04173 | `1be8f509ae749ed80efc24259b0758f86f80ce692a5ec8aec7c786072ee67c19` |

## Recommendation

No learned rollout candidate is recommended yet because the best validation score is a constant baseline. Train longer or revise the model/data setup before simulator rollout.

## Safety

- Offline HDF5 training/evaluation only.
- No real robot mode, robot connection, or robot motion behavior was touched.
