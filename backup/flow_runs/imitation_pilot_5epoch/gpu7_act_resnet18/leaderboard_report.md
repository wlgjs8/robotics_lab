# Imitation Learning Leaderboard

## Dataset

- Snapshot hash: `532de3a1c38a4b3f12dcc875d928f723d798a1ee0edf02b99e857d2483e9f9ce`
- Episodes: 146
- Frames: 42435
- Sessions: data_20260606_134608, data_20260606_140442, data_20260606_140813, data_20260606_175002, data_20260606_175635, data_20260606_183038, data_20260606_185446, data_20260606_190156
- Cameras: left_realsense_color, left_realsense_depth, right_realsense_color, right_realsense_depth
- Split: session_stratified_episode train=117 val=29
- Normalization: train-only, samples=32112

## Leaderboard

| Rank | Model | Family | Backbone | Norm MSE | Action MSE | Trans End | Rot End | Grip MSE | P50 Lat ms | Checkpoint |
| ---: | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | act_resnet18 | act_style_transformer_chunk | resnet18 | 0.782462 | 1.06243 | 0.00171198 | 0.00670573 | 7.43694 | 1.273 | `6ee09f78d64cff4e13eb905960fc8dfd2ba26216e4e6ffad7d369c2a222db668` |

## Recommendation

Next rollout candidate for simulator dry-run is `act_resnet18` from family `act_style_transformer_chunk`.

## Safety

- Offline HDF5 training/evaluation only.
- No real robot mode, robot connection, or robot motion behavior was touched.
