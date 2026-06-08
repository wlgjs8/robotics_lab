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
| 1 | direct_bc_resnet50 | direct_bc_chunk | resnet50 | 0.899118 | 1.22083 | 0.00208344 | 0.00678259 | 8.54571 | 2.212 | `6aad8c02504cf400610a46c47a62c1fccf87b516f1b4c1c3c4600df8f786934e` |

## Recommendation

Next rollout candidate for simulator dry-run is `direct_bc_resnet50` from family `direct_bc_chunk`.

## Safety

- Offline HDF5 training/evaluation only.
- No real robot mode, robot connection, or robot motion behavior was touched.
