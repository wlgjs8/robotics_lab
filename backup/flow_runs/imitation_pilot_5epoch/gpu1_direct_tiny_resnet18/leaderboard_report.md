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
| 1 | direct_bc_resnet18 | direct_bc_chunk | resnet18 | 0.806603 | 1.09521 | 0.00171339 | 0.00652529 | 7.66639 | 1.067 | `0d0859b0e1d42b3b253e6729a984ba2ee44c5be3b953d8131275fa1795ca1e90` |
| 2 | direct_bc_tiny | direct_bc_chunk | tiny_cnn | 0.813017 | 1.10392 | 0.00175411 | 0.00654296 | 7.72736 | 0.189 | `261e89940ee355bdc2eef868acb84393a53a5f0b33eb3de33e84c7075ad80710` |

## Recommendation

Next rollout candidate for simulator dry-run is `direct_bc_resnet18` from family `direct_bc_chunk`.

## Safety

- Offline HDF5 training/evaluation only.
- No real robot mode, robot connection, or robot motion behavior was touched.
