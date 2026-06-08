# Imitation Learning Leaderboard

## Dataset

- Snapshot hash: `532de3a1c38a4b3f12dcc875d928f723d798a1ee0edf02b99e857d2483e9f9ce`
- Episodes: 146
- Frames: 42435
- Sessions: data_20260606_134608, data_20260606_140442, data_20260606_140813, data_20260606_175002, data_20260606_175635, data_20260606_183038, data_20260606_185446, data_20260606_190156
- Cameras: left_realsense_color, left_realsense_depth, right_realsense_color, right_realsense_depth
- Split: session_stratified_episode train=117 val=29
- Normalization: train-only, samples=64

## Leaderboard

| Rank | Model | Family | Backbone | Norm MSE | Action MSE | Trans End | Rot End | Grip MSE | P50 Lat ms | Checkpoint |
| ---: | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | flow_tiny | flow_matching | tiny_cnn | 0.581707 | 0.0120876 | 0.00245971 | 0.0113304 | 0.0844157 | 1.097 | `3df9c55e11af1a98a724ecf5bc804eac2b0f379eab98ced2dfeaaa1c675a802b` |
| 2 | act_tiny | act_style_transformer_chunk | tiny_cnn | 1.14968 | 0.02389 | 0.00271094 | 0.0122989 | 0.166983 | 0.8414 | `a05151ba5c00e31325a923f538b7f0a55b5d9a46d125d0713d0238404801f31c` |

## Recommendation

Next rollout candidate for simulator dry-run is `flow_tiny` from family `flow_matching`.

## Safety

- Offline HDF5 training/evaluation only.
- No real robot mode, robot connection, or robot motion behavior was touched.
