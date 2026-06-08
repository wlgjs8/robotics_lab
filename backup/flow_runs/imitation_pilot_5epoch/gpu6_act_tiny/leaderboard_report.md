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
| 1 | act_tiny | act_style_transformer_chunk | tiny_cnn | 0.819056 | 1.11212 | 0.00181673 | 0.0067889 | 7.78475 | 0.4847 | `ca7b5e8dcb2276e7b059be78b2d9a6b8781021f5c34dbbda6b85c00646219e87` |

## Recommendation

Next rollout candidate for simulator dry-run is `act_tiny` from family `act_style_transformer_chunk`.

## Safety

- Offline HDF5 training/evaluation only.
- No real robot mode, robot connection, or robot motion behavior was touched.
