# Imitation Learning Leaderboard

## Dataset

- Snapshot hash: `532de3a1c38a4b3f12dcc875d928f723d798a1ee0edf02b99e857d2483e9f9ce`
- Episodes: 146
- Frames: 42435
- Sessions: data_20260606_134608, data_20260606_140442, data_20260606_140813, data_20260606_175002, data_20260606_175635, data_20260606_183038, data_20260606_185446, data_20260606_190156
- Cameras: left_realsense_color, left_realsense_depth, right_realsense_color, right_realsense_depth
- Split: session_stratified_episode train=117 val=29
- Normalization: train-only, samples=30240

## Leaderboard

| Rank | Model | Family | Backbone | Norm MSE | Action MSE | Trans End | Rot End | Grip MSE | P50 Lat ms | Checkpoint |
| ---: | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | act_resnet18 | act_style_transformer_chunk | resnet18 | 1.05909 | 1.29144 | 0.00203244 | 0.00743525 | 9.04001 | 1.373 | `91bfd7e290c5fe5f6a703fcbaaa37524b9abb577b145894ec9c035a322fd2937` |

## Recommendation

Next rollout candidate for simulator dry-run is `act_resnet18` from family `act_style_transformer_chunk`.

## Safety

- Offline HDF5 training/evaluation only.
- No real robot mode, robot connection, or robot motion behavior was touched.
