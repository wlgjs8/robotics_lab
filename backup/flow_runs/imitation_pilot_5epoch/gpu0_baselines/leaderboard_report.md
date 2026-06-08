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
| 1 | state_mlp | state_only_mlp |  | 0.834199 | 1.13268 | 0.00179829 | 0.00657564 | 7.92868 | 0.1077 | `76ed7c87d73a764d6bf819dac052e813bb480c0d993dd8f57c47f64ad080cb6d` |
| 2 | train_mean | constant_baseline |  | 0.977582 | 1.32737 | 0.00285857 | 0.00703895 | 9.29147 | 0 | `` |
| 3 | zero | constant_baseline |  | 0.977606 | 1.3274 | 0.0028188 | 0.00706883 | 9.29169 | 0 | `` |

## Recommendation

Next rollout candidate for simulator dry-run is `state_mlp` from family `state_only_mlp`.

## Safety

- Offline HDF5 training/evaluation only.
- No real robot mode, robot connection, or robot motion behavior was touched.
