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
| 1 | structured_tiny | arm_structured_direct | tiny_cnn | 0.63734 | 0.0132437 | 0.00250907 | 0.011356 | 0.092487 | 0.6404 | `ba72801a351f70e746f44e6be3f66f2586ca74a1d72d1b31644a6cad6d21a59e` |
| 2 | direct_bc_resnet18_huber | direct_bc_chunk | resnet18 | 0.649239 | 0.013491 | 0.00255727 | 0.0123319 | 0.0942174 | 1.07 | `113c87be485032c186d65b3f514157f2e2eac4aeac085804ebfcebe2dedbef70` |

## Recommendation

Next rollout candidate for simulator dry-run is `structured_tiny` from family `arm_structured_direct`.

## Failed Runs

- `does_not_exist`: ValueError: unknown imitation model family: does_not_exist

## Safety

- Offline HDF5 training/evaluation only.
- No real robot mode, robot connection, or robot motion behavior was touched.
