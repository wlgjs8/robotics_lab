# Imitation Learning Leaderboard

## Dataset

- Snapshot hash: `532de3a1c38a4b3f12dcc875d928f723d798a1ee0edf02b99e857d2483e9f9ce`
- Episodes: 146
- Frames: 42435
- Sessions: data_20260606_134608, data_20260606_140442, data_20260606_140813, data_20260606_175002, data_20260606_175635, data_20260606_183038, data_20260606_185446, data_20260606_190156
- Cameras: left_realsense_color, left_realsense_depth, right_realsense_color, right_realsense_depth
- Split mode: `primary`
- Split: session_stratified_episode train=117 val=29
- Normalization: train-only, samples=32

## Leaderboard

| Rank | Model | Split | Family | Backbone | Train Norm MSE | Norm MSE | Action MSE | Trans End | Rot End | Grip MSE | P50 Lat ms | Checkpoint |
| ---: | --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | zero | primary | constant_baseline |  | 1.16922 | 0.0176504 | 0.000374858 | 0.00268808 | 0.00877409 | 0.00247281 | 0 | `` |
| 2 | direct_bc_dinov3_convnext_tiny | primary | direct_bc_chunk | dinov3_convnext_tiny | 1.00577 | 0.211049 | 0.00448226 | 0.00275748 | 0.00842202 | 0.0311972 | 1.832 | `a0e5d8005c7f5f84a810631b00c4c763606a324a3772886c12bf0fb14ce614b8` |

## Recommendation

No learned rollout candidate is recommended yet because the best validation score is a constant baseline. Train longer or revise the model/data setup before simulator rollout.

## Warnings

- primary split training includes episodes from session_holdout_val; use --split-mode session_holdout for strict held-session evaluation

## Safety

- Offline HDF5 training/evaluation only.
- No real robot mode, robot connection, or robot motion behavior was touched.
