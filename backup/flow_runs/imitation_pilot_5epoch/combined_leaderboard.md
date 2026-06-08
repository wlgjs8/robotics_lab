# Combined Imitation Pilot Leaderboard

5 epoch full-data pilot on the fixed episode-level split.

| Rank | Model | Family | Backbone | Norm MSE | Action MSE | Trans End | Rot End | Grip MSE | P50 Lat ms | Source |
| ---: | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | act_resnet18 | act_style_transformer_chunk | resnet18 | 0.782462 | 1.06243 | 0.00171198 | 0.00670573 | 7.43694 | 1.273 | outputs/flow_runs/imitation_pilot_5epoch/gpu7_act_resnet18/leaderboard_summary.json |
| 2 | direct_bc_resnet18 | direct_bc_chunk | resnet18 | 0.806603 | 1.09521 | 0.00171339 | 0.00652529 | 7.66639 | 1.067 | outputs/flow_runs/imitation_pilot_5epoch/gpu1_direct_tiny_resnet18/leaderboard_summary.json |
| 3 | direct_bc_tiny | direct_bc_chunk | tiny_cnn | 0.813017 | 1.10392 | 0.00175411 | 0.00654296 | 7.72736 | 0.189 | outputs/flow_runs/imitation_pilot_5epoch/gpu1_direct_tiny_resnet18/leaderboard_summary.json |
| 4 | act_tiny | act_style_transformer_chunk | tiny_cnn | 0.819056 | 1.11212 | 0.00181673 | 0.0067889 | 7.78475 | 0.4847 | outputs/flow_runs/imitation_pilot_5epoch/gpu6_act_tiny/leaderboard_summary.json |
| 5 | flow_resnet18 | flow_matching | resnet18 | 0.832249 | 1.13003 | 0.00174017 | 0.00678499 | 7.91014 | 10.42 | outputs/flow_runs/imitation_pilot_5epoch/gpu4_flow_resnet18/leaderboard_summary.json |
| 6 | state_mlp | state_only_mlp |  | 0.834199 | 1.13268 | 0.00179829 | 0.00657564 | 7.92868 | 0.1077 | outputs/flow_runs/imitation_pilot_5epoch/gpu0_baselines/leaderboard_summary.json |
| 7 | flow_tiny | flow_matching | tiny_cnn | 0.845313 | 1.14777 | 0.00178638 | 0.00678361 | 8.03432 | 4.366 | outputs/flow_runs/imitation_pilot_5epoch/gpu3_flow_tiny/leaderboard_summary.json |
| 8 | flow_resnet50 | flow_matching | resnet50 | 0.850249 | 1.15447 | 0.00176457 | 0.00670102 | 8.08123 | 21.04 | outputs/flow_runs/imitation_pilot_5epoch/gpu5_flow_resnet50/leaderboard_summary.json |
| 9 | direct_bc_resnet50 | direct_bc_chunk | resnet50 | 0.899118 | 1.22083 | 0.00208344 | 0.00678259 | 8.54571 | 2.212 | outputs/flow_runs/imitation_pilot_5epoch/gpu2_direct_resnet50/leaderboard_summary.json |
| 10 | train_mean | constant_baseline |  | 0.977582 | 1.32737 | 0.00285857 | 0.00703895 | 9.29147 | 0 | outputs/flow_runs/imitation_pilot_5epoch/gpu0_baselines/leaderboard_summary.json |
| 11 | zero | constant_baseline |  | 0.977606 | 1.3274 | 0.0028188 | 0.00706883 | 9.29169 | 0 | outputs/flow_runs/imitation_pilot_5epoch/gpu0_baselines/leaderboard_summary.json |

## Summary

- Best overall: `act_resnet18` norm_mse=0.782462
- Best learned: `act_resnet18` norm_mse=0.782462
- Best constant baseline: `train_mean` norm_mse=0.977582
