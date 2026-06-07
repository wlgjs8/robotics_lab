# Combined Imitation Leaderboard

- Label: `fixed_holdout_diffusion`
- Snapshot hash: `532de3a1c38a4b3f12dcc875d928f723d798a1ee0edf02b99e857d2483e9f9ce`
- Split hash: `93ac654ea69705f967546f011fecc2efdb4047fc92590928b54c7708a061e7dc`
- Source reports: 15

| Rank | Model | Split | Family | Backbone | Loss | Norm MSE | Action MSE | Trans End | Rot End | Grip MSE | P50 Lat ms | Source |
| ---: | --- | --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | structured_resnet18 | primary | arm_structured_direct | resnet18 | mse | 0.763419 | 1.03657 | 0.00169907 | 0.00660812 | 7.25594 | 1.134 | /outputs/flow_runs/imitation_sweep_20epoch/gpu2_structured_resnet18/leaderboard_summary.json |
| 2 | state_mlp | primary | state_only_mlp |  | mse | 0.804666 | 1.09258 | 0.00171819 | 0.00654165 | 7.64798 | 0.1063 | /outputs/flow_runs/imitation_sweep_20epoch/gpu0_baselines_state/leaderboard_summary.json |
| 3 | flow_resnet18 | primary | flow_matching | resnet18 |  | 0.85418 | 1.15981 | 0.00185136 | 0.00699305 | 8.11858 | 11.74 | /outputs/flow_runs/imitation_sweep_20epoch/gpu5_flow_resnet18/leaderboard_summary.json |
| 4 | direct_bc_resnet18_huber | primary | direct_bc_chunk | resnet18 | huber | 0.861793 | 1.17015 | 0.00161855 | 0.0064841 | 8.19096 | 1.072 | /outputs/flow_runs/imitation_sweep_20epoch/gpu3_direct_resnet18_huber/leaderboard_summary.json |
| 5 | state_mlp | session_holdout | state_only_mlp |  | mse | 0.865241 | 1.15396 | 0.00178604 | 0.00644738 | 8.07764 | 0.1051 | /outputs/flow_runs/imitation_holdout_20epoch/gpu0_baselines_state/leaderboard_summary.json |
| 6 | structured_resnet18 | session_holdout | arm_structured_direct | resnet18 | mse | 0.890949 | 1.18825 | 0.00177874 | 0.0065394 | 8.31764 | 1.181 | /outputs/flow_runs/imitation_holdout_20epoch/gpu1_structured_resnet18/leaderboard_summary.json |
| 7 | direct_bc_resnet18_l1 | primary | direct_bc_chunk | resnet18 | l1 | 0.925293 | 1.25637 | 0.00162649 | 0.00647189 | 8.7945 | 1.106 | /outputs/flow_runs/imitation_sweep_20epoch/gpu4_direct_resnet18_l1/leaderboard_summary.json |
| 8 | flow_resnet18 | session_holdout | flow_matching | resnet18 |  | 0.925561 | 1.23441 | 0.00193688 | 0.00696998 | 8.64076 | 11.62 | /outputs/flow_runs/imitation_holdout_20epoch/gpu3_flow_resnet18/leaderboard_summary.json |
| 9 | direct_bc_resnet18_huber | session_holdout | direct_bc_chunk | resnet18 | huber | 0.928049 | 1.23772 | 0.00174435 | 0.00646175 | 8.66399 | 1.074 | /outputs/flow_runs/imitation_holdout_20epoch/gpu2_direct_resnet18_huber/leaderboard_summary.json |
| 10 | diffusion_resnet18 | primary | diffusion_policy_x0 | resnet18 | x0 | 0.946569 | 1.28526 | 0.0023086 | 0.00906068 | 8.99666 | 20 | /outputs/flow_runs/imitation_diffusion_20epoch_primary/gpu0_diffusion_resnet18/leaderboard_summary.json |
| 11 | train_mean | primary | constant_baseline |  |  | 0.977582 | 1.32737 | 0.00285857 | 0.00703895 | 9.29147 | 0 | /outputs/flow_runs/imitation_sweep_20epoch/gpu0_baselines_state/leaderboard_summary.json |
| 12 | zero | primary | constant_baseline |  |  | 0.977606 | 1.3274 | 0.0028188 | 0.00706883 | 9.29169 | 0 | /outputs/flow_runs/imitation_sweep_20epoch/gpu0_baselines_state/leaderboard_summary.json |
| 13 | diffusion_resnet18 | session_holdout | diffusion_policy_x0 | resnet18 | x0 | 1.0229 | 1.36423 | 0.0023454 | 0.00924645 | 9.54946 | 20.33 | /outputs/flow_runs/imitation_diffusion_20epoch_holdout/gpu1_diffusion_resnet18/leaderboard_summary.json |
| 14 | train_mean | session_holdout | constant_baseline |  |  | 1.04087 | 1.38819 | 0.00294744 | 0.00693174 | 9.71726 | 0 | /outputs/flow_runs/imitation_holdout_20epoch/gpu0_baselines_state/leaderboard_summary.json |
| 15 | zero | session_holdout | constant_baseline |  |  | 1.0409 | 1.38823 | 0.00289302 | 0.00695194 | 9.71749 | 0 | /outputs/flow_runs/imitation_holdout_20epoch/gpu0_baselines_state/leaderboard_summary.json |
| 16 | act_resnet18 | primary | act_style_transformer_chunk | resnet18 | mse | 1.05458 | 1.43191 | 0.00193896 | 0.00736079 | 10.0233 | 1.365 | /outputs/flow_runs/imitation_sweep_20epoch/gpu1_act_resnet18_h16/leaderboard_summary.json |
| 17 | act_resnet18 | primary | act_style_transformer_chunk | resnet18 | mse | 1.05909 | 1.29144 | 0.00203244 | 0.00743525 | 9.04001 | 1.373 | /outputs/flow_runs/imitation_sweep_20epoch/gpu6_act_resnet18_h32/leaderboard_summary.json |
| 18 | act_resnet50 | primary | act_style_transformer_chunk | resnet50 | mse | 1.08814 | 1.47748 | 0.00199747 | 0.00732237 | 10.3423 | 2.326 | /outputs/flow_runs/imitation_sweep_20epoch/gpu7_act_resnet50/leaderboard_summary.json |
| 19 | act_resnet18 | session_holdout | act_style_transformer_chunk | resnet18 | mse | 1.09128 | 1.45542 | 0.00200104 | 0.00725968 | 10.1879 | 1.268 | /outputs/flow_runs/imitation_holdout_20epoch/gpu4_act_resnet18/leaderboard_summary.json |

## Safety

- Offline HDF5 training/evaluation only.
- No robot commands or real-mode behavior are part of these reports.
