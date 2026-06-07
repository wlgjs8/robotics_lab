# Combined Imitation Leaderboard

- Label: `imitation_holdout_20epoch`
- Snapshot hash: `532de3a1c38a4b3f12dcc875d928f723d798a1ee0edf02b99e857d2483e9f9ce`
- Split hash: `93ac654ea69705f967546f011fecc2efdb4047fc92590928b54c7708a061e7dc`
- Source reports: 5

| Rank | Model | Split | Family | Backbone | Loss | Norm MSE | Action MSE | Trans End | Rot End | Grip MSE | P50 Lat ms | Source |
| ---: | --- | --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | state_mlp | session_holdout | state_only_mlp |  | mse | 0.865241 | 1.15396 | 0.00178604 | 0.00644738 | 8.07764 | 0.1051 | /outputs/flow_runs/imitation_holdout_20epoch/gpu0_baselines_state/leaderboard_summary.json |
| 2 | structured_resnet18 | session_holdout | arm_structured_direct | resnet18 | mse | 0.890949 | 1.18825 | 0.00177874 | 0.0065394 | 8.31764 | 1.181 | /outputs/flow_runs/imitation_holdout_20epoch/gpu1_structured_resnet18/leaderboard_summary.json |
| 3 | flow_resnet18 | session_holdout | flow_matching | resnet18 |  | 0.925561 | 1.23441 | 0.00193688 | 0.00696998 | 8.64076 | 11.62 | /outputs/flow_runs/imitation_holdout_20epoch/gpu3_flow_resnet18/leaderboard_summary.json |
| 4 | direct_bc_resnet18_huber | session_holdout | direct_bc_chunk | resnet18 | huber | 0.928049 | 1.23772 | 0.00174435 | 0.00646175 | 8.66399 | 1.074 | /outputs/flow_runs/imitation_holdout_20epoch/gpu2_direct_resnet18_huber/leaderboard_summary.json |
| 5 | train_mean | session_holdout | constant_baseline |  |  | 1.04087 | 1.38819 | 0.00294744 | 0.00693174 | 9.71726 | 0 | /outputs/flow_runs/imitation_holdout_20epoch/gpu0_baselines_state/leaderboard_summary.json |
| 6 | zero | session_holdout | constant_baseline |  |  | 1.0409 | 1.38823 | 0.00289302 | 0.00695194 | 9.71749 | 0 | /outputs/flow_runs/imitation_holdout_20epoch/gpu0_baselines_state/leaderboard_summary.json |
| 7 | act_resnet18 | session_holdout | act_style_transformer_chunk | resnet18 | mse | 1.09128 | 1.45542 | 0.00200104 | 0.00725968 | 10.1879 | 1.268 | /outputs/flow_runs/imitation_holdout_20epoch/gpu4_act_resnet18/leaderboard_summary.json |

## Safety

- Offline HDF5 training/evaluation only.
- No robot commands or real-mode behavior are part of these reports.
