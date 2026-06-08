# Flow Evaluation Report

## Dataset

- Formats: pika_umi_bimanual
- Episodes: 87
- Frames: 25279
- Samples: 23887
- Cameras: left_realsense_color, right_realsense_color
- Image decode count: 512
- Missing camera count: 0
- Action horizon: 16
- dt_mean_sec: 0.033402500994132035
- dt_p50_sec: 0.03339123725891113
- Arm mask counts: `{"left": 256, "right": 256}`

## Validation

- action_mse: 1.1377287
- gripper_mse: 7.9640115
- chunk_endpoint_error: 0.0072316425

## Checkpoint

- Schema: `robotics_lab.policy_runner.flow_matching.v1`
- SHA-256: `0064ad1b87d650c7acec2406c01a98b5222c7d7f312a526f80f857de30cbbf07`
- Path: `/outputs/flow_runs/realdata_resnet18_gpu0_epoch1_faststats/flow_policy.pt`

## Action Distribution Percentiles

| Dimension | p01 | p05 | p50 | p95 | p99 |
| --- | ---: | ---: | ---: | ---: | ---: |
| left_dx | -0.006529361 | -0.0052133501 | -0.00062307715 | 0.00073775649 | 0.0018272996 |
| left_dy | -0.0035345554 | -0.0017660856 | -0.00016474724 | 0.00054955482 | 0.0013554096 |
| left_dz | -0.0022447109 | -0.0016252995 | -2.3841858e-06 | 0.002548933 | 0.0035598278 |
| left_drx | -0.0074440269 | -0.0038346197 | 0.0010055503 | 0.017949797 | 0.021287145 |
| left_dry | -0.0093868719 | -0.0052072685 | 8.8569628e-05 | 0.0062234094 | 0.0077207861 |
| left_drz | -0.0048649283 | -0.0036583052 | 0.00071037398 | 0.018681817 | 0.028026734 |
| left_grip | -1.0862198 | -0.44540405 | 0 | 0.44540405 | 0.44540405 |
| right_dx | -0.0080301911 | -0.0068525448 | 1.9237399e-05 | 0.0034302175 | 0.0047410466 |
| right_dy | -0.0032334328 | -0.0024684668 | -0.00043833256 | 0.0046343803 | 0.0056060553 |
| right_dz | -0.0032734871 | -0.0011260509 | 0.00025939941 | 0.0051240921 | 0.0063464642 |
| right_drx | -0.010083119 | -0.0047612204 | 0.0012285227 | 0.0079112938 | 0.014102136 |
| right_dry | -0.019177541 | -0.015193165 | 0.0010126237 | 0.022146961 | 0.02970417 |
| right_drz | -0.01546626 | -0.012094454 | -0.0020992716 | 0.0059787259 | 0.0098975245 |
| right_grip | -3.6998615 | -1.9886169 | 0 | 0.48662567 | 2.9956465 |

## Warnings

- deployment_blocker: retarget_required: pose_frame steamvr_world must have measured or accepted retarget metadata to stand before physical real policy rollout; status=missing
- action_matches_observation_pose: left action pose columns are identical to current pose
- action_matches_observation_pose: right action pose columns are identical to current pose
- deployment_blocker: retarget_required: pose_frame steamvr_world must have measured or accepted retarget metadata to stand before physical real policy rollout; status=missing
