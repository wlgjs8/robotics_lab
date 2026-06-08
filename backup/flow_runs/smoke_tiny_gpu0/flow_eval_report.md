# Flow Evaluation Report

## Dataset

- Formats: pika_umi_bimanual
- Episodes: 1
- Frames: 416
- Samples: 412
- Cameras: left_realsense_color, right_realsense_color
- Image decode count: 32
- Missing camera count: 0
- Action horizon: 4
- dt_mean_sec: 0.03340245672019131
- dt_p50_sec: 0.03339123725891113
- Arm mask counts: `{"left": 16, "right": 16}`

## Validation

- action_mse: 0.96686152
- gripper_mse: 6.7678531
- chunk_endpoint_error: 0.011512257

## Checkpoint

- Schema: `robotics_lab.policy_runner.flow_matching.v1`
- SHA-256: `3c53cf59b41e99ff8eaded69baf979086fbad8c28282f2877b8651f718106abc`
- Path: `/outputs/flow_runs/smoke_tiny_gpu0/flow_policy.pt`

## Action Distribution Percentiles

| Dimension | p01 | p05 | p50 | p95 | p99 |
| --- | ---: | ---: | ---: | ---: | ---: |
| left_dx | 3.0882359e-05 | 0.00045827329 | 0.0012220144 | 0.0028120175 | 0.0028946698 |
| left_dy | -0.0008392334 | -0.00078630447 | 0.00042104721 | 0.0018496811 | 0.0018869638 |
| left_dz | -0.0016069698 | -0.0014898777 | -0.00047659874 | 0.001306355 | 0.0013079643 |
| left_drx | -0.009552761 | -0.0091435546 | 0.0052064504 | 0.014236046 | 0.014428559 |
| left_dry | -0.0099413507 | -0.0095402487 | 0.0019631686 | 0.0048121344 | 0.0065154447 |
| left_drz | -0.013355389 | -0.013033974 | -0.0032103627 | 0.0068092461 | 0.0068455203 |
| left_grip | -0.44540405 | -0.44540405 | 0 | 0.44540405 | 0.44540405 |
| right_dx | 0.0023514256 | 0.0024026707 | 0.0037379134 | 0.0049439877 | 0.0049744397 |
| right_dy | -0.0014719021 | -0.0012766123 | 0.00020599365 | 0.0021234632 | 0.0021430254 |
| right_dz | -0.0026453352 | -0.0016875267 | 0.003017664 | 0.0050054789 | 0.0050122738 |
| right_drx | -0.0064833295 | -0.0058630603 | 0.0045881094 | 0.019677977 | 0.019945614 |
| right_dry | -0.0062736473 | -0.0060419562 | 0.0045330017 | 0.01276433 | 0.014242345 |
| right_drz | -0.016711088 | -0.016670354 | -0.0050172349 | 0.0017994734 | 0.0056796488 |
| right_grip | -0.16740219 | 0 | 0 | 0 | 0 |

## Warnings

- deployment_blocker: retarget_required: pose_frame steamvr_world must have measured or accepted retarget metadata to stand before physical real policy rollout; status=missing
- action_matches_observation_pose: left action pose columns are identical to current pose
- action_matches_observation_pose: right action pose columns are identical to current pose
- deployment_blocker: retarget_required: pose_frame steamvr_world must have measured or accepted retarget metadata to stand before physical real policy rollout; status=missing
