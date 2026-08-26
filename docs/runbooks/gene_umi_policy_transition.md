# GENE UMI Policy Transition Runbook

This runbook tracks the transition from offline UMI/GENE-style flow-policy
training to supervised simulator, rbpodo controller `pgmode` simulation, and
explicitly gated physical `real_policy` rollout. The rollout-mode validation has
passed for accepted configurations and the lane is open, but this is not blanket
physical-robot approval: every physical send still requires the `real_policy`
validation, tracked server motion config, operator supervision, and hardware
E-stop.

## Scope And Non-Goals

Scope:

- collect UMI/Pika and robotics_lab HDF5 evidence before policy training
- train and evaluate flow-policy checkpoints from audited episodes
- run simulator and rbpodo controller `pgmode` simulation dry runs
- run physical `real_policy` only through the accepted/validated gate set
- preserve rollout summaries and artifact manifest evidence for review

Non-goals:

- no implicit real RB3-730 motion
- no policy-side force-control promotion or change to the server's calibrated
  force-control v2 profile
- no gripper or camera-driven physical promotion outside the explicit
  `real_policy` validation and tracked real config
- no replacement of controller-simulation or hardware-free regression evidence

## Why Controller-Sim Is Not Physical Real Robot Performance

Controller-reference (`pgmode` simulation) success is a lower-bound
controller-simulation signal, not physical TCP tracking. A report that scores
`tcp_ref_stand` in `pgmode` simulation does not prove that `tcp_actual_stand`
followed the
trajectory, does not clear diagnostics-suspect caveats, and does not approve
physical real Cartesian motion.

Do not enable `cartesian_control.allow_in_real` for controller-simulation policy
work (the legacy `RB_ALLOW_REAL_*` env gates were removed; real motion is now
config-driven). Future physical evidence must be gathered through a separate
real-hardware acceptance plan.

## UMI HDF5 Audit/Import/Convert Path

Use `hdf5-audit` before training or rollout:

```bash
PYTHONPATH=policy_runner python3 -m policy_runner hdf5-audit \
  --episodes-dir data/umi_episodes \
  --output-json data/umi_episodes/audit.json \
  --output-md data/umi_episodes/audit.md
```

`umi-import` links raw UMI HDF5 episodes into a managed dataset directory and
writes manifest/report metadata. `umi-convert` writes a
`FlowHdf5Dataset`-compatible HDF5 layout. Both are repository-side file
pipelines; they do not open a live UMI hardware SDK and do not send robot
commands.

## Flow-Train / Flow-Infer Path

Run dependency preflight before training:

```bash
PYTHONPATH=policy_runner python3 -m policy_runner ml-preflight --vision-backbone tiny_cnn
```

Train with audited HDF5 episodes:

```bash
PYTHONPATH=policy_runner python3 -m policy_runner flow-train \
  --episodes-dir data/umi_episodes \
  --checkpoint outputs/flow_policy.pt \
  --vision-backbone tiny_cnn \
  --write-eval-report outputs/flow_eval_report.md
```

Run `flow-infer` only with an explicit `--rollout-mode`. Live rollout composes
each per-step action delta into an absolute `TcpPoseTarget` setpoint using
`--policy-dt-sec` or checkpoint `dataset_stats.dt_mean_sec`.

## Rollout Modes

Keep these lanes separate:

| Lane | Command authority | Motion evidence |
| --- | --- | --- |
| `offline_eval` | none | checkpoint plus HDF5 action chunk review |
| `sim_dryrun` | dropped by default | mock state and SafetyGate decisions |
| `controller_sim` | rbpodo controller `pgmode` simulation only | controller reference with `physical_motion_expected=false` |
| `real_readonly` / `real_supervised` | none | real state/camera observation and rollout summary |
| `real_policy` | physical sends only when `_validate_real_policy`, server config, gripper gate, operator supervision, and E-stop are all satisfied | physical `TcpPoseTarget` + gripper rollout summary; lane open only for accepted/validated configs |

Example offline evaluation:

```bash
PYTHONPATH=policy_runner python3 -m policy_runner flow-infer \
  --config policy_runner/config/stack_sim.yaml \
  --checkpoint outputs/flow_policy.pt \
  --rollout-mode offline_eval \
  --episodes-dir data/umi_episodes \
  --rollout-summary outputs/rollout_summary.json
```

For `controller_sim`, policy dt must come from `--policy-dt-sec` or checkpoint
`dataset_stats.dt_mean_sec`. For non-offline simulator or read-only modes,
omitting both means `1 / command_rate_hz`; this fallback is for dry-run and
summary convenience only. Physical `real_policy` can send only when
rollout-mode validation, accepted/validated safety attestations, gripper gates,
and the tracked real server config are all satisfied.

## SpaceMouse -> policy_runner -> rbpodo pgmode simulation -> viser path

The SpaceMouse controller-simulation path uses `policy_runner` as the command
source, the rbpodo backend in controller `pgmode` simulation, and the
state-only `rb_gui`/viser view for operator feedback.

Use the unified stack entrypoint for this path:

```bash
make run MODE=sim
```

The old dedicated SpaceMouse wrapper is no longer part of the active tooling
surface.

The SpaceMouse path sends `TcpPoseTarget` only after command-source lease,
deadman, stale state, fault, controller-simulation Cartesian, and
`physical_motion_expected=false` gates are satisfied. It must not be run
concurrently with GUI teleop against the same command port.

## Artifact Manifest Requirements

Every transition run should produce or update an Artifact manifest /
`artifact_manifest` with schema
`robotics_lab.gene_umi.artifact_manifest.v1`.

The manifest must include paths and metadata for:

- control default validation reports
- `hdf5-audit` JSON/Markdown outputs
- `flow-train` checkpoints plus `flow_eval_summary` and reports
- `flow-infer` `rollout_summary` JSON
- pgmode transition or physical-promotion dry-run reports

Generate the manifest with:

```bash
python3 scripts/collect_gene_umi_artifact_manifest.py \
  --include-missing \
  --output-json artifacts/gene_umi/artifact_manifest.json \
  --output-md artifacts/gene_umi/artifact_manifest.md
```

Missing expected artifacts must stay visible in Markdown as `MISSING`; do not
silently omit them when reviewing promotion readiness.

## Physical Promotion Criteria

The accepted `real_policy` lane is open for configurations that satisfy every
gate below. A physical rollout may send motion only while all of them remain
satisfied:

- `mode: real` and `safety.allow_real_motion: true`
- measured/accepted geometry with `geometry_valid_for_real_policy: true`, or
  the explicit `safety.allow_configured_estimate_geometry_in_real: true`
  carve-out for accepted ee_local policies that do not consume unmeasured
  camera/tool extrinsics
- `retarget_status: measured|accepted` and `measured_retarget_available: true`
- `collision_model_status: measured|validated` and
  `measured_collision_model_available: true`
- `workspace_envelope_status: measured|validated`
- `minimum_inter_arm_distance_m` configured
- `selected_arm: left|right|both` or `selected_arms: [left, right]` matching
  the checkpoint `arm_mask`
- measured gripper integration with `measured_gripper_available: true`
- physical gripper block remains active unless
  `allow_real_gripper_motion: true` and `RB_ALLOW_REAL_GRIPPER=1`
- the tracked real stack config enables the required server motion paths
- J3 remains within the exact Rainbow/URDF safety range `[-150, +150]` degrees

Flow checkpoints are 14D: each arm has six Cartesian channels plus one gripper
channel. In `controller_sim`, gripper commands may be forwarded to the sim
gripper backend when the stack gripper server is enabled, but that remains
hardware-free feedback only. Controller simulation does not approve physical
gripper motion.

## Validation

Run the focused checks:

```bash
python3 scripts/collect_gene_umi_artifact_manifest.py --help
PYTHONPATH=scripts python3 -m unittest discover -s scripts -p 'test_collect_gene_umi_artifact_manifest.py' -v
make -n policy-hdf5-audit-smoke
python3 -m compileall -q policy_runner/policy_runner scripts
```
