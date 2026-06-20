# GENE UMI Policy Transition Runbook

This runbook tracks the transition from offline UMI/GENE-style flow-policy
training to supervised simulator and rbpodo controller `pgmode` simulation
rollout. It is not physical real robot approval.

## Scope And Non-Goals

Scope:

- collect UMI/Pika and robotics_lab HDF5 evidence before policy training
- train and evaluate flow-policy checkpoints from audited episodes
- run simulator and rbpodo controller `pgmode` simulation dry runs
- preserve rollout summaries and artifact manifest evidence for review

Non-goals:

- no implicit real RB3-730 motion
- no physical real Cartesian policy approval
- no force-control, gripper, or camera-driven physical policy promotion
- no replacement of the simulator-first Cartesian acceptance milestone

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

Run `flow-infer` only with an explicit `--rollout-mode`. The default live
command family is `tcp_twist_stand`, which converts each per-step stand-frame
6D flow action delta into a bounded stand-frame Cartesian velocity using
`--policy-dt-sec` or checkpoint `dataset_stats.dt_mean_sec`. Velocity clamps
come from checkpoint action statistics unless explicit linear/angular limits
are supplied.

## Rollout Modes

Keep these lanes separate:

| Lane | Command authority | Motion evidence |
| --- | --- | --- |
| `offline_eval` | none | checkpoint plus HDF5 action chunk review |
| `sim_dryrun` | dropped by default | mock state and SafetyGate decisions |
| `controller_sim` | rbpodo controller `pgmode` simulation only | controller reference with `physical_motion_expected=false` |
| `real_readonly` / `real_supervised` | none | real state/camera observation and rollout summary |
| `real_policy` | future only | blocked until measured or accepted retarget, collision, gripper, camera, and geometry gates pass |

Example offline evaluation:

```bash
PYTHONPATH=policy_runner python3 -m policy_runner flow-infer \
  --config policy_runner/config/replay_sim.yaml \
  --checkpoint outputs/flow_policy.pt \
  --rollout-mode offline_eval \
  --episodes-dir data/umi_episodes \
  --rollout-summary outputs/rollout_summary.json
```

For `controller_sim`, policy dt must come from `--policy-dt-sec` or checkpoint
`dataset_stats.dt_mean_sec`. For non-offline simulator or read-only modes,
omitting both means `1 / command_rate_hz`; this fallback is for dry-run and
summary convenience only. Physical `real_policy` remains blocked by
rollout-mode validation and measured-geometry safety gates.

## SpaceMouse -> policy_runner -> rbpodo pgmode simulation -> viser path

The SpaceMouse controller-simulation path uses `policy_runner` as the command
source, the rbpodo backend in controller `pgmode` simulation, and the
state-only `rb_gui`/viser view for operator feedback.

Prepared wrapper checks:

```bash
tools/rbpodo_pgmode_spacemouse.sh check
tools/rbpodo_pgmode_spacemouse.sh policy-dry-run
tools/rbpodo_pgmode_spacemouse.sh gui --dry-run
```

The SpaceMouse path sends `TcpTwistLocal` only after command-source lease,
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

`real_policy` is blocked by default. A future physical rollout must satisfy all
of these gates before `policy_runner` may send motion:

- `mode: real` and `safety.allow_real_motion: true`
- measured geometry with `geometry_valid_for_real_policy: true`
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
- normal real robot and real motion gates required by the servo server

Flow checkpoints are 14D: each arm has six Cartesian channels plus one gripper
channel. In `controller_sim`, proposed gripper commands are logged and dropped
by the noop gripper backend unless a simulator gripper backend is explicitly
provided. Controller simulation does not approve physical gripper motion.

## Validation

Run the focused checks:

```bash
python3 scripts/collect_gene_umi_artifact_manifest.py --help
PYTHONPATH=scripts python3 -m unittest discover -s scripts -p 'test_collect_gene_umi_artifact_manifest.py' -v
make -n policy-hdf5-audit-smoke
make -n pgmode-transition-dry-run
CODEX_SKIP_MISSING_CPP_DEPS=1 ./scripts/codex_gate.sh 09_docs_ci_artifact_manifest
```
