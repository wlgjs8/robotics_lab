# GENE UMI Policy Transition Runbook

This runbook tracks the transition from offline UMI/GENE-style flow-policy
training to supervised simulator and rbpodo controller `pgmode` simulation
rollout. It is not physical real robot approval.

## Evidence Boundary

Keep these lanes separate:

| Lane | Command authority | Motion evidence |
| --- | --- | --- |
| `offline_eval` | none | checkpoint plus HDF5 action chunk review |
| `sim_dryrun` | dropped by default | mock/simulator state and SafetyGate decisions |
| `controller_sim` | rbpodo controller `pgmode` simulation only | controller reference with `physical_motion_expected=false` |
| `real_readonly` / `real_supervised` | none | real state/camera observation and rollout summary |
| `real_policy` | future only | blocked until measured retarget, collision, gripper, camera, and geometry gates pass |

The GENE 26.5 / ACKON500 default remains a controller-simulation profile. It is
not the physical-real default and must be reported with
`physical_motion_expected=false`.

## Physical Real-Policy Gate List

`real_policy` is blocked by default. A future physical rollout must satisfy all
of these gates before policy-runner may send motion:

- `mode: real` and `safety.allow_real_motion: true`
- measured geometry with `geometry_valid_for_real_policy: true`
- `retarget_status: measured` and `measured_retarget_available: true`
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
channel. The gripper channel is handled separately from Cartesian arm motion.
In `controller_sim`, proposed gripper commands are logged and dropped by the
noop gripper backend unless a simulator gripper backend is explicitly provided.
Controller simulation does not approve physical gripper motion.

## Flow-Infer Command Family

`flow-infer` must declare `--rollout-mode`. It may also declare:

```bash
--command-family tcp_twist_local
--policy-dt-sec 0.01
--max-linear-velocity-m-s 0.03
--max-angular-velocity-rad-s 0.2
```

`tcp_twist_local` is the default command family for `controller_sim`,
`sim_dryrun`, and offline reporting. It converts each 6D flow action delta into
a bounded local-frame Cartesian velocity:

```text
linear_velocity = clamp(delta_xyz / policy_dt_sec, max_linear_velocity_m_s)
angular_velocity = clamp(delta_rotvec / policy_dt_sec, max_angular_velocity_rad_s)
```

For `controller_sim`, `--policy-dt-sec` is required. For non-offline simulator
or read-only modes, omitting it means `1 / command_rate_hz`; this fallback is
for dry-run and summary convenience only. Physical `real_policy` remains
blocked by rollout-mode validation and measured-geometry safety gates.

Policy-to-rbpodo `pgmode` simulation uses `TcpTwistLocal` first because it is
the same streaming Cartesian command family already exercised by the
SpaceMouse pgmode simulation path. The server-side Cartesian gates, command
freshness, velocity limits, and controller-simulation telemetry are designed
around streaming twist commands. This keeps flow rollout aligned with the
existing narrow rbpodo controller-simulation carve-out instead of promoting
one-shot jog semantics.

## Experimental Delta Debug Path

`tcp_delta_stand` remains available only for offline and simulator debugging:

```bash
python3 -m policy_runner flow-infer \
  --config policy_runner/config/simulator_hold.yaml \
  --checkpoint outputs/flow_policy.pt \
  --rollout-mode sim_dryrun \
  --command-family tcp_delta_stand
```

Using `tcp_delta_stand` with `controller_sim`, `real_readonly`, or future
`real_policy` requires the explicit experimental flag:

```bash
--allow-experimental-tcp-delta-stand
```

That flag does not approve physical motion. It only documents that the operator
is intentionally leaving the default streaming `TcpTwistLocal` path.

## Safety Notes

- Missing required camera frames produce no new nonzero motion intent.
- If a streaming twist was previously nonzero, the source emits a one-shot zero
  `TcpTwistLocal` stop command when policy input disappears or the next action
  is zero.
- Stale camera/state/fault conditions remain SafetyGate decisions and must not
  be bypassed by policy code.
- Controller simulation requires
  `cartesian_action_requirements(allow_rbpodo_controller_simulation=True)`,
  server `cartesian_gate` evidence, and `physical_motion_expected=false`.
- `RB_ALLOW_REAL_CARTESIAN` is not part of this controller-simulation workflow.
- `RB_ALLOW_REAL_GRIPPER` is not part of controller simulation; it is only a
  future physical `real_policy` gripper gate paired with
  `allow_real_gripper_motion: true`.

## Validation

Run the focused rollout checks:

```bash
PYTHONPATH=policy_runner python3 -m unittest discover -s policy_runner/tests -p 'test_flow_inference*.py' -v
PYTHONPATH=policy_runner python3 -m unittest discover -s policy_runner/tests -v
CODEX_SKIP_MISSING_CPP_DEPS=1 ./scripts/codex_gate.sh 07_policy_runner_rollout_modes
```

For dataset readiness before rollout, use `hdf5-audit` and keep the generated
audit report plus `rollout_summary` in the artifact manifest / `artifact_manifest`
for the run.
