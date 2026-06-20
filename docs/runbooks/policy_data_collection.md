# Policy Data Collection Runbook

This runbook defines the policy and teleop dataset schema for simulator data,
rbpodo controller pgmode simulation data, and future physical real
demonstrations. It is a recording contract only. It does not approve physical
motion, change deadman behavior, or weaken command-source lease checks.

## Evidence Categories

Keep these categories separate in every episode and report:

| Category | `backend_type` | `run_mode` | `operation_mode` | Physical motion |
| --- | --- | --- | --- | --- |
| `rbpodo_controller_simulation` | `rbpodo` | `real` | `simulation` | `physical_motion_expected=false` |
| future physical real robot | `rbpodo` | `real` | `real` | future acceptance only |

The controller-simulation path uses real Rainbow controller boxes in `pgmode`
simulation. Do not mix controller-simulation episodes with future physical real
episodes unless the training loader explicitly filters by these labels.

Controller-simulation (`pgmode` simulation) tuning is a high-performance
controller-reference signal only. It is not the physical-real default until the
physical promotion ladder produces actual TCP tracking evidence. Keep
controller-simulation episodes under the category `rbpodo_controller_simulation`
unless a future physical promotion artifact explicitly changes this contract.

For rbpodo controller pgmode collection,
`policy_runner.safety.allow_real_motion=false` is the expected policy setting.
The runner may send Cartesian SpaceMouse commands only when
`allow_rbpodo_controller_simulation_cartesian=true` and the server state proves
`backend_type=rbpodo`, `run_mode=real`, `operation_mode=simulation`, the
server's controller-simulation carve-out active (config-driven:
`servo.allow_controller_simulation_motion: true` +
`cartesian_control.allow_in_controller_simulation: true`), and
`physical_motion_expected=false`. The server's own real/controller-sim
authority is config-driven (the legacy `RB_ALLOW_REAL_*` /
`RB_ALLOW_RBPODO_CONTROLLER_SIM_*` / `RB_RBPODO_PGMODE_SIMULATION_CONFIRMED` env
gates were removed from the server runtime); none of that is policy-runner
physical-motion approval.

Flow-policy rollout must also declare `--rollout-mode`; the policy runner must
not infer rollout authority from `mode: real` alone. Use `offline_eval` for
checkpoint plus HDF5 sample review without state or UDP clients, `sim_dryrun`
for mock/simulator state with dropped intents by default, `controller_sim` for
the rbpodo `controller_simulation` carve-out, `real_readonly` for
`real_supervised` observation/inference with no command sends, and
`real_policy` only for future physical rollout after measured or accepted retarget,
collision, gripper, camera, and geometry gates are present. Every `flow-infer`
run writes `rollout_summary`, including decode/missing-camera counts, safety
decision counts, command send/drop counts, and backend/run_mode/operation_mode
observed in state.

Physical `real_policy` must remain blocked by default. The exact required gate
list is:

- `mode: real`
- `safety.allow_real_motion: true`
- measured geometry with `geometry_valid_for_real_policy: true`
- `retarget_status: measured|accepted` and `measured_retarget_available: true`
- `collision_model_status: measured|validated` and
  `measured_collision_model_available: true`
- `workspace_envelope_status: measured|validated`
- `minimum_inter_arm_distance_m` configured
- `selected_arm: left|right|both` or `selected_arms: [left, right]` matching
  the checkpoint `arm_mask`
- `measured_gripper_available: true`
- for nonzero checkpoint gripper channels, `allow_real_gripper_motion: true`
  and `RB_ALLOW_REAL_GRIPPER=1`

Flow actions are 14D, with per-arm Cartesian channels and separate gripper
delta channels. The Cartesian action contract is per-step ee_local body-frame
pose delta: translation plus rotation vector, with no division by dt in the
training label. Runtime command-family selection controls only how that same
delta is emitted. The default `--command-family tcp_twist_local` divides the
delta by `--policy-dt-sec` or checkpoint `dataset_stats.dt_mean_sec`, then clamps
the final `TcpTwistLocal` velocity with checkpoint action statistics unless
`--max-linear-velocity-m-s` and `--max-angular-velocity-rad-s` are supplied. The
opt-in `--command-family tcp_target_pose` clamps the per-step delta by the same
velocity limits times policy dt, composes it onto the running measured TCP pose,
and emits absolute `TcpPoseTarget` `tcp_target_stand` setpoints. `controller_sim`
and `real_policy` require policy dt from the CLI or checkpoint stats; dry-run and
read-only modes may use the documented `1 / command_rate_hz` fallback. Live
rollout requires the matching explicit opt-in flag:
`--allow-tcp-twist-local` or `--allow-tcp-target-pose`. In `controller_sim`,
gripper proposals are logged and dropped by the noop gripper backend unless an
explicit simulator gripper backend is configured; physical gripper motion
remains blocked by default.

## Episode Metadata

Every dataset shard should carry this metadata. Fields may be empty for legacy
or partially collected episodes, but new collection workflows should fill them
before training data is promoted:

```json
{
  "schema": "robotics_lab.policy_runner.dataset_metadata.v1",
  "git_commit": "string",
  "config_hash": "sha256 or string",
  "backend_type": "mock | rbpodo",
  "run_mode": "mock | simulation | real",
  "operation_mode": "simulation | real | unknown",
  "physical_motion_expected": false,
  "controller_pgmode": "simulation | real | unknown | not_applicable",
  "calibration_status": "configured_estimate | measured | unknown",
  "camera_status": "disabled | simulated | real_unmeasured | real_measured",
  "selected_arm": "left | right | both",
  "collision_model_status": "missing | configured_estimate | measured | validated",
  "command_source_id": "policy_runner",
  "benchmark_linkage": {}
}
```

The helper `policy_runner.recording.build_dataset_metadata()` builds this
shape for JSONL or HDF5 recorders. It is passive; it only formats metadata and
does not inspect hardware or send commands.

Use the active calibration registry for the calibration label:

```text
calibration/active_calibration.yaml
```

The current repository default is `status: configured_estimate` and
`geometry_valid_for_real_policy: false`. That is acceptable for simulator and
visualization work, but not for future real geometry-dependent policy.

## State Fields

Record raw server state whenever possible. Structured HDF5 episodes should
preserve these fields when present:

| Field | Meaning |
| --- | --- |
| `q_actual_deg` | measured physical/controller actual joints |
| `q_ref_deg` or `q_target_deg` | controller reference/target joints |
| `tcp_actual_stand` | TCP pose from `q_actual_deg` in stand frame |
| `tcp_ref_stand` | TCP pose from controller reference joints in stand frame |
| `tcp_tracking_source` | selected or recommended tracking source |
| `state_age_us` | age of the state sample |
| `diagnostics_suspect` | per-arm diagnostics-suspect state when reported |
| `fault_latched` | server fault latch |
| `send_duration_us` | backend send timing when reported |
| `ack_policy` | ACK-on, ACK-off, or socket-send semantics |
| `controller_acceptance_observed` | whether controller ACK/acceptance was observed |

For rbpodo controller pgmode simulation, `tcp_ref_stand` is normally the
measurement target for controller-reference tracking. `tcp_actual_stand` may
remain stationary because physical motion is not expected. Do not treat
reference TCP movement as physical motion.

## Action Fields

Record both the normalized command packet and raw teleop inputs when available:

| Field | Meaning |
| --- | --- |
| `mode` | `Hold`, `JointTarget`, `JointVelocity`, `TcpTwistLocal`, `TcpTwistStand`, etc. |
| `tcp_twist_local` | SpaceMouse local-frame twist in m/s and rad/s |
| `tcp_twist_stand` | flow-policy stand-frame twist in m/s and rad/s |
| `tcp_target_stand` | absolute `TcpPoseTarget` pose; quaternion_xyzw is authoritative when present |
| `q_target_deg` | joint target command |
| `dq_target_deg_s` | joint velocity command |
| `spacemouse_axes` | raw six-axis SpaceMouse sample before policy scaling, if captured |
| `spacemouse_buttons` | raw SpaceMouse button state, including deadman |
| `deadman` | per-arm deadman state used for command emission |
| `gripper_left/right` or `gripper_target` | current or target gripper state; real physical output additionally requires `allow_real_gripper_motion` and `RB_ALLOW_REAL_GRIPPER` |
| `command_seq` or `seq` | command sequence number |
| `source_id` | command source id, normally `policy_runner` |

Existing SpaceMouse action sources still require their deadman and safety
preflight before any motion command is emitted. Recorder fields do not bypass
those checks.

## Benchmark Linkage

`benchmark_linkage` is a generic optional metadata dict
(`build_dataset_metadata` defaults it to `{}`). The circle-tracking benchmark it
previously linked to was removed, so it is normally empty. The state stream
remains the source of truth for dataset state samples.

## Recorder Compatibility

JSONL episodes already preserve complete raw `robot_state.jsonl` payloads and
`actions.jsonl` packets. New metadata should be placed in
`episode_metadata.json` under the dataset metadata schema above.

HDF5 episodes keep the existing root schema
`robotics_lab.episode.v1` and add optional datasets/attrs for provenance,
actual/reference TCP, controller reference joints, ACK semantics, diagnostics,
and raw SpaceMouse fields when present. Old HDF5 logs remain readable because
new fields are additive.

## Safety Notes

- This schema work must not enable real motion.
- Physical real demonstrations require a separate future acceptance runbook.
- `policy_runner` must continue to respect stale-state, fault, deadman,
  command-source lease, and real-motion gates.
- Controller-simulation episodes must be labeled
  `physical_motion_expected=false`.
- Controller-simulation collection should keep
  `policy_runner.safety.allow_real_motion=false`; do not use physical-motion
  approval to make pgmode simulation pass.
- Do not train or evaluate a physical real policy on mixed simulator and
  controller-simulation data unless the loader filters by metadata.
