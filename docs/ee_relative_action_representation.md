# EE-relative (body-frame) action/proprio representation

Status: design (not yet implemented). Owner: policy_runner / flow imitation.

## Why

The flow / direct-BC policies are currently trained on **global stand-frame** deltas
(`tcp_delta_stand_from_poses`, `pose_delta` translation in world axes). Deploying a
global-frame policy requires a measured rotation `R = T_stand_steamvr` between the UMI
capture frame (`steamvr_world`, lighthouse) and the robot `stand` frame. That calibration
is hard here: the robot-mounted **Pika Gripper has no Vive tracker** (only the handheld
**Pika Sense** does), so co-locating a tracker with the robot TCP is awkward.

The UMI paper (Chi et al. 2024) shows the standard fix: represent actions as a **relative
trajectory in the current end-effector frame**. Their ablation found global/absolute
actions failed (5/20) precisely because they needed SLAM↔robot calibration, while
EE-relative actions are calibration-free and cross-platform.

This document specifies adopting an EE-relative (body-frame) representation so that **no
`steamvr_world → stand` rotation is ever needed**, using the **already-collected 146
episodes** (no re-collection, no robot, no R measurement).

## Key invariant (why existing data is sufficient)

Let `W` = steamvr_world, `S` = stand, related by an unknown fixed rigid transform
`X = T_S←W`. Gripper pose at step t is `P_t` (in W) or `P_t^S = X·P_t` (in S).

A body-relative action is `a_t = P_t⁻¹ · P_{t+1}`. Computed in either frame:

```
a_t^S = (X·P_t)⁻¹ (X·P_{t+1}) = P_t⁻¹ · X⁻¹X · P_{t+1} = P_t⁻¹ · P_{t+1} = a_t^W
```

`X` cancels. So the body-relative deltas computed from the recorded `steamvr_world` poses
are **numerically identical** to what they would be in the `stand` frame. Therefore:
- R need not be measured (it cancels), and
- the recorded poses already yield the correct training targets — only the *math used to
  derive targets* changes, then retrain.

A rotated capture frame is a pure isometry (preserves lengths/angles); it never distorts
the data. Only genuine non-rigid tracking error would, and that is a separate sensor-quality
concern (lighthouse is sub-mm/mm and rigid), already baked into the trained-on data.

## Representation definition

For reference pose `ref=(t_ref,q_ref)` and target `tgt=(t_tgt,q_tgt)` (quaternions xyzw):

```
t_rel = R(q_ref)^T · (t_tgt − t_ref)      # translation expressed in ref body frame
q_rel = q_ref⁻¹ ⊗ q_tgt                    # body-frame relative rotation
6D    = [t_rel, rotvec(q_rel)]
```

This unifies the two currently-inconsistent conventions:
- `pose_delta` (proprio, `flow_dataset.py`): translation is global; rotation already body
  (`q_ref⁻¹ ⊗ q_tgt`). → only the translation gains the `R(q_ref)^T·` rotation.
- `tcp_delta_stand_from_poses` (action): translation global, rotation **spatial**
  (`q_tgt ⊗ q_cur⁻¹`). → both change to the body form above.

Gripper channels (`left_grip`, `right_grip`) are frame-invariant and unchanged.

## Frames (do not conflate)

- `steamvr_world`, `stand`: **fixed** world frames; differ by the rigid `R` (mostly yaw).
- **body / EE / TCP frame**: a frame attached to the gripper that **moves with it**; a
  different frame at every timestep. The body-relative representation lives here. This is
  *not* the stand frame.

## Scope of code change

Add an `action_frame` mode, `"stand"` (default, current behavior) or `"ee_local"`, threaded
through training data build, dataset statistics, and inference. Keep both representations so
existing checkpoints/tests are unaffected; only new training opts into `ee_local`.

### `flow_dataset.py`
1. New helper `pose_delta_local(reference_pose, target_pose)` implementing the definition
   above; reuse `_quat_inverse`, `_quat_multiply`, `_quat_to_rotvec`. Rotate the translation
   into the reference frame by rotating `(t_tgt − t_ref)` with `q_ref⁻¹` (inverse rotation).
2. Module constant + `FlowHdf5Dataset.__init__(action_frame="stand")`, propagated to
   `raw_sample`.
3. `_proprio_vector` and `_action_chunk`: when `action_frame=="ee_local"` use
   `pose_delta_local` (for the action, the per-step reference is the step's current pose,
   matching the existing per-step structure); else keep current functions.
4. `compute_dataset_statistics`: add field `proprio_action_frame: <action_frame>` to the
   emitted stats dict (alongside the existing schema string). Stats are auto-derived from
   `raw_sample`, so means/stds update automatically.

### `flow_inference.py`
1. `runtime_proprio_from_state` must honor the same `action_frame` (read from the
   checkpoint's `dataset_stats.proprio_action_frame`); same helper as training → symmetric.
2. Command-family selection: when `proprio_action_frame=="ee_local"`, default the flow
   command family to **`tcp_twist_local`**. The step builders
   (`_tcp_twist_local/stand/delta_step_intent`) do **not** rotate the 6D — they slice,
   (÷`policy_dt_sec` for twist), clamp, and emit with a label. Frame interpretation is done
   by the C++ servo server (`TcpTwistLocal` = used directly in the TCP frame;
   `TcpTwistStand` = converted via `twistStandToLocal`). Therefore emitting the body-frame
   6D as `TcpTwistLocal` is correct and **does not double-convert**.
3. Gating: allow `tcp_twist_local` for `ee_local` in `offline_eval` and `sim_dryrun` (it is
   already permitted there). **Do not** relax `controller_sim`/`real_*` gating in this change
   — real-motion safety gates are out of scope and reviewed separately.

### Tool offset (tracker → gripper TCP) — still required
The recorded pose is the **tracker** pose; the robot controls the **gripper TCP**. Under
rotation these differ by a lever-arm, so the tracker→TCP offset `T_tcp_umi_gripper`
must be applied **before** computing body-relative deltas.

> **Correction (2026-06-11):** the offset is NOT a pure translation. The official
> pika_sdk transform is `T_tip = T_tracker(raw) · R_corr · Trans(0.172, 0, −0.076)` with
> `R_corr = Rx(−20°)·[Ry(−90°)·Rx(−90°)]` (hardcoded in `pika_sdk
> pika/tracker/vive_tracker.py`; the −20° compensates the tilted tracker mount). The
> `(0.172, 0, −0.076)` translation is defined in the rotation-corrected gripper frame —
> in the raw tracker frame the lever-arm is `R_corr·t = (0, −0.0126, +0.1876)` m.
> Earlier text treating `(0.172, 0, −0.076)` as a raw tracker-frame translation is wrong
> by ≈90°. `R_corr` is defined against the libsurvive raw frame; OpenVR-frame equivalence
> is pending the calibration clip. Live teleop already ships the tip pose on the wire
> (publisher `--pose-frame tip`); episodes recorded before 2026-06-11 carry the RAW
> tracker pose and need the full transform above at conversion time.

Reuse the existing converter:

```
umi-convert --output-format robotics_lab_dual_arm \
            --retarget <cfg with T_stand_source = identity, T_tcp_umi_gripper = CAD>
```

`_retarget_poses` already computes `T_stand_source · pose · inv(T_tcp_umi_gripper)`. With
`T_stand_source = identity`, this yields **gripper-TCP poses still in steamvr_world** — no R
needed, only the CAD tool offset. Train `ee_local` on the converted episodes.

### Tests
- Update mode-specific assertions: `test_flow_dataset.py` (`test_action_chunk_is_per_step_
  stand_delta_not_start_anchored`, `test_tcp_delta_stand_from_poses_uses_spatial_rotation_
  order`), `test_umi_pipeline.py` (stand-delta asserts), `test_hdf5_viewer.py`,
  `test_flow_inference_tcp_twistlocal.py` (command mapping/gates) → keep `stand` assertions
  for `stand` mode, add `ee_local` cases.
- New invariance test: apply a random fixed rigid transform `X` to every pose of a synthetic
  episode; assert `ee_local` proprio and action are unchanged (encodes the cancellation
  proof). This is the regression guard that R is irrelevant.

## One end-to-end cycle (all on existing data)
```
1) umi-convert (T_stand_source = I, T_tcp_umi_gripper = CAD)  -> gripper-TCP / steamvr episodes
2) compute stats (action_frame = ee_local)                   -> proprio_action_frame tag
3) flow-train                                                -> ee_local checkpoint
4) offline_eval (scripts/eval_viz_dump.py)                   -> A/B vs the stand checkpoint
5) sim_dryrun + TcpTwistLocal                                -> simulator validation
```
No re-collection, no robot, no R measurement.

## Decisions / risks
- **Coexistence via flag** (recommended) vs hard replace — flag protects the 168 existing
  checkpoints and current tests.
- **No double-conversion** (verified): builders are passthrough+label; the servo server does
  frame interpretation. Confirmed in `flow_inference.py` and `action_sources/tcp_delta.py`.
- **Real-mode gating unchanged**: this change targets sim/offline only.
- **proprio anchor**: keep current reset-anchor (minimal change); UMI uses current-EE anchor
  — a possible later refinement.
- **Tool-offset accuracy**: from CAD; per-step lever-arm error is sub-mm but worth applying.
- **gripper units**: unchanged (frame-invariant).
```
