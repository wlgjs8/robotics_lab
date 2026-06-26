# policy_runner

`policy_runner` owns Python action sources, safety gating, recording, HDF5
audit, and flow/openpi inference for the dual-arm RB servo stack.

## Public Motion Surface

The runner emits only these public motion primitives:

- `JointTarget`
- `TcpPoseTarget`
- `TcpLinearMove`

Lifecycle and admin packets such as `Hold`, `ArmMotion`, `DisarmMotion`,
`EmergencyStop`, `ResetFault`, lease acquire/release, safety-plane updates, and
`Freedrive` remain lifecycle/safety commands rather than motion primitives.

The ergonomic `CommandIntent.init_motion()` helper emits `JointTarget` with
per-arm `joint_target_profile: init_motion`; it never emits a separate packet
mode.

## Action Sources

Supported action sources:

- `hold`
- `joint_sine`
- `dual_spacemouse_pose_target`
- `umi_dual_cartesian`
- `teleop_mux`

`dual_spacemouse_pose_target` treats each SpaceMouse as a virtual target cursor.
On engagement it latches each arm's virtual target from `tcp_ref_stand` when
available, otherwise `tcp_actual_stand`. Each fresh non-neutral sample composes a
small local-frame pose step onto that cursor and emits an absolute stand-frame
`TcpPoseTarget`. Neutral, missing, or stale samples do not integrate. Release or
HID failure emits `Hold` according to the current lease/hold policy.

The `make run` SpaceMouse profile can also send gripper presets. Each
SpaceMouse controls the gripper on the same arm: button `0` opens and button
`1` closes by default. Tune the target percent values in YAML:

```yaml
spacemouse_pose_target_dual:
  gripper_buttons:
    enable: true
    open_button: 0
    close_button: 1
    open_percent: 100.0
    close_percent: 10.0
```

Button-only gripper commands are per-arm `Hold` payloads with
`gripper_target`, so they do not request arm motion. Under `teleop_mux`, they
are honored when the mux is idle or already owned by SpaceMouse; UMI ownership
continues to suppress SpaceMouse intents.

## Flow/OpenPI Rollout

`flow-infer` and OpenPI remote rollout always compose ee_local policy deltas into
absolute `TcpPoseTarget` setpoints. There is no public command-family selection
or opt-in flag for target-pose rollout.

Live rollout still requires an explicit `--rollout-mode`. `controller_sim` and
`real_policy` require a policy dt from CLI or checkpoint stats; dry-run and
read-only modes may use the command-rate fallback.

For live OpenPI `real_policy`, keep `make run` running and start
`tools/flow_infer_real_policy.sh` or `make flow-infer-real` from another
terminal. The stack teleop_mux uses state port `50376`; the flow configs use
`50378`, so `ACTION_SOURCE=none` is not part of the normal flow-infer path.

## HDF5 Schema

Current recordings use schema `robotics_lab.episode.v1`. The action group stores
absolute target-pose datasets:

- `/action/tcp_target_stand_left`
- `/action/tcp_target_stand_right`
- `/action/mode`
- `/action/seq`
- `/action/action_host_time_ns`
- optional raw teleop fields such as SpaceMouse axes/buttons and deadman state

Legacy dataset layouts are not supported.
