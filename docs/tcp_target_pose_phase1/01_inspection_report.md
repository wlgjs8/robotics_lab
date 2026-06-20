# Tcp Target Pose Phase-1 Inspection Report

Scope: inspection only. `docs/tcp_target_pose_phase1/CONTRACT.md` was read first. The contract makes Phase 1 offline Python tooling only and explicitly says it does "not modify the C++ runtime, the SMD tracker, IK, safety, or teleop" (`CONTRACT.md:19-22`) and "Do NOT modify any `servo_*` parameter" (`CONTRACT.md:25-27`).

## 1. Source Ingress

### Flow inference -> `TcpPoseTarget`

- The command family maps `tcp_target_pose` into a target-pose step intent: `policy_runner/policy_runner/flow_inference.py:787-812` builds left/right target payloads and calls `tcp_pose_target_stand_intent(...)`.
  - Snippet: `return tcp_pose_target_stand_intent(` (`flow_inference.py:806`).
- Flow target-pose conditioning integrates local policy deltas into an absolute per-arm target. On chunk boundary or missing target, it reanchors from live state, then composes a clamped local delta: `flow_inference.py:824-828`.
  - Snippets: `targets[arm] = pose_from_state_payload(payload, arm)` and `targets[arm] = pose_compose_local(...)`.
- The per-step clamp is source-rate based, not servo-rate based: `flow_inference.py:836-842`.
  - Snippet: `self.max_linear_velocity_m_s * self.policy_dt_sec`.
- `tcp_pose_target_stand_intent` creates a `CartesianCommandIntent` whose arm payload is `mode: TcpPoseTarget`: `policy_runner/policy_runner/action_sources/tcp_delta.py:113-126`.
  - Snippet: `CartesianCommandIntent("TcpPoseTarget", ...)`.
- `_pose_target_arm_payload` serializes 6-value or 7-value poses into the field that the C++ server later reads: `tcp_target_stand`: `policy_runner/policy_runner/action_sources/tcp_delta.py:201-228`.
  - Snippet: `"tcp_target_stand": values`.
- `ServoCommandClient.send()` JSON-encodes and sends the intent over UDP; `build_packet()` includes top-level mode plus per-arm payloads: `policy_runner/policy_runner/servo_command_client.py:138-145`, `:185-200`.
  - Snippet: `self._socket.sendto(data, self._address)`.

### UMI teleop -> `TcpPoseTarget`

- UMI teleop returns `tcp_pose_target_stand_intent(...)` when either side changed: `policy_runner/policy_runner/action_sources/umi_dual_cartesian.py:306-331`.
  - Snippet: `return tcp_pose_target_stand_intent(`.
- It applies input moving average before latch/compose math: `umi_dual_cartesian.py:389-391`.
  - Snippet: `sample = moving_average.filter(sample)`.
- On first deadman engagement it latches live robot TCP and Pika pose as relative anchors: `umi_dual_cartesian.py:422-432`.
  - Snippets: `state.arm_init = arm_init`, `state.pika_init = pika_now`.
- It composes either world-frame or tool-frame deltas into an absolute target, then applies workspace/deadband/per-step clamp before returning the pose: `umi_dual_cartesian.py:436-446`, `:464-465`.
  - Snippets: `target = _compose(state.arm_init, delta)`, `pose6 = self._clamp_against_previous(...)`.

### Replay paths

- The prompt-named replay scripts currently do not send `TcpPoseTarget`.
- `scripts/replay_episode_rollout.py` documents and emits `TcpTwistLocal`: `scripts/replay_episode_rollout.py:130-135`, `:182-190`; `build_source()` hardcodes `command_family="tcp_twist_local"` at `:242-248`.
  - Snippets: `as TcpTwistLocal`, `command_family="tcp_twist_local"`.
- `scripts/replay_policy_actions.py` chooses only twist builders for `"ee_local"` and `"stand"` frames: `scripts/replay_policy_actions.py:137-145`, then constructs `ReplayTwistActionSource`: `:182-185`.
  - Snippet: `{"ee_local": tcp_twist_local_intent, "stand": tcp_twist_stand_intent}`.
- Current target-pose eval/replay utilities do send `TcpPoseTarget`: `scripts/eval_tcp_target_pose_tracking.py:119-131`, `:335-340`; `scripts/cartesian_acceptance.py:347-355`; `scripts/eval_tcp_target_pose_500hz.py:153-164`.
  - Snippets: `"left": {"mode": "TcpPoseTarget", "tcp_target_stand": target}`, `client.send(tcp_pose_target_stand_intent(...))`.

### UDP server ingress

- C++ parses each arm object, reads `mode`, then reads `tcp_target_stand` into `ArmCommand::tcp_target_stand` and sets `has_tcp_target`: `rb_servo_server/src/network/command_server.cpp:662-743`.
  - Snippet: `readOptionalPose6D(object, "tcp_target_stand", &out->tcp_target_stand, &present)`.
- Top-level `tcp_target_stand` without per-arm objects is copied to both arms: `command_server.cpp:1015-1026`.
  - Snippet: `cmd.right.tcp_target_stand = cmd.left.tcp_target_stand`.
- Parsed non-lease packets replace the command buffer's latest command: `command_server.cpp:831-869`.
  - Snippet: `command_buffer_->setCommand(cmd)`.

## 2. ZOH Between Source Rate and 500 Hz

- Flow streamed mode is an explicit source-side ZOH: one chunk step is approx. 30 Hz, and the same intent is returned until `policy_dt_sec` elapses: `flow_inference.py:552-616`.
  - Snippet: `The same twist is re-emitted every servo tick`.
  - For `tcp_target_pose`, `_stream_hold_intent()` clears target-pose state and returns `None` on stalls: `flow_inference.py:674-678`.
- Server-side ZOH is `CommandBuffer::latestOrHold()`: the servo loop calls it once per tick at `dual_arm_servo_loop.cpp:1800-1802`; the buffer returns the last command until timeout, then `Hold`: `rb_servo_server/src/control/command_buffer.cpp:72-90`.
  - Snippets: `DualArmCommand cmd = *latest_command_`, `return makeHold(now_ns)`.
- `setCommand()` replaces `latest_command_` for non-lifecycle commands: `command_buffer.cpp:44-55`.
  - Snippet: `latest_command_ = command`.

## 3. Chunk Boundaries and Re-Anchor Behavior

- Flow target-pose reanchors at a policy chunk boundary (`_steps_since_boundary == 0`) from the live state payload: `flow_inference.py:824-828`.
  - Snippet: `targets[arm] = pose_from_state_payload(payload, arm)`.
- Flow clears target-pose state on streamed stalls: `flow_inference.py:674-678`.
  - Snippet: `self._clear_target_pose_state()`.
- UMI reanchors on deadman engagement by latching `arm_init` and `pika_init`: `umi_dual_cartesian.py:422-432`.
- C++ SMD reanchors on entry to `TcpPoseTarget` or after any non-target mode. `applyPoseTrackSmd()` resets the tracker to FK of `previous_sent_q_deg`: `rb_servo_server/src/control/dual_arm_servo_loop.cpp:202-231`.
  - Snippet: `tracker->reset(kinematics->computeTcpStand(... previous_sent_q_deg ...))`.

## 4. SMD Goal Receive and Goal Semantics

- The SMD call path is inside `computeServoTarget()`: first clamp absolute target pose to floor/ROI, then call `applyPoseTrackSmd()` for each arm: `dual_arm_servo_loop.cpp:3245-3275`.
  - Snippet: `cmd.tcp_target_stand = clampPoseToRoi(clampPoseToFloor(...))`.
- `applyPoseTrackSmd()` calls `tracker->updateGoalFromCommand(command.tcp_target_stand)` and replaces the command target with `tracker->step(dt_sec)`: `dual_arm_servo_loop.cpp:225-230`.
  - Snippets: `updateGoalFromCommand(...)`, `smoothed.tcp_target_stand = tracker->step(dt_sec)`.
- The goal is not simply the raw absolute command. First command after reset only latches reference; later command deltas integrate into the internal goal: `rb_servo_server/src/control/smd_pose_tracker.cpp:55-74`.
  - Snippets: `previous_command_ = command_pose; return;`, `goal_position_ += command_position - previous_position`.
- Cartesian IK then sees the SMD output as the `TcpPoseTarget` target pose: `rb_servo_server/src/control/cartesian_controller.cpp:65-76`.
  - Snippet: `target_tcp_stand = command.tcp_target_stand`.

## 5. Goal-Velocity Finite Difference and Spike Risk

- Yes. With `velocity_feedforward` enabled, the SMD estimates goal velocity from per-`step()` finite differences at the servo tick dt: `smd_pose_tracker.cpp:82-100`.
  - Snippets: `(goal_position_ - previous_goal_position_) / dt`, `log3(... goal_rotation_) / dt`.
- It then immediately stores the current goal for the next tick: `smd_pose_tracker.cpp:101-102`.
  - Snippet: `previous_goal_position_ = goal_position_`.
- With approx. 30 Hz ZOH feeding 500 Hz ticks, the internal goal is unchanged for most 2 ms ticks, then jumps by the full source-step delta on the tick after a new command arrives. That produces one tick of high goal velocity before returning to zero. The feedforward spike is norm-clipped if max velocity limits are positive; `clampNorm()` treats `max_norm <= 0` as unlimited: `smd_pose_tracker.cpp:25-30`, `:91-99`.

## 6. `velocity_feedforward`

- Config struct documents that goal velocity is estimated internally from the per-tick goal delta: `rb_servo_server/include/rb_servo/config/config.hpp:593-603`.
  - Snippet: `estimated internally from the per-tick goal delta`.
- Runtime code applies feedforward by damping on `(velocity_ - goal_linear_velocity)` and `(angular_velocity_ - goal_angular_velocity)`: `smd_pose_tracker.cpp:104-129`.
  - Snippets: `velocity_ - goal_linear_velocity`, `angular_velocity_ - goal_angular_velocity`.
- Config key is parsed as `cartesian_control.pose_track_smd.velocity_feedforward`: `rb_servo_server/src/config/config.cpp:2403-2416`, `:2455-2457`.

## 7. Velocity and Acceleration Limit Layers

- SMD pose-track caps are vector-norm limits on TCP-space velocity and acceleration, configured under `PoseTrackSmdConfig`: `config.hpp:580-603`.
  - Snippets: `max_linear_velocity_m_s`, `max_linear_accel_m_s2`, `max_angular_velocity_rad_s`, `max_angular_accel_rad_s2`.
- SMD applies linear acceleration and velocity clamps at `smd_pose_tracker.cpp:109-117`, and angular acceleration and velocity clamps at `:123-130`.
  - Snippets: `clampNorm(... max_linear_accel_m_s2)`, `clampNorm(... max_angular_velocity_rad_s)`.
- Joint-level safety clamps are separate and downstream of IK. `SafetyFilter::clampMotion()` performs joint limits, velocity, and acceleration: `rb_servo_server/src/control/safety_filter.cpp:119-129`.
  - Snippet: `out = clampVelocity(...); out = clampAcceleration(...)`.
- `dq_max_deg_s` limits per-tick joint motion: `safety_filter.cpp:190-202`.
  - Snippet: `max_step = config_.dq_max_deg_s[i] * dt_sec`.
- `ddq_max_deg_s2` limits per-tick joint velocity change: `safety_filter.cpp:204-227`.
  - Snippet: `max_dv = config_.ddq_max_deg_s2[i] * dt_sec`.

## 8. `output_moving_average_window`

- Config docs say it is a final-stage moving average applied after the safety filter: `rb_servo_server/include/rb_servo/config/config.hpp:507-510`.
- The loop constructs left/right moving averages from `config.servo.output_moving_average_window`: `dual_arm_servo_loop.cpp:1391-1392`.
- It is applied after `applySafety()` and before send bookkeeping: `dual_arm_servo_loop.cpp:2005-2031`.
  - Snippet: `output_filtered_target.left_q_target_deg = left_output_ma_.apply(...)`.
- The implementation confirms window `<= 1` is pass-through and otherwise averages the last N joint targets: `rb_servo_server/include/rb_servo/control/joint_moving_average.hpp:10-17`, `:22-32`.

## 9. IK and Safety Gates

- IK seed strategy: seed from previously sent target if finite, else actual state. This is explicit for 500 Hz `TcpPoseTarget`: `cartesian_controller.cpp:146-162`.
  - Snippet: `const JointArray seed_q_deg = isFiniteJoints(previous_safe_sent_q_deg) ? previous_safe_sent_q_deg : state.q_actual_deg`.
- Singular damping: Pinocchio IK uses SVD DLS, ramps damping toward `damping_max` below `singular_region_eps`, and records `last_applied_damping`: `rb_servo_server/src/kinematics/pinocchio_kinematics.cpp:487-511`.
  - Snippets: `sigma < eps`, `last_applied_damping = std::sqrt(max_lambda_sq)`.
- Branch-jump guard: `solveIk()` flags solution jumps over `max_solution_jump_deg`, optionally retries with higher damping, rate-limits, or clamps to seed: `pinocchio_kinematics.cpp:541-619`.
  - Snippets: `solution_jump_deg <= thresh`, `branch_jump_rate_limit`, `branch_jump_clamped_to_seed`.
- Safety filter joint clamp: `SafetyFilter::clampMotion()` applies joint limit, velocity, and acceleration clamps: `safety_filter.cpp:119-129`, with details at `:176-227`.
- Floor constraint:
  - Absolute Cartesian target pre-clamp before SMD: `dual_arm_servo_loop.cpp:3245-3256`, `:4287-4311`.
  - Final joint-space velocity projection/fail-closed gate: `dual_arm_servo_loop.cpp:3674-3778`.
- 3D TCP ROI box:
  - Absolute Cartesian target pre-clamp before SMD: `dual_arm_servo_loop.cpp:3245-3256`, `:4242-4285`.
  - Final joint-space velocity projection/fail-closed gate: `dual_arm_servo_loop.cpp:3781-3883`.
- Self-collision guard: async URDF mesh monitor is submitted final candidate targets, stale verdicts hold, hard violations latch, and near-pair constraints feed the shared projection: `dual_arm_servo_loop.cpp:3991-4094`.
  - Snippets: `collision_monitor_->submitTargets(...)`, `buildCollisionConstraints(...)`.
- Combined safety projection reconciles floor, ROI, reach, and self-collision rows in one solve: `dual_arm_servo_loop.cpp:4096-4151`.
  - Snippet: `solveVelocityProjection(...)`.

## 10. `servo_j` Parameters

- Canonical rbpodo fields are in `BackendConfig`: `servo_t1_sec`, `servo_t2_sec`, `servo_gain`, `servo_alpha`: `rb_servo_server/include/rb_servo/config/config.hpp:38-41`.
- YAML parsing allows and fills those canonical fields, while synchronizing deprecated aliases: `rb_servo_server/src/config/config.cpp:538-543`, `:594-600`.
- Real rbpodo validation checks positivity/range and requires `servo_t1_sec` to match the servo period: `config.cpp:1302-1334`.
- Rbpodo send path passes these config values directly to `move_servo_j(...)`: `rb_servo_server/src/robot/rbpodo_backend.cpp:1372-1380`.
  - Snippet: `impl_->config.servo_t1_sec, ... servo_alpha`.
- Phase-1 contract explicitly forbids modifying `servo_*` and runtime C++ (`CONTRACT.md:19-30`). This inspection proposes no servo parameter changes.

## Where A/B/C Are Currently Mixed

- Source-side command conditioning (A) already exists in runtime clients:
  - Flow integrates low-rate local deltas into absolute targets, clamps per source step, reanchors at chunk boundaries, and emits/holds intent state (`flow_inference.py:824-842`, `:552-616`).
  - UMI filters input, deadbands, clamps workspace/per-step deltas, and handles deadman hold/reanchor (`umi_dual_cartesian.py:389-446`).
- C++ command conditioning and reference generation are mixed before and inside SMD:
  - Floor/ROI absolute-pose pre-clamps happen before SMD (`dual_arm_servo_loop.cpp:3245-3256`), so safety conditioning changes the command that the SMD sees.
  - `applyPoseTrackSmd()` both reanchors to FK of previous sent joints and invokes SMD (`dual_arm_servo_loop.cpp:202-231`).
  - `SmdPoseTracker::updateGoalFromCommand()` turns absolute command samples into an integrated delta goal (`smd_pose_tracker.cpp:55-74`).
- SMD reference generation (B) is mixed with feedforward and saturation policy:
  - `step()` estimates goal velocity from per-tick finite differences, applies optional FF, and clamps TCP velocity/acceleration in the same function (`smd_pose_tracker.cpp:82-130`).
- Output polish (C) is mostly separate but downstream of safety and IK:
  - `output_moving_average_window` smooths safety-passed joint targets after `applySafety()` (`dual_arm_servo_loop.cpp:2005-2031`). It is not a pure Cartesian reference stage.

## Pipeline Diagram

```text
flow / UMI / eval source
  -> CommandIntent per arm {mode: TcpPoseTarget, tcp_target_stand}
  -> ServoCommandClient UDP JSON
  -> CommandServer parseArmObject/readOptionalPose6D
  -> CommandBuffer latestOrHold()  [server ZOH until timeout]
  -> computeServoTarget()
       -> floor/ROI absolute target pre-clamp
       -> applyPoseTrackSmd()
            reset anchor = FK(previous_sent_q)
            updateGoalFromCommand(command delta integrated into SMD goal)
            step(dt): goal-velocity finite diff, optional FF, vel/acc caps
       -> CartesianController TcpPoseTarget target_tcp_stand = SMD output
       -> IK: previous-sent seed, singular damping, branch-jump guard
  -> applySafety()
       joint limits + dq/ddq clamps
       floor + ROI + reach + self-collision velocity projection
  -> output_moving_average_window  [joint output polish after safety]
  -> ServoDispatcher / backend SendServoJRequest
  -> rbpodo move_servo_j(q, servo_t1_sec, servo_t2_sec, servo_gain, servo_alpha)
```

## CONTRACT Cross-Check

- Match: `cartesian_control.pose_track_smd.*` is the actual runtime key family; parser validates `enable`, `damping_ratio_linear`, `natural_frequency_linear_hz`, `damping_ratio_angular`, `natural_frequency_angular_hz`, `max_linear_velocity_m_s`, `max_linear_accel_m_s2`, `max_angular_velocity_rad_s`, `max_angular_accel_rad_s2`, `velocity_feedforward`: `config.cpp:2403-2416`.
- Match: `servo.output_moving_average_window` exists and is final-stage joint output polish: `config.hpp:507-510`, `dual_arm_servo_loop.cpp:2024-2031`.
- Match: `safety.dq_max_deg_s` and `safety.ddq_max_deg_s2` are the joint-level clamp keys reflected in `SafetyFilter`: `safety_filter.cpp:190-227`.
- Match: `kinematics.ik.*` includes branch/singular keys: `singular_region_eps`, `damping_max`, `max_solution_jump_deg`, `branch_jump_damping_scale`, `branch_jump_max_retries`, `branch_jump_clamp_to_seed`, `branch_jump_rate_limit`: `config.cpp:2487-2518`.
- Mismatch to flag for Phase-1 tooling: `CONTRACT.md` section 11 sweep names such as `nat_freq_lin`, `nat_freq_ang`, `damp_lin`, `damp_ang`, `vel_ff`, `max_lin_vel`, `max_ang_vel`, and `ma_window` are shorthand labels, not the real C++ config keys. Tooling should map them explicitly to the real keys above.
- No code contradiction found with the contract's hard Phase-1 boundaries: the real runtime has active SMD/IK/safety/servo knobs, but Phase 1 should only inspect/log/model them offline and must not modify them.

## Key Takeaways for Phase-1 Offline Tooling

- Reproduce current source behavior as `raw_zoh`: held low-rate absolute `tcp_target_stand` commands feeding a 500 Hz server-side ZOH.
- Treat Flow/UMI command conditioning as A-stage runtime behavior: reanchor, clamp, deadband, moving average, and chunk/deadman hold are not part of the C++ SMD alone.
- Model the current SMD B-stage as delta-integrated goals anchored at FK(previous sent q), not as direct filtering of raw absolute target samples.
- If simulating `velocity_feedforward`, include the 2 ms goal finite-difference; otherwise Phase-1 will miss the possible one-tick clipped spike at 30 Hz command updates.
- Keep C-stage output MA as a joint-space, post-safety filter in metrics/log schemas; it is not Cartesian source conditioning or SMD reference generation.
