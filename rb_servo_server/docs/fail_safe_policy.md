# Fail-safe Policy

This project treats every command-to-joint conversion as fallible. A failure must never become a synthesized zero pose.

## Invariant

```text
No failure path may output [0, 0, 0, 0, 0, 0] unless that was a validated user command.
```

## Failure handling table

| Situation | Behavior |
|---|---|
| stale command | Hold previous safe target |
| malformed JSON or unknown mode | Drop packet; command buffer is unchanged |
| missing `q_target_deg` for `JointTarget` | Drop packet; command buffer is unchanged |
| malformed numeric payload or array length | Drop packet; command buffer is unchanged |
| `timeout_sec <= 0` | Drop packet; command buffer is unchanged |
| invalid timeout already inside command buffer | Hold; do not substitute a default motion timeout |
| lifecycle command immediately followed by a motion command | Process lifecycle command first; then the latest motion command |
| unsupported Cartesian command | Hold previous safe target |
| future IK failure | Hold previous safe target or fault latch |
| joint command outside limits | clamp to configured limits |
| one late servo tick | filter dt is capped |
| invalid or missing robot joint state | Startup fails; runtime latches/holds last safe pose according to policy |
| tracking error in mock/rbsim | snap target to actual by default |
| tracking error in real | latch fault by default |
| robot disconnected/error | latch current/last-safe pose by default |
| EmergencyStop | latch current/last-safe pose |
| ResetFault | clear fault only; return to ConnectedHold |
| sendServoJ failure in mock/rbsim | failed arm target is not recorded; optional stop-both latch |
| sendServoJ failure in real | fault latch |

## Motion state

The server starts in `ConnectedHold`. Motion commands are ignored until an explicit:

```json
{"seq": 1, "mode": "ArmMotion"}
```

`DisarmMotion` returns to `ConnectedHold`. `ResetFault` returns to `ConnectedHold` only after backend reset succeeds and a fresh valid robot state is read; it must not resume motion directly.

If a command includes both `EmergencyStop` and `ResetFault`, `EmergencyStop` wins.

## ResetFault

A fault latch ignores motion commands until reset.

```json
{"seq": 10, "mode": "ResetFault"}
```

After reset succeeds, the server re-baselines previous targets to the freshly read current actual q and remains in `ConnectedHold`. If backend reset fails or the post-reset state is invalid, the fault latch remains active. Send `ArmMotion` again before sending motion targets.

## Real mode guard

Real mode requires:

- `RB_ALLOW_REAL_ROBOT=1`
- `servo.enable_realtime_priority=true`
- successful realtime setup in the servo loop
- `safety.tracking_error_policy=fault_latch`
- `safety.stop_both_arms_on_single_arm_error=true`
- `safety.latch_fault_on_robot_state_error=true`
- loopback `network.command_bind` and `network.state_pub_bind`, unless `RB_ALLOW_NETWORK_EXPOSURE=1` is explicitly set

## Robot State Validity

Startup requires both backends to return a connected, error-free, finite joint state inside configured joint limits. Backends must set `RobotState::has_valid_joint_state=true` only after reading real joint data from a trusted source.

Until the rbpodo data channel is implemented, `RbpodoBackend` refuses to report valid state or accept servo targets even when compiled with `RB_SERVO_ENABLE_RBPODO=ON`.

## Future Cartesian/IK rule

When `CartesianController` is implemented, its API should return an explicit result:

```cpp
struct CartesianSolveResult {
    bool ok;
    JointArray q_target_deg;
    SafetyVerdict failure_reason;
};
```

Never return a default-constructed `JointArray` on IK failure. Return `ok=false`, and let `DualArmServoLoop` hold or latch according to config.
