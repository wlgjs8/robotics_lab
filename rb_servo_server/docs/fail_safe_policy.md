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
| IK/follower refusal | Hold/decelerate from the previous safe target and publish the structured reason |
| joint command outside limits | clamp to configured limits |
| one late servo tick | filter dt is capped |
| invalid or missing robot joint state | Startup fails; runtime latches/holds last safe pose according to policy |
| tracking error in mock/simulator | snap target to actual by default |
| tracking error in real | latch fault by default |
| robot disconnected/error | latch current/last-safe pose by default |
| EmergencyStop | latch current/last-safe pose |
| ResetFault | clear fault only; return to ConnectedHold |
| read-only mode (`servo.send_servo_commands=false`) | connect/read/publish state; suppress all `sendServoJ` calls |
| motion command in read-only mode | reject and hold previous safe target |
| sendServoJ failure in mock/simulator | failed arm target is not recorded; optional stop-both latch |
| sendServoJ failure in real | fault latch |
| force control requested without a live valid tare bias | nominal motion continues without force coverage; publish the refusal reason |
| automatic tare armed but InitMotion fails/cancels/faults | old bias remains invalid; force coverage stays refused |

## Motion state

The server starts in `ConnectedHold`. Motion commands are ignored until an explicit:

```json
{"seq": 1, "mode": "ArmMotion"}
```

`DisarmMotion` returns to `ConnectedHold`. `ResetFault` returns to `ConnectedHold` only after backend reset succeeds and a fresh valid robot state is read; it must not resume motion directly.

If a command includes both `EmergencyStop` and `ResetFault`, `EmergencyStop` wins.

## Read-only mode

Set:

```yaml
servo:
  send_servo_commands: false
```

Read-only mode lets the server connect to backends, read robot state, and publish state snapshots without sending servo motion. `ArmMotion`, `JointTarget`, `TcpPoseTarget`, and `TcpLinearMove` are fail-closed and hold the previous safe target. `Hold` remains safe, but it is also not sent to the backend.

State output marks the policy with `send_suppressed: true` and `send_policy: "read_only"` so suppression is distinguishable from a send failure. In read-only mode `send_ok` means "policy suppressed without backend send failure", not "the robot accepted a servo command".

`EmergencyStop` only latches the server fault state in read-only mode; the loop still does not send motion. `ResetFault` does not call backend reset APIs while read-only is active, and an existing fault latch remains until the server is restarted or read-only is disabled under the normal real-motion gates.

## ResetFault

A fault latch ignores motion commands until reset.

```json
{"seq": 10, "mode": "ResetFault"}
```

After reset succeeds, the server re-baselines previous targets to the freshly read current actual q and remains in `ConnectedHold`. If backend reset fails or the post-reset state is invalid, the fault latch remains active. Send `ArmMotion` again before sending motion targets.

## Real mode guard

Real mode is config-driven, not env-gated (the legacy `RB_ALLOW_REAL_*` execution
gates were removed from the server runtime). Real mode requires:

- the tracked `rb_servo_server/config/stack_real.yaml` explicitly enabling the
  intended motion path
- `servo.enable_realtime_priority=true`
- successful realtime setup in the servo loop
- `safety.tracking_error_policy=fault_latch`
- `safety.stop_both_arms_on_single_arm_error=true`
- `safety.latch_fault_on_robot_state_error=true`
- the configured safety, lease/deadman, and backend-readiness checks

## Robot State Validity

Startup requires both backends to return a connected, error-free, finite joint state inside configured joint limits. Backends must set `RobotState::has_valid_joint_state=true` only after reading real joint data from a trusted source.

`RbpodoBackend` must report valid state only after reading real joint data from
a trusted rbpodo controller path. Compiling with `RB_SERVO_ENABLE_RBPODO=ON`
does not bypass the config-driven real-motion gating: real connection and
`servo_j` transmission still require the tracked real stack to explicitly
enable them (`servo.send_servo_commands: true`), plus the mode-independent
safety layers.

## Cartesian/IK rule

Cartesian solving returns an explicit result and structured telemetry:

```cpp
struct CartesianSolveResult {
    bool ok;
    JointArray q_target_deg;
    SafetyVerdict failure_reason;
};
```

Never return a default-constructed `JointArray` on IK failure. Return `ok=false`, and let `DualArmServoLoop` hold or latch according to config.

J3 is never widened as an IK fallback. Its supported safety/model envelope is
exactly `[-150 deg, +150 deg]`; a pose outside that envelope is unreachable.

## Force-control fail-closed rules

Force-law parameters are tracked server config, not command payload. A client
`force_control` object is rejected. `TareForceSensor` is the only public force
lifecycle command and is leaseless because a valid bias is a coverage
precondition.

Automatic tare invalidates the old bias on InitMotion request, waits until the
arm is parked and settled, then uses the same RT `raw - gravity` average as
manual tare. Failure to complete produces no guessed/default bias.
