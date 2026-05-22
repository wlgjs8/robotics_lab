# Control Loop

`DualArmServoLoop` is the only high-rate control thread.

Default mock target:

```text
200 Hz → 5 ms period
```

## Per-tick flow

```text
1. measure loop_start_time_ns
2. compute actual period
3. compute capped filter_dt
4. read left/right robot state and validate joint state
5. read pending lifecycle command or latest motion command from CommandBuffer
6. stale command → Hold
7. ResetFault command → clear latched fault if present, return to ConnectedHold, and Hold
8. EmergencyStop command → latch fault hold pose
9. validate payloads; missing payload → Hold/InvalidCommand
10. Cartesian modes currently → Hold/CartesianUnavailable
11. TrajectoryFilter computes left/right joint target
12. SafetyFilter clamps target and checks robot/tracking state
13. safety failure policy:
    - snap_to_actual for mock/rbsim tracking error
    - fault_latch for real tracking error
    - robot state error can latch fault
14. send left/right target through IRobotBackend and record send timestamps
15. publish latest `ServoSnapshot` for debug/publisher/test readers, including
    command seq/modes, sent targets, period/jitter/filter dt, send timing, and
    logger drop count
16. push ServoSample to async logger
17. sleep_until(next_tick)
```

## Timing columns

The logger records:

- `period_ms`: actual delta between loop starts
- `jitter_ms`: absolute deviation from nominal period
- `filter_dt_ms`: capped dt used by trajectory/safety math
- `loop_start_time_ns`
- `loop_end_time_ns`
- `logger_dropped_samples`: total samples dropped by the bounded logging queue
- `left_send_start_ns`, `left_send_end_ns`, `right_send_start_ns`, `right_send_end_ns`
- `send_skew_us`, `left_send_duration_us`, `right_send_duration_us`

These are used to decide whether 100–200 Hz is stable enough before trying rbsim/real hardware.

## Snapshot ownership

`DualArmServoLoop` owns robot state reads. Other components must observe servo state through the latest `ServoSnapshot`, not by reading robot backends directly. This keeps the mock plant from advancing twice when the state publisher is enabled and gives tests/debug tools one thread-safe read surface for command seq/modes, motion state, fault state, actual/sent/previous-sent targets, timing, logger health, and send timing.

## Hold behavior

Hold uses the previous sent target, not the current actual q every tick.

Reason:

- chasing `q_actual` can create micro target drift
- previous sent target is a stable hold latch

On a latched fault, the server holds a dedicated `fault_hold_q` captured from current actual q if available, otherwise from the last safe target.

## Command timeout

The C++ receive timestamp is authoritative. If the latest command becomes stale, both arms hold by default. If an invalid timeout somehow reaches `CommandBuffer`, the command is converted to Hold; the safety path does not silently substitute a hard-coded timeout.

Lifecycle commands (`ArmMotion`, `DisarmMotion`, `EmergencyStop`, `ResetFault`) are queued separately from the latest motion target. This prevents an immediate `JointTarget` packet from overwriting `ArmMotion` before the servo loop observes it.

## Safety behavior

The first scaffold safety checks are:

- robot connected
- no robot error state
- joint position clamp
- joint velocity clamp
- joint acceleration clamp
- tracking error threshold
- missing payload guard
- Cartesian unavailable guard
- emergency-stop fault latch

The tracking guard checks:

```text
abs(previous_sent_q - q_actual) <= max_tracking_error_deg
```

Recommended policy:

```yaml
# mock/rbsim
tracking_error_policy: snap_to_actual

# real
tracking_error_policy: fault_latch
```

## Fail-safe invariant

The server should never generate a zero joint target merely because something failed.

```text
invalid command → previous safe target
unsupported Cartesian/IK → previous safe target
tracking error with fault_latch → latched current/last-safe target
robot state error → latched current/last-safe target
EmergencyStop → latched current/last-safe target
```

## Still pending

- parallel left/right send
- time-based action chunk interpolation
- Ruckig or jerk-limited interpolation
- lock-free/latest command buffer for 500 Hz experiments
