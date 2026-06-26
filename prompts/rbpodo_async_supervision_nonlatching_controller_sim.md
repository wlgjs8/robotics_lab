# TASK: Make rbpodo async streaming supervision NON-LATCHING in controller-simulation (1hr-stable pgmode teleop)

## Goal

In rbpodo controller `pgmode` simulation, live UMI teleop must run for >= 1 hour
without the server latching a permanent `SendFailure` fault that kills control.
Today the async streaming supervision latches on transient/sustained controller
degradation (the controllers come up `diagnostics_suspect`/-2001 and intermittently
lag dispatch/ACK/q_ref), which drops robot control and forces a server restart.

The fix is comprehensive and single-point: **in controller-simulation mode, the
async streaming supervision must be ADVISORY (telemetry + warning), never a
latching top-level fault.** pgmode has `physical_motion_expected=false` and no
physical motion, so a non-following controller has no physical-safety
consequence — the supervision's job (stop motion on controller non-follow) is
moot there. REAL mode behavior MUST stay exactly as today (latches).

This is the proper fix replacing per-path config band-aids (max_pending_age,
reference_supervision disable) that only move the next fault path.

## Background (verified — do not re-investigate)

- The async supervision sets `telemetry.supervision_state = Fault` from several
  paths in `src/control/arm_worker.cpp`
  (`updateAsyncAckSupervisionLocked`, `updateAsyncReferenceSupervisionLocked`):
  missing-ack streak (`max_consecutive_missing_ack`), timing-reject streak
  (`consecutive_async_timing_rejects_`, already controller-sim tolerant via
  `controller_simulation_timing_reject_tolerance_enabled`), a single non-timing
  `!accepted` (immediate Fault), and the q_ref/tcp_ref watchdog
  (`async_q_ref_watchdog_miss`). The state self-recovers to `Ok` when sends
  resume succeeding (arm_worker.cpp ~797-799).
- The LATCH happens at TWO sites in `src/control/dual_arm_servo_loop.cpp`
  (~1685 and ~1941), both:
  ```cpp
  if (rbpodoAsyncIoMode() && !fault_latched_.load()) {
      const LatchedDualFaultContext async_fault_contexts =
          asyncSupervisionFaultContexts(left_async_telemetry, right_async_telemetry);
      if (async_fault_contexts.top_level.has_value()) {
          latchFault(SafetyVerdict::SendFailure,
                     "rbpodo async streaming supervision fault", ...);
          // (second site also sets safe_target/safety_verdict=FaultLatched)
      }
  }
  ```
  `asyncSupervisionFaultContext()` returns a context iff `supervision_state == Fault`.
- The controller-sim gate is `controllerSimulationMotionGateOpen(config)`
  (dual_arm_servo_loop.cpp:230), already used to enable the timing-reject
  tolerance (dual_arm_servo_loop.cpp:1270).

## Deliverable

1. **Config flag** `servo.controller_simulation_async_supervision_nonlatching`
   in `include/rb_servo/config/config.hpp` (default `false`), parsed in
   `src/config/config.cpp` with the other servo flags. Document it.

2. **Gate** (compute once, e.g. near the other controller-sim gating in
   dual_arm_servo_loop): `async_supervision_nonlatching =
   controllerSimulationMotionGateOpen(config) &&
   config.servo.controller_simulation_async_supervision_nonlatching`.
   Inert/false in real mode (fail-closed).

3. **Both latch sites (~1685, ~1941)**: when `async_supervision_nonlatching` is
   true and the async fault context is present, DO NOT call `latchFault(...)` and
   DO NOT set `safety_verdict = FaultLatched` / `safe_target = currentFaultHoldTarget()`.
   Instead treat it as a recoverable advisory: continue normal command handling.
   Keep it observable — see #4. Real mode (gate false) keeps the exact current
   latching behavior. Make sure the two sites stay consistent (factor a small
   helper if it reduces duplication).

4. **Observability (required — no silent masking)**: surface the degradation so
   the operator/viser/logs can SEE the controller is hiccuping even though it no
   longer latches:
   - Add a published advisory in the state JSON (`src/network/state_publisher.cpp`),
     e.g. top-level `async_supervision_degraded: bool` (true while a controller-sim
     async fault context is present-but-suppressed) plus the already-available
     async telemetry (supervision_state, missing_ack_count, q_ref_watchdog_miss_count,
     commands_dropped_total, last_failure). Per-arm is fine if cleaner.
   - Emit a THROTTLED `[WARN]` (e.g. once per N seconds or on state change) like
     "controller-sim async supervision degraded (suppressed, not latched): <reason>"
     so logs aren't flooded at 500Hz.
   - Whatever struct field you add (e.g. in `include/rb_servo/core/types.hpp`)
     must default to the non-degraded value.

5. Do NOT change `arm_worker.cpp` supervision-state logic — only the LATCH
   decision in dual_arm_servo_loop + the new flag + telemetry surfacing.

## Hard constraints (read AGENTS.md, REVIEW.md, docs/servo_backend_contract.md, CLAUDE.md first)
- Controller-simulation ONLY and config-opt-in (default false). REAL mode
  (`run_mode: real` without the controller-sim pgmode carve-out, or flag false)
  must latch async supervision faults exactly as today — verify with a test.
- Do NOT touch `RB_ALLOW_REAL_CARTESIAN` / real-motion gates. Preserve
  `BackendResult<RobotState>`, structured `BackendError`, fail-closed real.
- EmergencyStop, ResetFault, robot_state-error, tracking-error, and all
  NON-async-supervision fault paths must keep latching as today (only the async
  streaming supervision SendFailure latch is made advisory in controller-sim).

## Tests + gate
- Unit/integration (C++): with the flag ON + controller-sim gate open, a
  telemetry `supervision_state == Fault` (any path) does NOT latch
  (`fault_latched` stays false, motion command continues, advisory flag set in
  published state). With the flag OFF (or real mode), the same telemetry DOES
  latch `SendFailure` (unchanged). Cover both latch sites.
- Build + run: `cmake -S rb_servo_server -B rb_servo_server/build &&
  cmake --build rb_servo_server/build -j && ctest --test-dir rb_servo_server/build
  --output-on-failure` (Eigen3 + Pinocchio). Do NOT claim the gate passed unless
  it ran green.
- Enable the flag in the current tracked pgmode/controller-simulation example
  config, if one exists for the task, with a comment and document it in
  docs/runbooks/rbpodo_pgmode_umi.md +
  docs/servo_backend_contract.md.

## Out of scope
- The vendor diagnostics_suspect/-2001 SDK decode fix (separate).
- Real (non-pgmode) Cartesian motion promotion. Real-mode latching unchanged.

When done: summarize changed files, the config flag name/default, the gating
expression, how the advisory is surfaced (state field + throttled log), and the
real ctest output.
