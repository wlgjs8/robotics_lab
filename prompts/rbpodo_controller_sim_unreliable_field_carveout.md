# TASK: Controller-simulation decode carve-out for mis-decoded rbpodo status fields (unblock -2001)

## Background (root cause ALREADY pinpointed — do not re-investigate)

In rbpodo controller `pgmode` simulation, `rb_servo_server` marks both RB3-730E
controllers `diagnostics_suspect` / `error_code -2001`
(`interpretRbpodoDiagnostics` in `src/robot/rbpodo_backend.cpp`). This forces a
startup `robot_fault` and the async ServoJ supervision then latches a
`SendFailure` within ~1s, blocking all UMI/SpaceMouse pgmode teleop motion.

Read-only live capture (artifacts in `artifacts/rbpodo_measurement/diag_2001/`)
proved the cause precisely. The per-arm suspect `reason` is exactly:

```
op_stat_self_collision expected 0/1 but was 1984732816; time was implausible: 0.000000
```

- ONLY two fields are bad: `op_stat_self_collision` decodes to a garbage ~1.98e9
  value (0x7655xxxx band, varies per sample, differs per arm) instead of 0/1,
  and `robot_time_sec` reads 0.
- ALL real safety fields decode correctly and are 0: `op_stat_collision_occur`,
  `op_stat_soft_estop_occur`, `op_stat_ems_flag`, `op_stat_sos_flag`; plus
  `real_vs_simulation_mode=1`, `init_state_info=6`, `init_error=0`.
- The rbpodo PYTHON binding shows the SAME garbage on 80/80 samples
  (`rbpodo_python_diagnostics_suspect_rate=1.0`), so this is NOT a robotics_lab
  C++ mapping bug and NOT a real self-collision — it is an rbpodo SDK <->
  controller firmware field-layout mismatch (vendor level). The real
  `op_stat_self_collision == 1` collision-fault path is separate and unaffected
  (garbage != 1, so it never trips a real self-collision fault).

The vendor/SDK fix is deferred. This task is the robotics_lab-side
controller-simulation carve-out to UNBLOCK pgmode teleop without masking the
fields that DO decode correctly.

## Deliverable — Part 1 (primary): controller-sim "unavailable field" decode policy

Add a **config-gated, controller-simulation-only** decode policy so that, when
enabled, `interpretRbpodoDiagnostics` treats `op_stat_self_collision` and the
`robot_time_sec` plausibility as **unavailable** (does not set
`diagnostics_suspect`), while STILL enforcing every other check
(`op_stat_sos_flag` 0..12, `op_stat_ems_flag` 0..4, `op_stat_soft_estop_occur`
0/1, `op_stat_collision_occur` 0/1, `real_vs_simulation_mode` known). A genuine
EMS / soft-estop / collision must still trip suspect/fault.

Requirements:
1. **Config flag** in `BackendConfig` (sibling to
   `allow_controller_simulation_diagnostics_suspect`), canonical snake_case name,
   default `false`. Suggested:
   `controller_simulation_treat_unreliable_status_fields_as_unavailable`.
   Parse it in `src/config/config.cpp` with the other backend flags; default
   false; document in the config example/comments.
2. **Gate**: the carve-out is active ONLY when
   `rbpodoControllerSimulationMotionGateOpen(config)` is true AND the new flag is
   set. It MUST be inert in real mode (`run_mode != Real` or not pgmode sim) —
   fail-closed. Never key it off env alone.
3. **Thread the option into `interpretRbpodoDiagnostics`**: change its signature
   to accept the decode options (or a small struct), populated at the `mapState`
   call site (line ~339) from `config`. When the option is active:
   - SKIP the `markSuspiciousFlag(..., "op_stat_self_collision", ...)` check.
   - SKIP the `robot_time_sec` zero/implausible-small branch (treat sim time 0 as
     acceptable). Keep the non-finite check ONLY if you still want it; simplest is
     to treat the whole time plausibility as unavailable under the carve-out.
   - Do NOT skip any other field.
4. **Telemetry honesty (required, no silent masking)**: record which fields were
   treated as unavailable. Add e.g. `std::vector<std::string> unavailable_fields`
   (or a stable string) to `RbpodoDiagnosticsSnapshot` and surface it in the
   state JSON via `state_publisher.cpp`, plus set
   `rbpodo_state_decode_policy` to a distinct value when the carve-out is active
   (e.g. `"controller_sim_unreliable_fields_unavailable"`). The operator must be
   able to SEE that self_collision/time were suppressed. Keep the human-readable
   `reason` accurate (don't claim clean if fields were suppressed — note them).
5. When the carve-out suppresses the only two failing checks, `diagnostics_suspect`
   becomes false, so the arm comes up with `lifecycle_state` servo_enabled /
   connected (no -2001, no startup robot_fault from this path).

## Deliverable — Part 2 (secondary): async controller-sim single-reject tolerance

Today `updateAsyncAckSupervisionLocked` (`src/control/arm_worker.cpp`, ~line
763-772) sets `supervision_state = Fault` on a SINGLE `!result.accepted`
(including a timing `arm_worker_command_expired` / `arm_worker_send_result_timeout`),
with zero tolerance. Under controller simulation a single late send then latches
SendFailure. Add a **controller-simulation-only** small consecutive tolerance
(reuse/observe `max_consecutive_missing_ack`, default already 10/30) for the
`!accepted` timing-error path before latching Fault — i.e. require N consecutive
timing rejects, not one — gated so REAL mode keeps the current strict
zero-tolerance behavior. Do not relax genuine backend rejects (NAK / robot fault)
— only the CommandTimeout-family timing expiries. If this is non-trivial or risks
real-mode semantics, leave Part 1 only and clearly note Part 2 as not done; the
local config already relaxed `max_pending_age_ms` 10->100.

## Hard constraints (read AGENTS.md, REVIEW.md, docs/servo_backend_contract.md, CLAUDE.md first)
- rbpodo backend only; preserve `BackendResult<RobotState>`, structured
  `BackendError` (no string parsing), fail-closed real semantics.
- The carve-out is controller-simulation ONLY and config-opt-in; REAL mode
  behavior must be byte-for-byte unchanged when the flag is false.
- Do NOT weaken the real collision / soft-estop / EMS / SOS / self-collision==1
  fault paths. Do NOT touch `RB_ALLOW_REAL_CARTESIAN` or real-motion gates.
- No silent masking: suppressed fields must be visible in telemetry.

## Tests + gate
- Unit tests (C++): with the flag OFF, the captured garbage `op_stat_self_collision`
  (e.g. 1984732816) + `robot_time_sec=0` still yields `diagnostics_suspect=true`
  (unchanged). With the flag ON in controller-sim gate-open conditions, the SAME
  snapshot yields `diagnostics_suspect=false`, `unavailable_fields` lists
  self_collision (+ time), and a real `op_stat_collision_occur=1` or
  `op_stat_self_collision==1` STILL faults. Real-mode (flag irrelevant) unchanged.
- Build + run the gate: `cmake -S rb_servo_server -B rb_servo_server/build &&
  cmake --build rb_servo_server/build -j && ctest --test-dir rb_servo_server/build
  --output-on-failure` (Eigen3 + Pinocchio required). Keep Python suites green
  if touched. Do NOT claim the gate passed unless it actually ran.
- Enable the new flag in the local pgmode config
  `rb_servo_server/config/local/dual_real_rbpodo_pgmode_spacemouse_500hz_ack.yaml`
  AND in the tracked example
  `rb_servo_server/config/dual_real_rbpodo_pgmode_spacemouse_500hz_ack.example.yaml`
  (the example with a clear comment; the local copy is gitignored). Document the
  carve-out in `docs/runbooks/rbpodo_pgmode_umi.md` and
  `docs/runbooks/rbpodo_measurement_reliability.md`.

## Out of scope
- The vendor/SDK field-layout fix (deferred).
- Real (non-pgmode) Cartesian motion.

When done: summarize changed files, the exact config flag name/default, how the
carve-out is gated, test results, and the precise build/ctest command output.
