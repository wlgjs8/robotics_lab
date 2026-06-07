# Plan: Full operation in rbpodo pgmode simulation (controller-simulation)

Status: code-level spec for staged implementation (Claude Code → codex per stage).
Scope (approved): **A** server Cartesian unblock for controller-simulation + **B** rb_gui full operator for the pgmode-sim rbpodo backend.
Out of scope (future): C gripper end-to-end, D server-side `TcpCircleTrack`, E force control.

## 0. Goal & motivation

In **rbpodo pgmode simulation** (`backend_type=rbpodo`, `run_mode=real`, `operation_mode=simulation`) the
controller cannot move a physical arm (`physical_motion_expected=false`; the Virtual ControlBox has no
real-robot mode at all). The code is deliberately conservative and blocks many operations there that are
perfectly safe in simulation. Goal: make pgmode simulation a first-class, **fully operable** mode — drive
every joint and Cartesian primitive, and operate it all from rb_gui — without ever relaxing real behavior.

## 1. Hard safety boundary (every stage must hold)

- Changes apply ONLY when `operation_mode == simulation` (controller-simulation), composed with the
  EXISTING controller-simulation Cartesian gate. `operation_mode == real` paths stay **byte-for-byte
  identical**.
- Never set or weaken `RB_ALLOW_REAL_ROBOT` / `RB_ALLOW_REAL_MOTION` / `RB_ALLOW_REAL_CARTESIAN` or any
  real gate. The controller-sim Cartesian gate already requires `RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1`
  + `RB_RBPODO_PGMODE_SIMULATION_CONFIRMED=1` + `cartesian_control.allow_in_controller_simulation=true`
  (+ motion gate). Reuse it; do not invent a weaker path.
- rb_gui keeps **real mode status-only** (no motion when desired/observed mode is real). New capability is
  gated to observed `operation_mode==simulation`.
- `force_control` stays `provider:null, enable:false`. No real Cartesian. Preserve structured
  BackendResult/FaultContext contracts; no error-string parsing.
- Each stage: `cmake -B build_rbpodo -DRB_SERVO_ENABLE_RBPODO=ON` build, `ctest` (rbpodo), `HARDEN-10`
  green (real-mode unchanged), new unit tests, then live check against the two pgmode-sim VMs
  (10.0.2.7/10.0.2.8). codex implements one stage at a time; Claude Code reviews the diff before build.

## 2. Verified blocker inventory (file:line)

Already relaxed (exist): `allow_controller_simulation_motion`, `..._diagnostics_suspect`, `..._init_error`,
`..._not_activated`. Joint + `TcpPoseTarget` + `TcpDelta{Local,Stand}` already work in pgmode sim (WU-04/05).

### Stage A — server Cartesian primitives gated by `run_mode != Simulation` (should also allow controller-sim)
- `TcpLinearMove`: `src/control/cartesian_controller.cpp:77-84` and `src/control/cartesian_servo_controller.cpp:414-419` → `tcp_linear_move_simulation_only` when `run_mode != RunMode::Simulation`.
- `TcpTwistLocal`/`TcpTwistStand`: `src/control/cartesian_servo_controller.cpp:673-678` → `tcp_twist_simulation_only` when `run_mode != RunMode::Simulation`. (VERIFY exact site/behavior — one explorer reported twist already dispatched; confirm the gate before editing.)
- `TcpCircleMove`: `src/control/cartesian_servo_controller.cpp:835-849` + `src/control/dual_arm_servo_loop.cpp:325-327` (config `circle_move.allow_in_simulation && !allow_in_real`).
- Existing correct pattern to mirror: `controllerSimulationCartesianGateOpen()` and the cartesian-availability gate in `dual_arm_servo_loop.cpp:266-385` already key on `operation_mode==simulation` + the env/config gate for `TcpPoseTarget`/`TcpDelta`/`TcpCircleTrack` path.

### Stage B — rb_gui blocks
- TCP commands require simulator backend: `rb_gui/rb_servo_gui/safety.py:202-203` `if observed_backend != "simulator": return "TCP pose command requires simulator backend"` → blocks Cartesian for rbpodo pgmode sim.
- Feature flag + observed-sim gates: `safety.py:195-203` (`RB_GUI_ENABLE_TCP_POSE_COMMANDS`, observed simulation), `safety.py:183-185` (sim readiness).
- Missing controls: `TcpTwistLocal/Stand`, `JointVelocity`, gripper (gripper deferred to C). Command builders in `rb_gui/rb_servo_gui/command_client.py`.
- Real mode block to PRESERVE: `safety.py:174-175`.

## 2b. CRITICAL refinement — measure before editing

Static analysis is contradictory and must be grounded empirically. `isCartesianVelocityServoMode`
(`dual_arm_servo_loop.cpp:41-47`) ALREADY includes `TcpLinearMove`, `TcpCircleMove`, `TcpCircleTrack`,
`TcpTwistStand`, `TcpTwistLocal`. And `cartesianComputationRunModeForArm` (`:387-398`, called at
`:2954-2956`) maps the **effective** run_mode Real→Simulation for those modes when
`controllerSimulationCartesianGateOpen` is true. So the per-primitive `run_mode != Simulation` gates in
`cartesian_servo_controller.cpp:414/673/835` likely RECEIVE the mapped (Simulation) run_mode in
controller-sim and may already PASS. The static inventory (which read raw `run_mode`) probably overstated
the blockers.

**Stage A.0 (do first): empirical capability probe.** With `vm_dual_cartesian.yaml` + full controller-sim
Cartesian env, send EACH Cartesian primitive (TcpPoseTarget, TcpDelta{Local,Stand}, TcpLinearMove,
TcpTwistLocal/Stand, TcpCircleMove) and record accept/reject + exact reason + whether q_ref/TCP tracks.
Only then fix the genuinely-blocked ones. Candidate genuine gaps to confirm: (i) `cartesian_controller.cpp:78`
TcpLinearMove gate IF that controller path is used with RAW run_mode (vs the servo controller that gets the
mapped run_mode); (ii) `circle_move.allow_in_simulation` config requirement for TcpCircleMove; (iii) rb_gui
backend block (Stage B). Sending Cartesian via policy_runner needs geometry config (requires_geometry) —
set that up, or send raw command JSON.

## 3. Stage A — server: unblock simulation-only Cartesian for controller-sim (only what A.0 proves blocked)

Introduce ONE shared predicate used at the three+ gate sites:

```
// true when Cartesian "simulation-only" primitives may run for this arm/run:
//   - RunMode::Simulation (rbsim), OR
//   - controller-simulation Cartesian gate open (operation_mode==simulation + existing env/config gate)
bool cartesianSimContextAllowed(run_mode, backend_cfg, dual_cfg /*+ env*/)
```

- Replace `if (run_mode != RunMode::Simulation) reject` at the TcpLinearMove / TcpTwist / TcpCircleMove
  sites with `if (!cartesianSimContextAllowed(...)) reject`. The controller-sim branch must require the
  SAME gate already enforced for `TcpPoseTarget` controller-sim (so no new env/flag; `cartesian_control.
  allow_in_controller_simulation` + `RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN` etc. already required).
- `TcpCircleMove`: allow when controller-sim Cartesian gate open AND `circle_move.allow_in_simulation`;
  keep `allow_in_real=false` enforced. Reuse `enable_benchmark_primitives`.
- Keep all downstream safety (IK success/budget, velocity/step limits, tcp pose validity, tracking-error
  fault) UNCHANGED — they already apply in both Simulation and controller-sim.
- Config validation: ensure no new throw for `operation_mode==real`; the relaxation is purely the
  run_mode→sim-context substitution behind the controller-sim gate.
- Tests: for each primitive, (i) accepted in controller-sim with the gate open, (ii) still rejected in
  controller-sim with the gate closed, (iii) real mode (`operation_mode==real`) behavior unchanged,
  (iv) rbsim `RunMode::Simulation` unchanged.
- Live acceptance: with `vm_dual_cartesian.yaml` + full controller-sim cartesian env, drive
  TcpLinearMove / TcpTwistLocal / TcpCircleMove via policy_runner (tcp_delta/twist) or rb_gui and observe
  TCP/q_ref tracking; physical q_actual stays 0.

## 4. Stage B — rb_gui: full operator for pgmode-sim rbpodo

- `safety.py`: allow TCP/Cartesian commands when `observed_backend == "rbpodo"` AND
  `observed_server_mode == "simulation"` (operation_mode), behind a gui opt-in
  (e.g. `RB_GUI_ENABLE_CONTROLLER_SIM_CARTESIAN=1`; keep `RB_GUI_ENABLE_TCP_POSE_COMMANDS` semantics).
  Do NOT remove the `observed/desired == real` motion block.
- Add operator controls + command_client builders for `TcpTwistLocal/Stand` (deadman/lease-aware) and
  `JointVelocity`. Wire to the existing UDP command path (already used).
- Keep readiness/lease/deadman semantics; surface pgmode-sim status clearly (`physical_motion_expected=false`).
- Docs: update `rb_servo_server/docs/gui_operator_console.md` + `rb_gui/README.md` to state that pgmode
  simulation is fully operable from rb_gui while real stays status-only.
- Tests: extend `rb_gui/tests` to cover the new allow path (rbpodo + operation_mode=simulation) and the
  preserved real-mode block; new twist/velocity command building.
- Live acceptance: open rb_gui (http://127.0.0.1:8080) against the dual pgmode-sim server; jog joints,
  run TCP PTP/Linear/Twist; confirm motion via q_ref/TCP overlay; confirm real mode still disabled.

## 5. Sequencing & precedence

A before B (B depends on A for Cartesian to actually execute on the rbpodo path). Each stage is a separate
reviewable codex change + commit. Update `AGENTS.md`/`docs/architecture.md` only if the documented
"rb_gui keeps real motion disabled / simulator-only commands" wording needs to reflect the new pgmode-sim
operability (real stays disabled; this is an additive sim capability). Historical conservative wording is
amended, not the real-safety invariant.
