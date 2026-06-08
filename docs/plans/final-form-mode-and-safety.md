# Plan: final-form run-mode model (remove RB_ALLOW_* env toggles)

Status: agreed design (decisions below). Implementation is PHASED and the real
part is gated on the safety subsystems existing first. Do NOT delete real env
gates before those exist.

## Goal

The final robotics_lab: operate via the viser GUI; a config `run_mode` selects
`simulation` (Rainbow VM / pgmode sim) or `real` (actual robot); you just run
`rb_servo_server` — the scattered `RB_ALLOW_*` / `RB_RBPODO_*` env toggles
DISAPPEAR. Removing them must make real **harder to trigger accidentally**, not
easier: the env tripwire is replaced by (config intent) + (mandatory validated
safety subsystems) + (one deliberate GUI "arm real" action).

## Agreed decisions

1. **Real authorization = config `run_mode:real` + mandatory validated safety
   subsystems + one deliberate GUI "arm real motion" action.** No `RB_ALLOW_REAL_*`
   env. The server REFUSES to run real unless the safety subsystems (below) are
   configured and pass a startup self-test (fail-closed moves from env →
   safety-subsystem presence). Real motion stays disarmed until the operator
   performs the explicit GUI arm action (deadman-style), every session.
2. **sim↔real switch = deliberate reconnect**, not a transparent runtime
   hot-switch. sim and real use different endpoints (VM 10.0.2.x vs real robot
   IPs); switching tears down the control loop, loads the real config, connects
   real endpoints, runs the safety self-test, and requires the arm action.
3. **Separate sim/real configs** (the GUI/launcher picks one), matching the
   existing `dual_real.example.yaml` → `config/local/*.yaml` pattern. Real IPs
   stay in gitignored `config/local/`.

## The 13 env gates → disposition

DELETE (derive from `operation_mode==simulation` + `backend_type==rbpodo`; safe,
sim-only): RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION, ..._CARTESIAN,
RB_RBPODO_PGMODE_SIMULATION_CONFIRMED, RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM,
..._INIT_ERROR_CONTROLLER_SIM, ..._NOT_ACTIVATED_CONTROLLER_SIM,
RB_ALLOW_RBPODO_ACK_DISABLED_MOTION, ..._ASYNC_STREAMING, ..._SOCKET_SEND_ONLY_STREAMING.
(These are inherent to "I am running a Virtual ControlBox sim". → config-derived.)

REPLACE (real tripwire → config + safety self-test + GUI arm): RB_ALLOW_REAL_ROBOT,
RB_ALLOW_REAL_MOTION, RB_ALLOW_REAL_CARTESIAN.

CONFIG FIELD (not env): RB_ALLOW_NETWORK_EXPOSURE → `network.allow_exposed_bind`.

## Mandatory safety subsystems (the new real fail-closed) — see [[robotics-lab-final-form-and-safety]]

Real run_mode requires these configured + passing a startup self-test, else the
server refuses real:
1. Safety planes + safety zone (3D region) — TCP stop/clamp; both arms confined.
2. Dual-arm self-collision avoidance (6 links + gripper, configurable mm margin).
3. Stand collision avoidance (margin).
4. Never-all-zeros / IK-fail guard (never move_servo_j to [0,0,0,0,0,0]).

## Phased implementation (ORDER MATTERS — safety first)

- **C-phase-1 (DONE — commit bd12a62):** controller-sim carve-out now derived
  from `operation_mode==simulation` + the `servo.allow_controller_simulation_*` /
  `cartesian_control.allow_in_controller_simulation` config flags; the 9
  controller-sim env checks deleted from source, tests, and the pgmode-sim
  launcher (CONTROLLER_SIM_MOTION/CARTESIAN, PGMODE_CONFIRMED, DIAGNOSTICS_SUSPECT,
  INIT_ERROR, NOT_ACTIVATED, ASYNC_STREAMING, SOCKET_SEND_ONLY, and ACK_DISABLED
  for the controller-sim path only). Plumbed two new BackendConfig flags
  (allow_controller_simulation_diagnostics_suspect / _init_error) from servo.* so
  the per-arm backend keeps granular consent. Real stays byte-identical:
  REAL_ROBOT/MOTION/CARTESIAN unchanged; genuine-real ACK-off still needs
  RB_ALLOW_RBPODO_ACK_DISABLED_MOTION at config-load AND runtime. Validated:
  ctest 15/15 + live dual-VM smoke with no controller-sim env (config validation
  passed, both arms connected, overrides activated from config flags). The
  launcher still exports REAL_ROBOT/MOTION (retired only in C-phase-2).
- **B (safety subsystems):** build + validate the 4 safety subsystems in sim.
  - **B.2 dual-arm self-collision — DONE (sim-validated).** Per-link capsule model
    (segment between consecutive FK chain points + per-bone radius), pairwise
    capsule clearance vs a configurable `safety.self_collision.margin_m`, hooked
    into the servo loop's `applySafety` on the final candidate targets. Fail policy
    `clamp_hold` (default, holds last sent targets) or `fault_latch`; fail-closed if
    link geometry is unavailable; requires `kinematics.enable`. Motivated by the
    RB3-730 firmware not populating `op_stat_self_collision`. New: `math/capsule_distance.hpp`,
    `control/self_collision.hpp`, `safety.self_collision` config, `SafetyVerdict::SelfCollision`,
    state telemetry `self_collision{enabled,checked,violated,min_clearance_m}`,
    `IKinematics::linkCollisionPointsInStand`. Tests: capsule_distance, self_collision,
    kinematics_fk (link points), config_loader (self_collision). Live mock-sim
    validation: clearance tracks geometry; guard clamps when clearance < margin.
    TODO: subsystems 1/3/4 (safety planes+zone, stand collision, all-zeros/IK-fail
    guard); clamp_hold currently blocks all motion while violating (no escape
    direction yet); tune real link radii.
- **C-phase-2 (after B):** replace the real env gates with config `run_mode:real`
  + the mandatory safety self-test + the GUI arm action. Update all real configs,
  the GUI mode switch, docs (AGENTS.md / architecture / gui_operator_console),
  and the `pgmode_sim.env` launcher (drop the env exports).

## Hard invariants
- Never make real EASIER to trigger than today. During C-phase-1, real behavior
  is byte-identical (only sim env derivation changes).
- C-phase-2 must keep real fail-closed: no safety subsystems → no real.
- RB_ALLOW_REAL_CARTESIAN (and the real tripwire) only retire when replaced by a
  strictly-stronger gate; never just deleted.
- Keep canonical terminology (run_mode/operation_mode/backend_type).
