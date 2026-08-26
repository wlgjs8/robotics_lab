# AGENTS.md

## Current Project Phase

`robotics_lab` is a dual-arm RB3-730 integration workspace. The current milestone is **rbpodo pgmode-real physical robot bring-up**. Simulator-first Cartesian acceptance hardening is largely complete and is now the regression baseline.

The mock / rbpodo controller-simulation (pgmode) stack remains the regression baseline (it must keep passing before any physical work):

- structured backend result and fault telemetry
- joint commands: `JointTarget`
- Cartesian point-to-point: `TcpPoseTarget`
- Cartesian path tracking: `TcpLinearMove`
- GUI and policy-runner safety gates
- command-source lease/arbitration

Real motion is now an active bring-up lane: read-only diagnostics parity, a slow dual-arm physical Cartesian circle, UMI teleop/replay, and a full `flow-infer` `real_policy` closed-loop rollout (pi0.5/openpi, `TcpPoseTarget` + real gripper) have all run on hardware under operator supervision (`docs/runbooks/rbpodo_real_physical_circle.md`, ladder `docs/runbooks/pgmode_real_transition.md`). `flow-infer` composes ee_local deltas into absolute `TcpPoseTarget` setpoints. The `real_policy` rollout-mode validation was satisfied via accepted/validated config — the lane is open, not blocked; runtime is validated and task success is the remaining model-side gap. Real-motion execution authority is config-driven and server-owned: tracked stack config plus the mode-independent safety layers decide whether motion is sent. Operator supervision and an E-stop remain physical operation procedure, and passing simulator tests is never permission to move hardware. For real Cartesian motion the policy-side block was retired (PR #13), so `rb_servo_server` makes the final allow/deny decision (plus the async URDF-mesh `CollisionMonitor`). Project-native force control v2 is live and hardware-validated against `controller-manager`; its measured configuration, coverage/tare preconditions, gate, spring, and deviation fences remain load-bearing safety contracts. Measured hand-eye calibration is unneeded for the deployed pika ee_local image-conditioned policy but still required for general geometry-dependent policy.

## Required Reading

Always read the current source-of-truth docs before editing code:

- `README.md`
- `REVIEW.md`, if present
- `docs/architecture.md`
- `docs/current_review.md`, if present
- `docs/servo_backend_contract.md`
- `docs/frame_contract.md`
- `docs/joint_range_policy.md`
- `docs/hardware_free_validation.md`
- the component README/docs for the module being changed

Historical prompt/planning files such as `TODO.md`, `CODEX_*PROMPTS*.md`, `MIG-*`, `HARDEN-*`, and `CART-HARDEN-*` notes are audit context only unless a task explicitly names them. When those files conflict with the current source-of-truth docs, the current source-of-truth docs win.

## Canonical Public Terminology

Use these values in config, docs, GUI labels, logs, and tests:

```yaml
run_mode: mock | simulation | real
backend_type: mock | rbpodo
```

Do not introduce removed simulator backend aliases or mixed simulator terms.
`run_mode: simulation` now refers only to the rbpodo controller `pgmode`
simulation flavor.

Supported real-controller scope is rbpodo only. The `MockBackend` remains for
hardware-free validation; the retired software-simulator backend and raw script
TCP comparison backends are no longer part of the active code, config, gate, or
runbook surface.

The supported J3/elbow range is exactly `[-150 deg, +150 deg]`, matching the
Rainbow RB3-730E documentation and the Pinocchio URDF. Tracked safety limits,
joint-limit barriers, IK, examples, and runbooks must use that same range. Do
not restore the retired `+/-160 deg` margin or widen J3 to hide an unreachable
Cartesian target.

## Target Topology

Physical robot topology:

```text
rb_servo_server
  left_robot  backend_type=rbpodo -> 172.28.60.200
  right_robot backend_type=rbpodo -> 172.28.60.201
```

The rbpodo controller `pgmode` simulation topology mirrors this controller shape
(one rbpodo endpoint per arm), targeting either a Virtual ControlBox VM or a
physical box held in `pgmode`. The tracked `stack_real.yaml` and
`stack_sim.yaml` files are the only launch configs.

## Safety Boundary

Never enable real robot behavior implicitly. Real behavior is fail-closed, but
it is **no longer gated on env vars**: the legacy execution gates
(`RB_ALLOW_REAL_ROBOT`, `RB_ALLOW_REAL_MOTION`, `RB_ALLOW_REAL_CARTESIAN`,
`RB_ALLOW_RBPODO_ACK_DISABLED_MOTION`, and the other `RB_ALLOW_*`) were removed
from the server runtime. `run_mode`/`operation_mode` are telemetry labels only
and do not decide whether motion is allowed.

Real-motion execution authority is owned by **the tracked stack config + the
mode-independent safety layers**. Real motion requires `stack_real.yaml` to
enable it explicitly (e.g.
`cartesian_control.allow_in_real: true`). Real-hardware acceptance and operator
supervision are physical operation process, not extra software gates. The
controller `-2001` suspect-diagnostics acceptance and the rbpodo `pgmode`
controller-simulation carve-out are likewise config opt-ins (no env). Simulator
acceptance is not real-hardware acceptance.

The stand-frame floor plane constraint (`safety.floor_constraint`) is
mode-independent by design: when enabled it applies in mock,
controller-simulation, and real, to every motion primitive, at the final
joint-level safety gate. Enabling it requires `kinematics.enable=true`.
Runtime adjustment uses the leaseless `SetSafetyFloorZ` command and is
bounded server-side to the config `[runtime_min_z_m, runtime_max_z_m]`
envelope; `monitor_only: true` is a tuning aid only and never a real-motion
safety posture. Do not add env/mode gates that would disable it in real mode.

For new rbpodo configs, use canonical Rainbow Servo J fields only:
`servo_t1_sec`, `servo_t2_sec`, `servo_gain`, and `servo_alpha`.
Do not add new uses of deprecated aliases `servo_time_sec`,
`servo_lookahead_sec`, or `servo_acc`. `servo_t1_sec` must match the streaming
command period for supported real/controller-simulation configs: `0.002` at
500 Hz. Manual non-500 YAML overrides may remain parseable for compatibility,
but they are not supported profiles. ACK-off rbpodo settings are diagnostic
evidence only until a future real-motion task promotes them.

Tracked stack configs are the launch source of truth:

```text
rb_servo_server/config/stack_real.yaml
rb_servo_server/config/stack_sim.yaml
```

Do not create `config/local` launch variants. Change one reviewed setting at a
time in the appropriate tracked stack config so the effective runtime profile
remains visible and auditable.

## Force Control

The v1 stack was removed on 2026-08-26. A v2 was then rebuilt against
`controller-manager` as the calibration and design authority, starting from
sensor and tool setup, and is LIVE: `force_torque:` and `force_control:` are
server config sections again, both are declared in `stack_real.yaml`, and the
overlay has been validated on hardware (2026-08-26: deviation tracked F/k to
0.97-0.99, rotation 1.85 deg at 55 N, zero fence hits).

CM is the reference. Sensor axes, tool mass/COM and the TCP offset come from
`submodules/controller-manager/platforms/monkey/params-presets/` and were
calibrated by the operator; do not re-derive them from the URDF. The sensor basis
on this cell is LEFT-HANDED (det = -1) -- that is measured, not a bug.

Two invariants the hardware taught, both enforced by the loader:
- THE GATE AND THE SPRING SHIP TOGETHER. `k > 0` with no gate ramps the contact
  force without bound (961 N in 40 s); the gate with `k = 0` bounds force but not
  deviation (9.5 m in 300 s).
- THE WRENCH REFERENCE POINT AND THE COMPOSE PIVOT MOVE TOGETHER. Both are the
  TCP. A torque referenced at one point driving rotation about another makes a
  straight push twist the tool.

### The zero (tare)

Force control REFUSES to cover an arm with no bias (`forceControlCovered`), so the
tare is a precondition, not a nicety. Two ways in, one mechanism: the leaseless
`TareForceSensor` command (the GUI button), and
`force_torque.auto_tare_after_init_motion`. Both end in the same RT path — 250
consecutive ticks of `raw - gravity` averaged by `FtPipeline::tareSample/tareCommit`.
It averages `raw - gravity`, NEVER `raw`: the box is told a zero payload, so raw
still carries the tool's weight and averaging it would subtract gravity twice.

**Automatic tare on InitMotion does not sample when the move starts — it ARMS when
the move starts.** A tare averaged while the arm is accelerating records the arm's
own acceleration and the tool's swing as force, and nothing downstream can tell that
apart from a real bias. So the request tick arms it (`armAutoTareAfterInit`, called
from `applyInitMotionSequencer`'s fresh-request branch) and the samples are collected
only after that arm's sequencer reaches `Done`, has stood `settle_sec` at the init
pose, and its last SENT joint velocity is under `max_sent_speed_deg_s`
(`stepAutoTareAfterInit`, whose per-tick decision is the stateless
`stepAutoTareDecision` — unit-tested in `test_init_motion_pursuit`). It runs BEFORE
`applyInitMotionSequencer` in the tick so the `Done` set on the previous tick is still
readable; the next non-init command resets the exec to `Idle`.

Fail-closed, all of it: `invalidate_on_request` drops the existing zero the instant
InitMotion is requested, so an InitMotion that fails, stalls, is cancelled, or is
overtaken by a latched fault leaves the arm with NO zero and force control refuses it,
rather than leaving a stale zero the operator believes was just refreshed. The loader
refuses `enable: true` without a positive `settle_sec` and `max_sent_speed_deg_s`, and
refuses it with `safety.init_motion_planner.enable: false` (InitMotion then degrades
to a plain JointTarget with no completion event, so the tare would silently never
fire). Stage is published as `force_torque.<arm>.auto_tare_stage` and logged as
`<side>_ft_auto_tare_stage`.

THE OPERATOR'S CHECK IS UNCHANGED AND IS NOT WEAKER BECAUSE IT IS AUTOMATIC: whatever
load stands at the init pose becomes the new zero. Neither the GUI nor the server can
see a part in the gripper or a hand on the wrist.

Do not integrate an external force-control library into an active motion path
unless a task explicitly says to. Archived v1 design and evidence:
`docs/archive/force_control_v1/`.

## Motion Primitive Contract

Do not blur these modes.

### JointTarget

Absolute joint-space target. This is a joint-space point-to-point primitive.

### TcpPoseTarget

Cartesian point-to-point final-pose target. It is MoveJ-like in the sense that the final TCP pose is targeted, but the intermediate TCP path is not guaranteed linear and TCP orientation may vary along the joint-space path.

### TcpLinearMove

MoveL-like Cartesian path primitive. It has explicit timing/speed semantics and orientation interpolation semantics (`constant`/`slerp`). Real-motion-ready and used on the physical arms (the run-mode execution gate was retired; it computes in every run mode). It is a finite, bounded path (`linear_move.max_duration_sec`): once started it drives to completion from a single command even if the command's freshness/lease lapses, so one click always reaches the target; an explicit command-mode change, fault, or E-stop aborts it, and the per-tick safety gate still applies.

## Backend Contract

Do not reintroduce bool-only backend operations.

Backend APIs must preserve structured results:

- `BackendResult<RobotState>`
- `SendServoJResult`
- `BackendErrorKind`
- `BackendTiming`
- `FaultContext`

Do not parse backend error strings to infer safety behavior if structured fields are available. Mock and rbpodo backends should both map failures to the shared backend taxonomy.

Unsupported raw script TCP comparison paths must not be reintroduced. Rbpodo is
the only supported real backend; the mock path remains the hardware-free test
surface.

## Servo Loop And I/O Architecture

Target architecture:

```text
CommandBuffer
  -> ServoCoordinator / DualArmServoLoop
       -> Left ArmWorker  -> one left backend/controller endpoint
       -> Right ArmWorker -> one right backend/controller endpoint
```

The servo loop owns command freshness, lifecycle, target generation, FK/IK, Cartesian planning, safety filtering, fault classification, and dual-arm aggregation. Blocking backend I/O should live behind backend/worker boundaries and must produce structured result telemetry.

`servo.io_model: worker` is a supported real-mode path. The mock-only refusal was
retired: control-box queue sync needs each arm to own its own send cadence, and a
single loop has one period for two boxes running two different clocks.

## Calibration And Frames

Use `docs/frame_contract.md` and `calibration/active_calibration.yaml` as the frame/geometry source of truth. Current calibration is `configured_estimate`, not measured calibration. Joint-only control can run without measured calibration. Real geometry-dependent policy and real Cartesian camera-driven behavior require measured and accepted calibration.

## Development Rules

- Work only on the assigned task.
- Prefer small, reviewable changes.
- Keep configs explicit about what they enable and what safety layer owns it.
- Update tests, docs, and acceptance scripts together with behavior changes.
- Do not fake external APIs for rbpodo, Pinocchio, RealSense, SpaceMouse, or camera devices.
- If a dependency is missing, report it clearly and do not claim the gate passed.
- Do not claim C++ or Pinocchio runtime acceptance passed unless the command was actually run.
- Do not introduce custom SO(3), SE(3), quaternion interpolation, or frame-conversion math in production Cartesian control when Eigen/Pinocchio can provide it.
- Do not create production fallback math paths that bypass mandatory Eigen/Pinocchio Cartesian math.
- No silent fallback defaults for safety-relevant parameters. A value that bounds motion, contact, a tolerance, a geometry/frame, or any other safety-affecting decision MUST come from its authoritative source (server config / contract / measured evidence). If that source is missing or unreadable, FAIL CLOSED — do not fire, do not move, surface the reason — instead of substituting a guessed/hard-coded default. A guessed value can be wrong in the unsafe direction (e.g. a tolerance that lets the server plan a move when the caller assumed a no-op). Prefer `None`/error + a logged reason over a plausible constant. This applies to every component (C++, GUI, policy_runner), not just Cartesian math.
- Do not weaken command-source lease, deadman, stale-state, fault, or real-mode checks. Real Cartesian motion now relies on `rb_servo_server` for the final allow/deny decision (safety filter, tracking-error latch, self-collision guard, lease, deadman) — treat these as load-bearing, not optional.
- Real motion is explicit and operator-supervised — never enable or retune it incidentally as part of simulator/benchmark work. Force control is live only through its tracked, hardware-validated v2 configuration and must not be disabled, enabled, or retuned as an unrelated change. Gripper motion remains separately gated by `allow_real_gripper_motion`, measured gripper availability, and `RB_ALLOW_REAL_GRIPPER=1`.

## Expected Validation

For Python changes, run as applicable:

```bash
python3 -m unittest discover rb_gui/tests
python3 -m unittest discover policy_runner/tests
python3 -m compileall -q rb_gui/rb_servo_gui policy_runner/policy_runner scripts
```

For C++ servo changes, build and run the hardware-free C++ tests when dependencies are installed:

```bash
cmake -S rb_servo_server -B rb_servo_server/build
cmake --build rb_servo_server/build -j
ctest --test-dir rb_servo_server/build --output-on-failure
```

For Cartesian behavior, use the Pinocchio-backed C++ tests plus active-stack
smoke/acceptance on mock when a local mock config is available, rbpodo
controller `pgmode` simulation / VM, and physical real only through the separate
supervised runbooks. The old software-simulator-oriented Cartesian acceptance
runner has been removed.

## Required Final Report

Every agent must report:

1. Summary
2. Files changed
3. Config/schema changes
4. Tests run
5. Test results
6. Skipped checks and why
7. Remaining TODOs
8. Safety implications
9. Whether real-mode behavior was touched
