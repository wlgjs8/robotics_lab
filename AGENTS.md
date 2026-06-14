# AGENTS.md

## Current Project Phase

`robotics_lab` is a dual-arm RB3-730 integration workspace. The current milestone is **rbpodo pgmode-real physical robot bring-up**. Simulator-first Cartesian acceptance hardening is largely complete and is now the regression baseline.

The simulator stack remains the regression baseline (it must keep passing before any physical work):

- per-arm simulator topology
- structured backend result and fault telemetry
- joint commands: `JointTarget`, `JointVelocity`
- Cartesian point-to-point: `TcpPoseTarget`
- Cartesian path tracking: `TcpLinearMove`
- Cartesian streaming velocity: `TcpTwistLocal`, `TcpTwistStand`
- GUI and policy-runner safety gates
- command-source lease/arbitration

Real motion is now an active, gated bring-up lane: read-only diagnostics parity, a slow dual-arm physical Cartesian circle, UMI teleop/replay, and a full `flow-infer` `real_policy` closed-loop rollout (pi0.5/openpi, `TcpTwistLocal` + real gripper) have all run on hardware under operator supervision (`docs/runbooks/rbpodo_real_physical_circle.md`, ladder `docs/runbooks/pgmode_real_transition.md`). The `real_policy` gate stays fully enforced and was satisfied via accepted/validated config — the lane is open, not blocked; runtime is validated and task success is the remaining model-side gap. Real motion stays fail-closed — gates, site-local config, operator supervision, and an E-stop are all required — and passing simulator tests is never permission to move hardware. For real motion the policy-side gate was relaxed (PR #13), so `rb_servo_server` is the sole real-motion safety layer (plus the async URDF-mesh `CollisionMonitor`). Still off: force control; measured hand-eye calibration is unneeded for the deployed pika ee_local image-conditioned policy but still required for general geometry-dependent policy.

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
- `docs/runbooks/tcp_pose_simulator_acceptance.md`
- the component README/docs for the module being changed

Historical prompt/planning files such as `TODO.md`, `CODEX_*PROMPTS*.md`, `MIG-*`, `HARDEN-*`, and `CART-HARDEN-*` notes are audit context only unless a task explicitly names them. When those files conflict with the current source-of-truth docs, the current source-of-truth docs win.

## Canonical Public Terminology

Use these values in config, docs, GUI labels, logs, and tests:

```yaml
run_mode: mock | simulation | real
backend_type: mock | simulator | rbpodo
```

Do not introduce new public terms such as `rbsim_local`, public `rbsim`, or mixed simulator aliases. Legacy filenames may exist for compatibility, but new work should use `simulator`.

Supported real-controller scope is rbpodo only. Mock and simulator backends
remain for hardware-free validation; raw script TCP comparison backends are no
longer part of the active code, config, gate, or runbook surface.

## Target Topology

Physical robot topology:

```text
rb_servo_server
  left_robot  backend_type=rbpodo -> 172.28.60.200
  right_robot backend_type=rbpodo -> 172.28.60.201
```

Simulator topology mirrors the controller shape, not the physical IP addresses:

```text
rb_servo_server
  left_robot  backend_type=simulator -> rb_simulator_left
  right_robot backend_type=simulator -> rb_simulator_right
```

Do not make simulator configs use `172.28.60.200` or `172.28.60.201` as defaults.

## Hard Safety Rules

Never enable real robot behavior implicitly.

Real robot connection requires:

```bash
RB_ALLOW_REAL_ROBOT=1
```

Real joint servo motion requires:

```bash
RB_ALLOW_REAL_MOTION=1
```

Real rbpodo Servo J motion with controller ACK waiting disabled additionally requires:

```bash
RB_ALLOW_RBPODO_ACK_DISABLED_MOTION=1
```

Real Cartesian/TCP motion requires:

```bash
RB_ALLOW_REAL_CARTESIAN=1
```

Even with these environment variables, real motion must also be explicitly allowed by config and by the relevant real-hardware acceptance task. Simulator acceptance is not real-hardware acceptance.

The stand-frame floor plane constraint (`safety.floor_constraint`) is
mode-independent by design: when enabled it applies in mock, simulator,
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

Tracked real robot config must remain a template only:

```text
rb_servo_server/config/dual_real.example.yaml
```

Site-specific real configs belong under:

```text
rb_servo_server/config/local/
```

Do not add a tracked runnable real robot config.

## Force Control

Force/admittance/impedance control must remain inactive.

```yaml
force_control:
  provider: null
  enable: false
```

Do not integrate `mo_forcecontroller` or any other force-control library into an active motion path unless a future task explicitly approves it and defines a safety acceptance plan.

## Motion Primitive Contract

Do not blur these modes.

### JointTarget

Absolute joint-space target. This is a joint-space point-to-point primitive.

### JointVelocity

Streaming joint velocity command. This is suitable for low-level joint teleop/debug when safety gates allow it.

### TcpPoseTarget

Cartesian point-to-point final-pose target. It is MoveJ-like in the sense that the final TCP pose is targeted, but the intermediate TCP path is not guaranteed linear and TCP orientation may vary along the joint-space path.

### TcpLinearMove

Simulator-only MoveL-like Cartesian path primitive. It must have explicit timing/speed semantics and orientation interpolation semantics. It is not real-motion-ready until a future real-hardware acceptance task says so.

### TcpTwistLocal / TcpTwistStand

Streaming Cartesian velocity primitives. `TcpTwistLocal` is intended for SpaceMouse/local-frame continuous teleop. `TcpTwistStand` is the stand-frame low-level API. Deadman behavior, command-source arbitration, and server-side velocity limits are required for operator control.

### TcpDeltaLocal / TcpDeltaStand

Low-level one-shot/debug jog primitives. They are not the default GUI target movement primitive. GUI target movement should normally update the target marker and send an absolute `TcpPoseTarget` or `TcpLinearMove`.

## Backend Contract

Do not reintroduce bool-only backend operations.

Backend APIs must preserve structured results:

- `BackendResult<RobotState>`
- `SendServoJResult`
- `BackendErrorKind`
- `BackendTiming`
- `FaultContext`

Do not parse backend error strings to infer safety behavior if structured fields are available. Simulator, mock, and rbpodo backends should all map failures to the shared backend taxonomy.

Unsupported raw script TCP comparison paths must not be reintroduced. Rbpodo is
the only supported real backend; mock and simulator paths remain hardware-free
test surfaces.

## Servo Loop And I/O Architecture

Target architecture:

```text
CommandBuffer
  -> ServoCoordinator / DualArmServoLoop
       -> Left ArmWorker  -> one left backend/controller endpoint
       -> Right ArmWorker -> one right backend/controller endpoint
```

The servo loop owns command freshness, lifecycle, target generation, FK/IK, Cartesian planning, safety filtering, fault classification, and dual-arm aggregation. Blocking backend I/O should live behind backend/worker boundaries and must produce structured result telemetry.

`servo.io_model: worker` is simulator-only unless a future real-hardware acceptance task explicitly opens it.

## Calibration And Frames

Use `docs/frame_contract.md` and `calibration/active_calibration.yaml` as the frame/geometry source of truth. Current calibration is `configured_estimate`, not measured calibration. Joint-only control can run without measured calibration. Real geometry-dependent policy and real Cartesian camera-driven behavior require measured and accepted calibration.

## Development Rules

- Work only on the assigned task.
- Prefer small, reviewable changes.
- Keep configs strict and fail-closed.
- Update tests, docs, and acceptance scripts together with behavior changes.
- Do not fake external APIs for rbpodo, Pinocchio, RealSense, SpaceMouse, or camera devices.
- If a dependency is missing, report it clearly and do not claim the gate passed.
- Do not claim C++ or Pinocchio runtime acceptance passed unless the command was actually run.
- Do not introduce custom SO(3), SE(3), quaternion interpolation, or frame-conversion math in production Cartesian control when Eigen/Pinocchio can provide it.
- Do not create production fallback math paths that bypass mandatory Eigen/Pinocchio Cartesian math.
- Do not weaken command-source lease, deadman, stale-state, fault, or real-mode checks. Real motion now relies on `rb_servo_server` as its sole safety layer (safety filter, tracking-error latch, self-collision guard, lease, deadman) — treat these as load-bearing, not optional.
- Real motion is an explicit, operator-supervised, gated lane — never enable it incidentally as part of simulator/benchmark work, and keep grippers and force control off until separately validated.

## Expected Validation

For Python and simulator-facing changes, run as applicable:

```bash
python3 -m unittest discover rb_gui/tests
python3 -m unittest discover policy_runner/tests
PYTHONPATH=rb_simulator/src python3 -m unittest discover rb_simulator/tests
python3 -m compileall -q rb_gui/rb_servo_gui policy_runner/policy_runner rb_simulator/src scripts
```

For C++ servo changes, run the hardware-free C++ gate when dependencies are installed:

```bash
./scripts/codex_gate.sh HARDEN-10
```

For Cartesian behavior, run simulator acceptance when Pinocchio and C++ deps are installed:

```bash
CODEX_RUN_CARTESIAN_ACCEPTANCE=1 ./scripts/codex_gate.sh CART-HARDEN-05
```

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
