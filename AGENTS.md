# AGENTS.md

## Current Project Phase

`robotics_lab` is a dual-arm RB3-730 integration workspace. The current milestone is **simulator-first Cartesian acceptance hardening**.

Before any real robot work, the simulator stack must repeatedly validate:

- per-arm simulator topology
- structured backend result and fault telemetry
- joint commands: `JointTarget`, `JointVelocity`
- Cartesian point-to-point: `TcpPoseTarget`
- Cartesian path tracking: `TcpLinearMove`
- Cartesian streaming velocity: `TcpTwistLocal`, `TcpTwistStand`
- GUI and policy-runner safety gates
- command-source lease/arbitration

Real robot work is not the current default milestone. Passing simulator tests is not permission to move hardware.

## Required Reading

Always read the current source-of-truth docs before editing code:

- `README.md`
- `REVIEW.md`, if present
- `docs/architecture.md`
- `docs/current_review.md`, if present
- `docs/servo_backend_contract.md`
- `docs/frame_contract.md`
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

Real Cartesian/TCP motion requires:

```bash
RB_ALLOW_REAL_CARTESIAN=1
```

Even with these environment variables, real motion must also be explicitly allowed by config and by the relevant real-hardware acceptance task. Simulator acceptance is not real-hardware acceptance.

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
- Do not weaken command-source lease, deadman, stale-state, fault, or real-mode checks.
- Do not enable real robot motion, real Cartesian motion, grippers, or force control as part of simulator hardening.

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
