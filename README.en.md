# robotics_lab

`robotics_lab` is the integration workspace for a dual-arm RB3-730 system with servo control, a topology-isomorphic local simulator, camera capture, policy_runner, and an operator GUI.

## Current Phase

The project is currently in **rbpodo pgmode-real physical robot bring-up**.
Simulator-first Cartesian acceptance hardening is largely complete; validation
now proceeds on the physical RB3-730E hardware.

Simulator-side behavior that is repeatedly validated and stabilized:

- per-arm simulator topology
- structured backend result and fault telemetry
- `JointTarget` / `JointVelocity`
- `TcpPoseTarget`
- `TcpLinearMove`
- `TcpTwistLocal` / `TcpTwistStand`
- GUI operator controls
- policy_runner SpaceMouse path
- command-source lease/arbitration
- camera readiness contracts

What has additionally been validated on the physical robot is listed under
"Current Maturity" below. Real motion is still a fail-closed operation requiring
operator supervision, an E-stop in hand, and explicit gates; passing simulator
acceptance is not permission to move hardware.

## Current Maturity

Supported for mock/simulation:

- mock dual-arm servo control
- per-arm local simulator backend
- persistent simulator JSON-line transport
- simulator direct and worker I/O modes
- FK/TCP state publication with quaternion fields
- simulator-only TCP PTP, Linear, and Twist commands
- mock camera server
- GUI viewer/operator console for mock/simulation
- policy_runner joint and Cartesian simulator action sources
- simulator-only Cartesian acceptance scripts
- mandatory Eigen3/Pinocchio C++ Cartesian math path for `rb_servo_server`

Validated on pgmode-real (physical RB3-730E hardware):

- read-only physical diagnostics parity (controllers `.200`/`.201`, `tcp_actual_stand`)
- dual-arm physical Cartesian circle tracking — slow, TUNED-1 profile, median
  tracking ~1.42° (`docs/runbooks/rbpodo_real_physical_circle.md`)
- UMI dual-arm Cartesian teleop (relative-init) driving `TcpPoseTarget` on the real robot
- server-side URDF-capsule self-collision guard (`clamp_to_hold`) active during real motion
- policy-side real-Cartesian safety gate relaxation (PR #13) → `rb_servo_server`
  is the sole real-motion safety layer
- controller `-2001` (suspect diagnostics) accepted in real mode (PR #12);
  EMS/SOS/soft-estop/`collision_occur`/unknown-mode/init-error still latch

Not production-ready (unvalidated):

- force control (`provider: null`, `enable: false`)
- gripper control
- measured camera/robot calibration — currently `configured_estimate`, UMI frame gap unresolved
- real camera + policy + robot full closed-loop rollout (`flow-infer` `real_policy` lane still blocked)
- fast physical circle stages (15 cm / 16 s and above, transition ladder P7–P9)

## Source Of Truth

Start here:

- `AGENTS.md`: instructions for Codex/Claude/other agents
- `REVIEW.md`: current review baseline and open items
- `docs/current_review.md`: short redirect to `REVIEW.md`; do not duplicate review content there
- `docs/architecture.md`: system topology, terminology, motion primitive contract, safety boundaries
- `docs/code_architecture_map.md`: code-verified component map, ports/wire-formats, and a doc-vs-code drift list
- `docs/servo_backend_contract.md`: backend result, fault, worker I/O, and state telemetry contract
- `docs/frame_contract.md`: shared frames and calibration status
- `docs/hardware_free_validation.md`: hardware-free validation boundary
- `docs/runbooks/tcp_pose_simulator_acceptance.md`: Cartesian simulator acceptance
- `docs/runbooks/camera_acceptance.md`: real three-camera acceptance
- `calibration/active_calibration.yaml`: configured-estimate robot/camera/stand setup registry

Historical prompt/planning files are audit context. When they conflict with the files above, the files above win.

## Canonical Terms

```yaml
run_mode: mock | simulation | real
backend_type: mock | simulator | rbpodo
```

## Real And Simulator Topology

Physical system:

```text
rb_servo_server
  left_robot  backend_type=rbpodo -> 172.28.60.200
  right_robot backend_type=rbpodo -> 172.28.60.201
```

Simulator:

```text
rb_servo_server
  left_robot  backend_type=simulator -> rb_simulator_left
  right_robot backend_type=simulator -> rb_simulator_right
```

The simulator mirrors one-controller-per-arm topology. It must not reuse real robot IPs as defaults.

## Safety Gates

Real robot connection:

```bash
RB_ALLOW_REAL_ROBOT=1
```

Real joint servo motion:

```bash
RB_ALLOW_REAL_MOTION=1
```

Real Cartesian/TCP motion:

```bash
RB_ALLOW_REAL_CARTESIAN=1
```

Accepting the controller `-2001` suspect diagnostics in real mode additionally requires:

```bash
RB_ALLOW_RBPODO_SUSPECT_DIAGNOSTICS_REAL_MOTION=1
```

These gates are necessary but not sufficient. Config
(`cartesian_control.allow_in_real: true`) and operator supervision must also
allow the operation. Through these gates a dual-arm physical Cartesian circle has
already run under supervision (`docs/runbooks/rbpodo_real_physical_circle.md`).
The policy-side `SafetyGate` real-Cartesian block was relaxed in PR #13, so for
real motion `rb_servo_server` is the sole safety layer (safety filter,
tracking-error latch, URDF-capsule self-collision guard, lease, deadman);
controller-simulation safety is unchanged. EMS/SOS/soft-estop/`collision_occur`/
unknown-mode/init-error still latch regardless of these gates.

Force control remains inactive:

```yaml
force_control:
  provider: null
  enable: false
```

## Motion Primitive Summary

- `TcpPoseTarget`: PTP / MoveJ-like Cartesian final-pose target; path not guaranteed. Real mode opens via the gates + `cartesian_control.allow_in_real: true` and has been validated on a dual-arm physical circle.
- `TcpLinearMove`: simulator-only MoveL-like Cartesian path primitive.
- `TcpTwistLocal` / `TcpTwistStand`: streaming Cartesian velocity primitives (simulator and the rbpodo controller-simulation carve-out; real Cartesian uses the gated `allow_in_real` path).
- `TcpDeltaLocal` / `TcpDeltaStand`: low-level one-shot/debug jog primitives.

## Common Commands

Python checks:

```bash
python3 -m unittest discover rb_gui/tests
python3 -m unittest discover policy_runner/tests
PYTHONPATH=rb_simulator/src python3 -m unittest discover rb_simulator/tests
```

Hardware-free gate:

```bash
./scripts/codex_gate.sh HARDEN-10
```

`rb_servo_server` C++ builds require Eigen3 and Pinocchio. Cartesian FK/IK,
orientation interpolation, frame conversion, and SE(3) delta math delegate to
Eigen/Pinocchio. Missing Pinocchio is a missing C++ dependency, not a fallback
runtime mode.

Cartesian math rebaseline:

```bash
./scripts/codex_gate.sh CART-MATH-03
```

Cartesian simulator acceptance:

```bash
CODEX_RUN_CARTESIAN_ACCEPTANCE=1 ./scripts/codex_gate.sh CART-HARDEN-05
```

Start simulator operator stack:

```bash
make sim-up
```

Open:

```text
http://127.0.0.1:8080
```

## Canonical Configs

Servo server simulation configs:

- `rb_servo_server/config/dual_simulator.yaml`
- `rb_servo_server/config/dual_simulator_compose.yaml`
- `rb_servo_server/config/dual_simulator_worker.yaml`
- `rb_servo_server/config/dual_simulator_tcp_acceptance.yaml`

Simulator configs:

- `rb_simulator/config/left_rb3_730e.yaml`
- `rb_simulator/config/right_rb3_730e.yaml`
- `rb_simulator/config/left_rb3_730e_compose.yaml`
- `rb_simulator/config/right_rb3_730e_compose.yaml`

Real robot template:

- `rb_servo_server/config/dual_real.example.yaml`

Site-local real configs:

- `rb_servo_server/config/local/dual_real_readonly.yaml`
- `rb_servo_server/config/local/dual_real_motion.yaml`

No tracked runnable real robot config should be added.

Deprecated simulator config names are archived under `docs/archive/configs/`
for historical reference only. They are not runnable source-of-truth configs
and must not be used for new smoke or acceptance evidence. `README.md` is the
canonical root README; this English README is a best-effort translation.
