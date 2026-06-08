# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Source of Truth

`robotics_lab` is a dual-arm RB3-730 integration workspace. Before editing, read the current source-of-truth docs (they win over any historical prompt/planning file):

- `AGENTS.md` — agent working rules (mirrors much of this file in more detail)
- `README.md` (Korean; `README.en.md` is the preserved English original)
- `REVIEW.md` — current review baseline and open items
- `docs/architecture.md` — system topology, terminology, motion-primitive contract, safety boundaries
- `docs/servo_backend_contract.md` — backend result / fault / worker-I/O / state-telemetry contract
- `docs/frame_contract.md` and `calibration/active_calibration.yaml` — frame/geometry source of truth
- `docs/joint_range_policy.md` — rbpodo raw joint angle/range policy
- `docs/hardware_free_validation.md` — hardware-free validation boundary
- `docs/code_architecture_map.md` — code-verified component map, ports/wire-formats, and a doc-vs-code drift list
- the component README/docs for whatever module you change

Historical files (`TODO.md`, `CODEX_*PROMPTS*`, `MIG-*`, `HARDEN-*`, `CART-HARDEN-*`, `docs/archive/**`) are audit context only. `GOAL.md` and `REVIEW.md` are point-in-time snapshots, not direction — see the drift notes below.

## Current Phase

The repo is in **simulator-first Cartesian acceptance hardening**. Real robot motion is NOT the current milestone, and passing simulator acceptance is never permission to move hardware. Do not enable real motion, real Cartesian, grippers, or force control as part of simulator work.

## Architecture

Five cooperating services (real and simulator topology are isomorphic by endpoint count, not by IP):

```
                         policy_runner (Python action sources, SpaceMouse, flow ML)
rb_gui (viewer/operator) ─┐                       │
                          ▼                        ▼  UDP JSON commands
                     rb_servo_server (C++ control loop) ──► left/right backend endpoint
                          ▲  UDP state fanout
camera_server (C++) ──────┘ (RealSense/mock, shared-memory image transport)
```

- **`rb_servo_server`** (C++17) is the control owner. `DualArmServoLoop` owns command freshness, command-source lease/arbitration, FK/IK and Cartesian target generation, safety filtering, fault latching, dual-arm aggregation, and state publication. Per-arm blocking backend I/O lives behind `ArmWorker` → `IRobotBackend` (`MockBackend`, `RbsimBackend`, `RbpodoBackend`). Cartesian math (FK/IK, SO(3)/SE(3), orientation interpolation, frame conversion) MUST use Eigen3/Pinocchio — no local fallback math paths. Key dirs: `src/control/`, `src/robot/`, `src/kinematics/`, `src/math/`, `include/rb_servo/`.
- **`rb_simulator`** (Python, `src/rbsim/`) is one local controller endpoint per arm over a persistent JSON-lines TCP transport. Run with `PYTHONPATH=rb_simulator/src`.
- **`rb_gui`** (Python, `rb_servo_gui/`) is a mock/simulation viewer/operator console. It may send simulator-only TCP PTP/Linear commands; it must keep real motion disabled.
- **`policy_runner`** (Python package) owns action sources (`action_sources/`), recording, HDF5 audit, and flow-matching ML training/inference (`flow_*.py`). SpaceMouse Cartesian uses `TcpTwistLocal`, not repeated TCP deltas.
- **`camera_server`** (C++) owns capture, shared-memory ring-buffer transport, metadata, and health. Camera acceptance is separate from robot motion acceptance.

State fanout: `rb_servo_server` is the sole owner of UDP state publication via `network.state_pub_endpoints` (list). Commands go directly to `network.command_bind`. Benchmark overlay streams (desired geometry/metrics) are separate from robot state and must never carry commands.

Ports/protocols (verified against compose + config; full table in `docs/code_architecture_map.md`): command in `UDP 50010`; state out fanout to `UDP 50110` (gui) + `UDP 50120` (policy); server↔simulator `TCP 50200` (control) / `50201` (admin) per arm with JSON-lines schema `rbsim.v1`; camera metadata `ZMQ 5600` (`camera.bundle`/`camera.health`) with images in a POSIX shared-memory ring (`/camera_server_frames`); GUI web `HTTP 8080`; optional circle overlay `UDP 50261`. Command JSON `{seq, mode, left{…}, right{…}}` (`mode` parsed by `controlModeFromString` in `src/core/types.cpp`); state JSON schema `robotics_lab.servo_state.v1`.

## Canonical Terminology (use everywhere — config, docs, GUI, logs, tests)

```yaml
run_mode: mock | simulation | real
backend_type: mock | simulator | rbpodo
```

`rbpodo` is the only supported real-controller backend. Do not introduce `rbsim_local`, public `rbsim`, mixed simulator aliases, or raw-script TCP comparison backends.

## Safety Rules (do not weaken)

Real behavior is fail-closed and never implicit. Env gates are necessary but NOT sufficient — config and an explicit acceptance task must also allow the operation:

- `RB_ALLOW_REAL_ROBOT=1` — real controller connection
- `RB_ALLOW_REAL_MOTION=1` — real joint servo motion
- `RB_ALLOW_REAL_CARTESIAN=1` — real Cartesian/TCP motion
- `RB_ALLOW_RBPODO_ACK_DISABLED_MOTION=1` — Servo J with ACK waiting off (send evidence, not acceptance)
- rbpodo controller `pgmode` simulation carve-out (connects to real boxes but `operation_mode: simulation`, `physical_motion_expected=false`) additionally needs `RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1`, `RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1`, `RB_RBPODO_PGMODE_SIMULATION_CONFIRMED=1`. This is NOT `RB_ALLOW_REAL_CARTESIAN`.

Other invariants: never reintroduce bool-only backend results (preserve `BackendResult<RobotState>`, `SendServoJResult`, `BackendErrorKind`, `BackendTiming`, `FaultContext`); don't parse error strings when structured fields exist; force control stays `provider: null, enable: false`; `servo.io_model: worker` is simulator-only. Tracked real config is a template only (`rb_servo_server/config/dual_real.example.yaml`); site configs go in gitignored `rb_servo_server/config/local/`. New rbpodo configs use canonical Servo J fields `servo_t1_sec` / `servo_t2_sec` / `servo_gain` / `servo_alpha` (not `servo_time_sec` / `servo_lookahead_sec` / `servo_acc`); `servo_t1_sec: 0.002` at the supported 500 Hz.

## Motion Primitives (don't blur)

`JointTarget` (absolute joint PTP) · `JointVelocity` (streaming joint vel) · `TcpPoseTarget` (Cartesian final-pose PTP, path not guaranteed linear) · `TcpLinearMove` (simulator-only MoveL with `constant`/`slerp` orientation modes) · `TcpTwistLocal`/`TcpTwistStand` (streaming Cartesian velocity; need deadman, lease arbitration, server-side velocity limits) · `TcpDeltaLocal`/`TcpDeltaStand` (low-level debug jog, not the default GUI move). `TcpCircleMove` is an optional benchmark primitive; `TcpCircleTrack` is a disabled, not-implemented skeleton.

## Commands

Python tests (per component):
```bash
python3 -m unittest discover rb_gui/tests
python3 -m unittest discover policy_runner/tests
PYTHONPATH=rb_simulator/src python3 -m unittest discover rb_simulator/tests
python3 -m compileall -q rb_gui/rb_servo_gui policy_runner/policy_runner rb_simulator/src scripts
```
Run a single Python test: `python3 -m unittest policy_runner.tests.test_geometry_safety` (or `... -k <name>`).

C++ servo server build (requires Eigen3 + Pinocchio):
```bash
cmake -S rb_servo_server -B rb_servo_server/build
cmake --build rb_servo_server/build -j
ctest --test-dir rb_servo_server/build --output-on-failure
```

Gates (`scripts/codex_gate.sh <TASK>`) wrap build/test/acceptance:
```bash
./scripts/codex_gate.sh HARDEN-10                                  # hardware-free C++ gate
./scripts/codex_gate.sh CART-MATH-03                               # Cartesian math rebaseline
CODEX_RUN_CARTESIAN_ACCEPTANCE=1 ./scripts/codex_gate.sh CART-HARDEN-05   # Cartesian sim acceptance
./scripts/codex_gate.sh BENCH-CIRCLE-01                            # circle tracking benchmark
make sim-smoke                                                     # ./scripts/hardware_free_validation.sh
```

Docker/operator stacks (via `Makefile`):
```bash
make sim-local-up      # same-PC: GUI + servo server + per-arm simulators + passive recorder (alias: sim-up)
make sim-backend-up    # split-PC: simulators only (on the sim PC)
make sim-control-up    # split-PC: GUI + servo server (on the control PC)
make sim-teleop-up     # SpaceMouse teleop + recording
make policy-train      # imitation training
make sim-infer-up      # policy inference
make camera-mock-up / make camera-real-up
```
GUI: `http://127.0.0.1:8080`. Flow-matching multi-GPU training uses `docker-compose.flow-train.yml` (`make policy-flow-train-*`).

ML data flow: audit HDF5 episodes (`python3 -m policy_runner hdf5-audit ...`, schema `robotics_lab.policy_runner.hdf5_audit.v1`) before flow-train. `flow-infer` requires explicit `--rollout-mode` (`offline_eval` / `sim_dryrun` / `controller_sim` / `real_readonly` / `real_policy`); `real_policy` stays blocked without measured/accepted retarget, collision, gripper, and geometry gates.

## Doc Map & Drift (read before trusting a doc)

Scattered docs don't all track the latest direction. Key gotchas (full list in `docs/code_architecture_map.md`):

- **`GOAL.md` is not the project goal** — it's the verbatim text of one task prompt (`ACKON500-GENE-GOAL-01`, a 500 Hz rbpodo controller-sim circle-tracking tuning task). Treat it as a snapshot.
- **Milestone vs. activity**: the advertised milestone is simulator-first Cartesian hardening, but recent commits, `GOAL.md`, `scripts/rbpodo_*`, `configs/rbpodo_circle_ablation/*`, and many runbooks concentrate on the rbpodo controller-`pgmode` circle-tracking benchmark (ACKON500/500 Hz), which `docs/architecture.md` frames only as a narrow carve-out.
- **`TcpCircleTrack` is a stub** (`tcp_circle_track_not_implemented`); `TcpCircleMove` is implemented.
- **Precedence on conflict** (`AGENTS.md`): `AGENTS.md` → root `README.md` → `docs/architecture.md` → contract docs → component READMEs. `docs/current_review.md` redirects to `REVIEW.md`; `docs/archive/**` is audit-only.

## Development Rules

Work only on the assigned task; prefer small reviewable changes; keep configs strict and fail-closed; update tests, docs, and acceptance scripts together with behavior. Never fake rbpodo, Pinocchio, RealSense, SpaceMouse, or camera APIs — if a dependency is missing, report it and don't claim the gate passed. Don't claim C++/Pinocchio/Cartesian runtime acceptance passed unless the command was actually run.
