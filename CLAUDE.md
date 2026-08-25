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

Historical files (`TODO.md`, `CODEX_*PROMPTS*`, `MIG-*`, `HARDEN-*`, `CART-HARDEN-*`, `docs/archive/**`) are audit context only. `REVIEW.md` is a point-in-time snapshot, not direction — see the drift notes below. (The ACKON500 circle-tracking benchmark and its old root `GOAL.md` snapshot were removed 2026-06-20; `policy_runner/GOAL.md` is a separate active policy-training note.)

## Current Phase

The repo is in **rbpodo pgmode-real physical robot bring-up**. Simulator-first Cartesian acceptance hardening is largely complete and is now the regression baseline; active validation is on the physical RB3-730E hardware. Real motion is an operator-supervised lane that is actively exercised (`docs/runbooks/rbpodo_real_physical_circle.md`, ladder in `docs/runbooks/pgmode_real_transition.md`). Run on hardware: read-only diagnostics parity; dual-arm physical Cartesian circle (TUNED-1, ~1.42°); UMI Cartesian teleop + `data_tcp` replay on real arms; **a full `flow-infer` `real_policy` closed-loop rollout (pi0.5/openpi, `TcpPoseTarget` + real gripper)** — its rollout-mode validation was satisfied via accepted/validated config (`collision_model_status: validated`), so the lane is open, not blocked. Runtime is validated (smooth, in-distribution; async chunking; reset-relative proprio fixes the absolute-frame gap); **task success is the remaining model-side gap**, not runtime. Real Cartesian execution authority is config-driven and server-owned: PR #13 retired the policy-side real-Cartesian block, PR #12 accepted the `-2001` suspect-diagnostics carve-out, and async URDF-mesh `CollisionMonitor` is enforced. Pika gripper output still has its own `RB_ALLOW_REAL_GRIPPER` runtime gate. Still off: force control. Measured hand-eye calibration is unneeded for the deployed pika ee_local image-conditioned policy (reset-relative cancels it) but still required for general geometry-dependent policy. Passing simulator acceptance is never permission to move hardware.

## Architecture

Four cooperating services (real and controller-simulation topology are isomorphic by endpoint count, not by IP):

```
                         policy_runner (Python action sources, SpaceMouse, flow ML)
rb_gui (viewer/operator) ─┐                       │
                          ▼                        ▼  UDP JSON commands
                     rb_servo_server (C++ control loop) ──► left/right backend endpoint
                          ▲  UDP state fanout
camera_server (C++) ──────┘ (RealSense/mock, shared-memory image transport)
```

- **`rb_servo_server`** (C++17) is the control owner. `DualArmServoLoop` owns command freshness, command-source lease/arbitration, FK/IK and Cartesian target generation, safety filtering, fault latching, dual-arm aggregation, and state publication. Per-arm blocking backend I/O lives behind `ArmWorker` → `IRobotBackend` (`MockBackend`, `RbpodoBackend`). Cartesian math (FK/IK, SO(3)/SE(3), orientation interpolation, frame conversion) MUST use Eigen3/Pinocchio — no local fallback math paths. Key dirs: `src/control/`, `src/robot/`, `src/kinematics/`, `src/math/`, `include/rb_servo/`.
- **`rb_gui`** (Python, `rb_servo_gui/`) is a viewer/operator console. It exposes every motion primitive in every run mode; real-motion authority sits entirely on the server.
- **`policy_runner`** (Python package) owns action sources (`action_sources/`), recording, HDF5 audit, and flow-matching ML training/inference (`flow_*.py`). SpaceMouse Cartesian and `flow-infer` ee_local deltas emit absolute `TcpPoseTarget` setpoints.
- **`camera_server`** (C++) owns capture, shared-memory ring-buffer transport, metadata, and health. Camera acceptance is separate from robot motion acceptance.

State fanout: `rb_servo_server` is the sole owner of UDP state publication via `network.state_pub_endpoints` (list). Commands go directly to `network.command_bind`. Benchmark overlay streams (desired geometry/metrics) are separate from robot state and must never carry commands.

Ports/protocols (verified against compose + config; full table in `docs/code_architecture_map.md`): commands in `UDP 50256`; chunk frames on `UDP 50264`; state fanout to `UDP 50356` (joint scope dashboard), `50366` (viser GUI), `50376` (stack policy_runner/teleop_mux), and `50378` (external flow-infer readback); gripper command/feedback `UDP 50410`/`50420`; camera metadata `ZMQ 5600` (`camera.bundle`/`camera.health`) with images in a POSIX shared-memory ring (`/camera_server_frames`); GUI web `HTTP 8080`; optional circle overlay `UDP 50261`. Command JSON `{seq, mode, left{…}, right{…}}` (`mode` parsed by `controlModeFromString` in `src/core/types.cpp`); state JSON schema `robotics_lab.servo_state.v1`.

## Canonical Terminology (use everywhere — config, docs, GUI, logs, tests)

```yaml
run_mode: mock | simulation | real
backend_type: mock | rbpodo
```

`rbpodo` is the only supported real-controller backend. Do not introduce removed simulator-backend aliases or raw-script TCP comparison backends. (`run_mode: simulation` now refers only to the rbpodo controller `pgmode` simulation flavor — see `docs/architecture.md`.)

## Safety Boundary

Real behavior is fail-closed and never implicit, but it is **no longer gated on env vars**. The legacy execution gates — `RB_ALLOW_REAL_ROBOT`, `RB_ALLOW_REAL_MOTION`, `RB_ALLOW_REAL_CARTESIAN`, `RB_ALLOW_RBPODO_ACK_DISABLED_MOTION`, `RB_ALLOW_RBPODO_SUSPECT_DIAGNOSTICS_REAL_MOTION`, `RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION`, `RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN`, `RB_RBPODO_PGMODE_SIMULATION_CONFIRMED` — are removed from the server runtime. `run_mode`/`operation_mode` are telemetry labels only and do not decide whether motion is allowed.

Real-motion execution authority is owned by **site-local config + the mode-independent safety layers**:

- safety filter (joint clamp, stand-frame floor plane)
- tracking-error latch
- async URDF-mesh self-collision guard (`CollisionMonitor`)
- command-source lease / arbitration
- client deadman
- operator supervision + hardware E-stop

Real motion requires the gitignored site config (`rb_servo_server/config/local/`) to enable it explicitly. The controller `-2001` suspect-diagnostics acceptance is now a per-arm config opt-in (`allow_real_motion_with_suspect_diagnostics: true`, no env); EMS/SOS/soft-estop/`collision_occur`/unknown-mode/init-error still latch. The rbpodo `pgmode` controller-simulation carve-out (connects to real boxes but `operation_mode: simulation`, `physical_motion_expected=false`) is config-driven via `cartesian_control.allow_in_controller_simulation: true` + `servo.allow_controller_simulation_motion: true` (no env).

The policy-side `SafetyGate` real-Cartesian block was retired (PR #13); stale state, fault, camera, and kinematics readiness checks remain. For real Cartesian motion, `rb_servo_server` makes the final allow/deny decision. Controller-simulation safety is unchanged.

Other invariants: never reintroduce bool-only backend results (preserve `BackendResult<RobotState>`, `SendServoJResult`, `BackendErrorKind`, `BackendTiming`, `FaultContext`); don't parse error strings when structured fields exist; force control stays `provider: null, enable: false`; `servo.io_model: worker` is a supported real-mode path (the mock-only refusal was retired) — it is what control-box queue sync runs on, because holding each box's queue at a fixed depth requires the arm to own its own send cadence and one loop has a single period for two boxes with two clocks. The stand-frame floor plane (`safety.floor_constraint`) applies in EVERY run mode when enabled (no env/mode gate), covers all motion primitives at the final joint-level safety gate, requires `kinematics.enable`, and its runtime lowering via the leaseless `SetSafetyFloorZ` command is bounded to the config `[runtime_min_z_m, runtime_max_z_m]`; `monitor_only` is never a real-motion posture. Tracked stack config templates are `rb_servo_server/config/stack_real.yaml` and `rb_servo_server/config/stack_sim.yaml`; site-specific variants go in gitignored `rb_servo_server/config/local/`. New rbpodo configs use canonical Servo J fields `servo_t1_sec` / `servo_t2_sec` / `servo_gain` / `servo_alpha` (not `servo_time_sec` / `servo_lookahead_sec` / `servo_acc`); `servo_t1_sec: 0.002` at the supported 500 Hz.

## Motion Primitives (don't blur)

`JointTarget` (absolute joint PTP) · `TcpPoseTarget` (Cartesian final-pose PTP, path not guaranteed linear) · `TcpLinearMove` (MoveL with `constant`/`slerp` orientation modes; real-motion-ready, used on real; finite path completes from one click across lease/command lapse). `SetSafetyFloorZ` is a leaseless non-motion command that adjusts the floor plane height within config bounds. When `safety.floor_constraint` is enabled, every primitive above is FK-checked against the floor plane at the final safety gate (Cartesian paths additionally slide along the plane; joint-space primitives hold).

## Commands

Python tests (per component):
```bash
python3 -m unittest discover rb_gui/tests
python3 -m unittest discover policy_runner/tests
python3 -m compileall -q rb_gui/rb_servo_gui policy_runner/policy_runner scripts
```
Run a single Python test: `python3 -m unittest policy_runner.tests.test_geometry_safety` (or `... -k <name>`).

C++ servo server build (requires Eigen3 + Pinocchio):
```bash
cmake -S rb_servo_server -B rb_servo_server/build
cmake --build rb_servo_server/build -j
ctest --test-dir rb_servo_server/build --output-on-failure
```

C++ servo server build and tests:
```bash
cmake -S rb_servo_server -B rb_servo_server/build
cmake --build rb_servo_server/build -j
ctest --test-dir rb_servo_server/build --output-on-failure
```

Cartesian behavior is validated with the Pinocchio-backed C++ tests plus
active-stack smoke/acceptance on mock when a local mock config is available,
rbpodo controller `pgmode` simulation / VM, and physical real only through the
separate supervised runbooks. The prior simulator-first hardware-free Cartesian
acceptance lane and its software-simulator-oriented runner were retired.

Operator stacks (native via `Makefile`; the GUI/servo/policy stack runs without Docker):
```bash
make run               # full local teleop stack: rb_servo_server + viser GUI + policy_runner (SpaceMouse + UMI); pgmode real
make run MODE=sim      # same stack, pgmode controller-simulation
make build       # source-build + install the native stack (rbpodo backend) into the path `make run` launches
make vm-up / vm-down / vm-status   # boot/stop the Rainbow virtual control-box VMs for hardware-free MODE=sim
make pgmode-sim-up / pgmode-sim-down   # native rb_servo_server + rb_gui controller-sim bring-up
make cam-up / cam-down / cam-status   # the only Docker stack: camera_server (D435 head + dual D405 wrists, one container)
make cam-up-wrists    # wrist-only rig: dual D405 without the head D435 (wrist RGB-D bundles still flow)
```
GUI: `http://127.0.0.1:8080`. Docker is used ONLY for `camera_server`; everything else runs natively. Camera lifecycle is managed by the `cam-*` targets (`cam-up`/`cam-up-wrists`/`cam-down`/`cam-status`); override the rig with `make cam-up CAMERA_CONFIG=/app/config/<rig>.yaml`. Flow-matching training runs natively on the GPU server with `python3 -m policy_runner flow-train`.

ML data flow: audit HDF5 episodes (`python3 -m policy_runner hdf5-audit ...`, schema `robotics_lab.policy_runner.hdf5_audit.v1`) before flow-train. `flow-infer` requires explicit `--rollout-mode` (`offline_eval` / `sim_dryrun` / `controller_sim` / `real_readonly` / `real_policy`); `real_policy` enforces measured/accepted retarget, collision, gripper, and geometry gates — satisfiable, and exercised live (a full real_policy rollout has run on hardware).

## Doc Map & Drift (read before trusting a doc)

Scattered docs don't all track the latest direction. Key gotchas (full list in `docs/code_architecture_map.md`):

- **The ACKON500 circle-tracking benchmark was removed** — the 500 Hz rbpodo controller-sim circle-tracking benchmark subsystem (its scripts, `configs/rbpodo_circle_ablation/*`, ablation/report tooling, runbooks, and the old root `GOAL.md` task snapshot) was deleted 2026-06-20. Older notes that called root `GOAL.md` "a snapshot of the ACKON500 task" are obsolete; `policy_runner/GOAL.md` is unrelated.
- **Milestone**: the milestone is rbpodo pgmode-real physical bring-up (real motion exercised under supervision — see `docs/runbooks/rbpodo_real_physical_circle.md`). The Cartesian-circle evidence that matters is physical-real (`tcp_actual_stand`); the prior controller-reference (`tcp_ref_stand`) benchmark lane is gone.
- **Precedence on conflict** (`AGENTS.md`): `AGENTS.md` → root `README.md` → `docs/architecture.md` → contract docs → component READMEs. `docs/current_review.md` redirects to `REVIEW.md`; `docs/archive/**` is audit-only.

## Development Rules

Work only on the assigned task; prefer small reviewable changes; keep configs strict and fail-closed; update tests, docs, and acceptance scripts together with behavior. Never fake rbpodo, Pinocchio, RealSense, SpaceMouse, or camera APIs — if a dependency is missing, report it and don't claim the gate passed. Don't claim C++/Pinocchio/Cartesian runtime acceptance passed unless the command was actually run.

No silent fallback defaults for safety-relevant parameters (applies to every component — C++, `rb_gui`, `policy_runner`). Any value that bounds motion, contact, a tolerance, geometry/frames, or another safety-affecting decision must come from its authoritative source (server config, contract doc, measured evidence). If that source is missing or unreadable, FAIL CLOSED — don't fire/move, return `None`/error, and surface the reason — rather than substituting a guessed or hard-coded default that can be wrong in the unsafe direction (e.g. a tolerance that lets the server plan a move when the caller assumed a no-op).
