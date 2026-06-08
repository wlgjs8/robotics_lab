# Code-Centered Architecture Map

This document describes what the **code actually does** today, traced from source, and flags where scattered docs drift from the latest development direction. It complements—does not replace—the normative `docs/architecture.md`. When this map and a normative doc disagree about *intent/policy*, the normative doc wins; when they disagree about *what the code does*, trust the code and update whichever doc is stale.

Verified by reading source under `rb_servo_server/`, `rb_simulator/`, `rb_gui/`, `policy_runner/`, `camera_server/`, plus `docker-compose*.yml` and the component config YAMLs.

## Components (verified from code)

| Directory | Language | Role | Key sources |
|---|---|---|---|
| `rb_servo_server/` | C++17 (Eigen3 + mandatory Pinocchio) | Owner of the dual-arm synchronized servo loop: command freshness/lease, FK/IK, Cartesian target generation, safety filtering, fault latching, state publication | `src/control/dual_arm_servo_loop.cpp`, `src/network/{command_server,state_publisher}.cpp`, `src/robot/*backend.cpp`, `src/kinematics/pinocchio_kinematics.cpp`, `src/config/config.cpp` |
| `rb_simulator/` | Python | Hardware-free simulator, one controller endpoint per arm, JSON-lines TCP protocol with an admin fault-injection channel | `src/rbsim/{server,protocol,state_machine,config}.py` |
| `rb_gui/` | Python (viser web server) | mock/simulation operator console: state visualization + simulator-only commands; keeps real motion blocked client-side | `rb_servo_gui/{app,command_client,state_receiver,overlay_receiver,safety}.py` |
| `policy_runner/` | Python package | Action sources (SpaceMouse/joint/cartesian), episode recording, HDF5 audit, flow-matching training/inference, rollout-mode gating | `policy_runner/{main,servo_command_client,robot_state_client,camera_bundle_client,safety,flow_*,rollout_modes}.py` |
| `camera_server/` | C++ | RealSense/mock capture, shared-memory ring-buffer image transport, ZMQ metadata/health | `src/{main,camera/*,shm/*,publish/*,sync/*,health/*}.cpp` |
| `scripts/`, `tools/`, `configs/rbpodo_circle_ablation/` | Python/bash/yaml | rbpodo controller-`pgmode`-simulation circle-tracking benchmark, ablation, and report tooling | `scripts/rbpodo_circle_tracking_benchmark.py`, `scripts/run_rbpodo_circle_ablation.py`, … |

## Data flow and ports (verified against compose + config)

```
                 rb_gui (viser web :8080)
       state ◄── UDP :50110 ──┐        ┌── UDP :50010 ──► command
       overlay ◄─ UDP :50261 (optional, off by default)
                              │        │
   policy_runner ◄── UDP :50120 ──┐    ├── UDP :50010 ──► command
   (robot_state_client)           │    │
                                   ▼    ▼
                       ┌──────────────────────────────────┐
                       │        rb_servo_server (C++)       │
                       │  DualArmServoLoop @ servo.rate_hz   │
                       │  command_server  : UDP in  :50010   │
                       │  state_publisher : UDP fanout       │
                       │   state_pub_endpoints =             │
                       │     [:50110 gui, :50120 policy]     │
                       └───────────────┬────────────────────┘
                      TCP JSON-lines    │  control :50200 / admin :50201 (per arm)
                       ┌────────────────▼───────────────────┐
                       │   rb_simulator_left / _right         │
                       │   (ArmSimulator state machine)       │
                       └──────────────────────────────────────┘

   camera_server ── POSIX shm ring (/camera_server_frames) ──► policy_runner
                 ── ZMQ PUB tcp:5600 (topics: camera.bundle, camera.health) ──►
                    (camera_bundle_client subscribes metadata, reads images from shm)
```

In the **real** topology the simulator endpoints are replaced by the `rbpodo` backend talking to real controllers at `172.28.60.200` (left) / `172.28.60.201` (right). Simulator and real topologies are isomorphic by endpoint count and ownership, **not** by IP — simulator configs must not default to the real controller IPs.

The `rbpodo` backend also serves **controller (`pgmode`) simulation** (`run_mode: real`, `operation_mode: simulation`), pointed either at a Virtual ControlBox VM or a physical box held in `pgmode` — the same code path, distinguished only by deployment target. This is a separate flavor from the hardware-free `rb_simulator`; see the "Simulation flavors" table in `docs/architecture.md`.

### Port reference

| Path | Protocol | Port(s) | Notes |
|---|---|---|---|
| command in → `rb_servo_server` | UDP JSON | 50010 | `network.command_bind` |
| state out ← `rb_servo_server` | UDP JSON fanout | 50110 (gui), 50120 (policy) | `network.state_pub_endpoints` list; legacy single `state_pub_endpoint` is mirrored from the first entry |
| `rb_servo_server` ↔ simulator | TCP JSON-lines | 50200 control, 50201 admin (per arm) | `RbsimBackend`; separate containers may reuse internal ports |
| camera metadata | ZMQ PUB | 5600 | topics `camera.bundle`, `camera.health` |
| camera images | POSIX shared memory ring | `/camera_server_frames` (real) / `/camera_server_frames_test` (mock) | atomic seq begin/end for reader/writer coordination |
| GUI web | HTTP (viser) | 8080 | |
| circle overlay → gui | UDP JSON | 50261 | optional benchmark overlay, off by default |

### Wire formats

- **Command (UDP → server):** `{seq, mode, timeout_sec, left{…}, right{…}}`. `mode` parsed by `controlModeFromString()` in `rb_servo_server/src/core/types.cpp`. Per-arm payload fields include `q_target_deg`, `dq_target_deg_s`, `tcp_target_stand`, `tcp_delta_{stand,local}`, `tcp_twist_{stand,local}`, plus optional `lease_token`/`source_id`/`session_id`.
- **State (server → UDP fanout):** schema `robotics_lab.servo_state.v1`. Per arm: `q_actual_deg`, `q_target_deg`, `tcp_actual_stand`/`tcp_ref_stand` (with quaternion fields), `cartesian{ik…}` telemetry, `safety_tracking{…}`; top level `fault{latched, verdict, domain…}` and `physical_motion_expected`.
- **Simulator (server ↔ sim, TCP JSON-lines):** schema `rbsim.v1`, `op ∈ {connect, initialize, read_state, send_servo_j, stop, reset_fault}`. Separate **admin** channel (`admin.tick`, `admin.inject`, `admin.set_fault`, `admin.set_latency`, `admin.set_stale_state`, …) drives ticks and injects faults for tests.
- **Camera:** images live in the shared-memory ring; only metadata is published over ZMQ. Consumers subscribe to metadata, then read pixel data from shm by name/offset.

## Motion primitive implementation status (code-verified)

| Mode | Status | Notes |
|---|---|---|
| `JointTarget` / `JointVelocity` | implemented | passthrough / `dq*dt` integration |
| `TcpPoseTarget` | implemented | IK; MoveJ-like (intermediate TCP path not guaranteed linear) |
| `TcpLinearMove` | implemented | simulator-only MoveL; `orientation_mode` `constant`/`slerp` |
| `TcpDelta{Stand,Local}` / `TcpTwist{Stand,Local}` | implemented | debug jog / streaming velocity |
| `TcpCircleMove` | implemented | benchmark-only, gated by `cartesian_control.circle_move.allow_*` |
| `TcpCircleTrack` | **stub** | returns `SafetyVerdict::CartesianUnavailable`, reason `tcp_circle_track_not_implemented` (matches the docs' "disabled skeleton") |

## Safety gates (checked in code)

Real motion is fail-closed. Env gates are necessary but not sufficient — config and acceptance must also explicitly allow the operation. `rb_servo_server/src/config/config.cpp` `validateConfig()` (plus the rbpodo backend) enforce: `RB_ALLOW_REAL_ROBOT`, `RB_ALLOW_REAL_MOTION`, `RB_ALLOW_REAL_CARTESIAN`, `RB_ALLOW_NETWORK_EXPOSURE`, `RB_ALLOW_RBPODO_ACK_DISABLED_MOTION`, `RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION`, `RB_RBPODO_PGMODE_SIMULATION_CONFIRMED`, `RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN`, `RB_ALLOW_RBPODO_ASYNC_STREAMING`. Missing gates fail startup. `SafetyFilter` runs state-validity → tracking-error → joint/velocity/accel clamps; `stop_both_arms_on_single_arm_error` couples the arms. `rb_gui` and `policy_runner` re-apply their own client-side gates (stale state, fault latch, real-mode lockout, camera readiness).

## ML / dataset facts (code-verified)

- Flow-matching action dim **14** (per arm: 3 linear + 3 angular + 1 gripper); proprio dim **16**.
- Supported HDF5 layouts: `robotics_lab_dual_arm`, `pika_umi_single_arm`, `pika_umi_bimanual`. Audit schema `robotics_lab.policy_runner.hdf5_audit.v1`; manifest schema `robotics_lab.policy_runner.dataset_manifest.v1`.
- Rollout modes (`rollout_modes.py`): `offline_eval`, `sim_dryrun`, `controller_sim`, `real_readonly`, `real_policy`. `real_policy` stays blocked without measured/accepted retarget + collision + gripper + geometry gates. `controller_sim` is the `run_mode=real` + `operation_mode=simulation` carve-out.

## ⚠️ Doc vs. code drift (read before trusting a doc)

1. **`GOAL.md` is not a project-goal document.** It is the full text of a single task prompt (`ACKON500-GENE-GOAL-01`: tune rbpodo controller-sim 500 Hz ACK-ON 15 cm/4 s circle tracking to RMS ≤ 3 mm). Treat it as a point-in-time task snapshot, not direction. For overall direction trust `README.md` / `AGENTS.md` / `docs/architecture.md`.
2. **Stated milestone vs. where the code is busy.** The advertised milestone is "simulator-first Cartesian acceptance hardening," but recent commits, `GOAL.md`, `scripts/rbpodo_*`, `configs/rbpodo_circle_ablation/*`, and many runbooks concentrate on the rbpodo controller-`pgmode`-simulation circle-tracking benchmark (ACKON500 / 500 Hz). `docs/architecture.md` describes that only as a narrow "carve-out," yet by code volume it is the most active area. Don't assume the carve-out is a side concern.
3. **Server language.** `rb_servo_server` is **C++17 + Eigen3 + mandatory Pinocchio** (CMake hard-errors if Pinocchio is disabled; no fallback math). If a wiring summary calls it "unknown/Rust," that is wrong.
4. **State publication is a fanout, not a single port.** Some defaults/prose mention only `state_pub_endpoint :50110`; the real compose config publishes to both `:50110` (gui) and `:50120` (policy) via `state_pub_endpoints`.
5. **Port/protocol consistency is otherwise good.** 50010 (cmd), 50110/50120 (state), 50200/50201 (sim), 5600 (camera ZMQ), 8080 (gui), 50261 (overlay) all agree across code, config, and compose.
6. **Doc precedence.** Per `AGENTS.md`, when docs conflict the order is `AGENTS.md` → root `README.md` (KO; `README.en.md` is the English original) → `docs/architecture.md` → contract docs → component READMEs. `REVIEW.md` and `GOAL.md` are snapshots; `docs/current_review.md` is a redirect to `REVIEW.md`; `docs/archive/**` is audit-only.
