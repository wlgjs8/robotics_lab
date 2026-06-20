# Code-Centered Architecture Map

This document describes what the **code actually does** today, traced from source, and flags where scattered docs drift from the latest development direction. It complements—does not replace—the normative `docs/architecture.md`. When this map and a normative doc disagree about *intent/policy*, the normative doc wins; when they disagree about *what the code does*, trust the code and update whichever doc is stale.

Verified by reading source under `rb_servo_server/`, `rb_simulator/`, `rb_gui/`, `policy_runner/`, `camera_server/`, plus `docker-compose*.yml` and the component config YAMLs.

## Components (verified from code)

| Directory | Language | Role | Key sources |
|---|---|---|---|
| `rb_servo_server/` | C++17 (Eigen3 + mandatory Pinocchio) | Owner of the dual-arm synchronized servo loop: command freshness/lease, FK/IK, Cartesian target generation, safety filtering, fault latching, state publication | `src/control/dual_arm_servo_loop.cpp`, `src/network/{command_server,state_publisher}.cpp`, `src/robot/*backend.cpp`, `src/kinematics/pinocchio_kinematics.cpp`, `src/config/config.cpp` |
| `rb_simulator/` | Python | Hardware-free simulator, one controller endpoint per arm, JSON-lines TCP protocol with an admin fault-injection channel | `src/rbsim/{server,protocol,state_machine,config}.py` |
| `rb_gui/` | Python (viser web server) | operator console: state visualization + command emission. The client-side real/sim execution lock is **retired** (`safety.py`); controls emit in every run mode and real-motion authority sits entirely on the server. The GUI only enforces non-gating readiness (stale state, joint validity, sim-readiness tests, fault latch, FK/TCP availability) | `rb_servo_gui/{app,command_client,state_receiver,overlay_receiver,safety}.py` |
| `policy_runner/` | Python package | Action sources (SpaceMouse/joint/cartesian), episode recording, HDF5 audit, flow-matching training/inference, rollout-mode gating | `policy_runner/{main,servo_command_client,robot_state_client,camera_bundle_client,safety,flow_*,rollout_modes}.py` |
| `camera_server/` | C++ | RealSense/mock capture, shared-memory ring-buffer image transport, ZMQ metadata/health | `src/{main,camera/*,shm/*,publish/*,sync/*,health/*}.cpp` |
| `scripts/`, `tools/` | Python/bash/yaml | rbpodo bring-up, acceptance, diagnostics, and ML/dataset helpers (the rbpodo controller-simulation circle-tracking benchmark / ACKON500 tooling and its `configs/rbpodo_circle_ablation/` configs were removed 2026-06-20) | `scripts/rbpodo_servo_acceptance.py`, `scripts/collect_gene_umi_artifact_manifest.py`, … |

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

- **Command (UDP → server):** `{seq, mode, timeout_sec, left{…}, right{…}}`. `mode` parsed by `controlModeFromString()` in `rb_servo_server/src/core/types.cpp`. Per-arm payload fields include `q_target_deg`, `dq_target_deg_s`, `tcp_target_stand`, `tcp_delta_{stand,local}`, `tcp_twist_{stand,local}`, plus optional `lease_token`/`source_id`/`session_id`. `SetSafetyFloorZ` is a leaseless non-motion mode with a top-level `floor_z_m` payload (bounded by `safety.floor_constraint.runtime_{min,max}_z_m`).
- **State (server → UDP fanout):** schema `robotics_lab.servo_state.v1`. Per arm: `q_actual_deg`, `q_target_deg`, `tcp_actual_stand`/`tcp_ref_stand` (with quaternion fields), `cartesian{ik…}` telemetry, `safety_tracking{…}`; top level `fault{latched, verdict, domain…}`, `physical_motion_expected`, `self_collision{…}`, and `floor_constraint{enabled, monitor_only, z_min_m, config_z_min_m, runtime_{min,max}_z_m, left/right{checked, violated, tcp_z_m}, clamp_count, last_set_reject_reason}`.
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

Real Cartesian (`TcpPoseTarget` and `TcpTwist*`) is opened by the real-mode gates plus `cartesian_control.allow_in_real: true`, and has been driven on the physical arms via `TcpPoseTarget` (dual-arm slow circle). `TcpLinearMove` remains simulator-only.

## Safety gates (checked in code)

Real motion is fail-closed. Env gates are necessary but not sufficient — config and acceptance must also explicitly allow the operation. `rb_servo_server/src/config/config.cpp` `validateConfig()` (plus the rbpodo backend) enforce: `RB_ALLOW_REAL_ROBOT`, `RB_ALLOW_REAL_MOTION`, `RB_ALLOW_REAL_CARTESIAN`, `RB_ALLOW_NETWORK_EXPOSURE`, `RB_ALLOW_RBPODO_ACK_DISABLED_MOTION`, `RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION`, `RB_RBPODO_PGMODE_SIMULATION_CONFIRMED`, `RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN`, `RB_ALLOW_RBPODO_ASYNC_STREAMING`. The rbpodo backend additionally gates the real-mode `-2001` suspect-diagnostics carve-out behind `RB_ALLOW_RBPODO_SUSPECT_DIAGNOSTICS_REAL_MOTION` + config `servo.allow_real_motion_with_suspect_diagnostics` (PR #12). Missing gates fail startup. `SafetyFilter` runs state-validity → tracking-error → joint/velocity/accel clamps; `stop_both_arms_on_single_arm_error` couples the arms. `rb_gui` and `policy_runner` re-apply their own client-side gates (stale state, fault latch, camera readiness) — note that `policy_runner`'s `SafetyGate` no longer blocks real Cartesian motion (PR #13, scoped to `cartesian_gate.operation_mode == "real"`), so for real motion `rb_servo_server` is the sole safety layer. These server env gates are still present on `dev`; the 2026-06-11 working-tree experiment that removed them was not landed.

## ML / dataset facts (code-verified)

- Flow-matching action dim **14** (per arm: 3 linear + 3 angular + 1 gripper); proprio dim **16**.
- Supported HDF5 layouts: `robotics_lab_dual_arm`, `pika_umi_single_arm`, `pika_umi_bimanual`. Audit schema `robotics_lab.policy_runner.hdf5_audit.v1`; manifest schema `robotics_lab.policy_runner.dataset_manifest.v1`.
- Rollout modes (`rollout_modes.py`): `offline_eval`, `sim_dryrun`, `controller_sim`, `real_readonly`, `real_policy`. `real_policy`'s gate (`_validate_real_policy`) is fully enforced — it requires `mode=real`, `allow_real_motion`, measured/accepted geometry + retarget, validated collision model, workspace envelope, and the gripper gate — but it is *satisfiable*: a live pi0.5/openpi `real_policy` rollout has run end-to-end on the physical robot (`TcpTwistLocal` streaming + gripper; `outputs/rollout_*real*.json` show `may_send_commands: true`, `sent_command_count > 0`, `collision_model_status: validated`). So the lane is open and exercised, not blocked; task success is the remaining (model-side) gap. `controller_sim` is the `run_mode=real` + `operation_mode=simulation` carve-out.
- Real gripper motion goes through the Pika Gripper Backend (`gripper.py`), gated by `RB_ALLOW_REAL_GRIPPER=1` + `allow_real_gripper_motion` + `measured_gripper_available`; it has been driven during real-policy rollout.
- Self-collision is a server-side async URDF-mesh guard (`rb_servo_server` `CollisionMonitor`, ~33 geoms / 337 pairs) run off the 2 ms servo path; `applySafety` reads the latest verdict and applies a velocity barrier (sim `clamp_hold` / real `fault_latch`). It superseded the in-loop capsule guard, which is kept as a compiled fallback branch.

## ⚠️ Doc vs. code drift (read before trusting a doc)

1. **The ACKON500 circle-tracking benchmark and `GOAL.md` were removed (2026-06-20).** The rbpodo controller-sim 500 Hz ACK-ON circle-tracking benchmark subsystem — its scripts, ablation/report tooling, `configs/rbpodo_circle_ablation/*` configs, runbooks, and the `GOAL.md` task-prompt snapshot — has been deleted. For overall direction trust `README.md` / `AGENTS.md` / `docs/architecture.md`.
2. **Milestone is physical bring-up; the controller-reference circle-benchmark lane is gone.** The current milestone is rbpodo pgmode-real physical bring-up — real motion has been exercised under supervision (`docs/runbooks/rbpodo_real_physical_circle.md`). The Cartesian-circle evidence that matters is physical-real (`tcp_actual_stand`); the prior controller-reference (`tcp_ref_stand`) ACKON500 / 500 Hz benchmark lane was removed and is no longer tracked.
3. **Server language.** `rb_servo_server` is **C++17 + Eigen3 + mandatory Pinocchio** (CMake hard-errors if Pinocchio is disabled; no fallback math). If a wiring summary calls it "unknown/Rust," that is wrong.
4. **State publication is a fanout, not a single port.** Some defaults/prose mention only `state_pub_endpoint :50110`; the real compose config publishes to both `:50110` (gui) and `:50120` (policy) via `state_pub_endpoints`.
5. **Port/protocol consistency is otherwise good.** 50010 (cmd), 50110/50120 (state), 50200/50201 (sim), 5600 (camera ZMQ), 8080 (gui), 50261 (overlay) all agree across code, config, and compose.
6. **GUI client-side execution lock is retired (server gates are not).** `rb_gui` `safety.py` (`blocked_reason`, `tcp_command_disabled_reason`) no longer blocks real-mode commands or gates on a mode/backend match, and the old `RB_GUI_ENABLE_*` opt-in env flags are gone — controls emit in every run mode. This is *only* the client lock; the **server** `RB_ALLOW_REAL_ROBOT/MOTION/CARTESIAN` env gates remain present and fail-closed (`state_publisher.cpp`, `rbpodo_backend.cpp`). Don't read "GUI lock removed" as "real-motion gates removed." Older `rb_servo_server/docs/gui_operator_console.md` prose that says "real motion ... disabled in the GUI / connect-status only" predates this and has been corrected.
7. **Unified collision URDF is an untracked external asset.** The self-collision `CollisionMonitor` loads `safety.self_collision.mesh.unified_urdf` — a stand+both-arms URDF (`mo_robot_descriptions/.../dual_rb3_730e_ver3.urdf`) that lives in the **separate, non-submodule** `mo_robot_descriptions` repo and is **not** git-tracked here; only the single-arm `rb_servo_server/descriptions/urdf/rb3_730e.urdf` (used by `kinematics.urdf`) is tracked. Provenance/version-pin/regeneration of the dual URDF is documented in `docs/frame_contract.md` → "Collision Geometry Asset (unified URDF)".
8. **Doc precedence.** Per `AGENTS.md`, when docs conflict the order is `AGENTS.md` → root `README.md` (KO; `README.en.md` is the English original) → `docs/architecture.md` → contract docs → component READMEs. `REVIEW.md` is a snapshot; `docs/current_review.md` is a redirect to `REVIEW.md`; `docs/archive/**` is audit-only.
