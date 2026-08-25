# Code-Centered Architecture Map

This document describes what the **code actually does** today, traced from source, and flags where scattered docs drift from the latest development direction. It complements—does not replace—the normative `docs/architecture.md`. When this map and a normative doc disagree about *intent/policy*, the normative doc wins; when they disagree about *what the code does*, trust the code and update whichever doc is stale.

Verified by reading source under `rb_servo_server/`, `rb_gui/`, `policy_runner/`, `camera_server/`, plus `docker-compose*.yml` and the component config YAMLs. (The old Python software-simulator package and backend were removed 2026-06-20; hardware-free validation now uses `MockBackend` and rbpodo controller `pgmode` simulation.)

## Components (verified from code)

| Directory | Language | Role | Key sources |
|---|---|---|---|
| `rb_servo_server/` | C++17 (Eigen3 + mandatory Pinocchio) | Owner of the dual-arm synchronized servo loop: command freshness/lease, FK/IK, Cartesian target generation, safety filtering, fault latching, state publication | `src/control/dual_arm_servo_loop.cpp`, `src/network/{command_server,state_publisher}.cpp`, `src/robot/*backend.cpp`, `src/kinematics/pinocchio_kinematics.cpp`, `src/config/config.cpp` |
| `rb_gui/` | Python (viser web server) | operator console: state visualization + command emission. The client-side real/sim execution lock is **retired** (`safety.py`); controls emit in every run mode and real-motion authority sits entirely on the server. The GUI only enforces non-gating readiness (stale state, joint validity, sim-readiness tests, fault latch, FK/TCP availability) | `rb_servo_gui/{app,command_client,state_receiver,overlay_receiver,safety}.py` |
| `policy_runner/` | Python package | Action sources (SpaceMouse/joint/cartesian), episode recording, HDF5 audit, flow-matching training/inference, rollout-mode gating | `policy_runner/{main,servo_command_client,robot_state_client,camera_bundle_client,safety,flow_*,rollout_modes}.py` |
| `camera_server/` | C++ | RealSense/mock capture, shared-memory ring-buffer image transport, ZMQ metadata/health | `src/{main,camera/*,shm/*,publish/*,sync/*,health/*}.cpp` |
| `scripts/`, `tools/` | Python/bash/yaml | rbpodo bring-up, acceptance, diagnostics, and ML/dataset helpers (the rbpodo controller-simulation circle-tracking benchmark / ACKON500 tooling and its `configs/rbpodo_circle_ablation/` configs were removed 2026-06-20) | `scripts/rbpodo_servo_acceptance.py`, `scripts/collect_gene_umi_artifact_manifest.py`, … |

## Data flow and ports (verified against compose + config)

```
                 rb_gui (viser web :8080)
       state ◄── UDP :50366 ──┐        ┌── UDP :50256 ──► command
       overlay ◄─ UDP :50261 (optional, off by default)
                              │        │
   policy_runner ◄── UDP :50376 ──┐    ├── UDP :50256 ──► command
   (robot_state_client)           │    │
                                   ▼    ▼
                       ┌──────────────────────────────────┐
                       │        rb_servo_server (C++)       │
                       │  DualArmServoLoop @ servo.rate_hz   │
                       │  command_server  : UDP in  :50256   │
                       │  chunk_frame in  : UDP in  :50264   │
                       │  state_publisher : UDP fanout       │
                       │   state_pub_endpoints =             │
                       │     [:50356 scope, :50366 gui,      │
                       │      :50376 policy, :50378 flow]    │
                       │  gripper bridge : UDP 50410/50420   │
                       │   per-arm backend: MockBackend /    │
                       │     RbpodoBackend (real + pgmode)   │
                       └────────────────────────────────────┘

   camera_server ── POSIX shm ring (/camera_server_frames) ──► policy_runner
                 ── ZMQ PUB tcp:5600 (topics: camera.bundle, camera.health) ──►
                    (camera_bundle_client subscribes metadata, reads images from shm)
```

In the **real** topology the per-arm backend is the `rbpodo` backend talking to real controllers at `172.28.60.200` (left) / `172.28.60.201` (right). Hardware-free runs use `MockBackend`.

The `rbpodo` backend also serves **controller (`pgmode`) simulation** (`run_mode: real`, `operation_mode: simulation`), pointed either at a Virtual ControlBox VM or a physical box held in `pgmode` — the same code path, distinguished only by deployment target. See the "Simulation flavors" table in `docs/architecture.md`. (The former hardware-free software-simulator flavor was removed.)

### Port reference

| Path | Protocol | Port(s) | Notes |
|---|---|---|---|
| command in → `rb_servo_server` | UDP JSON | 50256 | `network.command_bind` in `rb_servo_server/config/stack_{real,sim}.yaml` |
| chunk frame in → `rb_servo_server` | UDP JSON | 50264 | `network.chunk_frame_bind`; separate from the lease-gated command socket |
| state out ← `rb_servo_server` | UDP JSON fanout | 50356 (scope), 50366 (viser GUI), 50376 (`make run` policy_runner), 50378 (external flow-infer) | `network.state_pub_endpoints` list |
| gripper command / feedback | UDP JSON | 50410 / 50420 | `gripper.command_endpoint` and `gripper.feedback_bind`; launched by `tools/run_stack.sh` |
| camera metadata | ZMQ PUB | 5600 | topics `camera.health` + bundle groups: `camera.bundle` (legacy full-rig), `camera.bundle.policy` (both wrists, real_policy gate), `camera.bundle.stereo` (head color + IR, head rig only; also feeds the rb_gui head view panel), `camera.bundle.wrist_left`/`.wrist_right` (per-side wrist RGB-D so one dead camera cannot block the healthy side). Wrist-only rig: `make cam-up-wrists` (dual_realsense_d405.yaml + `STEREO_HEAD=0`, no head D435) |
| camera images | POSIX shared memory ring | `/camera_server_frames` (real) / `/camera_server_frames_test` (mock) | atomic seq begin/end for reader/writer coordination |
| GUI web | HTTP (viser) | 8080 | |
| GUI overlay → gui | UDP JSON | 50261 | optional overlay, off by default |

### Wire formats

- **Command (UDP → server):** `{seq, mode, timeout_sec, left{…}, right{…}}`. `mode` parsed by `controlModeFromString()` in `rb_servo_server/src/core/types.cpp`. Public motion payloads are `q_target_deg` for `JointTarget`, `tcp_target_stand` / `target_tcp_stand` for `TcpPoseTarget` and `TcpLinearMove`, plus optional `joint_target_profile: init_motion` for the collision-free init profile. Lifecycle/safety packets carry their documented top-level payloads and optional `lease_token`/`source_id`/`session_id`.
- **State (server → UDP fanout):** schema `robotics_lab.servo_state.v1`. Per arm: `q_actual_deg`, `q_target_deg`, `tcp_actual_stand`/`tcp_ref_stand` (with quaternion fields), `cartesian{ik…}` telemetry, `safety_tracking{…}`; top level `fault{latched, verdict, domain…}`, `physical_motion_expected`, `self_collision{…}`, and `floor_constraint{enabled, monitor_only, z_min_m, config_z_min_m, runtime_{min,max}_z_m, left/right{checked, violated, tcp_z_m}, clamp_count, last_set_reject_reason}`.
- **Camera:** images live in the shared-memory ring; only metadata is published over ZMQ. Consumers subscribe to metadata, then read pixel data from shm by name/offset.

## Motion primitive implementation status (code-verified)

| Mode | Status | Notes |
|---|---|---|
| `JointTarget` | implemented | passthrough; optional `joint_target_profile: init_motion` |
| `TcpPoseTarget` | implemented | IK; MoveJ-like (intermediate TCP path not guaranteed linear) |
| `TcpLinearMove` | implemented | MoveL, real-motion-ready (used on real); `orientation_mode` `constant`/`slerp`; finite path completes from one click across lease/command lapse |

Real Cartesian execution is no longer run-mode gated by env. `TcpPoseTarget`
and `TcpLinearMove` compute in every run mode when site-local config opens the
Cartesian path and are exercised on the physical arms. The mode-independent
server layers (safety filter, floor/ROI/reach, self-collision barrier,
tracking-error latch, lease/deadman) make the final allow/deny decision; E-stop
and operator supervision remain physical operation procedure.

## Safety gates (checked in code)

The legacy `RB_ALLOW_REAL_*` server execution env gates were **removed** from the runtime (`rb_servo_server/src/config/config.cpp` notes "Real/sim env gates retired" at the former `validateConfig()` gate sites). Real-motion execution authority is site-local config (`cartesian_control.allow_in_real: true`) + the mode-independent server safety layers; `run_mode`/`operation_mode` are telemetry labels only. The real-mode `-2001` suspect-diagnostics carve-out is now a per-arm config opt-in (`servo.allow_real_motion_with_suspect_diagnostics`, no env; PR #12). `SafetyFilter` runs state-validity → tracking-error → joint/velocity/accel clamps; `stop_both_arms_on_single_arm_error` couples the arms. `rb_gui` and `policy_runner` still check readiness (stale state, fault latch, camera/kinematics as needed), but `policy_runner`'s `SafetyGate` no longer blocks real Cartesian motion (PR #13). For real Cartesian motion, `rb_servo_server` makes the final allow/deny decision.

## ML / dataset facts (code-verified)

- Flow-matching action dim **14** (per arm: 3 linear + 3 angular + 1 gripper); proprio dim **16**.
- Supported HDF5 layouts: `robotics_lab_dual_arm`, `pika_umi_single_arm`, `pika_umi_bimanual`. Audit schema `robotics_lab.policy_runner.hdf5_audit.v1`; manifest schema `robotics_lab.policy_runner.dataset_manifest.v1`.
- Rollout modes (`rollout_modes.py`): `offline_eval`, `sim_dryrun`, `controller_sim`, `real_readonly`, `real_policy`. `real_policy`'s gate (`_validate_real_policy`) is fully enforced — it requires `mode=real`, `allow_real_motion`, measured/accepted geometry + retarget, validated collision model, workspace envelope, and the gripper gate — but it is *satisfiable*: a live pi0.5/openpi `real_policy` rollout has run end-to-end on the physical robot (`TcpPoseTarget` + gripper; `outputs/rollout_*real*.json` show `may_send_commands: true`, `sent_command_count > 0`, `collision_model_status: validated`). So the lane is open and exercised, not blocked; task success is the remaining (model-side) gap. `controller_sim` is the `run_mode=real` + `operation_mode=simulation` carve-out.
- Real gripper motion goes through the Pika Gripper Backend (`gripper.py`), gated by `RB_ALLOW_REAL_GRIPPER=1` + `allow_real_gripper_motion` + `measured_gripper_available`; it has been driven during real-policy rollout.
- Self-collision is a server-side async URDF-mesh guard (`rb_servo_server` `CollisionMonitor`, ~33 geoms / 337 pairs) run off the 2 ms servo path; `applySafety` reads the latest verdict and applies a velocity barrier (sim `clamp_hold` / real `fault_latch`). It superseded the in-loop capsule guard, which is kept as a compiled fallback branch.

## ⚠️ Doc vs. code drift (read before trusting a doc)

1. **The ACKON500 circle-tracking benchmark and old root `GOAL.md` were removed (2026-06-20).** The rbpodo controller-sim 500 Hz ACK-ON circle-tracking benchmark subsystem — its scripts, ablation/report tooling, `configs/rbpodo_circle_ablation/*` configs, runbooks, and the old root `GOAL.md` task-prompt snapshot — has been deleted. `policy_runner/GOAL.md` is a separate active policy-training note. For overall direction trust `README.md` / `AGENTS.md` / `docs/architecture.md`.
2. **Milestone is physical bring-up; the controller-reference circle-benchmark lane is gone.** The current milestone is rbpodo pgmode-real physical bring-up — real motion has been exercised under supervision (`docs/runbooks/rbpodo_real_physical_circle.md`). The Cartesian-circle evidence that matters is physical-real (`tcp_actual_stand`); the prior controller-reference (`tcp_ref_stand`) ACKON500 / 500 Hz benchmark lane was removed and is no longer tracked.
3. **Server language.** `rb_servo_server` is **C++17 + Eigen3 + mandatory Pinocchio** (CMake hard-errors if Pinocchio is disabled; no fallback math). If a wiring summary calls it "unknown/Rust," that is wrong.
4. **State publication is a fanout, not a single port.** Current stack configs publish to `:50356` (scope), `:50366` (viser GUI), `:50376` (`make run` policy_runner), and `:50378` (external flow-infer) via `state_pub_endpoints`.
5. **Port/protocol consistency is otherwise good.** 50256 (command), 50264 (chunk frame), 50356/50366/50376/50378 (state), 50410/50420 (gripper), 5600 (camera ZMQ), 8080 (gui), and 50261 (overlay) agree across the stack configs and launch scripts. (The former server↔software-simulator TCP 50200/50201 ports were removed with that lane.)
6. **GUI client-side execution lock is retired; server real-motion authority is config-driven.** `rb_gui` `safety.py` (`blocked_reason`, `tcp_command_disabled_reason`) no longer blocks real-mode commands or gates on a mode/backend match, and the old `RB_GUI_ENABLE_*` opt-in env flags are gone — controls emit in every run mode. The legacy **server** `RB_ALLOW_REAL_ROBOT/MOTION/CARTESIAN` env gates were also removed from the runtime (`config.cpp` "Real/sim env gates retired"); real motion is now decided by site-local config (`cartesian_control.allow_in_real: true`) + the mode-independent server safety layers. Operator supervision remains physical procedure. Don't read "GUI lock removed" as "real-motion authority removed" — `rb_servo_server` still makes the final allow/deny decision. Older `rb_servo_server/docs/gui_operator_console.md` prose that says "real motion ... disabled in the GUI / connect-status only" predates this and has been corrected.
7. **Unified collision URDF is an untracked external asset.** The self-collision `CollisionMonitor` loads `safety.self_collision.mesh.unified_urdf` — a stand+both-arms URDF (`mo_robot_descriptions/.../dual_rb3_730e_ver3.urdf`) that lives in the **separate, non-submodule** `mo_robot_descriptions` repo and is **not** git-tracked here; only the single-arm `rb_servo_server/descriptions/urdf/rb3_730e.urdf` (used by `kinematics.urdf`) is tracked. Provenance/version-pin/regeneration of the dual URDF is documented in `docs/frame_contract.md` → "Collision Geometry Asset (unified URDF)".
8. **Doc precedence.** Per `AGENTS.md`, when docs conflict the order is `AGENTS.md` → root `README.md` (KO; `README.en.md` is the English original) → `docs/architecture.md` → contract docs → component READMEs. `REVIEW.md` is a snapshot; `docs/current_review.md` is a redirect to `REVIEW.md`; `docs/archive/**` is audit-only.
