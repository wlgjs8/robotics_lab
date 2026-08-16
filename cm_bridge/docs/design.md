# cm_bridge design — robotics_lab × controller-manager combined control stack

Status: DRAFT v0 (2026-08-16). Owner: robotics_lab. Companion repo:
`submodules/controller-manager` @ pinned SHA (consumed read-only).

## 1. Goal and non-goals

**Goal.** Route the robotics_lab policy/teleop command stream through
controller-manager's arm control (servo_j with LPF off on firmware 26071103,
qsync queue regulation, force control), while keeping robotics_lab's
self-collision gate and the entire upper stack (policy_runner, camera_server,
rb_gui) unmodified. controller-manager must stay `git pull`-updatable at all
times.

**Non-goals.** No reimplementation of controller-manager internals in this
repo; no edits inside the submodule; no waist/body axis support in the first
iteration (robotics_lab is a dual RB3-730 rig — the "monkey"-like
configuration with `body.joints: []`).

## 2. Decision record

- **Process boundary, not code extraction.** controller-manager runs untouched
  as its own process (ROS 2). The bridge speaks only its external interface
  (`cell_msgs` actions/services/messages + whatever streaming input §5 lands
  on). Rationale: controller-manager internals (Controller.cpp, Arm.cpp) churn
  under active development; extracting RobotLink/qsync as a library would break
  on every pull. Interface-level coupling is the only shape compatible with
  "pull and go".
- **Submodule pin + SILS gate** as the update mechanism (§8).
- **CollisionMonitor moves into the bridge** (robotics_lab-owned code, free to
  extract) and gates the stream BEFORE it reaches controller-manager. Same
  fail-closed posture as today.
- **rb_servo_server is kept in-tree during the transition** so old/new
  controller stacks can be A/B'd on the same task with the same telemetry
  (`make run` switch, exact mechanism TBD in P3).

## 3. Preconditions

| # | Precondition | Status |
|---|---|---|
| 1 | RB control boxes on firmware build **26071103** (label 8.7.3). controller-manager refuses to init otherwise (`Arm.cpp:48-60`). | **Done** — operator updated both boxes 2026-08-16 evening, version confirmed by operator. Bridge still re-verifies via CM's own init gate at first bring-up. |
| 2 | ROS 2 runtime for controller-manager (Jazzy native on 24.04, or the repo's Humble Docker path). Robot PC is Ubuntu 22.04 → start with the **Docker path** (`controller-manager/Makefile` + `docker-compose.yaml`), revisit native later. | Open |
| 3 | A platform `active.yaml` for this rig under `controller-manager/platforms/` conventions (device file, gitignored there). Our copy lives in `cm_bridge/config/` and is installed/mounted at launch. | Open (P1) |
| 4 | GitHub access to `PLAIF-dev/controller-manager` from every clone of robotics_lab. | Done (submodule added) |

## 4. Architecture

```
policy_runner (flow-infer, 30 Hz chunks)          rb_gui / recorders
        │ UDP 50264 chunk frames                        ▲ state fanout 50356/66/76/78/86
        │ UDP 50256 command JSON                        │ (schema robotics_lab.servo_state.v1)
        ▼                                               │
┌──────────────────────────── cm_bridge ────────────────┴──────────┐
│ ingress: chunk frames + command JSON (same wire contracts        │
│          rb_servo_server accepts today — upper stack unmodified) │
│ gate:    CollisionMonitor (async URDF-mesh, fail-closed)         │
│          + floor-plane / tracking guards ported as needed        │
│ egress:  controller-manager streaming input (§5)                 │
│ state:   CM feedback → servo_state.v1 fanout re-publish          │
│ force:   FtConfig / Wrench / AxisCompliance pass-through         │
└───────────────┬──────────────────────────────────────────────────┘
                │ ROS 2 (cell_msgs) + streaming input
                ▼
   controller-manager (UNTOUCHED submodule process)
   per-arm RT threads · qsync FLL (fill=5) · servo_j t1=2ms t2=21ms
   gain=1.0 alpha=10 (LPF off) · %.7f deg · firmware gate 26071103
                │ rbpodo 5000/5001 per arm
                ▼
        RB control boxes (fw 26071103)
```

Wire contracts the bridge must honor (so policy_runner/rb_gui stay
unmodified) — sources of truth in `docs/code_architecture_map.md`:

- Ingress: chunk frames UDP 50264 (`ChunkWindow` wire format incl.
  `policy_dt`, execute/runway steps), command JSON UDP 50256
  (`{seq, mode, left{…}, right{…}}`).
- Egress state: `robotics_lab.servo_state.v1` on the configured fanout ports.
  The bridge must populate the fields policy_runner actually consumes
  (q, tcp poses, validity, fault flags, seq/ack echo) — audit in P1.
- Gripper 50410/50420 stays on its existing path (grippers are not
  controller-manager's concern on this rig; unchanged).

## 5. RESOLVED — streaming input = `FollowUnit` (code analysis 2026-08-16)

Candidates were analyzed read-only at pin 41de741. Verdicts:

- **`FollowUnit` (`task Follow`) — SELECTED.** External topic
  `/chimpanzee/<side>/cmd/follow` (`geometry_msgs/PoseArray`); each Pose is a
  **per-period FLANGE-FRAME DELTA** (position m, orientation quaternion ->
  rotvec at the IPC boundary, `ControllerIpc.cpp:552-574`). Latest-value
  seqlock slot, chunk <= 64 (config cap 50): **REPLACE-at-boundary is the
  native contract** (`Channels.h:596-608`, `Tasks.cpp:3198-3213`) -- exactly
  our receding-horizon chunk semantics. Timing: one delta per
  `input_period_ms` (default 20 ms -> set **33.3** for our 30 Hz),
  `playback_margin 0.2*T` jitter buffer; a missed boundary commands zero
  delta and STAYS in Follow; silence exit only after `silence_periods*T`
  (2T) AND at rest (`Tasks.cpp:3367-3397`). Auto-enters Follow from
  OnTask+Idle on the first fresh chunk. Covered by `follow_unit_test`,
  `follow6_test`, SILS probe `verification/sils/follow6_stream.py`.
- `MovF/MovH/MovHF` -- one-shot register->replay units (action `Move` +
  `Sync` release), no append, never retargeted mid-run. Infeasible.
- `StreamSegment`/`StreamPool` (Design-B) -- internal cell->arm transport,
  production-gated off (`ENABLE_CELL_STREAM = false`), no external ingress.
  Do not target.
- `StreamFollower` (`ChunkPacer`) -- helper inside the units; the Cartesian
  half is test-only (production FollowUnit uses Ruckig `CartChannel`s). Not
  an ingress.

**Fortunate alignment:** flow-infer's native action space is already
per-step ee_local deltas -- the bridge translates delta-to-delta (frame
convention + quaternion encoding audit in P1), not absolute-to-delta.

Bridge obligations discovered with the selection:

1. **Envelope**: shipped follow envelope is `max_vel_mms 50` / `max_rot_dps
   10`, clipping oversize deltas with a WARN. Our task performs ~64 deg/s
   yaw maneuvers -> the rotation envelope must be raised in our platform
   `follow.yaml` (config-only, live-retunable via edit+`reset`), re-deriving
   `cart_acc_mms2` per that file's own `sqrt(v*j)` zero-margin note.
2. **`admittance_overlay: true` by default** on the follow path -- disable
   for pure position replay until force control is deliberately enabled
   (section 7); contact force on this path is currently unbounded upstream.
3. **Entry/exit choreography**: bridge drives `enable -> task on -> task
   idle` before streaming; while another unit runs, follow deposits are
   discarded. Singularity guard = full brake to Enabled and the stream stops
   being read; re-entry needs `task on -> task idle` -- the bridge must
   detect this and surface it fail-closed.
4. `input_period_ms 33.3` + keep `silence_periods >= 2` so one late ~33 ms
   row can never exit the unit.

## 6. Open item #2 — CollisionMonitor port

Today: async self-collision guard inside rb_servo_server (URDF-mesh,
`CollisionMonitor`), enforced at the servo loop. In the combined stack the
bridge owns it, checking the **outgoing stream** (post-IK joint targets or
FK-checked Cartesian rows) before egress.

Port questions for P2: (a) the monitor consumes joint-space q — the bridge
must run IK (or receive q rows) before the gate → decide whether the bridge
adopts rb_servo_server's IK stack (Pinocchio, own code, extractable) or gates
in Cartesian via conservative capsule margins; (b) latency budget — async
monitor with latched fault matches today's posture; (c) fault propagation —
on trip, stop streaming (controller-manager falls back to its own safe hold /
Idle transition) and surface the fault on the state fanout.

## 7. Force control

controller-manager provides Admittance (task) with `FtConfig` / `Wrench` /
`AxisCompliance{,Set}` messages. Bridge exposes a minimal pass-through first
(enable/disable + parameter set + wrench telemetry on the state fanout).
robotics_lab's own force stack (`mo_forcecontroller` experiments) stays
untouched as reference until parity is verified.

## 8. Submodule update gate (SILS)

```
cd submodules/controller-manager && git pull origin main && cd -
make cm-sils-gate        # (P1) bridge integration test against CM SILS/fakebox:
                         #  - boots CM with SILS box model (firmware answer "SILS")
                         #  - streams a canned 30 Hz trajectory through the bridge
                         #  - asserts: continuity, queue fill telemetry sane,
                         #    state fanout fields populated, collision gate trips
                         #    on a canned self-collision trajectory
git add submodules/controller-manager && git commit -m "chore: bump controller-manager to <sha>"
```

A pin bump commit without a green gate is not mergeable. The gate is also the
canary for interface drift: if `cell_msgs` or the streaming input changed
upstream, the gate fails here, not on the robot.

## 9. Phases

| Phase | Deliverable | Exit criterion |
|---|---|---|
| **P0** | submodule + this skeleton/design (this commit) | merged |
| **P1** | streaming-input probe in SILS; bridge minimal path (chunk ingress → CM SILS → state fanout re-publish); `cm-sils-gate` target | canned 30 Hz stream tracks in SILS; fanout consumed by rb_gui |
| **P2** | CollisionMonitor gate port + fault propagation | canned collision trajectory trips fail-closed in SILS |
| **P3** | real hardware bring-up (fw 26071103 verified), small supervised motions; `make run` controller switch | supervised MovJ + short stream on real arms |
| **P4** | policy rollout A/B: rb_servo_server vs cm_bridge, same task/telemetry | paired rollout report |

## 10. Risks

- **Streaming input mismatch** (§5) — highest risk; mitigated by SILS probe
  before any hardware time.
- **Interface drift on pull** — mitigated by the SILS gate (§8).
- **State-fanout fidelity** — policy_runner consumes specific
  `servo_state.v1` fields (reset anchors, tcp_*_stand, seq echo);
  an incomplete re-publish breaks flow-infer silently. P1 includes a field
  audit against `policy_runner`'s reader.
- **Two controllers, one robot** — rb_servo_server and controller-manager must
  never stream simultaneously; the `make run` switch must be mutually
  exclusive by construction (single owner of the rbpodo sockets).
- **ROS 2 runtime on the robot PC** — Docker path first; native Jazzy needs an
  OS decision (24.04) that is out of scope here.
