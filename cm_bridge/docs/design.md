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
| 2 | ROS 2 runtime for controller-manager. **RESOLVED 2026-08-18: NATIVE, Humble on this 22.04 host** (`tools/cm_local_setup.sh`; the docker path is retired — see D1). Deployment target stays Jazzy/24.04. | Done |
| 3 | A platform `active.yaml` for this rig. **Done 2026-08-18**: `cm_bridge/config/monkey/` is our own platform directory (device file installed there from the tracked template `active.monkey.real.yaml`; params-tasks/params-presets resolve from it — D4). | Done |
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
The legacy `submodules/mo_forcecontroller` tree was removed on 2026-08-16;
historical force-experiment code lives in git history, and controller-manager's
Admittance path is the sole forward direction.

## 7b. Observability: recorder schema 4 + sidecar + cm_replay (2026-08-18)

For the follow-fidelity / gripper-timing analysis (§`docs/replay.md`): controller-manager's
`func write` capture gained schema-4 columns (`mono_ns`, `fol_*` follow telemetry, `dev_*`,
`ext0..7` external scalars — `cm_bridge/upstream/0001-*.patch`, PR pending), the bridge stamps
`cmd/follow` with CLOCK_MONOTONIC and publishes gripper cmd/fb + chunk seq on
`<side>/cmd/ext_scalars`, and keeps an always-on JSONL sidecar (chunk content + gripper events on
the same clock). `cm_bridge/tools/cm_replay.py` replays both in 3D at 2 ms. Gate:
`make cm-record-gate` (isolated beside a live stack).

## 7c′. Time-stretch: no step is ever cut (2026-08-19, later the same day)

**Decision (operator).** Chunk fidelity first: a policy step's delta must be executed whole, at
no more than the envelope, taking as long as that needs — and its gripper target must fire at
the position the step actually reached. FollowUnit's own rule (cut a delta above `max_vel*T`,
play in one period) is therefore never allowed to bite: the bridge SUBDIVIDES every step into
`n = max(ceil(|dt|/(v_max T)), ceil(|dr|/(w_max T)), 1)` equal sub-deltas (envelope read from the
controller's own follow.yaml, fail-closed) and rides the step's gripper target only on the last
sub-delta (aux1 = policy step index). Hand-over to the newest candidate happens only at a
policy-step end that completes N=4 policy steps, sliced by the number of policy steps the
controller STARTED since that candidate arrived (the runner anchored it at the arm's command
pose) — the runner's own step counter is no longer used, so stretch, starvation and drift all
fall out of the same count. The controller runs `commit_steps: 1` in this mode (adoption
timing is the bridge's; it publishes only at hand-over points; a mid-step adoption is logged
as an error = "commit_steps 1 not loaded"). A runner Hold with the stream stopped publishes
one zero delta so the tail of the last chunk is not played out. Gate: `make cm-record-gate`
(two speed phases, the second 2.4× above the envelope): 4 policy steps per adopted message,
3 sub-deltas per fast step, gripper once per step arrival, **cmd chain moved 292.6 mm =
planned 292.6 mm** (a cut would have lost ~58 % of the fast phase); late-inference drill PASS.

## 7c. The commit structure: controller-paced N-step commit + gripper on the step (2026-08-19)

**Decision (operator).** FollowUnit no longer REPLACEs a chunk mid-window: it plays exactly N
deltas of an adopted chunk before it looks at the slot again (`follow.yaml commit_steps: 4`,
upstream patch 0002), and it reports every period boundary on `<side>/act/follow_step`. The
bridge (`FollowPacer`) keeps the newest runner chunk per arm and, one period before each commit
boundary, publishes it SLICED so its first delta is the step the controller is about to play
(`skip = S - chunk_metadata.activation_step_seq`); a late inference means the previous chunk is
continued (`skip 4`) and the newest one is re-sliced at the next boundary. The runner keeps
streaming inferences unchanged (RTC frozen prefix = N keeps consecutive plans continuous;
`prefetch_at 0`, `execute_steps 4`); it now publishes ALL remaining rows (runway 20) and its
per-step deltas (`left_delta`, delta 0 included) are what the bridge sends — the row-difference
path that lost delta 0 is a warned fallback. The per-step gripper target rides the chunk
(`cmd/follow_aux`, same stamp) and is applied when the controller reports THAT delta finished
(`prev_aux` on the event) — non-blocking, the arm never waits for the gripper; the runner's own
gripper dispatch on the command channel is ignored (`--gripper-source follow_step`).

Why: with the runner and controller on unlocked clocks, every hand-over skipped or replayed one
step (the row the policy meant was never landed on) and the gripper closed a step or two before
the arm arrived. Verified in SILS (`make cm-record-gate`, incl. `RECORD_GATE_LATE_EVERY=5`):
4 deltas per adopted chunk, adopted slices contiguous, gripper commands = step events.
`--follow-mode replace` keeps the old behaviour for A/B.

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
- **ROS 2 runtime on the robot PC** — CLOSED 2026-08-18: native Humble on
  22.04 (`tools/cm_local_setup.sh`). Native Jazzy still needs the 24.04 OS
  decision, which stays out of scope here.

## 10. P1–P3 implementation plan toward `make run MODE=real` (2026-08-16)

### Decisions (settled)

- **D1 Bridge runtime = Python (rclpy), NATIVE alongside the controller**
  (REVISED 2026-08-18; was "inside the CM docker image"). The docker path is
  retired: the RT pins are hardcoded in the submodule (`src/arm/Arm.cpp`,
  `Right ? 2 : 1`) and want the host's isolated cores directly, every override
  rode a single-file bind mount whose inode a rename-on-save editor replaces
  under the running container (the 2026-08-17 zero-compliance day), and the
  container wrote root-owned artifacts into the submodule tree. Both processes
  source the same `env.sh`, so they share one RMW (Fast DDS, shm-only profile). 30 Hz × small messages needs no C++;
  the collision gate runs pinocchio+hpp-fcl Python bindings at 30 Hz (33
  geom / 337 pairs is sub-ms). Port to C++ only if measured necessary.
- **D2 Delta conversion is frame-independent.** A follow delta is the LOCAL
  relative transform between consecutive absolute chunk rows
  (T_k^-1 * T_k+1): no stand<->CM-root extrinsic enters the DELTA path.
  What must agree is the TOOL POINT the rows and CM describe (R1 audit:
  FollowUnit delta frame vs our TCP; fix by selecting the matching CM tool
  preset or transporting deltas across the known tool offset).
- **D3 State republish needs the extrinsic.** `servo_state.v1` carries
  stand-frame absolutes -> map CM-root poses using robotics_lab calibration.
  Field set = audit of what policy_runner/rb_gui actually read.
- **D4 follow tuning without editing CM — REVISED 2026-08-18, now a PLATFORM
  DIRECTORY, not a mount.** `params-tasks/follow.yaml` is TRACKED in the
  submodule (read-only rule applies), but CM resolves BOTH its task params and
  its device presets relative to the LOADED active.yaml's own directory
  (`TaskConfig.cpp:44-50` checks `<active.yaml dir>/params-tasks` FIRST via
  `Arm.cpp:438`; `Config.cpp:496` sets `dir = cfg.parent_path()` for
  `params-presets/{models,tools,efts}`), and `env.sh` honours a pre-set
  `CONTROL_MANAGER_ACTIVE_YAML`. So `cm_bridge/config/monkey/` IS our platform
  directory: our `follow.yaml` and our measured `tools/pika.yaml` are real
  files, and everything we do not override is a SYMLINK to the submodule (no
  copies -> no drift on pull; `run_cm_stack.sh` refuses to launch on a dangling
  or empty one, because a missing task file only makes CM WARN and run
  compiled defaults). **The directory must satisfy MORE than the controller's
  own loader.** `core/Config.cpp` reads exactly three preset classes (models,
  tools, efts), but the device file also carries `stand: "stands/<x>.yaml"`,
  which the controller IGNORES and the COCKPIT's URDF composer resolves
  (`compose_urdf.py resolve_descriptor`). Deriving the link set from
  `Config.cpp` alone therefore shipped a directory that booted the controller
  clean and left the cockpit with an empty 3D pane and
  `composed.urdf is not valid XML`. `check_params_presets()` now also verifies
  every PATH-STYLE (`*.yaml`) reference in the device file, by the composer's
  own two-place rule. `cm_bridge/config/monkey-sils/` is the SILS twin,
  sharing the same params by one link each. This replaced the single-file
  bind mount, whose inode a rename-on-save editor silently replaced under the
  running container (the 2026-08-17 zero-compliance day). Optional later:
  upstream PR (owner's call).
- **D5 P1 MVP scope**: policy chunk stream + episode reset (command JSON
  `JointTarget`/`init_motion` -> CM `Move` action MOVJ) + gripper (existing
  UDP path, unchanged) + state republish. SpaceMouse/UMI teleop deferred.
- **D6 Single controller owner**: `run_cm_stack.sh real` will launch CM +
  bridge + rb_gui + policy_runner + gripper bridge and refuses while
  rb_servo_server runs (already enforced in sim).

### P1 — bridge command path (SILS, no hardware)

1. `cm_bridge/src/cm_bridge_node.py`: chunk UDP 50264 ingress (wire format
   per `docs/code_architecture_map.md`), absolute-row -> per-period delta
   conversion, `/monkey/<side>/cmd/follow` PoseArray chunks (~4 deltas,
   REPLACE cadence), command JSON 50256 subset (reset/init -> MOVJ), CM
   state audit + subscribe -> `servo_state.v1` UDP fanout re-publish.
2. `cm_bridge/config/monkey/` platform directory (D4) — our `follow.yaml`
   + measured `tools/pika.yaml`, the rest symlinked to the submodule.
3. `cm_bridge/tests/cm_sils_gate.py`: scripted 30 Hz stream — drop-one
   (stays in Follow), mid-tail REPLACE (re-anchors), silence exit timing,
   state-fanout field assertions. Wire as `make cm-sils-gate`.
4. Exit: rb_gui renders CM-driven state; synthetic chunks track in SILS.

### P2 — safety gate port

- Pinocchio-python collision monitor loading rb_servo_server's URDF/geom
  set; gate rows pre-publish; latched fault stops streaming and surfaces on
  the fanout. Floor-plane FK check at the same gate. Exit: canned collision
  trajectory trips fail-closed in SILS.

### P3 — real hardware

- Fill `platforms/monkey/active.yaml` (gitignored device file): box IPs
  .200/.201 + serials, pika tool preset, FT preset (RFT64-6A01-A) when
  force control turns on.
- **Mount reconciliation**: CM monkey preset carries the FIRST-SUPPLY
  inverted/symmetrized mounts; robotics_lab has its own
  `calibration/active_calibration.yaml` + `stack_real.yaml` mounts. Compare
  numerically; the delta path tolerates offsets but state absolutes and
  collision geometry must use one truth.
- Supervised ladder: enable/idle only -> single-arm low-envelope follow ->
  dual-arm -> policy rollout A/B vs `CONTROLLER=legacy`.
- Unlock: flip `run_cm_stack.sh real` from fail-closed to launch once P1+P2
  gates are green and the device file is filled.

### Risks

- R1 follow delta frame semantics (flange vs tool) — audit first, day 1.
- R2 which CM topics expose q/tcp state at usable rate — audit
  (`CellView`/`Health`/per-arm topics); worst case add a bridge-side poll.
- R3 `servo_state.v1` field completeness for flow-infer (silent breakage)
  — mitigated by the field audit + gate assertions.
- R4 overlay-mounted follow.yaml drifting on submodule pulls — the SILS
  gate is the canary.
- R5 stand<->CM-root calibration mismatch (P3 measurement).
