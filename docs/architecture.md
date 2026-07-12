# robotics_lab Architecture

This document is the current source of truth for the system architecture. Component READMEs may contain local details, but public terminology, safety boundaries, and topology must match this document.

## Current Phase

The repository is currently in **rbpodo pgmode-real physical robot bring-up**.
Simulator-first Cartesian acceptance hardening is largely complete and now serves
as the regression baseline; active validation has moved onto the physical
RB3-730E hardware.

The mock / rbpodo controller-simulation (pgmode) stack remains the regression baseline for:

- structured backend result and fault telemetry
- `JointTarget`
- `TcpPoseTarget`
- `TcpLinearMove`
- GUI operator controls
- policy_runner SpaceMouse command paths
- command-source lease/arbitration
- camera readiness contracts for future policy work

Real motion is now an active bring-up lane (see Maturity Boundary), not a
deferred milestone. Execution authority is config-driven and server-owned:
the tracked stack config plus the mode-independent safety layers decide whether motion
is sent. Operator supervision and an E-stop remain physical operation procedure,
and passing simulator acceptance is never permission to move hardware.

## Maturity Boundary

Supported for mock / controller-simulation work:

- mock dual-arm servo control
- direct and worker I/O modes (mock / hardware-free)
- FK/TCP state publication with quaternion fields
- Cartesian PTP and Linear commands
- mandatory Eigen3/Pinocchio-backed Cartesian math in `rb_servo_server`
- mock camera server
- GUI viewer/operator console for mock/simulation
- Python policy_runner with joint and Cartesian action sources

Run / validated on pgmode-real (physical RB3-730E hardware):

- read-only physical diagnostics parity against controllers `.200`/`.201`
  using `tcp_actual_stand` (not `tcp_ref_stand`)
- dual-arm physical Cartesian circle tracking — slow, TUNED-1 profile, median
  tracking ~1.42° (`docs/runbooks/rbpodo_real_physical_circle.md`)
- UMI dual-arm Cartesian teleop (relative-init) driving `TcpPoseTarget` on the
  physical arms; UMI `data_tcp` replay verified on hardware (ee_local + r_align)
- `flow-infer` `real_policy` full closed-loop rollout on the physical robot
  (pi0.5/openpi): `TcpPoseTarget` + gripper commands. The `real_policy`
  rollout-mode gate stays fully enforced (`_validate_real_policy`: `mode=real`,
  `allow_real_motion`, measured/accepted geometry + retarget, validated collision
  model, gripper gate) and was satisfied via accepted/validated runtime config —
  the lane is open and exercised, not blocked. Runtime is validated (smooth,
  in-distribution; async chunking decouples ~30 Hz policy from the 500 Hz servo;
  the absolute-proprio frame gap is fixed by reset-relative retrain). Task success
  is still model-limited (see below). ee_local action deltas are composed at
  runtime into absolute `TcpPoseTarget` setpoints.
- real gripper motion via the Pika Gripper Backend, gated by `RB_ALLOW_REAL_GRIPPER`
  + `measured_gripper_available` + `allow_real_gripper_motion`
- server-side async URDF-mesh self-collision guard (`CollisionMonitor`) enforced
  in real motion via a velocity barrier; stale / hard-breach fail closed.
  Structural false positives are curated by exact/glob
  `safety.self_collision.mesh.disabled_collision_pairs`, not by lowering global
  hard-distance or planner margins.
- policy-side real-Cartesian safety gate relaxation (PR #13): `rb_servo_server`
  is the sole real-motion safety layer; controller-simulation safety unchanged
- controller `-2001` suspect-diagnostics acceptance in real mode (PR #12) with
  EMS/SOS/soft-estop/`collision_occur`/unknown-mode/init-error still latching

Not yet production-ready:

- policy task success — rollout motion is smooth but inaccurate (model quality /
  data coverage / appearance-domain gap, not runtime); init-pose distribution
  matching is in progress
- force control
- fast physical circle stages (15 cm / 16 s and above, transition ladder P7–P9)
- measured camera/robot calibration remains `configured_estimate` and is still
  required for general geometry-dependent policy, but is not needed for the
  deployed pika Sense≡Gripper + ee_local + image-conditioned policy (reset-relative
  cancels the steamvr→stand transform; the tool offset is a known constant)

## Canonical Terms

Use only these values in public config, docs, GUI labels, and operator-facing logs:

```yaml
run_mode: mock | simulation | real
backend_type: mock | rbpodo
```

`run_mode` describes the environment. `backend_type` describes the backend implementation. Removed simulator backend aliases or mixed simulator terms must not be introduced in new public docs/configs.

Supported real-controller scope is rbpodo only. The `MockBackend` is the
hardware-free validation surface; the old software-simulator backend and
unsupported raw script TCP comparison paths are removed and must not be
presented as runnable backends.

### Simulation flavors (name them precisely)

"Simulation" now means exactly one thing — rbpodo controller `pgmode`
simulation. The former software-simulator flavor (`run_mode: simulation`,
`backend_type: simulator`) was removed. Name the remaining flavor by its
canonical config keys, never by the bare word "simulation":

| Flavor | Canonical name | `run_mode` | `backend_type` | `operation_mode` | Connects to |
|---|---|---|---|---|---|
| Controller simulation | rbpodo `pgmode` controller-sim | `real` | `rbpodo` | `simulation` | a real rbpodo controller running in `pgmode` |

Controller simulation is `run_mode: real` (it really connects to a controller)
with the controller's `operation_mode: simulation`.

Within controller simulation the **target** may be a Virtual ControlBox **VM** or
a **physical** controller box in `pgmode`. These are behaviourally identical to
the server — same `rbpodo` backend, same `operation_mode: simulation`, same
config-driven carve-out — so they must NOT get new `run_mode`/`backend_type`
values. Distinguish
them only by deployment target, via config filename suffix and docs:

- `…controller_sim_vm.yaml` — target is a Virtual ControlBox VM (no physical hardware on the wire)
- `…controller_sim_onbox.yaml` — target is a physical controller box held in `pgmode`

Launch settings live only in the tracked `stack_real.yaml` and `stack_sim.yaml`.

## Controller Topology

The physical system has one controller endpoint per arm:

```text
rb_servo_server
  left_robot  backend_type=rbpodo -> 172.28.60.200
  right_robot backend_type=rbpodo -> 172.28.60.201
```

The rbpodo controller `pgmode` simulation reuses this same per-arm rbpodo
endpoint shape, but targets a Virtual ControlBox VM or a physical box held in
`pgmode` (configured by the tracked simulation stack),
distinguished only by deployment target.

## State Publication Fanout

`rb_servo_server` is the owner of UDP state fanout. Multi-consumer configs use
the canonical list field:

```yaml
network:
  state_pub_endpoints:
    - "udp://127.0.0.1:50151"  # benchmark recorder
    - "udp://127.0.0.1:50161"  # rb_gui live viewer
```

The deprecated single-consumer `state_pub_endpoint` remains accepted for
legacy configs, and the first list entry is mirrored into that field for older
tools. Do not configure both fields together. Benchmark recorders and GUI
viewers should bind separate UDP ports and receive identical server-published
state JSON; benchmark tee/rebroadcast processes are not the primary state path.
Command traffic still goes directly to `network.command_bind`.

## Safety Gates

Real behavior is fail-closed and never implicit, but it is **no longer gated on
env vars**. The legacy execution gates — `RB_ALLOW_REAL_ROBOT`,
`RB_ALLOW_REAL_MOTION`, `RB_ALLOW_REAL_CARTESIAN`,
`RB_ALLOW_RBPODO_ACK_DISABLED_MOTION`,
`RB_ALLOW_RBPODO_SUSPECT_DIAGNOSTICS_REAL_MOTION`,
`RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION`,
`RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN`,
`RB_RBPODO_PGMODE_SIMULATION_CONFIRMED` — were removed from the server runtime.
`run_mode`/`operation_mode` are telemetry labels only and do not decide whether
motion is allowed.

Real-motion execution authority is owned by **the tracked stack config + the
mode-independent safety layers**. Real motion requires `stack_real.yaml` to enable it explicitly
(`cartesian_control.allow_in_real: true`). Operator-supervised acceptance remains
the physical operating process. This config-driven path has already carried a
supervised dual-arm physical Cartesian circle
(`docs/runbooks/rbpodo_real_physical_circle.md`).

The policy-side `SafetyGate` no longer blocks real Cartesian motion (PR #13);
stale state, fault, camera, and kinematics readiness checks remain client-side.
For real Cartesian motion, `rb_servo_server` makes the final allow/deny decision:
safety filter (dq/ddq/joint limits), tracking-error fault latch, the async
URDF-mesh self-collision guard (`CollisionMonitor`), lease arbitration, and
deadman. Mesh
self-collision pair curation is SRDF-style: disabled pairs remove only named
geometry pairs from the monitor/planner oracle, while non-disabled intra-arm,
arm-arm, arm-stand, floor, and external pairs keep the configured margins.
EMS/SOS/soft-estop/`collision_occur`/unknown-mode/init-error continue to latch
regardless of config. Controller-simulation safety is unchanged.

Accepting the controller `-2001` suspect diagnostics
(`op_stat_self_collision`/`robot_time` field-layout garbage) in real mode is a
per-arm config opt-in (`allow_real_motion_with_suspect_diagnostics: true`, no
env); EMS/SOS/soft-estop/`collision_occur`/unknown-mode/init-error still latch.

Rainbow controller `pgmode` simulation through the `rbpodo` backend is a
separate evidence category from both hardware-free mock and physical real
motion. It connects to real controller boxes, so configs use
`run_mode: real` and `backend_type: rbpodo`, but each robot must use
`operation_mode: simulation` and the controller must be confirmed in `pgmode`
simulation. Controller-simulation circle tracking should use
`tcp_ref_stand`/controller reference telemetry with
`physical_motion_expected=false`.

The narrow rbpodo controller-simulation streaming Cartesian carve-out is
config-driven (no env):

```yaml
cartesian_control:
  allow_in_controller_simulation: true
  allow_in_real: false
servo:
  allow_controller_simulation_motion: true
```

This carve-out does not approve physical real Cartesian motion; the config must
explicitly keep `allow_in_real: false`. Servo J ACKs in a controller-simulation
circle artifact do not by themselves prove that Cartesian commands executed;
check the Cartesian gate telemetry and `tcp_ref_stand` movement.

### Floor plane constraint (`safety.floor_constraint`)

A stand-frame keep-out plane for the TCP: when enabled, neither arm's TCP may be
commanded below `z = z_min_m` (default 0.010 m), regardless of motion primitive
and regardless of run mode (mock, controller-sim, and real all pass
through the same gate). It is enforced in two tiers:

- **Tier 1 (hard backstop, all primitives)**: at the final per-tick safety gate
  (`DualArmServoLoop::applySafety`), each arm's candidate joint target is
  FK-checked; a target whose TCP z falls below the plane reverts that arm to its
  last safe target (`fail_policy: clamp_hold`, non-latching) or latches a
  `FloorViolation` fault (`fail_policy: fault_latch`). FK failure fails closed.
  A candidate that strictly raises the TCP while already below the plane is
  allowed (escape), so an arm that starts below the plane can be jogged out
  without a fault reset.
- **Tier 2 (Cartesian sliding assist)**: absolute Cartesian targets
  (`TcpPoseTarget`, `TcpLinearMove`) have their stand z clamped to the plane
  before IK, so lateral teleop/policy motion slides along the plane instead of
  stuttering against the Tier-1 hold. Joint-space primitives get no Tier-2
  assist (Tier-1 hold only).

When `tcp_offset_points` is configured, floor evaluation checks the TCP plus
named local-frame offsets. The current local PIKA gripper stack configs check
`gripper_tip_a` and `gripper_tip_b` at TCP-frame x offsets of `+0.059 m` and
`-0.059 m`, based on a measured `118 mm` tip-to-tip width. These points are used
by both the Tier-1 FK backstop and the Tier-2 Cartesian streaming floor
projection.

The plane height is runtime-adjustable with the **leaseless** non-motion command
`SetSafetyFloorZ` (`{"mode": "SetSafetyFloorZ", "floor_z_m": <meters>}`):
raising the floor is safety-tightening and must never be blocked by a teleop
client holding the command lease, and lowering is bounded server-side to the
config envelope `[runtime_min_z_m, runtime_max_z_m]`. Every accepted set is
logged with its `source_id`; the effective value, per-arm TCP z / violation
flags, and the last reject reason are published every state tick under
`floor_constraint`. `monitor_only: true` publishes telemetry without clamping
and is never a real-motion safety posture. Enabling the constraint requires
`kinematics.enable: true` (TCP FK source) — enforced at config load.
The GUI initializes its floor-enforcement checkbox from this published server
state and does not restore a prior GUI-side disable across launches; the tracked
stack config remains the startup authority. An operator may still change the
runtime state explicitly during the current supervised session.

### Reachable-workspace shell constraint (`safety.reach_constraint`)

The radial generalization of the floor/ROI keep-out, centered on each arm's
shoulder: when enabled, the TCP (and each configured `tcp_offset_points`) must
stay inside the stand-frame spherical shell `r_min_m <= ||tcp − arm_base|| <=
r_max_m`, where `arm_base` is the mount origin (`left_mount`/`right_mount`
`.base_pose_in_stand`). It bounds how far a Cartesian command can drive the TCP
from the base so the controller never asks for a pose past the arm's reach —
the regime where IK fails (max iterations / joint limit) or hits a
full-extension singularity and the legacy behavior was for the arm to silently
**stop** at the boundary. It is enforced by the SAME unified Stage-3
velocity-damper projection as the floor plane and ROI box (`DualArmServoLoop`
`applySafety`): within `d_slow_m` of a shell, one closing-velocity row limits the
binding point's radial speed `d(r)/dt` to `±sqrt(2·a_brake·margin)` along that
point's radial direction (`computeStandDirectionJacobian`), so the TCP brakes to
zero radial speed AT the shell and is free to slide tangentially or return
inward. Outer shell binds the farthest checked point; inner shell binds the
nearest (`r_min_m <= 0` disables the inner shell). FK failure fails closed; a
candidate already outside that is not getting deeper outside is allowed (escape),
matching floor/ROI. `clamp_hold` slides at the shell (recommended); `fault_latch`
hard-latches on a PTP/jog push outside. Requires `kinematics.enable: true`. Each
tick the per-arm radial margins are evaluated at the safety gate and clamp
events are counted (`reach_clamp_count_`); a published `reach_constraint` state
block, mirroring `floor_constraint`/`roi_box`, is a follow-up.
`r_min_m`/`r_max_m` are measured with `tools/reach_envelope.py` (FK Monte-Carlo
of the URDF tip frame), which also exports the per-arm reachable-workspace
OUTER-SHELL surface mesh (per-direction max-radius, triangulated) that the rb_gui
viser overlay renders translucent + double-sided ("도달영역 표시" toggle) so the
operator sees the reach boundary through the robot/stand.

A complementary viewer-only overlay marks the **"A 영역" base-axis singularity
cylinder** — the column along each arm's J1 (base) axis where Move J reaches fine
but Cartesian/Move L (and streaming twist) control forces runaway joint speed. This
is the vendor's documented "A 영역" (rb_cobot_docs product_introduction/robot_workarea):
a *velocity/Jacobian* singularity, not a position hole (the TCP can be placed there
with Move J). It is deliberately distinct from the reach envelope: reach = "too far
to reach at all"; A 영역 = "reachable, but Cartesian motion through here saturates the
base joint". Geometry: a capped cylinder coaxial with the J1 axis, radius
`R = v_ref / dq_max` (v_ref = `smd.max_linear_velocity_m_s` = the binding SMD
pose-tracker Cartesian ceiling, dq_max = `safety.dq_max_deg_s[0]`; default ≈ 0.239 m
at 0.25 m/s — it GROWS with commanded speed), axial extent FK-clipped to the
reachable z-range. The cylinder is identical
in both arms' base frames (the singularity is mount-independent there), so one mesh
serves both; each `/stand/<side>_base` node applies the mount tilt. Built by
`tools/ik_infeasible_region.py` (pure Python, no C++ grid) into
`descriptions/ik_infeasible_rb3_730e.npz`. Static, precomputed viewer aid only (no
safety enforcement — a matching inner-cylinder velocity-damper guard is a possible
follow-up) — regenerate with `make ik-infeasible` (tunable `IK_CYL_SPEED` /
`IK_CYL_DQMAX` / `IK_CYL_RADIUS`) if the URDF, mount geometry, or speed cap changes.
Toggle: "A 영역(특이점 원통) 표시" in the 조작 → 안전 tab.

Tracked stack configs are the current launch source of truth:

```text
rb_servo_server/config/stack_real.yaml
rb_servo_server/config/stack_sim.yaml
```

Do not add parallel runnable real robot configs. Advance acceptance stages by
changing one reviewed setting at a time in `stack_real.yaml`.

The legacy `dual_real*.example.yaml` template surface is no longer tracked.

Rbpodo is the primary vendor-library real backend. Its Rainbow Servo J fields
must use canonical names in new configs:

- `servo_t1_sec` maps to `move_servo_j` `t1`
- `servo_t2_sec` maps to `move_servo_j` `t2`
- `servo_gain` maps to `gain`
- `servo_alpha` maps to `alpha`

Do not introduce new uses of deprecated aliases `servo_time_sec`,
`servo_lookahead_sec`, or `servo_acc`. `servo_t1_sec` must match the streaming
period for supported real/controller-simulation configs: `0.002` at 500 Hz.
Manual non-500 YAML overrides may remain parseable for compatibility, but they
are not supported profiles. ACK-off rbpodo settings are not a real baseline
until the supervised acceptance sequence passes with state-age, ACK, error-code,
and q_ref/q_actual evidence.

Supported 500 Hz Servo J configs pin `servo_t1_sec: 0.002`,
`servo_t2_sec: 0.021`, and `servo_gain: 1.0`. `servo_alpha` remains
script-level and is scaled by `0.1` inside the Rainbow controller:
`servo_alpha: 10.0` disables the controller LPF, while `servo_alpha: 1.0`
leaves an effective alpha of roughly `0.1`. The tracked physical-real stack uses
`servo_alpha: 1.0` because the LPF-off setting produced jerk/jitter on hardware;
controller-simulation diagnostics may still use `10.0` for a transparent
controller profile. The primary responsiveness/smoothness/accuracy tuning
surface is the server-side control loop (`TcpPoseTarget` →
`cartesian_control.pose_track_smd`) — see `docs/servo_backend_contract.md` →
"Servo J Streaming Profiles".

Deprecated simulator config names are archived under `docs/archive/configs/`
for historical reference only. They are not runnable source-of-truth profiles
and must not be used for new smoke or acceptance evidence.

Force control is integrated as a server-owned path: rbpodo EFT samples feed the
compensated wrench pipeline, contact guard, unilateral surface-normal
admittance, and bounded 6D Cartesian compliance before IK. In Cartesian
compliance mode, soft contact preserves the active flow-infer chunk and gripper
execution while loading translation/rotation increments are projected and a
bounded SE(3) correction is composed around a fixed Hold or latest accepted
policy-command equilibrium. Zero-wrench stiffness recenters all six offsets
without absorbing measured motion into that equilibrium. The current real
profile is a supervised translation-only Hold gate using `tcp_origin`: the
controller follows the accepted rbpodo EFT/TCP-axis orientation and applies the
correction about the TCP endpoint. Rotational axes remain disabled.
The Cartesian controller selects jerk from a recursively viable braking
envelope rather than integrating and clamping state. A same-direction wrench
outside an axis deadband reserves a zero-velocity, zero-acceleration loaded
hold without allowing the compliant offset to reverse toward zero; deadband
release restores the nominal stiffness-driven recenter path. Recontact during
that return is first jerk-bounded to a stop before the loaded-hold invariant
takes ownership. This preserves boundary braking, recontact, and release
recentering authority without adding wrench averaging.
Surface-normal and resultant hard-limit calculations remain in their existing
TCP/stand path. Hard-limit faults retain the normal
motion-epoch interruption path. Activation, telemetry, and promotion constraints are defined in
`rb_servo_server/docs/force_control.md`. The path is not safety-rated;
`stack_real.yaml` currently exposes a dual-arm supervised Gate 2 profile. Both
geometric floor constraints are disabled by explicit operator decision, so the
TCP/gripper-tip floor velocity damper and hard plane backstop are absent. This
is a physical bring-up configuration, not production acceptance.

```yaml
force_control:
  provider: project_native
  enable: true
  operating_mode: cartesian_admittance
  allow_in_real: true
  supervised_experimental_real: true
```

## Motion Primitive Contract

Every primitive below is additionally subject to the stand-frame floor plane
constraint when `safety.floor_constraint.enable` is set (see "Floor plane
constraint" under Safety Gates): the final per-tick joint target is FK-checked
and held/latched if it would put either TCP below the plane.

### `JointTarget`

Absolute joint-space target. This is a joint-space point-to-point command.

### `TcpPoseTarget`

Cartesian point-to-point final-pose target. It is MoveJ-like at the TCP level. Final TCP pose is targeted, but the intermediate TCP path is not guaranteed to be linear. Real mode is open through the real-mode gates plus `cartesian_control.allow_in_real: true`, and has been validated on the dual-arm physical Cartesian circle.

`policy_runner flow-infer` composes each ee_local per-step policy delta onto
the measured or running TCP pose and emits absolute `tcp_target_stand`
`TcpPoseTarget` setpoints.

For the tracked real flow profile, the chunk overlay is schema v3 and the server
uses `delta_preview`: the publisher aligns a warm result by the number of policy
steps actually emitted since its camera observation, then the server integrates
the remaining ee-local deltas and feeds the existing Ruckig p/v/a preview chain.
Velocity proprio is the measured body delta over the image-time window; an
unavailable bracket is explicit metadata and prevents preview activation. The
server bounds both requested-to-preview projection error and
preview-command-to-measured-TCP lead with mandatory config limits and a
persistent fault policy.

### `TcpLinearMove`

MoveL-like Cartesian path primitive. It plans a Cartesian path with explicit timing/speed semantics and orientation interpolation semantics. Current modes are:

- `constant`: keep start orientation along the path
- `slerp`: interpolate start orientation to target orientation

Optional **collision-free MoveL** (`cartesian_control.linear_move.collision_free: true`, requires
`safety.init_motion_planner.enable`): on each move the server first checks (off-thread, with a
private IK + the planner's collision/floor oracle incl. the ground plane) whether the straight
Cartesian path is clear. If clear it runs the exact straight MoveL (orientation mode preserved);
if the straight path would self-collide or cross a safety plane it falls back to a collision-free
joint-space detour (the `init_motion` profile planner) to the IK'd target and streams that with
pure-pursuit. Either way the move reaches the target without collision; default off (strict
straight MoveL guarded only by the reactive barrier).

Real-motion-ready and exercised on the physical arms (the legacy run-mode execution gate
was retired; `TcpLinearMove` computes in every run mode, same as `TcpPoseTarget`). The move
is a FINITE, bounded path (`linear_move.max_duration_sec`): once it starts it drives to
completion from a single command even if that command's freshness/lease lapses, so one click
always reaches the target; an explicit command-mode change, a fault, or E-stop still abort it,
and the per-tick safety gate (floor / ROI / reach / self-collision barrier) applies on every
streamed target.

## Servo Control Architecture

The control path is moving toward this structure:

```text
CommandBuffer
  -> ServoCoordinator / DualArmServoLoop
       -> Left ArmWorker  -> left IRobotBackend
       -> Right ArmWorker -> right IRobotBackend
```

`DualArmServoLoop` owns:

- command freshness
- command-source lease interpretation
- lifecycle state
- FK/IK and Cartesian target generation
- safety filtering
- fault latching
- dual-arm result aggregation
- state publication

`rb_servo_server` C++ builds require Eigen3 and Pinocchio. Cartesian FK/IK,
orientation interpolation, frame conversion, and nontrivial SO(3)/SE(3)
operations must use Eigen/Pinocchio instead of local fallback math.

`ArmWorker` owns blocking per-arm backend I/O in worker mode. Worker mode is hardware-free/mock-only until separate real-hardware acceptance exists.

## Backend Architecture

Backends must return structured operation results:

- `BackendResult<RobotState>`
- `SendServoJResult`
- `BackendErrorKind`
- `BackendTiming`
- `FaultContext`

Bool-only backend results must not be reintroduced.

`RbpodoBackend` separates state acquisition from motion readiness. Valid joint feedback with `servo_enabled=false` is a valid read state, not motion readiness. Real `servo_j` sends remain blocked unless real gates and controller readiness are satisfied. Real stop/reset API wiring remains conservative until verified.

Unsupported raw script TCP comparison backends are outside the current
architecture. Do not add direct-to-controller raw script command paths; real
controller integration goes through `RbpodoBackend`.

Lower raw TCP client overhead does not bypass controller parser, ACK, motion,
or safety limits. `rt_script` is future work and remains out of scope.

## GUI And Policy Roles

`rb_gui` is a viewer/operator console. It exposes `JointTarget`,
`TcpPoseTarget`, and `TcpLinearMove` controls in every run mode and no longer
keeps mode-based client gates or feature-flag/env unlocks; whether a control is
live is derived from the live server state stream (per-arm FK/TCP-pose validity,
the server Cartesian gate, fault latch, motion state) and the command-source
lease. The server is the sole real-motion authority and rejects any command its
own gates (site config + safety filter + lease + deadman) do not allow — so the
GUI driving a real command does not bypass real-motion safety.

`policy_runner` owns Python action sources, including SpaceMouse. SpaceMouse
Cartesian input is a virtual target cursor that emits absolute `TcpPoseTarget`
setpoints. `flow-infer` ee_local policy deltas also emit absolute
`TcpPoseTarget` setpoints. Joint-only action sources do not require camera
observations. Camera-dependent sources must declare camera readiness and fail
closed when camera state is stale.

Policy and teleop datasets must preserve the collection environment as
metadata. The required categories are hardware-free mock, rbpodo
controller `pgmode` simulation, and physical real demonstrations.
Controller-simulation data uses `backend_type: rbpodo`, `run_mode: real`,
`operation_mode: simulation`, and `physical_motion_expected=false`; it should
record both `tcp_actual_stand` and `tcp_ref_stand` when available. It is not
the same evidence class as mock data and must not be mixed with physical real
data without explicit metadata filtering. The dataset schema is
documented in `docs/runbooks/policy_data_collection.md`.

## Camera Role

`camera_server` owns RealSense/mock capture, shared-memory image transport, metadata, and health. Real camera acceptance is separate from robot motion acceptance.

## Frame And Calibration Contract

Shared frame names and transform directions are defined in `docs/frame_contract.md`. The active setup registry is:

```text
calibration/active_calibration.yaml
```

Current geometry is `configured_estimate`, not measured calibration. It may be used for visualization and simulation, but not for real geometry-dependent policy.

## Validation Contract

Hardware-free validation is described in `docs/hardware_free_validation.md`.

Cartesian behavior is validated with the Pinocchio-backed C++ tests plus
active-stack smoke/acceptance on mock when a local mock config is available,
rbpodo controller `pgmode` simulation / VM, and physical real only through the
separate supervised runbooks. The old software-simulator-oriented Cartesian
acceptance runner is no longer part of the active validation surface.

Real three-camera acceptance is described in `docs/runbooks/camera_acceptance.md`.

The conservative ladder from rbpodo `pgmode` simulation evidence to physical
`operation_mode: real` evidence (stages P0–P9) is described in
`docs/runbooks/pgmode_real_transition.md`; physical-pass evidence must use
`tcp_actual_stand`, never `tcp_ref_stand`. The realized dual-arm physical
Cartesian circle bring-up is described in
`docs/runbooks/rbpodo_real_physical_circle.md`.

Passing simulator acceptance is not permission to move hardware.
