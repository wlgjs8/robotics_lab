# robotics_lab Architecture

This document is the current source of truth for the system architecture. Component READMEs may contain local details, but public terminology, safety boundaries, and topology must match this document.

## Current Phase

The repository is currently in **rbpodo pgmode-real physical robot bring-up**.
Simulator-first Cartesian acceptance hardening is largely complete and now serves
as the regression baseline; active validation has moved onto the physical
RB3-730E hardware.

The simulator stack remains the regression baseline for:

- per-arm simulator topology
- structured backend result and fault telemetry
- `JointTarget` and `JointVelocity`
- `TcpPoseTarget`
- `TcpLinearMove`
- `TcpTwistLocal` and `TcpTwistStand`
- GUI operator controls
- policy_runner SpaceMouse command paths
- command-source lease/arbitration
- camera readiness contracts for future policy work

Real motion is now an active, gated bring-up lane (see Maturity Boundary), not a
deferred milestone. It stays fail-closed: gates, site-local config, operator
supervision, and an E-stop are all required, and passing simulator acceptance is
never permission to move hardware.

## Maturity Boundary

Supported for mock/simulation work:

- mock dual-arm servo control
- one local simulator endpoint per arm
- persistent simulator JSON-line transport
- simulator direct and worker I/O modes
- FK/TCP state publication with quaternion fields
- simulator-only Cartesian PTP, Linear, and Twist commands
- mandatory Eigen3/Pinocchio-backed Cartesian math in `rb_servo_server`
- mock camera server
- GUI viewer/operator console for mock/simulation
- Python policy_runner with joint and Cartesian simulator action sources
- simulator-only Cartesian acceptance scripts

Run / validated on pgmode-real (physical RB3-730E hardware):

- read-only physical diagnostics parity against controllers `.200`/`.201`
  using `tcp_actual_stand` (not `tcp_ref_stand`)
- dual-arm physical Cartesian circle tracking — slow, TUNED-1 profile, median
  tracking ~1.42° (`docs/runbooks/rbpodo_real_physical_circle.md`)
- UMI dual-arm Cartesian teleop (relative-init) driving `TcpPoseTarget` on the
  physical arms; UMI `data_tcp` replay verified on hardware (ee_local + r_align)
- `flow-infer` `real_policy` full closed-loop rollout on the physical robot
  (pi0.5/openpi): `TcpTwistLocal` streaming + gripper commands. The `real_policy`
  rollout-mode gate stays fully enforced (`_validate_real_policy`: `mode=real`,
  `allow_real_motion`, measured/accepted geometry + retarget, validated collision
  model, gripper gate) and was satisfied via accepted/validated runtime config —
  the lane is open and exercised, not blocked. Runtime is validated (smooth,
  in-distribution; async chunking decouples ~30 Hz policy from the 500 Hz servo;
  the absolute-proprio frame gap is fixed by reset-relative retrain). Task success
  is still model-limited (see below)
- real gripper motion via the Pika Gripper Backend, gated by `RB_ALLOW_REAL_GRIPPER`
  + `measured_gripper_available` + `allow_real_gripper_motion`
- server-side async URDF-mesh self-collision guard (`CollisionMonitor`, 33 geoms /
  337 pairs) enforced in real motion via a velocity barrier; stale / hard-breach
  fail closed
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
backend_type: mock | simulator | rbpodo
```

`run_mode` describes the environment. `backend_type` describes the backend implementation. Deprecated terms such as `rbsim_local`, public `rbsim`, or mixed simulator aliases must not be introduced in new public docs/configs.

Supported real-controller scope is rbpodo only. Mock and simulator backends are
hardware-free validation surfaces; unsupported raw script TCP comparison paths
must not be presented as runnable backends.

### Simulation flavors (name them precisely)

Three distinct things are loosely called "simulation". Name them by their
canonical config keys, never by the bare word "simulation":

| Flavor | Canonical name | `run_mode` | `backend_type` | `operation_mode` | Connects to |
|---|---|---|---|---|---|
| Software simulation | `rb_simulator` / simulator backend | `simulation` | `simulator` | — | hardware-free Python `ArmSimulator` over local TCP |
| Controller simulation | rbpodo `pgmode` controller-sim | `real` | `rbpodo` | `simulation` | a real rbpodo controller running in `pgmode` |

`run_mode` is `simulation` only for the software simulator. Controller simulation
is `run_mode: real` (it really connects to a controller) with the controller's
`operation_mode: simulation`.

Within controller simulation the **target** may be a Virtual ControlBox **VM** or
a **physical** controller box in `pgmode`. These are behaviourally identical to
the server — same `rbpodo` backend, same `operation_mode: simulation`, same env
gates — so they must NOT get new `run_mode`/`backend_type` values. Distinguish
them only by deployment target, via config filename suffix and docs:

- `…controller_sim_vm.yaml` — target is a Virtual ControlBox VM (no physical hardware on the wire)
- `…controller_sim_onbox.yaml` — target is a physical controller box held in `pgmode`

Site/VM configs live in gitignored `rb_servo_server/config/local/`.

## Controller Topology

The physical system has one controller endpoint per arm:

```text
rb_servo_server
  left_robot  backend_type=rbpodo -> 172.28.60.200
  right_robot backend_type=rbpodo -> 172.28.60.201
```

The simulator mirrors that controller shape:

```text
rb_servo_server
  left_robot  backend_type=simulator -> rb_simulator_left
  right_robot backend_type=simulator -> rb_simulator_right
```

The simulator topology is isomorphic to the physical topology by endpoint count and ownership, not by IP address. Simulator configs must not default to the real controller IPs.

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

For rbpodo controller-simulation circle live visualization, state fanout and
benchmark overlay are intentionally separate:

```text
rb_servo_server
  state_pub_endpoints[0] udp://127.0.0.1:50151 -> benchmark recorder
  state_pub_endpoints[1] udp://127.0.0.1:50161 -> rb_gui state receiver

rbpodo_circle_tracking_benchmark
  --overlay-pub-endpoint udp://127.0.0.1:50261 -> rb_gui circle overlay receiver
```

The state stream is robot/server telemetry, including `tcp_actual_stand`,
`tcp_ref_stand`, Cartesian gate fields, and
`physical_motion_expected=false` for pgmode simulation. The overlay stream is
desired benchmark geometry and live metrics only; it is not robot state and
must not carry commands.

### Docker Compose Simulator Topology

```text
rb_simulator_left container
  arm: left
  control: tcp://0.0.0.0:50200
  admin:   tcp://0.0.0.0:50201

rb_simulator_right container
  arm: right
  control: tcp://0.0.0.0:50200
  admin:   tcp://0.0.0.0:50201
```

Separate containers can reuse internal ports. Compose uses compose-specific configs and sets:

```bash
RB_SIMULATOR_ALLOW_NON_LOOPBACK=1
```

### Host-Local Simulator Topology

```text
left simulator
  control: tcp://127.0.0.1:50200
  admin:   tcp://127.0.0.1:50201

right simulator
  control: tcp://127.0.0.1:50210
  admin:   tcp://127.0.0.1:50211
```

## Safety Gates

Real robot connection is closed unless:

```bash
RB_ALLOW_REAL_ROBOT=1
```

Real joint servo motion is closed unless:

```bash
RB_ALLOW_REAL_MOTION=1
```

Rbpodo Servo J motion with controller ACK waiting disabled is additionally
closed unless:

```bash
RB_ALLOW_RBPODO_ACK_DISABLED_MOTION=1
```

Real Cartesian/TCP motion is closed unless:

```bash
RB_ALLOW_REAL_CARTESIAN=1
```

Accepting the controller `-2001` suspect diagnostics
(`op_stat_self_collision`/`robot_time` field-layout garbage) in real mode is
additionally closed unless:

```bash
RB_ALLOW_RBPODO_SUSPECT_DIAGNOSTICS_REAL_MOTION=1
```

These environment variables are necessary but not sufficient. Config
(`cartesian_control.allow_in_real: true`) and operator-supervised acceptance must
also explicitly allow the operation. These gates have already carried a
supervised dual-arm physical Cartesian circle
(`docs/runbooks/rbpodo_real_physical_circle.md`).

The policy-side `SafetyGate` no longer blocks real Cartesian motion (PR #13,
scoped to `cartesian_gate.operation_mode == "real"`); for real motion
`rb_servo_server` is therefore the sole safety layer — safety filter (dq/ddq/
joint limits), tracking-error fault latch, the async URDF-mesh self-collision guard (`CollisionMonitor`),
lease arbitration, and deadman. EMS/SOS/soft-estop/`collision_occur`/unknown-mode/
init-error continue to latch regardless of the gates above. Controller-simulation
safety is unchanged.

Rainbow controller `pgmode` simulation through the `rbpodo` backend is a
separate evidence category from both hardware-free `rb_simulator` and future
physical real motion. It connects to real controller boxes, so configs use
`run_mode: real` and `backend_type: rbpodo`, but each robot must use
`operation_mode: simulation` and the controller must be confirmed in `pgmode`
simulation. Controller-simulation circle tracking should use
`tcp_ref_stand`/controller reference telemetry with
`physical_motion_expected=false`.

The narrow rbpodo controller-simulation streaming Cartesian carve-out requires
all normal real-controller/motion gates plus:

```bash
RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1
RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1
RB_RBPODO_PGMODE_SIMULATION_CONFIRMED=1
```

The config must explicitly keep physical real Cartesian blocked:

```yaml
cartesian_control:
  allow_in_controller_simulation: true
  allow_in_real: false
```

This carve-out is not `RB_ALLOW_REAL_CARTESIAN` and does not approve physical
real Cartesian motion. Servo J ACKs in a controller-simulation circle artifact
do not by themselves prove that Cartesian commands executed; check the
Cartesian gate telemetry and `tcp_ref_stand` movement.

### Floor plane constraint (`safety.floor_constraint`)

A stand-frame keep-out plane for the TCP: when enabled, neither arm's TCP may be
commanded below `z = z_min_m` (default 0.010 m), regardless of motion primitive
and regardless of run mode (mock, simulator, controller-sim, and real all pass
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
  (`TcpPoseTarget`, `TcpDelta*`, `TcpLinearMove`) have their stand z clamped to
  the plane before IK, and streaming twists (`TcpTwist*`) have their downward
  stand-frame v_z zeroed near the plane, so lateral teleop/policy motion slides
  along the plane instead of stuttering against the Tier-1 hold. Joint-space
  primitives get no Tier-2 assist (Tier-1 hold only).

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

`TcpCircleMove` is an optional benchmark primitive for isolating server-side
circle generation from Python UDP streaming jitter. In `rb_simulator` it
requires `cartesian_control.enable_benchmark_primitives: true`,
`circle_move.allow_in_simulation: true`, and
`circle_move.allow_in_real: false`. In rbpodo controller `pgmode` simulation,
the same primitive is allowed only through the controller-simulation carve-out
above, with `operation_mode: simulation`, controller-reference state, and
`physical_motion_expected=false`. Its optional `phase_advance_sec` is visible
telemetry and must not be interpreted as proof of physical system latency.
Physical real `operation_mode: real` remains blocked for `TcpCircleMove`.

The official report term for `TcpCircleMove` / reserved `TcpCircleTrack`
evidence is server-side circle tracking. Both command names carry or imply
`command_family: server_side_circle` in benchmark/state metadata so reports can
group them without breaking backward compatibility. `TcpCircleMove` is the
implemented benchmark command today; `TcpCircleTrack` remains a disabled
closed-loop skeleton until a future acceptance task implements it.

Rbpodo async ACK-supervised 500 Hz streaming is another controller-simulation
only carve-out. It requires `operation_mode: simulation`,
`physical_motion_expected=false`, `RB_ALLOW_RBPODO_ASYNC_STREAMING=1`, normal
real-controller/motion gates, controller-simulation motion gates, and same-run
pgmode confirmation. `sdk_ack_worker` moves ACK waiting into a worker lane;
`socket_send_supervised` is `socket_send_only` evidence and must be guarded by
q_ref/tcp_ref watchdogs. This is no physical real approval and does not change
the default servo rate.

Benchmark summaries and reports must include canonical lane metadata:
`benchmark_lane`, `control_loop_location`,
`trajectory_generation_location`, `feedback_loop_location`,
`low_level_send_mode`, `acceptance_semantics`, `tracking_source`, and
`physical_motion_expected`. The ACKON500 official pass lane is
`rbpodo_server_side_circle_ackon500_sdk_worker`. The
`rbpodo_server_side_circle_500hz_socket_send_supervised` lane is send-only
evidence and must not be grouped as an ACKON500 pass.

### Server-Side Circle Tracking Skeleton

`TcpCircleTrack` is the reserved command schema for moving closed-loop circle
generation from Python into `rb_servo_server`. The long-term path is:

1. Parser/schema: accept a trajectory-parameter command and publish structured
   accepted/rejected telemetry without sending motion.
2. Simulator implementation: compute desired pose/twist and feedback inside the
   servo tick using fresh simulator state.
3. Rbpodo controller-simulation implementation: run the same tick-local control
   against controller-reference state in Rainbow `pgmode` simulation only.
4. Acceptance matrix: compare simulator, rbpodo controller-simulation, and
   future physical-real evidence as separate categories.

The skeleton is disabled by default:

```yaml
cartesian_control:
  enable_server_side_circle_track: false
```

When disabled, `TcpCircleTrack` is rejected with
`tcp_circle_track_disabled`. If explicitly enabled, the current skeleton still
rejects with `tcp_circle_track_not_implemented`; it does not produce Cartesian
twist targets. Physical real `operation_mode: real` is rejected with
`tcp_circle_track_physical_real_blocked`. Future controller-simulation work
must keep `operation_mode: simulation`, `allow_in_real: false`, and the existing
controller-simulation env gates.

Tracked real config is a template only:

```text
rb_servo_server/config/dual_real.example.yaml
```

Site-owned real configs belong under:

```text
rb_servo_server/config/local/
```

No tracked runnable real robot config should exist.

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

Deprecated simulator config names are archived under `docs/archive/configs/`
for historical reference only. They are not runnable source-of-truth profiles
and must not be used for new smoke or acceptance evidence.

Force control is intentionally unavailable:

```yaml
force_control:
  provider: null
  enable: false
```

## Motion Primitive Contract

Every primitive below is additionally subject to the stand-frame floor plane
constraint when `safety.floor_constraint.enable` is set (see "Floor plane
constraint" under Safety Gates): the final per-tick joint target is FK-checked
and held/latched if it would put either TCP below the plane.

### `JointTarget`

Absolute joint-space target. This is a joint-space point-to-point command.

### `JointVelocity`

Streaming joint velocity command. Suitable for joint teleop/debug when safety gates allow it.

### `TcpPoseTarget`

Cartesian point-to-point final-pose target. It is MoveJ-like at the TCP level. Final TCP pose is targeted, but the intermediate TCP path is not guaranteed to be linear. Real mode is open through the real-mode gates plus `cartesian_control.allow_in_real: true`, and has been validated on the dual-arm physical Cartesian circle.

### `TcpLinearMove`

Simulator-only MoveL-like Cartesian path primitive. It plans a Cartesian path with explicit timing/speed semantics and orientation interpolation semantics. Current modes are:

- `constant`: keep start orientation along the path
- `slerp`: interpolate start orientation to target orientation

Real mode remains blocked.

### `TcpTwistLocal` / `TcpTwistStand`

Streaming Cartesian velocity primitives. `TcpTwistLocal` is intended for SpaceMouse/local-frame teleop. `TcpTwistStand` is the stand-frame low-level API. Server-side Cartesian velocity limits, the server-side angular deadband for orientation hold, stale-state checks, deadman behavior, and command-source arbitration are required.

**Twist conditioning.** The only conditioning of the twist input is a per-tick
magnitude clamp (`limitTwist()` in `cartesian_servo_controller.cpp`: scale-to-limit
or reject by `exceed_limit_policy`); there is no slew-rate limit and no twist LPF
(the former `twist_lpf_*` option was removed — input smoothing for `TcpTwistLocal`
is handled by the `twist_via_smd` SMD pose tracker instead). Any additional
teleop-side smoothing (e.g. the SpaceMouse/UMI input EMA) lives upstream in
`policy_runner`, not here. Joint-space continuity near singularities is owned by
the shared IK solver's selective singularity-robust damping
(`kinematics.ik.singular_region_eps` / `damping_max`), not by the twist path.

**TrajectoryFilter defers all Cartesian modes.** `TrajectoryFilter::computeJointTarget`
handles only joint-space modes; every Cartesian mode (`TcpPoseTarget`,
`TcpLinearMove`, `TcpDelta*`, `TcpTwist*`, `TcpCircle*`) is intentionally
deferred — the filter deactivates its joint SMD and returns `holdTarget()`.
Cartesian commands are routed by `DualArmServoLoop` directly to
`CartesianServoController`, so any SMD/joint-trajectory shaping that applies to
joint primitives does **not** apply to the Cartesian/twist path.

Velocity-level Cartesian servo targets use an explicit joint target integration
mode. The simulator acceptance default is `previous_command`: the controller
integrates Cartesian velocity from the last safe joint target accepted after
SafetyFilter, rather than repeatedly generating a one-tick target from measured
`q_actual`. The legacy `measured_actual` mode remains available for debugging,
and `measured_actual_lookahead` can model fixed lookahead.

**Jacobian linearization point is fixed at `q_actual` in all modes.**
`velocity_target_integration` selects only the *integration base* (where `qdot`
is integrated from): `previous_command` integrates from the last sent joint
target, `measured_actual`/`measured_actual_lookahead` from measured state. It
does **not** change where the Jacobian is evaluated — `solveCartesianVelocity`
always linearizes at `state.q_actual_deg` regardless of mode. This matters for
controller-`pgmode` simulation: with
`controller_simulation_divergence_source: reference` the divergence check is
taken against the controller reference (`tcp_ref_stand`/`q_command`) while the
Jacobian still uses measured `q_actual`, so the two operate on different joint
configurations — expected, but a subtlety to keep in mind when tuning
reference-tracking. The integrator is reset on holds, faults, stale/invalid
state, lease loss, velocity-mode exit, and excessive command-vs-actual joint
divergence. Real Cartesian motion opens through the existing real-mode gates
plus `cartesian_control.allow_in_real: true`; the dual-arm physical circle
bring-up drives it via `TcpPoseTarget` streaming.

### `TcpDeltaLocal` / `TcpDeltaStand`

Low-level one-shot/debug jog commands. They are not the default GUI target-move primitive.

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

`ArmWorker` owns blocking per-arm backend I/O in worker mode. Worker mode is simulator-only until separate real-hardware acceptance exists.

## Backend Architecture

Backends must return structured operation results:

- `BackendResult<RobotState>`
- `SendServoJResult`
- `BackendErrorKind`
- `BackendTiming`
- `FaultContext`

Bool-only backend results must not be reintroduced.

`RbsimBackend` keeps one persistent JSON-lines TCP connection per simulator backend instance during healthy operation. Transport/protocol corruption closes the socket; robot/controller-level errors such as `RobotFault` remain structured backend results.

`RbpodoBackend` separates state acquisition from motion readiness. Valid joint feedback with `servo_enabled=false` is a valid read state, not motion readiness. Real `servo_j` sends remain blocked unless real gates and controller readiness are satisfied. Real stop/reset API wiring remains conservative until verified.

Unsupported raw script TCP comparison backends are outside the current
architecture. Do not add direct-to-controller raw script command paths; real
controller integration goes through `RbpodoBackend`.

Lower raw TCP client overhead does not bypass controller parser, ACK, motion,
or safety limits. `rt_script` is future work and remains out of scope.

## GUI And Policy Roles

`rb_gui` is a viewer/operator console. It exposes every motion primitive in every run mode and no longer keeps mode-based client gates or feature-flag/env unlocks; whether a control is live is derived from the live server state stream (per-arm FK/TCP-pose validity, the server Cartesian gate, fault latch, motion state) and the command-source lease. The server is the sole real-motion authority and rejects any command its own gates (`RB_ALLOW_REAL_*` + site config + safety filter + lease + deadman) do not allow — so the GUI driving a real command does not bypass real-motion safety.

`policy_runner` owns Python action sources, including SpaceMouse. SpaceMouse Cartesian uses `TcpTwistLocal`, not repeated TCP deltas. Joint-only action sources do not require camera observations. Camera-dependent sources must declare camera readiness and fail closed when camera state is stale.

For rbpodo controller-simulation circle live visualization, `rb_gui` is a
state and overlay consumer. It should show both actual/reference TCP telemetry
and the benchmark desired-circle overlay, but it must not route circle
commands through `policy_runner`. `policy_runner` remains a separate command
source for policy workflows, not a visualization broker.

Policy and teleop datasets must preserve the collection environment as
metadata. The required categories are hardware-free `rb_simulator`, rbpodo
controller `pgmode` simulation, and future physical real demonstrations.
Controller-simulation data uses `backend_type: rbpodo`, `run_mode: real`,
`operation_mode: simulation`, and `physical_motion_expected=false`; it should
record both `tcp_actual_stand` and `tcp_ref_stand` when available. It is not
the same evidence class as simulator data and must not be mixed with future
physical real data without explicit metadata filtering. The dataset schema is
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

Cartesian simulator acceptance is described in `docs/runbooks/tcp_pose_simulator_acceptance.md`.

Real three-camera acceptance is described in `docs/runbooks/camera_acceptance.md`.

The conservative ladder from rbpodo `pgmode` simulation evidence to physical
`operation_mode: real` evidence (stages P0–P9) is described in
`docs/runbooks/pgmode_real_transition.md`; physical-pass evidence must use
`tcp_actual_stand`, never `tcp_ref_stand`. The realized dual-arm physical
Cartesian circle bring-up is described in
`docs/runbooks/rbpodo_real_physical_circle.md`.

Passing simulator acceptance is not permission to move hardware.
