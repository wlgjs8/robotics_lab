# Project-native guarded and Cartesian compliance control

The servo server contains an integrated F/T monitor, contact guard, unilateral
contact-normal admittance path, and bounded 6D Cartesian compliance path. It is
intended for the `flow-infer -> TcpPoseTarget -> DeltaTwist follower` path. In
Cartesian compliance mode, soft contact does not invalidate the current policy
chunk: inference and gripper execution continue while the server projects
loading policy increments and adds a bounded SE(3) compliance correction.

The controller-simulation config remains inert. After the 2026-07-12 positive
X/Y/Z frame capture, clean post-fix repeat, and roll/pitch/yaw
direction/recenter captures, the tracked physical-real config exposes a
supervised six-axis Hold compliance gate:

```yaml
force_control:
  provider: project_native
  enable: true
  operating_mode: cartesian_admittance
  allow_in_real: true
  supervised_experimental_real: true
  left:
    enable: true
    surface_source: none
    compliance_frame: tcp_origin
    compliance_axes: [true, true, true, true, true, true]
  right:
    enable: true
    surface_source: none
    compliance_frame: tcp_origin
    compliance_axes: [true, true, true, true, true, true]
```

The physical acceptance profile is managed directly in `stack_real.yaml`, one
reviewed setting at a time. In this gate all three translations and rotations
can modify a fixed Hold target. Roll/pitch/yaw share a 0.10 Nm deadband,
3 Nm/rad stiffness, 1.55 Nms/rad damping, and 0.2 virtual mass. Both geometric floor
constraints are disabled by explicit operator decision, so neither the TCP nor
gripper-tip floor backstop is present. The implementation is not safety-rated
and does not replace E-stop, lease/deadman, tracking, collision, ROI, or final
joint safety checks.

## Installed sensor frame

Both active robot URDFs place the Robotous RFT64-6A01 between joint 6 and the
Pika gripper. The explicit CAD measurement frame is `ft_sensor_measurement`.
Its CAD-derived pose in the controller TCP frame is:

```yaml
T_tcp_sensor: [0.0, 0.0, -0.202642, 0.0, 0.0, 1.5707963267948966]
```

The first 2026-07-12 right-arm review mixed TCP-gizmo and FT-gizmo axis names
and temporarily selected yaw zero. The later Cartesian-compliance capture
invalidated that interpretation: runtime X/Y followed the TCP triad while the
operator load referenced the +90 degree FT triad. The corrected supervised
test value for both identical arm/sensor assemblies is therefore:

```yaml
T_tcp_sensor: [0.0, 0.0, -0.202642, 0.0, 0.0, 1.5707963267948966]
```

The convention is `point_tcp = T_tcp_sensor * point_sensor`. The corrected
2026-07-12 18:48 physical capture showed right-arm +X/+Y/+Z translation in the
pushed runtime FT-control-gizmo direction and zero-wrench recentering; the
19:04 repeat completed without a controller or hard-limit fault. The operator
declared the left assembly identical, so the same mapping is retained for both
arms. The 19:11 and 19:16 captures confirmed supervised yaw direction and
recenter, and the 19:26 capture confirmed roll/pitch/yaw direction and recenter,
without controller or hard-limit faults. All rotations now share the same
sensitive characterization profile. Per-arm serials, payload/COM, bias, and
production force motion remain pending in the acceptance runbook.

## Wrench path

```text
rbpodo eft wrench
  -> sensor bias removal
  -> T_tcp_sensor wrench transform, including moment arm
  -> payload/gravity compensation
  -> residual tare
  -> fast external wrench     (hard guard)
  -> one LPF update per fresh controller sample
  -> control external wrench  (contact/admittance)
  -> branch A: stand/surface projection (floor normal + hard-limit telemetry)
  -> branch B: configured compliance frame (symmetric 6D controller)
  -> normal unloading + selected-frame translational/rotational compliance
```

`force_control.<arm>.compliance_frame` selects branch B:

- `surface`: stand-fixed surface axes, with the origin at the TCP.
- `sensor_origin`: RFT64 measurement axes and the physical sensor origin from
  `T_tcp_sensor`. This was the first physical direction/pivot bring-up stage and
  matches both the runtime control gizmo origin and the +90 degree URDF sensor
  axes when this selector is active.
- `tcp_origin`: the same sensor-axis orientation translated to the TCP origin.
  This is the active Gate 2 frame: translation is controlled in the corrected
  +90 degree FT axes while the correction is applied about the TCP endpoint.
  Gate 1 used `surface` only to make the initial sign check stand-fixed.

The hard force/torque guard does not move with this selector: resultant force,
resultant torque, and floor-normal compression remain computed from the fast
compensated TCP/stand wrench. Changing the compliance frame therefore cannot
rotate or bypass the existing hard thresholds. A sensor/TCP frame selection is
rejected unless the matching F/T transform is marked `frame_configured`.

`force_control.<arm>.surface_source` selects the stand-frame contact normal used
by branch A (floor-normal compression telemetry and the scalar normal
unloading):

- `floor_constraint`: the horizontal stand +Z axis. A motion-affecting force
  mode with this selector requires an enforcing `safety.floor_constraint`.
- `user_floor_plane`: the runtime tilted `safety.user_floor_constraint` normal,
  and requires that plane enforcing.
- `none`: the **floorless posture** — no geometric surface. The server-owned
  `floor_constraint` and `user_floor_constraint` may both be disabled, so the
  arm runs without the User/Stand floor velocity damper (useful for a bolt-pick
  approach that the geometric floor slow-zone would otherwise brake). Branch A
  falls back to the nominal stand +Z as the hard-normal reference axis, so the
  resultant-force, resultant-torque, and +Z compression hard guards are
  unchanged. `none` carries no unilateral unload surface, so it is accepted only
  for `monitor` and the symmetric 6D `cartesian_admittance`; `guard` and
  `guarded_admittance` (which regulate a surface-normal contact) still require an
  enforcing floor / user plane. Selecting `none` does not disable the floor for
  other subsystems — it only removes force control's dependency on it; the floor
  still applies to every motion primitive whenever it is separately enabled.

The current rbpodo adapter provides six finite EFT fields. The backend also
publishes an acquisition sequence that advances only when a new CobotData
state frame is received; cached/held states retain the prior sequence. This
proves controller-frame freshness even when firmware reports `robot_time_ns=0`,
but it is not an independent sensor sequence and does not prove sensor-present,
fault, or overrange status. Runtime telemetry therefore reports
`source_assurance: controller_frame_only`, `sensor_health_verified: false`, and
`safety_rated: false`. Zero wrench can still mean that no external sensor is
selected. This limitation is why physical force motion needs the separate
`supervised_experimental_real` opt-in and remains subject to the acceptance
runbook.

## Modes and state machine

`force_control.operating_mode` has four values:

- `monitor`: publishes compensated wrench and force state without changing or
  stopping motion.
- `guard`: latches `ExternalForceLimit` after the debounced contact threshold,
  or immediately at a hard force/torque threshold.
- `guarded_admittance`: enters contact regulation after debounce. Hard limits
  still latch immediately.
- `cartesian_admittance`: runs one symmetric zero-wrench Cartesian admittance
  controller on all six configured axes. Soft contact does not
  increment `motion_epoch`, synthesize an upstream Hold, freeze the gripper, or
  discard the active flow-infer chunk. Hard limits still latch immediately.

The per-arm state progresses through `inactive/monitoring`, `armed`,
`regulating`, `release_braking`, `release_hold`, and `fault`. In legacy
`guarded_admittance`, contact entry and release increment `motion_epoch`, so
`flow-infer` discards cached/in-flight chunks and reanchors. In
`cartesian_admittance`, soft contact and release remain in the current epoch;
only hard faults retain the interruption contract.

When `auto_tare_after_init_motion` is enabled, each selected arm also passes
through `awaiting_init_completion -> settling -> collecting -> accepted`.
Requesting Init Motion immediately invalidates that arm's previous software
zero. Collection begins only after measured Init Motion completion and the
settle delay; a failed or interrupted Init Motion cannot retain or create a
valid zero. Until acceptance, enforcing force modes remain unavailable.
Left-only and right-only Init Motion tare only that arm; a dual-arm request tares
both independently.

The GUI payload/CoG workflow uses the ordinary `JointTarget` primitive with
`joint_target_profile: payload_identification`. Observing that profile latches a
per-arm `payload_identification_inhibit`, invalidates the pose-local tare, and
resets the Cartesian compliance state before the identification move proceeds.
For each live identification packet, force-Hold promotion is suppressed on both
arms: the selected arm remains on its direct JointTarget and the peer's explicit
Hold cannot become a Cartesian admittance target. A rejected/unavailable profile
uses the same two-arm joint-hold posture for that packet.
Admission also requires a currently healthy, non-stale selected-arm F/T stream.
The command shape is server-enforced: exactly one arm may carry a finite
`payload_identification` JointTarget and the peer must be a payload-free Hold.
Both-profile and profile-plus-peer-motion packets fail closed to two-arm Hold
with `InvalidCommand`.
While the identification inhibit makes the normal tare-dependent force path
unavailable, enforcing force-control profiles continue to debounce the configured
hard limits against the raw pre-tare TCP wrench; crossing one still latches the
normal external-force fault.
The latch prevents an expired calibration command from falling back to a Hold
that is promoted into Cartesian admittance. Stop, lease loss, command timeout,
and faults do not clear it while the server remains running. Only a later
successful Init Motion tare for that arm clears the inhibit. A server restart
does not restore this runtime telemetry latch, but the existing invalid-tare
startup posture still keeps force motion fail-closed until Init Motion accepts a
new tare. State JSON and the CSV expose the latch/profile so sample collection
can wait for server confirmation.

The same state surface publishes `gravity_tcp`, the stand gravity vector
expressed in the actual TCP orientation for that controller sample. It is paired
with the pre-payload/pre-tare `wrench_tcp` input for offline payload
identification; it is not a new control input and does not alter the wrench path.

An accepted Init Motion sequence number is processed once even though the
`CommandBuffer` retains the packet until its timeout. Completion hands the arm
to `Hold` at the last accepted/sent reference. The cached packet cannot
re-enter the no-op path or repeatedly re-anchor that Hold reference to the
measured joints while an operator applies an external load. A later explicit
Init Motion request with a new sequence remains able to plan again.

Because the arm is usually already parked at the configured init pose when the
stack starts, the server's Init Motion no-op path (measured joints within
`safety.init_motion_planner.noop_tol_deg` of the goal) establishes the software
zero WITHOUT moving. The GUI auto-presses Init Motion once at startup in exactly
that no-motion case (arm at init pose AND `tare_state == awaiting_init_motion`),
so a fresh `make run` reaches an accepted zero without an operator press. The
auto-press is strictly gated to the already-at-init case — an arm parked away
from the init pose is never auto-pressed (that would plan a real move), so the
operator must press Init Motion manually there. Disable the automatic press with
`RB_GUI_STARTUP_INIT_TARE=0`. The GUI reads the server's EFFECTIVE no-op band —
`max(init_motion_planner.noop_tol_deg, waypoint_tol_deg)` — from
`RB_GUI_SERVER_CONFIG_PATH` and auto-presses only within that band minus a small
drift margin, so a GUI "already at init" verdict always coincides with the
server's no-op path (a parked arm typically drifts ~1° from the saved init pose
via servo settling / gravity sag, which the old fixed sub-degree tolerance never
matched). There is NO fallback tolerance: if the server config cannot be read the
auto-press stays off (fail-closed) rather than guessing a band that might let the
server plan a move. `RB_GUI_STARTUP_INIT_TARE_TOL_DEG` may lower the band but
never above the safe cap.

In legacy `guarded_admittance`, the accepted normal target is:

```text
max(policy outward offset from contact anchor, controller unload offset)
```

Thus an inward policy delta cannot build a hidden catch-up target, while a
policy-requested retreat remains possible. At or below
`contact_release_force_n`, the scalar controller
enters `release_braking`: force-error drive is removed, but damping, stiffness,
jerk limits, downstream acceptance, and two-phase commit remain active. Force
rising above the release threshold resumes `regulating`. A release is accepted
only after the committed controller velocity remains within
`release_velocity_threshold_m_s` for `release_dwell_sec`. The release tick
reanchors a target to the measured TCP before the correction is reset.
In legacy guarded mode, `release_hold` continues overriding stale Cartesian
commands until flow-infer
acknowledges the new epoch by returning a non-synthetic raw `Hold`; that
measured-pose hold is sent successfully for one tick before a later Cartesian
command may take ownership again. A command-timeout fallback Hold does not
clear the latch.

Legacy guarded contact regulation also remains active across an upstream
`Hold`. This is
required when `motion_epoch` invalidation makes flow-infer discard a contact
chunk and wait for a fresh inference. During that gap the server synthesizes a
`TcpPoseTarget` from the measured pose, bypasses stale chunk/SMD followers, and
applies only the bounded outward correction. Controller state is committed only
after the resulting IK, safety checks, and backend send are accepted.

Cartesian compliance keeps a command equilibrium separately from its bounded
6D offset. In upstream `Hold`, the equilibrium is captured once and remains
fixed; it is never replaced by the measured TCP on later ticks. Under
flow-infer, each accepted/projected policy delta advances the equilibrium, and
the first `Hold -> policy` delta is projected by the same rule. Policy deltas
that load the measured reaction wrench are removed in the selected compliance
frame. The compliant offset is composed with Pinocchio SE(3) operations. For
`sensor_origin`, rotation is applied about the physical sensor origin; for
`tcp_origin`, it is applied about the TCP endpoint.

While an axis wrench remains outside its deadband in the same direction as the
existing compliant offset, the Cartesian jerk governor preserves a terminal
loaded-hold trajectory: it may continue moving or brake away from zero, but it
does not reverse the offset toward zero. At the motion boundary it reaches zero
velocity and acceleration instead of using a return-direction reversal as a
valid braking result. If contact returns while stiffness is already recentering
the axis, the bounded loaded-return branch first brakes that existing return
velocity; the loaded-hold invariant takes over once the velocity is stationary
or points with the load. This avoids treating a physically unavoidable jerk-limited
recontact transient as a controller fault. When external wrench returns inside
the six-axis deadband, Cartesian stiffness and damping drive the offset and
velocity back to zero around that equilibrium.
If deadband chatter makes a strict no-reversal loaded hold temporarily
unreachable even in the hard motion envelope, the controller retains the
ordinary jerk-safe envelope and brakes toward a reachable loaded hold. It may
briefly continue the already-realized return motion, but offset, velocity,
acceleration, jerk, and step bounds remain enforced. This condition is bounded
telemetry, not an `ExternalForceLimit` fault.
The correction is not absorbed into the equilibrium, so release returns to the
current command pose without a motion-epoch reset or a pre-contact chunk
catch-up. Telemetry publishes the equilibrium pose, its `hold_anchor` or
`policy_target` source, and whether recentering is active. Scalar and 6D
controller proposals use the same two-phase commit boundary.

The tracked Gate 3D test profile also enables `blockwise_release_recenter`. Its
translation and rotation blocks retain the existing component deadbands, but a
quiet sibling axis does not start its spring return while another axis in the
same block remains loaded. After the whole block is released, the controller
intersects each axis' existing jerk-feasibility interval and applies one common
scale to the block's unconstrained jerk vector. This preserves the released 3D
translation or rotation direction instead of letting independent jerk clamps
reshape it. Translation and rotation are intentionally separate because their
units, gains, and motion limits differ. Coupling applies only while every
enabled axis in the block remains inside its recursively viable soft envelope.
If one axis needs hard-envelope recovery, the controller keeps every axis'
already-selected per-axis jerk for that tick; this may briefly deviate from the
latched return direction, but it preserves recursive feasibility. Coupling
resumes automatically after all axes re-enter the soft envelope. A load that
returns immediately exits the coupled recenter path and re-enters the existing
loaded-hold path.

State JSON and servo CSV expose the transition directly through
`compliance_translation_recenter_coupled`,
`compliance_rotation_recenter_coupled`,
`compliance_translation_recenter_deferred`, and
`compliance_rotation_recenter_deferred`. During a fully released return,
`*_recenter_coupled` may briefly become false while
`compliance_limit_axes`/`jerk_limited_motion_envelope` reports bounded recovery;
this is expected only if `proposal_valid` remains true and no safety latch is
raised.

## Normal admittance

The scalar controller uses positive compressive force and positive outward
unload displacement:

```text
surface_normal_stand = outward geometric normal (+Z for the floor)
compressive_force = -surface_normal_stand dot measured_force_stand
```

The minus sign converts the installed sensor's TCP reaction-wrench convention
to positive compression. It does not reverse the geometric normal used for
outward unloading motion.

```text
M x_ddot + D x_dot + K x = measured_normal_force - target_force
```

It is unilateral: `0 <= x <= max_unload_offset_m`. A below-target force may
remove an existing unload correction, but the controller never seeks contact by
commanding farther inward than the contact anchor. Velocity, acceleration,
jerk, per-tick step, total offset, and observed energy are bounded.

The position and velocity bounds are enforced with a jerk-limited braking
envelope, not by clamping an already-integrated output. The controller reserves
enough future acceleration/jerk authority to stop at either unilateral position
boundary and at the velocity limit. Reaching a normal saturation is therefore a
valid `bounded` proposal; only a state that is genuinely outside the feasible
motion envelope faults. This prevents a contact pulse from latching
`ExternalForceLimit` merely because unloading reached `max_normal_velocity_m_s`.
The discrete admittance integrator uses the configured force-control period
(`1 / update_rate_hz`), which is validated equal to the servo rate. Scheduler
jitter remains telemetry but does not change the jerk envelope from one tick to
the next.

The Cartesian controller applies the same diagonal SMD rule to all three
translations and all three rotations:

```text
M_i x_i_ddot + D_i x_i_dot + K_i x_i = wrench_error_i
```

This nominal continuous-time displacement model is second order, not first
order. The bounded implementation stores displacement, velocity, and
acceleration and selects jerk as its control input. It is therefore implemented
as three coupled first-order state updates, with deadband, loaded-hold,
passivity, and motion-envelope switching making the complete controller a
constrained hybrid system rather than a single linear first-order equation.
Measured physical TCP twist contributes to the passivity-energy observer; the
SMD damping term uses the virtual compliant velocity.

On every tick the controller intersects jerk, acceleration, velocity, position,
and future braking constraints before advancing any state. A recursively viable
soft operating envelope preserves both boundary braking and loaded-hold
authority; the configured hard envelope remains available only for
deterministic recovery if a prior state is on the numerical boundary. This
replaces integrate-then-clamp behavior and does not depend on increasing the
wrench moving average.

Reaching an offset, velocity, acceleration, jerk, or per-tick step envelope is
a valid bounded compliance result, not a force fault. Telemetry exposes
`compliance_limit_axes` and
`compliance_limit_reason=jerk_limited_motion_envelope` so an operator can
distinguish normal compliance saturation from a hard force/torque trip.

Normal, transverse, and rotational contact hysteresis are tracked separately.
A pure tangential pull or torque can therefore enable Cartesian compliance
without creating a fictitious floor-contact correction. The scalar normal
controller is used only by `guarded_admittance`. The aggregate contact signal
remains available as
`contact_active`; the individual fields are `normal_contact_active`,
`transverse_contact_active`, and `rotational_contact_active`.

Controller state uses a two-phase update:

```text
propose -> IK + final safety + backend send accepted -> commit
        -> rejection, suppression, or intervention -> reject/freeze
```

## Configuration

Edit only the force-control block in the tracked `stack_real.yaml`, one
acceptance stage at a time. The following is a schema example, not an accepted
threshold profile:

```yaml
force_torque:
  source: rbpodo_eft
  left:
    enable: true
    frame_configured: true
    sensor_identity: "left-rft64-6a01-site-profile"
    calibration_id: "left-rft64-6a01-cal-v1"
    freshness_source: sequence
    max_sample_age_sec: 0.02
    max_source_stall_sec: 0.02
    control_lpf_alpha: 0.2
    max_tcp_speed_m_s: 0.0
    max_tcp_accel_m_s2: 0.0
    auto_tare_after_init_motion: true
    auto_tare_settle_sec: 0.5
    residual_tare_min_samples: 500
    residual_tare_max_force_stddev_n: 0.75
    residual_tare_max_torque_stddev_nm: 0.15
    T_tcp_sensor: [0.0, 0.0, -0.202642, 0.0, 0.0, 1.5707963267948966]
    sensor_bias: [0, 0, 0, 0, 0, 0]
    payload_mass_kg: 0.0
    payload_com_tcp_m: [0, 0, 0]
    residual_tare_tcp: [0, 0, 0, 0, 0, 0]

force_control:
  provider: project_native
  enable: true
  operating_mode: monitor
  allow_in_real: false
  supervised_experimental_real: false
  update_rate_hz: 500
  left:
    enable: true
    surface_source: floor_constraint
    # surface | sensor_origin | tcp_origin
    compliance_frame: sensor_origin
    target_force_n: 2.0
    contact_enter_force_n: 3.5
    contact_release_force_n: 2.75
    force_deadband_n: 0.5
    hard_normal_force_n: 15.0
    hard_force_norm_n: 20.0
    hard_torque_norm_nm: 3.0
    debounce_samples: 3
    hard_limit_debounce_samples: 5
    release_dwell_sec: 0.1
    release_velocity_threshold_m_s: 0.002
  right:
    enable: false
  normal_admittance:
    virtual_mass_kg: 5.0
    damping_n_s_m: 80.0
    stiffness_n_m: 0.0
    max_unload_offset_m: 0.01
    max_normal_velocity_m_s: 0.02
    max_normal_acceleration_m_s2: 0.2
    max_normal_jerk_m_s3: 2.0
    max_normal_step_m: 0.001
    max_energy_j: 2.0
  # Cartesian selected-frame order: x, y, z, roll, pitch, yaw.
  # cartesian_admittance owns all six axes and regulates zero wrench.
  virtual_mass: [2.0, 2.0, 2.0, 0.2, 0.2, 0.2]
  damping: [26.0, 26.0, 26.0, 2.8, 2.8, 2.8]
  stiffness: [80.0, 80.0, 80.0, 8.0, 8.0, 8.0]
  wrench_deadband: [1.5, 1.5, 1.5, 0.25, 0.25, 0.25]
  blockwise_release_recenter: false
  max_pos_offset_m: 0.02
  max_rot_offset_rad: 0.08
  max_linear_velocity_m_s: 0.03
  max_angular_velocity_rad_s: 0.15
```

The tracked responsive hand-guiding profile begins responding outside 1.5 N or
0.25 Nm, allows 20 mm / 0.08 rad compliance travel, and uses lower virtual
mass with near-critical damping. Its faster motion envelope is 0.03 m/s and
0.15 rad/s with 0.25 m/s² / 1.5 rad/s² acceleration and 2.0 m/s³ /
10 rad/s³ jerk. These are compliance limits, not hard-contact thresholds; the
40 N normal, 45 N resultant, and 7 Nm hard guards remain unchanged.

Each active arm must also enable its F/T pipeline. Threshold ordering is
validated as `target < release < enter < hard_normal`, with
`target + force_deadband <= release`,
`hard_force_norm >= hard_normal`, and a positive finite
`release_velocity_threshold_m_s`. Motion-affecting modes require kinematics,
`servo.send_at_tick_start: false`, and an update rate equal to the servo rate.
The `floor_constraint` / `user_floor_plane` surface sources additionally require
their enforcing plane; `surface_source: none` (the floorless posture, accepted
only for `monitor` / `cartesian_admittance`) requires no floor.
`sensor_origin` and `tcp_origin` additionally require the matching
`force_torque.<arm>.frame_configured: true` transform.
Automatic post-init tare requires `safety.init_motion_planner.enable: true`,
because that sequencer verifies completion against measured joints.
Physical real force motion additionally requires both `allow_in_real: true` and
`supervised_experimental_real: true`; those flags expose an experimental code
path and are not acceptance evidence.

Run the physical monitor profile with:

```bash
make run MODE=real
```

Before pressing Init Motion, make sure the selected tool is unloaded and not
touching the floor, fixture, or another object. The explicit Init Motion action
is the contact-clear assertion for software zeroing; the variance gate detects
motion/transients but cannot distinguish steady external contact from tool
weight. The accepted value is pose-local while payload mass/COM remain zero, so
re-zero after changing tool orientation. Payload characterization is still
required before general orientation-dependent force control.

Begin with `operating_mode: monitor`. The GUI is deliberately read-only: it
shows the raw wrench monitor plus freshness/health qualification, software-zero
state/sample count/validity, contact state,
measured normal force, target, correction, saturation, fault reason, and motion
epoch. Full compensated wrench fields remain available in state telemetry. The
GUI does not provide an enable toggle or tuning controls. The servo CSV records
the same per-arm raw/TCP/fast/control wrenches, freshness and health fields, and
force-control state so a completed physical test remains auditable offline.
State JSON, GUI status, and CSV identify `compliance_frame` and publish both
`control_wrench_surface` (floor interpretation) and
`control_wrench_compliance`/`wrench_error_compliance` (the actual 6D controller
input/error). They also publish `compliance_frame_pose_valid` and the resolved
`compliance_frame_actual_stand` pose used for that tick, so visualization and
offline logs never need to reconstruct a moving control frame from CAD data.
Historical fields ending in `_surface` for compliance
offset/velocity/acceleration and policy deltas are retained for log-schema
compatibility; their components follow `compliance_frame` when that selector is
not `surface`.
Monitor telemetry also publishes `fast_normal_force_n`, `fast_force_norm_n`,
`fast_torque_norm_nm`, `contact_threshold_exceeded`, and
`hard_limit_threshold_exceeded`, `hard_limit_sample_count`, and
`hard_limit_exceeded`. The threshold flag is the raw current-sample result;
only consecutive fresh F/T samples advance the count, and the debounced flag
becomes true at `hard_limit_debounce_samples`. Repeated servo ticks carrying the
same sensor sample never advance it. These are preflight diagnostics in monitor
mode only; they do not stop or modify motion.

Run the fail-closed static-window acceptance check after each monitor capture:

```bash
python3 rb_servo_server/tools/analyze_ft_acceptance.py \
  logs/servo_log.csv --static-start-sec 2 --static-end-sec 10
```

The command exits nonzero while promotion is blocked and prints an incremental
`residual_tare_tcp` candidate. A candidate is evidence for review, not an
automatic config edit; payload and orientation-sweep acceptance remain
separate requirements.
The required fast-projection/norm CSV columns also make pre-fix captures fail
closed; rebuild and collect fresh monitor data rather than reusing them.

## Promotion sequence

Follow `docs/runbooks/ft_force_control_acceptance.md`: read-only acquisition,
static sign/frame tests, payload and dynamic characterization, monitor-only
contact, guard, zero-force unloading, small positive force, then planned
`flow-infer` contact. Never use controller pgmode as evidence of physical sensor
dynamics.
