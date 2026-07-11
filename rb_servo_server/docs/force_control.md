# Project-native guarded force control

The servo server contains an integrated, default-off F/T monitor, contact guard,
and unilateral contact-normal admittance path. It is intended for the
`flow-infer -> TcpPoseTarget -> DeltaTwist follower` path: the policy retains
tangential position and orientation ownership, while the server rejects further
penetration on an accepted surface normal and adds a bounded outward unloading
offset when measured force exceeds the target.

The controller-simulation config remains inert. The tracked physical-real
config currently exposes a supervised experimental dual-arm motion stage after
the 2026-07-11 software-zero/sign/selectivity captures:

```yaml
force_control:
  provider: project_native
  enable: true
  operating_mode: guarded_admittance
  allow_in_real: true
  supervised_experimental_real: true
  left:
    enable: true
  right:
    enable: true
```

The physical acceptance profile is managed directly in `stack_real.yaml`, one
reviewed setting at a time. The implementation is not safety-rated and does not replace E-stop, lease/deadman,
tracking, collision, floor, ROI, or final joint safety checks.

## Installed sensor frame

Both active robot URDFs place the Robotous RFT64-6A01 between joint 6 and the
Pika gripper. The explicit measurement frame is `ft_sensor_measurement`. The
CAD-derived pose of that frame in the controller TCP frame is:

```yaml
T_tcp_sensor: [0.0, 0.0, -0.202642, 0.0, 0.0, 1.5707963267948966]
```

The convention is `point_tcp = T_tcp_sensor * point_sensor`. This transform is
geometry evidence only. Per-arm serial, wrench axis/sign, controller selection,
bias, payload, tare, and positive-load checks still require the acceptance
runbook before an enforcing mode is used.

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
  -> TCP-to-stand rotation
  -> accepted surface-normal projection
```

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

`force_control.operating_mode` has three values:

- `monitor`: publishes compensated wrench and force state without changing or
  stopping motion.
- `guard`: latches `ExternalForceLimit` after the debounced contact threshold,
  or immediately at a hard force/torque threshold.
- `guarded_admittance`: enters contact regulation after debounce. Hard limits
  still latch immediately.

The per-arm state progresses through `inactive/monitoring`, `armed`,
`regulating`, `release_braking`, `release_hold`, and `fault`. Contact entry and
release increment the server `motion_epoch`. `flow-infer` observes that epoch,
discards cached and in-flight chunks, clears accumulated absolute targets, and
reanchors from the fresh measured TCP poses.

When `auto_tare_after_init_motion` is enabled, each selected arm also passes
through `awaiting_init_completion -> settling -> collecting -> accepted`.
Requesting Init Motion immediately invalidates that arm's previous software
zero. Collection begins only after measured Init Motion completion and the
settle delay; a failed or interrupted Init Motion cannot retain or create a
valid zero. Until acceptance, enforcing force modes remain unavailable.
Left-only and right-only Init Motion tare only that arm; a dual-arm request tares
both independently.

During `regulating`, the accepted normal target is:

```text
max(policy outward offset from contact anchor, controller unload offset)
```

Thus an inward policy delta cannot build a hidden catch-up target, while a
policy-requested retreat remains possible. Tangential translation and TCP
orientation are unchanged. At or below `contact_release_force_n`, the controller
enters `release_braking`: force-error drive is removed, but damping, stiffness,
jerk limits, downstream acceptance, and two-phase commit remain active. Force
rising above the release threshold resumes `regulating`. A release is accepted
only after the committed controller velocity remains within
`release_velocity_threshold_m_s` for `release_dwell_sec`. The release tick
reanchors a target to the measured TCP before the correction is reset.
`release_hold` continues overriding stale Cartesian commands until flow-infer
acknowledges the new epoch by returning a non-synthetic raw `Hold`; that
measured-pose hold is sent successfully for one tick before a later Cartesian
command may take ownership again. A command-timeout fallback Hold does not
clear the latch.

Contact regulation also remains active across an upstream `Hold`. This is
required when `motion_epoch` invalidation makes flow-infer discard a contact
chunk and wait for a fresh inference. During that gap the server synthesizes a
`TcpPoseTarget` from the measured pose, bypasses stale chunk/SMD followers, and
applies only the bounded outward correction. Controller state is committed only
after the resulting IK, safety checks, and backend send are accepted.

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
    target_force_n: 2.0
    contact_enter_force_n: 6.0
    contact_release_force_n: 3.0
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
```

Each active arm must also enable its F/T pipeline. Threshold ordering is
validated as `target < release < enter < hard_normal`, with
`target + force_deadband <= release`,
`hard_force_norm >= hard_normal`, and a positive finite
`release_velocity_threshold_m_s`. Motion-affecting modes require kinematics,
`servo.send_at_tick_start: false`, an update rate equal to the servo rate, and
an enforcing selected floor plane.
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
