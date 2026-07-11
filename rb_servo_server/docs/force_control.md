# Force-control foundation

Force/admittance control remains inactive in every runtime path. Phase 1 adds a
deterministic F/T wrench pipeline and a bounded project-native Cartesian
admittance controller for hardware-free testing only. Neither component is
owned or called by `DualArmServoLoop` yet.

Tracked configurations must retain:

```yaml
force_control:
  provider: null
  enable: false
```

The loader also rejects `allow_in_real: true` and either arm's
`force_torque.*.enable: true`. These fail-closed checks are intentional until
the later contact-supervisor, telemetry, chunk-epoch, and motion-path
acceptance gates are complete.

## Wrench pipeline contract

`FtWrenchPipeline` consumes a backend-independent `FtRawSample`. A production
adapter must provide all of the following independently of the wrench values:

- confirmed sensor presence
- complete, finite fields
- sensor fault and overrange status
- acquisition sequence or equivalent source timestamp
- host receive timestamp

Poll time and a changing wrench value are not freshness evidence. Each arm
selects either a true acquisition `sequence` or a source-owned timestamp;
there is no host-time-only mode. Identical wrench values with an advancing
freshness source are valid; a stalled or regressed source fails closed.

The transform and compensation order is fixed:

```text
raw wrench in sensor frame
  - sensor-fixed bias
  -> T_tcp_sensor wrench action (including moment-arm torque)
  - modeled payload/gravity wrench in TCP frame
  - optional residual tare in TCP frame
  -> fast external wrench (future contact guard)
  -> low-pass filter
  -> control external wrench (future admittance input)
```

Changing the optional residual tare requires a continuous healthy window while
the caller reports the robot stationary and contact clear. The pipeline checks
the configured minimum sample count and per-axis force/torque standard
deviation before accepting the window mean. An unhealthy, moving, contact, or
high-variance window is rejected and cleared. Each acquisition sequence/time
value is counted at most once, even when the faster servo loop polls the same
sensor sample repeatedly. A future runtime adapter must
derive stationary/contact-clear from server state rather than operator intent.
Changing tare resets the control filter so pre-tare history is not mixed into
the next output.

`T_tcp_sensor` is the pose of the sensor frame expressed in the TCP frame. The
payload center of mass is also expressed in TCP coordinates. The pipeline uses
Pinocchio/Eigen spatial transforms; it does not introduce a separate SE(3)
implementation.

For the installed Robotous RFT64-6A01/Pika CAD stack, both active robot URDFs
author `ft_sensor_base` and `ft_sensor_measurement`. The CAD-derived transform
is `[0, 0, -0.202642, 0, 0, pi/2]` in `T_tcp_sensor` xyz/RPY order. This value
remains ineligible for enforcement until the per-arm controller wrench axes,
serial, adapter stack, payload, bias, and positive-axis load checks pass the
acceptance runbook. Tracked real/simulation configs intentionally remain off.

The current rbpodo EFT state is not eligible for enforcement. `eft_valid`
currently proves only that six finite fields were decoded. Zero values may
still mean no external sensor is selected, and the state does not expose an
independent acquisition sequence, sensor fault, or overrange flag.

## Standalone admittance contract

`ForceController` implements a diagonal variable-`dt` Cartesian admittance:

```text
M x_ddot + D x_dot + K x = measured_external_wrench - target_wrench
```

It enforces server-owned hard limits for:

- controller `dt`
- translational and rotational offset
- velocity, acceleration, and jerk
- per-update pose step
- observed external-work energy

A command may set a positive offset or step cap only to tighten the server
limit. A zero command cap means “use the server limit”; it can never enlarge
the configured safety boundary.

State mutation is two-phase:

```text
propose -> downstream IK/safety/send acceptance -> commit
        -> any rejection/intervention           -> reject/freeze
```

`engage()` cold-starts controller state; `release()` and `reset()` clear all
offset, velocity, acceleration, energy, and engagement state. A proposal is
invalid until the controller is explicitly engaged. Proposals carry controller,
lifecycle-generation, and base-state revision provenance, so a stale, replayed,
or cross-controller proposal cannot be committed.

After offset/step saturation, velocity is back-calculated from the emitted
offset. Acceleration and jerk are then checked from finite differences. If the
hard pose boundary makes those dynamic limits mutually infeasible, the
proposal is invalid instead of silently violating a cap. Future loop
integration must treat that fail-closed result as unload/hold or latch.

The current phase tests this contract in isolation. A future loop integration
must commit only after IK, final safety filtering, output intervention checks,
and backend-send acceptance. Passivity feedback must use fresh measured joint
state/FK velocity rather than the previous sent target.

## Inactive schema

The detailed schema may be parsed while disabled so profiles can be reviewed
and tested without granting motion authority:

```yaml
force_torque:
  source: rbpodo_eft
  left:
    enable: false
    frame_configured: false
    sensor_identity: ""
    calibration_id: ""
    freshness_source: sequence
    max_sample_age_sec: 0.02
    max_source_stall_sec: 0.02
    control_lpf_alpha: 0.2
    max_tcp_speed_m_s: 0.0
    max_tcp_accel_m_s2: 0.0
    residual_tare_min_samples: 50
    residual_tare_max_force_stddev_n: 0.1
    residual_tare_max_torque_stddev_nm: 0.01
    T_tcp_sensor: [0, 0, 0, 0, 0, 0]
    sensor_bias: [0, 0, 0, 0, 0, 0]
    payload_mass_kg: 0.0
    payload_com_tcp_m: [0, 0, 0]
    residual_tare_tcp: [0, 0, 0, 0, 0, 0]
  right:
    # Same schema, with an independent sensor/calibration/payload profile.
    enable: false

force_control:
  provider: null
  enable: false
  allow_in_real: false
  update_rate_hz: 500
  virtual_mass: [5.0, 5.0, 5.0, 0.5, 0.5, 0.5]
  damping: [80.0, 80.0, 80.0, 8.0, 8.0, 8.0]
  stiffness: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  max_dt_sec: 0.02
  max_pos_offset_m: 0.01
  max_rot_offset_rad: 0.1
  max_linear_velocity_m_s: 0.02
  max_angular_velocity_rad_s: 0.2
  max_linear_acceleration_m_s2: 0.2
  max_angular_acceleration_rad_s2: 2.0
  max_linear_jerk_m_s3: 2.0
  max_angular_jerk_rad_s3: 20.0
  max_pos_step_m: 0.001
  max_rot_step_rad: 0.01
  max_energy_j: 2.0
```

All mass, dynamic, pose, step, and energy bounds are finite and validated.
Virtual mass must be positive; damping, stiffness, payload mass, and tare
components must be non-negative or finite as appropriate.
`frame_configured: true` additionally requires non-empty sensor identity and
calibration IDs. Zero dynamic-envelope values mean no real dynamic envelope
has been accepted; they do not grant engagement.

## Promotion gates

The next changes must remain separate reviewable stages:

1. monitor-only contact supervisor and telemetry
2. same-tick fault/send suppression with `send_at_tick_start=false`
3. motion epoch plus observation provenance for flow chunk invalidation
4. DeltaTwist projection so force owns only a validated contact-normal axis
5. nominal pose plus admittance correction before IK, with propose/commit
6. deterministic loop-level replay acceptance
7. supervised real-sensor characterization and only then real contact motion

Before any enforcing or real stage, the system also needs an enforcing
floor/ROI/contact-zone envelope, output moving-average transition handling,
operator E-stop procedure, accepted sensor frame/sign/tare/payload profile, and
measured detection-to-send-suppression evidence. An F/T guard is not a
safety-rated function and does not replace the existing safety layers.
