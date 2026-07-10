# Servo Backend Contract

This document defines the current backend, fault, and servo I/O contract.

## Goals

The control layer must preserve truth from each backend operation. It must not collapse all backend failures into a single `false` result or string log.

The contract should support:

- mock backend
- rbpodo backend
- direct servo I/O
- worker servo I/O
- structured state publication
- future real read-only acceptance

## Backend Operations

Backends expose structured operations:

```cpp
BackendResult<RobotState> connect();
BackendResult<RobotState> initialize();
BackendResult<RobotState> readState();
SendServoJResult sendServoJ(const SendServoJRequest& request);
BackendResult<RobotState> stop();
BackendResult<RobotState> resetFault();
```

Do not reintroduce bool-only backend operations.

## Error Taxonomy

Backend failures are classified with `BackendErrorKind`, for example:

- transport/protocol: `TransportConnectFailed`, `TransportWriteFailed`, `TransportReadFailed`, `TransportTimeout`, `ProtocolError`
- topology: `WrongArm`, `WrongEndpoint`, `UnknownArm`
- robot lifecycle: `RobotDisconnected`, `RobotNotInitialized`, `ServoDisabled`, `WrongMode`, `RobotFault`, `InvalidJointState`
- command rejection: `InvalidTarget`, `ControllerRejected`, `CommandTimeout`
- software/policy: `DependencyUnavailable`, `SuppressedByPolicy`, `Unknown`

Each backend error should preserve:

- kind
- code/name/message
- retryable/recoverable flags
- robot_fault/transport_fault flags
- operation timing
- optional `state_after`

## Fault Classification

`FaultClassifier` maps structured backend and command results into the safety domain:

- robot/controller faults become robot-state safety failures
- transport failures stay transport/backend failures
- policy suppression is not a transport failure
- IK and Cartesian failures stay kinematics/control failures
- emergency and fault latch states suppress normal servo commands

The state stream should preserve both the current send policy and the original latched fault context. `last_send=SuppressedByPolicy` must not erase the first fault cause.

Dual-arm latches preserve per-arm context as well as the top-level summary. If both
arms report different failures in the same tick, `fault_context.top_level` remains
the deterministic summary used for the existing `latched_fault_reason`, while
`fault_context.left` and `fault_context.right` preserve each arm's classified
cause. Policy suppression is still not a fault by itself and must not override a
real backend, robot-state, emergency, command, or kinematics failure as the
top-level cause.

## Send Suppression

The servo loop must not keep sending regular `servo_j` while fault-latched, emergency-latched, or read-only. Suppression is explicit state, not a send failure.

Expected fields include:

```json
{
  "send_suppressed": true,
  "send_policy": "fault_latched",
  "fault_context": {
    "backend_error_kind": "RobotFault",
    "backend_error_name": "fault_latched",
    "top_level": {
      "backend_error_kind": "RobotFault",
      "backend_error_name": "fault_latched"
    },
    "left": {
      "backend_error_kind": "RobotFault",
      "backend_error_name": "fault_latched"
    },
    "right": null
  }
}
```

## Direct Teaching (Free-drive)

`IRobotBackend::setFreedrive(bool)` (op `BackendOp::SetFreedrive`) releases/re-acquires
`servo_j` authority for per-arm direct teaching. The default implementation is a benign
no-op success for the hardware-free mock backend; `RbpodoBackend` overrides it to
call the rbpodo SDK `set_freedrive_mode` (`freedrive_teach_on()` / `freedrive_teach_off()`).
While its free-drive latch is set, `RbpodoBackend::sendServoJ()` refuses to write
`move_servo_j` and returns `SuppressedByPolicy` (`rbpodo_freedrive_active`) — a defensive
backstop under the server's global suppression.

It is fail-closed behind `servo.allow_freedrive` (default `false`), is a leased lifecycle
command, and is routed through the `ArmWorker` lifecycle queue in worker/async I/O (direct
backend call otherwise). While any arm is in free-drive the servo loop sets
`send_policy == "freedrive"` (suppressing `servo_j` to both controllers, after the
fault/emergency checks) and bypasses the motion pipeline so the hand-driven actual
divergence cannot latch a tracking error. On exit the held target is resynced to the
current actual joints. State JSON exposes top-level
`freedrive.{left_active,right_active,any_active}`. See
`docs/runbooks/freedrive_direct_teaching.md`.

## Direct I/O And Worker I/O

### Direct I/O

The servo loop calls per-arm backend operations directly. This is simple and remains useful for baseline hardware-free validation.

### Worker I/O

Each `ArmWorker` owns one backend and one thread. The servo loop reads cached state and dispatches bounded send requests through worker interfaces. Worker mode allows left/right endpoint I/O to proceed independently.

Worker mode is hardware-free/mock-only until separately accepted on hardware.

## ArmWorker Command Policy

Streaming servo requests are latest-wins. If a pending command is overwritten before dispatch, the worker must expose drop/overwrite telemetry. This is intentional for servo targets but must not be silent.
The internal telemetry field for overwritten commands is
`worker_command_drops_total`, mirrored in state JSON as
`command_drops_total`. Per-arm state JSON also exposes
`pending_overwrites_total`, `last_dropped_seq`, `last_enqueued_seq`,
`last_dispatched_seq`, `last_completed_seq`, `queue_policy`,
`read_period_sec`, `read_rate_hz`, and `state_age_us` under the `worker`
object.

Lifecycle commands such as reset/stop should not be silently overwritten by streaming servo targets. They should use a separate lane or explicit policy.

## RbpodoBackend Semantics

State acquisition and motion readiness are separate.

A controller can return valid joint feedback while `servo_enabled=false`. That is a valid state read, not permission to send motion.

Read-only diagnostic startup is a controller bring-up mode, not a
motion-ready mode. When `servo.send_servo_commands: false` and the explicit
`servo.allow_readonly_*_startup` flags are set, startup may continue after
state acquisition succeeds even if motion readiness fails because of a robot
fault, controller-not-ready state, or configured joint-range violation. The
state stream must still publish the state as unsafe, including
`startup_validation`, per-arm `startup_invalid_reasons`,
`q_range_violations`, and backend readiness diagnostics such as
`motion_readiness_error_kind` / `motion_readiness_error_name`. This mode exists
only to surface raw controller diagnostics such as rbpodo status flags and
joint feedback in JSON; it must not send `servo_j`.

Rbpodo status fields are treated as firmware/SDK-layout-dependent diagnostics.
The backend preserves the raw values in per-arm state JSON under
`rbpodo_diagnostics.raw`, including `time`, `real_vs_simulation_mode`,
`init_state_info`, `init_error`, `op_stat_sos_flag`, `op_stat_ems_flag`,
`op_stat_soft_estop_occur`, `op_stat_collision_occur`, and
`op_stat_self_collision`.

The backend also decodes the controller's external F/T sensor wrench
(`sdata.eft_fx..eft_mz`) into `RobotState.eft_wrench` / `eft_valid`, published
in per-arm state JSON as `eft_wrench` (`[fx, fy, fz, tx, ty, tz]`, N / Nm),
`eft_valid`, and `eft_source: "rbpodo.sdata.eft"`. The values are in the
controller-reported external-sensor frame (tool-flange mounted), NOT verified
as a TCP-frame wrench — `RobotState.wrench_tcp` remains reserved for the
(TCP-frame) force-control path and is not populated from `eft_*`. The
controller reports zeros when no external FT sensor is selected; `eft_valid`
is `false` when the values are non-finite or the state frame ended before the
eft fields (short/old-firmware frame; boundary constant
`kRbpodoStateFrameEftEndOffsetBytes`, pinned to the SDK struct by a
static_assert in the rbpodo-enabled build). Telemetry only: nothing in the
motion/safety path consumes `eft_*` yet. Boolean status flags with values other than `0` or
`1`, non-finite or implausibly tiny nonzero controller time, and unknown
`real_vs_simulation_mode` values mark `diagnostics_suspect=true`.
Suspicious diagnostics do not make `readState()` fail when joint acquisition is
otherwise valid, but they do make the state motion-unsafe and block
`sendServoJ()` with a `RobotFault` named `rbpodo_diagnostics_suspect`.
Garbage-looking raw flag values must be kept in `rbpodo_diagnostics.raw` rather
than used as the sole controller `error_code`.

For rbpodo controller `pgmode` simulation only, configs may opt into
`servo.controller_simulation_treat_unreliable_status_fields_as_unavailable: true`.
The option defaults to `false` and is active only when the
controller-simulation motion gate is open. When active, the decoder treats only
`op_stat_self_collision` shape validation and controller time plausibility as
unavailable, publishes the suppressed field names in
`rbpodo_diagnostics.unavailable_fields`, and sets
`rbpodo_state_decode_policy` to
`controller_sim_unreliable_fields_unavailable`. Raw values remain visible.
SOS, EMS, soft-estop, collision, unknown `real_vs_simulation_mode`, and the
explicit `op_stat_self_collision == 1` self-collision fault path remain
enforced.

For `operation_mode: real` physical motion, the controller-simulation gate above
is closed, so that carve-out is inert. A separate, more strongly gated opt-in
`servo.allow_real_motion_with_suspect_diagnostics: true` extends the SAME
two-field suppression (`op_stat_self_collision` shape, `robot_time`) to real
motion, for sites that have accepted the vendor `-2001` field-layout mismatch and
choose to run physical motion without trusting the controller's self-collision
status. It defaults to `false` and is fail-closed: it requires
`operation_mode: real` and the per-arm config opt-in itself (the legacy
`RB_ALLOW_REAL_*` / `RB_ALLOW_RBPODO_SUSPECT_DIAGNOSTICS_REAL_MOTION` env gates
were removed from the server runtime). When active it
sets `rbpodo_state_decode_policy` to `real_motion_suspect_diagnostics_accepted`
(distinct from the controller-sim string so physical-motion telemetry is
unambiguous). It suppresses ONLY those two fields; SOS, EMS, soft-estop,
`collision_occur`, unknown `real_vs_simulation_mode`, init error, and the explicit
`op_stat_self_collision == 1` self-collision path all still latch. Because it does
not trust the controller's self-collision status, operators should pair it with
the server-side `safety.self_collision` capsule guard.

Rainbow Virtual ControlBox controller-simulation targets may permanently
report `init_error != 0` and `init_state_info != 6` even while accepting
simulation `move_servo_j` commands and updating controller reference joints.
The server may tolerate only tightly-scoped controller-simulation shapes when
all matching config gates are open: `operation_mode: simulation` and
`servo.allow_controller_simulation_motion: true` with the controller confirmed
in `pgmode` simulation (the legacy controller-sim env gates were removed from
the server runtime).

`diagnostic_error_source == "rbpodo_init_error"` additionally requires:

```yaml
servo.allow_controller_simulation_init_error: true
```

The init-error tolerance is limited to startup invalid reasons confined to
`robot_fault` and `servo_disabled`.

The fully-virtual "not activated" condition (`servo_enabled=false`,
`init_state_info != 6`, startup reason `servo_disabled`) is a separate
controller-simulation override. It additionally requires:

```yaml
servo.allow_controller_simulation_not_activated: true
```

When that not-activated gate is open, an otherwise-accepted
`rbpodo_diagnostics_suspect` controller-simulation startup/state may also
carry `servo_disabled`; when it is closed, the diagnostics-suspect startup
tolerance remains confined to `robot_fault` only. These overrides are not
physical real-motion acceptance.

Raw controller joint angles may use continuous or wrapped representations. When
`safety.joint_wrap_for_startup_validation: true`, startup range diagnostics may
normalize configured joints using `safety.joint_wrap_period_deg`; `0` disables
wrapping for that joint and `360` treats values separated by a full revolution
as equivalent for range diagnostics. For example, raw `-317 deg` with range
`[-190, 190]` and period `360` normalizes to `43 deg`.

For the supported rbpodo-only real-control scope, raw joint values are the
source of truth. Preserve controller degrees in `q_actual_deg`, `q_target_deg`,
`q_ref_deg`, `q_sent_deg`, state JSON, and servo logs. Control, safety,
tracking, and q-ref comparisons must not normalize raw values to `[-180, 180]`.
The tracked rbpodo real templates use explicit per-joint raw safety arrays
matching the current controller soft-limit configuration:
`q_min_deg: [-360, -360, -160, -360, -360, -360]` and
`q_max_deg: [360, 360, 160, 360, 360, 360]`. The J3 value stays near the
RB3-730E elbow's catalog `+/-150 deg` range while leaving the current
site-configured margin. Narrow ranges such as `[-180, 180]` are allowed only for
intentional tests or site-owned conservative overrides, not as production
defaults.

The startup-validation wrapping policy remains diagnostic only. State JSON must
keep raw `q_actual_deg`, publish any `q_range_wrapped` entries, and may publish
`q_actual_normalized_for_safety_deg` for the validation view. If the raw value
is already in range, it is left unchanged. If the configured range width is
greater than or equal to the wrap period, the normalized value is only a
deterministic diagnostic representative, not motion-ready evidence.
`safety.joint_wrap_for_motion_safety` remains refused because silently wrapping
targets mid-trajectory can create joint discontinuities.

Absolute `JointTarget` goals have a narrower, accepted goal-selection step:
before rate limiting/SMD, the filter may choose the in-range `target +/- 360*k`
representation closest to the current sent target. This is endpoint-equivalent,
limit-bounded, and goal-only; it does not normalize state or wrap while moving.
`safety.joint_target_literal_axes` opts individual joints out of that selection.
The stack configs mark J6/wrist yaw literal so InitMotion returns to the
configured raw yaw target instead of a closer equivalent that could wind the
Pika gripper cable.

When kinematics is enabled, the server warns if the configured rbpodo safety
range differs from the known `rb3_730e.urdf` model limits. This is diagnostic:
raw controller state and commands remain governed by configured safety limits,
while IK may still be limited by the URDF/Pinocchio model.

If `servo.send_servo_commands: true`, startup remains strict. Robot faults,
wrong mode/not-ready state, and joint-range violations must fail startup rather
than being converted into a healthy condition.

For rbpodo controller `pgmode` simulation only, configs may opt into
`servo.controller_simulation_async_supervision_nonlatching: true`. The option
defaults to `false` and is active only when the controller-simulation motion
shape and gates are open. When active, async streaming supervision faults are
published as a recoverable advisory instead of latching top-level
`SendFailure`: state JSON sets `async_supervision_degraded: true` while the
suppressed async fault context is present, per-arm async telemetry remains
visible, and the server emits a throttled warning. Physical real mode and all
non-async-supervision fault paths continue to latch exactly as before.

For the same rbpodo controller `pgmode` simulation only, configs may also opt
into `safety.controller_simulation_tracking_error_nonlatching: true` (default
`false`, active only when the controller-simulation motion gate is open). When
active, the reference/actual command-tracking divergence
(`SafetyVerdict::TrackingError` from the joint-tracking check) is treated as a
recoverable advisory instead of latching: the server keeps following the
rate-limited (`clampMotion`) desired target, sets state JSON
`tracking_error_degraded: true`, and emits a throttled warning. `tracking_error_policy`
itself stays `fault_latch` (still required and enforced in real mode); this flag
only suppresses the latch at runtime inside the pgmode gate. The separate
`controller_simulation_physical_motion` guard — an unexpected actual move in a
no-motion mode — is explicitly excluded and continues to latch, as do physical
real mode and every other fault path.

Real `sendServoJ()` requires:

- valid state acquisition
- controller motion readiness
- config `send_servo_commands: true`
- site-local config that enables real motion (the legacy `RB_ALLOW_REAL_*` env
  gates were removed from the server runtime)

Cartesian primitives are config-gated. `TcpPoseTarget` and `TcpLinearMove`
require `cartesian_control.enable: true` and the relevant site-local
Cartesian gate for the active topology.

The only real-controller carve-out is rbpodo controller `pgmode` simulation.
`run_mode: real` may execute Cartesian target generation only when the selected
backend is `rbpodo`, the robot `operation_mode` is `simulation`, the controller
is confirmed in `pgmode` simulation, and these config gates are set (no env):

- `cartesian_control.allow_in_controller_simulation: true`
- `servo.allow_controller_simulation_motion: true`
- `cartesian_control.allow_in_real: false`

This is not physical real Cartesian enablement. `operation_mode: real` remains
blocked for Cartesian primitives unless the site config explicitly sets
`cartesian_control.allow_in_real: true`. State JSON must expose `cartesian_available`,
`cartesian_unavailable_reason`, and a `cartesian_gate` object with the
backend/run-mode/config/env decision fields so a controller-simulation
benchmark cannot be mistaken for physical motion approval. The gate separates
prospective streaming availability from the current command:
`controller_simulation_streaming_cartesian_available` says a subsequent
streaming Cartesian command may be admitted under pgmode simulation, while
`controller_simulation_cartesian_enabled_for_current_command` is true only when
the current command is a controller-simulation Cartesian command. This split
prevents a startup `Hold` state from deadlocking the first policy-runner
`TcpPoseTarget` command.

Real stop/reset APIs remain conservative until verified. If no verified API is wired, return `DependencyUnavailable` and require operator intervention.

Rbpodo is the primary vendor-library real backend. The current tracked launch
configs are `rb_servo_server/config/stack_real.yaml` and
`rb_servo_server/config/stack_sim.yaml`; the legacy `dual_real*.example.yaml`
template surface is no longer tracked. Site-specific variants and acceptance
stage copies belong under `rb_servo_server/config/local/`, which is
intentionally user-owned and gitignored.

### Rbpodo ACK Semantics

`disable_waiting_ack: false` is the default and required baseline. In this
ACK-on mode, `sendServoJ().accepted=true` means rbpodo observed controller ACK
for the `move_servo_j` command. State JSON and servo logs expose this as:

- `ack_policy: "wait"`
- `ack_observed: true`
- `controller_acceptance_observed: true`
- `send_acceptance_semantics: "controller_ack_observed"`
- `rbpodo_waiting_ack: true`
- `ack_wait_duration_us`

`disable_waiting_ack: true` changes the meaning of a successful send. rbpodo's
SDK returns after the socket/API send path instead of waiting for immediate
controller ACK. In this ACK-off mode, `accepted=true` means the command was
sent through the client path; it does not prove the controller accepted it. The
telemetry must show:

- `ack_policy: "disabled"`
- `ack_observed: false`
- `controller_acceptance_observed: false`
- `send_acceptance_semantics: "socket_send_only"`
- `rbpodo_waiting_ack: false`

ACK-off is an experimental supervised acceptance mode, not a safe mode. Real
motion with ACK waiting disabled is config-driven and must be enabled explicitly
in the site-local config (the legacy `RB_ALLOW_RBPODO_ACK_DISABLED_MOTION` env
gate was removed from the server runtime).

ACK-off runs need stronger monitoring because immediate controller rejection is
not observed on the command call. At minimum, review `readState()`, controller
`error_code`, state staleness, `q_ref`/`q_actual` tracking, and Rainbow system
codes such as M561/M568/M569/M570 when they are observable.

`command_timeout_sec` is the rbpodo command/ACK timeout used by
`move_servo_j`. Supported 500 Hz configs use finite positive values chosen so
command stalls are visible quickly during acceptance.

### Rbpodo Async ACK-Supervised Streaming Contract

Synchronous ACK terminology is fixed:

- ACK-on means `controller_ack_observed`: the command call waited for and
  observed controller ACK.
- ACK-off means `socket_send_only`: the command was written through the client
  path and must not be reported as controller ACK acceptance.

`servo.rbpodo_async_streaming` defines the future async 500 Hz
controller-simulation contract. It is disabled by default and does not change
the default servo rate or current rbpodo transport behavior:

```yaml
servo:
  rbpodo_async_streaming:
    enable: false
    mode: disabled
    rate_hz: 500
    queue_policy: latest_wins
    max_pending_age_ms: 10
    ack_supervision:
      enable: true
      expected_ack_timeout_ms: 50
      missing_ack_fault_after_ms: 100
      max_consecutive_missing_ack: 10
    reference_supervision:
      enable: true
      q_ref_update_timeout_ms: 50
      q_ref_target_tolerance_deg: 0.5
      tcp_ref_update_timeout_ms: 50
    diagnostics:
      publish_per_command_jsonl: false
```

Allowed modes:

- `disabled`: existing synchronous behavior.
- `sdk_ack_worker`: the servo loop dispatches non-blockingly; a worker thread
  calls the rbpodo SDK with ACK waiting enabled. Accepted commands have
  `controller_ack_observed`, but throughput may be below 500 Hz if ACK waiting
  is slow. Worker backlog, drops, and overwrites must be visible.
- `socket_send_supervised`: the servo loop dispatches non-blockingly; the
  rbpodo SDK is configured with `disable_waiting_ack=true` or equivalent.
  Successful sends are `socket_send_only`, never `controller_ack_observed`.
  Controller acceptance must be inferred by `q_ref` and/or `tcp_ref` watchdogs.

When `enable: true`, config/startup must fail unless both arms are
`backend_type: rbpodo`, `run_mode: real`, and `operation_mode: simulation`.
Physical `operation_mode: real` is explicitly refused. The legacy
`RB_ALLOW_RBPODO_ASYNC_STREAMING` / `RB_ALLOW_REAL_*` /
`RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION` / `RB_RBPODO_PGMODE_SIMULATION_CONFIRMED`
env gates were removed from the server runtime; the controller-simulation
carve-out is now config-driven
(`servo.allow_controller_simulation_motion: true`,
`cartesian_control.allow_in_controller_simulation: true`,
`cartesian_control.allow_in_real: false`) with the controller confirmed in
`pgmode` simulation.

This is a Rainbow controller `pgmode` simulation carve-out only. Async
supervision is not proof of physical real safety, does not authorize
`operation_mode: real`, does not enable physical Cartesian motion, and does not
promote 500 Hz to a default rate.

State JSON exposes top-level `async_streaming_enabled`,
`async_streaming_mode`, and `async_streaming_policy`. Each arm exposes
`async_streaming` with command counters, ACK/socket-send counters, drop and
overwrite counters, q_ref/tcp_ref watchdog miss counters, sequence numbers,
last q_ref/socket-send timestamps, `last_controller_acceptance_semantics`,
worker backlog, max observed pending age, and `supervision_state`:
`ok`, `warning`, or `fault`.

### Rbpodo Servo J Parameters

`RbpodoBackend::sendServoJ()` maps rbpodo config fields directly to Rainbow
UI Script `move_servo_j(jnt, t1, t2, gain, alpha)`:

- `servo_t1_sec` -> `t1`, the controller arrival/command time. For streaming
  Servo J this should match the server command period, `1 / servo.rate_hz`.
- `servo_t2_sec` -> `t2`, the controller hold time. This is not UR-style
  lookahead.
- `servo_gain` -> `gain`.
- `servo_alpha` -> `alpha`, the controller's inner low-pass-filter gain (not
  acceleration). Note the controller's internal `0.1` scaling — see the range
  note below.

Rainbow joint targets are in degrees. Config validation refuses only
`servo_t1_sec` below `0.002` (and, for real motion, a `servo_t1_sec` that does
not match the `1 / servo.rate_hz` period). `servo_t2_sec`, `servo_gain`, and
`servo_alpha` only have to be finite and positive; values outside the
vendor-recommended range are accepted with a WARN, not refused
(`config.cpp` `validate_rbpodo_backend`):

- `servo_t1_sec >= 0.002` — refused otherwise; in real motion must equal
  `1 / servo.rate_hz` (`0.002` at 500 Hz) unless
  `servo.allow_servo_t1_rate_mismatch=true`.
- `0.02 < servo_t2_sec < 0.2` — vendor-recommended; outside this range only
  WARNs.
- `servo_gain > 0`.
- `0 < servo_alpha <= 10` — **script-level units**. The Rainbow controller
  scales `gain`/`alpha` by `0.1` internally (vendor-confirmed), so the script
  value we send is 10x the effective value. The effective vendor range
  `0 < alpha <= 1` therefore maps to script-level `0 < servo_alpha <= 10`, and
  `servo_alpha: 10.0` means "effective 1.0 = inner LPF OFF". The tracked real
  robot profile intentionally uses `servo_alpha: 1.0` (effective roughly `0.1`)
  to retain controller-side filtering after LPF-off motion showed jerk/jitter on
  hardware. The range check is in script-level units so both the real profile
  (`1.0`) and the diagnostic LPF-off profile (`10.0`) are valid. (Earlier docs
  using a sub-unit script-level alpha range were mixing effective and
  script-level units and are superseded by this range.)

Deprecated aliases remain accepted with warnings while configs migrate:

- `servo_time_sec` -> `servo_t1_sec`
- `servo_lookahead_sec` -> `servo_t2_sec`
- `servo_acc` -> `servo_alpha`

Setting both a canonical key and its deprecated alias is refused to avoid
ambiguous real-controller parameters. For real rbpodo configs with
`servo.send_servo_commands=true`, `servo_t1_sec` must match the configured
`servo.rate_hz` period within `servo.servo_t1_rate_match_tolerance_ratio`
unless `servo.allow_servo_t1_rate_mismatch=true` is set after explicit
acceptance. Read-only configs with `send_servo_commands=false` may load with a
mismatch warning, but the warning is not motion approval.

New configs must not use `servo_acc`; use `servo_alpha`. New configs must not
use `servo_lookahead_sec`; use `servo_t2_sec`. The old names are compatibility
aliases only.

### Servo J Streaming Profiles

The supported 500 Hz `move_servo_j` profiles keep `t1`, `t2`, and `gain` fixed
and vary only the script-level `alpha` according to target environment:

```yaml
servo_t1_sec: 0.002   # == 1 / servo.rate_hz at 500 Hz (command arrival/period)
servo_t2_sec: 0.021   # controller hold time, just above the 0.02 vendor floor
servo_gain:   1.0      # unity, no command scaling
```

Profile-specific alpha:

- Physical real robot: `servo_alpha: 1.0` (effective roughly `0.1`). This keeps
  Rainbow's inner LPF active enough to reduce jerk/jitter observed with LPF off.
- Controller-simulation / diagnostic transparency: `servo_alpha: 10.0`
  (effective `1.0`). This disables Rainbow's inner LPF and is useful when
  controller-side smoothing must be removed from acceptance evidence.

Do not treat `servo_t1_sec`, `servo_t2_sec`, or `servo_gain` as casual tuning
knobs in supported 500 Hz profiles. `servo_alpha` is deliberately profile-owned:
use `1.0` for the tracked real stack unless a supervised acceptance task
explicitly asks for the LPF-off diagnostic profile.

Responsiveness, smoothness, and accuracy are still primarily owned by the
`rb_servo_server` control loop. For Cartesian setpoint streaming
(`TcpPoseTarget` / `tcp_target_pose`) the server-side tuning surface is:

- `cartesian_control.pose_track_smd.*` — second-order (spring-mass-damper)
  target tracker. `natural_frequency_linear_hz` / `natural_frequency_angular_hz`
  set responsiveness; `damping_ratio_linear` / `damping_ratio_angular` set
  overshoot (keep `1.0` for no-tremble); `max_linear_velocity_m_s` /
  `max_linear_accel_m_s2` / `max_angular_velocity_rad_s` /
  `max_angular_accel_rad_s2` saturate the tracker; `velocity_feedforward: true`
  removes steady-state lag on ramps.
- `kinematics.ik.*` — feedforward seed (previous *sent* `q`, not measured
  state), convergence tolerances, and branch-jump handling
  (`max_solution_jump_deg`, `branch_jump_rate_limit`,
  `branch_jump_damping_scale`) to keep IK jump-free under fast moves.
- `safety.dq_max_deg_s` / `safety.ddq_max_deg_s2` — the outer per-joint
  velocity/accel ceiling, plus optional `servo.output_moving_average_window`
  for final-stage boxcar smoothing.

Trade-off across the two regimes: for large/fast UMI teleop moves raise
`pose_track_smd.natural_frequency_*` (and the velocity/accel caps); for
wrist-camera-stable imitation rollout keep damping critical (`1.0`) and the
velocity/accel caps bounded so the camera view does not shake. Keep the real
Servo J profile at `servo_alpha: 1.0` unless the task is specifically collecting
LPF-off diagnostic evidence. Tracked example configs that predate this profile
may still carry legacy `servo_t2_sec` / `servo_alpha` values (e.g. `0.05` /
`0.5`); new and migrated physical-real configs use the `1.0` real profile above.

### Rbpodo Real Acceptance Sequence

Real rbpodo acceptance is staged. Later stages depend on clean artifacts from
earlier stages:

1. `read_only`: connect with `send_servo_commands: false`, verify valid joint
   state, low `state_age_us`, no fault latch, expected rbpodo backend, and no
   Servo J sends.
2. 500 Hz ACK-on no-op or controller simulation-mode Servo J:
   `servo.rate_hz: 500`, `servo_t1_sec: 0.002`,
   `disable_waiting_ack: false`.
3. ACK-off diagnostics, if explicitly requested, are socket/API send evidence
   only and do not become supported motion profiles.
4. Tiny real joint motion: not authorized by the documentation above. It
   requires a future explicit motion approval task and operator supervision.

Do not copy aggressive ACK-off or high-rate settings into a real baseline until
the matching acceptance artifacts have been reviewed.

Required log and state fields for review include:

- `send_duration_us`
- `ack_wait_duration_us`
- `ack_observed`
- `controller_acceptance_observed`
- `state_age_us`
- `error_code`
- M561/M568/M569/M570 when available
- `q_ref` and `q_actual`

## Unsupported Raw Script TCP Paths

Unsupported raw script TCP comparison backends were removed from the active
backend contract. Do not reintroduce direct raw script command paths as
production or comparison backends. Real-controller integration is supported
through `RbpodoBackend`; the mock backend remains the hardware-free test
surface.

`rt_script` is future work. It is not part of the current backend contract and
must not be introduced as an undocumented controller setting or hidden bypass.

## State Publication

State JSON should expose:

- top-level observed mode/backend
- command source state
- fault context, including a backward-compatible top-level summary and optional per-arm latched contexts
- send policy
- per-arm read result
- per-arm send result
- worker telemetry, including `command_drops_total`,
  `pending_overwrites_total`, `last_dropped_seq`, `last_enqueued_seq`,
  `last_dispatched_seq`, and `last_completed_seq`
- rbpodo raw diagnostics and interpretation status under
  `rbpodo_diagnostics` when using `RbpodoBackend`
- rbpodo async streaming contract fields, including top-level
  `async_streaming_enabled` / `async_streaming_mode` / `async_streaming_policy`
  and per-arm `async_streaming` counters plus ACK/reference supervision state
- startup joint wrapping diagnostics, including raw-preserving
  `q_range_wrapped` and optional `q_actual_normalized_for_safety_deg`
- TCP pose and quaternion when FK is available
- Cartesian solve/path telemetry when Cartesian modes are active

State publication is an operational truth surface. Do not hide backend or safety details behind a single `ok` field.
