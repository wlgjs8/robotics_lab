# F/T force-control acceptance

## Current gate

`PRODUCTION REAL ENFORCEMENT: BLOCKED`

This runbook records the evidence required to promote the integrated F/T
monitor, guard, unilateral normal admittance, and Cartesian compliance path. It grants no motion
authority. The current rbpodo EFT state does not prove sensor presence, sensor
fault, or overrange status. The backend exposes a CobotData frame-acquisition
sequence, but this is not an independent sensor sequence, so the
real-enforcement gate cannot currently pass. The explicit
`supervised_experimental_real` config flag only exposes the implementation for
a supervised acceptance stage; it does not close this gate.

`stack_sim.yaml` remains force-off. The tracked `stack_real.yaml` is the single
physical bring-up profile and currently exposes Gate 3C supervised six-axis
Cartesian compliance in a fixed Hold. Production force behavior remains
blocked.
The production gate above remains
blocked because the EFT source does not provide independent sensor health:

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

Do not use controller `pgmode` as evidence of physical F/T dynamics.

Start the active gate with `make run MODE=real`. Both `safety.floor_constraint` and
`safety.user_floor_constraint` are disabled by explicit operator decision to
remove their approach velocity dampers. This also removes the TCP/gripper-tip
geometric floor backstop from every motion primitive. Wrist F/T cannot detect
every upstream-link or stand collision; ROI, self-collision, tracking,
lease/deadman, and the physical E-stop remain necessary but are not substitutes
for that removed floor plane.

The GUI must show both floor enforcement controls OFF/disabled and must skip a
persisted user-floor restore. `logs/stack/server.log` must not contain a repeated
`SetSafetyFloorEnabled` or `SetUserSafetyFloorPlane ... user_floor_disabled`
rejection stream. A repeated stream is a stop-and-fix condition before the sign
capture because it obscures the evidence log.

### Gate 1: dual-arm manual sign/frame capture

1. Keep flow-infer, teleop, jog, and gripper commands inactive. Have an operator
   at the E-stop and clear the complete swept volume of both arms.
2. Start `make run MODE=real`, perform the existing supervised Init Motion only
   if required for software zero, and then issue no motion commands.
3. Confirm both arms report `operating_mode=monitor`,
   `force_control_compliance_frame=surface`, accepted/fresh tare, and no fault.
   Any unexpected robot motion is an immediate stop condition because the F/T
   monitor itself has no motion authority.
   After Init Motion reports `done`, also confirm the command is terminal
   `Hold`; a gentle perturbation must not make the sent Hold reference ratchet
   along with measured joints while the cached Init Motion packet is still
   fresh.
4. For the current translation-axis gate, show the large runtime FT control
   gizmo and hide the generic TCP pose gizmo. Push the right arm once per
   runtime FT gizmo positive endpoint toward its origin in Z, X, and Y, leaving
   a clear neutral interval between loads. The observed wrench is the opposing
   reaction, so each accepted transformed component is positive. The earlier
   2026-07-12 capture mixed TCP- and FT-gizmo names and is not X/Y
   acceptance evidence. The operator declared the left assembly identical, so
   the same corrected candidate mapping is applied to both arms. Negative-force
   and torque-axis captures are deferred until a stage that can promote
   rotational or production force behavior.
5. Stop on stale/invalid tare, unexplained cross-axis response, reversed sign,
   hard-threshold telemetry, any safety/IK/backend fault, or any robot motion.
6. Preserve `logs/servo_log_<timestamp>.csv` (and the latest
   `logs/servo_log.csv` link) plus `logs/stack/server.log`. Do not promote the
   controller until both-arm logs are reviewed.

No additional logger is required for this gate: the existing CSV already
contains raw sensor, TCP, fast/control surface/compliance wrench, freshness,
tare, contact/threshold, TCP pose, and controller-state columns.

### Gate 2: translation-only Hold compliance

1. Keep flow-infer, teleop, jog, gripper commands, and all environment contact
   inactive. This stage tests only a fixed Hold in free space.
2. Confirm both arms have `ft_tare_state=accepted`,
   `operating_mode=cartesian_admittance`, `compliance_frame=tcp_origin`, and
   translation-only axes `[true,true,true,false,false,false]` before touching an
   arm. In the GUI, the small sensor-origin triad and large runtime control triad
   must be parallel (+90 degrees from the generic TCP triad) and differ only in
   origin. Stop if either arm is not zeroed and healthy or these axes disagree.
3. Start below the configured 1.5 N translation deadband and raise one load
   slowly just beyond it. The TCP must yield toward the hand's push (opposite
   the reported reaction wrench), while orientation remains held.
4. Release fully. The fixed command equilibrium must not ratchet with measured
   pose, and the translation offset must recenter smoothly to zero. Leave a
   clear settled interval before changing axes.
5. Validate Z, then X, then Y using only the large runtime FT control gizmo axis
   names. Hide the generic TCP pose gizmo during this capture. Use only the
   small force needed to observe motion;
   the preceding sign capture showed that a strong tangential push can cross the
   unchanged 7 Nm TCP torque hard limit through the 202.642 mm lever arm.
6. Stop immediately on unexpected-axis motion, orientation motion, chatter,
   equilibrium drift, stale/tare invalidation, IK/safety rejection, or any hard
   force/torque threshold telemetry. Preserve the CSV and server log for review.

This gate intentionally does not test policy/contact interaction. The next gate
will admit motion only when it does not increase the measured contact load while
retaining tangential and unloading motion; it must not be enabled until this
free-space compliance/recenter capture is reviewed.

The clean post-fix repeat is `servo_log_20260712_190425.csv`: 50,437 samples,
zero force-controller fault reason, zero hard-limit sample, and zero fault
latch. Right-arm X/Y/Z offsets each reached approximately 19.9 mm and returned
to numerical zero. This closes the Gate 2 late-recontact regression check; it
does not promote policy/contact behavior.

### Gate 3A: yaw Hold compliance

1. Use only `make run MODE=real`, Init Motion, and the terminal fixed Hold.
   Keep flow-infer, teleop, jog, gripper commands, environment contact, and the
   other arm inactive. Keep an operator at the E-stop and clear the full
   orientation swept volume.
2. Confirm accepted/fresh tare, `compliance_frame=tcp_origin`, and axes
   `[true,true,true,false,false,true]`. Roll and pitch must remain held.
3. Grasp near the TCP and apply a slow, nearly pure moment about the large
   runtime FT-control gizmo +Z axis. Do not push on a long lever: the unchanged
   hard limit is 7 Nm. Cross the 0.25 Nm quiet zone gradually.
4. The TCP must yaw with the applied twist, not against it, and the yaw offset
   must remain within 0.08 rad. Release fully; it must recenter smoothly to the
   fixed Hold orientation without roll/pitch motion or equilibrium ratcheting.
   Repeat twice with a settled neutral interval.
5. Stop immediately on opposite-direction rotation, roll/pitch motion, chatter,
   equilibrium drift, stale/tare invalidation, IK/safety/backend rejection, or
   any hard force/torque threshold. Preserve the CSV and server log.

The preceding translation log measured at most 1.18/1.14/0.43 Nm on Tx/Ty/Tz,
while no-contact torque norm had 99th-percentile magnitude 0.142 Nm. This is why
Gate 3A admits yaw first and does not yet admit roll/pitch. Payload mass and COM
remain zero in the tracked profile, so this is a pose-local, small-angle Hold
test only; do not change the starting orientation after tare.

The operator confirmed correct yaw direction and recenter in the 19:11 and
19:16 captures. `servo_log_20260712_191136.csv` and
`servo_log_20260712_191634.csv` contain zero force-controller fault reason and
zero hard-limit sample. Right yaw reached the configured 0.08 rad neighborhood;
the latter capture exercised both yaw directions. This closes Gate 3A.

### Gate 3B: roll/pitch Hold compliance and sensitive yaw

1. Use `make run MODE=real`, Init Motion, and terminal fixed Hold only. Confirm
   axes `[true,true,true,true,true,true]`, fresh accepted tare, and the same
   starting orientation used for tare.
2. On the right arm, apply a slow nearly pure moment around runtime FT-control
   +X (roll), then release completely and wait for recenter. Repeat twice.
3. Repeat independently around runtime FT-control +Y (pitch). Orientation must
   follow the applied twist and return to the fixed Hold orientation. Stop on
   reversed direction or material motion on the other rotational axis.
4. Recheck yaw with a smaller moment. Its deadband is now 0.15 Nm, stiffness is
   6 Nm/rad, and near-critical damping is 2.2 Nms/rad. At 0.3 Nm the static
   model predicts about 0.025 rad (1.43 degrees), versus 0.00625 rad
   (0.36 degrees) in the prior profile.
5. The common angular limits remain 0.08 rad, 0.15 rad/s, 1.5 rad/s2, and the
   unchanged hard torque limit is 7 Nm. Stop on chatter, equilibrium drift,
   stale/tare invalidation, IK/safety/backend rejection, or any hard threshold.
   Preserve the CSV and server log.

Payload mass and COM are still zero. Roll/pitch change gravity orientation more
directly than yaw, so this remains a small-angle, pose-local characterization;
do not use policy motion or change the initial orientation during this gate.

The operator confirmed that all three rotational axes followed the applied
moment and recentered. `servo_log_20260712_192634.csv` contains 62,015 samples,
zero force-controller fault reason, zero hard-limit sample, and final right
roll/pitch/yaw offsets numerically at zero. Each axis reached approximately
0.0794 rad during the capture. This closes Gate 3B.

### Gate 3C: uniform sensitive rotational compliance

All rotations now use the same profile:

```yaml
virtual_mass:    [..., 0.2, 0.2, 0.2]
damping:         [..., 2.0, 2.0, 2.0]
stiffness:       [..., 5.0, 5.0, 5.0]
wrench_deadband: [..., 0.12, 0.12, 0.12]
```

The selected 0.12 Nm deadband remains above the measured no-contact p99 values
of 0.068/0.074/0.005 Nm for Tx/Ty/Tz. At 0.2 Nm, each axis' static model predicts
0.016 rad (0.92 degrees), compared with 0.0083 rad (0.48 degrees) for the prior
yaw profile. The 0.08 rad angular limit is reached at approximately 0.52 Nm.

1. Use `make run MODE=real`, Init Motion, and terminal fixed Hold only.
2. Test right Roll, Pitch, then Yaw independently with a small moment. Start
   below the previous force and leave a complete recenter interval between axes.
3. Confirm each axis has comparable onset, displacement, and recenter behavior.
   Stop if no-contact jitter appears or one axis materially drives another.
4. Preserve the CSV and server log. Do not use policy motion or change the
   initial orientation while payload/COM remain uncharacterized.

### Gate 3D: block-coherent multi-axis release

The tracked real profile enables:

```yaml
blockwise_release_recenter: true
```

This gate specifically checks the multi-axis zig-zag reported after Gate 3C.

1. Use `make run MODE=real`, Init Motion, and fixed Hold only.
2. Apply a combined X/Y translation load, hold it briefly, then release one
   component while keeping the other loaded. The released component may brake,
   but must not spring back toward the equilibrium independently.
3. Release the remaining force. The translation offset must return on one
   visually coherent direction. Repeat with a combined Roll/Pitch moment at
   smaller amplitude, without changing the tared starting orientation.
4. In the CSV, confirm the partial-release interval sets
   `compliance_translation_recenter_deferred` or
   `compliance_rotation_recenter_deferred`. Full release must set the matching
   `*_recenter_coupled` field while `proposal_valid=1` and no hard limit/fault
   occurs.
5. Stop on a direction reversal, new oscillation, opposite-block motion, IK or
   safety rejection, or any hard force/torque event.

## Per-arm accepted profile

Record the accepted per-arm values in `stack_real.yaml` and preserve the
matching CSV artifact for each stage. Pending serials or transforms must remain
explicitly labelled as unaccepted estimates.

| Field | Left | Right |
| --- | --- | --- |
| Sensor manufacturer/model | Robotous RFT64-6A01; operator-confirmed, serial pending | Robotous RFT64-6A01; operator-confirmed, serial pending |
| Sensor serial | pending | pending |
| Acquisition firmware/driver | pending | pending |
| Nominal acquisition rate | pending | pending |
| Presence signal and evidence | pending | pending |
| Fault signal and evidence | pending | pending |
| Overrange signal and evidence | pending | pending |
| Backend frame sequence evidence | implemented; physical log acceptance pending | implemented; physical log acceptance pending |
| Independent sensor sequence/time evidence | unavailable | unavailable |
| `T_tcp_sensor` | `[0, 0, -0.202642, 0, 0, pi/2]`; inherited from identical right hardware | `[0, 0, -0.202642, 0, 0, pi/2]`; corrected translation-axis capture, 2026-07-12 |
| Positive force/torque axis check | +Fx/+Fy/+Fz and Tx/Ty/Tz behavior inherited from identical right assembly | +Fx/+Fy/+Fz direction/recenter observed in `servo_log_20260712_184841.csv` and clean repeat `servo_log_20260712_190425.csv`; yaw accepted with `servo_log_20260712_191136.csv` and `servo_log_20260712_191634.csv`; roll/pitch/yaw accepted by operator with `servo_log_20260712_192634.csv` |
| Tool/payload mass | pending | pending |
| Payload center of mass in TCP | pending | pending |
| Sensor bias artifact | pending | pending |
| Residual tare procedure | automatic after successful left/both Init Motion; physical acceptance pending | automatic after successful right/both Init Motion; physical acceptance pending |
| Profile revision/hash | `right-derived-identical-positive-force-ft-axis-20260712` | `physical-positive-force-ft-axis-20260712` |
| Reviewer and date | operator declaration, 2026-07-12 | operator + log review, 2026-07-12 |

Mounting evidence is recorded in `IMG_9188.JPG`. The image confirms the
joint-6 -> silver sensor/adapter stack -> Pika ordering; arm identity, per-arm
serials, adapter drawing, and measurement date still need to be added to the
accepted site artifact. The RFT64-6A01 source mesh matches the combined Pika
CAD at `attachment_site` z=15..45 mm; the explicit URDF measurement frame is at
the model's tool-side `sensor_site`, z=45 mm. This geometric match does not
replace the positive-axis load test below.

The URDF remains geometry evidence with a `+pi/2` sensor yaw. The former
yaw-zero interpretation mixed TCP and FT gizmo names and was invalidated by
the 18:32 compliance capture. The corrected supervised profile keeps the
runtime source/control orientation aligned with the FT sensor gizmo. The
transform convention is:

```text
point_tcp = T_tcp_sensor * point_sensor
```

Before production force behavior is promoted, record and review all three
torque-axis loads. A sign or frame mismatch is a hard failure; do not compensate
for it by changing a contact threshold.

The completed Gate 1 profile used `compliance_frame: surface`: x/y/z and
roll/pitch/yaw are expressed in stand-fixed axes at the TCP. Confirm the
preserved Gate 1 CSV field `force_control_compliance_frame=surface` and compare
`control_wrench_compliance` against the applied stand-axis load. This removes
moving-TCP orientation from the first sign decision. Gate 2 now uses
`compliance_frame: tcp_origin`: the controller axes follow the corrected
`+pi/2` FT orientation while the correction pivot stays at the TCP endpoint.
The 2026-07-12 18:48 runtime-FT-control-gizmo capture completed the right-arm
translation direction and recenter observation. That run exposed a
late-recontact jerk-governor fault; the 19:04 repeat completed without a
force-controller fault, hard-limit event, or fault latch. The subsequent 19:11
and 19:16 captures confirmed yaw direction/recenter without a controller or
hard-limit fault. The 19:26 capture then confirmed roll/pitch/yaw direction and
recenter without a controller or hard-limit fault. Gate 3C gives all three
rotations the same more-sensitive profile. Payload/COM remain pending.

For the floor contact channel, keep the stand-frame geometric normal pointing
outward (`+Z`). The installed sensor's reaction force points opposite that
normal when the floor pushes the TCP upward, so force-control telemetry uses
`compressive_force = -normal dot force_stand`. A valid upward floor-reaction
test must therefore produce positive measured/fast normal force without
changing the geometric normal or the outward unload direction.

## Characterization datasets

Every record must include:

- arm, sensor/profile revision, and fixture identifier
- host receive time, backend frame-acquisition sequence, and any independent sensor sequence/time
- raw sensor-frame wrench
- bias-corrected TCP-frame wrench
- modeled payload/gravity wrench
- fast external wrench and filtered control wrench
- robot joint state, TCP pose, TCP speed, and TCP acceleration
- sample age, source-stall age, presence, fault, overrange, and health reason
- controller tick and send-policy timestamp when latency is measured

Collect these datasets before choosing thresholds:

1. no-contact static windows at operating temperature
2. bias drift from cold start through the expected run duration
3. slow orientation sweep covering the task workspace
4. policy-equivalent free-space speed and acceleration envelope
5. manual sensor capacity/overrange exercise without robot motion

Report per-axis mean, standard deviation, robust high percentile, worst-case
residual, temperature/time drift, and missing/stalled sample bursts. Repeating
the same wrench value is not a fault when the acquisition sequence advances.

For every static capture, run:

```bash
python3 rb_servo_server/tools/analyze_ft_acceptance.py \
  logs/servo_log.csv --static-start-sec 2 --static-end-sec 10
```

Preserve its output with the CSV. Promotion requires `promotion_ready: true`
for both arms. The checker fails closed on FT freshness discontinuity, a
static force/torque norm already beyond the configured hard limits, or a
disagreement between the logged normal force and the normal projection
recomputed from the logged TCP quaternion and wrench. Its tare value is an
incremental static-window candidate only and must not be applied without the
payload/orientation and sign checks in this runbook.

The analyzer requires the force-control fast projection and norm columns added
with the value-return projection fix. Older captures are intentionally rejected:
their normal-force telemetry may have been produced from a dangling Eigen
expression and cannot be used as promotion evidence. Re-capture after rebuilding.

## Threshold and latency record

Do not enter placeholder force thresholds in production config.

For each proposed task/contact profile, record:

- allowed arm and task/fixture revision
- contact zone and unit normal in the stand/task frame
- allowed approach direction and maximum approach speed
- characterized free-space residual plus explicit margin
- soft threshold, debounce, hysteresis, and release threshold
- hard one-sample threshold below the accepted tool/environment ceiling
- raw sensor capacity/overrange ceiling
- sensor acquisition, host transport, 500 Hz loop, verdict, and backend-send
  components of detection-to-send-suppression latency
- no-predeceleration peak-force prediction and measured compliant-fixture result

The task profile fails if a soft threshold cannot be placed above the measured
free-space/dynamic residual while the hard threshold remains below the accepted
tool/environment ceiling.

## Promotion ladder

Each stage requires preserved logs, an operator, and an available E-stop. A
failure returns the system to the previous stage.

1. read-only sensor acquisition; no robot motion
2. static bias/tare and manual six-axis sign checks; no robot motion
3. payload orientation sweep under existing supervised motion procedure
4. dynamic free-space monitor-only replay inside the characterized envelope
5. monitor-only contact detection with a compliant fixture
6. enforcing guard only, force controller still off
7. zero-force one-axis unloading with a compliant fixture
8. small nonzero target force after zero-force acceptance
9. flow-infer planned contact with force ownership limited to the accepted normal

Promotion beyond monitor-only also requires, and the runtime now enforces the
applicable config/ownership parts of these requirements:

- `send_at_tick_start=false`
- output moving-average transition reset/bypass evidence
- same-tick fault/send-suppression tests
- an enforcing floor/ROI/contact-zone envelope outside the planned contact zone
- hard-fault server and Python motion-epoch/observation provenance invalidation
- DeltaTwist normal-axis projection with tangential-state preservation
- Cartesian loading projection and bounded SE(3) compliance on every enabled axis
- explicit sensor-origin frame/sign acceptance before TCP-origin promotion
- fixed Hold equilibrium with no measured-pose ratchet under sustained wrench
- zero-wrench release recentering to that equilibrium on x/y/z/rx/ry/rz
- block-coherent partial release and direction-preserving 3D translation/rotation return
- release-phase recontact brakes the return before loaded hold, without a fault
- Hold-to-policy first-delta projection without an equilibrium jump
- legacy guarded-mode reset interlock requiring a fresh post-event observation and chunk

In `cartesian_admittance`, soft-contact entry and release do not increment
`motion_epoch`: flow-infer inference, the active chunk, and gripper execution
continue while the server projects loading policy deltas and composes bounded
compliance. External-force hard faults still increment the epoch and invalidate
cached/in-flight chunks. All admittance state is committed only after IK, final
safety filtering, and accepted backend send. The older
`guarded_admittance` mode retains its contact/release epoch and synthesized-Hold
recovery behavior for regression compatibility.

During this stage, a Cartesian compliance limit is not itself a failed test:
`compliance_limit_reason=jerk_limited_motion_envelope` means the requested
deflection was reduced to preserve the configured offset, velocity,
acceleration, jerk, and step bounds. A run fails if that bounded condition
latches a controller fault, if any hard force/torque threshold is crossed, or
if the corrected target fails IK or a downstream safety gate. Review the
per-axis `compliance_limit_axes` together with the separate normal, transverse,
and rotational contact flags in the GUI and CSV.

The controller must remain proposal-valid throughout load and release sweeps,
including when velocity reaches its configured bound. The jerk-domain governor
must preserve a non-empty braking set and recenter without an
`ExternalForceLimit` or motion-envelope fault. Do not mask a recenter failure by
increasing the wrench moving average; the acceptance evidence is the selected
jerk/offset/velocity/acceleration telemetry and a zero-wrench return to the
fixed equilibrium.

Also reapply the same-axis load while a released offset is still recentering.
The return velocity must brake without crossing the command equilibrium or
latching a motion-envelope fault; after it stops, sustained load must not drive
the compliant offset back toward zero. Repeat both wrench signs for at least
one translation and one rotation before promotion.

Before flow-infer, validate `cartesian_admittance` in a fixed `Hold` one axis at
a time, followed by the Gate 3D combined-axis release test. The active Gate 3C
profile enables all six axes only for the accepted fixed-orientation,
pose-local supervised test. The
equilibrium pose/source must remain unchanged while held, the bounded offset
must oppose the wrench without a fault, and `compliance_recenter_active` must
drive every enabled offset back near zero after release. Stop immediately on an
equilibrium drift, repeated offset growth, unexpected-axis motion, or a hard
force/torque threshold event.

For the responsive profile, begin below the 1.5 N / 0.25 Nm deadband and raise
the load slowly. Verify first motion just outside the deadband, then verify that
release recenters without chatter before approaching the 20 mm / 0.08 rad
travel envelope. The hard thresholds are intentionally unchanged; do not use
them as a hand-guiding travel stop.

The F/T path is not safety-rated. It supplements but never replaces E-stop,
lease/deadman, stale-state, tracking-error, self-collision, floor, ROI, and
other final server-owned safety gates.

For automatic tare, leave the selected tool completely unloaded and clear of
contact before Init Motion. After measured arrival the server waits 0.5 s, then
accepts 500 fresh samples only when every force-axis standard deviation is at
most 0.75 N and every torque-axis standard deviation is at most 0.15 Nm.
Confirm `ft_tare_state=accepted` and `ft_tare_valid=1` in the GUI/CSV before
interpreting contact thresholds. This removes the installed tool's static
wrench only at the Init Motion orientation; it does not replace payload mass/COM
characterization.

## Gate close conditions

Keep real enforcement blocked or return it to blocked if any of these occur:

- sensor presence/fault/overrange/freshness cannot be proven
- frame, sign, payload, or tare evidence is incomplete
- unexplained false contact, stale sample, or source-sequence regression
- peak force or suppression latency exceeds its approved ceiling
- IK/safety/send rejection changes committed admittance state
- post-contact offset jump, policy normal-axis catch-up, or old chunk reuse
- the task leaves the accepted speed/acceleration/contact-zone envelope
