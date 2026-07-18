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
damping:         [..., 1.55, 1.55, 1.55]
stiffness:       [..., 3.0, 3.0, 3.0]
wrench_deadband: [..., 0.10, 0.10, 0.10]
```

The selected 0.10 Nm deadband remains above the measured no-contact p99 values
of 0.068/0.074/0.005 Nm for Tx/Ty/Tz. At 0.2 Nm, each axis' static model predicts
0.033 rad (1.91 degrees). The 0.08 rad angular limit is reached at approximately
0.34 Nm.

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
   `compliance_rotation_recenter_deferred`. Full release should set the matching
   `*_recenter_coupled` field while the block remains in its soft envelope. A
   brief `*_recenter_coupled=0` together with
   `compliance_limit_axes`/`jerk_limited_motion_envelope` is the expected
   per-axis hard-envelope recovery fallback; require `proposal_valid=1`, no
   `ExternalForceLimit` latch, and coupling to resume after the recovery.
5. Stop on a direction reversal, new oscillation, opposite-block motion, IK or
   safety rejection, or any hard force/torque event.

### Gravity-wrench / CoG waypoint identification

This stage produces a provisional rigid-payload or controller-compensated
gravity-residual candidate; it does not apply or accept either model.

1. In the WayPoint tab, teach at least five one-arm joint poses with materially
   different tool orientations and a manually verified safe order. Name them
   with one prefix such as `joint1` ... `joint10`. A direct JointTarget sequence
   is not collision-free planning.
2. Clear the full swept volume, keep the other arm stationary, keep the tool
   unloaded and out of contact, assign an E-stop operator, and open
   `조작 -> CoG / Gravity model`.
3. Select exactly one arm, refresh/validate the prefix, and press `Start`.
   `Start` must not move the arm. Confirm the GUI lease and server profile are
   ready before holding `Run/Continue`.
4. Keep `Run/Continue` held while each target moves, settles, and collects the
   configured number of unique fresh samples. Releasing the button pauses target
   renewal through the normal command timeout. Do not steady the wrist by hand;
   stable external contact can look like a valid bias.
5. A pose outside the configured force/torque variance bound must stop for an
   explicit Retry or Skip. Stop immediately on stale/unhealthy F/T, hard-limit
   telemetry, fault, lease loss, unexpected compliance, or an unsafe path.
6. After at least five accepted poses, press `Calculate`. The server profile's
   explicit `observation_model` is part of the fit contract. For
   `rigid_payload`, also review `wrench_convention`, mass and TCP CoG. For
   `controller_compensated_linear`, review both 3x3 gravity matrices, bias, fit
   RMS, rank/condition and leave-one-pose-out residuals; do not call those
   matrices a mass or CoG. Then `Save report` for a successful provisional result. A rejected
   calculation automatically saves a `BLOCKED / NOT APPLIED` report and raw
   sample CSV; preserve that bundle with the matching servo CSV and server log.
   Evidence schema v4 must contain non-empty raw sensor wrench,
   `T_tcp_sensor`, actual joint, and actual TCP columns. In the JSON candidate,
   compare each pose's observed and predicted force/torque before changing a
   transform or fit bound.
7. The output is always `PROVISIONAL / NOT APPLIED`. Do not copy it into
   `stack_real.yaml` or the Rainbow controller until a separate review compares
   repeated and held-out pose results. A linear model must have a unique
   calibration id and cannot be combined with nonzero payload mass/CoG. Run
   right and left as separate sessions.
8. Identification intentionally leaves `payload_identification_inhibit=true`
   and the old tare invalid. With the arm unloaded at the intended start pose,
   run the normal Init Motion tare and confirm `tare_state=accepted` before
   resuming Cartesian compliance.
9. After separately reviewing and copying a linear runtime candidate, rebuild
   and first repeat the unloaded orientation sweep. Stop on any unexpected
   motion, stale/unhealthy F/T, hard-limit telemetry, or torque outside the
   reviewed evidence envelope. Only then perform a gentle single-axis TCP push.
   Passing means the commanded translational compliance no longer produces the
   previous unintended rotation and all released axes recenter. This supervised
   observation is an experimental gate, not production force acceptance and not
   proof that the controller's internal compensation assumption is true.

The tracked real acquisition profile reuses the existing 1.5 degree InitMotion
waypoint tolerance, 0.5 second tare settling time, 500 fresh samples, and measured
stationary 0.75 N / 0.15 Nm variance bounds. Fit and condition bounds gate only
the provisional calculation; they do not authorize automatic payload or force
control changes. Controller pgmode has no deterministic F/T gravity fixture, so
its profile remains disabled.

Two 2026-07-16 right-arm captures passed the per-pose noise checks at the same
seven orientations. The rigid payload model remains invalid (about 3.8 N force
RMS), while the controller-compensated linear residual fits the repeated pose
means at 0.085--0.088 N force and 0.0116--0.0120 Nm torque component RMS. The
force and torque matrices repeated within about 1.84% and 1.52% Frobenius norm,
respectively. The second schema-v4 report is
`20260716T060343Z-421f0157` (SHA-256
`76a8bdb828a7d624c41e9da44ed0b2a00a6b9aedac3b096fee2049e1fa360ad1`).

Under the operator-requested assumption that Rainbow feedback has already
undergone gravity/source processing, that report is copied to the tracked
right-arm runtime profile for the next supervised gate. This remains an
experimental residual model, not physical CoG acceptance or proof of the
controller assumption. Its maximum leave-one-pose-out force/torque norm is
0.751 N / 0.121 Nm; the torque evidence is close to and on one component above
the 0.10 Nm rotational deadband, so an unloaded orientation sweep must precede
contact.

The post-identification Init Motion attempt in `servo_log_20260716_150320.csv`
did not authorize contact: right tare was rejected after the one-second window
reached 3.90 N / 0.695 Nm maximum force/torque standard deviation while the arm
pose remained nearly stationary. Repeat the tare with the tool, sensor cable,
and wrist fully unloaded and undisturbed. Do not push unless telemetry shows the
new calibration id, `tare_state=accepted`, the identification inhibit cleared,
and healthy/non-stale F/T.

### 2026-07-16 off-origin rotation regression and monitor A/B

The later `servo_log_20260716_152654.csv` did apply calibration
`right-20260716T060343Z-421f0157`, accepted the right-arm tare, and kept the F/T
stream healthy and fresh. It nevertheless recorded about 13.76 seconds of
rotational contact and a maximum rotational compliance offset of 0.079394 rad,
near the configured 0.08 rad limit. No force hard limit or controller fault
explains the motion.

The strongest push intervals are consistent with a real moment from an
off-origin contact: `tau = r x F` explains about 92--93% of the measured torque,
with an inferred contact point about 0.114--0.121 m behind the software TCP (or
about 0.082 m forward of the F/T measurement origin). The operator confirmed
that the capture mixed fingertip-plane and Pika-body/palm pushes, so it cannot
decide whether a fingertip-centre push is also misframed. A pointwise replay of
the same capture through the prior raw/zero-payload model gives about 0.509 Nm
torque RMS versus about 0.454 Nm with the configured linear residual model; this
capture therefore does not support reverting to raw data.

The tracked real profile is temporarily fail-closed at
`operating_mode: monitor`, `allow_in_real: false`, and
`supervised_experimental_real: false`.
Keep the reviewed right-arm model and transform unchanged while collecting this
A/B evidence at one fixed, accepted Init Motion pose:

1. Confirm the right calibration id, accepted tare, cleared identification
   inhibit, and healthy/non-stale F/T. Do not change tool orientation afterward.
2. With neutral unloaded intervals between trials, push the centre of the
   fingertip TCP plane in `+X`, `-X`, `+Y`, and `-Y` at only 3--5 N. Record the
   exact contact point and verify that the compensated TCP torque stays near the
   stationary noise envelope and below the rotational contact threshold.
3. Repeat the same directions on one marked Pika-body point. Measure and record
   that point relative to the fingertip TCP plane; compare the observed torque
   sign and magnitude with `tau = r x F` using that measured lever arm.
4. Stop on stale/unhealthy F/T, a rejected tare, unexpected robot motion, a
   hard-limit/fault indication, or evidence outside the configured envelope.

Analyze each capture offline with
`scripts/analyze_ft_application_point.py` (per-push baseline-subtracted
application point and `tau = r x F` verdict; thresholds below match the
3--5 N protocol pushes):

```bash
python3 scripts/analyze_ft_application_point.py logs/servo_log_<run>.csv \
    --arm right --enter-n 2.5 --exit-n 1.5 --min-delta-n 2 \
    --expect-r 0,0,0 --tol-mm 15 --json artifacts/ft_application_point_<run>.json
```

For the marked body-point branch, replace `--expect-r 0,0,0` with the measured
lever arm of that point in the TCP frame. The script also flags a missing
residual tare (large quiet baseline; auto tare fires only after Init Motion)
and reports a whole-log correlation fit whose effective lever should sit at
the tool CoM / cable clamp -- a value behind the sensor measurement face
(z < -0.2026 m) on a push-free capture is a sensor-origin-offset suspect.

Promotion back to six-axis Cartesian admittance requires both branches to pass:
fingertip-centre torque must remain near zero, and the marked body-point torque
must match the measured lever arm in sign and magnitude. A fingertip-centre
moment instead blocks promotion and requires correction of the sensor/TCP frame
or controller wrench convention before any further force-motion run.

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
| Gravity compensation model | `rigid_payload`, zero mass; arm-specific residual capture pending | `controller_compensated_linear`, experimental supervised gate |
| Gravity model evidence | pending | `right-20260716T060343Z-421f0157`; schema v4; report SHA-256 `76a8bdb828a7d624c41e9da44ed0b2a00a6b9aedac3b096fee2049e1fa360ad1` |
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
