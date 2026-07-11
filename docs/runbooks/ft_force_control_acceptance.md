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
physical bring-up profile and currently exposes a dual-arm supervised
experimental Cartesian-admittance stage with an identical per-arm contact
profile.
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
  right:
    enable: true
```

Do not use controller `pgmode` as evidence of physical F/T dynamics.

Start the supervised experimental profile with `make run MODE=real`. It is a
bring-up exposure requested after the per-arm monitor captures, not evidence
that the full promotion ladder or production gate has passed. Any failure
returns this block to monitor mode.

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
| `T_tcp_sensor` | `[0, 0, -0.202642, 0, 0, pi/2]`; URDF/CAD-derived, axis acceptance pending | `[0, 0, -0.202642, 0, 0, pi/2]`; URDF/CAD-derived, axis acceptance pending |
| Positive force/torque axis check | pending | pending |
| Tool/payload mass | pending | pending |
| Payload center of mass in TCP | pending | pending |
| Sensor bias artifact | pending | pending |
| Residual tare procedure | automatic after successful left/both Init Motion; physical acceptance pending | automatic after successful right/both Init Motion; physical acceptance pending |
| Profile revision/hash | pending | pending |
| Reviewer and date | pending | pending |

Mounting evidence is recorded in `IMG_9188.JPG`. The image confirms the
joint-6 -> silver sensor/adapter stack -> Pika ordering; arm identity, per-arm
serials, adapter drawing, and measurement date still need to be added to the
accepted site artifact. The RFT64-6A01 source mesh matches the combined Pika
CAD at `attachment_site` z=15..45 mm; the explicit URDF measurement frame is at
the model's tool-side `sensor_site`, z=45 mm. This geometric match does not
replace the positive-axis load test below.

The transform convention is:

```text
point_tcp = T_tcp_sensor * point_sensor
```

Record a manual positive load on every force and torque axis. A sign or frame
mismatch is a hard failure; do not compensate for it by changing a contact
threshold.

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
