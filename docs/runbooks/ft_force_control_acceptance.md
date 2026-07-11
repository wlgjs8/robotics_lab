# F/T force-control acceptance

## Current gate

`REAL ENFORCEMENT: BLOCKED`

This runbook records the evidence required to promote the hardware-free F/T
pipeline and admittance controller. It grants no motion authority. The current
rbpodo EFT state does not prove sensor presence and does not expose an
independent acquisition sequence, sensor fault, or overrange status, so the
real-enforcement gate cannot currently pass.

Tracked `stack_real.yaml` and `stack_sim.yaml` must remain:

```yaml
force_control:
  provider: null
  enable: false
```

Do not use controller `pgmode` as evidence of physical F/T dynamics.

## Per-arm accepted profile

Create a site-local artifact for each arm. Do not commit device serials or
site-specific transforms to tracked stack configs.

| Field | Left | Right |
| --- | --- | --- |
| Sensor manufacturer/model | Robotous RFT64-6A01; operator-confirmed, serial pending | Robotous RFT64-6A01; operator-confirmed, serial pending |
| Sensor serial | pending | pending |
| Acquisition firmware/driver | pending | pending |
| Nominal acquisition rate | pending | pending |
| Presence signal and evidence | pending | pending |
| Fault signal and evidence | pending | pending |
| Overrange signal and evidence | pending | pending |
| Source sequence/time evidence | pending | pending |
| `T_tcp_sensor` | `[0, 0, -0.202642, 0, 0, pi/2]`; URDF/CAD-derived, axis acceptance pending | `[0, 0, -0.202642, 0, 0, pi/2]`; URDF/CAD-derived, axis acceptance pending |
| Positive force/torque axis check | pending | pending |
| Tool/payload mass | pending | pending |
| Payload center of mass in TCP | pending | pending |
| Sensor bias artifact | pending | pending |
| Residual tare procedure | pending | pending |
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

## Characterization datasets

Every record must include:

- arm, sensor/profile revision, and fixture identifier
- host receive time and independent source sequence/time
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

Promotion beyond monitor-only also requires:

- `send_at_tick_start=false`
- output moving-average transition reset/bypass evidence
- same-tick fault/send-suppression tests
- an enforcing floor/ROI/contact-zone envelope outside the planned contact zone
- server and Python motion-epoch/observation provenance invalidation
- DeltaTwist normal-axis projection with tangential-state preservation
- reset interlock requiring a fresh post-event observation and chunk

The F/T path is not safety-rated. It supplements but never replaces E-stop,
lease/deadman, stale-state, tracking-error, self-collision, floor, ROI, and
other final server-owned safety gates.

## Gate close conditions

Keep real enforcement blocked or return it to blocked if any of these occur:

- sensor presence/fault/overrange/freshness cannot be proven
- frame, sign, payload, or tare evidence is incomplete
- unexplained false contact, stale sample, or source-sequence regression
- peak force or suppression latency exceeds its approved ceiling
- IK/safety/send rejection changes committed admittance state
- post-contact offset jump, policy normal-axis catch-up, or old chunk reuse
- the task leaves the accepted speed/acceleration/contact-zone envelope
