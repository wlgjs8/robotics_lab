# rb_gui

`rb_gui` is the operator viewer/console for `rb_servo_server`. It subscribes to
the server state stream, renders robot/TCP/safety status, and sends UDP command
packets through `rb_servo_gui.command_client`.

## Motion Controls

The GUI sends only the public motion primitives:

- `JointTarget`
- `TcpPoseTarget`
- `TcpLinearMove`

The Init Motion button remains an operator label, but the packet is
`JointTarget` with per-arm `joint_target_profile: init_motion`.

GUI TCP nudge controls move the visible target marker and send absolute
`TcpPoseTarget` commands. Linear moves send `TcpLinearMove` with explicit timing
and orientation mode.

## Safety And Lease Behavior

The GUI derives button disabled state from live server state: stale/missing
state, invalid joints, fault latch, FK/TCP-pose availability, Cartesian gate
status, and command-source lease. The server remains the real-motion authority;
client checks are operator feedback and do not bypass server gates.

Lifecycle/safety controls include arm/disarm, emergency stop, reset fault,
freedrive, and safety floor/ROI/user plane controls. One-shot commands
auto-bracket the command-source lease per click (no operator take/release
control), and the server remains the lease arbiter.

The GUI also reads the tracked server config path exported by the stack
launcher. When `safety.floor_constraint.enable` or
`safety.user_floor_constraint.enable` is `false`, its corresponding controls
initialize OFF and disabled. A persisted user-floor plane is not restored, and
enable/disable requests are handled as local no-ops, so the GUI does not flood
the server with lifecycle commands that the configured safety capability cannot
accept. If the config cannot be read, capability remains unknown and the server
still makes the authoritative accept/reject decision.

## F/T Sensor Visualization

The scene deliberately separates two Robotous RFT64-6A01 frames. The small
`ft_sensor_measurement` triad is loaded from the viewer URDF and represents the
Pika/RFT sensor frame at the physical sensor origin, including its +90 degree
yaw. Missing or invalid URDF data disables only this reference overlay instead
of substituting a guessed transform.

The larger runtime triad is driven exclusively by the server's resolved
`force_control.compliance_frame_actual_stand` pose and its validity flag. The
red force arrow is `control_wrench_compliance`, expressed under that same
runtime frame; raw rbpodo EFT components are never drawn under the CAD axes.
The runtime overlay fails closed on stale state, an invalid pose, or a disabled
force controller. In the corrected real `tcp_origin` profile the small sensor
triad and large runtime triad must be parallel and differ only in origin. The
large runtime triad is the sole X/Y/Z reference for compliance testing; the
generic TCP pose triad is not. Independent GUI checkboxes control the sensor
and runtime overlays. All fields remain read-only; the GUI has no force-control
enable or tuning control.

## Gravity-Wrench / CoG Waypoint Calibration

The `조작 -> CoG / Gravity model` tab identifies one of two server-selected models from saved
joint waypoints. `rigid_payload` estimates physical payload mass and TCP-frame
CoG. `controller_compensated_linear` estimates only the orientation-dependent
residual left in controller-processed feedback; it is not a physical mass/CoG
result. Create at least five safe, orientation-diverse WayPoints first (for
example `joint1` through `joint10`), then choose the arm and matching prefix.

`Start` only freezes and validates the waypoint list and acquires the GUI lease;
it sends no motion target. Robot motion is renewed only while the operator holds
`Run/Continue`. The selected arm receives the ordinary leased/deadman
`JointTarget` path with `joint_target_profile: payload_identification`; the other
arm remains a stationary joint `Hold` and server-side force-Hold promotion is
suppressed for both arms while each identification packet is active. Releasing
the button lets the existing command timeout stop renewal. The operator-authored
waypoint order is not collision-free planning, so the complete swept volume must
be clear and an E-stop operator remains required.

At each arrived/settled pose, the 100 Hz state callback collects unique fresh
samples from one server state snapshot. Evidence schema v4 records
`raw_sensor_wrench`, the effective `T_tcp_sensor`, `wrench_tcp`, `gravity_tcp`,
actual joints, and the actual stand-frame TCP pose against the same freshness
value. That profile must explicitly declare `observation_model`. A
`rigid_payload` profile also requires `wrench_convention: payload_load |
sensor_reaction`; the GUI never guesses the sign. A noisy pose is stopped for
explicit Retry or Skip. For `controller_compensated_linear`, `Calculate` fits
independent 3x3 force and torque maps in `wrench_tcp = A*gravity_tcp + bias`,
reports held-out residuals, and emits a runtime-config candidate without the
constant bias (the normal Init Motion tare owns that bias). `Save report`
atomically publishes a successful
provisional JSON/CSV evidence bundle under `logs/cog_calibration/<run_id>/`.
If calculation or a fit bound rejects the capture, the GUI automatically saves
the same raw samples with a `BLOCKED / NOT APPLIED` diagnostic report; rejected
data is not lost merely because no payload estimate was accepted.

Every result is labelled `PROVISIONAL / NOT APPLIED`. The GUI does not edit
`stack_real.yaml`, `active_calibration.yaml`, or Rainbow controller payload
settings. Entering the identification profile also leaves that arm's force
motion inhibited after Stop/timeout; only a later successful Init Motion tare
clears the inhibit and permits the normal Cartesian compliance path again.

## Realtime Timing Health

The Status tab renders three read-only health cards for `SERVO_J`, robot
feedback/F/T, and model inference. Servo and feedback values come from the
server's optional `realtime_timing` rolling aggregate, rather than attempting
to reconstruct a 500 Hz loop from the 100 Hz state stream or the 10 Hz GUI
repaint. The cards keep fractional rates visible (for example `499.03 Hz`) and
show p95 period/jitter/latency, deadline misses and catch-up ticks, held
feedback frames, and each arm's feedback receipt phase relative to the 2 ms
scheduled servo tick. Controller-time freshness is labeled unverified when the
source timestamp is not trustworthy.

An optional Plotly panel keeps the most recent 30 seconds at 2 Hz display
sampling. Its aligned rows compare servo/feedback rates, p95 dispatch/jitter/
feedback age, left/right feedback phase plus deadline-miss count, and inference
latency/jitter. This history is for visual correlation; the numeric p95/max
values still come from the producer-side aggregates.

Inference timing is read from the optional `inference_timing` block on the
policy chunk overlay. Missing blocks remain backward compatible and are shown
as unavailable. These displays are diagnostic telemetry only; they do not
change loop scheduling, command behavior, or safety gates.

## Wrist-Camera Quality Diagnostics

The `카메라 품질` tab subscribes directly to the independent
`camera.bundle.wrist_left` and `camera.bundle.wrist_right` topics and analyzes
the two D405 color streams at 320x240. It reports separate metrics instead of a
single ambiguous score:

- Crété-Roffet no-reference blur effect (`0` sharp, `1` blurred), plus a
  stationary per-arm baseline learned after a one-second settle and a
  three-second collection window
- sparse PyrLK/RANSAC global image motion and a one-second RMS of the
  above-2-Hz residual (`shake`)
- optional actual exposure/gain metadata and estimated exposure smear in pixels
- time-aligned TCP linear/angular speed, joint-speed RMS, and
  `q_sent - q_actual` RMS from the server state stream

The baseline is learned only while fresh robot telemetry and image flow both
indicate that the arm is stationary. It resets on camera reconnection/serial
change or through the GUI button. Low-texture or unreliable-flow frames are
marked explicitly; blur values from different scenes should not be compared as
an absolute lens-quality threshold.

The fixed operator overlay also shows an at-a-glance `Camera Quality Monitor`
below `FT Monitor`, with raw Blur and Shake values ordered RIGHT then LEFT.

Preview images are off by default and, when enabled, update at 5 Hz. Scalar
samples are written for the entire GUI process to
`logs/camera_quality/camera_quality_<KST>.csv`; image pixels are never written
by this diagnostic. The feature is read-only and is never used as a motion
gate, controller input, or safety decision.

Environment overrides:

```bash
RB_GUI_CAMERA_QUALITY=0
RB_GUI_CAMERA_QUALITY_ENDPOINT=tcp://127.0.0.1:5600
RB_GUI_CAMERA_QUALITY_LEFT_TOPIC=camera.bundle.wrist_left
RB_GUI_CAMERA_QUALITY_RIGHT_TOPIC=camera.bundle.wrist_right
RB_GUI_CAMERA_QUALITY_CSV_DIR=logs/camera_quality
```

If camera_server is absent, a topic is missing, or OpenCV/ZMQ is unavailable,
only this panel remains waiting/disabled; GUI and stack startup continue.
RealSense `actual_exposure_us`, `gain_level`, and `auto_exposure` fields are
optional and display as `N/A` when the device/backend does not expose them.

## Head-Camera View

The same `카메라 품질` tab carries an optional head (D435 color) viewer below the
wrist previews. It is display-only: a separate ZMQ subscriber on the head bundle
group (`camera.bundle.stereo`, stream `head.color`) plus the same shared-memory
ring the wrist previews use. It never feeds recording, the model, or a safety
decision — policy inference keeps reading the wrist-only `camera.bundle.policy`
group, so enabling this view changes nothing about what the model sees.

The `head view 표시 (5 Hz)` checkbox is off by default and gates the receiver
itself, not just the panel: while off, no head frame is copied out of shared
memory (the source is 1280x720 at 30 fps). While on, one frame per 200 ms is
copied, decimated to ~480 px wide, and JPEG-encoded for the browser. The status
line shows the source/preview resolution, frame age, and staleness.

Rig dependence: `make cam-up` (head D435 + both wrists) publishes the head
bundle group, so the view is live. `make cam-up-wrists` has no head camera at
all, so the panel stays `waiting` — that is the expected wrist-only state, not
an error. The legacy `head_wrists.yaml` rig has no per-group split and carries
`head.color` on `camera.bundle`; point the topic override there for that rig.

Environment overrides:

```bash
RB_GUI_HEAD_PREVIEW=0
RB_GUI_HEAD_PREVIEW_ENDPOINT=tcp://127.0.0.1:5600
RB_GUI_HEAD_PREVIEW_TOPIC=camera.bundle.stereo
RB_GUI_HEAD_PREVIEW_STREAM=head.color
```

## Running Tests

```bash
python3 -m unittest discover rb_gui/tests
python3 -m compileall -q rb_gui/rb_servo_gui
```
