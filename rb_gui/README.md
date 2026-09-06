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

## Gripper Controls

The `그리퍼` operator tab drives the Pika grippers through `gripper_server`
(`robotics_lab.gripper_cmd.v1` on UDP `127.0.0.1:50410`, overridable with
`RB_GUI_GRIPPER_CMD_ENDPOINT=udp://host:port`). Percent is `100` = open,
`0` = closed.

- Per-arm sliders (`Left/Right gripper %`) are both setpoint and live readout:
  `gripper_state.v1` feedback is written back into them, so only a genuine
  operator move emits a command, and a move holds auto-sync briefly so the
  operator's value is not overwritten before the jaws react.
- `양팔 그리퍼 열기` / `양팔 그리퍼 닫기` send one both-arm setpoint at the
  slider end stops (100 / 0). Same wire path, schema and authority as a slider
  move — one packet is enough because `gripper_server` holds the last setpoint
  (`on_stale: hold`). Unlike the sliders they command unconditionally: a click is
  unambiguous, so it works before the sliders have ever synced to hardware. The
  result is reported in `Last action`.

This is a gripper-only path; it carries no arm motion and takes no arm command
lease. Real gripper actuation still needs `gripper_server` running against the
pika backend.

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

## Self-Collision Highlight

While `self_collision.violated`, the viewer paints the colliding parts
translucent red. The collision model has FIVE body groups and only the ones the
violating pair names light up:

| group | monitor geometry |
| --- | --- |
| left arm / right arm | `<left_prefix>`/`<right_prefix>` arm links (`link0`..`link6`) |
| left gripper / right gripper | the Pika hulls the monitor attaches at `attachment_site` — `<prefix>pika_gripper_base`, `<prefix>pika_finger_left/right` (legacy single `<prefix>pika_gripper`) |
| stand | everything with no arm prefix: stand hulls, `ground_plane`, `external_box_*` |
| cell structure | the unified URDF's `env_*` links that carry a `<collision>` — the riser under the stand base plate |

So two grippers touching lights the two grippers, not two whole arms, and an arm
folding onto the stand leaves that arm's gripper normal. `env_*` boxes are `add_box`
handles rather than URDF meshes, so their highlight is the box's own colour swapped to
red and restored from `environment_rgb`; they share one group because the server gives
them one barrier class.

The groups come from the geometry NAMES of the near pairs that are IN HARD
VIOLATION — `clearance_m < d_hard_m`, each pair against **its own** published floor.
Side is decided by the manifest's `left_prefix`/`right_prefix`, not by searching for
the words "left"/"right": the unified URDF has stand links named `stand_left_arm_base`,
and the articulated gripper has `<right_prefix>pika_finger_left`.

`d_hard_m` / `d_slow_m` (and the `intra_arm` / `gripper_gripper` flags) are published
per near pair by the server because **nearest is not violating**. `near_pairs` is
ordered by RAW clearance while the monitor enforces five bands with floors an order of
magnitude apart:

| band | floor (`stack_real.yaml`) |
| --- | --- |
| self — arm↔arm, arm↔stand | 40 mm |
| cell structure (`env_*`) | 25 mm |
| gripper↔gripper | 25 mm |
| intra-arm (same arm folding) | 5 mm |
| external (floor / `ground_plane`) | 3 mm |
| external keep-out box | 10 mm |

Measured 2026-09-06 (`servo_log_20260906_131740.csv`, 53,795 ticks): the RB5's
structural intra-arm `link3_1 ↔ link5_0` pair sits at 22.9–23.6 mm — never violating
its own 5 mm floor — yet is the *nearest* pair on **99.4 %** of ticks. Keying off
`near_pairs[0]`, or banding every pair against the single 40 mm self floor, therefore
(a) painted that structural pair hard-red continuously and (b) named one arm as the
collider while an arm↔stand pair breaching its 40 mm floor anywhere in 40…23 mm went
unmarked. The per-pair floor is the fix; the same numbers come from
`nearPairHardFloorM`/`nearPairSlowBandM`, which `buildCollisionConstraints` enforces
with, so display and enforcement cannot drift.

Fallbacks, each strictly more conservative than the last: no pair carries a usable
`d_hard_m` (older server) → `near_pairs[0]`; no `near_pairs` → the coarse `pair`
category, where the arm groups include their gripper; unknown `pair`, or a violated
verdict whose breaching pair fell outside the published near list → all red.
External-box hits stay both-arms (that telemetry is per-box, so it cannot name a
side). If the viewer's arm URDF cannot be split into arm and gripper mesh nodes, the
pair of groups collapses back to the whole arm.

Enforcement was never affected: `violated` is a per-category OR over **every** checked
pair. Only the display's "which parts" answer used the nearest pair.

The red overlay REPLACES what it represents rather than overlapping it (z-fighting
at an identical configuration): in pgmode real it replaces the matching solid
links at `q_actual`; in pgmode simulation it replaces the commanded ghost at
`q_sent` and the solid robot keeps showing the true state.

## F/T Sensor Visualization

The `FT Monitor` card in the fixed operator overlay is back, reading the
`force_torque` / `force_control` blocks the rebuilt server publishes per arm. The
sensor-CAD frame and the runtime compliance-frame triad with a force arrow are
still gone.

The card is ONE TABLE over BOTH ARMS: rows are channels, columns are `LEFT` and
`RIGHT`. The question it answers is a comparison — which arm is feeling what —
and the per-arm stack it replaced put the two halves of that answer a scroll
apart. It fits its box without scrolling; the optional rows (`lever [mm]`,
`dev [mm]` / `dev [deg]` / `gate`) only appear while somebody is actually pushing
or a law is actually covering.

The numbers are `force_torque.comp_stand_axes_at_tcp` — the compensated wrench at
the TCP in STAND axes, the same surface the force law consumes. Stand axes because
this card is read by a human standing at the cell, and stand X/Y/Z are the
directions they can point at. That surface is already three subtractions deep:

```
raw  -  bias(tare)  -  tool gravity  ->  2 N / 0.5 Nm deadzone
```

so a resting arm reads `0.00` and so does the first ~2 N of any push. The
deadband is not a display choice — it is what the controller acts on, and a
monitor showing the pre-deadzone value would disagree with the arm. `load [kg]`
is the one channel that escapes it (a heavy low-pass on the pre-deadzone force),
which is how the ~200 g the band flattens to zero stays readable; a trailing `~`
means the estimate has not settled.

`--` and `0.00` are DIFFERENT answers and the card never collapses them:

| `zero` cell | channels | means |
| --- | --- | --- |
| `no stream` | `--` | the server is not publishing the block (old binary, or a GUI started before it did) |
| `off` | `--` | `force_torque.<arm>.enable: false` |
| `no sensor` | `--` | the liveness check found a flat stream; every compensated channel is pinned to zero upstream |
| `invalid` | `--` | the wrench was not six finite numbers |
| `영점 필요` | `0.00` | connected but never tared |
| `영점 OK` | live | tared |

Before a tare the channels read a TRUE zero rather than the sensor's own offset
(~20-40 N on this cell). Nothing is acting on that offset: the server refuses to
let any law cover an untared arm (`F/T has no bias yet - run a tare before
enabling compliance`), so zero is the honest reading of "no measured change from
a zero you have not set yet". Press `F/T 영점` in the safety panel to set it; the
server averages 250 ticks (0.5 s).

The tracked real stack also enables automatic tare after InitMotion. Pressing
InitMotion invalidates that arm's old bias immediately, but sampling does not
start during the move. The server waits for the init sequencer to finish, the
configured settle interval to pass, and last-sent joint speed to fall below the
configured threshold; it then runs the same 250-tick tare. The FT card exposes
the automatic-tare stage and reason so `awaiting_init`, `settling`, collection,
and an untared refusal remain distinguishable. Automatic tare cannot see a part
in the gripper or a hand on the wrist; the operator check is unchanged.

## Gravity-Wrench / CoG Waypoint Calibration

Removed. The payload-identification session (lease acquire, per-pose target
renew, disabled-reason gate, session end) identified tool mass and centre of
mass from wrench samples and cannot work without a sensor. The server rejects
`joint_target_profile: payload_identification` at parse, so the GUI no longer
offers it.

Payload identification remains removed even though the controller-manager-
referenced force pipeline and FT Monitor are live. Tool mass/COM calibration is
owned by controller-manager rather than this GUI session. Archived v1 design:
`docs/archive/force_control_v1/`.

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

## Cell Furniture (Work Tables And Stand Riser)

The viewer draws the room the arms actually stand in: two 800 mm work tables and
the riser that carries the stand base plate above them. 안전 탭 → **작업 셀 구조물
(테이블/라이저)** toggles them (default ON; solid geometry occludes the arms from
below, which is why the toggle exists).

The geometry is NOT in this package. Any link named `env_*` that is rigidly fixed
to the `stand` link of the unified URDF is picked up and drawn, with its `<box>` /
`<mesh>` and its `<material><color>`; adding a third table is a URDF edit, not a
code edit. The definitions live in `rb_servo_server/tools/make_rb5_850e_urdfs.py`
(`ENVIRONMENT`), which records how each dimension was obtained. Hardcoding
furniture poses here is the mistake the stand already taught us — see
`_stand_visual_from_urdf` in `scene.py`.

**Visual only.** `CollisionMonitor` builds from
`buildGeom(..., pinocchio::COLLISION, ...)`, so links carrying only `<visual>`
add zero geoms and the checked pair set is unchanged (verified: 49 collision
geoms before and after). The tables do not brake the arms and must not be read as
clearance.

**라이저 높이 (mm)** in the same folder retunes the riser live: it grows downward
from the stand base plate's underside and the tables travel with it, so the status
line shows the resulting table-top z. The value persists to `env_riser_height_m`
in `~/.rb_servo_gui/settings.json` and is re-applied at startup; every adjustment
recomputes from the URDF baseline, so repeated edits never compound.

The URDF now ships the measured 295 mm (table top z -310 mm), taken by parking
both gripper tips down on the table: the TCP *is* the fingertip plane, so a flat
tip reads the surface. Re-measure the same way rather than by tape — the tape
reads the columns alone and missed the end plates by ~10 mm. Anything the
operator dials in here should go back into `ENVIRONMENT` with its evidence.

## Running Tests

```bash
python3 -m unittest discover rb_gui/tests
python3 -m compileall -q rb_gui/rb_servo_gui
```
