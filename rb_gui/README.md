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

## Running Tests

```bash
python3 -m unittest discover rb_gui/tests
python3 -m compileall -q rb_gui/rb_servo_gui
```
