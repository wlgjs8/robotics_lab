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

## F/T Sensor Visualization

The scene renders each Robotous RFT64-6A01 URDF/CAD measurement-frame estimate
as a read-only child of the live actual TCP frame. Its local pose is loaded from
the explicit `ft_sensor_measurement` link in the active viewer URDF; missing or
invalid URDF frame data disables the overlay instead of falling back to a
guessed origin. When the state stream is current and `eft_valid` is true, the
raw controller-reported force vector is drawn in the authored local axes and
the label shows all six raw wrench components. The label explicitly states
that the axes and sensor presence are unverified: `eft_valid` currently proves
finite decoding only. This display is diagnostic only and does not indicate
that force control is active. When the server publishes the optional
`force_torque`, `force_control`, and `motion_epoch` telemetry, the status tab
and Pose/FT monitor also show the declared source assurance, contact state, and
controller state. Those fields are read-only; the GUI has no force-control
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
