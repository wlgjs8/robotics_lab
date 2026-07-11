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
freedrive, safety floor/ROI/user plane controls, and command-source lease
take/release.

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
that force control is active.

## Running Tests

```bash
python3 -m unittest discover rb_gui/tests
python3 -m compileall -q rb_gui/rb_servo_gui
```
