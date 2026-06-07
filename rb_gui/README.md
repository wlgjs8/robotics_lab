# rb_servo_gui

`rb_gui` is the browser GUI for `rb_servo_server` state visualization and
operator-facing simulator controls. Real robot motion gates remain outside the
GUI.

For an rbpodo controller in `pgmode` simulation (controller-simulation), the GUI
can drive its full existing control set (joint jog, lifecycle, TCP PTP/Linear/
Delta, plus the Velocity jog tab for streaming JointVelocity / TcpTwist, and the Circle tab for the TcpCircleMove benchmark on both arms) when
the operator opts in with `RB_GUI_ENABLE_TCP_POSE_COMMANDS=1` and
`RB_GUI_ENABLE_CONTROLLER_SIM_CARTESIAN=1` (plus `RB_GUI_OBSERVED_MODE=simulation`,
`RB_GUI_OBSERVED_BACKEND=rbpodo`). Real mode stays connect/status-only and the
GUI never sets `RB_ALLOW_REAL_CARTESIAN`. See
`rb_servo_server/docs/gui_operator_console.md`.

## rbpodo Circle Live View

For rbpodo controller-simulation circle benchmarks, the GUI consumes two UDP
telemetry streams:

```text
rb_servo_server state fanout
  udp://127.0.0.1:50151 -> benchmark recorder
  udp://127.0.0.1:50161 -> rb_gui state receiver

rbpodo_circle_tracking_benchmark overlay
  udp://127.0.0.1:50261 -> rb_gui circle overlay receiver
```

The state stream contains robot/server telemetry such as `tcp_actual_stand`,
`tcp_ref_stand`, Cartesian gate status, and
`physical_motion_expected=false`. The overlay stream contains desired circle
geometry and running benchmark metrics. The overlay is not robot state and does
not carry commands.

Run the GUI against the fanout state port and overlay port:

```bash
tools/rbpodo_circle_gui.sh --profile stable
```

The launcher sets `RB_GUI_DESCRIPTIONS_DIR` to
`rb_servo_server/descriptions` so the RB3 URDF and stand meshes resolve from
the repository checkout.

The stable profile listens on state port `50161`; the GENE-style stress
profile uses:

```bash
tools/rbpodo_circle_gui.sh --profile gene
```

The equivalent manual command is:

```bash
PYTHONPATH=rb_gui \
RB_GUI_DESCRIPTIONS_DIR=rb_servo_server/descriptions \
RB_GUI_STATE_BIND=0.0.0.0 \
RB_GUI_STATE_PORT=50161 \
RB_GUI_CIRCLE_OVERLAY_BIND=udp://0.0.0.0:50261 \
python3 -m rb_servo_gui.app
```

If TCP markers appear but the robot URDF does not, run the asset check:

```bash
PYTHONPATH=rb_gui \
RB_GUI_DESCRIPTIONS_DIR=rb_servo_server/descriptions \
python3 -m rb_servo_gui.app --check-assets
```

The check prints the robot URDF path, stand mesh path, whether each exists,
and visualization dependency status. Missing dependency output includes:
`Install with python3 -m pip install -e rb_gui`.

After installing this package, the console entry point is:

```bash
RB_GUI_STATE_BIND=0.0.0.0 \
RB_GUI_STATE_PORT=50161 \
RB_GUI_CIRCLE_OVERLAY_BIND=udp://0.0.0.0:50261 \
rb-servo-gui
```

Bare `python3 -m rb_servo_gui` is not the current module entry point unless a
future `rb_servo_gui/__main__.py` is added.

For Rainbow controller `pgmode` simulation, leave TCP display on `Auto`; it
selects `tcp_ref_stand` when the server recommends it. `tcp_actual_stand`
remains visible for physical-state inspection, but pgmode simulation tracking
should be scored against `tcp_ref_stand`.

`policy_runner` is a separate command source. The GUI does not route benchmark
commands through `policy_runner`; it only receives server state and benchmark
overlay telemetry for this workflow.

## rbpodo SpaceMouse pgmode Live View

The ACKON500 SpaceMouse pgmode profile uses separate state fanout ports so
`policy_runner` and the GUI do not bind the same UDP socket:

```text
rb_servo_server state fanout
  udp://127.0.0.1:50366 -> rb_gui / viser live viewer
  udp://127.0.0.1:50376 -> policy_runner safety readback
```

Launch the viewer with:

```bash
tools/rbpodo_pgmode_spacemouse.sh gui
```

The launcher prints the browser URL and binds the viewer state receiver to
`udp://0.0.0.0:50366`. This disables the circle overlay and only listens to
server state. The command does not send robot commands, does not read
SpaceMouse HID devices, and does not route SpaceMouse packets; those commands
route only through `policy_runner`.

The Status tab includes a compact pgmode line for this workflow, for example:

```text
pgmode_sim: backend=rbpodo run_mode=real operation_mode=simulation physical_motion_expected=false cartesian_available=true policy_runner_lease=active source=policy_runner command=TcpTwistLocal selected_tcp=tcp_ref_stand
```

If telemetry needed to prove the controller-simulation boundary is missing,
the line includes `degraded missing=...`. If `physical_motion_expected` is not
explicitly `false`, the line includes
`warning=physical_motion_expected_not_false`.

Leave TCP display on `Auto`; it selects `tcp_ref_stand` when the server
recommends `reference_for_controller_simulation`. Use `both` when inspecting
physical-state `tcp_actual_stand` alongside the controller-simulation
reference `tcp_ref_stand`.
