# rb_servo_gui

`rb_gui` is the browser GUI for `rb_servo_server` state visualization and
operator control. It no longer keeps mode-based client gates: **the server is the
sole real-motion authority** (safety filter, Cartesian gate, fault latch, lease,
deadman). The GUI is a faithful frontend — it offers every motion primitive in
every run mode and lets the server accept or reject each command.

The full control set (joint jog/velocity, lifecycle, InitMotion, TCP PTP/Linear/
Delta, streaming TcpTwist, and the Circle tab for the `TcpCircleMove` benchmark on
both arms) is wired in every mode. Whether a control is live is **derived from the
live server state stream** — per-arm FK/TCP-pose validity, the server Cartesian
gate (`cartesian_available` / `controller_simulation_streaming_cartesian_available`
/ `cartesian_unavailable_reason`), fault latch, and motion state. There is no
longer an env unlock: the former `RB_GUI_SIM_READINESS_*`,
`RB_GUI_CARTESIAN_AVAILABLE`, and `RB_GUI_ENABLE_TCP_POSE_COMMANDS` /
`RB_GUI_ENABLE_CONTROLLER_SIM_CARTESIAN` locks are retired (`RB_GUI_OBSERVED_MODE`
/ `RB_GUI_OBSERVED_BACKEND` remain display-only labels).

Real physical motion still requires the server's own gates
(`RB_ALLOW_REAL_ROBOT/MOTION/CARTESIAN` + site config + operator supervision +
E-stop); the GUI sending a real command does not bypass them. See
`rb_servo_server/docs/gui_operator_console.md`.

## Asset Check

The GUI resolves the RB3 URDF and stand meshes from
`RB_GUI_DESCRIPTIONS_DIR`. To verify assets resolve from the repository
checkout:

```bash
PYTHONPATH=rb_gui \
RB_GUI_DESCRIPTIONS_DIR=rb_servo_server/descriptions \
python3 -m rb_servo_gui.app --check-assets
```

The check prints the robot URDF path, stand mesh path, whether each exists,
and visualization dependency status. Missing dependency output includes:
`Install with python3 -m pip install -e rb_gui`.

After installing this package, the console entry point is `rb-servo-gui`. Bare
`python3 -m rb_servo_gui` is not the current module entry point unless a future
`rb_servo_gui/__main__.py` is added.

For Rainbow controller `pgmode` simulation, leave TCP display on `Auto`; it
selects `tcp_ref_stand` when the server recommends it. `tcp_actual_stand`
remains visible for physical-state inspection.

## rbpodo SpaceMouse pgmode Live View

The SpaceMouse pgmode profile uses separate state fanout ports so
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
