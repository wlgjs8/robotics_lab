# Viser GUI + Docker operator console

This milestone adds a browser operator console without changing ownership of the high-rate servo loop. `rb_servo_server` continues to own robot backend reads and servo sends; the GUI only consumes `StatePublisher` UDP snapshots and emits validated UDP JSON commands through the existing command protocol.

## Services and ports

Default compose stack:

- `rb_gui`: Python viser web GUI, HTTP `8080`, UDP state listener `50110`.
- `rb_simulator_left`: repo-local hardware-free simulator for the left arm.
- `rb_simulator_right`: repo-local hardware-free simulator for the right arm.
- `rb_servo_server`: C++ server, UDP command listener `50010`, using
  `config/dual_simulator_compose.yaml` for the simulator operator stack.

The GUI container does **not** mount `/var/run/docker.sock`. Container start/stop controls are status/manual only until a constrained ops helper is implemented.

## Simulator GUI stack

The repository root compose file defines the simulator GUI stack for manual
operator use:

```bash
cd /home/plaif/workspace/robotics_lab
make sim-local-up
```

`make sim-up` remains a compatibility alias for `make sim-local-up`.

The same-PC server uses `config/dual_simulator_compose.yaml`, which binds commands
inside the container on `udp://0.0.0.0:50010`. State publishes to
`udp://rb_gui:50110` through Docker Compose DNS. Static container IPs are not
required.

For split-PC simulator operation, run `make sim-backend-up` on simulator PC
`172.28.60.36`, then `make sim-control-up` on the control/GUI PC. The control
profile uses `config/dual_simulator_remote_172_28_60_36.yaml`.

The compose server image builds with Pinocchio enabled. The default simulator
config publishes FK TCP poses and enables simulator-only Cartesian IK, so the GUI
TCP target gizmos can send `TcpPoseTarget` commands after `ArmMotion` is active.
`cartesian_control.allow_in_real` stays false.

## Operator monitors

The GUI exposes live monitors as responsive fixed HTML overlays on the left side
of the viewport. This keeps monitoring out of the scrollable right-side control
tab panel. Older viser builds without `add_html()` fall back to a top-level
`Operator Monitors` folder in the root GUI panel.

- `Joint Monitor` shows per-arm actual joints in selectable `deg` or `rad`.
- `Pose Monitor` shows per-arm current `tcp_stand` pose. Position is
  displayed in `mm`; orientation is selectable as `deg` or `rad`.
- Overlay layout can be tuned with `RB_GUI_MONITOR_WIDTH_EM` and
  `RB_GUI_MONITOR_GAP_EM`. The width is a target maximum; narrow browser
  windows shrink the monitor cards instead of hiding them.

Host build/test remains:

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

GUI contract tests:

```bash
PYTHONPATH=../rb_gui python3 -m unittest discover -s ../rb_gui/tests
```

Mock smoke without a browser:

```bash
python3 tools/mock_gui_smoke.py
```

## Safety boundaries

- Real robot motion is out of scope and disabled in the GUI. Real mode is connect/status visibility only.
- Simulation motion is limited to the repo-local `rb_simulator` path until connect, valid state read, truthful `servo_j` send, stop/reset, hold, and low-amplitude jog tests pass with software-only artifacts. Rainbow Robotics external simulator/OVA and real robot validation remain out of scope.
- The stand frame axes are hidden; the visible 6D controls are left/right TCP target gizmos. They initialize from `tcp_stand` when the state stream provides it, otherwise from URDF FK, and fall back to the old joint marker estimate only when TCP/FK data is unavailable.
- TCP target buttons emit validated `TcpPoseTarget` UDP commands from the gizmo pose in mock/simulation-safe modes. The C++ Cartesian controller requires configured kinematics and Cartesian gates; when they are unavailable or disabled, it reports a safe failure and holds position.
- Joint jog requires a fresh valid state snapshot, clamps per-command step size, sets a bounded timeout, and never synthesizes `[0,0,0,0,0,0]` when state is missing or invalid.
- Desired mode and observed server mode are separate. Selecting a desired mode without an ops surface does not claim that the running C++ process changed config.
- GUI state comes from UDP snapshots only; it does not import or read robot backends.

## Troubleshooting

- **Disconnected/stale GUI:** confirm `rb_servo_server` is running and `state_pub_endpoint` targets the GUI listener (`rb_gui:50110` in compose, `127.0.0.1:50110` for host smoke).
- **Simulator stack:** `rb_simulator_left`, `rb_simulator_right`, and
  `rb_servo_server` are hardware-free compose wiring only. Use
  `docs/rb_simulator_dev.md` for the supported unit and local-smoke evidence.
- **Real guard:** motion buttons remain blocked by design. Do not use the GUI for real robot motion in this milestone.
- **Container controls disabled:** expected because the GUI has no Docker daemon authority. Start/stop remains an external manual operator action.
