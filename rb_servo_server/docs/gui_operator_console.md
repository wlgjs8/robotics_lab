# Viser GUI operator console

This milestone adds a browser operator console without changing ownership of the high-rate servo loop. `rb_servo_server` continues to own robot backend reads and servo sends; the GUI only consumes `StatePublisher` UDP snapshots and emits validated UDP JSON commands through the existing command protocol.

## Services and ports

The operator stack (`rb_gui` + `rb_servo_server`) runs
natively, not in Docker:

- `rb_gui`: Python viser web GUI, HTTP `8080`, UDP state listener from the
  active server config.
- `rb_servo_server`: C++ server, UDP command listener from the active server
  config, using a site-local mock YAML for hardware-free mock or the rbpodo
  controller `pgmode` simulation via `make run MODE=sim`.

## GUI stack

The native operator stack runs via the repository-root `Makefile`:

```bash
cd /home/plaif/workspace/robotics_lab
make run MODE=sim
```

Build/install the stack first with `make build` after editing source. The
server binds commands on the active config's `network.command_bind`
(`udp://127.0.0.1:50256` in the tracked stack configs) and publishes state to
the GUI fanout endpoint (`50366`).

The server is built with Pinocchio enabled. The mock / controller-simulation
config publishes FK TCP poses and enables Cartesian IK, so the GUI TCP target
gizmos can send `TcpPoseTarget` commands after `ArmMotion` is active.
`cartesian_control.allow_in_real` stays false.

### Execution gating (client-side lock retired)

The GUI's client-side real/sim execution lock has been **retired** in code
(`rb_gui/rb_servo_gui/safety.py` `blocked_reason` / `tcp_command_disabled_reason`).
Joint, lifecycle, and TCP PTP/Linear/Delta controls are now emittable in **every**
run mode (mock, simulation, real); the GUI no longer forces real mode to
connect/status-only and no longer gates on a mode/backend match. The old opt-in
env flags (`RB_GUI_ENABLE_TCP_POSE_COMMANDS`,
`RB_GUI_ENABLE_CONTROLLER_SIM_CARTESIAN`, `RB_GUI_OBSERVED_MODE/BACKEND` locks)
are retired and no longer required.

What the GUI still enforces client-side is **non-gating** readiness only: a fresh
valid state stream, joint-state validity, simulation readiness tests (sim mode),
fault-latch, and FK/TCP-pose availability for Cartesian.

Authority for real motion has moved entirely to the **server**, and is now
config-driven, not env-gated (the legacy `RB_ALLOW_REAL_*` /
`RB_ALLOW_RBPODO_CONTROLLER_SIM_*` / `RB_RBPODO_PGMODE_SIMULATION_CONFIRMED` env
gates were removed from the server runtime): the site-local config
(`cartesian_control.allow_in_real`), the SafetyFilter, tracking-error latch,
async URDF-mesh self-collision guard, lease, deadman, operator supervision, and
the hardware E-stop, all fail-closed. The GUI itself never sets any `RB_ALLOW_*`
env. For rbpodo `pgmode` simulation (`operation_mode=simulation`,
`physical_motion_expected=false`), the server-side controller-simulation
Cartesian carve-out is likewise config-driven
(`cartesian_control.allow_in_controller_simulation: true` +
`servo.allow_controller_simulation_motion: true`) and still applies on the server.

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

- The GUI's client-side real-motion lock is **retired** (see "Execution gating" above): controls emit in every run mode, and the server config plus SafetyFilter, collision guard, lease, and deadman decide whether real motion actually executes. The GUI never sets `RB_ALLOW_*`.
- Simulation motion is limited to mock mode and the rbpodo controller `pgmode` simulation until connect, valid state read, truthful `servo_j` send, stop/reset, hold, and low-amplitude jog tests pass with software-only artifacts. Rainbow Robotics external simulator/OVA and real robot validation remain out of scope.
- The stand frame axes are hidden; the visible 6D controls are left/right TCP target gizmos. They initialize from `tcp_stand` when the state stream provides it, otherwise from URDF FK, and fall back to the old joint marker estimate only when TCP/FK data is unavailable.
- TCP target buttons emit validated `TcpPoseTarget` UDP commands from the gizmo pose in mock/simulation-safe modes. The C++ Cartesian controller requires configured kinematics and Cartesian gates; when they are unavailable or disabled, it reports a safe failure and holds position.
- Joint jog requires a fresh valid state snapshot, clamps per-command step size, sets a bounded timeout, and never synthesizes `[0,0,0,0,0,0]` when state is missing or invalid.
- Desired mode and observed server mode are separate. Selecting a desired mode without an ops surface does not claim that the running C++ process changed config.
- GUI state comes from UDP snapshots only; it does not import or read robot backends.

## Troubleshooting

- **Disconnected/stale GUI:** confirm `rb_servo_server` is running and the
  configured state publisher targets the GUI listener.
- **Hardware-free stack:** `rb_servo_server` in mock mode uses an explicit
  site-local mock config. Controller-level simulation uses the rbpodo `pgmode`
  simulation (`make run MODE=sim`).
- **Real guard:** the client-side lock is retired — motion buttons are now emittable in real mode, and site-local config plus server safety layers decide whether anything moves. Operate real motion only under operator supervision with E-stop in hand.
