# Viser GUI + Docker operator console

This milestone adds a browser operator console without changing ownership of the high-rate servo loop. `rb_servo_server` continues to own robot backend reads and servo sends; the GUI only consumes `StatePublisher` UDP snapshots and emits validated UDP JSON commands through the existing command protocol.

## Services and ports

Default compose stack:

- `rb_servo_gui`: Python viser web GUI, HTTP `8080`, UDP state listener `50110`.
- `rb_servo_server`: C++ server, UDP command listener `50010`, mock config for first runtime.
- `rb_simulator`: profile-gated repo-local hardware-free simulator under `--profile sim`.
- `rb_servo_rbsim`: profile-gated C++ server using `config/dual_rb_simulator.yaml` and the simulator loopback endpoint.

The GUI container does **not** mount `/var/run/docker.sock`. Container start/stop controls are status/manual only until a constrained ops helper is implemented.

## Mock GUI stack

The compose file defines a mock GUI stack for manual operator use. The server
uses `config/dual_mock_compose.yaml`, which binds commands inside the container
on `udp://0.0.0.0:50010`; compose publishes that UDP port only on host loopback.
State publishes to `udp://rb_servo_gui:50110` through Docker Compose DNS. Static
container IPs are not required.

The compose server image targets the build stage so containerized regression
checks can use the same build environment when container validation is in scope.

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
- Simulation motion is limited to the repo-local `rb_simulator` path until connect, valid state read, truthful `servo_j` send, stop/reset, hold, and low-amplitude jog tests pass with software-only artifacts. Rainbow Robotics rbsim/OVA and real robot validation remain out of scope.
- The stand frame axes are hidden; the visible 6D controls are left/right TCP target gizmos. They initialize from `tcp_stand` when the state stream provides it, otherwise from URDF FK, and fall back to the old joint marker estimate only when TCP/FK data is unavailable.
- TCP target buttons emit validated `TcpPoseTarget` UDP commands from the gizmo pose in mock/simulation-safe modes. The current C++ Cartesian controller still reports `CartesianUnavailable` and holds position until IK is implemented.
- Joint jog requires a fresh valid state snapshot, clamps per-command step size, sets a bounded timeout, and never synthesizes `[0,0,0,0,0,0]` when state is missing or invalid.
- Desired mode and observed server mode are separate. Selecting a desired mode without an ops surface does not claim that the running C++ process changed config.
- GUI state comes from UDP snapshots only; it does not import or read robot backends.

## Troubleshooting

- **Disconnected/stale GUI:** confirm `rb_servo_server` is running and `state_pub_bind` targets the GUI listener (`rb_servo_gui:50110` in compose, `127.0.0.1:50110` for host smoke).
- **Simulator profile:** `rb_simulator` and `rb_servo_rbsim` are hardware-free compose wiring only. Use `docs/rb_simulator_dev.md` for the supported unit and local-smoke evidence.
- **Real guard:** motion buttons remain blocked by design. Do not use the GUI for real robot motion in this milestone.
- **Container controls disabled:** expected because the GUI has no Docker daemon authority. Start/stop remains an external manual operator action.
