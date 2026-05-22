# rb_servo_server

C++ control server for synchronizing two Rainbow RB3-730 arms through a shared `servo_j`-style control loop.

The server is designed for:

1. fast mock-mode development without robots,
2. later Rainbow simulator / real robot backends through `IRobotBackend`,
3. Python VLA / imitation policy integration through UDP commands,
4. future Cartesian TCP and force/admittance control layers.

## Current status

Implemented in this server:

- dual-arm same-tick servo loop
- mock backend
- actual UDP JSON command receiver
- minimal YAML config parser for the provided config files
- velocity/acceleration safety clamps
- tracking-error guard with configurable policy
- latched fault state for EmergencyStop / real-mode tracking errors / robot state errors
- fail-safe command validation so missing payloads do not become zero joint targets
- Hold mode using previous sent target
- capped filter dt so one late tick does not create a large motion step
- servo period/jitter/filter-dt/safety logging
- force-control design types, config, and optional controller scaffold

Still pending:

- real `RbpodoBackend`
- Rainbow simulator connection
- Cartesian FK/IK
- production force-control integration
- production robot state/TCP pose publisher fields
- lock-free buffers and parallel left/right send for 500 Hz experiments

The real RB3-730 backend implementation plan and hardware acceptance runbook
are in `docs/rbpodo_backend_plan.md`. Treat that document as a planning gate;
it does not make `dual_real.yaml` runnable.

## Build

```bash
cmake -S . -B build
cmake --build build -j
```

## Run mock mode

```bash
./build/rb_servo_server --config config/dual_mock.yaml
```

In another terminal:

```bash
python3 tools/send_dual_joint_sine.py --rate 20 --amp-deg 2 --freq 0.2
```

Stop the server with `Ctrl+C`.

Inspect timing:

```bash
python3 tools/plot_servo_log.py logs/servo_log.csv
```

## Run hardware-free rb_simulator mode

Use the repo-local software simulator for backend integration checks. Start
with the bounded smoke runner:

```bash
python3 ../rb_simulator/tools/rbsim_servo_smoke.py --self-test
```

When local binaries are available, run the full local smoke command shown in
`docs/rb_simulator_dev.md`; that runner starts the simulator and server with
`config/dual_rb_simulator.yaml`. This path is not Rainbow Robotics rbsim/OVA,
`rbpodo`, real robot, privileged Docker, or production network validation.

## Fault behavior

The server never falls back to `[0, 0, 0, 0, 0, 0]` for invalid commands, IK-unavailable commands, stale commands, or safety failures.

Fail-safe rule:

```text
valid command   → filtered/clamped target
invalid command → previous safe sent target
stale command   → Hold
Cartesian/IK not available → previous safe sent target
EmergencyStop   → latch current/last-safe pose and ignore motion commands
real tracking error → fault latch by default
mock/rbsim tracking error → snap target to actual by default
```

Reset a latched fault:

```bash
python3 tools/send_reset_fault.py
```

## Real robot guard

Real mode refuses to start unless explicitly enabled. Do not run real robot
configs in the hardware-free simulator phase; real validation is a separate
human-gated task. `dual_real.yaml` defaults to
`tracking_error_policy: fault_latch`.

## Command channel

Default command endpoint:

```text
udp://127.0.0.1:50010
```

Minimal command:

```json
{
  "seq": 1,
  "mode": "JointTarget",
  "timeout_sec": 0.2,
  "left": {"q_target_deg": [0, -30, 80, 0, 60, 0]},
  "right": {"q_target_deg": [0, -30, 80, 0, 60, 0]}
}
```

The C++ receive timestamp is used for timeout checks.

## Force-control status

Force control is present as a design scaffold only. It is disabled by default and not connected to the joint-only control path. See `docs/force_control.md`.

## Docker + viser operator GUI

Mock-mode browser operation is defined in the Docker Compose stack, with host
ports pinned to loopback. The GUI receives UDP state snapshots and sends only
validated UDP JSON commands. Real motion is disabled, simulator motion is
limited to the repo-local hardware-free `rb_simulator` path until local evidence
passes, TCP target gizmos can emit `TcpPoseTarget` commands but the C++
Cartesian controller still holds until IK is implemented, and the GUI does not
mount the raw Docker socket. See `docs/gui_operator_console.md`.

The `sim` compose profile now wires `rb_simulator` plus `rb_servo_rbsim` as a
hardware-free, non-OVA software simulator pair. Use it as configuration
documentation unless a container smoke is explicitly in scope; the supported
phase evidence is the unit/contract checks and local smoke runner in
`docs/rb_simulator_dev.md`.
