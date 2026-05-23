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
- per-arm local simulator backend through the `RbsimBackend` protocol client
- guarded `RbpodoBackend` integration path, disabled unless built and gated
- actual UDP JSON command receiver
- minimal YAML config parser for the provided config files
- velocity/acceleration safety clamps
- tracking-error guard with configurable policy
- latched fault state for EmergencyStop / real-mode tracking errors / robot state errors
- fail-safe command validation so missing payloads do not become zero joint targets
- Hold mode using previous sent target
- capped filter dt so one late tick does not create a large motion step
- servo period/jitter/filter-dt/safety logging
- structured backend result taxonomy for mock, simulator, and rbpodo paths
- direct and worker backend I/O models for simulator validation
- optional Pinocchio FK/IK support when built with `RB_SERVO_ENABLE_PINOCCHIO=ON`
- simulator-only Cartesian command routing when kinematics and Cartesian config
  gates are enabled
- force-control design types, config, and optional controller scaffold

Still pending:

- real-hardware acceptance for `RbpodoBackend`
- real `servo_j` motion acceptance
- real Cartesian/TCP motion acceptance
- production force-control integration
- gripper integration
- measured camera/robot calibration
- production promotion of worker I/O for real hardware

The real RB3-730 backend implementation plan and hardware acceptance runbook
are in `docs/rbpodo_backend_plan.md`. Treat that document as a planning gate;
it does not make real robot motion runnable without the documented build,
environment, and human acceptance gates.

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

Use the repo-local software simulator for backend integration checks. The
current topology is one simulator process per arm.

From the repository root, run the full validation gate:

```bash
./scripts/hardware_free_validation.sh
```

For a focused simulator smoke after the hardware-free CMake build exists:

```bash
PYTHONPATH=rb_simulator/src python3 rb_simulator/tools/rbsim_servo_smoke.py \
  --left-simulator-config rb_simulator/config/left_rb3_730e.yaml \
  --right-simulator-config rb_simulator/config/right_rb3_730e.yaml \
  --server rb_servo_server/build/hardware_free_gate/rb_servo_server \
  --server-config rb_servo_server/config/dual_simulator.yaml \
  --artifact-dir rb_simulator/artifacts/rbsim_servo_smoke
```

The simulator path is not Rainbow Robotics rbsim/OVA, real robot, privileged
Docker, or production network validation.

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
human-gated task. Start from the tracked
`config/dual_real.example.yaml` template, then create a site-owned
`config/local/dual_real_readonly.yaml` for read-only bring-up or
`config/local/dual_real_motion.yaml` for a separately approved motion
procedure. The real template defaults to `tracking_error_policy: fault_latch`
and `servo.send_servo_commands: false`.

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

The root Docker Compose stack defines the simulator operator path:

```bash
cd /home/plaif/workspace/robotics_lab
make sim-up
```

It starts `rb_gui`, `rb_simulator_left`, `rb_simulator_right`, and
`rb_servo_server` with `config/dual_simulator_compose.yaml`. Host GUI ports are
pinned to loopback. The GUI receives UDP state snapshots and sends only
validated UDP JSON commands. Real motion is disabled, and the GUI does not
mount the raw Docker socket. See `docs/gui_operator_console.md`.
