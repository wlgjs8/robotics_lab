# rb_servo_server

C++ control server for synchronizing two Rainbow RB3-730 arms through a shared `servo_j`-style control loop.

The server is designed for:

1. fast mock-mode development without robots,
2. later Rainbow real robot backends through `IRobotBackend`,
3. Python VLA / imitation policy integration through UDP commands,
4. future Cartesian TCP and force/admittance control layers.

## Current status

Implemented in this server:

- dual-arm same-tick servo loop
- mock backend
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
- structured backend result taxonomy for mock and rbpodo paths
- direct and worker backend I/O models for hardware-free/mock validation
- mandatory Pinocchio/Eigen FK, IK, and Cartesian math support
- Cartesian command routing when kinematics and Cartesian config gates are
  enabled
- force-control design types, config, and optional controller scaffold

Still pending:

- real-hardware acceptance for `RbpodoBackend`
- real `servo_j` motion acceptance
- real Cartesian/TCP motion acceptance
- production force-control integration
- gripper integration
- measured camera/robot calibration
- production promotion of worker I/O for real hardware

The active real-mode safety source of truth is the root `README.md`,
`AGENTS.md`, and `docs/servo_backend_contract.md`. Historical rbpodo planning
notes are archived under `docs/archive/planning/`; they are not runnable
operator instructions.

## Build

```bash
cmake -S . -B build
cmake --build build -j
```

The build requires Eigen3 and Pinocchio. Cartesian FK, IK, orientation
interpolation, frame conversion, and SE(3) delta math delegate to
Eigen/Pinocchio rather than local fallback math. Install Pinocchio through an
Ubuntu robotpkg package under `/opt/openrobots` using
`../scripts/install_deps_ubuntu.sh --profile hardware-free`, conda/mamba, or a
source install exposed through `CMAKE_PREFIX_PATH`.

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

## Hardware-free validation

Hardware-free validation runs in mock mode (`config/dual_mock.yaml`) and checks
Cartesian behavior against an already-running rbpodo or mock server with
`scripts/cartesian_acceptance.py --mode assume-running`. For controller-level
simulation, use the rbpodo controller `pgmode` simulation (`make run MODE=sim`)
or the Rainbow virtual control-box VMs.

This lane is not Rainbow Robotics external simulator/OVA, real robot, privileged
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
mock tracking error → snap target to actual by default
```

Reset a latched fault:

```bash
python3 tools/send_reset_fault.py
```

## Real robot guard

Real mode refuses to start unless explicitly enabled. Do not run real robot
configs during hardware-free validation; real validation is a separate
human-gated task. Start from the tracked
`config/dual_real.example.yaml` template, then create a site-owned
`config/local/dual_real_readonly.yaml` for read-only bring-up or
`config/local/dual_real_motion.yaml` for a separately approved motion
procedure. The real template defaults to `tracking_error_policy: fault_latch`
and `servo.send_servo_commands: false`.

Rbpodo joint states and commands preserve raw controller degrees. The tracked
real templates use explicit `q_min_deg: [-360, -360, -360, -360, -360, -360]`
and `q_max_deg: [360, 360, 360, 360, 360, 360]`; see
`../docs/joint_range_policy.md`.

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

## Viser operator GUI

The native operator stack runs `rb_servo_server` together with the viser GUI and
`policy_runner` (no Docker):

```bash
cd /home/plaif/workspace/robotics_lab
make run MODE=sim
```

`make run` launches the servo server, the viser GUI, and `policy_runner`
side by side; `MODE=sim` uses the rbpodo controller-simulation path, and the
plain `make run` targets the real controllers. Build/install the stack first
with `make build` after editing source. The GUI receives UDP state
snapshots and sends only validated UDP JSON commands; the server is built with
Pinocchio enabled so FK/IK powers the GUI TCP target tests. See
`docs/gui_operator_console.md`.

For hardware-free runs, start `rb_servo_server` directly with
`config/dual_mock.yaml` (MockBackend). Docker remains in use only for
`camera_server` / `camera_server_mock`.
