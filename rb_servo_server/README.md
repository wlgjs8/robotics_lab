# rb_servo_server

C++ control server for synchronizing two Rainbow RB3-730 arms through a shared `servo_j`-style control loop.

The server is designed for:

1. fast mock-mode development without robots,
2. Rainbow rbpodo real/controller-simulation backends through `IRobotBackend`,
3. Python VLA / imitation policy integration through UDP commands,
4. Cartesian TCP control and supervised experimental force/compliance control.

## Current status

Implemented in this server:

- dual-arm same-tick servo loop
- mock backend
- guarded `RbpodoBackend` integration path, enabled only by explicit stack config
- actual UDP JSON command receiver
- minimal YAML config parser for the provided config files
- velocity/acceleration safety clamps
- tracking-error guard with configurable policy
- latched fault state for EmergencyStop / real-mode tracking errors / robot state errors
- fail-safe command validation so missing payloads do not become zero joint targets
- Hold mode preserving the last accepted/sent joint reference as a recoverable
  pause; measured-pose re-anchoring is reserved for explicit lifecycle
  transitions such as Init Motion start/no-op, freedrive exit, and fault reset
- capped filter dt so one late tick does not create a large motion step
- servo period/jitter/filter-dt/safety logging
- structured backend result taxonomy for mock and rbpodo paths
- direct and worker backend I/O models for hardware-free/mock validation
- mandatory Pinocchio/Eigen FK, IK, and Cartesian math support
- Cartesian command routing when kinematics and Cartesian config gates are
  enabled
- F/T monitor, contact guard, unilateral normal admittance, and bounded 6D
  Cartesian compliance; activation remains explicit per stack/arm
- gripper command forwarding to the out-of-process `gripper_server`

Still pending:

- physical F/T characterization and force-control acceptance
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

The repository-level `make build` keeps this stack incremental. Layout-sensitive
`config.hpp` changes explicitly rebuild every shipped server object while
preserving the CMake build tree. Use `make rebuild` only for an intentional hard
reset of `build/rbpodo_real_gate`, such as cache or toolchain recovery.

The build requires Eigen3 and Pinocchio. Cartesian FK, IK, orientation
interpolation, frame conversion, and SE(3) delta math delegate to
Eigen/Pinocchio rather than local fallback math. Install Pinocchio through an
Ubuntu robotpkg package under `/opt/openrobots` using
`../scripts/install_deps_ubuntu.sh --profile hardware-free`, conda/mamba, or a
source install exposed through `CMAKE_PREFIX_PATH`.

## Run controller-simulation mode

```bash
cd /home/plaif/workspace/robotics_lab
make run MODE=sim
```

For a hardware-free mock smoke, use a temporary YAML outside the repository and
pass it explicitly:

```bash
./build/rb_servo_server --config /tmp/<mock-config>.yaml
```

Stop the server with `Ctrl+C`.

Inspect timing:

```bash
python3 tools/plot_servo_log.py logs/servo_log.csv
python3 tools/analyze_servo_log.py logs/servo_log.csv
```

The servo CSV also carries the latest chunk-frame receive age/interarrival,
policy inference timing, camera bundle/frame age and focus indicators, plus
DeltaTwist per-arm execution budgets, acceleration commands, and a stable
14-bit clamp mask. For `delta_preview`, it additionally records the Ruckig
projection error and the commanded-pose lead over measured TCP, including each
persistent-error count. `analyze_servo_log.py` summarizes these optional columns and
continues to accept older logs that do not contain them. These fields are CSV
telemetry only; they are not added to state JSON and do not affect control.

## Flow chunk preview controller

`cartesian_control`'s active `ruckig_follower.controller: delta_preview` profile is the
timestamp-aligned flow-infer path. It accepts only chunk overlay schema v3 with
valid camera-frame velocity proprio metadata, drops policy rows already elapsed
between observation and activation on the publisher, integrates the remaining
ee-local deltas with the canonical Eigen/Pinocchio SE(3) path, and previews the
result through the existing Ruckig position/velocity/acceleration chain. Both
arms consume the same aligned policy row; per-arm motion is preserved, including
the near-zero inactive-arm intervals present in sequential PIKA UMI episodes.

The projection-error and actual-lead thresholds and their consecutive-error
budgets are mandatory positive config values. `fallback_policy: fault` is also
mandatory for this controller. Missing v3 metadata, invalid velocity proprio,
or persistent infeasibility therefore holds and ultimately faults; the server
does not substitute guessed bounds. The legacy `delta_twist` controller remains
parseable for regression profiles but is no longer selected by the tracked real
flow profile.

## Hardware-free validation

Hardware-free validation runs C++/Python tests and, when a temporary mock config is
available, mock-mode smoke against that explicit YAML. Cartesian
behavior is covered by Pinocchio-backed C++ tests and active-stack smoke. For
controller-level simulation, use the rbpodo controller `pgmode` simulation
(`make run MODE=sim`) or the Rainbow virtual control-box VMs. The old
software-simulator-oriented Cartesian acceptance runner is no longer part of
this validation surface.

This lane is not Rainbow Robotics external simulator/OVA, real robot, privileged
Docker, or production network validation.

## Fault behavior

The server never falls back to `[0, 0, 0, 0, 0, 0]` for invalid commands, IK-unavailable commands, stale commands, or safety failures.

Fail-safe rule:

```text
valid command   → filtered/clamped target
invalid command → previous safe sent target
stale command   → Hold at the last accepted/sent reference
Cartesian/IK not available → previous safe sent target
EmergencyStop   → latch current/last-safe pose and ignore motion commands
real tracking error → fault latch by default
mock tracking error → snap target to actual by default
```

Reset a latched fault:

```bash
python3 tools/send_reset_fault.py
```

## Real robot config boundary

Real motion is config-driven and operator-supervised. Do not run real robot
configs during hardware-free validation; real validation is a separate
human-gated task. Use the tracked `config/stack_real.yaml` directly and change
one reviewed acceptance-stage setting at a time. It must keep unaccepted motion
paths off until the relevant acceptance task explicitly enables
`servo.send_servo_commands: true` and, for Cartesian motion,
`cartesian_control.allow_in_real: true`.

Rbpodo joint states and commands preserve raw controller degrees. The tracked
stack configs use explicit raw-degree ranges; see
`../docs/joint_range_policy.md`.

## Command channel

Current stack command endpoint:

```text
udp://127.0.0.1:50256
```

The stack state fanout uses `50356` (joint scope dashboard), `50366` (viser
GUI), `50376` (stack policy_runner/teleop_mux), `50378` (external flow-infer
readback), and `50386` (camera_server stereo_worker wrist-fusion). Gripper
command/feedback uses `50410`/`50420`.

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

Force control is connected to the Cartesian servo path. The tracked real stack
is currently at supervised Gate 3D: six-axis `cartesian_admittance`,
`surface_source: none`, and `compliance_frame: tcp_origin`. The controller axes
therefore follow the accepted rbpodo EFT/TCP orientation and corrections use the
TCP endpoint. Translation and rotation each use block-coherent release
recentering so sibling axes do not spring home independently and a common
feasible jerk scale preserves the released 3D direction. Both geometric floor
constraints are off by explicit operator decision, so this profile has no
TCP/gripper-tip floor backstop. The simulation stack remains force-off. See
`docs/force_control.md` and the acceptance runbook.

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

For hardware-free mock runs, start `rb_servo_server` directly with an explicit
temporary mock config outside the repository. Docker remains in use only for
`camera_server` (managed by `make cam-up` / `cam-down` / `cam-status`).
