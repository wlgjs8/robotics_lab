# robotics_lab

`robotics_lab` is the integration workspace for a dual-arm RB3-730 system with
servo control, a topology-isomorphic local simulator, camera capture, and an
operator GUI.

## Current Maturity

Currently supported:

- mock dual-arm servo control
- per-arm local simulator backend
- mock camera server
- GUI viewer/operator console for mock/simulation

Not production-ready yet:

- real RB3-730 motion
- Cartesian TCP motion in real mode
- force control
- gripper control
- measured camera/robot calibration

Real robot motion remains explicitly gated. Do not treat a passing mock or
simulation run as permission to move hardware.

## Source Of Truth

Start here, then follow component docs only for implementation details:

- [docs/architecture.md](docs/architecture.md): system topology, terms, safety
  gates, and roadmap.
- [docs/hardware_free_validation.md](docs/hardware_free_validation.md):
  hardware-free build and test gate.
- [docs/developer_environment.md](docs/developer_environment.md):
  dependency installation profiles and the MIG-26 rebaseline command set.
- [docs/frame_contract.md](docs/frame_contract.md): shared robot/camera frame
  names and transform direction.
- [docs/servo_backend_contract.md](docs/servo_backend_contract.md): MIG backend
  result contract and non-blocking servo-loop migration target.
- [docs/runbooks/tcp_pose_simulator_acceptance.md](docs/runbooks/tcp_pose_simulator_acceptance.md):
  simulator-only Cartesian PTP, Linear, and Twist acceptance.
- [docs/runbooks/camera_acceptance.md](docs/runbooks/camera_acceptance.md):
  real three-camera acceptance and policy-runner camera readiness criteria.
- [calibration/active_calibration.yaml](calibration/active_calibration.yaml):
  configured-estimate robot/camera/stand setup registry.
- [TODO.md](TODO.md): P0-P3 work packages and acceptance criteria.

Historical review and planning notes may remain in the tree for audit context.
When they disagree with this README or [docs/architecture.md](docs/architecture.md),
the root source-of-truth docs win.

## Canonical Terms

Public config, docs, logs intended for operators, and GUI labels use these
terms:

```yaml
run_mode: mock | simulation | real
backend_type: mock | simulator | rbpodo
```

`run_mode` describes the operating environment. `backend_type` describes the
servo backend implementation selected for each arm.

## Real And Simulator Topology

The physical system has one controller endpoint per arm:

```text
rb_servo_server
  left_robot  backend_type=rbpodo -> 172.28.60.200
  right_robot backend_type=rbpodo -> 172.28.60.201
```

The simulator mirrors that shape with one independent simulator endpoint per
arm:

```text
rb_servo_server
  left_robot  backend_type=simulator -> rb_simulator_left
  right_robot backend_type=simulator -> rb_simulator_right

rb_simulator_left
  arm: left
  control: tcp://0.0.0.0:50200 inside its container
  admin:   tcp://0.0.0.0:50201 inside its container

rb_simulator_right
  arm: right
  control: tcp://0.0.0.0:50200 inside its container
  admin:   tcp://0.0.0.0:50201 inside its container
```

Separate simulator containers may reuse the same internal ports because each
container has its own network namespace. Compose uses explicit
`rb_simulator/config/*_compose.yaml` profiles and sets
`RB_SIMULATOR_ALLOW_NON_LOOPBACK=1`; host-run profiles remain loopback-only by
default. Direct host execution must use separate loopback ports:

```text
left simulator:
  control: tcp://127.0.0.1:50200
  admin:   tcp://127.0.0.1:50201

right simulator:
  control: tcp://127.0.0.1:50210
  admin:   tcp://127.0.0.1:50211
```

Simulator endpoints must not default to the real robot IP addresses. The
isomorphism is one controller endpoint per arm, not reuse of physical network
addresses.

## Safety Gates

Real robot connection is closed unless:

```bash
RB_ALLOW_REAL_ROBOT=1
```

Real joint servo motion is closed unless:

```bash
RB_ALLOW_REAL_MOTION=1
```

Real Cartesian/TCP motion is closed unless:

```bash
RB_ALLOW_REAL_CARTESIAN=1
```

TCP Cartesian command support is simulation-only until P3 validation lands.
Even after P3, real Cartesian motion stays closed until a separate real-hardware
acceptance procedure approves it.

Force/admittance/impedance control is not implemented. Force control must remain
null:

```yaml
force_control:
  provider: null
  enable: false
```

## Component Map

```text
rb_servo_server/   dual-arm servo control, backend selection, safety gates
rb_simulator/      hardware-free per-arm simulator backend
camera_server/     RealSense/mock camera capture and metadata publishing
rb_gui/            operator viewer and console for mock/simulation workflows
docs/              shared architecture, validation, and frame contracts
scripts/           repository-level validation helpers
```

## Canonical Config Names

Simulator operator paths use these current configs:

- `rb_servo_server/config/dual_simulator.yaml`: host-loopback direct I/O.
- `rb_servo_server/config/dual_simulator_compose.yaml`: Docker Compose service
  DNS.
- `rb_servo_server/config/dual_simulator_worker.yaml`: host-loopback worker I/O
  evidence.
- `rb_servo_server/config/dual_simulator_tcp_acceptance.yaml`: simulator-only
  TCP Pose/Delta acceptance with Pinocchio/FK/IK enabled and
  `cartesian_control.allow_in_real: false`.
- `rb_simulator/config/left_rb3_730e.yaml`: left simulator process.
- `rb_simulator/config/right_rb3_730e.yaml`: right simulator process.
- `rb_simulator/config/left_rb3_730e_compose.yaml`: left simulator container
  bind profile.
- `rb_simulator/config/right_rb3_730e_compose.yaml`: right simulator container
  bind profile.

Deprecated simulator compatibility names, including `dual_rbsim.yaml`,
`dual_rb_simulator.yaml`, `dual_rb_simulator_compose.yaml`, `rbsim_local`, and
public `rbsim`, are retained only for migration compatibility or internal
package/protocol names. New operator docs and configs should use
`run_mode: simulation`, `backend_type: simulator`, and the canonical files
above; compatibility names should be removed after downstream configs stop
referencing them.

Real robot config guidance uses this split:

- tracked template: `rb_servo_server/config/dual_real.example.yaml`
- user local read-only config:
  `rb_servo_server/config/local/dual_real_readonly.yaml`
- user local motion config:
  `rb_servo_server/config/local/dual_real_motion.yaml`

No tracked runnable real robot config is canonical. The tracked real template
is read-only by default and documents the real controller IPs with the required
gates. Local real configs are site-owned and must stay under
`rb_servo_server/config/local/`. Read-only rbpodo state publishing can be
healthy while `servo_enabled=false` if joint feedback is valid; that is not
motion readiness. Real `servo_j` sends remain blocked by
`servo.send_servo_commands=false` in the tracked template and by
`RB_ALLOW_REAL_MOTION=1` in any motion config. Rbpodo stop/reset fault recovery
still requires operator intervention until verified controller APIs are wired.

## Make Quick Starts

Run all examples from the repository root:

```bash
cd /home/plaif/workspace/robotics_lab
```

The root `Makefile` wraps the current Docker Compose and validation entry
points. It does not enable real robot motion.

### Build all Compose images

```bash
make build
```

Equivalent command:

```bash
docker compose -p robotics_lab -f docker-compose.yml build
```

### Start the simulator operator stack

This starts the browser GUI, one left simulator container, one right simulator
container, and `rb_servo_server` with the per-arm simulator compose config:

```bash
make sim-up
```

Open the GUI at:

```text
http://127.0.0.1:8080
```

The default simulator stack builds `rb_servo_server` with Pinocchio enabled.
`dual_simulator_compose.yaml` publishes FK TCP poses and enables simulator-only
Cartesian IK, so the GUI TCP target gizmos can send bounded `TcpPoseTarget`
commands after `ArmMotion` is selected. Real Cartesian motion remains disabled
by config.

`make sim-up` runs in the foreground. Stop it with `Ctrl+C`, then clean up
containers with:

```bash
make sim-down
```

### Run the hardware-free validation gate

This builds and tests mock/stub paths, simulator unit tests, and per-arm
loopback simulator smoke checks when local prerequisites are available:

```bash
make sim-smoke
```

Equivalent command:

```bash
./scripts/hardware_free_validation.sh
```

For a final MIG-13+ developer rebaseline, including docs/shell checks, Python
unit tests, mock/stub CMake gates, hardware-free validation, and optional
Pinocchio/TCP acceptance when Pinocchio is installed, run:

```bash
bash scripts/codex_gate.sh MIG-26
```

The TCP acceptance branch also requires local AF_INET loopback sockets. In
sandboxed environments that deny loopback sockets, `MIG-26` reports that branch
as skipped while keeping build, unit, and non-socket validation active.

Install or inspect local dependencies with:

```bash
bash scripts/install_deps_ubuntu.sh --profile hardware-free
bash scripts/check_deps.sh --profile hardware-free
```

To require the direct and worker simulator smokes instead of allowing an
environment skip:

```bash
RBSIM_SMOKE_MODE=required RBSIM_WORKER_SMOKE_MODE=required make sim-smoke
```

### Start the mock camera server

This starts only the mock camera service through the `mock_camera` compose
profile:

```bash
make camera-mock-up
```

Stop it with:

```bash
make stop
```

### Start the real camera server

This starts the RealSense camera container with host IPC/network and USB device
access. It is camera hardware only; it does not enable robot motion:

```bash
make camera-real-up
```

Stop it with:

```bash
make stop
```

### Deploy or stop the default compose project

Use these when you want the default compose lifecycle directly:

```bash
make deploy
make stop
```

The Makefile variables are overrideable:

```bash
PROJECT=robotics_lab_dev COMPOSE_FILE=docker-compose.yml make sim-up
```

## Roadmap

- P0 aligns architecture docs, simulator topology, public terminology, and the
  hardware-free validation gate.
- P1 stabilizes joint-only servo behavior across mock, simulator, and real
  topology while preserving real-motion gates.
- P2 adds truthful FK/TCP pose publication and measured calibration workflows
  for visualization and policy inputs.
- P3 enables simulator-only Cartesian/TCP command validation. Real Cartesian
  motion remains separately gated after P3.
- MIG migrates `IRobotBackend` bool/log-string operations toward structured
  `BackendResult` and `SendServoJResult` diagnostics, then moves blocking
  network I/O out of `ServoLoop` toward `CommandBuffer -> ServoCoordinator ->
  Left/Right ArmWorker`. This migration is not a real-motion enablement and
  does not weaken `RB_ALLOW_REAL_ROBOT`, `RB_ALLOW_REAL_MOTION`, or
  `RB_ALLOW_REAL_CARTESIAN`.

## MIG-12 Migration Baseline

The current review baseline is:

- `CommandBuffer -> ServoCoordinator -> Left/Right ArmWorker` is the intended
  backend I/O ownership architecture. Direct loop operation remains the stable
  default until worker mode is promoted beyond simulator evidence.
- Backend results use structured taxonomy. Important operator-visible classes
  include `RobotFault` for controller/robot fault state,
  `TransportWriteFailed` for command-channel write failures,
  `SuppressedByPolicy` for environment gate or read-only suppression, and
  `WrongMode` for controller mode/config mismatch.
- Real read-only rbpodo connection requires `RB_ALLOW_REAL_ROBOT=1`. Real
  `servo_j` transmission additionally requires `RB_ALLOW_REAL_MOTION=1`.
  Real Cartesian/TCP motion additionally requires
  `RB_ALLOW_REAL_CARTESIAN=1`.
- Rbpodo state acquisition is separate from motion readiness. A real read-only
  run can publish valid `q_actual` with `servo_enabled=false`; that is a
  successful state read, while later `servo_j` attempts remain rejected until
  the real-motion gate and controller readiness are both satisfied.
- `stop()` and `resetFault()` for rbpodo controller recovery remain
  unverified. On a real robot fault, treat them as a fail-closed result that
  requires operator intervention, not as automatic recovery.
- `network.state_pub_rate_hz` controls UDP state publication rate. The default
  tracked configs use 20 Hz.
- `servo.io_model: direct` is the stable default. `servo.io_model: worker` is
  simulator-accepted when the MIG-10/MIG-11 worker smoke passes. Real +
  `worker` remains disabled or experimental until a separate real read-only
  acceptance task exists.

## MIG-26 Rebaseline

Current backend-contract architecture:

```text
CommandBuffer -> ServoCoordinator/DualArmServoLoop -> Left ArmWorker  -> simulator/rbpodo endpoint
                                                \-> Right ArmWorker -> simulator/rbpodo endpoint
```

`ServoCoordinator/DualArmServoLoop` owns timing policy, command freshness,
fault latching, safety checks, and result aggregation. In direct I/O mode it
still calls the per-arm backends directly. In worker I/O mode each `ArmWorker`
owns blocking connect/read/reset/`servo_j` work for one arm and publishes
cached structured results.

MIG-13+ status:

- `RbsimBackend` uses one persistent JSON-lines TCP transport per simulator
  backend instance during healthy operation. Transport/protocol failures close
  the socket for later reconnect; structured robot/controller rejections such
  as `RobotFault` do not by themselves corrupt the transport.
- `ArmWorker` streaming `servo_j` queue policy is `latest_wins`. Per-arm state
  publishes command drop/overwrite counters and last enqueued/dispatched/
  completed sequence values under the `worker` object.
- `FaultContext` is latched as structured state. Later suppression results
  remain live telemetry but do not overwrite the first latched fault context.
- Rbpodo read-only state can succeed while motion readiness is false.
  `stop()` and `resetFault()` remain fail-closed for real controller recovery
  and require operator intervention.
- Command source lease enforcement defaults to off for compatibility. State
  still publishes source/lease metadata, and simulator acceptance profiles may
  enable enforcement explicitly.
- TCP Pose/Delta acceptance is simulator-only and Pinocchio-gated. It does not
  enable real Cartesian motion.
- Camera acceptance is a separate real-camera hardware workflow. Joint-only
  `policy_runner` actions remain allowed without camera readiness; camera and
  camera-geometry sources fail closed when declared readiness is absent.
