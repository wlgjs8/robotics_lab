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
- [docs/frame_contract.md](docs/frame_contract.md): shared robot/camera frame
  names and transform direction.
- [docs/servo_backend_contract.md](docs/servo_backend_contract.md): MIG backend
  result contract and non-blocking servo-loop migration target.
- [docs/runbooks/tcp_pose_simulator_acceptance.md](docs/runbooks/tcp_pose_simulator_acceptance.md):
  simulator-only TCP Pose/Delta acceptance.
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
container has its own network namespace. Direct host execution must use
separate loopback ports:

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
