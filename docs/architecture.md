# robotics_lab Architecture

This is the current source-of-truth architecture for `robotics_lab`. Component
README files may describe local details, but public terminology and topology
must match this document.

## Maturity Boundary

Supported today:

- mock dual-arm servo control
- per-arm local simulator backend
- mock camera server
- GUI viewer/operator console for mock/simulation

Not production-ready:

- real RB3-730 motion
- real-mode Cartesian TCP motion
- force control
- gripper control
- measured camera/robot calibration

## Canonical Public Terms

Use only these values in public config, docs, GUI labels, and operator-facing
logs:

```yaml
run_mode: mock | simulation | real
backend_type: mock | simulator | rbpodo
```

`mock` is dependency-free local behavior. `simulation` uses local simulator
processes. `real` targets physical RB3-730 controllers and requires explicit
environment gates.

## Controller Topology

The real system has one controller endpoint per arm:

```text
rb_servo_server
  left_robot  backend_type=rbpodo -> 172.28.60.200
  right_robot backend_type=rbpodo -> 172.28.60.201
```

The simulator mirrors that controller shape:

```text
rb_servo_server
  left_robot  backend_type=simulator -> rb_simulator_left
  right_robot backend_type=simulator -> rb_simulator_right
```

Each simulator instance owns one arm state machine and one control/admin
endpoint pair.

Container topology:

```text
rb_simulator_left container
  arm: left
  control: tcp://0.0.0.0:50200
  admin:   tcp://0.0.0.0:50201

rb_simulator_right container
  arm: right
  control: tcp://0.0.0.0:50200
  admin:   tcp://0.0.0.0:50201
```

Docker Compose selects compose-specific simulator configs with these
container-internal binds and sets `RB_SIMULATOR_ALLOW_NON_LOOPBACK=1`. There is
no runtime YAML port rewrite or proxy bridge in the simulator image.

Host-local topology:

```text
left simulator
  control: tcp://127.0.0.1:50200
  admin:   tcp://127.0.0.1:50201

right simulator
  control: tcp://127.0.0.1:50210
  admin:   tcp://127.0.0.1:50211
```

Simulation must not use the physical robot IP addresses as defaults.

## Motion Safety Contract

Real robot connection requires:

```bash
RB_ALLOW_REAL_ROBOT=1
```

Real joint servo motion requires:

```bash
RB_ALLOW_REAL_MOTION=1
```

Real Cartesian/TCP motion requires:

```bash
RB_ALLOW_REAL_CARTESIAN=1
```

P3 may validate Cartesian/TCP commands in simulation, but real Cartesian motion
remains disabled until a separate real-hardware acceptance procedure approves
it. GUI and policy components must fail closed rather than route Cartesian
targets into real motion.

Force control is intentionally unavailable:

```yaml
force_control:
  provider: null
  enable: false
```

No component may activate force, admittance, or impedance control as a temporary
stand-in.

## Component Responsibilities

`rb_servo_server` owns dual-arm servo command ingestion, backend selection,
safety gates, state publication, and robot mount estimates. It must preserve
one backend instance per arm.

For `backend_type=rbpodo`, read-only state acquisition and motion readiness are
separate. A controller can return valid joint feedback while servo motion is not
enabled; `rb_servo_server` should publish that state with
`servo_enabled=false` and a non-ready lifecycle, while `sendServoJ()` remains
closed unless the real-motion gate and controller readiness are both true.

`rb_simulator` owns hardware-free simulator state. The target architecture is
one simulator process/container per arm, with deterministic control and admin
interfaces.

`RbsimBackend` keeps one persistent JSON-lines TCP connection per simulator
backend instance during healthy operation, matching the one endpoint per arm
topology. It reuses that socket for connect, initialize, state reads,
`servo_j`, stop, and reset requests; transport or protocol-corruption failures
close the socket so a later request can reconnect. Robot/controller-level
simulator responses such as `RobotFault` remain structured backend results and
do not imply TCP transport corruption.

`camera_server` owns camera capture, shared-memory frame transport, metadata,
and health reporting. Mock camera operation is supported; measured RealSense
calibration is still pending.

`rb_gui` owns visualization and operator controls for mock/simulation. It must
not make real motion available unless the servo server has already accepted the
required real-mode gates.

The future `policy_runner` owns Python action sources, including SpaceMouse
input. It consumes robot state, camera metadata, and calibration packages; it
must not bypass servo safety gates. Joint-only action sources do not require
camera observations. Camera-dependent action sources must declare
`requires_camera` with a `camera_stale_timeout_sec` and fail closed when camera
readiness is absent or stale. Camera geometry-dependent sources must also
declare `requires_camera_geometry` and require measured, accepted camera
geometry.

The backend-contract migration target is defined in
[servo_backend_contract.md](servo_backend_contract.md). `IRobotBackend` should
migrate from bool/log-string behavior to structured `BackendResult` and
`SendServoJResult` diagnostics, while `ServoLoop` gradually stops owning
blocking network I/O. The future ownership target is `CommandBuffer ->
ServoCoordinator -> Left/Right ArmWorker`. This is a diagnostics and loop
architecture migration, not a real-motion enablement.

## Frame And Calibration Contract

Shared frame names and transform direction are defined in
[frame_contract.md](frame_contract.md). The active global setup registry is
`calibration/active_calibration.yaml`.

Current mount values are configured estimates, not measured calibration. Servo
config mount transforms remain the current runtime source for
`rb_servo_server`; the calibration registry is the cross-component geometry
source of truth that future work should load or cross-check against runtime
config.

`configured_estimate` geometry is allowed for visualization and simulation. It
is not valid for real geometry-dependent policy, and the active registry marks
`geometry_valid_for_real_policy: false`. Joint-only control does not require
measured calibration. TCP/Cartesian and camera-geometry policy paths require
measured and accepted calibration in a later milestone.

## Validation Contract

The default local regression gate is
[hardware_free_validation.md](hardware_free_validation.md). It validates mock,
stub, and loopback simulator behavior only. It does not prove real robot,
RealSense, external simulator, force-control, gripper, or real Cartesian
readiness.

Hardware acceptance is a separate, human-gated workflow.

Real three-camera acceptance is defined in
[runbooks/camera_acceptance.md](runbooks/camera_acceptance.md). The canonical
profile is a D435f head camera at 1280x720@30 and D405 wrist cameras at
640x360@30, with an explicitly approved 640x480 D405 variant.

## Config Naming Rebaseline

Canonical simulator configs are documented in the root
[README.md](../README.md#canonical-config-names). The recommended simulator
paths are the per-arm host, compose, and worker configs listed there; no legacy
dual-arm simulator config is recommended for new evidence.

Real robot config uses a tracked read-only template,
`rb_servo_server/config/dual_real.example.yaml`, and user-owned local configs
under `rb_servo_server/config/local/`. Use
`rb_servo_server/config/local/dual_real_readonly.yaml` for read-only state
bring-up and `rb_servo_server/config/local/dual_real_motion.yaml` only for a
separately approved motion procedure. Real connection, joint motion, and
Cartesian motion remain gated by the environment variables above.
