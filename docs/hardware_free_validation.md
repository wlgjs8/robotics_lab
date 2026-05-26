# Hardware-Free Validation

This document defines the default validation boundary for development without physical robots or cameras.

## Scope

Hardware-free validation may exercise:

- mock servo backend
- per-arm simulator backend
- rb_gui parser/safety/model tests
- policy_runner action-source tests
- camera mock/stub paths
- C++ unit tests that do not require hardware
- simulator-only smoke tests

It does not prove:

- real RB3-730 readiness
- real Cartesian/TCP readiness
- RealSense readiness
- gripper readiness
- force/admittance/impedance readiness
- measured calibration validity

## Base Commands

Python checks:

```bash
python3 -m unittest discover rb_gui/tests
python3 -m unittest discover policy_runner/tests
PYTHONPATH=rb_simulator/src python3 -m unittest discover rb_simulator/tests
python3 -m compileall -q rb_gui/rb_servo_gui policy_runner/policy_runner rb_simulator/src scripts
```

Shell syntax checks:

```bash
bash -n scripts/codex_gate.sh
bash -n scripts/codex_run_sequence.sh
bash -n scripts/check_deps.sh
bash -n scripts/hardware_free_validation.sh
bash -n scripts/tcp_pose_simulator_acceptance.sh
```

C++ gate:

```bash
./scripts/codex_gate.sh HARDEN-10
```

Full simulator Cartesian acceptance:

```bash
CODEX_RUN_CARTESIAN_ACCEPTANCE=1 ./scripts/codex_gate.sh CART-HARDEN-05
```

## Dependency Preflight

```bash
./scripts/check_deps.sh --profile hardware-free
```

Install missing base packages on Ubuntu:

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential \
  cmake \
  libyaml-cpp-dev \
  nlohmann-json3-dev
```

Pinocchio-enabled gates require a valid Pinocchio CMake package. Missing Pinocchio may skip optional kinematics acceptance only when the gate explicitly says it was skipped.

## Simulator Smoke

The simulator must run one process/container per arm.

Host-local ports:

```text
left  control/admin: 127.0.0.1:50200 / 50201
right control/admin: 127.0.0.1:50210 / 50211
```

Compose uses service DNS and container-internal ports:

```text
rb_simulator_left:50200
rb_simulator_right:50200
```

Wrong-arm requests must fail closed.

## Direct And Worker I/O

Direct mode and worker mode are both simulator validation targets.

Direct mode validates the straightforward servo loop/backends path.

Worker mode validates the long-term architecture where each arm worker owns blocking backend I/O. Worker mode is still simulator-only unless a future real-hardware acceptance explicitly opens it.

## Expected State Telemetry

Hardware-free state snapshots should expose enough diagnostics to debug motion and I/O behavior:

- `observed_mode`
- `observed_backend`
- `fault_context`
- arm-level `last_read`
- arm-level `last_send`
- `send_policy`
- `send_suppressed`
- dispatch/send skew
- worker drop counters, when worker mode is active
- TCP pose fields, when FK is enabled
- Cartesian solve/path telemetry, when Cartesian modes are active

## Pass Criteria

A hardware-free validation run is useful only when:

- tests are not skipped silently
- skipped checks are clearly reported with the missing dependency
- no real robot IPs are used in simulator configs
- no real robot env gates are required
- no force control is enabled
- simulator motion primitives remain simulation-only

## Not A Hardware Acceptance

A passing hardware-free gate is not permission to run a physical robot. Real robot work must start with a separate read-only acceptance plan.
