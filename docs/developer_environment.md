# Developer Environment

This page documents reproducible local setup for hardware-free development.
It does not enable real robot connection, real robot motion, RealSense capture,
or real Cartesian/TCP motion.

## Quick Start

From the repository root on Ubuntu:

```bash
bash scripts/install_deps_ubuntu.sh --profile hardware-free
bash scripts/codex_gate.sh MIG-26
```

`MIG-26` is the final rebaseline gate for MIG-13 and later. It runs shell and
documentation checks, Python simulator tests, GUI tests, `policy_runner` tests,
mock/stub CMake/CTest gates, and the full hardware-free validation helper. If
Pinocchio is installed, it also runs the Pinocchio-enabled servo gate and the
simulator-only TCP pose acceptance runner when local AF_INET loopback sockets
are available.

If dependencies are managed outside apt, run only the preflight:

```bash
bash scripts/check_deps.sh --profile hardware-free
```

The preflight reports missing packages without installing anything.

## Dependency Profiles

Hardware-free development:

- CMake
- C++17 compiler and build tools
- Python 3.10 or newer
- `yaml-cpp`
- `nlohmann_json`

Kinematics and simulator-only TCP acceptance:

- hardware-free dependencies
- Eigen3
- Pinocchio CMake package

Optional real-camera acceptance:

- hardware-free dependencies
- librealsense2 development package and tools
- RealSense udev/device access
- ZeroMQ development package

Optional real-robot build readiness:

- hardware-free dependencies
- rbpodo SDK/package exposed through `CMAKE_PREFIX_PATH` or `RBPODO_ROOT`

The optional profiles are dependency checks only. They do not open
`RB_ALLOW_REAL_ROBOT`, `RB_ALLOW_REAL_MOTION`, or
`RB_ALLOW_REAL_CARTESIAN`.

## Python Packages

The hardware-free Python tests use the standard library for `rb_simulator` and
`policy_runner`. The GUI package declares its operator dependencies in
`rb_gui/pyproject.toml`:

```bash
python3 -m pip install -e rb_gui
```

`policy_runner` has no required third-party package for the default tests.
SpaceMouse support is optional:

```bash
python3 -m pip install -e policy_runner[spacemouse]
```

Use a virtual environment when installing Python packages into a developer
workstation:

```bash
python3 -m venv .venv
. .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -e rb_gui -e policy_runner
```

## Validation Command Set

The one-command hardware-free rebaseline is:

```bash
bash scripts/codex_gate.sh MIG-26
```

Useful component checks are:

```bash
PYTHONPATH=rb_simulator/src python3 -m unittest discover rb_simulator/tests
python3 -m unittest discover rb_gui/tests
python3 -m unittest discover policy_runner/tests
bash scripts/hardware_free_validation.sh
```

Dependency and optional acceptance checks:

```bash
bash scripts/check_deps.sh --profile hardware-free
bash scripts/check_deps.sh --profile kinematics
bash scripts/tcp_pose_simulator_acceptance.sh
```

Run the TCP pose acceptance only when Pinocchio is installed and the
Pinocchio-enabled `rb_servo_server` build is available. Without Pinocchio, use
`--allow-missing-pinocchio` only for syntax/config validation; it skips runtime
FK/IK simulator acceptance.

## Safety Boundary

Default runnable paths are hardware-free. Real hardware acceptance remains a
separate human-gated workflow:

- real robot connection requires `RB_ALLOW_REAL_ROBOT=1`
- real `servo_j` motion requires `RB_ALLOW_REAL_MOTION=1`
- real Cartesian/TCP motion requires `RB_ALLOW_REAL_CARTESIAN=1`
- force/admittance/impedance control remains disabled
- current calibration values are configured estimates, not measured acceptance
  calibration
