# Developer Environment

This page documents reproducible local setup for simulator-first development. It does not enable real robot connection, real robot motion, RealSense capture, or real Cartesian/TCP motion.

## Ubuntu Hardware-Free Dependencies

Install the base C++ and Python dependencies natively. On Ubuntu jammy, or when
`ROBOTPKG_DIST` is set explicitly, this helper installs Pinocchio from robotpkg
as `robotpkg-pinocchio` under `/opt/openrobots`. On non-jammy hosts such as
Ubuntu 24.04 noble with
`ROBOTPKG_DIST` unset, it builds pinned Pinocchio source release `v3.9.0` with
URDF support and installs it under the same prefix:

```bash
./scripts/install_deps_ubuntu.sh --profile hardware-free
```

Then run:

```bash
./scripts/check_deps.sh --profile hardware-free
```

The Pinocchio install prefix is `/opt/openrobots`. If CMake does not find it
automatically, set:

```bash
export CMAKE_PREFIX_PATH=/opt/openrobots${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}
```

## Cartesian Math Dependencies

Eigen3 and Pinocchio are mandatory for `rb_servo_server` C++ builds.
Cartesian FK/IK, orientation interpolation, frame conversion, and SE(3) delta
math delegate to Eigen/Pinocchio. There is no supported custom-math fallback
when Pinocchio is missing. Pinocchio must be 3.0.0 or newer because the
Cartesian code uses the one-argument `pinocchio::Jlog6(M)` overload added in
Pinocchio 3.0.0, matching the robotpkg jammy 3.x path used in the office.

Check availability:

```bash
./scripts/check_deps.sh --profile hardware-free
```

The supported Ubuntu helper path (`scripts/install_deps_ubuntu.sh --profile
hardware-free`) keeps robotpkg as the jammy default and uses a pinned source
build for non-jammy hosts when robotpkg is not selected. Set
`RB_PINOCCHIO_SOURCE=1` to force source or `PINOCCHIO_VERSION=<tag>` to override
the source tag. Source builds use an automatic memory-capped job limit to avoid
OOM on low-RAM WSL2 hosts; set `PINOCCHIO_BUILD_JOBS=<N>` to override it.
Conda/mamba or other source installs are also acceptable when they expose the
`pinocchio` CMake package through `CMAKE_PREFIX_PATH`.

## Python Checks

```bash
python3 -m unittest discover rb_gui/tests
python3 -m unittest discover policy_runner/tests
PYTHONPATH=rb_simulator/src python3 -m unittest discover rb_simulator/tests
python3 -m compileall -q rb_gui/rb_servo_gui policy_runner/policy_runner rb_simulator/src scripts
```

## C++ Hardware-Free Gate

```bash
./scripts/codex_gate.sh HARDEN-10
```

When dependencies are missing, the gate may skip C++ checks only if explicitly configured to do so. Do not report skipped C++ checks as passed C++ acceptance.

## Cartesian Math Acceptance

Run the Pinocchio-backed Cartesian math rebaseline gate:

```bash
./scripts/codex_gate.sh CART-MATH-03
```

This runs the Python suites and the mandatory `rb_servo_server` C++ Pinocchio
gate, including near-pi SO(3), quaternion convention, body-error convention, and
stand/local frame-conversion tests. Missing Pinocchio is a dependency failure
unless an explicit temporary skip variable is set.

## Cartesian Simulator Acceptance

After C++ dependencies are available:

```bash
CODEX_RUN_CARTESIAN_ACCEPTANCE=1 ./scripts/codex_gate.sh CART-HARDEN-05
```

This validates simulator-only PTP, Linear, and Twist primitives. It does not enable real robot motion.

## Native Simulator Stack

The operator stack (`rb_servo_server` + viser GUI + `policy_runner`) runs
natively, not in Docker:

```bash
make run MODE=sim
```

Build/install the stack first with `make build` after editing source.
Open:

```text
http://127.0.0.1:8080
```

Stop with `Ctrl+C`.

For a split-PC simulator stack, run the per-arm `rb_simulator` processes on
`172.28.60.36` (with `RB_SIMULATOR_ALLOW_NON_LOOPBACK=1`) and point the server at
`rb_servo_server/config/dual_simulator_remote_172_28_60_36.yaml` from the
control/GUI PC.

## Real Camera Dependencies

Real camera work is separate from hardware-free robot work. Install RealSense and camera dependencies only for camera acceptance. See `docs/runbooks/camera_acceptance.md`.

## Real Robot Dependencies

Real robot work requires rbpodo and site-specific network access. Do not add or run tracked runnable real configs. Use `rb_servo_server/config/dual_real.example.yaml` only as a template and create site-local configs under `rb_servo_server/config/local/`.

## Recommended Development Flow

1. Run Python/unit checks.
2. Run hardware-free C++ gate.
3. Run simulator stack manually in GUI.
4. Run Cartesian simulator acceptance.
5. Repeat until simulator behavior is stable.
6. Only then start a separate real robot read-only acceptance workflow.
