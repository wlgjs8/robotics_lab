# Developer Environment

This page documents reproducible local setup for hardware-free (mock) development. It does not enable real robot connection, real robot motion, RealSense capture, or real Cartesian/TCP motion.

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
python3 -m compileall -q rb_gui/rb_servo_gui policy_runner/policy_runner scripts
```

## C++ Hardware-Free Checks

```bash
cmake -S rb_servo_server -B rb_servo_server/build
cmake --build rb_servo_server/build -j
ctest --test-dir rb_servo_server/build --output-on-failure
```

When dependencies are missing, report the missing package clearly. Do not report skipped C++ checks as passed C++ acceptance.

## Cartesian Math Acceptance

Run the Pinocchio-backed C++ test suite:

```bash
cmake -S rb_servo_server -B rb_servo_server/build
cmake --build rb_servo_server/build -j
ctest --test-dir rb_servo_server/build --output-on-failure
```

This includes the mandatory `rb_servo_server` C++ Pinocchio tests for near-pi
SO(3), quaternion convention, body-error convention, and stand/local
frame-conversion behavior. Missing Pinocchio is a dependency failure.

## Cartesian Validation

Cartesian PTP, Linear, and Twist behavior is covered by the Pinocchio-backed C++
test suite, then exercised on the active stack: mock smoke when a local mock
config is available, rbpodo controller `pgmode` simulation / VM for
controller-sim evidence, and physical real only through the separate supervised
runbooks. The prior software-simulator-oriented Cartesian acceptance runner was
removed with the retired simulator-first lane.

## Native Stack

The operator stack (`rb_servo_server` + viser GUI + `policy_runner`) runs
natively, not in Docker:

```bash
make run MODE=sim
```

`MODE=sim` is the rbpodo controller `pgmode` simulation path. Build/install the
stack first with `make build` after editing source. Open:

```text
http://127.0.0.1:8080
```

Stop with `Ctrl+C`.

For hardware-free controller-simulation without physical boxes, boot the two
Rainbow virtual control-box VMs with `make vm-up` and then run
`make run MODE=sim` (`make vm-down` / `make vm-status`).

## Real Camera Dependencies

Real camera work is separate from hardware-free robot work. Install RealSense and camera dependencies only for camera acceptance. See `docs/runbooks/camera_acceptance.md`.

## Real Robot Dependencies

Real robot work requires rbpodo and site-specific network access. The tracked launch configs are `rb_servo_server/config/stack_real.yaml` and `rb_servo_server/config/stack_sim.yaml`; do not create parallel local launch configs, and advance acceptance by changing one reviewed setting at a time in the matching tracked stack.

## Recommended Development Flow

1. Run Python/unit checks.
2. Run hardware-free C++ checks.
3. Run the mock / controller-sim stack manually in the GUI.
4. Exercise Cartesian behavior on the active mock or controller-sim stack.
5. Repeat until behavior is stable.
6. Only then start a separate real robot read-only acceptance workflow.
