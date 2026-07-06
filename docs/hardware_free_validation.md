# Hardware-Free Validation

This document defines the default validation boundary for development without physical robots or cameras.

## Scope

Hardware-free validation may exercise:

- mock servo backend
- rb_gui parser/safety/model tests
- policy_runner action-source tests
- camera mock/stub paths
- C++ unit tests that do not require hardware
- mock-mode smoke tests

> The `rb_simulator` per-arm software-simulator backend and its hardware-free
> simulator lane were retired. Hardware-free validation now uses the mock
> backend; controller behavior beyond mock is validated on rbpodo controller
> `pgmode` simulation (VM or onbox) and real.

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
python3 -m compileall -q rb_gui/rb_servo_gui policy_runner/policy_runner scripts
```

Shell syntax checks:

```bash
bash -n scripts/check_deps.sh
if [ -f scripts/install_deps_ubuntu.sh ]; then bash -n scripts/install_deps_ubuntu.sh; fi
```

C++ hardware-free checks:

```bash
cmake -S rb_servo_server -B rb_servo_server/build
cmake --build rb_servo_server/build -j
ctest --test-dir rb_servo_server/build --output-on-failure
```

Cartesian acceptance (against an already-running rbpodo/mock server):

```bash
python3 scripts/cartesian_acceptance.py --mode assume-running
```

## Dependency Preflight

```bash
./scripts/check_deps.sh --profile hardware-free
```

Install missing base packages on Ubuntu:

```bash
./scripts/install_deps_ubuntu.sh --profile hardware-free
```

`rb_servo_server` hardware-free C++ gates require valid Eigen3 and Pinocchio
CMake packages. The Ubuntu helper installs Pinocchio under `/opt/openrobots`:
robotpkg remains the jammy default, while non-jammy hosts such as Ubuntu 24.04
noble use a pinned Pinocchio `v3.9.0` source build when `ROBOTPKG_DIST` is
unset. Source builds use an automatic memory-capped job limit, overridable with
`PINOCCHIO_BUILD_JOBS`, to avoid OOM on low-RAM hosts. Missing Pinocchio is
reported as `Missing CMake package: pinocchio`; it may be skipped only when
`CODEX_SKIP_MISSING_CPP_DEPS=1` is explicitly set, and skipped C++ gates are
not acceptance evidence.

## Mock Smoke

Run `rb_servo_server` with an explicit site-local mock config under
`rb_servo_server/config/local/` and drive it with the bundled sender tools; the
state stream and servo log are the smoke evidence. Controller behavior beyond
mock is validated on rbpodo controller `pgmode` simulation (VM or onbox).

## Direct And Worker I/O

Direct mode and worker mode are both hardware-free (mock) validation targets.

Direct mode validates the straightforward servo loop/backends path.

Worker mode validates the long-term architecture where each arm worker owns blocking backend I/O. Worker mode is still hardware-free/mock-only unless a future real-hardware acceptance explicitly opens it.

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
- no real robot IPs are used in mock configs
- no real robot env gates are required
- no force control is enabled
- mock-mode motion primitives stay hardware-free

## Not A Hardware Acceptance

A passing hardware-free gate is not permission to run a physical robot. Real robot work must start with a separate read-only acceptance plan.
