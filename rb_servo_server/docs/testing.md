# Testing

The repository-wide hardware-free contract is
[../../docs/hardware_free_validation.md](../../docs/hardware_free_validation.md).

## C++ gate

```bash
cmake -S rb_servo_server -B rb_servo_server/build
cmake --build rb_servo_server/build -j
ctest --test-dir rb_servo_server/build --output-on-failure
```

Eigen3 and Pinocchio are mandatory. A missing dependency or skipped C++ build
is not acceptance evidence.

## Config preflight

The server can validate a tracked config without connecting to hardware:

```bash
rb_servo_server/build/rbpodo_real_gate/rb_servo_server \
  --check-config --config rb_servo_server/config/stack_real.yaml

rb_servo_server/build/rbpodo_real_gate/rb_servo_server \
  --check-config --config rb_servo_server/config/stack_sim.yaml
```

Use whichever freshly built server path the repository build produced. A
preflight validates schema and fail-closed relationships; it does not validate
controllers, sensors, scheduling, force response, or physical motion.

## Mock smoke

No third tracked launch config exists. If a mock smoke is required, create an
explicit temporary YAML outside the repository (prefer a `mktemp -d` path), run
the server against it, and record the state/CSV artifact. Do not place mock or
real variants under `config/local`.

The smoke must preserve the supported J3 range `[-150 deg, +150 deg]`, must not
use real controller IPs, and must not enable a physical F/T or real-motion path.

## Evidence boundary

Hardware-free tests can exercise force-pipeline/law logic and automatic-tare
state machines, but cannot validate the measured sensor basis, tare quality,
contact response, or fences. Worker interpolation is unit-tested but remains a
separate supervised hardware A/B before it can become the tracked real profile.

Real/controller-simulation acceptance uses the supervised runbooks. Passing
this page's checks is never permission to move hardware.
