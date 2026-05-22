# Config Examples

The current config parser supports the simple YAML shape used in `config/*.yaml`:

```yaml
section:
  key: value
  array_key: [1, 2, 3]
```

It is intentionally minimal. Replace with `yaml-cpp` before adding complex nested structures.

## Mock

```bash
./build/rb_servo_server --config config/dual_mock.yaml
```

Mock mode uses `MockBackend` for both arms.

## Hardware-free simulator

`config/dual_simulator.yaml` pairs with two repo-local simulator processes,
one per arm:

```yaml
left_robot:
  backend_type: simulator
  run_mode: simulation
  simulator_control_endpoint: "tcp://127.0.0.1:50200"

right_robot:
  backend_type: simulator
  run_mode: simulation
  simulator_control_endpoint: "tcp://127.0.0.1:50210"
```

For Docker Compose, use `config/dual_simulator_compose.yaml`; it points to
`tcp://rb_simulator_left:50200` and `tcp://rb_simulator_right:50200`.
The deprecated `dual_rb_simulator.yaml`, `dual_rb_simulator_compose.yaml`, and
`dual_rbsim.yaml` profiles are compatibility aliases only. New configs should
use `backend_type: simulator`, `run_mode: simulation`, and
`simulator_control_endpoint`.

See `docs/rb_simulator_dev.md` for the supported unit and local-smoke evidence.

## Real robot

Real robot startup is intentionally omitted from the hardware-free simulator
phase. It requires explicit human-gated approval, real hardware readiness, and
the real-mode guard environment outside this local simulator workflow.

## Force control

Force control is present but disabled by default:

```yaml
force_control:
  enable: false
```

Enable only after Cartesian FK/IK and F/T sensor handling are implemented and tested.
