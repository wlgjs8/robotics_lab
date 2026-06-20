# Config Examples

The current config parser supports the simple YAML shape used in `config/*.yaml`:

```yaml
section:
  key: value
  array_key: [1, 2, 3]
```

The parser is implemented with `yaml-cpp` and validates known keys strictly.
When adding nested structures, update the parser allowlists, validation, and
config-loader tests together.

## Mock

```bash
./build/rb_servo_server --config config/dual_mock.yaml
```

Mock mode uses `MockBackend` for both arms.

## Hardware-free simulator

Use `config/dual_simulator.yaml` for host-run simulator checks. It pairs with
two repo-local simulator processes, one per arm, on separate loopback ports:

```yaml
left_robot:
  backend_type: simulator
  run_mode: simulation
  name: left_simulator
  simulator_control_endpoint: "tcp://127.0.0.1:50200"

right_robot:
  backend_type: simulator
  run_mode: simulation
  name: right_simulator
  simulator_control_endpoint: "tcp://127.0.0.1:50210"
```

For a split-PC simulator stack, `config/dual_simulator_remote_172_28_60_36.yaml`
points the per-arm endpoints at simulator processes on `172.28.60.36` instead of
loopback.

The deprecated `dual_rb_simulator.yaml`, `dual_rb_simulator_compose.yaml`, and
`dual_rbsim.yaml` names have been moved out of the active config directory to
`docs/archive/configs/` for historical reference only. They are not runnable
source-of-truth profiles and must not be used for new operator runs or
acceptance evidence. New configs should use `backend_type: simulator`,
`run_mode: simulation`, and `simulator_control_endpoint`.

See `docs/rb_simulator_dev.md` for the supported unit and local-smoke evidence.

## Real robot

`config/dual_real.example.yaml` is a template only. Actual site-specific real
robot YAML files belong under `config/local/`, and local `*.yaml` files there
are gitignored. Use `config/local/dual_real_readonly.yaml` for read-only
bring-up and `config/local/dual_real_motion.yaml` only for separately approved
motion procedures.

The real example uses the assigned controller IPs:

```yaml
left_robot:
  backend_type: rbpodo
  run_mode: real
  ip: "172.28.60.200"

right_robot:
  backend_type: rbpodo
  run_mode: real
  ip: "172.28.60.201"

servo:
  send_servo_commands: false
```

Real robot startup remains gated outside the hardware-free simulator workflow:
`RB_ALLOW_REAL_ROBOT=1` is required for read-only real connection, and
`RB_ALLOW_REAL_MOTION=1` plus `servo.send_servo_commands=true` is required for
real `servo_j` motion. Real Cartesian/TCP motion additionally requires
`RB_ALLOW_REAL_CARTESIAN=1`. The rbpodo `disable_waiting_ack` arm option is
wired to the SDK ACK-wait toggle and defaults to `false` in the tracked real
template; only change it in site-local motion configs after command-path
acceptance.

## Force control

Force control is present but disabled by default:

```yaml
force_control:
  enable: false
```

Enable only after Cartesian FK/IK and F/T sensor handling are implemented and tested.
