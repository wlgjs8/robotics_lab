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

## Hardware-free rb_simulator

`config/dual_rb_simulator.yaml` pairs with the repo-local `rb_simulator`
process on loopback endpoint `tcp://127.0.0.1:50200`. The current standard is
one dual-arm simulator process: both `RbsimBackend` instances use that same
endpoint and select left/right by the request `arm` field. It uses the
`rbsim_local` backend and does not require `rbpodo`, Rainbow Robotics OVA
assets, privileged Docker, real robot hardware, or exposed network binds.
`dual_rbsim.yaml` is a compatibility alias for this same local
software-simulator shape.

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
