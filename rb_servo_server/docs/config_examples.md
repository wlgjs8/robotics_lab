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
./build/rb_servo_server --config config/local/<mock-config>.yaml
```

Mock mode uses `MockBackend` for both arms. No tracked runnable mock config is
kept; use a site-local YAML under `config/local/`. Controller-level simulation
uses the rbpodo `pgmode` simulation flavor (`make run MODE=sim`).

## Real robot

`config/stack_real.yaml` is the tracked real-stack reference template. Actual
site-specific real robot YAML files belong under `config/local/`, and local
`*.yaml` files there are gitignored. Use a local copy such as
`config/local/stack_real_readonly.yaml` for read-only bring-up and a separate
local copy such as `config/local/stack_real_motion.yaml` only for approved motion
procedures.

The real stack uses the assigned controller IPs:

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

Real robot behavior remains gated outside the hardware-free workflow, but it is
config-driven, not env-gated (the legacy `RB_ALLOW_REAL_*` env gates were removed
from the server runtime). Read-only real connection keeps
`servo.send_servo_commands: false`; real `servo_j` motion requires
`servo.send_servo_commands: true` in a site-local real config; real Cartesian/TCP
motion additionally requires `cartesian_control.allow_in_real: true`. Together
with the mode-independent safety layers, operator supervision, and the hardware
E-stop, the site-local config is the sole decider. The rbpodo `disable_waiting_ack` arm option is
wired to the SDK ACK-wait toggle and defaults to `false` in the tracked stack
template; only change it in site-local motion configs after command-path
acceptance.

## Force control

Force control is enabled in the tracked real config (see `AGENTS.md` §Force
Control); the struct defaults below are the disabled shape:

```yaml
force_control:
  enable: false
```

Enable only after Cartesian FK/IK and F/T sensor handling are implemented and tested.
