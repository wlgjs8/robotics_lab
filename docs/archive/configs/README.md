# Archived Configs

These YAML files are historical compatibility configs only. They are not
runnable source-of-truth configs for the current simulator-first Cartesian
acceptance phase.

Use the active configs in:

- `rb_servo_server/config/`
- `rb_simulator/config/`

Current runnable simulator configs use `run_mode: simulation` and
`backend_type: simulator`, with one simulator endpoint per arm. Real robot
configs must remain site-local or use the tracked template
`rb_servo_server/config/dual_real.example.yaml`.
