# Archived Configs

These YAML files are historical compatibility configs only. They are not
runnable source-of-truth configs for the current controller-simulation
acceptance surface.

Use the active configs in:

- `rb_servo_server/config/`

Current runnable controller-simulation configs use the rbpodo backend with
`operation_mode: simulation`. Real robot configs must remain site-local or use
the tracked template `rb_servo_server/config/dual_real.example.yaml`.
