# Archived Configs

These YAML files are historical compatibility configs only. They are not
runnable source-of-truth configs for the current controller-simulation
acceptance surface.

Use the active configs in:

- `rb_servo_server/config/`

Current runnable stack configs use the rbpodo backend with either
`operation_mode: simulation` (`stack_sim.yaml`) or `operation_mode: real`
(`stack_real.yaml`). Site-specific variants and acceptance-stage copies belong
under `rb_servo_server/config/local/`; the legacy `dual_real*.example.yaml`
template surface is no longer tracked.
