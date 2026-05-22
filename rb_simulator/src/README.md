# Source Layout

`rbsim` contains the dependency-free simulator core:

- `config.py` loads the local YAML profile into typed config objects.
- `state_machine.py` owns the deterministic dual-arm lifecycle and joint
  stepping logic.
- `protocol.py` owns the versioned `rbsim.v1` JSON Lines contract.
- `server.py` owns the loopback-only control/admin TCP service.
- `__main__.py` exposes `python -m rbsim --config ...`.

Keep this workspace hardware-free, deterministic, and independent from
`mo_rbsim_docker`.
