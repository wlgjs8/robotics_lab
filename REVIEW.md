# Current Review Note

This file is a lightweight review snapshot. For active direction, use
`README.md`, `AGENTS.md`, `docs/architecture.md`, and the component READMEs.

Current public motion primitives:

- `JointTarget`
- `TcpPoseTarget`
- `TcpLinearMove`

`init_motion` is a `JointTarget` profile, not a packet mode. Policy/model
rollout and SpaceMouse teleop emit absolute `TcpPoseTarget` setpoints.

Real motion remains operator-supervised and fail-closed through the tracked
stack config, server safety layers, command-source lease/arbitration, and
hardware E-stop.

Current review-sensitive boundaries:

- J3 is fixed to the Rainbow/URDF range `[-150 deg, +150 deg]` in every
  supported profile. The checked-out controller-manager RB3 model still says
  `+/-155 deg`; do not propagate it here, and align that external config in a
  separately authorized change.
- Force-control v2 is live and hardware-validated; an untared arm is not
  covered, and the tracked gate/spring/fence configuration is indivisible.
- `servo.worker_setpoint_interpolation` is implemented and unit-tested but is
  still `false` in `stack_real.yaml` pending its own supervised hardware A/B.
- The tracked pgmode-sim telemetry tuple remains `run_mode: real`,
  `backend_type: rbpodo`, `operation_mode: simulation`; do not infer a retired
  software-simulator backend from the public `simulation` vocabulary.
