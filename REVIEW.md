# Current Review Note

This file is a lightweight review snapshot. For active direction, use
`README.md`, `AGENTS.md`, `docs/architecture.md`, and the component READMEs.

Current public motion primitives:

- `JointTarget`
- `TcpPoseTarget`
- `TcpLinearMove`

`init_motion` is a `JointTarget` profile, not a packet mode. Policy/model
rollout and SpaceMouse teleop emit absolute `TcpPoseTarget` setpoints.

Real motion remains operator-supervised and fail-closed through site-local
config, server safety layers, command-source lease/arbitration, and hardware
E-stop.
