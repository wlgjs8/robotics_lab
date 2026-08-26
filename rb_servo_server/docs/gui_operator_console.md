# Viser Operator Console Redirect

The current operator-console contract is maintained in
[../../rb_gui/README.md](../../rb_gui/README.md).

The GUI consumes server state and emits public command packets; it is not the
real-motion authority. Its client-side real lock is retired, while the tracked
server config, safety filters, collision monitor, lease/deadman, and backend
readiness remain authoritative.

The GUI includes live F/T monitoring plus manual/automatic tare visibility.
Force laws are server-config-owned. Use `make run MODE=sim` for controller
`pgmode` simulation and plain `make run` only for supervised physical real.
