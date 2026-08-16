# Archived: Rainbow Virtual ControlBox (VM) lane

The VM lane was **removed from the codebase on 2026-08-16**. These files are kept
as a historical record only — they are not runnable operator instructions, and
every tool, script, and `make` target they reference no longer exists.

## What the lane was

Two vendor Rainbow Virtual ControlBox VMs (VirtualBox, imported from an OVA) were
booted on a host-only network and mapped onto the *real* controller IPs
(`172.28.60.200/.201`) with iptables DNAT, so `make run MODE=sim` could reach
simulated control boxes without hardware.

## Why it was removed

- `make run MODE=sim` never depended on it: `rb_servo_server/config/stack_sim.yaml`
  points at the physical control boxes held in `pgmode`. The VM lane only worked
  by intercepting those addresses.
- The tracked `stack_sim.yaml` sets
  `controller_simulation_physical_motion_policy: fault_latch`, which is
  incompatible with a Virtual ControlBox (its simulated `q_actual` follows
  commands and would trip the latch). The lane was therefore already unusable.
- Its remaining cost was purely confusion: two parallel "simulation" meanings in
  the docs, `make` targets that could not work, and a `tools/vm/` directory that
  also held unrelated native scripts.

## Removed with it

`tools/vm_stack.sh`, `tools/vm/` (`home_vm_env.sh`, `probe_vm_reachability.sh`,
`verify_ova.sh`, and the already-broken native `pgmode_sim_{build,up,down}.sh`),
`scripts/check_vm_artifact_tagging.py` + its tests, `scripts/test_vm_stack.py`,
the `make vm-up / vm-down / vm-status` and `make pgmode-sim-*` targets, and the
`artifacts/vm_parity/**` evidence convention.

## Retained on purpose

- `${ROBOT_LEFT_IP}` / `${ROBOT_RIGHT_IP}` environment expansion in the rbpodo
  config loader — a general config-loader feature, covered by
  `rb_servo_server/tests/test_config_loader.cpp::testRobotIpEnvExpansion`. The VM
  runbooks were only one documented *use* of it.
- `servo.allow_controller_simulation_init_error` /
  `allow_controller_simulation_not_activated` — these exist because a Virtual
  ControlBox permanently reports `init_error=187` / not-activated, but they are
  live fail-closed config surface with C++ implementation and tests. Removing
  them is a separate, testable change.

Current simulation guidance: root `README.md` and `docs/architecture.md`
(controller simulation = a physical box held in `pgmode`).
