# cm_bridge — controller-manager integration bridge

`cm_bridge` connects the robotics_lab policy stack to the company controller
stack **controller-manager** (`submodules/controller-manager`, 박천만님 소유),
replacing `rb_servo_server`'s low-level rbpodo streaming while keeping the
robotics_lab safety and policy layers.

Read `docs/design.md` before touching anything here. Status: **P0 skeleton —
design only, no runtime code yet.**

## Why

- controller-manager owns the details that make high-fidelity servo_j streaming
  work on RB control boxes: firmware-gated bring-up (build 26071103 / label
  8.7.3), servo_j LPF **off** (`alpha=10.0`) enabled by that firmware, `%.7f`
  command precision, `system_manager_high_speed_comm(1)`, and the qsync FLL that
  regulates the box command-queue fill (target 5 ticks) using `RBACK[n]` ACKs.
  It also provides force control (Admittance task, `FtConfig`/`Wrench`/
  `AxisCompliance` messages).
- robotics_lab keeps: the policy runtime (policy_runner flow-infer chunk
  stream), cameras, GUI, and the **async URDF-mesh CollisionMonitor**
  (self-collision), which moves into this bridge as the pre-controller gate.

## Hard rules

1. **Never commit inside `submodules/controller-manager`.** It is consumed
   read-only, pinned by SHA. Changes to controller-manager are PRs to
   `PLAIF-dev/controller-manager`, never edits to the submodule tree.
2. Update flow: `cd submodules/controller-manager && git pull origin main`,
   run the SILS integration gate (see design doc §8), then commit the pin bump
   in robotics_lab. A pin bump without a passing SILS gate is not mergeable.
3. All robotics_lab safety invariants apply (fail-closed, no silent defaults —
   see repo `CLAUDE.md`). The bridge gates commands BEFORE controller-manager;
   controller-manager's own state machine and step guards remain the final
   authority below us.

## Layout

```
cm_bridge/
├── README.md          this file
├── docs/design.md     architecture, contracts, phases, open items
├── src/               bridge implementation (P1+)
├── config/            bridge configs (P1+)
└── tests/             SILS integration gate (P1+)
```
