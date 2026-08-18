# cm_bridge — controller-manager integration bridge

`cm_bridge` connects the robotics_lab policy stack to the company controller
stack **controller-manager** (`submodules/controller-manager`, 박천만님 소유),
replacing `rb_servo_server`'s low-level rbpodo streaming while keeping the
robotics_lab safety and policy layers.

Read `docs/design.md` before touching anything here. Status: **live.** The
bridge runs the policy chunk stream into controller-manager's FollowUnit; the
SILS gate is green and a full `real_policy` rollout has run on hardware.
controller-manager runs **natively** (no docker since 2026-08-18) — build it
with `tools/cm_local_setup.sh`, launch with `./run_cm_stack.sh {sim|real|gate}`.

Our task/tool overrides live in `config/monkey/` (and its SILS twin
`config/monkey-sils/`), which IS a controller-manager platform directory: CM
resolves `params-tasks` and `params-presets` relative to the loaded
`active.yaml`, and the launcher points `CONTROL_MANAGER_ACTIVE_YAML` there.
Files we override are real; everything else is a symlink into the submodule, so
a pull cannot leave a stale copy behind — and the launcher refuses to start on a
dangling or empty one, because CM downgrades a missing task file to compiled
defaults with only a WARN.

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
├── README.md            this file
├── docs/design.md       architecture, contracts, phases, open items
├── docs/replay.md       2 ms 3D replay: chunk input / follow target+ref / command / actual / gripper
├── docs/real_bringup.md operator ladder
├── src/                 cm_bridge_node.py (chunk->follow, state republish, gripper, sidecar log)
│                        collision_monitor.py
├── config/monkey/       OUR controller-manager platform dir (follow.yaml, pika preset; rest symlinked)
├── config/plans.yaml    named-plan library (init pose) for the cockpit / plan CLI
├── tools/               cm_replay.py (viser 3D replay), cm_record.sh (func write start/stop), chimpbin.py
├── tests/               cm_sils_gate.py (stream gate), record_gate.sh (recorder gate, isolated), noaffinity.c
└── upstream/            patches to controller-manager not yet merged upstream (+ how to re-apply)
```

**Follow structure (2026-08-19): controller-paced N-step commit.** `follow.yaml commit_steps: 4`
+ `act/follow_step` events (upstream patch 0002); the bridge's `FollowPacer` slices the newest
runner chunk to the step the controller is about to play and fires the per-step gripper target
when the controller reports that step finished (`--follow-mode commit --gripper-source
follow_step`, the defaults; `replace`/`command` = the old behaviour). `docs/design.md` §7c.

Recording + replay: `docs/replay.md`. The recorder columns it needs are a controller-manager
change (`upstream/0001-*.patch`, schema 4) that lives in the submodule working tree until it is
upstream — `run_cm_stack.sh` refuses to launch a binary without it.
