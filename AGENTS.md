# AGENTS.md

## Project

This repository is `robotics_lab`, a dual-arm RB3-730 robotics integration repo.

The target architecture is:

- One physical/simulated controller endpoint per arm.
- left real robot controller: 172.28.60.200
- right real robot controller: 172.28.60.201
- simulator must mimic this topology with two independent per-arm simulator processes/containers.
- Public terminology:
  - run_mode: mock | simulation | real
  - backend_type: mock | simulator | rbpodo
- Deprecated public terms:
  - rbsim_local
  - rbsim

## Required reading before making changes

Always read:

- TODO.md
- README.md if present
- Relevant component README/docs for the assigned work package

## Hard safety rules

Never enable real robot motion implicitly.

Do not allow real robot connection unless:

- RB_ALLOW_REAL_ROBOT=1

Do not allow real servo_j motion unless:

- RB_ALLOW_REAL_MOTION=1

Do not allow real Cartesian/TCP motion unless:

- RB_ALLOW_REAL_CARTESIAN=1

Do not make simulator endpoints use the real robot IP addresses by default.

Do not activate force/admittance/impedance control.
Force control must remain:

```yaml
force_control:
  provider: null
  enable: false
```
TCP Cartesian commands must remain disabled until P3 simulation-only enablement.

## Development rules

Work only on the assigned TODO.md work package.

Avoid touching unrelated modules.

If a change requires touching another module, explain why in the final report.

Prefer small, reviewable changes.

Do not remove historical docs unless TODO.md explicitly says to archive or mark them historical.

Do not fake external APIs. For rbpodo, Pinocchio, RealSense, or SpaceMouse APIs, inspect actual headers/docs or gate the feature behind a clear dependency check.

## Required final report

At the end, report:

Work package completed
Files changed
Config files added/changed
Tests run
Test results
Any failed tests and why
Intentional remaining TODOs
Cross-module assumptions