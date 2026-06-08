# Plan: Windows + WSL2 + Rainbow Virtual ControlBox (rbpodo pgmode)

Status: planning seed (for Claude Code to refine into a code-level spec, then Codex to implement).
Scope: bring up the **control system** on a home Windows laptop, **without a physical robot**, using the official Rainbow **Virtual ControlBox** as an rbpodo controller endpoint in **pgmode simulation** on **WSL2 (Ubuntu)**.

Controller-simulation evidence only. This plan does **not** authorize physical robot motion.
`physical_motion_expected=false`, `RB_ALLOW_REAL_CARTESIAN` is never set.

---

## 0. Source of truth (read first — do not duplicate)

This plan only adds the Windows/WSL2-specific layer on top of existing runbooks.
Existing runbooks remain authoritative for everything they cover:

- `docs/runbooks/vm_network_bringup.md` — OVA verify, import checklist, reachability probe, ports 5000/5001, one-VM-per-arm.
- `docs/runbooks/vm_real_parity.md` — home VM vs office real mode matrix, `${ROBOT_LEFT_IP}`/`${ROBOT_RIGHT_IP}` config pattern, `tools/vm/home_vm_env.sh`, WU-01..WU-05 sequence.
- `docs/developer_environment.md` — `scripts/install_deps_ubuntu.sh`, `scripts/check_deps.sh`, HARDEN-10, CART-MATH-03 gates.
- `docs/runbooks/rbpodo_pgmode_spacemouse.md`, `docs/runbooks/pgmode_real_transition.md`, `docs/runbooks/real_robot_readonly.md`.
- `AGENTS.md`, `REVIEW.md`, `docs/architecture.md`, `docs/code_architecture_map.md`.

Existing tooling to reuse (do not reinvent):
`tools/vm/verify_ova.sh`, `tools/vm/probe_vm_reachability.sh`, `tools/vm/home_vm_env.sh`, `scripts/check_vm_artifact_tagging.py`.

---

## 1. Facts established from the official VM manual

Source: https://rainbowrobotics.github.io/rb_cobot_docs/technical_docs/virtual_controlbox

- Virtualization platform: **VirtualBox** (.ova import). Not VMware.
- Network adapter: **Host-Only Adapter** required.
- Manual example network range: **10.0.2.x**, client connects at **10.0.2.7**.
  - Note: `vm_network_bringup.md` uses `192.168.56.x` as its host-only example. These are just examples; the actual host-only subnet is whatever VirtualBox assigns. Keep real values in local-only files.
- Mode: **simulation only** — the Virtual ControlBox has **no Real Robot mode**. This maps cleanly to `operation_mode: simulation` and is a safety bonus (physical motion is impossible by construction).
- Ports: manual does not list them; `vm_network_bringup.md` confirms rbpodo fixed **5000 (command) / 5001 (data)**.

---

## 2. The new problem this plan solves: WSL2 <-> VirtualBox networking

The existing runbooks assume a Linux host where VirtualBox host-only IPs are directly reachable.
Here the host is **Windows**, `rb_servo_server` runs in **WSL2**, and the VM runs in **VirtualBox on the Windows host**.

```text
Windows host
 |- VirtualBox: Rainbow Virtual ControlBox (Host-Only, 10.0.2.x / 192.168.56.x, pgmode)
 |- RB Window UI (Windows app) -> connects to the controller VM for monitoring/teach
 |- WSL2 (Ubuntu): robotics_lab / rb_servo_server (rbpodo) -> must reach the VM IP:5000/5001
```

Default WSL2 NAT **cannot** reach the VirtualBox host-only network. Primary proposed fix:

- **WSL2 mirrored networking** (Windows 11 22H2+). In `%USERPROFILE%\.wslconfig`:
  ```ini
  [wsl2]
  networkingMode=mirrored
  ```
  Then `wsl --shutdown` and reopen WSL. Mirrored mode shares the host's interfaces (incl. the VirtualBox host-only adapter), so WSL2 can reach the VM IP directly.

OPEN ITEM (must be verified empirically, see Stage 3): confirm that mirrored networking actually reaches a VirtualBox **host-only** adapter. If it does not, fall back options to evaluate:
  - VirtualBox **bridged** adapter (only if acceptable; manual prefers host-only),
  - a host-side TCP proxy / `netsh portproxy` forwarding `5000/5001` from a WSL-reachable host IP to the VM,
  - running `rb_servo_server` natively where reachability is simplest (not preferred — Linux toolchain).

---

## 3. Staged work plan

Each stage lists the goal, the existing assets to use, and the pass check.
Stages 0-1 are environment; Stages 2-3 are the new Windows/WSL2/VM layer; Stage 4 maps onto the existing WU-01..WU-05 ladder.

### Stage 0 — WSL2 + Docker + repo
- Install WSL2 Ubuntu + Docker Desktop (WSL2 backend).
- **Clone the repo inside the WSL filesystem** (e.g. `~/robotics_lab`), not under `/mnt/c`, to avoid slow I/O and CRLF/permission issues. Keep the Windows copy only for Claude Desktop file viewing.
- Ensure shell scripts keep **LF** line endings (they are bash). Verify `tools/*.sh` and `scripts/*.sh` are executable.
- Pass check: `git status` clean in WSL clone; `bash -n scripts/codex_gate.sh` ok.

### Stage 1 — Build & hardware-free sanity (no VM yet)
- `./scripts/install_deps_ubuntu.sh --profile hardware-free` (Eigen3 + Pinocchio via robotpkg at `/opt/openrobots`).
- `./scripts/check_deps.sh --profile hardware-free`.
- Python: `python3 -m unittest discover rb_gui/tests`, `policy_runner/tests`, `PYTHONPATH=rb_simulator/src python3 -m unittest discover rb_simulator/tests`.
- C++ gates: `./scripts/codex_gate.sh HARDEN-10`, then `./scripts/codex_gate.sh CART-MATH-03`.
- (Optional confidence) `make sim-local-up` -> open http://127.0.0.1:8080 -> `make sim-down`.
- Pass check: deps check ok, Python suites pass, HARDEN-10 + CART-MATH-03 pass. This proves the toolchain before adding VM complexity.
- Investigate: does rbpodo backend need an SDK install separate from the hardware-free profile? Document the rbpodo SDK acquisition/build step here (gap in current docs for the home setup).

### Stage 2 — VirtualBox + Virtual ControlBox import (single VM first)
- Install VirtualBox on the Windows host. Import the `.ova`.
- **Start single-arm**: one VM = one controller. Dual-arm (two VMs) is Stage 4+ per `vm_network_bringup.md`.
- Configure NIC = Host-Only Adapter per the manual. Note the assigned host-only subnet and the controller VM IP.
- Launch RB Window UI on Windows, connect (green indicator), and **confirm pgmode simulation**.
- Verify the OVA with the existing tool (run from WSL against the file path):
  `tools/vm/verify_ova.sh /path/to/rainbow-controller-sim.ova --output artifacts/vm_parity/WU-01/ova_verify.json`
- Pass check: VM boots, RB Window UI connects, pgmode confirmed, `ova_verify.json` written.

### Stage 3 — WSL2 reachability to the VM (the crux)
- Apply WSL2 mirrored networking (Section 2). `wsl --shutdown`, reopen.
- From WSL2: `ping <vm-ip>`, then `tools/vm/probe_vm_reachability.sh --left <vm-ip> --output artifacts/vm_parity/WU-01/reachability.json`.
  - NOTE: existing tooling expects `--left` and `--right` (two VMs). For single-VM start, confirm whether the probe / `home_vm_env.sh` support a single endpoint, or whether a small adaptation is needed. **Flag for Claude Code: single-arm support in `tools/vm/*` and in `rb_servo_server` config.**
- If host-only is unreachable from WSL2, evaluate the fallbacks in Section 2 and record which one works.
- Pass check (WU-01): TCP connect to `<vm-ip>:5000` and `:5001`; optional `--try-rbpodo-state` receives `CobotData` once.

### Stage 4 — rbpodo bring-up ladder (reuse existing WU sequence)
Follow `docs/runbooks/vm_real_parity.md` "Recommended Sequence" exactly:
- WU-02: read-only rbpodo state dump with `servo.send_servo_commands=false` (`tools/vm/home_vm_env.sh ... --readonly`).
- WU-03: local VM config under `rb_servo_server/config/local/` using `${ROBOT_LEFT_IP}` (env-indirected; gitignored).
- WU-04: no-op Servo J streaming, then tiny controller-simulation joint motion with explicit motion gates (`--motion`).
- WU-05: prove Cartesian rejected with closed gates, then accepted only with controller-simulation Cartesian gates (`--cartesian`), `physical_motion_expected=false`.
- Then 500Hz no-op -> circle ladder per `docs/runbooks/rbpodo_500hz_acceptance.md` and `rbpodo_controller_sim_circle.md`.
- Artifact hygiene: every artifact under `artifacts/vm_parity/` carries `{"source":"controller_simulation_vm","physical_motion":false}`; run `python3 scripts/check_vm_artifact_tagging.py`.

### Stage 5 — Dual-arm expansion
- Import the OVA a second time (`rbvm-left`, `rbvm-right`), distinct host-only IPs, regenerate MACs (per `vm_network_bringup.md`).
- Switch config to `${ROBOT_LEFT_IP}` + `${ROBOT_RIGHT_IP}`; rerun the ladder dual-arm.

---

## 4. Open questions / risks for Claude Code to resolve

1. Does WSL2 mirrored networking actually reach a VirtualBox host-only adapter? If not, which fallback (bridged / portproxy)? (Stage 3 — empirical)
2. rbpodo SDK acquisition/build on WSL2 for the home setup — current docs cover hardware-free deps, not the rbpodo SDK install path. Document it.
3. Single-arm (one VM) support in `tools/vm/probe_vm_reachability.sh`, `tools/vm/home_vm_env.sh`, and `rb_servo_server` config — do they require both arms, or is a single endpoint supported?
4. Host-only subnet discrepancy (manual `10.0.2.x` vs runbook `192.168.56.x`) — confirm actual assigned subnet and keep real IPs in local-only files.
5. Confirm actual exposed ports of this OVA build are `5000/5001` via `probe_vm_reachability.sh`.

## 5. Tool-workflow handoff

- **Claude Code**: refine this seed into a code-level spec (resolve Section 4), produce per-stage commands and acceptance, and author/adjust any needed `tools/vm` single-arm support.
- **Codex** (called from Claude Code): implement the concrete code/config/script changes per the refined spec, one reviewable change at a time, following `AGENTS.md`.
- Every change: do not weaken real-mode/Cartesian gates; keep `operation_mode: simulation`; controller-simulation evidence only.
