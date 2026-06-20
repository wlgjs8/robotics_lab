# Gripper Server — Design Proposal (DRAFT)

Status: **proposal**. Date: 2026-06-20.

Implemented so far (on `dev`, GUI-only, no server/hardware yet):
- **B2a articulated-gripper visualization.** `pika_gripper.STL` split into
  `pika_gripper_base.STL` + `pika_finger_left/right.STL` (fingers separated as the
  two STL components reaching the fingertip plane; +90° Z baked in). GUI-only
  `rb3_730e_pika_articulated.urdf` (base + two prismatic finger joints). `scene.py`
  drives the fingers from a gripper open-% (continuous → partial close); `app.py`
  adds an interim "그리퍼 표시 (preview)" L/R slider as the source. The C++
  Pinocchio URDF (`rb3_730e.urdf`) is untouched. Verified: URDF loads (yourdfpy,
  8 actuated joints), fingers track 100%/50%/0% (94/47/0 mm gap), 162 GUI tests
  pass. Still pending: the real `gripper.percent` feed (Phases 1–3 below) replacing
  the slider, and live viser confirmation.

Goal: make the gripper a first-class, single-owner subsystem like `camera_server`,
unify its command path through `rb_servo_server` (so it inherits lease /
arbitration / deadman), publish gripper feedback in the server state stream, and
visualize open/close in the viser GUI.

This document is the agreed design to build against. Implementation is phased
(see §10) — nothing here is built yet.

---

## 1. Why change (current gaps)

The gripper today is fully out-of-band and fragmented:

- **No server awareness.** `rb_servo_server` has a vestigial `ArmCommand.gripper_target`
  (`include/rb_servo/core/types.hpp:449`) but never parses, arbitrates, applies,
  or publishes it. `RobotState` and the published `servo_state` JSON have no
  gripper field; the GUI `ArmSnapshot` (`rb_gui/rb_servo_gui/models.py`) has none.
- **Two independent driver paths over the same serial ports** → contention risk:
  - policy rollout: `policy_runner` → `GripperRuntime` → `PikaSerialGripperBackend`
    (`policy_runner/policy_runner/gripper.py`) → Pika SDK serial.
  - UMI teleop: `scripts/umi_gripper_follow.py` (UDP 50382) → Pika SDK serial.
- **No arbitration for an actuator.** The gripper can be driven by either path
  with no lease/deadman, unlike arm motion (which the server arbitrates). An
  operator's arm and that arm's gripper can end up owned by different sources.
- **Static visual only.** The viser URDF (`rb_servo_server/descriptions/urdf/rb3_730e.urdf`)
  carries the gripper as a single FIXED `pika_gripper.STL` (no jaw joint);
  `_ROBOT_JOINT_NAMES` (`scene.py`) is the 6 arm joints only → the gripper never
  moves on screen.

## 2. Target architecture

```
 command sources (UMI bridge · policy_runner · GUI)
        │  existing command JSON {seq, mode, left{…, gripper}, right{…, gripper}}  UDP 50010
        ▼
 rb_servo_server  ──(arm: FK/IK/safety/servo)──► arm backends
   └ lease / arbitration / deadman / freshness applied to the WHOLE packet
        │ arbitrated gripper setpoint (non-blocking UDP, ~100 Hz)     ▲ feedback (UDP, ~30–60 Hz)
        ▼                                                             │
 gripper_server  (NATIVE process; wraps the existing PikaSerialGripperBackend)
        │ serial (Pika SDK)  → left / right Pika grippers
        └ server caches latest feedback and stamps it into the state JSON
                 │
                 ▼  state fanout (existing 50110 gui / 50120 policy)
        GUI (viser open/close viz) + policy_runner (proprio)
```

Key decisions (locked):
- **Command rides the existing packet** (`left/right.gripper`) → gripper inherits
  the server's lease/arbitration/deadman automatically. One source owns an arm +
  its gripper; no separate gripper command plane.
- **Hardware lives in a separate process** (`gripper_server`) so blocking serial
  never touches the 500 Hz RT loop — the server only does non-blocking UDP forward.
- **Feedback path = server state JSON** (chosen): `gripper_server` → `rb_servo_server`
  → existing per-arm state JSON. GUI viz and policy proprio become pure
  downstream additions. The server only caches the latest value and attaches it
  (never blocks on gripper I/O).
- **Native, not Docker.** "Like camera" means *separate server process*, not
  Docker specifically. The gripper is light serial (pyserial + Pika SDK); native
  avoids `--device` passthrough and matches the rest of the native stack. (Docker
  remains camera-only.)

## 3. Component responsibilities

| Component | Owns | Does NOT own |
|---|---|---|
| command sources | produce desired gripper value in the command packet | hardware, arbitration |
| `rb_servo_server` | parse/validate/arbitrate gripper setpoint, forward to gripper_server, cache+publish feedback | serial I/O, motor mapping |
| `gripper_server` | serial port(s), percent↔rad mapping, homing, read feedback, publish feedback | command arbitration, lease |
| GUI / policy | render / consume gripper state from the state JSON | driving hardware |

## 4. Data model & wire protocols

### 4.1 Command (source → server, existing packet)
Add an optional gripper sub-field to each arm in the command JSON:
```json
{ "seq": N, "mode": "...",
  "left":  { ...arm fields..., "gripper": { "value": 0..100, "type": "target" } },
  "right": { ...arm fields..., "gripper": { "value": 0..100, "type": "target" } } }
```
- **Unit: percent, 0 = closed, 100 = open** (matches dataset / Pika UMI
  `gripper_open_close_units: percent` and `policy_runner/gripper.py`).
- `type`: `target` (absolute) initially; `delta` optional later.
- Absent `gripper` field = "no gripper command this packet" (arm-only command);
  the server holds the last gripper setpoint.
- Maps onto the existing `ArmCommand.gripper_target` (re-typed/clarified as percent).

### 4.2 Server → gripper_server (new, non-blocking UDP)
```json
{ "schema": "robotics_lab.gripper_cmd.v1", "seq": N,
  "left":  { "percent": 0..100, "valid": true },
  "right": { "percent": 0..100, "valid": true },
  "deadman": true, "host_time_ns": ... }
```
- Sent every server tick the gripper setpoint is fresh (rate-limited, e.g. 100 Hz).
- `valid=false` / `deadman=false` signals the stale/deadman policy (§7).

### 4.3 gripper_server → server (new, feedback UDP)
```json
{ "schema": "robotics_lab.gripper_state.v1", "host_time_ns": ...,
  "left":  { "percent": <actual>, "target_percent": <cmd>, "moving": bool, "ok": bool, "fault": null|str },
  "right": { ... } }
```
- `percent` from `PikaSerialGripperBackend.current_percent()` (live motor angle).
- Published at the serial read rate (~30–60 Hz).

### 4.4 Server state JSON (extend per-arm block)
Add to each arm in `servo_state` (schema bump or additive field):
```json
"gripper": { "percent": <actual>, "target_percent": <cmd>,
             "moving": bool, "ok": bool, "fault": null|str, "stale": bool }
```
- Stamped from the latest cached feedback; `stale: true` if no feedback within a
  timeout. Additive → old GUIs ignore it; bump `servo_state` minor if needed.

### 4.5 GUI model
`ArmSnapshot` (`models.py`) parses the new `gripper` block into a small
`GripperSnapshot` (percent, target, moving, ok, fault, stale).

## 5. gripper_server (new native process)

- Language: **Python** (reuse `PikaSerialGripperBackend` verbatim — it already has
  `connect()`, `send()`, `current_percent()`, homing on connect, percent↔rad with
  `min_rad=0.0`/`max_rad=1.75`, deadband).
- Shell to add: a UDP command listener (4.2) + a feedback publisher (4.3) + a
  small control loop (apply latest setpoint, read feedback, handle stale/deadman).
- Lifecycle: `connect()` (homes both jaws → defines closed stop), serve, graceful
  close. Mirrors how the backend is constructed in `policy_runner/main.py`.
- Retire the two old paths: `policy_runner` and `umi_gripper_follow.py` stop
  driving serial directly; they emit gripper values into the **command packet**
  (4.1) instead. `umi_gripper_follow.py` is folded into the command publisher.

## 6. rb_servo_server changes (C++)

1. Parse `left/right.gripper` in `command_server.cpp` into `ArmCommand.gripper_target`
   (+ a `has_gripper` / validity flag); reuse the existing command-freshness path.
2. Arbitration: the gripper setpoint is accepted only from the **lease-holding
   source**, identical to arm commands (no new arbitration code — it rides the
   same packet/lease).
3. Forward the arbitrated setpoint to `gripper_server` (4.2) from the loop's
   publish step — **non-blocking `sendto`**, never a serial/blocking call.
4. Receive feedback (4.3) on a small async listener, cache the latest per-arm
   value, and stamp it into the state JSON (4.4). The RT loop only reads a cached
   struct; it never waits on the gripper.
5. Config: `gripper` block (enable, endpoints, rate, stale timeout, deadman policy).
   No safety-filter involvement (gripper is not floor/ROI/collision constrained).

## 7. Stale / deadman / safety policy

- **Command stale** (no fresh gripper setpoint within timeout) or **deadman released**:
  gripper_server **holds last commanded position** (recommended) — do NOT auto-open
  (an object would drop) and do NOT force-close (could crush). Configurable.
- **Feedback stale**: GUI shows the gripper grey/`stale`; policy proprio uses last
  known + stale flag.
- The arm safety layers (floor/ROI/self-collision) are unchanged and do not apply
  to the gripper. Gripper "safety" is limited to not exceeding motor limits
  (already clamped in the backend) and the hold-on-stale policy above.

## 8. viser visualization

Two render options on top of the new state feed (state feed is the prerequisite;
once `ArmSnapshot.gripper.percent` exists either is a GUI-only change):

- **B1 — fingertip markers (light, first).** Per arm, draw two small finger markers
  under `/stand/<side>_tcp` (the URDF TCP / fingertip plane, `tcp_joint` at
  +0.2476 m); inner separation = percent→width. Plus a numeric/gauge readout. No
  URDF/mesh change. Clear open/close, low effort.
- **B2 — articulated fingers (faithful, continuous partial-close).** Split the
  gripper into a fingerless base + two finger meshes (from the STEP, §9) and drive
  one finger DOF from `gripper.percent` (continuous → any partial-close pose).

  **Hard constraint:** the C++ Pinocchio FK/IK loads the SAME `rb3_730e.urdf`
  (`kinematics.urdf` in every stack config). Adding an actuated finger joint there
  changes the server's DOF/FK/IK model — **do NOT edit the shared URDF.** The
  articulation must be **GUI-isolated**. Two GUI-only ways:
  - **B2a (recommended) — GUI-private articulated URDF.** Author a viewer-only URDF
    (`rb3_730e_pika_articulated.urdf`, selected via `RB_GUI_ROBOT_URDF`/`_robot_urdf_path`):
    same arm chain, but the `tool` link uses a **fingerless base mesh** and two
    `finger_left`/`finger_right` links are added with ONE actuated jaw joint
    (`finger_right` `mimic`s `finger_left`). `ViserUrdf.get_actuated_joint_names()`
    then returns 6 arm + 1 finger; `_update_urdf_config` feeds a 7-value config =
    6 arm joints + `theta(percent)`. The C++ server URDF is untouched. Per-arm
    `gripper.percent` drives each arm's ViserUrdf independently.
  - **B2b — viser scene-node fingers (no URDF).** Keep the arm URDF, but render the
    fingerless base + two finger meshes as plain viser scene nodes parented under
    `/stand/<side>_tcp`, and set each finger's transform per frame from `percent`.
    No ViserUrdf/joint plumbing, but you hand-code the finger kinematics. Both
    B2a/B2b require the same fingerless-base + finger-mesh split.

  `theta(percent) = lerp(closed, open, percent/100)` with a revolute (pivoting jaw)
  or prismatic (parallel jaw) joint; limits chosen so 0 %/100 % match the CAD
  poses. Linear interp of a single DOF is the pragmatic default; the true 4-bar
  linkage (STEP `M-D080-180003-C` pair) can be modeled later for accuracy.

Recommendation: ship B1 with the state feed; upgrade to **B2a** using the STEP
fingers. Either B2 path needs the `gripper.percent` feed (or a debug slider).

## 9. Pika gripper STEP — findings & articulation plan

Source: `/home/plaif/Downloads/Pika Gripper.STEP` (SolidWorks 2019, AP214, mm).
Inspected via `cascadio` STEP→glb + trimesh. Orthographic views (fingers in red):
`docs/images/pika_gripper_step_views.png`.

- It is the **full Pika handheld assembly**: 17 solids — handle/body + Pika Tracker
  PCBs + a 4310-class servo motor + linkage + the two fingers. World bbox ≈
  **215 × 143 × 190 mm**.
- **Two mirror-symmetric finger solids** `M-D080-180008-B` (instances `NAUO10`,
  `NAUO11`), each ~9.9k verts:
  - centroids (world, mm): `(+62, -7, 202)` and `(-82, -7, 202)`.
  - **Open/close (jaw) axis = X** (fingers separated in X); **tips at the +Z end**
    (z up to ~271 mm); pivot region near z≈150 mm where the `M-D080-180003-C`
    **linkage pair** sits → the real mechanism is a motor-driven **linkage (likely
    revolute/4-bar)**, not pure prismatic.
- Articulation plan for B2:
  1. Export the two finger solids (and, if wanted, the linkage) to STL from the glb
     (per-instance, world transforms applied — cascadio keeps assembly transforms
     in the trimesh scene graph, so apply `scene.graph[node]` before export).
  2. Strip the PCBs/handle (not part of the robot tool already modeled by the
     existing `pika_gripper.STL`); keep only the moving fingers.
  3. **Align to the existing repo frame**: the URDF tool uses `pika_gripper.STL`
     scaled 0.001, rotated +90° about Z, attached at `attachment_site` (link6
     +0.1 m), TCP at +0.2476 m (fingertip plane). Match the STEP fingertip plane
     (+Z end) to the repo TCP and apply the same +90° Z rotation; solve the rigid
     offset once (we have a known reference, as the user noted).
  4. Add per-side finger links + **one actuated joint** to a **GUI-private URDF**
     (NOT the shared `rb3_730e.urdf` — see §8 B2). Simplest faithful model: a
     **revolute jaw joint** about the pivot, mirrored L/R (`finger_right` `mimic`s
     `finger_left`), driven by `angle = lerp(closed, open, percent/100)` with limits
     chosen so the closed/open poses match the CAD. (A prismatic approximation is
     also fine for pure visualization.) The GUI then feeds 6 arm + 1 finger value
     to `ViserUrdf.update_cfg`; the C++ FK/IK model is unchanged.
- Tooling note: `cascadio` (`pip install cascadio`) is the offline STEP→mesh path
  used here; no FreeCAD/OCC needed for export.

## 10. Phased plan

- **Phase 1 — gripper_server.** Wrap `PikaSerialGripperBackend` with the UDP
  command listener + feedback publisher. Point policy/UMI at it (or, interim, have
  it accept the old inputs). Removes serial contention immediately; produces a
  gripper state source.
- **Phase 2 — server command routing.** Parse `left/right.gripper`, arbitrate,
  forward to gripper_server (4.1–4.2). Unified command plane.
- **Phase 3 — feedback + viz.** Cache feedback, stamp into state JSON (4.3–4.4),
  add `ArmSnapshot.gripper`, ship **B1** markers + readout. Then **B2** using §9.

Each phase is independently testable; viz can come online at the end of Phase 1
(direct gripper_server feed) or Phase 3 (unified state JSON).

## 11. Ports / config (proposed, TBD)
- `UDP 50410` server → gripper_server command (4.2).
- `UDP 50420` gripper_server → server feedback (4.3).
- Retire `UDP 50382` (umi bridge) — folded into the command packet.
- Config: server `gripper:` block (enable, endpoints, rate_hz, stale_timeout_ms,
  on_stale: hold|open|close); gripper_server `ports:` (per-arm serial), min/max_rad,
  deadband, home_on_connect.

## 12. Test plan
- C++: command parse of `left/right.gripper`; arbitration (non-lease source
  dropped); feedback caching + state JSON stamping; stale flag.
- Python: gripper_server protocol round-trip (mock backend, no serial);
  `ArmSnapshot.gripper` parse; GUI viz update (markers track percent; grey on stale).
- Hardware-free: full path with a mock gripper backend (no real serial), mirroring
  the existing mock-only posture for worker I/O.

## 13. Open decisions
- Phase scope to start (Phase 1 recommended).
- B1 first vs straight to B2 articulated fingers.
- Stale policy default (hold recommended).
- Exact ports / schema version bump vs additive field.
