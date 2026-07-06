# Joint Range Policy

This document defines the supported joint-angle range policy for the rbpodo-only
500 Hz control stack.

## Raw Representation

`rbpodo` state and command values are raw controller joint angles in degrees.
The server must preserve raw values for:

- `q_actual_deg`
- `q_target_deg`
- `q_ref_deg`
- `q_sent_deg`
- servo logs and state JSON

Control, safety filtering, tracking comparisons, and q-ref comparisons must not
normalize these values to `[-180, 180]`. A UI may add a display-only normalized
view, but raw values must remain visible and must remain the source of truth.

## Supported Rbpodo Range

The current controller soft-limit configuration for the supported real rbpodo
scope is represented as explicit per-joint raw limits:

```yaml
safety:
  q_min_deg: [-360, -360, -160, -360, -360, -360]
  q_max_deg: [360, 360, 160, 360, 360, 360]
```

J3 (elbow) is **not** `+/-360`: the RB3-730E elbow physical range is `+/-150 deg`
(catalog). The tracked stack configs currently use a narrow site margin of
`+/-160 deg` so JointTarget/InitMotion PTP and shortest-path goal selection stay
near the true hardware range while leaving a small operational margin around the
catalog value. The IK URDF model still uses the physical `+/-150 deg` limit, so
Cartesian IK may hold before the final raw safety clamp (see "Kinematics
Alignment" below).
The other joints stay at the broad `+/-360` raw controller range.

Tracked rbpodo stack configs (`rb_servo_server/config/stack_real.yaml` and
`rb_servo_server/config/stack_sim.yaml`) must carry these arrays explicitly.
Site-owned configs under `rb_servo_server/config/local/` may override them
only after confirming the active controller soft limits and the physical setup.

`[-180, 180]` is not a supported production rbpodo default. It may appear only
in tests or diagnostic fixtures that intentionally prove range-violation
behavior.

## Wrapping

`joint_wrap_for_startup_validation` is diagnostic/read-only only. It may publish
`q_range_wrapped` and `q_actual_normalized_for_safety_deg`, but state JSON must
still preserve raw `q_actual_deg`.

`joint_wrap_for_motion_safety` remains rejected: a target is never wrapped
*mid-trajectory* across `+/-180` or any other period, and out-of-range targets
clamp to configured raw `q_min_deg`/`q_max_deg`.

**Accepted (2026-06-21): shortest-path `JointTarget` goal selection.** The
"continuous motion-safe unwrapping" that was deferred above is now implemented and
accepted for absolute `JointTarget` PTP *goals only*
(`TrajectoryFilter::computeJointTarget` -> `shortestPathJointGoal`,
`src/control/trajectory_filter.cpp`). Because one physical orientation has up to
three valid raw representations (`theta`, `theta +/- 360`) inside the supported
`[-360, 360]` range, a bare absolute target can sit a full revolution from the
joint's current angle and make an InitMotion/PTP spin ~360 deg to an equivalent
pose (observed: `q1 = 251.8 deg`, target `-131.66 deg` -> `-383 deg` the long way).
The filter now picks, per joint, the equivalent angle `target + 360*k` that is
**in-range AND closest to the current sent target**. Properties that keep this
motion-safe (NOT the rejected mid-trajectory wrap):

- **Limit-bounded** — a candidate is used only if it lies within `q_min_deg`/
  `q_max_deg`; if no closer in-range equivalent exists the literal target is kept
  (a near-limit target still takes the only legal long way around).
- **Endpoint-equivalent** — the chosen goal is the same physical pose mod 360.
- **Goal-only / monotonic ramp** — only the goal representation is chosen; the
  downstream rate-limit / SMD ramp from the current target stays monotonic, so no
  angle is wrapped while moving.
- **No-op when range is unset** — a degenerate `q_min >= q_max` (default zero
  arrays) disables it, so configs without explicit raw limits are unaffected.

For cable-sensitive joints, `safety.joint_target_literal_axes` can opt a joint
out of the shortest-path equivalent selection. `false` keeps the default behavior
above; `true` keeps the commanded raw `JointTarget` value exactly. The current
stack configs set J6/wrist yaw to `true` so InitMotion returns to the configured
raw yaw target instead of choosing a closer `target +/- 360` equivalent that can
leave the Pika gripper cable wound.

## Kinematics Alignment

The `rb3_730e.urdf` model has limits of `+/-360 deg` (`+/-6.2832 rad`) for J1, J2,
J4, J5, J6 and `+/-150 deg` (`+/-2.618 rad`) for `elbow_joint` (J3). The J3 value
is the true RB3-730E elbow physical range (catalog: the elbow cannot reach
`+/-360`), so the IK model limit, the raw safety limit (`q_min_deg`/`q_max_deg[2]`
above), and the hardware agree on `+/-150` for J3.

**History (do not repeat):** J3 was once widened to `+/-360` in the URDF to stop
IK from returning `reason=joint_limit` (which held the arm) on poses that "looked"
reachable. Because the elbow physically cannot exceed `+/-150`, widening did not
make those poses reachable — it only let IK pick an elbow branch the controller
then rejected/clamped, surfacing as a TCP branch-flip / lurch. A pose needing
`|J3| > 150 deg` is genuinely unreachable; an honest IK `joint_limit` hold (arm
stops, WARN names J3) is the correct outcome. Keep J3 at the physical range in
every layer (URDF, safety `q_min/q_max`, collision model, `config.cpp` expected
limits); do not widen it to mask an unreachable target.

For joints whose raw safety range is intentionally broader than the URDF IK limit
(none by default now — J3 is aligned), the config loader emits a warning that IK
may still be model-limited. This warning is intentional. It prevents silent
confusion between:

- raw controller safety limits used for state, command, tracking, and logging
- URDF/Pinocchio model limits used by IK

Do not treat the warning as physical acceptance. Real motion still requires the
normal real robot gates and separate acceptance evidence.
