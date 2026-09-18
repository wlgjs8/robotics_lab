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

J3 (elbow) is **not** `+/-360`: it is bounded by whichever of the arm's two elbow
limits binds first, and the tracked stack configs clamp to exactly that. The IK URDF
model carries the same bound, so the raw safety clamp and the kinematic model agree
and there is no band where one permits what the other refuses. The other joints stay
at the broad `+/-360` raw controller range.

**The bound is per-robot. It is a property of the arm, not a tuning knob:**

| arm | J3 catalog (mechanical) | supported bound | in service |
| --- | --- | --- | --- |
| RB5-850E | `+/-165 deg` | `+/-160 deg` (`+/-2.792527 rad`) — controller self-collision | since 2026-09-02 |
| RB3-730E | `+/-150 deg` | `+/-150 deg` (`+/-2.618 rad`) — mechanical | until 2026-09-02 |

Source for the mechanical column: Rainbow RB Series catalog, p6 (RB3-730) and p7
(RB5-850), which lists `J3 : +/- 165 deg` for every RB5/RB10/RB16 and `+/-150` for
RB3-730/RB20-1900. `config.cpp` carries the **supported** bound in `kKnownArmRanges`
and warns at startup if `kinematics.urdf` is an arm it does not know, rather than
skipping the check.

### Why the RB5-850E supported bound is 160, not the catalog 165

The mechanics allow 165. The **controller does not**: its own self-collision detector
raises `op_stat_self_collision` (data-structure item 35, lower 2 bits; our backend maps
it to code 1005 `rbpodo_self_collision`) while the elbow is still inside the catalog
range, and that latches the run.

Measured 2026-09-16 on the left arm, two separate policy runs:

| log | time | J3 at trip | J4 / J5 / J6 | our mesh clearance, link2 vs link4 |
| --- | --- | --- | --- | --- |
| `servo_log_20260916_171039.csv` | 576.32 s | `161.10 deg` | `-37.3 / -82.2 / 32.4` | 13.6 mm |
| `servo_log_20260916_172135.csv` | 122.65 s | `161.55 deg` | `-28.3 / -99.4 / 47.6` | 11.8 mm |

The two poses differ by 9-17 deg at every wrist joint but trip at the same J3, so the
judgement is upper arm (`link2`) against forearm (`link4`) — a function of the elbow
angle alone — closing at roughly 3.6 mm per degree. No joint-limit code appeared
(`M109` EMS "Joint angle limit over", `A20`-`A31` per-joint boundary): this is the
collision detector, not the range check, which is why the catalog value is both correct
and unreachable in practice. The gripper was 250-290 mm from anything, so the
Tool/TCP "tool area" self-collision zone is not involved either.

`160` leaves ~1.1 deg to the measured trip, and `joint_limit_barrier` brakes into it
from `148` (`d_slow_deg` 12). Cost, measured on the same six runs and excluding the
post-fault hold: `|J3| > 160` for 0.11 % and 1.07 % of ticks in the two runs that
reached it, `> 155` for 1.2 % and 3.6 %, and the other four runs never passed 148.

**This is not the retired `+/-160` site margin.** That one (RB3, mid-2026) was WIDER
than its `+/-150` URDF and created the pinning band described below. This one is
NARROWER than the catalog and is applied to *every* layer at once — both generated
URDFs, `safety.q_min_deg`/`q_max_deg`, `joint_limit_barrier`, `kKnownArmRanges` — so
no layer permits what another refuses. A safety clamp narrowed *alone* would be the
mirror trap: IK would keep solving elbow angles past 160 that the clamp then cuts, so
the dispatched joints would stop matching the planned ones and the preview executor's
dispatch acceptance would fault.

If a 1005 is ever seen at `|J3| < 160`, the fix is not to narrow further first: raise
`safety.self_collision.mesh.intra_arm.d_hard_m` (5 mm today, our monitor read 12-14 mm
at both trips) so our own barrier stops the fold before the box judges it.

**Why the clamp and the URDF must agree, whichever arm is fitted.** `JointTarget` /
`InitMotion` PTP bypass IK and pass only this clamp, so any band where the clamp is
WIDER than the URDF is a band the elbow can be parked in and never commanded out of:
every subsequent Cartesian tick is refused by IK. This was measured, on the RB3 at
its `+/-150`: for part of 2026-07 the arrays carried a `+/-160` site margin, and on
real pi0.5 rollouts the left elbow sat pinned at exactly `150.000 deg` for 7 s
(`servo_log_20260826_063359.csv`) and 4.5 s (`servo_log_20260826_075010.csv`).
Narrowing back to the catalog value on 2026-08-26 removed the band and silenced the
startup `safety q_min/q_max differs from ... URDF IK limit` WARN.

A pose that needs `|J3|` past the fitted arm's bound is genuinely unreachable.
Widening either limit to "fix" it only defers the failure to the controller as a
silent branch jump — do not do it. Move the commanded pose instead.

Tracked rbpodo stack configs (`rb_servo_server/config/stack_real.yaml` and
`rb_servo_server/config/stack_sim.yaml`) must carry these arrays explicitly, and
`safety.joint_limit_barrier` pins the same bound so a future widening of the clamp
cannot silently move the braking point. Do not widen them through a site profile or
an acceptance-stage edit. Changes to other joint limits still require confirmation of
controller soft limits and the physical setup.

The checked-out `controller-manager` submodule carries its own RB3 model value of
`+/-155 deg`. That value is not an accepted J3 source for `robotics_lab` and must not
be copied into its stack, safety, IK, tests, or runbooks.

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

The `rb5_850e.urdf` model has limits of `+/-360 deg` (`+/-6.2832 rad`) for J1, J2,
J4, J5, J6 and `+/-160 deg` (`+/-2.792527 rad`) for `elbow_joint` (J3). The J3 value
is the RB5-850E's supported elbow range (the mechanical catalog range is `+/-165`,
but the controller's self-collision detector trips at `~161` — see above), so the IK
model limit, the raw safety limit (`q_min_deg`/`q_max_deg[2]` above), the barrier and
the hardware agree. The retired `rb3_730e.urdf` carries `+/-150 deg`
(`+/-2.618 rad`) for the same joint and remains correct for that arm.

Both RB5 URDFs — the single-arm IK model and the unified collision model — are
generated by `rb_servo_server/tools/make_rb5_850e_urdfs.py`, which sets the bound
from one constant. Upstream's `dual_rb5_850e_ver1.urdf` (the stand revision this cell
has, since 2026-09-06) ships `+/-179.9 deg`, which is neither the catalog nor the
supported value; consuming it unchanged would recreate exactly the trap described
above. `ver2` ships the same
`+/-179.9`, so the elbow correction is independent of which stand is used.

**History (do not repeat):** J3 was once widened to `+/-360` in the URDF to stop
IK from returning `reason=joint_limit` (which held the arm) on poses that "looked"
reachable. Because the elbow physically cannot exceed its catalog range, widening did
not make those poses reachable — it only let IK pick an elbow branch the controller
then rejected/clamped, surfacing as a TCP branch-flip / lurch. Such a pose is
genuinely unreachable; an honest IK `joint_limit` hold (arm stops, WARN names J3) is
the correct outcome. Keep J3 at the physical range in every layer (URDF, safety
`q_min/q_max`, joint-limit barrier, collision model, `config.cpp` `kKnownArmRanges`);
do not widen it to mask an unreachable target.

For joints whose raw safety range is intentionally broader than the URDF IK limit
(none by default now — J3 is aligned), the config loader emits a warning that IK
may still be model-limited. This warning is intentional. It prevents silent
confusion between:

- raw controller safety limits used for state, command, tracking, and logging
- URDF/Pinocchio model limits used by IK

Do not treat the warning as physical acceptance. Real motion still requires the
normal real robot gates and separate acceptance evidence.
