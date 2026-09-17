# Force control and delta preview: single target

Status: implementation and offline validation revision, 2026-09-15. **Physical acceptance is pending.**
This supersedes the old rest/peak force pair, 0.5 N direction switch and executor time dilation.
Measured sensor/tool calibration, lease, stale-state handling, coverage, tare, deviation fences,
workspace/collision/joint constraints, dispatch acceptance and emergency-stop authority remain in force.
The earlier staircase and the contact-free 17:28 vibration are distinct incidents.

## Law and measurable limits

In stand coordinates, with compensated pre-deadzone force `F_law`:

```
m dv/dt + b v = max(|F_law| - F_target, 0) F_law / |F_law|
d = integral(v dt), k = 0; rotational deviation = 0
```

The zero vector has zero drive. This is isotropic translation: expressing the same physical force
in another orthonormal frame gives the same physical translation after transformation. It is not
6D torque compliance. Sensor axes remain the measured electrical basis (`det=-1`); orientation and
gravity use acquired joints, and the existing delayed commanded-COM inertia compensation remains.

The single target is **20 N**, virtual mass **20 kg**, damping **500 N s/m** (`m/b=40 ms`).
At 22 N, steady yield is 4 mm/s; at 30 N it is 20 mm/s. The existing norm velocity/acceleration
caps apply. Below target the drive is zero and existing velocity decays; it does not disappear in
one tick. Hold does not seek a surface to restore exactly 20 N. It keeps the yielded position.
A sustained head-on nominal press approaches target in the qualified software contact model.

A single F/T sensor measures the resultant wrench. Opposing hand/floor forces below the sensor
can cancel, so individual contacts are not observable separately. Friction, stiff/delayed contact,
actuator dynamics and simultaneous contacts can exceed the target. Neither this equation nor
passing the tests proves an impact ceiling or absence of all physical oscillations. These remain
physical acceptance requirements. Rotation stays rigid, including for pure torque.

## Configuration migration

Both tracked stacks declare:

```yaml
force_control:
  enable: true
  target_force_n: 20.0
  law:
    translation: {m: 20.0, damping_n_s_m: 500.0}
    rotation: rigid
  force_gate:
    enable: true
    contact_noise_low_n: 2.0
    contact_noise_full_n: 3.0
    close_tau_s: 0.10
    open_tau_s: 0.40
  wrench_filter_hz: 3.0
  law_filter_hz: 25.0
```

The law, target and confidence keys are required when force control is enabled. Values must be
finite and positive, `noise_low < noise_full < target`, gate enabled, and `m >= 2*b*0.002` under
the existing conservative Euler contract. Rotation must be `rigid`. Old `peak_force_n`,
`rest_force_n`, `peak_vel_mm_s` and mixed schemas fail loading with a migration reason.
Old per-axis/spring/latch keys stay rejected. This is a deliberate schema change: custom configs
must migrate; do not silently infer damping from the old pair.

## Confidence and force authority

Let `S(x)=clamp(x,0,1)^2*(3-2*clamp(x,0,1))` and let `F_gate` be the existing **3 Hz vector**
filter of compensated force. The law independently uses its **25 Hz vector** filter.

```
c = S((|F_gate|-2 N)/(3 N-2 N))
g_desired = 1-S(|F_gate|/20 N)
g_physical += min(dt/tau,1)*(g_desired-g_physical)
g_effective = 1-c*(1-g_physical)
```

`tau` is 0.10 s when closing, 0.40 s when opening. Below 2 N no new confidence or normal is
formed; between 2 and 3 N they rise smoothly; at 3 N confidence is full. The band is
not subtracted from physical force and does not turn 20 N into 22–23 N. A unit direction becoming
available must never be used as a binary stop. Source demand is logged but does not control the
new curve; zero nominal demand can coexist with a closed gate and a yielding arm.

**Contact memory (2026-09-16).** A contact that was seen is remembered: its normal and the PEAK
confidence it reached persist after `|F_gate|` drops out of the band, and the authority is
`1-max(c, c_peak)*(1-g_physical)` until `g_physical` has re-opened past 0.98 on the 0.40 s
time constant (~1.5 s from a closed gate), when the memory and the normal are dropped. A
contact that only grazed the band remembers only its small peak, so the band edge stays
continuous. Without it the loss of force after an impact opened the closing authority to 1.0
on that tick and the executor re-dived into the surface it had just bounced off (14:38 run,
left 317.87–318.07 s: `g` 0.41 → 1.00 in 30 ms, re-approach 100–139 mm/s, second impact 55 N).

Evidence for the initial band: the right-arm post-stop 3 Hz residual in
`servo_log_20260915_172801.csv` reached 0.641 N, crossing 0.5 N 718 times in about 35.7 s;
no 2 or 3 N crossing occurred there. During contact-free motion the same gate norm reached
6.42 N. Thus the band improves static residual rejection but cannot distinguish all dynamic
compensation error from contact. The stored ~33.44 N right bias is already subtracted and must
not be reused as an activation threshold. Collect both static and moving residuals after each tare.

Tare now records committed sample count, force standard deviation per axis in the axis-mapped,
flange-aligned sensor-origin basis used by the stored bias, and the norm
of those standard deviations, separately from the estimated bias. These describe the **unfiltered
tare residual samples**, not the 3 Hz post-tare distribution. Config-loaded bias has no measured
noise statistics. No online bias adaptation or automatic contact tare was added.

## Preview ownership and continuity

1. The raw chunk source receives geometry authority but no ForceGate attenuation while preview
   owns force execution. The worker solves a physically feasible nominal candidate from the
   current source and the same splice state, then solves the constrained candidate. Since
   2026-09-16 the candidate's splice is **de-braked**: any acceleration component along the
   normal that points away from the contact is dropped from its initial state (the constrained
   plan still starts from the true splice). The candidate measures demand; the braking
   acceleration a constrained plan carries after an impact is contact-induced, and from
   (+8 mm/s, -3.5 m/s^2) a free plan retreats for 100 ms before it turns (offline -48 mm/s) even
   with its source 8 mm deeper, which collapsed the bound to zero exactly when it was needed.
2. The constraint points into contact (`n=-F_gate/|F_gate|`) and bounds `n dot v` on BOTH
   sides. The ceiling is `g_effective` times a certified upper envelope of the nominal
   candidate's positive closing velocity; the floor (2026-09-16) is `min(g*min, min)` of the
   same Bernstein controls minus `tracker.contact_retreat_slack_m_s` (5 mm/s): closing is
   scaled by the authority, retreat keeps full authority, and the plan may not back out of a
   contact on the objective's own account. Quadratic-velocity Bernstein controls on servo
   subintervals bound the full interval; adjacent extrema form continuous piecewise-linear
   envelopes, and one two-sided qpOASES row per control carries both. The scaled candidate
   `g*v_nominal` lies inside the tube at every sample, and the slew (3.) lands on exactly its
   state, so the constrained problem is feasible by construction; the slack exists because an
   exact tube at `g ~ 0` is micrometres per second wide and thrashed the working set. At
   `g -> 1`, the result approaches the same nominal optimum. At `g=0` the steady closing
   bound is zero. Measured need: on the 14:38 run (left 317.69–317.80 s) the plan left a 46 N
   contact at -86 mm/s with the ceiling at 0 and the source 8 mm deeper, lost the force, and
   re-dived 16 mm at 130 mm/s into a second 55 N impact; offline the same state gave 4.5 mm
   of retreat with the one-sided authority and 0.16 mm with the tube.
3. A newly tightened bound cannot instantaneously stop existing momentum. Since 2026-09-15
   night the authority is **slewed** from the dispatched closing state (velocity and
   acceleration along the normal) to the envelope along the smoothest two-planning-interval
   jerk profile within `tracker.contact_slew_jerk_m_s3` (400 m/s^3; longer slews for larger
   cuts, escalating to the physical jerk limit only if that fails), then follows the envelope.
   With the two-sided tube the landing is the scaled candidate's own (velocity, acceleration)
   at T, and during the slew the plan is held within one Bernstein margin of the profile on
   both sides. The earlier widening along the fastest brake reached zero in ~3 ms and, because the
   planning jerk is constant per 10 ms interval, demanded a cut the QP could only meet by
   overshooting into a retreat: reproduced offline on the 19:54 run's inputs, `g=0.9` alone
   turned a 10 mm/s approach into a -69 mm/s retreat, `g=0.5` into -183 mm/s, and each replan
   inherited it (the 2.2 Hz floor bounce, 141 N). With the slew both stay within a few mm/s of
   `g` times the free candidate (`test_preview_execution_worker`, contact slew case).
   Rejected/late contact solves never fall back to an unconstrained command. Both solves share
   the original splice deadline, with one servo period reserved for delivery/admission.
4. The follower's actual forecast is trusted for `tracker.trusted_future_sec` (0.10 s = three
   rows of the four-row execute window; `0` restores the 2026-09-15 evening rule of the selected
   segment only). Beyond it, reference construction uses that prefix endpoint's constant-velocity
   continuation. The one-segment rule extrapolated a single segment's end velocity over ~220 ms;
   chunk rows jitter 1-2 mm per 33 ms, so the executor swung +-60..110 mm/s around a +-20 mm/s
   source and attenuated slow segments to 0.67-0.69 of the source path (22:33 run). Rows beyond
   the trusted window still cannot pull the current command. Full forecast stays diagnostic
   for phase tracking. `execute_steps` is a publisher cadence, not proof later rows are committed.
5. **All accepted trajectories, derivatives, splices and brakes use wall seconds.** Plan lead
   reduces the future reference rate inside the QP (`reference_rate_gate`); it does not scale
   output sample time. Since 2026-09-15 night the leash reads the **signed** lead
   `plan_lead_along_m` (the plan-minus-source offset projected onto the source's direction of
   travel, 0 when the source is below the cursor velocity floor); the unsigned `plan_lead_m`
   stays as diagnostics. The norm had leashed a plan that was BEHIND its source (18:25 run,
   241-243 s: every leash-limited tick was behind, cursor backlog at its 100 ms cap, three
   recovery brakes). This replaces the draft's variable-clock derivative approach and removes
   its `g_dot`/`g_ddot` mismatch entirely. Legacy `plan_clock_gate` telemetry is always 1.
6. Source retirement is **source-based** (2026-09-16): every tick
   `(1-g)^2*max(0, n dot v_source)`, the source's own closing advance weighted by the square
   of the refused authority, plus `(1-g)^2*max(0, gap)/tracker.contact_realign_sec` for a
   source already deeper than the executor (`gap = n dot (p_source - p_executor)`, realign
   0.10 s), is retired from the raw source only; the older nominal-minus-constrained term is
   kept as a floor. At `g=1` nothing is retired. The square (evening revision) keeps the
   confidence band, where `g` sits at 0.75–0.98 on residual force, from retiring the policy's
   own travel (a linear share took 86 / 28 mm over the 25-minute 16:45 run, 4–22 µm per
   tick); a real contact reaches `g <= 0.1` within ~0.2 s and still retires almost the whole
   refused advance. The source therefore stays beside the executor at the contact instead of
   running ahead: on the 14:38 run it ran 48–53 mm ahead at `g` 0.55–0.7, the cursor leash
   slowed the plan clock, the backlog hit its 100 ms cap and the executor was stopped for a
   recovery brake five times (101.7–140.4 s), all in sustained 8–15 N contacts. Physical
   output is never position-clamped for this retirement. Force yielding remains a separate
   common-frame fold. The raw source's existing safety and recovery pacing still applies; no
   old worker result may outlive its authority/deadline.

## Source lifecycle and folds

- **Chunk preview:** force deviation books into the existing common-frame fold; only blocked
  nominal closing advance is retired separately. This avoids applying force authority twice.
- **Hold:** zero-demand pose latches from the commanded TCP after the motion generator rests.
  The force fold moves that pose within existing ROI/floor constraints.
- **Absolute teleop with pose-track SMD:** incoming absolute command deltas already integrate
  into a persistent relative goal. The force fold shifts SMD state, goal and feedforward history,
  while retaining the incoming-command cursor. Contact velocity restriction is inside integration
  with the existing acceleration cap. Back-calculation of unexecuted closing demand prevents a
  long hold from accumulating an unbounded release catch-up goal; tangent/retreat stays available.
- **Literal absolute PTP without SMD:** there is no persistent relative source. Fold is explicitly
  declined and the existing 40 mm deviation fence remains. This revision does not claim that
  arbitrary PTP or JointTarget commands acquire continuous force-following semantics.
- **Timeout / explicit Hold from preview:** retire the source and stale frames, cancel staged
  successors, and sample a finite brake seeded from the last accepted physical nominal state.
  Repeated Hold does not renew its clock. Hold begins after the brake terminal sample is actually
  accepted by dispatch, including send-at-tick-start. Fresh source input is required for restart.
  During this finite brake the same force integrator may compose a fenced deviation; Hold takes
  the fold after stopping. Fault/E-stop/invalid-state handling keeps its own stronger veto.

## Brake seed headroom and cursor rotation band

A finite brake starts from the dispatched sample, which can sit exactly on a cap. On the
22:33 run the policy asked for a wrist rotation beyond the 1.4 rad/s angular cap; the plan
rode the cap, three solves came back infeasible, the plan expired and the brake seed was
refused by 1e-7 (`brake_initial_outside_limits`), turning the expiry brake into a
ChunkFollowerFault. Since 2026-09-15 night a seed up to 25 % above a velocity or acceleration
cap is braked, certified against max(cap, what the seed already does); beyond that it is still
refused. The same run showed the preview cursor's 2-5.7 deg rotation band slowing the reference
clock to its 0.25 floor while the tracker was saturated in rotation, filling the 100 ms backlog
in 130 ms and forcing a recovery stop/restart; on the preview profile that band is now
10-30 deg (`plan_leash_start_rad`/`plan_leash_full_rad`, cursor-only there).

## Fault stop delivery

A non-emergency latch (accepted deviation, tracking error, chunk-follower fault) keeps
sending its decelerate-then-latch ramp under send policy `fault_brake` until every joint's
delivered velocity is zero, bounded by twice the declared `dq_max/ddq_max` stop time plus ten
ticks, and only then goes silent under `fault_latched`. Before 2026-09-15 night the ramp was
computed but never delivered: the latch tick already suppressed regular `servo_j` and
stopped booking `prev_sent`, so the box drained its FIFO and hard-stopped from 65 and 31 deg/s
(18:30 and 18:25 runs; 10-12 Hz ringing logged as 33.5k / 19.7k deg/s^2). Emergency,
backend, robot-state and transport latches keep their immediate suppression. Covered by
`test_preview_servo_integration` (fault brake, both send orders).

## Diagnostics and offline validation

New force fields: `target_force_n`, `contact_confidence`, `physical_gate`. Existing `gate_m_eff` and
`gate_b_eff` identify the law. Deprecated `gate_rest_force_n` / `gate_peak_force_n` both mirror
`target_force_n`; `gate_cross_speed_m_s` is zero. These are telemetry compatibility fields only.

New preview CSV fields include reference rate, nominal closing velocity, accepted contact bound
and its normal, executed closing velocity, retired source advance, nominal solve time, trusted
prefix length, source stop requests, **completed terminal dispatches** and, since 2026-09-15
night, the signed `plan_lead_along_m`. Existing accepted seed,
worker/result identity, deadline and final-safety telemetry remain available. UDP stays a compact
summary; detailed forensic fields are CSV-only to preserve packet size and safety-state precision.

`tools/analyze_force_stage.py` detects the new target telemetry and compares the one-step law
using its **own filtered input**, m/b dynamics and norm caps. It checks the physical gate recurrence
and confidence separately, excludes coasting transients from rest checks, and does not infer floor
contact from net force. Missing required inputs are SKIPPED. Legacy pair analysis needs matching
legacy parameters. A fixed recorded force trace is not a closed-loop robot simulation.

`tools/live_preview_replay.cpp` audits the accepted QP bound rather than raw follower speed, but
its historical source gates remain recorded exogenous inputs. It does not reconstruct the new
production source retirement/leash/timeout loop. Production coordinator tests use memory backends
for those paths. The original 17:28 chunk JSONL was not found, so that exact run is not replayed.

Regression coverage: diminishing `1-g` over axial/oblique directions; unselected-tail independence;
physical-time derivative/splice continuity; source timeout and accepted brake terminal in both
send orders; force frame/calibration invariants; tare bias/noise separation; 10/20 N software wall
models; long SMD hold/release; source fold/coverage/tare/init lifecycle; state/CSV contracts. The
worker's optional `--wall-clock-benchmark` uses actual steady time and the tracked preview limits;
it reports latency and deadline rejections, not a worst-case execution guarantee.

Policy chunk rows already log automatically from the publisher, including monotonic/wall time,
original model rows, projected rows and recovery metadata. Preserve the announced `chunk rows ->`
JSONL path together with server CSV/config/commit identity on the next run; no servo file I/O was added.

## Supervised physical acceptance still required

Follow existing activation, E-stop, lease, tare and coverage procedures. Start with one arm and
low nominal speed, then repeat for the other arm before bimanual/model tests.

1. Init and tare: verify bias, noise statistics and coverage. Record a static no-contact interval
   and a moving no-contact interval; compare 3 Hz residuals against the 2–3 N confidence band.
2. Push to contact from init at low speed; label whether the hand is above or below the sensor.
   Check sustained net force near the 20 N target under a press, free retreat, retained position
   after release, no fence hits or repeated oscillation-guard trips. Hold may rest below 20 N.
3. Test small nominal motion with hand hold, lateral push and diagonal push; verify task tangent
   and escape, no accumulated catch-up on release, and continuous command derivatives.
4. Run pi0.5, then controlled source timeout/Hold and fresh restart. Check accepted terminal-stop
   count, source identity, worker deadlines, raw/measured/sent motion, force spectrum and contact
   force peaks. Stop the test at vibration or safety refusal and retain the full logs.

Hardware-free checks do not authorize a hardware launch. No controller or robot was started by
this implementation work. Historical measured incidents are preserved in
[the archived contract](../archive/force_control_20260915_before_target.md).
