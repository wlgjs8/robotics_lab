# Force reference development — 2026-09-11

Baseline: `c2518c62` (contact dispatch clamp removal). The earlier staged changes
were committed by the user during this task; they were preserved. No new commit
was made by the development agent.

## Applied runtime correction

`DualArmServoLoop::stepFtPipeline` now computes the flange transform from the
valid finite `q_actual_deg` accompanying the sensor sample. Previously it used
`*_prev_sent_q_deg`, which can lead the physical sensor due to transport and
servo delay. Invalid actual joints no longer allow compensation from a command.
Sensor basis, tool mass/COM, TCP moment reference, tare lifecycle and force-law
settings are unchanged. The runtime still has the existing gate/overlay law.

`force_wrench_measured_pose` exercises the production adapter using in-memory
backends and both arms' tracked calibrated sensor parameters. It checks gravity
rejection, a known external force in stand coordinates, the TCP moment, differing
command/measured wrist poses, invalid actual joints, and nonfinite actual joints.
It sends no command to a real or controller-simulation endpoint.

This is a frame correctness fix, not evidence that the reported vibration has
been solved. In recorded active portions of runs 15:46:38, 16:05:15 and 16:12:19,
the gravity-frame discrepancy estimated from logged TCP rotations is at most
about 0.073 N. The logged command can differ by one tick from the pipeline's prior
sent input. The small discrepancy does not explain the reported large oscillation.

## Requirement under development

- Preserve policy intent, free-space travel, tangential motion and force response.
- No separate accumulating force displacement, workspace-radius fence or
  task-specific press-axis gate in the new reference law. *(The live law's own
  declared press axis was removed on 2026-09-15; the live gate now cuts along the
  measured force vector.)*
- 10 N is the current test threshold; an excursion may be transient but sustained
  contact must recover below it. The prototype uses an explicit 8 N recovery
  target. This is a development margin, not a hardware-qualified parameter.
- 20 N is a possible later operating value, not a configured limit or an approved
  impact ceiling.
- Holding the gripper/tool should not accumulate release motion. Coupled dual-arm
  contact/handover requires further validation.

## Offline candidate, not a runtime feature

`rb_servo::experimental::ForceReference` owns one generated translation p/v/a.
It turns policy position/velocity intent into a velocity reference, predicts
local force from measured contact and in-flight displacement, projects the
velocity onto a force-recovery half-space, and generates a jerk-bounded command
with Ruckig. The half-space is a reference condition; the downstream OTG and
actuator delay mean it is **not an instantaneous force guarantee**.

The original position-pursuit `step()` is now retained as a comparison baseline:
its absolute error can grow behind a moving, obstructed target. Absence of a
separate offset integrator does not establish release anti-windup. The later
`stepVelocity()` interface below removes that positional catch-up term.

The candidate expects gravity-, bias- and inertia-corrected external force.
The production F/T pipeline supplies only gravity/bias compensation. Its output
cannot be passed directly to this candidate. The experimental three-position
acceleration estimator is a finite-difference baseline with acquisition/freshness
handling, not a deployable force observer. It is sensitive to pose noise and
assumes inertial acceleration is estimated at the correct COM and time.

Explicit synthetic profile: M=4 kg, D=500 N s/m, prediction stiffness=44400 N/m,
noise allowance=3 N, recovery=8 N, test threshold=10 N, speed=0.6 m/s,
per-axis acceleration=5 m/s² and jerk=2000 m/s³. The prediction stiffness is a
local approximation, not a certified bound on environments. Runtime configs
do not contain or enable these values.

Build only through `RB_SERVO_BUILD_FORCE_EXPERIMENTS=ON` with `BUILD_TESTING=ON`.
The library is linked only to the experiment executable, never `rb_servo_core`
or `rb_servo_server`. The real build keeps the option OFF.

```sh
cmake -S rb_servo_server -B rb_servo_server/build/preview_live \
  -DRB_SERVO_ENABLE_RBPODO=OFF -DRB_SERVO_ENABLE_PREVIEW_EXECUTION=ON \
  -DRB_SERVO_BUILD_FORCE_EXPERIMENTS=ON
cmake --build rb_servo_server/build/preview_live -j2
ctest --test-dir rb_servo_server/build/preview_live --output-on-failure -j2
rb_servo_server/build/preview_live/test_force_reference --audit-csv /tmp/force-reference-audit.csv
```

The audit command's exit status reports whether it produced the audit. Inspect
each row's `valid`, `sustained_pass`, and `ripple_pass` fields; exit zero does not
mean qualification passed.

## Results and model limits

The ideal regression passes: free motion, 36 single-plane contacts (1–44.4 kN/m,
nominal command FIFO delay 6/18/26 ms, approach 30/100/150 mm/s), four contact
directions with tangential motion, hold/release duration independence, invalid
inputs, and fresh/nonuniform/duplicate acceleration samples. Contact force in
the last second is below 10 N, with worst peak-to-peak 0.0425 N. The worst initial
force peak is **294.5 N in the toy model**, so this does not qualify physical
impact performance. The FIFO is not a model of qsync's full inner servo dynamics;
plant application occurs before each new controller update.

Robustness audit: 270 cases, with 1 N at 50 Hz force ripple, oblique contact,
80/100/120% compensation mass and position perturbations of 0/1/5 micrometers at
73/41/113 Hz. Only **66/270** meet both declared sustained-force and ripple
criteria. Without position perturbation and with exact compensation, all 24
contact cases stay below 10 N in their final second; six exceed the stricter
0.2 N peak-to-peak ripple criterion. With 1 or 5 micrometers of position noise,
all 144 contact cases fail the sustained criterion. Worst synthetic peak/tail
force is 1557.3 N. These are rejected-model results, not robot measurements.

No physical contact/grasp noise model is established by these perturbations.
They are deterministic sensitivity tests. Three-point differentiation amplifies
noise; residual inertial components tilt the force normal, and the fixed
stiffness extrapolation amplifies that error. The first predictor also activated
an in-flight correction abruptly at the noise boundary. The follow-up below
fixes that discontinuity, but the candidate still fails qualification. Ideal
scalar contact does not establish general stability.

The existing runtime gate remains at peak=12 N/rest=10 N/open_tau=0.40 s and the
qsync target fill remains 2. Thus this patch alone does **not** implement the new
under-10-N requirement.

## Offline follow-up: sampling and force observability

`rb_servo_server/tools/analyze_force_observability.py` now provides a reproducible
CSV audit with no robot/backend dependency. It selects the longest contiguous
preview-active interval for each arm, never joining rollouts or missing ticks.
It reports packet repeats separately from per-joint value changes, timing and
spectral resolution, readback/reference alignment, and the age/noise gain of
local quadratic acceleration fits. Fit outputs are attributed to their window
midpoints; they must not be subtracted from current F/T feedback.

The optional `--calibration-config` reconstructs COM as
`p_TCP + R_stand_flange * (tool_com - tool_xyz)`, and rotates the pre-deadzone
flange-aligned force into stand axes. Both offsets originate at the SRO, so
sensor_offset cancels. This supplied current calibration is an explicit
assumption for historical logs, not a recovered run snapshot. The logged mass
is cross-checked; the mass is never fitted from the oscillating contact data.

Audited runs: 14:54:03, 15:45:14, 15:46:38, 16:05:15 and 16:12:19. The 15:45:14
run has no preview-active interval and is reported unavailable for this analysis.
The other longest intervals are only 0.36–1.308 seconds, insufficient for a
validated plant transfer function. Observed sent-to-readback alignment is
16–18 ms; this includes effects beyond queue occupancy and does not identify
the queue's causal delay.

At 16:12:19 each arm has a contiguous 0.580-second interval. Joint values change
on about 42–48% of fresh packet transitions, frequently as complementary groups
of three joints. Partial joint changes occur on 74.8% / 89.7% of transitions
(left/right). This establishes a staggered value pattern, not a firmware
acquisition timestamp for each joint: stationary/quantized values remain
indistinguishable from held samples.

The differentiated actual readback has 180–251 Hz vector RMS of 35.4 / 34.7 deg/s,
versus only 0.076 / 0.062 deg/s in the differentiated sent reference. A synthetic
constant-velocity fixture with alternating updates reproduces this high-frequency
artifact. Separately, the 10–12 Hz region is present in sent references and F/T
as well as readback. Sampling artifacts do not explain away that oscillation.

For the same right-arm interval, the P99 diagnostic `mass * |a_TCP|` falls from
482.7 N with 3 samples to 27.1 / 18.3 / 11.4 N with 9 / 17 / 33 samples. The
estimates are then about 2 / 8 / 16 / 32 ms old. These are neither measured contact
forces nor validated COM inertia. Longer smoothing does not provide a free
force-response improvement. With calibrated COM reconstruction and a 17-sample
fit, fixed-mass subtraction reduces the right-arm force fluctuation RMS from
20.47 to 15.31 N at matching logged times; substantial residual remains. Closed-
loop correlation cannot determine which residual is contact or modeling error.

The candidate now weights stiffness extrapolation by
`r² / (r² + (recovery_force - noise_force)²)`, where
`r = max(0, |F| - noise_force)`. Thus unresolved direction cannot create a finite
prediction jump when force crosses the noise allowance. This changes contact
model confidence, not policy motion authority. Regression coverage includes
zero/nonzero noise allowances, several normals and epsilon-size crossings with
in-flight motion. The ideal regression remains passing. The original 270-case
audit improves from 66 to **73 passing cases**, which is still a rejection.

`test_force_reference --staggered-audit-csv PATH` adds a synthetic two-tick,
staggered-coordinate pose sensor while maintaining fresh packet timestamps.
It is a Cartesian sensitivity model, not identified RB5 encoder/FK dynamics.
Both the original and continuous predictor pass **0/30** combined sustained/
ripple criteria. This is now a reproducible prerequisite case for future
observer/control designs. No acceleration fit or new reference controller has
been installed into the live force path.

The production CSV logger additionally records both arms' integer
`*_state_acquisition_sequence` and `*_state_robot_time_ns`. Sequence identifies
received states, not individual encoder updates. Robot time is raw and may be
unavailable/unreliable under the existing backend contract. Existing actual-
joint derivative columns remain host-loop finite differences, not qualified
physical acceleration/jerk measurements. Column parity and integer preservation
above 2^53 are regression-tested.

```sh
.venv/bin/python rb_servo_server/tools/analyze_force_observability.py \
  logs/servo_log_20260911_161219.csv \
  --calibration-config rb_servo_server/config/stack_real.yaml \
  --output /tmp/force-observability.json --trace-dir /tmp/force-observability-traces
rb_servo_server/build/preview_live/test_force_reference --audit-csv /tmp/force-reference.csv
rb_servo_server/build/preview_live/test_force_reference --staggered-audit-csv /tmp/force-staggered.csv
```

## Remaining implementation and qualification

1. Establish an external-wrench observer with measured acquisition timing, COM
   dynamics, uncertainty and realistic encoder/F/T noise. Do not promote raw
   three-point differentiation or increase force limits to mask its error.
2. Redesign prediction for unresolved/moving contact normals and uncertainty;
   retain acceleration/jerk continuity and force recovery through servo delay.
3. Integrate a single accepted execution state into preview replanning, cold
   splice, braking, dispatch acceptance, Hold, and fresh policy chunks. Reusing
   an unconstrained moving nominal plus an output correction would restore the
   reference accumulation being removed.
4. Validate queued setpoint/inner servo dynamics, stale packets, IK/geometry
   refusal, goal reversal, moving contacts, and dual-arm internal force. A net
   wrist wrench does not identify every simultaneous contact force.
5. Only then select a tracked runtime profile and run supervised physical
   acceptance: free-air inference, hand-held tool, soft contact, stiff contact,
   release, and finally dual-arm contact. No physical run was started here.

The requirement is both force recovery and stability. Passivity by itself does
not enforce a numerical force ceiling. For the relationship between sensing time,
measured pose and positional interfaces, see [Lange and Hirzinger's DLR research
page](https://www.robotic.dlr.de/41); for delayed discrete admittance dynamics,
see [De Stefano et al.](https://elib.dlr.de/134108/1/de_stefano_tro.pdf).

## 2026-09-12 — velocity intent and preview experiment

The new offline path is `VelocityPreview -> ForceReference::stepVelocity`.
It is a translation-only library experiment, not a replacement selected by
`make run`. No live force parameters, qsync fill, deployed profile or backend
were changed during this follow-up. The earlier measured-wrench frame and
acquisition logging changes remain in the working tree.

`VelocityPreview` optimizes a known finite velocity horizon. Its objective
contains velocity error, acceleration, jerk and jerk differences; it contains
no position error or accumulated displacement. Acceleration regularization was
needed: the first velocity/jerk-only objective overshot a constant input in its
own regression. The completed test reproduces 20 mm/s without that overshoot.
The independent axes share the same objective. Hard per-axis derivative limits
use continuous Bernstein certificates for each polynomial interval. These are
sufficient bounds, not an exact description of every physically feasible seed:
near a velocity limit the planner can refuse a state that could physically
brake. Such refusal is explicit, leaves the previous trajectory unchanged, and
does not authorize renewing that trajectory's lifetime. No seed is clipped.

The force reference receives each nominal velocity at the 2 ms servo rate and
owns the only generated translation p/v/a. `stepVelocity` does not feed an
unreachable moving target's position error back into velocity. It retains the
existing experimental radial noise allowance, continuous local-force prediction,
minimum-change recovery projection, implicit admittance and Ruckig output.
There is no press axis, scalar force gate, contact latch, arming timer, separate
force offset, or workspace-radius constraint in this path. Tangential intent is
preserved by the projection; the overall speed budget can still scale a vector.
An absolute goal can be represented by an explicitly speed-bounded pursuit
task, as tested, without integrating obstructed displacement.

The force projection is still a **reference condition before the OTG**, not a
physical force certificate. In-flight motion, imperfect force estimates and
inner servo dynamics remain relevant. The velocity interface changes the
anti-windup mechanism; it does not qualify the older contact predictor. The
ideal force input contract and its missing production inertia observer still
apply. The sensitivity tests deliberately include unremoved tool inertia to
test a limitation of the available production signal.

Nominal preview velocity is independent of force and has no position to fall
behind. The generated force state is not a controller acknowledgement: live
integration would still need dispatch acceptance, epoch/lease/freshness handling,
stopping, IK/geometry refusal, and asynchronous worker ownership. Rotation,
torque and gripper execution are not implemented by the new translation module.
The replay's interpretation of each SE(3) increment as interval velocity is
explicitly **not equivalent** to the current position follower's progress,
fresh-chunk anchor or gripper cursor.

All experiment parameters are supplied explicitly. The tested velocity profile
uses M=10 kg, D=500 N s/m, local prediction stiffness=44400 N/m, noise=3 N,
recovery=8 N, test threshold=10 N, speed=0.6 m/s, per-axis acceleration=5 m/s²
and jerk=2000 m/s³. These are sensitivity-test values, not measured robot
parameters or runtime defaults. Preview uses 24 intervals of 10 ms, a velocity
tracking scale of 0.1 m/s, normalized acceleration/jerk/jerk-difference weights
0.1/0.02/0.01, and explicit solver precision/budget. The 4 kg comparison and
10/20/40/80 kg damping study are retained rather than silently discarded.

### Verification and rejected assumptions

Native regressions cover invalid data/configurations, transactional plan refusal,
expired sampling, continuous velocity/acceleration splices, interval velocity
extrema, known-future reversals, coordinate transforms, tangential motion, free
travel, absolute-goal pursuit and obstruction durations of 3/9/20 seconds.
`sampleLastStep()` exposes only the most recent force-generated 2 ms interval
for read-only inspection. Its 32 subdivisions catch acceleration ramps that
endpoint telemetry misses: the zero-force manual replay reports zero endpoint
acceleration throughout, while sub-tick samples reach 1.5 m/s². Thus endpoint
acceleration differences are not reported as continuous jerk measurements.
These checks concern generated trajectories, not physical joint derivatives.

`replay_velocity_force_reference` links only the two experiment libraries and
JSON/Eigen/Ruckig/qpOASES dependencies, with no servo core or robot backend.
Its `robotics_lab.velocity_force_replay.v1` input declares all parameters,
stand-frame force samples, constant velocity or timestamped velocity frames,
and explicit stop intent. It only previews an already arrived frame; expired
rows have zero velocity intent. It does not inspect later frames early.
Changing the last recorded model frame leaves all 441 preceding output rows
identical. Nonmonotonic arrivals/sequences are refused. Complete force/zero-force
ablation pairs produce identical nominal preview velocity.

Five replay inputs cover the original 13:45 manual force excerpt and seven
16:12 model frames, plus zero-force ablations and a 50 ms synthetic release.
Every selected source force number was rechecked against the original CSV.
Force is rotated with historical actual TCP orientation and the current
identity tool rotation assumption; future velocity increments use Pinocchio
`exp6` and historical command orientation at frame arrival. This is fixed-force
playback with ideal applied translation, not physical contact feedback, old
follower equivalence, or a full production servo replay.

For a constant 20 mm/s manual intent, the new candidate's release maximum is
20.00 mm/s for both instantaneous and 50 ms force removal. The earlier full
production-path replay gave 135.85/137.54 mm/s for those two cases. Different
execution paths are being compared; this is a reproducible anti-windup result,
not evidence that physical vibration is solved. The new model playback still
has 5–15 Hz finite-difference command-velocity RMS of 34.87 mm/s with recorded
force versus 21.13 mm/s with zero force. This includes commanded motion and
forced response, and does not identify autonomous oscillation or a transfer
function. The earlier production-path values must not be substituted as this
new velocity interpretation's matched baseline.

Six audits execute 6,498 scenario runs, including deliberate repeated conditions
for controlled comparisons. A pass below means **last-second** force <10 N and
force peak-to-peak <0.2 N for contact; free-space checks use speed ripple/error.
It is not hardware acceptance or a limit on earlier force excursions.

| Audit | Last-second criteria passed | Runs |
| --- | ---: | ---: |
| Velocity force: 4 kg and 10 kg comparison | 135 | 180 |
| Combined velocity preview + force, 10 kg | 90 | 90 |
| Force gain, vector noise, independent force delay and stiffness prediction | 1,488 | 2,430 |
| Same additional delay applied to force and observed position | 1,497 | 2,430 |
| Equal M/D damping study | 622 | 1,080 |
| Coupled two-arm normal-spring model | 205 | 288 |

The 90-case combined experiment settles at at most 8.107 N with at most
0.101 N peak-to-peak, yet its initial synthetic peak reaches **254.81 N** and
its longest continuous excursion above 10 N is **0.882 s**. It is not an accepted
impact envelope. No acceptable excursion duration or peak was invented from
the user's allowance for brief overshoot. In the stress audits, force/pose
timestamp pairing reduces some failures but does not eliminate them. Larger
damping alone also fails the combined criteria. Two cooperating controllers
remain unstable in some coupled cases; this is not handover qualification.

The toy plant applies delayed commanded positions before each next update and
uses discrete differences for inertial force. Listed command FIFO times are
6/18/26 ms; the causal update ordering adds one 2 ms sample. Additional force
delay is 0/4/8 ms, either unmatched or paired with pose. Pose coordinates can
update in alternating groups, with 0/1/5 micrometer perturbations. These are
sensitivity models, not an identified qsync queue, sensor acquisition process,
inner servo or environment. Reported hundreds of newtons are rejected synthetic
results, not measured robot forces. 20 N has not been selected for operation.

Reproduction after enabling the existing offline experiment build option:

```sh
cmake --build rb_servo_server/build/preview_live -j2
ctest --test-dir rb_servo_server/build/preview_live --output-on-failure -j2
rb_servo_server/build/preview_live/test_force_velocity_preview --audit-csv /tmp/velocity-force.csv
rb_servo_server/build/preview_live/test_force_velocity_preview --combined-audit-csv /tmp/velocity-combined.csv
rb_servo_server/build/preview_live/test_force_velocity_preview --stress-audit-csv /tmp/velocity-stress.csv
rb_servo_server/build/preview_live/test_force_velocity_preview --paired-sensor-audit-csv /tmp/velocity-paired.csv
rb_servo_server/build/preview_live/test_force_velocity_preview --damping-audit-csv /tmp/velocity-damping.csv
rb_servo_server/build/preview_live/test_force_velocity_preview --coupled-audit-csv /tmp/velocity-coupled.csv
rb_servo_server/build/preview_live/replay_velocity_force_reference INPUT.json OUTPUT.csv
```

Audit process exit zero means its table was written; inspect `valid`,
`tail_below_10`, `tail_ripple_pass`, peak and excursion duration. Solver wall
times are offline measurements, not servo scheduling acceptance. The build
does not select any new real profile. A source/progress adapter, qualified force
estimate and delay model, bounded force transients, full lifecycle/dispatch
integration, and multi-contact/rotation validation remain prerequisites to
promoting this candidate. Live safety layers and the existing force/gate path
remain responsible for current operation; this experiment does not repair
`make run` by itself.
