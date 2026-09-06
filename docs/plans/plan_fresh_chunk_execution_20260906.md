# Fresh chunk execution redesign — 2026-09-06

The implementation adds an opt-in path for the `boltv2_griponly_40k` H24,
boundary, command-anchor, fixed-step velocity-grip setup. The motivating
13:48:54 real rollout has both avoidable ready-result waiting and repeated
IK-refusal output reseeding. Python's integrated command pose is also a
different signal from the coordinator's attempted joint-target FK.

## Implemented contracts

1. **Policy scheduling:** `--chunk-activation-mode ready_event` requests the
   next chunk at row 0 and replaces the current chunk on the next policy-row
   deadline after the result is ready. It preserves the deadline grid and
   left/right/gripper source-row alignment. The configured execute window
   remains the maximum when inference is late. One pending request/result is
   retained; stale generations and expired results are rejected. Observation
   step count is captured at request creation, not worker startup.
2. **Input provenance:** `--velproprio-source servo_command` collects the
   server's actual published `tcp_command_stand` history. Both arm features
   use the frozen server tick endpoint and one policy period of body delta.
   Gripper selection is frozen with the request. Images retain the existing
   worker-time selection; image/state/gripper timestamp differences are
   explicit, and this is not an exactly synchronized camera observation.
3. **Execution:** `--tcp-target-profile flow_infer_fresh` selects the C++
   `delta_preview` path with `fresh_chunk_replan` and
   `continuous_hold_resume`. A fresh delta frame integrates from the sampled
   position, velocity and acceleration of the current plan, then replans on
   the next servo tick. It does not wait for the old segment's future endpoint.
   Rotational tangent changes and output feed-forward use Pinocchio/Eigen
   derivative/frame transforms, including at every fresh splice to avoid a
   fixed tangent accumulating through the principal-log branch cut. A
   transformed current state is passed intact to Ruckig rather than clipped
   instantaneously; its brake profile uses the unchanged v/a/j limits to
   return within those component limits. Target guards remain unchanged.
   An already-refused command holds at the
   nominal last-sent reference and resumes from rest with the current window,
   without repeatedly deactivating/reseeding the output filter during the hold.

The two C++ flags default to false. Each tracked stack's `flow_infer_fresh`
profile initially copied that stack's existing `flow_infer_smooth` settings and
changed only these flags. The subsequent 15:21 rollout analysis adds a real-only
linear velocity-FF gain of 0.8; see
[conditioner tradeoff](../reference/follower_output_conditioning.md). Scheduling, proprio and execution can be selected separately
for controlled comparisons. Existing measured/Python-command sources and the
fixed-step schedule remain available. The request-time step-counter race is
fixed in both scheduler modes.

After the operator reported both-arm vibration at 14:44 using the original
command, a same-delta replay reproduced the logged prefilter and output stage
at CSV rounding precision. The unfiltered sampled-v/a feed-forward passed
large acceleration pulses at the roughly 33 ms row transitions through the
output SMD, including transitions within a single chunk. Both real profiles
now explicitly set `output_smd.profile_feedforward: false`, retaining the
velocity LPF conditioning and all other dynamics. This restores
output smoothing at a position-lag cost (right-arm p95 about 6 mm in this
short replay); it is not a physical stability or zero-lag claim. The three
execution changes remain separately selectable. Detailed ablation evidence:
`outputs/chunk_review_20260906_144454/replay/ablation/report.md`.

The new execution flags also select sampled, wall-time physical velocity as
the input to the FF-off LPF. Reference-body angular velocity is transported
to the output body, and the stored angular LPF state follows that body's
rotation. Reset seeds use the actual reset pose body. Acceleration is still
not fed forward in this mode. With both flags false, the existing endpoint
tangent-velocity LPF behavior remains unchanged. This separates the
conditioning choice from the correctness of velocity's clock and frame.

The server advertises `chunk_execution_profiles` in state JSON. The policy
requires one enabled `flow_infer_fresh` entry with `delta_preview` and both
flags true before emitting policy commands. Missing or older server support
fails closed. Unsupported ready-event combinations (RTC, ensemble, chain
anchor, sequential inference, teacher-forced replay, explicit prefetch index)
fail before constructing the model/camera/gripper path.

## Boundaries

- `servo_command` is coordinator attempted q_sent FK, not controller ACK or
  measured TCP. The state collector uses actual UDP publication cadence,
  a bounded 2048-sample history, same-host monotonic time, per-arm reset and
  global motion epochs, no extrapolation and no source fallback. Invalid
  windows prevent model invocation. See
  [the signal/time contract](../reference/servo_command_proprio.md).
- Position/velocity/acceleration continuity applies to a feasible plan splice
  at a fixed gate. A hard IK/safety hold or an abrupt plan gate change does not
  become a C2 physical trajectory. Output conditioning, safety projection,
  worker sending and the physical plant are distinct stages.
- No velocity, acceleration, jerk, force, deviation, IK, tracking, collision,
  stale-state, lease or deadman bound is relaxed by this redesign.
- InitMotion/tare owns reset and force coverage. The existing deviation
  strip/compose invariant and the selected-arm reset behavior remain required.
- More frequent requests can change GPU latency and closed-loop model output.
  Lower ready waiting is not evidence of higher task success or reduced
  physical vibration.

## Selection for a later supervised comparison

Keep the existing H24/boundary/W4/runway4/command-anchor/crossfade0/RTC0 setup
and explicit `last_emitted_continuous`, and add or replace these variables in
the same launch command:

```bash
FLOW_INFER_CHUNK_ACTIVATION_MODE=ready_event
FLOW_INFER_TCP_TARGET_PROFILE=flow_infer_fresh
FLOW_INFER_VELPROPRIO_SOURCE=servo_command
```

These are variable values to incorporate, not a launch script. The server
must be built with this implementation and started with the tracked config
that advertises the profile. The ordinary configuration remains
`fixed_steps` / `flow_infer_smooth` / `command`. No physical run or server
restart is part of this development validation.

Compare the three changes separately before their combination, keeping
checkpoint and scene/task constant. Record actual inference latency/queue and
ready waiting, observation age/skew, source validity, emitted/source rows,
chunk receipt-to-consumption, segment duration/policy period, output reseeds,
IK hold/restart events, stage/sent/measured motion, and task-stage success.
Do not infer model quality from one uncontrolled rollout.

## Evidence

The master development report is
`outputs/chunk_execution_redesign_20260906/report.md`; component evidence is
under `outputs/fresh_chunk_redesign_20260906/`,
`outputs/chunk_continuity_impl_20260906/`, and
`outputs/chunk_review_20260906_134854/proprio/`.

Hardware-free evidence consists of production-dispatcher latency replay,
same-window signal comparison, sampled-p/v/a transition tests, recorded-delta
replay with exogenous force/IK gates, and the actual servo loop's in-memory
InitMotion/tare regression under both old and new flags. A recorded refusal
schedule plus an ideal accepted-reference surrogate is not a replay of the
physical IK/force closed loop. Simulator tests cannot certify absence of the
previous strong vibration or controller shutdown.
