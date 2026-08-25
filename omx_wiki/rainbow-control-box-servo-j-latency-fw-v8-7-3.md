---
title: "Rainbow Control Box Servo J Latency (firmware v8.7.3)"
tags: ["rainbow-control-box", "firmware-v8.7.3", "servo-j", "latency", "queue-sync", "rback", "servo-alpha", "rb3-730e"]
created: 2026-08-25T12:41:26.355Z
updated: 2026-08-25T21:34:24.000Z
sources: ["logs/servo_log_20260826_060642.csv", "logs/servo_log_20260826_050510.csv", "logs/servo_log_20260826_042818.csv", "docs/runbooks/box_latency_offline.md", "scripts/analyze_box_latency.py"]
links: ["rainbow-control-box-servo-j-latency-fw-v8-6-1.md", "flow-infer-delta-preview-controller-contract.md"]
category: reference
confidence: high
schemaVersion: 1
---

# Rainbow Control Box Servo J Latency (firmware v8.7.3)

Successor to [[rainbow-control-box-servo-j-latency-fw-v8-6-1]]. **Every number is
paired to control-box firmware v8.7.3** and must be re-measured after a firmware
change: the box-side scheduling model is the thing being characterised.

v8.7.3 replaces v8.6.1's latest-queue with a **FIFO**. That is not automatically
better — it is only better if the host regulates the queue.

## The Result That Matters

```
box delay = RBACK queue fill + 1 tick     (exactly, both arms, all joints)
```

Nothing else. No filter, once the LPF is off. So the box delay is whatever the
host lets the queue fill become, and the fill is a pure integrator:
`dfill/dt = f_send - f_box`.

| configuration | queue fill | drift | sent -> ref | end to end |
|---|---|---|---|---|
| v8.6.1 latest-queue, LPF on | n/a | — | 10.1 tk (20 ms) | **13.1 tk (26.2 ms)** |
| v8.7.3 unregulated, LPF on | 22-28, rising | +0.67 tk/s | 32.6-38.0 tk (65-76 ms) | 35-40 tk (70-80 ms) |
| v8.7.3 unregulated, LPF off | 19, rising | +0.65 tk/s | 20 tk (40 ms) | 23 tk (46 ms) |
| v8.7.3 + queue_sync(5), LPF off | **5.0, locked** | -0.11/-0.19 tk/s | **7.05 / 7.46 tk (14.1 / 14.9 ms)** | **10.00 / 10.44 tk (20.0 / 20.9 ms)** |
| **v8.7.3 CURRENT (2026-08-26)** | **5.0, 99.4 % exactly 5** | **-0.000 tk/s** | **8.17 / 8.03 tk (16.3 / 16.1 ms)** | **11.14 / 11.13 tk (22.3 / 22.3 ms)** |

**Upgrading the firmware alone made latency 3x worse.** The FIFO only pays off
with queue regulation.

`sent -> ref` is identical across all six joints of an arm (residual
0.002-0.044 deg) -- the box does not treat joints differently.

**The current row is 2 ms SLOWER than the 2026-08-25 row, deliberately.** The
regulation got tighter (84.6 %/74.3 % of ticks at exactly 5 -> 99.4 %, drift
-0.11/-0.19 -> -0.000 tk/s, and the startup backlog below went away entirely),
but the host now runs a **pipelined non-blocking state read** that costs one tick.
That is a host-side choice, not a firmware property -- see the breakdown below.
Measured over 334 s / 167,155 ticks with both arms moving.

### `sent -> ref` minus queue fill is the LPF test

Because the box delay IS the fill, whatever is left over after subtracting it is
everything else in the path. That difference is a direct, single-number readout
of whether the controller LPF is on:

```
v8.7.3 unregulated, LPF on         difference  +10.0 / +11.0 ticks
v8.7.3 + queue_sync, LPF off       difference   +2.0 /  +2.0 ticks
v8.7.3 CURRENT (pipelined read)    difference   +3.17 / +3.03 ticks
```

Roughly +10 means the controller LPF is on; +1 to +3 means it is off and what
remains is host-side path. The current +3.0 breaks down as:

| component | ticks | removable? |
|---|---|---|
| transport | 1.0 | no -- also present on v8.6.1 |
| loop -> worker mailbox hop | ~1.0 | yes, by per-arm setpoint generation (see Open) |
| pipelined non-blocking state read | ~1.0 | yes, by config -- but it is the prerequisite for deleting the worker |

Use this rather than the free-decay pole, which the queue drain makes meaningless
here (measured r2 = 0.109 on one arm in the current run -- exactly the failure the
report warns about).

## Unregulated, the Queue Grows Without Bound

```
left : fill = +0.665 tk/s * t + 19.22   (residual std 0.44)
right: fill = +0.670 tk/s * t + 13.95   (residual std 0.41)
```

Send 500.006 Hz against a measured box drain of **499.34 Hz** -- a 0.13 % clock
mismatch, integrated. That is +1.33 ms/s of latency, forever: 15 s of observation
took the fill from 19 to 29, and a 5-minute run extrapolates to ~217 ticks
(435 ms). It matches the +1.3 ms/s drift previously seen on the legacy
controller, whose cause was inferred; RBACK now shows it directly.

**Safety note.** A FIFO means a software stop command queues BEHIND the backlog.
At 29 ticks that is 58 ms; at 5 minutes unregulated it is over 400 ms. The
hardware E-stop is unaffected, but the server's fault latch and tracking-error
response inherit that delay. v8.6.1's latest-queue had no such property.

## The LPF Is Separable, and Off Is Worth ~10 Ticks

`servo_alpha` is scaled by 0.1 inside the controller (vendor-confirmed, recorded
in the rbpodo servo-param validator), so script-level 10.0 = effective 1.0 =
pass-through = LPF off.

```
                            sent->ref   RBACK fill   difference = LPF share
v8.7.3 alpha=1  (LPF on)      33 tk        22            +11.0
v8.7.3 alpha=1  (LPF on)      38 tk        28            +10.0
v8.7.3 alpha=10 (LPF off)     20 tk        19             +1.0
```

With the LPF off, `sent->ref` fits as a pure dead time: 20.00 ticks on every
usable joint, residual 0.0001-0.003 deg. The remaining +1 tick is the same
transport delay measured on v8.6.1.

## Regulated: fill Locked at 5

With `queue_sync.enable: true, target_fill: 5` and per-arm worker cadence:

```
                   2026-08-25                 2026-08-26 (current)
RBACK observed     100 % of ticks             100 % of ticks
fill               median 5.0; 84 %/80 %      median 5.0; 99.4 % at exactly 5
                   at exactly 5               (steady running never leaves 5 +/- 1)
drift              -0.065 / -0.136 tk/s       -0.000 / -0.000 tk/s  LOCKED
trim (converged)   --                         +2.60 us both arms
sent -> ref        7.23 / 7.08 tk             8.17 / 8.03 tk
ref -> actual      ~2.7-3.3 tk                2.97 / 3.09 tk
end to end         10.26 / 10.40 tk           11.14 / 11.13 tk (22.3 ms)
                   (20.5 / 20.8 ms)           334 s / 167,155 ticks
```

The converged integral is **+2.60 us on both arms** -- 0.13 % of the 2 ms period,
the same clock mismatch the unregulated drift showed (500.006 Hz send vs 499.34 Hz
drain). This is a PHASE lock, not just a rate lock: the integral converges to the
per-cycle period extension that matches the box's consumption, so each send lands
at a fixed phase inside the box's tick. The residual 0.6 % of ticks at fill 6 is
what is left of that sub-tick freedom.

Predicted `sent->ref` was 6 ticks (fill 5 + transport 1); measured 7.1. The extra
**~1.1 ticks is the loop -> worker mailbox hop**: the servo loop generates the
setpoint and the worker picks it up on its own next cadence tick. Removing it
requires per-arm setpoint generation, not just per-arm sending.

## The Box Ignores the Stream for ~254 ms After Connect

The single most expensive property of v8.7.3, and it is not in the steady-state
numbers at all.

A box **already at activation stage 6** (`init_state_info == 6`, `servo_enabled`
true on the very first logged tick -- both instrumented specifically to test this)
consumes NOTHING for ~254 ms after connect while reporting **RBACK fill 0 the
whole time**, then reveals the entire backlog at once:

```
 t(s)   fill  parsed  init  phase     <- 2026-08-26, left arm
0.002      0       2     6  warmup
0.250      0     126     6  warmup    127 ticks at 0 while RBACK arrives normally
0.260     12     131     6  warmup    the jump
0.312    125                          peak == the ~128 commands sent meanwhile
2.540      5              track       2.3 s to drain
```

**The backlog is real, not a reporting artifact.** Under Drain the fill fell
monotonically with the applied trim (125 -> 92 -> 48 -> 12 -> 5), which is a FIFO
emptying. Had the box been consuming all along, slowing the sends would have
starved it to 0 instead.

Consequences while it lasts: box dead time ~260 ms, and since this is a FIFO, a
software stop queues behind it.

**Cause unknown.** The controller-manager author confirms it as a Rainbow control
box characteristic of v8.7.3 and that a drain phase at control start is expected.
Note the scale mismatch that rules out cold activation: that is documented as
~10 s on this hardware, and this window is 254 ms.

### The fix is structural: do not stream before there is a task

controller-manager never enters this window **by construction** -- it streams only
in `State::OnTask` and is silent in `Enabled`. rb_servo_server streamed a Hold from
connect, which is why it walked into it every start.

It now sends nothing until the first motion command. The box holds its last
reference (the same mechanism the existing freedrive suppression relies on) and the
arm stays stiff -- verified on hardware. In the log this reads as
`N tick(s) before the first RBACK`, because no send means no RBACK.

| | before | after |
|---|---|---|
| startup backlog | 131 ticks (262 ms) | **none** |
| time to `track` | 2.54 s | **0.63 s after arming** |
| `phase: drain` | 2.6 % | **0.1 %** |
| fill at exactly 5 | 94.1 % | **99.4 %** |
| fill excursion in steady running | -- | **5 +/- 1** |

Two host-side control fixes were tried first and both were wrong in an instructive
way: an activation gate on `init_state_info == 6` (refuted by instrumenting the
field BEFORE writing the gate -- the box was at 6 for all 167k ticks), and an
evidence-based warmup back-pressure that throttled sends (worked, 131 -> 26, but
treated a box property as a control problem; deleted once the structural fix
landed). **A drain phase at control start remains correct and is kept** -- it is
what brings the queue to 5 -- but it now runs against a queue that was never
allowed to grow, and it fills UP to 5 from below (the trim goes negative,
-14.4 / -12.5 us, which never happened while draining down).

## Five Bugs This Cost, All Host-Side

Worth recording because each was silent and each produced plausible-looking data:

1. **Worker send starvation.** With per-arm cadence, a tick whose latest-wins
   mailbox was already drained sent nothing. Two ~500 Hz sources at arbitrary
   phase means about half the ticks: measured **231 Hz of actual sends** against
   a 500 Hz loop. The fill fell to the protect floor and the short latency looked
   like success -- it was starvation. Fix: repeat the last setpoint, which is
   what a servo stream does anyway.
2. **RBACK never reached the log.** The loop read `queue_ack` from
   `lastSendResult()`, which the enqueue path overwrites every tick: **1 real
   observation in 7408 ticks**. Fix: a dedicated accessor written only on a real
   send.
3. **Worker state staleness.** A worker-cached read crosses a thread boundary --
   median age **1567 us** against a hardcoded 4 ms budget (direct I/O is
   25-125 us). One tick over it returned a jointless state, FK produced no TCP,
   and force control latched `ExternalForceLimit`, which rbpodo cannot reset.
   Now `servo.worker_state_max_age_periods` (default 4).
4. **Shared `pinocchio::Data`.** One scratch object served all FK/IK. Under
   concurrent per-arm access this does not return a wrong pose -- it **crashes**
   (5/5 Killed/Aborted in a targeted test). Fixed to per-thread scratch before
   any per-arm threading was enabled.
5. **Suppressing an action without stepping its state machine.** The warmup
   back-pressure above decided per tick whether to hold the send -- but the
   control law was only stepped on ticks that actually sent. So the first hold
   latched: the law was never asked again, the phase never advanced, and the
   arm went **42 s with no servo stream and the command 54 deg from actual**.
   Twelve green unit tests on the law itself said nothing about it, because the
   defect was in the caller's stepping contract. If an output can suppress the
   action that feeds its own input, the state machine must still advance.

## Open

- **The ~1.1-tick mailbox hop.** Needs per-arm setpoint generation (the servo
  loop's ~1500-line tick body, plus 32 per-arm member pairs, split across
  threads). Expected gain ~2-3 ms on 22.3 ms. Doing it also deletes the worker's
  cross-thread state cache, which is what makes the pipelined read below pay for
  itself.
- **Whether to keep the pipelined non-blocking state read.** It costs a measured
  1 tick today (`sent->ref` minus fill +2.0 -> +3.0) and buys removal of a
  blocking `request_data()` (126 us median, 3.8 ms max) from the worker. It is a
  prerequisite for deleting the worker, not a win on its own. Zero held reads in
  167,155 ticks, so it is not fragile -- the question is purely the 2 ms.
- **Why a stage-6 box ignores servo_j for 254 ms.** We now avoid the window
  instead of understanding it, and the shape of the startup transient changed
  (131 -> none) without knowing what the box actually needs.
- **Resampling.** The loop generates at 500.1 Hz and the worker sends at 499.0,
  so ~1.1 setpoints/s are overwritten. With the LPF off that discontinuity goes
  straight to the servo. Whether it matters is unmeasured.
- **The LPF-off jitter question is UNANSWERED, and jerk is the wrong statistic
  for it.** The historic note that script-level 10.0 "produced jerk/jitter on
  hardware" could not be confirmed or refuted from `q_actual` jerk. The stated
  reason was the log's 0.001 deg quantisation, quoted as "one LSB = 1.25e8
  deg/s^3"; that figure is `1/dt^3`, and per LSB it is **1.25e5**, so on its own
  arithmetic the signal would clear the noise. Re-measured 2026-08-26, the
  conclusion survives for a different reason: the LOGGED jerk column (computed in
  double precision before the CSV truncates) matches a text-derived jerk to within
  15 %, and at rest it is **exactly zero on 92 % of ticks** -- the box holds
  `q_actual` constant, so the quantisation is UPSTREAM of the logger and
  differentiating in double precision buys nothing. At rest jerk is a train of
  quantisation impulses; while moving it is dominated by the trajectory's own
  jerk. Neither regime isolates a tremor. Use band-limited velocity PSD over a
  long STEADY excitation with stale readback ticks masked, paired A/B against
  alpha=1 -- not jerk.
- Left-arm numbers in the LPF-off run are weak (0.47 deg excitation); the right
  arm carried that measurement.
- ~~**Worker-cached state age is creeping up**~~ -- RESOLVED 2026-08-26. The tail
  reached 7.6 ms against the 8 ms budget (95 %) and correlated with NEITHER
  `reqdata` duration NOR a lengthened send period, which ruled out both the box
  and the queue-sync trim: the thread simply was not being scheduled. `queue_sync`
  had promoted `ArmWorker` to owner of its arm's 500 Hz send instant while it was
  still a plain `SCHED_OTHER` thread with no affinity, and only the servo loop got
  FIFO + an isolated core. Now `SCHED_FIFO 80` + per-arm isolated cores (left
  cpu1 / right cpu2, loop keeps cpu3), mirroring controller-manager's `Arm::run`.
  After: median 1045 us (= half the read period, the irreducible inter-thread
  phase offset), **p99.9 = 2037 us (one period), zero ticks over budget**, one
  tick over 4 ms in 167,155.

## Reproducing

```bash
make run MODE=real
.venv/bin/python scripts/analyze_box_latency.py logs/servo_log.csv
```

Config: `servo.io_model: worker`, `queue_sync.enable: true`, `servo_alpha: 10.0`,
`servo.worker_realtime_priority: 80` + `worker_cpu_core_left/right: 1/2`,
`state_read_pipelined: true`. Method and interpretation caveats in
`docs/runbooks/box_latency_offline.md`.

**Exercise a motion.** The server streams nothing until the first motion command,
so a run that only starts the stack produces no sends, no RBACK and no latency at
all -- the report says `N tick(s) before the first RBACK` and then finds no moving
segment. Trigger InitMotion from the GUI a few times, both arms.

**Do not compare read paths with `state_age_us`.** `host_time_ns` is stamped at
frame-consume time, so it excludes the pipelined read's request->response transit
and makes that path look up to 2 ms better than it is. Use `sent->ref` minus RBACK
fill. Note also that `reqdata_call_duration_us` and `read calls advanced` read 0
under the pipelined path -- it does not call `request_data()` -- which is not a
fault.

**Do not read the analyzer's free-decay pole as a filter measurement on v8.7.3.**
After the command stops the tail is the queue draining, not an exponential; the
report flags this, but the number is still printed.
