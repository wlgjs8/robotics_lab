# queue_sync: what it does well, and the five things it could not see

Reviewed 2026-09-03 against `submodules/controller-manager` at `cad3047` (v2.5.1-19),
whose `Arm::qsync_step` our `QueueSyncController` is a direct port of. The gains,
phases and actuator all transfer as-is and are unchanged. What differed was
**observability**, and one of the gaps had already happened on hardware.

## The regulator itself is performing well

On RB5-850E with firmware v8.9.1, in the Track phase, across **1,525,000 fresh RBACK
observations** over the 09-02 and 09-03 runs:

| | |
|---|---|
| minimum fill ever observed | **4** |
| observations at 4 | 10 |
| observations below 4 | **0** |
| confirmed underruns | 0 |

That is tighter than CM's own cell, which holds 5.02 [5..6] and has a population of
deep dips. The 130 sub-4 events found in the 2026-08-26 logs are RB3-era, from before
this hardware and before qsync telemetry existed in the log schema at all.

**So none of what follows is a performance problem.** It is that the instrumentation
could not distinguish a healthy queue from an unmeasured one — which is a problem
precisely when something does go wrong.

## The one that already happened: `locked` cannot go false

`servo_log_20260902_230031`, left arm:

| | |
|---|---|
| RBACK frames parsed | **299** (right arm: 77,321 — a 258× deficit) |
| last fresh RBACK | tick 1,585 = **2.0 %** into the run |
| ticks that then ran with no feedback | **77,130 ≈ 154 s** |
| `left_qsync_locked` over that window | **77,130 / 77,130 = 100 % true** |
| events reported | `stall_events` = 1, and nothing else |

The regulator ran **fully open loop for 154 seconds** and the health indicator
asserted a phase lock the entire time.

The cause is one missing term. `locked` was

```cpp
phase_ == Track && last_fill_ >= 0 && std::abs(last_fill_ - target_fill) <= 1
```

and `last_fill_` **persists between RBACKs by design** — it is the last thing the box
said, not a statement that the box is still saying it. A frozen reading that happens
to sit near the setpoint therefore reports a lock forever.

**It was harmless only by luck: the frozen value was exactly the setpoint.** The error
was 0, so the integral stayed 0 and the trim stayed 0, and the regulator did nothing
at all. Had it frozen at 3, the PI would have integrated a −2 error for 77,000 cycles
and wound to its clamp against a queue nobody was measuring.

It is also **intermittent** — 2 of the 12 real runs on 09-02 (`221109` ratio 0.060,
`230031` ratio 0.004), and not once in the five runs since, including all of 09-03.
That makes it harder to chase, not easier.

`locked` now also requires `stale_cycles_ < stall_cycles`, and `stale_cycles` is
published so the condition is visible for its whole duration rather than as a single
edge 154 seconds in the past. `locked` gates nothing — `hold_motion_until_track` tests
the *phase*, not this flag — so no motion was ever authorised by the false lock.

## The other four

**2. The underrun counter fired on one sample.** CM requires three consecutive *fresh*
observations, added after a single bad reading de-energized a healthy cell whose queue
read 5 in 13,000+ samples. Their diagnosis is worth keeping: `last_fill` is an integer
scraped out of a TCP byte stream across chunk boundaries with a carry. We only count,
never fault, so the stake is not safety — it is whether `underrun_events` can be cited
as evidence at all. It now requires `underrun_confirm` (3) consecutive fresh samples.

**3. Events re-armed off the band edge, not at target.** A queue hovering at the line
reports 3,4,3,4,… and re-arming at the edge makes every return a fresh event — at 500
fresh RBACKs a second, an event every other cycle. That is a per-sample test wearing
per-episode clothes; CM hit it and re-armed on recovery to the setpoint. So do we now.

**4. The band from 2 to 4 was unwatched.** `protect_fill` (1) was the only sub-target
detector, so a queue sagging for seconds produced no counter and no line. There is now
a `warn_fill` band with per-episode `warn_events`, and a dip episode that records its
depth and duration and reports once on recovery.

**5. No approach history.** CM keeps 20 fills because their first guard "reported the
word QueueUnderrun and not one number". `QueueSyncDecision::fill_trace` now keeps the
same 20, fresh observations only, and a dip episode prints them on one line.

## The trap that comes with a dip detector

**Do not derive the dip/warn line from `target_fill`.** CM tested `fill < TARGET`, then
raised the setpoint 5 → 6, and the detector moved with it onto the regulator's own
ripple trough — firing once per episode, forever, on both arms, on a controller that
was working correctly. A diagnostic must not be tied to the thing it watches: the
setpoint is a tuning knob and will move again, but the question "did the box come near
starving?" does not move with it.

`warn_fill` is therefore its own number, and `config.cpp` **refuses** `warn_fill >=
target_fill` by name so this cannot be reintroduced by a tuning edit.

## What the trace is for

A dip episode ends with one line carrying its approach, because the shape is what
classifies it — the same distinction CM reached by reading 51 deep dips individually
(38 clustered singles, 11 enable/reset handshakes with an empty trace, and exactly one
true descent, which was a box collision stop):

* `[6 5 4 3 2 1]` — a ramp. A real drain.
* `[6 6 6 6 6 1]` then 6 again — **a reporting glitch.** A queue fed one command per
  tick cannot climb from 1 to the setpoint in a single observation, so the queue never
  drained; the *reading* was wrong.

## Not changed

* **`target_fill` stays 5.** CM raised theirs to 6 on 2026-08-23 to buy a third tick of
  margin above their de-energize floor, at the cost of 2 ms of command latency. We have
  no such floor (our underrun path counts and does not act), so the argument transfers
  only partly, and the latency is real. Operator decision, 2026-09-03: hold at 5.
* Every gain, phase threshold and actuator limit. This review changed what the
  regulator *reports*, not what it *does*.

## Tests

`rb_servo_server/tests/test_queue_sync_controller.cpp`, four new cases. Each was run
against the pre-fix behaviour and fails there:

```
CHECK failed ...:449: !d.locked                        <- the 154 s field case
CHECK failed ...:471: d.underrun_events == base        <- single-sample underrun
CHECK failed ...:509: d.warn_events == base_warn + 1   <- per-sample band flood
```

The freshness case is the one worth noting: the frozen fill is *exactly* the setpoint,
so every term of the old condition is legitimately satisfied and only freshness
separates a locked regulator from a blind one.
