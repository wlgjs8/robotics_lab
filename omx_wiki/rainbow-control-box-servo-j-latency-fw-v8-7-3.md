---
title: "Rainbow Control Box Servo J Latency (firmware v8.7.3)"
tags: ["rainbow-control-box", "firmware-v8.7.3", "servo-j", "latency", "queue-sync", "rback", "servo-alpha", "rb3-730e"]
created: 2026-08-25T12:41:26.355Z
updated: 2026-08-25T12:41:26.355Z
sources: ["logs/servo_log_20260825_211851.csv", "logs/servo_log_20260825_195125.csv", "logs/servo_log_20260825_210330.csv", "docs/runbooks/box_latency_offline.md", "scripts/analyze_box_latency.py"]
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

| configuration | queue fill | sent -> ref | end to end |
|---|---|---|---|
| v8.6.1 latest-queue, LPF on | n/a | 10.1 tk | **13.1 tk (26.2 ms)** |
| v8.7.3 unregulated, LPF on | 22-28, rising | 33-38 tk | 35-40 tk (70-80 ms) |
| v8.7.3 unregulated, LPF off | 19, rising | 20 tk | 23 tk (46 ms) |
| v8.7.3 + queue_sync(5), LPF off | **5.0, locked** | 7.1 tk | **10.3 tk (20.5 ms)** |

**Upgrading the firmware alone made latency 3x worse.** The FIFO only pays off
with queue regulation.

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
worker send rate   499.0 Hz both arms (99.8 % of the 500 Hz loop)
RBACK observed     100 % of ticks
fill               median 5.0; 84 % (left) / 80 % (right) at exactly 5
drift              -0.065 / -0.136 tk/s over the moving window
                   +0.011 / -0.027 tk/s over the later three quarters
sent -> ref        7.23 / 7.08 tk -- identical across all six joints
ref -> actual      ~2.7-3.3 tk
end to end         10.26 / 10.40 tk (20.5 / 20.8 ms)
```

Predicted `sent->ref` was 6 ticks (fill 5 + transport 1); measured 7.1. The extra
**~1.1 ticks is the loop -> worker mailbox hop**: the servo loop generates the
setpoint and the worker picks it up on its own next cadence tick. Removing it
requires per-arm setpoint generation, not just per-arm sending.

## Four Bugs This Cost, All Host-Side

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

## Open

- **The ~1.1-tick mailbox hop.** Needs per-arm setpoint generation (the servo
  loop's ~1500-line tick body, plus 32 per-arm member pairs, split across
  threads). Expected gain ~2-3 ms on 20.5 ms.
- **Resampling.** The loop generates at 500.1 Hz and the worker sends at 499.0,
  so ~1.1 setpoints/s are overwritten. With the LPF off that discontinuity goes
  straight to the servo. Whether it matters is unmeasured.
- **The LPF-off jitter question is UNANSWERED.** The historic note that
  script-level 10.0 "produced jerk/jitter on hardware" could not be confirmed or
  refuted: `q_actual` jerk is dominated by the log's 0.001 deg quantisation
  (one LSB = 1.25e8 deg/s^3 at 500 Hz, against measured medians of 1.7e6-7.3e6).
  Answering it needs a different observable, not this column.
- Left-arm numbers in the LPF-off run are weak (0.47 deg excitation); the right
  arm carried that measurement.

## Reproducing

```bash
make run MODE=real
.venv/bin/python scripts/analyze_box_latency.py logs/servo_log.csv
```

Config: `servo.io_model: worker`, `queue_sync.enable: true`,
`servo_alpha: 10.0`. Method and interpretation caveats in
`docs/runbooks/box_latency_offline.md`.

**Do not read the analyzer's free-decay pole as a filter measurement on v8.7.3.**
After the command stops the tail is the queue draining, not an exponential; the
report flags this, but the number is still printed.
