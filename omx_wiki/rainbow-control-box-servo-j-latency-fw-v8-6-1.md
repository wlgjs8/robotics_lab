---
title: "Rainbow Control Box Servo J Latency (firmware v8.6.1)"
tags: ["rainbow-control-box", "firmware-v8.6.1", "servo-j", "latency", "servo-alpha", "latest-queue", "rb3-730e"]
created: 2026-08-25T10:02:21.788Z
updated: 2026-08-25T10:02:21.788Z
sources: ["logs/servo_log_20260825_171603.csv", "docs/runbooks/box_latency_offline.md", "scripts/analyze_box_latency.py"]
links: ["flow-infer-delta-preview-controller-contract.md"]
category: reference
confidence: high
schemaVersion: 1
---

# Rainbow Control Box Servo J Latency (firmware v8.6.1)

Measured latency of the `servo_j` command path on the RB3-730E control boxes.
**Every number here is paired to control-box firmware v8.6.1 and must be
re-measured after any firmware change** — the box-side scheduling model is the
thing being characterised, so a firmware upgrade invalidates the page rather
than amending it. The v8.7.3 successor belongs in its own sibling page.

## Operating Point

| | |
|---|---|
| control-box firmware | **v8.6.1** |
| box scheduling model | **latest queue** — the box tracks the newest command rather than replaying a FIFO |
| `servo_alpha` | 1.0 (controller LPF ON) |
| `servo_t1_sec` / `servo_t2_sec` / `servo_gain` | 0.002 / 0.021 / 1.0 |
| servo loop | 500 Hz, `io_model: direct`, 1 tick = 2 ms |
| evidence | `logs/servo_log_20260825_171603.csv`, dual-arm InitMotion, 5587 ticks @ 500.0 Hz, 0 period overruns |

## Result

Three stages, from the servo_j target the host sent to the encoder position:

```
q_sent  --(a)-->  q_ref  --(b)-->  q_actual
```

| stage | dead time | first-order `a` | tau | steady-state lag |
|---|---|---|---|---|
| (a) sent -> ref | **1 tick** (2 ms) | **0.100** | 9.49 tk (19.0 ms) | **10.06 tk (20.1 ms)** |
| (b) ref -> actual | **3 ticks** (6 ms) | none (`a` >= 1, tau ~ 0) | — | 2.90 tk (5.8 ms) |
| end to end | — | — | — | **13.05 tk (26.1 ms)** right, 13.53 tk left |

The per-joint spread on stage (a) is 10.06–10.07 ticks across all twelve joints,
both arms — this is a box-level constant, not a per-joint or per-load effect.

### The single "lag" number is not a transport delay

A dead time and a low-pass filter both push a response later, so a shift fit
cannot tell them apart. Stage (a) reads as "10.1 ticks" to a shift fit, but only
**1 tick of that is transport**; the other nine are the filter. Onset in the raw
log makes the transport part directly visible:

```
tick    q_sent      q_ref       q_actual
3901  -243.9050  -243.9050  -243.9050    all held
3902  -243.9010  -243.9050  -243.9050    q_sent departs
3903  -243.8940  -243.9040  -243.9050    q_ref departs      <- 1 tick
3909  -243.7820  -243.8790  -243.9030    q_actual departs   <- 6 ticks after q_ref
```

The left arm reads 4 ticks at the same threshold only because its command ramps
in more slowly and takes longer to cross the log's 0.001 deg quantisation; the
transport delay itself is the same on both arms.

## The Box LPF Sits Upstream of `jnt_ref`

After the command stops, the `q_sent - q_ref` error decays geometrically with a
pole of exactly **0.9000 per tick** (r2 = 1.0000 over 30 consecutive ticks):

```
q_ref[k] = q_ref[k-1] + 0.100 * (q_sent[k-1] - q_ref[k-1])     tau = 9.49 tk = 19.0 ms
```

Nine of the ten joints with a usable decay measured 0.9000 +/- 0.0001; left J2
read 0.8947 at small amplitude. The remaining two had already settled below the
0.001 deg quantum.

**This exponential appears in `q_sent - q_ref`, never in `q_ref - q_actual`.**
Stage (b) fits `a` >= 1 — no first-order lag at all, just 3 ticks of dead time —
and the settled offset is <= 0.001 deg (the log quantum) on all twelve joints.
So `sdata.jnt_ref` is the **post-filter** reference: changing `servo_alpha`
moves stage (a) and cannot affect how `q_actual` converges to `q_ref`.

Cost of the filter, as settling time on stage (a):

| error falls to | ticks | ms |
|---|---|---|
| 63 % (1 tau) | 9.5 | 19 |
| 90 % | 21.9 | 44 |
| 95 % | 28.4 | 57 |
| 99 % | 43.7 | 87 |
| 99.9 % | 65.6 | 131 |

Stop commanding and the box reference still needs **87 ms to come within 1 %**.

## Budget Consumer

The 13.05-tick (26.1 ms) end-to-end figure is the floor under the
command-to-measured-actual lead that
[[flow-infer-delta-preview-controller-contract]] requires to be bounded by
positive config. Any lead bound set below ~13 ticks is unreachable on this
firmware no matter what the projection chain does, because the box alone spends
that much before the encoder moves.

## Why This Is Consistent With a Latest Queue

A FIFO of depth N would show pure dead time of N ticks with no filter, and its
lag would move with queue fill. What is measured instead is 1 tick of transport
followed by a first-order approach to the newest command — the box is always
chasing the latest value, which is what a latest-queue scheme does. Nothing in
the data suggests an accumulating queue.

## Open: `a = servo_alpha / 10`

`a` measured exactly 0.100 at `servo_alpha: 1.0`, and the tracked config records
that "script-level 10.0 disables controller LPF". Both facts fit
**`a = servo_alpha / 10`** (alpha 10 -> a = 1.0 -> pass-through). This is inferred
from a single operating point and is **not confirmed**. `servo_t2_sec` is the
other candidate but fits worse: t1/t2 = 2/21 = 0.0952 against a measured 0.1000.

To settle it, run once at `servo_alpha: 2.0` and check for `a = 0.2`
(pole 0.800). **Do this before the v8.7.3 upgrade** — if the successor firmware
schedules on a fixed delay instead, the alpha law may no longer be observable.

## Successor: v8.7.3 (fixed 5-tick delay)

v8.7.3 is reported to hold a constant 5-tick delay. If that is a fixed-depth
pipeline rather than a latest queue, the same measurement should show:

- stage (a) dead time **5 ticks**, filter `a` ~ 1.0, tau ~ 0
- stage (a) steady-state lag ~ **5 tk (10 ms)**, down from 10.06 tk (20.1 ms)
- end to end ~ **8 ticks**, down from 13
- no exponential tail after the command stops — the free-decay pole should come
  back unusable or with a poor r2 instead of a clean 0.9

Those are the discriminating checks; record them on the v8.7.3 page rather than
editing this one.

## Reproducing

```bash
make run MODE=real                     # exercise the motion, then stop
.venv/bin/python scripts/analyze_box_latency.py logs/servo_log.csv
.venv/bin/python scripts/plot_servo_joints.py logs/servo_log.csv --html --open
```

Method and interpretation caveats are in `docs/runbooks/box_latency_offline.md`.
The joint-space `q_ref` columns this depends on were added to the servo logger on
2026-08-25; logs before that carry `q_sent`/`q_actual` only and the analyzer
refuses them.

## Limits

- One InitMotion run, a single 470-tick moving segment per arm. The filter was
  time-invariant across that window, but **long-run stationarity of stage (a) is
  not yet established** on real `q_ref` data.
- Left J4 moved only 0.64 deg and its stage-(b) fit is unreliable; a run that
  exercises J4 properly is needed before trusting a per-joint stage-(b) number.
- The CSV writes 6 significant digits, so a three-digit joint angle quantises at
  0.001 deg. That bounds onset detection and the usable decay tail, not the fits.
