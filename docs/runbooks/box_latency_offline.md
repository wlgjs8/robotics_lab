# Offline Control-Box Latency Runbook

This runbook measures how the Rainbow control box consumes the 500 Hz servo_j
stream, using a servo CSV recorded during a normal supervised session. It is an
analysis procedure, not a motion procedure: it authorizes nothing on its own and
adds no runtime component to the control loop.

Record on hardware, stop the stack, analyze afterwards. Nothing here needs to
run in real time.

## Firmware Pairing (read first)

Every latency number is paired to a control-box firmware version, because the
box-side scheduling model is the thing being characterised. The current pairing
is **v8.7.3**, and it is not a refinement of v8.6.1 — the two firmwares consume
the stream by different rules, so a v8.6.1-era number is not comparable:

| firmware | box-side scheme | what stage (a) is |
| --- | --- | --- |
| v8.6.1 | latest queue | ~1 tick of transport behind a first-order LPF (`a = servo_alpha / 10`) |
| v8.7.3 | **FIFO** | **`RBACK queue fill + 1 tick`, exactly — no filter at all once the LPF is off** |

What that changes about this procedure:

- On v8.7.3 the box delay is whatever the host lets the queue depth become. A
  latency number without the queue fill recorded beside it says nothing.
- The fill is a pure integrator (`dfill/dt = f_send - f_box`), and the clocks do
  not match: measured box drain **499.34 Hz** against a 500.006 Hz send. An
  unregulated stream therefore grows the fill **+0.67 tk/s (+1.33 ms/s) without
  bound** — 19 to 29 ticks in 15 s, and ~217 ticks (435 ms) extrapolated at five
  minutes.
- The tracked real profile regulates it. `rb_servo_server/config/stack_real.yaml`
  pins `servo.io_model: worker`, `queue_sync.enable: true`, `target_fill: 5`, and
  `queue_sync.hold_motion_until_track: true`, plus `servo_alpha: 10.0`
  (controller LPF off, so the server SMD owns all smoothing).
  Measured on that profile: fill locked at 5, stage (a) 7.05 / 7.46 tk, end to
  end **20.0 / 20.9 ms**. The controller-simulation stack (`stack_sim.yaml`)
  does **not** regulate: `io_model: direct`, no `queue_sync`. If those boxes also
  run v8.7.3, MODE=sim latency drifts the same way and is not comparable to
  MODE=real — confirm the VM/box firmware before reading a sim log as a real
  proxy.
- `servo.worker_setpoint_interpolation` remains `false` in the tracked real
  stack. The host-loop/box-clock mismatch can therefore overwrite mailbox
  setpoints; interpolation has implementation and unit-test evidence, but its
  separate on-robot A/B has not been accepted. Record the worker counters below
  and do not describe interpolation as active or physically validated.
- **Safety.** A FIFO means a software stop command queues BEHIND the backlog, so
  the server's fault latch and tracking-error response inherit the queue depth
  (58 ms at 29 ticks; >400 ms after five unregulated minutes). The hardware
  E-stop is unaffected. This is a property v8.6.1's latest queue did not have,
  and it is a standing reason to keep `queue_sync` enabled rather than a tuning
  preference.

The full characterisation, including the numbers the checks below are compared
against, is `omx_wiki/rainbow-control-box-servo-j-latency-fw-v8-7-3.md`; the
superseded v8.6.1 measurement is its sibling page. After a firmware change,
write a new sibling page and re-measure — do not amend an existing one.

## What Gets Measured

Every servo tick the CSV holds the three joint-space signals side by side
(see `docs/servo_backend_contract.md`):

```
q_sent   ->   q_ref   ->   q_actual
       (a)          (b)
```

- **(a) sent -> ref** — control-box ingest. This is the box-side latency the
  runbook exists for. On v8.7.3 it is queue depth plus one tick of transport; on
  v8.6.1 it was transport plus the controller's own filtering.
- **(b) ref -> actual** — servo loop and mechanics tracking the box reference.
- **sent -> actual** — end to end. It must equal (a) + (b); the analyzer reports
  all three so that identity is an internal consistency check on the fit.

Beside them the CSV carries the box's own reported command-queue occupancy,
parsed from the drained Servo J ACK text:

```
<arm>_rback_fill              # commands waiting in the box FIFO, this tick
<arm>_rback_fill_min/_max     # spread within the tick
<arm>_rback_observed          # whether a real RBACK landed on this tick
<arm>_rback_parsed_total      # parsed / drained / malformed counters
<arm>_rback_drained_total
<arm>_rback_malformed_total
```

On v8.7.3 that is not context, it is the measurement: stage (a) equals
`rback_fill + 1`. Logs recorded before these columns existed still analyze, but
they cannot separate queueing from anything else.

The current CSV also records the worker/mailbox boundary:

```text
<arm>_worker_pending_overwrites_total
<arm>_worker_repeated_sends_total
<arm>_worker_wire_dispatches_total
<arm>_worker_wire_send_start_ns / _end_ns
<arm>_worker_interp_active / _delay_setpoints / _rebase_total / _hold_total
<arm>_state_host_time_ns
```

Equal consecutive `state_host_time_ns` values are duplicated readback samples,
not two fresh controller measurements. Their apparent 0-then-2x `q_ref` step
must not be labeled a physical reference impulse. Use
`scripts/analyze_smoothness.py` with these counters when comparing mailbox or
interpolation A/B runs.

Delay is fitted as the tick shift minimising residual RMSE over contiguous
moving segments, refined to sub-tick resolution by parabolic interpolation.

That single number is not the transport delay, and reading it as one is the
easiest mistake here. Queueing, dead time, and a low-pass filter all push the
response later, so the report also fits `dst[k] = (1-a)*dst[k-1] + a*src[k-D]`
and measures the free decay of the error after the command stops.

### Splitting stage (a): subtract the queue fill

On v8.7.3 the split is arithmetic rather than inferred. Because the box delay IS
the fill, whatever is left after subtracting it is everything else in the path,
and that difference is a direct one-number readout of whether the controller LPF
is on:

```
                                sent->ref     RBACK fill   difference
v8.7.3 unregulated, LPF on       33 tk           22           +11.0
v8.7.3 unregulated, LPF on       38 tk           28           +10.0
v8.7.3 unregulated, LPF off      20 tk           19            +1.0
v8.7.3 + queue_sync, LPF off      7.05/7.46 tk    5.0          +2.0 / +2.0
```

Roughly +10 ticks means the controller LPF is active; +1 to +2 means it is off
and you are looking at transport plus the loop -> worker mailbox hop. Compute it
by hand from the report's `stage` line and its `RBACK queue fill median` line —
the analyzer prints both but does not subtract them for you.

**Do not use the free-decay pole for this on v8.7.3.** After the command stops,
the tail is the FIFO draining — dead time, not an exponential — so fitting a
pole to it reports a filter that is not there. The analyzer flags this
(`!! NOT a filter measurement`) when the stage has >= 3 ticks of dead time, but
it still prints the number. The pole was the trustworthy discriminator on
v8.6.1's latest queue, where stage (a) had ~1 tick of dead time and a real
first-order filter behind it; that reasoning does not carry over.

For the historical record: measured on firmware **v8.6.1** with
`servo_alpha: 1.0`, stage (a) was 1 tick of dead time behind a first-order filter
with `a = 0.100` (pole 0.900 per 2 ms tick, tau 9.5 ticks) — the shift fit alone
reported "10.1 ticks", of which only one was transport. The ref -> actual stage
carries no comparable filter on either firmware.

Note the estimator choice. Correlation-based lag estimation degenerates here:
`q_sent` and `q_ref` are near-identical signals, so the correlation curve is
flat to ~1e-4 across many shifts and the argmax is noise. Residual RMSE has a
sharp minimum on the same data. Do not swap it for a correlation peak.

## Record

Run the stack as usual under operator supervision:

```bash
make run MODE=real
```

Exercise the motion whose latency is in question — teleop, replay, or a policy
rollout. Latency is only observable while the arm is commanded to move; an idle
window carries no information and the analyzer says so instead of fitting noise.

Stop the stack (Ctrl-C). The log is `logs/servo_log.csv` (symlinked to the
timestamped file for that session). Budget disk accordingly: the log runs
~3.6 MB/s, about 1 GB per 5 minutes of session.

## Analyze

```bash
.venv/bin/python scripts/analyze_box_latency.py logs/servo_log.csv
```

Useful flags:

- `--arm left|right|both` (default `both`)
- `--start-sec S --duration-sec D` — restrict to one manoeuvre
- `--segments K` — split the moving budget into K chunks and refit sent -> ref
  per chunk, so a growing box queue shows up as rising lag instead of being
  averaged away. `--segments 1` disables. On v8.7.3 read it together with the
  RBACK `drift` line, which measures the same thing directly.
- `--max-lag-ticks N` — search bound, default 40 ticks (80 ms at 500 Hz). The
  report flags `!! at search bound` when the fit hits it; raise N and refit
  rather than believing a pinned value. An unregulated v8.7.3 box reaches the
  default bound within a few minutes of streaming, so a pinned 40 there is the
  growing queue, not a fixed delay.
- `--json` — machine-readable, same content.

## Plot

```bash
.venv/bin/python scripts/plot_servo_joints.py logs/servo_log.csv
```

One panel per joint per arm, x axis in servo ticks, `q_sent` / `q_ref` /
`q_actual` overlaid, with `init_motion` planning/executing phases shaded. It
auto-windows to the ticks where something was commanded to move; `--all` or
`--tick-start/--tick-stop` override that, `--relative` plots displacement from
the window start, and `--error` swaps the raw signals for `q_sent - q_ref` and
`q_ref - q_actual`.

The three signals sit within ~10 ticks of each other on the regulated v8.7.3
profile (~13 on v8.6.1, and 35-40 on an unregulated v8.7.3 box), so at
full-motion zoom they overlap. Narrow the tick window to see the stage delays
directly; the `--error` view shows them without zooming. Requires matplotlib.

### Interactive zoom

```bash
.venv/bin/python scripts/plot_servo_joints.py logs/servo_log.csv --html --open
```

`--html` writes a standalone page (plotly inlined, no network needed, ~6 MB)
instead of a PNG. Box-zoom or scroll-zoom on any panel and all twelve follow,
because the x axes are linked. The axis is clamped to whole ticks as you zoom
in — 5-tick steps under a 60-tick span, 2 under 30, and every single tick under
12 — so the x scale resolves to one 2 ms control period at full zoom, and
per-tick markers appear at the same depth. Hover reports both the tick index and
its time in ms. Zooming back out returns the axis to automatic ticking.

No GUI toolkit is involved: this workstation's venv has neither tkinter nor Qt,
so an interactive matplotlib window is not available. The page opens in any
browser.

## Reading the Report

Check these before trusting any latency number:

- **tick dt** — confirm the loop really ran at the expected period and count
  overruns. A log full of overruns dates the delays to the host, not the box.
- **q_ref_valid** — well below 100 % means the readback was not decodable and
  the fit is built on gaps.
- **readback freshness** — the fraction of ticks where `q_ref`/`q_actual`
  changed at all, and the box update rate that implies. If the box publishes
  slower than the servo tick, part of the measured sent -> ref delay is
  sampling, not queueing. Quantify it here before attributing it to the box.
- **RBACK queue** — `fill median/min/max`, the histogram, and the `drift` line
  with its `LOCKED` / `DRIFTING` verdict. This is the first thing to read on
  v8.7.3: it *is* stage (a) minus one tick. `firmware reported no occupancy on
  any tick` means the ACK text was never parsed — either the box is not
  reporting or the wire format changed, and every latency number below it is
  then unattributed. `DRIFTING` means the send rate is not locked to the box and
  the run's latency is a moving target, so a single averaged lag is the wrong
  summary. `!! N drained response(s) mentioned RBACK but did not parse` is a
  wire-format regression, not a measurement problem.
- **gain** — the least-squares amplitude ratio at the fitted lag. A gain near
  1.0 with small residual means a clean pure delay. A gain well under 1.0 means
  a filtering stage is attenuating, and a single delay number under-describes
  what the box is doing.
- **transport vs filter block** — dead time, `a`, `tau`, and the ramp lag they
  imply. If `resid` is large the split is not trustworthy; if `a >= 1` the stage
  has no first-order lag and the whole delay is dead time.
- **free decay** — the pole measured after the command stops, with `r2`. On
  v8.7.3 this is **not** a filter measurement and the report says so
  (`!! NOT a filter measurement`) whenever the stage carries >= 3 ticks of dead
  time: the tail is the FIFO draining. Use `sent -> ref` minus the RBACK fill
  instead. The pole is still meaningful for a stage with negligible dead time,
  and a low `r2` still means the decay was not exponential.
  `--decay-floor-deg` must stay well above the log's 0.001 deg quantisation or
  the tail is noise.
- **stationarity block** — sent -> ref lag per time chunk. A monotone rise is
  the signature of an uncontrolled box command queue, not of a fixed transport
  delay, and is a different problem with a different fix. On v8.7.3 it should
  agree with the RBACK `drift` line; if the lag rises while the fill reads
  `LOCKED`, one of the two is wrong and neither should be quoted until that is
  resolved.

`servo_j send`, `state age`, and `reqdata call` in the same report bound the
host-side contribution, so a box-side claim can be separated from a host-side
one rather than asserted.

## Limits

- The analyzer refuses logs recorded before the `q_ref` CSV columns existed. It
  does not substitute `q_sent` for the readback.
- The `<arm>_rback_*` columns are optional: a log without them still fits lags,
  but on v8.7.3 it cannot separate queueing from transport, which is most of
  what stage (a) is. Prefer re-recording over interpreting such a log.
- A log without worker accounting and `state_host_time_ns` can still support
  the basic latency fit, but it cannot attribute skipped setpoints or separate
  duplicated state readback from a controller impulse. Do not use it to accept
  worker interpolation.
- Per-joint fits are reported and only joints with real excitation are used; a
  joint that barely moved is marked unusable rather than fitted.
- The CSV writes doubles at the stream default of 6 significant digits, so a
  three-digit joint angle is quantised to 0.001 deg. That is far below the
  per-tick motion of any commanded move and does not bound the fit, but it does
  bound how small a residual is meaningful.
- `robot_time` is a vendor-unreliable field on this hardware (the `-2001`
  suspect-diagnostics carve-out), so box-side timestamps are not used to derive
  latency. Everything here is host-clock and tick-count based.
