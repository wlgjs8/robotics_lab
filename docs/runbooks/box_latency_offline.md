# Offline Control-Box Latency Runbook

This runbook measures how the Rainbow control box consumes the 500 Hz servo_j
stream, using a servo CSV recorded during a normal supervised session. It is an
analysis procedure, not a motion procedure: it authorizes nothing on its own and
adds no runtime component to the control loop.

Record on hardware, stop the stack, analyze afterwards. Nothing here needs to
run in real time.

## What Gets Measured

Every servo tick the CSV holds the three joint-space signals side by side
(see `docs/servo_backend_contract.md`):

```
q_sent   ->   q_ref   ->   q_actual
       (a)          (b)
```

- **(a) sent -> ref** — control-box ingest: command queue depth plus the box's
  own interpolation. This is the box-side latency the runbook exists for.
- **(b) ref -> actual** — servo loop and mechanics tracking the box reference.
- **sent -> actual** — end to end. It must equal (a) + (b); the analyzer reports
  all three so that identity is an internal consistency check on the fit.

Delay is fitted as the tick shift minimising residual RMSE over contiguous
moving segments, refined to sub-tick resolution by parabolic interpolation.

That single number is not the transport delay, and reading it as one is the
easiest mistake here. A dead time and a low-pass filter both push the response
later, so the report also fits `dst[k] = (1-a)*dst[k-1] + a*src[k-D]` and
measures the free decay of the error after the command stops. Measured on
firmware v8.6.1 with `servo_alpha: 1.0`, the sent -> ref stage is 1 tick of dead
time behind a first-order filter with `a = 0.100` (pole 0.900 per 2 ms tick,
tau 9.5 ticks) -- the shift fit alone reported "10.1 ticks", of which only one is
transport. The ref -> actual stage carries no comparable filter.

The free-decay pole is the more trustworthy of the two: it is measured while the
command is constant, so nothing about the trajectory shape can bias it, and the
reported `r2` says whether the decay was actually exponential.

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
  averaged away. `--segments 1` disables.
- `--max-lag-ticks N` — search bound, default 40 ticks (80 ms at 500 Hz). The
  report flags `!! at search bound` when the fit hits it; raise N and refit
  rather than believing a pinned value.
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

The three signals sit within ~13 ticks of each other, so at full-motion zoom
they overlap. Narrow the tick window to see the stage delays directly; the
`--error` view shows them without zooming. Requires matplotlib.

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
- **gain** — the least-squares amplitude ratio at the fitted lag. A gain near
  1.0 with small residual means a clean pure delay. A gain well under 1.0 means
  a filtering stage is attenuating, and a single delay number under-describes
  what the box is doing.
- **transport vs filter block** — dead time, `a`, `tau`, and the ramp lag they
  imply. If `resid` is large the split is not trustworthy; if `a >= 1` the stage
  has no first-order lag and the whole delay is dead time.
- **free decay** — the pole measured after the command stops, with `r2`. A low
  `r2` means the error was not decaying exponentially and the pole is
  meaningless. `--decay-floor-deg` must stay well above the log's 0.001 deg
  quantisation or the tail is noise.
- **stationarity block** — sent -> ref lag per time chunk. A monotone rise is
  the signature of an uncontrolled box command queue, not of a fixed transport
  delay, and is a different problem with a different fix.

`servo_j send`, `state age`, and `reqdata call` in the same report bound the
host-side contribution, so a box-side claim can be separated from a host-side
one rather than asserted.

## Limits

- The analyzer refuses logs recorded before the `q_ref` CSV columns existed. It
  does not substitute `q_sent` for the readback.
- Per-joint fits are reported and only joints with real excitation are used; a
  joint that barely moved is marked unusable rather than fitted.
- The CSV writes doubles at the stream default of 6 significant digits, so a
  three-digit joint angle is quantised to 0.001 deg. That is far below the
  per-tick motion of any commanded move and does not bound the fit, but it does
  bound how small a residual is meaningful.
- `robot_time` is a vendor-unreliable field on this hardware (the `-2001`
  suspect-diagnostics carve-out), so box-side timestamps are not used to derive
  latency. Everything here is host-clock and tick-count based.
