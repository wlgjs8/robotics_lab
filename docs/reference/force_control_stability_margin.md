# What our admittance loop is stable against

Our force-control numbers are stable against a hand and unstable against a rigid
surface, and the crossover is at a **contact stiffness of about 26 kN/m**. This page
says where that number comes from, which channel fails first, and what it costs.

It exists because we have already had the failure once — the 2026-08-27 incident
recorded in `stack_real.yaml` (`oscillation_guard_*`), where a hand push pinned the
40 mm fence and a ~5.3 Hz oscillation grew until J6 saturated at `dq_max` and the
operator hit E-stop. At the time that was attributed to "F/T dynamic feedback through
the ~12 ms pipeline delay". That is the right cause, and this page makes it a number.

## The model

From controller-manager, `wiki/findings/follow-contact-ring-is-delay-not-the-gate.md`.
The admittance loop closes through the environment, and the binding constraint is its
phase margin against transport delay:

```
L(s) = k_env · 1/(m·s² + b·s + k) · 1/(1 + s/ω_f) · exp(−T_d·s)
        ↑                ↑                ↑              ↑
     contact      the admittance     wrench_filter_hz   loop delay
    stiffness         law            (ours; CM has none)
```

CM validated it by predicting their ring at **5.43 Hz against 5.64 Hz measured** — 4%,
which is a mechanism rather than a curve fit. Reimplementing it reproduces all four
rows of their published sweep to three significant figures.

**It predicts our incident too.** Our rotation law (`m=1, b=15.72, k=8`) at the 12 ms
delay our own config states rings at **5.60 Hz**, against the **~5.3 Hz** observed — 6%,
the same quality of fit, on a robot and a law CM never analysed.

### Two traps CM paid for, worth not re-paying

* **ζ is the wrong criterion when there is loop delay.** Their 2026-08-26 analysis
  computed ζ = b/(2√(m·k_env)) = 0.34, called the damper correctly matched, and was
  wrong: that form has no delay in it. Our own `force_control:` block derives `b` from
  ζ = 0.25 — a sound way to pick `b`, but **not** a statement about stability.
* **An exact subharmonic lock is entrainment, not cause.** Their first capture rang at
  exactly half the input rate and was read as the force gate relaying. It was not; the
  ring persisted with the gate provably inert.

## What our numbers are stable against

`T_d = 12 ms` (our config's stated F/T pipeline delay, and what our own 5.3 Hz ring
independently implies), `wrench_filter_hz = 25`.

**Translation** — `m=15, b=434.7, k=400`, ring at 5.88 Hz:

| k_env [N/m] | what that is | gain margin | |
|---|---|---|---|
| 1,000 | a human hand / arm | +28.4 dB | stable by a mile |
| 5,000 | soft foam, flesh | +14.5 dB | stable |
| 15,000 | wood, light contact | +4.9 dB | marginal |
| **26,425** | **the 0 dB crossing** | **0.0 dB** | **the boundary** |
| 30,663 | CM's 2026-08-27 surface | −1.3 dB | unstable |
| 44,400 | CM's 2026-08-26 surface | −4.5 dB | unstable |
| 50,000 | the `k_env` our own config assumes when deriving `b` | −5.5 dB | unstable |

**Rotation is the weaker channel** — `m=1, b=15.72, k=8`, ring at 4.47 Hz. Rotational
environment stiffness is `k_env · L²`, so the lever decides it, and our lever is not
the one the numbers were derived for:

| lever L | 0 dB at k_env of | margin at 30,663 | at 44,400 |
|---|---|---|---|
| 140 mm — CM's assumption, which our `k_r` and `max_torque_nm` still carry | 46,550 N/m | +3.6 dB | +0.4 dB |
| **202.642 mm — the pika's actual SRO→TCP lever** | **22,219 N/m** | **−2.8 dB** | **−6.0 dB** |

`stack_real.yaml` already flags the 140 mm assumption as known-stale and deliberately
not re-based (re-basing *raises* a force limit, which is an operator decision). This is
the quantified cost of that decision: it is the rotation channel that goes first, and
it is the rotation channel that saturated J6 on 08-27.

## The wrench filter costs stability margin

`wrench_filter_hz: 25.0` was added on 2026-08-27 to tame contact shock (a compensated
|F| swinging 0→98.6 N inside a few ticks). It does that. But it sits **inside the loop**,
between the F/T pipeline and the law, so it also adds phase lag:

| | 0 dB at k_env — translation | — rotation @ 202.6 mm |
|---|---|---|
| filter off | 38,195 N/m | 32,870 N/m |
| filter at 25 Hz | 26,425 N/m | 22,219 N/m |

**~31% less tolerable contact stiffness, on both channels.** That is a real trade, not a
defect — shock and ring are different failures and the filter buys one with the other.
It is recorded here so the next person to move `wrench_filter_hz` knows it moves both.

## The gate's own margin, at the speed we actually stream

Separately from the ring, CM's `follow_force_params.py` scores the force gate as an
integrator with loop gain `K_gate = 1.5·v_max/d_max`, and asks that `K_gate·T_d` stay
well under 1 (their bands: <0.3 comfortable, <0.6 watch it, else too hot).

Running that tool with `--check 10 15` reproduces our config exactly — `k = 400.0`,
`b = 434.7413`, `k_r = 8.0214`, `b_r = 15.7164`, `M_thr = 1.4`. **Our force block is
CM's 10 N / 15 kg row at the tool's default `--v-max 50`.** But this cell streams at
0.45 m/s:

| v_max | K_gate | K·T_d @ 12 ms | @ 26 ms |
|---|---|---|---|
| 50 mm/s — what the parameters were derived at | 3.0 /s | 0.04 comfortable | 0.08 comfortable |
| **450 mm/s — what this cell streams** | **27.0 /s** | 0.32 watch it | **0.70 too hot** |

CM's tool now warns in its own source that `LOOP_LAG_S` "IS NOT A CONSTANT OF THE
CONTROLLER" — its ZOH half is the input period, so a set built for one cadence has a
different gate margin at another. The same caution applies to the speed: these numbers
were solved at 50 mm/s and we stream at nine times that.

## What to do with this

* **Hand-pushing the arm in free space is safe** and is the intended way to check the
  law — +28 dB at a hand's stiffness. Do that.
* **The instability needs a rigid surface.** Do not validate force control by driving
  the tool into a table, a fixture, or the stand; that is above the 26 kN/m boundary on
  translation and above 22 kN/m on rotation.
* **The oscillation guard is untested.** It was written in response to 08-27 and has
  **never fired**: zero trips across every logged run since, with a peak deviation ever
  recorded of 12 mm / 3.4° against a 40 mm / 15° fence. It is a detector, not a
  stabiliser — it needs 4 velocity reversals inside 0.75 s before it freezes.
* **If a ring is ever wanted deliberately, `b` is the lever and it is not free.** A
  sustained contact costs `F = b·v`, so raising `b` raises what a hand must push with,
  1:1. CM shipped `b = 1500` for its 6 dB and an operator called it sluggish within one
  session; they backed off to 1000. Halving `T_d` buys the same margin at half the
  effort, which is why the delay is the lever worth spending on.

## Reproduce

The model is 20 lines; the numbers above were generated with `m, b, k` read from
`rb_servo_server/config/stack_real.yaml` and `T_d` from our own measured ring. Solve
`∠L(jω) = −180°` for ω, then read `|L|` there:

```
phase(ω) = −atan2(ω·b, k − m·ω²) − atan2(ω, 2π·f_filter) − ω·T_d
gain(ω)  = k_env / |k − m·ω² + jωb| / |1 + jω/(2π·f_filter)|
```

Do **not** use `np.angle` for the phase — it wraps, and the root-find silently misses.

## Related

* `submodules/controller-manager/wiki/findings/follow-contact-ring-is-delay-not-the-gate.md`
* `submodules/controller-manager/wiki/decisions/0039-the-one-force-law.md` — the law is
  `{m, b, k, f_ref}`; the `mode` field we still print at boot was retired upstream
* `docs/reference/pika_tool_geometry.md` — where the 202.642 mm lever comes from

## 2026-09-03 — spring 0 (k = 0), like controller-manager

The tracked stacks now run CM's live follow shape: `k = 0` on every axis with
`force_control.fold_deviation: true` (the deviation is booked into the plan every
tick — see `AGENTS.md` § Force Control). With no spring the loop is
`L(s) = k_env / (s·(m·s + b)) · 1/(1 + s/ω_f) · e^(−T_d·s)` and **b is the only knob**.
`rb_servo_server/tools/force_loop_margin.py` generates every number below.

**Translation, k = 0, 25 Hz filter — gain margin [dB] at k_env 30.7 kN/m / 44.4 kN/m:**

| m | b | T_d 12 ms | T_d 18 ms | T_d 26 ms | hand at 30 mm/s |
|---|---|---|---|---|---|
| 15 | 434.7 (the old row, k removed) | −1.3 / −4.5 | −3.7 / −6.9 | −6.0 / −9.2 | 13 N |
| 12 | 700 | +3.5 / +0.3 | +1.2 / −2.1 | −1.1 / −4.4 | 21 N |
| **12** | **1000 (live, CM's pick)** | **+7.1 / +3.9** | **+4.7 / +1.5** | **+2.4 / −0.8** | **30 N** |
| 12 | 1500 | +11.1 / +7.9 | +8.7 / +5.5 | +6.3 / +3.1 | 45 N |

**Rotation, k = 0, on the pika's 202.6 mm lever (k_env·L²):**

| m | b | T_d 12 ms | T_d 18 ms | T_d 26 ms |
|---|---|---|---|---|
| 0.3 | 10 (CM's row, derived for 140 mm) | −6.2 / −9.4 | −8.6 / −11.8 | −10.9 / −14.1 |
| 0.3 | 20 | +0.5 / −2.7 | −1.8 / −5.0 | −4.1 / −7.3 |
| **0.3** | **30 (live)** | **+4.6 / +1.4** | **+2.2 / −1.0** | **−0.1 / −3.4** |
| 0.3 | 40 | +7.5 / +4.3 | +5.0 / +1.8 | +2.7 / −0.6 |

Which T_d column applies is **not settled**: 12 ms is what the RB3 / fw 8.7.3 ring
implied; CM measured the command→reference echo at 18 ms on fw 8.9.1, and this
cell's own end-to-end command→motion was measured at 22.3 ms on 8.7.3. Read the
18–26 ms columns until it is re-measured on the RB5 boxes.

### The gate is a second loop, and a nonlinear one

The linear margin is necessary, not sufficient. Closing the loop offline through a
rigid wall with the gate in it (`test_force_control.cpp`, `WallLoop`: a plan streamed
at 50 mm/s into a wall, the wrench delayed and low-passed as the servo loop does it,
the projective gate on the plan advance, the overlay on the emitted pose):

| b | gate open τ | 18 ms / 30.7 kN/m | 18 ms / 44 kN/m | 26 ms / 30.7 kN/m |
|---|---|---|---|---|
| 1000 | 0.40 s (old) | 9.2 N p-p ring | 19 N | 19 N |
| 1000 | **1.0 s (live)** | **settles flat** | 12 N | 12 N |
| 1500 | 1.0 s | flat | 6 N | 7 N |

A hand (1 kN/m) and wood (15 kN/m) are quiet at every delay with either b. Two
things follow:

* **`force_gate.open_tau_s` went 0.40 → 1.0.** A fast re-open makes the gate a relay,
  and with no spring the relay feeds the damper ring directly. CM had this exact
  change "on the table" and held it pending a re-measure; the model that predicted
  both cells' rings says take it. Cost: after a contact releases, the plan's authority
  *into* the last contact direction recovers over ~1 s instead of ~0.4 s (retreat and
  sliding are never gated).
* **Against a rigid surface at this cell's delay, b = 1000 is marginal** — the same
  verdict CM reached on its jig (+0.6 dB). The lever that buys margin is the delay;
  b = 1500 is the fallback if a ring shows up on hardware, at 45 N per 30 mm/s by hand.

### What k = 0 converges to

With no spring a streamed contact rests where the plan's gated creep equals the
damper's retreat, `b·v_cmd·g(F/F_max) = F` — ~7.6 N for 50 mm/s at b = 1000 against
the 10 N gate, a **by-product of the stream speed, not a designed number** (CM 0028 §2
measured the same). A designed converged force needs a `ref_force` (CM 0039's one law);
that is the next step, not this one.
