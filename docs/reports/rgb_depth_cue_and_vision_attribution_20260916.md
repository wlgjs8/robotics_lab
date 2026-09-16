# How much depth is in the RGB frame, and where does the action head read it? (2026-09-16)

Context: the deployed pika checkpoints are RGB-only (`include_depth=False` on every r6/r7 arm), and
rollouts come up short or dive past the bolt on the approach. This report puts a number on the
depth signal that physically exists in a wrist frame, and measures which part of the frame the
served checkpoint actually uses to set that depth.

Two new instruments, both hardware-free:

- `tools/analyze_depth_pixel_cue.py` — the px-per-mm budget, analytic and measured on recordings.
- `tools/vision_token_attribution.py` — per-vision-token attribution for the served policy.

Run both with the openpi venv (`~/workspace/openpi/.venv/bin/python`), which is where
`openpi_client` and `openpi` live.

## 1. The optical facts this rests on

| quantity | value | source |
| --- | --- | --- |
| wrist D405 colour intrinsics | `fx` 393.3 (left) / 393.8 (right) px @ 640x480 | each episode's own `observations/<arm>/camera_calib/color_intrinsics` |
| model input | 224x224, `resize_with_pad` | `model.py:183`, `config.py:2120` |
| resize scale | x0.35 (width sets it) -> `fx_model` 137.7 px | 224/640 |
| scene rows inside the model input | 28..195 of 224; the rest is black bar | (224 - 480x0.35)/2 = 28 |
| SigLIP variant | `So400m/14` -> patch 14 -> **16x16 = 256 vision tokens per camera** | `pi0.py:107`, read back off the served checkpoint |
| dead tokens | token rows 0, 1, 14, 15 are entirely black bar = **64 of 256 (25%) per camera** | geometry above |
| one vision token | 14 model px = 40 native px = **24.4 mm of scene** at Z = 240 mm | derived |

The "SigLIP is a 16x16-patch ViT, so 224 -> 14x14 = 196 tokens" comment in openpi's
`pi0_config.py:94` is stale for this model. The served checkpoint reports 256 tokens per camera.

## 2. Camera-to-bolt distance, measured rather than assumed

The existing probes (`~/probe_shift_sensitivity.py`) hardcode `Z_BOLT_MM = 200`. That number was
never measured, and the collection rig records no depth stream, so `analyze_depth_pixel_cue.py
measure` recovers it from the imagery instead: on approach frames it pairs images whose wrist
travelled `dz` along its OWN optical axis (rejecting pairs that also swung laterally or tilted),
fits a RANSAC similarity between them over the scene region with the fingers masked out, and
inverts `s = Z/(Z-dz)`.

On the 5 local 2026-09-02 episodes, 43 usable pairs:

- measured magnification **+4.59% for a median dz of 10.7 mm**
- implied **Z = 194 / 240 / 281 mm** (p25 / p50 / p75) over the last 1.5 s before the jaw shuts

So 200 mm is the low end of the approach, not its middle. Everything below is quoted at Z = 240 mm.

## 3. The depth budget: 10 mm of dz in pixels

A camera translating along its optical axis does not translate the image, it SCALES it about the
principal point. 10 mm of approach at Z = 240 mm is a **+4.35% zoom** — and that splits into three
cues that differ by an order of magnitude:

| cue | native px | **model px** | model tokens |
| --- | --- | --- | --- |
| size of the M12 shank (12 mm) | 0.85 | **0.30** | 0.021 |
| size of the bolt head (19 mm) | 1.35 | **0.47** | 0.034 |
| size of the whole bolt (25 mm) | 1.78 | **0.62** | 0.045 |
| radial shift of a feature 150 native px off-centre | 6.52 | **2.28** | 0.163 |
| self-parallax of a 12 mm tall bolt at that radius | 7.89 | **2.76** | 0.197 |

Inverted: **one model pixel of bolt-size change is 15-30 mm of dz.** The scale cue on a bolt is a
third of a pixel for the 10 mm the rollouts are missing — it is not a usable depth channel at
224x224, and no amount of training fixes that.

The radial cue is an order of magnitude larger (1 model px ~ 4.4 mm of dz), but it is zero for a
bolt near the principal point and it is perfectly degenerate with the bolt physically translating.
It is only a depth cue *relative to something at a known depth* — which the fingers are, because
they are rigid to the camera.

Consistent with that, the 2026-09-14 scenezoom probe measured the served policy's z response to a
synthetic scale change at 0.035-0.083 mm per % against a geometric rate of 2.0 mm per %: the model
uses **2-4% of the scale cue**. That is not a training failure, it is the cue being sub-pixel.

## 4. Where the action head actually looks

`tools/vision_token_attribution.py` answers this two independent ways on the same frame. Both were
run on the served `pi05_pika_umi_boltv2r6_anchAB_griponly_devjit_h24_40k` (port 8002), target
`arm=right target=z` — i.e. the commanded approach depth itself.

**occlusion** (live server, no model surgery): ablate one token-aligned patch, re-ask, keep the
SIGNED change in the commanded z. The server draws fresh flow noise per request, so the probe
measures that noise floor first and reports everything in units of it.

- unoccluded commanded z = -10.90 mm, sampling noise 1 sigma = 0.39 mm (10 draws of 6-query means)
- right wrist (the acting arm): a single coherent cluster, peak **+2.89 mm = 7.4 sigma**, sitting on
  the yellow fingertips and the bolt between them; 13/168 scene patches beyond 3 sigma
- left wrist (the idle arm, a control): scattered, max 3.8 sigma, 5/168 beyond 3 sigma — noise

**tokengrad** (checkpoint in-process): `d(commanded z)/d(vision token embedding)` taken straight
through an unrolled flow sampler, deterministic.

- right wrist: the **top 10% of scene tokens hold 51%** of the scene saliency, in the same
  fingertip/bolt cluster the occlusion map found
- the **64 black-bar padding tokens hold 60% of the total** per-token saliency, in a handful of
  isolated tokens — classic attention-sink / register behaviour, not scene evidence. They are
  excluded from the overlays and reported separately.

The two methods agree, which matters because they share no machinery.

## 5. What this says about the short/deep dz

The policy is not reading absolute monocular depth — the cue for that is a third of a pixel. It is
reading the **tip-to-bolt relation**, which is exactly the region both attributions light up, and
which is a *relative* cue: it says "the bolt is this far from my fingers in the image", and it
converts to millimetres only through an assumed Z. A Z error, or a camera-mount difference between
the collection unit and the robot wrist (measured at +13 px of translation on the right arm and a
~5 deg pitch on the left — `config.py` DeviceJitterEgoWrist notes), lands directly on dz.

Implications worth testing, in order of cost:

1. The 25% of vision tokens that are pure padding are free capacity. A no-pad resize (the
   `resize_pad=False` path already in `config.py:551`) spends all 256 tokens on the scene and
   raises the effective `fx_model` on the vertical axis.
2. The scale cue is sub-pixel at 224. At `image_resolution=(392, 392)` (already a config option,
   `config.py:2197`) it is ~1.75x larger and the token grid is 28x28. That is the only lever that
   changes the physics rather than the fitting.
3. `include_depth=True` arms exist in `config.py` and the D405 does produce depth. Whether the
   bolt_v2 corpus carries it is NOT established here: the 5 local 2026-09-02 episodes have only
   `images/realsense_color`, no depth dataset. Check the corpus on the storage server before
   costing an RGB-D arm.

## Reproducing

```bash
V=~/workspace/openpi/.venv/bin/python

# depth budget: analytic, then measured on recordings
$V tools/analyze_depth_pixel_cue.py analytic --z-mm 240
$V tools/analyze_depth_pixel_cue.py measure \
    --episodes ~/Downloads/data_20260902_221844/episode_*.hdf5 --csv /tmp/depth_cue_pairs.csv

# attribution against the live server (nothing to restart)
$V tools/vision_token_attribution.py occlusion --port 8002 --arm right --target z --repeats 6 \
    --episode ~/Downloads/data_20260902_221844/episode_000.hdf5 --out-dir outputs/vision_attribution

# per-vision-token gradient (loads a second copy of the checkpoint, ~9 GB of GPU)
$V tools/vision_token_attribution.py tokengrad --arm right --target z \
    --episode ~/Downloads/data_20260902_221844/episode_000.hdf5 --out-dir outputs/vision_attribution
```

`--live` swaps the recorded frame for the current `camera.bundle.policy` bundle; `--target` also
accepts `xyz`, `rot`, `grip`, `all`.

---

# Addendum, same day: the gripper number was the wrong physical quantity

The attribution above showed the gripper opening is the ONLY non-visual input this checkpoint has
(`mask_velocity_sentinel=True` replaces all 12 velocity dims with a sentinel; measured on the live
8002 server, velocity values of 0 / negated / x10 / +5.0 all move the chunk by less than the
sampling noise, while the gripper dims move it 56-93x). Following that thread found a unit bug.

## The bug

| side | what it reports | source |
| --- | --- | --- |
| collection (pika sense) | **mm of jaw opening** (`get_gripper_distance()`) | `pika/recorder.py:444`, `pika_sdk/pika/sense.py:162` |
| deploy (pika gripper) | **fraction of motor angular travel** x100 | `policy_runner/gripper.py:329`, `min_rad` 0.0 / `max_rad` 1.75 |

The SDK linkage between motor angle and jaw opening is a four-bar, so those are not proportional.
The model was trained on millimetres and was being fed, and was commanding, an angle fraction.

Measured on the real grippers (`tools/measure_gripper_units.py`, 2026-09-16 20:45, both arms swept
0 -> open stop after the runtime's own homing):

- real open stop **left 1.6741 rad / 96.90 mm, right 1.6687 rad / 96.65 mm** — `max_rad = 1.75` sat
  PAST the stop, so a full-open command just pressed on it
- the old percent **over-reported the opening by up to +4.85 mm, peaking at a physical 23 mm jaw**,
  mean +4.40 mm across the 10-45 mm grasp band, crossing zero near 68 mm
- both arms agreed to 0.05 mm, so this is geometry, not a per-arm calibration drift

That band is exactly where the jaw sits while grasping, and exactly where the commanded approach z
has its step (measured 0.31 mm of z per 1 unit of jaw, with a step between 20 and 40). For a 12 mm
M12 shank: the demos' "closed" reading of 15 is 15 mm of jaw; at deploy the same 15 was 11.3 mm,
i.e. below the bolt diameter.

## The fix

`gripper.units` (new, tracked config, default **`sdk_mm`**) selects the unit of every gripper number
exchanged with the policy — state AND action. `sdk_mm` reimplements the SDK's own
`get_gripper_distance()` / `set_gripper_distance()` conversion; `motor_fraction` restores the old
behaviour for reproducing pre-2026-09-16 runs. `max_rad` 1.75 -> **1.66** (just inside the measured
stop). Both backend construction sites are covered: `policy_runner/main.py` and
`policy_runner/gripper_server.py`, which is the one the live rollout uses (`gripper.backend: none`
in `flow_real_realsense.yaml` routes the gripper through the command stream to that server).

Two things fell out of the change:

- the delta path now integrates in UNITS space and converts once. A fixed `delta_rad` was only ever
  correct for a linear unit map.
- `gripper_server`'s `moving` flag compared the RAW command against the measurement. Once a command
  can be clamped (a >96 mm request), the two can never agree and `moving` latched forever. It now
  compares against `PikaSerialGripperBackend.target_units()`, the setpoint actually held.

`scripts/umi_gripper_follow.py` is NOT affected: it passes the Sense encoder angle straight through
to the robot motor, and both devices share the linkage, so teleop was always geometrically right.

## Verified on hardware

Commanding N mm through the fixed backend and reading the jaw back:

```
commanded  left   err     right  err    legacy would have given
   15.0   14.65  -0.35   15.11  +0.11        11.33
   23.0   22.63  -0.37   22.34  -0.66        18.39
   30.0   29.51  -0.49   29.44  -0.56        25.13
   40.0   39.28  -0.72   39.32  -0.68        35.62
   60.0   59.65  -0.35   59.30  -0.70        58.70
   74.0   73.65  -0.35   72.89  -1.11        75.11
```

The residual -0.35..-1.11 mm is the jaw settling slightly short of the commanded angle (~0.01 rad of
servo/friction lag, visible in the sweep as `cmd - reached`), not a unit error.

`python -m unittest discover policy_runner/tests`: 631 tests, 0 failures. (21 errors are
pre-existing in this working tree and reproduce with the change stashed.)

## What is NOT fixed, deliberately

The two devices' zeros still differ. The collection rig's calibration reads **-3.53 (left) /
-2.21 (right)** at its closed stop, the robot reads **+0.02 / +0.11** after homing. Do NOT transplant
the calibration value as a global offset: it comes from the hard human squeeze the calib script asks
for ("꽉 쥐었다 펴기"), while the demos' actual operating minimum is p1 ~ +7.2 / +6.7. Applying -3.5
everywhere would trade a +4.4 mm error for a -3.5 mm one. Align the zero against a COMMON PHYSICAL
REFERENCE instead — the same gauge object or bolt shank between the jaws on both rigs.

Expect a behaviour change on the next run: a commanded 15 used to land at 11.3 mm, so the robot was
gripping HARDER than the demos meant. It will now grip less hard. That is correct, but the extra
tightness may have been propping up grip retention — re-check hold rate.

Re-run `tools/compare_gripper_proprio.py` after the next rollout: the aligned-on-close curves should
now sit on top of the demonstrations. Note that rollout logs from before this change carry the old
unit in `gripper_*_pct` and are not comparable.

---

# Addendum 2: after the unit fix, dz is right and the CLOSE is starved

Runs 21:03 (`--gripper-proprio-source actual`) and 21:04 / 21:06 (`command`), both post-fix.

The closed VALUE now matches: deploy jaw floor 6.8-8.6 mm against 8.17 mm in the demonstrations.
The closing RATE does not:

| | close rate (p50) |
| --- | --- |
| demonstrations | **28.5 mm/s** (p25 21.8 / p75 46.0) |
| deploy left | **3.9 / 5.2 mm/s** |
| deploy right | 35.7 / 10.3 / 16.6 mm/s |

`meas - cmd` is only +0.6/+0.8 mm on the left, so the jaw is tracking its command — **the command
itself is slow**. The chunk log says why.

## The two columns of an anchored chunk have opposite shapes

Over the model's own 19-row horizon, on closing chunks:

```
arm     closing  row0  row3  row6  row12 row18   rows0-3 of the close
left       186   32.1  31.7  30.5  21.0   9.5          4.2%
right      159   40.4  38.8  36.6  31.3  17.3          9.7%
tool-z                                                40.0% / 40.2%
```

A flat ramp would put 16.7% in rows 0-3. So **tool-z is front-loaded (40%) and the gripper is
back-loaded (4-10%)**. The model's own close is healthy — 32.4 (left) / 34.8 (right) mm/s over the
full horizon, against 28.5 mm/s demonstrated — but `--chunk-execute-steps 4` runs rows 0-3 and then
re-plans, so the jaw only ever executes the flat head of the ramp. Every new chunk again says "I
will close, mostly later". A Zeno close.

Executed close rate as a function of the execute window:

```
execute_steps    2     3     4     6     8    10    12    14    16    18
left           6.4   7.2   9.2  15.3  21.0  26.9  31.4  32.4  32.6  32.4
right         18.0  19.1  20.0  21.6  24.1  25.5  27.9  31.0  34.5  34.8
```

Reaching the demonstrations' p25 rate needs rows out to 9 (left) / 7 (right). Raising
`--chunk-execute-steps` to that would make the POSE 2-3x more open-loop, which is the opposite of
what the dz work needed — and the pose is already over-served at 4.

## `--gripper-lookahead-steps` (new, default 0 = off)

The gripper reads chunk row `index + lead` instead of `index`. Sound for the gripper and not for
the pose, because the gripper column is an ABSOLUTE per-frame opening — openpi's
`_anchor_relative_chunk` leaves cols 6/13 untouched — so a later row is a setpoint the model itself
predicted, with no anchor to drift from. Refused at parse time in delta gripper mode, where rows are
increments that must be integrated in order.

### How big a lead, and should `--chunk-execute-steps` move instead?

Measured on the same chunk log. The rate that matters is the LOCAL one over the window actually
executed, `(g[L] - g[L+3]) / 3dt`, not the average from row 0 — the ramp is steep from about row 4
on, so the two differ a lot.

```
lookahead L        0     2     4     6     8    10    12
left  rate (mm/s) 7.2  15.3  28.0  26.7  27.3  30.6  29.8
right rate        19.1 21.1  23.4  24.5  23.2  28.5  31.0
left  lead (mm)   0.0   0.4   1.2   3.1   5.6   9.0  12.6
right lead (mm)   0.0   1.2   2.7   4.3   6.4   8.5  11.2
```

**L = 4 is the pick**: both arms are already at the demonstrated 28.5 mm/s (28.0 / 23.4) with the
smallest lead (1.2 / 2.7 mm). Past 4 the rate is flat — the ramp slope is roughly constant — so a
bigger lead only parks the setpoint further ahead of what the model currently wants. A lead of 4 is
also the clean statement of the idea: the gripper runs one execute window ahead of the pose.

Raising `--chunk-execute-steps` instead is strictly worse:

```
E                  4     6     8    10    12
left  rate (mm/s) 7.2  11.6  18.9  23.0  30.9
right rate       19.1  20.7  23.4  25.2  27.2
pose plan staleness, 4-row window:  lag 4 -> 1.0-1.1 mm, lag 8 -> 1.3-1.5, lag 12 -> 1.7-1.8
(the plan's own 4-row z displacement is 1.2 left / 1.8 right mm, so this is not a small fraction)
```

E would have to reach **12** to match what a lead of 4 gives, and it drags every pose row out with
it: +0.3-0.4 mm of extra staleness at E=8, +0.7 mm at E=12. dz only just started working at E=4, so
spending that on a problem the gripper-only knob solves for free is a bad trade. **Keep
`--chunk-execute-steps 4`.**

Suggested first A/B: `FLOW_INFER_GRIPPER_LOOKAHEAD_STEPS=4` against 0, everything else fixed.

Not yet explained: the left/right asymmetry in the ramp shape (4.2% vs 9.7%) is a model property,
`--gripper-close-bias` is NOT a confounder here: both per-arm defaults are 0.0
(`DEFAULT_GRIPPER_CLOSE_BIAS_LEFT/RIGHT`) and the runs did not set it, so the policy's own opening
is commanded as-is. (A stale comment in `openpi_remote.py` claimed defaults of 2.0/6.0; corrected
2026-09-16.)
