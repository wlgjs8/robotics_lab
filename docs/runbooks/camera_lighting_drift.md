# Camera Lighting Drift (time-of-day appearance shift)

Investigating "morning rollouts work, afternoon rollouts degrade" when the only
plausible difference is scene illumination. This runbook is read-only: it adds no
robot command path and does not change the camera rig.

## What the rig can and cannot compensate

Measured from the tracked logs and the live bundle metadata (2026-09-09):

- The wrist D405s run auto-exposure with **no `controls:` block** in the active
  `camera_server/config/dual_realsense_d405_90fps.yaml`, so AE comes from
  `realsense_d405_90fps.json` (`controls-autoexposure-auto: True`,
  `param-autoexposure-setpoint: 1000`).
- `actual_exposure_us` is a hard constant **9947 us** on both cameras, in every
  `logs/camera_quality/*.csv` recorded since 2026-09-02 22:11 (~1.4 M frames
  sampled across 09-03 .. 09-09, morning and afternoon alike). Before the 90 fps
  switch it was an equally constant 31979 us at 30 fps. Both values sit at ~90 %
  of the frame period: **AE is railed at its exposure ceiling and never moves.**
- The color frames report `gain_level = 0` at all times; the depth frames of the
  same stereo module report a per-camera gain (178 left / 209 right on
  2026-09-09 11:03). The stereo module's gain is therefore the only AE degree of
  freedom left, and exposure is not one.

Consequence: room illumination lands in the policy's input pixels essentially
unattenuated. There is no AE headroom to absorb a morning-to-afternoon change.

Because the deployed launch line uses `crop_frac=0.0` and no
`FLOW_INFER_K_NORMALIZE`, the policy input **is** the raw 640x480 bundle color
frame (openpi resizes with pad to 224 server-side). Raw bundle captures are
therefore directly comparable to what the model sees.

## Gap this closes

No tracked log carries image brightness. `logs/camera_quality/*.csv` has blur,
edge energy, exposure and gain but no photometry; `outputs/sweep/*.jsonl` has no
camera fields at all. The per-inference photometry that `flow-infer` computes
(`camera_diagnostics.rgb_image_metrics`) stays in memory and is never persisted.
So a time-of-day claim cannot be checked against history — it has to be recorded.

## 1. Continuous photometry + reference frames

`tools/camera_light_recorder.py` subscribes to `camera.bundle.policy` and reads
the shm rings (same posture as `rb_gui`'s camera-quality monitor: no commands, no
device access). It logs per-camera luminance percentiles, per-channel means,
clipping fractions, focus energy, and the AE metadata, and writes bounded
full-resolution JPEGs.

```bash
# all-day session: metrics at 5 Hz, one image pair per second, 14 GiB budget
setsid nohup .venv/bin/python tools/camera_light_recorder.py \
  --label day --duration-min 540 --metrics-hz 5 --image-hz 1.0 --max-gb 14 \
  > logs/light_recorder_day.log 2>&1 < /dev/null &
```

Cost measured on this host: ~0.26 MB/s (~0.95 GB/h) at `--image-hz 1.0`; drop to
`--image-hz 0.2` for an unattended multi-day run. The session stops on its own at
`--duration-min`, at `--max-gb`, or when free space falls under `--min-free-gb`;
`SIGTERM`/`SIGINT` also close it cleanly and write `summary.json`.

Output: `logs/light/<label>_<stamp>/{session.json,metrics.csv,images/,summary.json}`.

## 2. Paired reference captures (removes the wrist-pose confound)

The cameras are wrist-mounted, so a raw AM-vs-PM luminance difference is partly a
difference in where the arms were pointing. For a controlled comparison, park both
arms at the rollout init pose and take a short capture at each time of day:

```bash
.venv/bin/python tools/camera_light_recorder.py --label ref_am --duration-min 1 \
  --image-hz 5 --note "init pose, blinds as usual"
# ... same pose, same note, in the afternoon:
.venv/bin/python tools/camera_light_recorder.py --label ref_pm --duration-min 1 --image-hz 5
```

## 3. Compare

```bash
.venv/bin/python tools/analyze_camera_light.py logs/light/day_*/ --am 9-12 --pm 13-19
.venv/bin/python tools/analyze_camera_light.py logs/light/ref_am_* logs/light/ref_pm_*
```

The per-hour table and the AM/PM delta cover `lum_mean` / `lum_p05` / `lum_p95`
(illumination and dynamic range), `R/G/B` means (color temperature — a warm
afternoon sun shifts these apart), `clip_hi_frac` (blown highlights destroy
texture the policy needs), `focus_gradient_energy`, and the AE state
(`actual_exposure_us`, `gain_level`, `depth_gain_level`).

Reference points: the 30 fps AE baseline measured by `tools/probe_d405_gain.py`
was ~111/115 mean RGB; the 2026-09-09 11:00 idle scene sits at
`lum_mean` 123.8 (left) / 124.8 (right), `clip_hi_frac` ~0.04, depth gain
178/209. Static-scene noise over 2 minutes is +/-0.5 luminance counts, so a
time-of-day shift of even a few counts is resolvable.

## 4. Capture the exact policy input during rollouts

`flow-infer` already has a bounded JPEG dump of the post-preprocessing images.
Prepend to the tracked launch line (see the daily `~/NNNN VLA` notes):

```bash
FLOW_INFER_DIAGNOSTIC_IMAGES=/home/plaif/workspace/robotics_lab/logs/flow_obs_am \
FLOW_INFER_DIAGNOSTIC_IMAGE_MAX_BUNDLES=20000 \
... ./tools/flow_infer_sweep_run.sh <model> --proprio-mode velocity_grip
```

Each run lands in its own `logs/flow_obs_am/run_<timestamp>` child (the explicit
path is the parent; `auto` names it `logs/flow_obs_<timestamp>` instead), and the
resolved path is printed at startup as `[flow-infer] rgb snapshots -> ...`. This
matters because filenames carry only `bundle_seq`, which restarts with
camera_server: before 2026-09-09 two consecutive rollouts interleaved 1014 pairs
into one fixed directory with no run boundary in the names.

The writer is best-effort on a background thread with a bounded queue, so it
cannot stall inference; it stops at the bundle cap. Measured on the 2026-09-09
12:00 rollouts: 1014 pairs for ~1010 inferences at ~9.8 Hz (no drops), 276 MB,
and the same-window luminance distribution as the standalone recorder
(94.0/119.6/131.7 vs 93.7/120.9/134.7 min/median/max) — the two capture paths see
the same pixels.

## 5. If the shift is confirmed

Two independent remedies, in order of reviewability:

1. **Fix the illumination**, not the camera: blackout the daylight path and light
   the cell with the same constant fixture used during data collection. This is
   the only remedy that also removes the color-temperature shift.
2. **Pin the camera controls.** The `controls:` block (`auto_exposure: 0`,
   `ir_exposure_us`, `ir_gain`) reaches the D405 stereo module since the
   2026-08-28 sensor-selection fix in `camera_server/src/camera/realsense_device.cpp`
   (it silently reached nothing before). `tools/probe_d405_gain.py` has the
   brightness/temporal-noise cost per gain step at 90 fps. Pinning removes the
   AE gain drift but not the underlying scene change, and it must be re-checked
   against the training distribution brightness before a rollout.

Whichever is chosen, the decisive test is not the image statistic but the policy
response: feed paired AM/PM frames of the same scene to the serving checkpoint
with a fixed proprio state and compare the action chunks against the
within-condition noise floor (`tools/ab_policy_30v90.py` is the precedent).
