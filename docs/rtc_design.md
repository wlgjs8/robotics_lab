# RTC (Real-Time Chunking) — design plan

Status: **proposal / pre-implementation** (2026-06-22). Owner decision pending.
Scope: add Real-Time Chunking to the openpi pi0.5 deploy lane so a long action
horizon (now **H=24**, retrained) keeps its *commitment* while recovering the
*reactivity* of a short horizon — without retraining.

References: paper arXiv 2506.07339 (Black, Galliker, Levine, NeurIPS 2025);
reference impl `Physical-Intelligence/real-time-chunking-kinetix` (JAX/Kinetix,
`src/model.py::FlowPolicy.realtime_action`); wiki `research/vla/rtc.md`.

---

## 1. Why RTC, why now

- We retrained with `action_horizon = 24` (was 8). Long chunks = strong
  commitment (stays on the already-chosen bolt) but poor reactivity (slow to
  correct), and async replan creates **chunk-boundary discontinuities**.
- Today we paper over the boundary with a velocity **crossfade hack**
  (`--chunk-crossfade-steps`, default 2) — not principled.
- RTC is the principled fix: generate the next chunk while the current one
  executes, **freeze** the actions guaranteed to run during inference latency,
  and **inpaint** the rest to stay consistent. Retraining-free; the paper
  demonstrates it on pi0.5 (H=50). Our deployed model is pi0.5 → directly
  applicable.

**Net effect:** RTC *replaces / subsumes* the crossfade and the time-hold of the
current `enable_async_chunking` with a flow-level inpaint.

---

## 2. The algorithm (grounded in the reference impl)

Per replan, the model receives the **previous chunk** `prev` (H×A, in the model's
own action space) and an integer **inference delay `d`** (the number of leading
actions that WILL execute before this new chunk can take over). It samples the
new chunk with the standard flow Euler loop, modified two ways:

**(a) Hard-freeze the first `d` actions** every denoising step
(`real-time-chunking-kinetix/src/model.py`):
```python
mask = arange(action_chunk_size) < inference_delay      # [H], True for i < d
x_t = where(mask[:, :, None], prev_action_chunk, x_t)   # overwrite frozen prefix
```

**(b) Soft-masked pseudoinverse guidance** added to the velocity each step:
```python
# soft weight schedule over chunk index (start≈d, end≈prefix_attention_horizon)
w = clip((start - 1 - arange(total)) / (end - start + 1) + 1, 0, 1)
# "exp" schedule (default): w = w * expm1(w) / (e - 1)   ; "zeros": hard mask only
# guidance weight per flow time t (data-ness; t=1 clean, t=0 noise)
inv_r2 = (t**2 + (1 - t)**2) / ((1 - t)**2)
c = nan_to_num((1 - t) / t, posinf=max_guidance_weight)
guidance_weight = min(c * inv_r2, max_guidance_weight)   # max_guidance_weight = 5
# error = w * (prev - x1_hat); pinv_correction = VJP of x1_hat wrt x_t applied to error
v_t = v_t + guidance_weight * pinv_correction
```
Params: `num_flow_steps = 5` (n=5), `execute_horizon = s`,
`prefix_attention_horizon = action_chunk_size - s`, `inference_delay = d`.
`prefix_attention_schedule ∈ {exp, zeros}` (exp = soft, the paper's default).

Intuition: the first `d` are pinned to the plan we already committed; the next
`s−d`-ish region is softly pulled toward the old plan (decaying weight) so the
trajectory is continuous; the tail is free to react to the new observation.

---

## 3. Current stack — where RTC hooks (code-grounded)

| Layer | File | Today | RTC change |
|---|---|---|---|
| Model sampling | `~/openpi/src/openpi/models_pytorch/pi0_pytorch.py::sample_actions` (L379–422) | vanilla Euler `x_t += dt·v_t`, `num_steps=10`, time **1→0** (`dt<0`) | add freeze + guidance; accept `prev_action_chunk`, `inference_delay`, schedule, `max_guidance_weight` |
| Serving wrapper | `~/openpi/src/openpi/policies/policy.py::Policy.infer` (L68–106) | `self._sample_actions(dev, obs, **sample_kwargs)` | pull `prev_action_chunk`+`d` from the obs dict, forward as sample_kwargs |
| Wire protocol | websocket obs dict (client → server) | `{left/right_wrist_0_rgb, state, prompt}` | add `prev_action_chunk` (H×14 float32, model units) + `inference_delay` (int) |
| Client | `robotics_lab/policy_runner/policy_runner/openpi_remote.py::_sample_chunk` (L393–420) + async prefetch | sends obs, scales gripper ×100, then crossfade | cache the **raw** returned chunk, send it back as `prev_action_chunk`, set `d` from the scheduler; drop crossfade |

The async prefetch infra (`flow_inference.enable_async_chunking`) already runs the
next inference off the 500 Hz loop — RTC slots on top of it; `d` is exactly "how
many steps of the current chunk remain when the prefetch fires."

---

## 4. Design by layer

### Layer A — openpi model (`pi0_pytorch.sample_actions`)
New signature: `sample_actions(device, observation, noise=None, num_steps=5, *,
prev_action_chunk=None, inference_delay=0, execute_horizon=None,
prefix_attention_schedule="exp", max_guidance_weight=5.0)`. When
`prev_action_chunk is None` → behave exactly as today (zero-risk default).

Porting notes (JAX→torch), the **hard parts**:
1. **Time convention.** Kinetix integrates t: 0→1 (t=1 clean); our loop integrates
   `time`: 1→0 (time=1 noise). Map data-ness `t = 1 − time` everywhere in the
   guidance formulas, and `x1_hat = x_t + (1 − time)·v_t` (estimate of the clean
   action). Verify numerically against the reference on a toy, not by eyeballing.
2. **VJP without JAX.** `pinv_correction = VJP(x1_hat wrt x_t)(error)`. In torch:
   `x_t.requires_grad_(True)`, compute `x1_hat` through `denoise_step`, then
   `torch.autograd.grad(x1_hat, x_t, grad_outputs=error)` (or `torch.func.vjp`).
   Adds one backward per denoise step (×5). Keep the **prefix KV-cache detached**
   (frozen image/lang); differentiate only the suffix/action path. Confirm it
   works under bf16 + the `eager` attention impl (L394).
3. **Freeze overwrite** each step on the first `d` rows of `x_t`.
4. Compute budget: ~2× per-step cost from the backward → ~5 steps with guidance
   vs 10 vanilla; expected net similar latency (paper: 76→97 ms). Measure.

### Layer B — serving protocol (`policy.py`)
`infer()` pops `prev_action_chunk` / `inference_delay` from `obs` **before**
`Observation.from_dict` (they aren't model observation fields), tensorizes them to
the device, and passes as `sample_kwargs`. Backward compatible: absent ⇒ vanilla.
`serve_policy` gains `--rtc-*` defaults (schedule, max_guidance_weight, num_steps)
but per-call `prev_chunk`+`d` come from the client.

### Layer C — robotics_lab client (`openpi_remote.py`)
- **Cache the RAW chunk.** `prev_action_chunk` MUST be in the model's action space:
  ee_local deltas with gripper in **/100** units, **before** `_sample_chunk`'s
  `×100` gripper scale and before any `r_align`/twist conversion. So snapshot
  `result["actions"]` raw (pre-scale) and feed that back. Getting this wrong
  (sending percent-scaled or r_align-rotated actions) silently corrupts guidance.
- **Estimate `d`.** From the async scheduler: `d = remaining steps of the current
  chunk at prefetch time` ≈ `ceil(inference_latency / policy_dt)`, clamped to
  `[0, execute_horizon]`. Start conservative (e.g. d = measured median).
- **Set `execute_horizon = chunk_execute_steps`** (commit length per replan).
- **Remove crossfade dependence** when RTC is on (RTC handles continuity);
  keep crossfade as the fallback path.
- First chunk of a rollout: no `prev` ⇒ vanilla sample (cold start).

---

## 5. Parameters (initial)
`num_flow_steps=5`, `prefix_attention_schedule="exp"`, `max_guidance_weight=5.0`,
`execute_horizon = chunk_execute_steps` (start 8), `d` = measured median delay in
steps (start ~2–4 at 30 Hz). All exposed as flags so they're sweepable.

---

## 6. Testing & rollout (no torch locally → GPU/openpi venv)
1. **Numeric parity (offline, server-side).** With `prev=None`, RTC path == current
   `sample_actions` bit-for-bit (default off). With `prev=`recorded chunk + d=0,
   guidance weight→0 ⇒ ≈ vanilla. Unit-test the schedule/weight formulas against
   values computed from the reference equations.
2. **Continuity metric (offline).** Replay an episode through async RTC vs
   crossfade; measure chunk-boundary jerk (Δtwist at boundaries) — expect RTC ≤
   crossfade with no commitment loss.
3. **sim_dryrun → controller_sim → real** ladder, viser ON
   (`motion-test-viser-always`), behind a `--rtc {off,on}` flag defaulting **off**
   until validated. real_policy gates unchanged.
4. **Latency check** on the deploy GPU (RTX 5090): per-call inference with guidance
   vs vanilla; confirm it fits the prefetch budget.

---

## 7. Risks / open questions
- **VJP cost & correctness** under the pytorch KV-cache + bf16 eager attention is
  the main technical risk (Layer A note 2). De-risk first with a tiny standalone
  torch flow toy before touching pi0_pytorch.
- **Time-convention remap** is the most likely silent bug — validate against the
  Kinetix reference outputs.
- **`d` estimation** couples to the async scheduler; wrong `d` either over-freezes
  (laggy) or under-freezes (boundary jump). Make it measured + logged.
- **gripper dim** rides in the same action vector — it's inpainted too; confirm
  that's benign (gripper is near-piecewise-constant; freezing/guiding it is fine).
- Reference impl is JAX/Kinetix — a port, not a drop-in. Budget for that.

---

## 8. Milestones
- **M0 ✅ DONE** Standalone torch RTC core (`openpi/src/openpi/models_pytorch/rtc.py`):
  `get_prefix_weights`, `guidance_weight`, `clean_estimate`, `freeze_prefix`,
  `rtc_guided_velocity`, `rtc_sample` (reference t-convention). Unit-tested vs the
  reference equations + analytic VJP (`rtc_test.py`, 10 tests).
- **M1 ✅ DONE** Integrated into `pi0_pytorch.sample_actions` behind keyword args
  (`prev_action_chunk=None` ⇒ byte-identical vanilla = parity by construction, the
  original Euler loop is untouched and the RTC branch early-returns). Added
  openpi-convention helpers `rtc_guided_velocity_openpi` / `rtc_sample_openpi`
  (time = 1−t remap; guidance SUBTRACTED because dt<0). 3 extra tests incl. a
  loop-level parity test (guidance-off ≡ vanilla Euler, atol 1e-6).
- **M2 ✅ DONE** Protocol in `policy.py`: `infer` pops RTC fields from the obs dict
  before transforms (`_pop_rtc_kwargs`) and forwards to pytorch `sample_actions`;
  JAX ignores them. **Client-driven** (per-call knobs) → no `serve_policy` change.
  The server returns the MODEL-SPACE chunk as `rtc_raw_actions` (pre output-
  transform) so the client can round-trip it. `policy_rtc_test.py` (3 tests).
- **M3 ✅ DONE** Client in `openpi_remote.py` + `main.py`: caches `rtc_raw_actions`
  (model space, NOT the ×100/r_aligned `actions`), sends it back as
  `prev_action_chunk` with `inference_delay` (clamped to `[0, chunk_execute_steps]`),
  `execute_horizon = chunk_execute_steps`, schedule, max_guidance_weight; disables
  the crossfade when on; `reset_rtc()` for rollout cold-start. CLI `--rtc` (default
  OFF) + `--rtc-inference-delay/--rtc-schedule/--rtc-max-guidance-weight`.
  `test_openpi_remote_rtc.py` (6 tests).
- **M4 TODO (GPU)** sim→controller_sim→real validation + latency + continuity
  metrics. **The remaining unverified pieces need a live model + GPU:** (a) true
  real-model parity (prev=None ⇒ identical actions to before), (b) VJP through the
  KV-cache + bf16 (memory/latency), (c) `d` tuning from measured latency, (d)
  wire `reset_rtc()` into the rollout reset.

Each milestone is independently reviewable; M0/M1 carry the algorithmic risk.
All unit tests run in the openpi venv (`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`); the
robotics_lab client test runs there too (its import chain needs torch).
