#!/usr/bin/env python3
"""Where does the action head actually look? Per-vision-token attribution for the served pi0.5.

The pika checkpoints are RGB-only, so every millimetre of commanded approach depth has to come
out of the two wrist images. This tool answers *which part of those images* the action expert
used, at the resolution the model actually has: one SigLIP vision token = one 14x14 patch of the
224x224 model input = a 40x40 px tile of the native 640x480 frame (openpi resizes 640->224
with `resize_with_pad`, so the 480 rows land on 168 with 28 rows of black bar top and bottom --
token rows 0, 1, 14 and 15 are ENTIRELY bar, so 64 of every camera's 256 tokens can never carry
scene evidence. They are reported separately rather than drawn on the image.)

Two modes, same grid, same overlays, different costs:

  occlusion  (default) Ablate one patch at a time and re-ask the LIVE server. Needs nothing but
             the running websocket policy on --port; does not touch the checkpoint, does not
             allocate a second copy on the GPU, and measures the thing you actually care about:
             how much the COMMANDED action moves when that patch is removed. 192 scene patches x 2
             cameras at ~50 ms is about 20 s per frame.
             Caveat it handles honestly: the server draws fresh flow noise per request, so a
             repeat of the SAME observation does not return the same chunk. The probe measures
             that noise floor first (--baseline-repeats) and reports every patch in units of it,
             so a heatmap that is entirely inside the noise reads as exactly that.

  tokengrad  Load the checkpoint in-process and take d(target)/d(vision token embedding) straight
             through the flow-matching sampler. This is the literal quantity asked for -- the
             sensitivity of the action expert's output to each vision token that was fed to it --
             and it is deterministic (fixed noise). Costs a second model on the GPU (~9 GB).

The target scalar is chosen, not averaged blindly. `--target z` is the commanded approach depth,
which is the whole reason this tool exists: the chunk is `action_mode=anchored`, so row k's z is
"how far along my own tool z I intend to be by row k", i.e. exactly the dz that is coming out
short or long.

Examples
    # live server, recorded frame, "which patches set the right arm's dz?"
    tools/vision_token_attribution.py occlusion --port 8002 --arm right --target z \
        --episode ~/Downloads/data_20260902_221844/episode_000.hdf5 --frame 1200

    # true per-token gradient on the same frame
    tools/vision_token_attribution.py tokengrad --arm right --target z \
        --config pi05_pika_umi_boltv2r6_anchAB_griponly_devjit_h24_40k \
        --checkpoint ~/workspace/pika_umi_models_v2/boltv2_griponly_devjit_40k/39999 \
        --episode ~/Downloads/data_20260902_221844/episode_000.hdf5 --frame 1200

    # live cameras instead of a recording (needs the camera_server stack up)
    tools/vision_token_attribution.py occlusion --live --arm right --target z

Run it with the openpi venv, which is where `openpi_client` (and, for tokengrad, `openpi`) live:
    ~/workspace/openpi/.venv/bin/python tools/vision_token_attribution.py ...
"""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import sys
import time

import numpy as np

PROMPT = (
    "pick up the black bolt with the right arm and put it in the right box, "
    "then pick up the gray bolt with the left arm and put it in the left box"
)
CAMS = ("left", "right")
OBS_KEY = {"left": "observation/left_wrist_0_rgb", "right": "observation/right_wrist_0_rgb"}
# 14-D action layout (PikaUmiOutputs): [L p3, L rotvec3, L grip, R p3, R rotvec3, R grip]
ARM_BASE = {"left": 0, "right": 7}
TARGET_DIMS = {"z": (2,), "xyz": (0, 1, 2), "rot": (3, 4, 5), "grip": (6,), "all": tuple(range(7))}

MODEL_RES = 224
# openpi pi0.5 uses SigLIP variant "So400m/14" (pi0.py:107), so the patch is 14, NOT the 16 the
# stale comment in pi0_config.py claims: 224/14 = 16 -> 16x16 = 256 vision tokens per camera.
# Verified live against the served checkpoint (`# vision tokens per camera: {...: 256}`).
PATCH = 14
GRID = MODEL_RES // PATCH  # 16


# ------------------------------------------------------------------ geometry


def native_rect(row: int, col: int, w: int, h: int) -> tuple[int, int, int, int] | None:
    """Native-image rectangle covered by model patch (row, col) under resize_with_pad.

    resize_with_pad scales by min(224/w, 224/h) -- for a 640x480 wrist frame that is 224/640, so
    the scene occupies model rows [pad, pad + h*s) and the patch rows outside that window are
    black bar. Returns None for a patch that sees only bar."""
    s = min(MODEL_RES / w, MODEL_RES / h)
    pad_y = (MODEL_RES - h * s) / 2.0
    pad_x = (MODEL_RES - w * s) / 2.0
    y0 = (row * PATCH - pad_y) / s
    y1 = ((row + 1) * PATCH - pad_y) / s
    x0 = (col * PATCH - pad_x) / s
    x1 = ((col + 1) * PATCH - pad_x) / s
    xa, xb = int(np.floor(max(x0, 0))), int(np.ceil(min(x1, w)))
    ya, yb = int(np.floor(max(y0, 0))), int(np.ceil(min(y1, h)))
    if xb - xa < 1 or yb - ya < 1:
        return None
    return xa, ya, xb, yb


# ------------------------------------------------------------- frame sources


def _decode(enc) -> np.ndarray:
    import cv2

    bgr = cv2.imdecode(np.asarray(enc, dtype=np.uint8), cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError("jpeg decode failed")
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def frames_from_episode(path: str, frame: int | None, phase: str) -> tuple[dict, np.ndarray, str]:
    import h5py

    with h5py.File(path, "r") as f:
        G = {s: f[f"observations/{s}"] for s in CAMS}
        grip = {
            s: np.asarray(G[s]["gripper_synced" if "gripper_synced" in G[s] else "gripper"],
                          dtype=np.float64).reshape(-1, 2)[:, 0]
            for s in CAMS
        }
        n = min(len(G["left"]["images/realsense_color"]), len(grip["left"]))
        t = frame
        if t is None:
            arm = phase.split(":")[0] if ":" in phase else "right"
            g = grip[arm]
            # last frame before the jaw shuts = the moment the dz decision has already been made
            closed = np.where(g < 15.0)[0]
            closed = closed[closed > 60]
            t = int(closed[0]) - 10 if len(closed) else n // 2
        t = int(np.clip(t, 0, n - 1))
        imgs = {s: _decode(G[s]["images/realsense_color"][t]) for s in CAMS}
        state = np.zeros(14, dtype=np.float32)  # pose dims are masked server-side for this config
        state[6] = grip["left"][t] / 100.0
        state[13] = grip["right"][t] / 100.0
    return imgs, state, f"{pathlib.Path(path).stem}_t{t}"


def frames_from_live(timeout_s: float) -> tuple[dict, np.ndarray, str]:
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "policy_runner"))
    from policy_runner.camera_bundle_client import CameraBundleClient, resolve_frame

    client = CameraBundleClient(topic="camera.bundle.policy", max_age_ms=500.0)
    deadline = time.monotonic() + timeout_s
    bundle = None
    while time.monotonic() < deadline:
        bundle = client.latest()
        if bundle is not None and bundle.complete:
            break
        time.sleep(0.02)
    if bundle is None or not bundle.complete:
        raise SystemExit("no complete camera bundle on camera.bundle.policy -- is camera_server up?")
    imgs = {}
    for s in CAMS:
        fr = resolve_frame(bundle.frames, f"{s}_realsense.color")
        if fr is None:
            raise SystemExit(f"bundle is missing {s}_realsense.color")
        imgs[s] = np.asarray(fr.pixels)[..., :3]
    return imgs, np.zeros(14, dtype=np.float32), f"live_{bundle.bundle_seq}"


def frames_from_png(left: str, right: str) -> tuple[dict, np.ndarray, str]:
    import cv2

    imgs = {}
    for s, p in (("left", left), ("right", right)):
        bgr = cv2.imread(p, cv2.IMREAD_COLOR)
        if bgr is None:
            raise SystemExit(f"cannot read {p}")
        imgs[s] = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    return imgs, np.zeros(14, dtype=np.float32), pathlib.Path(left).stem


def load_frames(args) -> tuple[dict, np.ndarray, str]:
    if args.live:
        return frames_from_live(args.live_timeout_s)
    if args.png_left:
        return frames_from_png(args.png_left, args.png_right or args.png_left)
    if args.episode:
        return frames_from_episode(args.episode, args.frame, f"{args.arm}:close")
    raise SystemExit("give one of --episode / --png-left / --live")


# ---------------------------------------------------------------- rendering


def render(img, heat, title, out_path, *, diverging=False, vmax=None):
    """Overlay the GRID x GRID patch map on the native frame.

    diverging=False -> magnitude on INFERNO. diverging=True -> signed, blue = the occlusion made
    the target SMALLER (shallower dz), red = larger (deeper), white = inside the noise floor."""
    import cv2

    h, w = img.shape[:2]
    hot = np.where(np.isfinite(heat), heat, 0.0).astype(np.float32)
    top = float(vmax if vmax is not None else max(np.abs(hot).max(), 1e-9))
    m = np.zeros((h, w), np.float32)
    for r in range(heat.shape[0]):
        for c in range(heat.shape[1]):
            rect = native_rect(r, c, w, h)
            if rect is None:
                continue
            x0, y0, x1, y1 = rect
            m[y0:y1, x0:x1] = hot[r, c] / top
    m = cv2.GaussianBlur(m, (0, 0), sigmaX=6.0)

    if diverging:
        t = np.clip(m, -1.0, 1.0)[..., None]
        pos = np.array([220.0, 40.0, 30.0], np.float32)   # RGB red  = target grew
        neg = np.array([30.0, 80.0, 220.0], np.float32)   # RGB blue = target shrank
        white = np.array([255.0, 255.0, 255.0], np.float32)
        cmap = white + np.clip(t, 0, 1) * (pos - white) + np.clip(-t, 0, 1) * (neg - white)
        cmap = cmap.astype(np.uint8)
    else:
        lut = (np.clip(m, 0, 1) * 255).astype(np.uint8)
        cmap = cv2.cvtColor(cv2.applyColorMap(lut, cv2.COLORMAP_INFERNO), cv2.COLOR_BGR2RGB)

    blend = (0.55 * img.astype(np.float32) + 0.45 * cmap.astype(np.float32)).astype(np.uint8)
    panel = cv2.cvtColor(np.concatenate([img, blend], axis=1), cv2.COLOR_RGB2BGR)
    bar = np.zeros((26, panel.shape[1], 3), np.uint8)
    cv2.putText(bar, title, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.imwrite(out_path, np.concatenate([bar, panel], axis=0))


def grid_report(heat: np.ndarray, unit: str) -> str:
    nr, nc = heat.shape
    lines = ["        " + "".join(f"{c:>6d}" for c in range(nc))]
    for r in range(nr):
        cells = "".join("     ." if not np.isfinite(heat[r, c]) else f"{heat[r, c]:6.2f}" for c in range(nc))
        lines.append(f"  r{r:<3d}" + cells)
    lines.append(f"  (units: {unit})")
    return "\n".join(lines)


# ---------------------------------------------------------------- occlusion


def occlusion(args) -> int:
    from openpi_client import websocket_client_policy as wcp

    imgs, state, stem = load_frames(args)
    client = wcp.WebsocketClientPolicy(args.host, args.port)
    dims = TARGET_DIMS[args.target]
    base = ARM_BASE[args.arm]
    cols = [base + d for d in dims]
    r0, r1 = (int(x) for x in args.rows.split(":"))

    def ask_once(mod: dict) -> float:
        """One query -> the scalar target, in mm (or /100 gripper units x 1000)."""
        obs = {
            OBS_KEY["left"]: mod["left"],
            OBS_KEY["right"]: mod["right"],
            "observation/state": state,
            "prompt": args.prompt,
        }
        a = np.asarray(client.infer(obs)["actions"], dtype=np.float64)[r0:r1][:, cols]
        return float(a.mean()) * 1000.0

    def ask(mod: dict) -> float:
        return float(np.mean([ask_once(mod) for _ in range(args.repeats)]))

    # The server draws fresh flow noise per request, so the SAME observation does not return the
    # same chunk. Measure that first, with the same --repeats averaging the patches get, and
    # report every patch as a multiple of it. A map that is entirely inside +/-1 is a map of noise.
    base_draws = np.array([ask(imgs) for _ in range(args.baseline_repeats)])
    ref = float(base_draws.mean())
    sigma = float(base_draws.std(ddof=1)) if len(base_draws) > 1 else 0.0
    if sigma <= 0:
        sigma = 1e-9
    print(f"# frame {stem}   arm {args.arm}   target {args.target}   rows {r0}:{r1}   repeats {args.repeats}")
    print(f"# unoccluded target = {ref:+.3f} mm   sampling noise 1 sigma = {sigma:.3f} mm "
          f"({args.baseline_repeats} draws of {args.repeats}-query means)")

    out_dir = pathlib.Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    heats_mm, summary = {}, {}
    for cam in (CAMS if args.cameras == "both" else (args.cameras,)):
        h, w = imgs[cam].shape[:2]
        fill = np.array(imgs[cam].reshape(-1, 3).mean(axis=0), dtype=np.uint8)
        heat = np.full((GRID, GRID), np.nan, np.float64)
        t0 = time.time()
        for r in range(GRID):
            for c in range(GRID):
                rect = native_rect(r, c, w, h)
                if rect is None:
                    continue
                x0, y0, x1, y1 = rect
                occ = imgs[cam].copy()
                occ[y0:y1, x0:x1] = fill
                mod = dict(imgs)
                mod[cam] = occ
                heat[r, c] = ask(mod) - ref
            print(f"  {cam} row {r + 1}/{GRID}  ({time.time() - t0:.0f}s)", flush=True)
        heats_mm[cam] = heat
        z = heat / sigma
        finite = np.isfinite(z)
        summary[cam] = {
            "max_abs_mm": float(np.nanmax(np.abs(heat))),
            "max_abs_sigma": float(np.nanmax(np.abs(z))),
            "argmax_patch": [int(x) for x in np.unravel_index(np.nanargmax(np.abs(heat)), heat.shape)],
            "patches_over_3_sigma": int(np.sum(np.abs(z[finite]) > 3.0)),
            "patches_scored": int(finite.sum()),
        }
        print(f"\n# {cam} wrist -- signed change in the target when that patch is occluded, in sigma")
        print(grid_report(z, "sigma of the sampling noise; + = target grew"))
        print(f"# max |{summary[cam]['max_abs_mm']:.3f}| mm = {summary[cam]['max_abs_sigma']:.1f} sigma "
              f"at patch {summary[cam]['argmax_patch']}; "
              f"{summary[cam]['patches_over_3_sigma']}/{summary[cam]['patches_scored']} patches beyond 3 sigma\n")
        render(imgs[cam], z, f"{stem} {cam}-wrist occlusion arm={args.arm} target={args.target} (sigma)",
               str(out_dir / f"{stem}_{cam}_occl_{args.arm}_{args.target}.png"), diverging=True)

    payload = {"stem": stem, "mode": "occlusion", "arm": args.arm, "target": args.target,
               "rows": [r0, r1], "repeats": args.repeats, "unoccluded_mm": ref, "sigma_mm": sigma,
               "summary": summary,
               "heat_mm": {k: np.where(np.isfinite(v), v, None).tolist() for k, v in heats_mm.items()}}
    (out_dir / f"{stem}_occl_{args.arm}_{args.target}.json").write_text(json.dumps(payload, indent=1))
    print(f"# wrote {out_dir}/{stem}_*_occl_{args.arm}_{args.target}.png")
    return 0


# ---------------------------------------------------------------- tokengrad


def tokengrad(args) -> int:
    os.environ.setdefault("XLA_PYTHON_CLIENT_PREALLOCATE", "false")
    import einops
    import jax
    import jax.numpy as jnp

    from openpi.models import model as _model
    from openpi.models.pi0 import make_attn_mask
    from openpi.policies import policy_config as _policy_config
    from openpi.training import config as _config

    imgs, state, stem = load_frames(args)
    train_cfg = _config.get_config(args.config)
    policy = _policy_config.create_trained_policy(train_cfg, args.checkpoint)
    model = policy._model  # noqa: SLF001
    print(f"# loaded {args.config} from {args.checkpoint}")

    raw = {
        OBS_KEY["left"]: imgs["left"],
        OBS_KEY["right"]: imgs["right"],
        "observation/state": state,
        "prompt": args.prompt,
    }
    inputs = policy._input_transform(jax.tree.map(lambda x: x, raw))  # noqa: SLF001
    inputs = jax.tree.map(lambda x: jnp.asarray(x)[np.newaxis, ...], inputs)
    obs = _model.Observation.from_dict(inputs)
    obs = _model.preprocess_observation(
        None, obs, train=False, image_keys=list(obs.images.keys()),
        image_resolution=model.image_resolution, photometric_aug=model.photometric_aug,
    )
    img_keys = list(obs.images.keys())
    print(f"# image keys fed to the prefix: {img_keys}")

    tokens0 = {k: model.PaliGemma.img(obs.images[k], train=False)[0] for k in img_keys}
    ntok = {k: int(v.shape[1]) for k, v in tokens0.items()}
    print(f"# vision tokens per camera: {ntok}")

    num_steps = args.num_steps
    dt = -1.0 / num_steps
    noise = jax.random.normal(jax.random.key(args.seed), (1, model.action_horizon, model.action_dim))
    dims = TARGET_DIMS[args.target]
    base = ARM_BASE[args.arm]
    col_idx = [base + d for d in dims]
    cols = jnp.array(col_idx)
    r0, r1 = (int(x) for x in args.rows.split(":"))

    # The model emits NORMALISED actions; the policy's output transform un-normalises them. That
    # map is affine, so one finite difference per column recovers the exact per-dim scale and lets
    # the target (and therefore every gradient below it) be stated in millimetres instead of in
    # normalised units, which are not comparable across dims.
    def out_scale(c: int) -> float:
        zero = np.zeros((model.action_horizon, model.action_dim), np.float64)
        one = zero.copy()
        one[:, c] = 1.0
        # the un-normalise step is keyed on every field the norm stats cover, so hand it a
        # state as well even though only the action scale is wanted back
        st = np.zeros(model.action_dim, np.float64)
        a0 = policy._output_transform({"actions": zero, "state": st})["actions"]  # noqa: SLF001
        a1 = policy._output_transform({"actions": one, "state": st})["actions"]  # noqa: SLF001
        return float(np.mean(np.asarray(a1)[:, c] - np.asarray(a0)[:, c])), float(np.mean(np.asarray(a0)[:, c]))

    try:
        cal = [out_scale(c) for c in col_idx]
        scales = np.array([s for s, _ in cal], dtype=np.float64)
        offsets = np.array([o for _, o in cal], dtype=np.float64)
        units = "mm"
    except Exception as exc:  # noqa: BLE001
        print(f"# could not calibrate output units ({exc}); reporting NORMALISED units", file=sys.stderr)
        scales, offsets = np.ones(len(col_idx)), np.zeros(len(col_idx))
        units = "normalised"
    w_cols = jnp.asarray(scales * 1000.0)  # metres -> mm (gripper dims are /100 units x 1000)
    offset_mm = float(np.mean(offsets)) * 1000.0  # additive part: constant, so it never enters the gradient
    print(f"# output-unit affine for {args.target} dims {col_idx}: scale {scales}, offset {offsets * 1000} mm")

    def forward(tok: dict):
        """Rebuild the prefix from THESE vision tokens and run the flow sampler on it.

        This is pi0.sample_actions with two changes and no others: the image tokens are an
        explicit differentiable input instead of being computed inside embed_prefix, and the
        `jax.lax.while_loop` is unrolled so reverse-mode autodiff can run through it."""
        toks, input_mask, ar = [], [], []
        for name in img_keys:
            t = tok[name]
            toks.append(t)
            input_mask.append(einops.repeat(obs.image_masks[name], "b -> b s", s=t.shape[1]))
            ar += [False] * t.shape[1]
        if obs.tokenized_prompt is not None:
            lang = model.PaliGemma.llm(obs.tokenized_prompt, method="embed")
            toks.append(lang)
            input_mask.append(obs.tokenized_prompt_mask)
            ar += [False] * lang.shape[1]
        prefix_tokens = jnp.concatenate(toks, axis=1)
        prefix_mask = jnp.concatenate(input_mask, axis=1)
        prefix_ar = jnp.array(ar)

        prefix_attn = make_attn_mask(prefix_mask, prefix_ar)
        positions = jnp.cumsum(prefix_mask, axis=1) - 1
        _, kv_cache = model.PaliGemma.llm([prefix_tokens, None], mask=prefix_attn, positions=positions)

        def velocity(x_t, t_scalar):
            suffix, suffix_mask, suffix_ar, adarms = model.embed_suffix(
                obs, x_t, jnp.broadcast_to(jnp.asarray(t_scalar, jnp.float32), 1)
            )
            sa = make_attn_mask(suffix_mask, suffix_ar)
            pa = einops.repeat(prefix_mask, "b p -> b s p", s=suffix.shape[1])
            full = jnp.concatenate([pa, sa], axis=-1)
            pos = jnp.sum(prefix_mask, axis=-1)[:, None] + jnp.cumsum(suffix_mask, axis=-1) - 1
            (_, suffix_out), _ = model.PaliGemma.llm(
                [None, suffix], mask=full, positions=pos, kv_cache=kv_cache, adarms_cond=[None, adarms]
            )
            return model.action_out_proj(suffix_out[:, -model.action_horizon :])

        x_t, t_now = noise, 1.0
        for _ in range(num_steps):
            x_t = x_t + dt * velocity(x_t, t_now)
            t_now += dt
        return x_t

    def target(tok: dict):
        a = forward(tok)[0, r0:r1][:, cols] * w_cols
        return jnp.mean(a) if args.reduce == "mean" else jnp.sqrt(jnp.sum(a * a))

    val, grads = jax.value_and_grad(target)(tokens0)
    print(f"# target ({args.reduce} of arm={args.arm} {args.target} over rows {r0}:{r1}) = "
          f"{float(val) + (offset_mm if args.reduce == 'mean' else 0.0):+.3f} {units}"
          f"   [gradient is taken on the scale term only; the offset is constant]")

    out_dir = pathlib.Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    heats, summary = {}, {}
    for cam in CAMS:
        key = next((k for k in img_keys if k.startswith(cam)), None)
        if key is None:
            continue
        g = np.asarray(grads[key][0], dtype=np.float64)  # (ntok, emb)
        t = np.asarray(tokens0[key][0], dtype=np.float64)
        sal = np.abs(np.sum(g * t, axis=-1)) if args.attribution == "grad_x_input" else np.linalg.norm(g, axis=-1)
        n = sal.shape[0]
        side = int(round(np.sqrt(n)))
        if side * side != n:
            print(f"# {cam}: {n} tokens is not a square grid, skipping", file=sys.stderr)
            continue
        if side != GRID:
            print(f"# {cam}: token grid is {side}x{side}, expected {GRID}x{GRID} -- using {side}",
                  file=sys.stderr)
        heat = sal.reshape(side, side)
        h, w = imgs[cam].shape[:2]
        # resize_with_pad puts black bar on the top/bottom token rows. Those tokens still carry
        # gradient (the action expert uses them as registers), but they are NOT scene evidence, so
        # they are scored separately instead of being painted onto the image.
        is_scene = np.array([[native_rect(r, c, w, h) is not None for c in range(side)] for r in range(side)])
        med = float(np.median(heat[is_scene])) if is_scene.any() else float(np.median(heat))
        heat = heat / max(med, 1e-12)  # multiples of the median SCENE token
        scene = np.where(is_scene, heat, np.nan)
        heats[cam] = heat
        pad_share = float(np.sum(heat[~is_scene]) / max(np.sum(heat), 1e-12)) if (~is_scene).any() else 0.0
        summary[cam] = {
            "max_over_median_scene": float(np.nanmax(scene)),
            "argmax_scene_patch": [int(x) for x in np.unravel_index(np.nanargmax(scene), scene.shape)],
            "padding_tokens": int((~is_scene).sum()),
            "padding_share_of_total_saliency": pad_share,
            "top10_scene_share": float(
                np.sort(scene[is_scene])[::-1][: max(1, int(0.1 * is_scene.sum()))].sum() / np.nansum(scene)
            ),
        }
        print(f"\n# {cam} wrist -- d(target)/d(vision token), in multiples of the median SCENE token")
        print(grid_report(scene, "x median scene token; '.' = resize_with_pad black bar"))
        print(f"# max {summary[cam]['max_over_median_scene']:.2f}x at token "
              f"{summary[cam]['argmax_scene_patch']}; top 10% of scene tokens hold "
              f"{100 * summary[cam]['top10_scene_share']:.0f}% of the scene saliency; "
              f"{summary[cam]['padding_tokens']} padding tokens hold "
              f"{100 * pad_share:.0f}% of the total\n")
        render(imgs[cam], np.where(is_scene, heat, 0.0),
               f"{stem} {cam}-wrist tokengrad arm={args.arm} target={args.target}",
               str(out_dir / f"{stem}_{cam}_grad_{args.arm}_{args.target}.png"), diverging=False)

    (out_dir / f"{stem}_grad_{args.arm}_{args.target}.json").write_text(
        json.dumps({"stem": stem, "mode": "tokengrad", "arm": args.arm, "target": args.target,
                    "rows": [r0, r1], "reduce": args.reduce, "attribution": args.attribution,
                    "target_value": float(val) + (offset_mm if args.reduce == "mean" else 0.0), "target_units": units, "summary": summary,
                    "heat": {k: v.tolist() for k, v in heats.items()}}, indent=1))
    print(f"# wrote {out_dir}/{stem}_*_grad_{args.arm}_{args.target}.png")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p):
        p.add_argument("--arm", choices=["left", "right"], default="right")
        p.add_argument("--target", choices=sorted(TARGET_DIMS), default="z")
        p.add_argument("--rows", default="0:24", help="chunk rows to include, python slice a:b")
        p.add_argument("--prompt", default=PROMPT)
        p.add_argument("--episode", default="")
        p.add_argument("--frame", type=int, default=None, help="default: 10 frames before the jaw shuts")
        p.add_argument("--png-left", default="")
        p.add_argument("--png-right", default="")
        p.add_argument("--live", action="store_true")
        p.add_argument("--live-timeout-s", type=float, default=5.0)
        p.add_argument("--out-dir", default="outputs/vision_attribution")

    o = sub.add_parser("occlusion", help="patch ablation against the live websocket server")
    o.add_argument("--host", default="127.0.0.1")
    o.add_argument("--port", type=int, default=8002)
    o.add_argument("--cameras", choices=["left", "right", "both"], default="both")
    o.add_argument("--repeats", type=int, default=1, help="queries averaged per occluded patch")
    o.add_argument("--baseline-repeats", type=int, default=8, help="queries used for the noise floor")
    common(o)
    o.set_defaults(func=occlusion)

    g = sub.add_parser("tokengrad", help="d(action)/d(vision token) through the flow sampler")
    g.add_argument("--config", default="pi05_pika_umi_boltv2r6_anchAB_griponly_devjit_h24_40k")
    g.add_argument("--checkpoint", default=os.path.expanduser(
        "~/workspace/pika_umi_models_v2/boltv2_griponly_devjit_40k/39999"))
    g.add_argument("--num-steps", type=int, default=10)
    g.add_argument("--seed", type=int, default=0)
    g.add_argument("--reduce", choices=["mean", "norm"], default="mean")
    g.add_argument("--attribution", choices=["grad", "grad_x_input"], default="grad")
    common(g)
    g.set_defaults(func=tokengrad)

    args = ap.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
