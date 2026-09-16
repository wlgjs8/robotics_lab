#!/usr/bin/env python3
"""How many PIXELS is 10 mm of wrist-camera depth worth?

The deployed pika policy sees RGB only (`include_depth=False` on every r6/r7 checkpoint), so
every millimetre of approach depth it commands has to be read out of a monocular 224x224 wrist
frame. This tool quantifies the signal that is physically there, so "the model cannot see dz" is
a measurement instead of an intuition.

A camera translating dz ALONG its optical axis does not translate the image, it SCALES it about
the principal point by s = Z / (Z - dz). That single fact splits into three separate cues, and
they differ by more than an order of magnitude:

  1. size cue      a feature of physical extent L grows by  L*fx*dz / (Z*(Z-dz))  px.
                   This is the cue for "how far is the bolt", and for a bolt it is TINY.
  2. radial cue    a feature sitting r px off the principal point moves radially by r*dz/(Z-dz) px.
                   Much bigger, but it is zero at the image centre and it is perfectly
                   degenerate with the bolt physically translating -- it is only a depth cue
                   relative to something at a KNOWN depth.
  3. self-parallax the top of a bolt lying on the table is `bolt_h` mm closer than the table it
                   sits on, so its silhouette is magnified relative to its own footprint. This is
                   the "the bolt looks 3D" cue and it is the smallest of the three.

Everything is reported twice: at the native 640x480 capture, and at the 224x224 the model
actually gets (openpi `resize_with_pad` 640->224 = x0.35, so 480 rows land on 168 with 28 rows of
black bar top and bottom -- `config.py:2120`). The model-input number is the one that matters;
the native number is only there to explain why a human looking at the recording disagrees.

Two modes:

  analytic   closed form from a measured K. No data needed.
               tools/analyze_depth_pixel_cue.py analytic --z-mm 200

  measure    the same quantity MEASURED on recorded pika episodes: find approach frames, pair
             them so the wrist travelled ~dz along its own optical axis, and recover the image
             scale change between the pair with a RANSAC similarity fit. That measured scale
             gives Z_bolt without ever needing a depth camera (Z = dz*s/(s-1)) and validates the
             analytic table on the actual scene texture.
               tools/analyze_depth_pixel_cue.py measure --episodes ~/Downloads/data_*/episode_*.hdf5

Intrinsics are never guessed: `measure` reads each episode's own recorded
`observations/<arm>/camera_calib/color_intrinsics`, and `analytic` needs --fx unless you accept
the printed default, which is the measured inference-unit value.
"""

from __future__ import annotations

import argparse
import json
import math
import sys

import numpy as np

# Measured inference wrist units (tools/read_wrist_intrinsics.py, 640x480 colour).
FX_DEFAULT = 393.3
NATIVE_W = 640
MODEL_RES = 224
# openpi resize_with_pad(640x480 -> 224x224): the WIDTH sets the scale, the height is padded.
MODEL_SCALE = MODEL_RES / NATIVE_W
# pi0.5 uses SigLIP "So400m/14" (openpi pi0.py:107): patch 14, so 224/14 = 16 -> 16x16 = 256
# vision tokens per camera. (The "16x16 patch / 14x14 tokens" comment in openpi's pi0_config.py
# is stale -- 256 tokens per camera was read back off the served checkpoint 2026-09-16.)
SIGLIP_PATCH = 14

# Bolt geometry, measured 2026-09-14 (see sim-bolt-threads-button-head memory):
#   gray  = ISO 7380-1 M12x20 button head, head dia 20.5 mm, 23 g
#   black = ISO 4762  M12x25 socket head,  head dia 18.0 mm, 35 g
FEATURES_MM = {
    "bolt shank dia (M12)": 12.0,
    "bolt head dia (~19)": 19.0,
    "bolt overall length": 25.0,
    "jaw opening (open)": 74.0,
}


def cues(fx: float, z_mm: float, dz_mm: float, bolt_h_mm: float, radius_px: float) -> dict:
    """All three depth cues at one operating point. `radius_px` is native-resolution."""
    s = z_mm / (z_mm - dz_mm)  # image magnification for dz of approach
    out = {
        "scale_factor": s,
        "scale_percent": 100.0 * (s - 1.0),
        "size_cue_px": {},
        "radial_cue_px_native": radius_px * (s - 1.0),
        "self_parallax_px_native": 0.0,
    }
    for name, L in FEATURES_MM.items():
        out["size_cue_px"][name] = L * fx * dz_mm / (z_mm * (z_mm - dz_mm))
    # self-parallax: the bolt top (Z - bolt_h) vs its own footprint (Z), radially at `radius_px`
    out["self_parallax_px_native"] = radius_px * (z_mm / (z_mm - bolt_h_mm) - 1.0)
    return out


def to_model(px_native: float) -> float:
    return px_native * MODEL_SCALE


def analytic(args) -> int:
    fx, z, dz = args.fx, args.z_mm, args.dz_mm
    fx_model = fx * MODEL_SCALE
    c = cues(fx, z, dz, args.bolt_height_mm, args.radius_px)

    print(f"# camera fx {fx:.1f} px @ {NATIVE_W}x480   ->  model fx {fx_model:.1f} px @ {MODEL_RES}x{MODEL_RES}")
    print(f"# bolt at Z = {z:.0f} mm from the wrist camera; approach step dz = {dz:.1f} mm")
    print(f"# 1 native px = {z/fx:.3f} mm of scene   |   1 model px = {z/fx_model:.3f} mm of scene")
    print(f"# 1 vision token ({SIGLIP_PATCH} model px, {SIGLIP_PATCH/MODEL_SCALE:.0f} native px) = "
          f"{SIGLIP_PATCH*z/fx_model:.1f} mm of scene at that depth")
    print()
    print(f"dz = {dz:.1f} mm scales the whole image by {c['scale_percent']:+.2f}%")
    print()
    print(f"{'cue':<34}{'native px':>12}{'model px':>12}{'model tokens':>15}")
    print("-" * 73)
    for name, px in c["size_cue_px"].items():
        m = to_model(px)
        print(f"{'size: ' + name:<34}{px:>12.2f}{m:>12.3f}{m/SIGLIP_PATCH:>15.4f}")
    r = c["radial_cue_px_native"]
    print(f"{f'radial: feature at r={args.radius_px:.0f} px':<34}{r:>12.2f}{to_model(r):>12.3f}{to_model(r)/SIGLIP_PATCH:>15.4f}")
    p = c["self_parallax_px_native"]
    print(f"{f'self-parallax: {args.bolt_height_mm:.0f} mm tall bolt':<34}{p:>12.2f}{to_model(p):>12.3f}{to_model(p)/SIGLIP_PATCH:>15.4f}")
    print()
    print("# inverse: how much dz does ONE model pixel of each cue buy you?")
    for name, L in FEATURES_MM.items():
        # solve L*fx_model*dz/(z*(z-dz)) = 1  ->  dz = z^2 / (L*fx_model + z)
        dz1 = z * z / (L * fx_model + z)
        print(f"#   size of {name:<28} 1 model px = {dz1:7.2f} mm of dz")
    dz1 = z / (to_model(args.radius_px)) if args.radius_px > 0 else float("inf")
    print(f"#   radial at r={args.radius_px:.0f} native px{'':<14} 1 model px = {dz1:7.2f} mm of dz")

    if args.json:
        payload = {"fx": fx, "fx_model": fx * MODEL_SCALE, "z_mm": z, "dz_mm": dz, **c}
        payload["size_cue_model_px"] = {k: to_model(v) for k, v in c["size_cue_px"].items()}
        payload["radial_cue_model_px"] = to_model(c["radial_cue_px_native"])
        payload["self_parallax_model_px"] = to_model(c["self_parallax_px_native"])
        with open(args.json, "w") as fh:
            json.dump(payload, fh, indent=1)
        print(f"\n# wrote {args.json}", file=sys.stderr)
    return 0


# ---------------------------------------------------------------- measure mode


def _quat_z_axis(q: np.ndarray) -> np.ndarray:
    """Tool +z (= wrist camera optical axis to within 1.7 deg, calibration/T_tcp_cam.npy) from
    a pika pose quaternion stored as (qx, qy, qz, qw)."""
    x, y, z, w = (float(v) for v in q)
    return np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])


def _decode(enc) -> np.ndarray:
    import cv2

    bgr = cv2.imdecode(np.asarray(enc, dtype=np.uint8), cv2.IMREAD_COLOR)
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def _scene_mask(shape) -> np.ndarray:
    """Everything except the gripper fingers, which are RIGID to the camera and therefore do not
    scale with dz -- including them biases the fitted scale toward 1."""
    import cv2  # noqa: F401

    h, w = shape[:2]
    m = np.full((h, w), 255, np.uint8)
    m[int(0.55 * h) :, : int(0.30 * w)] = 0
    m[int(0.55 * h) :, int(0.70 * w) :] = 0
    return m


def _fit_similarity(img_a, img_b, nfeat: int):
    """RANSAC similarity (scale+rot+translation) from a -> b over the scene region.
    Returns (scale, n_inliers, median_radius_px) or None."""
    import cv2

    ga = cv2.cvtColor(img_a, cv2.COLOR_RGB2GRAY)
    gb = cv2.cvtColor(img_b, cv2.COLOR_RGB2GRAY)
    mask = _scene_mask(ga.shape)
    orb = cv2.ORB_create(nfeatures=nfeat, scaleFactor=1.15, nlevels=12, fastThreshold=7)
    ka, da = orb.detectAndCompute(ga, mask)
    kb, db = orb.detectAndCompute(gb, mask)
    if da is None or db is None or len(ka) < 12 or len(kb) < 12:
        return None
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(da, db)
    if len(matches) < 12:
        return None
    pa = np.float32([ka[m.queryIdx].pt for m in matches])
    pb = np.float32([kb[m.trainIdx].pt for m in matches])
    M, inl = cv2.estimateAffinePartial2D(pa, pb, method=cv2.RANSAC, ransacReprojThreshold=1.5,
                                         maxIters=4000, confidence=0.995)
    if M is None or inl is None or int(inl.sum()) < 10:
        return None
    scale = float(math.hypot(M[0, 0], M[1, 0]))
    keep = inl.ravel().astype(bool)
    return scale, int(keep.sum()), float(np.median(np.linalg.norm(pa[keep] - np.array([320.0, 240.0]), axis=1)))


def measure(args) -> int:
    import h5py

    rows = []
    for path in args.episodes:
        try:
            f = h5py.File(path, "r")
        except OSError as exc:
            print(f"# skip {path}: {exc}", file=sys.stderr)
            continue
        with f:
            for arm in (("left", "right") if args.arm == "both" else (args.arm,)):
                g = f[f"observations/{arm}"]
                intr = g["camera_calib/color_intrinsics"].attrs
                fx = float(intr["fx"])
                pose = np.asarray(g["pose_synced" if "pose_synced" in g else "pose"], dtype=np.float64)
                grip = np.asarray(g["gripper_synced" if "gripper_synced" in g else "gripper"],
                                  dtype=np.float64).reshape(-1, 2)[:, 0]
                imgs = g["images/realsense_color"]
                n = min(len(imgs), len(pose), len(grip))

                # approach frames: jaw open now, closes within 1.5 s (90 Hz capture)
                cand = []
                for t in range(30, n - 30, args.stride):
                    closes = np.where(grip[t : min(t + 135, n)] < 15.0)[0]
                    if grip[t] > 40.0 and len(closes) and closes[0] > 5:
                        cand.append(t)
                if not cand:
                    continue

                axis_all = np.array([_quat_z_axis(pose[t, 3:7]) for t in range(n)])
                for t in cand[:: max(1, len(cand) // max(1, args.max_pairs_per_arm))]:
                    # walk forward until the camera has advanced ~dz ALONG ITS OWN optical axis
                    u = axis_all[t]
                    t2 = None
                    for k in range(t + 1, min(t + 180, n)):
                        d = float(np.dot(pose[k, :3] - pose[t, :3], u)) * 1000.0
                        if d >= args.dz_mm:
                            t2 = k
                            break
                    if t2 is None:
                        continue
                    dz = float(np.dot(pose[t2, :3] - pose[t, :3], u)) * 1000.0
                    # reject pairs that also swung: pure-approach pairs only
                    lat = float(np.linalg.norm((pose[t2, :3] - pose[t, :3]) * 1000.0 - dz * u))
                    tilt = math.degrees(math.acos(np.clip(float(np.dot(u, axis_all[t2])), -1, 1)))
                    if lat > args.max_lateral_mm or tilt > args.max_tilt_deg:
                        continue
                    try:
                        fit = _fit_similarity(_decode(imgs[t]), _decode(imgs[t2]), args.orb_features)
                    except Exception as exc:  # noqa: BLE001
                        print(f"# fit failed {path} {arm} {t}: {exc}", file=sys.stderr)
                        continue
                    if fit is None:
                        continue
                    s, ninl, rmed = fit
                    if s <= 1.0005:  # no measurable magnification -> Z estimate is meaningless
                        continue
                    z_implied = dz * s / (s - 1.0)
                    rows.append({
                        "episode": path.split("/")[-1], "arm": arm, "t": int(t), "t2": int(t2),
                        "dz_mm": dz, "lateral_mm": lat, "tilt_deg": tilt, "fx": fx,
                        "scale": s, "inliers": ninl, "median_radius_px": rmed,
                        "z_implied_mm": z_implied,
                    })

    if not rows:
        print("no usable approach pairs found (need pika HDF5 with pose_synced + realsense_color)",
              file=sys.stderr)
        return 1

    z = np.array([r["z_implied_mm"] for r in rows])
    sc = np.array([r["scale"] for r in rows])
    dz = np.array([r["dz_mm"] for r in rows])
    fx = float(np.median([r["fx"] for r in rows]))
    keep = (z > 40) & (z < 900)  # outside this the similarity fit latched onto the fingers/background
    zk, sck, dzk = z[keep], sc[keep], dz[keep]

    print(f"# pairs fitted {len(rows)}  usable {int(keep.sum())}   fx(median) {fx:.1f}")
    print(f"# measured image magnification per pair: median {np.median(sck):.4f}"
          f"  ({100*(np.median(sck)-1):+.2f}% for dz median {np.median(dzk):.1f} mm)")
    print(f"# IMPLIED camera->scene depth Z: p25 {np.percentile(zk,25):.0f}  p50 {np.median(zk):.0f}"
          f"  p75 {np.percentile(zk,75):.0f} mm")
    print()
    z50 = float(np.median(zk))
    args2 = argparse.Namespace(fx=fx, z_mm=z50, dz_mm=args.dz_mm,
                               bolt_height_mm=args.bolt_height_mm, radius_px=args.radius_px,
                               json=args.json)
    print(f"# ---- analytic table at the MEASURED Z = {z50:.0f} mm ----")
    analytic(args2)

    if args.csv:
        import csv

        with open(args.csv, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print(f"\n# wrote {args.csv}", file=sys.stderr)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p):
        p.add_argument("--dz-mm", type=float, default=10.0, help="approach step to evaluate (default 10)")
        p.add_argument("--bolt-height-mm", type=float, default=12.0,
                       help="height of the bolt lying on the table (M12 shank dia = 12)")
        p.add_argument("--radius-px", type=float, default=150.0,
                       help="native-res distance of the bolt from the principal point, for the radial cue")
        p.add_argument("--json", default="", help="write the analytic numbers here")

    a = sub.add_parser("analytic", help="closed form from a measured K")
    a.add_argument("--fx", type=float, default=FX_DEFAULT)
    a.add_argument("--z-mm", type=float, default=200.0)
    common(a)
    a.set_defaults(func=analytic)

    m = sub.add_parser("measure", help="measure it on recorded pika episodes")
    m.add_argument("--episodes", nargs="+", required=True)
    m.add_argument("--arm", choices=["left", "right", "both"], default="both")
    m.add_argument("--stride", type=int, default=5)
    m.add_argument("--max-pairs-per-arm", type=int, default=25)
    m.add_argument("--max-lateral-mm", type=float, default=4.0)
    m.add_argument("--max-tilt-deg", type=float, default=3.0)
    m.add_argument("--orb-features", type=int, default=1500)
    m.add_argument("--csv", default="")
    common(m)
    m.set_defaults(func=measure)

    args = ap.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
