"""Read-only wrist RGB comparison: letterbox vs 480-square crop, at 224x224.

Run with ./tools/subscribe_camera_crop.sh. Both transforms use the same decoded
frame; the separate overlay window aligns their common source-center region
before comparing detail and reports the number of samples covering that region.
"""
from __future__ import annotations

import argparse
import math
import time

from .camera_bundle_client import CameraBundleClient, resolve_frame
from .camera_preview import _opencv_gui_error, _print_gui_error, _resize_window_to_image

OUTPUT_SIZE = 224
CROP_SIZE = 480
COMPARE_TITLE = "camera crop comparison (q/ESC closes)"
DIFF_TITLE = "camera crop aligned center diff - red (q/ESC closes)"
CARD_HEIGHT = 374


def _letterbox_geometry(height: int, width: int):
    scale = OUTPUT_SIZE / max(height, width)
    target_w = max(1, round(width * scale))
    target_h = max(1, round(height * scale))
    return target_w, target_h, (OUTPUT_SIZE - target_w) // 2, (OUTPUT_SIZE - target_h) // 2


def letterbox_224(image):
    """Fit a BGR uint8 image inside 224x224 and center it on zero padding."""
    import cv2
    import numpy as np

    height, width = image.shape[:2]
    target_w, target_h, x, y = _letterbox_geometry(height, width)
    interpolation = cv2.INTER_AREA if max(height, width) > OUTPUT_SIZE else cv2.INTER_LINEAR
    resized = cv2.resize(image, (target_w, target_h), interpolation=interpolation)
    result = np.zeros((OUTPUT_SIZE, OUTPUT_SIZE, 3), dtype=np.uint8)
    result[y:y + target_h, x:x + target_w] = resized
    return result


def center_crop_224(image):
    """Take exactly 480x480 source pixels, then area-resize to 224x224.

    An odd excess pixel is left on the right/bottom. Never silently shrink or
    pad the crop when the input is smaller than the requested source region.
    """
    import cv2

    height, width = image.shape[:2]
    if min(height, width) < CROP_SIZE:
        raise ValueError(f"480x480 crop needs source >=480x480 (got {width}x{height})")
    x = (width - CROP_SIZE) // 2
    y = (height - CROP_SIZE) // 2
    return cv2.resize(
        image[y:y + CROP_SIZE, x:x + CROP_SIZE],
        (OUTPUT_SIZE, OUTPUT_SIZE),
        interpolation=cv2.INTER_AREA,
    )


def align_letterbox_center(letterbox, source_shape):
    """Map A's common 480x480 source ROI onto B's 224x224 pixel centers.

    Inverse pixel-center mapping accounts for actual rounded resize dimensions
    and the integer crop origin (including odd-sized sources). Interpolation is
    linear; sampling is clamped to the image content, never the black padding.
    Return the aligned image and the ROI's effective (possibly fractional)
    width/height in A's native output sample grid. Upsampling adds no samples.
    """
    import cv2
    import numpy as np

    height, width = source_shape[:2]
    if min(height, width) < CROP_SIZE:
        raise ValueError("480x480 crop needs source >=480x480")
    target_w, target_h, pad_x, pad_y = _letterbox_geometry(height, width)
    content = letterbox[pad_y:pad_y + target_h, pad_x:pad_x + target_w]
    crop_x, crop_y = (width - CROP_SIZE) // 2, (height - CROP_SIZE) // 2
    centers = (np.arange(OUTPUT_SIZE, dtype=np.float32) + 0.5) * (CROP_SIZE / OUTPUT_SIZE)
    map_x, map_y = np.meshgrid(
        (crop_x + centers) * (target_w / width) - 0.5,
        (crop_y + centers) * (target_h / height) - 0.5,
    )
    aligned = cv2.remap(content, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    return aligned, (CROP_SIZE * target_w / width, CROP_SIZE * target_h / height)


def pixel_diff_overlay(aligned, cropped, gain: float = 1.0):
    """Return a red overlay and max-channel absolute difference (0..255).

    Inputs must already cover the same source ROI at the same display scale.
    A grayscale 50:50 blend supplies spatial context. Red opacity is clipped
    gain*max(abs(A-B))/255. Gain affects display only, never returned differences
    or sample counts. This is a resampling residual, not a quality score.
    """
    import cv2
    import numpy as np

    difference = cv2.absdiff(aligned, cropped).max(axis=2)
    blend = cv2.addWeighted(aligned, 0.5, cropped, 0.5, 0.0)
    gray = cv2.cvtColor(blend, cv2.COLOR_BGR2GRAY)
    base = np.repeat(gray[..., None], 3, axis=2).astype(np.float32)
    strength = np.minimum(difference.astype(np.float32) * gain, 255.0)
    alpha = strength[..., None] / 255.0
    result = base * (1.0 - alpha)
    result[..., 2] += strength
    return np.rint(result).clip(0, 255).astype(np.uint8), difference


def _card(image, name: str, title: str, status: str, detail: str, footer):
    """Place labels outside the 224x224 image, preserving every output pixel."""
    import cv2
    import numpy as np

    card = np.full((CARD_HEIGHT, 248, 3), 25, dtype=np.uint8)
    card[76:300, 12:236] = image
    color = (110, 220, 110) if status == "fresh" else (70, 180, 255)
    footers = [footer] if isinstance(footer, str) else footer
    lines = [
        (name, 18, (225, 225, 225)),
        (title, 38, (225, 225, 225)),
        (f"[{status}] {detail}", 60, color),
    ]
    lines.extend((line, 320 + 20 * i, (195, 195, 195)) for i, line in enumerate(footers))
    for text, y, ink in lines:
        # Long custom camera names must not spill outside their panel.
        width = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)[0][0]
        font_scale = 0.42 * min(1.0, 224 / max(width, 1))
        cv2.putText(card, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale, ink, 1, cv2.LINE_AA)
    return card


def render_views(bundle, cameras: list[str], fresh: bool, diff_gain: float = 4.0):
    """Render both windows from one bundle; absent/invalid crops stay explicit."""
    import cv2
    import numpy as np

    comparison_rows = []
    overlay_rows = []
    for name in cameras:
        frame = resolve_frame(bundle.frames, name) if bundle is not None else None
        pixels = getattr(frame, "pixels", None)
        blank = np.zeros((OUTPUT_SIZE, OUTPUT_SIZE, 3), dtype=np.uint8)
        padded, cropped, aligned, overlay = blank, blank, blank, blank
        status = "MISSING"
        detail = "no RGB frame"
        crop_status = status
        crop_detail = detail
        diff_footer = "diff unavailable"
        crop_footer = "needs source >=480x480"
        aligned_footer = "common center unavailable"
        pad_footer = "whole source / black = zero pad"
        native_footer = "common center unavailable"
        if pixels is not None:
            arr = np.asarray(pixels)
            if arr.ndim == 3 and arr.shape[2] == 3 and arr.dtype == np.uint8 and min(arr.shape[:2]) > 0:
                source = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
                height, width = source.shape[:2]
                padded = letterbox_224(source)
                status = "fresh" if fresh else "STALE"
                detail = f"{width}x{height}"
                crop_status, crop_detail = status, detail
                if min(height, width) >= CROP_SIZE:
                    cropped = center_crop_224(source)
                    aligned, (samples_w, samples_h) = align_letterbox_center(padded, source.shape)
                    overlay, difference = pixel_diff_overlay(aligned, cropped, diff_gain)
                    x, y = (width - CROP_SIZE) // 2, (height - CROP_SIZE) // 2
                    a_samples = samples_w * samples_h
                    b_samples = OUTPUT_SIZE ** 2
                    ratio = b_samples / a_samples
                    footprint = f"{samples_w:g}x{samples_h:g}"
                    # Fractional footprints can occur after integer resize rounding.
                    if not (samples_w.is_integer() and samples_h.is_integer()):
                        footprint = f"~{samples_w:.1f}x{samples_h:.1f}"
                    counts = f"A {a_samples:,.0f} -> B {b_samples:,} samples"
                    gain_text = f"+{b_samples - a_samples:,.0f} samples (+{(ratio - 1) * 100:.1f}%)"
                    pad_footer = ("whole source / black = zero pad", f"center ROI uses {footprint} samples")
                    crop_footer = (f"source ROI x={x}, y={y}, 480x480", counts, gain_text)
                    aligned_footer = (f"A center: {footprint} -> 224x224", "display upsampling; no new detail", counts)
                    native_footer = ("B center: native 224x224 samples",
                                     f"density: {224 / samples_w:.2f}x X / {224 / samples_h:.2f}x Y",
                                     gain_text)
                    diff_footer = (f"mean aligned |A-B|: {difference.mean():.1f}/255",
                                   f"red display gain: {diff_gain:g}x",
                                   f"center sample area: {ratio:.2f}x (+{(ratio - 1) * 100:.1f}%)")
                else:
                    crop_status, crop_detail = "TOO SMALL", detail
            else:
                status = crop_status = "INVALID"
                detail = crop_detail = "expected uint8 RGB"
        comparison_rows.append(cv2.hconcat([
            _card(padded, name, "A: ratio + zero pad -> 224x224", status, detail,
                  pad_footer),
            _card(cropped, name, "B: center 480x480 -> 224x224", crop_status, crop_detail,
                  crop_footer),
        ]))
        overlay_rows.append(cv2.hconcat([
            _card(aligned, name, "A: aligned center (upsampled)", crop_status, crop_detail, aligned_footer),
            _card(cropped, name, "B: same center, native 224x224", crop_status, crop_detail, native_footer),
            _card(overlay, name, "Red = aligned detail difference", crop_status, crop_detail, diff_footer),
        ]))
    return cv2.vconcat(comparison_rows), cv2.vconcat(overlay_rows)


def _positive_float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be a finite positive number")
    return number


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check-gui", action="store_true", help="Check GUI dependencies and exit")
    parser.add_argument("--zmq-endpoint", default="tcp://127.0.0.1:5600")
    parser.add_argument("--topic", default="camera.bundle.policy")
    parser.add_argument("--cameras", default="left_realsense_color,right_realsense_color")
    parser.add_argument("--max-age-ms", type=_positive_float, default=200.0)
    parser.add_argument("--refresh-hz", type=_positive_float, default=30.0)
    parser.add_argument("--scale", type=_positive_float, default=1.0,
                        help="Display zoom only; preprocessing/diff stay at 224x224")
    parser.add_argument("--diff-gain", type=_positive_float, default=4.0,
                        help="Red overlay display gain (default: 4); raw diff/counts are unchanged")
    args = parser.parse_args(argv)
    args.cameras = [name.strip() for name in args.cameras.split(",") if name.strip()]
    if not args.cameras:
        parser.error("--cameras requires at least one camera name")
    return args


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    try:
        import cv2
        import numpy as np  # noqa: F401 - dependency preflight
        import zmq  # noqa: F401 - dependency preflight
    except ImportError as exc:
        _print_gui_error(f"missing runtime dependency: {exc}")
        return 2
    backend, error = _opencv_gui_error(cv2)
    if error is not None:
        _print_gui_error(error, cv2_module=cv2)
        return 2
    if args.check_gui:
        print(f"OpenCV HighGUI backend={backend}")
        return 0

    print(f"[camera-crop] topic={args.topic} cameras={','.join(args.cameras)}", flush=True)
    print("[camera-crop] A: ratio + zero pad -> 224x224; B: center 480x480 -> 224x224", flush=True)
    print("[camera-crop] overlay: A center aligned to B's source ROI and display scale; padding excluded", flush=True)
    print(f"[camera-crop] red gain={args.diff_gain:g}x; sample counts compare the same 480x480 source ROI", flush=True)
    print("[camera-crop] q/ESC or closing either window exits", flush=True)
    period = 1.0 / max(args.refresh_hz, 1.0)
    window_sizes = {}
    created_windows = []
    client = None
    try:
        try:
            for title in (COMPARE_TITLE, DIFF_TITLE):
                cv2.namedWindow(title, cv2.WINDOW_NORMAL)
                created_windows.append(title)
        except Exception as exc:
            _print_gui_error(f"namedWindow failed: {exc}", cv2_module=cv2)
            return 2
        client = CameraBundleClient(args.zmq_endpoint, topic=args.topic, max_age_ms=args.max_age_ms)
        positioned = False
        while True:
            start = time.monotonic()
            bundle = client.poll(timeout_ms=int(period * 1000)) or client.latest()
            fresh = client.is_fresh(bundle) if bundle is not None else False
            comparison, overlay = render_views(bundle, args.cameras, fresh, args.diff_gain)
            for title, view in ((COMPARE_TITLE, comparison), (DIFF_TITLE, overlay)):
                if args.scale != 1.0:
                    size = (max(1, round(view.shape[1] * args.scale)),
                            max(1, round(view.shape[0] * args.scale)))
                    view = cv2.resize(view, size, interpolation=cv2.INTER_NEAREST)
                window_sizes[title] = _resize_window_to_image(cv2, title, view, window_sizes.get(title))
                cv2.imshow(title, view)
            if not positioned:
                cv2.moveWindow(COMPARE_TITLE, 40, 40)
                cv2.moveWindow(DIFF_TITLE, 60 + window_sizes[COMPARE_TITLE][0], 40)
                positioned = True
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                return 0
            if any(cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1 for title in created_windows):
                return 0
            remaining = period - (time.monotonic() - start)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        return 0
    finally:
        if client is not None:
            client.close()
        for title in created_windows:
            try:
                cv2.destroyWindow(title)
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
