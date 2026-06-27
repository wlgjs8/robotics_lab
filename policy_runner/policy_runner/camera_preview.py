"""Live preview window of the camera frames the flow policy consumes.

Subscribes to the SAME camera_server ZMQ bundle + shared-memory ring as the
flow-infer runtime and resolves frames with the SAME resolve_frame() mapping,
so the window shows exactly what the policy sees (ZMQ PUB fans out to multiple
subscribers; the shm ring is read-only — no interference with inference).

Run standalone:
    PYTHONPATH=policy_runner python3 -m policy_runner.camera_preview \
        --cameras left_realsense_color,right_realsense_color

or let flow-infer spawn/terminate it with --camera-preview.
"""
from __future__ import annotations

import argparse
import sys
import time

from .camera_bundle_client import CameraBundleClient, resolve_frame

WINDOW_TITLE = "policy camera preview (q/ESC closes)"
DEFAULT_PANEL_SIZE = (640, 480)


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--zmq-endpoint", default="tcp://127.0.0.1:5600")
    parser.add_argument("--topic", default="camera.bundle")
    parser.add_argument(
        "--cameras",
        default="left_realsense_color,right_realsense_color",
        help="Comma-separated dataset camera names (resolve_frame mapping applies)",
    )
    parser.add_argument("--max-age-ms", type=float, default=500.0)
    parser.add_argument("--refresh-hz", type=float, default=15.0)
    parser.add_argument("--scale", type=float, default=1.0, help="Display scale factor")
    return parser.parse_args(argv)


def _image_window_size(image) -> tuple[int, int]:
    """Return OpenCV resizeWindow width/height for an HWC image."""

    return int(image.shape[1]), int(image.shape[0])


def _resize_window_to_image(
    cv2_module,
    title: str,
    image,
    last_size: tuple[int, int] | None,
) -> tuple[int, int]:
    size = _image_window_size(image)
    if size != last_size:
        cv2_module.resizeWindow(title, size[0], size[1])
    return size


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    try:
        import cv2
        import numpy as np
    except ImportError as exc:  # pragma: no cover - environment guard
        print(f"camera_preview requires cv2/numpy: {exc}", file=sys.stderr)
        return 2

    cameras = [name.strip() for name in str(args.cameras).split(",") if name.strip()]
    client = CameraBundleClient(
        args.zmq_endpoint, topic=args.topic, max_age_ms=args.max_age_ms
    )
    period = 1.0 / max(float(args.refresh_hz), 1.0)
    placeholder_shape = (DEFAULT_PANEL_SIZE[1], DEFAULT_PANEL_SIZE[0], 3)
    last_window_size: tuple[int, int] | None = None
    cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)
    try:
        while True:
            loop_start = time.monotonic()
            bundle = client.poll(timeout_ms=int(period * 1000)) or client.latest()
            fresh = client.is_fresh(bundle) if bundle is not None else False
            panels = []
            for name in cameras:
                frame = resolve_frame(bundle.frames, name) if bundle is not None else None
                pixels = getattr(frame, "pixels", None)
                if pixels is None:
                    panel = np.zeros(placeholder_shape, dtype=np.uint8)
                    status = "MISSING"
                    resolution = f"{DEFAULT_PANEL_SIZE[0]}x{DEFAULT_PANEL_SIZE[1]}"
                else:
                    panel = cv2.cvtColor(np.asarray(pixels), cv2.COLOR_RGB2BGR)
                    status = "fresh" if fresh else "STALE"
                    resolution = f"{panel.shape[1]}x{panel.shape[0]}"
                color = (0, 255, 0) if status == "fresh" else (0, 0, 255)
                cv2.putText(panel, f"{name} {resolution} [{status}]", (8, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                panels.append(panel)
            if panels:
                height = max(panel.shape[0] for panel in panels)
                padded = [
                    cv2.copyMakeBorder(panel, 0, height - panel.shape[0], 0, 0,
                                       cv2.BORDER_CONSTANT, value=(0, 0, 0))
                    for panel in panels
                ]
                mosaic = cv2.hconcat(padded)
                if args.scale != 1.0:
                    mosaic = cv2.resize(mosaic, None, fx=args.scale, fy=args.scale)
                if bundle is not None:
                    cv2.putText(mosaic, f"bundle_seq={bundle.bundle_seq}",
                                (8, mosaic.shape[0] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                last_window_size = _resize_window_to_image(
                    cv2, WINDOW_TITLE, mosaic, last_window_size
                )
                cv2.imshow(WINDOW_TITLE, mosaic)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                return 0
            if cv2.getWindowProperty(WINDOW_TITLE, cv2.WND_PROP_VISIBLE) < 1:
                return 0
            sleep_left = period - (time.monotonic() - loop_start)
            if sleep_left > 0:
                time.sleep(sleep_left)
    except KeyboardInterrupt:
        return 0
    finally:
        client.close()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
