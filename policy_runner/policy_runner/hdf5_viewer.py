from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, TextIO

import h5py
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .dataset_manifest import parse_camera_names
from .flow_dataset import (
    DEFAULT_ACTION_FRAME,
    FLOW_ACTION_DIM,
    FLOW_ARM_DIM,
    FlowEpisodeIndex,
    decode_hdf5_image_value,
    load_flow_episode_index,
    normalize_action_frame,
    pose_delta_local,
)


DEFAULT_VIEWER_IMAGE_SIZE = 320
DEFAULT_TRAIL_LENGTH = 120
MAX_DEFAULT_FPS = 30.0


@dataclass(frozen=True)
class ViewerEpisode:
    episode: FlowEpisodeIndex
    camera_names: tuple[str, ...]
    single_arm_side: str
    action_frame: str = DEFAULT_ACTION_FRAME


def load_viewer_episode(
    episode_path: str | Path,
    *,
    single_arm_side: str = "left",
    camera_names: list[str] | tuple[str, ...] | None = None,
    action_frame: str = DEFAULT_ACTION_FRAME,
) -> ViewerEpisode:
    episode = load_flow_episode_index(episode_path, single_arm_side=single_arm_side)
    selected_cameras = (
        tuple(camera_names) if camera_names is not None else tuple(sorted(episode.camera_paths))
    )
    return ViewerEpisode(
        episode=episode,
        camera_names=selected_cameras,
        single_arm_side=single_arm_side,
        action_frame=normalize_action_frame(action_frame),
    )


def inferred_fps(viewer: ViewerEpisode) -> float:
    timestamps = np.asarray(viewer.episode.timestamps, dtype=np.float64)
    if timestamps.size >= 2:
        diffs = np.diff(timestamps)
        finite = diffs[np.isfinite(diffs) & (diffs > 1e-9)]
        if finite.size:
            return min(max(1.0 / float(np.median(finite)), 1.0), MAX_DEFAULT_FPS)
    return MAX_DEFAULT_FPS


def frame_summary(viewer: ViewerEpisode, frame_index: int) -> dict[str, Any]:
    episode = viewer.episode
    index = _clamp_frame(frame_index, episode.length)
    timestamp = float(episode.timestamps[index]) if episode.timestamps.size else float(index)
    return {
        "path": str(episode.path),
        "format_name": episode.format_name,
        "frame_index": index,
        "frame_count": episode.length,
        "timestamp": timestamp,
        "camera_names": list(viewer.camera_names),
        "arms": {
            "left": _arm_summary(episode, "left", index, action_frame=viewer.action_frame),
            "right": _arm_summary(episode, "right", index, action_frame=viewer.action_frame),
        },
    }


def render_viewer_frame(
    viewer: ViewerEpisode,
    frame_index: int,
    *,
    image_size: int = DEFAULT_VIEWER_IMAGE_SIZE,
    trail_length: int = DEFAULT_TRAIL_LENGTH,
) -> np.ndarray:
    if image_size <= 0:
        raise ValueError("image_size must be positive")
    if trail_length <= 0:
        raise ValueError("trail_length must be positive")

    episode = viewer.episode
    index = _clamp_frame(frame_index, episode.length)
    side_width = max(460, image_size * 2 + 36)
    header_height = 54
    telemetry_height = 250
    trail_height = 170
    camera_rows = max(
        _camera_row_count(viewer, "left"),
        _camera_row_count(viewer, "right"),
        1,
    )
    camera_height = camera_rows * (image_size + 34) + 12
    height = header_height + camera_height + telemetry_height + trail_height
    width = side_width * 2

    canvas = Image.new("RGB", (width, height), (18, 22, 28))
    draw = ImageDraw.Draw(canvas)
    font = ImageFont.load_default()

    summary = frame_summary(viewer, index)
    title = (
        f"{Path(summary['path']).name} | frame {index + 1}/{episode.length} | "
        f"t={summary['timestamp']:.3f}s | {summary['format_name']}"
    )
    _draw_text(draw, (12, 10), title, fill=(238, 242, 247), font=font)
    _draw_text(
        draw,
        (12, 30),
        "keys: space pause/play, a/d or arrows step, [/] speed, home/end jump, q quit",
        fill=(164, 174, 190),
        font=font,
    )

    _draw_side_panel(
        canvas,
        draw,
        viewer,
        "left",
        index,
        x0=0,
        y0=header_height,
        side_width=side_width,
        image_size=image_size,
        telemetry_y=header_height + camera_height,
        trail_y=header_height + camera_height + telemetry_height,
        trail_height=trail_height,
        trail_length=trail_length,
        font=font,
    )
    _draw_side_panel(
        canvas,
        draw,
        viewer,
        "right",
        index,
        x0=side_width,
        y0=header_height,
        side_width=side_width,
        image_size=image_size,
        telemetry_y=header_height + camera_height,
        trail_y=header_height + camera_height + telemetry_height,
        trail_height=trail_height,
        trail_length=trail_length,
        font=font,
    )

    return np.asarray(canvas, dtype=np.uint8)


def run_hdf5_viewer_cli(args: Any, *, stderr: TextIO = sys.stderr) -> int:
    try:
        import cv2  # type: ignore[import-not-found]
    except Exception:
        print(
            "OpenCV cv2 is required for hdf5-view; install opencv-python in this environment.",
            file=stderr,
        )
        return 1

    try:
        viewer = load_viewer_episode(
            args.episode,
            single_arm_side=args.single_arm_side,
            camera_names=parse_camera_names(args.camera_names),
            action_frame=getattr(args, "action_frame", DEFAULT_ACTION_FRAME),
        )
    except Exception as exc:
        print(f"policy_runner hdf5-view failed: {exc}", file=stderr)
        return 1

    fps = float(args.fps) if args.fps is not None else inferred_fps(viewer)
    fps = max(fps, 0.1)
    index = _clamp_frame(int(args.start_frame), viewer.episode.length)
    paused = False
    window_name = "robotics_lab hdf5-view"

    try:
        while True:
            started = time.monotonic()
            rgb = render_viewer_frame(
                viewer,
                index,
                image_size=int(args.image_size),
                trail_length=int(args.trail_length),
            )
            cv2.imshow(window_name, rgb[:, :, ::-1])
            delay_ms = 50 if paused else max(1, int(1000.0 / fps))
            key = int(cv2.waitKeyEx(delay_ms))

            if key in (27, ord("q"), ord("Q")):
                break
            if key == ord(" "):
                paused = not paused
            elif key in (ord("a"), ord("A"), 2424832):
                index = max(0, index - 1)
                paused = True
            elif key in (ord("d"), ord("D"), 2555904):
                index = min(viewer.episode.length - 1, index + 1)
                paused = True
            elif key in (ord("["), ord("{")):
                fps = max(0.1, fps / 2.0)
            elif key in (ord("]"), ord("}")):
                fps = min(240.0, fps * 2.0)
            elif key == 2359296:
                index = 0
                paused = True
            elif key == 2293760:
                index = viewer.episode.length - 1
                paused = True
            elif not paused:
                elapsed = time.monotonic() - started
                if elapsed < 1.0 / fps:
                    time.sleep((1.0 / fps) - elapsed)
                index = min(viewer.episode.length - 1, index + 1)
                if index >= viewer.episode.length - 1:
                    paused = True
    finally:
        try:
            cv2.destroyWindow(window_name)
        except Exception:
            pass

    return 0


def _arm_summary(
    episode: FlowEpisodeIndex,
    side: str,
    index: int,
    *,
    action_frame: str = DEFAULT_ACTION_FRAME,
) -> dict[str, Any]:
    normalize_action_frame(action_frame)  # validate ee_local
    reset_delta_fn = pose_delta_local
    action_delta_fn = pose_delta_local
    if side == "left":
        pose = episode.left_pose[index]
        reset_pose = episode.reset_left_pose
        previous_pose = episode.left_pose[max(0, index - 1)]
        gripper = float(episode.left_gripper[index])
        action_pose = episode.action_left_pose
        action_delta = episode.action_left_delta
        action_gripper = episode.action_left_gripper
        active = bool(episode.arm_mask[0] > 0.0)
    else:
        pose = episode.right_pose[index]
        reset_pose = episode.reset_right_pose
        previous_pose = episode.right_pose[max(0, index - 1)]
        gripper = float(episode.right_gripper[index])
        action_pose = episode.action_right_pose
        action_delta = episode.action_right_delta
        action_gripper = episode.action_right_gripper
        active = bool(episode.arm_mask[1] > 0.0)

    if not active:
        action = np.zeros(FLOW_ARM_DIM, dtype=np.float32)
    elif episode.action_kind == "delta" and action_delta is not None:
        action = _pad_action_delta(action_delta[index])
        action[6] = _per_step_gripper_delta(action_gripper, index)
    elif index >= episode.length - 1:
        action = np.zeros(FLOW_ARM_DIM, dtype=np.float32)
    else:
        target = action_pose[index] if action_pose is not None else pose
        next_target = action_pose[index + 1] if action_pose is not None else _next_pose(episode, side, index)
        target_delta = action_delta_fn(target, next_target)
        action = np.concatenate([target_delta, [_per_step_gripper_delta(action_gripper, index)]]).astype(np.float32)

    return {
        "active": active,
        "pose": np.asarray(pose, dtype=np.float32),
        "reset_delta": reset_delta_fn(reset_pose, pose),
        "frame_delta": reset_delta_fn(previous_pose, pose) if index > 0 else np.zeros(6, dtype=np.float32),
        "gripper": gripper,
        "action": action,
        "action_kind": episode.action_kind,
    }


def _pad_action_delta(value: np.ndarray) -> np.ndarray:
    out = np.zeros(FLOW_ARM_DIM, dtype=np.float32)
    arr = np.asarray(value, dtype=np.float32).reshape(-1)
    out[: min(FLOW_ARM_DIM - 1, arr.size)] = arr[: FLOW_ARM_DIM - 1]
    return out


def _next_pose(episode: FlowEpisodeIndex, side: str, index: int) -> np.ndarray:
    if side == "left":
        return episode.left_pose[index + 1]
    return episode.right_pose[index + 1]


def _per_step_gripper_delta(values: np.ndarray | None, index: int) -> float:
    if values is None or index >= len(values) - 1:
        return 0.0
    return float(values[index + 1]) - float(values[index])


def _draw_side_panel(
    canvas: Image.Image,
    draw: ImageDraw.ImageDraw,
    viewer: ViewerEpisode,
    side: str,
    index: int,
    *,
    x0: int,
    y0: int,
    side_width: int,
    image_size: int,
    telemetry_y: int,
    trail_y: int,
    trail_height: int,
    trail_length: int,
    font: ImageFont.ImageFont,
) -> None:
    panel_color = (24, 30, 38)
    border_color = (55, 65, 81)
    draw.rectangle((x0 + 6, y0, x0 + side_width - 6, canvas.height - 8), fill=panel_color, outline=border_color)
    arm = frame_summary(viewer, index)["arms"][side]
    status = "active" if arm["active"] else "inactive"
    title_color = (134, 239, 172) if arm["active"] else (148, 163, 184)
    _draw_text(draw, (x0 + 16, y0 + 10), f"{side.upper()} {status}", fill=title_color, font=font)

    camera_names = _side_camera_names(viewer, side)
    if not camera_names:
        placeholder = _placeholder_image(image_size, image_size, f"{side} view inactive")
        canvas.paste(placeholder, (x0 + 18, y0 + 34))
    else:
        for offset, camera_name in enumerate(camera_names):
            row = offset // 2
            col = offset % 2
            tile_x = x0 + 18 + col * (image_size + 14)
            tile_y = y0 + 34 + row * (image_size + 34)
            image = _load_camera_image(viewer, camera_name, index, image_size)
            image = _label_tile(image, camera_name)
            canvas.paste(image, (tile_x, tile_y))

    text_x = x0 + 16
    text_y = telemetry_y + 10
    for line in _arm_lines(arm):
        _draw_text(draw, (text_x, text_y), line, fill=(226, 232, 240), font=font)
        text_y += 18

    _draw_trail(
        draw,
        viewer.episode,
        side,
        index,
        x0=x0 + 18,
        y0=trail_y + 12,
        width=side_width - 36,
        height=trail_height - 30,
        trail_length=trail_length,
    )


def _arm_lines(arm: dict[str, Any]) -> list[str]:
    pose = np.asarray(arm["pose"], dtype=np.float32)
    reset_delta = np.asarray(arm["reset_delta"], dtype=np.float32)
    frame_delta = np.asarray(arm["frame_delta"], dtype=np.float32)
    action = np.asarray(arm["action"], dtype=np.float32)
    return [
        f"pose xyz: {_fmt_vec(pose[:3])}",
        f"pose qxyzw: {_fmt_vec(pose[3:7])}",
        f"reset delta dxyz: {_fmt_vec(reset_delta[:3])}",
        f"reset delta rvec: {_fmt_vec(reset_delta[3:6])}",
        f"frame delta dxyz: {_fmt_vec(frame_delta[:3])}",
        f"action dxyz: {_fmt_vec(action[:3])}",
        f"action rvec: {_fmt_vec(action[3:6])}",
        f"gripper/action grip delta: {arm['gripper']:.4f} / {float(action[6]):+.4f}",
        f"action kind: {arm['action_kind']}",
    ]


def _fmt_vec(value: np.ndarray) -> str:
    return " ".join(f"{float(x):+.4f}" for x in value)


def _side_camera_names(viewer: ViewerEpisode, side: str) -> list[str]:
    active = bool(viewer.episode.arm_mask[0 if side == "left" else 1] > 0.0)
    out: list[str] = []
    for name in viewer.camera_names:
        if name.startswith(f"{side}_"):
            out.append(name)
        elif not name.startswith(("left_", "right_")) and active:
            out.append(name)
    return out


def _camera_row_count(viewer: ViewerEpisode, side: str) -> int:
    count = max(1, len(_side_camera_names(viewer, side)))
    return (count + 1) // 2


def _load_camera_image(viewer: ViewerEpisode, camera_name: str, index: int, image_size: int) -> Image.Image:
    camera_path = viewer.episode.camera_paths.get(camera_name)
    if camera_path is None:
        return _placeholder_image(image_size, image_size, f"missing {camera_name}")
    try:
        with h5py.File(viewer.episode.path, "r") as handle:
            dataset = handle[camera_path]
            if index >= dataset.shape[0]:
                return _placeholder_image(image_size, image_size, f"missing {camera_name}")
            chw = decode_hdf5_image_value(dataset[index], image_size=image_size)
    except Exception:
        return _placeholder_image(image_size, image_size, f"decode error {camera_name}")
    arr = np.clip(np.transpose(chw, (1, 2, 0)) * 255.0, 0, 255).astype(np.uint8)
    return Image.fromarray(arr, mode="RGB")


def _label_tile(image: Image.Image, label: str) -> Image.Image:
    out = image.copy()
    draw = ImageDraw.Draw(out)
    font = ImageFont.load_default()
    draw.rectangle((0, 0, out.width, 18), fill=(0, 0, 0))
    _draw_text(draw, (4, 4), label, fill=(248, 250, 252), font=font)
    return out


def _placeholder_image(width: int, height: int, label: str) -> Image.Image:
    image = Image.new("RGB", (width, height), (31, 41, 55))
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()
    draw.rectangle((0, 0, width - 1, height - 1), outline=(75, 85, 99))
    _draw_text(draw, (10, max(10, height // 2 - 8)), label, fill=(148, 163, 184), font=font)
    return image


def _draw_trail(
    draw: ImageDraw.ImageDraw,
    episode: FlowEpisodeIndex,
    side: str,
    index: int,
    *,
    x0: int,
    y0: int,
    width: int,
    height: int,
    trail_length: int,
) -> None:
    draw.rectangle((x0, y0, x0 + width, y0 + height), fill=(15, 23, 42), outline=(71, 85, 105))
    _draw_text(draw, (x0 + 8, y0 + 8), "XY trail", fill=(203, 213, 225), font=ImageFont.load_default())
    poses = episode.left_pose if side == "left" else episode.right_pose
    start = max(0, index - trail_length + 1)
    points = np.asarray(poses[start : index + 1, :2], dtype=np.float32)
    if points.size == 0 or not np.isfinite(points).all():
        return
    min_xy = points.min(axis=0)
    max_xy = points.max(axis=0)
    span = np.maximum(max_xy - min_xy, 1e-6)
    pad = 24
    drawable_w = max(1, width - 2 * pad)
    drawable_h = max(1, height - 2 * pad)

    def project(point: np.ndarray) -> tuple[int, int]:
        norm = (point - min_xy) / span
        x = int(x0 + pad + float(norm[0]) * drawable_w)
        y = int(y0 + height - pad - float(norm[1]) * drawable_h)
        return x, y

    projected = [project(point) for point in points]
    if len(projected) >= 2:
        draw.line(projected, fill=(96, 165, 250), width=2)
    cx, cy = projected[-1]
    draw.ellipse((cx - 4, cy - 4, cx + 4, cy + 4), fill=(248, 250, 252))


def _draw_text(
    draw: ImageDraw.ImageDraw,
    xy: tuple[int, int],
    text: str,
    *,
    fill: tuple[int, int, int],
    font: ImageFont.ImageFont,
) -> None:
    draw.text(xy, text, fill=fill, font=font)


def _clamp_frame(frame_index: int, frame_count: int) -> int:
    if frame_count <= 0:
        raise ValueError("episode has no frames")
    return min(max(int(frame_index), 0), frame_count - 1)
