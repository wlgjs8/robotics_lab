"""Probe dual D405 behavior at 30/60/90 fps: exposure, gain, brightness, latency, drops.

Run with the robotics_lab venv while camera_server is stopped:
  .venv/bin/python tools/probe_d405_fps.py
"""
import time

import numpy as np
import pyrealsense2 as rs

SERIALS = ["412622272078", "260322278348"]  # left arm, right arm (2026-07-23 binding)
FPS_LIST = [30, 60, 90]
WARMUP_SEC = 3.0
MEASURE_SEC = 6.0
W, H = 640, 480


def probe(fps):
    pipes = []
    for serial in SERIALS:
        cfg = rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.color, W, H, rs.format.rgb8, fps)
        cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, fps)
        pipe = rs.pipeline()
        profile = pipe.start(cfg)
        dev = profile.get_device()
        for sensor in dev.query_sensors():
            if sensor.supports(rs.option.global_time_enabled):
                sensor.set_option(rs.option.global_time_enabled, 1)
        pipes.append((serial, pipe))

    t_end = time.time() + WARMUP_SEC
    while time.time() < t_end:
        for _, pipe in pipes:
            pipe.wait_for_frames()

    stats = {s: {"exp": [], "gain": [], "bright": [], "lat_ms": [], "fnum": []} for s in SERIALS}
    t_end = time.time() + MEASURE_SEC
    while time.time() < t_end:
        for serial, pipe in pipes:
            frames = pipe.wait_for_frames()
            color = frames.get_color_frame()
            if not color:
                continue
            now_ms = time.time() * 1000.0
            st = stats[serial]
            st["lat_ms"].append(now_ms - color.get_timestamp())
            st["fnum"].append(color.get_frame_number())
            if color.supports_frame_metadata(rs.frame_metadata_value.actual_exposure):
                st["exp"].append(color.get_frame_metadata(rs.frame_metadata_value.actual_exposure))
            if color.supports_frame_metadata(rs.frame_metadata_value.gain_level):
                st["gain"].append(color.get_frame_metadata(rs.frame_metadata_value.gain_level))
            img = np.asanyarray(color.get_data())
            st["bright"].append(float(img.mean()))

    for _, pipe in pipes:
        pipe.stop()
    time.sleep(1.0)

    print(f"\n=== {fps} fps (color+depth {W}x{H}, both cameras) ===")
    for serial in SERIALS:
        st = stats[serial]
        fnums = np.array(st["fnum"])
        gaps = np.diff(fnums)
        dropped = int((gaps - 1).clip(min=0).sum()) if len(gaps) else -1
        lat = np.array(st["lat_ms"])
        exp = np.array(st["exp"], dtype=float)
        gain = np.array(st["gain"], dtype=float)
        bright = np.array(st["bright"])
        print(
            f"  {serial}: n={len(fnums)} dropped={dropped} | "
            f"exposure_us mean={exp.mean():.0f} p95={np.percentile(exp, 95):.0f} | "
            f"gain mean={gain.mean():.1f} max={gain.max():.0f} | "
            f"brightness mean={bright.mean():.1f} | "
            f"capture->host lat_ms p50={np.percentile(lat, 50):.1f} p95={np.percentile(lat, 95):.1f}"
        )


def main():
    ctx = rs.context()
    found = {d.get_info(rs.camera_info.serial_number) for d in ctx.query_devices()}
    missing = [s for s in SERIALS if s not in found]
    if missing:
        raise SystemExit(f"missing devices: {missing} (found: {sorted(found)})")
    for fps in FPS_LIST:
        probe(fps)


if __name__ == "__main__":
    main()
