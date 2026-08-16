"""At 90 fps, sweep manual gain to see if brightness can recover to the 30 fps
baseline (~111/115 mean) and what temporal-noise cost it carries.

Noise proxy: per-pixel std over 30 consecutive frames of a static scene.
"""
import time

import numpy as np
import pyrealsense2 as rs

SERIALS = ["412622272078", "260322278348"]
W, H, FPS = 640, 480, 90
GAINS = [16, 32, 48, 64]


def measure(pipe, n=30):
    frames_buf = []
    for _ in range(n):
        frames = pipe.wait_for_frames()
        color = frames.get_color_frame()
        frames_buf.append(np.asanyarray(color.get_data()).astype(np.float32))
    stack = np.stack(frames_buf)
    return float(stack.mean()), float(stack.std(axis=0).mean())


def main():
    for serial in SERIALS:
        cfg = rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.color, W, H, rs.format.rgb8, FPS)
        pipe = rs.pipeline()
        profile = pipe.start(cfg)
        sensor = profile.get_device().query_sensors()[0]
        print(f"\n--- {serial} @ {FPS} fps ---")
        gain_range = sensor.get_option_range(rs.option.gain)
        exp_range = sensor.get_option_range(rs.option.exposure)
        print(f"  gain range {gain_range.min}-{gain_range.max}, exposure range {exp_range.min}-{exp_range.max} us")

        sensor.set_option(rs.option.enable_auto_exposure, 1)
        time.sleep(2.0)
        bright, noise = measure(pipe)
        print(f"  AE on           : brightness={bright:6.1f} temporal_noise={noise:.2f}")

        sensor.set_option(rs.option.enable_auto_exposure, 0)
        max_exp = min(9900.0, exp_range.max)
        sensor.set_option(rs.option.exposure, max_exp)
        for gain in GAINS:
            if gain > gain_range.max:
                break
            sensor.set_option(rs.option.gain, float(gain))
            time.sleep(0.7)
            bright, noise = measure(pipe)
            print(f"  exp={max_exp:.0f} gain={gain:3d}: brightness={bright:6.1f} temporal_noise={noise:.2f}")
        sensor.set_option(rs.option.enable_auto_exposure, 1)
        pipe.stop()
        time.sleep(1.0)


if __name__ == "__main__":
    main()
