#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


REPO_POLICY_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_POLICY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_POLICY_ROOT))

from policy_runner.spacemouse import HidSpaceMouseReader, SpaceMouseSample


DEFAULT_LEFT_PATH = "/dev/hidraw5"
DEFAULT_RIGHT_PATH = "/dev/hidraw10"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Probe one or two SpaceMouse HID devices and print live axes/buttons.",
    )
    parser.add_argument("--list", action="store_true", help="List detected SpaceMouse/HID devices and exit.")
    parser.add_argument("--duration-sec", type=float, default=30.0, help="Probe duration. Use 0 for unlimited.")
    parser.add_argument("--rate-hz", type=float, default=30.0, help="Polling/display rate.")
    parser.add_argument("--log", action="store_true", help="Print every sample as log lines instead of refreshing.")
    parser.add_argument("--single", action="store_true", help="Open only the left/default reader.")
    parser.add_argument("--device", default=None, help="pyspacemouse device name to open for both readers.")
    parser.add_argument("--left-device", default=None, help="pyspacemouse device name for the left reader.")
    parser.add_argument("--right-device", default=None, help="pyspacemouse device name for the right reader.")
    parser.add_argument(
        "--left-path",
        default=DEFAULT_LEFT_PATH,
        help=f"HID path for the left reader. Default: {DEFAULT_LEFT_PATH}.",
    )
    parser.add_argument(
        "--right-path",
        default=DEFAULT_RIGHT_PATH,
        help=f"HID path for the right reader. Default: {DEFAULT_RIGHT_PATH}.",
    )
    parser.add_argument("--left-device-number", type=int, default=0, help="pyspacemouse DeviceNumber for left.")
    parser.add_argument("--right-device-number", type=int, default=1, help="pyspacemouse DeviceNumber for right.")
    args = parser.parse_args(argv)

    if args.list:
        return list_devices()

    if args.rate_hz <= 0.0:
        raise SystemExit("--rate-hz must be positive")
    if args.duration_sec < 0.0:
        raise SystemExit("--duration-sec must be non-negative")

    readers = []
    try:
        readers.append(
            (
                "LEFT",
                HidSpaceMouseReader(
                    device=args.left_device or args.device,
                    path=args.left_path,
                    device_number=args.left_device_number,
                ),
            )
        )
        if not args.single:
            readers.append(
                (
                    "RIGHT",
                    HidSpaceMouseReader(
                        device=args.right_device or args.device,
                        path=args.right_path,
                        device_number=args.right_device_number,
                    ),
                )
            )
    except Exception as exc:
        close_readers(readers)
        print(f"failed to open SpaceMouse reader: {type(exc).__name__}: {exc}", file=sys.stderr)
        print("Try --list, or pass --left-path/--right-path with stable /dev/hidraw* paths.", file=sys.stderr)
        return 2

    period = 1.0 / args.rate_hz
    deadline = None if args.duration_sec == 0.0 else time.monotonic() + args.duration_sec
    latest: dict[str, SpaceMouseSample | None] = {side: None for side, _reader in readers}
    if args.log:
        print("Move each SpaceMouse and press buttons. Ctrl+C exits.")
        print("time_sec side   tx      ty      tz      rx      ry      rz      buttons")
    try:
        while deadline is None or time.monotonic() < deadline:
            loop_start = time.monotonic()
            for side, reader in readers:
                sample = reader.read(timeout_sec=0.0)
                if sample is not None:
                    latest[side] = sample
                    if args.log:
                        print(format_sample(side, sample), flush=True)
            if not args.log:
                render_screen(latest, args.duration_sec, deadline)
            sleep_sec = period - (time.monotonic() - loop_start)
            if sleep_sec > 0.0:
                time.sleep(sleep_sec)
    except KeyboardInterrupt:
        pass
    finally:
        close_readers(readers)
        if not args.log:
            print()
    return 0


def list_devices() -> int:
    try:
        import pyspacemouse  # type: ignore

        print("pyspacemouse supported connected devices:")
        for index, name in enumerate(pyspacemouse.list_devices()):
            print(f"  {index}: {name}")
    except Exception as exc:
        print(f"pyspacemouse list failed: {type(exc).__name__}: {exc}", file=sys.stderr)

    try:
        from easyhid import Enumeration  # type: ignore

        print("\n3Dconnexion HID interfaces:")
        count = 0
        for dev in Enumeration().find() or []:
            vendor_id = getattr(dev, "vendor_id", None)
            product_id = getattr(dev, "product_id", None)
            manufacturer = str(getattr(dev, "manufacturer_string", ""))
            product = str(getattr(dev, "product_string", ""))
            if vendor_id == 0x256F or "3Dconnexion" in manufacturer or "Space" in product:
                count += 1
                path = getattr(dev, "path", "")
                phys = hid_phys_for_path(path)
                interface_number = getattr(dev, "interface_number", None)
                usage_page = getattr(dev, "usage_page", None)
                usage = getattr(dev, "usage", None)
                print(
                    f"  vendor={hex(vendor_id or 0)} product={hex(product_id or 0)} "
                    f"manufacturer={manufacturer!r} product_name={product!r} path={path} "
                    f"interface={interface_number} usage_page={usage_page} usage={usage} phys={phys}"
                )
        if count == 0:
            print("  none")
    except Exception as exc:
        print(f"easyhid enumeration failed: {type(exc).__name__}: {exc}", file=sys.stderr)
    return 0


def hid_phys_for_path(path: str) -> str:
    name = Path(path).name
    if not name.startswith("hidraw"):
        return ""
    uevent = Path("/sys/class/hidraw") / name / "device" / "uevent"
    try:
        for line in uevent.read_text(encoding="utf-8", errors="replace").splitlines():
            if line.startswith("HID_PHYS="):
                return line.split("=", 1)[1]
    except OSError:
        return ""
    return ""


def format_sample(side: str, sample: SpaceMouseSample) -> str:
    buttons = "".join("1" if button else "0" for button in sample.buttons) or "-"
    return (
        f"{sample.timestamp_monotonic:8.3f} {side:<5} "
        f"{sample.tx:+.3f} {sample.ty:+.3f} {sample.tz:+.3f} "
        f"{sample.rx:+.3f} {sample.ry:+.3f} {sample.rz:+.3f}  {buttons}"
    )


def render_screen(
    latest: dict[str, SpaceMouseSample | None],
    duration_sec: float,
    deadline: float | None,
) -> None:
    remaining = "unlimited"
    if deadline is not None:
        remaining = f"{max(0.0, deadline - time.monotonic()):.1f}s"
    lines = [
        "\033[2J\033[H",
        "SpaceMouse probe",
        f"duration={duration_sec:g}s remaining={remaining}",
        "Move each SpaceMouse and press buttons. Ctrl+C exits.",
        "",
        "side   age_ms  tx      ty      tz      rx      ry      rz      buttons",
    ]
    for side in sorted(latest):
        sample = latest[side]
        if sample is None:
            lines.append(f"{side:<5} {'-':>6}  {'waiting for input':<44} -")
        else:
            age_ms = max(0.0, (time.monotonic() - sample.timestamp_monotonic) * 1000.0)
            buttons = "".join("1" if button else "0" for button in sample.buttons) or "-"
            lines.append(
                f"{side:<5} {age_ms:6.0f}  "
                f"{sample.tx:+.3f} {sample.ty:+.3f} {sample.tz:+.3f} "
                f"{sample.rx:+.3f} {sample.ry:+.3f} {sample.rz:+.3f}  {buttons}"
            )
    print("\n".join(lines), end="", flush=True)


def close_readers(readers: list[tuple[str, HidSpaceMouseReader]]) -> None:
    for _side, reader in readers:
        reader.close()


if __name__ == "__main__":
    raise SystemExit(main())
