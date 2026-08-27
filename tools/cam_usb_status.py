#!/usr/bin/env python3
"""Report each configured RealSense camera's USB link speed and name what throttles a slow one.

camera_server refuses to start when a `required: true` RealSense enumerates below
USB3 (`validate_realsense_preflight()` -> "required RealSense camera is not on
USB3"), so one wrist camera that negotiates 480 Mbps takes the whole capture
stack down with it. `make cam-status` could not see that: it only looked at
running containers and at `[CAM] status=` lines, both of which are absent once
the process has already died on the preflight check.

Link speed is read straight out of sysfs (`/sys/bus/usb/devices/<node>/speed`).
That is passive — unlike `rs-enumerate-devices` it never opens the device, so it
is safe to run while camera_server is streaming.

Camera name -> USB port comes from camera_server's own startup log line

    [CAM] device <name> serial=<S> ... usb=<T> port=<sysfs path>

because the USB iSerial the kernel exposes is the D4xx *ASIC* serial
(e.g. 243323070989), not the librealsense serial the configs pin
(e.g. 412622272078); nothing in sysfs relates the two. Without a successful
start to read that mapping from, cameras are reported by ASIC serial instead of
by name rather than guessed at.

Exit status is 1 when any configured RealSense is missing or below SuperSpeed.

Usage:
  python3 tools/cam_usb_status.py                    # rig config auto-detected from the container
  python3 tools/cam_usb_status.py --config camera_server/config/dual_realsense_d405.yaml
"""

import argparse
import os
import re
import subprocess
import sys

SYSFS_USB = "/sys/bus/usb/devices"
INTEL_VENDOR = "8086"
# D4xx tops out at SuperSpeed 5 Gbps; camera_server rejects anything below USB3.
REQUIRED_MBPS = 5000

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# main.cpp 가 기동 시 stderr 로 찍는 줄: "[CAM] config=/app/config/x.yaml mode=... simulate=0".
RE_CONFIG = re.compile(r"\[CAM\] config=(\S+)")
RE_DEVICE = re.compile(r"\[CAM\] device (\S+) serial=(\S+).* usb=(\S+) port=(\S+)")
RE_NOT_USB3 = re.compile(r"not on USB3: (\S+) serial=(\S+) usb_type=(\S+)")
RE_USB_NODE = re.compile(r"\d+-\d+(\.\d+)*$")


def color(text, code, stream=sys.stdout):
    return f"\033[{code}m{text}\033[0m" if stream.isatty() else text


def read_attr(node, name):
    try:
        with open(os.path.join(SYSFS_USB, node, name)) as fh:
            return fh.read().strip()
    except OSError:
        return None


def scan_realsense():
    """Every RealSense currently enumerated, keyed by sysfs node ('4-1.2')."""
    found = {}
    try:
        entries = sorted(os.listdir(SYSFS_USB))
    except OSError as exc:
        print(f"usb links: sysfs 읽기 실패 ({exc})", file=sys.stderr)
        return found
    for entry in entries:
        if ":" in entry:  # interface node, not a device
            continue
        if read_attr(entry, "idVendor") != INTEL_VENDOR:
            continue
        product = read_attr(entry, "product") or ""
        if "RealSense" not in product:
            continue
        found[entry] = {
            "product": " ".join(product.split()),
            "asic_serial": read_attr(entry, "serial") or "?",
            "mbps": to_mbps(read_attr(entry, "speed")),
        }
    return found


def to_mbps(speed):
    try:
        return float(speed)
    except (TypeError, ValueError):
        return None


def fmt_speed(mbps):
    if mbps is None:
        return "unknown"
    if mbps >= 1000:
        return f"{mbps / 1000:g} Gbps"
    return f"{mbps:g} Mbps"


def speed_label(mbps):
    if mbps is None:
        return "unknown"
    if mbps >= REQUIRED_MBPS:
        return f"{fmt_speed(mbps)} SuperSpeed"
    if mbps >= 480:
        return f"{fmt_speed(mbps)} USB2"
    return f"{fmt_speed(mbps)} USB1.x"


def parent_of(node):
    """'4-1.2' -> '4-1' -> 'usb4' -> None (the xHCI root)."""
    if node.startswith("usb"):
        return None
    if "." in node:
        return node.rsplit(".", 1)[0]
    return "usb" + node.split("-", 1)[0]


def uplink_chain(node):
    """[(node, mbps, product)] from the xHCI root down to (but excluding) `node`."""
    chain = []
    cur = parent_of(node)
    while cur:
        chain.append((cur, to_mbps(read_attr(cur, "speed")), read_attr(cur, "product") or ""))
        cur = parent_of(cur)
    chain.reverse()
    return chain


def bottleneck(node):
    """Topmost link in the uplink chain that is below SuperSpeed, if any."""
    for hop, mbps, product in uplink_chain(node):
        if mbps is not None and mbps < REQUIRED_MBPS:
            return hop, mbps, " ".join(product.split())
    return None


def node_from_port(port):
    """Deepest USB device node in a sysfs path ('.../usb4/4-1/4-1.2/4-1.2:1.0/...' -> '4-1.2')."""
    node = None
    for segment in port.split("/"):
        if ":" not in segment and RE_USB_NODE.fullmatch(segment):
            node = segment
    return node


def read_container_log(container):
    """Last startup's config path, name->port device map, and any USB3 preflight failure.

    Streams the log so a long-running container (one `[CAM] status=` line per
    second) costs no more memory than a short one, and keeps only the newest
    occurrence of each line so a restarted container reports its latest start.
    """
    info = {"config": None, "devices": {}, "not_usb3": {}}
    try:
        proc = subprocess.Popen(
            ["docker", "logs", container],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            errors="replace",
        )
    except (OSError, subprocess.SubprocessError):
        return info
    with proc:
        for line in proc.stdout:
            match = RE_CONFIG.search(line)
            if match:
                # A fresh start supersedes whatever the previous one reported.
                info["config"], info["devices"], info["not_usb3"] = match.group(1), {}, {}
                continue
            match = RE_DEVICE.search(line)
            if match:
                name, serial, usb, port = match.groups()
                info["devices"][name] = {"serial": serial, "usb": usb, "node": node_from_port(port)}
                continue
            match = RE_NOT_USB3.search(line)
            if match:
                name, serial, usb_type = match.groups()
                info["not_usb3"][name] = {"serial": serial, "usb_type": usb_type}
    return info


def host_config_path(path):
    """Map the container's config path onto this checkout ('/app/config/x.yaml')."""
    if path and path.startswith("/app/"):
        return os.path.join(REPO_ROOT, "camera_server", path[len("/app/"):])
    return path


def load_cameras(path):
    """[(name, serial)] of the RealSense cameras the rig config declares."""
    try:
        import yaml
    except ImportError:
        print("usb links: PyYAML 없음 — 설정 대조 없이 발견된 카메라만 표시", file=sys.stderr)
        return None
    try:
        with open(path) as fh:
            cfg = yaml.safe_load(fh) or {}
    except OSError as exc:
        print(f"usb links: 설정 읽기 실패 {path} ({exc})", file=sys.stderr)
        return None
    cameras = cfg.get("cameras") or {}
    return [
        (name, str((spec or {}).get("serial", "?")))
        for name, spec in cameras.items()
        if (spec or {}).get("backend", "realsense") == "realsense"
    ]


def build_rows(cameras, devices, present, not_usb3):
    """One report row per configured camera, plus any RealSense the config doesn't claim."""
    rows = []
    claimed = set()
    for name, serial in cameras:
        logged = devices.get(name)
        node = logged["node"] if logged else None
        if node and node in present:
            claimed.add(node)
            rows.append({"state": None, "name": name, "serial": serial, "node": node,
                         "mbps": present[node]["mbps"]})
        elif name in not_usb3:
            # Died on preflight before the device line was ever logged.
            rows.append({"state": "FAIL", "name": name, "serial": not_usb3[name]["serial"],
                         "node": "?", "mbps": None,
                         "note": f"preflight 거부: usb_type={not_usb3[name]['usb_type']} (USB3 아님)"})
        elif node:
            rows.append({"state": "FAIL", "name": name, "serial": serial, "node": node,
                         "mbps": None,
                         "note": f"{node} 에 카메라 없음 — 분리됐거나 다른 포트로 이동"})
        else:
            rows.append({"state": "?", "name": name, "serial": serial, "node": "-", "mbps": None,
                         "note": "포트 미상 — camera_server가 한 번 정상 기동해야 이름↔포트가 잡힘"})

    for node, dev in sorted(present.items()):
        if node not in claimed and node not in {r["node"] for r in rows}:
            rows.append({"state": None, "name": "(unclaimed)", "serial": f"asic {dev['asic_serial']}",
                         "node": node, "mbps": dev["mbps"], "unclaimed": True})

    for row in rows:
        if row["state"] is None:
            row["state"] = "OK" if (row["mbps"] or 0) >= REQUIRED_MBPS else "FAIL"
    return rows


def print_rows(rows):
    widths = [max(len(str(row[key])) for row in rows) for key in ("name", "serial", "node")]
    for row in rows:
        state = row["state"]
        tint = {"OK": "32", "FAIL": "1;31"}.get(state, "1;33")
        # A node the log named but sysfs no longer has would print a chain of "unknown" hops.
        known = os.path.isdir(os.path.join(SYSFS_USB, str(row["node"])))
        chain = " > ".join(f"{hop} {fmt_speed(mbps)}" for hop, mbps, _ in uplink_chain(row["node"])) \
            if known else ""
        print(f"  {color(f'{state:<4}', tint)} "
              f"{row['name']:<{widths[0]}}  {row['serial']:<{widths[1]}}  "
              f"{row['node']:<{widths[2]}}  {speed_label(row['mbps']):<20}  {chain}")
        if row.get("note"):
            print(f"        └ {row['note']}")
        elif row["state"] == "FAIL" and row["mbps"] is not None:
            slow = bottleneck(row["node"])
            if slow and slow[0].startswith("usb"):
                # A USB3 hub that fails to bring its SuperSpeed link up re-enumerates on the
                # controller's USB2 bus, so the whole chain hangs off a 480 Mbps root.
                hop, mbps, _ = slow
                print(f"        └ 루트 버스 {hop} 부터 {fmt_speed(mbps)} — 체인 전체가 USB2로 "
                      f"enumerate됐다(USB3 허브라면 SuperSpeed 링크업 실패).")
                print(f"          USB3 포트로 옮기고, 업스트림 케이블이 USB3인지 확인할 것.")
            elif slow:
                hop, mbps, product = slow
                print(f"        └ 허브 {hop} ('{product}') 부터 {fmt_speed(mbps)} — "
                      f"이 허브의 업스트림이 SuperSpeed로 링크업하지 못했다.")
                print(f"          허브를 USB3 포트에 다시 꽂거나 USB3 케이블로 교체할 것.")
            else:
                print(f"        └ 상위 링크는 정상 — 카메라 케이블 자체가 USB2이거나 접촉 불량.")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", help="rig YAML (기본: 컨테이너 로그에서 자동 감지)")
    parser.add_argument("--container", default="camera_server", help="camera_server 컨테이너 이름")
    args = parser.parse_args()

    log = read_container_log(args.container)
    config_path = args.config or host_config_path(log["config"])
    present = scan_realsense()

    cameras = load_cameras(config_path) if config_path else None
    if cameras is None:
        if not args.config and not log["config"]:
            print("usb links: rig 설정 미확인(컨테이너 로그 없음) — 발견된 RealSense 전체를 표시")
        cameras = []

    rows = build_rows(cameras, log["devices"], present, log["not_usb3"])
    if not rows:
        print(color("usb links: RealSense 카메라를 하나도 찾지 못했다", "1;31"))
        return 1

    print(f"usb links (RealSense D4xx는 {fmt_speed(REQUIRED_MBPS)} SuperSpeed 필요"
          f"{'; rig=' + os.path.basename(config_path) if config_path else ''}):")
    print_rows(rows)
    return 1 if any(row["state"] != "OK" and not row.get("unclaimed") for row in rows) else 0


if __name__ == "__main__":
    sys.exit(main())
