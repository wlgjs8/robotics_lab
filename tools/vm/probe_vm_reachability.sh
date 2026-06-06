#!/usr/bin/env bash
set -euo pipefail

LEFT_IP=""
RIGHT_IP=""
OUTPUT="artifacts/vm_parity/WU-01/reachability.json"
TIMEOUT_SEC="1.0"
TRY_RBPODO_STATE=0

usage() {
  cat <<'EOF'
Usage: tools/vm/probe_vm_reachability.sh --left IP --right IP [options]

Probe the two Rainbow controller-simulation VMs from the host. rbpodo uses
fixed command/data ports 5000/5001, so left and right must be different IPs.

Options:
  --left IP             Left VM/controller IP.
  --right IP            Right VM/controller IP.
  --timeout-sec SEC     TCP connect timeout, default 1.0.
  --output PATH         JSON output, default artifacts/vm_parity/WU-01/reachability.json.
  --try-rbpodo-state    Also run scripts/rbpodo_state_dump.py once per IP if available.
  -h, --help            Show this help.
EOF
}

fail() {
  echo "probe_vm_reachability: ERROR: $*" >&2
  exit 2
}

while (($# > 0)); do
  case "$1" in
    --left)
      [[ $# -ge 2 ]] || fail "--left requires an IP"
      LEFT_IP="$2"
      shift 2
      ;;
    --right)
      [[ $# -ge 2 ]] || fail "--right requires an IP"
      RIGHT_IP="$2"
      shift 2
      ;;
    --timeout-sec)
      [[ $# -ge 2 ]] || fail "--timeout-sec requires a value"
      TIMEOUT_SEC="$2"
      shift 2
      ;;
    --output)
      [[ $# -ge 2 ]] || fail "--output requires a path"
      OUTPUT="$2"
      shift 2
      ;;
    --try-rbpodo-state)
      TRY_RBPODO_STATE=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      fail "unknown argument: $1"
      ;;
  esac
done

[[ -n "${LEFT_IP}" ]] || fail "missing --left"
[[ -n "${RIGHT_IP}" ]] || fail "missing --right"
[[ "${LEFT_IP}" != "${RIGHT_IP}" ]] || fail "left and right VM IPs must differ"

python3 - "${LEFT_IP}" "${RIGHT_IP}" "${TIMEOUT_SEC}" "${OUTPUT}" "${TRY_RBPODO_STATE}" <<'PY'
import json
import socket
import subprocess
import sys
from pathlib import Path

left_ip, right_ip, timeout_raw, output_raw, try_state_raw = sys.argv[1:6]
timeout = float(timeout_raw)
output = Path(output_raw)
try_state = try_state_raw == "1"


def probe_tcp(ip: str, port: int) -> dict[str, object]:
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return {"port": port, "ok": True, "error": None}
    except OSError as exc:
        return {"port": port, "ok": False, "error": f"{type(exc).__name__}: {exc}"}


def state_probe(ip: str, artifact: Path) -> dict[str, object]:
    script = Path("scripts/rbpodo_state_dump.py")
    if not script.is_file():
        return {"attempted": False, "skipped": True, "reason": "scripts/rbpodo_state_dump.py not found"}
    cmd = [
        sys.executable,
        str(script),
        "--ips",
        ip,
        "--timeout-sec",
        str(timeout),
        "--output",
        str(artifact),
        "--json",
        "--i-understand-this-connects-to-real-controller",
    ]
    completed = subprocess.run(cmd, check=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    return {
        "attempted": True,
        "returncode": completed.returncode,
        "artifact": str(artifact),
        "stdout_tail": completed.stdout[-2000:],
        "stderr_tail": completed.stderr[-2000:],
        "ok": completed.returncode == 0,
    }


arms = {"left": left_ip, "right": right_ip}
result: dict[str, object] = {
    "schema": "robotics_lab.vm_parity.reachability.v1",
    "source": "controller_simulation_vm",
    "physical_motion": False,
    "timeout_sec": timeout,
    "rbpodo_fixed_ports": [5000, 5001],
    "same_ip_forbidden": left_ip == right_ip,
    "arms": {},
    "status": "FAIL",
}

all_ok = True
for arm, ip in arms.items():
    tcp = [probe_tcp(ip, port) for port in (5000, 5001)]
    arm_ok = all(item["ok"] for item in tcp)
    state = {"attempted": False, "skipped": True, "reason": "not requested"}
    if try_state:
        state = state_probe(ip, output.parent / f"state_dump_{arm}.json")
        arm_ok = arm_ok and bool(state.get("ok"))
    result["arms"][arm] = {"ip": ip, "tcp": tcp, "state_probe": state, "ok": arm_ok}
    all_ok = all_ok and arm_ok

result["status"] = "PASS" if all_ok else "FAIL"
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
print(f"probe_vm_reachability: wrote {output} ({result['status']})")
if not all_ok:
    raise SystemExit(1)
PY
