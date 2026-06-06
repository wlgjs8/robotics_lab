#!/usr/bin/env bash
set -euo pipefail

OUTPUT="artifacts/vm_parity/WU-01/ova_verify.json"
EXPECTED_SHA256=""
OVA_PATH=""

usage() {
  cat <<'EOF'
Usage: tools/vm/verify_ova.sh <path-to-ova> [options]

Verify Rainbow controller-simulation OVA metadata without booting or modifying
the appliance.

Options:
  --output PATH            JSON result path, default artifacts/vm_parity/WU-01/ova_verify.json
  --expected-sha256 HASH   Optional known-good SHA-256 to compare.
  -h, --help               Show this help.

The result always includes physical_motion=false and
source=controller_simulation_vm. Official provenance still has to be confirmed
from Rainbow/FAE distribution plus successful boot/reachability checks.
EOF
}

fail() {
  echo "verify_ova: ERROR: $*" >&2
  exit 2
}

while (($# > 0)); do
  case "$1" in
    --output)
      [[ $# -ge 2 ]] || fail "--output requires a path"
      OUTPUT="$2"
      shift 2
      ;;
    --expected-sha256)
      [[ $# -ge 2 ]] || fail "--expected-sha256 requires a value"
      EXPECTED_SHA256="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --*)
      fail "unknown argument: $1"
      ;;
    *)
      if [[ -n "${OVA_PATH}" ]]; then
        fail "multiple OVA paths provided"
      fi
      OVA_PATH="$1"
      shift
      ;;
  esac
done

[[ -n "${OVA_PATH}" ]] || fail "missing OVA path"
[[ -f "${OVA_PATH}" ]] || fail "OVA not found: ${OVA_PATH}"

python3 - "${OVA_PATH}" "${OUTPUT}" "${EXPECTED_SHA256}" <<'PY'
import hashlib
import json
import re
import subprocess
import sys
import tarfile
from pathlib import Path

ova = Path(sys.argv[1])
output = Path(sys.argv[2])
expected_sha256 = sys.argv[3] or None


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def run_optional(command: list[str]) -> dict[str, object]:
    try:
        completed = subprocess.run(
            command,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=60,
        )
    except FileNotFoundError:
        return {"available": False, "skipped": True, "reason": "tool_not_found"}
    except subprocess.TimeoutExpired:
        return {"available": True, "skipped": False, "returncode": None, "reason": "timeout"}
    return {
        "available": True,
        "skipped": False,
        "returncode": completed.returncode,
        "stdout_tail": completed.stdout[-4000:],
        "stderr_tail": completed.stderr[-4000:],
    }


result: dict[str, object] = {
    "schema": "robotics_lab.vm_parity.ova_verify.v1",
    "source": "controller_simulation_vm",
    "physical_motion": False,
    "ova_path": str(ova),
    "sha256": sha256(ova),
    "expected_sha256": expected_sha256,
    "expected_sha256_match": None,
    "tar_readable": False,
    "ovf_files": [],
    "vmdk_files": [],
    "manifest_files": [],
    "guest_os_linux_like": False,
    "nic_count": 0,
    "disk_capacities": [],
    "optional_verifiers": {},
    "status": "FAIL",
    "failures": [],
}

if expected_sha256:
    result["expected_sha256_match"] = result["sha256"] == expected_sha256.lower()
    if not result["expected_sha256_match"]:
        result["failures"].append("sha256_mismatch")

try:
    with tarfile.open(ova, "r:*") as archive:
        members = archive.getmembers()
        names = [member.name for member in members if member.isfile()]
        ovf_files = [name for name in names if name.lower().endswith(".ovf")]
        vmdk_files = [name for name in names if name.lower().endswith(".vmdk")]
        manifest_files = [name for name in names if name.lower().endswith(".mf")]
        result.update({
            "tar_readable": True,
            "ovf_files": ovf_files,
            "vmdk_files": vmdk_files,
            "manifest_files": manifest_files,
        })
        if len(ovf_files) != 1:
            result["failures"].append("expected_exactly_one_ovf")
        if not vmdk_files:
            result["failures"].append("expected_at_least_one_vmdk")
        if ovf_files:
            ovf_member = archive.getmember(ovf_files[0])
            ovf_handle = archive.extractfile(ovf_member)
            if ovf_handle is None:
                raise OSError("failed to read OVF member")
            ovf_text = ovf_handle.read().decode("utf-8", errors="ignore")
            result["guest_os_linux_like"] = "linux" in ovf_text.lower()
            result["nic_count"] = len(
                re.findall(r"<[^>]*ResourceType[^>]*>\s*10\s*</", ovf_text, flags=re.IGNORECASE)
            )
            result["disk_capacities"] = re.findall(
                r'ovf:capacity\s*=\s*"([^"]+)"', ovf_text, flags=re.IGNORECASE
            )
            if not result["guest_os_linux_like"]:
                result["failures"].append("guest_os_not_linux_like")
            if int(result["nic_count"]) < 1:
                result["failures"].append("expected_at_least_one_nic")
except (tarfile.TarError, OSError) as exc:
    result["failures"].append(f"tar_read_failed:{type(exc).__name__}:{exc}")

result["optional_verifiers"] = {
    "ovftool_verify": run_optional(["ovftool", "--verify", str(ova)]),
    "vboxmanage_import_dry_run": run_optional(["VBoxManage", "import", str(ova), "--dry-run"]),
}

result["status"] = "PASS" if not result["failures"] else "FAIL"
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
print(f"verify_ova: wrote {output} ({result['status']})")
if result["failures"]:
    print("verify_ova: failures: " + ", ".join(str(item) for item in result["failures"]), file=sys.stderr)
    raise SystemExit(1)
PY
