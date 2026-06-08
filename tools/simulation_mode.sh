#!/usr/bin/env bash
set -u
set -o pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT="5000"
TIMEOUT_SEC="1.0"
SUMMARY_JSON="artifacts/rbpodo_pgmode/simulation_mode_summary.json"
# Connect acknowledgment: pass --i-understand-this-connects-to-real-controller, or
# set RB_I_UNDERSTAND_REAL_CONTROLLER=1 once (convenience for routine pgmode-sim use).
# This only acknowledges connecting to a real controller IP; no physical motion.
CONFIRM=0
case "${RB_I_UNDERSTAND_REAL_CONTROLLER:-}" in
  1 | true | TRUE | yes | YES | on | ON) CONFIRM=1 ;;
esac

ROBOTS=(
  "172.28.60.200"
  "172.28.60.201"
)

usage() {
  cat <<'EOF'
Usage: tools/simulation_mode.sh --i-understand-this-connects-to-real-controller [options]
       (or set RB_I_UNDERSTAND_REAL_CONTROLLER=1 instead of passing the flag)

Options:
  --ips IP [IP ...]       Override controller IPs.
  --port PORT             Rainbow command TCP port, default 5000.
  --timeout-sec SEC       TCP/read timeout, default 1.0.
  --summary-json PATH     JSON artifact path.
  --verify-only           Verify CobotData.real_vs_simulation_mode only.

This wrapper never sends pgmode real, reset, servo-power, collision-threshold,
or motion commands.
EOF
}

MODE_FLAG="--set-simulation"
while (($# > 0)); do
  case "$1" in
    --i-understand-this-connects-to-real-controller)
      CONFIRM=1
      shift
      ;;
    --port)
      PORT="${2:?missing --port value}"
      shift 2
      ;;
    --timeout-sec)
      TIMEOUT_SEC="${2:?missing --timeout-sec value}"
      shift 2
      ;;
    --summary-json)
      SUMMARY_JSON="${2:?missing --summary-json value}"
      shift 2
      ;;
    --verify-only)
      MODE_FLAG="--verify-only"
      shift
      ;;
    --ips)
      shift
      ROBOTS=()
      while (($# > 0)) && [[ "$1" != --* ]]; do
        ROBOTS+=("$1")
        shift
      done
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ "$CONFIRM" != "1" ]]; then
  echo "ERROR: refusing controller connection without --i-understand-this-connects-to-real-controller" >&2
  exit 2
fi

if ((${#ROBOTS[@]} == 0)); then
  echo "ERROR: no controller IPs provided" >&2
  exit 2
fi

cd "$ROOT_DIR"
python3 scripts/rainbow_pgmode.py \
  --ips "${ROBOTS[@]}" \
  --port "$PORT" \
  --timeout-sec "$TIMEOUT_SEC" \
  "$MODE_FLAG" \
  --summary-json "$SUMMARY_JSON" \
  --i-understand-this-connects-to-real-controller
