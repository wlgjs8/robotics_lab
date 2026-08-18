#!/usr/bin/env bash
# One ablation run of flow_infer_real_policy.sh with an auto-named rollout-step
# log plus a sidecar .meta capturing the exact configuration, so runs stay
# comparable in scripts/compare_rollout_step_logs.py.
#
# Usage (identical to flow_infer_real_policy.sh, plus a leading TAG):
#   FLOW_INFER_SPEED_SCALE=1.0 ./tools/flow_infer_sweep_run.sh speed1.0 \
#     --proprio-mode velocity --depth-z-near-mm 50 --depth-z-far-mm 700 --depth-units-m 1e-4
#
# After the run:
#   python3 scripts/analyze_rollout_step_log.py outputs/sweep/<stamp>_<tag>.jsonl
#   python3 scripts/compare_rollout_step_logs.py outputs/sweep/*.jsonl
set -euo pipefail
cd "$(dirname "$0")/.."

if [ $# -lt 1 ]; then
  echo "usage: $0 <run-tag> [flow_infer_real_policy.sh args...]" >&2
  exit 2
fi
TAG="$1"
shift

STAMP="$(date +%Y%m%d_%H%M%S)"
SWEEP_DIR="outputs/sweep"
mkdir -p "$SWEEP_DIR"
STEP_LOG="$SWEEP_DIR/${STAMP}_${TAG}.jsonl"
META="$SWEEP_DIR/${STAMP}_${TAG}.meta"
# Persist the runner console (tick-profile / tick-spike lines go to stderr and
# were previously lost with the terminal scrollback; the 2026-08-14 loop-freeze
# diagnosis needs them next to the step log).
CONSOLE_LOG="$SWEEP_DIR/${STAMP}_${TAG}.console.log"

{
  echo "tag=$TAG"
  echo "stamp=$STAMP"
  echo "args=$*"
  env | grep -E '^FLOW_INFER_|^OPENPI_REMOTE_|^RB_ALLOW_' | sort
} > "$META"
echo "[sweep] step_log=$STEP_LOG"
echo "[sweep] meta=$META"
echo "[sweep] console=$CONSOLE_LOG"

# Observation dump beside the step log (lossless PNG per inference, ~0.5-1 MB each at 7.5 Hz ->
# a few hundred MB per rollout). Opt out with FLOW_INFER_OBS_DUMP=0.
if [ "${FLOW_INFER_OBS_DUMP:-1}" != 0 ]; then
  export FLOW_INFER_OBS_DUMP_DIR="${FLOW_INFER_OBS_DUMP_DIR:-outputs/obs_dump/${STAMP}_${TAG}}"
  echo "[sweep] obs_dump=$FLOW_INFER_OBS_DUMP_DIR"
  echo "obs_dump=$FLOW_INFER_OBS_DUMP_DIR" >> "$META"
fi

FLOW_INFER_STEP_LOG="$STEP_LOG" ./tools/flow_infer_real_policy.sh "$@" 2>&1 | tee "$CONSOLE_LOG"
