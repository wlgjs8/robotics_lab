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

{
  echo "tag=$TAG"
  echo "stamp=$STAMP"
  echo "args=$*"
  env | grep -E '^FLOW_INFER_|^OPENPI_REMOTE_|^RB_ALLOW_' | sort
} > "$META"
echo "[sweep] step_log=$STEP_LOG"
echo "[sweep] meta=$META"

FLOW_INFER_STEP_LOG="$STEP_LOG" ./tools/flow_infer_real_policy.sh "$@"
