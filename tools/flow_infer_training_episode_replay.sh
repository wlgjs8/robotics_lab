#!/usr/bin/env bash
# Teacher-force the live OpenPI server with one saved training episode and send
# only the model predictions to the rbpodo pgmode controller-simulation stack.
set -euo pipefail
cd "$(dirname "$0")/.."

EPISODE="${FLOW_TRAINING_EPISODE_HDF5:-${1:-}}"
if [ -z "$EPISODE" ]; then
  echo "usage: FLOW_TRAINING_EPISODE_HDF5=/path/episode.hdf5 $0" >&2
  exit 2
fi
if [ "${1:-}" = "$EPISODE" ]; then
  shift
fi

OUTPUT_DIR="${FLOW_TRAINING_EPISODE_OUTPUT_DIR:-outputs/training_episode_replay}"
VIDEO_DIR="${FLOW_TRAINING_EPISODE_VIDEO_DIR:-}"
PARQUET="${FLOW_TRAINING_EPISODE_PARQUET:-}"
RETARGET="${FLOW_TRAINING_EPISODE_RETARGET_CONFIG:-calibration/umi_retarget_eelocal.yaml}"

export FLOW_INFER_PYTHON="${FLOW_INFER_PYTHON:-/home/plaif/openpi/.venv/bin/python}"
export FLOW_INFER_CONFIG="policy_runner/config/flow_sim_offline.yaml"
export FLOW_INFER_ROLLOUT_MODE="controller_sim"
export FLOW_INFER_ACTION_HORIZON="24"
export FLOW_INFER_CHUNK_EXECUTE_STEPS="${FLOW_INFER_CHUNK_EXECUTE_STEPS:-3}"
export FLOW_INFER_CHUNK_OVERLAY_RUNWAY_STEPS="0"
export FLOW_INFER_SPEED_SCALE="${FLOW_INFER_SPEED_SCALE:-0.1}"
export FLOW_INFER_SEQUENTIAL="1"
export FLOW_INFER_CHUNK_ANCHOR="chain"
export FLOW_INFER_CHUNK_CROSSFADE_STEPS="0"
export FLOW_INFER_TCP_REANCHOR_MODE="last_emitted_continuous"
export FLOW_INFER_STITCH="boundary"
export FLOW_INFER_RTC="0"
export FLOW_INFER_VELPROPRIO_SAMPLE="replan"
export FLOW_INFER_VELPROPRIO_SOURCE="measured"
export OPENPI_REMOTE_SKIP_WARMUP="${OPENPI_REMOTE_SKIP_WARMUP:-1}"
export FLOW_INFER_PRINT_TRACKING="${FLOW_INFER_PRINT_TRACKING:-1}"
export FLOW_INFER_ROLLOUT_SUMMARY="${FLOW_INFER_ROLLOUT_SUMMARY:-$OUTPUT_DIR/rollout_summary.json}"
export POLICY_RUNNER_ACTION_LOG="${POLICY_RUNNER_ACTION_LOG:-$OUTPUT_DIR/actions.jsonl}"
mkdir -p "$OUTPUT_DIR"

EXTRA=(
  --training-episode-hdf5 "$EPISODE"
  --training-episode-retarget-config "$RETARGET"
  --training-episode-output-dir "$OUTPUT_DIR"
  --proprio-mode velocity
  --rotation-axes xyz
  --execute-arms both
  --depth-z-near-mm 50
  --depth-z-far-mm 700
  --depth-units-m 0.0001
)
if [ -n "$VIDEO_DIR" ]; then
  EXTRA+=(--training-episode-video-dir "$VIDEO_DIR")
fi
if [ -n "$PARQUET" ]; then
  EXTRA+=(--training-episode-parquet "$PARQUET")
fi

echo "[training-replay] episode=$EPISODE"
echo "[training-replay] exact_training_video=${VIDEO_DIR:-<raw-hdf5-images>}"
echo "[training-replay] output=$OUTPUT_DIR controller=pgmode-simulation live_camera=disabled"
echo "[training-replay] exact_window=W${FLOW_INFER_CHUNK_EXECUTE_STEPS} runway=0 crossfade=0 speed_scale=${FLOW_INFER_SPEED_SCALE}"
exec ./tools/flow_infer_real_policy.sh "${EXTRA[@]}" "$@"
