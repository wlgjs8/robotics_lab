#!/usr/bin/env bash
# Record the 4 local-checkpoint rollouts (flow, direct_bc) x (train, val) into viser mp4s.
set -uo pipefail
cd "$(dirname "$0")/.."
export DISPLAY=:96

MODELS_DIR=/home/plaif/pika_umi_models_v2
TRAIN=/home/plaif/workspace/robotics_lab/data_tcp/data_20260606_134608/episode_000.hdf5
VAL=/home/plaif/workspace/robotics_lab/data_tcp/data_20260606_175635/episode_000.hdf5

run() { echo "##### $3 #####"; DISPLAY=:96 scripts/record_rollout.sh "$1" "$2" "$3" 0.0334; echo; }

run "$MODELS_DIR/flow/checkpoint.pt"       "$TRAIN" flow_train
run "$MODELS_DIR/flow/checkpoint.pt"       "$VAL"   flow_val
run "$MODELS_DIR/direct_bc/checkpoint.pt"  "$TRAIN" direct_bc_train
run "$MODELS_DIR/direct_bc/checkpoint.pt"  "$VAL"   direct_bc_val

echo "===== ALL LOCAL ROLLOUTS DONE ====="
ls -la outputs/replay_videos/
