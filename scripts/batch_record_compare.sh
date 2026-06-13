#!/usr/bin/env bash
# Record BLIND (pre-fix: camera_names=[]) vs VISION (fixed) flow rollouts and
# build a side-by-side comparison mp4. Open-loop (recorded proprio) so the clip
# shows the model's true output quality, not closed-loop divergence.
set -uo pipefail
cd "$(dirname "$0")/.."
export DISPLAY=:96
FLOW=/home/plaif/pika_umi_models_v2/flow/checkpoint.pt
TRAIN=/home/plaif/workspace/robotics_lab/data_tcp/data_20260606_134608/episode_000.hdf5
VAL=/home/plaif/workspace/robotics_lab/data_tcp/data_20260606_175635/episode_000.hdf5
OUTDIR=outputs/replay_videos

rec() { echo "### $3"; DISPLAY=:96 scripts/record_rollout.sh "$FLOW" "$1" "$3" 0.0334 "$2"; }

for pair in "TRAIN:$TRAIN" "VAL:$VAL"; do
  tag="${pair%%:*}"; ep="${pair#*:}"; lc=$(echo "$tag" | tr A-Z a-z)
  rec "$ep" "--blind --open-loop"  "flow_${lc}_blind"
  rec "$ep" "--open-loop"          "flow_${lc}_vision"
  b="$OUTDIR/flow_${lc}_blind.mp4"; v="$OUTDIR/flow_${lc}_vision.mp4"
  out="$OUTDIR/flow_${lc}_fix_compare.mp4"
  ffmpeg -y -i "$b" -i "$v" -filter_complex \
    "[0:v]drawtext=text='BLIND (pre-fix\: no camera)':x=12:y=12:fontsize=26:fontcolor=white:box=1:boxcolor=red@0.6[a];\
     [1:v]drawtext=text='VISION (fixed)':x=12:y=12:fontsize=26:fontcolor=white:box=1:boxcolor=green@0.6[c];\
     [a][c]hstack=shortest=1" -c:v libx264 -preset veryfast -pix_fmt yuv420p "$out" >/tmp/hstack_${lc}.log 2>&1 \
    && echo "made $out" || { echo "hstack FAILED for $lc"; tail -3 /tmp/hstack_${lc}.log; }
done
echo "===== COMPARE DONE ====="; ls -la $OUTDIR/*fix_compare.mp4 2>/dev/null
