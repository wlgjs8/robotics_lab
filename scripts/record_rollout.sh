#!/usr/bin/env bash
# Record one offline-replay rollout from viser (Xvfb :96) to mp4 + screenshot.
#   record_rollout.sh <checkpoint> <episode_hdf5> <out_basename> [policy_dt_sec]
# Pre-moves the arms to the folded rest pose (so the clip starts clean), records
# the viser canvas while the replay drives the sim robot, then grabs a final frame.
set -uo pipefail
cd "$(dirname "$0")/.."

CKPT="$1"; EP="$2"; OUT="$3"; PDT="${4:-0.0334}"
DISP=":96"
GEOM="1600x1000"
OUTDIR="outputs/replay_videos"
mkdir -p "$OUTDIR"
MP4="$OUTDIR/${OUT}.mp4"
PNG="$OUTDIR/${OUT}.png"
VENV=~/openpi/.venv/bin/python

echo "[rec] === $OUT ==="
echo "[rec] init arms to rest pose..."
PYTHONPATH=policy_runner $VENV -u /tmp/init_pose.py >/tmp/init_${OUT}.log 2>&1
tail -1 /tmp/init_${OUT}.log

echo "[rec] start ffmpeg capture of $DISP -> $MP4"
ffmpeg -y -f x11grab -framerate 20 -video_size "$GEOM" -i "$DISP" \
  -vf "crop=1180:1000:0:0" -c:v libx264 -preset veryfast -pix_fmt yuv420p "$MP4" \
  >/tmp/ffmpeg_${OUT}.log 2>&1 &
FFPID=$!
sleep 1.5

echo "[rec] run replay driver (no-init)..."
PYTHONPATH=policy_runner $VENV -u scripts/replay_episode_rollout.py \
  --config policy_runner/config/replay_sim.yaml \
  --checkpoint "$CKPT" --episode "$EP" --policy-dt-sec "$PDT" --no-init \
  >/tmp/replay_${OUT}.log 2>&1
RC=$?
echo "[rec] replay rc=$RC"; tail -2 /tmp/replay_${OUT}.log

# hold final pose on screen briefly, then stop capture
sleep 2
kill -INT "$FFPID" 2>/dev/null
wait "$FFPID" 2>/dev/null
sleep 0.5

# final screenshot from the tail of the recording
ffmpeg -y -sseof -1.5 -i "$MP4" -frames:v 1 "$PNG" >/dev/null 2>&1
echo "[rec] wrote $MP4 ($(du -h "$MP4" 2>/dev/null | cut -f1)) and $PNG"
ls -la "$MP4" "$PNG" 2>/dev/null
