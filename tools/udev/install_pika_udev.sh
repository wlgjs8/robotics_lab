#!/usr/bin/env bash
# Install legacy fixed-port /dev/pika-left and /dev/pika-right symlinks for
# direct diagnostics. The main make-run stack auto-pairs from camera.health and
# does not use these links. Needs sudo.
#
#   tools/udev/install_pika_udev.sh
#
# After install, replug the grippers (or reboot) once so the rule fires, then:
#   ls -l /dev/pika-*
set -euo pipefail
cd "$(dirname "$0")"

RULE="99-pika-grippers.rules"
DEST="/etc/udev/rules.d/$RULE"

[ -f "$RULE" ] || { echo "missing $RULE next to this script" >&2; exit 1; }

echo "[pika-udev] installing $RULE -> $DEST (sudo)"
echo "[pika-udev] WARNING: legacy fixed-port mapping; main make run does not need this"
sudo cp "$RULE" "$DEST"
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty --action=add
sudo udevadm settle --timeout=10

echo "[pika-udev] done. Current symlinks:"
missing=0
for link in /dev/pika-left /dev/pika-right; do
  if [ -e "$link" ]; then
    ls -l "$link"
  else
    echo "[pika-udev] missing $link" >&2
    missing=1
  fi
done
if [ "$missing" = "1" ]; then
  echo "[pika-udev] unplug/replug the missing gripper(s) or reboot, then: ls -l /dev/pika-*" >&2
fi
