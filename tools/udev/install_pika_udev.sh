#!/usr/bin/env bash
# Install the stable /dev/pika-left and /dev/pika-right symlinks for the two
# robot-side Pika Grippers (see 99-pika-grippers.rules for why USB port path is
# the only stable discriminator). Needs sudo.
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
sudo cp "$RULE" "$DEST"
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty --action=add

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
