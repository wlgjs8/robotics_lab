#!/usr/bin/env bash
# Box bring-up for firmware 8.7.3 (build 26071103, 2026-08-16 update).
# ORDER REVERSED vs the pre-8.7.3 script: on 8.7.3 'Real Mode' is only
# available AFTER the arm is activated (UI error M186 if attempted before;
# confirmed live 2026-08-16 + CM wiki enable-on-real-boxes: the activation
# sequence itself carries a REAL-mode flip). Sequence per box:
#   1. mc jall init      (activation — allowed in Simulation)
#   2. wait for the activation to complete (real hardware ~13.4 s)
#   3. pgmode real       (tools/real_mode.sh)
# NOTE: dismiss any modal dialog on the tablet UI first (a pending M186/update
# dialog can block the activation state machine).
set -u
set -o pipefail

PORT="5000"
ROBOTS=("172.28.60.200" "172.28.60.201")
ACTIVATION_WAIT_S="${ACTIVATION_WAIT_S:-18}"

ok=0; fail=0
for ip in "${ROBOTS[@]}"; do
  echo "[INFO] mc jall init -> $ip:$PORT ..."
  if resp=$(nc -w 2 "$ip" "$PORT" <<'EOF'
mc jall init
EOF
  ); then
    echo "$resp   $ip"; ok=$((ok+1))
  else
    echo "[FAIL] $ip"; fail=$((fail+1))
  fi
done

if [ "$fail" -ne 0 ]; then
  echo "[SUMMARY] init send failed on $fail box(es) — not switching mode."; exit 1
fi

echo "[INFO] waiting ${ACTIVATION_WAIT_S}s for activation (real hardware ~13.4s) ..."
sleep "$ACTIVATION_WAIT_S"

exec "$(dirname "$0")/real_mode.sh"
