#!/usr/bin/env bash
# Switch both boxes to REAL operation mode. Firmware 8.7.3 (build 26071103)
# ORDER CONTRACT: 'Real Mode' is only available AFTER the arm is activated
# (UI error M186 otherwise; CM wiki enable-on-real-boxes: the activation
# sequence itself carries the mode flip). Run AFTER mc jall init completes —
# robot_on.sh does the full ordered sequence; this script is the mode half.
set -u
set -o pipefail

PORT="5000"
ROBOTS=("172.28.60.200" "172.28.60.201")

ok=0; fail=0
for ip in "${ROBOTS[@]}"; do
  echo "[INFO] pgmode real -> $ip:$PORT ..."
  if resp=$(nc -w 2 "$ip" "$PORT" <<'EOF'
pgmode real
EOF
  ); then
    echo "$resp   $ip"; ok=$((ok+1))
  else
    echo "[FAIL] $ip"; fail=$((fail+1))
  fi
done
echo "-----------------------------"
echo "[SUMMARY] ok=$ok fail=$fail total=${#ROBOTS[@]}"
[ "$fail" -eq 0 ]
