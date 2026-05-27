#!/usr/bin/env bash
set -u
set -o pipefail

PORT="5000"

ROBOTS=(
  "172.28.60.200"
  "172.28.60.201"
)

send_sim() {
  local ip="$1"

  # 공백-only 라인 제거 + CR 제거
  cat <<'EOF' | sed -e 's/\r$//' -e '/^[[:space:]]*$/d' | nc -w 2 "$ip" "$PORT"
pgmode simulation
EOF
}

ok=0
fail=0
failed_ips=()

for ip in "${ROBOTS[@]}"; do
  echo "[INFO] Setting SIM mode on $ip:$PORT ..."
  if send_sim "$ip"; then
    echo "[OK]   $ip"
    ok=$((ok+1))
  else
    echo "[FAIL] $ip" >&2
    fail=$((fail+1))
    failed_ips+=("$ip")
  fi
done

echo "-----------------------------"
echo "[SUMMARY] ok=$ok fail=$fail total=$((ok+fail))"
if (( fail > 0 )); then
  echo "[FAILED IPs] ${failed_ips[*]}" >&2
  exit 1
fi