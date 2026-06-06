#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: tools/create_rbpodo_pgmode_spacemouse_local_config.sh [options]

Create the ignored local rbpodo pgmode SpaceMouse server config from the
tracked .example.yaml template.

Options:
  --left-ip IP              left controller IP or site-local hostname
  --right-ip IP             right controller IP or site-local hostname
  --command-port PORT       command port for both controllers
  --data-port PORT          data port for both controllers
  --left-command-port PORT  left command port
  --right-command-port PORT right command port
  --left-data-port PORT     left data port
  --right-data-port PORT    right data port
  --output PATH             output path, default rb_servo_server/config/local/...
  --root DIR                repository root, for tests or alternate checkouts
  --force                   overwrite an existing local file
  -h, --help                show this help

Environment:
  RB_PGMODE_SPACEMOUSE_LEFT_IP
  RB_PGMODE_SPACEMOUSE_RIGHT_IP
  RB_PGMODE_SPACEMOUSE_COMMAND_PORT
  RB_PGMODE_SPACEMOUSE_DATA_PORT
  RB_PGMODE_SPACEMOUSE_LEFT_COMMAND_PORT
  RB_PGMODE_SPACEMOUSE_RIGHT_COMMAND_PORT
  RB_PGMODE_SPACEMOUSE_LEFT_DATA_PORT
  RB_PGMODE_SPACEMOUSE_RIGHT_DATA_PORT
EOF
}

fail() {
  echo "create_rbpodo_pgmode_spacemouse_local_config: ERROR: $*" >&2
  exit 1
}

note() {
  echo "create_rbpodo_pgmode_spacemouse_local_config: $*"
}

validate_port() {
  local name="$1"
  local value="$2"
  if [[ ! "${value}" =~ ^[0-9]+$ ]] || ((value < 1 || value > 65535)); then
    fail "${name} must be an integer port in 1..65535"
  fi
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root_dir="$(cd "${script_dir}/.." && pwd)"
template_rel="rb_servo_server/config/dual_real_rbpodo_pgmode_spacemouse_500hz_ack.example.yaml"
output_rel="rb_servo_server/config/local/dual_real_rbpodo_pgmode_spacemouse_500hz_ack.yaml"
output_path="${output_rel}"
force=0

left_ip="${RB_PGMODE_SPACEMOUSE_LEFT_IP:-}"
right_ip="${RB_PGMODE_SPACEMOUSE_RIGHT_IP:-}"
command_port="${RB_PGMODE_SPACEMOUSE_COMMAND_PORT:-}"
data_port="${RB_PGMODE_SPACEMOUSE_DATA_PORT:-}"
left_command_port="${RB_PGMODE_SPACEMOUSE_LEFT_COMMAND_PORT:-}"
right_command_port="${RB_PGMODE_SPACEMOUSE_RIGHT_COMMAND_PORT:-}"
left_data_port="${RB_PGMODE_SPACEMOUSE_LEFT_DATA_PORT:-}"
right_data_port="${RB_PGMODE_SPACEMOUSE_RIGHT_DATA_PORT:-}"

while (($# > 0)); do
  case "$1" in
    --left-ip)
      [[ $# -ge 2 ]] || fail "--left-ip requires a value"
      left_ip="$2"
      shift 2
      ;;
    --right-ip)
      [[ $# -ge 2 ]] || fail "--right-ip requires a value"
      right_ip="$2"
      shift 2
      ;;
    --command-port)
      [[ $# -ge 2 ]] || fail "--command-port requires a value"
      command_port="$2"
      shift 2
      ;;
    --data-port)
      [[ $# -ge 2 ]] || fail "--data-port requires a value"
      data_port="$2"
      shift 2
      ;;
    --left-command-port)
      [[ $# -ge 2 ]] || fail "--left-command-port requires a value"
      left_command_port="$2"
      shift 2
      ;;
    --right-command-port)
      [[ $# -ge 2 ]] || fail "--right-command-port requires a value"
      right_command_port="$2"
      shift 2
      ;;
    --left-data-port)
      [[ $# -ge 2 ]] || fail "--left-data-port requires a value"
      left_data_port="$2"
      shift 2
      ;;
    --right-data-port)
      [[ $# -ge 2 ]] || fail "--right-data-port requires a value"
      right_data_port="$2"
      shift 2
      ;;
    --output)
      [[ $# -ge 2 ]] || fail "--output requires a path"
      output_path="$2"
      shift 2
      ;;
    --root)
      [[ $# -ge 2 ]] || fail "--root requires a directory"
      root_dir="$2"
      shift 2
      ;;
    --force)
      force=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      fail "unknown argument: $1"
      ;;
  esac
done

root_dir="$(cd "${root_dir}" && pwd)"
template="${root_dir}/${template_rel}"
case "${output_path}" in
  /*)
    output="${output_path}"
    ;;
  *)
    output="${root_dir}/${output_path}"
    ;;
esac

left_ip="${left_ip:-192.0.2.10}"
right_ip="${right_ip:-192.0.2.11}"
left_command_port="${left_command_port:-${command_port:-5000}}"
right_command_port="${right_command_port:-${command_port:-5000}}"
left_data_port="${left_data_port:-${data_port:-5001}}"
right_data_port="${right_data_port:-${data_port:-5001}}"

[[ -n "${left_ip}" ]] || fail "left controller IP is empty"
[[ -n "${right_ip}" ]] || fail "right controller IP is empty"
validate_port "--left-command-port" "${left_command_port}"
validate_port "--right-command-port" "${right_command_port}"
validate_port "--left-data-port" "${left_data_port}"
validate_port "--right-data-port" "${right_data_port}"
[[ -f "${template}" ]] || fail "template not found: ${template_rel}"

if [[ -e "${output}" && "${force}" != "1" ]]; then
  fail "refusing to overwrite existing local config: ${output}; use --force after reviewing local edits"
fi

mkdir -p "$(dirname "${output}")"
tmp="${output}.tmp.$$"
trap 'rm -f "${tmp}"' EXIT

awk \
  -v left_ip="${left_ip}" \
  -v right_ip="${right_ip}" \
  -v left_command_port="${left_command_port}" \
  -v right_command_port="${right_command_port}" \
  -v left_data_port="${left_data_port}" \
  -v right_data_port="${right_data_port}" '
  /^left_robot:/ { section = "left"; print; next }
  /^right_robot:/ { section = "right"; print; next }
  /^[^[:space:]#][^:]*:/ && $0 !~ /^(left_robot|right_robot):/ { section = "" }
  section == "left" && $0 ~ /^[[:space:]]+ip:[[:space:]]/ {
    print "  ip: \"" left_ip "\""
    next
  }
  section == "right" && $0 ~ /^[[:space:]]+ip:[[:space:]]/ {
    print "  ip: \"" right_ip "\""
    next
  }
  section == "left" && $0 ~ /^[[:space:]]+command_port:[[:space:]]/ {
    print "  command_port: " left_command_port
    next
  }
  section == "right" && $0 ~ /^[[:space:]]+command_port:[[:space:]]/ {
    print "  command_port: " right_command_port
    next
  }
  section == "left" && $0 ~ /^[[:space:]]+data_port:[[:space:]]/ {
    print "  data_port: " left_data_port
    next
  }
  section == "right" && $0 ~ /^[[:space:]]+data_port:[[:space:]]/ {
    print "  data_port: " right_data_port
    next
  }
  { print }
' "${template}" >"${tmp}"

mv "${tmp}" "${output}"
trap - EXIT

note "created ${output}"
note "warning: this file is ignored by git and must be reviewed locally before use"
note "review operation_mode=simulation, allow_in_real=false, controller IPs, ports, and site safety settings before connecting"
