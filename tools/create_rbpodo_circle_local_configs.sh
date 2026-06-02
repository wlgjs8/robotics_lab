#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: tools/create_rbpodo_circle_local_configs.sh [--force] [--include-500hz] [--include-goal] [--root DIR]

Create operator-local rbpodo controller-simulation circle configs from the
repository templates.

Options:
  --force         overwrite existing local files
  --include-500hz also create staged 500 Hz controller-simulation templates
  --include-goal  also create the named ACKON500 best goal profile
  --root DIR      repository root to write into, for tests or alternate checkouts
  -h, --help      show this help
EOF
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root_dir="$(cd "${script_dir}/.." && pwd)"
force=0
include_500hz=0
include_goal=0

while (($# > 0)); do
  case "$1" in
    --force)
      force=1
      shift
      ;;
    --include-500hz)
      include_500hz=1
      shift
      ;;
    --include-goal)
      include_goal=1
      shift
      ;;
    --root)
      if (($# < 2)); then
        echo "error: --root requires a directory" >&2
        exit 2
      fi
      root_dir="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "error: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

root_dir="$(cd "${root_dir}" && pwd)"
config_dir="${root_dir}/rb_servo_server/config"
local_dir="${config_dir}/local"

sources=(
  "dual_real_rbpodo_circle_15cm16s.example.yaml"
  "dual_real_rbpodo_circle_15cm4s.example.yaml"
)
destinations=(
  "dual_real_rbpodo_circle_15cm16s.yaml"
  "dual_real_rbpodo_circle_15cm4s.yaml"
)

if [[ "${include_500hz}" -eq 1 ]]; then
  sources+=(
    "dual_real_rbpodo_circle_5cm10s_500hz.example.yaml"
    "dual_real_rbpodo_circle_15cm16s_500hz.example.yaml"
    "dual_real_rbpodo_circle_15cm8s_500hz.example.yaml"
    "dual_real_rbpodo_circle_15cm4s_500hz.example.yaml"
  )
  destinations+=(
    "dual_real_rbpodo_circle_5cm10s_500hz.yaml"
    "dual_real_rbpodo_circle_15cm16s_500hz.yaml"
    "dual_real_rbpodo_circle_15cm8s_500hz.yaml"
    "dual_real_rbpodo_circle_15cm4s_500hz.yaml"
  )
fi

if [[ "${include_500hz}" -eq 1 || "${include_goal}" -eq 1 ]]; then
  sources+=(
    "dual_real_rbpodo_circle_15cm4s_500hz_goal.example.yaml"
  )
  destinations+=(
    "dual_real_rbpodo_circle_15cm4s_500hz_goal.yaml"
  )
fi

for src in "${sources[@]}"; do
  if [[ ! -f "${config_dir}/${src}" ]]; then
    echo "error: template not found: ${config_dir}/${src}" >&2
    exit 1
  fi
done

mkdir -p "${local_dir}"

blocked=0
for dst in "${destinations[@]}"; do
  if [[ -e "${local_dir}/${dst}" && "${force}" -ne 1 ]]; then
    echo "error: refusing to overwrite existing local config: ${local_dir}/${dst}" >&2
    blocked=1
  fi
done

if [[ "${blocked}" -ne 0 ]]; then
  echo "Run again with --force only after reviewing any local operator edits." >&2
  exit 1
fi

for i in "${!sources[@]}"; do
  cp "${config_dir}/${sources[$i]}" "${local_dir}/${destinations[$i]}"
  echo "created ${local_dir}/${destinations[$i]}"
done

cat <<EOF

Next checks:
  grep -H "allow_in_controller_simulation: true" "${local_dir}"/dual_real_rbpodo_circle_15cm*.yaml
  grep -H "allow_in_real: false" "${local_dir}"/dual_real_rbpodo_circle_15cm*.yaml

Stable controller-simulation benchmark uses:
  --server-config rb_servo_server/config/local/dual_real_rbpodo_circle_15cm16s.yaml

GENE-style stress benchmark uses:
  --server-config rb_servo_server/config/local/dual_real_rbpodo_circle_15cm4s.yaml

Staged 500 Hz configs are created only when --include-500hz is passed:
  tools/create_rbpodo_circle_local_configs.sh --include-500hz
  --server-config rb_servo_server/config/local/dual_real_rbpodo_circle_5cm10s_500hz.yaml
  --server-config rb_servo_server/config/local/dual_real_rbpodo_circle_15cm16s_500hz.yaml
  --server-config rb_servo_server/config/local/dual_real_rbpodo_circle_15cm8s_500hz.yaml
  --server-config rb_servo_server/config/local/dual_real_rbpodo_circle_15cm4s_500hz.yaml
  --server-config rb_servo_server/config/local/dual_real_rbpodo_circle_15cm4s_500hz_goal.yaml
  grep -H "servo_t1_sec: 0.002" "${local_dir}"/*_500hz.yaml
  grep -H "operation_mode: simulation" "${local_dir}"/*_500hz.yaml

The named ACKON500 best goal profile is created with --include-goal or
--include-500hz:
  tools/create_rbpodo_circle_local_configs.sh --include-goal
  --server-config rb_servo_server/config/local/dual_real_rbpodo_circle_15cm4s_500hz_goal.yaml

Required env gates include:
  RB_ALLOW_REAL_ROBOT=1
  RB_ALLOW_REAL_MOTION=1
  RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1
  RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN=1
EOF
