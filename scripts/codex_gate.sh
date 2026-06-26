#!/usr/bin/env bash
set -euo pipefail

TASK="${1:-}"

if [[ -z "$TASK" ]]; then
  echo "Usage: $0 <TASK_ID>" >&2
  exit 2
fi

if git rev-parse --show-toplevel >/dev/null 2>&1; then
  REPO_ROOT="$(git rev-parse --show-toplevel)"
else
  REPO_ROOT="$(pwd)"
fi
cd "$REPO_ROOT"

run_with_timeout() {
  local seconds="$1"
  shift

  local timeout_cmd=""
  if command -v timeout >/dev/null 2>&1 && timeout --version 2>/dev/null | grep -qi "GNU coreutils"; then
    timeout_cmd="timeout"
  elif command -v gtimeout >/dev/null 2>&1 && gtimeout --version 2>/dev/null | grep -qi "GNU coreutils"; then
    timeout_cmd="gtimeout"
  fi

  if [[ -z "${timeout_cmd}" ]]; then
    echo "codex_gate: warning: GNU timeout unavailable; running without ${seconds}s limit: $*" >&2
    "$@"
    return
  fi

  "${timeout_cmd}" "${seconds}s" "$@"
}

cmake_prefix_args() {
  if [[ -n "${CMAKE_PREFIX_PATH:-}" ]]; then
    printf '%s\n' "-DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}"
  elif [[ -d "${HOME}/miniconda3/share/cmake/nlohmann_json" || -d "${HOME}/miniconda3/lib/cmake/nlohmann_json" ]]; then
    printf '%s\n' "-DCMAKE_PREFIX_PATH=${HOME}/miniconda3"
  elif [[ -d "/opt/openrobots" ]]; then
    printf '%s\n' "-DCMAKE_PREFIX_PATH=/opt/openrobots"
  fi
}

cmake_package_available() {
  local package_name="$1"
  local tmpdir
  tmpdir="$(mktemp -d)"
  cat > "${tmpdir}/CMakeLists.txt" <<EOF_CMAKE
cmake_minimum_required(VERSION 3.16)
project(check_${package_name} LANGUAGES CXX)
find_package(${package_name} REQUIRED)
EOF_CMAKE

  local args=()
  mapfile -t args < <(cmake_prefix_args)

  if cmake -S "${tmpdir}" -B "${tmpdir}/build" "${args[@]}" >/dev/null 2>&1; then
    rm -rf "${tmpdir}"
    return 0
  fi
  rm -rf "${tmpdir}"
  return 1
}

cpp_base_deps_available() {
  cmake_package_available yaml-cpp && cmake_package_available nlohmann_json
}

rb_servo_cpp_deps_available() {
  cpp_base_deps_available && cmake_package_available Eigen3 && cmake_package_available pinocchio
}

run_ctest_with_retry() {
  local test_dir="$1"
  if ctest --test-dir "${test_dir}" --output-on-failure; then
    return 0
  fi
  echo "codex_gate: ctest failed in ${test_dir}; rerunning failed tests once" >&2
  ctest --test-dir "${test_dir}" --rerun-failed --output-on-failure
}

run_servo_gate() {
  local args=()
  mapfile -t args < <(cmake_prefix_args)

  cmake -S rb_servo_server -B rb_servo_server/build/hardware_free_gate \
    -DCMAKE_BUILD_TYPE=Debug \
    -DRB_SERVO_ENABLE_RBPODO=OFF \
    -DRB_SERVO_ALLOW_FETCHCONTENT=OFF \
    -DBUILD_TESTING=ON \
    "${args[@]}"

  cmake --build rb_servo_server/build/hardware_free_gate -j
  run_ctest_with_retry rb_servo_server/build/hardware_free_gate
}

build_servo_server_only() {
  local args=()
  mapfile -t args < <(cmake_prefix_args)

  cmake -S rb_servo_server -B rb_servo_server/build/hardware_free_gate \
    -DCMAKE_BUILD_TYPE=Debug \
    -DRB_SERVO_ENABLE_RBPODO=OFF \
    -DRB_SERVO_ALLOW_FETCHCONTENT=OFF \
    -DBUILD_TESTING=ON \
    "${args[@]}"

  cmake --build rb_servo_server/build/hardware_free_gate --target rb_servo_server -j
}

run_servo_gate_or_skip_missing_deps() {
  if rb_servo_cpp_deps_available; then
    run_servo_gate
    return 0
  fi

  if [[ "${CODEX_SKIP_MISSING_CPP_DEPS:-0}" == "1" ]]; then
    echo "codex_gate: skipping rb_servo_server C++ gate; required CMake package missing: yaml-cpp, nlohmann_json, Eigen3, or pinocchio"
    return 0
  fi

  echo "ERROR: missing rb_servo_server C++ dependencies: yaml-cpp, nlohmann_json, Eigen3, and pinocchio are required" >&2
  echo "Install on Ubuntu with: scripts/install_deps_ubuntu.sh --profile hardware-free" >&2
  echo "That installs robotpkg-pinocchio under /opt/openrobots; for conda/mamba/source installs, set CMAKE_PREFIX_PATH to the install prefix." >&2
  echo "Or rerun with CODEX_SKIP_MISSING_CPP_DEPS=1 to skip C++ gates temporarily." >&2
  return 1
}

run_servo_pinocchio_gate() {
  if ! rb_servo_cpp_deps_available; then
    if [[ "${CODEX_SKIP_MISSING_CPP_DEPS:-0}" == "1" ]]; then
      echo "codex_gate: skipping mandatory rb_servo_server Pinocchio gate; required CMake package missing"
      return 0
    fi
    echo "ERROR: cannot run rb_servo_server Pinocchio gate because required C++ deps are missing" >&2
    echo "Missing CMake package: pinocchio" >&2
    return 1
  fi

  local args=()
  mapfile -t args < <(cmake_prefix_args)

  cmake -S rb_servo_server -B rb_servo_server/build/pinocchio_gate \
    -DCMAKE_BUILD_TYPE=Debug \
    -DRB_SERVO_ENABLE_RBPODO=OFF \
    -DRB_SERVO_ALLOW_FETCHCONTENT=OFF \
    -DBUILD_TESTING=ON \
    "${args[@]}"

  cmake --build rb_servo_server/build/pinocchio_gate -j
  run_ctest_with_retry rb_servo_server/build/pinocchio_gate
}

run_camera_gate() {
  cmake -S camera_server -B camera_server/build/hardware_free_gate \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCAMERA_SERVER_FORCE_MOCK_CAMERA=ON \
    -DCAMERA_SERVER_FORCE_ZMQ_STUB=ON \
    -DCAMERA_SERVER_BUILD_TESTS=ON

  cmake --build camera_server/build/hardware_free_gate -j
  run_ctest_with_retry camera_server/build/hardware_free_gate
}

run_camera_gate_or_skip_missing_deps() {
  if cpp_base_deps_available; then
    run_camera_gate
    return 0
  fi

  if [[ "${CODEX_SKIP_MISSING_CPP_DEPS:-0}" == "1" ]]; then
    echo "codex_gate: skipping camera_server C++ gate; yaml-cpp or nlohmann_json CMake package is missing"
    return 0
  fi

  echo "ERROR: missing camera_server C++ dependencies: yaml-cpp and/or nlohmann_json" >&2
  echo "Install on Ubuntu: sudo apt-get install -y libyaml-cpp-dev nlohmann-json3-dev" >&2
  return 1
}

loopback_socket_available() {
  python3 - <<'PY'
import socket
import sys
sockets = []
try:
    for kind in (socket.SOCK_STREAM, socket.SOCK_DGRAM):
        sock = socket.socket(socket.AF_INET, kind)
        sock.bind(("127.0.0.1", 0))
        sockets.append(sock)
except OSError:
    sys.exit(1)
finally:
    for sock in sockets:
        sock.close()
PY
}

run_gui_tests() {
  python3 -m unittest discover rb_gui/tests
}

run_policy_runner_tests() {
  PYTHONPATH=policy_runner python3 -m unittest discover policy_runner/tests
}

run_python_surface_tests() {
  run_gui_tests
  run_policy_runner_tests
}

run_optional_python_help() {
  local script="$1"
  if [[ -f "${script}" ]]; then
    python3 "${script}" --help >/dev/null
  else
    echo "codex_gate: optional script not present: ${script}"
  fi
}

run_optional_script_tests() {
  local pattern="$1"
  if find scripts -maxdepth 1 -name "${pattern}" -print -quit | grep -q .; then
    PYTHONPATH=scripts python3 -m unittest discover scripts -p "${pattern}"
  else
    echo "codex_gate: optional script tests not present: ${pattern}"
  fi
}

require_file() {
  local path="$1"
  if [[ ! -f "${path}" ]]; then
    echo "ERROR: required file is missing: ${path}" >&2
    return 1
  fi
}

run_required_python_help() {
  local script="$1"
  require_file "${script}"
  python3 "${script}" --help >/dev/null
}

run_required_script_tests() {
  local pattern="$1"
  if ! find scripts -maxdepth 1 -name "${pattern}" -print -quit | grep -q .; then
    echo "ERROR: required script tests not present: ${pattern}" >&2
    return 1
  fi
  PYTHONPATH=scripts python3 -m unittest discover -s scripts -p "${pattern}"
}

run_required_policy_runner_tests() {
  local pattern="$1"
  if ! find policy_runner/tests -maxdepth 1 -name "${pattern}" -print -quit | grep -q .; then
    echo "ERROR: required policy_runner tests not present: ${pattern}" >&2
    return 1
  fi
  PYTHONPATH=policy_runner python3 -m unittest discover -s policy_runner/tests -p "${pattern}"
}

run_required_policy_runner_tests_with_timeout() {
  local seconds="$1"
  local pattern="$2"
  if ! find policy_runner/tests -maxdepth 1 -name "${pattern}" -print -quit | grep -q .; then
    echo "ERROR: required policy_runner tests not present: ${pattern}" >&2
    return 1
  fi
  run_with_timeout "${seconds}" env PYTHONPATH=policy_runner python3 -m unittest discover -s policy_runner/tests -p "${pattern}"
}

run_policy_runner_help() {
  local subcommand="$1"
  PYTHONPATH=policy_runner python3 -m policy_runner "${subcommand}" --help >/dev/null
}

run_tiny_cnn_ml_preflight_with_timeout() {
  run_with_timeout 120 env PYTHONPATH=policy_runner python3 -m policy_runner ml-preflight --vision-backbone tiny_cnn
}

run_required_make_dry_run() {
  local target="$1"
  make -n "${target}" >/dev/null
}

run_optional_rbpodo_measurement_readonly() {
  local script="$1"
  shift

  if [[ "${CODEX_RUN_RBPODO_MEASUREMENT:-0}" != "1" ]]; then
    echo "codex_gate: skipping rbpodo measurement controller read-only check; set CODEX_RUN_RBPODO_MEASUREMENT=1 with explicit args to enable"
    return 0
  fi

  if [[ ! -f "${script}" ]]; then
    echo "ERROR: CODEX_RUN_RBPODO_MEASUREMENT=1 but ${script} is missing" >&2
    return 1
  fi
  if [[ -z "${CODEX_RBPODO_MEASUREMENT_ARGS:-}" ]]; then
    echo "ERROR: CODEX_RUN_RBPODO_MEASUREMENT=1 requires CODEX_RBPODO_MEASUREMENT_ARGS with explicit read-only script arguments" >&2
    return 1
  fi

  local required_arg
  for required_arg in "$@"; do
    if [[ "${CODEX_RBPODO_MEASUREMENT_ARGS}" != *"${required_arg}"* ]]; then
      echo "ERROR: CODEX_RBPODO_MEASUREMENT_ARGS must include ${required_arg} for this read-only measurement gate" >&2
      return 1
    fi
  done

  # shellcheck disable=SC2086
  python3 "${script}" ${CODEX_RBPODO_MEASUREMENT_ARGS}
}

run_python_compile_checks() {
  python3 -m compileall -q \
    rb_gui/rb_servo_gui \
    policy_runner/policy_runner \
    scripts
}

run_shell_syntax_checks() {
  bash -n scripts/codex_gate.sh
  bash -n scripts/codex_run_sequence.sh
  bash -n scripts/check_deps.sh
  if [[ -f scripts/install_deps_ubuntu.sh ]]; then
    bash -n scripts/install_deps_ubuntu.sh
  fi
}

grep_existing() {
  local pattern="$1"
  shift

  local paths=()
  local path
  for path in "$@"; do
    if [[ -e "$path" ]]; then
      paths+=("$path")
    fi
  done

  if [[ "${#paths[@]}" -eq 0 ]]; then
    echo "ERROR: no existing paths to grep for pattern: $pattern" >&2
    return 2
  fi

  grep -R -E -- "$pattern" "${paths[@]}" >/dev/null
}

grep_absent() {
  local pattern="$1"
  shift

  local paths=()
  local path
  for path in "$@"; do
    if [[ -e "$path" ]]; then
      paths+=("$path")
    fi
  done

  if [[ "${#paths[@]}" -eq 0 ]]; then
    return 0
  fi

  if grep -R -E -- "$pattern" "${paths[@]}" >/dev/null; then
    echo "ERROR: forbidden pattern found: $pattern" >&2
    grep -R -n -E -- "$pattern" "${paths[@]}" >&2 || true
    return 1
  fi
}

run_optional_tcp_pose_acceptance() {
  # The hardware-free software-simulator TCP pose acceptance lane was retired
  # along with its deleted launcher script. cartesian_acceptance.py now only
  # runs --mode assume-running against an already-running server, so this gate
  # is reduced to a CLI smoke check of the surviving harness.
  if [[ -f scripts/cartesian_acceptance.py ]]; then
    python3 scripts/cartesian_acceptance.py --help >/dev/null
  else
    echo "codex_gate: optional Cartesian acceptance harness not present: scripts/cartesian_acceptance.py"
  fi
}

run_cart_harden_05_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts rb_servo_server/tools
  python3 rb_servo_server/tools/send_tcp_linear_move.py --help >/dev/null
  grep_existing "TcpPoseTarget|TcpLinearMove" \
    rb_servo_server/docs/network_protocol.md
  grep_existing "path_s|path_line_deviation_m|orientation preservation|quaternion" \
    rb_servo_server/docs/network_protocol.md
  if [[ "${CODEX_RUN_CARTESIAN_ACCEPTANCE:-0}" == "1" ]]; then
    run_servo_pinocchio_gate
  else
    echo "codex_gate: skipping full Cartesian Pinocchio gate; set CODEX_RUN_CARTESIAN_ACCEPTANCE=1 to enable"
  fi
}

run_cart_math_gate() {
  run_shell_syntax_checks
  run_python_compile_checks
  run_gui_tests
  run_policy_runner_tests
  local pinocchio_var
  local forbidden_pinocchio_off
  pinocchio_var='RB_SERVO_ENABLE_PINOCCHIO'
  forbidden_pinocchio_off="-D${pinocchio_var}=OFF"
  grep_absent "${forbidden_pinocchio_off}" scripts rb_servo_server README.md docs AGENTS.md REVIEW.md
  run_servo_pinocchio_gate
}

run_cart_accept_gate() {
  run_shell_syntax_checks
  if [[ -f scripts/cartesian_acceptance.py ]]; then
    python3 -m compileall -q scripts
  fi
  if [[ -f rb_servo_server/tools/send_tcp_linear_move.py ]]; then
    python3 rb_servo_server/tools/send_tcp_linear_move.py --help >/dev/null
  fi
  if [[ "${CODEX_RUN_CARTESIAN_ACCEPTANCE:-0}" == "1" ]]; then
    run_servo_pinocchio_gate
  else
    echo "codex_gate: skipping full Cartesian Pinocchio gate; set CODEX_RUN_CARTESIAN_ACCEPTANCE=1 to enable"
  fi
}

run_cart_servo_01_gate() {
  run_shell_syntax_checks
  run_servo_gate_or_skip_missing_deps
  run_gui_tests
  run_policy_runner_tests
}

run_cart_servo_02_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  run_gui_tests
  run_policy_runner_tests
}

run_cart_servo_03_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  run_servo_gate_or_skip_missing_deps
  if [[ "${CODEX_RUN_CARTESIAN_ACCEPTANCE:-0}" == "1" ]]; then
    run_cart_harden_05_gate
  else
    echo "codex_gate: skipping full Cartesian simulator acceptance; set CODEX_RUN_CARTESIAN_ACCEPTANCE=1 to enable"
  fi
}

run_cart_tune_02_gate() {
  run_shell_syntax_checks
  run_servo_gate_or_skip_missing_deps
  run_python_surface_tests
}

run_supported_scope_name_scan() {
  local removed_backend
  removed_backend="rb""script"
  local removed_env
  removed_env="RB_ALLOW_RB""SCRIPT"
  local removed_ablation
  removed_ablation="rb_backend_""ablation"
  local removed_rate_probe
  removed_rate_probe="rainbow_rate_""probe"
  local removed_compare_dir
  removed_compare_dir="backend_""compare"
  local disallowed=(
    "${removed_backend}"
    "Rb""script"
    "${removed_env}"
    "${removed_ablation}"
    "${removed_rate_probe}"
    "${removed_compare_dir}"
  )
  local pattern
  pattern="$(IFS='|'; echo "${disallowed[*]}")"

  local matches
  set +e
  matches="$(
    find \
      AGENTS.md README.md README.en.md REVIEW.md docs rb_servo_server policy_runner scripts configs \
      -path './.git' -prune -o \
      -path './.codex' -prune -o \
      -path './artifacts' -prune -o \
      -path './rb_servo_server/build' -prune -o \
      -path 'rb_servo_server/build' -prune -o \
      -path 'rb_servo_server/build/*' -prune -o \
      -path './scripts/codex_gate.sh' -prune -o \
      -path './scripts/codex_run_sequence.sh' -prune -o \
      -path '*/__pycache__' -prune -o \
      -type f -print 2>/dev/null | \
      xargs -r grep -IEn "${pattern}"
  )"
  local rc=$?
  set -e
  if [[ "${rc}" == "0" ]]; then
    echo "ERROR: removed backend or comparison-surface references remain:" >&2
    echo "${matches}" >&2
    return 1
  fi
}

run_supported_scope_file_scan() {
  local removed_backend_glob="*rb""script*"
  local removed_ablation_glob="*rb_backend_""ablation*"
  local removed_compare_glob="*backend_""compare*"
  local removed_rate_probe_glob="*rainbow_rate_""probe*"
  local matches
  set +e
  matches="$(
    find . \
      -path './.git' -prune -o \
      -path './.codex' -prune -o \
      -path './artifacts' -prune -o \
      -path './rb_servo_server/build' -prune -o \
      -path 'rb_servo_server/build' -prune -o \
      -path 'rb_servo_server/build/*' -prune -o \
      -path '*/__pycache__' -prune -o \
      \( -iname "${removed_backend_glob}" -o -iname "${removed_ablation_glob}" -o -iname "${removed_compare_glob}" -o -iname "${removed_rate_probe_glob}" \) \
      -print
  )"
  local rc=$?
  set -e
  if [[ "${rc}" == "0" && -n "${matches}" ]]; then
    echo "ERROR: removed backend/comparison files remain:" >&2
    echo "${matches}" >&2
    return 1
  fi
}

run_supported_scope_500hz_scan() {
  python3 - <<'PY'
from pathlib import Path
import re
import sys

roots = [
    Path("rb_servo_server/config"),
    Path("policy_runner/config"),
    Path("configs"),
]
paths: list[Path] = []
for root in roots:
    if root.exists():
        paths.extend(p for p in root.rglob("*.yaml") if "local" not in p.parts)

bad: list[str] = []
checks = [
    re.compile(r"^\s*command_rate_hz:\s*(?:100|200)\b"),
    re.compile(r"^\s*(?:servo|servo\.rbpodo_async_streaming)\.rate_hz:\s*(?:100|200)\b"),
    re.compile(r"^\s*servo\.worker_read_rate_hz:\s*(?:100|200)\b"),
    re.compile(r"^\s*servo\.worker_read_period_sec:\s*(?:0\.01|0\.005)\b"),
    re.compile(r"^\s*(?:left_robot|right_robot)\.servo_t1_sec:\s*(?:0\.01|0\.005)\b"),
]
real_template_checks = [
    re.compile(r"^(?:rate_hz:\s*(?:100|200)|servo_t1_sec:\s*(?:0\.01|0\.005)|servo_time_sec:\s*(?:0\.01|0\.005))\b"),
    re.compile(r"^worker_read_rate_hz:\s*(?:100|200)\b"),
    re.compile(r"^worker_read_period_sec:\s*(?:0\.01|0\.005)\b"),
]

for path in sorted(paths):
    text = path.read_text(errors="ignore").splitlines()
    is_real_template = path.match("rb_servo_server/config/dual_real*.yaml")
    for lineno, line in enumerate(text, start=1):
        stripped = line.strip()
        if is_real_template and any(check.search(stripped) for check in real_template_checks):
            bad.append(f"{path}:{lineno}:{line}")
        if any(check.search(line) for check in checks):
            bad.append(f"{path}:{lineno}:{line}")

if bad:
    print("ERROR: unsupported non-500 robot-control defaults remain:", file=sys.stderr)
    print("\n".join(bad), file=sys.stderr)
    sys.exit(1)
PY
}

run_supported_scope_common_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts policy_runner/policy_runner
  run_supported_scope_name_scan
  run_supported_scope_file_scan
}

run_supported_scope_backend_gate() {
  run_supported_scope_common_gate
  run_servo_gate_or_skip_missing_deps
}

run_supported_scope_500hz_gate() {
  run_supported_scope_common_gate
  run_supported_scope_500hz_scan
  run_required_policy_runner_tests 'test_cartesian_action_source.py'
}

run_supported_scope_docs_gate() {
  run_supported_scope_common_gate
  run_supported_scope_500hz_scan
  grep_existing "supported scope|500 Hz|rbpodo" README.md README.en.md docs AGENTS.md REVIEW.md
  grep_existing "unsupported raw script TCP|raw script TCP.*removed|removed.*raw script TCP" \
    README.md README.en.md docs AGENTS.md REVIEW.md
}

run_rbpodo_accept_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  run_optional_python_help scripts/rbpodo_servo_acceptance.py
  if [[ -f scripts/rbpodo_servo_acceptance.py ]]; then
    python3 scripts/rbpodo_servo_acceptance.py --self-test
  fi
  run_optional_python_help scripts/rbpodo_ack_rate_probe.py
  if [[ "${CODEX_RUN_RBPODO_ACCEPTANCE:-0}" == "1" ]]; then
    if [[ -z "${CODEX_RBPODO_ACCEPTANCE_ARGS:-}" ]]; then
      echo "ERROR: CODEX_RUN_RBPODO_ACCEPTANCE=1 requires CODEX_RBPODO_ACCEPTANCE_ARGS with explicit script arguments and safety preflight flags" >&2
      return 1
    fi
    if [[ -f scripts/rbpodo_servo_acceptance.py ]]; then
      # shellcheck disable=SC2086
      python3 scripts/rbpodo_servo_acceptance.py ${CODEX_RBPODO_ACCEPTANCE_ARGS}
    elif [[ -f scripts/rbpodo_ack_rate_probe.py ]]; then
      # shellcheck disable=SC2086
      python3 scripts/rbpodo_ack_rate_probe.py ${CODEX_RBPODO_ACCEPTANCE_ARGS}
    else
      echo "ERROR: CODEX_RUN_RBPODO_ACCEPTANCE=1 but no rbpodo acceptance script is present" >&2
      return 1
    fi
  else
    echo "codex_gate: skipping rbpodo real/controller acceptance; set CODEX_RUN_RBPODO_ACCEPTANCE=1 with explicit args to enable"
  fi
}

run_rbpodo_readonly_diag_gate() {
  run_shell_syntax_checks
  run_servo_gate_or_skip_missing_deps
  run_python_surface_tests
}

run_rbpodo_status_fields_gate() {
  run_servo_gate_or_skip_missing_deps
}

run_rbpodo_joint_wrap_gate() {
  python3 scripts/check_joint_range_policy.py
  run_servo_gate_or_skip_missing_deps
}

run_rbpodo_joint_range_policy_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  python3 scripts/check_joint_range_policy.py --self-test
  python3 scripts/check_joint_range_policy.py
  run_servo_gate_or_skip_missing_deps
}

run_rbpodo_bringup_tools_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  run_optional_python_help scripts/rbpodo_state_dump.py
  run_optional_python_help scripts/rbpodo_servo_acceptance.py
}

run_rbpodo_doc_gate() {
  run_shell_syntax_checks
  grep_existing "servo_t1_sec" README.md REVIEW.md docs rb_servo_server/docs rb_servo_server/config
  grep_existing "servo_t2_sec" README.md REVIEW.md docs rb_servo_server/docs rb_servo_server/config
  grep_existing "servo_alpha" README.md REVIEW.md docs rb_servo_server/docs rb_servo_server/config
  grep_existing "disable_waiting_ack" README.md REVIEW.md docs rb_servo_server/docs rb_servo_server/config
  grep_existing "ACK disabled" README.md REVIEW.md docs rb_servo_server/docs rb_servo_server/config
  grep_existing "500[[:space:]]*Hz|500Hz" README.md REVIEW.md docs rb_servo_server/docs rb_servo_server/config
  grep_existing "supported scope|rbpodo-only|rbpodo only" README.md REVIEW.md docs rb_servo_server/docs rb_servo_server/config
  grep_existing "RB_ALLOW_REAL_MOTION" README.md REVIEW.md AGENTS.md docs rb_servo_server/docs rb_servo_server/config
}

run_rbpodo_pgmode_sim_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts tools
  if [[ -f tools/simulation_mode.sh ]]; then
    bash -n tools/simulation_mode.sh
  else
    echo "codex_gate: optional pgmode helper not present: tools/simulation_mode.sh"
  fi
  run_optional_python_help scripts/rainbow_pgmode.py
  if [[ "${CODEX_RUN_RBPODO_PGMODE_CHECK:-0}" == "1" ]]; then
    if [[ ! -f scripts/rainbow_pgmode.py ]]; then
      echo "ERROR: CODEX_RUN_RBPODO_PGMODE_CHECK=1 but scripts/rainbow_pgmode.py is missing" >&2
      return 1
    fi
    if [[ -z "${CODEX_RBPODO_PGMODE_CHECK_ARGS:-}" ]]; then
      echo "ERROR: CODEX_RUN_RBPODO_PGMODE_CHECK=1 requires CODEX_RBPODO_PGMODE_CHECK_ARGS with explicit controller IP, pgmode, and confirmation flags" >&2
      return 1
    fi
    # shellcheck disable=SC2086
    python3 scripts/rainbow_pgmode.py ${CODEX_RBPODO_PGMODE_CHECK_ARGS}
  else
    echo "codex_gate: skipping rbpodo pgmode simulation controller check; set CODEX_RUN_RBPODO_PGMODE_CHECK=1 with explicit args to enable"
  fi
}

run_rbpodo_reference_tcp_gate() {
  run_servo_gate_or_skip_missing_deps
  run_python_surface_tests
}

run_rbpodo_controller_sim_gate() {
  run_shell_syntax_checks
  run_servo_gate_or_skip_missing_deps
  run_optional_python_help scripts/rbpodo_servo_acceptance.py
  if [[ "${CODEX_RUN_RBPODO_CONTROLLER_SIM:-0}" == "1" ]]; then
    if [[ -z "${CODEX_RBPODO_CONTROLLER_SIM_ARGS:-}" ]]; then
      echo "ERROR: CODEX_RUN_RBPODO_CONTROLLER_SIM=1 requires CODEX_RBPODO_CONTROLLER_SIM_ARGS with explicit script arguments and safety preflight flags" >&2
      return 1
    fi
    if [[ -f scripts/rbpodo_servo_acceptance.py ]]; then
      # shellcheck disable=SC2086
      python3 scripts/rbpodo_servo_acceptance.py ${CODEX_RBPODO_CONTROLLER_SIM_ARGS}
    else
      echo "ERROR: CODEX_RUN_RBPODO_CONTROLLER_SIM=1 but no rbpodo controller-simulation script is present" >&2
      return 1
    fi
  else
    echo "codex_gate: skipping rbpodo controller-simulation run; set CODEX_RUN_RBPODO_CONTROLLER_SIM=1 with explicit args to enable"
  fi
}

run_tools_shell_syntax_checks() {
  local tool
  local found=0
  for tool in tools/*.sh; do
    if [[ ! -e "${tool}" ]]; then
      continue
    fi
    bash -n "${tool}"
    found=1
  done
  if [[ "${found}" != "1" ]]; then
    echo "codex_gate: skipping tools shell syntax checks; no tools/*.sh files found"
  fi
}

run_rbpodo_measure_state_parity_gate() {
  run_shell_syntax_checks
  run_servo_gate_or_skip_missing_deps
  python3 -m compileall -q scripts
  python3 scripts/rbpodo_state_dump.py --help >/dev/null
  python3 scripts/rbpodo_state_dump.py --self-test
  run_optional_script_tests 'test_rbpodo_state_dump.py'
  run_optional_script_tests 'test_rbpodo_measure_state_parity.py'
  run_optional_rbpodo_measurement_readonly scripts/rbpodo_state_dump.py
}

run_rbpodo_measure_raw_data_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  run_optional_python_help scripts/rbpodo_raw_data_capture.py
  run_optional_script_tests 'test_rbpodo_raw_data*.py'
  run_optional_script_tests 'test_rainbow_raw_data*.py'
  run_optional_rbpodo_measurement_readonly \
    scripts/rbpodo_state_dump.py \
    "--artifact-dir"
}

run_rbpodo_measure_timestamp_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  run_optional_python_help scripts/rbpodo_timestamp_audit.py
  run_optional_python_help scripts/rbpodo_measure_timestamp_audit.py
  run_optional_script_tests 'test_rbpodo_timestamp*.py'
  run_optional_script_tests 'test_rbpodo_measure_timestamp*.py'
  echo "codex_gate: skipping rbpodo measurement controller run by default"
}

run_rbpodo_measure_reliability_report_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  run_optional_python_help scripts/rbpodo_measure_reliability_report.py
  run_optional_python_help scripts/generate_rbpodo_measurement_report.py
  run_optional_script_tests 'test_rbpodo_measure_reliability*.py'
  for token in \
    "diagnostics_suspect" \
    "tcp_ref_stand" \
    "lower bound" \
    "timestamp alignment" \
    "q_ref" \
    "raw 5001"
  do
    grep_existing "${token}" README.md REVIEW.md docs rb_servo_server/docs scripts
  done
  echo "codex_gate: skipping rbpodo measurement report generation by default"
}

run_rbpodo_p0_measurement_common_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
}

run_p0_parity_repair_gate() {
  run_rbpodo_p0_measurement_common_gate
  python3 scripts/rbpodo_state_parity_check.py --help >/dev/null
  python3 scripts/rbpodo_state_dump.py --help >/dev/null
  run_optional_script_tests 'test_rbpodo_state_parity_check.py'
  run_optional_script_tests 'test_rbpodo_state_dump.py'
  run_optional_rbpodo_measurement_readonly \
    scripts/rbpodo_state_parity_check.py \
    "--ips" \
    "--state-endpoint" \
    "--artifact-dir"
}

run_p0_diagnostics_rootcause_gate() {
  run_rbpodo_p0_measurement_common_gate
  python3 scripts/rbpodo_state_dump.py --help >/dev/null
  python3 scripts/rbpodo_state_parity_check.py --help >/dev/null
  python3 scripts/rainbow_data_port_capture.py --help >/dev/null
  run_optional_script_tests 'test_rbpodo_state_dump.py'
  run_optional_script_tests 'test_rbpodo_state_parity_check.py'
  run_optional_script_tests 'test_rainbow_data_port_capture.py'
  run_optional_rbpodo_measurement_readonly \
    scripts/rainbow_data_port_capture.py \
    "--ips" \
    "--artifact-dir" \
    "--also-rbpodo-python"
}

run_p0_raw_payload_fixture_gate() {
  run_rbpodo_p0_measurement_common_gate
  python3 scripts/rainbow_data_port_capture.py --help >/dev/null
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_rainbow_data_port_capture.py'
  run_optional_rbpodo_measurement_readonly \
    scripts/rainbow_data_port_capture.py \
    "--ips" \
    "--artifact-dir"
}

run_p0_measurement_gating_gate() {
  run_rbpodo_p0_measurement_common_gate
  python3 scripts/timestamp_alignment_audit.py --help >/dev/null
  python3 scripts/generate_rbpodo_measurement_reliability_report.py --help >/dev/null
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_timestamp_alignment_audit.py'
  for token in \
    "diagnostics_suspect" \
    "tcp_ref_stand" \
    "lower bound" \
    "timestamp alignment" \
    "q_ref" \
    "raw 5001"
  do
    grep_existing "${token}" README.md REVIEW.md docs rb_servo_server/docs scripts
  done
  echo "codex_gate: skipping rbpodo measurement controller run by default"
}

run_rbpodo_async_common_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  grep_existing "500 Hz|ACK|pgmode simulation" REVIEW.md
}

run_rbpodo_async_contract_probe_gate() {
  run_rbpodo_async_common_gate
  echo "codex_gate: skipping rbpodo async ACK-supervised controller probe by default"
}

run_rbpodo_async_cpp_gate() {
  run_rbpodo_async_common_gate
  run_servo_gate_or_skip_missing_deps
  echo "codex_gate: skipping rbpodo async ACK-supervised controller run by default"
}

run_rbpodo_async_report_gate() {
  run_rbpodo_async_common_gate
  run_optional_python_help scripts/generate_rbpodo_measurement_reliability_report.py
  echo "codex_gate: skipping rbpodo async ACK-supervised report generation by default"
}

run_rbpodo_async_runbook_gate() {
  run_rbpodo_async_common_gate
  grep_existing "async|ACK-supervised|500 Hz|pgmode simulation" REVIEW.md
  echo "codex_gate: skipping rbpodo async ACK-supervised controller run by default"
}

run_ackon500_followup_common_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  run_optional_python_help scripts/generate_rbpodo_measurement_reliability_report.py
  run_optional_script_tests 'test_*.py'
  run_gui_tests
  run_policy_runner_tests
}

run_physical_readiness_blockers_clarity_gate() {
  run_ackon500_followup_common_gate
  grep_existing "physical_readiness|physical_tracking_result|controller-reference lower-bound|not physical" \
    scripts docs/runbooks REVIEW.md README.md
}

run_gene_umi_physical_transition_gate() {
  run_source_hygiene_local_configs_gate
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  run_required_python_help scripts/rbpodo_physical_transition_acceptance.py
  run_required_python_help scripts/generate_rbpodo_physical_transition_report.py
  run_required_script_tests 'test_*physical*transition*.py'
  run_yaml_parse_checks_if_available rb_servo_server/config/dual_real_rbpodo_physical_*.example.yaml
  grep_existing "pgmode_real_transition|tiny_cartesian|slow_physical_circle|physical_readiness" \
    scripts docs/runbooks rb_servo_server/config
  grep_existing "send_servo_commands:[[:space:]]*false" rb_servo_server/config/dual_real_rbpodo_physical_*.example.yaml
  grep_existing "allow_in_real:[[:space:]]*false" rb_servo_server/config/dual_real_rbpodo_physical_*.example.yaml
  grep_existing "tracking_source:[[:space:]]*tcp_actual_stand|tcp_actual_stand" scripts docs/runbooks
}

run_gene_umi_hdf5_audit_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q policy_runner/policy_runner scripts
  run_policy_runner_help hdf5-audit
  run_required_policy_runner_tests 'test_flow_dataset.py'
  run_required_policy_runner_tests 'test_hdf5_audit*.py'
  grep_existing "robotics_lab\\.policy_runner\\.hdf5_audit\\.v1|DatasetManifest|dataset_manifest" \
    policy_runner/policy_runner policy_runner/tests docs README.md
  local smoke_hdf5="${CODEX_UPLOADED_HDF5_SMOKE:-}"
  if [[ -n "${smoke_hdf5}" ]]; then
    PYTHONPATH=policy_runner python3 -m policy_runner hdf5-audit \
      --episodes-dir "${smoke_hdf5}" \
      --output-json /tmp/robotics_lab_hdf5_audit_gate.json \
      --output-md /tmp/robotics_lab_hdf5_audit_gate.md
  elif [[ -f episode_002.hdf5 ]]; then
    PYTHONPATH=policy_runner python3 -m policy_runner hdf5-audit \
      --episodes-dir episode_002.hdf5 \
      --output-json /tmp/robotics_lab_hdf5_audit_gate.json \
      --output-md /tmp/robotics_lab_hdf5_audit_gate.md
  else
    echo "codex_gate: skipping uploaded episode audit smoke; episode_002.hdf5 is absent"
  fi
}

run_gene_umi_bimanual_collection_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q policy_runner/policy_runner scripts
  run_policy_runner_help umi-import
  run_policy_runner_help umi-convert
  require_file calibration/umi_retarget.example.yaml
  run_yaml_parse_checks_if_available calibration/umi_retarget.example.yaml
  run_required_policy_runner_tests 'test_umi_*.py'
  grep_existing "robotics_lab\\.umi_retarget\\.v1|retarget_status|configured_estimate|measured" \
    calibration policy_runner docs README.md
}

run_gene_umi_flow_training_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q policy_runner/policy_runner scripts
  run_policy_runner_help flow-train
  run_policy_runner_help ml-preflight
  run_required_policy_runner_tests 'test_flow_matching.py'
  run_required_policy_runner_tests_with_timeout 120 'test_flow_training*.py'
  run_tiny_cnn_ml_preflight_with_timeout
  grep_existing "tiny_cnn|dataset-manifest|write-eval-report|flow_eval_summary" \
    policy_runner/policy_runner policy_runner/tests docs README.md
}

run_gene_umi_policy_rollout_modes_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q policy_runner/policy_runner scripts
  run_policy_runner_help flow-infer
  run_required_policy_runner_tests 'test_flow_inference*.py'
  run_required_policy_runner_tests 'test_policy_rollout_modes*.py'
  grep_existing "rollout-mode|controller_simulation|real_supervised|rollout_summary" \
    policy_runner/policy_runner policy_runner/tests docs README.md
}

run_gene_umi_dual_arm_gripper_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q policy_runner/policy_runner scripts
  run_policy_runner_help flow-infer
  run_required_policy_runner_tests 'test_gripper*.py'
  run_required_policy_runner_tests 'test_dual_arm_policy*.py'
  grep_existing "RB_ALLOW_REAL_GRIPPER|allow_real_gripper_motion|collision_model_status|selected[-_]arm" \
    policy_runner/policy_runner policy_runner/tests docs README.md
}

run_gene_umi_docs_ci_artifact_manifest_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q policy_runner/policy_runner scripts
  run_policy_runner_tests
  run_required_script_tests 'test_*.py'
  run_required_python_help scripts/collect_gene_umi_artifact_manifest.py
  require_file docs/runbooks/gene_umi_policy_transition.md
  grep_existing "GENE 26.5|ACKON500|hdf5-audit|flow-infer|real[-_]supervised|[Aa]rtifact manifest|artifact_manifest" \
    docs/runbooks/gene_umi_policy_transition.md README.md REVIEW.md policy_runner/README.md docs/runbooks/policy_data_collection.md
  run_required_make_dry_run policy-hdf5-audit-smoke
  run_required_make_dry_run policy-flow-smoke
  run_required_make_dry_run pgmode-transition-dry-run
}

run_required_policy_runner_tests_any() {
  local description="$1"
  shift

  local ran_any=0
  local pattern
  for pattern in "$@"; do
    if find policy_runner/tests -maxdepth 1 -name "${pattern}" -print -quit | grep -q .; then
      PYTHONPATH=policy_runner python3 -m unittest discover -s policy_runner/tests -p "${pattern}"
      ran_any=1
    fi
  done

  if [[ "${ran_any}" != "1" ]]; then
    echo "ERROR: required policy_runner tests not present for ${description}: $*" >&2
    return 1
  fi
}

require_active_gitignore_entry() {
  local description="$1"
  local pattern="$2"
  if ! grep -Eq "^[[:space:]]*${pattern}" .gitignore; then
    echo "ERROR: .gitignore missing active coverage for ${description}" >&2
    return 1
  fi
}

fail_if_tracked_matches() {
  local description="$1"
  local pattern="$2"
  local matches
  matches="$(
    git ls-files | grep -E "${pattern}" | while IFS= read -r path; do
      [[ -e "${path}" ]] && printf '%s\n' "${path}"
    done || true
  )"
  if [[ -n "${matches}" ]]; then
    echo "ERROR: tracked files include forbidden ${description}:" >&2
    printf '%s\n' "${matches}" >&2
    return 1
  fi
}

fail_if_tracked_real_config_without_example_suffix() {
  local matches
  matches="$(git ls-files 'rb_servo_server/config/dual_real*.yaml' | grep -Ev '\.example\.yaml$' || true)"
  if [[ -n "${matches}" ]]; then
    echo "ERROR: tracked real robot configs must be .example.yaml templates:" >&2
    printf '%s\n' "${matches}" >&2
    return 1
  fi
}

check_spacemouse_pgmode_config_safety() {
  local policy_config="policy_runner/config/rbpodo_pgmode_spacemouse_500hz_ack.yaml"
  local server_template="rb_servo_server/config/dual_real_rbpodo_pgmode_spacemouse_500hz_ack.example.yaml"

  require_file "${policy_config}"
  require_file "${server_template}"

  grep_absent "allow_real_motion:[[:space:]]*true" "${policy_config}"
  grep_existing "allow_rbpodo_controller_simulation_cartesian:[[:space:]]*true|controller_simulation_cartesian" \
    "${policy_config}" policy_runner/policy_runner policy_runner/tests policy_runner/README.md
  grep_existing "allow_in_controller_simulation:[[:space:]]*true|controller_simulation_cartesian" \
    "${server_template}" docs/runbooks/rbpodo_pgmode_spacemouse.md docs/servo_backend_contract.md docs/architecture.md
  grep_existing "allow_in_real:[[:space:]]*false" "${server_template}" docs/runbooks/rbpodo_pgmode_spacemouse.md
  grep_existing "physical_motion_expected:[[:space:]]*false|physical_motion_expected=false" \
    "${policy_config}" "${server_template}" docs/runbooks/rbpodo_pgmode_spacemouse.md policy_runner/README.md
  grep_existing "RB_ALLOW_REAL_MOTION|RB_ALLOW_REAL_ROBOT|RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION" \
    README.md docs/architecture.md docs/servo_backend_contract.md tools/rbpodo_pgmode_spacemouse.sh
  grep_existing "RB_ALLOW_REAL_CARTESIAN.*must not|Do not set.*RB_ALLOW_REAL_CARTESIAN|do not set.*RB_ALLOW_REAL_CARTESIAN" \
    tools/rbpodo_pgmode_spacemouse.sh docs/runbooks/rbpodo_pgmode_spacemouse.md policy_runner/README.md README.md
}

run_pgmode_spacemouse_python_checks() {
  run_shell_syntax_checks
  run_python_compile_checks
  run_policy_runner_tests
}

run_spacemouse_fix_controller_sim_safety_gate() {
  run_pgmode_spacemouse_python_checks
  check_spacemouse_pgmode_config_safety
}

run_pgmode_spacemouse_end_to_end_dryrun_gate() {
  run_shell_syntax_checks
  run_python_compile_checks
  run_policy_runner_tests
  run_gui_tests
  run_servo_gate_or_skip_missing_deps
  bash -n tools/rbpodo_pgmode_spacemouse.sh
  grep_existing "dual_spacemouse_pose_target" \
    policy_runner/policy_runner policy_runner/tests policy_runner/config policy_runner/README.md
  grep_existing "deadman|deadman_button" \
    policy_runner/policy_runner/action_sources policy_runner/tests policy_runner/config policy_runner/README.md
  grep_existing "release[-_ ]?zero|zero.*release|deadman.*released|released.*no motion|emits no motion" \
    policy_runner/policy_runner/action_sources policy_runner/tests policy_runner/README.md docs/runbooks/policy_data_collection.md
  grep_existing "sample_stale_timeout_sec|state_stream_stale|stale.*timeout|timeout.*stale" \
    policy_runner/policy_runner policy_runner/tests policy_runner/config
  grep_existing "TcpPoseTarget|tcp_target_stand" \
    policy_runner/policy_runner policy_runner/tests policy_runner/README.md
  grep_existing "controller_simulation_cartesian_enabled|cartesian_gate|controller_simulation.*readback|lease_readback|50376" \
    policy_runner/policy_runner policy_runner/tests policy_runner/config docs/runbooks/rbpodo_pgmode_spacemouse.md
  grep_existing "device:[[:space:]]*null|path:[[:space:]]*null|Fake.*SpaceMouse|fake.*spacemouse|FakeSpaceMouse" \
    policy_runner/config policy_runner/tests
}

run_spacemouse_flow_infer_rollout_modes_gate() {
  run_gene_umi_policy_rollout_modes_gate
  run_policy_runner_help flow-infer
  run_required_policy_runner_tests 'test_flow_inference*.py'
  run_required_policy_runner_tests 'test_policy_rollout_modes*.py'
  grep_existing "rollout-mode|controller_simulation|rollout_summary" \
    policy_runner/policy_runner policy_runner/tests docs README.md
  grep_existing "real_readonly|real_supervised" \
    policy_runner/policy_runner policy_runner/tests docs README.md
}

run_flow_policy_tcp_pose_target_controller_sim_gate() {
  run_shell_syntax_checks
  run_python_compile_checks
  run_policy_runner_tests
  run_policy_runner_help flow-infer
  run_required_policy_runner_tests_any \
    "flow policy TcpPoseTarget controller-simulation rollout" \
    'test_flow_inference*.py' \
    'test_flow_inference_tcp_targetpose*.py'
  grep_existing "TcpPoseTarget|tcp_target_stand" \
    policy_runner/policy_runner policy_runner/tests docs/runbooks/policy_data_collection.md
  grep_existing "clamp|clip|max_linear_step_m|max_angular_step_rad|max_linear_velocity_m_s|max_angular_velocity_rad_s" \
    policy_runner/policy_runner policy_runner/tests docs README.md
  grep_existing "policy_dt_sec|policy[-_ ]dt|command_rate_hz|dt_sec" \
    policy_runner/policy_runner policy_runner/tests policy_runner/config docs README.md
  grep_existing "controller_simulation|allow_rbpodo_controller_simulation_cartesian|controller_simulation_cartesian" \
    policy_runner/policy_runner policy_runner/tests policy_runner/config docs README.md
  grep_existing "real_readonly|real_supervised|physical real|physical_real|allow_real_motion:[[:space:]]*false" \
    policy_runner/policy_runner policy_runner/tests policy_runner/config docs README.md
}

run_viser_pgmode_operator_view_gate() {
  run_shell_syntax_checks
  run_python_compile_checks
  run_gui_tests
  grep_existing "50366" \
    rb_gui rb_gui/tests docs/runbooks/rbpodo_pgmode_spacemouse.md tools/rbpodo_pgmode_spacemouse.sh \
    rb_servo_server/config/dual_real_rbpodo_pgmode_spacemouse_500hz_ack.example.yaml
  grep_existing "50376" \
    rb_gui rb_gui/tests docs/runbooks/rbpodo_pgmode_spacemouse.md tools/rbpodo_pgmode_spacemouse.sh \
    rb_servo_server/config/dual_real_rbpodo_pgmode_spacemouse_500hz_ack.example.yaml policy_runner/config
  grep_existing "physical_motion_expected" rb_gui rb_gui/tests docs/runbooks/rbpodo_pgmode_spacemouse.md
  grep_existing "tcp_ref_stand|selected_tcp_source|selected_source|selected TCP|selected tcp" \
    rb_gui rb_gui/tests docs/runbooks/rbpodo_pgmode_spacemouse.md
  grep_existing "lease|source_id|command_source|policy_runner safety readback" \
    rb_gui rb_gui/tests docs/runbooks/rbpodo_pgmode_spacemouse.md tools/rbpodo_pgmode_spacemouse.sh
  grep_existing "controller_simulation_mode|controller simulation|controller_simulation" \
    rb_gui rb_gui/tests docs/runbooks/rbpodo_pgmode_spacemouse.md
  echo "codex_gate: not launching live viewer; 05_viser_pgmode_operator_view is documentation/test evidence only"
}

run_spacemouse_gripper_dual_arm_policy_gate() {
  run_gene_umi_dual_arm_gripper_gate
  grep_existing "RB_ALLOW_REAL_GRIPPER|allow_real_gripper_motion" \
    policy_runner/policy_runner policy_runner/tests docs README.md
  grep_existing "selected[-_]arm|collision_model_status" \
    policy_runner/policy_runner policy_runner/tests docs README.md
  grep_existing "physical.*gripper.*block|gripper.*physical.*block|allow_real_gripper_motion:[[:space:]]*false|blocked by default" \
    policy_runner/policy_runner policy_runner/tests docs README.md
}

run_ml_preflight_gate_stability_gate() {
  run_gene_umi_flow_training_gate
  run_policy_runner_help ml-preflight
  run_policy_runner_help flow-train
  run_tiny_cnn_ml_preflight_with_timeout
  grep_existing "tiny_cnn" policy_runner/policy_runner policy_runner/tests docs README.md
  grep_existing "seed|deterministic" \
    policy_runner/policy_runner policy_runner/tests docs README.md
  grep_existing "no network|network downloads|no internet|offline|download" \
    policy_runner/policy_runner policy_runner/tests docs README.md
}

run_umi_hdf5_manifest_robustness_gate() {
  run_gene_umi_hdf5_audit_gate
  run_gene_umi_bimanual_collection_gate
  run_policy_runner_help hdf5-audit
  run_policy_runner_help umi-import
  run_policy_runner_help umi-convert
  run_required_policy_runner_tests 'test_hdf5_audit*.py'
  run_required_policy_runner_tests 'test_flow_dataset.py'
  run_required_policy_runner_tests 'test_umi_*.py'
  grep_existing "dataset_manifest|DatasetManifest" policy_runner/policy_runner policy_runner/tests docs README.md
  grep_existing "schema|schema_version|version" policy_runner/policy_runner policy_runner/tests docs README.md
  if [[ -n "${CODEX_UPLOADED_HDF5_SMOKE:-}" ]]; then
    echo "codex_gate: CODEX_UPLOADED_HDF5_SMOKE is set; hdf5 audit smoke runs through 04_umi_hdf5_audit_adapter"
  elif [[ -f episode_002.hdf5 ]]; then
    echo "codex_gate: episode_002.hdf5 is present; hdf5 audit smoke remains optional and local"
  else
    echo "codex_gate: episode_002.hdf5 absent as expected; gate does not require committed dataset files"
  fi
}

run_artifact_manifest_docs_makefile_gate() {
  run_gene_umi_docs_ci_artifact_manifest_gate
}

run_source_hygiene_local_configs_gate() {
  run_shell_syntax_checks
  run_python_compile_checks
  bash -n tools/create_rbpodo_pgmode_spacemouse_local_config.sh
  require_active_gitignore_entry "HDF5 datasets (*.hdf5)" '\*\.hdf5([[:space:]]|$)'
  require_active_gitignore_entry "HDF5 datasets (*.h5)" '\*\.h5([[:space:]]|$)'
  require_active_gitignore_entry "artifact directories" '(\*\*/)?artifacts/'
  require_active_gitignore_entry "rb_servo_server local YAML configs" 'rb_servo_server/config/local/\*\.ya?ml'
  require_active_gitignore_entry "rb_servo_server local YML configs" 'rb_servo_server/config/local/\*\.yml'
  require_active_gitignore_entry "runtime output directories" '(\*\*/)?(logs|episodes|checkpoints)/'
  fail_if_tracked_matches "large local dataset files" '\.(hdf5|h5)$'
  fail_if_tracked_matches "Python cache files" '(^|/)__pycache__(/|$)|\.pyc$'
  fail_if_tracked_matches "Codex run artifacts" '^artifacts/codex_runs/'
  fail_if_tracked_matches "local rb_servo_server YAML configs" '^rb_servo_server/config/local/.+\.(yaml|yml)$'
  fail_if_tracked_real_config_without_example_suffix
  grep_existing "create_rbpodo_pgmode_spacemouse_local_config|RB_PGMODE_SPACEMOUSE_LEFT_IP|ignored by git" \
    tools/create_rbpodo_pgmode_spacemouse_local_config.sh docs/runbooks/rbpodo_pgmode_spacemouse.md tools/rbpodo_pgmode_spacemouse.sh
  grep_absent "172\\.28\\.60\\.(200|201)" docs/runbooks/rbpodo_pgmode_spacemouse.md
  grep_existing "\\.example\\.yaml|site-specific|placeholder|config/local|Copy to config/local" \
    README.md docs rb_servo_server/config/dual_real.example.yaml rb_servo_server/config/dual_real_rbpodo_pgmode_spacemouse_500hz_ack.example.yaml
}

run_vm_parity_guardrails_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
  local tool
  for tool in tools/vm/*.sh; do
    if [[ -e "${tool}" ]]; then
      bash -n "${tool}"
      bash "${tool}" --help >/dev/null
    fi
  done
  python3 scripts/check_vm_artifact_tagging.py --help >/dev/null
  PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_check_vm_artifact_tagging.py'
  python3 scripts/check_vm_artifact_tagging.py \
    --root . \
    --physical-root artifacts/circle_tracking \
    --physical-root artifacts/rbpodo_physical_transition \
    --physical-root artifacts/physical_acceptance
  grep_existing "controller_simulation_vm|physical_motion=false|5000/5001|ROBOT_LEFT_IP" \
    docs/runbooks/vm_network_bringup.md docs/runbooks/vm_real_parity.md scripts/check_vm_artifact_tagging.py tools/vm
}

run_rbpodo_p1_common_gate() {
  run_shell_syntax_checks
  python3 -m compileall -q scripts
}

run_p1_servo_param_sweep_gate() {
  run_rbpodo_p1_common_gate
  grep_existing "servo_t1_sec|servo_t2_sec|servo_gain|servo_alpha|speed_bar" \
    scripts docs REVIEW.md rb_servo_server/config
}

run_yaml_parse_checks_if_available() {
  local paths=()
  local path
  for path in "$@"; do
    if [[ -e "${path}" ]]; then
      paths+=("${path}")
    fi
  done

  if [[ "${#paths[@]}" -eq 0 ]]; then
    echo "codex_gate: skipping YAML parse checks; no matching YAML files found"
    return 0
  fi

  python3 - "${paths[@]}" <<'PY'
import sys
from pathlib import Path

try:
    import yaml
except ModuleNotFoundError:
    print("codex_gate: skipping YAML parse checks; PyYAML is unavailable")
    raise SystemExit(0)

failed = False
for raw_path in sys.argv[1:]:
    path = Path(raw_path)
    try:
        with path.open("r", encoding="utf-8") as handle:
            yaml.safe_load(handle)
    except Exception as exc:
        print(f"ERROR: failed to parse YAML {path}: {exc}", file=sys.stderr)
        failed = True

raise SystemExit(1 if failed else 0)
PY
}

run_rbpodo_controller_sim_cartesian_gate() {
  run_shell_syntax_checks
  run_servo_gate_or_skip_missing_deps
  run_python_surface_tests
  echo "codex_gate: skipping rbpodo controller-simulation Cartesian controller run by default"
}

run_state_fanout_gate() {
  run_servo_gate_or_skip_missing_deps
  run_yaml_parse_checks_if_available rb_servo_server/config/*.yaml rb_servo_server/config/local/*.yaml configs/**/*.yaml
  run_python_surface_tests
}

run_gui_tcp_ref_actual_gate() {
  run_gui_tests
  python3 -m compileall -q rb_gui/rb_servo_gui
}

run_policy_dataset_schema_gate() {
  run_policy_runner_tests
  grep_existing "robotics_lab\\.policy_runner\\.episode\\.v1|robotics_lab\\.episode\\.v1" \
    policy_runner README.md docs
  grep_existing "dataset|episode" \
    policy_runner/README.md policy_runner/policy_runner policy_runner/tests README.md docs
  grep_existing "observations|action" \
    policy_runner/README.md policy_runner/policy_runner/recording.py policy_runner/tests
  echo "codex_gate: skipping real policy/data collection run by default"
}

run_doc_hygiene_gate() {
  run_shell_syntax_checks
  if [[ -e README_DOCS_UPDATE.md ]]; then
    echo "ERROR: README_DOCS_UPDATE.md must not exist; fold doc-update notes into source-of-truth docs" >&2
    return 1
  fi
  if [[ -e rb_servo_server/docker-compose.yml ]]; then
    echo "ERROR: rb_servo_server/docker-compose.yml must not exist; use the repository-root docker-compose.yml (camera_server)" >&2
    return 1
  fi

  grep_existing "source-of-truth current review" docs/current_review.md
  grep_existing "REVIEW\.md" docs/current_review.md
  local current_review_lines
  current_review_lines="$(wc -l < docs/current_review.md)"
  if [[ "${current_review_lines}" -gt 12 ]]; then
    echo "ERROR: docs/current_review.md must be a short redirect to REVIEW.md, not a mirrored review" >&2
    return 1
  fi

  local forbidden_config
  for forbidden_config in \
    rb_servo_server/config/dual_rbsim.yaml \
    rb_servo_server/config/dual_rb_simulator.yaml \
    rb_servo_server/config/dual_rb_simulator_compose.yaml \
    rb_simulator/config/dual_rb3_730e.yaml
  do
    if [[ -e "${forbidden_config}" ]]; then
      echo "ERROR: deprecated config remains in active config directory: ${forbidden_config}" >&2
      return 1
    fi
  done

  local archived_config
  for archived_config in \
    docs/archive/configs/rb_servo_server_dual_rbsim.legacy.yaml \
    docs/archive/configs/rb_servo_server_dual_rb_simulator.legacy.yaml \
    docs/archive/configs/rb_servo_server_dual_rb_simulator_compose.legacy.yaml \
    docs/archive/configs/rb_simulator_dual_rb3_730e.legacy.yaml
  do
    if [[ ! -f "${archived_config}" ]]; then
      echo "ERROR: missing archived deprecated config: ${archived_config}" >&2
      return 1
    fi
    grep_existing "Deprecated|deprecated|HISTORICAL|historical|archive|archived" "${archived_config}"
  done

  grep_existing "historical reference only|not runnable source-of-truth" docs/archive/configs/README.md README.md README.en.md
  grep_absent '(^|[^[:alnum:]_.-])dual_real\.yaml([^[:alnum:]_.-]|$)' README.md docs AGENTS.md rb_servo_server/docs
}

run_gui_split_gate() {
  run_gui_tests
  python3 -m compileall -q rb_gui/rb_servo_gui
}

check_real_config_safety_docs() {
  grep_existing "RB_ALLOW_REAL_ROBOT" README.md docs AGENTS.md rb_servo_server/docs rb_servo_server/config/dual_real.example.yaml
  grep_existing "RB_ALLOW_REAL_MOTION" README.md docs AGENTS.md rb_servo_server/docs rb_servo_server/config/dual_real.example.yaml
  grep_existing "RB_ALLOW_REAL_CARTESIAN" README.md docs AGENTS.md rb_servo_server/docs rb_servo_server/config/dual_real.example.yaml
  grep_existing "dual_real.example.yaml" README.md docs rb_servo_server/docs rb_servo_server/config/dual_real.example.yaml
  grep_existing "dual_real_readonly.yaml" README.md docs rb_servo_server/docs rb_servo_server/config/dual_real.example.yaml
  grep_existing "dual_real_motion.yaml" README.md docs rb_servo_server/docs rb_servo_server/config/dual_real.example.yaml

  if [[ -f rb_servo_server/config/dual_real.yaml ]]; then
    echo "ERROR: rb_servo_server/config/dual_real.yaml must not exist; use dual_real.example.yaml and config/local/*.yaml" >&2
    return 1
  fi
}

check_backend_contract_docs() {
  grep_existing "BackendResult|SendServoJResult" README.md docs AGENTS.md rb_servo_server/docs
  grep_existing "RobotFault" README.md docs rb_servo_server/docs
  grep_existing "TransportWriteFailed|TransportTimeout|TransportReadFailed" README.md docs rb_servo_server/docs
  grep_existing "SuppressedByPolicy" README.md docs rb_servo_server/docs
  grep_existing "FaultContext|fault_context" README.md docs rb_servo_server/docs
}

check_worker_docs() {
  grep_existing "ArmWorker" README.md docs rb_servo_server/docs
  grep_existing "latest_wins|latest-wins|worker_queue_policy" README.md docs rb_servo_server/docs
  grep_existing "worker_command_drops_total|command_drops_total" README.md docs rb_servo_server/docs
  grep_existing "io_model: worker|worker io_model" README.md docs rb_servo_server/docs rb_servo_server/config
  grep_existing "io_model: direct|direct io_model" README.md docs rb_servo_server/docs rb_servo_server/config
}

check_canonical_config_docs() {
  check_real_config_safety_docs
  grep_existing "Canonical Config Names" README.md docs rb_servo_server/docs || true
  grep_existing "dual_mock.yaml" README.md docs rb_servo_server/docs rb_servo_server/config
  grep_absent "dual_real\.yaml" README.md docs AGENTS.md rb_servo_server/docs
}

check_command_source_docs() {
  grep_existing "command_source|Command Source Lease|source_id|lease_token|AcquireLease" \
    README.md docs rb_servo_server/docs rb_servo_server/docs/network_protocol.md policy_runner/README.md
  grep_existing 'enforce_lease: false|defaults to off|defaults to `false`|defaults to false' \
    README.md docs rb_servo_server/docs rb_servo_server/docs/network_protocol.md
}

check_tcp_pose_docs() {
  grep_existing "quaternion_xyzw|qx|qy|qz|qw" README.md docs rb_servo_server/docs rb_gui/README.md || true
  grep_existing "ik_duration_us|fk_duration_us|cartesian_solve" README.md docs rb_servo_server/docs || true
}

check_camera_acceptance_docs() {
  grep_existing "camera_stale_timeout_sec|requires_camera" README.md docs policy_runner/README.md
  grep_existing "camera acceptance|3-camera|30 FPS|D435f|D405" README.md docs camera_server/README.md camera_server/docs || true
}

check_dev_env_docs() {
  grep_existing "libyaml-cpp-dev" README.md docs scripts || true
  grep_existing "nlohmann-json3-dev" README.md docs scripts || true
  grep_existing "install_deps_ubuntu.sh" README.md docs scripts || true
}

run_p3f_gate() {
  run_shell_syntax_checks
  grep_existing "RB_ALLOW_REAL_CARTESIAN" README.md rb_servo_server/docs
  run_servo_pinocchio_gate
  run_optional_tcp_pose_acceptance
}

run_mig12_gate() {
  run_shell_syntax_checks
  check_real_config_safety_docs
  check_backend_contract_docs
  check_worker_docs
  run_gui_tests
  run_policy_runner_tests
  run_servo_gate_or_skip_missing_deps
  run_servo_pinocchio_gate
}

run_mig13_gate() {
  run_shell_syntax_checks
  run_servo_gate_or_skip_missing_deps
}

run_mig20_gate() {
  run_shell_syntax_checks
  run_servo_gate_or_skip_missing_deps
  run_servo_pinocchio_gate
}

run_mig26_gate() {
  run_shell_syntax_checks
  check_canonical_config_docs
  check_backend_contract_docs
  check_worker_docs
  check_command_source_docs
  check_tcp_pose_docs
  check_camera_acceptance_docs
  check_dev_env_docs
  if [[ "${CODEX_STRICT_DEPS:-0}" == "1" ]]; then
    ./scripts/check_deps.sh --profile hardware-free
  else
    echo "codex_gate: CODEX_STRICT_DEPS is not 1; dependency preflight is advisory"
    ./scripts/check_deps.sh --profile hardware-free || true
  fi
  run_python_compile_checks
  run_gui_tests
  run_policy_runner_tests
  run_camera_gate_or_skip_missing_deps
  run_servo_gate_or_skip_missing_deps
  run_servo_pinocchio_gate
  if [[ "${CODEX_RUN_TCP_ACCEPTANCE:-0}" == "1" ]]; then
    run_optional_tcp_pose_acceptance
  else
    echo "codex_gate: skipping TCP pose acceptance; set CODEX_RUN_TCP_ACCEPTANCE=1 to enable"
  fi
}

case "$TASK" in
  P0-A)
    grep_existing "RB_ALLOW_REAL_MOTION" README.md docs
    ;;
  P0-B)
    :
    ;;
  P0-D|P1-A|P1-B|P1-F|P2-A|P2-C|P3-A|P3-B|P3-C)
    run_servo_gate_or_skip_missing_deps
    ;;
  P0-G)
    run_camera_gate_or_skip_missing_deps
    ;;
  P0-H|P2-D|P3-D)
    run_gui_tests
    ;;
  P1-C)
    run_servo_gate_or_skip_missing_deps
    ;;
  P1-D|P1-E|P2-E|P3-E)
    run_policy_runner_tests
    ;;
  P2-B)
    grep_existing "geometry_valid_for_real_policy" calibration geometry docs README.md
    grep_existing "configured_estimate" calibration geometry docs README.md
    ;;
  P3-F)
    run_p3f_gate
    ;;

  MIG-00)
    run_shell_syntax_checks
    check_real_config_safety_docs
    ;;
  MIG-01|MIG-02|MIG-04|MIG-05|MIG-06|MIG-07|MIG-08|MIG-09|MIG-11)
    run_servo_gate_or_skip_missing_deps
    ;;
  MIG-03)
    run_servo_gate_or_skip_missing_deps
    ;;
  MIG-10)
    run_servo_gate_or_skip_missing_deps
    ;;
  MIG-12)
    run_mig12_gate
    ;;
  MIG-13)
    run_mig13_gate
    ;;
  MIG-14|MIG-15|MIG-16|MIG-21|MIG-22|MIG-23|MIG-24)
    run_servo_gate_or_skip_missing_deps
    ;;
  MIG-17)
    run_policy_runner_tests
    ;;
  MIG-18)
    run_shell_syntax_checks
    check_canonical_config_docs
    run_gui_tests
    run_policy_runner_tests
    ;;
  MIG-19|MIG-25)
    run_shell_syntax_checks
    run_gui_tests
    run_policy_runner_tests
    ;;
  MIG-20)
    run_mig20_gate
    ;;
  MIG-26)
    run_mig26_gate
    ;;

  HARDEN-00)
    run_shell_syntax_checks
    ;;
  HARDEN-01)
    run_shell_syntax_checks
    check_dev_env_docs
    ;;
  HARDEN-02)
    run_shell_syntax_checks
    run_servo_gate_or_skip_missing_deps
    ;;
  HARDEN-03)
    run_shell_syntax_checks
    run_servo_gate_or_skip_missing_deps
    check_worker_docs
    ;;
  HARDEN-04)
    run_shell_syntax_checks
    run_servo_gate_or_skip_missing_deps
    grep_existing "disable_waiting_ack|rbpodo_stop_unverified|operator intervention|read-only" README.md docs rb_servo_server/docs rb_servo_server/config || true
    ;;
  HARDEN-05)
    run_policy_runner_tests
    run_python_compile_checks
    grep_existing "dual_spacemouse_pose_target|tcp_pose_target" policy_runner policy_runner/README.md
    ;;
  HARDEN-06)
    run_shell_syntax_checks
    check_tcp_pose_docs
    run_servo_gate_or_skip_missing_deps
    run_servo_pinocchio_gate
    run_optional_tcp_pose_acceptance
    ;;
  HARDEN-07)
    run_servo_gate_or_skip_missing_deps
    run_policy_runner_tests
    check_command_source_docs
    ;;
  HARDEN-08)
    run_shell_syntax_checks
    run_camera_gate_or_skip_missing_deps
    check_camera_acceptance_docs
    ;;
  HARDEN-09)
    run_shell_syntax_checks
    check_canonical_config_docs
    check_backend_contract_docs
    check_worker_docs
    check_command_source_docs
    run_gui_tests
    run_policy_runner_tests
    ;;
  HARDEN-10)
    run_mig26_gate
    ;;
  CART-HARDEN-05)
    run_cart_harden_05_gate
    ;;
  CART-MATH-01|CART-MATH-02|CART-MATH-03)
    run_cart_math_gate
    ;;
  SIM-HARDEN-01)
    run_servo_gate_or_skip_missing_deps
    ;;
  CART-TUNE-01|FAULT-DIAG-01)
    run_servo_gate_or_skip_missing_deps
    ;;
  CART-TUNE-02)
    run_cart_tune_02_gate
    ;;
  CART-ACCEPT-01)
    run_cart_accept_gate
    ;;
  CART-SERVO-01)
    run_cart_servo_01_gate
    ;;
  CART-SERVO-02)
    run_cart_servo_02_gate
    ;;
  CART-SERVO-03)
    run_cart_servo_03_gate
    ;;
  GATE-RBPODO-00)
    run_shell_syntax_checks
    ;;
  RBPODO-SERVO-PARAM-01)
    run_shell_syntax_checks
    run_servo_gate_or_skip_missing_deps
    run_python_surface_tests
    ;;
  RBPODO-ACK-01)
    run_shell_syntax_checks
    run_servo_gate_or_skip_missing_deps
    run_python_surface_tests
    ;;
  RBPODO-ACCEPT-01)
    run_rbpodo_accept_gate
    ;;
  RBPODO-READONLY-DIAG-01)
    run_rbpodo_readonly_diag_gate
    ;;
  RBPODO-STATUS-FIELDS-01)
    run_rbpodo_status_fields_gate
    ;;
  RBPODO-JOINT-WRAP-01)
    run_rbpodo_joint_wrap_gate
    ;;
  RBPODO-JOINT-RANGE-POLICY-01|00_joint_range_policy_rbpodo_raw_controller_limits)
    run_rbpodo_joint_range_policy_gate
    ;;
  RBPODO-BRINGUP-TOOLS-01)
    run_rbpodo_bringup_tools_gate
    ;;
  RBPODO-DOC-01)
    run_rbpodo_doc_gate
    ;;
  RBPODO-PGMODE-SIM-01)
    run_rbpodo_pgmode_sim_gate
    ;;
  RBPODO-REFERENCE-TCP-01)
    run_rbpodo_reference_tcp_gate
    ;;
  RBPODO-CONTROLLER-SIM-GATE-01)
    run_rbpodo_controller_sim_gate
    ;;
  RBPODO-CONTROLLER-SIM-CARTESIAN-00)
    run_shell_syntax_checks
    ;;
  RBPODO-CONTROLLER-SIM-CARTESIAN-01)
    run_rbpodo_controller_sim_cartesian_gate
    ;;
  RBPODO-LIVE-VIZ-00)
    run_shell_syntax_checks
    ;;
  STATE-FANOUT-01)
    run_state_fanout_gate
    ;;
  GUI-TCP-REF-ACTUAL-01)
    run_gui_tcp_ref_actual_gate
    ;;
  RBPODO-TUNE-GATE-00)
    run_shell_syntax_checks
    ;;
  MEASURE-P0-GATE-00)
    run_shell_syntax_checks
    ;;
  RBPODO-MEASURE-STATE-PARITY-01)
    run_rbpodo_measure_state_parity_gate
    ;;
  RBPODO-MEASURE-RAW-DATA-01)
    run_rbpodo_measure_raw_data_gate
    ;;
  RBPODO-MEASURE-TIMESTAMP-01)
    run_rbpodo_measure_timestamp_gate
    ;;
  RBPODO-MEASURE-RELIABILITY-REPORT-01)
    run_rbpodo_measure_reliability_report_gate
    ;;
  GATE-RBPODO-500-P0-P1-00)
    run_shell_syntax_checks
    ;;
  P0-PARITY-REPAIR-01)
    run_p0_parity_repair_gate
    ;;
  P0-DIAGNOSTICS-ROOTCAUSE-01)
    run_p0_diagnostics_rootcause_gate
    ;;
  P0-RAW-PAYLOAD-FIXTURE-02)
    run_p0_raw_payload_fixture_gate
    ;;
  P0-MEASUREMENT-GATING-01)
    run_p0_measurement_gating_gate
    ;;
  RBPODO-ASYNC-GATE-00)
    run_shell_syntax_checks
    ;;
  RBPODO-ASYNC-CONTRACT-01)
    run_rbpodo_async_contract_probe_gate
    ;;
  RBPODO-ASYNC-SDK-PROBE-01)
    run_rbpodo_async_contract_probe_gate
    ;;
  RBPODO-ASYNC-WORKER-01)
    run_rbpodo_async_cpp_gate
    ;;
  RBPODO-ASYNC-REFERENCE-SUPERVISOR-01)
    run_rbpodo_async_cpp_gate
    ;;
  RBPODO-ASYNC-REPORT-01)
    run_rbpodo_async_report_gate
    ;;
  RBPODO-ASYNC-RUNBOOK-01)
    run_rbpodo_async_runbook_gate
    ;;
  03_pgmode_real_transition_acceptance)
    run_gene_umi_physical_transition_gate
    ;;
  04_umi_hdf5_audit_adapter)
    run_gene_umi_hdf5_audit_gate
    ;;
  05_umi_bimanual_collection)
    run_gene_umi_bimanual_collection_gate
    ;;
  06_flow_training_preflight_eval)
    run_gene_umi_flow_training_gate
    ;;
  07_policy_runner_rollout_modes)
    run_gene_umi_policy_rollout_modes_gate
    ;;
  08_dual_arm_real_policy_and_gripper)
    run_gene_umi_dual_arm_gripper_gate
    ;;
  09_docs_ci_artifact_manifest)
    run_gene_umi_docs_ci_artifact_manifest_gate
    ;;
  01_fix_controller_sim_safety_semantics)
    run_spacemouse_fix_controller_sim_safety_gate
    ;;
  02_pgmode_spacemouse_end_to_end_dryrun)
    run_pgmode_spacemouse_end_to_end_dryrun_gate
    ;;
  03_flow_infer_rollout_modes)
    run_spacemouse_flow_infer_rollout_modes_gate
    ;;
  04_flow_policy_tcp_pose_target_controller_sim)
    run_flow_policy_tcp_pose_target_controller_sim_gate
    ;;
  05_viser_pgmode_operator_view)
    run_viser_pgmode_operator_view_gate
    ;;
  06_gripper_and_dual_arm_policy_gate)
    run_spacemouse_gripper_dual_arm_policy_gate
    ;;
  07_ml_preflight_gate_stability)
    run_ml_preflight_gate_stability_gate
    ;;
  08_umi_hdf5_manifest_robustness)
    run_umi_hdf5_manifest_robustness_gate
    ;;
  09_artifact_manifest_docs_makefile)
    run_artifact_manifest_docs_makefile_gate
    ;;
  10_source_hygiene_local_configs)
    run_source_hygiene_local_configs_gate
    ;;
  PHYSICAL-READINESS-BLOCKERS-CLARITY-01)
    run_physical_readiness_blockers_clarity_gate
    ;;
  P1-SERVO-PARAM-SWEEP-01)
    run_p1_servo_param_sweep_gate
    ;;
  POLICY-DATASET-SCHEMA-01)
    run_policy_dataset_schema_gate
    ;;
  00_update_codex_gate_supported_scope)
    run_supported_scope_common_gate
    ;;
  "01_remove_rb""script_tcp_backend_and_experiments")
    run_supported_scope_backend_gate
    ;;
  02_rbpodo_only_supported_real_backend_contract)
    run_supported_scope_backend_gate
    ;;
  03_standardize_500hz_control_defaults)
    run_supported_scope_500hz_gate
    ;;
  04_supported_scope_docs_ci_hygiene)
    run_supported_scope_docs_gate
    ;;
  DOC-HYGIENE-01)
    run_doc_hygiene_gate
    ;;
  VM-PARITY-GUARDRAILS-01)
    run_vm_parity_guardrails_gate
    ;;
  GUI-SPLIT-01)
    run_gui_split_gate
    ;;

  *)
    echo "ERROR: unknown task: $TASK" >&2
    exit 2
    ;;
esac
