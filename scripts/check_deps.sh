#!/usr/bin/env bash
set -u -o pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROFILE="hardware-free"

usage() {
  cat <<'USAGE'
Usage: scripts/check_deps.sh --profile <hardware-free|real-camera|real-robot|kinematics>

Profiles:
  hardware-free  local mock/stub validation: CMake, C++17, Python, yaml-cpp, nlohmann_json
  real-camera    RealSense capture readiness: CMake, C++17, yaml-cpp, librealsense2, libzmq
  real-robot     RB controller SDK readiness: hardware-free basics plus rbpodo SDK/package
  kinematics     FK/IK readiness: CMake, C++17, Eigen3, Pinocchio

These profiles check dependencies only. They do not enable real robot
connection, real motion, RealSense capture, or Cartesian motion gates.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile)
      if [[ $# -lt 2 ]]; then
        echo "check_deps: --profile requires a value" >&2
        exit 2
      fi
      PROFILE="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "check_deps: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

failures=()
warnings=()

add_failure() {
  failures+=("$1")
}

add_warning() {
  warnings+=("$1")
}

have_command() {
  command -v "$1" >/dev/null 2>&1
}

require_command() {
  local command_name="$1"
  local hint="$2"
  if ! have_command "${command_name}"; then
    add_failure "missing command '${command_name}'. ${hint}"
  fi
}

cmake_prefix_args=()
if [[ -n "${CMAKE_PREFIX_PATH:-}" ]]; then
  cmake_prefix_args=(-DCMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}")
elif [[ -d "${HOME}/miniconda3/share/cmake/nlohmann_json" || -d "${HOME}/miniconda3/lib/cmake/nlohmann_json" ]]; then
  cmake_prefix_args=(-DCMAKE_PREFIX_PATH="${HOME}/miniconda3")
fi

check_cxx17() {
  local compiler="${CXX:-}"
  if [[ -z "${compiler}" ]]; then
    if have_command c++; then
      compiler="c++"
    elif have_command g++; then
      compiler="g++"
    elif have_command clang++; then
      compiler="clang++"
    else
      add_failure "missing C++ compiler. Install g++ or clang++."
      return
    fi
  fi

  local tmpdir
  tmpdir="$(mktemp -d)"
  printf '%s\n' '#include <optional>' 'int main(){ std::optional<int> v = 1; return *v - 1; }' > "${tmpdir}/cxx17.cpp"
  if ! "${compiler}" -std=c++17 "${tmpdir}/cxx17.cpp" -o "${tmpdir}/cxx17" >/dev/null 2>&1; then
    add_failure "C++ compiler '${compiler}' cannot compile a C++17 smoke file."
  fi
  rm -rf "${tmpdir}"
}

check_cmake_package() {
  local package_name="$1"
  local hint="$2"
  if ! have_command cmake; then
    return
  fi

  local tmpdir
  tmpdir="$(mktemp -d)"
  cat > "${tmpdir}/CMakeLists.txt" <<EOF
cmake_minimum_required(VERSION 3.16)
project(check_${package_name} LANGUAGES CXX)
find_package(${package_name} REQUIRED)
EOF
  if ! cmake -S "${tmpdir}" -B "${tmpdir}/build" "${cmake_prefix_args[@]}" >/dev/null 2>&1; then
    add_failure "missing CMake package '${package_name}'. ${hint}"
  fi
  rm -rf "${tmpdir}"
}

check_pkg_config_module() {
  local module_name="$1"
  local hint="$2"
  if ! have_command pkg-config; then
    add_failure "missing command 'pkg-config'. ${hint}"
    return
  fi
  if ! pkg-config --exists "${module_name}"; then
    add_failure "missing pkg-config module '${module_name}'. ${hint}"
  fi
}

check_python() {
  require_command python3 "Install python3."
  if have_command python3; then
    if ! python3 - <<'PY' >/dev/null 2>&1
import sys
raise SystemExit(0 if sys.version_info >= (3, 10) else 1)
PY
    then
      add_failure "python3 must be version 3.10 or newer."
    fi
    if ! python3 -m venv --help >/dev/null 2>&1; then
      add_warning "python3 venv is unavailable; install python3-venv if you need isolated local environments."
    fi
  fi
}

check_hardware_free() {
  require_command cmake "Install cmake."
  require_command make "Install make or build-essential."
  check_cxx17
  check_python
  check_cmake_package yaml-cpp "Ubuntu: sudo apt-get install libyaml-cpp-dev"
  check_cmake_package nlohmann_json "Ubuntu: sudo apt-get install nlohmann-json3-dev, or set CMAKE_PREFIX_PATH."
}

check_real_camera() {
  require_command cmake "Install cmake."
  check_cxx17
  check_cmake_package yaml-cpp "Ubuntu: sudo apt-get install libyaml-cpp-dev"
  check_pkg_config_module realsense2 "Install librealsense2-dev and RealSense udev rules; USB access is required at runtime."
  check_pkg_config_module libzmq "Ubuntu: sudo apt-get install libzmq3-dev"
  require_command rs-enumerate-devices "Install RealSense tools such as librealsense2-utils for serial identification."
  add_warning "real-camera runtime also needs USB device access, udev rules, shared memory sizing, serial-specific local config, and the real_camera compose profile."
  add_warning "real-camera acceptance is documented in docs/runbooks/camera_acceptance.md and does not imply real robot readiness."
}

check_real_robot() {
  check_hardware_free
  check_cmake_package rbpodo "Install the Rainbow rbpodo SDK and expose it through CMAKE_PREFIX_PATH or RBPODO_ROOT."
  add_warning "real robot startup remains gated by RB_ALLOW_REAL_ROBOT=1; real motion also needs RB_ALLOW_REAL_MOTION=1."
}

check_kinematics() {
  require_command cmake "Install cmake."
  check_cxx17
  check_cmake_package Eigen3 "Ubuntu: sudo apt-get install libeigen3-dev"
  check_cmake_package pinocchio "Install Pinocchio and set CMAKE_PREFIX_PATH when it is outside the system prefix."
}

case "${PROFILE}" in
  hardware-free)
    check_hardware_free
    ;;
  real-camera)
    check_real_camera
    ;;
  real-robot)
    check_real_robot
    ;;
  kinematics)
    check_kinematics
    ;;
  *)
    echo "check_deps: unknown profile '${PROFILE}'" >&2
    usage >&2
    exit 2
    ;;
esac

if ((${#warnings[@]})); then
  printf 'check_deps: warnings for profile %s:\n' "${PROFILE}"
  for warning in "${warnings[@]}"; do
    printf '  - %s\n' "${warning}"
  done
fi

if ((${#failures[@]})); then
  printf 'check_deps: missing dependencies for profile %s:\n' "${PROFILE}" >&2
  for failure in "${failures[@]}"; do
    printf '  - %s\n' "${failure}" >&2
  done
  exit 1
fi

printf 'check_deps: profile %s passed\n' "${PROFILE}"
