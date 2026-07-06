#!/usr/bin/env bash
set -euo pipefail

PROFILE="hardware-free"
DRY_RUN=0
WITH_PYTHON_DEV=0
ROBOTPKG_DIST="${ROBOTPKG_DIST:-}"
RB_PINOCCHIO_SOURCE="${RB_PINOCCHIO_SOURCE:-auto}"
PINOCCHIO_VERSION="${PINOCCHIO_VERSION:-v3.9.0}"
PINOCCHIO_BUILD_JOBS="${PINOCCHIO_BUILD_JOBS:-}"
ROBOTPKG_LIST="/etc/apt/sources.list.d/robotpkg.list"
ROBOTPKG_KEYRING="/etc/apt/keyrings/robotpkg.asc"
ROBOTPKG_URL="http://robotpkg.openrobots.org/packages/debian/pub"
ROBOTPKG_KEY_URL="http://robotpkg.openrobots.org/packages/debian/robotpkg.asc"
PINOCCHIO_REPO_URL="https://github.com/stack-of-tasks/pinocchio"
PINOCCHIO_INSTALL_PREFIX="/opt/openrobots"

usage() {
  cat <<'USAGE'
Usage: scripts/install_deps_ubuntu.sh [--profile <hardware-free|kinematics|real-camera|real-robot|all>] [--dry-run] [--with-python-dev]

Installs Ubuntu apt packages for robotics_lab development profiles.

Profiles:
  hardware-free  CMake, C++17 toolchain, Python, yaml-cpp, nlohmann_json, Eigen3, Pinocchio
  kinematics     alias for hardware-free Cartesian math dependencies
  real-camera    hardware-free plus RealSense/ZMQ development packages
  real-robot     hardware-free basics; rbpodo SDK still requires vendor install
  all            all apt-managed packages from the profiles above

Pinocchio:
  Hardware-free and kinematics dependencies install Pinocchio under /opt/openrobots.
  The default provider is robotpkg-pinocchio on Ubuntu jammy, or when ROBOTPKG_DIST
  is set explicitly. On non-jammy hosts with ROBOTPKG_DIST unset, the helper builds
  a pinned source release instead. Set RB_PINOCCHIO_SOURCE=1 to force the source
  build, or RB_PINOCCHIO_SOURCE=0 to force robotpkg and keep robotpkg codename
  checks. PINOCCHIO_VERSION overrides the source tag (default: v3.9.0).
  PINOCCHIO_BUILD_JOBS overrides the source build parallelism. When unset, the
  source build is capped by available memory to avoid OOM on low-RAM hosts.

Dry run:
  --dry-run prints apt packages, robotpkg/source selection, source clone/tag,
  CMake flags, install prefix, and sudo steps without running apt, git, cmake,
  or sudo.

This script installs dependencies only. It does not enable RealSense capture,
site-local real robot configs, or robot motion. Legacy RB_ALLOW_REAL_* execution
gates are retired; real motion is config-driven.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile)
      [[ $# -ge 2 ]] || { echo "install_deps_ubuntu: --profile requires a value" >&2; exit 2; }
      PROFILE="$2"
      shift 2
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    --with-python-dev)
      WITH_PYTHON_DEV=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "install_deps_ubuntu: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if ! command -v apt-get >/dev/null 2>&1; then
  echo "install_deps_ubuntu: apt-get not found; this helper is intended for Ubuntu/Debian hosts." >&2
  exit 2
fi

common_packages=(
  build-essential
  ca-certificates
  cmake
  curl
  git
  gnupg
  make
  pkg-config
  python3
  python3-pip
  python3-venv
  libyaml-cpp-dev
  nlohmann-json3-dev
  libeigen3-dev
)

robotpkg_pinocchio_packages=(
  robotpkg-pinocchio
)

pinocchio_source_build_packages=(
  libboost-all-dev
  liburdfdom-dev
  liburdfdom-headers-dev
  libconsole-bridge-dev
)

python_dev_packages=(
  python3-dev
)

kinematics_packages=()

real_camera_packages=(
  libzmq3-dev
  librealsense2-dev
  librealsense2-utils
)

packages=()
PINOCCHIO_REQUIRED=0
add_packages() {
  local package
  for package in "$@"; do
    packages+=("${package}")
  done
}

add_common() {
  add_packages "${common_packages[@]}"
  PINOCCHIO_REQUIRED=1
  if [[ "${WITH_PYTHON_DEV}" -eq 1 ]]; then
    add_packages "${python_dev_packages[@]}"
  fi
}

host_ubuntu_codename() {
  if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    . /etc/os-release
    printf '%s\n' "${VERSION_CODENAME:-${UBUNTU_CODENAME:-}}"
    return
  fi
  printf '\n'
}

robotpkg_codename() {
  local host_codename="$1"
  if [[ -n "${ROBOTPKG_DIST}" ]]; then
    printf '%s\n' "${ROBOTPKG_DIST}"
    return
  fi
  printf '%s\n' "${host_codename}"
}

pinocchio_install_method() {
  local host_codename="$1"
  # Selection logic:
  # - jammy keeps the existing robotpkg-pinocchio path.
  # - ROBOTPKG_DIST explicitly opts into robotpkg for that distribution.
  # - non-jammy without ROBOTPKG_DIST uses the pinned source build.
  # - RB_PINOCCHIO_SOURCE=1 forces source; RB_PINOCCHIO_SOURCE=0 forces robotpkg.
  case "${RB_PINOCCHIO_SOURCE}" in
    1|true|TRUE|yes|YES|on|ON)
      printf 'source\n'
      ;;
    0|false|FALSE|no|NO|off|OFF)
      printf 'robotpkg\n'
      ;;
    auto|AUTO|"")
      if [[ -n "${ROBOTPKG_DIST}" || "${host_codename}" == "jammy" ]]; then
        printf 'robotpkg\n'
      else
        printf 'source\n'
      fi
      ;;
    *)
      echo "install_deps_ubuntu: invalid RB_PINOCCHIO_SOURCE='${RB_PINOCCHIO_SOURCE}'; use 1, 0, or auto." >&2
      exit 2
      ;;
  esac
}

configure_robotpkg_repo() {
  local codename="$1"
  if [[ -z "${codename}" ]]; then
    echo "install_deps_ubuntu: cannot determine Ubuntu codename; set ROBOTPKG_DIST=jammy to use robotpkg." >&2
    exit 2
  fi
  if [[ "${codename}" != "jammy" && -z "${ROBOTPKG_DIST:-}" ]]; then
    echo "install_deps_ubuntu: robotpkg auto-setup is pinned to jammy for robotpkg binary compatibility (the hardware-free build baseline)." >&2
    echo "install_deps_ubuntu: set ROBOTPKG_DIST=${codename} explicitly if robotpkg supports your distribution." >&2
    exit 2
  fi

  echo "install_deps_ubuntu: configuring robotpkg (${codename}) for robotpkg-pinocchio"
  if [[ "${DRY_RUN}" -eq 1 ]]; then
    echo "install_deps_ubuntu: dry run; would run: sudo apt-get update"
    echo "install_deps_ubuntu: dry run; would run: sudo apt-get install -y --no-install-recommends ca-certificates curl gnupg"
    echo "install_deps_ubuntu: dry run; would run: sudo install -d -m 0755 /etc/apt/keyrings"
    echo "install_deps_ubuntu: dry run; would import ${ROBOTPKG_KEY_URL} to ${ROBOTPKG_KEYRING}"
    echo "install_deps_ubuntu: dry run; would write ${ROBOTPKG_LIST}: deb [arch=amd64 signed-by=${ROBOTPKG_KEYRING}] ${ROBOTPKG_URL} ${codename} robotpkg"
    return
  fi

  sudo apt-get update
  sudo apt-get install -y --no-install-recommends ca-certificates curl gnupg
  sudo install -d -m 0755 /etc/apt/keyrings
  curl -fsSL "${ROBOTPKG_KEY_URL}" | sudo tee "${ROBOTPKG_KEYRING}" >/dev/null
  echo "deb [arch=amd64 signed-by=${ROBOTPKG_KEYRING}] ${ROBOTPKG_URL} ${codename} robotpkg" | \
    sudo tee "${ROBOTPKG_LIST}" >/dev/null
}

pinocchio_cmake_flags=(
  "-DBUILD_PYTHON_INTERFACE=OFF"
  "-DBUILD_WITH_URDF_SUPPORT=ON"
  "-DBUILD_TESTING=OFF"
  "-DBUILD_WITH_COLLISION_SUPPORT=OFF"
  "-DCMAKE_BUILD_TYPE=Release"
  "-DCMAKE_INSTALL_PREFIX=${PINOCCHIO_INSTALL_PREFIX}"
)

pinocchio_cpu_count() {
  local cpus
  cpus="$(nproc 2>/dev/null || printf '1\n')"
  if [[ ! "${cpus}" =~ ^[1-9][0-9]*$ ]]; then
    cpus=1
  fi
  printf '%s\n' "${cpus}"
}

pinocchio_mem_available_kb() {
  local key value unit
  if [[ -r /proc/meminfo ]]; then
    while read -r key value unit; do
      if [[ "${key}" == "MemAvailable:" && "${value}" =~ ^[0-9]+$ ]]; then
        printf '%s\n' "${value}"
        return
      fi
    done < /proc/meminfo
  fi
  printf '\n'
}

pinocchio_default_build_jobs() {
  local cpus mem_available_kb mem_jobs jobs
  cpus="$(pinocchio_cpu_count)"
  mem_available_kb="$(pinocchio_mem_available_kb)"
  if [[ -n "${mem_available_kb}" ]]; then
    mem_jobs=$((mem_available_kb / 4000000))
    if ((mem_jobs < 1)); then
      mem_jobs=1
    fi
  else
    mem_jobs="${cpus}"
  fi
  jobs="${mem_jobs}"
  if ((jobs > cpus)); then
    jobs="${cpus}"
  fi
  printf '%s\n' "${jobs}"
}

pinocchio_build_jobs() {
  if [[ -n "${PINOCCHIO_BUILD_JOBS}" ]]; then
    if [[ ! "${PINOCCHIO_BUILD_JOBS}" =~ ^[1-9][0-9]*$ ]]; then
      echo "install_deps_ubuntu: invalid PINOCCHIO_BUILD_JOBS='${PINOCCHIO_BUILD_JOBS}'; use a positive integer." >&2
      exit 2
    fi
    printf '%s\n' "${PINOCCHIO_BUILD_JOBS}"
    return
  fi
  pinocchio_default_build_jobs
}

print_pinocchio_build_jobs() {
  local jobs="$1"
  echo "install_deps_ubuntu:   build jobs: ${jobs}"
  if [[ -n "${PINOCCHIO_BUILD_JOBS}" ]]; then
    echo "install_deps_ubuntu:   build jobs set by PINOCCHIO_BUILD_JOBS."
  else
    echo "install_deps_ubuntu:   build jobs are memory-capped to avoid OOM; set PINOCCHIO_BUILD_JOBS to override."
  fi
}

print_pinocchio_source_plan() {
  local jobs
  jobs="$(pinocchio_build_jobs)"
  echo "install_deps_ubuntu: Pinocchio source build plan:"
  echo "install_deps_ubuntu:   repository: ${PINOCCHIO_REPO_URL}"
  echo "install_deps_ubuntu:   tag/version: ${PINOCCHIO_VERSION}"
  echo "install_deps_ubuntu:   recursive submodules: yes"
  echo "install_deps_ubuntu:   install prefix: ${PINOCCHIO_INSTALL_PREFIX}"
  print_pinocchio_build_jobs "${jobs}"
  echo "install_deps_ubuntu:   CMake flags:"
  printf 'install_deps_ubuntu:     %s\n' "${pinocchio_cmake_flags[@]}"
  if [[ "${DRY_RUN}" -eq 1 ]]; then
    echo "install_deps_ubuntu: dry run; would run: git clone --recurse-submodules --branch ${PINOCCHIO_VERSION} ${PINOCCHIO_REPO_URL} <temp>/pinocchio"
    echo "install_deps_ubuntu: dry run; would run: git -C <temp>/pinocchio submodule update --init --recursive"
    echo "install_deps_ubuntu: dry run; would run: cmake -S <temp>/pinocchio -B <temp>/build ${pinocchio_cmake_flags[*]}"
    echo "install_deps_ubuntu: dry run; would run: cmake --build <temp>/build -j ${jobs}"
    echo "install_deps_ubuntu: dry run; would run: sudo cmake --install <temp>/build"
  fi
}

run_step() {
  local label="$1"
  shift
  echo "install_deps_ubuntu: ${label}"
  if ! "$@"; then
    echo "install_deps_ubuntu: failed: ${label}" >&2
    exit 1
  fi
}

install_pinocchio_from_source() {
  local work_dir
  work_dir="$(mktemp -d "${TMPDIR:-/tmp}/pinocchio-build.XXXXXX")"
  local source_dir="${work_dir}/pinocchio"
  local build_dir="${work_dir}/build"
  local jobs
  jobs="$(pinocchio_build_jobs)"

  echo "install_deps_ubuntu: using temporary Pinocchio work directory: ${work_dir}"
  echo "install_deps_ubuntu: Pinocchio source build jobs: ${jobs}"
  if [[ -n "${PINOCCHIO_BUILD_JOBS}" ]]; then
    echo "install_deps_ubuntu: using PINOCCHIO_BUILD_JOBS override."
  else
    echo "install_deps_ubuntu: Pinocchio source build jobs are memory-capped to avoid OOM; set PINOCCHIO_BUILD_JOBS to override."
  fi
  run_step "cloning Pinocchio ${PINOCCHIO_VERSION}" \
    git clone --recurse-submodules --branch "${PINOCCHIO_VERSION}" "${PINOCCHIO_REPO_URL}" "${source_dir}"
  run_step "fetching Pinocchio submodules recursively" \
    git -C "${source_dir}" submodule update --init --recursive
  run_step "configuring Pinocchio with URDF support and collision/Python/tests disabled" \
    cmake -S "${source_dir}" -B "${build_dir}" "${pinocchio_cmake_flags[@]}"
  run_step "building Pinocchio" \
    cmake --build "${build_dir}" -j "${jobs}"
  run_step "installing Pinocchio to ${PINOCCHIO_INSTALL_PREFIX}" \
    sudo cmake --install "${build_dir}"
  rm -rf "${work_dir}"
}

case "${PROFILE}" in
  hardware-free)
    add_common
    ;;
  kinematics)
    add_common
    ;;
  real-camera)
    add_common
    add_packages "${real_camera_packages[@]}"
    ;;
  real-robot)
    add_common
    echo "install_deps_ubuntu: rbpodo is vendor-provided; install the SDK/package separately and expose it through CMAKE_PREFIX_PATH or RBPODO_ROOT." >&2
    ;;
  all)
    add_common
    add_packages "${kinematics_packages[@]}" "${real_camera_packages[@]}"
    echo "install_deps_ubuntu: rbpodo is vendor-provided; install the SDK/package separately and expose it through CMAKE_PREFIX_PATH or RBPODO_ROOT." >&2
    ;;
  *)
    echo "install_deps_ubuntu: unknown profile '${PROFILE}'" >&2
    usage >&2
    exit 2
    ;;
esac

host_codename="$(host_ubuntu_codename)"
pinocchio_method="none"
robotpkg_dist=""
if [[ "${PINOCCHIO_REQUIRED}" -eq 1 ]]; then
  pinocchio_method="$(pinocchio_install_method "${host_codename}")"
  case "${pinocchio_method}" in
    robotpkg)
      add_packages "${robotpkg_pinocchio_packages[@]}"
      robotpkg_dist="$(robotpkg_codename "${host_codename}")"
      ;;
    source)
      add_packages "${pinocchio_source_build_packages[@]}"
      ;;
  esac
fi

unique_packages=()
declare -A seen=()
for package in "${packages[@]}"; do
  if [[ -z "${seen[${package}]:-}" ]]; then
    unique_packages+=("${package}")
    seen["${package}"]=1
  fi
done

echo "install_deps_ubuntu: profile ${PROFILE}"
if [[ "${PINOCCHIO_REQUIRED}" -eq 1 ]]; then
  echo "install_deps_ubuntu: host Ubuntu codename: ${host_codename:-unknown}"
  echo "install_deps_ubuntu: Pinocchio provider: ${pinocchio_method}"
  if [[ "${pinocchio_method}" == "robotpkg" ]]; then
    configure_robotpkg_repo "${robotpkg_dist}"
  else
    echo "install_deps_ubuntu: robotpkg skipped; source build selected for Pinocchio."
  fi
fi
printf 'install_deps_ubuntu: apt packages:'
printf ' %s' "${unique_packages[@]}"
printf '\n'
if [[ "${PINOCCHIO_REQUIRED}" -eq 1 ]]; then
  if [[ "${pinocchio_method}" == "source" ]]; then
    print_pinocchio_source_plan
  else
    echo "install_deps_ubuntu: Pinocchio is installed through robotpkg as robotpkg-pinocchio under ${PINOCCHIO_INSTALL_PREFIX}."
  fi
  echo "install_deps_ubuntu: Pinocchio is installed under ${PINOCCHIO_INSTALL_PREFIX}."
  echo "install_deps_ubuntu: export CMAKE_PREFIX_PATH=${PINOCCHIO_INSTALL_PREFIX}\${CMAKE_PREFIX_PATH:+:\${CMAKE_PREFIX_PATH}} before local CMake builds if auto-detection is unavailable."
fi

if [[ "${DRY_RUN}" -eq 1 ]]; then
  echo "install_deps_ubuntu: dry run; would run: sudo apt-get update"
  printf 'install_deps_ubuntu: dry run; would run: sudo apt-get install -y --no-install-recommends'
  printf ' %s' "${unique_packages[@]}"
  printf '\n'
  echo "install_deps_ubuntu: dry run; not invoking apt-get, git, cmake, or sudo"
  exit 0
fi

sudo apt-get update
sudo apt-get install -y --no-install-recommends "${unique_packages[@]}"
if [[ "${pinocchio_method}" == "source" ]]; then
  install_pinocchio_from_source
fi

echo "install_deps_ubuntu: install complete; run scripts/check_deps.sh --profile ${PROFILE} to verify."
