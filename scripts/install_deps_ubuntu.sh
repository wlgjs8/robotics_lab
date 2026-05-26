#!/usr/bin/env bash
set -euo pipefail

PROFILE="hardware-free"
DRY_RUN=0
WITH_PYTHON_DEV=0
ROBOTPKG_DIST="${ROBOTPKG_DIST:-}"
ROBOTPKG_LIST="/etc/apt/sources.list.d/robotpkg.list"
ROBOTPKG_KEYRING="/etc/apt/keyrings/robotpkg.asc"
ROBOTPKG_URL="http://robotpkg.openrobots.org/packages/debian/pub"
ROBOTPKG_KEY_URL="http://robotpkg.openrobots.org/packages/debian/robotpkg.asc"

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

This script installs dependencies only. It does not enable RB_ALLOW_REAL_ROBOT,
RB_ALLOW_REAL_MOTION, RB_ALLOW_REAL_CARTESIAN, RealSense capture, or robot motion.
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
  robotpkg-pinocchio
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
add_packages() {
  local package
  for package in "$@"; do
    packages+=("${package}")
  done
}

add_common() {
  add_packages "${common_packages[@]}"
  if [[ "${WITH_PYTHON_DEV}" -eq 1 ]]; then
    add_packages "${python_dev_packages[@]}"
  fi
}

ubuntu_codename() {
  if [[ -n "${ROBOTPKG_DIST}" ]]; then
    printf '%s\n' "${ROBOTPKG_DIST}"
    return
  fi
  if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    . /etc/os-release
    printf '%s\n' "${VERSION_CODENAME:-${UBUNTU_CODENAME:-}}"
    return
  fi
  printf '\n'
}

configure_robotpkg_repo() {
  local codename="$1"
  if [[ -z "${codename}" ]]; then
    echo "install_deps_ubuntu: cannot determine Ubuntu codename; set ROBOTPKG_DIST=jammy to use robotpkg." >&2
    exit 2
  fi
  if [[ "${codename}" != "jammy" && -z "${ROBOTPKG_DIST:-}" ]]; then
    echo "install_deps_ubuntu: robotpkg auto-setup is pinned to jammy to match scripts/docker/rb_servo_server.hardware_free.Dockerfile." >&2
    echo "install_deps_ubuntu: set ROBOTPKG_DIST=${codename} explicitly if robotpkg supports your distribution." >&2
    exit 2
  fi

  echo "install_deps_ubuntu: configuring robotpkg (${codename}) for robotpkg-pinocchio"
  if [[ "${DRY_RUN}" -eq 1 ]]; then
    echo "install_deps_ubuntu: dry run; would install ca-certificates curl gnupg, write ${ROBOTPKG_LIST}, and import ${ROBOTPKG_KEY_URL}"
    return
  fi

  sudo apt-get update
  sudo apt-get install -y --no-install-recommends ca-certificates curl gnupg
  sudo install -d -m 0755 /etc/apt/keyrings
  curl -fsSL "${ROBOTPKG_KEY_URL}" | sudo tee "${ROBOTPKG_KEYRING}" >/dev/null
  echo "deb [arch=amd64 signed-by=${ROBOTPKG_KEYRING}] ${ROBOTPKG_URL} ${codename} robotpkg" | \
    sudo tee "${ROBOTPKG_LIST}" >/dev/null
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

unique_packages=()
declare -A seen=()
for package in "${packages[@]}"; do
  if [[ -z "${seen[${package}]:-}" ]]; then
    unique_packages+=("${package}")
    seen["${package}"]=1
  fi
done

echo "install_deps_ubuntu: profile ${PROFILE}"
robotpkg_dist="$(ubuntu_codename)"
configure_robotpkg_repo "${robotpkg_dist}"
printf 'install_deps_ubuntu: apt packages:'
printf ' %s' "${unique_packages[@]}"
printf '\n'
echo "install_deps_ubuntu: Pinocchio is installed through robotpkg as robotpkg-pinocchio under /opt/openrobots."
echo "install_deps_ubuntu: export CMAKE_PREFIX_PATH=/opt/openrobots\${CMAKE_PREFIX_PATH:+:\${CMAKE_PREFIX_PATH}} before local CMake builds if auto-detection is unavailable."

if [[ "${DRY_RUN}" -eq 1 ]]; then
  echo "install_deps_ubuntu: dry run; not invoking apt-get"
  exit 0
fi

sudo apt-get update
sudo apt-get install -y --no-install-recommends "${unique_packages[@]}"

echo "install_deps_ubuntu: install complete; run scripts/check_deps.sh --profile ${PROFILE} to verify."
