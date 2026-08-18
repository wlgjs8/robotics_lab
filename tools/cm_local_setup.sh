#!/usr/bin/env bash
# cm_local_setup.sh — DOCKER-FREE, NATIVE bring-up of submodules/controller-manager
# on THIS host (Ubuntu 22.04 jammy).
#
#   tools/cm_local_setup.sh --check   # report what is missing, change nothing (no sudo)
#   tools/cm_local_setup.sh           # install what is missing + colcon build
#
# WHY THIS FILE EXISTS INSTEAD OF THE SUBMODULE'S OWN local_setup.sh
# ------------------------------------------------------------------
# controller-manager/local_setup.sh is the upstream one-shot bring-up, and it is
# the right script — but its step 1 REFUSES any OS that is not Ubuntu 24.04
# noble, because the controller's deployment target pins ROS 2 Jazzy
# (wiki/decisions/0001). This workstation is 22.04 jammy, so it exits 1 before
# doing anything. The submodule is consumed READ-ONLY (AGENTS.md / CLAUDE.md:
# never edited in-tree), so the gate is bypassed from OUR side, here, rather than
# patched over there.
#
# Building on jammy/Humble is not an improvisation — it is the submodule's own
# documented interim path: docker/Dockerfile builds this exact source tree on
# jammy/Humble, and ADR 0001 records that Humble <-> Jazzy "needed ZERO code
# changes". The submodule's platforms/*/scripts/env.sh already falls back to
# /opt/ros/humble when jazzy is absent (it prints a loud warning and continues).
# This script does natively what that Dockerfile does in a container:
#   ROS 2 Humble + build deps -> rustup (jammy's apt cargo is too old for
#   ik-geo-cpp) -> third-party submodules -> colcon build, repo-as-workspace.
#
# WHAT IS DIFFERENT FROM THE CONTAINER PATH: nothing about the build. The
# container exists to co-locate the controller with manipulation_server on a
# shared Humble DDS bus; running native drops the bind-mounts and the compose
# lifecycle, which is what makes the RT pins (cpu1/cpu2, see
# tools/rt_core_isolation.sh) and SCHED_FIFO privileges behave predictably.
#
# ROOT: only the apt steps use sudo, and only for what is genuinely missing. The
# BUILD runs as you — never run this whole script under sudo, or build/ and
# install/ land in the submodule owned by root.
set -euo pipefail

ROS_DISTRO_WANT=humble
REPO_DEFAULT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/submodules/controller-manager"
CM_REPO="${CM_REPO:-$REPO_DEFAULT}"

MODE=apply
[ "${1:-}" = "--check" ] && MODE=check
[ "${1:-}" = "-h" ] && { sed -n '2,6p' "$0"; exit 0; }

say()  { printf '\n\033[1m== %s\033[0m\n' "$*"; }
die()  { printf 'cm_local_setup: %s\n' "$*" >&2; exit 1; }

[ -f "$CM_REPO/local_setup.sh" ] || die "not a controller-manager checkout: $CM_REPO"
[ "$(id -u)" -ne 0 ] || die "do not run this under sudo — it sudos only the apt steps itself,
                 so that build/ and install/ stay owned by you"

# ---- 1. OS -> ROS distro ---------------------------------------------------
say "1/6 OS check"
. /etc/os-release 2>/dev/null || die "cannot read /etc/os-release"
case "${VERSION_CODENAME:-}" in
    jammy) ;;   # this script's reason for existing
    noble) die "this host is 24.04 noble — use the submodule's OWN script instead, which
                 installs the pinned Jazzy toolchain:  (cd $CM_REPO && ./local_setup.sh)" ;;
    *) die "unsupported OS: ${PRETTY_NAME:-unknown}. Native paths today:
                 22.04 jammy -> ROS 2 humble (this script, interim, per docker/Dockerfile)
                 24.04 noble -> ROS 2 jazzy  (controller-manager/local_setup.sh, the target)" ;;
esac
echo "   ${PRETTY_NAME} -> ROS 2 ${ROS_DISTRO_WANT} (interim; deployment target stays jazzy/24.04)"

# ---- 1b. docker leftovers --------------------------------------------------
# The compose path bind-mounts this repo into a root container, so every artifact
# it wrote (ik-geo's rust target/, tools/__pycache__, and — the one that actually
# bites — platforms/<kind>/active.yaml) is owned by root on the host. A native
# build runs as YOU: cargo cannot refresh a root-owned target/, and env.sh cannot
# re-seed a root-owned active.yaml. Hand them back before anything else.
say "1b/6 docker leftovers"
ROOT_OWNED="$(find "$CM_REPO" -user root -print -quit 2>/dev/null || true)"
if [ -n "$ROOT_OWNED" ]; then
    n="$(find "$CM_REPO" -user root 2>/dev/null | wc -l)"
    echo "   ${n} root-owned path(s) left by the container build (e.g. ${ROOT_OWNED#$CM_REPO/})"
    if [ "$MODE" = check ]; then
        echo "   would run: sudo chown -R $(id -un):$(id -gn) $CM_REPO"
    else
        echo "   chown -R $(id -un):$(id -gn) (sudo)"
        sudo chown -R "$(id -un):$(id -gn)" "$CM_REPO"
    fi
else
    echo "   none — the tree is entirely yours"
fi

# ---- 2. dependencies -------------------------------------------------------
say "2/6 dependencies"
ROS_PKGS=(ros-${ROS_DISTRO_WANT}-ros-base
          ros-${ROS_DISTRO_WANT}-rosidl-default-generators
          ros-${ROS_DISTRO_WANT}-rosidl-default-runtime
          ros-${ROS_DISTRO_WANT}-rmw-cyclonedds-cpp)
# rmw-cyclonedds is not for us: env.sh pins fastrtps for the controller alone.
# It is here because the ONE peer that shares this bus, PLAIF's manipulation_server,
# segfaults creating a Fast DDS participant (its ADR-0017 / docker/Dockerfile), and
# ROS 2 cannot mix RMWs across a bus.

# FUNCTIONAL probes, not dpkg bookkeeping — a dependency satisfied any other way
# must not drag apt onto a machine that already builds.
MISSING=()
command -v cmake      >/dev/null || MISSING+=(build-essential cmake)
command -v git        >/dev/null || MISSING+=(git)
command -v pkg-config >/dev/null || MISSING+=(pkg-config)
command -v colcon     >/dev/null || MISSING+=(python3-colcon-common-extensions)
[ -d /usr/include/yaml-cpp ] || MISSING+=(libyaml-cpp-dev)
[ -d /usr/include/eigen3 ]   || MISSING+=(libeigen3-dev)
python3 -c 'import yaml' 2>/dev/null || MISSING+=(python3-yaml)
NEED_ROS=0
[ -f "/opt/ros/${ROS_DISTRO_WANT}/setup.bash" ] || NEED_ROS=1

# cargo: jammy's apt rustc is 1.66 and ik-geo-cpp's rust-wrapper needs newer, so
# this comes from rustup (user-owned, NO sudo) — never from apt on this distro.
NEED_RUST=0
IK_GEO_SO="$CM_REPO/src/arm/kinematics/third-party/ik-geo-cpp/rust-wrapper/target/release/libik_geo.so"
if ! command -v cargo >/dev/null && [ ! -f "$HOME/.cargo/bin/cargo" ]; then
    # a prebuilt .so only counts if its target/ is OURS to refresh — the container
    # left a root-owned one, and cargo re-runs (and writes fingerprints) regardless.
    if [ ! -f "$IK_GEO_SO" ] || [ ! -w "$(dirname "$IK_GEO_SO")" ]; then
        NEED_RUST=1
    fi
fi

echo "   ros 2 ${ROS_DISTRO_WANT}: $([ "$NEED_ROS" = 1 ] && echo 'MISSING (apt, needs sudo)' || echo present)"
echo "   apt packages : $([ "${#MISSING[@]}" -gt 0 ] && echo "MISSING: ${MISSING[*]}" || echo 'all present')"
echo "   rust (cargo) : $([ "$NEED_RUST" = 1 ] && echo 'MISSING (rustup, no sudo)' || echo present)"

if [ "$MODE" = check ]; then
    say "--check only — nothing installed, nothing built"
    exit 0
fi

if [ "$NEED_ROS" = 1 ]; then
    echo "   installing ROS 2 ${ROS_DISTRO_WANT} (sudo + network)"
    sudo apt-get update -y
    sudo apt-get install -y --no-install-recommends ca-certificates curl gnupg lsb-release
    if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
        sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
             -o /usr/share/keyrings/ros-archive-keyring.gpg
    fi
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${VERSION_CODENAME} main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
    sudo apt-get update -y
    MISSING+=("${ROS_PKGS[@]}")
fi
if [ "${#MISSING[@]}" -gt 0 ]; then
    echo "   apt install: ${MISSING[*]}"
    sudo apt-get install -y "${MISSING[@]}"
else
    echo "   all apt dependencies present"
fi
command -v colcon >/dev/null || die "colcon still missing after apt — install manually and re-run"

if [ "$NEED_RUST" = 1 ]; then
    echo "   installing rustup (user-local, no sudo)"
    curl --proto '=https' --tlsv1.2 -fsSL https://sh.rustup.rs \
        | sh -s -- -y --no-modify-path --default-toolchain stable --profile minimal
fi
[ -d "$HOME/.cargo/bin" ] && export PATH="$HOME/.cargo/bin:$PATH"

# ---- 4. third-party submodules --------------------------------------------
# ik-geo-cpp / ruckig / rbpodo are pinned sources; a clone that missed
# --recurse-submodules fails late with a CMake error instead of here.
say "4/6 third-party submodules"
if git -C "$CM_REPO" submodule status | grep -q '^-'; then
    git -C "$CM_REPO" submodule update --init --recursive
else
    echo "   already initialized"
fi
git -C "$CM_REPO" submodule status | sed 's/^/   /'

# ---- 5. build (repo-as-workspace) -----------------------------------------
say "5/6 build"
# ROS setup.bash reads variables it never sets — it is not `set -u`-clean.
set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO_WANT}/setup.bash"
set -u
# --packages-up-to is PART of the command, not an optimization: a bare colcon
# build also discovers mo_robot_descriptions (non-ROS CMake) and aborts.
( cd "$CM_REPO" && colcon build --packages-up-to controller_manager \
      --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 )

# ---- 6. next steps ---------------------------------------------------------
say "6/6 done — native (no docker) run"
cat <<EOF
   The submodule is now its own colcon workspace: ${CM_REPO}/install
   env.sh resolves it automatically and falls back to /opt/ros/${ROS_DISTRO_WANT}
   (it prints a "JAZZY NOT FOUND" warning — expected on this host, not an error).

     SILS (no hardware):
       cd ${CM_REPO}
       platforms/monkey/scripts/start.sh --sils
       platforms/monkey/scripts/start.sh --sils --cockpit    # + web UI :8770
       platforms/monkey/scripts/start.sh --stop

     REAL hardware:
       source platforms/monkey/scripts/env.sh                # seeds platforms/monkey/active.yaml
       \$EDITOR platforms/monkey/active.yaml                  # arms.left.ip / arms.right.ip, serials
       platforms/monkey/scripts/start.sh

   RT host prep lives on OUR side, not the submodule's: tools/rt_core_isolation.sh
   (the submodule's platforms/*/scripts/cpu-isolate.sh writes an overlapping but
   differently-derived GRUB line — do not run both).
EOF
