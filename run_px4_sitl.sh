#!/usr/bin/env bash
set -e

# Setup overlay environment and model paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/setup_env.sh"

PX4_DIR="/home/hasan-fashtouk/Documents/PX4-Autopilot"
cd "${PX4_DIR}"

export GIT_SUBMODULES_ARE_EVIL=1

# Optional world argument and optional --gui flag
WORLD="default"
HEADLESS=1

for arg in "$@"; do
    case "$arg" in
        --gui)
            HEADLESS=""
            ;;
        --headless)
            HEADLESS=1
            ;;
        *)
            WORLD="$arg"
            ;;
    esac
done

export HEADLESS

# Ensure no stale PX4 or Gazebo instances are lingering in the background
echo "Cleaning up any previous PX4 / Gazebo instances..."
killall -9 px4 gz-sim-main 2>/dev/null || true
sleep 1

echo "=========================================================="
if [ -n "${HEADLESS}" ]; then
    echo " Starting PX4 SITL Headless (No Gazebo GUI) (World: ${WORLD})"
else
    echo " Starting PX4 SITL with Gazebo GUI (World: ${WORLD})"
fi
echo "=========================================================="

if [ "${WORLD}" != "default" ]; then
    PX4_GZ_WORLD="${WORLD}" make px4_sitl gz_x500_lidar_3d
else
    make px4_sitl gz_x500_lidar_3d
fi

