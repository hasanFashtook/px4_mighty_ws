#!/usr/bin/env bash
# Environment setup script for px4_mighty_ws overlay workspace

# Source ROS 2 base
if [ -f "/opt/ros/lyrical/setup.bash" ]; then
  source /opt/ros/lyrical/setup.bash
else
  echo "[px4_mighty_ws] Warning: /opt/ros/lyrical/setup.bash not found"
fi

# Source underlay workspace (mighty_lyrical_ws)
if [ -f "/home/hasan-fashtouk/code/mighty_lyrical_ws/install/setup.bash" ]; then
  source /home/hasan-fashtouk/code/mighty_lyrical_ws/install/setup.bash
else
  echo "[px4_mighty_ws] Warning: /home/hasan-fashtouk/code/mighty_lyrical_ws/install/setup.bash not found"
fi

# Source overlay workspace if built
if [ -f "/home/hasan-fashtouk/code/px4_mighty_ws/install/setup.bash" ]; then
  source /home/hasan-fashtouk/code/px4_mighty_ws/install/setup.bash
fi

# Ensure compatibility libraries (such as libtinyxml2 for MicroXRCEAgent) are accessible
export LD_LIBRARY_PATH="/home/hasan-fashtouk/code/lib:${LD_LIBRARY_PATH}"

# Gazebo Sim resource paths for custom models and worlds in px4_mighty_bridge
BRIDGE_SHARE="/home/hasan-fashtouk/code/px4_mighty_ws/src/px4_mighty_bridge"
export GZ_SIM_RESOURCE_PATH="${BRIDGE_SHARE}/models:${BRIDGE_SHARE}/worlds:${GZ_SIM_RESOURCE_PATH}"
