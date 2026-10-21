# PX4-MIGHTY Integration Workspace (`px4_mighty_ws`)

Dedicated, isolated ROS 2 overlay workspace integrating the **MIGHTY** trajectory planner with **PX4 Autopilot** (Micro XRCE-DDS) and Gazebo Sim Harmonic (Ubuntu 26.04 / ROS 2 Lyrical Luth).

---

## Architecture Overview

```
                      +-----------------------------+
                      |   PX4 Autopilot (SITL/HW)   |
                      +--------------+--------------+
                                     | Micro XRCE-DDS (UDP 8888)
                                     v
                      +-----------------------------+
                      |      MicroXRCEAgent         |
                      +--------------+--------------+
                                     | Native ROS 2 Topics (/fmu/...)
                                     v
+-----------------------------------------------------------------------+
|  px4_mighty_bridge_node (px4_mighty_ws)                               |
|                                                                       |
|  * NED -> ENU Telemetry Downlink:                                     |
|      /fmu/out/vehicle_odometry -> /NX01/state (50+ Hz)                |
|                                -> /NX01/odom                          |
|  * ENU -> NED Setpoint Uplink:                                        |
|      /NX01/goal               -> /fmu/in/trajectory_setpoint (20 Hz)  |
|  * Offboard Heartbeat:                                                |
|      -> /fmu/in/offboard_control_mode (20 Hz)                         |
|  * Arming & Mode Management:                                          |
|      -> /fmu/in/vehicle_command (Arm / Offboard)                      |
|  * Watchdog Safety Fallback:                                          |
|      Staleness > 250 ms -> Position Hold at latest confirmed position |
+-----------------------------------------------------------------------+
         ^                          |
         | /NX01/state              | /NX01/goal
         v                          v
+------------------+       +------------------+
|   mighty_node    | <---> | global_mapper_ros|
+------------------+       +------------------+
                                    ^
                                    | /mid360_PointCloud2
                           +------------------+
                           |  ros_gz_bridge   |
                           +------------------+
                                    ^
                                    | /lidar/points
                           +------------------+
                           | Gazebo Harmonic  |
                           |  x500_lidar_3d   |
                           +------------------+
```

---

## Workspace Structure

- `src/px4_msgs`: Official PX4 message definitions compiled for ROS 2.
- `src/px4_mighty_bridge`:
  - `include/px4_mighty_bridge/`: Header files (`px4_mighty_bridge_node.hpp`, `frame_transforms.hpp`, `watchdog.hpp`, `timestamp_sync.hpp`).
  - `src/`: Implementation files.
  - `test/`: Unit tests and synthetic integration benchmark.
  - `config/bridge_params.yaml`: Node parameters.
  - `launch/`: Unified and modular launch files (`px4_mighty.launch.py`, `ros_gz_bridge.launch.py`).
  - `models/x500_lidar_3d/`: Self-contained Gazebo airframe model with 3D GPU LiDAR sensor.
  - `worlds/`: Gazebo worlds (`mighty_forest.sdf`).
- `setup_env.sh`: Sourcing script configuring ROS 2 base, `mighty_lyrical_ws` underlay, overlay, and compatibility library paths.

---

## Quickstart & Usage

### 1. Build Overlay Workspace
```bash
source ~/code/px4_mighty_ws/setup_env.sh
cd ~/code/px4_mighty_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 2. Run Unit Tests (All 21 Passing)
```bash
colcon test --packages-select px4_mighty_bridge
colcon test-result --verbose
```

### 3. Launch Simulation & Navigation

**Terminal 1 — PX4 SITL & Gazebo (Headless by default, saves CPU/GPU):**
```bash
cd ~/code/px4_mighty_ws
./run_px4_sitl.sh easy_forest
# To open with Gazebo GUI if desired: ./run_px4_sitl.sh easy_forest --gui
```

**Terminal 2 — MIGHTY Planner + Auto-Takeoff + RViz2:**
```bash
source ~/code/px4_mighty_ws/setup_env.sh
ros2 launch px4_mighty_bridge px4_mighty.launch.py launch_rviz:=true
```

The bridge node will automatically:
1. Arm the vehicle.
2. Issue an automatic takeoff command (`2.5m`).
3. Smoothly transition to `OFFBOARD` mode once target altitude is reached.
4. Stream setpoints and follow goals commanded in RViz2 without back-and-forth oscillation.

---

## Constitutional Guarantees
- **Zero Modifications to Canonical Workspaces**: Neither `/home/hasan-fashtouk/mighty` nor `~/code/mighty_lyrical_ws` are modified.
- **Resource Bounds**: CPU $< 5\%$, RSS memory $< 35\text{ MB}$.
- **Safe Fallback**: Automatic position hold upon planner delay $> 1000\text{ ms}$.
