#!/usr/bin/env python3
"""Bring up the PX4 <-> MIGHTY companion stack.

PX4 SITL and Gazebo are started separately (see the workspace README and
``specs/002-px4-mighty-integration/quickstart.md``) so that the simulator can
live in its own terminal and use its own ``ROS_DOMAIN_ID``/``PX4_UXRCE_DDS_PORT``.

This launch file starts:

* Micro XRCE-DDS Agent (unless ``launch_agent:=false``)
* ``px4_mighty_bridge_node``
* ``ros_gz_bridge`` translating ``/lidar/points`` into PointCloud2
* static transforms completing the ``map -> NX01/NX01_livox`` chain
* ``global_mapper_ros`` (unless ``launch_mapping:=false``)
* ``mighty`` (unless ``launch_planner:=false``)
* optional RViz2 (``launch_rviz:=true``)
"""

import os
import re

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _sanitize(value):
    """Convert scientific-notation strings that rclcpp cannot infer."""
    if isinstance(value, dict):
        return {k: _sanitize(v) for k, v in value.items()}
    if isinstance(value, list):
        return [_sanitize(v) for v in value]
    if isinstance(value, str):
        if re.match(r"^[+-]?\d+([.]\d*)?[eE][+-]?\d+$", value):
            try:
                return float(value)
            except ValueError:
                pass
    return value


def _load_mighty_params(namespace, map_frame_id):
    mighty_share = get_package_share_directory("mighty")
    with open(os.path.join(mighty_share, "config", "mighty.yaml"), "r") as stream:
        params = yaml.safe_load(stream)["mighty_node"]["ros__parameters"]

    params = _sanitize(params)
    # Gazebo Sim provides the state through the PX4 bridge; there is no fake_sim
    # node and no Vicon converter in this stack.
    params.update(
        {
            # --- Simulation & Framework Core ---
            "sim_env": "gazebo",
            "use_hardware": False,
            "map_frame_id": map_frame_id,
            "use_sim_time": True,
            "num_agents": 1,
            "visual_level": min(int(params.get("visual_level", 2)), 1),
            "default_goal_z": 2.0,
            "z_min": 1.2,  # Prevent dipping near ground, tree roots, or floor clutter

            # --- Physical Footprint & Convex Corridor Safety (SFC) ---
            # Holybro x500: 500mm wheelbase + 10" props = ~760mm tip-to-tip span.
            # drone_radius = bbox[0] / 2.0 = 0.425m inflation for decomp polyhedra.
            "drone_bbox": [0.85, 0.85, 0.6],
            "sfc_use_unknown_as_obstacle": True,  # Do not carve corridors into unmapped / occluded voxels

            # --- Global Planning Obstacle Clearance (HGP / A*) ---
            "inflation_hgp": 0.65,  # Inflate static obstacles by 0.65m during A* search
            "los_cells": 5,  # 5 cells * 0.15m = 0.75m clearance for LOS shortcutting
            "static_heat_Hmax": 80.0,  # Stronger repulsive heat field around obstacles
            "static_heat_rmax_m": 2.5,  # Heat repulsion extends 2.5m from obstacle boundary

            # --- Local Trajectory Optimization (L-BFGS) ---
            "planner_Co": 0.25,  # 0.25m safety clearance inside corridor walls (prevents cutting corners)
            "stat_weight": 5.0e+3,  # High penalty weight for approaching corridor boundaries
            "jerk_weight": 2.0e-1,  # Extra smoothness penalty to prevent sudden aggressive jerks

            # --- Dynamic Limits & Autopilot Tracking Margin ---
            "mass": 2.0,  # Realistic Holybro x500 mass with battery & sensor payload
            "v_max": 6.0,  # Controlled cruise speed to guarantee autopilot tracking
            "a_max": 8.0,  # Gentle acceleration to eliminate overshoot at turns
            "j_max": 10.0,  # Controlled jerk
            "tilt_max_rad": 0.436,  # 25 deg max tilt (keeps camera oriented forward, prevents altitude sag)
            "f_min": 4.0,  # [N] Minimum thrust limit
            "f_max": 40.0,  # [N] Maximum thrust limit
            "dyn_constr_vel_weight": 5.0e+3,  # Strict dynamic limits enforcement
            "dyn_constr_acc_weight": 5.0e+3,
            "dyn_constr_jerk_weight": 5.0e+3,
        }
    )
    # rclpy cannot infer the type of an empty list in a dict parameter.
    return {k: v for k, v in params.items() if not (isinstance(v, list) and not v)}


def _launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() in ("true", "1")
    map_frame_id = LaunchConfiguration("map_frame_id").perform(context)
    ros_domain_id = LaunchConfiguration("ros_domain_id").perform(context)
    gz_partition = LaunchConfiguration("gz_partition").perform(context)
    agent_port = LaunchConfiguration("agent_port").perform(context)
    agent_lib_path = LaunchConfiguration("agent_lib_path").perform(context)

    actions = [
        SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id),
        SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4"),
    ]
    if gz_partition:
        actions.append(SetEnvironmentVariable("GZ_PARTITION", gz_partition))

    if LaunchConfiguration("launch_agent").perform(context).lower() in ("true", "1"):
        env = {}
        if agent_lib_path:
            env["LD_LIBRARY_PATH"] = (
                agent_lib_path + ":" + os.environ.get("LD_LIBRARY_PATH", "")
            )
        actions.append(
            ExecuteProcess(
                cmd=["MicroXRCEAgent", "udp4", "-p", agent_port],
                additional_env=env,
                output="screen",
            )
        )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("px4_mighty_bridge"),
                        "launch",
                        "ros_gz_bridge.launch.py",
                    ]
                )
            )
        )
    )

    actions.append(
        Node(
            package="px4_mighty_bridge",
            executable="px4_mighty_bridge_node",
            name="px4_mighty_bridge_node",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "use_sim_time": use_sim_time,
                    "drone_namespace": namespace,
                    "world_frame_id": map_frame_id,
                    "body_frame_id": f"{namespace}/base_link",
                    "livox_frame_id": f"{namespace}/{namespace}_livox",
                    "auto_takeoff": LaunchConfiguration("auto_takeoff"),
                    "takeoff_altitude": LaunchConfiguration("takeoff_altitude"),
                    "enable_acceleration_feedforward": LaunchConfiguration("enable_acceleration_feedforward"),
                    "yaw_mode": LaunchConfiguration("yaw_mode"),
                },
            ],
        )
    )

    # The bridge publishes map -> <ns>/base_link at 20 Hz.  These static
    # transforms carry the LiDAR frame used by MIGHTY and global_mapper_ros,
    # and expose the raw Gazebo frame for RViz.
    actions.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_base_link_to_livox",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=[
                "--x", "0.12", "--y", "0", "--z", "0.26",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", f"{namespace}/base_link",
                "--child-frame-id", f"{namespace}/{namespace}_livox",
            ],
        )
    )
    actions.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_livox_to_gz_frame",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=[
                "--frame-id", f"{namespace}/{namespace}_livox",
                "--child-frame-id", "livox_frame",
            ],
        )
    )

    global_mapper_config = PathJoinSubstitution(
        [
            FindPackageShare("px4_mighty_bridge"),
            "config",
            "global_mapper.yaml",
        ]
    )
    actions.append(
        Node(
            package="global_mapper_ros",
            executable="global_mapper_node",
            name="global_mapper_ros",
            namespace=namespace,
            output="screen",
            condition=IfCondition(LaunchConfiguration("launch_mapping")),
            parameters=[
                global_mapper_config,
                {"use_sim_time": use_sim_time},
            ],
            remappings=[
                ("depth_pointcloud_topic", "mid360_PointCloud2"),
                ("pose_topic", "state"),
                ("goal_topic", "/move_base_simple/goal"),
                ("odom_topic", "odom"),
                ("occupancy_grid_topic", "occupancy_grid"),
                ("unknown_grid_topic", "unknown_grid"),
                ("free_grid_topic", "free_grid"),
                ("explored_volume", "explored_volume"),
                ("frontier_grid_topic", "frontier_grid"),
                ("distance_grid_topic", "distance_grid"),
                ("cost_grid_topic", "cost_grid"),
                ("path_topic", "path"),
                ("sparse_path_topic", "sparse_path"),
            ],
            arguments=["--ros-args", "--log-level", "error"],
        )
    )

    actions.append(
        Node(
            package="mighty",
            executable="mighty",
            name="mighty_node",
            namespace=namespace,
            output="screen",
            emulate_tty=True,
            parameters=[_load_mighty_params(namespace, map_frame_id)],
            remappings=[("lidar_cloud_in", "/mid360_PointCloud2")],
            arguments=["--ros-args", "--log-level", "info", "--log-level", "mighty:=error"],
            condition=IfCondition(LaunchConfiguration("launch_planner")),
        )
    )

    actions.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=[
                "-d",
                os.path.join(
                    get_package_share_directory("px4_mighty_bridge"), "rviz", "px4_mighty.rviz"
                ),
            ],
            condition=IfCondition(LaunchConfiguration("launch_rviz")),
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("namespace", default_value="NX01"),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("px4_mighty_bridge"),
                        "config",
                        "bridge_params.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "map_frame_id",
                default_value="map",
                description="World/map frame shared by the bridge and MIGHTY",
            ),
            DeclareLaunchArgument(
                "ros_domain_id",
                default_value=os.environ.get("ROS_DOMAIN_ID", "0"),
                description="DDS domain used by all ROS 2 nodes in this stack",
            ),
            DeclareLaunchArgument(
                "gz_partition",
                default_value="",
                description="Gazebo Transport partition used by PX4/Gazebo",
            ),
            DeclareLaunchArgument(
                "agent_port",
                default_value="8888",
                description="UDP port of MicroXRCEAgent (must match PX4_UXRCE_DDS_PORT)",
            ),
            DeclareLaunchArgument(
                "agent_lib_path",
                default_value="/home/hasan-fashtouk/code/lib",
                description=(
                    "Directory containing the libtinyxml2.so.10 compatibility "
                    "library required by MicroXRCEAgent (set empty if the "
                    "system provides it)"
                ),
            ),
            DeclareLaunchArgument("launch_agent", default_value="true"),
            DeclareLaunchArgument("launch_mapping", default_value="true"),
            DeclareLaunchArgument("launch_planner", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument(
                "auto_takeoff",
                default_value="true",
                description="Automatically arm, takeoff, and transition to offboard mode",
            ),
            DeclareLaunchArgument(
                "takeoff_altitude",
                default_value="2.5",
                description="Target altitude in meters for automatic takeoff",
            ),
            DeclareLaunchArgument(
                "enable_acceleration_feedforward",
                default_value="true",
                description="Enable acceleration feedforward in offboard control mode (Position + Velocity + Acceleration)",
            ),
            DeclareLaunchArgument(
                "yaw_mode",
                default_value="none",
                description="Yaw control mode for PX4 setpoint: 'none' (do not send yaw, keep heading fixed), 'zero' (fixed yaw = 0.0), or 'follow' (follow MIGHTY's planned yaw)",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
