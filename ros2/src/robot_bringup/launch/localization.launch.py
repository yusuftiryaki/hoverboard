"""Dual EKF + navsat_transform (roadmap step 5).

    wheel odom (/odom) ─┬─► ekf_local  ──► /odometry/filtered/local   [odom->base_link]
    IMU (/imu/data)  ───┤
                        └─► ekf_global ──► /odometry/filtered/global  [map->odom]
                              ▲    │
              /odometry/gps ──┘    └──► navsat_transform ◄── /gps/fix, /imu/data
                                              │
                                              └──► /gps/filtered (for debugging)

Both EKFs publish on `odometry/filtered` by default, so they MUST be remapped
apart — two nodes on one topic is a silent, miserable bug where the estimate
appears to teleport.

Set use_gps:=false to run the local half alone. That is the honest configuration
until the GPS is mounted: ekf_global with no GPS input is just a slower copy of
ekf_local, and map->odom would be identity while pretending to mean something.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("robot_bringup")
    ekf_params = os.path.join(pkg, "config", "ekf.yaml")

    use_gps = LaunchConfiguration("use_gps")
    use_sim_time = LaunchConfiguration("use_sim_time")

    common = [ekf_params, {"use_sim_time": use_sim_time}]

    return LaunchDescription([
        DeclareLaunchArgument("use_gps", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        # odom -> base_link. Smooth, continuous, no absolute references.
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_local",
            output="screen",
            parameters=common,
            remappings=[
                ("odometry/filtered", "odometry/filtered/local"),
            ],
        ),

        # map -> odom. Absorbs the GPS's jumps so base_link never jumps.
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_global",
            output="screen",
            condition=IfCondition(use_gps),
            parameters=common,
            remappings=[
                ("odometry/filtered", "odometry/filtered/global"),
            ],
        ),

        # lat/lon -> the map frame's cartesian coordinates.
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            output="screen",
            condition=IfCondition(use_gps),
            parameters=common,
            remappings=[
                ("imu/data", "imu/data"),
                ("gps/fix", "gps/fix"),
                # Feed it the GLOBAL estimate: closing this loop with the local
                # one would leave the GPS transform blind to its own corrections.
                ("odometry/filtered", "odometry/filtered/global"),
                ("odometry/gps", "odometry/gps"),
                ("gps/filtered", "gps/filtered"),
            ],
        ),
    ])
