"""twist_mux: the one publisher of /cmd_vel. Included by robot.launch.py.

    teleop ─► /cmd_vel_teleop    (priority 100) ─┐
    Nav2 ─► nav_cmd_gate ─► /cmd_vel_nav_gated (10) ─┴─► twist_mux ─► /cmd_vel

Its own file so test_twist_mux.py can start exactly this wiring — the remap
and the parameters together — without a bridge or a world behind it. The
priorities and their reasoning live in config/twist_mux.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("robot_bringup")
    params = os.path.join(pkg, "config", "twist_mux.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="twist_mux",
            executable="twist_mux",
            name="twist_mux",
            output="screen",
            parameters=[params, {"use_sim_time": use_sim_time}],
            remappings=[("cmd_vel_out", "cmd_vel")],
            # Same reasoning as the bridge: if it dies nothing reaches /cmd_vel,
            # the bridge's deadman stops the wheels, and coming back is right.
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
