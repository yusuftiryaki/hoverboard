"""Start the A3 Gazebo world, model, ROS-GZ bridge and simulator adapter."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory("robot_sim")
    bringup_share = get_package_share_directory("robot_bringup")
    world = os.path.join(sim_share, "gazebo", "empty.sdf")
    model = os.path.join(sim_share, "gazebo", "hoverbot.sdf")
    bridge = os.path.join(sim_share, "gazebo", "bridge.yaml")
    link = LaunchConfiguration("esp32_link")

    return LaunchDescription([
        DeclareLaunchArgument("esp32_link", default_value="/tmp/fake_esp32_gazebo"),
        ExecuteProcess(cmd=["gz", "sim", "-r", world], output="screen"),
        TimerAction(period=2.0, actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=["-world", "empty", "-file", model, "-name", "hoverbot"],
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=["--ros-args", "-p", f"config_file:={bridge}"],
                output="screen",
            ),
            Node(
                package="robot_sim",
                executable="sim_node",
                parameters=[{
                    "backend": "gazebo",
                    "link": link,
                }],
                output="screen",
            ),
        ]),
    ])