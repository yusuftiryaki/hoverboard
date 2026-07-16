"""Bench teleop — roadmap step 4, the moment this becomes a drivable robot.

    # on the Pi, with the ESP32 plugged in and THE WHEELS OFF THE GROUND:
    ros2 launch robot_bringup teleop.launch.py

    # dev machine, no hardware at all — drive the simulator:
    ros2 run hoverboard_bridge fake_esp32
    ros2 launch robot_bringup teleop.launch.py esp32_port:=/tmp/fake_esp32

No EKF here, so the bridge publishes odom->base_link itself (publish_tf:=true) —
otherwise RViz has no transform tree and shows you nothing.

⚠️ First real power-up: wheels in the air, hand on the E-stop, and read
docs/bringup-checklist.md first. The speed limit in the firmware (300) is low on
purpose; leave it there until the robot has proven it stops when told to.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("robot_bringup")
    bridge_params = os.path.join(pkg, "config", "hoverboard_bridge.yaml")

    esp32_port = LaunchConfiguration("esp32_port")
    use_keyboard = LaunchConfiguration("use_keyboard")

    return LaunchDescription([
        DeclareLaunchArgument("esp32_port", default_value="/dev/esp32"),
        DeclareLaunchArgument(
            "use_keyboard", default_value="true",
            description="Set false to drive from your own /cmd_vel publisher.",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, "launch", "description.launch.py")
            )
        ),

        Node(
            package="hoverboard_bridge",
            executable="hoverboard_bridge",
            name="hoverboard_bridge",
            output="screen",
            parameters=[bridge_params, {"port": esp32_port, "publish_tf": True}],
        ),

        # teleop_twist_keyboard needs a real tty for its raw key reads, so it has
        # to run in its own xterm — launching it as a plain Node gives you a node
        # that reads nothing and a robot that never moves.
        ExecuteProcess(
            cmd=["xterm", "-e", "ros2", "run", "teleop_twist_keyboard",
                 "teleop_twist_keyboard"],
            condition=IfCondition(use_keyboard),
            output="screen",
        ),
    ])
