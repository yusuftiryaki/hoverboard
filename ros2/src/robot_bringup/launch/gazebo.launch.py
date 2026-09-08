"""The PHYSICS half of the A3 world: Gazebo, the model in it, and the ROS bridge.

This file deliberately does NOT start sim_node or the robot stack, so that the
Gazebo world is started exactly one way whether a human or a test is driving it.
The full three-command sequence is:

    # 1. physics + model + bridge (this file)
    ros2 launch robot_bringup gazebo.launch.py

    # 2. the simulated ESP32, on the physics backend
    ros2 run robot_sim sim_node --ros-args \\
        -p backend:=gazebo -p use_sim_time:=true -p link:=/tmp/fake_esp32_gazebo

    # 3. the real robot stack, on sim time
    ros2 launch robot_bringup robot.launch.py \\
        esp32_port:=/tmp/fake_esp32_gazebo use_sim_time:=true \\
        use_localization:=true use_imu:=false use_gps:=false

⚠️ use_sim_time:=true IS NOT OPTIONAL in steps 2 and 3. Gazebo publishes /clock
and runs on it; a ROS node left on the wall clock computes every dt against a
different clock than the one the physics advanced. Nothing errors — the
velocities and accelerations just come out scaled by the real-time factor. The
argument existed in description/localization/nav2 for a long time with nothing
ever setting it; robot.launch.py now threads it everywhere.

⚠️ Headless by default. There is no display in the dev container, and `gz sim`
without -s brings up a GUI that fails in a way that looks like a physics
problem. Pass gui:=true on a machine that has a screen.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory("robot_sim")
    world_file = os.path.join(sim_share, "gazebo", "empty.sdf")
    model = os.path.join(sim_share, "gazebo", "hoverbot.sdf")
    bridge = os.path.join(sim_share, "gazebo", "bridge.yaml")

    gui = LaunchConfiguration("gui")
    # ⚠️ Both the world NAME (used to address gz services and to build the
    # contact sensor's topic) and the model NAME are baked into bridge.yaml's
    # gz topic paths. test_gazebo_config.py checks they still agree.
    world = LaunchConfiguration("world")
    model_name = LaunchConfiguration("model_name")

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("world", default_value="empty"),
        DeclareLaunchArgument("model_name", default_value="hoverbot"),

        ExecuteProcess(cmd=["gz", "sim", "-s", "-r", world_file], output="screen"),
        ExecuteProcess(cmd=["gz", "sim", "-g"], output="screen",
                       condition=IfCondition(gui)),

        TimerAction(period=3.0, actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=["-world", world, "-file", model, "-name", model_name],
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="ros_gz_bridge",
                arguments=["--ros-args", "-p", f"config_file:={bridge}"],
                # It republishes Gazebo's own stamps, but it also has to answer
                # parameter and lifecycle traffic on the same clock as everyone
                # else once /clock is the system clock.
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
        ]),
    ])
