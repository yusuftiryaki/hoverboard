"""Publish the URDF and the fixed transforms between the sensor frames.

Split out because everything needs it: the EKFs, navsat_transform, RViz on the
dev machine and Nav2 later all read frames out of this tree.

Note there is no joint_state_publisher here — hoverboard_bridge publishes the
real wheel angles on /joint_states from the hall sensors. Adding the fake one
would fight it for the same topic.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("robot_bringup")
    default_model = os.path.join(pkg, "urdf", "robot.urdf.xacro")

    model = LaunchConfiguration("model")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("model", default_value=default_model),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                # ParameterValue(..., value_type=str) is required: without it the
                # xacro output gets parsed as YAML and the launch dies with a
                # baffling type error.
                "robot_description": ParameterValue(
                    Command(["xacro ", model]), value_type=str
                ),
                "use_sim_time": use_sim_time,
            }],
        ),
    ])
