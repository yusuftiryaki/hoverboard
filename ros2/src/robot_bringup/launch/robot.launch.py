"""Top-level robot bringup — this is what robot.service starts on the Pi
(docs/deployment.md step 8).

    ros2 launch robot_bringup robot.launch.py

Defaults are deliberately the LEAST hardware: the drivetrain and the URDF, and
nothing else. Every sensor and the localization stack are opt-in, because a
launch file that dies looking for /dev/gps when the GPS is still in its bag is a
launch file people stop trusting.

As hardware lands, turn things on:

    # bench, ESP32 only (roadmap step 4)
    ros2 launch robot_bringup robot.launch.py

    # + IMU-based localization, still indoors (robot still at startup!)
    ros2 launch robot_bringup robot.launch.py use_localization:=true use_imu:=true

    # full outdoor stack (roadmap step 5-6)
    ros2 launch robot_bringup robot.launch.py \\
        use_localization:=true use_imu:=true use_gps:=true use_camera:=true

    # dev machine, no hardware at all — both the ESP32 and the IMU simulated
    ros2 run hoverboard_bridge fake_esp32
    ros2 launch robot_bringup robot.launch.py esp32_port:=/tmp/fake_esp32 \\
        use_localization:=true use_imu:=true fake_imu:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("robot_bringup")
    launch_dir = os.path.join(pkg, "launch")
    bridge_params = os.path.join(pkg, "config", "hoverboard_bridge.yaml")

    use_localization = LaunchConfiguration("use_localization")
    use_gps = LaunchConfiguration("use_gps")
    use_imu = LaunchConfiguration("use_imu")
    use_camera = LaunchConfiguration("use_camera")
    esp32_port = LaunchConfiguration("esp32_port")
    fake_imu = LaunchConfiguration("fake_imu")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_localization", default_value="false",
            description="Run the dual EKF. Needs a working /imu/data to be worth much.",
        ),
        DeclareLaunchArgument(
            "use_gps", default_value="false",
            description="Run the GPS driver, ekf_global and navsat_transform.",
        ),
        DeclareLaunchArgument(
            "use_imu", default_value="false",
            description="Run the MPU6050. The robot must be still while it calibrates.",
        ),
        DeclareLaunchArgument("use_camera", default_value="false"),
        DeclareLaunchArgument(
            "fake_imu", default_value="false",
            description="Simulate the IMU — dev machine only.",
        ),
        DeclareLaunchArgument(
            "esp32_port", default_value="/dev/esp32",
            description="Point at /tmp/fake_esp32 to drive the simulator instead.",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "description.launch.py"))
        ),

        Node(
            package="hoverboard_bridge",
            executable="hoverboard_bridge",
            name="hoverboard_bridge",
            output="screen",
            parameters=[bridge_params, {"port": esp32_port}],
            # If the serial port vanishes (ESP32 unplugged, USB brownout) the
            # node dies on purpose. Respawning is right: the ESP32's own watchdog
            # has already stopped the motors, and coming back up is what we want.
            respawn=True,
            respawn_delay=2.0,
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "sensors.launch.py")),
            launch_arguments={
                "use_gps": use_gps,
                "use_imu": use_imu,
                "use_camera": use_camera,
                "fake_imu": fake_imu,
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "localization.launch.py")),
            condition=IfCondition(use_localization),
            launch_arguments={"use_gps": use_gps}.items(),
        ),
    ])
