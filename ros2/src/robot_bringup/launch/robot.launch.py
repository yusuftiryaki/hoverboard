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

    # full outdoor stack with GPS waypoint following (roadmap A2 / B7)
    ros2 launch robot_bringup robot.launch.py \\
        use_localization:=true use_imu:=true use_gps:=true use_nav2:=true

    # dev machine, no hardware at all — both the ESP32 and the IMU simulated
    ros2 run hoverboard_bridge fake_esp32
    ros2 launch robot_bringup robot.launch.py esp32_port:=/tmp/fake_esp32 \\
        use_localization:=true use_imu:=true fake_imu:=true

    # the simulated world, with Nav2 on top (robot_sim publishes /imu/data itself)
    ros2 run robot_sim sim_node
    ros2 launch robot_bringup robot.launch.py esp32_port:=/tmp/fake_esp32 \\
        use_localization:=true use_gps:=true use_imu:=false use_nav2:=true
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
    battery_params = os.path.join(pkg, "config", "battery_manager.yaml")

    use_localization = LaunchConfiguration("use_localization")
    use_gps = LaunchConfiguration("use_gps")
    use_imu = LaunchConfiguration("use_imu")
    use_mag = LaunchConfiguration("use_mag")
    use_imu_filter = LaunchConfiguration("use_imu_filter")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_camera = LaunchConfiguration("use_camera")
    esp32_port = LaunchConfiguration("esp32_port")
    fake_imu = LaunchConfiguration("fake_imu")
    fake_mag = LaunchConfiguration("fake_mag")
    fake_battery = LaunchConfiguration("fake_battery")
    use_sim_time = LaunchConfiguration("use_sim_time")

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
        DeclareLaunchArgument(
            "use_mag", default_value="false",
            description="Run the QMC5883L. Calibrate it first — docs/mag-calibration.md.",
        ),
        DeclareLaunchArgument(
            "use_imu_filter", default_value="false",
            description="Fuse IMU + mag into /imu/data. REQUIRED with use_gps.",
        ),
        DeclareLaunchArgument("use_camera", default_value="false"),
        DeclareLaunchArgument(
            "fake_imu", default_value="false",
            description="Simulate the IMU — dev machine only.",
        ),
        DeclareLaunchArgument(
            "fake_mag", default_value="false",
            description="Simulate the magnetometer — dev machine only.",
        ),
        DeclareLaunchArgument(
            "fake_battery", default_value="false",
            description="Simulate the INA228 — dev machine only.",
        ),
        DeclareLaunchArgument(
            "use_nav2", default_value="false",
            description="Run Nav2. Requires use_localization:=true AND use_gps:=true — "
                        "Nav2 plans in `map`, which only exists once navsat_transform runs.",
        ),
        DeclareLaunchArgument(
            "esp32_port", default_value="/dev/esp32",
            description="Point at /tmp/fake_esp32 to drive the simulator instead.",
        ),
        # ⚠️ THE WHOLE STACK OR NONE OF IT. description/localization/nav2 each
        # declared their own use_sim_time and nothing ever set it, so with
        # Gazebo running the ROS half timestamped everything off the wall clock
        # while the physics ran on sim time. Nothing errors: the EKF just
        # integrates against dt values that belong to a different clock, and
        # every velocity and acceleration it derives is wrong by the real-time
        # factor. Declared here and threaded into every node below.
        #
        # ⚠️⚠️ NEVER TRUE ON THE ROBOT. With no /clock publisher a node on sim
        # time sits at t = 0 forever, so hoverboard_bridge's cmd_timeout
        # (age = now - last_cmd_time) is permanently zero and NEVER trips — it
        # would keep resending the last /cmd_vel after the publisher died. The
        # ESP32's own watchdog cannot save that either: the bridge is still
        # talking. Default false, and it belongs to the Gazebo world only.
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Take time from /clock. Required with the Gazebo backend "
                        "(gazebo.launch.py). NEVER on the real robot: with no "
                        "/clock the clock never advances and the bridge's "
                        "cmd_timeout deadman never fires.",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "description.launch.py")),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),

        Node(
            package="hoverboard_bridge",
            executable="hoverboard_bridge",
            name="hoverboard_bridge",
            output="screen",
            parameters=[bridge_params, {"port": esp32_port,
                                        "use_sim_time": use_sim_time}],
            remappings=[("battery", "battery_raw")],
            # If the serial port vanishes (ESP32 unplugged, USB brownout) the
            # node dies on purpose. Respawning is right: the ESP32's own watchdog
            # has already stopped the motors, and coming back up is what we want.
            respawn=True,
            respawn_delay=2.0,
        ),

        Node(
            package="battery_manager",
            executable="battery_monitor",
            name="battery_monitor",
            output="screen",
            parameters=[battery_params, {"use_fake_bus": fake_battery,
                                         "use_sim_time": use_sim_time}],
            respawn=True,
            respawn_delay=2.0,
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "sensors.launch.py")),
            launch_arguments={
                "use_gps": use_gps,
                "use_imu": use_imu,
                "use_mag": use_mag,
                "use_imu_filter": use_imu_filter,
                "use_camera": use_camera,
                "fake_imu": fake_imu,
                "fake_mag": fake_mag,
                "use_sim_time": use_sim_time,
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "localization.launch.py")),
            condition=IfCondition(use_localization),
            launch_arguments={"use_gps": use_gps,
                              "use_sim_time": use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "nav2.launch.py")),
            condition=IfCondition(use_nav2),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),
    ])
