"""Pi-side sensor drivers: GPS, IMU, camera (docs/wiring-map.md section 3d).

Everything is behind a launch argument and defaults OFF, so the stack still
comes up while the hardware is still in a box. Turn each one on as it lands.

⚠️ IMU: there is no MPU6050 driver in the Jazzy apt repo. That node is NOT
wired up here yet — see the note below. Until it exists, ekf_local runs on wheel
odometry alone, which drifts in yaw fast. This is the next real gap.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_gps = LaunchConfiguration("use_gps")
    use_camera = LaunchConfiguration("use_camera")
    gps_port = LaunchConfiguration("gps_port")

    return LaunchDescription([
        DeclareLaunchArgument("use_gps", default_value="false"),
        DeclareLaunchArgument("use_camera", default_value="false"),
        # udev symlink, not /dev/ttyUSB1 — it swaps with the ESP32 across boots
        # (docs/deployment.md step 4).
        DeclareLaunchArgument("gps_port", default_value="/dev/gps"),

        # NEO-6M over a USB-TTL adapter. Publishes /gps/fix (NavSatFix), which
        # navsat_transform consumes.
        # ⚠️ ros-jazzy-nmea-navsat-driver may not exist in the Jazzy apt repo
        # (docs/handoff.md, known blockers). If the image build fails on it,
        # build nmea_navsat_driver from source into this workspace instead.
        Node(
            package="nmea_navsat_driver",
            executable="nmea_serial_driver",
            name="gps_driver",
            output="screen",
            condition=IfCondition(use_gps),
            parameters=[{
                "port": gps_port,
                "baud": 9600,          # NEO-6M factory default
                "frame_id": "gps_link",
                "useRMC": False,       # GGA carries the fix quality we want
            }],
            remappings=[("fix", "gps/fix")],
        ),

        # Pi Camera V2 over CSI, for ground segmentation (roadmap step 7).
        Node(
            package="camera_ros",
            executable="camera_node",
            name="camera",
            output="screen",
            condition=IfCondition(use_camera),
            parameters=[{
                "camera": 0,
                "width": 640,
                "height": 480,
                "frame_id": "camera_optical_link",
            }],
            remappings=[("~/image_raw", "camera/image_raw")],
        ),

        # TODO — IMU (MPU6050 on the Pi's I2C, address 0x68).
        # No apt package ships a driver for it on Jazzy, so this needs a decision:
        #   a) write a small rclpy node in this repo (smbus2, ~150 lines: read the
        #      raw registers, publish sensor_msgs/Imu with a real covariance)
        #   b) vendor a third-party driver into the workspace
        # Whichever we pick must publish /imu/data in the imu_link frame, per
        # REP-103 (x forward, y left, z up), or the EKF will confidently fuse a
        # rotated world. It must NOT publish an absolute yaw: the 6-axis part has
        # no heading reference (docs/handoff.md decision 4) — the magnetometer
        # (not yet purchased) is what fills that in.
    ])
