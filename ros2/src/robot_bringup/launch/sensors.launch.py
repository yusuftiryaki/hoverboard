"""Pi-side sensor drivers: GPS, IMU, camera (docs/wiring-map.md section 3d).

Everything is behind a launch argument and defaults OFF, so the stack still
comes up while the hardware is still in a box. Turn each one on as it lands.
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
    imu_params = os.path.join(pkg, "config", "mpu6050.yaml")
    mag_params = os.path.join(pkg, "config", "qmc5883l.yaml")

    use_gps = LaunchConfiguration("use_gps")
    use_imu = LaunchConfiguration("use_imu")
    use_mag = LaunchConfiguration("use_mag")
    use_imu_filter = LaunchConfiguration("use_imu_filter")
    use_camera = LaunchConfiguration("use_camera")
    gps_port = LaunchConfiguration("gps_port")
    fake_imu = LaunchConfiguration("fake_imu")
    fake_mag = LaunchConfiguration("fake_mag")

    return LaunchDescription([
        DeclareLaunchArgument("use_gps", default_value="false"),
        DeclareLaunchArgument("use_imu", default_value="false"),
        DeclareLaunchArgument("use_mag", default_value="false"),
        DeclareLaunchArgument(
            "use_imu_filter", default_value="false",
            description="Fuse imu/data_raw + imu/mag into imu/data. Required for "
                        "GPS: navsat_transform needs an orientation, and only "
                        "this produces one.",
        ),
        DeclareLaunchArgument("use_camera", default_value="false"),
        DeclareLaunchArgument(
            "fake_imu", default_value="false",
            description="Simulate the MPU6050 — dev machine only, there is no I2C there.",
        ),
        DeclareLaunchArgument(
            "fake_mag", default_value="false",
            description="Simulate the QMC5883L — dev machine only.",
        ),

        # MPU6050 on the Pi's I2C bus. Ours: Jazzy ships no driver for this chip.
        # ⚠️ It calibrates the gyro bias at startup, so THE ROBOT MUST BE STILL
        # for the first few seconds after this launches. It logs an error if it
        # thinks you moved.
        Node(
            package="mpu6050_driver",
            executable="mpu6050",
            name="mpu6050",
            output="screen",
            condition=IfCondition(use_imu),
            parameters=[imu_params, {"use_fake_bus": fake_imu}],
            # I2C on a vibrating robot throws transient errors; the node already
            # rides those out. Respawn covers the harder failures.
            respawn=True,
            respawn_delay=2.0,
        ),
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

        # Magnetometer on the mast (0x0D, same I2C bus as the IMU). The robot's
        # only absolute heading reference — without it ekf_global's yaw is
        # unobservable and GPS waypoint following does not work at all.
        # ⚠️ Publishes a heading you can trust ONLY after calibration:
        # docs/mag-calibration.md. It warns loudly if hard_iron is still zeros.
        Node(
            package="qmc5883l_driver",
            executable="qmc5883l",
            name="qmc5883l",
            output="screen",
            condition=IfCondition(use_mag),
            parameters=[mag_params, {"use_fake_bus": fake_mag}],
            respawn=True,
            respawn_delay=2.0,
        ),

        # Fuses gyro + accel (/imu/data_raw) with the field (/imu/mag) into
        # /imu/data, which carries a real orientation. Nothing else in the stack
        # can do this: robot_localization consumes Imu, never MagneticField, so
        # the field has to become an orientation somewhere — and doing it here
        # gets tilt compensation for free, which a bare compass reading lacks the
        # moment the robot leaves level ground.
        Node(
            package="imu_filter_madgwick",
            executable="imu_filter_madgwick_node",
            name="imu_filter_madgwick",
            output="screen",
            condition=IfCondition(use_imu_filter),
            parameters=[{
                "use_mag": True,
                # ENU, per REP-103. The default is NWU, which would rotate every
                # heading by 90 degrees and look almost plausible.
                "world_frame": "enu",
                # The bias is corrected in qmc5883l_driver, where the calibration
                # numbers live; doing it twice would double-subtract.
                "mag_bias_x": 0.0,
                "mag_bias_y": 0.0,
                "mag_bias_z": 0.0,
                "gain": 0.1,
                # It would otherwise broadcast imu_link -> base_link and fight
                # robot_state_publisher for a transform the URDF already owns.
                "publish_tf": False,
            }],
        ),
    ])
