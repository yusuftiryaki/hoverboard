"""Nav2 for open-ground GPS waypoint following (A2).

    # simulated, no hardware:
    ros2 run robot_sim sim_node
    ros2 launch robot_bringup robot.launch.py esp32_port:=/tmp/fake_esp32 \\
        use_localization:=true use_gps:=true use_imu:=false use_nav2:=true

⚠️ Needs use_gps:=true. Nav2 plans in `map`, and map->odom only exists when
ekf_global and navsat_transform are running. Without it Nav2 comes up and then
waits forever for a transform that never arrives.

This is hand-rolled rather than an include of nav2_bringup's navigation_launch.py,
which assumes a robot we do not have. That file wires
    controller -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel_smoothed
                -> collision_monitor -> /cmd_vel
and collision_monitor exists to watch range sensors. We have none, so it would be
an extra hop that inspects nothing. Here the controller publishes /cmd_vel
directly and the chain is short enough to read.

Deliberately not launched (and why):
  * velocity_smoother — the ESP32 sends at 50 Hz and the board ramps its own
    output. Worth revisiting if the real robot jerks (roadmap B5).
  * collision_monitor — needs a range sensor. See A3.
  * smoother_server — smooths paths around obstacles. Ours are straight lines.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Started, then driven through configure/activate by the lifecycle manager.
LIFECYCLE_NODES = [
    "controller_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
]


def generate_launch_description():
    pkg = get_package_share_directory("robot_bringup")
    params = os.path.join(pkg, "config", "nav2.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    common = [params, {"use_sim_time": use_sim_time}]

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "autostart", default_value="true",
            description="Let the lifecycle manager bring the Nav2 nodes up on start.",
        ),

        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=common,
            # No remap: hoverboard_bridge listens on /cmd_vel.
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=common,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=common,
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=common,
        ),
        # Serves both FollowWaypoints and FollowGPSWaypoints. The GPS variant
        # converts lat/lon via navsat_transform's /fromLL service, which is why
        # localization must already be up.
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=common,
        ),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": LIFECYCLE_NODES,
            }],
        ),
    ])
