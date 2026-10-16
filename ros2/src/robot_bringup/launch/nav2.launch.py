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
an extra hop that inspects nothing. Here the chain is

    controller_server, behavior_server -> /cmd_vel_nav -> nav_cmd_gate
        -> /cmd_vel_nav_gated -> twist_mux -> /cmd_vel

⚠️ nav_cmd_gate (hoverboard_bridge) forwards only while ekf_local's output is
sane, and sends zero otherwise. robot_localization 3.8.3 once took ekf_local
to 1e148 m silently and Nav2 stopped only because its planner overflowed; see
the gate's docstring. BOTH servers that publish velocities go through it — a
recovery spin or backup on a broken estimate is still driving on it. Teleop is
deliberately not gated: it reaches twist_mux (robot.launch.py) on
/cmd_vel_teleop at a HIGHER priority, so a human can take the wheels during a
goal, and while the gate is blocking, instead of racing Nav2's commands.

Also here, because they need Nav2's actions (SP4/SP5):
  * docking_server (opennav_docking) — the final approach onto the charging
    contacts, steered by the marker on /detected_dock_pose. Its cmd_vel goes
    through nav_cmd_gate like the other two: it drives on ekf_local too.
  * battery_supervisor (battery_manager) — /battery LOW: cancel the mission,
    stage, face the dock, dock; CRITICAL: cancel everything and hold
    twist_mux's nav_lock. Both read dock_database, so home is defined once.

Deliberately not launched (and why):
  * velocity_smoother — the ESP32 sends at 50 Hz and the board ramps its own
    output. Worth revisiting if the real robot jerks (roadmap B5).
  * collision_monitor — needs a range sensor. See A3.
  * smoother_server — smooths paths around obstacles. Ours are straight lines.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from hoverboard_bridge.nav2_watchdog import NAV2_STUCK_EXIT

# Started, then driven through configure/activate by the lifecycle manager.
LIFECYCLE_NODES = [
    "controller_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "docking_server",
]


# NAV2_STUCK_EXIT is nav2_watchdog's "Nav2 is stuck" exit code. Only THAT
# shuts the bringup down: a watchdog that crashed for its own reasons (exit 1,
# a traceback) must not take a working robot down with it — it is logged and
# left dead. (Checked: with the code mismatched, the launch is NOT shut down.)
def _on_watchdog_exit(event, _context):
    if event.returncode == NAV2_STUCK_EXIT:
        return [LogInfo(msg="nav2_watchdog gave up on Nav2 — shutting the bringup "
                            "down so systemd can restart it"),
                EmitEvent(event=Shutdown(reason="nav2_watchdog: Nav2 not active"))]
    if event.returncode not in (0, -2, -15):     # clean, SIGINT, SIGTERM
        return [LogInfo(msg=f"nav2_watchdog died (exit {event.returncode}); Nav2 is "
                            "no longer watched")]
    return []


def generate_launch_description():
    pkg = get_package_share_directory("robot_bringup")
    params = os.path.join(pkg, "config", "nav2.yaml")
    trees = os.path.join(pkg, "behavior_trees")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    battery_config = LaunchConfiguration("battery_config")
    dock_database = LaunchConfiguration("dock_database")
    site_file = LaunchConfiguration("site_file")
    watchdog_timeout = LaunchConfiguration("watchdog_timeout")

    # ⚠️ Nav2 1.3.13 can hang in bring-up for good (hoverboard_bridge
    # nav2_watchdog.py has the measured story). The watchdog only notices and
    # exits; THIS turns its exit into a Shutdown of the whole launch — robot.
    # launch.py included — so systemd starts everything afresh.
    # ⚠️ `ros2 launch` exits 0 after such a Shutdown (measured), so
    # robot.service needs Restart=always, NOT on-failure (deployment.md step 8).
    watchdog = Node(
        package="hoverboard_bridge",
        executable="nav2_watchdog",
        name="nav2_watchdog",
        output="screen",
        condition=IfCondition(autostart),     # without autostart nothing activates
        parameters=[{"timeout_s": watchdog_timeout, "use_sim_time": use_sim_time}],
    )

    common = [params, {"use_sim_time": use_sim_time}]

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "autostart", default_value="true",
            description="Let the lifecycle manager bring the Nav2 nodes up on start.",
        ),
        DeclareLaunchArgument(
            "watchdog_timeout", default_value="120.0",
            description="Seconds Nav2 may be not-active (at bring-up or later) "
                        "before the whole bringup is shut down for a restart.",
        ),
        DeclareLaunchArgument(
            "battery_config",
            default_value=os.path.join(pkg, "config", "battery_manager.yaml"),
        ),
        DeclareLaunchArgument(
            "site_file", default_value="",
            description="The site (docs/missions.md). A station named like the "
                        "supervisor's dock_id there REPLACES docks.yaml's pose.",
        ),
        DeclareLaunchArgument(
            "dock_database",
            default_value=os.path.join(pkg, "config", "docks.yaml"),
            description="Where the charging station is (surveyed, `map`).",
        ),

        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=common,
            remappings=[("cmd_vel", "cmd_vel_nav")],   # through nav_cmd_gate
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
            remappings=[("cmd_vel", "cmd_vel_nav")],   # spin/backup too
        ),
        # Not a lifecycle node: it must already be gating before the servers
        # above activate and publish their first command.
        # Its output is twist_mux's Nav2 input, not /cmd_vel itself: the mux
        # (config/twist_mux.yaml) puts teleop above it.
        Node(
            package="hoverboard_bridge",
            executable="nav_cmd_gate",
            name="nav_cmd_gate",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            remappings=[("cmd_vel", "cmd_vel_nav_gated")],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            # ⚠️ Our trees, not the stock ones: they name the goal checker,
            # which controller_server needs now that it has two (nav2.yaml).
            parameters=[*common, {
                "default_nav_to_pose_bt_xml": os.path.join(trees, "navigate_to_pose.xml"),
                "default_nav_through_poses_bt_xml":
                    os.path.join(trees, "navigate_through_poses.xml"),
            }],
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
            package="opennav_docking",
            executable="opennav_docking",
            name="docking_server",
            output="screen",
            parameters=[*common, {"dock_database": dock_database}],
            remappings=[
                ("cmd_vel", "cmd_vel_nav"),          # through nav_cmd_gate
                # SimpleChargingDock reads `battery_state`; ours is /battery.
                ("battery_state", "battery"),
            ],
        ),
        # Plain node, not lifecycle: it must watch the battery from the start.
        Node(
            package="battery_manager",
            executable="battery_supervisor",
            name="battery_supervisor",
            output="screen",
            parameters=[battery_config, {"dock_database": dock_database,
                                         "site_file": site_file,
                                         "use_sim_time": use_sim_time}],
            respawn=True,
            respawn_delay=2.0,
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

        watchdog,
        RegisterEventHandler(OnProcessExit(target_action=watchdog,
                                           on_exit=_on_watchdog_exit)),
    ])
