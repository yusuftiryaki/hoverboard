"""The same obstacles, rendered as Gazebo geometry. ROS-free, deterministic.

obstacle_map.py turns a tuple of CircularObstacle into the OccupancyGrid Nav2
reads. This turns THE SAME tuple into the SDF the physics engine collides
against. That is the whole point of it living here rather than in a static
world file: hoverbot.sdf and empty.sdf are checked in, so obstacles written
into them would be a second description of the world that nothing forces to
agree with the map — the A6 failure exactly, and the reason sim_node used to
refuse the gazebo backend and obstacle parameters together.

    obstacle params ──► parse_obstacle_params ──┬─► build_obstacle_grid ──► /obstacle_map
                                                └─► obstacle_model_sdf  ──► Gazebo physics

⚠️ The grid is 2-D and Gazebo is not, so one number has to be invented here:
how TALL an obstacle is. OBSTACLE_HEIGHT_M is chosen to span the chassis box
(which sits between 0.09 m and 0.27 m above the ground) with room to spare, so
every obstacle on the map is something the robot can actually hit. An obstacle
shorter than 0.09 m would be drawn on the map and driven straight over.
"""

from __future__ import annotations

import math
from typing import Sequence
from xml.sax.saxutils import quoteattr

from robot_sim.world import CircularObstacle

# Tall enough that the chassis (0.09 m to 0.27 m up) always hits it, short
# enough to stay a plausible rock/post rather than a wall.
OBSTACLE_HEIGHT_M = 1.0


def obstacle_model_sdf(
    obstacles: Sequence[CircularObstacle],
    name: str = "obstacles",
    height: float = OBSTACLE_HEIGHT_M,
) -> str:
    """One static model holding every obstacle as a vertical cylinder.

    One model rather than one per obstacle so the whole world arrives in a
    single spawn: a half-spawned obstacle list is a world the map disagrees
    with, and doing it in N calls is N chances to end up there.

    The cylinder's radius is the obstacle's radius exactly — the same bare
    obstacle the grid marks, with no safety margin baked in. Inflation is the
    costmap's job and it is done in one place (nav2.yaml). See obstacle_map.py.
    """
    if not obstacles:
        raise ValueError("obstacle_model_sdf called with no obstacles")

    links = []
    for index, obstacle in enumerate(obstacles):
        links.append(f"""
    <link name="obstacle_{index}">
      <pose>{obstacle.x!r} {obstacle.y!r} {height / 2.0!r} 0 0 0</pose>
      <collision name="collision">
        <geometry><cylinder><radius>{obstacle.radius!r}</radius><length>{height!r}</length></cylinder></geometry>
      </collision>
      <visual name="visual">
        <geometry><cylinder><radius>{obstacle.radius!r}</radius><length>{height!r}</length></cylinder></geometry>
      </visual>
    </link>""")

    return (
        '<?xml version="1.0"?>\n'
        '<sdf version="1.9">\n'
        f"  <model name={quoteattr(name)}>\n"
        # Static: an obstacle that can be shoved aside is not an obstacle, and
        # a dynamic one would also need a mass nobody has any basis to pick.
        "    <static>true</static>"
        + "".join(links)
        + "\n  </model>\n</sdf>\n"
    )


# --- the charging station, with its marker (camera:=true worlds) ---------------

# A low body under the marker: tall enough for the chassis box (0.09-0.27 m
# up) to hit, low enough not to hide the marker board standing on it (the
# board's lower edge is at tag_height - board/2: 0.125 m for the 0.20 m tag).
STATION_BODY_HEIGHT_M = 0.12
# The printed board round the marker: the black square plus a white quiet
# zone, which the detector needs to find the square at all — two of the
# tag's eight cells on each side (0.10 m square -> 0.15 m board).
TAG_BOARD_PER_MARKER = 1.5


def station_model_sdf(station, body, texture_path: str, name: str = "station",
                      marker_size: float = 0.20) -> str:
    """The station as Gazebo sees it: its body (the same CircularObstacle the
    kinematic world stops the robot at, but STATION_BODY_HEIGHT_M tall) and a
    board carrying an AprilTag 36h11 #0 at DockStation.tag_position, facing
    the robot driving in.

    ⚠️ The kinematic model puts the tag INSIDE the body's footprint (0.55 m
    ahead of the docked base_link; the body spans 0.42-0.72 m) — harmless with
    no renderer, but a 1 m obstacle cylinder there hides the marker from the
    camera completely. Hence the low body with the board standing on it.

    The board link's +X is the marker's normal (back at the robot); the
    texture is drawn on that face. Its orientation is not taken on faith:
    test_gazebo_marker.py checks the real detector decodes it and reads it
    the way dock_model.tag_rotation says.
    """
    board = marker_size * TAG_BOARD_PER_MARKER
    tx, ty, tz = station.tag_position
    if tz - board / 2 < -1e-9:
        raise ValueError(f"a {marker_size} m marker at {tz} m height reaches below the ground")
    body_height = STATION_BODY_HEIGHT_M
    if math.hypot(tx - body.x, ty - body.y) <= body.radius + 0.01:
        # The board stands ON the body: never above its lower edge, or the
        # body hides the bottom of the marker.
        body_height = min(body_height, tz - board / 2 - 0.005)
        if body_height <= 0.0:
            raise ValueError(f"a {marker_size} m marker at {tz} m leaves no room for the body under it")
    face = station.yaw + 3.141592653589793
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name={quoteattr(name)}>
    <static>true</static>
    <link name="body">
      <pose>{body.x!r} {body.y!r} {body_height / 2!r} 0 0 0</pose>
      <collision name="collision">
        <geometry><cylinder><radius>{body.radius!r}</radius><length>{body_height!r}</length></cylinder></geometry>
      </collision>
      <visual name="visual">
        <geometry><cylinder><radius>{body.radius!r}</radius><length>{body_height!r}</length></cylinder></geometry>
        <material><ambient>0.3 0.3 0.35 1</ambient><diffuse>0.35 0.35 0.4 1</diffuse></material>
      </visual>
    </link>
    <link name="tag_board">
      <pose>{tx!r} {ty!r} {tz!r} 0 0 {face!r}</pose>
      <visual name="tag">
        <geometry><box><size>0.004 {board!r} {board!r}</size></box></geometry>
        <material>
          <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>
          <pbr><metal><albedo_map>{texture_path}</albedo_map><roughness>1</roughness><metalness>0</metalness></metal></pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""
