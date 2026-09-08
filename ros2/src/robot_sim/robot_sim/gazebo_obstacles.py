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
