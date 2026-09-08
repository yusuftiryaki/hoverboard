"""sim_node's obstacle wiring, in-process. Skips cleanly without ROS.

test_obstacle_map.py proves the grid maths. This proves the node actually
reaches it: that the ROS parameters parse, that the map is latched on the right
frame, and — the one that matters — that the obstacle the map draws is the same
obstacle the world will physically stop the robot at. A map the physics does
not share is the phantom-obstacle version of A6.
"""

import pytest

rclpy = pytest.importorskip("rclpy", reason="ROS 2 not sourced")

from nav_msgs.msg import OccupancyGrid                       # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile           # noqa: E402

from robot_sim.gazebo_obstacles import (                     # noqa: E402
    OBSTACLE_HEIGHT_M, obstacle_model_sdf)
from robot_sim.obstacle_map import (                         # noqa: E402
    DEFAULT_GRID, FREE, OCCUPIED, build_obstacle_grid, parse_obstacle_params)
from robot_sim.sim_node import SimNode                       # noqa: E402
from robot_sim.world import CircularObstacle                 # noqa: E402


@pytest.fixture
def context():
    own = not rclpy.ok()
    if own:
        rclpy.init()
    yield
    if own:
        rclpy.shutdown()


def make_sim(tmp_path, **overrides):
    # Its own pty per test: the default /tmp/fake_esp32 symlink would clobber a
    # simulator someone is running by hand in another terminal.
    overrides.setdefault("link", str(tmp_path / "fake_esp32"))
    return SimNode(parameter_overrides=[
        rclpy.parameter.Parameter(name, value=value) for name, value in overrides.items()
    ])


def test_no_obstacle_params_means_no_obstacles_and_no_map(context, tmp_path):
    node = make_sim(tmp_path)
    received = []
    listener = rclpy.create_node("empty_map_listener")
    listener.create_subscription(
        OccupancyGrid, "obstacle_map", received.append,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    try:
        assert node._obstacles == ()
        assert node._world._obstacles == ()
        # ⚠️ An empty world must still latch an ALL-FREE map, not silence.
        # Nav2's StaticLayer blocks its costmap until a map arrives, so staying
        # quiet aborts every goal rather than meaning "nothing in the way" —
        # measured as status 6 on a wall-free run before this was unconditional.
        deadline = node.get_clock().now().nanoseconds + 5_000_000_000
        while not received and node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(listener, timeout_sec=0.1)
        assert received, "obstacle-free world latched no map at all"
        assert set(received[0].data) == {0}
    finally:
        listener.destroy_node()
        node.destroy_node()


def test_obstacle_params_reach_both_the_world_and_a_latched_map(context, tmp_path):
    node = make_sim(tmp_path, obstacle_centers=[2.0, -1.0], obstacle_radii=[0.5])
    received = []
    listener = rclpy.create_node("obstacle_map_listener")
    listener.create_subscription(
        OccupancyGrid, "obstacle_map", received.append,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    try:
        expected = (CircularObstacle(x=2.0, y=-1.0, radius=0.5),)
        # The SAME objects, not merely equal ones: one parse, one world.
        assert node._obstacles == expected
        assert node._world._obstacles == expected

        deadline = node.get_clock().now().nanoseconds + 5_000_000_000
        while not received and node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(listener, timeout_sec=0.1)
        assert received, "latched obstacle_map hiç gelmedi"

        message = received[0]
        # `map`, so Nav2's global costmap can consume it: the obstacles are
        # declared to be surveyed in the navsat datum. See _publish_obstacle_map.
        assert message.header.frame_id == "map"
        assert message.info.width == DEFAULT_GRID.width
        assert message.info.height == DEFAULT_GRID.height
        assert message.info.resolution == pytest.approx(DEFAULT_GRID.resolution)
        assert message.info.origin.position.x == pytest.approx(DEFAULT_GRID.origin_x)
        assert message.info.origin.position.y == pytest.approx(DEFAULT_GRID.origin_y)

        # The obstacle's own centre cell, worked out from the grid origin:
        # (2.0 - -30.0) / 0.1 = 320, (-1.0 - -30.0) / 0.1 = 290.
        assert message.data[DEFAULT_GRID.index(320, 290)] == OCCUPIED
        # 1 m north of it is outside a 0.5 m obstacle.
        assert message.data[DEFAULT_GRID.index(320, 300)] != OCCUPIED
    finally:
        listener.destroy_node()
        node.destroy_node()


def test_mismatched_obstacle_params_fail_loudly_at_startup(context, tmp_path):
    with pytest.raises(ValueError, match="obstacle_radii"):
        make_sim(tmp_path, obstacle_centers=[1.0, 2.0, 3.0, 4.0], obstacle_radii=[0.3])


def test_unsurveyed_obstacles_collide_but_never_reach_the_map(context, tmp_path):
    """The real world's asymmetry: physically there, absent from the survey.

    Not to be confused with the phantom-obstacle bug, which was the opposite —
    a map the physics did not share. That one is impossible by construction now.
    This one is the tree nobody wrote down, and the whole point is that Nav2
    cannot see it. test_nav2.py measures what that costs.
    """
    node = make_sim(
        tmp_path,
        obstacle_centers=[2.0, 0.0], obstacle_radii=[0.5],
        unsurveyed_obstacle_centers=[5.0, 1.0], unsurveyed_obstacle_radii=[0.25],
    )
    received = []
    listener = rclpy.create_node("unsurveyed_listener")
    listener.create_subscription(
        OccupancyGrid, "obstacle_map", received.append,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    try:
        surveyed = CircularObstacle(x=2.0, y=0.0, radius=0.5)
        unsurveyed = CircularObstacle(x=5.0, y=1.0, radius=0.25)
        # The world collides against both; only the map distinguishes them.
        assert node._world._obstacles == (surveyed, unsurveyed)

        deadline = node.get_clock().now().nanoseconds + 5_000_000_000
        while not received and node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(listener, timeout_sec=0.1)
        assert received, "latched obstacle_map hiç gelmedi"
        data = received[0].data

        # Cell centres of each obstacle: (x - -30.0) / 0.1, likewise y.
        assert data[DEFAULT_GRID.index(320, 300)] == OCCUPIED    # surveyed
        assert data[DEFAULT_GRID.index(350, 310)] != OCCUPIED    # unsurveyed
    finally:
        listener.destroy_node()
        node.destroy_node()


def test_one_obstacle_list_reaches_both_the_map_and_the_gazebo_geometry(tmp_path):
    """The gazebo backend used to REFUSE obstacles; now it renders them.

    Refusing was the right call while the backend could not put a ROS parameter
    into Gazebo: warning and carrying on would have published a map of
    obstacles the physics drove straight through — a phantom obstacle with a
    warning in front of it that nobody reads in a test log. What replaces the
    refusal has to be stronger than a promise, so the map and the physics are
    rendered from THE SAME parsed tuple, and this checks the two renderers
    against one another without needing Gazebo or ROS at all.

    test_gazebo_physics.py is where a real obstacle is driven into.
    """
    obstacles = parse_obstacle_params([3.0, -1.5, -2.0, 4.0], [0.5, 0.25])
    sdf = obstacle_model_sdf(obstacles)

    # Every obstacle, with its own radius, at its own place.
    for obstacle in obstacles:
        assert f"<radius>{obstacle.radius!r}</radius>" in sdf
        assert f"<pose>{obstacle.x!r} {obstacle.y!r} " in sdf
    assert sdf.count("<link ") == len(obstacles)

    # The bare obstacle in both, with no safety margin baked into either: the
    # robot radius is the costmap inflation layer's job and it is applied in
    # exactly one place (nav2.yaml). See obstacle_map.py.
    grid = build_obstacle_grid(obstacles)
    assert grid[DEFAULT_GRID.index(330, 285)] == OCCUPIED     # (3.0, -1.5)
    assert grid[DEFAULT_GRID.index(280, 340)] == OCCUPIED     # (-2.0, 4.0)
    # 0.4 m east of the SMALL one is outside its 0.25 m radius but would be
    # inside the big one's — so this also catches the radii being swapped.
    assert grid[DEFAULT_GRID.index(284, 340)] == FREE


def test_an_obstacle_is_tall_enough_for_the_chassis_to_hit():
    """The grid is 2-D and Gazebo is not, so the height is invented here.

    hoverbot.sdf's chassis box sits between 0.09 m and 0.27 m above the ground
    (9 cm of clearance for the casters). An obstacle shorter than that would be
    drawn on the map, planned around by Nav2, and driven straight over.
    """
    assert OBSTACLE_HEIGHT_M > 0.27


def test_the_gazebo_backend_refuses_a_slip_factor(context, tmp_path):
    """slip_factor is a KinematicWorld knob and it is not silently ignored.

    Slip in this world comes out of wheel friction, mass and load transfer —
    hoverbot.sdf's mu is the knob, and test_gazebo_physics.py measures what it
    implies. Accepting the parameter and dropping it would let a test believe
    it had configured slip while the number went nowhere.
    """
    with pytest.raises(ValueError, match="slip_factor"):
        make_sim(tmp_path, backend="gazebo", slip_factor=0.1)
