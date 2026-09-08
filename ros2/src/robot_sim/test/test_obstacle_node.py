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

from robot_sim.obstacle_map import DEFAULT_GRID, OCCUPIED    # noqa: E402
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


def test_the_gazebo_backend_refuses_obstacle_parameters(context, tmp_path):
    """Refuse, do not warn.

    Gazebo's obstacles live in hoverbot.sdf; the backend cannot honour a ROS
    parameter. It used to log a warning and carry on, which published a map of
    obstacles the physics would drive straight through — the phantom-obstacle
    bug with a warning in front of it that nobody reads in a test log.
    """
    with pytest.raises(ValueError, match="gazebo backend"):
        make_sim(tmp_path, backend="gazebo",
                 obstacle_centers=[1.0, 1.0], obstacle_radii=[0.3])
