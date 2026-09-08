"""Do the SDFs and the ROS-GZ bridge actually name the same topics?

⚠️ THIS IS THE TEST THAT WOULD HAVE CAUGHT A3's REAL BUG, in milliseconds and
with no Gazebo running. bridge.yaml pointed at /model/hoverbot/cmd_vel and
/model/hoverbot/odometry while hoverbot.sdf configured DiffDrive with
<topic>cmd_vel</topic> and <odom_topic>odometry</odom_topic>. Nothing errors
when a bridge is pointed at a topic nobody publishes: the GZ_TO_ROS side simply
stays silent and the ROS_TO_GZ side swallows every command. The whole physics
backend was wired to nothing and the only way anyone found out was by reading
`gz topic -l` by hand.

Everything here is text: no Gazebo, no ROS, milliseconds. The acceptance test in
test_gazebo_physics.py is the one that proves the robot moves, and it costs a
minute of Gazebo per case — this one is what keeps a rename from silently
unplugging it.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

yaml = pytest.importorskip("yaml")

GAZEBO_DIR = Path(__file__).resolve().parent.parent / "gazebo"
WORLD = GAZEBO_DIR / "empty.sdf"
MODEL = GAZEBO_DIR / "hoverbot.sdf"
BRIDGE = GAZEBO_DIR / "bridge.yaml"


@pytest.fixture(scope="module")
def bridge():
    return {entry["gz_topic_name"]: entry for entry in yaml.safe_load(BRIDGE.read_text())}


@pytest.fixture(scope="module")
def model():
    return ET.parse(MODEL).getroot().find("model")


@pytest.fixture(scope="module")
def world():
    return ET.parse(WORLD).getroot().find("world")


def plugin(root, name):
    for element in root.iter("plugin"):
        if element.get("name") == name:
            return element
    raise AssertionError(f"no plugin named {name} in {root.get('name')}")


def test_diff_drive_publishes_the_topics_the_bridge_subscribes_to(model, bridge):
    diff_drive = plugin(model, "gz::sim::systems::DiffDrive")
    command = diff_drive.findtext("topic")
    odometry = diff_drive.findtext("odom_topic")
    assert command in bridge, (
        f"DiffDrive takes commands on {command!r}; bridge.yaml bridges "
        f"{sorted(bridge)} — the ROS->GZ side would swallow every command")
    assert bridge[command]["direction"] == "ROS_TO_GZ"
    assert odometry in bridge, (
        f"DiffDrive publishes odometry on {odometry!r}, which bridge.yaml does "
        "not carry — the hall feedback would be silently absent")
    assert bridge[odometry]["direction"] == "GZ_TO_ROS"


def test_ground_truth_comes_from_odometry_publisher_and_not_from_diff_drive(model, bridge):
    """The two odometries must be two DIFFERENT topics.

    DiffDrive dead-reckons from the wheel joint angles, so its odometry is what
    the hall sensors would say — it keeps counting while the wheels spin on the
    spot. Measured on the pre-A3 model, whose wheels were buried inside the
    chassis: DiffDrive said 5.47 m, the true pose said 3.8e-9 m. If these two
    ever became one topic, /ground_truth would agree with /odom by construction
    and every localization number measured in this world would be a tautology.
    """
    truth = plugin(model, "gz::sim::systems::OdometryPublisher").findtext("odom_topic")
    wheels = plugin(model, "gz::sim::systems::DiffDrive").findtext("odom_topic")
    assert truth != wheels, "ground truth and wheel odometry are the same topic"
    assert truth in bridge and bridge[truth]["direction"] == "GZ_TO_ROS"
    # And the ROS names must not collide either.
    assert bridge[truth]["ros_topic_name"] != bridge[wheels]["ros_topic_name"]


def test_the_clock_is_bridged(bridge):
    """Without /clock, use_sim_time:=true leaves every node waiting forever.

    The other half of the same bug: the ROS stack had no /clock to run on, so
    even had someone set use_sim_time it would have hung rather than run on the
    wall clock. Both halves were missing at once, which is why neither showed.
    """
    assert "/clock" in bridge, "bridge.yaml does not carry /clock"
    assert bridge["/clock"]["ros_type_name"] == "rosgraph_msgs/msg/Clock"
    assert bridge["/clock"]["direction"] == "GZ_TO_ROS"


def test_the_contact_sensor_topic_matches_the_world_and_model_names(model, world, bridge):
    """gz-sim names a contact sensor's topic itself and ignores the SDF's <topic>.

    The real name is /world/<world>/model/<model>/link/<link>/sensor/<sensor>/contact,
    so renaming the world in empty.sdf or the model in hoverbot.sdf silently
    unplugs /collision_truth. Reconstructed here rather than copied.
    """
    link = next(l for l in model.findall("link") if l.get("name") == "base_link")
    sensor = next(s for s in link.findall("sensor") if s.get("type") == "contact")
    expected = (f"/world/{world.get('name')}/model/{model.get('name')}"
                f"/link/{link.get('name')}/sensor/{sensor.get('name')}/contact")
    assert expected in bridge, (
        f"the contact sensor publishes on {expected!r}; bridge.yaml has "
        f"{sorted(bridge)} — /collision_truth would read False forever")


def test_the_world_loads_the_systems_its_models_depend_on(world):
    """A sensor whose system is absent exists in the SDF and never publishes."""
    filenames = {element.get("filename") for element in world.iter("plugin")}
    assert "gz-sim-physics-system" in filenames
    # Without this the contact sensor is parsed, accepted, and silent.
    assert "gz-sim-contact-system" in filenames, (
        "empty.sdf has no Contact system, so hoverbot.sdf's chassis contact "
        "sensor will never publish and /collision_truth stays False forever")


def test_the_world_states_its_physics_rather_than_defaulting(world):
    """Sim time is the whole stack's clock once use_sim_time is on.

    Measured with no <physics> element at all: step 1 ms, achieved RTF 1.0000.
    So this element does not change today's behaviour — it stops the numbers
    every timing measurement in this package depends on from being an
    unstated default that a Gazebo upgrade could move.
    """
    physics = world.find("physics")
    assert physics is not None, "empty.sdf leaves step size and RTF to the defaults"
    assert float(physics.findtext("max_step_size")) == pytest.approx(0.001)
    assert float(physics.findtext("real_time_factor")) == pytest.approx(1.0)


def test_both_wheels_are_placed_and_placed_apart(model):
    """A link with no <pose> sits at the model origin, and a joint's <pose>
    does not move its child.

    Both wheels used to have no <pose> at all: two coincident discs buried
    inside the chassis box, touching nothing, while joint <pose> elements a
    reader would take for wheel placement sat on the joints. Gazebo drove them
    anyway and published odometry anyway. This asserts the wheels are where the
    DiffDrive plugin is told they are.
    """
    poses = {}
    for link in model.findall("link"):
        name = link.get("name")
        if name.endswith("_wheel"):
            assert link.find("pose") is not None, f"{name} has no <pose>"
            poses[name] = [float(v) for v in link.findtext("pose").split()]
    assert set(poses) == {"left_wheel", "right_wheel"}

    diff_drive = plugin(model, "gz::sim::systems::DiffDrive")
    separation = float(diff_drive.findtext("wheel_separation"))
    radius = float(diff_drive.findtext("wheel_radius"))
    left, right = poses["left_wheel"], poses["right_wheel"]

    # The plugin's kinematics are only right if the geometry matches them.
    assert abs(left[1] - right[1]) == pytest.approx(separation), (
        f"wheels are {abs(left[1] - right[1])} m apart but DiffDrive is told "
        f"{separation} m")
    # The model frame is at ground level, so a wheel's centre height IS its
    # radius. Anything else means the wheels hover or are buried.
    for name, pose in poses.items():
        assert pose[2] == pytest.approx(radius), (
            f"{name} centre is {pose[2]} m up but its radius is {radius} m — "
            "it does not touch the ground")
        assert pose[0] == pytest.approx(0.0), f"{name} is off the axle line"


def test_the_wheels_state_their_friction(model):
    """mu decides how much the robot can slip, which is this backend's whole
    reason to exist. Left to the engine's default of 1.0 it would be dry rubber
    on asphalt, and the slip figures in test_gazebo_physics.py are derived from
    the number being stated."""
    for name in ("left_wheel", "right_wheel"):
        link = next(l for l in model.findall("link") if l.get("name") == name)
        mu = link.find("collision").find("surface/friction/ode/mu")
        assert mu is not None, f"{name} does not state a friction coefficient"
        assert 0.0 < float(mu.text) <= 1.0
