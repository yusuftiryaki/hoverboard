"""The station's geometry and the fake marker detection, against hand numbers.

The detection must speak image_proc's track_marker_node convention, because
nav2.yaml's external_detection_* values are set for that and are meant to be
the REAL robot's. These tests pin the convention by hand rather than by
re-deriving it with the same maths (handoff decision 10), and sweep the
robot's heading (decision 11), since a frame mix-up hides at yaw 0.

ROS-free; runs in the `unit` stage.
"""

import math

import pytest

from robot_sim.dock_model import (CameraMount, DockStation, matrix_from_quaternion,
                                  quaternion_from_matrix, rot_z)
from robot_sim.world import KinematicWorld, Pose

# REP-103 optical frame of a LEVEL camera: z forward, x right, y down —
# robot.urdf.xacro's camera_optical_joint rpy (-pi/2, 0, -pi/2) with no pitch.
LEVEL_OPTICAL = ((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0))
LEVEL_CAMERA = CameraMount(translation=(0.30, 0.0, 0.20), rotation=LEVEL_OPTICAL)


def flat_station(yaw=0.0, x=1.0, y=0.0):
    # tag 0.55 ahead of the docked base, at the camera's height of 0.20 m.
    return DockStation(x=x, y=y, yaw=yaw, tag_distance=0.55, tag_height=0.20)


def test_a_tag_dead_ahead_is_on_the_optical_axis_and_faces_back():
    # Robot at the origin facing east; dock at x=1.0 so the tag is at x=1.55.
    # Camera at x=0.30: the tag is 1.25 m straight down the optical axis.
    position, (qx, qy, qz, qw) = flat_station().tag_in_camera(Pose(), LEVEL_CAMERA)
    assert position == pytest.approx((0.0, 0.0, 1.25))
    # Marker x (right, seen facing it) = camera x; marker y (up) = camera -y;
    # marker z (towards the viewer) = camera -z. That is 180 deg about x.
    assert (abs(qx), qy, qz, qw) == pytest.approx((1.0, 0.0, 0.0, 0.0), abs=1e-9)


def test_a_robot_left_of_the_line_sees_the_tag_to_its_right():
    position, _ = flat_station().tag_in_camera(Pose(y=0.2), LEVEL_CAMERA)
    # 0.2 m left of the line: the tag sits 0.2 m right of the axis (+x optical).
    assert position == pytest.approx((0.2, 0.0, 1.25))


def test_a_tag_above_the_camera_is_up_in_the_image():
    station = DockStation(x=1.0, y=0.0, yaw=0.0, tag_height=0.30)
    position, _ = station.tag_in_camera(Pose(), LEVEL_CAMERA)
    # optical y points DOWN
    assert position == pytest.approx((0.0, -0.10, 1.25))


@pytest.mark.parametrize("heading_deg", range(0, 360, 30))
def test_the_relative_measurement_does_not_depend_on_where_the_world_is_turned(heading_deg):
    """The same robot-to-tag geometry, rotated as a whole around the world:
    the camera must report the identical pose every time."""
    yaw = math.radians(heading_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    station = flat_station(yaw=yaw, x=1.0 * c - 0.0 * s, y=1.0 * s + 0.0 * c)
    robot = Pose(x=-0.2 * s, y=0.2 * c, yaw=yaw)     # 0.2 m to its own left
    position, quaternion = station.tag_in_camera(robot, LEVEL_CAMERA)
    assert position == pytest.approx((0.2, 0.0, 1.25), abs=1e-9)
    assert tuple(abs(q) for q in quaternion) == pytest.approx((1, 0, 0, 0), abs=1e-9)


def test_out_of_view_out_of_range_and_from_behind_report_nothing():
    station = flat_station()
    # Turned 90 deg away: outside the 62 deg horizontal field.
    assert station.tag_in_camera(Pose(yaw=math.pi / 2), LEVEL_CAMERA) is None
    # 6 m back: beyond max_range.
    assert station.tag_in_camera(Pose(x=-5.0), LEVEL_CAMERA) is None
    # Behind the station, looking at the back of the tag.
    assert station.tag_in_camera(Pose(x=3.0, yaw=math.pi), LEVEL_CAMERA) is None


def test_the_real_mount_sees_the_tag_all_the_way_in():
    """robot.urdf.xacro's actual camera: x 0.30, z 0.2325, pitched 0.35 rad
    down. The docking controller needs a fresh detection on EVERY step of the
    approach (opennav external_detection_timeout), so a tag that drops out of
    the frame in the last few centimetres fails a dock that was going fine."""
    pitch = 0.35
    cp, sp = math.cos(pitch), math.sin(pitch)
    pitched = ((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp))
    optical = tuple(tuple(sum(pitched[i][k] * LEVEL_OPTICAL[k][j] for k in range(3))
                          for j in range(3)) for i in range(3))
    mount = CameraMount(translation=(0.30, 0.0, 0.2325), rotation=optical)
    station = DockStation(x=0.0, y=0.0, yaw=0.0)
    for along in (-1.5, -1.0, -0.5, -0.2, 0.0, 0.02):
        assert station.tag_in_camera(Pose(x=along), mount) is not None, along


def test_contact_window():
    station = DockStation(x=2.0, y=1.0, yaw=math.pi / 2)
    assert station.in_contact(Pose(2.0, 1.0, math.pi / 2))
    assert station.in_contact(Pose(2.0, 1.0 - 0.05, math.pi / 2))      # 5 cm short
    assert not station.in_contact(Pose(2.0, 1.0 - 0.08, math.pi / 2))  # 8 cm short
    assert not station.in_contact(Pose(2.0 + 0.08, 1.0, math.pi / 2))  # 8 cm sideways
    assert not station.in_contact(Pose(2.0, 1.0, math.pi / 2 + 0.4))   # skewed


def test_the_station_body_stops_a_robot_just_past_the_docked_pose():
    station = DockStation(x=0.0, y=0.0, yaw=0.0)
    world = KinematicWorld(obstacles=(station.stop_obstacle(0.4),), robot_radius=0.4)
    world.pose = Pose(x=-0.5)
    for _ in range(400):                      # drive straight in, hard
        world.step(100.0, 100.0, 0.02)
    along, _, _ = station.offset_of(world.pose)
    assert world.collision
    assert 0.0 < along <= station.body_overrun + 1e-9
    assert station.in_contact(world.pose)


def test_quaternion_round_trip():
    for angle in (0.0, 0.7, math.pi, -2.0):
        m = rot_z(angle)
        back = matrix_from_quaternion(*quaternion_from_matrix(m))
        flat = [v for row in back for v in row]
        assert flat == pytest.approx([v for row in m for v in row], abs=1e-12)
