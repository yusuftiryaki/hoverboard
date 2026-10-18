"""SP3: the REAL marker detector agrees with the simulator's fake one.

robot_sim/dock_model.py publishes /detected_dock_pose the way it believes
image_proc's track_marker_node does, and nav2.yaml's external_detection_*
values are tuned against that belief. If the belief is wrong — an axis flipped,
the yaw sign reversed — every docking test stays green and the real robot
turns the wrong way at the station. A6 again: the fake agreeing with itself.

So here the real detector looks at a picture. The picture is drawn BY HAND
from physics — an upright tag on a wall, its face turned so many degrees,
projected through a pinhole — never from dock_model's rotation. Then
track_marker_node's answer is compared with dock_model's for the same scene.

Same parameters as sensors.launch.py's dock_marker node. Spawns one process,
~15 s. SKIPs without ROS, or without OpenCV's aruco.
"""

import math
import os
import signal
import subprocess
import time

import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")
cv2 = pytest.importorskip("cv2", reason="python3-opencv missing")
np = pytest.importorskip("numpy")

import rclpy                                          # noqa: E402
from geometry_msgs.msg import PoseStamped             # noqa: E402
from rclpy.node import Node                           # noqa: E402
from sensor_msgs.msg import CameraInfo, Image         # noqa: E402

from robot_sim.dock_model import CameraMount, DockStation   # noqa: E402
from robot_sim.world import Pose                      # noqa: E402

WIDTH, HEIGHT = 640, 480
FX = FY = 500.0
CX, CY = (WIDTH - 1) / 2.0, (HEIGHT - 1) / 2.0
MARKER_SIZE = 0.20            # sensors.launch.py
DICTIONARY = 20               # cv::aruco::DICT_APRILTAG_36h11
RANGE = 0.5                   # camera to tag centre, m

LEVEL_OPTICAL = ((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0))
CAMERA = CameraMount(translation=(0.30, 0.0, 0.20), rotation=LEVEL_OPTICAL)


def draw_tag(turn):
    """A tag RANGE metres straight ahead, upright, its face turned `turn` rad
    about the vertical — positive turns its normal towards the camera's right
    (optical +x). Corners from geometry, not from any marker convention."""
    normal = np.array([math.sin(turn), 0.0, -math.cos(turn)])   # towards viewer side
    up = np.array([0.0, -1.0, 0.0])                             # optical y is down
    right = np.cross(-normal, up)       # the right of someone facing the tag
    centre = np.array([0.0, 0.0, RANGE])
    h = MARKER_SIZE / 2.0
    corners3d = [centre - right * h + up * h, centre + right * h + up * h,
                 centre + right * h - up * h, centre - right * h - up * h]
    dst = np.array([[FX * p[0] / p[2] + CX, FY * p[1] / p[2] + CY] for p in corners3d],
                   dtype=np.float32)
    side = 600
    marker = cv2.aruco.drawMarker(cv2.aruco.getPredefinedDictionary(DICTIONARY), 0, side)
    src = np.array([[-0.5, -0.5], [side - 0.5, -0.5], [side - 0.5, side - 0.5],
                    [-0.5, side - 0.5]], dtype=np.float32)
    warp = cv2.getPerspectiveTransform(src, dst)
    image = cv2.warpPerspective(marker, warp, (WIDTH, HEIGHT), flags=cv2.INTER_AREA,
                                borderMode=cv2.BORDER_CONSTANT, borderValue=255)
    return image


def expected_from_dock_model(turn):
    """The same scene in dock_model's terms: robot at the origin facing east,
    tag RANGE ahead of its camera, station yawed by `turn`. A station yawed by
    +a has its marker's normal along world (-cos a, -sin a, 0), which is
    optical (sin a, 0, -cos a) — draw_tag's normal for the same `turn`.
    Checked the other way round too: feeding -turn here fails both oblique
    cases by 60.7 deg."""
    tag_x = CAMERA.translation[0] + RANGE
    station = DockStation(x=tag_x - 0.55 * math.cos(turn), y=-0.55 * math.sin(turn),
                          yaw=turn, tag_distance=0.55, tag_height=CAMERA.translation[2],
                          tag_size=MARKER_SIZE)
    return station.tag_in_camera(Pose(), CAMERA)


class Camera(Node):
    def __init__(self):
        super().__init__("marker_convention_camera")
        self.image_pub = self.create_publisher(Image, "/camera/image_raw", 10)
        self.info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", 10)
        self.poses = []
        self.create_subscription(PoseStamped, "/detected_dock_pose", self.poses.append, 10)

    def show(self, gray, seconds):
        self.poses.clear()
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            stamp = self.get_clock().now().to_msg()
            image = Image(height=HEIGHT, width=WIDTH, encoding="mono8", step=WIDTH,
                          data=gray.tobytes())
            image.header.stamp, image.header.frame_id = stamp, "camera_optical_link"
            info = CameraInfo(height=HEIGHT, width=WIDTH, distortion_model="plumb_bob",
                              d=[0.0] * 5,
                              k=[FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0],
                              p=[FX, 0.0, CX, 0.0, 0.0, FY, CY, 0.0, 0.0, 0.0, 1.0, 0.0])
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.header = image.header
            self.image_pub.publish(image)
            self.info_pub.publish(info)
            end_spin = time.monotonic() + 0.1
            while time.monotonic() < end_spin:
                rclpy.spin_once(self, timeout_sec=0.02)
        return list(self.poses)


@pytest.fixture(scope="module")
def detector(ros):
    proc = subprocess.Popen(
        ["ros2", "run", "image_proc", "track_marker_node", "--ros-args",
         "-r", "__node:=dock_marker",
         "-p", f"dictionary:={DICTIONARY}", "-p", "marker_id:=0",
         "-p", f"marker_size:={MARKER_SIZE}",
         "-r", "image:=camera/image_raw", "-r", "camera_info:=camera/camera_info",
         "-r", "tracked_pose:=detected_dock_pose"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    node = Camera()
    try:
        deadline = time.monotonic() + 20.0
        while not node.count_publishers("/detected_dock_pose") and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.show(draw_tag(0.0), 2.0)      # let the lazy subscription form
        yield node
    finally:
        node.destroy_node()
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        proc.wait(timeout=15)


def angle_between(q1, q2):
    dot = abs(sum(a * b for a, b in zip(q1, q2)))
    return 2.0 * math.acos(min(1.0, dot))


@pytest.mark.parametrize("turn_deg", [0, 30, -30])
def test_the_real_detector_reports_what_the_fake_one_does(detector, turn_deg):
    turn = math.radians(turn_deg)
    poses = detector.show(draw_tag(turn), 2.0)
    assert poses, "track_marker_node decoded nothing from a clean, upright tag"
    got = poses[-1].pose
    position = (got.position.x, got.position.y, got.position.z)
    quaternion = (got.orientation.x, got.orientation.y, got.orientation.z,
                  got.orientation.w)
    want_position, want_quaternion = expected_from_dock_model(turn)

    assert position == pytest.approx(want_position, abs=0.01), (
        f"detector says {position}, dock_model {want_position}")
    error = math.degrees(angle_between(quaternion, want_quaternion))
    assert error < 3.0, (
        f"turned {turn_deg} deg: the real detector's marker orientation is "
        f"{error:.1f} deg from the simulator's — nav2.yaml's "
        f"external_detection_rotation_* are tuned to the simulator's")
