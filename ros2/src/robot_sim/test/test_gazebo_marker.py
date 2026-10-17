"""The REAL marker detector on RENDERED pixels — the first time track_marker_node
sees an image anywhere in this project. Skips without ROS or Gazebo.

    Gazebo (camera:=true, software rendering) ─► /camera/image_raw + camera_info
        ─► track_marker_node (image_proc, the node the robot runs) ─► /detected_dock_pose
    against
    dock_model.DockStation.tag_in_camera — the geometry the docking chain was
    built and tested on (test_marker_convention.py held it to the detector's
    convention with a hand-drawn tag; this holds it to a rendered one)

The station is spawned by sim_node from dock_pose (gazebo_obstacles.
station_model_sdf: a low body and an AprilTag 36h11 #0 board). The robot stays
at the origin facing east; the station is moved instead, one Gazebo world per
case: straight ahead, off to each side, turned each way, and further back
(decision 11: a mirrored or rotated texture is exact in exactly one of them).

What it does NOT prove: the real camera's lens, exposure, sunlight, motion
blur, a printed tag's flatness. Perfect intrinsics here (camera_info from the
renderer), and a 10 cm tag rendered at 640 x 480.

~40 s per world. CI stage `gazebo`.
"""

import math
import time

import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")

import numpy as np                                      # noqa: E402
import rclpy                                            # noqa: E402
from geometry_msgs.msg import PoseStamped               # noqa: E402
from tf2_ros import Buffer, TransformListener           # noqa: E402

from robot_sim.dock_model import (                      # noqa: E402
    CameraMount, DockStation, matrix_from_quaternion)
from robot_sim.world import Pose                        # noqa: E402

# (dock x, y, yaw): where base_link sits docked; the marker is 0.55 m beyond.
CASES = {
    "ahead":   (1.5, 0.0, 0.0),
    "left":    (1.5, 0.30, 0.0),
    "right":   (1.5, -0.30, 0.0),
    "turned+": (1.5, 0.0, 0.35),
    "turned-": (1.5, 0.0, -0.35),
    "far":     (2.5, 0.0, 0.0),
}


def mount_from_tf(node, timeout_s=20.0):
    """camera_optical_link in base_link, from robot_state_publisher — the
    same mount the detector's output is interpreted with."""
    from rclpy.time import Time
    buffer = Buffer()
    TransformListener(buffer, node)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if buffer.can_transform("base_link", "camera_optical_link", Time()):
            tf = buffer.lookup_transform("base_link", "camera_optical_link", Time())
            t, q = tf.transform.translation, tf.transform.rotation
            return CameraMount(translation=(t.x, t.y, t.z),
                               rotation=matrix_from_quaternion(q.x, q.y, q.z, q.w))
    raise AssertionError("no base_link -> camera_optical_link on /tf_static")


def detections(node, seconds):
    got = []
    node.create_subscription(PoseStamped, "/detected_dock_pose", got.append, 10)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    return got


# ⚠️ THE PLANAR POSE AMBIGUITY, measured: a small square marker has two poses
# that project almost alike, mirror images in tilt. With the station turned
# 0.35 rad one way the detector picks the right one; turned the other way it
# reports the station turned ~10 deg the WRONG way — 30 deg off, every frame,
# no jitter that a filter could average out. A docking controller steering on
# that drives in 30 deg skewed. xfail(strict): when it starts passing, this
# test fails and says so (the fix landed, update the docs).
AMBIGUOUS = pytest.mark.xfail(strict=True, reason=(
    "planar PnP ambiguity: 0.10 m marker, station turned +0.35 rad reads ~30 deg off"))


def cases():
    out = []
    for case in CASES:
        marks = [AMBIGUOUS] if case == "turned+" else []
        out.append(pytest.param(case, 0.10, marks=marks, id=f"{case}-0.10"))
    # Does a bigger marker help? The tag size is an SP6 decision, so measure it.
    for case in ("turned+", "far"):
        out.append(pytest.param(case, 0.20, id=f"{case}-0.20"))
    return out


@pytest.mark.parametrize("case, marker_size", cases())
def test_the_real_detector_reads_the_rendered_marker_where_the_geometry_says(
        gazebo_camera, case, marker_size):
    dock = CASES[case]
    gazebo_camera("-p", "dock_pose:=[{}, {}, {}]".format(*map(float, dock)),
                  marker_size=marker_size)
    node = rclpy.create_node(f"marker_probe_{case.replace('+', 'p').replace('-', 'm')}")
    try:
        mount = mount_from_tf(node)
        seen = detections(node, 15.0)
        assert seen, ("no /detected_dock_pose in 15 s — the detector never decoded the "
                      "rendered tag (texture mirrored? camera not rendering?)")
        assert seen[-1].header.frame_id == "camera_optical_link"
        expected = DockStation(*dock).tag_in_camera(Pose(0.0, 0.0, 0.0), mount)
        assert expected is not None, "the geometry says the camera cannot see this tag"
        e_pos, e_q = np.array(expected[0]), expected[1]
        p = seen[-1].pose
        d_pos = np.array([p.position.x, p.position.y, p.position.z])
        o = p.orientation
        a = np.array(matrix_from_quaternion(*e_q))
        b = np.array(matrix_from_quaternion(o.x, o.y, o.z, o.w))
        turn = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(a.T @ b) - 1) / 2))))
        error = float(np.linalg.norm(d_pos - e_pos))
        # The marker's normal seen from the camera, as a heading (deg): what
        # the docking controller steers on.
        def heading(m):
            return math.degrees(math.atan2(m[0, 2], -m[2, 2]))
        spread = np.ptp([heading(np.array(matrix_from_quaternion(
            s.pose.orientation.x, s.pose.orientation.y, s.pose.orientation.z,
            s.pose.orientation.w))) for s in seen])
        print(f"marker {case} ({marker_size} m): {len(seen)} detections, position error {error:.3f} m "
              f"(range {np.linalg.norm(e_pos):.2f} m, detected {d_pos.round(3)}, expected "
              f"{e_pos.round(3)}), rotation error {turn:.1f} deg, normal heading "
              f"{heading(b):.1f} vs {heading(a):.1f} deg (spread over the run {spread:.1f})")
        # Thresholds from the measured runs (docs/missions.md, "AprilTag"),
        # split by what each error means for docking:
        #   sideways (camera x) — where the dock is across the approach: mm;
        #   range — the black square reads SMALL as it shrinks in the frame
        #     (render filtering at ~20 px), so the dock reads further away:
        #     +2.8% at 1.75 m, +7.6% at 2.75 m for the 0.10 m tag, +2.1% for
        #     a 0.20 m one at 2.75 m. Same ratio in every axis: a scale.
        #   heading of the marker's normal — what the approach steers on.
        rng = float(np.linalg.norm(e_pos))
        assert abs(d_pos[0] - e_pos[0]) < 0.03, f"{case}: sideways {d_pos[0]:.3f} vs {e_pos[0]:.3f}"
        assert abs(np.linalg.norm(d_pos) / rng - 1.0) < 0.09, f"{case}: range {np.linalg.norm(d_pos):.3f} vs {rng:.3f}"
        assert abs(heading(b) - heading(a)) < 6.0, (
            f"{case}: the station reads turned {heading(b):.1f} deg, truly {heading(a):.1f}")
        assert turn < 12.0, f"{case}: rotation off by {turn:.1f} deg"
    finally:
        node.destroy_node()
