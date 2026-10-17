"""The charging station as the simulator sees it: where the robot must sit to
touch the contacts, where the marker is, and what the camera would report.

ROS-free, pure Python (the `unit` CI stage has no numpy).

    station seen from above (dock yaw = 0, robot drives east into it):

        staging ───1.5 m───►  base_link ─0.57─► body │ tag (0.70 ahead, on the
        pose                  when docked     stop    │  body's back edge, 0.275 m
                                                       │  up, facing the robot)

`DockStation.x/y/yaw` is the pose base_link has when the robot is DOCKED: the
same meaning opennav_docking gives a dock pose, which is why nav2.yaml's
external_detection_translation_x is minus the tag distance here.

What the camera reports is computed the way image_proc's track_marker_node
reports it (cv::aruco::estimatePoseSingleMarkers): the marker's pose in the
camera's OPTICAL frame, marker axes x = right as seen by someone facing it,
y = up, z = out of the marker towards that someone. opennav_docking's default
external_detection_rotation_* exist to undo exactly that convention, so the
simulator speaking it is what lets nav2.yaml carry the real robot's values.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from robot_sim.world import CircularObstacle, Pose

Vec = Tuple[float, float, float]
Mat = Tuple[Vec, Vec, Vec]          # row-major 3x3

# AprilTag 36h11: the black square is 8 cells across (6 data + the black
# border); the detector needs at least one more cell of white round it.
TAG_CELLS = 8
TAG_QUIET_CELLS = 1

# Pi Camera V2 (Sony IMX219): 62.2 x 48.8 deg field of view.
PI_CAMERA_V2_HFOV = math.radians(62.2)
PI_CAMERA_V2_VFOV = math.radians(48.8)


# ---- tiny 3D helpers ---------------------------------------------------------
def mat_mul(a: Mat, b: Mat) -> Mat:
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
                 for i in range(3))


def mat_vec(a: Mat, v: Sequence[float]) -> Vec:
    return tuple(sum(a[i][k] * v[k] for k in range(3)) for i in range(3))


def transpose(a: Mat) -> Mat:
    return tuple(tuple(a[j][i] for j in range(3)) for i in range(3))


def rot_z(angle: float) -> Mat:
    c, s = math.cos(angle), math.sin(angle)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def from_columns(x: Vec, y: Vec, z: Vec) -> Mat:
    return tuple((x[i], y[i], z[i]) for i in range(3))


def quaternion_from_matrix(m: Mat) -> Tuple[float, float, float, float]:
    """(x, y, z, w) of a proper rotation matrix."""
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
        return ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s,
                (m[1][0] - m[0][1]) / s, 0.25 * s)
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = 2.0 * math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2])
        return (0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s,
                (m[2][1] - m[1][2]) / s)
    if m[1][1] > m[2][2]:
        s = 2.0 * math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2])
        return ((m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s,
                (m[0][2] - m[2][0]) / s)
    s = 2.0 * math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1])
    return ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s,
            (m[1][0] - m[0][1]) / s)


def matrix_from_quaternion(x: float, y: float, z: float, w: float) -> Mat:
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


@dataclass(frozen=True)
class CameraMount:
    """camera_optical_link in base_link: what robot.urdf.xacro says, read off
    tf by sim_node so the simulator and the stack share one camera."""
    translation: Vec
    rotation: Mat
    hfov: float = PI_CAMERA_V2_HFOV
    vfov: float = PI_CAMERA_V2_VFOV
    min_range: float = 0.15       # (the whole-tag check below binds first)
    max_range: float = 4.0        # and is too few pixels beyond this
    # Beyond this angle between the tag's normal and the line of sight the
    # square is too foreshortened to decode.
    max_view_angle: float = math.radians(60.0)


@dataclass(frozen=True)
class DockStation:
    x: float
    y: float
    yaw: float
    # The marker, relative to the DOCKED base_link: this far ahead, this high.
    # ⚠️ Chosen, not guessed (docs/handoff.md "İstasyon etiketi", measured in
    # Gazebo with the real detector): a 0.20 m tag, because a 0.10 m one read
    # a station turned 20 deg as turned 10 deg the OTHER way from staging
    # (planar ambiguity) and a 0.15 m one still did; standing on the body's
    # back edge, because a body in front of the board hid its lower half; and
    # with the camera tilted only 0.09 rad down (robot.urdf.xacro), because at
    # 0.35 rad no size and height kept the tag in frame near the contacts.
    # SP6 builds it to these numbers — or changes them here and in nav2.yaml.
    tag_distance: float = 0.70
    tag_height: float = 0.275
    # The black square's edge (the detector's marker_size). The camera must
    # see all of it plus one cell of white round it (TAG_QUIET_CELLS) — the
    # detector finds the square by its edges, a cropped one is not found.
    tag_size: float = 0.20
    # Spring contacts: they close over the last `contact_travel` of approach and
    # tolerate `contact_overrun` past the nominal pose, `contact_lateral`
    # sideways (pad width) and `contact_yaw` of skew. GUESSES until SP6 builds
    # the station; the V-funnel in the design would widen the lateral figure.
    contact_travel: float = 0.06
    contact_overrun: float = 0.03
    contact_lateral: float = 0.06
    contact_yaw: float = 0.25
    # The station's body: the robot physically cannot go further than this
    # past the docked pose (see stop_obstacle).
    body_overrun: float = 0.02

    def offset_of(self, pose: Pose) -> Tuple[float, float, float]:
        """(along, lateral, yaw error) of a base pose in the dock's frame.
        `along` is negative short of the dock, positive past it."""
        dx, dy = pose.x - self.x, pose.y - self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        yaw_error = math.atan2(math.sin(pose.yaw - self.yaw), math.cos(pose.yaw - self.yaw))
        return c * dx + s * dy, -s * dx + c * dy, yaw_error

    def in_contact(self, pose: Pose) -> bool:
        along, lateral, yaw_error = self.offset_of(pose)
        return (-self.contact_travel <= along <= self.contact_overrun
                and abs(lateral) <= self.contact_lateral
                and abs(yaw_error) <= self.contact_yaw)

    def stop_obstacle(self, robot_radius: float, radius: float = 0.15) -> CircularObstacle:
        """The station body as a circle, placed so a robot driving straight in
        is stopped `body_overrun` past the docked pose. 0.30 m across: the
        design's V-funnel and contact plate, a GUESS until SP6 builds it.
        sim_node collides with it and, unless told otherwise, also draws it on
        the survey map (dock_in_survey)."""
        reach = robot_radius + radius + self.body_overrun
        return CircularObstacle(self.x + reach * math.cos(self.yaw),
                                self.y + reach * math.sin(self.yaw), radius)

    @property
    def tag_position(self) -> Vec:
        return (self.x + self.tag_distance * math.cos(self.yaw),
                self.y + self.tag_distance * math.sin(self.yaw),
                self.tag_height)

    @property
    def tag_rotation(self) -> Mat:
        """The marker's axes in the world (the aruco convention above)."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        normal = (-c, -s, 0.0)            # z: out of the marker, back at the robot
        up = (0.0, 0.0, 1.0)              # y
        right = (s, -c, 0.0)              # x = y cross z
        return from_columns(right, up, normal)

    def tag_in_camera(self, base: Pose, mount: CameraMount
                      ) -> Optional[Tuple[Vec, Tuple[float, float, float, float]]]:
        """The marker's pose in camera_optical_link, or None if the camera
        cannot see it from this base pose. Noise-free; the caller adds noise."""
        r_world_base = rot_z(base.yaw)
        r_world_cam = mat_mul(r_world_base, mount.rotation)
        cam = mat_vec(r_world_base, mount.translation)
        cam = (base.x + cam[0], base.y + cam[1], cam[2])
        r_cam_world = transpose(r_world_cam)
        tag = self.tag_position
        position = mat_vec(r_cam_world, (tag[0] - cam[0], tag[1] - cam[1], tag[2] - cam[2]))
        x, y, z = position
        distance = math.sqrt(x * x + y * y + z * z)
        if z <= 0.0 or not mount.min_range <= distance <= mount.max_range:
            return None
        if abs(math.atan2(x, z)) > mount.hfov / 2 or abs(math.atan2(y, z)) > mount.vfov / 2:
            return None
        rotation = mat_mul(r_cam_world, self.tag_rotation)
        # The marker's normal (its z) must face the camera: z_tag . (cam - tag) > 0.
        normal = tuple(rotation[i][2] for i in range(3))
        cos_view = -sum(n * p for n, p in zip(normal, position)) / distance
        if cos_view < math.cos(mount.max_view_angle):
            return None
        # ⚠️ The WHOLE square in the frame, not just its centre. Checking the
        # centre alone passed a 0.10 m tag 0.20 m up as visible "all the way
        # in" while the camera, pitched 20 deg down, cut its top edge off for
        # the last 0.3 m of the approach — exactly where the docking
        # controller needs a fresh detection every step.
        half = self.tag_size * (0.5 + TAG_QUIET_CELLS / TAG_CELLS)
        tan_h, tan_v = math.tan(mount.hfov / 2), math.tan(mount.vfov / 2)
        right = tuple(rotation[i][0] for i in range(3))
        up = tuple(rotation[i][1] for i in range(3))
        for u in (-half, half):
            for v in (-half, half):
                cx, cy, cz = (position[i] + u * right[i] + v * up[i] for i in range(3))
                if cz <= 0.0 or abs(cx) > tan_h * cz or abs(cy) > tan_v * cz:
                    return None
        return position, quaternion_from_matrix(rotation)
