"""The kinematic world's cheap camera, against hand-worked geometry. ROS-free."""

import math

import numpy as np
import pytest

from robot_sim.sim_camera import render

W, H = 320, 240
HFOV = math.radians(62.2)
F = (W / 2) / math.tan(HFOV / 2)            # 265.0 px


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


# A camera pitched 0.35 rad down (robot.urdf.xacro's old tilt — the renderer
# is tested on its own mount, any tilt will do), the optical frame turned
# (-pi/2, 0, -pi/2) from the link (z forward, x right, y down).
MOUNT_R = rpy(0, 0.35, 0) @ rpy(-math.pi / 2, 0, -math.pi / 2)
MOUNT_T = (0.3, 0.0, 0.35)


def frame(robot=(0.0, 0.0, 0.0), obstacles=()):
    return render(W, H, HFOV, MOUNT_T, MOUNT_R, robot, obstacles).astype(int)


def is_sky(px):
    return px[2] > px[1]


def test_the_horizon_sits_where_the_pitch_puts_it():
    # 0.35 rad down: the horizon is F * tan(0.35) = 96.6 px above the centre,
    # at row 120 - 96.6 = 23.4. Above it sky, below it grass.
    img = frame()
    assert is_sky(img[15, 160]) and is_sky(img[5, 40])
    assert not is_sky(img[30, 160]) and not is_sky(img[200, 300])


def expected_column(x, y, z):
    """Pinhole column of a base_link point, derived by hand (not through the
    renderer's ray maths): into the camera link (pitch p about y, camera
    0.3 m ahead and 0.35 m up), then optical x = -y_link, z = x_link."""
    p = 0.35
    x, z = x - MOUNT_T[0], z - MOUNT_T[2]
    forward = math.cos(p) * x - math.sin(p) * z
    return W / 2 - F * y / forward


@pytest.mark.parametrize("bearing_deg", [-18, -10, 0, 10, 18])
def test_a_rock_appears_where_its_bearing_says(bearing_deg):
    # A grey rock 2 m away at a bearing (left = positive, REP-103), its centre
    # column by hand at mid-height (0.2 m). Swept both sides (decision 11): a
    # mirrored image is exact at 0 deg.
    b = math.radians(bearing_deg)
    rx, ry = 2.0 * math.cos(b), 2.0 * math.sin(b)
    img = frame(obstacles=[(rx, ry, 0.15, 0.4)])
    grey = (np.abs(img[..., 0] - img[..., 1]) < 12) & (np.abs(img[..., 1] - img[..., 2]) < 12)
    cols = np.nonzero(grey.any(axis=0))[0]
    assert len(cols) and cols.min() > 0 and cols.max() < W - 1, "rock clipped or missing"
    assert abs((cols.min() + cols.max()) / 2 - expected_column(rx, ry, 0.2)) < 4


def test_a_rock_behind_is_not_seen_and_turning_round_sees_it():
    rock = [(-2.0, 0.0, 0.3, 0.4)]
    grey_count = lambda img: int(((np.abs(img[..., 0] - img[..., 1]) < 12)
                                  & (np.abs(img[..., 1] - img[..., 2]) < 12)).sum())
    assert grey_count(frame(obstacles=rock)) == 0
    assert grey_count(frame(robot=(0.0, 0.0, math.pi), obstacles=rock)) > 500


def test_the_frame_is_rgb_uint8_of_the_asked_size():
    img = render(64, 48, HFOV, MOUNT_T, MOUNT_R, (0, 0, 0), [])
    assert img.shape == (48, 64, 3) and img.dtype == np.uint8


# --- the lawn's edge (docs/learning.md 6b) ---------------------------------------

def is_grass(px):
    return px[1] > px[0] + 15          # green over red


def is_soil(px):
    return px[0] > px[1]                # red over green: brown


def test_the_lawn_edge_is_where_the_geometry_puts_it():
    from shapely.geometry import box
    # The centre row looks 0.35 rad down from 0.35 m: it meets the ground
    # 0.35 / tan(0.35) = 0.957 m ahead of the camera, which is 0.3 m ahead of
    # base_link — world x = 1.257. An edge there splits the frame at row 120:
    # nearer rows (below) grass, farther rows (above, under the horizon) soil.
    edge = 0.3 + 0.35 / math.tan(0.35)
    lawn = box(-10.0, -10.0, edge, 10.0)
    image = render(W, H, HFOV, MOUNT_T, MOUNT_R, (0.0, 0.0, 0.0), (), [lawn]).astype(int)
    col = W // 2
    assert all(is_grass(image[row, col]) for row in range(125, H, 10))
    assert all(is_soil(image[row, col]) for row in range(40, 116, 10))
    # Turned round, the camera looks over the lawn only.
    image = render(W, H, HFOV, MOUNT_T, MOUNT_R, (0.0, 0.0, math.pi), (), [lawn]).astype(int)
    assert all(is_grass(image[row, col]) for row in range(40, H, 10))


def test_without_lawns_all_ground_is_grass_as_before():
    image = frame()
    assert all(is_grass(image[row, W // 2]) for row in range(40, H, 10))


def test_the_palette_varies_per_world_but_is_repeatable():
    from robot_sim.sim_camera import DEFAULT_PALETTE, palette
    assert palette(0)["grass_a"].tolist() == DEFAULT_PALETTE["grass_a"].tolist()
    a, b = palette(7), palette(8)
    assert a["grass_a"].tolist() == palette(7)["grass_a"].tolist()
    assert a["grass_a"].tolist() != b["grass_a"].tolist()
    for seed in range(1, 50):                 # grass stays green, soil stays brown
        p = palette(seed)
        assert p["grass_a"][1] > p["grass_a"][0] and p["soil_a"][0] > p["soil_a"][2]
