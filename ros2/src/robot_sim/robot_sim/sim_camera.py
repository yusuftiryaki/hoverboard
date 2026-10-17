"""A cheap camera for the kinematic world: ground, sky, and the obstacles as
upright cylinders, ray-cast per pixel. ROS-free (numpy).

What it is FOR: the image PIPELINE — the bumper's snapshot, the web page's
live view — needs pictures to carry, and the kinematic world has no renderer.
What it is NOT: anything a vision algorithm may be judged on. Grass is a
procedural checker, rocks are grey cylinders, there is no lighting, no lens,
no motion blur. track_marker_node never sees these frames (the marker is
simulated geometrically, dock_model.py), and nothing may be calibrated or
tuned against them (handoff decision 10's spirit: a fake sensor proves the
wiring, never the physics).

Geometry is the real one, though: the camera's pose comes from the same tf the
stack uses (robot.urdf.xacro, read by sim_node), with the Pi Camera V2's field
of view, so what is in frame is what the real camera would have in frame.
"""

from __future__ import annotations

import math
from typing import Sequence, Tuple

import numpy as np

GRASS_A = np.array([46, 125, 50], dtype=np.float32)     # RGB
GRASS_B = np.array([67, 150, 64], dtype=np.float32)
SKY = np.array([170, 205, 235], dtype=np.float32)
ROCK = np.array([135, 132, 128], dtype=np.float32)


def render(width: int, height: int, hfov: float, mount_translation: Sequence[float],
           mount_rotation: Sequence[Sequence[float]], robot: Tuple[float, float, float],
           obstacles: Sequence[Tuple[float, float, float, float]]) -> np.ndarray:
    """An RGB uint8 image (height x width x 3).

    mount_*: camera_optical_link in base_link (x right, y down, z forward in
    the optical frame). robot: (x, y, yaw) of base_link in the world.
    obstacles: (x, y, radius, height) upright cylinders standing on the ground.
    """
    f = (width / 2.0) / math.tan(hfov / 2.0)
    us = (np.arange(width, dtype=np.float32) + 0.5 - width / 2.0) / f
    vs = (np.arange(height, dtype=np.float32) + 0.5 - height / 2.0) / f
    u, v = np.meshgrid(us, vs)
    rays = np.stack([u, v, np.ones_like(u)], axis=-1)                 # optical frame
    rotation = np.asarray(mount_rotation, dtype=np.float32)
    rays = rays @ rotation.T                                          # -> base_link
    x, y, yaw = robot
    c, s = math.cos(yaw), math.sin(yaw)
    turn = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float32)
    rays = rays @ turn.T                                              # -> world
    tx, ty, tz = mount_translation
    origin = np.array([x + c * tx - s * ty, y + s * tx + c * ty, tz], dtype=np.float32)

    np.seterr(invalid="ignore", over="ignore")    # inf * 0 on sky pixels, masked below
    dx, dy, dz = rays[..., 0], rays[..., 1], rays[..., 2]
    image = np.broadcast_to(SKY, rays.shape).copy()
    depth = np.full(u.shape, np.inf, dtype=np.float32)

    # Ground: z = 0, in front of the camera only.
    ground = dz < -1e-6
    t_ground = np.where(ground, -origin[2] / np.where(ground, dz, -1.0), np.inf)
    gx = origin[0] + t_ground * dx
    gy = origin[1] + t_ground * dy
    gx, gy = np.where(ground, gx, 0.0), np.where(ground, gy, 0.0)
    checker = ((np.floor(gx * 4.0) + np.floor(gy * 4.0)) % 2 == 0)
    grass = np.where(checker[..., None], GRASS_A, GRASS_B)
    # Fade to sky colour with distance, so the horizon reads as a horizon.
    fog = np.clip(t_ground / 40.0, 0.0, 1.0)[..., None]
    grass = grass * (1.0 - fog) + SKY * fog
    image = np.where(ground[..., None], grass, image)
    depth = np.where(ground, t_ground, depth)

    # Cylinders: the 2D ray-circle intersection, then the hit's height.
    a = dx * dx + dy * dy
    for ox, oy, radius, tall in obstacles:
        px, py = origin[0] - ox, origin[1] - oy
        b = 2.0 * (px * dx + py * dy)
        cc = px * px + py * py - radius * radius
        disc = b * b - 4.0 * a * cc
        ok = (disc >= 0.0) & (a > 1e-9)
        root = np.sqrt(np.where(ok, disc, 0.0))
        t_hit = (-b - root) / np.where(ok, 2.0 * a, 1.0)
        z_hit = origin[2] + t_hit * dz
        hit = ok & (t_hit > 0.0) & (z_hit >= 0.0) & (z_hit <= tall) & (t_hit < depth)
        # Shade by how squarely the surface faces the camera.
        hx = origin[0] + t_hit * dx - ox
        hy = origin[1] + t_hit * dy - oy
        facing = np.clip(-(hx * dx + hy * dy) / (radius * np.sqrt(a) + 1e-9), 0.2, 1.0)
        shade = ROCK * facing[..., None]
        image = np.where(hit[..., None], shade, image)
        depth = np.where(hit, t_hit, depth)
    return image.clip(0, 255).astype(np.uint8)
