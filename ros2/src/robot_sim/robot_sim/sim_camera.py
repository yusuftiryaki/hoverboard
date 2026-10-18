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

Learning mode (docs/learning.md, 6b) adds ONE thing a policy may learn from:
the lawn's edge. Ground inside a lawn polygon is grass, outside it soil, so
the edge is where the real one is (site_world.py, the simulator's truth). It
is structurally right and visually crude: a policy that learns it proves the
pipeline and the method, not the field. `palette()` varies the colours and
the checker per world (domain randomization) so a policy cannot key on one
exact green.

Geometry is the real one, though: the camera's pose comes from the same tf the
stack uses (robot.urdf.xacro, read by sim_node), with the Pi Camera V2's field
of view, so what is in frame is what the real camera would have in frame.
"""

from __future__ import annotations

import math
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

GRASS_A = np.array([46, 125, 50], dtype=np.float32)     # RGB
GRASS_B = np.array([67, 150, 64], dtype=np.float32)
SKY = np.array([170, 205, 235], dtype=np.float32)
ROCK = np.array([135, 132, 128], dtype=np.float32)
SOIL_A = np.array([118, 92, 62], dtype=np.float32)
SOIL_B = np.array([134, 108, 76], dtype=np.float32)
DEFAULT_PALETTE = {"grass_a": GRASS_A, "grass_b": GRASS_B, "soil_a": SOIL_A,
                   "soil_b": SOIL_B, "grass_cells": 4.0, "soil_cells": 10.0}


def palette(seed: int) -> Dict[str, np.ndarray]:
    """Colours and checker sizes for one world. 0 = the default palette."""
    if seed == 0:
        return dict(DEFAULT_PALETTE)
    rng = np.random.default_rng(seed)

    def jitter(colour, spread):
        return np.clip(colour * rng.uniform(0.75, 1.25) + rng.normal(0, spread, 3), 0, 255) \
            .astype(np.float32)

    return {"grass_a": jitter(GRASS_A, 10), "grass_b": jitter(GRASS_B, 10),
            "soil_a": jitter(SOIL_A, 14), "soil_b": jitter(SOIL_B, 14),
            "grass_cells": float(rng.uniform(2.0, 8.0)),
            "soil_cells": float(rng.uniform(5.0, 16.0))}


def render(width: int, height: int, hfov: float, mount_translation: Sequence[float],
           mount_rotation: Sequence[Sequence[float]], robot: Tuple[float, float, float],
           obstacles: Sequence[Tuple[float, float, float, float]], lawns: Sequence = (),
           colours: Optional[Dict[str, np.ndarray]] = None) -> np.ndarray:
    """An RGB uint8 image (height x width x 3).

    mount_*: camera_optical_link in base_link (x right, y down, z forward in
    the optical frame). robot: (x, y, yaw) of base_link in the world.
    obstacles: (x, y, radius, height) upright cylinders standing on the ground.
    lawns: shapely Polygons in the world; with any, ground outside them is
    soil. Without, all ground is grass (as before 6b). colours: palette().
    """
    colours = colours or DEFAULT_PALETTE
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
    cells = colours["grass_cells"]
    checker = ((np.floor(gx * cells) + np.floor(gy * cells)) % 2 == 0)
    grass = np.where(checker[..., None], colours["grass_a"], colours["grass_b"])
    if lawns:
        import shapely
        near = ground & (t_ground < 30.0)
        inside = np.zeros(u.shape, dtype=bool)
        for lawn in lawns:
            inside |= shapely.contains_xy(lawn, gx, gy)
        cells = colours["soil_cells"]
        fine = ((np.floor(gx * cells) + np.floor(gy * cells)) % 2 == 0)
        soil = np.where(fine[..., None], colours["soil_a"], colours["soil_b"])
        grass = np.where((near & ~inside)[..., None], soil, grass)
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
