"""edge_batch end to end, one lawn (docs/learning.md, 6b). SKIPs without ROS, ~2 min.

The batch tool spawns its own simulator and stack, the expert drives a lap of
a random lawn, learning mode records it — then the episode is exported and
the CAMERA is checked: the lawn's edge is on it (soil beside the grass), which
is the whole point of 6b. And nothing is left running afterwards.
"""

import json
import os
import subprocess

import numpy as np
import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")

import cv2                                              # noqa: E402
import yaml                                             # noqa: E402

from conftest import stray_stack_pids                   # noqa: E402

SEED = 8                     # the smallest of the first lawns: a ~16 m lap


def test_one_random_lawn_is_driven_recorded_and_its_edge_is_on_camera(ros, tmp_path):
    assert not stray_stack_pids(), "a stack is already running"
    out = tmp_path / "episodes"
    run = subprocess.run(["ros2", "run", "robot_sim", "edge_batch", "--lawns", "1",
                          "--seed", str(SEED), "--directions", "ccw", "--gps", "quiet",
                          "--out", str(out)], capture_output=True, text=True, timeout=600)
    print(run.stdout[-2000:], run.stderr[-2000:])
    assert run.returncode == 0, run.stderr[-2000:]
    assert not stray_stack_pids(), "edge_batch left processes running"

    (name,) = os.listdir(out)
    with open(out / name / "episode.yaml", encoding="utf-8") as handle:
        episode = yaml.safe_load(handle)
    note = json.loads(episode["note"])
    assert episode["outcome"] == "success" and episode["source"] == "sim"
    assert (note["seed"], note["direction"], note["gps"]) == (SEED, "ccw", "quiet")
    assert note["lawn_wkt"].startswith("POLYGON")

    from robot_learning.export import export
    dataset = tmp_path / "dataset"
    (item,) = export([str(out)], str(dataset), fps=5.0)["episodes"]
    video = cv2.VideoCapture(str(dataset / item["video"]))
    soil_frames, frames = 0, 0
    while True:
        ok, frame = video.read()
        if not ok:
            break
        frames += 1
        b, g, r = (frame[frame.shape[0] // 2:].reshape(-1, 3).astype(int).T)
        soil = np.mean((r > g) & (g > b))           # brown: red over green over blue
        soil_frames += soil > 0.02
    print(f"\nMEASURED batch: {item['frames']} frames, soil in view on "
          f"{soil_frames}/{frames}, expert labels "
          f"{np.mean(~np.isnan(np.load(dataset / item['data'])['action_expert'][:, 0])):.0%}")
    assert frames == item["frames"]
    # Driving 0.4 m inside the edge, the edge is in view most of the lap.
    assert soil_frames > 0.5 * frames, f"the edge was on camera in {soil_frames}/{frames}"
