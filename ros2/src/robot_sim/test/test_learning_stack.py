"""Learning mode on a running stack (docs/learning.md, 5c). SKIPs without ROS, ~2 min.

The page starts an episode, the robot is driven from the page, a marker is
set, the episode is stopped — then the BAG is opened and checked for what a
learning algorithm will need: the action labels (who wanted what, what reached
the bridge), the state, the camera as JPEG with the raw frame's stamp, the
marker, the simulator's answer key, and episode.yaml's context.

And the crash: the recording process is SIGKILLed mid-episode (a power cut,
an OOM kill). The episode must end as "crashed", and `reindex` must bring
back the splits that had closed.
"""

import io
import json
import math
import os
import signal
import subprocess
import time
import urllib.error
import urllib.request
import zipfile

import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")

import rosbag2_py                                       # noqa: E402
import yaml                                             # noqa: E402
from rclpy.serialization import deserialize_message     # noqa: E402
from rosidl_runtime_py.utilities import get_message     # noqa: E402

from conftest import LearningStack                      # noqa: E402
from test_web_stack import LAWN, QUIET_GPS              # noqa: E402

BASE = f"http://127.0.0.1:{LearningStack.WEB_PORT}"
CAMERA = ("-p", "camera_rate_hz:=5.0")


def http(method, path, body=None, timeout=30, raw=False):
    data = None if body is None else json.dumps(body).encode()
    request = urllib.request.Request(BASE + path, data=data, method=method,
                                     headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            payload = response.read()
            return payload if raw else json.loads(payload)
    except urllib.error.HTTPError as exc:
        raise AssertionError(f"{method} {path} -> {exc.code}: {exc.read().decode()}") from None


def wait_status(predicate, timeout, what):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        try:
            last = http("GET", "/api/status")
            if predicate(last):
                return last
        except (OSError, AssertionError):
            pass
        time.sleep(0.5)
    raise AssertionError(f"{what} within {timeout} s; last status {last}")


def drive(seconds, linear=0.5, angular=0.0):
    token = http("POST", "/api/teleop/acquire", {})["token"]
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        http("POST", "/api/teleop", {"token": token, "linear": linear, "angular": angular})
        time.sleep(0.1)
    http("POST", "/api/teleop/release", {"token": token})


def read_bag(uri):
    """{topic: [(bag time ns, message)]}"""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = {}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        out.setdefault(topic, []).append((stamp, deserialize_message(data, get_message(types[topic]))))
    return out


def episode_yaml(stack, name):
    with open(os.path.join(stack.episodes_dir, name, "episode.yaml"), encoding="utf-8") as f:
        return yaml.safe_load(f)


def test_an_episode_from_the_page_holds_what_learning_needs(learning_stack):
    stack = learning_stack([LAWN], *QUIET_GPS, *CAMERA)
    wait_status(lambda s: s["robot"] is not None and s["episode"] is not None, 60.0,
                "the recorder's status")
    name = http("POST", "/api/episode/start",
                {"task": "Kenar dibi biçme", "mode": "demo", "note": "test"})["episode"]
    began = time.monotonic()
    wait_status(lambda s: s["episode"]["recording"] and s["episode"]["ready"], 30.0,
                "READY (the recorder subscribed to every live topic)")
    ready_after = time.monotonic() - began
    drive(3.0)
    http("POST", "/api/episode/mark", {"label": "ağaç başladı"})
    drive(2.0, linear=0.0, angular=0.5)
    time.sleep(1.0)
    answer = http("POST", "/api/episode/stop", {"outcome": "success", "note": "temiz"})
    assert answer["episode"] == name
    status = wait_status(lambda s: not s["episode"]["recording"], 10.0, "the end of REC")
    assert status["episode"]["last_error"] is None

    bag = read_bag(os.path.join(stack.episodes_dir, name, "bag"))
    counts = {topic: len(messages) for topic, messages in bag.items()}
    print(f"\nMEASURED episode topics: {json.dumps(counts, sort_keys=True)}")

    # The action labels, three ways, and who held the wheels (learning.md decision 3).
    teleop = [m for _, m in bag["/cmd_vel_teleop"]]
    assert max(m.linear.x for m in teleop) == pytest.approx(0.3, abs=1e-6)
    assert max(m.angular.z for m in teleop) == pytest.approx(0.5, abs=1e-6)
    assert any(m.linear.x == pytest.approx(0.3) for _, m in bag["/cmd_vel"]), \
        "what reached the bridge is not in the bag"
    states = [json.loads(m.data) for _, m in bag["/teleop/state"]]
    assert any(s["active"] for s in states) and any(not s["held"] for s in states)

    # State and the answer key.
    for topic in ("/odom", "/joint_states", "/gps/filtered", "/odometry/filtered/local",
                  "/tf", "/ground_truth"):
        assert counts.get(topic, 0) >= 20, f"{topic}: {counts.get(topic)}"
    for latched in ("/tf_static", "/robot_description"):
        assert counts.get(latched, 0) >= 1, f"{latched} (latched) missing"
    # READY means the WHOLE drive is in: the first pose is where it began.
    start = bag["/ground_truth"][0][1].pose.pose.position
    end = bag["/ground_truth"][-1][1].pose.pose.position
    print(f"MEASURED ready after {ready_after:.1f} s (page), "
          f"first ground truth at x={start.x:.3f}")
    assert math.hypot(start.x, start.y) < 0.02, "the bag starts after the drive did"
    assert math.hypot(end.x - start.x, end.y - start.y) > 0.8, "the drive is not in the bag"

    # The marker, in the bag AND in episode.yaml.
    assert [m.data for _, m in bag["/learning/marker"]] == ["ağaç başladı"]

    # The camera: JPEG, decodable, stamped by the raw frame, ~5 Hz.
    import cv2
    import numpy as np
    jpegs = bag["/camera/jpeg"]
    assert len(jpegs) >= 20, f"{len(jpegs)} camera frames in ~7 s at 5 Hz"
    frame = cv2.imdecode(np.frombuffer(bytes(jpegs[0][1].data), np.uint8), cv2.IMREAD_COLOR)
    assert frame is not None and frame.shape[2] == 3
    stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for _, m in jpegs]
    assert all(b > a for a, b in zip(stamps, stamps[1:])), "camera stamps must increase"
    # The stamp is the RAW frame's (when the light hit the sensor): never
    # newer than the JPEG's arrival, and older by render + encode + transport.
    # sim_camera ray-traces in Python, so that age is large here (0.6 s seen
    # under load); a camera_ros frame on the Pi should be far fresher.
    ages = [t * 1e-9 - s for (t, _), s in zip(jpegs, stamps)]
    assert min(ages) >= 0.0, "a JPEG stamped after it arrived: restamped, not copied"
    assert max(ages) < 2.0, f"a JPEG's stamp is {max(ages):.2f} s older than its arrival"
    assert "/camera/image_raw" not in bag

    # The snapshot (site copy, `ros2 param dump`s) runs beside the episode.
    deadline = time.monotonic() + 60.0
    while episode_yaml(stack, name)["snapshot"] is None and time.monotonic() < deadline:
        time.sleep(0.5)
    episode = episode_yaml(stack, name)
    assert (episode["task"], episode["mode"], episode["outcome"], episode["source"]) == \
        ("Kenar dibi biçme", "demo", "success", "sim")
    assert episode["markers"][0]["label"] == "ağaç başladı"
    assert 0 < episode["markers"][0]["t"] < episode["duration_s"]
    assert episode["recorder_exit"] == 0
    assert 0 < episode["ready_after_s"] < 30
    snapshot = episode["snapshot"]
    assert snapshot["site"] == "site.geojson"
    assert "hoverboard_bridge" in snapshot["params"] and "twist_mux" in snapshot["params"]
    with open(os.path.join(stack.episodes_dir, name, "params", "hoverboard_bridge.yaml")) as f:
        assert "max_linear_vel" in f.read()

    # The page lists it and hands it out as a zip.
    listed = http("GET", "/api/episodes")["episodes"]
    assert listed[0]["name"] == name and listed[0]["markers"] == 1
    archive = zipfile.ZipFile(io.BytesIO(http("GET", f"/api/episodes/{name}.zip", raw=True)))
    assert f"{name}/bag/metadata.yaml" in archive.namelist()
    assert f"{name}/episode.yaml" in archive.namelist()


def test_a_killed_recorder_ends_the_episode_and_reindex_recovers_the_closed_splits(
        learning_stack):
    stack = learning_stack([LAWN], *QUIET_GPS)
    wait_status(lambda s: s["robot"] is not None and s["episode"] is not None, 60.0,
                "the recorder's status")
    name = http("POST", "/api/episode/start", {"task": "kaza", "mode": "demo"})["episode"]
    # From READY: before it the recorder is still subscribing (seconds, more
    # under load), and the splits it has not written cannot be recovered.
    wait_status(lambda s: s["episode"]["recording"] and s["episode"]["ready"], 30.0, "READY")
    time.sleep(3 * LearningStack.SPLIT_S + 1.0)       # three splits close
    pids = subprocess.run(["pgrep", "-f", "bin/ros2 bag record"], capture_output=True,
                          text=True).stdout.split()
    assert len(pids) == 1, f"expected one recorder, found {pids}"
    os.kill(int(pids[0]), signal.SIGKILL)             # a power cut, as far as the bag knows

    status = wait_status(lambda s: not s["episode"]["recording"], 15.0,
                         "the recorder noticing")
    assert "exited" in status["episode"]["last_error"]
    episode = episode_yaml(stack, name)
    assert episode["outcome"] == "crashed" and episode["recorder_exit"] == -signal.SIGKILL

    bag_dir = os.path.join(stack.episodes_dir, name, "bag")
    assert not os.path.exists(os.path.join(bag_dir, "metadata.yaml")), \
        "a killed recorder wrote no metadata — reindex is what the docs promise"
    splits = sorted(f for f in os.listdir(bag_dir) if f.endswith(".mcap"))
    assert len(splits) >= 3, splits
    rosbag2_py.Reindexer().reindex(rosbag2_py.StorageOptions(uri=bag_dir, storage_id="mcap"))
    bag = read_bag(bag_dir)
    odom = bag.get("/odom", [])
    span = (odom[-1][0] - odom[0][0]) * 1e-9 if odom else 0.0
    print(f"\nMEASURED crash: {len(splits)} splits, {len(odom)} /odom recovered over "
          f"{span:.1f} s (recorded ~{3 * LearningStack.SPLIT_S + 1} s)")
    assert span >= 2 * LearningStack.SPLIT_S - 1.0, \
        f"only {span:.1f} s recovered; the closed splits should give back at least two"

    # And the robot was never the recorder's to stop: a new episode starts.
    http("POST", "/api/episode/start", {"task": "sonraki", "mode": "demo"})
    wait_status(lambda s: s["episode"]["recording"], 10.0, "REC again")
    http("POST", "/api/episode/stop", {"outcome": "aborted"})
