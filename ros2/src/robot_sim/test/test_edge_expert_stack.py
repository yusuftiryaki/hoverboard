"""The privileged edge expert on a running stack, recorded (docs/learning.md, 6a).

SKIPs without ROS, ~2 min. The expert drives one lap of a 6 x 4 m lawn through
/cmd_vel_teleop — twist_mux, bridge, ESP32 simulator, kinematic world — while
learning mode records it. Then: the lap scored on the bag's /ground_truth with
the same metric a policy will get, and the episode exported with the expert's
label in its own column.
"""

import json
import os
import time

import numpy as np
import pytest

pytest.importorskip("rclpy", reason="ROS 2 not sourced")

import rclpy                                            # noqa: E402
from shapely.geometry import Polygon                    # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile      # noqa: E402
from std_msgs.msg import String                         # noqa: E402
from std_srvs.srv import Trigger                        # noqa: E402

from robot_sim.edge_expert import edge_metrics           # noqa: E402
from test_learning_stack import http, read_bag, wait_status   # noqa: E402
from test_web_stack import QUIET_GPS, polygon            # noqa: E402

CORNERS = [(-1, -1), (5, -1), (5, 3), (-1, 3)]          # sim_world metres
LAWN = polygon("area", "cim", CORNERS)
OFFSET = 0.4


def call(node, service, timeout=10.0):
    client = node.create_client(Trigger, service)
    assert client.wait_for_service(timeout_sec=timeout), f"{service} not up"
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    assert future.done() and future.result().success, future.result()
    return future.result()


def test_the_expert_drives_the_edge_and_the_episode_carries_its_label(learning_stack):
    stack = learning_stack([LAWN], *QUIET_GPS)
    stack._spawn("ros2", "run", "robot_sim", "edge_expert", "--ros-args",
                 "-p", f"site_file:={stack.site_file}", "-p", f"offset_m:={OFFSET}")
    wait_status(lambda s: s["robot"] is not None and s["episode"] is not None, 60.0,
                "the recorder's status")
    name = http("POST", "/api/episode/start", {"task": "kenar takibi (uzman)",
                                               "mode": "demo"})["episode"]
    wait_status(lambda s: s["episode"]["ready"], 30.0, "READY")

    node = rclpy.create_node("edge_expert_test")
    try:
        states = []
        node.create_subscription(
            String, "/edge_expert/status", lambda m: states.append(json.loads(m.data)),
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        call(node, "/edge_expert/start")
        deadline = time.monotonic() + 150.0
        while time.monotonic() < deadline and not (states and states[-1]["state"] == "done"):
            rclpy.spin_once(node, timeout_sec=0.5)
        assert states and states[-1]["state"] == "done", \
            f"the expert did not finish its lap: {states[-1] if states else None}"
    finally:
        node.destroy_node()
    http("POST", "/api/episode/stop", {"outcome": "success"})
    wait_status(lambda s: not s["episode"]["recording"], 15.0, "the end of REC")

    # The lap, scored on the recorded ground truth.
    bag = read_bag(os.path.join(stack.episodes_dir, name, "bag"))
    labels = [m for _, m in bag["/expert/cmd_vel"]]
    first = bag["/expert/cmd_vel"][0][0]
    last = bag["/expert/cmd_vel"][-1][0]
    poses = [(m.pose.pose.position.x, m.pose.pose.position.y)
             for t, m in bag["/ground_truth"] if first <= t <= last]
    lawn = Polygon(CORNERS)
    # From the first pose on the edge (the robot starts 1 m in, at (0, 0)).
    on = next(i for i, (x, y) in enumerate(poses)
              if abs(lawn.exterior.distance(__import__("shapely").geometry.Point(x, y))
                     - OFFSET) < 0.1)
    m = edge_metrics(lawn, OFFSET, poses[on:])
    print(f"\nMEASURED expert lap (stack): {json.dumps({k: round(v, 3) for k, v in m.items()})}")
    assert m["error_mean"] < 0.06 and m["error_p95"] < 0.2, m
    assert m["outside"] == 0.0 and m["reached"] > 0.9, m
    assert max(l.linear.x for l in labels) == pytest.approx(0.3, abs=1e-6)

    # Exported: the expert drove (teleop) and labelled — the same command.
    from robot_learning.export import export
    out = os.path.join(stack.episodes_dir, "..", "dataset")
    (item,) = export([stack.episodes_dir], out, fps=10.0)["episodes"]
    data = np.load(os.path.join(out, item["data"]))
    labelled = ~np.isnan(data["action_expert"][:, 0])
    print(f"MEASURED export: {item['frames']} frames, expert labels on {labelled.mean():.0%}, "
          f"teleop {item['teleop_fraction']:.0%}")
    assert labelled.mean() > 0.8
    # The expert publishes the same command on two topics back to back; their
    # arrival times differ by ms, so where the command CHANGES at a frame
    # boundary one may land a frame later than the other (seen in CI under
    # load). Everywhere else they are the same command.
    both = labelled & data["teleop"]
    same = np.all(np.isclose(data["action"], data["action_expert"]), axis=1)
    assert same[both].mean() > 0.95, f"label and expert agree on {same[both].mean():.0%}"
    changed = np.r_[True, np.any(np.diff(data["action_expert"], axis=0) != 0, axis=1)]
    near_change = changed | np.r_[changed[1:], True] | np.r_[True, changed[:-1]]
    assert near_change[both & ~same].all(), "a disagreement away from a command change"
