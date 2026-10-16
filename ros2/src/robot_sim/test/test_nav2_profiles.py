"""Nav2's two driving profiles, held to the rules that make them work. No ROS.

nav2.yaml has a general profile (FollowPath + goal_checker, loose for GPS
wander) and a coverage one (FollowRow + precise_goal_checker, which
mission_runner calls straight on controller_server's follow_path). Each rule below broke
something real, silently:
  * an EMPTY goal_checker_id with two goal checkers fails FollowPath outright
    (controller_server.cpp findGoalCheckerId) — the stock trees leave it empty;
  * a goal tolerance at or above the controller's min lookahead locks RPP into
    rotate-to-heading forever (nav2.yaml, goal_checker comment);
  * bt_navigator still on the stock trees would hit the first rule on every goal.
"""

import os
import re
import xml.etree.ElementTree as ET

import pytest
import yaml

BRINGUP = os.path.join(os.path.dirname(__file__), "..", "..", "robot_bringup")
TREES = os.path.join(BRINGUP, "behavior_trees")
with open(os.path.join(BRINGUP, "config", "nav2.yaml"), encoding="utf-8") as handle:
    NAV2 = yaml.safe_load(handle)
CONTROLLER = NAV2["controller_server"]["ros__parameters"]

# tree -> (controller, goal checker) it must drive with
PROFILES = {
    "navigate_to_pose.xml": ("{selected_controller}", "goal_checker"),
    "navigate_through_poses.xml": ("{selected_controller}", "goal_checker"),
}
PAIRS = {"FollowPath": "goal_checker", "FollowRow": "precise_goal_checker"}


@pytest.mark.parametrize("tree", sorted(PROFILES))
def test_every_follow_path_names_its_controller_and_goal_checker(tree):
    root = ET.parse(os.path.join(TREES, tree)).getroot()
    nodes = list(root.iter("FollowPath"))
    assert nodes, f"{tree} has no FollowPath"
    controller, checker = PROFILES[tree]
    for node in nodes:
        assert node.get("goal_checker_id") == checker, (
            f"{tree}: FollowPath goal_checker_id={node.get('goal_checker_id')!r} — with two "
            "goal checkers an empty one fails every goal")
        assert node.get("controller_id") == controller


def test_every_named_plugin_exists():
    assert set(PAIRS) == set(CONTROLLER["controller_plugins"])
    assert set(PAIRS.values()) == set(CONTROLLER["goal_checker_plugins"])


@pytest.mark.parametrize("controller, checker", sorted(PAIRS.items()))
def test_goal_tolerance_stays_below_the_min_lookahead(controller, checker):
    tolerance = CONTROLLER[checker]["xy_goal_tolerance"]
    lookahead = CONTROLLER[controller]["min_lookahead_dist"]
    assert tolerance < lookahead, (
        f"{checker} xy_goal_tolerance {tolerance} >= {controller} min_lookahead_dist "
        f"{lookahead}: RPP's carrot is always 'at the goal' and the robot only rotates")


def test_bt_navigator_runs_our_trees_not_the_stock_ones():
    with open(os.path.join(BRINGUP, "launch", "nav2.launch.py"), encoding="utf-8") as handle:
        launch = handle.read()
    for key, tree in (("default_nav_to_pose_bt_xml", "navigate_to_pose.xml"),
                      ("default_nav_through_poses_bt_xml", "navigate_through_poses.xml")):
        assert re.search(rf'"{key}":\s*\n?\s*os\.path\.join\(trees,\s*"{tree}"\)', launch), key


def test_the_coverage_profile_is_the_tighter_one():
    assert (CONTROLLER["precise_goal_checker"]["xy_goal_tolerance"]
            < CONTROLLER["goal_checker"]["xy_goal_tolerance"])
    assert CONTROLLER["FollowRow"]["min_lookahead_dist"] < CONTROLLER["FollowPath"]["min_lookahead_dist"]


def test_mission_runner_asks_for_profiles_that_exist():
    # mission_runner sends rows to follow_path by NAME; a renamed controller or
    # checker would fail every row with INVALID_CONTROLLER and skip the lawn.
    source = os.path.join(BRINGUP, "..", "robot_missions", "robot_missions", "mission_runner.py")
    with open(source, encoding="utf-8") as handle:
        text = handle.read()
    controller = re.search(r'declare_parameter\("row_controller", "(\w+)"\)', text).group(1)
    checker = re.search(r'declare_parameter\("row_goal_checker", "(\w+)"\)', text).group(1)
    assert controller in CONTROLLER["controller_plugins"]
    assert checker in CONTROLLER["goal_checker_plugins"]
    assert PAIRS[controller] == checker
