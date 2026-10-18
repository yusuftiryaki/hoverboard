"""episode_recorder — learning mode (docs/learning.md).

    ~/command (robot_missions_interfaces/Episode)   start | stop | mark (JSON data)
    ~/status (String JSON, latched)                 recording?, episode, elapsed,
                                                    size, free space, last error
    learning/marker (String, out)                   each mark, so it is IN the bag

The episode logic is episode.Recorder (ROS-free). This node adds the ROS end:
the service, the status, the marker topic, and the snapshot taken at start —
the site file and the parameters of the nodes whose calibration the data
depends on (`ros2 param dump`), because B4 will change those constants and an
episode whose constants are unknown is worthless after that day.

⚠️ The recording process outlives nothing: it gets SIGINT when this node shuts
down (episode outcome "shutdown"), and a parent-death SIGINT when this node is
killed outright — an orphan recorder would fill the disk forever.
"""

from __future__ import annotations

import json
import os
import shutil
import socket
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Dict, List

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from robot_learning.episode import (
    LOG_FILE, RECORDER_NODE, EpisodeError, Recorder, load_profile)
from robot_missions_interfaces.srv import Episode

DEFAULT_PARAM_NODES = ["hoverboard_bridge", "ekf_local", "ekf_global", "navsat_transform",
                       "twist_mux", "web_ui", "battery_monitor", "battery_supervisor",
                       "imu_filter", "image_compressor"]


def spawn_recorder(argv: List[str], directory: str) -> subprocess.Popen:
    """`setpriv --pdeathsig INT` (util-linux): the recorder gets SIGINT — and
    closes its bag — if this node dies without stopping it. Not preexec_fn:
    that is unsafe in a multi-threaded process (the executor's)."""
    if shutil.which("setpriv"):
        argv = ["setpriv", "--pdeathsig", "INT", "--", *argv]
    # ⚠️ Output to a file, never an undrained pipe (episode.py, _spawn).
    with open(os.path.join(directory, LOG_FILE), "wb") as log:
        return subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)


def software_version() -> Dict[str, Any]:
    here = os.path.dirname(os.path.realpath(__file__))   # src/ under --symlink-install
    out: Dict[str, Any] = {"host": socket.gethostname(),
                           "ros_distro": os.environ.get("ROS_DISTRO")}
    try:
        out["git"] = subprocess.run(["git", "-C", here, "rev-parse", "HEAD"], check=True,
                                    capture_output=True, text=True, timeout=5).stdout.strip()
        dirty = subprocess.run(["git", "-C", here, "status", "--porcelain"], check=True,
                               capture_output=True, text=True, timeout=5).stdout.strip()
        out["git_dirty"] = bool(dirty)
    except (OSError, subprocess.SubprocessError):
        out["git"] = None
    return out


class EpisodeRecorderNode(Node):
    def __init__(self, **kwargs):
        super().__init__("episode_recorder", **kwargs)
        self.declare_parameter("episodes_dir", os.path.expanduser("~/.robot/episodes"))
        self.declare_parameter("topics_file", "")
        self.declare_parameter("sim", False)
        self.declare_parameter("split_s", 60)
        self.declare_parameter("min_free_gb", 5.0)
        self.declare_parameter("site_file", "")
        self.declare_parameter("param_nodes", DEFAULT_PARAM_NODES)
        p = self.get_parameter

        topics_file = p("topics_file").value
        if not topics_file:
            from ament_index_python.packages import get_package_share_directory
            topics_file = os.path.join(get_package_share_directory("robot_bringup"),
                                       "config", "learning_topics.yaml")
        sim = bool(p("sim").value)
        topics = load_profile(topics_file, sim)
        self.recorder = Recorder(
            p("episodes_dir").value, topics, split_s=int(p("split_s").value),
            min_free_bytes=int(float(p("min_free_gb").value) * 1e9),
            use_sim_time=bool(p("use_sim_time").value), source="sim" if sim else "real",
            context={"software": software_version(), "topics_file": topics_file},
            snapshot=self._snapshot, spawn=spawn_recorder)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(String, "~/status", latched)
        self._marker_pub = self.create_publisher(String, "learning/marker", 10)
        # The command can block for a few seconds (stopping waits for rosbag2
        # to close its files); the status timer must not wait behind it.
        self.create_service(Episode, "~/command", self._on_command,
                            callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(2.0, self._poll, callback_group=MutuallyExclusiveCallbackGroup())
        self._topics = topics
        self.create_timer(0.25, self._check_ready, callback_group=MutuallyExclusiveCallbackGroup())
        self._publish_status()
        self.get_logger().info(f"episodes in {self.recorder.episodes_dir}, {len(topics)} "
                               f"topics ({'sim' if sim else 'real'})")

    def _on_command(self, request: Episode.Request, response: Episode.Response):
        try:
            data = json.loads(request.data) if request.data.strip() else {}
            if request.command == "start":
                response.episode = self.recorder.start(data)
                response.message = f"recording {response.episode}"
                self.get_logger().info(response.message)
            elif request.command == "stop":
                response.episode = self.recorder.stop(data)
                response.message = f"stopped {response.episode} ({data.get('outcome')})"
                self.get_logger().info(response.message)
            elif request.command == "mark":
                marker = self.recorder.mark(data)
                self._marker_pub.publish(String(data=marker["label"]))
                response.message = f"marked {marker['label']!r} at {marker['t']} s"
            else:
                raise EpisodeError(f"unknown command {request.command!r} (start, stop, mark)")
            response.success = True
        except json.JSONDecodeError as exc:
            response.success, response.message = False, f"data is not JSON: {exc}"
        except EpisodeError as exc:
            response.success, response.message = False, str(exc)
        self._publish_status()
        return response

    def _check_ready(self) -> None:
        """READY = the recorder subscribes to every profile topic that has a
        publisher right now. Measured: several seconds after start on the dev
        machine — a drive begun at once was missing its first metre."""
        if not self.recorder.status()["recording"] or self.recorder.ready:
            return
        live = {t for t in self._topics if self.count_publishers(t) > 0}
        try:
            subscribed = {name for name, _ in self.get_subscriber_names_and_types_by_node(
                RECORDER_NODE, "/")}
        except Exception:                     # noqa: BLE001 — not in the graph yet
            subscribed = set()
        before = self.recorder.ready
        self.recorder.set_waiting(sorted(live - subscribed))
        if self.recorder.ready and not before:
            self.get_logger().info("recorder subscribed to every live topic: READY")
            self._publish_status()

    def _poll(self) -> None:
        why = self.recorder.poll()
        if why is not None:
            self.get_logger().error(f"episode ended on its own: {why}")
        self._publish_status()

    def _publish_status(self) -> None:
        self._status_pub.publish(String(data=json.dumps(self.recorder.status())))

    def _snapshot(self, directory: str) -> Dict[str, Any]:
        """The context the bag cannot carry. Runs in its own thread."""
        out: Dict[str, Any] = {"site": None, "params": [], "params_failed": {}}
        site = self.get_parameter("site_file").value
        if site and os.path.exists(site):
            shutil.copy(site, os.path.join(directory, "site.geojson"))
            out["site"] = "site.geojson"
        params_dir = os.path.join(directory, "params")
        os.makedirs(params_dir, exist_ok=True)
        wanted = list(self.get_parameter("param_nodes").value)
        # A node can be briefly missing from the graph (discovery, or the
        # bridge respawning): look a few times before calling it absent.
        # ⚠️ Found in CI: the bridge's dump — the calibration constants this
        # snapshot exists for — went missing once while every other one worked.
        running: set = set()
        for _ in range(5):
            running = {name for name, _ in self.get_node_names_and_namespaces()}
            if all(n in running for n in wanted):
                break
            time.sleep(1.0)

        def dump(name: str) -> str:
            """'' on success, else why."""
            if name not in running:
                return "not running"
            why = ""
            for _ in range(3):
                try:
                    result = subprocess.run(["ros2", "param", "dump", "/" + name],
                                            capture_output=True, text=True, timeout=30)
                except (OSError, subprocess.SubprocessError) as exc:
                    why = repr(exc)
                    continue
                if result.returncode == 0 and result.stdout.strip():
                    with open(os.path.join(params_dir, name + ".yaml"), "w") as handle:
                        handle.write(result.stdout)
                    return ""
                why = (result.stderr or result.stdout).strip()[-200:] or \
                    f"exit {result.returncode}"
                time.sleep(1.0)
            return why

        # Each `ros2 param dump` is a CLI start-up (~1-2 s here, more on a Pi).
        with ThreadPoolExecutor(max_workers=4) as pool:
            for name, why in zip(wanted, pool.map(dump, wanted)):
                if not why:
                    out["params"].append(name)
                elif why != "not running":
                    out["params_failed"][name] = why
        return out


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = EpisodeRecorderNode()
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.recorder.shutdown()             # close the bag, outcome "shutdown"
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
