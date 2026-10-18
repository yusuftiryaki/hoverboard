"""edge_batch — many expert edge-following episodes on random lawns (docs/learning.md, 6b).

    ros2 run robot_sim edge_batch --lawns 20 --out ~/.robot/sim_episodes
    ros2 run robot_learning episode_export ~/.robot/sim_episodes -o ~/dataset

For each lawn (lawn_gen.random_lawn(seed + i)): a fresh simulator and robot
stack — the same robot.launch.py as the Pi, learning mode on — the camera
painting that lawn (grass inside, soil outside, palette seeded per lawn), and
the privileged expert (edge_expert) driving one lap per direction through
/cmd_vel_teleop while the episode recorder records. Then everything is killed
and the next lawn starts clean: sim_node always puts the robot at the origin.

Each episode's note carries the seed, direction, GPS mode and the lawn (WKT
in sim_world), so a dataset can be traced back to its world.

Slow on purpose: a lap is ~1-2 min of real time (the kinematic world runs at
wall-clock speed). 20 lawns x 2 directions ~ 1.5 h — an overnight job.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
from typing import Dict, List, Optional

QUIET_GPS = ["-p", "gps_wander_sigma:=0.0", "-p", "gps_noise_sigma:=0.02"]
ESP32_LINK = "/tmp/fake_esp32"


class Stack:
    """sim_node + robot.launch.py (+ the expert), killed as one."""

    def __init__(self, workdir: str, site_file: str, episodes_dir: str, seed: int,
                 gps: str, camera_hz: float):
        self.procs: List[subprocess.Popen] = []
        self.workdir = workdir
        self.site_file = site_file
        sim = ["ros2", "run", "robot_sim", "sim_node", "--ros-args",
               "-p", "publish_obstacle_map:=false", "-p", f"lawn_file:={site_file}",
               "-p", f"ground_seed:={seed}", "-p", f"camera_rate_hz:={camera_hz}"]
        if gps == "quiet":
            sim += QUIET_GPS
        self.spawn(sim)
        time.sleep(3.0)
        self.spawn(["ros2", "launch", "robot_bringup", "robot.launch.py",
                    f"esp32_port:={ESP32_LINK}", "use_localization:=true", "use_imu:=false",
                    "use_mag:=false", "use_gps:=true", "use_imu_filter:=true",
                    "use_site_map:=true", f"site_file:={site_file}",
                    f"detected_file:={os.path.join(workdir, 'detected.json')}",
                    "use_web:=false", "use_learning:=true", "learning_sim:=true",
                    f"episodes_dir:={episodes_dir}"])

    def spawn(self, argv: List[str]) -> subprocess.Popen:
        log = open(os.path.join(self.workdir, f"proc{len(self.procs)}.log"), "wb")
        proc = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)
        self.procs.append(proc)
        return proc

    def close(self) -> None:
        for proc in reversed(self.procs):
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + 20.0
        for proc in reversed(self.procs):
            try:
                proc.wait(timeout=max(0.1, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
                proc.wait(timeout=10)
        subprocess.run(["fastdds", "shm", "clean"], stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL, check=False)


class Driver:
    """The runner's ROS end: the recorder's and the expert's services."""

    def __init__(self):
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import String
        self.rclpy = rclpy
        self.node = rclpy.create_node("edge_batch")
        self.recorder: Optional[Dict] = None
        self.expert: Optional[Dict] = None
        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String, "/episode_recorder/status",
                                      lambda m: setattr(self, "recorder", json.loads(m.data)),
                                      latched)
        self.node.create_subscription(String, "/edge_expert/status",
                                      lambda m: setattr(self, "expert", json.loads(m.data)),
                                      latched)

    def wait(self, predicate, timeout: float, what: str) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.2)
            if predicate():
                return
        raise TimeoutError(f"{what} within {timeout:.0f} s")

    def call(self, srv_type, name: str, request, timeout: float = 30.0):
        client = self.node.create_client(srv_type, name)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise TimeoutError(f"{name} is not up")
            future = client.call_async(request)
            self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
            if not future.done():
                raise TimeoutError(f"{name} did not answer")
            return future.result()
        finally:
            self.node.destroy_client(client)

    def episode(self, command: str, data: Dict) -> str:
        from robot_missions_interfaces.srv import Episode
        request = Episode.Request()
        request.command, request.data = command, json.dumps(data)
        response = self.call(Episode, "/episode_recorder/command", request)
        if not response.success:
            raise RuntimeError(f"episode {command}: {response.message}")
        return response.episode

    def trigger(self, name: str) -> None:
        from std_srvs.srv import Trigger
        response = self.call(Trigger, name, Trigger.Request())
        if not response.success:
            raise RuntimeError(f"{name}: {response.message}")

    def close(self) -> None:
        self.node.destroy_node()


def run_lawn(driver: Driver, seed: int, out: str, gps: str, directions: List[str],
             camera_hz: float, offset: float, lap_timeout: float) -> List[str]:
    from robot_sim.lawn_gen import random_lawn
    from robot_sim.site_world import area_feature
    lawn = random_lawn(seed)
    workdir = tempfile.mkdtemp(prefix=f"edge_batch_{seed}_")
    site_file = os.path.join(workdir, "site.geojson")
    with open(site_file, "w", encoding="utf-8") as handle:
        json.dump({"type": "FeatureCollection",
                   "features": [area_feature("cim", lawn, 41.0, 29.0)]}, handle)
    stack = Stack(workdir, site_file, out, seed, gps, camera_hz)
    names = []
    try:
        driver.recorder = driver.expert = None
        driver.wait(lambda: driver.recorder is not None, 90.0, "the episode recorder")
        for direction in directions:
            expert = stack.spawn(["ros2", "run", "robot_sim", "edge_expert", "--ros-args",
                                  "-p", f"site_file:={site_file}", "-p", f"offset_m:={offset}",
                                  "-p", f"direction:={direction}"])
            driver.expert = None
            driver.wait(lambda: driver.expert is not None, 30.0, "the expert")
            note = {"seed": seed, "direction": direction, "gps": gps, "offset_m": offset,
                    "lawn_wkt": lawn.wkt, "expert": "robot_sim edge_expert (privileged)"}
            name = driver.episode("start", {"task": "kenar takibi (uzman)", "mode": "demo",
                                            "note": json.dumps(note)})
            outcome = "success"
            try:
                driver.wait(lambda: (driver.recorder or {}).get("ready"), 60.0, "READY")
                driver.trigger("/edge_expert/start")
                driver.wait(lambda: (driver.expert or {}).get("state") == "done", lap_timeout,
                            "the lap")
            except (TimeoutError, RuntimeError) as exc:
                outcome = "failure"
                print(f"  {direction}: {exc}", file=sys.stderr)
            driver.episode("stop", {"outcome": outcome})
            os.killpg(os.getpgid(expert.pid), signal.SIGINT)
            expert.wait(timeout=15)
            print(f"  {name}: {direction} {outcome}")
            names.append(name)
    finally:
        stack.close()
    return names


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--lawns", type=int, default=10)
    parser.add_argument("--seed", type=int, default=1, help="first lawn's seed (0 = no randomization)")
    parser.add_argument("--out", default=os.path.expanduser("~/.robot/sim_episodes"))
    parser.add_argument("--gps", choices=("quiet", "neo6m"), default="neo6m")
    parser.add_argument("--directions", default="ccw,cw")
    parser.add_argument("--camera-hz", type=float, default=10.0)
    parser.add_argument("--offset", type=float, default=0.4)
    parser.add_argument("--lap-timeout", type=float, default=300.0)
    args = parser.parse_args(argv)

    import rclpy
    rclpy.init()
    driver = Driver()
    made = []
    try:
        for i in range(args.lawns):
            seed = args.seed + i
            print(f"lawn {i + 1}/{args.lawns} (seed {seed})")
            made += run_lawn(driver, seed, args.out, args.gps, args.directions.split(","),
                             args.camera_hz, args.offset, args.lap_timeout)
    finally:
        driver.close()
        rclpy.try_shutdown()
    print(f"{len(made)} episodes in {args.out}")
    return 0 if made else 1


if __name__ == "__main__":
    sys.exit(main())
