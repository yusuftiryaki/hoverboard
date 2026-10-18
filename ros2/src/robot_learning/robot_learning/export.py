"""episode_export — recorded episodes -> a training dataset (docs/learning.md, 5d).

    ros2 run robot_learning episode_export ~/.robot/episodes -o ~/dataset --fps 10

Each episode becomes
    episodes/episode_000000.npz   timestamp, state, action, action_source,
                                  action_executed, teleop, image_age (dataset.py)
    videos/episode_000000.mp4     one camera frame per row (the latest stamped
                                  at or before it), image_size, mp4v
and meta.json lists them with their task, mode, outcome, markers (as frame
indices), source and software, plus the column names.

Why not parquet / LeRobot / HDF5: no algorithm is chosen yet (decision 1) and
none of those is installable here without new dependencies (no pyarrow in
Noble's apt). npz + mp4 open anywhere; a converter for the chosen trainer is a
small step on the training machine, and the bag stays the source of truth —
re-export with other choices at any time.

Runs where ROS is (the dev container): it reads bags with rosbag2_py. The
frames start at the episode's READY (before it the bag is partial) and end at
its last message. Aborted episodes are skipped unless --include-aborted;
a bag without metadata.yaml (a crash) is skipped unless --reindex.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import yaml

from robot_learning.dataset import (
    ACTION_NAMES, SOURCES, STATE_NAMES, Series, build_frames, marker_frames)

# topic -> (series name, extractor(msg) -> values). Twists have no header:
# their time is when the bag received them; images use their header stamp
# (when the light hit the sensor).


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _pose(m) -> Tuple[float, ...]:
    p = m.pose.pose
    return (p.position.x, p.position.y, _yaw(p.orientation))


EXTRACT = {
    "/cmd_vel_teleop": ("teleop", lambda m: (m.linear.x, m.angular.z)),
    "/cmd_vel_nav": ("nav", lambda m: (m.linear.x, m.angular.z)),
    "/cmd_vel": ("executed", lambda m: (m.linear.x, m.angular.z)),
    "/odom": ("odom", lambda m: (m.twist.twist.linear.x, m.twist.twist.angular.z)),
    "/odometry/filtered/local": ("local", _pose),
    "/odometry/filtered/global": ("map", _pose),
    "/imu/data_raw": ("imu", lambda m: (m.angular_velocity.z, m.linear_acceleration.x,
                                        m.linear_acceleration.y)),
    "/gps/filtered": ("gps", lambda m: (m.latitude, m.longitude)),
    "/battery": ("battery", lambda m: (m.percentage,)),
    "/bumper": ("bumper", lambda m: (float(bool(m.data)),)),
}
CAMERA = "/camera/jpeg"


def read_series(bag_dir: str) -> Tuple[Dict[str, Series], Series, float, float]:
    """(series, images, first t, last t) — seconds, the bag's clock."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = [t for t in (*EXTRACT, CAMERA) if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    raw: Dict[str, List[Tuple[float, ...]]] = {}
    shots: List[Tuple[float, bytes]] = []
    first, last = math.inf, -math.inf
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        t = stamp * 1e-9
        first, last = min(first, t), max(last, t)
        msg = deserialize_message(data, get_message(types[topic]))
        if topic == CAMERA:
            s = msg.header.stamp
            shots.append((s.sec + s.nanosec * 1e-9, bytes(msg.data)))
        else:
            name, extract = EXTRACT[topic]
            raw.setdefault(name, []).append((t, *extract(msg)))
    return ({k: Series.of(v) for k, v in raw.items()}, Series.of(shots), first, last)


def write_video(path: str, images: Series, index: np.ndarray, size: Tuple[int, int],
                fps: float) -> int:
    import cv2
    writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
    if not writer.isOpened():
        raise RuntimeError(f"cannot write {path}")
    blank = np.zeros((size[1], size[0], 3), np.uint8)
    cache: Dict[int, np.ndarray] = {}
    for i in index:
        if i < 0:
            writer.write(blank)
            continue
        if i not in cache:
            cache.clear()
            frame = cv2.imdecode(np.frombuffer(images.values[i][0], np.uint8), cv2.IMREAD_COLOR)
            cache[i] = blank if frame is None else cv2.resize(frame, size)
        writer.write(cache[i])
    writer.release()
    return len(index)


def export_episode(directory: str, out_dir: str, number: int, fps: float,
                   size: Tuple[int, int], reindex: bool = False) -> Optional[Dict[str, Any]]:
    with open(os.path.join(directory, "episode.yaml"), encoding="utf-8") as handle:
        episode = yaml.safe_load(handle)
    bag = os.path.join(directory, "bag")
    if not os.path.exists(os.path.join(bag, "metadata.yaml")):
        if not reindex:
            print(f"skip {episode['name']}: no metadata.yaml (crashed?) — --reindex",
                  file=sys.stderr)
            return None
        import rosbag2_py
        rosbag2_py.Reindexer().reindex(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"))
    series, images, first, last = read_series(bag)
    ready = episode.get("ready_after_s") or 0.0
    t0 = start_of_frames(first, last, episode.get("started_unix"), ready)
    frames = build_frames(series, images, fps, t0, last)
    stem = f"episode_{number:06d}"
    np.savez_compressed(os.path.join(out_dir, "episodes", stem + ".npz"),
                        timestamp=frames.timestamp, state=frames.state, action=frames.action,
                        action_source=frames.action_source,
                        action_executed=frames.action_executed, teleop=frames.teleop,
                        image_age=frames.image_age)
    video = None
    if len(images.times):
        video = os.path.join("videos", stem + ".mp4")
        write_video(os.path.join(out_dir, video), images, frames.image_index, size, fps)
    return {"index": number, "name": episode["name"], "task": episode["task"],
            "mode": episode["mode"], "outcome": episode["outcome"],
            "source": episode.get("source"), "frames": int(len(frames.timestamp)),
            "duration_s": round(float(frames.timestamp[-1]), 2), "t0_unix": frames.t0,
            "data": os.path.join("episodes", stem + ".npz"), "video": video,
            "markers": marker_frames(episode.get("markers") or [], ready, fps),
            "teleop_fraction": round(float(frames.teleop.mean()), 3),
            "software": (episode.get("context") or {}).get("software")}


def start_of_frames(first: float, last: float, started_unix: Optional[float],
                    ready_after_s: float) -> float:
    """Frames begin at READY. ⚠️ The bag's clock is sim time under Gazebo
    (use_sim_time) while started_unix is wall time: only when the two are the
    same clock (the bag's span is near started_unix) can READY be placed;
    otherwise the bag's own first message is the best start there is."""
    if started_unix is not None and first - 3600.0 < started_unix < last:
        return min(max(first, started_unix + ready_after_s), last)
    return first


def find_episodes(paths: List[str]) -> List[str]:
    found = []
    for path in paths:
        if os.path.isfile(os.path.join(path, "episode.yaml")):
            found.append(path)
        elif os.path.isdir(path):
            found += [os.path.join(path, n) for n in sorted(os.listdir(path))
                      if os.path.isfile(os.path.join(path, n, "episode.yaml"))]
    return found


def export(paths: List[str], out_dir: str, fps: float = 10.0, size=(320, 240),
           include_aborted: bool = False, reindex: bool = False) -> Dict[str, Any]:
    os.makedirs(os.path.join(out_dir, "episodes"), exist_ok=True)
    os.makedirs(os.path.join(out_dir, "videos"), exist_ok=True)
    listed = []
    for directory in find_episodes(paths):
        with open(os.path.join(directory, "episode.yaml"), encoding="utf-8") as handle:
            episode = yaml.safe_load(handle)
        if episode.get("status") != "done":
            print(f"skip {episode['name']}: still recording", file=sys.stderr)
            continue
        if episode.get("outcome") == "aborted" and not include_aborted:
            print(f"skip {episode['name']}: aborted (--include-aborted)", file=sys.stderr)
            continue
        item = export_episode(directory, out_dir, len(listed), fps, size, reindex)
        if item is not None:
            listed.append(item)
            print(f"{item['name']}: {item['frames']} frames, {item['task']!r}")
    meta = {"format": "hoverbot-episodes-v1", "fps": fps, "image_size": list(size),
            "state_names": list(STATE_NAMES), "action_names": list(ACTION_NAMES),
            "action_sources": list(SOURCES),
            "action_rule": "the command given in (t, t + 1/fps], else the latest still "
                           "fresh (0.5 s); teleop outranks nav",
            "episodes": listed}
    with open(os.path.join(out_dir, "meta.json"), "w", encoding="utf-8") as handle:
        json.dump(meta, handle, indent=1, ensure_ascii=False)
    return meta


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("paths", nargs="+", help="episode directories or episodes_dir")
    parser.add_argument("-o", "--out", required=True, help="dataset directory")
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--image-size", default="320x240", help="WxH of the video")
    parser.add_argument("--include-aborted", action="store_true")
    parser.add_argument("--reindex", action="store_true",
                        help="rebuild a crashed bag's metadata.yaml first")
    args = parser.parse_args(argv)
    width, height = (int(v) for v in args.image_size.lower().split("x"))
    meta = export(args.paths, args.out, args.fps, (width, height), args.include_aborted,
                  args.reindex)
    print(f"{len(meta['episodes'])} episodes -> {args.out}")
    return 0 if meta["episodes"] else 1


if __name__ == "__main__":
    sys.exit(main())
