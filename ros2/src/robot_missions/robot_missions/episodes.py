"""The web page's view of the recorded episodes (docs/learning.md). ROS-free.

robot_learning's episode_recorder WRITES episodes; this only lists, zips and
deletes them, from the files. The layout is robot_learning.episode's:
<episodes_dir>/<YYYYmmdd-HHMMSS>_<slug>/episode.yaml + bag/ + extras. Keep the
name pattern and EPISODE_FILE in step with that module.
"""

from __future__ import annotations

import os
import re
import shutil
import zipfile
from typing import Any, BinaryIO, Dict, List, Optional

import yaml

EPISODE_FILE = "episode.yaml"
_NAME = re.compile(r"^[0-9]{8}-[0-9]{6}_[a-z0-9-]+$")
SUMMARY = ("name", "task", "mode", "note", "status", "outcome", "outcome_note", "started",
           "duration_s", "size_bytes", "source")


def is_episode_name(name: Any) -> bool:
    """A name robot_learning made — never a path (no '/', no '..')."""
    return isinstance(name, str) and bool(_NAME.match(name))


def _read(directory: str) -> Optional[Dict[str, Any]]:
    try:
        with open(os.path.join(directory, EPISODE_FILE), encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
        return data if isinstance(data, dict) else None
    except (OSError, yaml.YAMLError):
        return None


def list_episodes(episodes_dir: str) -> List[Dict[str, Any]]:
    """Newest first; directories without a readable episode.yaml are skipped."""
    if not episodes_dir or not os.path.isdir(episodes_dir):
        return []
    out = []
    for name in sorted(os.listdir(episodes_dir), reverse=True):
        if not is_episode_name(name):
            continue
        data = _read(os.path.join(episodes_dir, name))
        if data is None:
            continue
        item = {key: data.get(key) for key in SUMMARY}
        item["name"] = name
        item["markers"] = len(data.get("markers") or [])
        out.append(item)
    return out


def episode_dir(episodes_dir: str, name: Any) -> str:
    if not is_episode_name(name):
        raise ValueError(f"not an episode name: {name!r}")
    path = os.path.join(episodes_dir, name)
    if not os.path.isfile(os.path.join(path, EPISODE_FILE)):
        raise FileNotFoundError(name)
    return path


def delete_episode(episodes_dir: str, name: Any, recording: Optional[str]) -> None:
    path = episode_dir(episodes_dir, name)
    if name == recording:
        raise PermissionError(f"{name} is being recorded: stop it first")
    shutil.rmtree(path)


def write_zip(episodes_dir: str, name: Any, out: BinaryIO) -> None:
    """The episode as a zip, streamed to `out` (which need not seek). STORED,
    not deflated: MCAP chunks are already zstd, and the Pi's CPU is better
    spent elsewhere."""
    path = episode_dir(episodes_dir, name)
    with zipfile.ZipFile(out, "w", compression=zipfile.ZIP_STORED, allowZip64=True) as archive:
        for root, _, files in os.walk(path):
            for file_name in sorted(files):
                full = os.path.join(root, file_name)
                arcname = os.path.join(name, os.path.relpath(full, path))
                with open(full, "rb") as source, archive.open(arcname, "w",
                                                               force_zip64=True) as target:
                    shutil.copyfileobj(source, target, 1024 * 1024)
