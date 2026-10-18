"""Episodes for imitation learning (docs/learning.md). ROS-free.

An episode is one directory under episodes_dir:

    20261018-101530_kenar-bicme/
      episode.yaml     ours: task, mode, note, outcome, markers, timing, context
      bag/             rosbag2 output (metadata.yaml + *.mcap, split every split_s)
      recorder.log     the recorder process's output
      ...              whatever the snapshot callback adds (site copy, param dumps)

The recorder is a separate `ros2 bag record` process, started and stopped
here. It only subscribes: nothing in the robot's command path depends on it,
and when the disk runs low the RECORDING stops, never the robot (decision 2).

⚠️ A recording process that dies (or a robot that loses power) leaves its last
split without a footer and the bag without metadata.yaml. The closed splits
are intact; `ros2 bag reindex <bag> -s mcap` rebuilds metadata.yaml so they
can be read. split_s bounds what is lost.
"""

from __future__ import annotations

import datetime as dt
import os
import re
import shutil
import signal
import subprocess
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Sequence

import yaml

MODES = ("demo", "intervention", "autonomous")
OUTCOMES = ("success", "failure", "aborted")          # what the person says
# What the robot says when the person did not get to: the disk filled up, the
# recorder process died, the node was shut down mid-episode.
SYSTEM_OUTCOMES = ("disk_full", "crashed", "shutdown")
EPISODE_FILE = "episode.yaml"
BAG_DIR = "bag"
LOG_FILE = "recorder.log"
RECORDER_NODE = "episode_bag_recorder"     # `ros2 bag record --node-name`
_NAME = re.compile(r"^[0-9]{8}-[0-9]{6}_[a-z0-9-]+$")


class EpisodeError(ValueError):
    """A refused request; the message says why."""


def load_profile(path: str, sim: bool) -> List[str]:
    """The topics to record: `always`, plus `sim` in the simulator."""
    with open(path, encoding="utf-8") as handle:
        document = yaml.safe_load(handle) or {}
    profile = document.get("learning_topics", document)
    topics: List[str] = []
    for group in ("always", "sim") if sim else ("always",):
        for topic in profile.get(group) or []:
            if not isinstance(topic, str) or not topic.startswith("/"):
                raise EpisodeError(f"{path}: topic names are absolute, got {topic!r}")
            if topic not in topics:
                topics.append(topic)
    if not topics:
        raise EpisodeError(f"{path}: no topics to record")
    return topics


_TR = str.maketrans("çğıöşüÇĞİÖŞÜâîû", "cgiosuCGIOSUaiu")


def slug(text: str) -> str:
    text = text.translate(_TR).lower()
    text = re.sub(r"[^a-z0-9]+", "-", text).strip("-")
    return text[:40].rstrip("-") or "bolum"


def is_episode_name(name: Any) -> bool:
    """A directory name this module made — never a path."""
    return isinstance(name, str) and bool(_NAME.match(name))


def record_command(bag_dir: str, topics: Sequence[str], split_s: int,
                   use_sim_time: bool) -> List[str]:
    argv = ["ros2", "bag", "record", "-o", bag_dir, "-s", "mcap",
            "--storage-preset-profile", "zstd_fast", "--max-bag-duration", str(int(split_s)),
            "--disable-keyboard-controls", "--node-name", RECORDER_NODE]
    if use_sim_time:
        argv.append("--use-sim-time")
    return argv + ["--topics", *topics]


def dir_size(path: str) -> int:
    total = 0
    for root, _, files in os.walk(path):
        for name in files:
            try:
                total += os.path.getsize(os.path.join(root, name))
            except OSError:
                pass
    return total


def free_bytes(path: str) -> int:
    return shutil.disk_usage(path).free


def read_episode(directory: str) -> Optional[Dict[str, Any]]:
    try:
        with open(os.path.join(directory, EPISODE_FILE), encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
        return data if isinstance(data, dict) else None
    except (OSError, yaml.YAMLError):
        return None


def _write_yaml(path: str, data: Dict[str, Any]) -> None:
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as handle:
        yaml.safe_dump(data, handle, allow_unicode=True, sort_keys=False)
    os.replace(tmp, path)


def _spawn(argv: List[str], directory: str) -> subprocess.Popen:
    # Own session: SIGINT goes to the recorder and its children, not to us.
    # ⚠️ Output to a FILE, never a pipe nobody drains: rosbag2 logs every
    # split, and a full 64 KB pipe would freeze the recorder hours into an
    # episode — silently, while the page still said REC.
    with open(os.path.join(directory, LOG_FILE), "wb") as log:
        return subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)


def _tail(path: str, size: int = 500) -> str:
    try:
        with open(path, "rb") as handle:
            handle.seek(0, os.SEEK_END)
            handle.seek(max(0, handle.tell() - size))
            return handle.read().decode(errors="replace").strip()
    except OSError:
        return ""


class Recorder:
    """One episode at a time. Thread-safe."""

    def __init__(self, episodes_dir: str, topics: Sequence[str], split_s: int = 60,
                 min_free_bytes: int = 5 * 10**9, use_sim_time: bool = False,
                 source: str = "real", context: Optional[Dict[str, Any]] = None,
                 snapshot: Optional[Callable[[str], Dict[str, Any]]] = None,
                 spawn: Callable[[List[str], str], subprocess.Popen] = _spawn,
                 stop_timeout_s: float = 15.0, clock: Callable[[], float] = time.time):
        self.episodes_dir = episodes_dir
        self.topics = list(topics)
        self.split_s = split_s
        self.min_free_bytes = min_free_bytes
        self.use_sim_time = use_sim_time
        self.source = source
        self.context = dict(context or {})
        self.snapshot = snapshot
        self.spawn = spawn
        self.stop_timeout_s = stop_timeout_s
        self.clock = clock
        self._lock = threading.RLock()
        self._proc: Optional[subprocess.Popen] = None
        self._episode: Optional[Dict[str, Any]] = None
        self._dir: Optional[str] = None
        self._size = 0
        self._ready_at: Optional[float] = None
        self._waiting_for: List[str] = []
        self._last_error: Optional[str] = None
        os.makedirs(episodes_dir, exist_ok=True)

    # --- commands ---------------------------------------------------------------

    def start(self, spec: Any) -> str:
        if not isinstance(spec, dict):
            raise EpisodeError("start needs a JSON object")
        task, mode, note = spec.get("task"), spec.get("mode", "demo"), spec.get("note", "")
        if not isinstance(task, str) or not task.strip() or len(task) > 100:
            raise EpisodeError("an episode needs a task (what are we doing?), ≤ 100 characters")
        if mode not in MODES:
            raise EpisodeError(f"mode must be one of {MODES}, got {mode!r}")
        if not isinstance(note, str):
            raise EpisodeError("note must be text")
        with self._lock:
            if self._proc is not None:
                raise EpisodeError(f"already recording {self._episode['name']}: stop it first")
            free = free_bytes(self.episodes_dir)
            if free < self.min_free_bytes:
                raise EpisodeError(f"only {free / 1e9:.1f} GB free in {self.episodes_dir} "
                                   f"(need {self.min_free_bytes / 1e9:.1f})")
            now = self.clock()
            name = self._unique(dt.datetime.fromtimestamp(now).strftime("%Y%m%d-%H%M%S")
                                + "_" + slug(task))
            directory = os.path.join(self.episodes_dir, name)
            os.makedirs(directory)
            bag_dir = os.path.join(directory, BAG_DIR)
            episode = {
                "name": name, "task": task.strip(), "mode": mode, "note": note,
                "status": "recording", "outcome": None, "outcome_note": "",
                "started": dt.datetime.fromtimestamp(now).astimezone().isoformat(),
                "started_unix": now, "ended": None, "duration_s": None, "size_bytes": None,
                "source": self.source, "topics": list(self.topics), "split_s": self.split_s,
                "markers": [], "context": dict(self.context), "snapshot": None,
                "ready_after_s": None,
            }
            _write_yaml(os.path.join(directory, EPISODE_FILE), episode)
            try:
                proc = self.spawn(record_command(bag_dir, self.topics, self.split_s,
                                                 self.use_sim_time), directory)
            except OSError as exc:
                episode.update(status="done", outcome="crashed", ended=episode["started"],
                               outcome_note=f"recorder did not start: {exc}")
                _write_yaml(os.path.join(directory, EPISODE_FILE), episode)
                raise EpisodeError(f"recorder did not start: {exc}") from exc
            self._proc, self._episode, self._dir, self._size = proc, episode, directory, 0
            self._ready_at, self._waiting_for = None, []
            self._last_error = None
        if self.snapshot is not None:
            threading.Thread(target=self._take_snapshot, args=(name, directory),
                             daemon=True).start()
        return name

    def stop(self, spec: Any, system: bool = False) -> str:
        spec = spec if isinstance(spec, dict) else {}
        outcome, note = spec.get("outcome"), spec.get("note", "")
        allowed = OUTCOMES + SYSTEM_OUTCOMES if system else OUTCOMES
        if outcome not in allowed:
            raise EpisodeError(f"outcome must be one of {allowed}, got {outcome!r}")
        if not isinstance(note, str):
            raise EpisodeError("note must be text")
        with self._lock:
            if self._proc is None:
                raise EpisodeError("not recording")
            proc = self._proc
            try:
                os.killpg(proc.pid, signal.SIGINT)     # rosbag2 closes its files
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=self.stop_timeout_s)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5.0)
                note = (note + " " if note else "") + \
                    "(recorder did not stop on SIGINT; killed — reindex the bag)"
            return self._finalize(outcome, note, proc.returncode)

    def mark(self, spec: Any) -> Dict[str, Any]:
        label = spec.get("label") if isinstance(spec, dict) else None
        if not isinstance(label, str) or not label.strip() or len(label) > 100:
            raise EpisodeError("a marker needs a label, ≤ 100 characters")
        with self._lock:
            if self._proc is None:
                raise EpisodeError("not recording")
            marker = {"t": round(self.clock() - self._episode["started_unix"], 2),
                      "label": label.strip()}
            self._episode["markers"].append(marker)
            _write_yaml(os.path.join(self._dir, EPISODE_FILE), self._episode)
            return marker

    def set_waiting(self, missing: Sequence[str]) -> None:
        """The node's graph check: topics that have a publisher but no
        subscription from the recorder yet. Empty once = READY — from then
        on the bag holds every live topic (docs/learning.md: the recorder
        takes seconds to subscribe; what is driven before READY is partial)."""
        with self._lock:
            if self._proc is None or self._ready_at is not None:
                return
            self._waiting_for = sorted(missing)
            if not missing:
                self._ready_at = self.clock()
                self._episode["ready_after_s"] = round(
                    self._ready_at - self._episode["started_unix"], 2)
                _write_yaml(os.path.join(self._dir, EPISODE_FILE), self._episode)

    @property
    def ready(self) -> bool:
        with self._lock:
            return self._ready_at is not None

    def poll(self) -> Optional[str]:
        """Call every few seconds. Ends the episode on its own when the
        recorder died or the disk is nearly full; returns why, or None."""
        with self._lock:
            if self._proc is None:
                return None
            code = self._proc.poll()
            if code is not None:
                log = _tail(os.path.join(self._dir, LOG_FILE))
                self._finalize("crashed", f"recorder exited with {code}: {log}", code)
                self._last_error = f"recorder exited with {code}"
                return "crashed"
            self._size = dir_size(self._dir)
            if free_bytes(self.episodes_dir) < self.min_free_bytes:
                self.stop({"outcome": "disk_full",
                           "note": "stopped: free space below the limit"}, system=True)
                self._last_error = "disk nearly full: recording stopped (the robot was not)"
                return "disk_full"
            return None

    def shutdown(self) -> None:
        with self._lock:
            if self._proc is not None:
                self.stop({"outcome": "shutdown", "note": "the recorder node shut down"},
                          system=True)

    # --- state ------------------------------------------------------------------

    def status(self) -> Dict[str, Any]:
        with self._lock:
            out: Dict[str, Any] = {"recording": self._proc is not None,
                                   "last_error": self._last_error}
            try:
                out["free_bytes"] = free_bytes(self.episodes_dir)
            except OSError:
                out["free_bytes"] = None
            if self._episode is not None and self._proc is not None:
                e = self._episode
                out.update(episode=e["name"], task=e["task"], mode=e["mode"],
                           elapsed_s=round(self.clock() - e["started_unix"], 1),
                           size_bytes=self._size, markers=len(e["markers"]),
                           ready=self._ready_at is not None,
                           waiting_for=self._waiting_for[:5])
            return out

    # --- helpers ------------------------------------------------------------------

    def _finalize(self, outcome: str, note: str, code: Optional[int]) -> str:
        episode, directory = self._episode, self._dir
        now = self.clock()
        episode.update(status="done", outcome=outcome, outcome_note=note,
                       ended=dt.datetime.fromtimestamp(now).astimezone().isoformat(),
                       duration_s=round(now - episode["started_unix"], 1),
                       size_bytes=dir_size(directory), recorder_exit=code)
        _write_yaml(os.path.join(directory, EPISODE_FILE), episode)
        self._proc = self._episode = self._dir = None
        return episode["name"]

    def _take_snapshot(self, name: str, directory: str) -> None:
        try:
            result = self.snapshot(directory)
        except Exception as exc:              # noqa: BLE001 — never lose the episode
            result = {"error": repr(exc)}
        with self._lock:
            if self._episode is not None and self._episode["name"] == name:
                self._episode["snapshot"] = result
                _write_yaml(os.path.join(directory, EPISODE_FILE), self._episode)
            else:                             # already over: patch the file
                episode = read_episode(directory)
                if episode is not None:
                    episode["snapshot"] = result
                    _write_yaml(os.path.join(directory, EPISODE_FILE), episode)

    def _unique(self, name: str) -> str:
        candidate, n = name, 2
        while os.path.exists(os.path.join(self.episodes_dir, candidate)):
            candidate, n = f"{name}-{n}", n + 1
        return candidate
