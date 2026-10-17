"""Obstacles the robot found by running into them. ROS-free.

docs/missions.md, "Robotun kendi tespit ettiği engeller": with no range sensor
(handoff decision 8) the front bumper is the only way the robot learns about a
rock nobody surveyed. Each press becomes a DETECTED obstacle:

  * it goes on the survey map at once — the same mission does not drive into
    the same rock twice;
  * it is kept on disk in lat/lon, so it survives a restart (and `map`, which
    does not);
  * it is NOT written into the site file. A person confirms it, labels it
    ("tree, 0.4 m") or throws it away (phase 4's web page). A bumper also
    presses on a dog, a ball, a person's foot.

A press that lands ON a known detection (within its drawn radius) is the same
contact again (count + 1): a jammed robot presses many times a second. A press
just beside one is NOT merged — it is the same rock seen from another side,
and the part of it nobody drew yet. Merging those (they were, within 0.5 m)
let a rock 0.4 m wide hold one 0.25 m detection on its near edge while the
robot hit its middle 27 times: each new press vanished into the old circle,
so the rows beside it were never cut round the rest.
"""

from __future__ import annotations

import json
import math
import os
import tempfile
import time
from dataclasses import asdict, dataclass, field
from typing import List, Optional, Tuple


@dataclass
class Detection:
    x: float                            # `map`, this run; recomputed from lat/lon after a restart
    y: float
    radius_m: float
    count: int = 1
    first_seen: float = field(default_factory=time.time)
    last_seen: float = field(default_factory=time.time)
    lat: Optional[float] = None         # filled in by /toLL; None until then
    lon: Optional[float] = None
    # The camera's frame from the moment of the press (a file name in the
    # images directory), so a person can tell a rock from a dog. None: no
    # camera, or no fresh frame.
    image: Optional[str] = None


def contact_point(x: float, y: float, yaw: float, offset: float) -> Tuple[float, float]:
    """Where the front bumper touched: `offset` ahead of base_link."""
    return x + offset * math.cos(yaw), y + offset * math.sin(yaw)


class DetectedStore:
    def __init__(self, path: str, merge_distance: Optional[float] = None):
        """merge_distance: None = each detection's own radius (see above)."""
        self.path = path
        self.merge_distance = merge_distance
        self.items: List[Detection] = []

    def load(self) -> List[Detection]:
        """Read the file. Positions come back WITHOUT map coordinates (x, y are
        NaN) — only lat/lon means anything after a restart."""
        if not self.path or not os.path.exists(self.path):
            return []
        with open(self.path, encoding="utf-8") as handle:
            raw = json.load(handle)
        self.items = []
        for entry in raw.get("detected", []):
            if entry.get("lat") is None or entry.get("lon") is None:
                continue                # never made it to lat/lon: meaningless now
            self.items.append(Detection(
                x=math.nan, y=math.nan, radius_m=float(entry["radius_m"]),
                count=int(entry.get("count", 1)),
                first_seen=float(entry.get("first_seen", 0.0)),
                last_seen=float(entry.get("last_seen", 0.0)),
                lat=float(entry["lat"]), lon=float(entry["lon"]),
                image=entry.get("image")))
        return self.items

    def add_contact(self, x: float, y: float, radius_m: float) -> Tuple[Detection, bool]:
        """(the detection, True if it is new)."""
        for item in self.items:
            reach = item.radius_m if self.merge_distance is None else self.merge_distance
            if math.isfinite(item.x) and math.hypot(item.x - x, item.y - y) <= reach:
                item.count += 1
                item.last_seen = time.time()
                return item, False
        item = Detection(x=x, y=y, radius_m=radius_m)
        self.items.append(item)
        return item, True

    def save(self) -> None:
        if not self.path:
            return
        directory = os.path.dirname(self.path) or "."
        os.makedirs(directory, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=directory, suffix=".tmp")
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump({"detected": [
                {k: v for k, v in asdict(item).items() if k not in ("x", "y")}
                for item in self.items]}, handle, indent=1)
        os.replace(tmp, self.path)

    def placed(self) -> List[Detection]:
        """The ones with a position in this run's `map`."""
        return [item for item in self.items if math.isfinite(item.x)]


def images_dir(detected_file: str) -> str:
    """Where the snapshots live: beside the detections file."""
    return os.path.join(os.path.dirname(detected_file) or ".", "detected_images")


def safe_image_name(name) -> bool:
    """A plain file name this code wrote — never a path."""
    return (isinstance(name, str) and name.endswith(".jpg") and "/" not in name
            and "\\" not in name and not name.startswith("."))
