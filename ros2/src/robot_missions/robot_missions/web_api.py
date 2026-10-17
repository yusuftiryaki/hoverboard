"""The web page's HTTP API. ROS-free: the robot side is a `backend` object.

    GET  /                          the page (web/index.html, app.js, style.css)
    GET  /api/site                  site.geojson (an empty FeatureCollection if none)
    PUT  /api/site                  replace it — validated first, never half-written
    GET  /api/task_types            the mission registry (mission.TASK_TYPES), for the form
    GET  /api/status                robot, mission, battery, supervisor (backend.status())
    POST /api/mission               {"type": ..., ...}  -> mission_runner ~/start
    POST /api/mission/pause|resume|abort
    GET  /api/detected              the bumper's finds (detected_obstacles.json)
    GET  /api/detected/<i>/image    the camera's frame from the press (JPEG), if any
    GET  /api/camera.jpg            the camera now (backend.camera_jpeg)
    GET  /api/camera/stream         the camera live, MJPEG (multipart/x-mixed-replace)
    POST /api/detected/<i>/confirm  {"name": ..., "radius_m": ...} -> a site obstacle
    POST /api/detected/<i>/drop     forget it

Edits are files: the site file and the detections file, each written
atomically, the site's previous version kept as site.geojson.bak. After each
edit the backend is asked to reload (site_map_server, battery_supervisor);
mission_runner reads the site afresh at every start.

⚠️ No authentication. Anyone on the robot's network can start a mission. It
listens on the LAN because a laptop in the garden is the point; keep the robot
off networks you do not control (docs/missions.md, 4. aşama).
"""

from __future__ import annotations

import json
import math
import os
import re
import tempfile
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, Tuple

from robot_missions.detected import images_dir, safe_image_name
from robot_missions.mission import TASK_TYPES
from robot_missions.site import SiteError, parse_site

STATIC_TYPES = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8",
                ".css": "text/css; charset=utf-8"}
MAX_BODY = 5 * 1024 * 1024
STREAM_MAX_S = 600.0


class ApiError(Exception):
    def __init__(self, message: str, status: HTTPStatus = HTTPStatus.BAD_REQUEST):
        super().__init__(message)
        self.status = status


def _atomic_write(path: str, text: str) -> None:
    directory = os.path.dirname(path) or "."
    os.makedirs(directory, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=directory, suffix=".tmp")
    with os.fdopen(fd, "w", encoding="utf-8") as handle:
        handle.write(text)
    os.replace(tmp, path)


class WebApi:
    def __init__(self, site_file: str, detected_file: str, backend):
        self.site_file = site_file
        self.detected_file = detected_file
        self.backend = backend
        self._lock = threading.Lock()   # one file edit at a time

    # --- site -----------------------------------------------------------------

    def get_site(self) -> Dict[str, Any]:
        if not self.site_file or not os.path.exists(self.site_file):
            return {"type": "FeatureCollection", "features": []}
        with open(self.site_file, encoding="utf-8") as handle:
            return json.load(handle)

    def put_site(self, document: Any) -> Dict[str, Any]:
        if not self.site_file:
            raise ApiError("the robot has no site_file configured", HTTPStatus.CONFLICT)
        try:
            site = parse_site(document)          # the robot's own reader decides
        except SiteError as exc:
            raise ApiError(f"site rejected: {exc}") from exc
        with self._lock:
            if os.path.exists(self.site_file):
                with open(self.site_file, encoding="utf-8") as handle:
                    _atomic_write(self.site_file + ".bak", handle.read())
            _atomic_write(self.site_file, json.dumps(document, indent=1, ensure_ascii=False))
        reload_ok, message = self.backend.reload_site()
        return {"saved": True, "features": sum(1 for _ in document["features"]),
                "empty": site.is_empty(), "reloaded": reload_ok, "reload_message": message}

    # --- missions -------------------------------------------------------------

    @staticmethod
    def task_types() -> Dict[str, Any]:
        out = {}
        for name, task in TASK_TYPES.items():
            out[name] = {"on": task.on, "describe": task.describe, "params": {
                key: {"default": p.default, "kind": p.kind.__name__, "low": p.low,
                      "high": p.high, "choices": list(p.choices), "optional": p.optional}
                for key, p in task.params.items()}}
        return out

    def start_mission(self, spec: Any) -> Dict[str, Any]:
        if not isinstance(spec, dict):
            raise ApiError("a mission is a JSON object")
        ok, message = self.backend.start_mission(spec)
        if not ok:
            raise ApiError(message, HTTPStatus.CONFLICT)
        return {"accepted": True, "message": message}

    def command(self, name: str) -> Dict[str, Any]:
        ok, message = self.backend.command(name)
        if not ok:
            raise ApiError(message, HTTPStatus.CONFLICT)
        return {"ok": True, "message": message}

    # --- detections -----------------------------------------------------------

    def _read_detected(self) -> Dict[str, Any]:
        if not self.detected_file or not os.path.exists(self.detected_file):
            return {"detected": []}
        with open(self.detected_file, encoding="utf-8") as handle:
            return json.load(handle)

    def get_detected(self):
        items = self._read_detected().get("detected", [])
        return [dict(item, index=i) for i, item in enumerate(items)]

    def confirm_detected(self, index: int, body: Any) -> Dict[str, Any]:
        """A person says: this is a real obstacle. It becomes a site obstacle
        (lat/lon, a name, its real radius) and leaves the detections."""
        if not isinstance(body, dict):
            raise ApiError("body must be a JSON object")
        name = body.get("name")
        if not isinstance(name, str) or not name.strip():
            raise ApiError("the obstacle needs a name")
        with self._lock:
            raw = self._read_detected()
            items = raw.get("detected", [])
            if not 0 <= index < len(items):
                raise ApiError(f"no detection #{index}", HTTPStatus.NOT_FOUND)
            item = items[index]
            if item.get("lat") is None or item.get("lon") is None:
                raise ApiError("this detection has no lat/lon yet")
            radius = body.get("radius_m", item.get("radius_m"))
            if (not isinstance(radius, (int, float)) or isinstance(radius, bool)
                    or not math.isfinite(radius) or radius <= 0):
                raise ApiError("radius_m must be a positive number")
            site = self.get_site()
            site.setdefault("features", []).append({
                "type": "Feature",
                "geometry": {"type": "Point", "coordinates": [item["lon"], item["lat"]]},
                "properties": {"kind": "obstacle", "name": name.strip(),
                               "radius_m": float(radius)}})
            try:
                parse_site(site)
            except SiteError as exc:
                raise ApiError(f"site rejected: {exc}") from exc
            if not self.site_file:
                raise ApiError("the robot has no site_file configured", HTTPStatus.CONFLICT)
            if os.path.exists(self.site_file):
                with open(self.site_file, encoding="utf-8") as handle:
                    _atomic_write(self.site_file + ".bak", handle.read())
            _atomic_write(self.site_file, json.dumps(site, indent=1, ensure_ascii=False))
            self._forget_image(items[index])
            del items[index]
            _atomic_write(self.detected_file, json.dumps(raw, indent=1))
        ok, message = self.backend.reload_site()
        return {"confirmed": True, "reloaded": ok, "reload_message": message}

    def drop_detected(self, index: int) -> Dict[str, Any]:
        with self._lock:
            raw = self._read_detected()
            items = raw.get("detected", [])
            if not 0 <= index < len(items):
                raise ApiError(f"no detection #{index}", HTTPStatus.NOT_FOUND)
            self._forget_image(items[index])
            del items[index]
            _atomic_write(self.detected_file, json.dumps(raw, indent=1))
        ok, message = self.backend.reload_site()
        return {"dropped": True, "reloaded": ok, "reload_message": message}

    def detected_image(self, index: int) -> bytes:
        items = self._read_detected().get("detected", [])
        if not 0 <= index < len(items):
            raise ApiError(f"no detection #{index}", HTTPStatus.NOT_FOUND)
        name = items[index].get("image")
        path = os.path.join(images_dir(self.detected_file), name) if safe_image_name(name) else ""
        if not path or not os.path.isfile(path):
            raise ApiError("this detection has no picture", HTTPStatus.NOT_FOUND)
        with open(path, "rb") as handle:
            return handle.read()

    def _forget_image(self, item: Dict[str, Any]) -> None:
        name = item.get("image")
        if safe_image_name(name):
            try:
                os.remove(os.path.join(images_dir(self.detected_file), name))
            except OSError:
                pass


def make_handler(api: WebApi, static_dir: str):
    routes_post = {
        re.compile(r"^/api/mission$"): lambda m, body: api.start_mission(body),
        re.compile(r"^/api/mission/(pause|resume|abort)$"):
            lambda m, body: api.command(m.group(1)),
        re.compile(r"^/api/detected/(\d+)/confirm$"):
            lambda m, body: api.confirm_detected(int(m.group(1)), body),
        re.compile(r"^/api/detected/(\d+)/drop$"):
            lambda m, body: api.drop_detected(int(m.group(1))),
    }

    class Handler(BaseHTTPRequestHandler):
        server_version = "hoverbot"

        def log_message(self, fmt, *args):     # quiet: the node logs what matters
            pass

        def _send(self, status: HTTPStatus, payload: Any) -> None:
            data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def _body(self) -> Any:
            length = int(self.headers.get("Content-Length") or 0)
            if length > MAX_BODY:
                raise ApiError("body too large", HTTPStatus.REQUEST_ENTITY_TOO_LARGE)
            raw = self.rfile.read(length) if length else b""
            try:
                return json.loads(raw.decode("utf-8")) if raw else {}
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise ApiError(f"body is not JSON: {exc}") from exc

        def _guard(self, fn) -> None:
            try:
                self._send(HTTPStatus.OK, fn())
            except ApiError as exc:
                self._send(exc.status, {"error": str(exc)})
            except Exception as exc:          # noqa: BLE001 — never kill the server
                self._send(HTTPStatus.INTERNAL_SERVER_ERROR, {"error": repr(exc)})

        def _send_jpeg(self, data: bytes, cache: bool = False) -> None:
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(data)))
            # A detection's photo never changes (the list re-polls it every
            # few seconds); the camera's always does.
            self.send_header("Cache-Control", "max-age=3600" if cache else "no-store")
            self.end_headers()
            self.wfile.write(data)

        def _image(self, index: int) -> None:
            try:
                self._send_jpeg(api.detected_image(index), cache=True)
            except ApiError as exc:
                self._send(exc.status, {"error": str(exc)})

        def _camera_still(self) -> None:
            frame = api.backend.camera_jpeg(wait_s=3.0)
            if frame is None:
                return self._send(HTTPStatus.SERVICE_UNAVAILABLE,
                                  {"error": "no camera picture (is the camera running?)"})
            self._send_jpeg(frame[1])

        def _camera_stream(self) -> None:
            """MJPEG: every new frame as a part, until the browser goes away
            (or STREAM_MAX_S, so a forgotten tab does not stream forever)."""
            first = api.backend.camera_jpeg(wait_s=3.0)
            if first is None:
                return self._send(HTTPStatus.SERVICE_UNAVAILABLE,
                                  {"error": "no camera picture (is the camera running?)"})
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            seq, frame = first
            ends = time.monotonic() + STREAM_MAX_S
            try:
                while time.monotonic() < ends:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     + f"Content-Length: {len(frame)}\r\n\r\n".encode()
                                     + frame + b"\r\n")
                    self.wfile.flush()
                    nxt = api.backend.camera_jpeg(wait_s=3.0, after=seq)
                    if nxt is None:
                        break                  # the camera stopped
                    seq, frame = nxt
            except (BrokenPipeError, ConnectionResetError):
                pass                           # the tab was closed

        def do_GET(self) -> None:             # noqa: N802
            path = self.path.split("?", 1)[0]
            match = re.match(r"^/api/detected/(\d+)/image$", path)
            if match:
                return self._image(int(match.group(1)))
            if path == "/api/camera.jpg":
                return self._camera_still()
            if path == "/api/camera/stream":
                return self._camera_stream()
            if path == "/api/site":
                return self._guard(api.get_site)
            if path == "/api/task_types":
                return self._guard(api.task_types)
            if path == "/api/status":
                return self._guard(api.backend.status)
            if path == "/api/detected":
                return self._guard(api.get_detected)
            self._static(path)

        def do_PUT(self) -> None:             # noqa: N802
            if self.path.split("?", 1)[0] == "/api/site":
                return self._guard(lambda: api.put_site(self._body()))
            self._send(HTTPStatus.NOT_FOUND, {"error": "no such endpoint"})

        def do_POST(self) -> None:            # noqa: N802
            path = self.path.split("?", 1)[0]
            for pattern, handler in routes_post.items():
                match = pattern.match(path)
                if match:
                    return self._guard(lambda: handler(match, self._body()))
            self._send(HTTPStatus.NOT_FOUND, {"error": "no such endpoint"})

        def _static(self, path: str) -> None:
            name = "index.html" if path in ("", "/") else path.lstrip("/")
            # Only plain file names from the web directory — no paths at all.
            if "/" in name or "\\" in name or name.startswith("."):
                return self._send(HTTPStatus.NOT_FOUND, {"error": "not found"})
            full = os.path.join(static_dir, name)
            kind = STATIC_TYPES.get(os.path.splitext(name)[1])
            if kind is None or not os.path.isfile(full):
                return self._send(HTTPStatus.NOT_FOUND, {"error": "not found"})
            with open(full, "rb") as handle:
                data = handle.read()
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

    return Handler


def make_server(api: WebApi, static_dir: str, host: str, port: int) -> ThreadingHTTPServer:
    server = ThreadingHTTPServer((host, port), make_handler(api, static_dir))
    server.daemon_threads = True
    return server


def default_static_dir() -> str:
    """robot_missions/web next to this file (source tree), or the installed share."""
    here = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")
    if os.path.isdir(here):
        return here
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("robot_missions"), "web")
    except Exception:                         # noqa: BLE001
        return here


__all__: Tuple[str, ...] = ("WebApi", "ApiError", "make_server", "default_static_dir")
