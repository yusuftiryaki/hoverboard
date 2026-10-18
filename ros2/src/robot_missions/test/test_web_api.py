"""The web page's HTTP API against a stand-in robot. ROS-free; a real server on
an ephemeral port, real HTTP from urllib."""

import json
import os
import threading
import urllib.error
import urllib.request

import pytest

from robot_missions.site import LatLon
from robot_missions.teach import Teacher
from robot_missions.teleop import TeleopGate
from robot_missions.web_api import WebApi, default_static_dir, make_server

LAT0, LON0 = 41.0, 29.0
D = 1e-4


class FakeRobot:
    def __init__(self):
        self.started = []
        self.commands = []
        self.reloads = 0
        self.accept = True

    def status(self):
        return {"robot": {"lat": LAT0, "lon": LON0, "heading_deg": 90.0},
                "mission": {"state": "IDLE"}, "supervisor": "NORMAL", "battery": None}

    def start_mission(self, spec):
        self.started.append(spec)
        return (True, "planning") if self.accept else (False, "a mission is already RUNNING")

    def command(self, name):
        self.commands.append(name)
        return True, name

    def reload_site(self):
        self.reloads += 1
        return True, "reloaded"

    def to_ll(self, points):
        """map metres -> lat/lon, flat, around (LAT0, LON0) — a stand-in for
        navsat_transform's /toLL."""
        return [LatLon(LAT0 + y * 8.993216e-06, LON0 + x * 1.191613e-05) for x, y in points]

    camera = True                       # False: no camera running
    frames = 0

    def camera_jpeg(self, wait_s=3.0, after=None):
        if not self.camera:
            return None
        self.frames += 1
        return self.frames, b"\xff\xd8JPEG" + bytes([self.frames % 256]) + b"\xff\xd9"


def lawn():
    ring = [[LON0, LAT0], [LON0 + D, LAT0], [LON0 + D, LAT0 + D], [LON0, LAT0 + D], [LON0, LAT0]]
    return {"type": "FeatureCollection", "features": [
        {"type": "Feature", "geometry": {"type": "Polygon", "coordinates": [ring]},
         "properties": {"kind": "area", "name": "cim"}}]}


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


@pytest.fixture
def web(tmp_path):
    robot = FakeRobot()
    site = tmp_path / "site.geojson"
    detected = tmp_path / "detected.json"
    clock = FakeClock()
    gate = TeleopGate(max_linear=0.6, max_angular=1.0, deadman_s=0.3, lease_s=2.0, clock=clock)
    teacher = Teacher(min_fixes=2)
    api = WebApi(str(site), str(detected), robot, gate, teacher)
    server = make_server(api, default_static_dir(), "127.0.0.1", 0)
    thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.05},
                              daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_address[1]}"

    def call(method, path, body=None):
        data = None if body is None else json.dumps(body).encode()
        request = urllib.request.Request(base + path, data=data, method=method,
                                         headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(request, timeout=5) as response:
                raw = response.read()
                kind = response.headers.get("Content-Type", "")
                return response.status, (json.loads(raw) if "json" in kind else raw)
        except urllib.error.HTTPError as exc:
            return exc.code, json.loads(exc.read())

    call.base = base
    call.gate, call.clock, call.api, call.teacher = gate, clock, api, teacher
    yield call, robot, site, detected
    server.shutdown()
    server.server_close()


def test_the_page_and_its_assets_are_served(web):
    call, *_ = web
    for path, marker in (("/", b"<title>Robot Saha</title>"), ("/app.js", b"/api/site"),
                         ("/style.css", b"#panel"), ("/drive.js", b"/api/teleop")):
        status, body = call("GET", path)
        assert status == 200 and marker in body, path


@pytest.mark.parametrize("path", ["/../setup.py", "/web_api.py", "/.hidden.js",
                                  "/%2e%2e/setup.py", "/sub/app.js"])
def test_nothing_outside_the_web_directory_is_served(web, path):
    call, *_ = web
    status, _ = call("GET", path)
    assert status == 404


def test_no_site_yet_is_an_empty_collection(web):
    call, *_ = web
    assert call("GET", "/api/site") == (200, {"type": "FeatureCollection", "features": []})


def test_a_valid_site_is_saved_backed_up_and_reloaded(web):
    call, robot, site, _ = web
    status, body = call("PUT", "/api/site", lawn())
    assert status == 200 and body["saved"] and body["reloaded"]
    assert json.loads(site.read_text()) == lawn() and robot.reloads == 1
    second = lawn()
    second["features"][0]["properties"]["name"] = "bahce"
    call("PUT", "/api/site", second)
    # The previous version is kept beside it.
    assert json.loads((site.parent / "site.geojson.bak").read_text()) == lawn()
    assert call("GET", "/api/site")[1] == second


def test_a_broken_site_is_refused_and_the_file_is_untouched(web):
    call, robot, site, _ = web
    call("PUT", "/api/site", lawn())
    bad = lawn()
    bad["features"][0]["properties"]["kind"] = "lawn"
    status, body = call("PUT", "/api/site", bad)
    assert status == 400 and "kind must be one of" in body["error"]
    assert json.loads(site.read_text()) == lawn() and robot.reloads == 1


def test_task_types_describe_the_form(web):
    call, *_ = web
    status, types = call("GET", "/api/task_types")
    assert status == 200 and set(types) == {"coverage", "patrol", "route", "goto"}
    overlap = types["coverage"]["params"]["overlap"]
    assert overlap["kind"] == "float" and overlap["high"] == 0.9
    assert types["patrol"]["params"]["mode"]["choices"] == ["perimeter", "random"]
    assert types["coverage"]["params"]["angle_deg"]["optional"] is True


def test_missions_and_commands_reach_the_robot(web):
    call, robot, *_ = web
    spec = {"type": "coverage", "area": "cim", "params": {"overlap": 0.3}}
    assert call("POST", "/api/mission", spec)[0] == 200
    assert robot.started == [spec]
    robot.accept = False
    status, body = call("POST", "/api/mission", spec)
    assert status == 409 and "RUNNING" in body["error"]
    for name in ("pause", "resume", "abort"):
        assert call("POST", f"/api/mission/{name}")[0] == 200
    assert robot.commands == ["pause", "resume", "abort"]
    assert call("POST", "/api/mission/explode")[0] == 404


def test_status_is_the_robots(web):
    call, *_ = web
    status, body = call("GET", "/api/status")
    assert status == 200 and body["supervisor"] == "NORMAL" and body["robot"]["lat"] == LAT0


def write_detections(detected, *items):
    detected.write_text(json.dumps({"detected": list(items)}), encoding="utf-8")


ROCK = {"radius_m": 0.25, "count": 3, "first_seen": 1.0, "last_seen": 2.0,
        "lat": LAT0 + D / 2, "lon": LON0 + D / 2}


def test_confirming_a_detection_makes_it_a_site_obstacle(web):
    call, robot, site, detected = web
    call("PUT", "/api/site", lawn())
    write_detections(detected, ROCK, dict(ROCK, lon=LON0))
    status, items = call("GET", "/api/detected")
    assert status == 200 and [i["index"] for i in items] == [0, 1]

    status, body = call("POST", "/api/detected/0/confirm", {"name": "kaya", "radius_m": 0.2})
    assert status == 200 and body["confirmed"]
    obstacle = json.loads(site.read_text())["features"][-1]
    assert obstacle["properties"] == {"kind": "obstacle", "name": "kaya", "radius_m": 0.2}
    assert obstacle["geometry"]["coordinates"] == [ROCK["lon"], ROCK["lat"]]
    # It left the detections; the other one is still there, now #0.
    remaining = call("GET", "/api/detected")[1]
    assert len(remaining) == 1 and remaining[0]["lon"] == LON0
    assert robot.reloads == 2


def test_confirming_needs_a_name_and_a_sane_radius(web):
    call, robot, site, detected = web
    call("PUT", "/api/site", lawn())
    write_detections(detected, ROCK)
    assert call("POST", "/api/detected/0/confirm", {"name": " "})[0] == 400
    assert call("POST", "/api/detected/0/confirm", {"name": "kaya", "radius_m": -1})[0] == 400
    assert call("POST", "/api/detected/5/confirm", {"name": "kaya"})[0] == 404
    # A name the site already uses for an obstacle: the site's own rule refuses it.
    call("POST", "/api/detected/0/confirm", {"name": "kaya"})
    write_detections(detected, ROCK)
    status, body = call("POST", "/api/detected/0/confirm", {"name": "kaya"})
    assert status == 400 and "both named" in body["error"]
    assert len(json.loads(detected.read_text())["detected"]) == 1   # untouched


def test_dropping_a_detection_forgets_it(web):
    call, robot, site, detected = web
    write_detections(detected, ROCK)
    assert call("POST", "/api/detected/0/drop")[0] == 200
    assert json.loads(detected.read_text())["detected"] == []
    assert not site.exists()                     # the site was never touched
    assert robot.reloads == 1


def test_a_body_that_is_not_json_is_a_400(web):
    call, *_ = web
    request = urllib.request.Request(call.base + "/api/site", data=b"{ nope", method="PUT")
    with pytest.raises(urllib.error.HTTPError) as caught:
        urllib.request.urlopen(request, timeout=5)
    assert caught.value.code == 400
    assert "not JSON" in json.loads(caught.value.read())["error"]


def test_an_empty_body_is_not_a_site(web):
    call, *_ = web
    status, body = call("PUT", "/api/site", None)
    assert status == 400 and "FeatureCollection" in body["error"]


def test_the_static_dir_is_found_from_the_source_tree():
    assert os.path.isfile(os.path.join(default_static_dir(), "index.html"))


def test_a_detection_photo_is_served_and_goes_with_the_detection(web, tmp_path):
    call, robot, site, detected = web
    images = tmp_path / "detected_images"
    images.mkdir()
    (images / "1700.jpg").write_bytes(b"\xff\xd8photo\xff\xd9")
    write_detections(detected, dict(ROCK, image="1700.jpg"), dict(ROCK, lon=LON0))
    status, body = call("GET", "/api/detected/0/image")
    assert status == 200 and body == b"\xff\xd8photo\xff\xd9"
    assert call("GET", "/api/detected/1/image")[0] == 404        # no photo
    assert call("POST", "/api/detected/0/drop")[0] == 200
    assert not (images / "1700.jpg").exists()                    # forgotten with it


@pytest.mark.parametrize("name", ["../site.geojson", "/etc/passwd", ".x.jpg", "a.png"])
def test_a_detection_cannot_point_its_photo_elsewhere(web, name):
    call, robot, site, detected = web
    write_detections(detected, dict(ROCK, image=name))
    assert call("GET", "/api/detected/0/image")[0] == 404


def test_the_camera_still_and_its_absence(web):
    call, robot, *_ = web
    status, body = call("GET", "/api/camera.jpg")
    assert status == 200 and body.startswith(b"\xff\xd8")
    robot.camera = False
    status, body = call("GET", "/api/camera.jpg")
    assert status == 503 and "camera" in body["error"]


def test_the_live_stream_is_mjpeg_frame_after_frame(web):
    call, robot, *_ = web
    response = urllib.request.urlopen(call.base + "/api/camera/stream", timeout=5)
    assert response.headers["Content-Type"].startswith("multipart/x-mixed-replace")
    data = b""
    while data.count(b"--frame") < 3:
        data += response.read1(4096)
    response.close()
    parts = [p for p in data.split(b"--frame") if p.strip()]
    assert all(b"Content-Type: image/jpeg" in p for p in parts[:2])
    # Different frames, not the same one repeated.
    assert parts[0].split(b"\r\n\r\n", 1)[1][:8] != parts[1].split(b"\r\n\r\n", 1)[1][:8]


# --- teleop (docs/missions.md 5a) -----------------------------------------------

def test_driving_from_the_page_reaches_the_gate_scaled_to_its_limits(web):
    call, *_ = web
    status, lease = call("POST", "/api/teleop/acquire", {})
    assert status == 200 and lease["max_linear"] == 0.6 and lease["lease_s"] == 2.0
    status, _ = call("POST", "/api/teleop", {"token": lease["token"], "linear": 0.5,
                                             "angular": -1.0})
    assert status == 200
    assert call.gate.output() == pytest.approx((0.3, -1.0))
    teleop = call("GET", "/api/status")[1]["teleop"]
    assert teleop["held"] and teleop["active"] and teleop["linear"] == pytest.approx(0.3)


def test_a_second_tab_cannot_take_the_wheels_or_drive_without_the_token(web):
    call, *_ = web
    token = call("POST", "/api/teleop/acquire", {})[1]["token"]
    status, body = call("POST", "/api/teleop/acquire", {})
    assert status == 409 and "someone else" in body["error"]
    for bad in ({"linear": 1.0, "angular": 0.0}, {"token": "nope", "linear": 1.0, "angular": 0.0}):
        assert call("POST", "/api/teleop", bad)[0] == 409
    assert call.gate.output() == (0.0, 0.0), "the holder's robot must stay stopped"
    assert call("POST", "/api/teleop/release", {"token": "nope"})[1] == {"released": False}
    assert call("POST", "/api/teleop/release", {"token": token})[1] == {"released": True}
    assert call.gate.output() is None
    assert call("POST", "/api/teleop/acquire", {})[0] == 200


@pytest.mark.parametrize("linear", ["1", None, True, float("nan"), [1]])
def test_a_malformed_command_is_rejected_and_changes_nothing(web, linear):
    call, *_ = web
    token = call("POST", "/api/teleop/acquire", {})[1]["token"]
    call("POST", "/api/teleop", {"token": token, "linear": 0.5, "angular": 0.0})
    # json.dumps writes NaN as a bare NaN token, which Python's json reads back.
    status, body = call("POST", "/api/teleop", {"token": token, "linear": linear, "angular": 0})
    assert status == 400 and "linear" in body["error"]
    assert call.gate.output() == pytest.approx((0.3, 0.0))


def test_a_page_that_goes_quiet_stops_the_robot_then_lets_go(web):
    call, *_ = web
    token = call("POST", "/api/teleop/acquire", {})[1]["token"]
    call("POST", "/api/teleop", {"token": token, "linear": 1.0, "angular": 0.0})
    call.clock.now += 0.29
    assert call.gate.output() == (0.6, 0.0)
    call.clock.now += 0.02                  # past deadman_s: zero, lease still held
    assert call.gate.output() == (0.0, 0.0)
    call.clock.now += 1.7                   # 2.01 s since the command: lease gone
    assert call.gate.output() is None
    status, body = call("POST", "/api/teleop", {"token": token, "linear": 1.0, "angular": 0.0})
    assert status == 409 and "acquire" in body["error"]


def test_teleop_turned_off_on_the_robot_says_so(web):
    call, *_ = web
    call.api.teleop = None
    status, body = call("POST", "/api/teleop/acquire", {})
    assert status == 409 and "turned off" in body["error"]
    assert call("GET", "/api/status")[1]["teleop"] is None


# --- teach by driving (docs/missions.md 5b) ----------------------------------------

def test_a_trace_is_recorded_finished_and_comes_back_as_an_unnamed_candidate(web):
    call, robot, site, _ = web
    assert call("POST", "/api/teach/trace", {"kind": "route"})[0] == 200
    for i in range(31):                                 # 3 m east, a pose every 0.1 m
        call.teacher.feed(LatLon(LAT0, LON0 + i * 1.191613e-06), 90.0, i * 0.1,
                          (i * 0.1, 0.0, 0.0), (0.0, 0.0, 0.0))
    teach = call("GET", "/api/status")[1]["teach"]
    assert teach["mode"] == "trace" and len(teach["points"]) >= 5
    status, body = call("POST", "/api/teach/finish", {})
    assert status == 200 and body["result"]["geometry"] == {
        "type": "LineString", "coordinates": [[LON0, LAT0], [LON0 + 3 * 1.191613e-05, LAT0]]}
    assert "name" not in body["result"]["properties"]
    # Nothing was written: the person names it and saves through PUT /api/site.
    assert not site.exists() and robot.reloads == 0
    assert call("POST", "/api/teach/take", {})[1]["result"]["id"] == body["result"]["id"]
    assert call("POST", "/api/teach/take", {})[1] == {"result": None}


def test_a_point_is_averaged_then_taken(web):
    call, *_ = web
    assert call("POST", "/api/teach/point", {"kind": "poi", "seconds": 1})[0] == 200
    for i in range(12):
        call.teacher.feed(LatLon(LAT0, LON0), None, i * 0.1)
    assert call("GET", "/api/status")[1]["teach"]["result_ready"]
    result = call("POST", "/api/teach/take", {})[1]["result"]
    assert result["properties"] == {"kind": "poi"}
    assert result["geometry"]["coordinates"] == [LON0, LAT0]


@pytest.mark.parametrize("path, body, message", [
    ("/api/teach/trace", {"kind": "station"}, "a trace is one of"),
    ("/api/teach/point", {"kind": "poi", "seconds": 0}, "seconds"),
    ("/api/teach/finish", {}, "no trace"),
    ("/api/teach/corner", {}, "start one first"),
])
def test_teach_mistakes_are_400_with_the_reason(web, path, body, message):
    call, *_ = web
    status, answer = call("POST", path, body)
    assert status == 400 and message in answer["error"]


def test_a_second_recording_waits_for_the_first(web):
    call, *_ = web
    call("POST", "/api/teach/trace", {"kind": "area"})
    status, body = call("POST", "/api/teach/point", {"kind": "station"})
    assert status == 400 and "already recording" in body["error"]
    assert call("POST", "/api/teach/cancel", {})[0] == 200
    assert call("POST", "/api/teach/point", {"kind": "station"})[0] == 200
