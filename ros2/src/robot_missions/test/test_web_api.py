"""The web page's HTTP API against a stand-in robot. ROS-free; a real server on
an ephemeral port, real HTTP from urllib."""

import json
import os
import threading
import urllib.error
import urllib.request

import pytest

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


def lawn():
    ring = [[LON0, LAT0], [LON0 + D, LAT0], [LON0 + D, LAT0 + D], [LON0, LAT0 + D], [LON0, LAT0]]
    return {"type": "FeatureCollection", "features": [
        {"type": "Feature", "geometry": {"type": "Polygon", "coordinates": [ring]},
         "properties": {"kind": "area", "name": "cim"}}]}


@pytest.fixture
def web(tmp_path):
    robot = FakeRobot()
    site = tmp_path / "site.geojson"
    detected = tmp_path / "detected.json"
    api = WebApi(str(site), str(detected), robot)
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
    yield call, robot, site, detected
    server.shutdown()
    server.server_close()


def test_the_page_and_its_assets_are_served(web):
    call, *_ = web
    for path, marker in (("/", b"<title>Robot Saha</title>"), ("/app.js", b"/api/site"),
                         ("/style.css", b"#panel")):
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
