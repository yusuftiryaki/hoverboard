"""episode.Recorder with a stand-in recorder process. ROS-free.

The stand-in behaves like `ros2 bag record` where it matters here: it creates
the bag directory, writes data while running, and on SIGINT writes
metadata.yaml and exits 0. Variants crash, or ignore SIGINT. The real recorder
is exercised in robot_sim/test/test_learning_stack.py.
"""

import os
import subprocess
import sys
import time

import pytest
import yaml

from robot_learning.episode import (
    BAG_DIR, EPISODE_FILE, LOG_FILE, EpisodeError, Recorder, is_episode_name, load_profile,
    record_command, slug)

FAKE = r"""
import os, signal, sys, time
bag, behaviour = sys.argv[1], sys.argv[2]
os.makedirs(bag)
def done(*_):
    open(os.path.join(bag, "metadata.yaml"), "w").write("rosbag2_bagfile_information: {}\n")
    sys.exit(0)
signal.signal(signal.SIGINT, signal.SIG_IGN if behaviour == "stubborn" else done)
print("recording", flush=True)
if behaviour == "crash":
    print("boom: storage plugin exploded", flush=True)
    sys.exit(3)
with open(os.path.join(bag, "bag_0.mcap"), "ab") as f:
    while True:
        f.write(b"x" * 1000); f.flush(); time.sleep(0.05)
"""


def fake_spawn(behaviour="normal", seen=None):
    def spawn(argv, directory):
        if seen is not None:
            seen.append(argv)
        bag = argv[argv.index("-o") + 1]
        log = open(os.path.join(directory, LOG_FILE), "wb")
        return subprocess.Popen([sys.executable, "-c", FAKE, bag, behaviour], stdout=log,
                                stderr=subprocess.STDOUT, start_new_session=True)
    return spawn


@pytest.fixture
def make(tmp_path):
    made = []

    def _make(behaviour="normal", **kwargs):
        kwargs.setdefault("min_free_bytes", 0)
        recorder = Recorder(str(tmp_path / "episodes"), ["/odom", "/cmd_vel_teleop"],
                            spawn=fake_spawn(behaviour, kwargs.pop("seen", None)),
                            stop_timeout_s=kwargs.pop("stop_timeout_s", 5.0), **kwargs)
        made.append(recorder)
        return recorder

    yield _make
    for recorder in made:
        try:
            recorder.shutdown()
        except Exception:            # noqa: BLE001
            pass


def episode_of(recorder, name):
    with open(os.path.join(recorder.episodes_dir, name, EPISODE_FILE), encoding="utf-8") as f:
        return yaml.safe_load(f)


def wait_for(predicate, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.05)
    return False


# --- pure helpers ----------------------------------------------------------------

def test_slug_is_ascii_and_short():
    assert slug("Kenar Dibi Biçme — ağaç çevresi!") == "kenar-dibi-bicme-agac-cevresi"
    assert slug("ŞÜĞİÖÇ ışık") == "sugioc-isik"
    assert slug("!!!") == "bolum"
    assert len(slug("a" * 200)) == 40


@pytest.mark.parametrize("name, ok", [
    ("20261018-101530_kenar-bicme", True), ("20261018-101530_kenar-bicme-2", True),
    ("../etc", False), ("20261018-101530_x/../../y", False), ("bag", False), (None, False)])
def test_is_episode_name_only_accepts_names_this_module_makes(name, ok):
    assert is_episode_name(name) is ok


def test_record_command_is_mcap_split_and_names_every_topic():
    argv = record_command("/e/bag", ["/a", "/b"], 60, use_sim_time=True)
    assert argv[:3] == ["ros2", "bag", "record"]
    assert argv[argv.index("-o") + 1] == "/e/bag"
    assert argv[argv.index("-s") + 1] == "mcap"
    assert argv[argv.index("--max-bag-duration") + 1] == "60"
    assert argv[argv.index("--storage-preset-profile") + 1] == "zstd_fast"
    assert "--use-sim-time" in argv
    assert argv[argv.index("--topics") + 1:] == ["/a", "/b"]
    assert "--use-sim-time" not in record_command("/e/bag", ["/a"], 60, use_sim_time=False)


def test_profile_adds_sim_topics_only_in_the_sim(tmp_path):
    path = tmp_path / "topics.yaml"
    path.write_text("learning_topics:\n  always: [/odom, /imu/data]\n  sim: [/ground_truth, /odom]\n")
    assert load_profile(str(path), sim=False) == ["/odom", "/imu/data"]
    assert load_profile(str(path), sim=True) == ["/odom", "/imu/data", "/ground_truth"]


@pytest.mark.parametrize("text", ["learning_topics:\n  always: [odom]\n",
                                  "learning_topics:\n  always: []\n"])
def test_a_bad_profile_is_refused(tmp_path, text):
    path = tmp_path / "topics.yaml"
    path.write_text(text)
    with pytest.raises(EpisodeError):
        load_profile(str(path), sim=False)


def test_the_real_profile_loads_and_records_the_action_labels():
    """docs/learning.md decision 3: who wanted what, and what happened."""
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "..", "..", "robot_bringup", "config", "learning_topics.yaml")
    topics = load_profile(path, sim=True)
    for needed in ("/cmd_vel_teleop", "/cmd_vel_nav", "/cmd_vel", "/teleop/state", "/odom",
                   "/joint_states", "/camera/jpeg", "/tf", "/tf_static", "/learning/marker",
                   "/robot_description", "/ground_truth"):
        assert needed in topics, needed
    assert "/ground_truth" not in load_profile(path, sim=False)
    assert "/camera/image_raw" not in topics, "raw camera is ~27 MB/s: record the JPEG"


# --- the recorder ------------------------------------------------------------------

def test_an_episode_is_recorded_stopped_and_described(make):
    seen = []
    recorder = make(seen=seen, context={"git": "abc123"})
    name = recorder.start({"task": "Kenar biçme", "mode": "demo", "note": "ilk deneme"})
    assert is_episode_name(name) and name.endswith("_kenar-bicme")
    assert seen[0][seen[0].index("--topics") + 1:] == ["/odom", "/cmd_vel_teleop"]
    assert episode_of(recorder, name)["status"] == "recording"
    status = recorder.status()
    assert status["recording"] and status["episode"] == name and status["mode"] == "demo"
    assert wait_for(lambda: os.path.exists(
        os.path.join(recorder.episodes_dir, name, BAG_DIR, "bag_0.mcap")))
    recorder.mark({"label": "ağaç başladı"})
    assert recorder.stop({"outcome": "success", "note": "temiz"}) == name

    episode = episode_of(recorder, name)
    assert episode["status"] == "done" and episode["outcome"] == "success"
    assert episode["task"] == "Kenar biçme" and episode["outcome_note"] == "temiz"
    assert episode["markers"][0]["label"] == "ağaç başladı"
    assert episode["recorder_exit"] == 0 and episode["size_bytes"] > 0
    assert episode["context"] == {"git": "abc123"} and episode["duration_s"] >= 0
    assert os.path.exists(os.path.join(recorder.episodes_dir, name, BAG_DIR, "metadata.yaml")), \
        "SIGINT must let the recorder close its bag"
    assert recorder.status()["recording"] is False


def test_one_episode_at_a_time(make):
    recorder = make()
    recorder.start({"task": "a"})
    with pytest.raises(EpisodeError, match="already recording"):
        recorder.start({"task": "b"})


@pytest.mark.parametrize("spec, message", [
    ({}, "needs a task"), ({"task": "  "}, "needs a task"), ({"task": "x" * 101}, "needs a task"),
    ({"task": "a", "mode": "dream"}, "mode must be"), ({"task": "a", "note": 3}, "note"),
    ("not a dict", "JSON object")])
def test_a_bad_start_is_refused_and_records_nothing(make, spec, message):
    recorder = make()
    with pytest.raises(EpisodeError, match=message):
        recorder.start(spec)
    assert os.listdir(recorder.episodes_dir) == []


def test_stop_and_mark_need_a_recording_and_a_real_outcome(make):
    recorder = make()
    with pytest.raises(EpisodeError, match="not recording"):
        recorder.stop({"outcome": "success"})
    with pytest.raises(EpisodeError, match="not recording"):
        recorder.mark({"label": "x"})
    recorder.start({"task": "a"})
    with pytest.raises(EpisodeError, match="outcome must be"):
        recorder.stop({"outcome": "disk_full"})       # the robot's word, not the page's
    with pytest.raises(EpisodeError, match="label"):
        recorder.mark({"label": ""})


def test_no_room_no_episode(make):
    recorder = make(min_free_bytes=10**18)
    with pytest.raises(EpisodeError, match="GB free"):
        recorder.start({"task": "a"})


def test_the_disk_filling_up_stops_the_recording(make):
    recorder = make()
    name = recorder.start({"task": "a"})
    assert wait_for(lambda: os.path.exists(
        os.path.join(recorder.episodes_dir, name, BAG_DIR, "bag_0.mcap")))
    recorder.min_free_bytes = 10**18                  # the disk "fills"
    assert recorder.poll() == "disk_full"
    episode = episode_of(recorder, name)
    assert episode["outcome"] == "disk_full" and episode["recorder_exit"] == 0
    assert "disk" in recorder.status()["last_error"]


def test_a_recorder_that_dies_ends_the_episode_as_crashed_with_its_last_words(make):
    recorder = make("crash")
    name = recorder.start({"task": "a"})
    assert wait_for(lambda: recorder.poll() == "crashed")
    episode = episode_of(recorder, name)
    assert episode["outcome"] == "crashed" and episode["recorder_exit"] == 3
    assert "storage plugin exploded" in episode["outcome_note"]
    assert recorder.status()["recording"] is False
    recorder.start({"task": "b"})                     # and the next one can start


def test_a_recorder_that_ignores_sigint_is_killed_and_the_note_says_reindex(make):
    recorder = make("stubborn", stop_timeout_s=0.5)
    name = recorder.start({"task": "a"})
    time.sleep(0.3)
    recorder.stop({"outcome": "aborted"})
    episode = episode_of(recorder, name)
    assert "reindex" in episode["outcome_note"] and episode["recorder_exit"] != 0


def test_a_snapshot_lands_in_the_episode_even_after_it_ended(make, tmp_path):
    def snapshot(directory):
        time.sleep(0.3)
        open(os.path.join(directory, "site.geojson"), "w").write("{}")
        return {"files": ["site.geojson"]}

    recorder = make(snapshot=snapshot)
    name = recorder.start({"task": "a"})
    recorder.stop({"outcome": "success"})             # before the snapshot finished
    assert wait_for(lambda: (episode_of(recorder, name) or {}).get("snapshot") is not None)
    assert episode_of(recorder, name)["snapshot"] == {"files": ["site.geojson"]}
    assert episode_of(recorder, name)["outcome"] == "success"


def test_a_failing_snapshot_never_loses_the_episode(make):
    def snapshot(directory):
        raise RuntimeError("param dump timed out")

    recorder = make(snapshot=snapshot)
    name = recorder.start({"task": "a"})
    assert wait_for(lambda: episode_of(recorder, name).get("snapshot") is not None)
    assert "param dump timed out" in episode_of(recorder, name)["snapshot"]["error"]
    recorder.stop({"outcome": "success"})


def test_two_episodes_in_the_same_second_get_different_names(make):
    recorder = make(clock=lambda: 1_760_000_000.0)
    first = recorder.start({"task": "a"})
    recorder.stop({"outcome": "success"})
    second = recorder.start({"task": "a"})
    assert first != second and second == first + "-2"


def test_ready_once_the_recorder_subscribes_everything_live(make):
    now = [1_760_000_000.0]
    recorder = make(clock=lambda: now[0])
    name = recorder.start({"task": "a"})
    assert recorder.status()["ready"] is False
    now[0] += 1.0
    recorder.set_waiting(["/odom", "/camera/jpeg"])
    assert recorder.status()["waiting_for"] == ["/camera/jpeg", "/odom"]
    assert not recorder.ready
    now[0] += 2.5
    recorder.set_waiting([])
    assert recorder.ready and recorder.status()["ready"]
    recorder.set_waiting(["/late"])                     # once ready, stays ready
    assert recorder.ready
    assert episode_of(recorder, name)["ready_after_s"] == 3.5
    recorder.stop({"outcome": "success"})
    recorder.start({"task": "b"})
    assert not recorder.ready, "readiness is per episode"
