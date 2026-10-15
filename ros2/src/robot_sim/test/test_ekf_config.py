"""The accelerometer stays out of both EKFs until robot_localization is fixed.

⚠️ robot_localization 3.8.3 — the Jazzy apt release, and so what the Pi runs —
never initialises RosFilter::angular_acceleration_, last_state_twist_rot_ or
last_diff_time_, and prepareAcceleration adds
    imu_origin x angular_acceleration_
to every accelerometer measurement. Our IMU is mounted above base_link, so that
term is whatever the heap held. The first CI run lost ekf_local to 1e148 m/s
within 33 ms of startup and Nav2 aborted every goal; nothing logged a thing. It
never showed on the dev box because it depends on heap contents, and it
vanished under the filter's own debug output. valgrind memcheck finds it on
every run and finds nothing once ax is off. Fixed upstream in a8318cc3 (#945).

So this is a tripwire, not a style rule. A higher version number is NOT proof
of the fix: 3.9.0-3.9.4 and 3.10.0 are newer than 3.8.3 and still have the bug.
Accel fusion is allowed only on a version listed in KNOWN_FIXED, and a version
goes there only after someone has read its source or changelog for #945.

The config check is plain text and runs everywhere, ROS or not. Only the version
lookup needs ROS; without it the version is unknown, which counts as unfixed.
See the header of robot_bringup/config/ekf.yaml.
"""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

yaml = pytest.importorskip("yaml")

EKF_YAML = (Path(__file__).resolve().parents[2]
            / "robot_bringup" / "config" / "ekf.yaml")

# robot_localization's 15-element *_config order.
STATE = ("x", "y", "z", "roll", "pitch", "yaw",
         "vx", "vy", "vz", "vroll", "vpitch", "vyaw",
         "ax", "ay", "az")
ACCEL = {"ax", "ay", "az"}

# Versions whose source has been CHECKED for a8318cc3 / #945. Empty today: no
# Jazzy release carries it (rosdistro jazzy: 3.8.3-1).
KNOWN_FIXED = set()


def accelerometer_hazards(config):
    """Every way a config would feed the uninitialised angular acceleration
    into a filter or out of it. Malformed *_config lists fail loudly here:
    skipping them would let a typo switch the check off."""
    found = []
    for name, node in config.items():
        params = node.get("ros__parameters", {})
        for key, flags in params.items():
            if not key.endswith("_config"):
                continue
            assert len(flags) == len(STATE), (
                f"{name}.{key} has {len(flags)} entries, robot_localization "
                f"expects {len(STATE)}")
            fused = sorted(f for f, on in zip(STATE, flags) if on and f in ACCEL)
            if fused:
                found.append(f"{name}.{key[:-len('_config')]} fuses {fused}")
        # publishes angular_acceleration_ itself on accel/filtered
        if params.get("publish_acceleration"):
            found.append(f"{name}.publish_acceleration is true")
    return found


def installed_robot_localization_version():
    """'3.8.3', or None without ROS — and None counts as unfixed."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory("robot_localization"))
    except Exception:
        return None
    return ET.parse(share / "package.xml").getroot().findtext("version").strip()


def test_the_check_sees_every_way_in():
    """Counter-test: a check that never finds anything would pass forever."""
    config = {"ekf_local": {"ros__parameters": {
        "imu0_config": [False] * 12 + [True, False, False],
        "odom0_config": [False] * 6 + [True] + [False] * 8,
        "publish_acceleration": True,
    }}}
    assert accelerometer_hazards(config) == [
        "ekf_local.imu0 fuses ['ax']",
        "ekf_local.publish_acceleration is true",
    ]


def test_a_malformed_config_fails_instead_of_slipping_past():
    config = {"ekf_local": {"ros__parameters": {"imu0_config": [True] * 14}}}
    with pytest.raises(AssertionError, match="expects 15"):
        accelerometer_hazards(config)


def test_no_filter_touches_the_accelerometer_on_an_unfixed_robot_localization():
    version = installed_robot_localization_version()
    hazards = accelerometer_hazards(yaml.safe_load(EKF_YAML.read_text()))
    assert version in KNOWN_FIXED or not hazards, (
        f"robot_localization {version or '(unknown: ROS not sourced)'} is not "
        f"known to be fixed, and {hazards}. Unfixed releases add an "
        "UNINITIALISED angular acceleration to every accel measurement (our "
        "IMU is above base_link, so it does not cancel): the EKF diverges at "
        "startup, silently, depending on heap contents. Turn it off, or add "
        "the installed version to KNOWN_FIXED after checking its source for "
        "upstream a8318cc3 (#945). See ekf.yaml.")
