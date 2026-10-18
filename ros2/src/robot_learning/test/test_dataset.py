"""dataset.build_frames on hand-made series. ROS-free.

Times are seconds; every expected value below is read off the series by hand.
"""

import math

import numpy as np
import pytest

from robot_learning.dataset import (
    ACTION_NAMES, SOURCES, STATE_NAMES, Series, build_frames, marker_frames)

FPS = 10.0


def frames(**series):
    images = series.pop("images", Series())
    return build_frames({k: Series.of(v) for k, v in series.items()}, images, FPS, 100.0, 101.0)


def test_eleven_frames_for_one_second_at_10_hz_from_t0():
    f = frames()
    assert len(f.timestamp) == 11 and f.timestamp[0] == 0.0
    assert f.timestamp[-1] == pytest.approx(1.0)
    assert f.state.shape == (11, len(STATE_NAMES)) and f.action.shape == (11, len(ACTION_NAMES))


def test_state_is_the_latest_value_and_nan_before_the_first():
    f = frames(odom=[(100.25, 0.3, 0.0), (100.55, 0.4, 0.1)])
    v = f.state[:, STATE_NAMES.index("odom.v")]
    assert np.isnan(v[:3]).all()                  # t = 0.0, 0.1, 0.2: nothing yet
    assert list(v[3:6]) == [0.3, 0.3, 0.3]         # 0.3 .. 0.5
    assert list(v[6:]) == [0.4] * 5
    assert np.isnan(f.state[:, STATE_NAMES.index("gps.lat")]).all(), "missing is NaN, not 0"


def test_the_action_is_what_was_done_in_answer_to_the_frame():
    # A command at 100.15 answers frame 1 (t = 0.1): it lands in (0.1, 0.2].
    f = frames(teleop=[(100.15, 0.3, 0.0), (100.45, 0.0, 0.5)])
    assert f.action[0].tolist() == [0.0, 0.0]      # (0.0, 0.1]: nothing yet
    assert f.action[1].tolist() == [0.3, 0.0]
    assert f.action[2].tolist() == [0.3, 0.0]      # still in force (fresh)
    assert f.action[4].tolist() == [0.0, 0.5]      # (0.4, 0.5] has the new one
    assert SOURCES[f.action_source[1]] == "teleop" and f.teleop[1]


def test_a_stale_command_is_no_command():
    f = frames(teleop=[(100.05, 0.3, 0.0)])
    # Fresh until 100.55 (twist_mux's 0.5 s): frames whose interval ends by then.
    assert f.teleop[:5].all() and not f.teleop[5:].any()
    assert (f.action[5:] == 0).all() and (f.action_source[5:] == 0).all()


def test_teleop_outranks_nav_like_the_mux():
    f = frames(nav=[(100.0 + 0.1 * k + 0.05, 0.5, 0.0) for k in range(10)],
               teleop=[(100.35, -0.2, 0.0)])
    sources = [SOURCES[s] for s in f.action_source]
    assert sources[:3] == ["nav"] * 3
    assert sources[3:8] == ["teleop"] * 5           # 0.35 fresh until 0.85
    assert sources[8:10] == ["nav", "nav"]
    assert f.action[4].tolist() == [-0.2, 0.0] and f.action[1].tolist() == [0.5, 0.0]


def test_executed_is_kept_apart_from_the_label():
    # The bumper vetoed: the person asked 0.3, the bridge got 0.
    f = frames(teleop=[(100.15, 0.3, 0.0)], executed=[(100.16, 0.0, 0.0)])
    assert f.action[1].tolist() == [0.3, 0.0] and f.action_executed[1].tolist() == [0.0, 0.0]


def test_the_image_is_the_latest_stamped_before_the_frame():
    images = Series.of([(100.02, b"a"), (100.38, b"b")])
    f = frames(images=images)
    assert f.image_index[0] == -1 and math.isnan(f.image_age[0])
    assert list(f.image_index[1:4]) == [0, 0, 0] and f.image_age[1] == pytest.approx(0.08)
    assert list(f.image_index[4:]) == [1] * 7


def test_markers_count_from_ready():
    assert marker_frames([{"t": 7.76, "label": "ağaç"}, {"t": 1.0, "label": "erken"}],
                         ready_after_s=3.0, fps=10.0) == [
        {"frame": 48, "label": "ağaç"}, {"frame": 0, "label": "erken"}]


def test_nonsense_is_refused():
    with pytest.raises(ValueError):
        build_frames({}, Series(), 0.0, 0.0, 1.0)
    with pytest.raises(ValueError):
        build_frames({}, Series(), 10.0, 1.0, 1.0)


@pytest.mark.parametrize("first, last, started, ready, expected", [
    (1000.0, 1010.0, 999.0, 3.0, 1002.0),        # same clock: frames from READY
    (1000.0, 1010.0, 999.0, 0.0, 1000.0),        # never before the bag's first message
    (1000.0, 1010.0, 999.0, 30.0, 1010.0),       # never after its last
    (12.0, 22.0, 1_760_000_000.0, 3.0, 12.0),    # sim time vs wall time: the bag's start
    (12.0, 22.0, None, 3.0, 12.0),
])
def test_frames_start_at_ready_only_when_the_clocks_agree(first, last, started, ready, expected):
    from robot_learning.export import start_of_frames
    assert start_of_frames(first, last, started, ready) == expected


def test_the_expert_label_is_its_own_column_and_nan_where_it_was_silent():
    # The expert labels while the policy (here: nav) drives — DAgger (6d).
    f = frames(nav=[(100.0 + 0.1 * k + 0.05, 0.5, 0.0) for k in range(10)],
               expert=[(100.15, 0.2, -0.1)])
    assert f.action[1].tolist() == [0.5, 0.0] and SOURCES[f.action_source[1]] == "nav"
    assert f.action_expert[1].tolist() == [0.2, -0.1]
    assert np.isnan(f.action_expert[0]).all()                   # before it spoke
    assert np.isnan(f.action_expert[-1]).all()                  # stale after 0.5 s
