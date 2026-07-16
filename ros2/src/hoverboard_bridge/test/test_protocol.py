"""Protocol tests — pure Python, no ROS and no hardware needed.

These are the cheap insurance against the expensive bug: a framing or checksum
mismatch with firmware/esp32_bridge/src/main.cpp shows up on the bench as
"the motors twitch and stop", which is a miserable thing to debug next to a
36 V battery.
"""

import pytest

from hoverboard_bridge.protocol import (
    ESP_FEEDBACK_SIZE,
    PI_COMMAND_SIZE,
    SPEED_LIMIT,
    EspFeedback,
    FeedbackParser,
    pack_esp_feedback,
    pack_pi_command,
    unpack_esp_feedback,
)


def test_frame_sizes_match_the_packed_c_structs():
    # uint16 + 2*int16 + 2*uint8 + uint16
    assert PI_COMMAND_SIZE == 10
    # uint16 + 4*int16 + 2*uint8 + uint16
    assert ESP_FEEDBACK_SIZE == 14


def test_pi_command_starts_with_the_little_endian_marker():
    frame = pack_pi_command(0, 0)
    assert frame[0] == 0xCD and frame[1] == 0xAB


@pytest.mark.parametrize(
    "speed,steer",
    [(0, 0), (100, -50), (-300, 300), (7, 7), (-1, 1)],
)
def test_pi_command_checksum_matches_the_firmware_formula(speed, steer):
    frame = pack_pi_command(speed, steer)
    # Recompute the way main.cpp does: XOR of every 16-bit word, low 16 bits.
    words = [
        int.from_bytes(frame[i : i + 2], "little") for i in range(0, len(frame), 2)
    ]
    payload, checksum = words[:-1], words[-1]
    expected = 0
    for word in payload:
        expected ^= word
    assert checksum == expected & 0xFFFF


def test_pi_command_clamps_to_the_firmware_limits():
    frame = pack_pi_command(99999, -99999)
    speed = int.from_bytes(frame[2:4], "little", signed=True)
    steer = int.from_bytes(frame[4:6], "little", signed=True)
    assert speed == SPEED_LIMIT
    assert steer == -SPEED_LIMIT


def test_feedback_round_trip():
    original = EspFeedback(
        speed_l=-120,
        speed_r=118,
        bat_voltage=3712,
        board_temp=294,
        estop=True,
        watchdog_ok=False,
    )
    assert unpack_esp_feedback(pack_esp_feedback(original)) == original


def test_feedback_with_a_corrupt_byte_is_rejected():
    raw = bytearray(pack_esp_feedback(EspFeedback(10, 10, 3700, 300, False, True)))
    raw[3] ^= 0xFF
    assert unpack_esp_feedback(bytes(raw)) is None


def test_parser_extracts_frames_from_a_stream():
    frames = [
        EspFeedback(i, -i, 3700 + i, 300, False, True) for i in range(5)
    ]
    stream = b"".join(pack_esp_feedback(f) for f in frames)
    parser = FeedbackParser()
    assert parser.feed(stream) == frames
    assert parser.checksum_errors == 0


def test_parser_survives_leading_garbage_and_resyncs_after_a_bad_frame():
    good = EspFeedback(42, -42, 3700, 300, False, True)
    bad = bytearray(pack_esp_feedback(good))
    bad[5] ^= 0xFF  # corrupt payload -> checksum fails

    parser = FeedbackParser()
    # Garbage, then a broken frame, then a good one: only the good one survives
    # and the stream is not permanently desynced.
    out = parser.feed(b"\x00\xff\x13garbage" + bytes(bad) + pack_esp_feedback(good))
    assert out == [good]
    assert parser.checksum_errors == 1


def test_parser_handles_bytes_arriving_one_at_a_time():
    good = EspFeedback(7, 7, 3700, 300, False, True)
    parser = FeedbackParser()
    out = []
    for byte in pack_esp_feedback(good):
        out += parser.feed(bytes([byte]))
    assert out == [good]
