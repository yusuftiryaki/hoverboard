"""Image fields -> JPEG. ROS-free."""

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from robot_missions.camera import jpeg_from_image        # noqa: E402


def decode(jpeg):
    return cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)


@pytest.mark.parametrize("encoding, pixel, channels", [
    ("rgb8", (255, 0, 0), 3), ("bgr8", (0, 0, 255), 3),
    ("rgba8", (255, 0, 0, 255), 4), ("bgra8", (0, 0, 255, 255), 4)])
def test_colour_order_survives_whatever_the_encoding(encoding, pixel, channels):
    # Pure red in every layout must decode as red (BGR 0, 0, 255).
    data = bytes(pixel) * (8 * 6)
    out = decode(jpeg_from_image(encoding, 8, 6, 8 * channels, data))
    b, g, r = out[3, 4]
    assert r > 200 and g < 50 and b < 50


def test_row_padding_is_dropped():
    # step wider than width * 3: the padding bytes must not shear the picture.
    width, height, step = 5, 4, 20
    row = bytes((0, 255, 0) * width) + b"\xff" * (step - 3 * width)
    out = decode(jpeg_from_image("rgb8", width, height, step, row * height))
    assert out.shape == (4, 5, 3) and out[2, 2][1] > 200 and out[2, 2][2] < 60


def test_unknown_or_short_input_is_none():
    assert jpeg_from_image("yuv422", 4, 4, 8, bytes(32)) is None
    assert jpeg_from_image("rgb8", 4, 4, 12, bytes(10)) is None
