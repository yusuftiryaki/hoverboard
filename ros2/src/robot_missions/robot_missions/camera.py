"""sensor_msgs/Image fields -> JPEG bytes. ROS-free (numpy + OpenCV).

For the bumper's snapshot (site_map_server) and the web page's live view
(web_ui). Raw images, not image_transport's compressed topic: the compressed
plugin is not installed here, and the frame is encoded once, where it is used.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

# encoding -> (channels, OpenCV conversion to BGR, or None if already BGR/grey)
_LAYOUTS = {"rgb8": (3, "RGB2BGR"), "bgr8": (3, None), "rgba8": (4, "RGBA2BGR"),
            "bgra8": (4, "BGRA2BGR"), "mono8": (1, None)}


def jpeg_from_image(encoding: str, width: int, height: int, step: int, data: bytes,
                    quality: int = 80) -> Optional[bytes]:
    """None for an encoding this does not know (the caller says so once)."""
    import cv2                               # only where a camera is in use

    layout = _LAYOUTS.get(encoding.lower())
    if layout is None or width <= 0 or height <= 0:
        return None
    channels, conversion = layout
    raw = np.frombuffer(data, dtype=np.uint8)
    if raw.size < step * height:
        return None
    rows = raw[: step * height].reshape(height, step)[:, : width * channels]
    pixels = rows.reshape(height, width, channels) if channels > 1 else rows.reshape(height, width)
    if conversion is not None:
        pixels = cv2.cvtColor(pixels, getattr(cv2, "COLOR_" + conversion))
    ok, encoded = cv2.imencode(".jpg", pixels, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    return encoded.tobytes() if ok else None
