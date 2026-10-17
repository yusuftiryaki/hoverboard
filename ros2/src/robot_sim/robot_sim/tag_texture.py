"""The AprilTag the station carries, as an image file for Gazebo. Needs OpenCV.

tag36h11 #0 (cv::aruco::DICT_APRILTAG_36h11 — what track_marker_node decodes,
sensors.launch.py), the black square filling `marker_fraction` of the board,
white round it (the quiet zone).
"""

from __future__ import annotations


def write_tag_texture(path: str, board_px: int = 512, marker_fraction: float = 1 / 1.6,
                      tag_id: int = 0) -> str:
    import cv2
    import numpy as np

    marker_px = int(round(board_px * marker_fraction))
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(dictionary, tag_id, marker_px)
    else:                                          # OpenCV < 4.7
        marker = cv2.aruco.drawMarker(dictionary, tag_id, marker_px)
    board = np.full((board_px, board_px), 255, dtype=np.uint8)
    offset = (board_px - marker_px) // 2
    board[offset:offset + marker_px, offset:offset + marker_px] = marker
    if not cv2.imwrite(path, board):
        raise OSError(f"could not write {path}")
    return path
