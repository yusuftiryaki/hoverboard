"""image_compressor — camera/image_raw -> camera/jpeg for the episode recorder.

    camera/image_raw (sensor_msgs/Image)  ──►  camera/jpeg (sensor_msgs/CompressedImage)

Raw frames are ~27 MB/s at 640x480 30 Hz; an episode records the JPEG
(docs/learning.md). It encodes ONLY while someone subscribes to camera/jpeg —
in practice while an episode is recording — and at most `fps` frames a second.
The header (stamp, frame_id) is the raw frame's: the stamp is what lines the
picture up with the action label later, so it must be when the light hit the
sensor, not when this node got round to it.

Same node on the robot (camera_ros) and in the simulator (sim_camera, Gazebo).
Uses robot_missions.camera's encoder, the one the web page's live view uses.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

from robot_missions.camera import jpeg_from_image


class ImageCompressor(Node):
    def __init__(self, **kwargs):
        super().__init__("image_compressor", **kwargs)
        self.declare_parameter("fps", 15.0)
        self.declare_parameter("quality", 80)
        self._pub = self.create_publisher(CompressedImage, "camera/jpeg", qos_profile_sensor_data)
        self.create_subscription(Image, "camera/image_raw", self._on_image,
                                 qos_profile_sensor_data)
        self._last = 0.0
        self._warned = False

    def _on_image(self, msg: Image) -> None:
        if self._pub.get_subscription_count() == 0:
            return                                   # nobody recording: no CPU spent
        now = time.monotonic()
        if now - self._last < 1.0 / max(0.5, self.get_parameter("fps").value):
            return
        self._last = now
        data = jpeg_from_image(msg.encoding, msg.width, msg.height, msg.step, bytes(msg.data),
                               quality=int(self.get_parameter("quality").value))
        if data is None:
            if not self._warned:
                self.get_logger().warn(f"cannot encode {msg.encoding!r} images")
                self._warned = True
            return
        out = CompressedImage()
        out.header = msg.header
        out.format = "jpeg"
        out.data = data
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ImageCompressor()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
