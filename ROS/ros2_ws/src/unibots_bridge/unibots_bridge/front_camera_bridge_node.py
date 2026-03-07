import urllib.request

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class FrontCameraBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('front_camera_bridge_node')

        self.declare_parameter('remote_host', '192.168.50.2')
        self.declare_parameter('remote_port', 5003)
        self.declare_parameter('camera_path', '/front_camera')
        self.declare_parameter('publish_topic', '/sim/front_camera')
        self.declare_parameter('poll_hz', 10.0)
        self.declare_parameter('request_timeout', 0.25)

        self.remote_host = self.get_parameter('remote_host').get_parameter_value().string_value
        self.remote_port = int(self.get_parameter('remote_port').get_parameter_value().integer_value)
        self.camera_path = self.get_parameter('camera_path').get_parameter_value().string_value
        self.publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        poll_hz = float(self.get_parameter('poll_hz').get_parameter_value().double_value)
        self.request_timeout = float(self.get_parameter('request_timeout').get_parameter_value().double_value)

        if self.request_timeout <= 0.0:
            self.request_timeout = 0.25

        self._bridge = CvBridge()
        self._last_payload: bytes | None = None
        self._error_logged = False

        self._publisher = self.create_publisher(Image, self.publish_topic, 10)

        period = 0.1 if poll_hz <= 0.0 else (1.0 / poll_hz)
        self.create_timer(period, self._poll_once)

        self.get_logger().info(
            f'front_camera_bridge_node started; upstream={self.remote_host}:{self.remote_port}{self.camera_path}, '
            f'topic={self.publish_topic}, poll_hz={poll_hz}'
        )

    def _poll_once(self) -> None:
        url = f'http://{self.remote_host}:{self.remote_port}{self.camera_path}'
        try:
            with urllib.request.urlopen(url, timeout=self.request_timeout) as res:
                body = res.read()
        except Exception as exc:
            if not self._error_logged:
                self.get_logger().warn(f'front camera fetch failed for {url}: {exc}')
                self._error_logged = True
            return

        self._error_logged = False
        if not body:
            return
        if self._last_payload is not None and body == self._last_payload:
            return

        image = cv2.imdecode(np.frombuffer(body, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().warn('front camera payload could not be decoded')
            return

        msg = self._bridge.cv2_to_imgmsg(image, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'front_camera'
        self._publisher.publish(msg)
        self._last_payload = body


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontCameraBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
