import urllib.request

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class FrontCameraNode(Node):
    def __init__(self) -> None:
        super().__init__('front_camera_node')

        self.declare_parameter('camera_remote_host', '192.168.50.1')
        self.declare_parameter('camera_remote_port', 5003)
        self.declare_parameter('camera_path', '/data/front_camera')
        self.declare_parameter('camera_topic', '/front_camera')
        self.declare_parameter('camera_poll_hz', 10.0)
        self.declare_parameter('camera_request_timeout', 1.0)

        self.camera_remote_host = self.get_parameter('camera_remote_host').get_parameter_value().string_value
        self.camera_remote_port = int(self.get_parameter('camera_remote_port').get_parameter_value().integer_value)
        self.camera_path = self.get_parameter('camera_path').get_parameter_value().string_value
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        camera_poll_hz = float(self.get_parameter('camera_poll_hz').get_parameter_value().double_value)
        self.camera_request_timeout = float(
            self.get_parameter('camera_request_timeout').get_parameter_value().double_value
        )

        if self.camera_request_timeout <= 0.0:
            self.camera_request_timeout = 1.0

        self._url = f'http://{self.camera_remote_host}:{self.camera_remote_port}{self.camera_path}'
        self._pub_front_camera = self.create_publisher(Image, self.camera_topic, 10)
        self._cv_bridge = CvBridge()
        self._camera_error_logged = False

        camera_period = 0.1 if camera_poll_hz <= 0.0 else (1.0 / camera_poll_hz)
        self.create_timer(camera_period, self._poll_camera_once)

        self.get_logger().info(
            f'front_camera_node started; upstream={self._url}, topic={self.camera_topic}, poll_hz={camera_poll_hz}'
        )

    def _poll_camera_once(self) -> None:
        try:
            with urllib.request.urlopen(self._url, timeout=self.camera_request_timeout) as res:
                content_type = (res.headers.get_content_type() or '').lower()
                body = res.read()
        except Exception as exc:
            if not self._camera_error_logged:
                self.get_logger().warn(f'front camera fetch failed for {self._url}: {exc}')
                self._camera_error_logged = True
            return

        self._camera_error_logged = False
        if not body:
            return
        if content_type and not content_type.startswith('image/'):
            self.get_logger().warn(
                f'front camera upstream returned non-image content-type: {content_type}; '
                f'check camera_path={self.camera_path}'
            )
            return

        image = cv2.imdecode(np.frombuffer(body, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().warn('front camera payload could not be decoded')
            return

        msg = self._cv_bridge.cv2_to_imgmsg(image, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'front_camera'
        self._pub_front_camera.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
