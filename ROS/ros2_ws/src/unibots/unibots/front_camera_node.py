import urllib.request
from datetime import datetime
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class FrontCameraNode(Node):
    def __init__(self) -> None:
        super().__init__('front_camera_node')
        default_snapshot_dir = str(Path.home() / 'Desktop' / 'front_camera_snapshots')

        self.declare_parameter('use_real_sensor', False)
        self.declare_parameter('usb_camera_device', '')
        self.declare_parameter('usb_camera_index', 0)
        self.declare_parameter('camera_remote_host', '192.168.50.2')
        self.declare_parameter('camera_remote_port', 5003)
        self.declare_parameter('camera_path', '/data/front_camera')
        self.declare_parameter('camera_topic', '/front_camera')
        self.declare_parameter('camera_poll_hz', 10.0)
        self.declare_parameter('camera_request_timeout', 1.0)
        self.declare_parameter('snapshot_save_dir', default_snapshot_dir)
        self.declare_parameter('snapshot_interval_sec', 1.0)

        self.use_real_sensor = self.get_parameter('use_real_sensor').get_parameter_value().bool_value
        self.usb_camera_device = self.get_parameter('usb_camera_device').get_parameter_value().string_value.strip()
        self.usb_camera_index = int(self.get_parameter('usb_camera_index').get_parameter_value().integer_value)
        self.camera_remote_host = self.get_parameter('camera_remote_host').get_parameter_value().string_value
        self.camera_remote_port = int(self.get_parameter('camera_remote_port').get_parameter_value().integer_value)
        self.camera_path = self.get_parameter('camera_path').get_parameter_value().string_value
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        camera_poll_hz = float(self.get_parameter('camera_poll_hz').get_parameter_value().double_value)
        self.camera_request_timeout = float(
            self.get_parameter('camera_request_timeout').get_parameter_value().double_value
        )
        self.snapshot_save_dir = self.get_parameter('snapshot_save_dir').get_parameter_value().string_value.strip()
        self.snapshot_interval_sec = float(
            self.get_parameter('snapshot_interval_sec').get_parameter_value().double_value
        )

        if self.camera_request_timeout <= 0.0:
            self.camera_request_timeout = 1.0
        if not self.snapshot_save_dir:
            self.snapshot_save_dir = default_snapshot_dir
        if self.snapshot_interval_sec <= 0.0:
            self.snapshot_interval_sec = 1.0

        self.snapshot_save_dir = str(Path(self.snapshot_save_dir).expanduser().resolve())

        self._url = f'http://{self.camera_remote_host}:{self.camera_remote_port}{self.camera_path}'
        self._pub_front_camera = self.create_publisher(Image, self.camera_topic, 10)
        self._cv_bridge = CvBridge()
        self._camera_error_logged = False
        self._usb_camera_error_logged = False
        self._usb_capture = None
        self._usb_capture_source = ''
        self._snapshot_dir = Path(self.snapshot_save_dir)
        self._snapshot_last_save_ts = 0.0
        self._snapshot_error_logged = False

        try:
            self._snapshot_dir.mkdir(parents=True, exist_ok=True)
        except Exception as exc:
            self.get_logger().warn(f'failed to create snapshot_save_dir={self._snapshot_dir}: {exc}')

        if self.use_real_sensor:
            if self.usb_camera_device:
                self._usb_capture = cv2.VideoCapture(self.usb_camera_device)
                if self._usb_capture.isOpened():
                    self._usb_capture_source = self.usb_camera_device
                else:
                    self.get_logger().warn(
                        f'use_real_sensor=true but USB camera could not be opened at device={self.usb_camera_device}; '
                        f'falling back to index={self.usb_camera_index}'
                    )
                    self._usb_capture.release()
                    self._usb_capture = None

            if self._usb_capture is None:
                self._usb_capture = cv2.VideoCapture(self.usb_camera_index)
                if self._usb_capture.isOpened():
                    self._usb_capture_source = f'index={self.usb_camera_index}'

            if not self._usb_capture.isOpened():
                self.get_logger().warn(
                    f'use_real_sensor=true but USB camera could not be opened '
                    f'(device={self.usb_camera_device or "<empty>"}, index={self.usb_camera_index})'
                )
                self._usb_capture.release()
                self._usb_capture = None
                self._usb_capture_source = ''

        camera_period = 0.1 if camera_poll_hz <= 0.0 else (1.0 / camera_poll_hz)
        self.create_timer(camera_period, self._poll_camera_once)

        if self.use_real_sensor:
            self.get_logger().info(
                f'front_camera_node started; source=usb_camera({self._usb_capture_source or "unavailable"}), '
                f'topic={self.camera_topic}, poll_hz={camera_poll_hz}, '
                f'snapshot_dir={self._snapshot_dir}, snapshot_interval_sec={self.snapshot_interval_sec}'
            )
        else:
            self.get_logger().info(
                f'front_camera_node started; source=web, upstream={self._url}, topic={self.camera_topic}, poll_hz={camera_poll_hz}, '
                f'snapshot_dir={self._snapshot_dir}, snapshot_interval_sec={self.snapshot_interval_sec}'
            )

    def _save_snapshot_if_due(self, image: np.ndarray) -> None:
        now = time.time()
        if (now - self._snapshot_last_save_ts) < self.snapshot_interval_sec:
            return

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        image_path = self._snapshot_dir / f'front_camera_{timestamp}.jpg'
        ok = cv2.imwrite(str(image_path), image)
        if not ok:
            if not self._snapshot_error_logged:
                self.get_logger().warn(f'failed to save front camera snapshot to {image_path}')
                self._snapshot_error_logged = True
            return

        self._snapshot_last_save_ts = now
        self._snapshot_error_logged = False

    def _poll_camera_once(self) -> None:
        if self.use_real_sensor:
            if self._usb_capture is None:
                if not self._usb_camera_error_logged:
                    self.get_logger().warn('USB camera is unavailable; check connection and permissions')
                    self._usb_camera_error_logged = True
                return

            ok, image = self._usb_capture.read()
            if not ok or image is None:
                if not self._usb_camera_error_logged:
                    self.get_logger().warn('failed to read frame from USB camera')
                    self._usb_camera_error_logged = True
                return

            # image = cv2.flip(image, 0)

            self._usb_camera_error_logged = False
            msg = self._cv_bridge.cv2_to_imgmsg(image, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'front_camera'
            self._pub_front_camera.publish(msg)
            # Snapshot auto-save disabled (no longer needed).
            # self._save_snapshot_if_due(image)
            return

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
        # Snapshot auto-save disabled (no longer needed).
        # self._save_snapshot_if_due(image)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontCameraNode()
    try:
        rclpy.spin(node)
    finally:
        if node._usb_capture is not None:
            node._usb_capture.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
