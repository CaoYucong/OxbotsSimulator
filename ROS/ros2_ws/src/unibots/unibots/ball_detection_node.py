import json
import math
import socket
import time
import base64
import os
import urllib.error
import urllib.parse
import urllib.request
from typing import Optional

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class BallDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__('ball_detection_node')

        self.declare_parameter('front_camera_topic', '/front_camera')
        self.declare_parameter('ball_detection_enabled', True)
        self.declare_parameter('ball_detection_image_topic', '/ball_detection_image')
        self.declare_parameter('ball_pose_topic', '/ball_pose')
        self.declare_parameter('ball_detections_topic', '/ball_detections')
        self.declare_parameter('camera_pose_topic', '/camera_pose')
        self.declare_parameter('visible_balls_topic', '/visible_balls')
        self.declare_parameter('local_inference_url', 'http://127.0.0.1:9001')
        self.declare_parameter('local_model_id', 'unibot-ball-detection/1')
        self.declare_parameter('roboflow_api_key', '')
        self.declare_parameter('request_timeout_sec', 10.0)
        self.declare_parameter('infer_hz', 1.0)
        self.declare_parameter('min_confidence', 0.25)
        self.declare_parameter('debug_logs', False)
        self.declare_parameter('debug_log_every_sec', 2.0)
        self.declare_parameter('perf_logs', True)
        self.declare_parameter('perf_log_every_sec', 2.0)
        self.declare_parameter('camera_intrinsic_path', '')
        self.declare_parameter('ball_diameter_m', 0.043)
        self.declare_parameter('ping_ball_diameter_m', 0.040)
        self.declare_parameter('metal_ball_diameter_m', 0.020)
        self.declare_parameter('camera_height_m', 0.200)
        self.declare_parameter('ping_ball_center_height_m', 0.020)
        self.declare_parameter('metal_ball_center_height_m', 0.100)
        self.declare_parameter('camera_offset_x_robot', 0.105)
        self.declare_parameter('camera_offset_y_robot', 0.0)

        front_camera_topic = self.get_parameter('front_camera_topic').get_parameter_value().string_value
        self._ball_detection_enabled = bool(
            self.get_parameter('ball_detection_enabled').get_parameter_value().bool_value
        )
        ball_detection_image_topic = (
            self.get_parameter('ball_detection_image_topic').get_parameter_value().string_value
        )
        self._ball_pose_topic = self.get_parameter('ball_pose_topic').get_parameter_value().string_value
        self._ball_detections_topic = (
            self.get_parameter('ball_detections_topic').get_parameter_value().string_value
        )
        self._camera_pose_topic = self.get_parameter('camera_pose_topic').get_parameter_value().string_value
        self._visible_balls_topic = self.get_parameter('visible_balls_topic').get_parameter_value().string_value
        self._local_inference_url = (
            self.get_parameter('local_inference_url').get_parameter_value().string_value.rstrip('/')
        )
        self._local_model_id = self.get_parameter('local_model_id').get_parameter_value().string_value
        configured_api_key = (
            self.get_parameter('roboflow_api_key').get_parameter_value().string_value.strip()
        )
        self._roboflow_api_key = configured_api_key or os.getenv('ROBOFLOW_API_KEY', '').strip()
        self._request_timeout_sec = float(
            self.get_parameter('request_timeout_sec').get_parameter_value().double_value
        )
        self._infer_hz = float(self.get_parameter('infer_hz').get_parameter_value().double_value)
        self._min_confidence = float(self.get_parameter('min_confidence').get_parameter_value().double_value)
        self._debug_logs = bool(self.get_parameter('debug_logs').get_parameter_value().bool_value)
        self._debug_log_every_sec = float(
            self.get_parameter('debug_log_every_sec').get_parameter_value().double_value
        )
        self._perf_logs = bool(self.get_parameter('perf_logs').get_parameter_value().bool_value)
        self._perf_log_every_sec = float(
            self.get_parameter('perf_log_every_sec').get_parameter_value().double_value
        )
        configured_intrinsic_path = (
            self.get_parameter('camera_intrinsic_path').get_parameter_value().string_value.strip()
        )
        self._camera_intrinsic_path = configured_intrinsic_path or self._find_default_intrinsic_path()
        self._ball_diameter_m = float(self.get_parameter('ball_diameter_m').get_parameter_value().double_value)
        self._ping_ball_diameter_m = float(
            self.get_parameter('ping_ball_diameter_m').get_parameter_value().double_value
        )
        self._metal_ball_diameter_m = float(
            self.get_parameter('metal_ball_diameter_m').get_parameter_value().double_value
        )
        self._camera_height_m = float(self.get_parameter('camera_height_m').get_parameter_value().double_value)
        self._ping_ball_center_height_m = float(
            self.get_parameter('ping_ball_center_height_m').get_parameter_value().double_value
        )
        self._metal_ball_center_height_m = float(
            self.get_parameter('metal_ball_center_height_m').get_parameter_value().double_value
        )
        self._camera_offset_x_robot = float(
            self.get_parameter('camera_offset_x_robot').get_parameter_value().double_value
        )
        self._camera_offset_y_robot = float(
            self.get_parameter('camera_offset_y_robot').get_parameter_value().double_value
        )
        self._crop_y_start = 150

        if self._request_timeout_sec <= 0.0:
            self._request_timeout_sec = 1.0
        if self._infer_hz <= 0.0:
            self._infer_hz = 0.1
        if self._debug_log_every_sec <= 0.0:
            self._debug_log_every_sec = 2.0
        if self._perf_log_every_sec <= 0.0:
            self._perf_log_every_sec = 2.0
        if self._ball_diameter_m <= 0.0:
            self._ball_diameter_m = 0.043
        if self._ping_ball_diameter_m <= 0.0:
            self._ping_ball_diameter_m = 0.040
        if self._metal_ball_diameter_m <= 0.0:
            self._metal_ball_diameter_m = 0.020
        if self._camera_height_m <= 0.0:
            self._camera_height_m = 0.200
        if self._ping_ball_center_height_m < 0.0:
            self._ping_ball_center_height_m = 0.020
        if self._metal_ball_center_height_m < 0.0:
            self._metal_ball_center_height_m = 0.100

        self._bridge = CvBridge()
        self._latest_front_image: Optional[np.ndarray] = None
        self._latest_front_stamp = None
        self._latest_detections: list[dict] = []
        self._last_infer_error_warn = 0.0
        self._last_debug_log: dict[str, float] = {}
        self._last_perf_log: dict[str, float] = {}
        self._last_error_text = 'waiting for front camera image'
        self._fallback_image = self._build_fallback_image(self._last_error_text)
        self._last_success_detection_image: Optional[np.ndarray] = None
        self._camera_intrinsics = self._load_camera_intrinsics(self._camera_intrinsic_path)
        self._current_camera_pose: Optional[tuple[np.ndarray, np.ndarray]] = None

        self._pub_ball_detection_image = self.create_publisher(Image, ball_detection_image_topic, 10)
        self._pub_ball_pose = self.create_publisher(PoseStamped, self._ball_pose_topic, 10)
        self._pub_ball_detections = self.create_publisher(String, self._ball_detections_topic, 10)
        self._pub_visible_balls = self.create_publisher(String, self._visible_balls_topic, 10)

        self.create_subscription(Image, front_camera_topic, self._on_front_image, 10)
        self.create_subscription(PoseStamped, self._camera_pose_topic, self._on_camera_pose, 10)

        self.create_timer(1.0 / self._infer_hz, self._infer_tick)

        self.get_logger().info(
            'ball_detection_node started; '
            f'front_camera_topic={front_camera_topic}, '
            f'ball_detection_image_topic={ball_detection_image_topic}, '
            f'ball_detection_enabled={self._ball_detection_enabled}, '
            f'mode=local, model_id={self._local_model_id}, '
            f'infer_hz={self._infer_hz:.2f}, min_confidence={self._min_confidence:.2f}, '
            f'crop_y_start={self._crop_y_start}, '
            f'camera_pose_topic={self._camera_pose_topic}, '
            f'visible_balls_topic={self._visible_balls_topic}'
        )
        self._debug(
            'debug config: '
            f'request_timeout_sec={self._request_timeout_sec:.2f}, '
            f'api_key_set={bool(self._roboflow_api_key)}, '
            f'debug_log_every_sec={self._debug_log_every_sec:.2f}'
        )
        self.get_logger().info(
            'perf config: '
            f'perf_logs={self._perf_logs}, '
            f'perf_log_every_sec={self._perf_log_every_sec:.2f}'
        )
        self._log_camera_intrinsic_status()
        self._log_inference_startup_check()

    def _debug(self, message: str) -> None:
        if self._debug_logs:
            self.get_logger().info(f'[debug] {message}')

    def _debug_throttled(self, key: str, message: str) -> None:
        if not self._debug_logs:
            return
        now = time.monotonic()
        last = self._last_debug_log.get(key, 0.0)
        if now - last >= self._debug_log_every_sec:
            self._last_debug_log[key] = now
            self.get_logger().info(f'[debug] {message}')

    def _perf_throttled(self, key: str, message: str) -> None:
        if not self._perf_logs:
            return
        now = time.monotonic()
        last = self._last_perf_log.get(key, 0.0)
        if now - last >= self._perf_log_every_sec:
            self._last_perf_log[key] = now
            self.get_logger().info(f'[perf] {message}')

    def _on_front_image(self, msg: Image) -> None:
        try:
            full_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._latest_front_stamp = msg.header.stamp
            full_h, full_w = full_image.shape[:2]
            if full_h > self._crop_y_start:
                self._latest_front_image = full_image[self._crop_y_start :, :].copy()
            else:
                self._latest_front_image = full_image

            cropped_h, cropped_w = self._latest_front_image.shape[:2]
            self._debug_throttled(
                'front_image',
                f'received front image: {full_w}x{full_h}, '
                f'cropped to: {cropped_w}x{cropped_h} (keep y>={self._crop_y_start})',
            )
        except Exception as exc:
            now = time.monotonic()
            if now - self._last_infer_error_warn >= 1.0:
                self._last_infer_error_warn = now
                self.get_logger().warn(f'failed to decode front camera image: {exc}')

    @staticmethod
    def _quaternion_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
        n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if n <= 1e-9:
            return np.eye(3, dtype=np.float64)
        x = qx / n
        y = qy / n
        z = qz / n
        w = qw / n
        return np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )

    def _on_camera_pose(self, msg: PoseStamped) -> None:
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        z = float(msg.pose.position.z)
        qx = float(msg.pose.orientation.x)
        qy = float(msg.pose.orientation.y)
        qz = float(msg.pose.orientation.z)
        qw = float(msg.pose.orientation.w)
        cam_world = np.array([x, y, z], dtype=np.float64)
        r_wc = self._quaternion_to_rotation_matrix(qx, qy, qz, qw)
        self._current_camera_pose = (cam_world, r_wc)

    @staticmethod
    def _to_ball_type(detection_class: str) -> str:
        text = (detection_class or '').strip().lower()
        if any(keyword in text for keyword in ('metal', 'steel', 'iron')):
            return 'METAL'
        if any(keyword in text for keyword in ('ping', 'pong', 'table_tennis', 'table-tennis')):
            return 'PING'
        return 'PING'

    def _publish_visible_balls_topic(self, detections_payload: list[dict]) -> None:
        lines: list[str] = []
        for detection in detections_payload:
            world_position = detection.get('world_position')
            if not isinstance(world_position, dict):
                continue
            try:
                x = float(world_position.get('x'))
                y = float(world_position.get('y'))
            except Exception:
                continue
            ball_type = self._to_ball_type(str(detection.get('class', 'PING')))
            lines.append(f'({x:.6f}, {y:.6f}, {ball_type})')

        msg = String()
        if lines:
            msg.data = '\n'.join(lines) + '\n'
        else:
            msg.data = ''
        self._pub_visible_balls.publish(msg)

    def _find_default_intrinsic_path(self) -> str:
        local_candidate = os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..', 'config', 'real_camera_intrinsic.yaml')
        )
        if os.path.isfile(local_candidate):
            return local_candidate
        return ''

    def _load_camera_intrinsics(self, intrinsic_path: str) -> Optional[dict]:
        if not intrinsic_path:
            return None
        if not os.path.isfile(intrinsic_path):
            self.get_logger().warn(f'camera intrinsic file not found: {intrinsic_path}')
            return None

        try:
            with open(intrinsic_path, 'r', encoding='utf-8') as file:
                payload = yaml.safe_load(file) or {}
        except Exception as exc:
            self.get_logger().warn(f'failed to load camera intrinsics: {exc}')
            return None

        try:
            matrix_data = payload.get('camera_matrix', {}).get('data', [])
            distortion_data = payload.get('distortion_coefficients', {}).get('data', [])
            image_width = int(payload.get('image_width', 0))
            image_height = int(payload.get('image_height', 0))
            distortion_model = str(payload.get('distortion_model', '')).strip().lower()

            camera_matrix = np.array(matrix_data, dtype=np.float64).reshape(3, 3)
            distortion = np.array(distortion_data, dtype=np.float64).reshape(-1, 1)
        except Exception as exc:
            self.get_logger().warn(f'invalid camera intrinsic format: {exc}')
            return None

        return {
            'path': intrinsic_path,
            'camera_matrix': camera_matrix,
            'distortion': distortion,
            'distortion_model': distortion_model,
            'image_width': image_width,
            'image_height': image_height,
            'fx': float(camera_matrix[0, 0]),
            'fy': float(camera_matrix[1, 1]),
            'cx': float(camera_matrix[0, 2]),
            'cy': float(camera_matrix[1, 2]),
        }

    def _log_camera_intrinsic_status(self) -> None:
        if self._camera_intrinsics is None:
            self.get_logger().warn(
                'camera intrinsics unavailable; skip 3D ball position estimation '
                f'(camera_intrinsic_path={self._camera_intrinsic_path or "(empty)"})'
            )
            return
        self.get_logger().info(
            'camera intrinsics loaded: '
            f"path={self._camera_intrinsics['path']}, "
            f"model={self._camera_intrinsics['distortion_model'] or 'none'}, "
            f"fx={self._camera_intrinsics['fx']:.2f}, fy={self._camera_intrinsics['fy']:.2f}, "
            f"cx={self._camera_intrinsics['cx']:.2f}, cy={self._camera_intrinsics['cy']:.2f}, "
            f'camera_height_m={self._camera_height_m:.3f}, '
            f'ping_ball_center_height_m={self._ping_ball_center_height_m:.3f}, '
            f'metal_ball_center_height_m={self._metal_ball_center_height_m:.3f}, '
            f'ping_ball_diameter_m={self._ping_ball_diameter_m:.4f}, '
            f'metal_ball_diameter_m={self._metal_ball_diameter_m:.4f}'
        )

    def _ball_diameter_for_class(self, detection_class: str) -> float:
        ball_type = self._to_ball_type(detection_class)
        if ball_type == 'METAL':
            return self._metal_ball_diameter_m
        if ball_type == 'PING':
            return self._ping_ball_diameter_m
        return self._ball_diameter_m

    def _ball_center_height_for_class(self, detection_class: str) -> float:
        ball_type = self._to_ball_type(detection_class)
        if ball_type == 'METAL':
            return self._metal_ball_center_height_m
        if ball_type == 'PING':
            return self._ping_ball_center_height_m
        return self._ping_ball_center_height_m

    def _estimate_relative_position(self, detection: dict) -> Optional[dict]:
        if self._camera_intrinsics is None or self._current_camera_pose is None:
            return None

        try:
            u = float(detection['x'])
            v_cropped = float(detection['y'])
            width_px = float(detection['width'])
            height_px = float(detection['height'])
        except Exception:
            return None

        fx = float(self._camera_intrinsics['fx'])
        fy = float(self._camera_intrinsics['fy'])
        v = v_cropped + float(self._crop_y_start)

        camera_matrix = self._camera_intrinsics['camera_matrix']
        distortion = self._camera_intrinsics['distortion']
        distortion_model = str(self._camera_intrinsics.get('distortion_model', '')).lower()

        if distortion_model == 'fisheye' and distortion.size >= 4:
            points = np.array([[[u, v]]], dtype=np.float64)
            undistorted = cv2.fisheye.undistortPoints(points, camera_matrix, distortion)
            x_norm = float(undistorted[0, 0, 0])
            y_norm = float(undistorted[0, 0, 1])
        else:
            cx = float(self._camera_intrinsics['cx'])
            cy = float(self._camera_intrinsics['cy'])
            x_norm = (u - cx) / max(fx, 1e-6)
            y_norm = (v - cy) / max(fy, 1e-6)

        direction_cam = np.array([x_norm, y_norm, 1.0], dtype=np.float64)
        norm = np.linalg.norm(direction_cam)
        if norm <= 1e-9:
            return None
        direction_cam = direction_cam / norm

        cam_world, r_wc = self._current_camera_pose
        direction_world = r_wc @ direction_cam

        ball_class = str(detection.get('class', 'PING'))
        plane_z = self._ball_center_height_for_class(ball_class)
        dz = float(direction_world[2])
        if abs(dz) <= 1e-6:
            return None

        t = (plane_z - float(cam_world[2])) / dz
        if t <= 0.0:
            return None

        point_world = cam_world + t * direction_world
        point_cam = r_wc.T @ (point_world - cam_world)
        x = float(point_cam[0])
        y = float(point_cam[1])
        z = float(point_cam[2])
        range_m = float(np.linalg.norm(point_cam))

        return {
            'x': x,
            'y': y,
            'z': z,
            'range': range_m,
            'world_x': float(point_world[0]),
            'world_y': float(point_world[1]),
            'world_z': float(point_world[2]),
        }

    def _log_inference_startup_check(self) -> None:
        endpoint = f'{self._local_inference_url}/{self._local_model_id}'
        endpoint_with_key = (
            self._append_api_key(endpoint, self._roboflow_api_key) if self._roboflow_api_key else endpoint
        )
        endpoint_for_log = self._redact_api_key(endpoint_with_key)

        parsed = urllib.parse.urlparse(self._local_inference_url)
        host = parsed.hostname or ''
        port = parsed.port or (443 if parsed.scheme == 'https' else 80)
        is_local_host = host in ('127.0.0.1', 'localhost', '::1')

        self.get_logger().info(
            'inference target: '
            f'base={self._local_inference_url}, '
            f'model_id={self._local_model_id}, '
            f'endpoint={endpoint_for_log}, '
            f'host={host or "(unknown)"}, '
            f'port={port}, '
            f'is_local_host={is_local_host}, '
            f'api_key_set={bool(self._roboflow_api_key)}'
        )

        probe_timeout = min(max(0.2, self._request_timeout_sec), 2.0)
        tcp_start = time.perf_counter()
        try:
            with socket.create_connection((host, port), timeout=probe_timeout):
                pass
            tcp_ms = (time.perf_counter() - tcp_start) * 1000.0
            self.get_logger().info(
                f'inference startup check: tcp_connect_ok {host}:{port} in {tcp_ms:.1f}ms'
            )
        except Exception as exc:
            tcp_ms = (time.perf_counter() - tcp_start) * 1000.0
            self.get_logger().warn(
                f'inference startup check: tcp_connect_failed {host}:{port} in {tcp_ms:.1f}ms ({type(exc).__name__})'
            )
            return

        http_start = time.perf_counter()
        request = urllib.request.Request(self._local_inference_url, method='GET')
        try:
            with urllib.request.urlopen(request, timeout=probe_timeout) as response:
                http_ms = (time.perf_counter() - http_start) * 1000.0
                self.get_logger().info(
                    'inference startup check: '
                    f'http_ok status={getattr(response, "status", "unknown")} '
                    f'in {http_ms:.1f}ms'
                )
        except urllib.error.HTTPError as exc:
            http_ms = (time.perf_counter() - http_start) * 1000.0
            self.get_logger().info(
                f'inference startup check: http_responded status={exc.code} in {http_ms:.1f}ms'
            )
        except Exception as exc:
            http_ms = (time.perf_counter() - http_start) * 1000.0
            self.get_logger().warn(
                f'inference startup check: http_failed in {http_ms:.1f}ms ({type(exc).__name__})'
            )

    def _infer_tick(self) -> None:
        tick_start = time.perf_counter()
        if not self._ball_detection_enabled:
            self._latest_detections = []
            self._publish_sticky_detection_image_or_fallback('ball detection disabled')
            if self._latest_front_image is not None:
                image_h, image_w = self._latest_front_image.shape[:2]
            else:
                image_w, image_h = 0, 0
            self._publish_detection_topics([], image_w, image_h)
            self._debug_throttled('ball_detection_disabled', 'infer tick skipped: ball_detection_enabled=false')
            return

        if self._latest_front_image is None:
            self._debug_throttled('no_front_image', 'infer tick skipped: no front camera image yet')
            return

        image = self._latest_front_image
        detections, infer_timing = self._infer_with_roboflow(image)
        self._latest_detections = detections
        publish_topics_ms = 0.0
        publish_image_ms = 0.0
        t_publish_image = time.perf_counter()
        self._publish_detection_image(image, detections)
        publish_image_ms = (time.perf_counter() - t_publish_image) * 1000.0
        if not detections:
            self._debug_throttled('no_detection_publish', 'skip publish: no valid detections in this tick')
            tick_total_ms = (time.perf_counter() - tick_start) * 1000.0
            # self._perf_throttled(
            #     'infer_tick',
            #     'stage_ms '
            #     f"encode={infer_timing.get('encode_ms', 0.0):.1f}, "
            #     f"request={infer_timing.get('request_ms', 0.0):.1f}, "
            #     f"extract={infer_timing.get('extract_ms', 0.0):.1f}, "
            #     f"filter={infer_timing.get('filter_ms', 0.0):.1f}, "
            #     f"infer_total={infer_timing.get('total_ms', 0.0):.1f}, "
            #     f'publish_topics={publish_topics_ms:.1f}, '
            #     f'publish_image={publish_image_ms:.1f}, '
            #     f'tick_total={tick_total_ms:.1f}, '
            #     f'detections={len(detections)}',
            # )
            return

        t_publish_topics = time.perf_counter()
        self._publish_detection_topics(detections, image.shape[1], image.shape[0])
        publish_topics_ms = (time.perf_counter() - t_publish_topics) * 1000.0

        tick_total_ms = (time.perf_counter() - tick_start) * 1000.0
        # self._perf_throttled(
        #     'infer_tick',
        #     'stage_ms '
        #     f"encode={infer_timing.get('encode_ms', 0.0):.1f}, "
        #     f"request={infer_timing.get('request_ms', 0.0):.1f}, "
        #     f"extract={infer_timing.get('extract_ms', 0.0):.1f}, "
        #     f"filter={infer_timing.get('filter_ms', 0.0):.1f}, "
        #     f"infer_total={infer_timing.get('total_ms', 0.0):.1f}, "
        #     f'publish_topics={publish_topics_ms:.1f}, '
        #     f'publish_image={publish_image_ms:.1f}, '
        #     f'tick_total={tick_total_ms:.1f}, '
        #     f'detections={len(detections)}',
        # )

    def _infer_with_roboflow(self, image: np.ndarray) -> tuple[list[dict], dict[str, float]]:
        timing = {
            'encode_ms': 0.0,
            'request_ms': 0.0,
            'extract_ms': 0.0,
            'filter_ms': 0.0,
            'total_ms': 0.0,
        }
        infer_start = time.perf_counter()

        t_encode = time.perf_counter()
        ok, encoded = cv2.imencode('.jpg', image)
        timing['encode_ms'] = (time.perf_counter() - t_encode) * 1000.0
        if not ok:
            self._debug_throttled('encode_failed', 'cv2.imencode failed for inference frame')
            timing['total_ms'] = (time.perf_counter() - infer_start) * 1000.0
            return [], timing

        endpoint = f'{self._local_inference_url}/{self._local_model_id}'
        endpoint_with_key = (
            self._append_api_key(endpoint, self._roboflow_api_key) if self._roboflow_api_key else endpoint
        )
        endpoint_for_log = self._redact_api_key(endpoint_with_key)
        image_bytes = encoded.tobytes()
        self._debug(
            f'infer request: endpoint={endpoint_for_log}, '
            f'api_key_set={bool(self._roboflow_api_key)}, frame={image.shape[1]}x{image.shape[0]}, '
            f'timeout={self._request_timeout_sec:.2f}s'
        )

        t_request = time.perf_counter()
        try:
            payload = self._send_infer_request_with_fallback(endpoint_with_key, image_bytes)
        except urllib.error.HTTPError as exc:
            timing['request_ms'] = (time.perf_counter() - t_request) * 1000.0
            if exc.code in (401, 404) and self._roboflow_api_key:
                self._debug(f'retry with Bearer auth after http {exc.code}')
                try:
                    t_retry_request = time.perf_counter()
                    payload = self._send_infer_request_with_fallback(
                        endpoint,
                        image_bytes,
                        {'Authorization': f'Bearer {self._roboflow_api_key}'},
                    )
                    timing['request_ms'] += (time.perf_counter() - t_retry_request) * 1000.0
                except urllib.error.HTTPError as retry_exc:
                    timing['request_ms'] += (time.perf_counter() - t_retry_request) * 1000.0
                    if retry_exc.code == 401:
                        self._last_error_text = 'roboflow 401 (invalid api key)'
                        self.get_logger().warn(f'roboflow http error: {retry_exc.code} {retry_exc.reason}')
                        timing['total_ms'] = (time.perf_counter() - infer_start) * 1000.0
                        return [], timing
                    else:
                        self.get_logger().warn(f'roboflow http error: {retry_exc.code} {retry_exc.reason}')
                        self._last_error_text = f'roboflow http error: {retry_exc.code}'
                        timing['total_ms'] = (time.perf_counter() - infer_start) * 1000.0
                        return [], timing
            else:
                self.get_logger().warn(f'roboflow http error: {exc.code} {exc.reason}')
                if exc.code == 401:
                    self._last_error_text = 'roboflow 401 (set roboflow_api_key or ROBOFLOW_API_KEY)'
                else:
                    self._last_error_text = f'roboflow http error: {exc.code}'
                timing['total_ms'] = (time.perf_counter() - infer_start) * 1000.0
                return [], timing
        except Exception as exc:
            timing['request_ms'] = (time.perf_counter() - t_request) * 1000.0
            now = time.monotonic()
            if now - self._last_infer_error_warn >= 1.0:
                self._last_infer_error_warn = now
                self.get_logger().warn(f'roboflow infer failed: {exc}')
            self._last_error_text = f'roboflow infer failed: {type(exc).__name__}'
            timing['total_ms'] = (time.perf_counter() - infer_start) * 1000.0
            return [], timing
        else:
            timing['request_ms'] = (time.perf_counter() - t_request) * 1000.0

        t_extract = time.perf_counter()
        predictions = self._extract_predictions(payload)
        timing['extract_ms'] = (time.perf_counter() - t_extract) * 1000.0
        if not isinstance(predictions, list):
            self._debug('unexpected payload format: predictions is not a list')
            timing['total_ms'] = (time.perf_counter() - infer_start) * 1000.0
            return [], timing

        self._debug(f'infer response: raw_predictions={len(predictions)}')

        t_filter = time.perf_counter()
        result: list[dict] = []
        for item in predictions:
            try:
                conf = float(item.get('confidence', 0.0))
                if conf < self._min_confidence:
                    continue
                x = float(item['x'])
                y = float(item['y'])
                width = float(item['width'])
                height = float(item['height'])
                cls = str(item.get('class', 'ball'))
            except Exception:
                continue

            result.append(
                {
                    'class': cls,
                    'confidence': conf,
                    'x': x,
                    'y': y,
                    'width': width,
                    'height': height,
                }
            )
            timing['filter_ms'] = (time.perf_counter() - t_filter) * 1000.0

        result.sort(key=lambda d: float(d.get('confidence', 0.0)), reverse=True)
        if result:
            self._last_error_text = ''
            best = result[0]
            self._debug(
                f'filtered detections={len(result)}, best='
                f"{best.get('class', 'ball')}@{float(best.get('confidence', 0.0)):.3f}"
            )
        else:
            self._last_error_text = 'no ball detected'
            self._debug('filtered detections=0 (after min_confidence)')
        timing['total_ms'] = (time.perf_counter() - infer_start) * 1000.0
        return result, timing

    def _send_infer_request_with_fallback(
        self,
        endpoint: str,
        image_bytes: bytes,
        extra_headers: Optional[dict] = None,
    ) -> dict:
        json_body, json_content_type = self._encode_json_base64_image(image_bytes)
        try:
            return self._send_infer_request(
                endpoint,
                json_body,
                json_content_type,
                extra_headers,
            )
        except urllib.error.HTTPError as exc:
            error_body = self._read_http_error_body(exc)
            if exc.code == 400 and (
                'Malformed base64 input image' in error_body or 'Invalid base64 input' in error_body
            ):
                self._debug('json payload rejected; fallback to text/plain base64')
                plain_body, plain_content_type = self._encode_plain_base64_image(image_bytes)
                return self._send_infer_request(
                    endpoint,
                    plain_body,
                    plain_content_type,
                    extra_headers,
                )
            raise

    def _send_infer_request(
        self,
        endpoint: str,
        body: bytes,
        content_type: str,
        extra_headers: Optional[dict] = None,
    ) -> dict:
        headers = {'Content-Type': content_type}
        if extra_headers:
            headers.update(extra_headers)
        request = urllib.request.Request(
            endpoint,
            data=body,
            headers=headers,
            method='POST',
        )
        with urllib.request.urlopen(request, timeout=self._request_timeout_sec) as response:
            return json.loads(response.read().decode('utf-8', errors='ignore'))

    @staticmethod
    def _append_api_key(endpoint: str, api_key: str) -> str:
        separator = '&' if '?' in endpoint else '?'
        return f'{endpoint}{separator}api_key={urllib.parse.quote(api_key, safe="")}'

    @staticmethod
    def _redact_api_key(url: str) -> str:
        if 'api_key=' not in url:
            return url
        key_start = url.find('api_key=') + len('api_key=')
        key_end = url.find('&', key_start)
        if key_end == -1:
            return f'{url[:key_start]}***'
        return f'{url[:key_start]}***{url[key_end:]}'

    @staticmethod
    def _draw_detections(image: np.ndarray, detections: list[dict]) -> np.ndarray:
        output = image.copy()
        h, w = output.shape[:2]
        for det in detections:
            cx = float(det['x'])
            cy = float(det['y'])
            bw = float(det['width'])
            bh = float(det['height'])
            conf = float(det['confidence'])
            cls = str(det.get('class', 'ball'))

            x1 = int(max(0.0, cx - bw * 0.5))
            y1 = int(max(0.0, cy - bh * 0.5))
            x2 = int(min(float(w - 1), cx + bw * 0.5))
            y2 = int(min(float(h - 1), cy + bh * 0.5))

            cv2.rectangle(output, (x1, y1), (x2, y2), (0, 140, 255), 2)
            text = f'{cls} {conf:.2f}'
            cv2.putText(
                output,
                text,
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 140, 255),
                2,
                cv2.LINE_AA,
            )

        return output

    @staticmethod
    def _build_fallback_image(error_text: str) -> np.ndarray:
        image = np.zeros((120, 160, 3), dtype=np.uint8)
        image[:] = (30, 30, 30)
        cv2.putText(
            image,
            'ball detection',
            (6, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        text = (error_text or 'unknown error')[:42]
        cv2.putText(
            image,
            text,
            (6, 62),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            (0, 0, 255),
            1,
            cv2.LINE_AA,
        )
        return image

    def _publish_fallback_image(self, error_text: str) -> None:
        self._fallback_image = self._build_fallback_image(error_text)
        msg = self._bridge.cv2_to_imgmsg(self._fallback_image, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'front_camera'
        self._pub_ball_detection_image.publish(msg)

    def _publish_cached_success_image(self) -> bool:
        if self._last_success_detection_image is None:
            return False
        msg = self._bridge.cv2_to_imgmsg(self._last_success_detection_image, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'front_camera'
        self._pub_ball_detection_image.publish(msg)
        return True

    def _publish_sticky_detection_image_or_fallback(self, error_text: str) -> None:
        if self._publish_cached_success_image():
            return
        self._publish_fallback_image(error_text)

    def _publish_detection_image(self, image: np.ndarray, detections: list[dict]) -> None:
        if image is None:
            return
        output = image.copy()
        if detections:
            output = self._draw_detections(output, detections)
        self._last_success_detection_image = output.copy()
        msg = self._bridge.cv2_to_imgmsg(output, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'front_camera'
        self._pub_ball_detection_image.publish(msg)

    @staticmethod
    def _encode_json_base64_image(image_bytes: bytes) -> tuple[bytes, str]:
        encoded = base64.b64encode(image_bytes).decode('ascii')
        payload = {
            'image': {
                'type': 'base64',
                'value': encoded,
            }
        }
        return json.dumps(payload, ensure_ascii=False).encode('utf-8'), 'application/json'

    @staticmethod
    def _encode_plain_base64_image(image_bytes: bytes) -> tuple[bytes, str]:
        encoded = base64.b64encode(image_bytes).decode('ascii')
        return encoded.encode('ascii'), 'text/plain'

    @staticmethod
    def _read_http_error_body(error: urllib.error.HTTPError) -> str:
        try:
            body = error.read().decode('utf-8', errors='ignore')
        except Exception:
            return ''
        return body

    @staticmethod
    def _extract_predictions(payload: dict) -> list:
        predictions = payload.get('predictions')
        if isinstance(predictions, list):
            return predictions

        outputs = payload.get('outputs')
        if isinstance(outputs, list) and outputs:
            first = outputs[0]
            if isinstance(first, dict):
                preds = first.get('predictions')
                if isinstance(preds, list):
                    return preds

        return []

    def _publish_detection_topics(self, detections: list[dict], image_w: int, image_h: int) -> None:
        detections_payload: list[dict] = []
        relative_positions: list[Optional[dict]] = []
        for detection in detections:
            payload_item = dict(detection)
            relative_position = self._estimate_relative_position(detection)
            relative_positions.append(relative_position)
            if relative_position is not None:
                payload_item['camera_relative_position_m'] = {
                    'x': float(relative_position['x']),
                    'y': float(relative_position['y']),
                    'z': float(relative_position['z']),
                }
                payload_item['camera_range_m'] = float(relative_position['range'])
                payload_item['world_position'] = {
                    'x': float(relative_position['world_x']),
                    'y': float(relative_position['world_y']),
                    'z': float(relative_position['world_z']),
                }
            detections_payload.append(payload_item)

        payload = {
            'count': len(detections),
            'image_width': int(image_w),
            'image_height': int(image_h),
            'detections': detections_payload,
        }
        msg_dets = String()
        msg_dets.data = json.dumps(payload, ensure_ascii=False)
        self._pub_ball_detections.publish(msg_dets)
        self._publish_visible_balls_topic(detections_payload)
        self._debug(f'published detections topic: count={len(detections)}, image={image_w}x{image_h}')

        if not detections:
            return

        best = detections[0]
        best_relative = relative_positions[0] if relative_positions else None
        cx = float(best['x'])
        cy = float(best['y'])
        conf = float(best['confidence'])

        msg_pose = PoseStamped()
        msg_pose.header.stamp = self.get_clock().now().to_msg()
        msg_pose.header.frame_id = 'front_camera'
        if best_relative is not None:
            msg_pose.pose.position.x = float(best_relative['x'])
            msg_pose.pose.position.y = float(best_relative['y'])
            msg_pose.pose.position.z = float(best_relative['z'])
        else:
            msg_pose.pose.position.x = cx / max(1.0, float(image_w))
            msg_pose.pose.position.y = cy / max(1.0, float(image_h))
            msg_pose.pose.position.z = 0.0
        msg_pose.pose.orientation.x = 0.0
        msg_pose.pose.orientation.y = 0.0
        msg_pose.pose.orientation.z = 0.0
        msg_pose.pose.orientation.w = conf
        self._pub_ball_pose.publish(msg_pose)
        self._debug(f'published best pose: x={msg_pose.pose.position.x:.3f}, y={msg_pose.pose.position.y:.3f}, conf={conf:.3f}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BallDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
