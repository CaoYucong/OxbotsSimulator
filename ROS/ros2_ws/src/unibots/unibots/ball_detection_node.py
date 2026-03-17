import json
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
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class BallDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__('ball_detection_node')

        self.declare_parameter('front_camera_topic', '/front_camera')
        self.declare_parameter('ball_detection_image_topic', '/ball_detection_image')
        self.declare_parameter('ball_pose_topic', '/ball_pose')
        self.declare_parameter('ball_detections_topic', '/ball_detections')
        self.declare_parameter('local_inference_url', 'http://127.0.0.1:9001')
        self.declare_parameter('local_model_id', 'unibot-ball-detection/1')
        self.declare_parameter('roboflow_api_key', '')
        self.declare_parameter('request_timeout_sec', 10.0)
        self.declare_parameter('infer_hz', 0.1)
        self.declare_parameter('min_confidence', 0.25)
        self.declare_parameter('debug_logs', True)
        self.declare_parameter('debug_log_every_sec', 2.0)

        front_camera_topic = self.get_parameter('front_camera_topic').get_parameter_value().string_value
        ball_detection_image_topic = (
            self.get_parameter('ball_detection_image_topic').get_parameter_value().string_value
        )
        self._ball_pose_topic = self.get_parameter('ball_pose_topic').get_parameter_value().string_value
        self._ball_detections_topic = (
            self.get_parameter('ball_detections_topic').get_parameter_value().string_value
        )
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

        if self._request_timeout_sec <= 0.0:
            self._request_timeout_sec = 1.0
        if self._infer_hz <= 0.0:
            self._infer_hz = 0.1
        if self._debug_log_every_sec <= 0.0:
            self._debug_log_every_sec = 2.0

        self._bridge = CvBridge()
        self._latest_front_image: Optional[np.ndarray] = None
        self._latest_front_stamp = None
        self._latest_detections: list[dict] = []
        self._last_infer_error_warn = 0.0
        self._last_debug_log: dict[str, float] = {}
        self._last_error_text = 'waiting for front camera image'
        self._fallback_image = self._build_fallback_image(self._last_error_text)
        self._last_success_detection_image: Optional[np.ndarray] = None

        self._pub_ball_detection_image = self.create_publisher(Image, ball_detection_image_topic, 10)
        self._pub_ball_pose = self.create_publisher(PoseStamped, self._ball_pose_topic, 10)
        self._pub_ball_detections = self.create_publisher(String, self._ball_detections_topic, 10)

        self.create_subscription(Image, front_camera_topic, self._on_front_image, 10)

        self.create_timer(1.0 / self._infer_hz, self._infer_tick)

        self.get_logger().info(
            'ball_detection_node started; '
            f'front_camera_topic={front_camera_topic}, '
            f'ball_detection_image_topic={ball_detection_image_topic}, '
            f'mode=local, model_id={self._local_model_id}, '
            f'infer_hz={self._infer_hz:.2f}, min_confidence={self._min_confidence:.2f}'
        )
        self._debug(
            'debug config: '
            f'request_timeout_sec={self._request_timeout_sec:.2f}, '
            f'api_key_set={bool(self._roboflow_api_key)}, '
            f'debug_log_every_sec={self._debug_log_every_sec:.2f}'
        )

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

    def _on_front_image(self, msg: Image) -> None:
        try:
            self._latest_front_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._latest_front_stamp = msg.header.stamp
            h, w = self._latest_front_image.shape[:2]
            self._debug_throttled('front_image', f'received front image: {w}x{h}')
        except Exception as exc:
            now = time.monotonic()
            if now - self._last_infer_error_warn >= 1.0:
                self._last_infer_error_warn = now
                self.get_logger().warn(f'failed to decode front camera image: {exc}')

    def _infer_tick(self) -> None:
        if self._latest_front_image is None:
            self._debug_throttled('no_front_image', 'infer tick skipped: no front camera image yet')
            return

        image = self._latest_front_image
        detections = self._infer_with_roboflow(image)
        self._latest_detections = detections
        if not detections:
            self._debug_throttled('no_detection_publish', 'skip publish: no valid detections in this tick')
            return
        self._publish_detection_topics(detections, image.shape[1], image.shape[0])
        self._publish_detection_image(image, detections)

    def _infer_with_roboflow(self, image: np.ndarray) -> list[dict]:
        ok, encoded = cv2.imencode('.jpg', image)
        if not ok:
            self._debug_throttled('encode_failed', 'cv2.imencode failed for inference frame')
            return []

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

        try:
            payload = self._send_infer_request_with_fallback(endpoint_with_key, image_bytes)
        except urllib.error.HTTPError as exc:
            if exc.code in (401, 404) and self._roboflow_api_key:
                self._debug(f'retry with Bearer auth after http {exc.code}')
                try:
                    payload = self._send_infer_request_with_fallback(
                        endpoint,
                        image_bytes,
                        {'Authorization': f'Bearer {self._roboflow_api_key}'},
                    )
                except urllib.error.HTTPError as retry_exc:
                    if retry_exc.code == 401:
                        self._last_error_text = 'roboflow 401 (invalid api key)'
                        self.get_logger().warn(f'roboflow http error: {retry_exc.code} {retry_exc.reason}')
                        return []
                    else:
                        self.get_logger().warn(f'roboflow http error: {retry_exc.code} {retry_exc.reason}')
                        self._last_error_text = f'roboflow http error: {retry_exc.code}'
                        return []
            else:
                self.get_logger().warn(f'roboflow http error: {exc.code} {exc.reason}')
                if exc.code == 401:
                    self._last_error_text = 'roboflow 401 (set roboflow_api_key or ROBOFLOW_API_KEY)'
                else:
                    self._last_error_text = f'roboflow http error: {exc.code}'
                return []
        except Exception as exc:
            now = time.monotonic()
            if now - self._last_infer_error_warn >= 1.0:
                self._last_infer_error_warn = now
                self.get_logger().warn(f'roboflow infer failed: {exc}')
            self._last_error_text = f'roboflow infer failed: {type(exc).__name__}'
            return []

        predictions = self._extract_predictions(payload)
        if not isinstance(predictions, list):
            self._debug('unexpected payload format: predictions is not a list')
            return []

        self._debug(f'infer response: raw_predictions={len(predictions)}')

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
        return result

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
        if not detections:
            return

        output = self._draw_detections(image, detections)
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
        payload = {
            'count': len(detections),
            'image_width': int(image_w),
            'image_height': int(image_h),
            'detections': detections,
        }
        msg_dets = String()
        msg_dets.data = json.dumps(payload, ensure_ascii=False)
        self._pub_ball_detections.publish(msg_dets)
        self._debug(f'published detections topic: count={len(detections)}, image={image_w}x{image_h}')

        if not detections:
            return

        best = detections[0]
        cx = float(best['x'])
        cy = float(best['y'])
        conf = float(best['confidence'])

        msg_pose = PoseStamped()
        msg_pose.header.stamp = self.get_clock().now().to_msg()
        msg_pose.header.frame_id = 'front_camera'
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
