import json
import math
import os
import re
import threading
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from . import decision_cruise as planner


THIS_DIR = os.path.dirname(__file__)
PROJECT_ROOT = os.path.abspath(os.path.join(THIS_DIR, "..", "..", "..", "..", ".."))
WHO_IS_DEV_JSON_FILE = os.path.join(PROJECT_ROOT, "config.json")


def _load_default_linear_velocity() -> float:
    default_linear_velocity = 3.0
    try:
        with open(WHO_IS_DEV_JSON_FILE, "r") as f:
            payload = json.loads(f.read().strip())
        if isinstance(payload, dict):
            speed_raw = payload.get("default_linear_velocity", default_linear_velocity)
            parsed_speed = float(speed_raw)
            if parsed_speed > 0:
                default_linear_velocity = parsed_speed
    except Exception:
        pass
    return default_linear_velocity


class _MirrorState:
    def __init__(self) -> None:
        default_speed_payload = json.dumps(
            {"dynamic_waypoints": "", "speed": f"{_load_default_linear_velocity():.6f}"},
            ensure_ascii=True,
            separators=(",", ":"),
        ).encode("utf-8")
        self._lock = threading.Lock()
        self._items = {
            "/data/simulation_data": {
                "body": b"{}",
                "content_type": "application/json; charset=utf-8",
                "has_data": False,
            },
            "/data/front_camera": {
                "body": b"",
                "content_type": "image/jpeg",
                "has_data": False,
            },
            "/data/decisions": {
                "body": default_speed_payload,
                "content_type": "application/json; charset=utf-8",
                "has_data": True,
            },
            "/data/decision_making_data": {
                "body": b"{}",
                "content_type": "application/json; charset=utf-8",
                "has_data": True,
            },
        }

    def set(self, path: str, body: bytes, content_type: str) -> bool:
        with self._lock:
            item = self._items.get(path)
            if item is None:
                return False
            changed = (item["body"] != body) or (item["content_type"] != content_type) or (not item["has_data"])
            item["body"] = body
            item["content_type"] = content_type
            item["has_data"] = True
            return changed

    def get(self, path: str):
        with self._lock:
            item = self._items.get(path)
            if item is None:
                return None
            return {
                "body": item["body"],
                "content_type": item["content_type"],
                "has_data": item["has_data"],
            }


def _build_handler(state: _MirrorState):
    class MirrorHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            item = state.get(self.path)
            if item is None:
                self.send_response(404)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.end_headers()
                self.wfile.write(b"Not Found")
                return
            if not item["has_data"]:
                self.send_response(503)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.end_headers()
                self.wfile.write(b"Mirror has no upstream data yet")
                return
            body = item["body"]
            self.send_response(200)
            self.send_header("Content-Type", item["content_type"])
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            if self.path not in ("/data/decisions", "/data/decision_making_data"):
                self.send_response(404)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.end_headers()
                self.wfile.write(b"Not Found")
                return

            try:
                length = int(self.headers.get("Content-Length", "0"))
            except Exception:
                length = 0

            body = self.rfile.read(length) if length > 0 else b"{}"
            try:
                payload = json.loads(body.decode("utf-8", errors="ignore"))
                if not isinstance(payload, dict):
                    raise ValueError("payload must be a JSON object")
            except Exception:
                self.send_response(400)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.end_headers()
                self.wfile.write(b"Invalid JSON payload")
                return

            canonical = json.dumps(payload, ensure_ascii=True, separators=(",", ":")).encode("utf-8")
            state.set(self.path, canonical, "application/json; charset=utf-8")

            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", "2")
            self.end_headers()
            self.wfile.write(b"{}")

        def log_message(self, fmt, *args):
            # Keep HTTP server logs quiet; ROS logger is used for state changes.
            return

    return MirrorHandler


class WebBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('web_bridge_node')

        self.declare_parameter('remote_host', '192.168.50.1')
        self.declare_parameter('remote_port', 5003)
        self.declare_parameter('local_host', '127.0.0.1')
        self.declare_parameter('local_port', 5003)
        self.declare_parameter('poll_hz', 10.0)
        self.declare_parameter('request_timeout', 1.0)
        self.declare_parameter('camera_remote_host', '192.168.50.1')
        self.declare_parameter('camera_remote_port', 5003)
        self.declare_parameter('camera_path', '/data/front_camera')
        self.declare_parameter('camera_topic', '/front_camera')
        self.declare_parameter('camera_poll_hz', 10.0)
        self.declare_parameter('camera_request_timeout', 1.0)
        self.declare_parameter('pose_estimation', False)
        self.declare_parameter('web_debug', False)

        self.remote_host = self.get_parameter('remote_host').get_parameter_value().string_value
        self.remote_port = int(self.get_parameter('remote_port').get_parameter_value().integer_value)
        self.local_host = self.get_parameter('local_host').get_parameter_value().string_value
        self.local_port = int(self.get_parameter('local_port').get_parameter_value().integer_value)
        poll_hz = float(self.get_parameter('poll_hz').get_parameter_value().double_value)
        self.request_timeout = float(self.get_parameter('request_timeout').get_parameter_value().double_value)
        self.camera_remote_host = self.get_parameter('camera_remote_host').get_parameter_value().string_value
        self.camera_remote_port = int(self.get_parameter('camera_remote_port').get_parameter_value().integer_value)
        self.camera_path = self.get_parameter('camera_path').get_parameter_value().string_value
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        camera_poll_hz = float(self.get_parameter('camera_poll_hz').get_parameter_value().double_value)
        self.camera_request_timeout = float(
            self.get_parameter('camera_request_timeout').get_parameter_value().double_value
        )
        self._pose_estimation_enabled = bool(
            self.get_parameter('pose_estimation').get_parameter_value().bool_value
        )
        self._web_debug_enabled = bool(
            self.get_parameter('web_debug').get_parameter_value().bool_value
        )

        planner.DATA_FLOW = 'web'

        if self.request_timeout <= 0.0:
            self.request_timeout = 1.0
        if self.camera_request_timeout <= 0.0:
            self.camera_request_timeout = 1.0

        self.remote_targets = (
            {
                "path": "/data/simulation_data",
                "url": f"http://{self.remote_host}:{self.remote_port}/data/simulation_data",
                "content_type": "application/json; charset=utf-8",
            },
        )

        self._state = _MirrorState()
        self._server = ThreadingHTTPServer((self.local_host, self.local_port), _build_handler(self._state))
        self._server_thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._server_thread.start()

        self._error_logged = {item["path"]: False for item in self.remote_targets}

        self._last_decisions_payload: dict | None = None
        self._last_decision_making_payload: dict | None = None

        self.create_subscription(String, '/decisions', self._on_decisions, 10)
        self.create_subscription(String, '/decision_making_data', self._on_decision_making_data, 10)

        self.pub_current_position = self.create_publisher(PoseStamped, '/current_position', 10)
        self.pub_visible_balls = self.create_publisher(String, '/visible_balls', 10)
        self.pub_waypoint_status = self.create_publisher(String, '/waypoint_status', 10)
        self.pub_time = self.create_publisher(String, '/time', 10)
        self.pub_radar_sensor = self.create_publisher(String, '/radar_sensor', 10)
        self.pub_front_camera = self.create_publisher(Image, self.camera_topic, 10)

        self._cv_bridge = CvBridge()
        self._camera_error_logged = False

        period = 0.1 if poll_hz <= 0.0 else (1.0 / poll_hz)
        self.create_timer(period, self._poll_once)
        camera_period = 0.1 if camera_poll_hz <= 0.0 else (1.0 / camera_poll_hz)
        self.create_timer(camera_period, self._poll_camera_once)

        self.get_logger().info(
            f'web_bridge_node started; upstream={self.remote_host}:{self.remote_port}, '
            f'local_mirror=http://{self.local_host}:{self.local_port}, poll_hz={poll_hz}'
        )
        self.get_logger().info(
            f'front_camera bridge enabled; upstream={self.camera_remote_host}:{self.camera_remote_port}'
            f'{self.camera_path}, topic={self.camera_topic}, poll_hz={camera_poll_hz}'
        )

    def _poll_once(self) -> None:
        for target in self.remote_targets:
            path = target["path"]
            url = target["url"]
            try:
                with urllib.request.urlopen(url, timeout=self.request_timeout) as res:
                    content_type = res.headers.get_content_type()
                    body = res.read()
                merged_content_type = f"{content_type}; charset=utf-8" if content_type else target["content_type"]
                changed = self._state.set(path, body, merged_content_type)
                if changed:
                    self.get_logger().info(f'updated mirror payload for {path} from {url}')
                self._error_logged[path] = False
            except Exception as exc:
                if not self._error_logged[path]:
                    self.get_logger().warn(f'upstream fetch failed for {url}: {exc}')
                    self._error_logged[path] = True

        self._publish_sim_topics_from_cache()

    def _publish_sim_topics_from_cache(self) -> None:
        data_item = self._state.get('/data/simulation_data')
        if not data_item or not data_item.get('has_data'):
            return
        try:
            payload = json.loads(data_item['body'].decode('utf-8', errors='ignore'))
        except Exception:
            return
        if not isinstance(payload, dict):
            return

        current_text = str(payload.get('current_position', '')).strip()
        visible_text = str(payload.get('visible_balls', '')).strip()
        waypoint_text = str(payload.get('waypoint_status', '')).strip()
        time_text = str(payload.get('time', '')).strip()
        radar_text = str(payload.get('radar_sensor', '')).strip()

        if not self._pose_estimation_enabled:
            pose = self._parse_current_position(current_text)
            if pose is not None:
                x, y, theta_deg = pose
                theta = math.radians(theta_deg)
                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'map'
                msg.pose.position.x = x
                msg.pose.position.y = y
                msg.pose.position.z = 0.0
                msg.pose.orientation.x = 0.0
                msg.pose.orientation.y = 0.0
                msg.pose.orientation.z = math.sin(theta * 0.5)
                msg.pose.orientation.w = math.cos(theta * 0.5)
                self.pub_current_position.publish(msg)

        vis_msg = String()
        vis_msg.data = visible_text
        self.pub_visible_balls.publish(vis_msg)

        wp_msg = String()
        wp_msg.data = waypoint_text
        self.pub_waypoint_status.publish(wp_msg)

        if time_text:
            t_msg = String()
            t_msg.data = time_text
            self.pub_time.publish(t_msg)

        if radar_text:
            radar_msg = String()
            radar_msg.data = radar_text
            self.pub_radar_sensor.publish(radar_msg)

    def _poll_camera_once(self) -> None:
        url = f'http://{self.camera_remote_host}:{self.camera_remote_port}{self.camera_path}'
        try:
            with urllib.request.urlopen(url, timeout=self.camera_request_timeout) as res:
                content_type = (res.headers.get_content_type() or '').lower()
                body = res.read()
        except Exception as exc:
            if not self._camera_error_logged:
                self.get_logger().warn(f'front camera fetch failed for {url}: {exc}')
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

        merged_content_type = f"{content_type}; charset=utf-8" if content_type else "image/jpeg"
        changed = self._state.set('/data/front_camera', body, merged_content_type)
        if not changed:
            return

        self.get_logger().info(f'updated mirror payload for /data/front_camera from {url}')

        image = cv2.imdecode(np.frombuffer(body, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().warn('front camera payload could not be decoded')
            return

        msg = self._cv_bridge.cv2_to_imgmsg(image, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'front_camera'
        self.pub_front_camera.publish(msg)

    def _on_decisions(self, msg: String) -> None:
        payload = self._parse_json_payload(msg.data)
        if payload is None:
            return
        self._last_decisions_payload = payload
        if not self._web_debug_enabled:
            return
        if not planner._post_decisions_data(payload):
            self.get_logger().warn('failed to post decisions payload')

    def _on_decision_making_data(self, msg: String) -> None:
        payload = self._parse_json_payload(msg.data)
        if payload is None:
            return
        self._last_decision_making_payload = payload
        if not self._web_debug_enabled:
            return
        if not planner._post_decision_making_data(payload):
            self.get_logger().warn('failed to post decision_making_data payload')


    def _parse_json_payload(self, raw: str) -> dict | None:
        try:
            payload = json.loads(raw)
        except Exception:
            return None
        if not isinstance(payload, dict):
            return None
        return payload

    def _parse_current_position(self, text: str):
        nums = re.findall(r'[-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?', text or '')
        if len(nums) < 3:
            return None
        try:
            return float(nums[0]), float(nums[1]), float(nums[2])
        except Exception:
            return None

    def destroy_node(self):
        try:
            self._server.shutdown()
            self._server.server_close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
