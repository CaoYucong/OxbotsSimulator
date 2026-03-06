import rclpy
from rclpy.node import Node
import threading
import urllib.request
import json
import math
import re
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class _MirrorState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._items = {
            "/simulation_data": {
                "body": b"",
                "content_type": "text/html; charset=utf-8",
                "has_data": False,
            },
            "/data/simulation_data": {
                "body": b"{}",
                "content_type": "application/json; charset=utf-8",
                "has_data": False,
            },
            "/data/decisions": {
                "body": b'{"dynamic_waypoints":"","speed":"0.300000"}',
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


class FileBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('file_bridge_node')

        self.declare_parameter('remote_host', '192.168.50.1')
        self.declare_parameter('remote_port', 5003)
        self.declare_parameter('local_host', '127.0.0.1')
        self.declare_parameter('local_port', 5003)
        self.declare_parameter('poll_hz', 10.0)
        self.declare_parameter('request_timeout', 0.25)

        self.remote_host = self.get_parameter('remote_host').get_parameter_value().string_value
        self.remote_port = int(self.get_parameter('remote_port').get_parameter_value().integer_value)
        self.local_host = self.get_parameter('local_host').get_parameter_value().string_value
        self.local_port = int(self.get_parameter('local_port').get_parameter_value().integer_value)
        poll_hz = float(self.get_parameter('poll_hz').get_parameter_value().double_value)
        self.request_timeout = float(self.get_parameter('request_timeout').get_parameter_value().double_value)

        if self.request_timeout <= 0.0:
            self.request_timeout = 0.25

        self.remote_targets = (
            {
                "path": "/simulation_data",
                "url": f"http://{self.remote_host}:{self.remote_port}/simulation_data",
                "content_type": "text/html; charset=utf-8",
            },
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

        self.pub_current_position = self.create_publisher(PoseStamped, '/sim/current_position', 10)
        self.pub_visible_balls = self.create_publisher(String, '/sim/visible_balls', 10)
        self.pub_waypoint_status = self.create_publisher(String, '/sim/waypoint_status', 10)
        self.pub_time = self.create_publisher(String, '/sim/time', 10)

        period = 0.1 if poll_hz <= 0.0 else (1.0 / poll_hz)
        self.create_timer(period, self._poll_once)

        self.get_logger().info(
            f'file_bridge_node started; upstream={self.remote_host}:{self.remote_port}, '
            f'local_mirror=http://{self.local_host}:{self.local_port}, poll_hz={poll_hz}'
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
    node = FileBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
