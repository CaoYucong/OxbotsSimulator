import base64
import json
import math
import os
import re
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import cv2
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
FIELD_VIEWER_ASSETS_DIR = os.path.join(THIS_DIR, "field_viewer")
INDEX_FILE = os.path.join(FIELD_VIEWER_ASSETS_DIR, "index.html")
SIM_DATA_FILE = os.path.join(FIELD_VIEWER_ASSETS_DIR, "simulation_data.html")


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


def _read_lines_from_text(text: str) -> list[str]:
    return [line.strip() for line in (text or "").splitlines() if line.strip()]


def _read_first_line_number_from_text(text: str) -> str:
    lines = _read_lines_from_text(text)
    if not lines:
        return ""
    m = re.search(r"[-+]?\d*\.?\d+", lines[0])
    return m.group(0) if m else ""


def _read_last_line_from_text(text: str) -> str:
    lines = _read_lines_from_text(text)
    if not lines:
        return ""
    return lines[-1]


def _parse_tuple(line: str) -> list[str]:
    if line.startswith("(") and line.endswith(")"):
        line = line[1:-1]
    return [p.strip() for p in line.split(",")]


def _parse_xy_bearing(line: str):
    parts = _parse_tuple(line)
    if len(parts) < 2:
        return None
    try:
        x = float(parts[0])
        y = float(parts[1])
    except Exception:
        return None
    bearing = None
    if len(parts) >= 3:
        try:
            bearing = float(parts[2])
        except Exception:
            bearing = None
    return x, y, bearing


def _extract_xy_from_line(line: str):
    line = line.split("#", 1)[0].strip()
    if not line:
        return None
    if line.endswith(","):
        line = line[:-1].strip()
    nums = re.findall(r"[-+]?[0-9]*\.?[0-9]+", line)
    if len(nums) < 2:
        return None
    try:
        return float(nums[0]), float(nums[1])
    except Exception:
        return None


def _parse_xy_type(line: str):
    parts = _parse_tuple(line)
    if len(parts) < 2:
        return None
    try:
        x = float(parts[0])
        y = float(parts[1])
    except Exception:
        return None
    typ = parts[2].upper() if len(parts) >= 3 else "PING"
    return x, y, typ


def _parse_current(line: str):
    parts = _parse_tuple(line)
    if len(parts) < 2:
        return None
    try:
        x = float(parts[0])
        y = float(parts[1])
    except Exception:
        return None
    bearing = None
    if len(parts) >= 3:
        try:
            bearing = float(parts[2])
        except Exception:
            bearing = None
    return {"x": x, "y": y, "bearing": bearing}


def _read_numeric_matrix_from_text(text: str) -> list[list[float]]:
    lines = _read_lines_from_text(text)
    if not lines:
        return []

    matrix: list[list[float]] = []
    for line in lines:
        nums = re.findall(r"[-+]?\d*\.?\d+", line)
        if not nums:
            continue
        row: list[float] = []
        for n in nums:
            try:
                row.append(float(n))
            except Exception:
                row.append(0.0)
        matrix.append(row)
    return matrix


def _matrix_to_world_tiles(matrix: list[list[float]]) -> list[dict[str, float]]:
    if not matrix:
        return []

    rows = len(matrix)
    cols = min(len(r) for r in matrix if r) if any(matrix) else 0
    if rows <= 0 or cols <= 0:
        return []

    tile_w = 2.0 / cols
    tile_h = 2.0 / rows
    x0 = -1.0 + tile_w * 0.5
    y0 = 1.0 - tile_h * 0.5

    out: list[dict[str, float]] = []
    for r in range(rows):
        for c in range(cols):
            try:
                v = float(matrix[r][c])
            except Exception:
                v = 0.0
            x = x0 + c * tile_w
            y = y0 - r * tile_h
            out.append({"x": x, "y": y, "value": v})
    return out


def _read_text_value(payload: dict, key: str) -> str:
    value = payload.get(key)
    if isinstance(value, str):
        return value
    if value is None:
        return ""
    return str(value)


def _parse_radar_sensor_text(text: str):
    parts = [p.strip() for p in (text or "").split(",")]
    if len(parts) < 5:
        return None
    try:
        return {
            "time": float(parts[0]),
            "front": float(parts[1]),
            "right": float(parts[2]),
            "left": float(parts[3]),
            "rear": float(parts[4]),
        }
    except Exception:
        return None


def _state_payload(state: "_MirrorState", path: str) -> dict:
    item = state.get(path)
    if not item or not item.get("has_data"):
        return {}
    try:
        payload = json.loads(item["body"].decode("utf-8", errors="ignore"))
    except Exception:
        return {}
    return payload if isinstance(payload, dict) else {}


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
                "seq": 0,
            },
            "/data/current_position": {
                "body": b"{}",
                "content_type": "application/json; charset=utf-8",
                "has_data": False,
                "seq": 0,
            },
            "/data/visible_balls": {
                "body": b"{}",
                "content_type": "application/json; charset=utf-8",
                "has_data": False,
                "seq": 0,
            },
            "/data/waypoint_status": {
                "body": b"{}",
                "content_type": "application/json; charset=utf-8",
                "has_data": False,
                "seq": 0,
            },
            "/data/time": {
                "body": b"{}",
                "content_type": "application/json; charset=utf-8",
                "has_data": False,
                "seq": 0,
            },
            "/data/radar_sensor": {
                "body": b"{}",
                "content_type": "application/json; charset=utf-8",
                "has_data": False,
                "seq": 0,
            },
            "/data/front_camera": {
                "body": b"",
                "content_type": "image/jpeg",
                "has_data": False,
                "seq": 0,
            },
            "/data/processed_image": {
                "body": b"",
                "content_type": "image/jpeg",
                "has_data": False,
                "seq": 0,
            },
            "/data/decisions": {
                "body": default_speed_payload,
                "content_type": "application/json; charset=utf-8",
                "has_data": True,
                "seq": 0,
            },
            "/data/decision_making_data": {
                "body": b"{}",
                "content_type": "application/json; charset=utf-8",
                "has_data": True,
                "seq": 0,
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
            if changed:
                item["seq"] += 1
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
                "seq": item.get("seq", 0),
            }


def _build_handler(state: _MirrorState):
    class MirrorHandler(BaseHTTPRequestHandler):
        def _send_json(self, payload, status=200):
            body = json.dumps(payload).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _send_text(self, text: str, status=200, content_type="text/html"):
            body = text.encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _send_bytes(self, body: bytes, status=200, content_type="application/octet-stream"):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _send_html_file(self, path: str):
            try:
                with open(path, "r") as f:
                    self._send_text(f.read(), 200, "text/html")
            except Exception:
                self._send_text("not found", 404, "text/plain")

        def _redirect(self, location: str, status: int = 302):
            self.send_response(status)
            self.send_header("Location", location)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()

        def _get_current(self):
            payload = _state_payload(state, "/data/current_position")
            if not payload:
                return None
            try:
                x = float(payload.get("x"))
                y = float(payload.get("y"))
            except Exception:
                return None
            bearing = payload.get("bearing")
            try:
                bearing = float(bearing) if bearing is not None else None
            except Exception:
                bearing = None
            return {"x": x, "y": y, "bearing": bearing}

        def _get_visible_text(self) -> str:
            payload = _state_payload(state, "/data/visible_balls")
            return _read_text_value(payload, "visible_balls")

        def _get_waypoint_status_text(self) -> str:
            payload = _state_payload(state, "/data/waypoint_status")
            return _read_text_value(payload, "waypoint_status")

        def _get_time_text(self) -> str:
            payload = _state_payload(state, "/data/time")
            return _read_text_value(payload, "time")

        def _get_balls_from_text(self, text: str):
            out = []
            for line in _read_lines_from_text(text):
                item = _parse_xy_type(line)
                if item is None:
                    continue
                x, y, typ = item
                out.append({"x": x, "y": y, "type": typ})
            return out

        def _get_obstacles(self, sim_payload: dict):
            out = []
            for line in _read_lines_from_text(_read_text_value(sim_payload, "obstacle_robot")):
                item = _parse_xy_bearing(line)
                if item is None:
                    continue
                x, y, bearing = item
                out.append({"x": x, "y": y, "bearing": bearing})
            return out

        def _get_dynamic_waypoint(self, decisions_payload: dict):
            for line in _read_lines_from_text(_read_text_value(decisions_payload, "dynamic_waypoints")):
                item = _extract_xy_from_line(line)
                if item is None:
                    continue
                x, y = item
                return {"x": x, "y": y}
            return None

        def _get_stack_waypoint(self, decision_making_payload: dict):
            lines = _read_lines_from_text(_read_text_value(decision_making_payload, "waypoints_stack"))
            for line in reversed(lines):
                item = _extract_xy_from_line(line)
                if item is None:
                    continue
                x, y = item
                return {"x": x, "y": y}
            return None

        def _get_robot_around(self, decision_making_payload: dict):
            out = []
            for line in _read_lines_from_text(_read_text_value(decision_making_payload, "robot_around")):
                item = _extract_xy_from_line(line)
                if item is None:
                    continue
                x, y = item
                out.append({"x": x, "y": y})
            return out

        def _get_radar_history(self, decision_making_payload: dict):
            out = []
            for line in _read_lines_from_text(_read_text_value(decision_making_payload, "radar_memory")):
                parts = [p.strip() for p in line.split(",")]
                if len(parts) < 5:
                    continue
                try:
                    out.append(
                        {
                            "t": float(parts[0]),
                            "front": float(parts[1]),
                            "right": float(parts[2]),
                            "left": float(parts[3]),
                            "rear": float(parts[4]),
                        }
                    )
                except Exception:
                    continue
            return out

        def _get_tiles(self, decision_making_payload: dict, key: str):
            matrix = _read_numeric_matrix_from_text(_read_text_value(decision_making_payload, key))
            return _matrix_to_world_tiles(matrix)

        def _get_all_ball_path(self, decision_making_payload: dict):
            path = []
            for line in _read_lines_from_text(_read_text_value(decision_making_payload, "planned_waypoints")):
                item = _extract_xy_from_line(line)
                if item is None:
                    continue
                x, y = item
                path.append({"x": x, "y": y})
            return {"enabled": len(path) > 0, "path": path}

        def _stream_json_payload(self, source_path: str):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.end_headers()

            last_seq = -1
            last_payload_text = ""
            while True:
                try:
                    item = state.get(source_path)
                    if item and item.get("has_data"):
                        seq = item.get("seq", 0)
                        if seq != last_seq:
                            try:
                                payload = json.loads(item["body"].decode("utf-8", errors="ignore"))
                                if not isinstance(payload, dict):
                                    payload = {}
                            except Exception:
                                payload = {}
                            payload_text = json.dumps(payload)
                            self.wfile.write(f"data: {payload_text}\n\n".encode("utf-8"))
                            self.wfile.flush()
                            last_seq = seq
                    else:
                        payload_text = "{}"
                        if payload_text != last_payload_text:
                            self.wfile.write(f"data: {payload_text}\n\n".encode("utf-8"))
                            self.wfile.flush()
                            last_payload_text = payload_text
                    time.sleep(0.1)
                except (BrokenPipeError, ConnectionResetError):
                    break
                except Exception:
                    break

        def do_GET(self):
            path = urlparse(self.path).path

            if path in ("/", "/index.html"):
                self._send_html_file(INDEX_FILE)
                return
            if path == "/simulation_data":
                self._send_html_file(SIM_DATA_FILE)
                return
            if path == "/decisions":
                self._redirect("/")
                return
            if path == "/decision_making_data":
                self._redirect("/")
                return
            if path == "/processed_image":
                page = """<!doctype html>
<html lang=\"en\">
<head>
    <meta charset=\"utf-8\" />
    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />
    <title>Processed Image</title>
    <style>
        body { font-family: Arial, sans-serif; background: #0f1115; color: #f3f4f6; margin: 0; }
        .wrap { padding: 16px; }
        .frame { max-width: 100%; aspect-ratio: 16 / 9; border: 1px solid #23262d; background: #151a21; }
        .frame img { width: 100%; height: 100%; object-fit: contain; display: block; }
        .meta { font-size: 12px; color: #9ca3af; margin-top: 8px; }
    </style>
</head>
<body>
    <div class=\"wrap\">
        <h2>Processed Image</h2>
        <div class=\"frame\"><img id=\"cam\" alt=\"processed image\" /></div>
        <div class=\"meta\" id=\"meta\"></div>
    </div>
    <script>
        const img = document.getElementById('cam');
        const meta = document.getElementById('meta');
        function refresh() {
            const ts = Date.now();
            img.src = `/data/processed_image?t=${ts}`;
            meta.textContent = `Updated ${new Date(ts).toLocaleTimeString()}`;
        }
        refresh();
        setInterval(refresh, 200);
    </script>
</body>
</html>"""
                self._send_text(page, 200, "text/html")
                return

            sim_payload = _state_payload(state, "/data/simulation_data")
            decisions_payload = _state_payload(state, "/data/decisions")
            decision_making_payload = _state_payload(state, "/data/decision_making_data")
            decision_making_payload.setdefault("waypoints_stack", "")

            if path == "/data/current":
                self._send_json({"current": self._get_current()})
                return
            if path == "/data/balls":
                balls_text = self._get_visible_text()
                self._send_json({"balls": self._get_balls_from_text(balls_text)})
                return
            if path == "/data/visible":
                visible_text = self._get_visible_text()
                self._send_json({"visible": self._get_balls_from_text(visible_text)})
                return
            if path == "/data/obstacles":
                self._send_json({"obstacles": self._get_obstacles(sim_payload)})
                return
            if path == "/data/waypoints":
                self._send_json(
                    {
                        "dynamic": self._get_dynamic_waypoint(decisions_payload),
                        "stack": self._get_stack_waypoint(decision_making_payload),
                    }
                )
                return
            if path == "/data/robot-around":
                self._send_json({"vectors": self._get_robot_around(decision_making_payload)})
                return
            if path == "/data/radar-history":
                self._send_json({"history": self._get_radar_history(decision_making_payload)})
                return
            if path == "/data/tile-seen-time":
                self._send_json({"tiles": self._get_tiles(decision_making_payload, "tile_seen_time")})
                return
            if path == "/data/ball-tile-memory":
                self._send_json({"tiles": self._get_tiles(decision_making_payload, "ball_tile_memory")})
                return
            if path == "/data/unseen-tile-memory":
                self._send_json({"tiles": self._get_tiles(decision_making_payload, "unseen_tile_memory")})
                return
            if path == "/data/unseen-regions":
                self._send_json({"tiles": self._get_tiles(decision_making_payload, "unseen_regions")})
                return
            if path == "/data/text-status":
                self._send_json(
                    {
                        "waypoint_status": self._get_waypoint_status_text(),
                        "mode": _read_text_value(decision_making_payload, "mode"),
                        "collision_avoiding": _read_text_value(decision_making_payload, "collision_avoiding"),
                        "simulation_time": _read_first_line_number_from_text(self._get_time_text()),
                        "random_seed": _read_text_value(sim_payload, "random_seed"),
                        "collision_counter": _read_first_line_number_from_text(
                            _read_text_value(decision_making_payload, "collision_counter")
                        ),
                        "last_ball_taken": _read_last_line_from_text(_read_text_value(sim_payload, "ball_taken_history")),
                    }
                )
                return
            if path == "/data/all-ball-path":
                self._send_json(self._get_all_ball_path(decision_making_payload))
                return

            if path == "/data/simulation_data":
                self._send_json(sim_payload)
                return
            if path == "/data/decisions":
                self._send_json(decisions_payload)
                return
            if path == "/data/decision_making_data":
                self._send_json(decision_making_payload)
                return
            if path == "/data/front_camera":
                item = state.get("/data/front_camera")
                if item and item.get("has_data") and item.get("body"):
                    self._send_bytes(item["body"], 200, item.get("content_type") or "image/jpeg")
                else:
                    self._send_text("no image", 404, "text/plain")
                return
            if path == "/data/processed_image":
                item = state.get("/data/processed_image")
                if item and item.get("has_data") and item.get("body"):
                    self._send_bytes(item["body"], 200, item.get("content_type") or "image/jpeg")
                else:
                    self._send_text("no image", 404, "text/plain")
                return
            if path == "/data/simulation-stream":
                self._stream_json_payload("/data/simulation_data")
                return
            if path == "/data/decisions-stream":
                self._stream_json_payload("/data/decisions")
                return
            if path == "/data/decision_making_data-stream":
                self._stream_json_payload("/data/decision_making_data")
                return

            item = state.get(path)
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
            path = urlparse(self.path).path
            if path not in (
                "/data/simulation_data",
                "/data/decisions",
                "/data/decision_making_data",
                "/front_camera",
                "/processed_image",
            ):
                self.send_response(404)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.end_headers()
                self.wfile.write(b"Not Found")
                return

            try:
                length = int(self.headers.get("Content-Length", "0"))
            except Exception:
                length = 0

            body = self.rfile.read(length) if length > 0 else b""

            if path == "/front_camera":
                self._send_text("front camera upload disabled", 200, "text/plain")
                return

            if path == "/processed_image":
                if not body:
                    self._send_text("empty payload", 400, "text/plain")
                    return
                content_type = self.headers.get("Content-Type", "application/octet-stream")
                if content_type.startswith("application/json"):
                    try:
                        payload = json.loads(body.decode("utf-8", errors="ignore"))
                        if not isinstance(payload, dict):
                            raise ValueError("payload must be object")
                        data_uri = payload.get("image")
                        image_base64 = payload.get("image_base64")
                        mime = payload.get("mime")
                        if isinstance(data_uri, str) and data_uri.startswith("data:"):
                            header, b64 = data_uri.split(",", 1)
                            mime = header.split(";", 1)[0].split(":", 1)[1]
                            image_bytes = base64.b64decode(b64)
                            state.set("/data/processed_image", image_bytes, mime)
                        elif isinstance(image_base64, str):
                            image_bytes = base64.b64decode(image_base64)
                            state.set("/data/processed_image", image_bytes, mime or "image/jpeg")
                        else:
                            self._send_text("invalid image payload", 400, "text/plain")
                            return
                    except Exception:
                        self._send_text("invalid json", 400, "text/plain")
                        return
                else:
                    state.set("/data/processed_image", body, content_type)
                self._send_text("ok", 200, "text/plain")
                return

            if not body:
                body = b"{}"
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
            state.set(path, canonical, "application/json; charset=utf-8")

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
        self.declare_parameter('camera_topic', '/front_camera')
        self.declare_parameter('pose_estimation', False)
        self.declare_parameter('web_debug', False)

        self.remote_host = self.get_parameter('remote_host').get_parameter_value().string_value
        self.remote_port = int(self.get_parameter('remote_port').get_parameter_value().integer_value)
        self.local_host = self.get_parameter('local_host').get_parameter_value().string_value
        self.local_port = int(self.get_parameter('local_port').get_parameter_value().integer_value)
        poll_hz = float(self.get_parameter('poll_hz').get_parameter_value().double_value)
        self.request_timeout = float(self.get_parameter('request_timeout').get_parameter_value().double_value)
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self._pose_estimation_enabled = bool(
            self.get_parameter('pose_estimation').get_parameter_value().bool_value
        )
        self._web_debug_enabled = bool(
            self.get_parameter('web_debug').get_parameter_value().bool_value
        )

        planner.DATA_FLOW = 'web'

        if self.request_timeout <= 0.0:
            self.request_timeout = 1.0

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

        self.create_subscription(String, '/simulation_data', self._on_simulation_data, 10)
        self.create_subscription(PoseStamped, '/current_position', self._on_current_position_topic, 10)
        self.create_subscription(String, '/visible_balls', self._on_visible_balls_topic, 10)
        self.create_subscription(String, '/waypoint_status', self._on_waypoint_status_topic, 10)
        self.create_subscription(String, '/time', self._on_time_topic, 10)
        self.create_subscription(String, '/radar_sensor', self._on_radar_sensor_topic, 10)
        self.create_subscription(String, '/decisions', self._on_decisions, 10)
        self.create_subscription(String, '/decision_making_data', self._on_decision_making_data, 10)

        self.pub_simulation_data = self.create_publisher(String, '/simulation_data', 10)
        self.pub_current_position = self.create_publisher(PoseStamped, '/current_position', 10)
        self.pub_visible_balls = self.create_publisher(String, '/visible_balls', 10)
        self.pub_waypoint_status = self.create_publisher(String, '/waypoint_status', 10)
        self.pub_time = self.create_publisher(String, '/time', 10)
        self.create_subscription(Image, self.camera_topic, self._on_front_camera_msg, 10)
        self.create_subscription(Image, '/processed_image', self._on_processed_image_msg, 10)

        self._cv_bridge = CvBridge()

        period = 0.1 if poll_hz <= 0.0 else (1.0 / poll_hz)
        self.create_timer(period, self._poll_once)

        self.get_logger().info(
            f'web_bridge_node started; upstream={self.remote_host}:{self.remote_port}, '
            f'local_mirror=http://{self.local_host}:{self.local_port}, poll_hz={poll_hz}'
        )

    def _poll_once(self) -> None:
        for target in self.remote_targets:
            path = target["path"]
            url = target["url"]
            try:
                with urllib.request.urlopen(url, timeout=self.request_timeout) as res:
                    body = res.read()
                payload = json.loads(body.decode('utf-8', errors='ignore'))
                if not isinstance(payload, dict):
                    raise ValueError('simulation_data payload must be a JSON object')
                canonical = json.dumps(payload, ensure_ascii=True, separators=(",", ":"))
                sim_msg = String()
                sim_msg.data = canonical
                self.pub_simulation_data.publish(sim_msg)
                self._error_logged[path] = False
            except Exception as exc:
                if not self._error_logged[path]:
                    self.get_logger().warn(f'upstream fetch failed for {url}: {exc}')
                    self._error_logged[path] = True

    def _publish_sim_topics_from_payload(self, payload: dict) -> None:

        current_text = str(payload.get('current_position', '')).strip()
        visible_text = str(payload.get('visible_balls', '')).strip()
        waypoint_text = str(payload.get('waypoint_status', '')).strip()
        time_text = str(payload.get('time', '')).strip()

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

    def _on_simulation_data(self, msg: String) -> None:
        payload = self._parse_json_payload(msg.data)
        if payload is None:
            return
        canonical = json.dumps(payload, ensure_ascii=True, separators=(",", ":")).encode("utf-8")
        self._state.set('/data/simulation_data', canonical, 'application/json; charset=utf-8')
        self._publish_sim_topics_from_payload(payload)

    def _on_current_position_topic(self, msg: PoseStamped) -> None:
        qx = float(msg.pose.orientation.x)
        qy = float(msg.pose.orientation.y)
        qz = float(msg.pose.orientation.z)
        qw = float(msg.pose.orientation.w)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        payload = {
            "x": float(msg.pose.position.x),
            "y": float(msg.pose.position.y),
            "bearing": math.degrees(yaw_rad),
        }
        canonical = json.dumps(payload, ensure_ascii=True, separators=(",", ":")).encode("utf-8")
        self._state.set('/data/current_position', canonical, 'application/json; charset=utf-8')

    def _on_visible_balls_topic(self, msg: String) -> None:
        payload = {"visible_balls": msg.data if msg.data else ""}
        canonical = json.dumps(payload, ensure_ascii=True, separators=(",", ":")).encode("utf-8")
        self._state.set('/data/visible_balls', canonical, 'application/json; charset=utf-8')

    def _on_waypoint_status_topic(self, msg: String) -> None:
        payload = {"waypoint_status": msg.data if msg.data else ""}
        canonical = json.dumps(payload, ensure_ascii=True, separators=(",", ":")).encode("utf-8")
        self._state.set('/data/waypoint_status', canonical, 'application/json; charset=utf-8')

    def _on_time_topic(self, msg: String) -> None:
        payload = {"time": msg.data if msg.data else ""}
        canonical = json.dumps(payload, ensure_ascii=True, separators=(",", ":")).encode("utf-8")
        self._state.set('/data/time', canonical, 'application/json; charset=utf-8')

    def _on_radar_sensor_topic(self, msg: String) -> None:
        parsed = _parse_radar_sensor_text(msg.data if msg.data else "")
        payload = parsed if parsed is not None else {}
        canonical = json.dumps(payload, ensure_ascii=True, separators=(",", ":")).encode("utf-8")
        self._state.set('/data/radar_sensor', canonical, 'application/json; charset=utf-8')

    def _on_front_camera_msg(self, msg: Image) -> None:
        try:
            image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return
        ok, encoded = cv2.imencode('.jpg', image)
        if not ok:
            return
        self._state.set('/data/front_camera', encoded.tobytes(), 'image/jpeg')

    def _on_processed_image_msg(self, msg: Image) -> None:
        try:
            image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return
        ok, encoded = cv2.imencode('.jpg', image)
        if not ok:
            return
        self._state.set('/data/processed_image', encoded.tobytes(), 'image/jpeg')

    def _on_decisions(self, msg: String) -> None:
        payload = self._parse_json_payload(msg.data)
        if payload is None:
            return
        self._last_decisions_payload = payload
        canonical = json.dumps(payload, ensure_ascii=True, separators=(",", ":")).encode("utf-8")
        self._state.set('/data/decisions', canonical, 'application/json; charset=utf-8')
        if not self._web_debug_enabled:
            return
        if not planner._post_decisions_data(payload):
            self.get_logger().warn('failed to post decisions payload')

    def _on_decision_making_data(self, msg: String) -> None:
        payload = self._parse_json_payload(msg.data)
        if payload is None:
            return
        self._last_decision_making_payload = payload
        canonical = json.dumps(payload, ensure_ascii=True, separators=(",", ":")).encode("utf-8")
        self._state.set('/data/decision_making_data', canonical, 'application/json; charset=utf-8')
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
