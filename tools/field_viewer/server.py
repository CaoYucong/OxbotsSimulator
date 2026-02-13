#!/usr/bin/env python3
"""Lightweight HTTP server for field visualization (no external deps)."""

from __future__ import annotations

import json
import os
import re
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(ROOT_DIR, "..", ".."))
WHO_IS_DEV_FILE = os.path.join(PROJECT_ROOT, "who_is_developing.txt")

def _resolve_decision_making_dir() -> str:
    """Select decision_making folder based on who_is_developing.txt (cyc/wly)."""
    default_dir = os.path.join(PROJECT_ROOT, "decision_making")
    try:
        with open(WHO_IS_DEV_FILE, "r") as f:
            dev = f.read().strip().lower()
        if dev == "cyc":
            return os.path.join(PROJECT_ROOT, "decision_making_cyc")
        if dev == "wly":
            return os.path.join(PROJECT_ROOT, "decision_making_wly")
    except Exception:
        pass
    return default_dir

DECISION_MAKING_DIR = _resolve_decision_making_dir()

DATA_DIR = os.path.join(PROJECT_ROOT, "controllers", "supervisor_controller", "real_time_data")

INDEX_FILE = os.path.join(ROOT_DIR, "index.html")
CURRENT_FILE = os.path.join(DATA_DIR, "current_position.txt")
BALLS_FILE = os.path.join(DATA_DIR, "ball_position.txt")
VISIBLE_FILE = os.path.join(DATA_DIR, "visible_balls.txt")
OBSTACLES_FILE = os.path.join(DATA_DIR, "obstacle_robot.txt")
DYNAMIC_FILE = os.path.join(DATA_DIR, "dynamic_waypoints.txt")

STACK_FILE = os.path.join(DECISION_MAKING_DIR, "real_time_data", "waypoints_stack.txt")
ROBOT_AROUND_FILE = os.path.join(DECISION_MAKING_DIR, "real_time_data", "robot_around.txt")
RADAR_HISTORY_FILE = os.path.join(DECISION_MAKING_DIR, "real_time_data", "radar_memory.txt")
TILE_SEEN_TIME_FILE = os.path.join(DECISION_MAKING_DIR, "real_time_data", "tile_seen_time.txt")
BALL_TILE_MEMORY_FILE = os.path.join(DECISION_MAKING_DIR, "real_time_data", "ball_tile_memory.txt")


def _read_lines(path: str) -> list[str]:
    try:
        with open(path, "r") as f:
            return [line.strip() for line in f if line.strip()]
    except Exception:
        return []


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


def _get_current():
    lines = _read_lines(CURRENT_FILE)
    if not lines:
        return None
    return _parse_current(lines[0])


def _get_balls(path: str):
    out = []
    for line in _read_lines(path):
        item = _parse_xy_type(line)
        if item is None:
            continue
        x, y, typ = item
        out.append({"x": x, "y": y, "type": typ})
    return out


def _get_obstacles():
    out = []
    for line in _read_lines(OBSTACLES_FILE):
        item = _parse_xy_bearing(line)
        if item is None:
            continue
        x, y, bearing = item
        out.append({"x": x, "y": y, "bearing": bearing})
    return out


def _get_dynamic_waypoint():
    for line in _read_lines(DYNAMIC_FILE):
        item = _extract_xy_from_line(line)
        if item is None:
            continue
        x, y = item
        return {"x": x, "y": y}
    return None


def _get_stack_waypoint():
    lines = _read_lines(STACK_FILE)
    for line in reversed(lines):
        item = _extract_xy_from_line(line)
        if item is None:
            continue
        x, y = item
        return {"x": x, "y": y}
    return None


def _get_robot_around():
    out = []
    for line in _read_lines(ROBOT_AROUND_FILE):
        item = _extract_xy_from_line(line)
        if item is None:
            continue
        x, y = item
        out.append({"x": x, "y": y})
    return out


def _get_radar_history():
    out = []
    for line in _read_lines(RADAR_HISTORY_FILE):
        parts = [p.strip() for p in line.split(",")]
        if len(parts) < 5:
            continue
        try:
            t = float(parts[0])
            front = float(parts[1])
            right = float(parts[2])
            left = float(parts[3])
            rear = float(parts[4])
        except Exception:
            continue
        out.append({
            "t": t,
            "front": front,
            "right": right,
            "left": left,
            "rear": rear,
        })
    return out


def _get_tile_seen_time():
    """Return tile seen-time entries mapped to world coordinates.

    Matrix corners are mapped as:
    - top-left     => (-0.95,  0.95)
    - bottom-right => ( 0.95, -0.95)

    Works for any matrix size (e.g. 10x10 or 20x20).
    """
    lines = _read_lines(TILE_SEEN_TIME_FILE)
    if not lines:
        return []

    matrix = []
    for line in lines:
        nums = re.findall(r"[-+]?[0-9]*\.?[0-9]+", line)
        if not nums:
            continue
        row = []
        for n in nums:
            try:
                row.append(float(n))
            except Exception:
                row.append(0.0)
        matrix.append(row)

    if not matrix:
        return []

    rows = len(matrix)
    cols = min(len(r) for r in matrix if r) if any(matrix) else 0
    if rows <= 0 or cols <= 0:
        return []

    x_step = 1.9 / (cols - 1) if cols > 1 else 0.0
    y_step = 1.9 / (rows - 1) if rows > 1 else 0.0

    out = []
    for r in range(rows):
        for c in range(cols):
            try:
                v = float(matrix[r][c])
            except Exception:
                v = 0.0
            x = -0.95 + c * x_step
            y = 0.95 - r * y_step
            out.append({"x": x, "y": y, "value": v})
    return out


def _get_ball_tile_memory():
    """Return ball tile-memory entries mapped to world coordinates.

    Matrix corners are mapped as:
    - top-left     => (-0.95,  0.95)
    - bottom-right => ( 0.95, -0.95)

    Works for any matrix size (e.g. 10x10 or 20x20).
    """
    lines = _read_lines(BALL_TILE_MEMORY_FILE)
    if not lines:
        return []

    matrix = []
    for line in lines:
        nums = re.findall(r"[-+]?[0-9]*\.?[0-9]+", line)
        if not nums:
            continue
        row = []
        for n in nums:
            try:
                row.append(float(n))
            except Exception:
                row.append(0.0)
        matrix.append(row)

    if not matrix:
        return []

    rows = len(matrix)
    cols = min(len(r) for r in matrix if r) if any(matrix) else 0
    if rows <= 0 or cols <= 0:
        return []

    x_step = 1.9 / (cols - 1) if cols > 1 else 0.0
    y_step = 1.9 / (rows - 1) if rows > 1 else 0.0

    out = []
    for r in range(rows):
        for c in range(cols):
            try:
                v = float(matrix[r][c])
            except Exception:
                v = 0.0
            x = -0.95 + c * x_step
            y = 0.95 - r * y_step
            out.append({"x": x, "y": y, "value": v})
    return out


class Handler(BaseHTTPRequestHandler):
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

    def do_GET(self):
        path = urlparse(self.path).path
        if path in ("/", "/index.html"):
            try:
                with open(INDEX_FILE, "r") as f:
                    self._send_text(f.read(), 200, "text/html")
            except Exception:
                self._send_text("index.html not found", 404, "text/plain")
            return

        if path == "/data/current":
            self._send_json({"current": _get_current()})
            return
        if path == "/data/balls":
            self._send_json({"balls": _get_balls(BALLS_FILE)})
            return
        if path == "/data/visible":
            self._send_json({"visible": _get_balls(VISIBLE_FILE)})
            return
        if path == "/data/obstacles":
            self._send_json({"obstacles": _get_obstacles()})
            return
        if path == "/data/waypoints":
            self._send_json({"dynamic": _get_dynamic_waypoint(), "stack": _get_stack_waypoint()})
            return
        if path == "/data/robot-around":
            self._send_json({"vectors": _get_robot_around()})
            return
        if path == "/data/radar-history":
            self._send_json({"history": _get_radar_history()})
            return
        if path == "/data/tile-seen-time":
            self._send_json({"tiles": _get_tile_seen_time()})
            return
        if path == "/data/ball-tile-memory":
            self._send_json({"tiles": _get_ball_tile_memory()})
            return

        self._send_text("not found", 404, "text/plain")

    def log_message(self, format, *args):
        return


def main():
    port = int(os.environ.get("PORT", "5001"))
    HTTPServer.allow_reuse_address = True
    try:
        server = HTTPServer(("0.0.0.0", port), Handler)
    except OSError as e:
        if getattr(e, "errno", None) == 48:
            print(f"Field viewer already running on http://localhost:{port}")
            return
        raise
    print(f"Field viewer running on http://localhost:{port}")
    server.serve_forever()


if __name__ == "__main__":
    main()
