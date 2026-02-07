#!/usr/bin/env python3
"""Lightweight HTTP server for field visualization (no external deps)."""

from __future__ import annotations

import json
import os
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.abspath(
    os.path.join(ROOT_DIR, "..", "..", "controllers", "supervisor_controller", "real_time_data")
)

INDEX_FILE = os.path.join(ROOT_DIR, "index.html")
CURRENT_FILE = os.path.join(DATA_DIR, "current_position.txt")
BALLS_FILE = os.path.join(DATA_DIR, "ball_position.txt")
VISIBLE_FILE = os.path.join(DATA_DIR, "visible_balls.txt")
OBSTACLES_FILE = os.path.join(DATA_DIR, "obstacle_robot.txt")


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


def _parse_xy(line: str):
    parts = _parse_tuple(line)
    if len(parts) < 2:
        return None
    try:
        return float(parts[0]), float(parts[1])
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
        item = _parse_xy(line)
        if item is None:
            continue
        x, y = item
        out.append({"x": x, "y": y})
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

        self._send_text("not found", 404, "text/plain")

    def log_message(self, format, *args):
        return


def main():
    port = int(os.environ.get("PORT", "5000"))
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
