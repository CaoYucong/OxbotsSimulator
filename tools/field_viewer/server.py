#!/usr/bin/env python3
"""Lightweight HTTP server for field visualization (no external deps)."""

from __future__ import annotations

import base64
import json
from typing import Optional
import os
import re
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(ROOT_DIR, "..", ".."))

CONFIG_FILE = os.path.join(PROJECT_ROOT, "config.json")

DATA_DIR = os.path.join(PROJECT_ROOT, "controllers", "supervisor_controller", "real_time_data")

INDEX_FILE = os.path.join(ROOT_DIR, "index.html")
SIM_DATA_FILE = os.path.join(ROOT_DIR, "simulation_data.html")
DECISIONS_FILE = os.path.join(ROOT_DIR, "decisions.html")
DECISION_MAKING_DATA_FILE = os.path.join(ROOT_DIR, "decision_making_data.html")
RANDOM_SEED_FILE = os.path.join(PROJECT_ROOT, "controllers", "supervisor_controller", "random_seed.txt")
BALL_TAKEN_HISTORY_FILE = os.path.join(DATA_DIR, "ball_taken_history.txt")


SIM_DATA_DIR = os.path.join(PROJECT_ROOT, "controllers", "supervisor_controller", "real_time_data")
HTML_PORT_FILE = os.path.join(PROJECT_ROOT, "controllers", "supervisor_controller", "html_port.txt")


def _load_runtime_config() -> tuple[str, str, bool, str, float]:
    branch = ""
    data_flow = "web"
    run_on_pi = False
    pi_ip = "127.0.0.1"
    viewer_refresh_seconds = 0.1
    try:
        with open(CONFIG_FILE, "r") as f:
            payload = json.loads(f.read().strip())
        if isinstance(payload, dict):
            branch = str(payload.get("develop_branch", payload.get("develope_brancch", ""))).strip().lower()
            flow_raw = str(payload.get("data_flow", payload.get("data flow", "web"))).strip().lower()
            if flow_raw in ("web", "file"):
                data_flow = flow_raw
            run_on_pi_raw = payload.get("run_on_pi", False)
            if isinstance(run_on_pi_raw, bool):
                run_on_pi = run_on_pi_raw
            elif isinstance(run_on_pi_raw, (int, float)):
                run_on_pi = bool(run_on_pi_raw)
            else:
                run_on_pi = str(run_on_pi_raw).strip().lower() in ("1", "true", "yes", "y", "on")

            ip_raw = str(payload.get("pi_ip", "")).strip()
            if ip_raw:
                pi_ip = ip_raw

            refresh_raw = payload.get("viewer_refresh_seconds", 0.1)
            try:
                viewer_refresh_seconds = float(refresh_raw)
                if viewer_refresh_seconds <= 0:
                    viewer_refresh_seconds = 0.1
            except Exception:
                viewer_refresh_seconds = 0.1
    except Exception:
        pass
    return branch, data_flow, run_on_pi, pi_ip, viewer_refresh_seconds


def _load_html_port(path: str, default_port: int = 5001) -> int:
    try:
        with open(path, "r") as f:
            raw = f.read().strip()
        port = int(raw)
        if 1 <= port <= 65535:
            return port
    except Exception:
        pass
    return default_port


DEVELOP_BRANCH, DATA_FLOW, RUN_ON_PI, PI_IP, VIEWER_REFRESH_SECONDS = _load_runtime_config()
FIELD_VIEWER_PORT = _load_html_port(HTML_PORT_FILE)
REMOTE_HOST = PI_IP if RUN_ON_PI else "localhost"
DECISION_MAKING_DIR = os.path.join(PROJECT_ROOT, f"decision_making_{DEVELOP_BRANCH}") if DEVELOP_BRANCH else ""
DECISION_DATA_DIR = os.path.join(DECISION_MAKING_DIR, "real_time_data") if DECISION_MAKING_DIR else ""

SIM_DATA_CACHE = {}
SIM_DATA_SEQ = 0
DECISIONS_CACHE = {}
DECISIONS_SEQ = 0
DECISION_MAKING_DATA_CACHE = {}
DECISION_MAKING_DATA_SEQ = 0
DECISION_MAKING_DEFAULTS = {
    "waypoints_stack": "",
}

FRONT_CAMERA_IMAGE = b""
FRONT_CAMERA_MIME = "image/png"
FRONT_CAMERA_UPDATED = 0.0

REMOTE_PULL_INTERVAL_SECONDS = VIEWER_REFRESH_SECONDS
_LAST_DECISIONS_PULL_TS = 0.0
_LAST_DECISION_MAKING_PULL_TS = 0.0


def _fetch_json(url: str, timeout: float = 0.25) -> dict:
    try:
        with urllib.request.urlopen(url, timeout=timeout) as res:
            payload = json.loads(res.read().decode("utf-8"))
        if isinstance(payload, dict):
            return payload
    except Exception:
        pass
    return {}


def _maybe_pull_remote_decisions(force: bool = False) -> None:
    global _LAST_DECISIONS_PULL_TS
    if not RUN_ON_PI:
        return
    now = time.time()
    if not force and (now - _LAST_DECISIONS_PULL_TS) < REMOTE_PULL_INTERVAL_SECONDS:
        return
    _LAST_DECISIONS_PULL_TS = now
    remote = _fetch_json(f"http://{REMOTE_HOST}:{FIELD_VIEWER_PORT}/data/decisions")
    if remote:
        _set_decisions_cache(remote)


def _maybe_pull_remote_decision_making_data(force: bool = False) -> None:
    global _LAST_DECISION_MAKING_PULL_TS
    if not RUN_ON_PI:
        return
    now = time.time()
    if not force and (now - _LAST_DECISION_MAKING_PULL_TS) < REMOTE_PULL_INTERVAL_SECONDS:
        return
    _LAST_DECISION_MAKING_PULL_TS = now
    remote = _fetch_json(f"http://{REMOTE_HOST}:{FIELD_VIEWER_PORT}/data/decision_making_data")
    if remote:
        _set_decision_making_data_cache(remote)


def _read_text(path: str) -> str:
    try:
        with open(path, "r") as f:
            return f.read().strip()
    except Exception:
        return ""


def _read_first_line_number(path: str) -> str:
    try:
        with open(path, "r") as f:
            first = f.readline().strip()
    except Exception:
        return ""
    if not first:
        return ""
    m = re.search(r"[-+]?\d*\.?\d+", first)
    return m.group(0) if m else ""


def _read_last_line(path: str) -> str:
    try:
        with open(path, "r") as f:
            lines = [line.strip() for line in f if line.strip()]
    except Exception:
        return ""
    if not lines:
        return ""
    return lines[-1]


def _read_lines_from_text(text: str) -> list[str]:
    return [line.strip() for line in text.splitlines() if line.strip()]


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


def _get_decision_text(key: str) -> str:
    if DATA_FLOW == "file" and DECISION_DATA_DIR:
        return _read_text(os.path.join(DECISION_DATA_DIR, f"{key}.txt"))
    value = _get_decision_making_data_cached().get(key)
    return value if isinstance(value, str) else ""


def _get_decision_lines(key: str) -> list[str]:
    return _read_lines_from_text(_get_decision_text(key))


def _get_sim_text(key: str) -> str:
    if DATA_FLOW == "file":
        return _read_text(os.path.join(SIM_DATA_DIR, f"{key}.txt"))
    if not SIM_DATA_CACHE:
        return ""
    value = SIM_DATA_CACHE.get(key)
    return value if isinstance(value, str) else ""


def _get_sim_lines(key: str) -> list[str]:
    return _read_lines_from_text(_get_sim_text(key))


def _get_decisions_text(key: str) -> str:
    if DATA_FLOW == "file" and DECISION_DATA_DIR:
        return _read_text(os.path.join(DECISION_DATA_DIR, f"{key}.txt"))
    value = _get_decisions_data_cached().get(key)
    return value if isinstance(value, str) else ""


def _get_decisions_lines(key: str) -> list[str]:
    return _read_lines_from_text(_get_decisions_text(key))


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
    lines = _get_sim_lines("current_position")
    if not lines:
        return None
    return _parse_current(lines[0])


def _get_balls_from_lines(lines: list[str]):
    out = []
    for line in lines:
        item = _parse_xy_type(line)
        if item is None:
            continue
        x, y, typ = item
        out.append({"x": x, "y": y, "type": typ})
    return out


def _get_obstacles():
    out = []
    for line in _get_sim_lines("obstacle_robot"):
        item = _parse_xy_bearing(line)
        if item is None:
            continue
        x, y, bearing = item
        out.append({"x": x, "y": y, "bearing": bearing})
    return out


def _get_dynamic_waypoint():
    for line in _get_decisions_lines("dynamic_waypoints"):
        item = _extract_xy_from_line(line)
        if item is None:
            continue
        x, y = item
        return {"x": x, "y": y}
    return None


def _get_stack_waypoint():
    lines = _get_decision_lines("waypoints_stack")
    for line in reversed(lines):
        item = _extract_xy_from_line(line)
        if item is None:
            continue
        x, y = item
        return {"x": x, "y": y}
    return None


def _get_robot_around():
    out = []
    for line in _get_decision_lines("robot_around"):
        item = _extract_xy_from_line(line)
        if item is None:
            continue
        x, y = item
        out.append({"x": x, "y": y})
    return out


def _get_radar_history():
    out = []
    for line in _get_decision_lines("radar_memory"):
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
    """Map matrix cells to tile-center world coordinates over [-1, 1] x [-1, 1].

    For an N-column matrix, x centers are at:
    -1 + (1/N), -1 + 3*(1/N), ..., 1 - (1/N)
    Equivalent to x = -1 + tile_half + c * tile_size where tile_size = 2/N.
    Same for y (top to bottom).
    """
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


def _get_tile_seen_time():
    matrix = _read_numeric_matrix_from_text(_get_decision_text("tile_seen_time"))
    return _matrix_to_world_tiles(matrix)


def _get_ball_tile_memory():
    matrix = _read_numeric_matrix_from_text(_get_decision_text("ball_tile_memory"))
    return _matrix_to_world_tiles(matrix)


def _get_unseen_tile_memory():
    matrix = _read_numeric_matrix_from_text(_get_decision_text("unseen_tile_memory"))
    return _matrix_to_world_tiles(matrix)


def _get_unseen_regions():
    matrix = _read_numeric_matrix_from_text(_get_decision_text("unseen_regions"))
    return _matrix_to_world_tiles(matrix)


def _get_text_status():
    return {
        "waypoint_status": _get_sim_text("waypoint_status"),
        "mode": _get_decision_text("mode"),
        "collision_avoiding": _get_decision_text("collision_avoiding"),
        "simulation_time": _read_first_line_number_from_text(_get_sim_text("time")),
        "random_seed": _read_text(RANDOM_SEED_FILE),
        "collision_counter": _read_first_line_number_from_text(
            _get_decision_text("collision_counter")
        ),
        "last_ball_taken": _read_last_line(BALL_TAKEN_HISTORY_FILE),
    }


def _get_all_ball_path():
    path = []
    for line in _get_decision_lines("planned_waypoints"):
        item = _extract_xy_from_line(line)
        if item is None:
            continue
        x, y = item
        path.append({"x": x, "y": y})
    return {"enabled": len(path) > 0, "path": path}


def _get_simulation_data():
    blocked = {
        ".ds_store",
        "ball_position",
        "ball_taken_history",
        "obstacle_plan",
        "obstacle_robot",
        "sweep_active_quadrant",
        "supervisor_controller_status",
        "sweep_index_q3",
        "waypoints_history",
        "ball_taken_number"
    }
    sim_data = {}
    try:
        for name in sorted(os.listdir(SIM_DATA_DIR)):
            full_path = os.path.join(SIM_DATA_DIR, name)
            if not os.path.isfile(full_path):
                continue
            base = os.path.splitext(name)[0].lower()
            if base in blocked:
                continue
            if base == "time":
                value = _read_first_line_number(full_path)
            else:
                value = _read_text(full_path)
            sim_data[base] = value
    except Exception:
        sim_data = {}
    return sim_data


def _get_simulation_data_cached():
    return SIM_DATA_CACHE if SIM_DATA_CACHE else {}


def _set_simulation_cache(payload: dict):
    global SIM_DATA_CACHE, SIM_DATA_SEQ
    SIM_DATA_CACHE = payload
    SIM_DATA_SEQ += 1


def _get_decisions_data_cached():
    _maybe_pull_remote_decisions()
    if DECISIONS_CACHE:
        return DECISIONS_CACHE
    return _get_decisions_data()


def _set_decisions_cache(payload: dict):
    global DECISIONS_CACHE, DECISIONS_SEQ
    DECISIONS_CACHE = payload
    DECISIONS_SEQ += 1


def _get_decision_making_data_cached():
    _maybe_pull_remote_decision_making_data()
    if DECISION_MAKING_DATA_CACHE:
        return DECISION_MAKING_DATA_CACHE
    return _get_decision_making_data()


def _get_decisions_data() -> dict:
    _maybe_pull_remote_decisions(force=True)
    return DECISIONS_CACHE if DECISIONS_CACHE else {}


def _get_decision_making_data() -> dict:
    _maybe_pull_remote_decision_making_data(force=True)
    return DECISION_MAKING_DATA_CACHE if DECISION_MAKING_DATA_CACHE else {}


def _set_decision_making_data_cache(payload: dict):
    global DECISION_MAKING_DATA_CACHE, DECISION_MAKING_DATA_SEQ
    merged = dict(DECISION_MAKING_DATA_CACHE)
    merged.update(payload)
    for key, value in DECISION_MAKING_DEFAULTS.items():
        merged.setdefault(key, value)
    DECISION_MAKING_DATA_CACHE = merged
    DECISION_MAKING_DATA_SEQ += 1


def _set_front_camera_image(image_bytes: bytes, mime: Optional[str] = None):
    global FRONT_CAMERA_IMAGE, FRONT_CAMERA_MIME, FRONT_CAMERA_UPDATED
    if image_bytes:
        FRONT_CAMERA_IMAGE = image_bytes
        if mime:
            FRONT_CAMERA_MIME = mime
        FRONT_CAMERA_UPDATED = time.time()


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

    def _send_bytes(self, body: bytes, status=200, content_type="application/octet-stream"):
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

        if path == "/simulation_data":
            try:
                with open(SIM_DATA_FILE, "r") as f:
                    self._send_text(f.read(), 200, "text/html")
            except Exception:
                self._send_text("simulation_data.html not found", 404, "text/plain")
            return

        if path == "/decisions":
            try:
                with open(DECISIONS_FILE, "r") as f:
                    self._send_text(f.read(), 200, "text/html")
            except Exception:
                self._send_text("decisions.html not found", 404, "text/plain")
            return

        if path == "/decision_making_data":
            try:
                with open(DECISION_MAKING_DATA_FILE, "r") as f:
                    self._send_text(f.read(), 200, "text/html")
            except Exception:
                self._send_text("decision_making_data.html not found", 404, "text/plain")
            return

        if path == "/front_camera":
            page = """<!doctype html>
<html lang=\"en\">
<head>
    <meta charset=\"utf-8\" />
    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />
    <title>Front Camera</title>
    <style>
        body { font-family: Arial, sans-serif; background: #0f1115; color: #f3f4f6; margin: 0; }
        .wrap { padding: 16px; }
        img { max-width: 100%; height: auto; border: 1px solid #23262d; background: #151a21; }
        .meta { font-size: 12px; color: #9ca3af; margin-top: 8px; }
    </style>
</head>
<body>
    <div class=\"wrap\">
        <h2>Front Camera</h2>
        <img id=\"cam\" alt=\"front camera\" />
        <div class=\"meta\" id=\"meta\"></div>
    </div>
    <script>
        const img = document.getElementById('cam');
        const meta = document.getElementById('meta');
        function refresh() {
            const ts = Date.now();
            img.src = `/data/front_camera?t=${ts}`;
            meta.textContent = `Updated ${new Date(ts).toLocaleTimeString()}`;
        }
        refresh();
        setInterval(refresh, 200);
    </script>
</body>
</html>"""
            self._send_text(page, 200, "text/html")
            return

        if path == "/data/current":
            self._send_json({"current": _get_current()})
            return
        if path == "/data/balls":
            self._send_json({"balls": _get_balls_from_lines(_get_sim_lines("ball_position"))})
            return
        if path == "/data/visible":
            self._send_json({"visible": _get_balls_from_lines(_get_sim_lines("visible_balls"))})
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
        if path == "/data/unseen-tile-memory":
            self._send_json({"tiles": _get_unseen_tile_memory()})
            return
        if path == "/data/unseen-regions":
            self._send_json({"tiles": _get_unseen_regions()})
            return
        if path == "/data/text-status":
            self._send_json(_get_text_status())
            return
        if path == "/data/all-ball-path":
            self._send_json(_get_all_ball_path())
            return
        if path == "/data/simulation_data":
            self._send_json(_get_simulation_data_cached())
            return
        if path == "/data/decisions":
            self._send_json(_get_decisions_data_cached())
            return
        if path == "/data/decision_making_data":
            self._send_json(_get_decision_making_data_cached())
            return
        if path == "/data/front_camera":
            if FRONT_CAMERA_IMAGE:
                self._send_bytes(FRONT_CAMERA_IMAGE, 200, FRONT_CAMERA_MIME)
            else:
                self._send_text("no image", 404, "text/plain")
            return
        if path == "/data/simulation-stream":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.end_headers()

            last_seq = -1
            last_payload_text = ""
            while True:
                try:
                    if SIM_DATA_CACHE:
                        seq = SIM_DATA_SEQ
                        if seq != last_seq:
                            payload_text = json.dumps(SIM_DATA_CACHE)
                            self.wfile.write(f"data: {payload_text}\n\n".encode("utf-8"))
                            self.wfile.flush()
                            last_seq = seq
                    time.sleep(0.1)
                except (BrokenPipeError, ConnectionResetError):
                    break
                except Exception:
                    break
            return

        if path == "/data/decisions-stream":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.end_headers()

            last_seq = -1
            last_payload_text = ""
            while True:
                try:
                    _maybe_pull_remote_decisions()
                    if DECISIONS_CACHE:
                        seq = DECISIONS_SEQ
                        if seq != last_seq:
                            payload_text = json.dumps(DECISIONS_CACHE)
                            self.wfile.write(f"data: {payload_text}\n\n".encode("utf-8"))
                            self.wfile.flush()
                            last_seq = seq
                    else:
                        payload = _get_decisions_data()
                        payload_text = json.dumps(payload)
                        if payload_text != last_payload_text:
                            self.wfile.write(f"data: {payload_text}\n\n".encode("utf-8"))
                            self.wfile.flush()
                            last_payload_text = payload_text
                    time.sleep(0.1)
                except (BrokenPipeError, ConnectionResetError):
                    break
                except Exception:
                    break
            return

        if path == "/data/decision_making_data-stream":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.end_headers()

            last_seq = -1
            last_payload_text = ""
            while True:
                try:
                    _maybe_pull_remote_decision_making_data()
                    if DECISION_MAKING_DATA_CACHE:
                        seq = DECISION_MAKING_DATA_SEQ
                        if seq != last_seq:
                            payload_text = json.dumps(DECISION_MAKING_DATA_CACHE)
                            self.wfile.write(f"data: {payload_text}\n\n".encode("utf-8"))
                            self.wfile.flush()
                            last_seq = seq
                    else:
                        payload = _get_decision_making_data_cached()
                        payload_text = json.dumps(payload)
                        if payload_text != last_payload_text:
                            self.wfile.write(f"data: {payload_text}\n\n".encode("utf-8"))
                            self.wfile.flush()
                            last_payload_text = payload_text
                    time.sleep(0.1)
                except (BrokenPipeError, ConnectionResetError):
                    break
                except Exception:
                    break
            return

        self._send_text("not found", 404, "text/plain")

    def do_POST(self):
        path = urlparse(self.path).path
        if path not in (
            "/data/simulation_data",
            "/data/decisions",
            "/data/decision_making_data",
            "/front_camera",
        ):
            self._send_text("not found", 404, "text/plain")
            return

        try:
            length = int(self.headers.get("Content-Length", "0"))
        except Exception:
            length = 0
        if length <= 0:
            self._send_text("empty payload", 400, "text/plain")
            return

        try:
            raw = self.rfile.read(length)
        except Exception:
            self._send_text("invalid payload", 400, "text/plain")
            return

        if path == "/front_camera":
            content_type = self.headers.get("Content-Type", "application/octet-stream")
            if content_type.startswith("application/json"):
                try:
                    payload = json.loads(raw.decode("utf-8"))
                    if not isinstance(payload, dict):
                        raise ValueError("payload must be object")
                    data_uri = payload.get("image")
                    image_base64 = payload.get("image_base64")
                    mime = payload.get("mime")
                    if isinstance(data_uri, str) and data_uri.startswith("data:"):
                        header, b64 = data_uri.split(",", 1)
                        mime = header.split(";", 1)[0].split(":", 1)[1]
                        image_bytes = base64.b64decode(b64)
                        _set_front_camera_image(image_bytes, mime)
                    elif isinstance(image_base64, str):
                        image_bytes = base64.b64decode(image_base64)
                        _set_front_camera_image(image_bytes, mime)
                    else:
                        self._send_text("invalid image payload", 400, "text/plain")
                        return
                except Exception:
                    self._send_text("invalid json", 400, "text/plain")
                    return
            else:
                if not raw:
                    self._send_text("empty payload", 400, "text/plain")
                    return
                _set_front_camera_image(raw, content_type)
            self._send_text("ok", 200, "text/plain")
            return

        try:
            payload = json.loads(raw.decode("utf-8"))
            if not isinstance(payload, dict):
                raise ValueError("payload must be object")
        except Exception:
            self._send_text("invalid json", 400, "text/plain")
            return

        if path == "/data/decisions":
            _set_decisions_cache(payload)
        elif path == "/data/decision_making_data":
            _set_decision_making_data_cache(payload)
        else:
            _set_simulation_cache(payload)
        self._send_text("ok", 200, "text/plain")

    def log_message(self, format, *args):
        return


def main():
    port = int(os.environ.get("PORT", "5001"))
    ThreadingHTTPServer.allow_reuse_address = True
    try:
        server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    except OSError as e:
        if getattr(e, "errno", None) == 48:
            print(f"Field viewer already running on http://localhost:{port}")
            return
        raise
    print(f"Field viewer running on http://localhost:{port}")
    server.serve_forever()


if __name__ == "__main__":
    main()
