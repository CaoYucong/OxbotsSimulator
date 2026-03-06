#!/usr/bin/env python3

"""waypoints_cruise.py
Mode-dispatch script for generating/updating dynamic waypoints.

Design:
- MODE is chosen from the environment variable `MODE` or the first CLI arg.
- A dispatch table maps MODE -> handler function. Handlers must be
  idempotent and exit quickly (script is run frequently).

Current provided mode:
- "random": if `waypoint_status.txt` == 'reached', generate a new random
  (x,y,None) within configured bounds and atomically overwrite
  `dynamic_waypoints.txt` with the tuple.

Add new modes by adding a function and registering it in _MODE_HANDLERS.
"""

from __future__ import annotations

import argparse

import json

import io

import os

import random

import time

import sys

import math

import re

import urllib.request

from typing import Optional

THIS_DIR = os.path.dirname(__file__)

PROJECT_ROOT = os.path.abspath(os.path.join(THIS_DIR, ".."))

WHO_IS_DEV_JSON_FILE = os.path.join(PROJECT_ROOT, "config.json")

REAL_TIME_DIR = os.path.join(THIS_DIR, "real_time_data")

BASE_DIR = os.path.abspath(os.path.join(THIS_DIR, "..", "controllers", "supervisor_controller", "real_time_data"))

SUPERVISOR_DIR = os.path.dirname(BASE_DIR)

HTML_PORT_FILE = os.path.join(SUPERVISOR_DIR, "html_port.txt")

WAYPOINT_STATUS_FILE = os.path.join(BASE_DIR, "waypoint_status.txt")

BALL_POS_FILE = os.path.join(BASE_DIR, "ball_position.txt")

CURRENT_POSITION_FILE = os.path.join(BASE_DIR, "current_position.txt")

OBSTACLE_ROBOT_FILE = os.path.join(BASE_DIR, "obstacle_robot.txt")

TIME_FILE = os.path.join(BASE_DIR, "time.txt")

VISIBLE_BALLS_FILE = os.path.join(BASE_DIR, "visible_balls.txt")

MODE_FILE = os.path.join(THIS_DIR, "mode.txt")

WAYPOINTS_STACK_FILE = os.path.join(REAL_TIME_DIR, "waypoints_stack.txt")

RADAR_HISTORY_FILE = os.path.join(REAL_TIME_DIR, "radar_memory.txt")

WALL_ONLY_RADAR_MEMORY_FILE = os.path.join(REAL_TIME_DIR, "wall_only_radar_memory.txt")

ROBOT_ONLY_RADAR_MEMORY_FILE = os.path.join(REAL_TIME_DIR, "robot_only_radar_memory.txt")

COLLISION_STATUS_FILE = os.path.join(REAL_TIME_DIR, "collision_avoiding_status.txt")

COLLISION_COUNTER_FILE = os.path.join(REAL_TIME_DIR, "collision_counter.txt")

COLLISION_COUNTER_STATE_FILE = os.path.join(REAL_TIME_DIR, "collision_counter_state.txt")

TOTAL_CONTACT_TIME_FILE = os.path.join(REAL_TIME_DIR, "total_contact_time.txt")

DYNAMIC_WAYPOINTS_TYPE_FILE = os.path.join(REAL_TIME_DIR, "dynamic_waypoints_type.txt")

ROBOT_AROUND_FILE = os.path.join(REAL_TIME_DIR, "robot_around.txt")

LAST_BEST_VECTOR_FILE = os.path.join(REAL_TIME_DIR, "last_best_vector.txt")

SEEN_TILE_FILE = os.path.join(REAL_TIME_DIR, "see_tile.txt")

TILE_SEEN_TIME_FILE = os.path.join(REAL_TIME_DIR, "tile_seen_time.txt")

BALL_MEMORY_FILE = os.path.join(REAL_TIME_DIR, "ball_tile_memory.txt")

LAST_SECOND_FILE = os.path.join(REAL_TIME_DIR, "last_second.txt")

BALL_LIST_MEMORY_FILE = os.path.join(REAL_TIME_DIR, "ball_memory.txt")

UNSEEN_TILE_MEMORY_FILE = os.path.join(REAL_TIME_DIR, "unseen_tile_memory.txt")

LAST_SECOND_TILES_FILE = os.path.join(REAL_TIME_DIR, "last_second_tiles.txt")

UNSEEN_REGIONS_FILE = os.path.join(REAL_TIME_DIR, "unseen_regions.txt")

COLLISION_AVOIDING_CONFIG_FILE = os.path.join(THIS_DIR, "collision_avoiding.txt")

RADAR_MAX_RANGE = 0.8

VISIBLE_RANGE_METERS = 2.0

MAX_LINEAR_VELOCITY = 0.7

DEFAULT_LINEAR_VELOCITY = 0.3

DEFAULT_ANGULAR_VELOCITY = 90 # degrees per second

VIRTUAL_WALL = 1.1  # Virtual wall distance for collision avoiding (meters)

INTAKE_RANGE = 0.1  # Range within which the robot can reliably intake the ball (meters)

FIELD_OF_VIEW_DEGREES = 120.0


def _load_runtime_config() -> tuple[str, str]:
    branch = ""
    data_flow = "web"
    try:
        with open(WHO_IS_DEV_JSON_FILE, "r") as f:
            payload = json.loads(f.read().strip())
        if isinstance(payload, dict):
            branch = str(payload.get("develope_brancch", "")).strip().lower()
            flow_raw = str(payload.get("data_flow", payload.get("data flow", "web"))).strip().lower()
            if flow_raw in ("web", "file"):
                data_flow = flow_raw
    except Exception:
        pass
    return branch, data_flow


_DEVELOPE_BRANCCH, DATA_FLOW = _load_runtime_config()


def _read_file_text(path: str) -> str:
    try:
        with _REAL_OPEN(path, "r") as f:
            return f.read().strip()
    except Exception:
        return ""


def _write_file_text(path: str, content: str) -> bool:
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with _REAL_OPEN(path, "w") as f:
            f.write(content)
        return True
    except Exception:
        return False

def _load_html_port(path, default_port=5001):
    try:
        with open(path, "r") as f:
            raw = f.read().strip()
        port = int(raw)
        if 1 <= port <= 65535:
            return port
    except Exception:
        pass
    return default_port

FIELD_VIEWER_PORT = _load_html_port(HTML_PORT_FILE)

SIM_DATA_URL = f"http://localhost:{FIELD_VIEWER_PORT}/simulation_data"

SIM_DATA_URL_FALLBACK = f"http://localhost:{FIELD_VIEWER_PORT}/data/simulation_data"

SIM_DATA_TIMEOUT = 0.2

SIM_DATA_CACHE = {}

DECISIONS_URL = f"http://localhost:{FIELD_VIEWER_PORT}/decisions"

DECISIONS_URL_FALLBACK = f"http://localhost:{FIELD_VIEWER_PORT}/data/decisions"

DECISIONS_TIMEOUT = 0.2

DECISIONS_CACHE = {}

DECISIONS_LOCAL_CACHE = {
    "dynamic_waypoints": "",
    "speed": f"{DEFAULT_LINEAR_VELOCITY:.6f}",
}

DECISION_MAKING_DATA_URL_FALLBACK = f"http://localhost:{FIELD_VIEWER_PORT}/data/decision_making_data"

DECISION_MAKING_DATA_TIMEOUT = 0.2

DECISION_MAKING_DATA_CACHE = {}

DECISION_MAKING_DATA_LOCAL_CACHE = {}

WEB_ONLY_FILES = {
    WAYPOINT_STATUS_FILE,
    BALL_POS_FILE,
    CURRENT_POSITION_FILE,
    OBSTACLE_ROBOT_FILE,
    TIME_FILE,
    VISIBLE_BALLS_FILE,
}

DEFAULT_MODE = 'improved_nearest_v3_5'

TILE_SIZE = 0.1

TILE_HALF = TILE_SIZE / 2.0

GRID_COUNT = int(round(2.0 / TILE_SIZE))

FIELD_TILES = [
    [
        (
            round(-1.0 + TILE_HALF + col * TILE_SIZE, 3),
            round(1.0 - TILE_HALF - row * TILE_SIZE, 3),
        )
        for col in range(GRID_COUNT)
    ]
    for row in range(GRID_COUNT)
]

def _parse_sim_data_from_html(text: str) -> dict:
    start = text.find("{")
    end = text.rfind("}")
    if start < 0 or end < 0 or end <= start:
        return {}
    try:
        return json.loads(text[start:end + 1])
    except Exception:
        return {}

def _refresh_sim_data() -> None:
    global SIM_DATA_CACHE
    if DATA_FLOW == "file":
        payload: dict[str, str] = {}
        for path in WEB_ONLY_FILES:
            key = os.path.splitext(os.path.basename(path))[0].lower()
            payload[key] = _read_file_text(path)
        SIM_DATA_CACHE = payload
        return
    for url in (SIM_DATA_URL_FALLBACK, SIM_DATA_URL):
        try:
            with urllib.request.urlopen(url, timeout=SIM_DATA_TIMEOUT) as res:
                raw = res.read().decode("utf-8", errors="ignore")
            if url.endswith("/simulation_data"):
                data = _parse_sim_data_from_html(raw)
            else:
                data = json.loads(raw)
            if isinstance(data, dict):
                if url.endswith("/simulation_data") and not data:
                    continue
                SIM_DATA_CACHE = data
                return
        except Exception:
            continue

def _get_sim_value(key: str):
    if not SIM_DATA_CACHE:
        return None
    return SIM_DATA_CACHE.get(key)

def _refresh_decisions_data() -> None:
    global DECISIONS_CACHE
    if DATA_FLOW == "file":
        payload: dict[str, str] = {}
        for key in DECISIONS_LOCAL_CACHE.keys():
            payload[key] = _read_file_text(os.path.join(REAL_TIME_DIR, f"{key}.txt"))
        DECISIONS_CACHE = payload
        return
    for url in (DECISIONS_URL_FALLBACK, DECISIONS_URL):
        try:
            with urllib.request.urlopen(url, timeout=DECISIONS_TIMEOUT) as res:
                raw = res.read().decode("utf-8", errors="ignore")
            if url.endswith("/decisions"):
                data = _parse_sim_data_from_html(raw)
            else:
                data = json.loads(raw)
            if isinstance(data, dict):
                if url.endswith("/decisions") and not data:
                    continue
                DECISIONS_CACHE = data
                return
        except Exception:
            continue

def _get_decision_value(key: str):
    if not DECISIONS_CACHE:
        return None
    return DECISIONS_CACHE.get(key)

def _require_decision_value(key: str, source_path: str):
    value = _get_decision_value(key)
    if value is None:
        _refresh_decisions_data()
        value = _get_decision_value(key)
    if value is None:
        raise RuntimeError(
            f"Missing decisions data for '{key}' (required for {source_path})."
        )
    return value

def _post_decisions_data(payload: dict) -> bool:
    global DECISIONS_CACHE
    if DATA_FLOW == "file":
        ok = True
        for key, value in payload.items():
            path = os.path.join(REAL_TIME_DIR, f"{key}.txt")
            ok = _write_file_text(path, str(value) + "\n") and ok
        if ok:
            DECISIONS_CACHE = dict(payload)
        return ok
    try:
        body = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            DECISIONS_URL_FALLBACK,
            data=body,
            method="POST",
            headers={"Content-Type": "application/json", "Content-Length": str(len(body))},
        )
        with urllib.request.urlopen(req, timeout=0.3) as res:
            res.read()
        DECISIONS_CACHE = payload
        return True
    except Exception:
        return False

def _refresh_decision_making_data() -> bool:
    global DECISION_MAKING_DATA_CACHE
    if DATA_FLOW == "file":
        payload: dict[str, str] = {}
        try:
            for name in os.listdir(REAL_TIME_DIR):
                if not name.endswith(".txt"):
                    continue
                key = os.path.splitext(name)[0].lower()
                payload[key] = _read_file_text(os.path.join(REAL_TIME_DIR, name))
        except Exception:
            payload = {}
        DECISION_MAKING_DATA_CACHE = payload
        return True
    try:
        with urllib.request.urlopen(
            DECISION_MAKING_DATA_URL_FALLBACK,
            timeout=DECISION_MAKING_DATA_TIMEOUT,
        ) as res:
            data = json.loads(res.read().decode("utf-8"))
        if isinstance(data, dict):
            DECISION_MAKING_DATA_CACHE = data
            return True
    except Exception:
        return False
    return False

def _get_decision_making_value(key: str):
    if DECISION_MAKING_DATA_CACHE:
        return DECISION_MAKING_DATA_CACHE.get(key)
    _refresh_decision_making_data()
    return DECISION_MAKING_DATA_CACHE.get(key)

def _post_decision_making_data(payload: dict) -> bool:
    global DECISION_MAKING_DATA_CACHE
    if DATA_FLOW == "file":
        ok = True
        for key, value in payload.items():
            path = os.path.join(REAL_TIME_DIR, f"{key}.txt")
            ok = _write_file_text(path, str(value) + "\n") and ok
        if ok:
            DECISION_MAKING_DATA_CACHE = dict(payload)
        return ok
    try:
        body = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            DECISION_MAKING_DATA_URL_FALLBACK,
            data=body,
            method="POST",
            headers={"Content-Type": "application/json", "Content-Length": str(len(body))},
        )
        with urllib.request.urlopen(req, timeout=0.3) as res:
            res.read()
        DECISION_MAKING_DATA_CACHE = payload
        return True
    except Exception:
        return False

def _update_decision_making_local(key: str, value: str) -> bool:
    DECISION_MAKING_DATA_LOCAL_CACHE[key] = value
    if not DECISION_MAKING_DATA_CACHE:
        _refresh_decision_making_data()
    payload = dict(DECISION_MAKING_DATA_CACHE) if DECISION_MAKING_DATA_CACHE else {}
    payload.update(DECISION_MAKING_DATA_LOCAL_CACHE)
    return _post_decision_making_data(payload)

def _bootstrap_stack_data() -> None:
    _refresh_decision_making_data()
    payload = dict(DECISION_MAKING_DATA_CACHE) if DECISION_MAKING_DATA_CACHE else {}
    existing = payload.get("waypoints_stack")
    if existing is None:
        payload["waypoints_stack"] = ""
        DECISION_MAKING_DATA_LOCAL_CACHE.update(payload)
        _post_decision_making_data(payload)

def _decision_key(path: str) -> str:
    return os.path.splitext(os.path.basename(path))[0].lower()

def _read_decision_text(path: str) -> str:
    if DATA_FLOW == "file":
        return _read_file_text(path)
    key = _decision_key(path)
    value = _get_decision_making_value(key)
    return "" if value is None else str(value)

def _write_decision_text(path: str, content: str) -> bool:
    if DATA_FLOW == "file":
        return _write_file_text(path, content)
    key = _decision_key(path)
    return _update_decision_making_local(key, content)

_REAL_OPEN = open

class _DecisionTextIO(io.StringIO):
    def __init__(self, path: str, mode: str, initial: str):
        super().__init__(initial)
        self._path = path
        self._mode = mode
        if "a" in mode:
            self.seek(0, io.SEEK_END)

    def close(self):
        if not self.closed and any(m in self._mode for m in ("w", "a", "+")):
            _write_decision_text(self._path, self.getvalue())
        super().close()

def _is_decision_path(path: str) -> bool:
    if not isinstance(path, str):
        return False
    abs_path = os.path.abspath(path)
    if abs_path.startswith(os.path.abspath(BASE_DIR)):
        return False
    return abs_path.startswith(os.path.abspath(THIS_DIR))

def open(path, mode="r", *args, **kwargs):
    if DATA_FLOW == "file":
        return _REAL_OPEN(path, mode, *args, **kwargs)
    if "b" in mode or not _is_decision_path(path):
        return _REAL_OPEN(path, mode, *args, **kwargs)
    if "r" in mode and "w" not in mode and "a" not in mode and "+" not in mode:
        return _DecisionTextIO(path, mode, _read_decision_text(path))
    if "a" in mode:
        return _DecisionTextIO(path, mode, _read_decision_text(path))
    return _DecisionTextIO(path, mode, "")

def _update_decisions_local(key: str, value: str) -> bool:
    DECISIONS_LOCAL_CACHE[key] = value
    return _post_decisions_data(dict(DECISIONS_LOCAL_CACHE))

def _require_sim_value(key: str, source_path: str):
    value = _get_sim_value(key)
    if value is None:
        _refresh_sim_data()
        value = _get_sim_value(key)
    if value is None:
        raise RuntimeError(
            f"Missing web sim data for '{key}' (required for {source_path})."
        )
    return value

def _read_status(path: str) -> Optional[str]:
    base = os.path.splitext(os.path.basename(path))[0].lower()
    if path in WEB_ONLY_FILES:
        return str(_require_sim_value(base, path)).strip()
    cached = _get_sim_value(base)
    if cached is not None:
        return str(cached).strip()
    text = _read_decision_text(path).strip()
    return text if text else None

def _read_mode(path: str = MODE_FILE) -> Optional[str]:
    mode = _read_status(path)
    if mode is None:
        return None
    mode = mode.strip().lower()
    return mode if mode else None

def _atomic_write(path: str, content: str) -> bool:
    return _write_decision_text(path, content)

def _read_collision_counter(path: str = COLLISION_COUNTER_FILE) -> tuple[int, list[float]]:
    count = 0
    times: list[float] = []
    for raw in _read_decision_text(path).splitlines():
        line = raw.strip()
        if not line:
            continue
        if line.startswith("count="):
            try:
                count = int(line.split("=", 1)[1].strip())
            except Exception:
                pass
        elif line.startswith("time="):
            try:
                times.append(float(line.split("=", 1)[1].strip()))
            except Exception:
                pass
    return count, times

def _read_dynamic_waypoints():
    """Read dynamic waypoints from file. Returns list of (x, y, orientation_or_none)."""
    raw = str(_require_decision_value("dynamic_waypoints", "dynamic_waypoints")).strip()
    if not raw:
        return None
    line = raw
    if line.startswith("(") and line.endswith(")"):
        line = line[1:-1]
    parts = [p.strip() for p in line.split(",")]
    if len(parts) < 2:
        return None
    try:
        x = float(parts[0])
        y = float(parts[1])
        bearing = float(parts[2]) if len(parts) >= 3 else None
        return (x, y, bearing)
    except Exception:
        return None

def _write_collision_counter(count: int,
                             times: list[float],
                             path: str = COLLISION_COUNTER_FILE) -> bool:
    content = f"count={int(count)}\n" + "".join(f"time={t:.3f}\n" for t in times)
    return _atomic_write(path, content)

def _read_collision_state(path: str = COLLISION_COUNTER_STATE_FILE) -> tuple[Optional[float], bool]:
    last_time: Optional[float] = None
    in_collision = False
    for raw in _read_decision_text(path).splitlines():
        line = raw.strip()
        if not line:
            continue
        if line.startswith("last_time="):
            try:
                last_time = float(line.split("=", 1)[1].strip())
            except Exception:
                last_time = None
        elif line.startswith("in_collision="):
            val = line.split("=", 1)[1].strip().lower()
            in_collision = val in ("1", "true", "yes", "on")
    return last_time, in_collision

def _write_collision_state(last_time: Optional[float],
                           in_collision: bool,
                           path: str = COLLISION_COUNTER_STATE_FILE) -> bool:
    last_time_str = "" if last_time is None else f"{last_time:.6f}"
    content = f"last_time={last_time_str}\nin_collision={1 if in_collision else 0}\n"
    return _atomic_write(path, content)

def _read_total_contact_time(path: str = TOTAL_CONTACT_TIME_FILE) -> float:
    raw = _read_decision_text(path).strip()
    if not raw:
        return 0.0
    try:
        return max(0.0, float(raw))
    except Exception:
        return 0.0

def _write_total_contact_time(total_seconds: float,
                              path: str = TOTAL_CONTACT_TIME_FILE) -> bool:
    safe_total = max(0.0, float(total_seconds))
    return _atomic_write(path, f"{safe_total:.6f}\n")

def _process_collision_counter_from_history(
    history_file: str = ROBOT_ONLY_RADAR_MEMORY_FILE,
    counter_file: str = COLLISION_COUNTER_FILE,
    state_file: str = COLLISION_COUNTER_STATE_FILE,
    total_contact_time_file: str = TOTAL_CONTACT_TIME_FILE,
    threshold: float = -0.01,
) -> None:
    """Count a collision only when distances recover above threshold after being below it."""
    entries: list[tuple[float, list[float]]] = []
    for raw in _read_decision_text(history_file).splitlines():
        line = raw.strip()
        if not line:
            continue
        parts = [p.strip() for p in line.split(",")]
        if len(parts) < 5:
            continue
        try:
            t = float(parts[0])
            dists = [float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])]
        except Exception:
            continue
        entries.append((t, dists))

    if not entries:
        return

    entries.sort(key=lambda item: item[0])
    last_time, in_collision = _read_collision_state(state_file)

    # First-time initialization: set baseline state, do not backfill old collisions.
    if last_time is None:
        latest_t, latest_dists = entries[-1]
        latest_in_collision = any(d < threshold for d in latest_dists)
        _write_total_contact_time(0.0, total_contact_time_file)
        _write_collision_state(latest_t, latest_in_collision, state_file)
        return

    new_entries = [(t, dists) for (t, dists) in entries if t > last_time + 1e-9]
    if not new_entries:
        return

    count, times = _read_collision_counter(counter_file)
    total_contact_time = _read_total_contact_time(total_contact_time_file)
    prev_time = last_time
    for t, dists in new_entries:
        dt = t - prev_time
        if in_collision and dt > 0.0:
            total_contact_time += dt
        now_in_collision = any(d < threshold for d in dists)
        if now_in_collision:
            in_collision = True
        elif in_collision:
            count += 1
            times.append(t)
            in_collision = False
        prev_time = t

    _write_collision_counter(count, times, counter_file)
    _write_total_contact_time(total_contact_time, total_contact_time_file)
    _write_collision_state(new_entries[-1][0], in_collision, state_file)

def _stack_current_waypoint(stack_file: str = WAYPOINTS_STACK_FILE,
                            current_file: Optional[str] = None) -> None:
    """Overwrite stack file with current dynamic waypoint content and timestamp.
    
    Format:
    Line 1: waypoint (x, y, orientation)
    Line 2: timestamp (seconds)
    """
    if _read_status(DYNAMIC_WAYPOINTS_TYPE_FILE) != "task":
        return
    if current_file is None:
        current = str(_require_decision_value("dynamic_waypoints", "dynamic_waypoints")).strip()
    else:
        current = _read_decision_text(current_file).strip()
        if not current:
            return

    if not current:
        return

    # Get current simulation time
    sim_time = _read_time_seconds(TIME_FILE)
    timestamp = f"{sim_time:.3f}" if sim_time is not None else "0.000"
    
    _atomic_write(stack_file, current + "\n" + timestamp + "\n")

def _write_collision_status(active: bool) -> None:
    status = "activated" if active else "inactive"
    _atomic_write(COLLISION_STATUS_FILE, f"{status}\n")

def _parse_ball_lines(text: str):
    out = []
    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            continue
        if line.startswith("(") and line.endswith(")"):
            line = line[1:-1]
        parts = [p.strip() for p in line.split(",")]
        if len(parts) < 2:
            continue
        try:
            x = float(parts[0])
            y = float(parts[1])
        except Exception:
            continue
        typ = parts[2] if len(parts) >= 3 else "ping"
        out.append((x, y, typ))
    return out

def _read_ball_memory_points(path: str) -> list[tuple[float, float]]:
    """Read remembered ball points from ball_memory.txt."""
    out = []
    for raw in _read_decision_text(path).splitlines():
        line = raw.strip()
        if not line:
            continue
        if line.startswith("(") and line.endswith(")"):
            line = line[1:-1]
        parts = [p.strip() for p in line.split(",")]
        if len(parts) < 2:
            continue
        try:
            x = float(parts[0])
            y = float(parts[1])
            out.append((x, y))
        except Exception:
            continue
    return out

def _write_ball_memory_points(path: str, points: list[tuple[float, float]]) -> bool:
    content = "\n".join(f"({x:.6f}, {y:.6f})" for x, y in points) + ("\n" if points else "")
    return _atomic_write(path, content)

def _read_visible_ball_positions(path: str):
    """Return list of (x,y,typ) from visible_balls.txt. Ignores invalid lines."""
    cached = _require_sim_value("visible_balls", path)
    return _parse_ball_lines(str(cached))

def _read_current_position(path: str):
    """Return (x, y, bearing_deg) from current_position.txt or None.
    
    Returns tuple of (x, y, bearing_deg) if bearing is present,
    or (x, y, None) if only coordinates are available.
    """
    cached = _require_sim_value("current_position", path)
    raw = str(cached).strip()
    if not raw:
        return None
    line = raw
    if line.startswith("(") and line.endswith(")"):
        line = line[1:-1]
    parts = [p.strip() for p in line.split(",")]
    if len(parts) < 2:
        return None
    try:
        x = float(parts[0])
        y = float(parts[1])
        bearing = float(parts[2]) if len(parts) >= 3 else None
        return (x, y, bearing)
    except Exception:
        return None

def _read_obstacle_positions(path: str):
    """Return list of (x, y, bearing_deg_or_none) from obstacle_robot.txt. Ignores invalid lines."""
    cached = _require_sim_value("obstacle_robot", path)
    return _parse_obstacle_lines(str(cached))

def _parse_obstacle_lines(text: str):
    out = []
    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            continue
        if line.startswith("(") and line.endswith(")"):
            line = line[1:-1]
        parts = [p.strip() for p in line.split(",")]
        if len(parts) < 2:
            continue
        try:
            x = float(parts[0])
            y = float(parts[1])
            bearing = float(parts[2]) if len(parts) >= 3 else None
            out.append((x, y, bearing))
        except Exception:
            continue
    return out

def _read_time_seconds(path: str) -> Optional[float]:
    """Return current simulation time in seconds from time.txt, or None."""
    cached = _require_sim_value("time", path)
    try:
        return float(str(cached).strip())
    except Exception:
        return None

def _read_wall_only_memory(
    path: str = WALL_ONLY_RADAR_MEMORY_FILE,
) -> list[tuple[float, dict[str, float]]]:
    """Read wall-only radar memory records.

    Format per line:
    time,front,right,left,rear
    Missing/invalid direction values are normalized to RADAR_MAX_RANGE.
    """
    entries: list[tuple[float, dict[str, float]]] = []
    for raw in _read_decision_text(path).splitlines():
        line = raw.strip()
        if not line:
            continue
        parts = [p.strip() for p in line.split(",")]
        if len(parts) < 5:
            continue
        try:
            t = float(parts[0])
        except Exception:
            continue
        values: dict[str, float] = {}
        keys = ("front", "right", "left", "rear")
        for i, key in enumerate(keys, start=1):
            try:
                value = float(parts[i])
                if not math.isfinite(value):
                    value = RADAR_MAX_RANGE
                values[key] = value
            except Exception:
                values[key] = RADAR_MAX_RANGE
        entries.append((t, values))
    return entries

def _write_wall_only_memory(
    entries: list[tuple[float, dict[str, float]]],
    path: str = WALL_ONLY_RADAR_MEMORY_FILE,
) -> bool:
    lines = []
    for t, values in entries:
        row = [f"{t:.3f}"]
        for key in ("front", "right", "left", "rear"):
            v = values.get(key, RADAR_MAX_RANGE)
            if not math.isfinite(v):
                v = RADAR_MAX_RANGE
            row.append(f"{v:.6f}")
        lines.append(",".join(row))
    content = "\n".join(lines) + ("\n" if lines else "")
    return _atomic_write(path, content)

def _read_robot_only_memory(
    path: str = ROBOT_ONLY_RADAR_MEMORY_FILE,
) -> list[tuple[float, dict[str, float]]]:
    """Read robot-only radar memory records.

    Format per line:
    time,front,right,left,rear
    Missing/invalid direction values are normalized to RADAR_MAX_RANGE.
    """
    entries: list[tuple[float, dict[str, float]]] = []
    for raw in _read_decision_text(path).splitlines():
        line = raw.strip()
        if not line:
            continue
        parts = [p.strip() for p in line.split(",")]
        if len(parts) < 5:
            continue
        try:
            t = float(parts[0])
        except Exception:
            continue
        values: dict[str, float] = {}
        keys = ("front", "right", "left", "rear")
        for i, key in enumerate(keys, start=1):
            try:
                value = float(parts[i])
                if not math.isfinite(value):
                    value = RADAR_MAX_RANGE
                values[key] = value
            except Exception:
                values[key] = RADAR_MAX_RANGE
        entries.append((t, values))
    return entries

def _write_robot_only_memory(
    entries: list[tuple[float, dict[str, float]]],
    path: str = ROBOT_ONLY_RADAR_MEMORY_FILE,
) -> bool:
    lines = []
    for t, values in entries:
        row = [f"{t:.3f}"]
        for key in ("front", "right", "left", "rear"):
            v = values.get(key, RADAR_MAX_RANGE)
            if not math.isfinite(v):
                v = RADAR_MAX_RANGE
            row.append(f"{v:.6f}")
        lines.append(",".join(row))
    content = "\n".join(lines) + ("\n" if lines else "")
    return _atomic_write(path, content)

def _read_collision_avoiding_config(
    path: str = COLLISION_AVOIDING_CONFIG_FILE,
) -> tuple[bool, Optional[float]]:
    """Read collision avoiding runtime config.

    Supported values in collision_avoiding.txt:
    - "off": disable collision avoiding.
    - "smart_factor = <number>": override smart_factor.

    Returns:
        (enabled, smart_factor_override)
    """
    text = _read_decision_text(path).strip()
    if not text:
        return True, None

    if text.lower() == "off":
        return False, None

    m = re.search(r"smart_factor\s*=\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", text, re.IGNORECASE)
    if m:
        try:
            return True, float(m.group(1))
        except Exception:
            return True, None

    return True, None

def _maybe_run_collision_avoiding(
    current_file: str = CURRENT_POSITION_FILE,
    default_smart_factor: float = 2.0,
    stack_waypoint: bool = True,
) -> bool:
    """Run collision avoiding based on collision_avoiding.txt config."""
    enabled, smart_factor_override = _read_collision_avoiding_config()
    if not enabled:
        radar_sensor()  # Still update radar memory for potential collision counting, even if avoiding is off.
        return False

    smart_factor = (
        smart_factor_override
        if smart_factor_override is not None
        else default_smart_factor
    )
    return collision_avoiding_v3(current_file, smart_factor=smart_factor, stack_waypoint=stack_waypoint)

def _read_stack_waypoint(path: Optional[str] = None) -> Optional[tuple[float, float, Optional[float]]]:
    """Return (x, y, orientation_or_none) from waypoints_stack.txt, or None.
    
    Format:
    Line 1: waypoint (x, y, orientation)
    Line 2: timestamp (seconds) - ignored by this function
    """
    try:
        if path is None:
            line = str(_require_decision_value("dynamic_waypoints", "dynamic_waypoints")).strip()
        else:
            lines = _read_decision_text(path).splitlines()
            if not lines:
                return None
            # Read first line only (waypoint data)
            line = lines[0].strip()
        if not line:
            return None
        if line.startswith("(") and line.endswith(")"):
            line = line[1:-1]
        parts = [p.strip() for p in line.split(",")]
        if len(parts) < 2:
            return None
        x = float(parts[0])
        y = float(parts[1])
        orientation = None
        if len(parts) >= 3 and parts[2] and parts[2].lower() != "none":
            orientation = float(parts[2])
        return (x, y, orientation)
    except Exception:
        return None

def _read_stack_timestamp(path: str) -> Optional[float]:
    """Return timestamp from line 2 of waypoints_stack.txt, or None."""
    lines = _read_decision_text(path).splitlines()
    if len(lines) >= 2:
        try:
            return float(lines[1].strip())
        except Exception:
            return None
    return None

def in_view(point,
            FOV: float = FIELD_OF_VIEW_DEGREES,
            Range: float = VISIBLE_RANGE_METERS,
            current_file: str = CURRENT_POSITION_FILE,
            obstacle_file: str = OBSTACLE_ROBOT_FILE) -> bool:
    """Return whether robot can see a world point, considering FOV/range/occlusion.

    - If point is outside field bounds [-1, 1] x [-1, 1], return False.
    - Visibility is constrained by robot pose, FOV (degrees), and Range (meters).
    - Obstacles are modeled as 0.2 x 0.2 squares centered at obstacle positions,
      rotated by each obstacle bearing (if missing, 0 deg).
    """

    try:
        px = float(point[0])
        py = float(point[1])
    except Exception:
        return False

    # Out of field bounds => invisible.
    if abs(px) > 1.0 or abs(py) > 1.0:
        return False

    cur = _read_current_position(current_file)
    if cur is None:
        return False
    cx, cy, bearing = cur
    if bearing is None:
        return False

    if Range <= 0.0 or FOV <= 0.0:
        return False

    dx = px - cx
    dy = py - cy
    dist = math.hypot(dx, dy)
    if dist > Range:
        return False
    if dist < 0.1:  # Too close to see clearly (inside robot radius).
        return False

    # FOV check around current bearing.
    target_angle = math.atan2(dy, dx)
    heading = math.radians(bearing)
    angle_diff = math.atan2(math.sin(target_angle - heading), math.cos(target_angle - heading))
    if abs(angle_diff) > math.radians(FOV) * 0.5:
        return False

    # Occlusion by obstacle robots (0.2 x 0.2 square).
    obstacles = _read_obstacle_positions(obstacle_file)
    if not obstacles:
        return True

    half = 0.1
    half_diag = math.sqrt(2.0) * half

    for ox, oy, obearing in obstacles:
        # Quick reject: obstacle too far beyond target to intersect line-of-sight.
        if math.hypot(ox - cx, oy - cy) > dist + half_diag:
            continue

        theta = math.radians(obearing) if obearing is not None else 0.0
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        local_corners = [(-half, -half), (half, -half), (half, half), (-half, half)]
        corners = []
        for lx, ly in local_corners:
            wx = ox + lx * cos_t - ly * sin_t
            wy = oy + lx * sin_t + ly * cos_t
            corners.append((wx, wy))

        # If line of sight intersects any edge of this square, point is occluded.
        for i in range(4):
            a = corners[i]
            b = corners[(i + 1) % 4]
            # if _segment_intersects(line_start, line_end, a, b):
                # return False

    return True

def radar_sensor(max_range: float = RADAR_MAX_RANGE, corridor: float = 0.2) -> list[tuple[str, float]]:
    """Return obstacle directions and distances in robot frame.

    Directions: "front", "right", "left", "rear" within +/- corridor/2
    lateral band, and within max_range. Returns [] if nothing.
    """
    cur = _read_current_position(CURRENT_POSITION_FILE)
    if cur is None:
        return []
    cx, cy, bearing = cur
    if bearing is None:
        return []

    half_band = corridor / 2.0
    theta = math.radians(bearing)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    hits = {}

    obstacle_half = 0.1
    robot_half = 0.1
    sample_points_world = []

    # 1) Collect obstacle edge samples in world frame.
    obstacles = _read_obstacle_positions(OBSTACLE_ROBOT_FILE)
    if obstacles:
        sample_spacing = 0.1 * obstacle_half  # 0.01 spacing between sample points
        obstacle_edge_samples_local = []

        num_samples = int(2 * obstacle_half / sample_spacing) + 1
        for i in range(num_samples):
            offset = -obstacle_half + i * sample_spacing
            obstacle_edge_samples_local.append((offset, obstacle_half))
            obstacle_edge_samples_local.append((offset, -obstacle_half))
            obstacle_edge_samples_local.append((-obstacle_half, offset))
            obstacle_edge_samples_local.append((obstacle_half, offset))

        for ox, oy, obearing in obstacles:
            otheta = math.radians(obearing) if obearing is not None else 0.0
            cos_o = math.cos(otheta)
            sin_o = math.sin(otheta)
            for lx, ly in obstacle_edge_samples_local:
                wx = ox + lx * cos_o - ly * sin_o
                wy = oy + lx * sin_o + ly * cos_o
                sample_points_world.append((wx, wy))

    # 2) Collect wall boundary samples in world frame.
    edge_samples = [i * 0.05 for i in range(-20, 21)]
    wall_samples = (
        [(x, 1.0) for x in edge_samples]
        + [(x, -1.0) for x in edge_samples]
        + [(1.0, y) for y in edge_samples]
        + [(-1.0, y) for y in edge_samples]
    )
    sample_points_world.extend(wall_samples)

    # 3) Single pass radar projection/classification for all sample points.
    for wx, wy in sample_points_world:
        dx = wx - cx
        dy = wy - cy
        x_robot = dx * cos_t + dy * sin_t
        y_robot = -dx * sin_t + dy * cos_t

        if x_robot > 0 and abs(y_robot) <= half_band and x_robot <= max_range:
            dist = x_robot - robot_half
            direction = "front"
        elif x_robot < 0 and abs(y_robot) <= half_band and -x_robot <= max_range:
            dist = -x_robot - robot_half
            direction = "rear"
        elif y_robot > 0 and abs(x_robot) <= half_band and y_robot <= max_range:
            dist = y_robot - robot_half
            direction = "left"
        elif y_robot < 0 and abs(x_robot) <= half_band and -y_robot <= max_range:
            dist = -y_robot - robot_half
            direction = "right"
        else:
            continue

        # dist = max(0.0, dist)
        if dist <= max_range:
            prev = hits.get(direction)
            if prev is None or dist < prev:
                hits[direction] = dist

    memory_values = {
        "front": RADAR_MAX_RANGE,
        "right": RADAR_MAX_RANGE,
        "left": RADAR_MAX_RANGE,
        "rear": RADAR_MAX_RANGE,
    }
    for direction, dist in hits.items():
        if direction in memory_values:
            memory_values[direction] = dist

    sim_time = _read_time_seconds(TIME_FILE)
    if sim_time is not None:
        history_lines = []
        cutoff_time = sim_time - 2
        for raw in _read_decision_text(RADAR_HISTORY_FILE).splitlines():
            line = raw.strip()
            if not line:
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) < 5:
                continue
            try:
                t = float(parts[0])
            except Exception:
                continue
            if cutoff_time <= t <= sim_time:
                history_lines.append(line)

        record = (
            f"{sim_time:.3f},{memory_values['front']:.6f},{memory_values['right']:.6f},"
            f"{memory_values['left']:.6f},{memory_values['rear']:.6f}"
        )
        history_lines.append(record)
        _atomic_write(RADAR_HISTORY_FILE, "\n".join(history_lines) + "\n")
        

    # print([(direction, dist) for direction, dist in hits.items()])
    wall_only_radar()
    robot_only_radar()
    _process_collision_counter_from_history()
    return [(direction, dist) for direction, dist in hits.items()]

def wall_only_radar(
    current_file: str = CURRENT_POSITION_FILE,
    memory_window_seconds: float = 2.0,
    memory_file: str = WALL_ONLY_RADAR_MEMORY_FILE,
) -> dict[str, float]:
    """Predict wall-only radar distances using the same method as radar_sensor.

    This function samples points on field walls and applies the same
    projection/classification logic as radar_sensor, but without obstacle points.

    Also keeps a short memory (default 2s) and fills missing directions from
    recent wall-only predictions.
    """
    cur = _read_current_position(current_file)
    if cur is None:
        return {}

    cx, cy, bearing = cur
    if bearing is None:
        return {}

    max_range = RADAR_MAX_RANGE
    corridor = 0.2
    half_band = corridor / 2.0

    theta = math.radians(bearing)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    robot_half = 0.1
    predicted: dict[str, float] = {}

    edge_samples = [i * 0.05 for i in range(-20, 21)]
    wall_samples = (
        [(x, 1.0) for x in edge_samples]
        + [(x, -1.0) for x in edge_samples]
        + [(1.0, y) for y in edge_samples]
        + [(-1.0, y) for y in edge_samples]
    )

    for wx, wy in wall_samples:
        dx = wx - cx
        dy = wy - cy
        x_robot = dx * cos_t + dy * sin_t
        y_robot = -dx * sin_t + dy * cos_t

        if x_robot > 0 and abs(y_robot) <= half_band and x_robot <= max_range:
            dist = x_robot - robot_half
            direction = "front"
        elif x_robot < 0 and abs(y_robot) <= half_band and -x_robot <= max_range:
            dist = -x_robot - robot_half
            direction = "rear"
        elif y_robot > 0 and abs(x_robot) <= half_band and y_robot <= max_range:
            dist = y_robot - robot_half
            direction = "left"
        elif y_robot < 0 and abs(x_robot) <= half_band and -y_robot <= max_range:
            dist = -y_robot - robot_half
            direction = "right"
        else:
            continue

        # dist = max(0.0, dist)
        if dist <= max_range:
            prev = predicted.get(direction)
            if prev is None or dist < prev:
                predicted[direction] = dist

    sim_time = _read_time_seconds(TIME_FILE)
    if sim_time is None:
        return predicted

    cutoff_time = sim_time - max(0.0, float(memory_window_seconds))
    entries = _read_wall_only_memory(memory_file)
    entries = [(t, vals) for (t, vals) in entries if cutoff_time <= t <= sim_time]

    merged = dict(predicted)
    for direction in ("front", "right", "left", "rear"):
        if direction in merged:
            continue
        for _, vals in reversed(entries):
            v = vals.get(direction, max_range)
            if math.isfinite(v) and 0.0 <= v <= max_range:
                merged[direction] = v
                break

    current_record = {
        "front": predicted.get("front", max_range),
        "right": predicted.get("right", max_range),
        "left": predicted.get("left", max_range),
        "rear": predicted.get("rear", max_range),
    }
    entries.append((sim_time, current_record))
    _write_wall_only_memory(entries, memory_file)

    # print(f"[waypoints_cruise] predicted wall radar distances: {predicted}", file=sys.stderr)
    return merged

def robot_only_radar(
    current_file: str = CURRENT_POSITION_FILE,
    obstacle_file: str = OBSTACLE_ROBOT_FILE,
    memory_window_seconds: float = 2.0,
    memory_file: str = ROBOT_ONLY_RADAR_MEMORY_FILE,
) -> dict[str, float]:
    """Predict robot-only radar distances using obstacle samples only.

    Uses the same projection/classification logic as `radar_sensor` and
    `wall_only_radar`, but excludes wall samples.

    Also keeps a short memory (default 2s) and fills missing directions from
    recent robot-only predictions.
    """
    cur = _read_current_position(current_file)
    if cur is None:
        return {}

    cx, cy, bearing = cur
    if bearing is None:
        return {}

    max_range = RADAR_MAX_RANGE
    corridor = 0.2
    half_band = corridor / 2.0

    theta = math.radians(bearing)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    robot_half = 0.1
    predicted: dict[str, float] = {}

    obstacle_half = 0.1
    sample_spacing = 0.1 * obstacle_half  # 0.01m
    sample_points_world: list[tuple[float, float]] = []

    obstacles = _read_obstacle_positions(obstacle_file)
    if not obstacles:
        return {}

    obstacle_edge_samples_local: list[tuple[float, float]] = []
    num_samples = int(2 * obstacle_half / sample_spacing) + 1
    for i in range(num_samples):
        offset = -obstacle_half + i * sample_spacing
        obstacle_edge_samples_local.append((offset, obstacle_half))
        obstacle_edge_samples_local.append((offset, -obstacle_half))
        obstacle_edge_samples_local.append((-obstacle_half, offset))
        obstacle_edge_samples_local.append((obstacle_half, offset))

    for ox, oy, obearing in obstacles:
        otheta = math.radians(obearing) if obearing is not None else 0.0
        cos_o = math.cos(otheta)
        sin_o = math.sin(otheta)
        for lx, ly in obstacle_edge_samples_local:
            wx = ox + lx * cos_o - ly * sin_o
            wy = oy + lx * sin_o + ly * cos_o
            sample_points_world.append((wx, wy))

    for wx, wy in sample_points_world:
        dx = wx - cx
        dy = wy - cy
        x_robot = dx * cos_t + dy * sin_t
        y_robot = -dx * sin_t + dy * cos_t

        if x_robot > 0 and abs(y_robot) <= half_band and x_robot <= max_range:
            dist = x_robot - robot_half
            direction = "front"
        elif x_robot < 0 and abs(y_robot) <= half_band and -x_robot <= max_range:
            dist = -x_robot - robot_half
            direction = "rear"
        elif y_robot > 0 and abs(x_robot) <= half_band and y_robot <= max_range:
            dist = y_robot - robot_half
            direction = "left"
        elif y_robot < 0 and abs(x_robot) <= half_band and -y_robot <= max_range:
            dist = -y_robot - robot_half
            direction = "right"
        else:
            continue

        if dist <= max_range:
            prev = predicted.get(direction)
            if prev is None or dist < prev:
                predicted[direction] = dist

    sim_time = _read_time_seconds(TIME_FILE)
    if sim_time is None:
        return predicted

    cutoff_time = sim_time - max(0.0, float(memory_window_seconds))
    entries = _read_robot_only_memory(memory_file)
    entries = [(t, vals) for (t, vals) in entries if cutoff_time <= t <= sim_time]

    merged = dict(predicted)
    for direction in ("front", "right", "left", "rear"):
        if direction in merged:
            continue
        for _, vals in reversed(entries):
            v = vals.get(direction, max_range)
            if math.isfinite(v) and 0.0 <= v <= max_range:
                merged[direction] = v
                break

    current_record = {
        "front": predicted.get("front", max_range),
        "right": predicted.get("right", max_range),
        "left": predicted.get("left", max_range),
        "rear": predicted.get("rear", max_range),
    }
    entries.append((sim_time, current_record))
    _write_robot_only_memory(entries, memory_file)

    return merged

def collision_activating_condition(current_file: str = CURRENT_POSITION_FILE) -> bool:
    cur = _read_current_position(current_file)
    if cur is None:
        return False
    _, _, _ = cur

    radar_hits = radar_sensor()
    if not radar_hits:
        return False

    predicted_wall_hits = wall_only_radar(current_file)
    collision_threshold = 0.05
    tolerance_ratio = 0.10

    filtered_hits: list[tuple[str, float]] = []
    for direction, dist in radar_hits:
        predicted = predicted_wall_hits.get(direction)

        # No wall prediction for this direction => keep this radar hit.
        if predicted is None or predicted > RADAR_MAX_RANGE:
            filtered_hits.append((direction, dist))
            continue

        # If radar hit is within 10% of wall-only prediction, treat it as wall
        # and exclude it from collision triggering.
        baseline = max(abs(predicted), 1e-9)
        if abs(dist - predicted) <= (tolerance_ratio * baseline):
            # print(f"[waypoints_cruise] ignoring radar hit in {direction} direction as it matches wall prediction (predicted: {predicted:.3f}, actual: {dist:.3f})", file=sys.stderr)
            continue

        filtered_hits.append((direction, dist))

    if any(dist < collision_threshold for _, dist in filtered_hits):
        return True

    return False

def collision_avoiding_v3(current_file: str = CURRENT_POSITION_FILE,
                          smart_factor: float = 4.0,
                          stack_waypoint: bool = True) -> bool:

    trigger_distance = 0.05
    cur = _read_current_position(current_file)
    if cur is None:
        _write_collision_status(False)
        set_velocity(DEFAULT_LINEAR_VELOCITY)
        return False
    cx, cy, bearing = cur

    collision_status = _read_status(COLLISION_STATUS_FILE)
    waypoint_status = _read_status(WAYPOINT_STATUS_FILE)
    abandon_time_threshold = 2.0  # seconds, if stacked waypoint is older than this, abandon it to avoid going to stale location
    if collision_status == "activated":
        if waypoint_status == "reached":
            _write_collision_status(False)
            set_velocity(DEFAULT_LINEAR_VELOCITY)
            stack_wp = _read_stack_waypoint(WAYPOINTS_STACK_FILE)
            if stack_wp is not None:
                # Check if stacked waypoint is too old
                stack_timestamp = _read_stack_timestamp(WAYPOINTS_STACK_FILE)
                current_time = _read_time_seconds(TIME_FILE)
                
                if stack_timestamp is not None and current_time is not None:
                    time_elapsed = current_time - stack_timestamp
                    if time_elapsed > abandon_time_threshold:
                        # Waypoint is too old, abandon it
                        _atomic_write(WAYPOINTS_STACK_FILE, "")
                        return True  # Don't goto anywhere, just return
                
                # Waypoint is still valid, proceed as normal
                _atomic_write(WAYPOINTS_STACK_FILE, "")
                x, y, orientation = stack_wp
                if orientation is None:
                    goto(x, y)
                else:
                    goto(x, y, orientation)
                return True
    if collision_status == "inactive":
        _atomic_write(LAST_BEST_VECTOR_FILE, "")
            
    radar_hits = radar_sensor()
    # radar_hits = predict_next_radar(tau=0.01)

    values = {"front": 0.8, "right": 0.8, "left": 0.8, "rear": 0.8}
    for direction, dist in radar_hits:
        if direction in values:
            values[direction] = min(0.8, dist)

    # Unit vectors every 30 degrees across 0-360.
    jump_step = 0.15
    unit_vectors_10deg = [
        (math.cos(math.radians(deg)), math.sin(math.radians(deg)))
        for deg in range(0, 360, 10)
    ]

    weights_rob_around = {
        key: (jump_step + 0.1) if val > jump_step + 0.1 else val
        for key, val in values.items()
    }
    weights_rob_around = {
        key: value - 0 for key, value in weights_rob_around.items()
    }

    weighted_vectors = []
    for ux, uy in unit_vectors_10deg:
        wx = (weights_rob_around["front"] * ux if ux >= 0 else weights_rob_around["rear"] * ux)
        wy = (weights_rob_around["left"] * uy if uy >= 0 else weights_rob_around["right"] * uy)
        weighted_vectors.append((wx, wy))

    _atomic_write(
        ROBOT_AROUND_FILE,
        "\n".join(f"({vx:.6f}, {vy:.6f})" for vx, vy in weighted_vectors) + "\n",
    )

    if collision_activating_condition():
        
        destination_vector = None
        dynamic_type = _read_status(DYNAMIC_WAYPOINTS_TYPE_FILE)
        if dynamic_type == "task":
            dynamic_wp = _read_stack_waypoint()
            if dynamic_wp is not None:
                destination_vector = (dynamic_wp[0] - cx, dynamic_wp[1] - cy)
        elif dynamic_type == "collision":
            stacked_wp = _read_stack_waypoint(WAYPOINTS_STACK_FILE)
            if stacked_wp is not None:
                destination_vector = (stacked_wp[0] - cx, stacked_wp[1] - cy)


        # dynamic_wp = _read_stack_waypoint()
        # if dynamic_wp is not None:
        #     destination_vector = (dynamic_wp[0] - cx, dynamic_wp[1] - cy)


        if destination_vector is None:
            destination_vector = (0.0, 0.0)
        dest_mag = math.hypot(destination_vector[0], destination_vector[1])
        destination_vector = (destination_vector[0] / dest_mag, destination_vector[1] / dest_mag) if dest_mag > 0 else (0.0, 0.0)

        if bearing is not None:
            theta = math.radians(bearing)
            cos_t = math.cos(theta)
            sin_t = math.sin(theta)
            weighted_vectors_world = [
                (vx * cos_t - vy * sin_t, vx * sin_t + vy * cos_t)
                for vx, vy in weighted_vectors
            ]

        min_mag = jump_step + 0.05
        filtered_vectors = [
            v for v in weighted_vectors_world if math.hypot(v[0], v[1]) >= min_mag
        ]
        if not filtered_vectors:
            filtered_vectors = weighted_vectors_world

        normals_robot = [
            (1.0, 0.0),   # front
            (0.0, -1.0),  # right
            (0.0, 1.0),   # left
            (-1.0, 0.0),  # rear
        ]
        theta = math.radians(bearing)
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        total_w = sum(values.values())
        if total_w <= 0.0:
            _write_collision_status(False)
            set_velocity(DEFAULT_LINEAR_VELOCITY)
            return False

        world_normals = []
        for (nx, ny) in normals_robot:
            wx = nx * cos_t - ny * sin_t
            wy = nx * sin_t + ny * cos_t
            world_normals.append((wx, wy))

        # Pairwise sums: front+left, left+rear, rear+right, right+front.
        pair_indices = [(0, 2), (2, 3), (3, 1), (1, 0)]
        safest_vec = None
        safest_mag = None
        ordered_values = [
            values["front"],
            values["right"],
            values["left"],
            values["rear"],
        ]
        for i, j in pair_indices:
            vx = ordered_values[i] * world_normals[i][0] + ordered_values[j] * world_normals[j][0]
            vy = ordered_values[i] * world_normals[i][1] + ordered_values[j] * world_normals[j][1]
            mag = math.hypot(vx, vy)
            if safest_mag is None or mag > safest_mag:
                safest_mag = mag
                safest_vec = (vx, vy)

        # Invert direction after selecting the strongest pairwise vector.
        safest_vec = (safest_vec[0]/safest_mag, safest_vec[1]/safest_mag)

        # Score each vector by its projection onto the destination vector.
        last_best_vec = _read_stack_waypoint(LAST_BEST_VECTOR_FILE)
        best_vec = None
        best_score = None
        
        # Calculate minimum radar distance for scoring weight
        min_radar_distance = max(0, min(values.values()))
        safety_factor = 8 + (min_radar_distance / trigger_distance) * 20 if trigger_distance > 0 else 10.0
        # safety_factor = 20
        # print(f"Safety factor: {safety_factor}, min radar distance: {min_radar_distance}, trigger distance: {trigger_distance}", file=sys.stderr)
        # print(f"Smart factor: {smart_factor}", file=sys.stderr)

        for vec in filtered_vectors:
            score = 0.0
            score += (vec[0] * safest_vec[0] + vec[1] * safest_vec[1]) * safety_factor
            # First time avoiding: only consider destination alignment to encourage moving towards the goal.
            if last_best_vec is None:
                if destination_vector is not None:
                    score += (vec[0] * destination_vector[0] + vec[1] * destination_vector[1]) * smart_factor
            #  Not the first time: consider last best direction to encourage stability, and also consider destination alignment but with lower weight to avoid oscillation.
            if last_best_vec is not None:
                score += (vec[0] * last_best_vec[0] + vec[1] * last_best_vec[1]) * smart_factor
                if destination_vector is not None:
                    score += (vec[0] * destination_vector[0] + vec[1] * destination_vector[1]) * smart_factor
            if best_score is None or score > best_score:
                best_score = score
                best_vec = vec
                # print(f"[waypoints_cruise] best vector: ({vec[0]:.3f}, {vec[1]:.3f}), score: {score:.6f}", file=sys.stderr)

        _atomic_write(
            LAST_BEST_VECTOR_FILE,
            f"({best_vec[0]:.6f}, {best_vec[1]:.6f})\n",
        )

        # Invert direction after selecting the strongest pairwise vector.
        best_mag = math.hypot(best_vec[0], best_vec[1])
        best_vec = (best_vec[0], best_vec[1])

        dx_world = jump_step * (best_vec[0] / best_mag)
        dy_world = jump_step * (best_vec[1] / best_mag)
        set_velocity(MAX_LINEAR_VELOCITY)
        _write_collision_status(True)
        if stack_waypoint:
            _stack_current_waypoint()
        # print(f"[waypoints_cruise] collision avoiding activated, radar values: {weights}, move vector: ({dx_world:.3f}, {dy_world:.3f})", file=sys.stderr)

        # Determine orientation based on waypoint hierarchy
        target_orientation = bearing  # Default to current bearing
        target_x = cx + dx_world
        target_y = cy + dy_world
        
        if min_radar_distance / trigger_distance > 0.7:
            # If obstacle is not too close, try to orient towards the task waypoint to encourage progress
          if dynamic_type == "task":
              # If current waypoint is task, point towards dynamic waypoint
              dynamic_waypoint = _read_stack_waypoint()
              if dynamic_waypoint is not None:
                  dx_to_goal = dynamic_waypoint[0] - target_x
                  dy_to_goal = dynamic_waypoint[1] - target_y
                  target_orientation = math.degrees(math.atan2(dy_to_goal, dx_to_goal))
          else:
              # If not task, check if stack waypoint exists and point towards it
              stack_waypoint = _read_stack_waypoint(WAYPOINTS_STACK_FILE)
              if stack_waypoint is not None:
                  if stack_waypoint[2] is not None:
                      target_orientation = stack_waypoint[2]
                  else:
                      dx_to_goal = stack_waypoint[0] - target_x
                      dy_to_goal = stack_waypoint[1] - target_y
                      target_orientation = math.degrees(math.atan2(dy_to_goal, dx_to_goal))

        if abs(cx + dx_world) < 0.9 and abs(cy + dy_world) < 0.9:
            goto(cx + dx_world, cy + dy_world, target_orientation, waypoint_type="collision")
        else:
            stop("collision")

        return True
        
    if collision_status == "activated":
        if waypoint_status == "going":
            return True

    return False

def next_point_time_cost(
    prev_position: tuple[float, float],
    prev_bearing_deg: Optional[float],
    next_position: tuple[float, float],
    next_bearing_deg: Optional[float],
    type: Optional[str] = None,
    debug: bool = False) -> float:

    distance = math.hypot(prev_position[0] - next_position[0], prev_position[1] - next_position[1])
    move_time = distance / DEFAULT_LINEAR_VELOCITY

    half_robot = 0.1
    intake_angle = abs(math.degrees(math.atan(INTAKE_RANGE / (2.0 * half_robot))))
    turning_angle = 0.0
    rotation_avoidance_buffer = 0.2 if type == "MEMORY" else 0.1

    if next_bearing_deg is None:
        next_bearing_deg = math.degrees(math.atan2(next_position[1] - prev_position[1], next_position[0] - prev_position[0]))

    next_x_in_prev = None
    next_y_in_prev = None
    if prev_bearing_deg is not None:
        dx = next_position[0] - prev_position[0]
        dy = next_position[1] - prev_position[1]

        # Transform next_position into prev_position reference frame.
        # In this project, bearing corresponds to robot -x axis, so shift by 180°
        # to obtain the expected robot-forward (+x) frame.
        theta = math.radians(prev_bearing_deg)
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        next_x_in_prev = dx * cos_t + dy * sin_t
        next_y_in_prev = -dx * sin_t + dy * cos_t

        # Compute turning angle by quadrant in prev frame:
        # - Q1/Q2: use angle from +x axis directly.
        # - Q3/Q4: mirror across x-axis first, then use angle from +x axis.
        if next_y_in_prev >= 0.0:
            angle_from_x_deg = math.degrees(math.atan2(next_y_in_prev, next_x_in_prev))
        else:
            mirrored_y = -next_y_in_prev
            angle_from_x_deg = math.degrees(math.atan2(mirrored_y, next_x_in_prev))

        turning_angle = max(0.0, angle_from_x_deg - intake_angle)
        turn_time = turning_angle / DEFAULT_ANGULAR_VELOCITY

    time_threashold = move_time - rotation_avoidance_buffer / DEFAULT_LINEAR_VELOCITY

    total_time = 0
    if turn_time < time_threashold:
        total_time = move_time
    else:
        total_time = move_time + turn_time - time_threashold
    
    if debug:
        print(
            f"------[next_point_time_cost]------\n"
            f"_single_cost from {prev_position} (bearing={prev_bearing_deg}) to {next_position} (bearing={next_bearing_deg}): \n"
            f"next_in_prev=({next_x_in_prev}, {next_y_in_prev}), angle_from_x={angle_from_x_deg:.1f}, turning_angle={turning_angle:.1f}, \n"
            f"distance={distance:.3f}, linear move time={move_time:.3f}, turning_angle={turning_angle:.1f}, turn_time={turn_time:.3f},\n"
            f"threashold = {time_threashold:.3f}s, total_cost={total_time:.3f}\n",
            f"\n",
        )
    return total_time

def _minimal_abs_angle_diff_deg(a_deg: float, b_deg: float) -> float:
    """Return minimal absolute angle difference in degrees, in [0, 180]."""
    diff = (a_deg - b_deg + 180.0) % 360.0 - 180.0
    return abs(diff)

def goto(x: float, y: float, orientation=None, waypoint_type: str = "task", rotation_avoidance_buffer: float = 0.1) -> bool:
    """Set the dynamic waypoint to the specified coordinates.
    
    Args:
        x: X coordinate
        y: Y coordinate
        orientation: Optional orientation angle in degrees (default None)
        waypoint_type: "task" or "collision" (default "task")
    
    Returns:
        True on success, False on failure
    """

    type_value = (waypoint_type or "task").strip()
    _atomic_write(DYNAMIC_WAYPOINTS_TYPE_FILE, f"{type_value}\n")

    cur = _read_current_position(CURRENT_POSITION_FILE)
    cx, cy, bearing = (0.0, 0.0, None) if cur is None else cur
    heading_deg = math.degrees(math.atan2(y - cy, x - cx))

    half_robot = 0.1

    x = max(-abs(VIRTUAL_WALL - half_robot), min(abs(VIRTUAL_WALL - half_robot), x))
    y = max(-abs(VIRTUAL_WALL - half_robot), min(abs(VIRTUAL_WALL - half_robot), y))

    final_deg = None
    intake_angle = abs(math.degrees(math.atan(INTAKE_RANGE / (2.0 * half_robot))))
    turning_angle = max(0.0, _minimal_abs_angle_diff_deg(heading_deg, bearing) - intake_angle) if bearing is not None else 0.0

    if orientation is None:
        distance_threashold = turning_angle / DEFAULT_ANGULAR_VELOCITY * DEFAULT_LINEAR_VELOCITY + rotation_avoidance_buffer
        if math.hypot(x - cx, y - cy) >= distance_threashold:
            final_deg = heading_deg
        else:
            final_deg = None
    else:
        final_deg = orientation

    if final_deg is not None:
        coord_line = f"({x:.6f}, {y:.6f}, {final_deg:.6f})\n"
    else:
        coord_line = f"({x:.6f}, {y:.6f}, None)\n"

    return _update_decisions_local("dynamic_waypoints", coord_line.strip())

def stop(waypoint_type: str = "task") -> bool:
    """Stop by setting dynamic waypoint to current robot position."""
    cur = _read_current_position(CURRENT_POSITION_FILE)
    if cur is None:
        return False
    x, y, bearing = cur
    if bearing is None:
        return goto(x, y, waypoint_type=waypoint_type)
    return goto(x, y, bearing, waypoint_type=waypoint_type)

def set_velocity(velocity: float) -> bool:
    """Set cruise speed (m/s) via decisions web data."""
    try:
        speed_value = float(velocity)
    except Exception:
        return False
    if speed_value <= 0:
        return False
    return _update_decisions_local("speed", f"{speed_value:.6f}")

def _read_seen_tile_matrix(path: str, rows: int, cols: int) -> list[list[float]]:
    """Read seen tile matrix from text file; return zero matrix on failure/missing data."""
    matrix = [[0.0 for _ in range(cols)] for _ in range(rows)]
    lines = _read_decision_text(path).splitlines()
    if not lines:
        return matrix

    for r in range(min(rows, len(lines))):
        nums = re.findall(r"[-+]?[0-9]*\.?[0-9]+", lines[r])
        for c in range(min(cols, len(nums))):
            try:
                matrix[r][c] = float(nums[c])
            except Exception:
                matrix[r][c] = 0.0
    return matrix

def _write_seen_tile_matrix(path: str, matrix: list[list[float]]) -> bool:
    def _fmt(v: float) -> str:
        return "0" if abs(v) < 1e-12 else f"{v:.3f}"

    content = "\n".join(
        ",".join(_fmt(v) for v in row)
        for row in matrix
    ) + "\n"
    return _atomic_write(path, content)

def update_seen_tiles(seen_tile_file: str = SEEN_TILE_FILE,
                      fov: float = FIELD_OF_VIEW_DEGREES,
                      view_range: float = VISIBLE_RANGE_METERS) -> bool:
    """Update seen_tile.txt based on FIELD_TILES and current visibility.

    Rules:
    - Visible & old value != 0: keep unchanged.
    - Visible & old value == 0: write current simulation time.
    - Not visible: write 0.
    """
    rows = len(FIELD_TILES)
    cols = len(FIELD_TILES[0]) if rows > 0 else 0
    if rows == 0 or cols == 0:
        return False

    seen = _read_seen_tile_matrix(seen_tile_file, rows, cols)
    sim_time = _read_time_seconds(TIME_FILE)
    current_time = sim_time if sim_time is not None else time.time()

    for r in range(rows):
        for c in range(cols):
            point = FIELD_TILES[r][c]
            visible = in_view(point, FOV=fov, Range=view_range)
            old_val = seen[r][c]

            if visible:
                if abs(old_val) < 1e-12:
                    seen[r][c] = current_time
            else:
                seen[r][c] = 0.0

    seen_ok = _write_seen_tile_matrix(seen_tile_file, seen)

    tile_seen_time = [
        [
            0.0 if abs(v) < 1e-12 else max(0.0, current_time - v)
            for v in row
        ]
        for row in seen
    ]
    time_ok = _write_seen_tile_matrix(TILE_SEEN_TIME_FILE, tile_seen_time)

    return seen_ok and time_ok

def update_unseen_tiles(unseen_tile_file: str = UNSEEN_TILE_MEMORY_FILE,
                        seen_tile_file: str = SEEN_TILE_FILE,
                        last_second_tiles_file: str = LAST_SECOND_TILES_FILE) -> bool:
    """Update unseen_tile_memory matrix with per-second accumulation.

        Rules:
        - If current integer second differs from last_second_tiles.txt, write current
            second and add +1 to every unseen tile memory cell.
        - Then read see_tile.txt and reset unseen tile memory cell to 0 only when
            a tile has remained visible for at least 2.0 seconds.
    """
    rows = len(FIELD_TILES)
    cols = len(FIELD_TILES[0]) if rows > 0 else 0
    if rows == 0 or cols == 0:
        return False

    unseen = _read_seen_tile_matrix(unseen_tile_file, rows, cols)
    seen = _read_seen_tile_matrix(seen_tile_file, rows, cols)

    sim_time = _read_time_seconds(TIME_FILE)
    current_time = sim_time if sim_time is not None else time.time()
    current_second = int(current_time)
    last_second_raw = _read_status(last_second_tiles_file)
    last_second = None
    if last_second_raw is not None:
        try:
            last_second = int(float(last_second_raw))
        except Exception:
            last_second = None

    second_ok = True
    if last_second != current_second:
        second_ok = _atomic_write(last_second_tiles_file, f"{current_second}\n")
        for r in range(rows):
            for c in range(cols):
                unseen[r][c] += 1.0

    radar_hits = radar_sensor()
    radar_front = RADAR_MAX_RANGE
    # print(f"[update_ball_memory_v3] radar_hits={radar_hits}", file=sys.stderr)
    if radar_hits:
        for direction, dist in radar_hits:
            if direction == "front":
                radar_front = dist
                break
    radar_front = max(0.0, min(RADAR_MAX_RANGE, radar_front))

    cur = _read_current_position(CURRENT_POSITION_FILE)
    cx, cy, _ = (0.0, 0.0, None) if cur is None else cur

    for r in range(rows):
        for c in range(cols):
            seen_since = seen[r][c]
            distance = math.hypot(FIELD_TILES[r][c][0] - cx, FIELD_TILES[r][c][1] - cy)
            if seen_since > 0.0 and (current_time - seen_since) >= 0.5 and distance <= radar_front + 0.1:
                unseen[r][c] = 0.0

    unseen_ok = _write_seen_tile_matrix(unseen_tile_file, unseen)
    return second_ok and unseen_ok

def update_unseen_regions(unseen_tile_file: str = UNSEEN_TILE_MEMORY_FILE,
                          unseen_regions_file: str = UNSEEN_REGIONS_FILE) -> bool:
    """Compute 0.4m-square local average unseen values for each tile center.

    For each tile center, use a square window of side length 0.4m centered at
    that tile. With 0.1m grid spacing this is a 5x5 index window (radius=2).
    Out-of-field samples contribute 0 to the sum and are excluded from the
    average denominator (average by actually existing points).
    """
    rows = len(FIELD_TILES)
    cols = len(FIELD_TILES[0]) if rows > 0 else 0
    if rows == 0 or cols == 0:
        return False

    unseen = _read_seen_tile_matrix(unseen_tile_file, rows, cols)
    regions = [[0.0 for _ in range(cols)] for _ in range(rows)]

    radius = int(round(0.2 / TILE_SIZE))
    for r in range(rows):
        for c in range(cols):
            total = 0.0
            count = 0
            for rr in range(r - radius, r + radius + 1):
                for cc in range(c - radius, c + radius + 1):
                    if 0 <= rr < rows and 0 <= cc < cols:
                        total += unseen[rr][cc]
                        count += 1
                    else:
                        total += 0.0

            regions[r][c] = (total / count) if count > 0 else 0.0

    return _write_seen_tile_matrix(unseen_regions_file, regions)

def goto_unseen_region(cx: float,
                      cy: float,
                      unseen_tile_file: str = UNSEEN_REGIONS_FILE) -> bool:
    """Goto the max unseen tile (nearest on ties). Return True if dispatched.

    If all unseen tiles are 0, return False.
    """
    rows = len(FIELD_TILES)
    cols = len(FIELD_TILES[0]) if rows > 0 else 0
    if rows == 0 or cols == 0:
        return False

    memory_unseen = _read_seen_tile_matrix(unseen_tile_file, rows, cols)
    best_unseen_rc = None
    best_unseen_val = -1.0
    best_unseen_d2 = None

    for r in range(rows):
        for c in range(cols):
            v = memory_unseen[r][c]
            tx, ty = FIELD_TILES[r][c]
            dx = tx - cx
            dy = ty - cy
            d2 = dx * dx + dy * dy

            if v > best_unseen_val:
                best_unseen_val = v
                best_unseen_rc = (r, c)
                best_unseen_d2 = d2
            elif abs(v - best_unseen_val) < 1e-9:
                if best_unseen_d2 is None or d2 < best_unseen_d2:
                    best_unseen_rc = (r, c)
                    best_unseen_d2 = d2

    if best_unseen_rc is None or best_unseen_val <= 0.0:
        return False

    tr, tc = best_unseen_rc
    tx, ty = FIELD_TILES[tr][tc]
    dx = tx - tx / math.hypot(ty, tx) * 0.4
    dy = ty - ty / math.hypot(ty, tx) * 0.4
    heading_deg = math.degrees(math.atan2(ty, tx))
    goto(dx, dy, heading_deg)
    # print(f"[waypoints_cruise] goto unseen region at tile ({tr}, {tc}, {heading_deg})", file=sys.stderr)
    return True

def tile_completely_seen(tx, ty, FOV=FIELD_OF_VIEW_DEGREES, Range=VISIBLE_RANGE_METERS) -> bool:
    """Check if tile center is completely seen by current radar."""
    corners = [
        (tx - TILE_HALF, ty - TILE_HALF),
        (tx - TILE_HALF, ty + TILE_HALF),
        (tx + TILE_HALF, ty - TILE_HALF),
        (tx + TILE_HALF, ty + TILE_HALF),
    ]
    return all(in_view(corner, FOV=FOV, Range=Range) for corner in corners)

def update_ball_memory_v2(memory_tile_file: str = BALL_MEMORY_FILE,
                       visible_balls_file: str = VISIBLE_BALLS_FILE,
                       last_second_file: str = LAST_SECOND_FILE,
                       ball_memory_file: str = BALL_LIST_MEMORY_FILE) -> bool:
    """Update ball_tile_memory matrix and maintain deduplicated ball_memory.txt.

    For each NEW visible ball (distance > 0.05 to all remembered points):
    - nearest tile point: +5
    - tiles within 0.1m: +3
    - tiles within 0.2m: +2
    """
    rows = len(FIELD_TILES)
    cols = len(FIELD_TILES[0]) if rows > 0 else 0
    if rows == 0 or cols == 0:
        return False

    memory = _read_seen_tile_matrix(memory_tile_file, rows, cols)

    sim_time = _read_time_seconds(TIME_FILE)
    current_second = int(sim_time if sim_time is not None else time.time())
    last_second_raw = _read_status(last_second_file)
    last_second = None
    if last_second_raw is not None:
        try:
            last_second = int(float(last_second_raw))
        except Exception:
            last_second = None

    if last_second != current_second:
        _atomic_write(last_second_file, f"{current_second}\n")
        for r in range(rows):
            for c in range(cols):
                memory[r][c] = max(0.0, memory[r][c] - 1.0)

    visible_balls = _read_visible_ball_positions(visible_balls_file)
    remembered_points = _read_ball_memory_points(ball_memory_file)
    new_visible_balls = []

    for bx, by, _ in visible_balls:
        is_new = True
        if is_new:
            remembered_points.append((bx, by))
            new_visible_balls.append((bx, by))

    points_ok = _write_ball_memory_points(ball_memory_file, remembered_points)

    flat_tiles = []
    for r in range(rows):
        for c in range(cols):
            tx, ty = FIELD_TILES[r][c]
            flat_tiles.append((r, c, tx, ty))

    cur = _read_current_position(CURRENT_POSITION_FILE)
    if cur is not None:
        cx, cy, bearing = cur
        theta = math.radians(bearing) if bearing is not None else 0.0
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        half = 0.2  # robot half-size (actually 0.2m square)
        tile_half = TILE_HALF

        for r, c, tx, ty in flat_tiles:
            # Four corners of the tile
            corners = [
                (tx - tile_half, ty - tile_half),
                (tx - tile_half, ty + tile_half),
                (tx + tile_half, ty - tile_half),
                (tx + tile_half, ty + tile_half),
            ]
            all_in_robot = True
            for corner_x, corner_y in corners:
                dx = corner_x - cx
                dy = corner_y - cy
                x_robot = dx * cos_t + dy * sin_t
                y_robot = -dx * sin_t + dy * cos_t
                if not (abs(x_robot) <= half and abs(y_robot) <= half):
                    all_in_robot = False
                    break
            if all_in_robot:
                memory[r][c] = 0.0

        for r, c, tx, ty in flat_tiles:
            if not tile_completely_seen(tx, ty, FOV=FIELD_OF_VIEW_DEGREES, Range=VISIBLE_RANGE_METERS):
                continue

            has_ball_in_tile = False
            for bx, by, _ in visible_balls:
                if abs(bx - tx) <= tile_half and abs(by - ty) <= tile_half:
                    has_ball_in_tile = True
                    break

            if not has_ball_in_tile:
                memory[r][c] = 0.0

    for bx, by in new_visible_balls:
        nearest_rc = None
        nearest_d2 = None

        for r, c, tx, ty in flat_tiles:
            dx = tx - bx
            dy = ty - by
            d2 = dx * dx + dy * dy

            if nearest_d2 is None or d2 < nearest_d2:
                nearest_d2 = d2
                nearest_rc = (r, c)

        if nearest_rc is not None:
            nr, nc = nearest_rc
            memory[nr][nc] = 180

    tile_ok = _write_seen_tile_matrix(memory_tile_file, memory)
    return points_ok and tile_ok

def mode_improved_nearest_v3_5(status_file: str = WAYPOINT_STATUS_FILE,
                          waypoint_file: Optional[str] = None,
                          visible_balls_file: str = VISIBLE_BALLS_FILE,
                          current_file: str = CURRENT_POSITION_FILE) -> int:
    """Realistic nearest mode: choose nearest ball from visible_balls.txt only."""

    update_seen_tiles()
    update_unseen_tiles()
    update_unseen_regions()
    
    update_ball_memory_v2(visible_balls_file=visible_balls_file)

    if _maybe_run_collision_avoiding(current_file, default_smart_factor=2.0):
        return 0

    sim_time = _read_time_seconds(TIME_FILE)
    if sim_time is not None and sim_time > 170.0:
        goto(-0.9, 0.0, 180.0)
        return 0

    cur = _read_current_position(current_file)

    status = _read_status(status_file)

    if cur is None:
        return 0
    bx = _read_visible_ball_positions(visible_balls_file)
    cx, cy, bearing = cur
    if not bx:
        
        rows = len(FIELD_TILES)
        cols = len(FIELD_TILES[0]) if rows > 0 else 0
        memory_ball = _read_seen_tile_matrix(BALL_MEMORY_FILE, rows, cols)

        if status != "reached":
            dynamic_waypoint = _read_dynamic_waypoints()
            # print(f"[mode_improved_nearest_v2] no visible balls, status={status}, dynamic_waypoint={dynamic_waypoint}", file=sys.stderr)
            dx, dy, dbearing = dynamic_waypoint if dynamic_waypoint is not None else (None, None, None)
            # print(f"[mode_improved_nearest_v2] current position=({cx}, {cy}), dynamic_waypoint=({dx}, {dy})", file=sys.stderr)
            if dx is not None and dy is not None:
              for r in range(rows):
                  for c in range(cols):
                      tx, ty = FIELD_TILES[r][c]
                      if abs(tx - dx) <= TILE_HALF and abs(ty - dy) <= TILE_HALF:
                          if memory_ball[r][c] > 0.0:
                              return 0

        best_ball_rc = None
        best_ball_time_cost = 180.0
        best_ball_val = -1.0
        for r in range(rows):
            for c in range(cols):
                v = memory_ball[r][c]
                tx, ty = FIELD_TILES[r][c]
                dist = math.hypot(tx - cx, ty - cy)
                time_cost = next_point_time_cost((cx, cy), bearing, (tx, ty), None)
                if time_cost <  best_ball_time_cost and dist >= 0.15 and v > 0.0:
                    best_ball_time_cost = time_cost
                    best_ball_val = v
                    best_ball_rc = (r, c)

        # print(f"[mode_improved_nearest_v3] (-0.25, -0.55)best_ball_time_cost={next_point_time_cost((cx, cy), bearing, ((-0.25, -0.55)), None, debug = True)}")
        # print(f"[mode_improved_nearest_v3] (0.45, -0.55)best_ball_time_cost={next_point_time_cost((cx, cy), bearing, ((0.45, -0.55)), None, debug = True)}")
        # print("----Comparison done----")


        if best_ball_rc is not None and best_ball_val > 0.0:
            tr, tc = best_ball_rc
            tx, ty = FIELD_TILES[tr][tc]
            goto(tx, ty, rotation_avoidance_buffer=0.2)
            return 0

        if status != "reached":
            # stop()
            return 0
        
        if goto_unseen_region(cx, cy):
            return 0

    else:
        pass

    best = None
    best_d2 = None
    for (x, y, typ) in bx:
        d2 = next_point_time_cost((cx, cy), bearing, (x, y), None)
        if best_d2 is None or d2 < best_d2:
            best_d2 = d2
            best = (x, y)

    if best is None:
        return 0

    target_x, target_y = best
    heading_deg = math.degrees(math.atan2(target_y - cy, target_x - cx))
    ok = goto(target_x, target_y, heading_deg)
    return 0 if ok else 1

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("mode", nargs="?", help="Mode to run (overrides MODE env)")
    return p.parse_args()

def main() -> int:
    _refresh_sim_data()
    _bootstrap_stack_data()
    args = parse_args()
    # precedence: mode.txt -> CLI arg -> MODE env var -> DEFAULT_MODE
    mode = _read_mode(MODE_FILE) or args.mode or os.environ.get("MODE") or DEFAULT_MODE
    mode = mode.strip().lower()

    handler = _MODE_HANDLERS.get(mode)
    if handler is None:
        print(f"[waypoints_cruise] unknown mode: {mode}", file=sys.stderr)
        return 2

    return handler()


_MODE_HANDLERS = {
    "improved_nearest_v3_5": mode_improved_nearest_v3_5,
}


def _parse_visible_balls_from_topic(text: str) -> list[tuple[float, float, str]]:
    raw = (text or '').strip()
    if not raw:
        return []

    # Accept JSON list payloads first.
    try:
        data = json.loads(raw)
        if isinstance(data, list):
            out = []
            for item in data:
                if isinstance(item, dict):
                    x = float(item.get('x'))
                    y = float(item.get('y'))
                    typ = str(item.get('type', 'PING')).upper()
                    out.append((x, y, typ))
            return out
    except Exception:
        pass

    # Fallback format:
    # (x, y, PING)\n(x, y, METAL)
    return _parse_ball_lines(raw)


def decide_from_ros_state(
    current_x: float,
    current_y: float,
    current_theta: float,
    visible_balls_json: str,
    sim_time_seconds: float,
    waypoint_status: str = 'going',
    mode: str = 'mode_improved_nearest_v3_5',
    default_speed: float = DEFAULT_LINEAR_VELOCITY,
):
    """Lightweight ROS planner entrypoint used by decision_node.

    Consumes data from ROS topics and returns command payload:
    {'dynamic_waypoint': (x, y, theta_rad), 'speed': v}
    """
    _ = mode
    _ = sim_time_seconds

    balls = _parse_visible_balls_from_topic(visible_balls_json)
    status = (waypoint_status or '').strip().lower()

    # Keep current command while moving and no better target is visible.
    if not balls and status != 'reached':
        return None

    if not balls:
        # No balls and waypoint reached: keep heading and stay put.
        return {
            'dynamic_waypoint': (float(current_x), float(current_y), float(current_theta)),
            'speed': float(default_speed),
        }

    best = None
    best_cost = None
    cur_heading_deg = math.degrees(float(current_theta))
    for x, y, _typ in balls:
        cost = next_point_time_cost(
            (float(current_x), float(current_y)),
            cur_heading_deg,
            (float(x), float(y)),
            None,
        )
        if best_cost is None or cost < best_cost:
            best_cost = cost
            best = (float(x), float(y))

    if best is None:
        return None

    tx, ty = best
    theta = math.atan2(ty - float(current_y), tx - float(current_x))
    return {
        'dynamic_waypoint': (tx, ty, theta),
        'speed': float(default_speed),
    }

if __name__ == "__main__":
    sys.exit(main())
