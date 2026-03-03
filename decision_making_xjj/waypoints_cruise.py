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
import os
import random
import io
import time
import sys
import math
import re
import urllib.request
from typing import Optional


# =============================================================================
# PATHS AND FILE LOCATIONS
# Shared file paths used for supervisor <-> decision-making data exchange.
# =============================================================================
THIS_DIR = os.path.dirname(__file__)
REAL_TIME_DIR = os.path.join(THIS_DIR, "real_time_data")
BASE_DIR = os.path.abspath(os.path.join(THIS_DIR, "..", "controllers", "supervisor_controller", "real_time_data"))
SUPERVISOR_DIR = os.path.dirname(BASE_DIR)
HTML_PORT_FILE = os.path.join(SUPERVISOR_DIR, "html_port.txt")

WAYPOINT_STATUS_FILE = os.path.join(BASE_DIR, "waypoint_status.txt")
DYNAMIC_WAYPOINTS_FILE = os.path.join(BASE_DIR, "dynamic_waypoints.txt")
BALL_POS_FILE = os.path.join(BASE_DIR, "ball_position.txt")
CURRENT_POSITION_FILE = os.path.join(BASE_DIR, "current_position.txt")
OBSTACLE_ROBOT_FILE = os.path.join(BASE_DIR, "obstacle_robot.txt")
TIME_FILE = os.path.join(BASE_DIR, "time.txt")
SPEED_FILE = os.path.join(BASE_DIR, "speed.txt")
VISIBLE_BALLS_FILE = os.path.join(BASE_DIR, "visible_balls.txt")

PLANNED_WAYPOINTS_FILE = os.path.join(THIS_DIR, "planned_waypoints.txt")
MODE_FILE = os.path.join(THIS_DIR, "mode.txt")

PLANNED_INDEX_FILE = os.path.join(REAL_TIME_DIR, "planned_waypoints_index.txt")
TEMP_STATE_FILE = os.path.join(REAL_TIME_DIR, "search_state.txt")
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
DECISION_MAKING_DATA_URL = f"http://localhost:{FIELD_VIEWER_PORT}/decision_making_data"
DECISION_MAKING_DATA_URL_FALLBACK = f"http://localhost:{FIELD_VIEWER_PORT}/data/decision_making_data"
DECISION_MAKING_DATA_TIMEOUT = 0.2
DECISION_MAKING_DATA_CACHE = {}
DECISIONS_CACHE = {}
DECISIONS_LOCAL_CACHE = {
    "dynamic_waypoints": "",
    "speed": "0.300000",
}
DECISION_MAKING_DATA_LOCAL_CACHE = {}
WEB_ONLY_FILES = {
    WAYPOINT_STATUS_FILE,
    BALL_POS_FILE,
    CURRENT_POSITION_FILE,
    OBSTACLE_ROBOT_FILE,
    TIME_FILE,
    VISIBLE_BALLS_FILE,
}
DECISION_MAKING_SEED_FILES = (
    MODE_FILE,
    PLANNED_WAYPOINTS_FILE,
    PLANNED_INDEX_FILE,
    TEMP_STATE_FILE,
    WAYPOINTS_STACK_FILE,
    RADAR_HISTORY_FILE,
    WALL_ONLY_RADAR_MEMORY_FILE,
    ROBOT_ONLY_RADAR_MEMORY_FILE,
    COLLISION_STATUS_FILE,
    COLLISION_COUNTER_FILE,
    COLLISION_COUNTER_STATE_FILE,
    TOTAL_CONTACT_TIME_FILE,
    DYNAMIC_WAYPOINTS_TYPE_FILE,
    ROBOT_AROUND_FILE,
    LAST_BEST_VECTOR_FILE,
    SEEN_TILE_FILE,
    TILE_SEEN_TIME_FILE,
    BALL_MEMORY_FILE,
    BALL_LIST_MEMORY_FILE,
    UNSEEN_TILE_MEMORY_FILE,
    LAST_SECOND_FILE,
    LAST_SECOND_TILES_FILE,
    UNSEEN_REGIONS_FILE,
    COLLISION_AVOIDING_CONFIG_FILE,
)

# =============================================================================
# GLOBAL CONSTANTS
# Runtime mode, geometric bounds, speed limits, and static grids/sequences.
# =============================================================================
# Set the default mode here for convenience. Edit this file and set
# `DEFAULT_MODE` to the mode you want the script to use when no CLI arg
# or `MODE` environment variable is provided. Example: 'random', 'nearest', 'realistic_nearest', 'planned' or 'developing'.
DEFAULT_MODE = 'improved_nearest'

# generation bounds (match supervisor playground bounds)
X_MIN, X_MAX = -0.86, 0.86
Y_MIN, Y_MAX = -0.86, 0.86
RADAR_MAX_RANGE = 0.8
MAX_SPEED = 0.7
NORMAL_SPEED = 0.3

SEARCHING_SEQUENCE = [
    (0, 0, 0),
    (0, 0, 90),
    (0, 0, 180),
    (0, 0, -90),
    (0.5, 0.5, 0),
    (0.5, 0.5, 90),
    (0.5, 0.5, 180),
    (0.5, 0.5, -90),
    (-0.5, 0.5, 0),
    (-0.5, 0.5, 90),
    (-0.5, 0.5, 180),
    (-0.5, 0.5, -90),
    (-0.5, -0.5, 0),
    (-0.5, -0.5, 90),
    (-0.5, -0.5, 180),
    (-0.5, -0.5, -90),
    (0.5, -0.5, 0),
    (0.5, -0.5, 90),
    (0.5, -0.5, 180),
    (0.5, -0.5, -90),
    (0, 0.5, 0),
    (0, 0.5, 90),
    (0, 0.5, 180),
    (0, 0.5, -90),
    (0.5, 0, 0),
    (0.5, 0, 90),
    (0.5, 0, 180),
    (0.5, 0, -90),
    (0, -0.5, 0),
    (0, -0.5, 90),
    (0, -0.5, 180),
    (0, -0.5, -90),
    (-0.5, 0, 0),
    (-0.5, 0, 90),
    (-0.5, 0, 180),
    (-0.5, 0, -90),
]

# Build 0.1m point grid over [-1, 1] x [-1, 1].
# Top-left is (-0.95, 0.95), bottom-right is (0.95, -0.95).
FIELD_TILES = [
    [
        (round(-0.95 + col * 0.1, 2), round(0.95 - row * 0.1, 2))
        for col in range(20)
    ]
    for row in range(20)
]


# =============================================================================
# CORE IO HELPERS
# Safe file reads/writes and lightweight state parsing utilities.
# =============================================================================

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
    return _post_decision_making_data(dict(DECISION_MAKING_DATA_LOCAL_CACHE))


def _decision_key(path: str) -> str:
    return os.path.splitext(os.path.basename(path))[0].lower()


def _read_decision_text(path: str) -> str:
    key = _decision_key(path)
    value = _get_decision_making_value(key)
    return "" if value is None else str(value)


def _write_decision_text(path: str, content: str) -> bool:
    key = _decision_key(path)
    return _update_decision_making_local(key, content)


_REAL_OPEN = open


def _read_local_text(path: str) -> str:
    try:
        with _REAL_OPEN(path, "r") as f:
            return f.read().strip()
    except Exception:
        return ""


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


def _bootstrap_decision_making_data() -> None:
    _refresh_decision_making_data()
    payload = dict(DECISION_MAKING_DATA_CACHE) if DECISION_MAKING_DATA_CACHE else {}
    changed = False
    for path in DECISION_MAKING_SEED_FILES:
        key = _decision_key(path)
        existing = payload.get(key)
        if isinstance(existing, str) and existing.strip():
            continue
        if existing is not None and not isinstance(existing, str):
            continue
        text = _read_local_text(path)
        if not text:
            continue
        payload[key] = text
        changed = True
    if changed:
        DECISION_MAKING_DATA_LOCAL_CACHE.update(payload)
        _post_decision_making_data(payload)


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
                            current_file: str = DYNAMIC_WAYPOINTS_FILE) -> None:
    """Overwrite stack file with current dynamic waypoint content and timestamp.
    
    Format:
    Line 1: waypoint (x, y, orientation)
    Line 2: timestamp (seconds)
    """
    if _read_status(DYNAMIC_WAYPOINTS_TYPE_FILE) != "task":
        return
    if current_file == DYNAMIC_WAYPOINTS_FILE:
        current = str(_require_decision_value("dynamic_waypoints", current_file)).strip()
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


def _read_ball_positions(path: str):
    """Return list of (x,y,typ) from ball_position.txt. Ignores invalid lines."""
    cached = _require_sim_value("ball_position", path)
    return _parse_ball_lines(str(cached))


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
    return collision_avoiding_v3(current_file, smart_factor=smart_factor)


def _read_state_pair(path: str) -> Optional[tuple[float, float]]:
    raw = _read_decision_text(path).strip()
    if not raw:
        _atomic_write(path, "(0, 0)\n")
        return (0.0, 0.0)
    line = raw
    if line.startswith("(") and line.endswith(")"):
        line = line[1:-1]
    nums = re.findall(r"[-+]?[0-9]*\.?[0-9]+", line)
    if len(nums) < 2:
        return None
    try:
        return (float(nums[0]), float(nums[1]))
    except Exception:
        return None


def _read_stack_waypoint(path: str) -> Optional[tuple[float, float, Optional[float]]]:
    """Return (x, y, orientation_or_none) from waypoints_stack.txt, or None.
    
    Format:
    Line 1: waypoint (x, y, orientation)
    Line 2: timestamp (seconds) - ignored by this function
    """
    try:
        if path == DYNAMIC_WAYPOINTS_FILE:
            line = str(_require_decision_value("dynamic_waypoints", path)).strip()
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


# =============================================================================
# GEOMETRY AND VISIBILITY
# Visibility checks in world/robot frames and line-of-sight occlusion.
# =============================================================================
def in_view(point,
            FOV: float = 60.0,
            Range: float = 0.8,
            current_file: str = CURRENT_POSITION_FILE,
            obstacle_file: str = OBSTACLE_ROBOT_FILE) -> bool:
    """Return whether robot can see a world point, considering FOV/range/occlusion.

    - If point is outside field bounds [-1, 1] x [-1, 1], return False.
    - Visibility is constrained by robot pose, FOV (degrees), and Range (meters).
    - Obstacles are modeled as 0.2 x 0.2 squares centered at obstacle positions,
      rotated by each obstacle bearing (if missing, 0 deg).
    """

    def _cross(ax: float, ay: float, bx: float, by: float) -> float:
        return ax * by - ay * bx

    def _segment_intersects(p1, p2, q1, q2, eps: float = 1e-9) -> bool:
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = q1
        x4, y4 = q2

        d1x, d1y = (x2 - x1), (y2 - y1)
        d2x, d2y = (x4 - x3), (y4 - y3)
        denom = _cross(d1x, d1y, d2x, d2y)
        qpx, qpy = (x3 - x1), (y3 - y1)

        if abs(denom) <= eps:
            # Parallel / collinear
            if abs(_cross(qpx, qpy, d1x, d1y)) > eps:
                return False

            # Collinear overlap check using projection onto dominant axis
            if abs(d1x) >= abs(d1y):
                a_min, a_max = sorted((x1, x2))
                b_min, b_max = sorted((x3, x4))
            else:
                a_min, a_max = sorted((y1, y2))
                b_min, b_max = sorted((y3, y4))
            return max(a_min, b_min) <= min(a_max, b_max) + eps

        t = _cross(qpx, qpy, d2x, d2y) / denom
        u = _cross(qpx, qpy, d1x, d1y) / denom
        return (-eps <= t <= 1.0 + eps) and (-eps <= u <= 1.0 + eps)

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

    line_start = (cx, cy)
    line_end = (px, py)
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
            if _segment_intersects(line_start, line_end, a, b):
                return False

    return True


# =============================================================================
# RADAR SAMPLING
# Build directional proximity observations from obstacle and wall samples.
# =============================================================================
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


# =============================================================================
# COLLISION AVOIDANCE
# Radar-triggered collision handling and avoidance waypoint generation.
# =============================================================================
def collision_avoiding_v1(current_file: str = CURRENT_POSITION_FILE) -> bool:
    """Stop when radar detects a close obstacle inside the safe zone."""
    cur = _read_current_position(current_file)
    if cur is None:
        return False
    radar_hits = radar_sensor()
    if not radar_hits:
        return False
    if any(dist < 0.1 for _, dist in radar_hits) and abs(cur[0]) < 0.7 and abs(cur[1]) < 0.7:
        return stop("collision")
    return False

def collision_avoiding_v2(current_file: str = CURRENT_POSITION_FILE) -> bool:
    cur = _read_current_position(current_file)
    if cur is None:
        _write_collision_status(False)
        set_velocity(NORMAL_SPEED)
        return False
    cx, cy, bearing = cur

    collision_status = _read_status(COLLISION_STATUS_FILE)
    waypoint_status = _read_status(WAYPOINT_STATUS_FILE)
    if collision_status == "activated":
        if waypoint_status == "reached":
            _write_collision_status(False)
            set_velocity(NORMAL_SPEED)
            stack_wp = _read_stack_waypoint(WAYPOINTS_STACK_FILE)
            if stack_wp is not None:
                _atomic_write(WAYPOINTS_STACK_FILE, "")
                x, y, orientation = stack_wp
                if orientation is None:
                    goto(x, y)
                else:
                    goto(x, y, orientation)
                return True
            
    radar_hits = radar_sensor()

    values = {"front": 0.8, "right": 0.8, "left": 0.8, "rear": 0.8}
    for direction, dist in radar_hits:
        if direction in values:
            values[direction] = min(0.8, dist)

    if any(dist < 0.05 for _, dist in radar_hits) and bearing is not None and abs(cx) <= 0.82 and abs(cy) <= 0.82:
        
        # print(f"[waypoints_cruise] radar hits: {radar_hits}, values: {values}", file=sys.stderr)
     
        weights = [
            values["front"],
            values["right"],
            values["left"],
            values["rear"],
        ]

        normals_robot = [
            (1.0, 0.0),   # front
            (0.0, -1.0),  # right
            (0.0, 1.0),   # left
            (-1.0, 0.0),  # rear
        ]
        theta = math.radians(bearing)
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        total_w = sum(weights)
        if total_w <= 0.0:
            _write_collision_status(False)
            set_velocity(NORMAL_SPEED)
            return False

        world_normals = []
        for (nx, ny) in normals_robot:
            wx = nx * cos_t - ny * sin_t
            wy = nx * sin_t + ny * cos_t
            world_normals.append((wx, wy))

        # Pairwise sums: front+left, left+rear, rear+right, right+front.
        pair_indices = [(0, 2), (2, 3), (3, 1), (1, 0)]
        best_vec = None
        best_mag = None
        for i, j in pair_indices:
            vx = weights[i] * world_normals[i][0] + weights[j] * world_normals[j][0]
            vy = weights[i] * world_normals[i][1] + weights[j] * world_normals[j][1]
            mag = math.hypot(vx, vy)
            if best_mag is None or mag > best_mag:
                best_mag = mag
                best_vec = (vx, vy)

        # Invert direction after selecting the strongest pairwise vector.
        best_vec = (best_vec[0]/best_mag, best_vec[1]/best_mag)

        step = 0.15
        dx_world = step * best_vec[0]
        dy_world = step * best_vec[1]
        set_velocity(MAX_SPEED)
        _write_collision_status(True)
        _stack_current_waypoint()
        # print(f"[waypoints_cruise] collision avoiding activated, radar values: {weights}, move vector: ({dx_world:.3f}, {dy_world:.3f})", file=sys.stderr)
        goto(cx + dx_world, cy + dy_world, bearing, waypoint_type="collision")
        return True
        
    if collision_status == "activated":
        if waypoint_status == "going":
            return True

    return False

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
                          smart_factor: float = 4.0) -> bool:

    trigger_distance = 0.05
    cur = _read_current_position(current_file)
    if cur is None:
        _write_collision_status(False)
        set_velocity(NORMAL_SPEED)
        return False
    cx, cy, bearing = cur

    collision_status = _read_status(COLLISION_STATUS_FILE)
    waypoint_status = _read_status(WAYPOINT_STATUS_FILE)
    abandon_time_threshold = 2.0  # seconds, if stacked waypoint is older than this, abandon it to avoid going to stale location
    if collision_status == "activated":
        if waypoint_status == "reached":
            _write_collision_status(False)
            set_velocity(NORMAL_SPEED)
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
            dynamic_wp = _read_stack_waypoint(DYNAMIC_WAYPOINTS_FILE)
            if dynamic_wp is not None:
                destination_vector = (dynamic_wp[0] - cx, dynamic_wp[1] - cy)
        elif dynamic_type == "collision":
            stacked_wp = _read_stack_waypoint(WAYPOINTS_STACK_FILE)
            if stacked_wp is not None:
                destination_vector = (stacked_wp[0] - cx, stacked_wp[1] - cy)


        # dynamic_wp = _read_stack_waypoint(DYNAMIC_WAYPOINTS_FILE)
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
            set_velocity(NORMAL_SPEED)
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
        set_velocity(MAX_SPEED)
        _write_collision_status(True)
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
              dynamic_waypoint = _read_stack_waypoint(DYNAMIC_WAYPOINTS_FILE)
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

def _read_planned_waypoints(path: str):
    """Return list of (x, y, angle_or_none) from planned_waypoints.txt."""
    namespace = {"North": math.pi / 2, "East": 0.0, "South": -math.pi / 2, "West": math.pi, "None": None}
    out = []
    for raw in _read_decision_text(path).splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        if line.endswith(","):
            line = line[:-1].strip()
        try:
            wp = eval(line, {"__builtins__": None}, namespace)
        except Exception:
            nums = re.findall(r"[-+]?[0-9]*\.?[0-9]+", line)
            if len(nums) >= 2:
                x = float(nums[0])
                y = float(nums[1])
                ang = float(nums[2]) if len(nums) >= 3 else None
                wp = (x, y, ang)
            else:
                continue
        if isinstance(wp, tuple) and len(wp) >= 2:
            if len(wp) == 2:
                out.append((float(wp[0]), float(wp[1]), None))
            else:
                ang = wp[2]
                ang = float(ang) if ang is not None else None
                out.append((float(wp[0]), float(wp[1]), ang))
    return out


def _read_planned_index(path: str) -> Optional[int]:
    raw = _read_decision_text(path).strip()
    if not raw:
        return None
    try:
        return int(raw)
    except Exception:
        return None


def _write_planned_index(path: str, index: int) -> bool:
    return _atomic_write(path, f"{int(index)}\n")
    


# =============================================================================
# WAYPOINT AND SPEED COMMANDS
# Action primitives for writing dynamic waypoint and speed outputs.
# =============================================================================
def goto(x: float, y: float, orientation=None, waypoint_type: str = "task") -> bool:
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

    x = max(-0.9, min(0.9, x))
    y = max(-0.9, min(0.9, y))

    if orientation is None:
        coord_line = f"({x:.6f}, {y:.6f}, None)\n"
    else:
        coord_line = f"({x:.6f}, {y:.6f}, {orientation:.6f})\n"

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

# =============================================================================
# DECISION MODES
# Mode handlers executed on each cruise-script tick.
# =============================================================================



# Mode dispatch table: add new handlers here
_MODE_HANDLERS = {
    
}


# =============================================================================
# CLI ENTRYPOINT
# Resolve mode from CLI/env/default and dispatch to the selected handler.
# =============================================================================
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("mode", nargs="?", help="Mode to run (overrides MODE env)")
    return p.parse_args()


def main() -> int:
    _refresh_sim_data()
    _bootstrap_decision_making_data()
    args = parse_args()
    # precedence: mode.txt -> CLI arg -> MODE env var -> DEFAULT_MODE
    mode = _read_mode(MODE_FILE) or args.mode or os.environ.get("MODE") or DEFAULT_MODE
    mode = mode.strip().lower()

    handler = _MODE_HANDLERS.get(mode)
    if handler is None:
        print(f"[waypoints_cruise] unknown mode: {mode}", file=sys.stderr)
        return 2

    return handler()


if __name__ == "__main__":
    sys.exit(main())
