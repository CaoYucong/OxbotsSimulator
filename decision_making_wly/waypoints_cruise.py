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
import time
import sys
import math
import re
import urllib.request
from typing import Optional


BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "controllers", "supervisor_controller", "real_time_data"))
SUPERVISOR_DIR = os.path.dirname(BASE_DIR)
HTML_PORT_FILE = os.path.join(SUPERVISOR_DIR, "html_port.txt")
WAYPOINT_STATUS_FILE = os.path.join(BASE_DIR, "waypoint_status.txt")
DYNAMIC_WAYPOINTS_FILE = os.path.join(BASE_DIR, "dynamic_waypoints.txt")
BALL_POS_FILE = os.path.join(BASE_DIR, "ball_position.txt")
VISIBLE_BALLS_FILE = os.path.join(BASE_DIR, "visible_balls.txt")
CURRENT_POSITION_FILE = os.path.join(BASE_DIR, "current_position.txt")
OBSTACLE_ROBOT_FILE = os.path.join(BASE_DIR, "obstacle_robot.txt")
TIME_FILE = os.path.join(BASE_DIR, "time.txt")

PLANNED_WAYPOINTS_FILE = os.path.join(os.path.dirname(__file__), "planned_waypoints.txt")

PLANNED_INDEX_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "planned_waypoints_index.txt")
TEMP_STATE_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "search_state.txt")
WAYPOINTS_STACK_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "waypoints_stack.txt")
RADAR_HISTORY_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "radar_memory.txt")
WALL_ONLY_RADAR_MEMORY_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "wall_only_radar_memory.txt")
ROBOT_ONLY_RADAR_MEMORY_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "robot_only_radar_memory.txt")
COLLISION_STATUS_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "collision_avoiding_status.txt")
COLLISION_COUNTER_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "collision_counter.txt")
COLLISION_COUNTER_STATE_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "collision_counter_state.txt")
TOTAL_CONTACT_TIME_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "total_contact_time.txt")
DYNAMIC_WAYPOINTS_TYPE_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "dynamic_waypoints_type.txt")
ROBOT_AROUND_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "robot_around.txt")
LAST_BEST_VECTOR_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "last_best_vector.txt")
COLLISION_AVOIDING_CONFIG_FILE = os.path.join(os.path.dirname(__file__), "collision_avoiding.txt")

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
    "speed": "0.300000",
}
DECISION_MAKING_DATA_URL = f"http://localhost:{FIELD_VIEWER_PORT}/decision_making_data"
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
DEVELOPING_SEED_DEFAULTS = {
    "develop_state": "",
    "develop_sweep_counts": "",
    "develop_rotate_idx": "0",
    "visited_coords": "",
}

# Set the default mode here for convenience. Edit this file and set
# `DEFAULT_MODE` to the mode you want the script to use when no CLI arg
# or `MODE` environment variable is provided. Example: 'random', 'nearest', 'improved_nearest', 'planned' or 'developing'.
DEFAULT_MODE = 'developing'

# generation bounds (match supervisor playground bounds)
X_MIN, X_MAX = -0.86, 0.86
Y_MIN, Y_MAX = -0.86, 0.86
RADAR_MAX_RANGE = 0.8
MAX_SPEED = 0.5
NORMAL_SPEED = 0.3

# Tunable parameters (adjust based on your robot speed/time resolution)
V_MAX = 0.5          # m/s, physical upper limit for relative velocity (for gating and clipping)
MEDIAN_WINDOW = 3    # Median filter window (frames)
GATE_HARD = 25.0     # Hard threshold for Mahalanobis distance (discard measurement if exceeded)
GATE_SOFT = 4.0      # Soft threshold for Mahalanobis distance (soft inflate R if exceeded)
SOFT_R_MULT = 10.0   # Maximum multiplier for soft inflation
MIN_SAMPLES_FOR_KF = 2
MAX_REASONABLE_DISTANCE = 10.0  # m, values exceeding this are considered anomalous (tunable)


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
    if not DECISION_MAKING_DATA_CACHE:
        _refresh_decision_making_data()
    payload = dict(DECISION_MAKING_DATA_CACHE) if DECISION_MAKING_DATA_CACHE else {}
    payload.update(DECISION_MAKING_DATA_LOCAL_CACHE)
    return _post_decision_making_data(payload)


def _decision_key(path: str) -> str:
    return os.path.splitext(os.path.basename(path))[0].lower()


def _read_decision_text(path: str) -> str:
    key = _decision_key(path)
    value = _get_decision_making_value(key)
    return "" if value is None else str(value)


def _write_decision_text(path: str, content: str) -> bool:
    key = _decision_key(path)
    return _update_decision_making_local(key, content)


def _update_decisions_local(key: str, value: str) -> bool:
    DECISIONS_LOCAL_CACHE[key] = value
    return _post_decisions_data(dict(DECISIONS_LOCAL_CACHE))


def _bootstrap_developing_data() -> None:
    _refresh_decision_making_data()
    payload = dict(DECISION_MAKING_DATA_CACHE) if DECISION_MAKING_DATA_CACHE else {}
    changed = False
    for key, default_value in DEVELOPING_SEED_DEFAULTS.items():
        existing = payload.get(key)
        if isinstance(existing, str) and existing.strip():
            continue
        if existing is not None and not isinstance(existing, str):
            continue
        payload[key] = default_value
        changed = True
    if changed:
        DECISION_MAKING_DATA_LOCAL_CACHE.update(payload)
        _post_decision_making_data(payload)


def _bootstrap_stack_data() -> None:
    _refresh_decision_making_data()
    payload = dict(DECISION_MAKING_DATA_CACHE) if DECISION_MAKING_DATA_CACHE else {}
    existing = payload.get("waypoints_stack")
    if existing is None:
        payload["waypoints_stack"] = ""
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


def _write_collision_counter(
    count: int,
    times: list[float],
    path: str = COLLISION_COUNTER_FILE,
) -> bool:
    content = f"count={int(count)}\n" + "".join(f"time={t:.3f}\n" for t in times)
    return _atomic_write(path, content)


def _read_collision_state(
    path: str = COLLISION_COUNTER_STATE_FILE,
) -> tuple[Optional[float], bool]:
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


def _write_collision_state(
    last_time: Optional[float],
    in_collision: bool,
    path: str = COLLISION_COUNTER_STATE_FILE,
) -> bool:
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


def _write_total_contact_time(
    total_seconds: float,
    path: str = TOTAL_CONTACT_TIME_FILE,
) -> bool:
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

    obstacles = _read_obstacle_positions(OBSTACLE_ROBOT_FILE)
    if not obstacles:
        return []

    half_band = corridor / 2.0
    theta = math.radians(bearing)
    hits = {}

    obstacle_half = 0.1
    robot_half = 0.1
    sample_spacing = 0.1 * obstacle_half  # 0.01 spacing between sample points
    corners_local = []
    
    # Generate dense samples along all four edges of the obstacle
    num_samples = int(2 * obstacle_half / sample_spacing) + 1
    for i in range(num_samples):
        offset = -obstacle_half + i * sample_spacing
        # Top edge (y = obstacle_half)
        corners_local.append((offset, obstacle_half))
        # Bottom edge (y = -obstacle_half)
        corners_local.append((offset, -obstacle_half))
        # Left edge (x = -obstacle_half)
        corners_local.append((-obstacle_half, offset))
        # Right edge (x = obstacle_half)
        corners_local.append((obstacle_half, offset))

    for ox, oy, obearing in obstacles:
        otheta = math.radians(obearing) if obearing is not None else 0.0
        cos_o = math.cos(otheta)
        sin_o = math.sin(otheta)
        for lx, ly in corners_local:
            # Obstacle local -> world
            wx = ox + lx * cos_o - ly * sin_o
            wy = oy + lx * sin_o + ly * cos_o

            dx = wx - cx
            dy = wy - cy
            # World -> robot frame: x forward, y left
            x_robot = dx * math.cos(theta) + dy * math.sin(theta)
            y_robot = -dx * math.sin(theta) + dy * math.cos(theta)

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
                prev = hits.get(direction)
                if prev is None or dist < prev:
                    hits[direction] = dist

    # Add virtual points along boundaries using evenly spaced samples.
    edge_samples = [i * 0.05 for i in range(-20, 21)]
    virtual_points = (
        [(x, 1.0) for x in edge_samples]
        + [(x, -1.0) for x in edge_samples]
        + [(1.0, y) for y in edge_samples]
        + [(-1.0, y) for y in edge_samples]
    )
    for vx, vy in virtual_points:
        dx = vx - cx
        dy = vy - cy
        x_robot = dx * math.cos(theta) + dy * math.sin(theta)
        y_robot = -dx * math.sin(theta) + dy * math.cos(theta)

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

    wall_only_radar()
    robot_only_radar()
    _process_collision_counter_from_history()

    # print([(direction, dist) for direction, dist in hits.items()])
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

        best_vec = (best_vec[0] / best_mag, best_vec[1] / best_mag)

        step = 0.15
        dx_world = step * best_vec[0]
        dy_world = step * best_vec[1]
        set_velocity(MAX_SPEED)
        _write_collision_status(True)
        _stack_current_waypoint()
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
            continue

        filtered_hits.append((direction, dist))

    if any(dist < collision_threshold for _, dist in filtered_hits):
        return True

    return False


def collision_avoiding_v3(
    current_file: str = CURRENT_POSITION_FILE,
    smart_factor: float = 4.0,
    stack_waypoint: bool = True,
) -> bool:

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
                        _atomic_write(WAYPOINTS_STACK_FILE, "")
                        return True  # Don't goto anywhere, just return

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

    values = {"front": 0.8, "right": 0.8, "left": 0.8, "rear": 0.8}
    for direction, dist in radar_hits:
        if direction in values:
            values[direction] = min(0.8, dist)

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

    if collision_activating_condition(current_file):

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

        safest_vec = (safest_vec[0] / safest_mag, safest_vec[1] / safest_mag)

        last_best_vec = _read_stack_waypoint(LAST_BEST_VECTOR_FILE)
        best_vec = None
        best_score = None

        min_radar_distance = max(0, min(values.values()))
        safety_factor = 8 + (min_radar_distance / trigger_distance) * 20 if trigger_distance > 0 else 10.0

        for vec in filtered_vectors:
            score = 0.0
            score += (vec[0] * safest_vec[0] + vec[1] * safest_vec[1]) * safety_factor
            if last_best_vec is None:
                if destination_vector is not None:
                    score += (vec[0] * destination_vector[0] + vec[1] * destination_vector[1]) * smart_factor
            if last_best_vec is not None:
                score += (vec[0] * last_best_vec[0] + vec[1] * last_best_vec[1]) * smart_factor
                if destination_vector is not None:
                    score += (vec[0] * destination_vector[0] + vec[1] * destination_vector[1]) * smart_factor
            if best_score is None or score > best_score:
                best_score = score
                best_vec = vec

        _atomic_write(
            LAST_BEST_VECTOR_FILE,
            f"({best_vec[0]:.6f}, {best_vec[1]:.6f})\n",
        )

        best_mag = math.hypot(best_vec[0], best_vec[1])
        best_vec = (best_vec[0], best_vec[1])

        dx_world = jump_step * (best_vec[0] / best_mag)
        dy_world = jump_step * (best_vec[1] / best_mag)
        set_velocity(MAX_SPEED)
        _write_collision_status(True)
        if stack_waypoint:
            _stack_current_waypoint()

        # Determine orientation based on waypoint hierarchy
        target_orientation = bearing  # Default to current bearing
        target_x = cx + dx_world
        target_y = cy + dy_world

        if min_radar_distance / trigger_distance > 0.7:
            # If obstacle is not too close, try to orient towards the task waypoint to encourage progress
            if dynamic_type == "task":
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

    # critical_alpha = None
    # if abs(x) > 0.86 or abs(y) > 0.86:
    #     if x < -0.86:
    #         critical_alpha = 45.0 - math.degrees(math.acos((x + 1.0) / (0.1 * math.sqrt(2.0))))
    #         coord_line = f"({x:.6f}, {y:.6f}, 180)\n"
    #     if x > 0.86:
    #         critical_alpha = 45.0 - math.degrees(math.acos((-x + 1.0) / (0.1 * math.sqrt(2.0))))
    #         coord_line = f"({x:.6f}, {y:.6f}, 0)\n"
    #     if y < -0.86:
    #         critical_alpha = 45.0 - math.degrees(math.acos((y + 1.0) / (0.1 * math.sqrt(2.0))))
    #         coord_line = f"({x:.6f}, {y:.6f}, -90)\n"
    #     if y > 0.86:
    #         critical_alpha = 45.0 - math.degrees(math.acos((-y + 1.0) / (0.1 * math.sqrt(2.0))))
    #         coord_line = f"({x:.6f}, {y:.6f}, 90)\n"
    #     # print(f"[waypoints_cruise] critical alpha: {critical_alpha:.2f} degrees", file=sys.stderr)

    # if critical_alpha is not None:
    #     pass
    
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


def mode_random(status_file: str = WAYPOINT_STATUS_FILE, waypoint_file: str = DYNAMIC_WAYPOINTS_FILE) -> int:
    """Random mode: if status == 'reached', generate and write a new waypoint.

    Returns 0 on success (or nothing to do), non-zero on error.
    """
    status = _read_status(status_file)
    if status != "reached":
        return 0

    # generate new coordinate
    seed = int(time.time()) ^ (os.getpid() << 16)
    random.seed(seed)
    x = float(random.uniform(X_MIN, X_MAX))
    y = float(random.uniform(Y_MIN, Y_MAX))

    ok = goto(x, y)
    return 0 if ok else 1


def mode_nearest(status_file: str = WAYPOINT_STATUS_FILE,
                 waypoint_file: str = DYNAMIC_WAYPOINTS_FILE,
                 balls_file: str = BALL_POS_FILE,
                 current_file: str = CURRENT_POSITION_FILE) -> int:
    """Nearest mode: if status == 'reached', pick nearest ball to robot and write it as waypoint."""

    if collision_avoiding_v3(current_file):
        return 0

    sim_time = _read_time_seconds(TIME_FILE)
    if sim_time is not None and sim_time > 170.0:
        goto(-0.9, 0.0, 180.0)
        return 0

    status = _read_status(status_file)
    # if status != "reached":
    #     return 0

    cur = _read_current_position(current_file)
    if cur is None:
        return 0
    bx = _read_ball_positions(balls_file)
    if not bx:
        return 0

    cx, cy, _ = cur
    best = None
    best_d2 = None
    for (x, y, typ) in bx:
        try:
            dx = x - cx; dy = y - cy
            d2 = dx*dx + dy*dy
        except Exception:
            continue
        if best_d2 is None or d2 < best_d2:
            best_d2 = d2
            best = (x, y)

    if best is None:
        return 0

    target_x, target_y = best
    heading_deg = math.degrees(math.atan2(target_y - cy, target_x - cx))
    ok = goto(target_x, target_y, heading_deg)
    return 0 if ok else 1

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

def mode_improved_nearest(status_file: str = WAYPOINT_STATUS_FILE,
                          waypoint_file: str = DYNAMIC_WAYPOINTS_FILE,
                          visible_balls_file: str = VISIBLE_BALLS_FILE,
                          current_file: str = CURRENT_POSITION_FILE) -> int:
    """Improved nearest mode: choose nearest ball from visible_balls.txt only."""

    if collision_avoiding_v3(current_file, smart_factor=3.0):
        return 0

    sim_time = _read_time_seconds(TIME_FILE)
    if sim_time is not None and sim_time > 170.0:
        goto(-0.9, 0.0, 180.0)
        return 0

    cur = _read_current_position(current_file)

    status = _read_status(status_file)
    # if status != "reached":
    #     return 0

    if cur is None:
        return 0
    bx = _read_visible_ball_positions(visible_balls_file)
    cx, cy, bearing = cur
    if not bx:
        if status != "reached":
            return 0
        state = _read_state_pair(TEMP_STATE_FILE)
        if state is None:
            state = (0.0, 0.0)
        miss_time = int(state[0])
        search_record = int(state[1])
        # print(f"[waypoints_cruise] no visible balls, miss_time={miss_time}, search_record={search_record}", file=sys.stderr)
        if miss_time == 0.0:
          if abs(cx) > 0.8 or abs(cy) > 0.8:
              cx = max(-0.8, min(0.8, cx))
              cy = max(-0.8, min(0.8, cy))
              goto(cx, cy, bearing)
          else:
              heading = None if bearing is None else bearing + 179.0
              goto(cx, cy, heading)
          _atomic_write(TEMP_STATE_FILE, f"({miss_time+1}, {search_record})\n")
          return 0
        if miss_time == 1.0:
          heading = None if bearing is None else bearing + 179.0
          goto(cx, cy, heading)
          _atomic_write(TEMP_STATE_FILE, f"({miss_time+1}, {search_record})\n")
        if miss_time >= 2.0:
            index = search_record % len(SEARCHING_SEQUENCE)
            goto(SEARCHING_SEQUENCE[index][0], SEARCHING_SEQUENCE[index][1], SEARCHING_SEQUENCE[index][2])
            _atomic_write(TEMP_STATE_FILE, f"({miss_time}, {(search_record+1)%len(SEARCHING_SEQUENCE)})\n")
    else:
        state = _read_state_pair(TEMP_STATE_FILE)
        if state is not None:
          search_record = int(state[1])
        _atomic_write(TEMP_STATE_FILE, f"(0, {search_record})\n")
    best = None
    best_d2 = None
    for (x, y, typ) in bx:
        try:
            dx = x - cx
            dy = y - cy
            d2 = dx * dx + dy * dy
        except Exception:
            continue
        if best_d2 is None or d2 < best_d2:
            best_d2 = d2
            best = (x, y)

    if best is None:
        return 0

    target_x, target_y = best
    heading_deg = math.degrees(math.atan2(target_y - cy, target_x - cx))
    ok = goto(target_x, target_y, heading_deg)
    return 0 if ok else 1


def mode_planned(status_file: str = WAYPOINT_STATUS_FILE,
                 planned_file: str = PLANNED_WAYPOINTS_FILE,
                 index_file: str = PLANNED_INDEX_FILE,
                 current_file: str = CURRENT_POSITION_FILE) -> int:
    """Planned mode: cycle through planned_waypoints.txt in order."""

    if collision_avoiding_v3(current_file, smart_factor=1.0):
        return 0
    
    set_velocity(0.7)

    sim_time = _read_time_seconds(TIME_FILE)
    if sim_time is not None and sim_time > 170.0:
        goto(-0.9, 0.0, 180.0)
        return 0
    
    waypoints = _read_planned_waypoints(planned_file)
    if not waypoints:
        return 0

    status = _read_status(status_file)
    index = _read_planned_index(index_file)
    index_missing = index is None
    if index is None:
        index = 0

    if index >= len(waypoints):
        index = 0

    if index_missing or status == "reached":
        if not index_missing:
            index = (index + 1) % len(waypoints)
        _write_planned_index(index_file, index)

        x, y, ang = waypoints[index]
        if ang is None:
            goto(x, y)
        else:
            goto(x, y, ang)

    return 0


def mode_developing(status_file: str = WAYPOINT_STATUS_FILE,
                    waypoint_file: str = DYNAMIC_WAYPOINTS_FILE,
                    current_file: str = CURRENT_POSITION_FILE) -> int:
    
    """Developing mode: for testing and development purposes.
    
    This function is for developers to test new features. Available utility functions are documented below.
    
    Returns 0 on success, non-zero on error.
    """
    status = _read_status(status_file)

    # If status is not "reached", do nothing and return 0
    # This ensures development mode logic only executes after the robot reaches the current target,
    # avoiding interference with normal cruise behavior.
    # Only in emergency situations (e.g., collision detection requiring immediate avoidance)
    # should you call goto() to set a new target without waiting for "reached" status.
    
    if collision_avoiding_v3(current_file, smart_factor=2.0): return 0
    
    if status != "reached":
        return 0
    
    # ==================== Development Guide ====================
    
    # 1. Get current robot position (returns (x, y, bearing_deg) or None)
    #    bearing_deg is the heading angle in degrees
    cur_pos = _read_current_position(CURRENT_POSITION_FILE)
    if cur_pos:
        x, y, bearing = cur_pos
        # print(f"Current position: x={x:.3f}, y={y:.3f}, bearing={bearing}°")
    
    # 2. Get all ball positions and types (returns [(x, y, type), ...] list)
    #    type is 'PING' or 'METAL'
    balls = _read_ball_positions(BALL_POS_FILE)
    # for ball_x, ball_y, ball_type in balls:
    #     print(f"Ball: x={ball_x:.3f}, y={ball_y:.3f}, type={ball_type}")
    
    # 3. Get all obstacle robot positions (returns [(x, y), ...] list)
    obstacles = _read_obstacle_positions(OBSTACLE_ROBOT_FILE)
    # for obs_x, obs_y in obstacles:
    #     print(f"Obstacle robot: x={obs_x:.3f}, y={obs_y:.3f}")

    # 4. Get current simulation time (seconds)
    sim_time = _read_time_seconds(TIME_FILE)
    # if sim_time is not None:
    #     print(f"Current simulation time: {sim_time:.3f} s")
    
    # 5. Use goto() function to set target waypoint
    #    goto(x, y) - set position only
    #    goto(x, y, orientation) - set position and heading (orientation in degrees)
    #    Returns True on success, False on failure
    
    # Example: Go to the first ball's position
    # if balls:
    #     target_x, target_y, _ = balls[0]
    #     ok = goto(target_x, target_y)
    #     if ok:
    #         print(f"Target set successfully: ({target_x:.3f}, {target_y:.3f})")
    
    # Example: Go to specified position with 90-degree heading
    # ok = goto(0.5, 0.3, 90.0)
    
    # Example: Go to origin
    # ok = goto(0.0, 0.0)
    
    # ==================== Add Your Logic Here ====================
    
    # TODO: 添加你的开发代码
    """
    Developing mode with:
      - 60% forward speed (capped at MAX_SPEED)
      - initial 180deg sweep to find high-density sector
      - density-aware nearest-in-vision collection
      - logs visited coords to real_time_data/visited_coords.txt
      - immediate clockwise rotation when no visible balls
      - if rotates 360° without seeing a traceable ball -> pick least-visited of 9 grid squares and goto its center
      - if any visible balls appear while moving -> switch to collect_loop immediately
      - after 60s, bias fallback to unvisited areas
    Note: internal obstacle avoidance removed; use collision_avoiding_v3() externally.
    """
    import json

    # Tunables
    NUM_SECTORS = 6
    NUM_STEPS = 4
    SWEEP_STEP_DEG = 180.0 / NUM_STEPS
    SECTOR_WIDTH = 180.0 / NUM_SECTORS
    SCAN_DISTANCE = 0.7
    COLLECT_DURATION = 10.0
    STATE_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "develop_state.json")
    SWEEP_COUNTS_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "develop_sweep_counts.json")
    ROTATE_IDX_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "develop_rotate_idx.txt")
    VISITED_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "visited_coords.txt")

    # rotation steps (scan + when no balls)
    ROT_STEP_SCAN = 45.0     # degrees per step during fast sweep
    ROT_STEP_NO_BALL = 90.0  # degrees added each rotation attempt when no balls seen

    # selection tie tolerances for "similar distance"
    REL_TOL = 0.15        # relative tolerance (15%)
    ABS_TOL = 0.05        # absolute tolerance (meters)

    # density radius for local density counting
    DENSITY_RADIUS = 0.25  # meters

    # speed: reduce forward speed to 60% of NORMAL_SPEED (cap at MAX_SPEED)
    try:
        set_velocity(min(MAX_SPEED, NORMAL_SPEED * 0.6))
    except Exception:
        pass

    # helpers
    def _normalize_deg(a):
        return (a + 180.0) % 360.0 - 180.0

    def _rel_angle_deg_to_robot(ball_x, ball_y, cx, cy, bearing):
        ang = math.degrees(math.atan2(ball_y - cy, ball_x - cx))
        rel = _normalize_deg(ang - bearing)
        return rel

    def _read_json(path):
        raw = _read_decision_text(path).strip()
        if not raw:
            return None
        try:
            return json.loads(raw)
        except Exception:
            return None

    def _write_json(path, obj):
        try:
            return _atomic_write(path, json.dumps(obj) + "\n")
        except Exception:
            return False

    def _append_visit(path, x, y):
        try:
            existing = _read_decision_text(path)
            new = existing + f"({x:.6f}, {y:.6f})\n"
            return _atomic_write(path, new)
        except Exception:
            return False

    def _read_visits(path):
        out = []
        for raw in _read_decision_text(path).splitlines():
            line = raw.strip()
            if not line:
                continue
            if line.startswith("(") and line.endswith(")"):
                line = line[1:-1]
            parts = [p.strip() for p in line.split(",")]
            if len(parts) >= 2:
                try:
                    vx = float(parts[0]); vy = float(parts[1])
                    out.append((vx, vy))
                except Exception:
                    continue
        return out

    def _near_wall(x, y, margin=0.88):
        return abs(x) >= margin or abs(y) >= margin

    # choose nearest ball with density tie-breaker
    def _choose_ball_by_density(visible, cx, cy):
        if not visible:
            return None
        dist_list = []
        for bx, by, typ in visible:
            try:
                d = math.hypot(bx - cx, by - cy)
            except Exception:
                d = float("inf")
            dist_list.append(d)
        if not dist_list:
            return None
        dmin = min(dist_list)
        candidates = []
        for i, d in enumerate(dist_list):
            if d <= dmin * (1.0 + REL_TOL) or d <= dmin + ABS_TOL:
                candidates.append(i)
        best_idx = None
        best_score = None
        for idx in candidates:
            bx, by, typ = visible[idx]
            count = 0
            for jx, jy, _ in visible:
                if math.hypot(jx - bx, jy - by) <= DENSITY_RADIUS:
                    count += 1
            score = (count, -dist_list[idx], -idx)
            if best_score is None or score > best_score:
                best_score = score
                best_idx = idx
        if best_idx is None:
            return None
        bx, by, _ = visible[best_idx]
        return (bx, by)

    # ----- NEW: choose least-visited square from 3x3 grid -----
    def _choose_least_visited_square_center(cx, cy, visits):
        """
        Divide arena into 3x3 equal squares using X_MIN..X_MAX and Y_MIN..Y_MAX.
        Count how many visit points fall into each square and return center of
        the square with the fewest visits. Tie-break by distance from robot.
        """
        # Prepare grid
        nx = 3; ny = 3
        xmin, xmax = X_MIN, X_MAX
        ymin, ymax = Y_MIN, Y_MAX
        cell_w = (xmax - xmin) / nx
        cell_h = (ymax - ymin) / ny

        # initialize counts
        counts = [[0 for _ in range(nx)] for _ in range(ny)]
        for (vx, vy) in visits:
            # clamp to grid range
            if vx < xmin or vx > xmax or vy < ymin or vy > ymax:
                continue
            ix = int((vx - xmin) / cell_w)
            iy = int((vy - ymin) / cell_h)
            # handle edge-case on upper boundary
            if ix >= nx:
                ix = nx - 1
            if iy >= ny:
                iy = ny - 1
            counts[iy][ix] += 1

        # find least-visited cells
        best_cells = []
        best_count = None
        for iy in range(ny):
            for ix in range(nx):
                c = counts[iy][ix]
                if best_count is None or c < best_count:
                    best_count = c
                    best_cells = [(ix, iy)]
                elif c == best_count:
                    best_cells.append((ix, iy))

        # if multiple ties, pick the one whose center is closest (so robot doesn't go extremely far) or optionally furthest — choose closest here
        best_pt = None
        best_dist = None
        for (ix, iy) in best_cells:
            center_x = xmin + (ix + 0.5) * cell_w
            center_y = ymin + (iy + 0.5) * cell_h
            d = math.hypot(center_x - cx, center_y - cy)
            if best_dist is None or d < best_dist:
                best_dist = d
                best_pt = (center_x, center_y, counts[iy][ix])

        return best_pt  # (tx, ty, count) or None

    # choose unvisited target from coarse grid (used elsewhere too)
    def _choose_unvisited_target(cx, cy, visits, bias_factor=2.0):
        # keep previous fallback (fine-grain) if needed, but for 360° event we use 3x3 grid
        return _choose_least_visited_square_center(cx, cy, visits)

    # ---------- main ----------
    if _read_status(status_file) != "reached":
        return 0

    sim_time = _read_time_seconds(TIME_FILE)
    if sim_time is None:
        return 0
    cur = _read_current_position(current_file)
    if cur is None:
        return 0
    cx, cy, bearing = cur
    bearing = 0.0 if bearing is None else float(bearing)

    raw_state = _read_status(STATE_FILE)
    if raw_state:
        try:
            state = json.loads(raw_state)
        except Exception:
            state = None
    else:
        state = None

    counts = _read_json(SWEEP_COUNTS_FILE) or [0] * NUM_SECTORS
    try:
        rotate_idx = int(_read_decision_text(ROTATE_IDX_FILE).strip() or "0")
    except Exception:
        rotate_idx = 0

    if state is None:
        state = {
            "phase": "scan",                # scan -> goto_area -> collect_loop -> move_to_unvisited -> global_fallback
            "scan_started_time": sim_time,
            "chosen_sector": None,
            "collect_started_time": None,
            "rotation_unseen_deg": 0.0
        }
        _write_json(SWEEP_COUNTS_FILE, counts)
        _atomic_write(STATE_FILE, json.dumps(state) + "\n")

    phase = state.get("phase", "scan")
    rotation_unseen_deg = float(state.get("rotation_unseen_deg", 0.0))

    visible = _read_visible_ball_positions(VISIBLE_BALLS_FILE)

    # Reset rotation counter when any balls are visible
    if visible:
        rotation_unseen_deg = 0.0
        state["rotation_unseen_deg"] = rotation_unseen_deg
        _atomic_write(STATE_FILE, json.dumps(state) + "\n")

    # No visible balls => rotate & increment counter
    if not visible:
        rotation_unseen_deg += ROT_STEP_NO_BALL
        state["rotation_unseen_deg"] = rotation_unseen_deg
        _atomic_write(STATE_FILE, json.dumps(state) + "\n")

        if rotation_unseen_deg >= 360.0:
            # pick the least-visited square center (3x3), goto it and reset rotation counter
            visits = _read_visits(VISITED_FILE)
            # bias factor increases over time (keeps compatibility with previous logic)
            bias_factor = 1.0
            if sim_time >= 60.0:
                bias_factor += min(5.0, (sim_time - 60.0) / 60.0 * 4.0)

            target = _choose_least_visited_square_center(cx, cy, visits)
            if target is not None:
                tx, ty, count = target
                _append_visit(VISITED_FILE, tx, ty)
                state["phase"] = "move_to_unvisited"
                state["rotation_unseen_deg"] = 0.0
                _atomic_write(STATE_FILE, json.dumps(state) + "\n")
                goto(tx, ty, None)
                return 0
            else:
                # fallback to a rotation if no target
                state["rotation_unseen_deg"] = 0.0
                _atomic_write(STATE_FILE, json.dumps(state) + "\n")
                new_heading = (bearing - ROT_STEP_SCAN) % 360.0
                goto(cx, cy, new_heading)
                return 0
        else:
            new_heading = (bearing - ROT_STEP_NO_BALL) % 360.0
            goto(cx, cy, new_heading)
            return 0

    # -------------------- SCAN --------------------
    if phase == "scan":
        step_heading = (bearing - 90.0) + rotate_idx * SWEEP_STEP_DEG
        step_heading = (step_heading + 360.0) % 360.0
        goto(cx, cy, step_heading)

        for (bx, by, typ) in visible:
            rel = _rel_angle_deg_to_robot(bx, by, cx, cy, bearing)
            if abs(rel) <= 90.0:
                rel_shifted = rel + 90.0
                idx = int(rel_shifted // SECTOR_WIDTH)
                idx = max(0, min(NUM_SECTORS - 1, idx))
                counts[idx] += 1

        _write_json(SWEEP_COUNTS_FILE, counts)

        rotate_idx = (rotate_idx + 1) % NUM_STEPS
        _atomic_write(ROTATE_IDX_FILE, f"{rotate_idx}\n")

        elapsed = sim_time - float(state.get("scan_started_time", sim_time))
        if rotate_idx == 0 or elapsed >= max(0.5, NUM_STEPS * 0.03):
            last_visible = visible
            dist_sums = [0.0] * NUM_SECTORS
            dist_counts = [0] * NUM_SECTORS
            for (bx, by, typ) in last_visible:
                rel = _rel_angle_deg_to_robot(bx, by, cx, cy, bearing)
                if abs(rel) <= 90.0:
                    rel_shifted = rel + 90.0
                    idx = int(rel_shifted // SECTOR_WIDTH)
                    idx = max(0, min(NUM_SECTORS - 1, idx))
                    d = math.hypot(bx - cx, by - cy)
                    dist_sums[idx] += d
                    dist_counts[idx] += 1

            best_idx = None
            best_count = None
            best_mean = None
            for i in range(NUM_SECTORS):
                c = counts[i]
                mean_d = (dist_sums[i] / dist_counts[i]) if dist_counts[i] > 0 else float("inf")
                if best_count is None or c > best_count or (c == best_count and mean_d < best_mean):
                    best_idx = i
                    best_count = c
                    best_mean = mean_d

            if best_idx is None:
                best_idx = NUM_SECTORS // 2

            sector_center_rel = -90.0 + (best_idx + 0.5) * SECTOR_WIDTH
            global_angle = (bearing + sector_center_rel) % 360.0
            rad = math.radians(global_angle)
            tx = cx + SCAN_DISTANCE * math.cos(rad)
            ty = cy + SCAN_DISTANCE * math.sin(rad)

            tx = max(-0.9, min(0.9, tx))
            ty = max(-0.9, min(0.9, ty))

            state["phase"] = "goto_area"
            state["chosen_sector"] = int(best_idx)
            state["sector_center_angle"] = float(sector_center_rel)
            state["collect_started_time"] = sim_time
            state["rotation_unseen_deg"] = 0.0
            _atomic_write(STATE_FILE, json.dumps(state) + "\n")

            _write_json(SWEEP_COUNTS_FILE, [0] * NUM_SECTORS)
            _atomic_write(ROTATE_IDX_FILE, "0\n")

            _append_visit(VISITED_FILE, tx, ty)
            goto(tx, ty, math.degrees(math.atan2(ty - cy, tx - cx)))
            return 0

        return 0

    # -------------------- GOTO_AREA --------------------
    if phase == "goto_area":
        state["phase"] = "collect_loop"
        state["rotation_unseen_deg"] = 0.0
        _atomic_write(STATE_FILE, json.dumps(state) + "\n")
        return 0

    # -------------------- MOVE_TO_UNVISITED --------------------
    if phase == "move_to_unvisited":
        # if visible balls appear while moving, switch to collect_loop
        if visible:
            state["phase"] = "collect_loop"
            state["rotation_unseen_deg"] = 0.0
            _atomic_write(STATE_FILE, json.dumps(state) + "\n")
            return 0
        # otherwise let the goto to the unvisited area proceed (we commanded it earlier)
        return 0

    # -------------------- COLLECT_LOOP --------------------
    if phase == "collect_loop":
        if visible:
            # density-aware nearest selection
            dist_list = []
            for bx, by, typ in visible:
                try:
                    d = math.hypot(bx - cx, by - cy)
                except Exception:
                    d = float("inf")
                dist_list.append(d)
            # choose by density tie-breaker helper
            choice = _choose_ball_by_density(visible, cx, cy)
            if choice is not None:
                tx, ty = choice
                _append_visit(VISITED_FILE, tx, ty)
                heading_deg = math.degrees(math.atan2(ty - cy, tx - cx))
                ok = goto(tx, ty, heading_deg)
                state["rotation_unseen_deg"] = 0.0
                _atomic_write(STATE_FILE, json.dumps(state) + "\n")
                return 0 if ok else 1
        # no visible: rotate and increment rotation counter
        rotation_unseen_deg += ROT_STEP_NO_BALL
        state["rotation_unseen_deg"] = rotation_unseen_deg
        _atomic_write(STATE_FILE, json.dumps(state) + "\n")
        if rotation_unseen_deg >= 360.0:
            visits = _read_visits(VISITED_FILE)
            bias_factor = 1.0
            if sim_time >= 60.0:
                bias_factor += min(5.0, (sim_time - 60.0) / 60.0 * 4.0)
            target = _choose_least_visited_square_center(cx, cy, visits)
            if target is not None:
                tx, ty, cnt = target
                _append_visit(VISITED_FILE, tx, ty)
                state["phase"] = "move_to_unvisited"
                state["rotation_unseen_deg"] = 0.0
                _atomic_write(STATE_FILE, json.dumps(state) + "\n")
                goto(tx, ty, None)
                return 0
        new_heading = (bearing - ROT_STEP_NO_BALL) % 360.0
        goto(cx, cy, new_heading)
        return 0

    # -------------------- GLOBAL_FALLBACK --------------------
    if phase == "global_fallback":
        if visible:
            state["phase"] = "collect_loop"
            state["rotation_unseen_deg"] = 0.0
            _atomic_write(STATE_FILE, json.dumps(state) + "\n")
            return 0
        # use least-visited square as fallback target
        visits = _read_visits(VISITED_FILE)
        target = _choose_least_visited_square_center(cx, cy, visits)
        if target is None:
            new_heading = (bearing - ROT_STEP_NO_BALL) % 360.0
            goto(cx, cy, new_heading)
            return 0
        tx, ty, cnt = target
        _append_visit(VISITED_FILE, tx, ty)
        goto(tx, ty, None)
        return 0

    # unknown phase -> reset
    try:
        state["rotation_unseen_deg"] = 0.0
        _atomic_write(STATE_FILE, "")
    except Exception:
        pass
    try:
        _write_json(SWEEP_COUNTS_FILE, [0] * NUM_SECTORS)
    except Exception:
        pass
    return 0





# Mode dispatch table: add new handlers here
_MODE_HANDLERS = {
    "random": mode_random,
    "nearest": mode_nearest,
    "improved_nearest": mode_improved_nearest,
    "planned": mode_planned,
    "developing": mode_developing,
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("mode", nargs="?", help="Mode to run (overrides MODE env)")
    return p.parse_args()


def main() -> int:
    _refresh_sim_data()
    _bootstrap_stack_data()
    args = parse_args()
    # precedence: CLI arg -> MODE env var -> DEFAULT_MODE
    mode = args.mode or os.environ.get("MODE") or DEFAULT_MODE
    mode = mode.strip().lower()
    if mode == "developing":
        _bootstrap_developing_data()

    handler = _MODE_HANDLERS.get(mode)
    if handler is None:
        print(f"[waypoints_cruise] unknown mode: {mode}", file=sys.stderr)
        return 2

    return handler()


if __name__ == "__main__":
    sys.exit(main())
