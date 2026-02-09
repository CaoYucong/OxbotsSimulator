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
import os
import random
import time
import sys
import math
import re
from typing import Optional


BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "controllers", "supervisor_controller", "real_time_data"))
WAYPOINT_STATUS_FILE = os.path.join(BASE_DIR, "waypoint_status.txt")
DYNAMIC_WAYPOINTS_FILE = os.path.join(BASE_DIR, "dynamic_waypoints.txt")
BALL_POS_FILE = os.path.join(BASE_DIR, "ball_position.txt")
CURRENT_POSITION_FILE = os.path.join(BASE_DIR, "current_position.txt")
OBSTACLE_ROBOT_FILE = os.path.join(BASE_DIR, "obstacle_robot.txt")
TIME_FILE = os.path.join(BASE_DIR, "time.txt")
SPEED_FILE = os.path.join(BASE_DIR, "speed.txt")
VISIBLE_BALLS_FILE = os.path.join(BASE_DIR, "visible_balls.txt")
PLANNED_WAYPOINTS_FILE = os.path.join(os.path.dirname(__file__), "planned_waypoints.txt")
PLANNED_INDEX_FILE = os.path.join(os.path.dirname(__file__), "planned_waypoints_index.txt")
TEMP_STATE_FILE = os.path.join(os.path.dirname(__file__), "search_state.txt")
WAYPOINTS_STACK_FILE = os.path.join(os.path.dirname(__file__), "waypoints_stack.txt")
RADAR_HISTORY_FILE = os.path.join(os.path.dirname(__file__), "radar_memory.txt")
COLLISION_STATUS_FILE = os.path.join(os.path.dirname(__file__), "collision_avoiding_status.txt")
DYNAMIC_WAYPOINTS_TYPE_FILE = os.path.join(os.path.dirname(__file__), "dynamic_waypoints_type.txt")

# Set the default mode here for convenience. Edit this file and set
# `DEFAULT_MODE` to the mode you want the script to use when no CLI arg
# or `MODE` environment variable is provided. Example: 'random', 'nearest', 'improved_nearest', 'planned' or 'developing'.
DEFAULT_MODE = 'improved_nearest'

# generation bounds (match supervisor playground bounds)
X_MIN, X_MAX = -0.86, 0.86
Y_MIN, Y_MAX = -0.86, 0.86
RADAR_MAX_RANGE = 0.8
MAX_SPEED = 0.7
NORMAL_SPEED = 0.3


def _read_status(path: str) -> Optional[str]:
    try:
        with open(path, "r") as f:
            return f.read().strip()
    except Exception:
        return None


def _atomic_write(path: str, content: str) -> bool:
    try:
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            f.write(content)
        os.replace(tmp, path)
        return True
    except Exception as e:
        # print(f"[waypoints_cruise] write failed: {e}", file=sys.stderr)
        return False


def _stack_current_waypoint(stack_file: str = WAYPOINTS_STACK_FILE,
                            current_file: str = DYNAMIC_WAYPOINTS_FILE) -> None:
    """Overwrite stack file with current dynamic waypoint content, if present."""
    if _read_status(DYNAMIC_WAYPOINTS_TYPE_FILE) != "task":
        return
    try:
        with open(current_file, "r") as f:
            current = f.read().strip()
    except Exception:
        return

    if not current:
        return

    _atomic_write(stack_file, current + "\n")


def _write_collision_status(active: bool) -> None:
    status = "activated" if active else "inactive"
    _atomic_write(COLLISION_STATUS_FILE, f"{status}\n")


def _read_ball_positions(path: str):
    """Return list of (x,y,typ) from ball_position.txt. Ignores invalid lines."""
    out = []
    try:
        with open(path, "r") as f:
            for raw in f:
                line = raw.strip()
                if not line:
                    continue
                # strip parentheses
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
    except Exception:
        pass
    return out


def _read_visible_ball_positions(path: str):
    """Return list of (x,y,typ) from visible_balls.txt. Ignores invalid lines."""
    return _read_ball_positions(path)


def _read_current_position(path: str):
    """Return (x, y, bearing_deg) from current_position.txt or None.
    
    Returns tuple of (x, y, bearing_deg) if bearing is present,
    or (x, y, None) if only coordinates are available.
    """
    try:
        with open(path, "r") as f:
            raw = f.read().strip()
            if not raw:
                return None
            line = raw
            if line.startswith("(") and line.endswith(")"):
                line = line[1:-1]
            parts = [p.strip() for p in line.split(",")]
            if len(parts) < 2:
                return None
            x = float(parts[0])
            y = float(parts[1])
            bearing = float(parts[2]) if len(parts) >= 3 else None
            return (x, y, bearing)
    except Exception:
        return None


def _read_obstacle_positions(path: str):
    """Return list of (x, y, bearing_deg_or_none) from obstacle_robot.txt. Ignores invalid lines."""
    out = []
    try:
        with open(path, "r") as f:
            for raw in f:
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
    except Exception:
        pass
    return out


def _read_time_seconds(path: str) -> Optional[float]:
    """Return current simulation time in seconds from time.txt, or None."""
    try:
        with open(path, "r") as f:
            raw = f.read().strip()
            if not raw:
                return None
            return float(raw)
    except Exception:
        return None


def _read_state_pair(path: str) -> Optional[tuple[float, float]]:
    try:
        with open(path, "r") as f:
            raw = f.read().strip()
            if not raw:
                raise FileNotFoundError
            line = raw
            if line.startswith("(") and line.endswith(")"):
                line = line[1:-1]
            nums = re.findall(r"[-+]?[0-9]*\.?[0-9]+", line)
            if len(nums) < 2:
                return None
            return (float(nums[0]), float(nums[1]))
    except FileNotFoundError:
        _atomic_write(path, "(0, 0)\n")
        return (0.0, 0.0)
    except Exception:
        return None


def _read_stack_waypoint(path: str) -> Optional[tuple[float, float, Optional[float]]]:
    """Return (x, y, orientation_or_none) from waypoints_stack.txt, or None."""
    try:
        with open(path, "r") as f:
            raw = f.read().strip()
            if not raw:
                return None
            line = raw
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
    corners_local = [
        (obstacle_half, obstacle_half),
        (obstacle_half, -obstacle_half),
        (-obstacle_half, obstacle_half),
        (-obstacle_half, -obstacle_half),
        (-obstacle_half, 0),
        (-obstacle_half, 0.5*obstacle_half),
        (-obstacle_half, -0.5*obstacle_half),
        (obstacle_half, 0),
        (obstacle_half, 0.5*obstacle_half),
        (obstacle_half, -0.5*obstacle_half),
        (0, -obstacle_half),
        (0.5*obstacle_half, -obstacle_half),
        (-0.5*obstacle_half, -obstacle_half),
        (0, obstacle_half),
        (0.5*obstacle_half, obstacle_half),
        (-0.5*obstacle_half, obstacle_half),
    ]

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

    return [(direction, dist) for direction, dist in hits.items()]


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
    if collision_status == "activated":
        waypoint_status = _read_status(WAYPOINT_STATUS_FILE)
        if waypoint_status == "going":
            return True
        if waypoint_status == "reached":
            _write_collision_status(False)
            set_velocity(NORMAL_SPEED)
            stack_wp = _read_stack_waypoint(WAYPOINTS_STACK_FILE)
            if stack_wp is not None:
                x, y, orientation = stack_wp
                if orientation is None:
                    goto(x, y)
                else:
                    goto(x, y, orientation)
                _atomic_write(WAYPOINTS_STACK_FILE, "")
                return True
    radar_hits = radar_sensor()
    sim_time = _read_time_seconds(TIME_FILE)
    if sim_time is None:
        _write_collision_status(False)
        set_velocity(NORMAL_SPEED)
        return False

    values = {"front": 0.0, "right": 0.0, "left": 0.0, "rear": 0.0}
    for direction, dist in radar_hits:
        if direction in values:
            values[direction] = max(0.0, RADAR_MAX_RANGE - dist)

    history_lines = []
    cutoff_time = sim_time - 2
    try:
        with open(RADAR_HISTORY_FILE, "r") as f:
            for raw in f:
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
                if t >= cutoff_time:
                    history_lines.append(line)
    except Exception:
        pass

    record = f"{sim_time:.3f},{values['front']:.6f},{values['right']:.6f},{values['left']:.6f},{values['rear']:.6f}"
    history_lines.append(record)
    _atomic_write(RADAR_HISTORY_FILE, "\n".join(history_lines) + "\n")

    if any(dist < 0.05 for _, dist in radar_hits) and bearing is not None and abs(cx) < 0.8 and abs(cy) < 0.7:
        weights = [
            max(0.0, values["front"]),
            max(0.0, values["right"]),
            max(0.0, values["left"]),
            max(0.0, values["rear"]),
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
            vx = (RADAR_MAX_RANGE - weights[i]) * world_normals[i][0] + (RADAR_MAX_RANGE - weights[j]) * world_normals[j][0]
            vy = (RADAR_MAX_RANGE - weights[i]) * world_normals[i][1] + (RADAR_MAX_RANGE - weights[j]) * world_normals[j][1]
            mag = math.hypot(vx, vy)
            if best_mag is None or mag > best_mag:
                best_mag = mag
                best_vec = (vx, vy)

        if best_vec is None or best_mag is None or best_mag <= 1e-6:
            _write_collision_status(False)
            set_velocity(NORMAL_SPEED)
            return False

        # Invert direction after selecting the strongest pairwise vector.
        best_vec = (best_vec[0], best_vec[1])

        step = 0.15
        dx_world = step * (best_vec[0] / best_mag)
        dy_world = step * (best_vec[1] / best_mag)
        set_velocity(MAX_SPEED)
        _write_collision_status(True)
        _stack_current_waypoint()
        # print(f"[waypoints_cruise] collision avoiding activated, radar values: {weights}, move vector: ({dx_world:.3f}, {dy_world:.3f})", file=sys.stderr)
        return goto(cx + dx_world, cy + dy_world, bearing, waypoint_type="collision")

def _read_planned_waypoints(path: str):
    """Return list of (x, y, angle_or_none) from planned_waypoints.txt."""
    namespace = {"North": math.pi / 2, "East": 0.0, "South": -math.pi / 2, "West": math.pi, "None": None}
    out = []
    try:
        with open(path, "r") as f:
            for raw in f:
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
    except Exception:
        return []
    return out


def _read_planned_index(path: str) -> Optional[int]:
    try:
        with open(path, "r") as f:
            raw = f.read().strip()
            if not raw:
                return None
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
    
    return _atomic_write(DYNAMIC_WAYPOINTS_FILE, coord_line)


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
    """Set cruise speed (m/s) by writing to speed.txt.

    Returns True on success, False on failure.
    """
    try:
        speed_value = float(velocity)
    except Exception:
        return False
    if speed_value <= 0:
        return False
    return _atomic_write(SPEED_FILE, f"{speed_value:.6f}\n")


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

    # if collision_avoiding_v2(current_file):
    #     return 0

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
          if abs(cx) > 0.85 or abs(cy) > 0.85:
              cx = max(-0.85, min(0.85, cx))
              cy = max(-0.85, min(0.85, cy))
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
                 index_file: str = PLANNED_INDEX_FILE) -> int:
    """Planned mode: cycle through planned_waypoints.txt in order."""
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
    
    这个函数供开发者测试新功能。下面是可用的工具函数说明：
    
    Returns 0 on success, non-zero on error.
    """
    status = _read_status(status_file)

    # 如果状态不是 "reached"，则不执行任何操作，直接返回 0
    # 这确保了开发模式下的逻辑只在机器人到达当前目标点后才会执行，避免干扰正常的巡航行为。
    # 只有紧急状态下才会覆盖当前目标点，比如检测到碰撞要紧急避障，
    # 可以直接调用 goto() 设置新的目标点，无需等待状态变为 "reached"。
    if status != "reached":
        return 0
    
    # ==================== 开发指南 ====================
    
    # 1. 获取当前机器人位置（返回 (x, y, bearing_deg) 或 None）
    #    bearing_deg 是角度制的朝向角度
    cur_pos = _read_current_position(CURRENT_POSITION_FILE)
    if cur_pos:
        x, y, bearing = cur_pos
        # print(f"当前位置: x={x:.3f}, y={y:.3f}, bearing={bearing}°")
    
    # 2. 获取所有小球的位置和类型（返回 [(x, y, type), ...] 列表）
    #    type 是 'PING' 或 'METAL'
    balls = _read_ball_positions(BALL_POS_FILE)
    # for ball_x, ball_y, ball_type in balls:
    #     print(f"小球: x={ball_x:.3f}, y={ball_y:.3f}, 类型={ball_type}")
    
    # 3. 获取所有障碍机器人的位置（返回 [(x, y), ...] 列表）
    obstacles = _read_obstacle_positions(OBSTACLE_ROBOT_FILE)
    # for obs_x, obs_y in obstacles:
    #     print(f"障碍机器人: x={obs_x:.3f}, y={obs_y:.3f}")

    # 4. 获取当前仿真时间（秒）
    sim_time = _read_time_seconds(TIME_FILE)
    # if sim_time is not None:
    #     print(f"当前仿真时间: {sim_time:.3f} s")
    
    # 5. 使用 goto() 函数设置目标点
    #    goto(x, y) - 只设置位置
    #    goto(x, y, orientation) - 设置位置和朝向（orientation 是角度制）
    #    返回 True 表示成功，False 表示失败
    
    # 示例：前往第一个小球的位置
    # if balls:
    #     target_x, target_y, _ = balls[0]
    #     ok = goto(target_x, target_y)
    #     if ok:
    #         print(f"设置目标点成功: ({target_x:.3f}, {target_y:.3f})")
    
    # 示例：前往指定位置并设置朝向为 90 度
    # ok = goto(0.5, 0.3, 90.0)
    
    # 示例：前往原点
    # ok = goto(0.0, 0.0)
    
    # ==================== 在此处添加你的逻辑 ====================
    
    # TODO: 添加你的开发代码
    
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
    args = parse_args()
    # precedence: CLI arg -> MODE env var -> DEFAULT_MODE
    mode = args.mode or os.environ.get("MODE") or DEFAULT_MODE
    mode = mode.strip().lower()

    handler = _MODE_HANDLERS.get(mode)
    if handler is None:
        print(f"[waypoints_cruise] unknown mode: {mode}", file=sys.stderr)
        return 2

    return handler()


if __name__ == "__main__":
    sys.exit(main())
