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

PLANNED_INDEX_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "planned_waypoints_index.txt")
TEMP_STATE_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "search_state.txt")
WAYPOINTS_STACK_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "waypoints_stack.txt")
RADAR_HISTORY_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "radar_memory.txt")
COLLISION_STATUS_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "collision_avoiding_status.txt")
DYNAMIC_WAYPOINTS_TYPE_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "dynamic_waypoints_type.txt")
ROBOT_AROUND_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "robot_around.txt")
LAST_BEST_VECTOR_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "last_best_vector.txt")
SEEN_TILE_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "see_tile.txt")
TILE_SEEN_TIME_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "tile_seen_time.txt")
BALL_MEMORY_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "ball_tile_memory.txt")
LAST_SECOND_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "last_second.txt")
BALL_LIST_MEMORY_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "ball_memory.txt")
UNSEEN_TILE_MEMORY_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "unseen_tile_memory.txt")
LAST_SECOND_TILES_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "last_second_tiles.txt")
UNSEEN_REGIONS_FILE = os.path.join(os.path.dirname(__file__), "real_time_data", "unseen_regions.txt")

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
    """Overwrite stack file with current dynamic waypoint content and timestamp.
    
    Format:
    Line 1: waypoint (x, y, orientation)
    Line 2: timestamp (seconds)
    """
    if _read_status(DYNAMIC_WAYPOINTS_TYPE_FILE) != "task":
        return
    try:
        with open(current_file, "r") as f:
            current = f.read().strip()
    except Exception:
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


def _read_ball_memory_points(path: str) -> list[tuple[float, float]]:
    """Read remembered ball points from ball_memory.txt."""
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
                    out.append((x, y))
                except Exception:
                    continue
    except Exception:
        pass
    return out


def _write_ball_memory_points(path: str, points: list[tuple[float, float]]) -> bool:
    content = "\n".join(f"({x:.6f}, {y:.6f})" for x, y in points) + ("\n" if points else "")
    return _atomic_write(path, content)


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
    """Return (x, y, orientation_or_none) from waypoints_stack.txt, or None.
    
    Format:
    Line 1: waypoint (x, y, orientation)
    Line 2: timestamp (seconds) - ignored by this function
    """
    try:
        with open(path, "r") as f:
            lines = f.readlines()
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
    try:
        with open(path, "r") as f:
            lines = f.readlines()
        if len(lines) >= 2:
            return float(lines[1].strip())
    except Exception:
        return None
    return None


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
    if dist < 0.15:
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
                    if cutoff_time <= t <= sim_time:
                        history_lines.append(line)
        except Exception:
            pass

        record = (
            f"{sim_time:.3f},{memory_values['front']:.6f},{memory_values['right']:.6f},"
            f"{memory_values['left']:.6f},{memory_values['rear']:.6f}"
        )
        history_lines.append(record)
        _atomic_write(RADAR_HISTORY_FILE, "\n".join(history_lines) + "\n")

    # print([(direction, dist) for direction, dist in hits.items()])
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
    abandon_time_threshold = 5.0  # seconds, if stacked waypoint is older than this, abandon it to avoid going to stale location
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

    if any(dist < trigger_distance for _, dist in radar_hits) and bearing is not None and abs(cx) <= 0.82 and abs(cy) <= 0.82:
        
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

def mode_realistic_nearest(status_file: str = WAYPOINT_STATUS_FILE,
                          waypoint_file: str = DYNAMIC_WAYPOINTS_FILE,
                          visible_balls_file: str = VISIBLE_BALLS_FILE,
                          current_file: str = CURRENT_POSITION_FILE) -> int:
    """Realistic nearest mode: choose nearest ball from visible_balls.txt only."""

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

# Build 0.1m point grid over [-1, 1] x [-1, 1].
# Top-left is (-0.95, 0.95), bottom-right is (0.95, -0.95).
FIELD_TILES = [
    [
        (round(-0.95 + col * 0.1, 2), round(0.95 - row * 0.1, 2))
        for col in range(20)
    ]
    for row in range(20)
]


def _read_seen_tile_matrix(path: str, rows: int, cols: int) -> list[list[float]]:
    """Read seen tile matrix from text file; return zero matrix on failure/missing data."""
    matrix = [[0.0 for _ in range(cols)] for _ in range(rows)]
    try:
        with open(path, "r") as f:
            lines = f.readlines()
    except Exception:
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
                      fov: float = 60.0,
                      view_range: float = 0.8) -> bool:
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


def update_ball_memory(memory_tile_file: str = BALL_MEMORY_FILE,
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
        for mx, my in remembered_points:
            if math.hypot(bx - mx, by - my) <= 0.1:
                is_new = False
                break
        if is_new:
            remembered_points.append((bx, by))
            new_visible_balls.append((bx, by))

    points_ok = _write_ball_memory_points(ball_memory_file, remembered_points)

    flat_tiles = []
    for r in range(rows):
        for c in range(cols):
            tx, ty = FIELD_TILES[r][c]
            flat_tiles.append((r, c, tx, ty))

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

            if d2 <= 0.1 * 0.1:
                memory[r][c] += 3.0

        if nearest_rc is not None:
            nr, nc = nearest_rc
            memory[nr][nc] += 5

    cur = _read_current_position(CURRENT_POSITION_FILE)
    if cur is not None:
        cx, cy, bearing = cur
        theta = math.radians(bearing) if bearing is not None else 0.0
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        half = 0.2

        for r, c, tx, ty in flat_tiles:
            dx = tx - cx
            dy = ty - cy
            x_robot = dx * cos_t + dy * sin_t
            y_robot = -dx * sin_t + dy * cos_t
            if abs(x_robot) <= half and abs(y_robot) <= half:
                memory[r][c] = 0.0

    tile_ok = _write_seen_tile_matrix(memory_tile_file, memory)
    return points_ok and tile_ok


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

    for r in range(rows):
        for c in range(cols):
            seen_since = seen[r][c]
            if seen_since > 0.0 and (current_time - seen_since) >= 0.5:
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

    radius = 2  # 0.2m / 0.1m
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
    if tx >= 0:
        dx = tx - 0.4
    else:
        dx = tx + 0.4
    if ty >= 0:
        dy = ty - 0.4
    else:
        dy = ty + 0.4
    heading_deg = math.degrees(math.atan2(ty - dy, tx - dx))
    goto(dx, dy, heading_deg)
    # print(f"[waypoints_cruise] goto unseen region at tile ({tr}, {tc}, {heading_deg})", file=sys.stderr)
    return True

def mode_improved_nearest(status_file: str = WAYPOINT_STATUS_FILE,
                          waypoint_file: str = DYNAMIC_WAYPOINTS_FILE,
                          visible_balls_file: str = VISIBLE_BALLS_FILE,
                          current_file: str = CURRENT_POSITION_FILE) -> int:
    """Realistic nearest mode: choose nearest ball from visible_balls.txt only."""

    update_seen_tiles()
    update_unseen_tiles()
    update_unseen_regions()
    
    update_ball_memory(visible_balls_file=visible_balls_file)

    if collision_avoiding_v3(current_file, smart_factor=3.0):
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
        if status != "reached":
            return 0

        rows = len(FIELD_TILES)
        cols = len(FIELD_TILES[0]) if rows > 0 else 0

        memory_ball = _read_seen_tile_matrix(BALL_MEMORY_FILE, rows, cols)
        best_ball_rc = None
        best_ball_val = -1.0
        for r in range(rows):
            for c in range(cols):
                v = memory_ball[r][c]
                if v > best_ball_val:
                    best_ball_val = v
                    best_ball_rc = (r, c)

        if best_ball_rc is not None and best_ball_val > 0.0:
            tr, tc = best_ball_rc
            tx, ty = FIELD_TILES[tr][tc]
            heading_deg = math.degrees(math.atan2(ty - cy, tx - cx))
            goto(tx, ty, heading_deg)
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
          return 0
        
        if goto_unseen_region(cx, cy):
            _atomic_write(TEMP_STATE_FILE, f"(0, {search_record})\n")
            return 0

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



# Mode dispatch table: add new handlers here
_MODE_HANDLERS = {
    "random": mode_random,
    "nearest": mode_nearest,
    "realistic_nearest": mode_realistic_nearest,
    "improved_nearest": mode_improved_nearest,
    "planned": mode_planned,
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
