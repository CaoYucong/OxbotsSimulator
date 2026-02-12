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
import math


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
    Nearest-quadrant sweep WITH cardinal pre-orientation.
    Steps:
      1) If not oriented to nearest 0/90/180/270, issue goto(current_x,current_y,cardinal) and wait.
      2) Once oriented, send entry corner (position-only) and wait.
      3) Once at corner, perform zig-zag sweep with position-only waypoints.
    Uses per-quadrant files: entry_done_q{q}.txt and oriented_q{q}.txt and sweep_index_q{q}.txt.
    """
    import os, math, sys

    # run only after follower reports last waypoint reached
    status = _read_status(status_file)
    if status != "reached":
        return 0

    # constants: 2m x 2m arena centered at 0,0
    ARENA = 2.0
    HALF = ARENA / 2.0
    X_MIN, X_MAX = -HALF, HALF
    Y_MIN, Y_MAX = -HALF, HALF

    ROBOT_SIZE = 0.20
    ROBOT_RADIUS = ROBOT_SIZE / 2.0
    SAFETY = 0.05

    GRID_NX = 4
    GRID_NY = 4

    # read robot pose
    cur = _read_current_position(current_file)
    if cur is None:
        print("[mode_developing] no current position", file=sys.stderr)
        return 0
    rx, ry, bearing_deg = cur
    if bearing_deg is None:
        bearing_deg = 0.0

    # pick nearest quadrant by center distance (1: top-right, 2: top-left, 3: bottom-left, 4: bottom-right)
    half_q = ARENA / 4.0  # 0.5
    centers = {1: ( half_q*3,  half_q*3), 2: (-half_q,  half_q*3),
               3: (-half_q, -half_q),        4: ( half_q*3, -half_q)}
    def _dist(a,b,c,d): return math.hypot(a-c, b-d)
    q = min(centers.keys(), key=lambda k: _dist(rx, ry, centers[k][0], centers[k][1]))

    # quadrant raw bounds
    x_mid = (X_MIN + X_MAX) / 2.0
    y_mid = (Y_MIN + Y_MAX) / 2.0
    if q == 1:
        raw_x0, raw_x1 = x_mid, X_MAX; raw_y0, raw_y1 = y_mid, Y_MAX
    elif q == 2:
        raw_x0, raw_x1 = X_MIN, x_mid; raw_y0, raw_y1 = y_mid, Y_MAX
    elif q == 3:
        raw_x0, raw_x1 = X_MIN, x_mid; raw_y0, raw_y1 = Y_MIN, y_mid
    else:
        raw_x0, raw_x1 = x_mid, X_MAX; raw_y0, raw_y1 = Y_MIN, y_mid

    # margin inward so waypoints not at walls
    margin_x = ROBOT_RADIUS + SAFETY
    margin_y = ROBOT_RADIUS + SAFETY
    x0 = raw_x0 + (margin_x if raw_x0 < raw_x1 else -margin_x)
    x1 = raw_x1 - (margin_x if raw_x1 > raw_x0 else -margin_x)
    y0 = raw_y0 + (margin_y if raw_y0 < raw_y1 else -margin_y)
    y1 = raw_y1 - (margin_y if raw_y1 > raw_y0 else -margin_y)

    # fallback when margins invert
    if x0 > x1:
        xm = (raw_x0 + raw_x1) / 2.0; x0 = x1 = xm
    if y0 > y1:
        ym = (raw_y0 + raw_y1) / 2.0; y0 = y1 = ym

    # entry corner (inside margin)
    if q == 1:
        entry_x, entry_y = x1, y1
    elif q == 2:
        entry_x, entry_y = x0, y1
    elif q == 3:
        entry_x, entry_y = x0, y0
    else:
        entry_x, entry_y = x1, y0

    # compute 'into-quadrant' angle and snap to nearest cardinal
    cx = (x0 + x1) / 2.0; cy = (y0 + y1) / 2.0
    into_deg = (math.degrees(math.atan2(cy - entry_y, cx - entry_x)) + 360.0) % 360.0
    cardinal = (round(into_deg / 90.0) * 90.0) % 360.0  # 0,90,180,270

    # helper funcs & files
    progress_file = os.path.join(BASE_DIR, f"sweep_index_q{q}.txt")
    entry_flag_file = os.path.join(BASE_DIR, f"entry_done_q{q}.txt")
    oriented_flag_file = os.path.join(BASE_DIR, f"oriented_q{q}.txt")

    def _read_index(path: str) -> int:
        try:
            with open(path, "r") as f:
                return int(f.read().strip())
        except Exception:
            return 0
    def _write_index(path: str, idx: int) -> bool:
        try:
            tmp = path + ".tmp"
            with open(tmp, "w") as f:
                f.write(str(idx))
            os.replace(tmp, path)
            return True
        except Exception as e:
            print(f"[mode_developing] write index error: {e}", file=sys.stderr)
            return False
    def _read_flag(path: str) -> bool:
        try:
            with open(path, "r") as f:
                return f.read().strip() == "1"
        except Exception:
            return False
    def _write_flag(path: str, val: bool) -> bool:
        try:
            tmp = path + ".tmp"
            with open(tmp, "w") as f:
                f.write("1\n" if val else "0\n")
            os.replace(tmp, path)
            return True
        except Exception as e:
            print(f"[mode_developing] write flag error: {e}", file=sys.stderr)
            return False

    entry_done = _read_flag(entry_flag_file)
    oriented_done = _read_flag(oriented_flag_file)

    # build zig-zag grid (rows are horizontal sweeps)
    grid = []
    for iy in range(GRID_NY):
        row = []
        for ix in range(GRID_NX):
            fx = ix / (GRID_NX - 1) if GRID_NX > 1 else 0.5
            fy = iy / (GRID_NY - 1) if GRID_NY > 1 else 0.5
            gx = x0 + fx * (x1 - x0)
            gy = y0 + fy * (y1 - y0)
            row.append((gx, gy))
        if iy % 2 == 1:
            row.reverse()
        grid.extend(row)
    if not grid:
        print("[mode_developing] empty grid", file=sys.stderr)
        return 1

    # helpers: distance and angle difference
    def _dist(ax, ay, bx, by): return math.hypot(ax-bx, ay-by)
    def _ang_diff(a, b):
        d = abs(((a - b + 180.0) % 360.0) - 180.0)
        return d

    ENTRY_TOL = max(0.12, ROBOT_RADIUS + 0.02)  # meters
    ORIENT_TOL = 7.5  # degrees

    # Step A: ensure robot is oriented to cardinal (rotate-in-place)
    if not oriented_done:
        cur_ang = (bearing_deg % 360.0)
        if _ang_diff(cur_ang, cardinal) <= ORIENT_TOL:
            # already oriented -> mark oriented
            _write_flag(oriented_flag_file, True)
            oriented_done = True
        else:
            # issue rotation-in-place (goto at current pos with cardinal orientation)
            print(f"[mode_developing] rotating in place to {cardinal}° (cur {cur_ang:.1f}°)")
            ok = goto(rx, ry, cardinal)
            if not ok:
                print("[mode_developing] failed to write rotation waypoint", file=sys.stderr)
                return 1
            return 0  # wait for follower to finish rotation (status -> reached)

    # Step B: ensure robot goes to the quadrant entry corner (position-only)
    d_entry = _dist(rx, ry, entry_x, entry_y)
    if not entry_done:
        if d_entry <= ENTRY_TOL:
            # already at entry -> mark entry done
            _write_flag(entry_flag_file, True)
            entry_done = True
        else:
            # send only position (no orientation) so follower drives straight
            print(f"[mode_developing] sending entry waypoint ({entry_x:.3f},{entry_y:.3f}) (pos-only)")
            ok = goto(entry_x, entry_y)
            if not ok:
                print("[mode_developing] failed to write entry waypoint", file=sys.stderr)
                return 1
            return 0  # wait until follower reaches corner

    # Step C: entry done -> perform zig-zag sweep (position-only waypoints)
    idx = _read_index(progress_file)
    idx = idx % len(grid)
    tx, ty = grid[idx]
    print(f"[mode_developing] issuing sweep waypoint #{idx} -> ({tx:.3f},{ty:.3f})")
    ok = goto(tx, ty)  # position-only
    if not ok:
        print("[mode_developing] failed to write sweep waypoint", file=sys.stderr)
        return 1

    # advance index
    next_idx = (idx + 1) % len(grid)
    if not _write_index(progress_file, next_idx):
        print("[mode_developing] warning: could not save index", file=sys.stderr)

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
