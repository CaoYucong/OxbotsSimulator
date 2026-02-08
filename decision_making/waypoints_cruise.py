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
PLANNED_WAYPOINTS_FILE = os.path.join(BASE_DIR, "planned_waypoints.txt")
PLANNED_INDEX_FILE = os.path.join(BASE_DIR, "planned_waypoints_index.txt")
TEMP_STATE_FILE = os.path.join(BASE_DIR, "waypoints_temp_state.txt")

# Set the default mode here for convenience. Edit this file and set
# `DEFAULT_MODE` to the mode you want the script to use when no CLI arg
# or `MODE` environment variable is provided. Example: 'random', 'nearest', 'improved_nearest', 'planned' or 'developing'.
DEFAULT_MODE = 'improved_nearest'

# generation bounds (match supervisor playground bounds)
X_MIN, X_MAX = -0.86, 0.86
Y_MIN, Y_MAX = -0.86, 0.86


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
        print(f"[waypoints_cruise] write failed: {e}", file=sys.stderr)
        return False


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
    """Return list of (x, y) from obstacle_robot.txt. Ignores invalid lines."""
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
                return None
            line = raw
            if line.startswith("(") and line.endswith(")"):
                line = line[1:-1]
            nums = re.findall(r"[-+]?[0-9]*\.?[0-9]+", line)
            if len(nums) < 2:
                return None
            return (float(nums[0]), float(nums[1]))
    except Exception:
        return None


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
    

def goto(x: float, y: float, orientation=None) -> bool:
    """Set the dynamic waypoint to the specified coordinates.
    
    Args:
        x: X coordinate
        y: Y coordinate
        orientation: Optional orientation angle in degrees (default None)
    
    Returns:
        True on success, False on failure
    """

    x = max(-0.9, min(0.9, x))
    y = max(-0.9, min(0.9, y))

    if orientation is None:
        coord_line = f"({x:.6f}, {y:.6f}, None)\n"
    else:
        coord_line = f"({x:.6f}, {y:.6f}, {orientation:.6f})\n"

    critical_alpha = None
    if abs(x) > 0.86 or abs(y) > 0.86:
        if x < -0.86:
            critical_alpha = 45.0 - math.degrees(math.acos((x + 1.0) / (0.1 * math.sqrt(2.0))))
            coord_line = f"({x:.6f}, {y:.6f}, 180)\n"
        if x > 0.86:
            critical_alpha = 45.0 - math.degrees(math.acos((-x + 1.0) / (0.1 * math.sqrt(2.0))))
            coord_line = f"({x:.6f}, {y:.6f}, 0)\n"
        if y < -0.86:
            critical_alpha = 45.0 - math.degrees(math.acos((y + 1.0) / (0.1 * math.sqrt(2.0))))
            coord_line = f"({x:.6f}, {y:.6f}, -90)\n"
        if y > 0.86:
            critical_alpha = 45.0 - math.degrees(math.acos((-y + 1.0) / (0.1 * math.sqrt(2.0))))
            coord_line = f"({x:.6f}, {y:.6f}, 90)\n"
        # print(f"[waypoints_cruise] critical alpha: {critical_alpha:.2f} degrees", file=sys.stderr)

    if critical_alpha is not None:
        pass
    
    return _atomic_write(DYNAMIC_WAYPOINTS_FILE, coord_line)


def stop() -> bool:
    """Stop by setting dynamic waypoint to current robot position."""
    cur = _read_current_position(CURRENT_POSITION_FILE)
    if cur is None:
        return False
    x, y, bearing = cur
    if bearing is None:
        return goto(x, y)
    return goto(x, y, bearing)


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
        print(f"[waypoints_cruise] no visible balls, miss_time={miss_time}, search_record={search_record}", file=sys.stderr)
        if miss_time == 0.0:
          if abs(cx) > 0.86 or abs(cy) > 0.86:
              cx = max(-0.86, min(0.86, cx))
              cy = max(-0.86, min(0.86, cy))
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
