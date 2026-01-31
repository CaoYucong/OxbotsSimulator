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
from typing import Optional


BASE_DIR = os.path.dirname(__file__)
WAYPOINT_STATUS_FILE = os.path.join(BASE_DIR, "waypoint_status.txt")
DYNAMIC_WAYPOINTS_FILE = os.path.join(BASE_DIR, "dynamic_waypoints.txt")
BALL_POS_FILE = os.path.join(BASE_DIR, "ball_position.txt")
CURRENT_POSITION_FILE = os.path.join(BASE_DIR, "current_position.txt")

# Set the default mode here for convenience. Edit this file and set
# `DEFAULT_MODE` to the mode you want the script to use when no CLI arg
# or `MODE` environment variable is provided. Example: 'random' or 'nearest'.
DEFAULT_MODE = 'nearest'

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
    coord_line = f"({x:.6f}, {y:.6f}, None)\n"

    ok = _atomic_write(waypoint_file, coord_line)
    return 0 if ok else 1


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


def _read_current_position(path: str):
    """Return (x,y) from current_position.txt or None."""
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
            x = float(parts[0]); y = float(parts[1])
            return (x, y)
    except Exception:
        return None


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

    cx, cy = cur
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

    coord_line = f"({best[0]:.6f}, {best[1]:.6f}, None)\n"
    ok = _atomic_write(waypoint_file, coord_line)
    return 0 if ok else 1


# Mode dispatch table: add new handlers here
_MODE_HANDLERS = {
    "random": mode_random,
    "nearest": mode_nearest,
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
