"""Obstacle avoidance helpers for the motion control node.

Pure Python — no ROS imports. Fully unit-testable in isolation.
"""

from __future__ import annotations

import math

_SENSOR_DEFAULT: float = 0.80  # fail / out-of-range value (meters)
_SENSOR_MIN: float = 0.02
_SENSOR_MAX: float = 0.80


def parse_radar_sensor(text: str) -> dict[str, float]:
    """Parse the /radar_sensor CSV string into a direction dict.

    Expected format: "time,front,right,left,rear" (all distances in meters).
    On any parse error or out-of-range value, the affected direction defaults
    to _SENSOR_DEFAULT (0.80 m) — the sensor fail/max value.

    Returns: {'front': float, 'right': float, 'left': float, 'rear': float}
    """
    result: dict[str, float] = {
        'front': _SENSOR_DEFAULT,
        'right': _SENSOR_DEFAULT,
        'left':  _SENSOR_DEFAULT,
        'rear':  _SENSOR_DEFAULT,
    }
    if not text:
        return result

    parts = text.split(',')
    # Expected: [time, front, right, left, rear]
    keys_in_order = ['front', 'right', 'left', 'rear']
    for i, key in enumerate(keys_in_order):
        idx = i + 1  # skip the leading time field
        if idx >= len(parts):
            break
        try:
            val = float(parts[idx].strip())
        except (TypeError, ValueError):
            continue
        if math.isfinite(val) and _SENSOR_MIN <= val <= _SENSOR_MAX:
            result[key] = val

    return result


def select_target(
    dynamic_wp: tuple[float, float] | None,
    collision_wp: tuple[float, float] | None,
) -> tuple[float, float] | None:
    """Return collision_wp when set, else dynamic_wp.

    Args:
        dynamic_wp:   Parsed (x, y) from /dynamic_waypoint, or None.
        collision_wp: Parsed (x, y) from /collision_avoiding_waypoint, or None.

    Returns:
        The effective (x, y) target, or None if both are absent.
    """
    if collision_wp is not None:
        return collision_wp
    return dynamic_wp


def should_emergency_brake(
    radar: dict[str, float],
    nav_motion_mode: str | None,
    threshold_m: float,
) -> bool:
    """Return True when a front obstacle demands an emergency brake.

    Conditions (both must hold):
      1. nav_motion_mode == 'drive'  (robot is currently moving forward)
      2. front sensor distance < threshold_m  AND  >= 0.02 m
         (the lower bound prevents glitch zero-reads from triggering the brake;
          the 0.80 fail value never satisfies < 0.15, so sensor failures are safe)

    Args:
        radar:          Output of parse_radar_sensor().
        nav_motion_mode: Current nav phase string from motion_control_node.
        threshold_m:    Emergency brake distance threshold (meters).
    """
    if nav_motion_mode != 'drive':
        return False
    front = radar.get('front', _SENSOR_DEFAULT)
    return _SENSOR_MIN <= front < threshold_m
