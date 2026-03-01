# unibots_file_bridge

ROS2 Jazzy Python package providing:

- `file_bridge_node`: file-system bridge between simulator and ROS topics
- `decision_node`: decision logic node adapted from `waypoints_cruise.py`

## Notes

- Uses atomic writes (`temp file + rename`) for simulator command files.
- Ignores missing/malformed files safely.
- Assumes radians for all orientation values.
