# unibots_bridge

ROS2 Jazzy Python package providing:

- `file_bridge_node`: HTTP mirror bridge for simulation data
- `decision_making_node`: decision making node adapted from `waypoints_cruise.py`

## Notes

- `file_bridge_node` polls upstream endpoints:
	- `http://192.168.50.1:5003/simulation_data`
	- `http://192.168.50.1:5003/data/simulation_data`
- Then serves mirrored data on local endpoints:
	- `http://127.0.0.1:5003/simulation_data`
	- `http://127.0.0.1:5003/data/simulation_data`
- Cache is updated only when upstream content changes.
