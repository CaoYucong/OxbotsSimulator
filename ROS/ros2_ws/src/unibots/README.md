# unibots

ROS2 Jazzy Python package providing:

- `web_bridge_node`: HTTP mirror bridge for simulation data + front camera publisher
- `decision_node`: decision node that consumes topics and publishes decisions
- `pose_estimation`: optional pose estimation from front camera

## Notes

- `web_bridge_node` polls upstream endpoints:
	- `http://192.168.50.1:5003/simulation_data`
	- `http://192.168.50.1:5003/data/simulation_data`
- Then serves mirrored data on local endpoints:
	- `http://127.0.0.1:5003/simulation_data`
	- `http://127.0.0.1:5003/data/simulation_data`
- Cache is updated only when upstream content changes.
- Front camera JPEG is fetched from `/front_camera` and published to `/front_camera`.
- Topics published by `web_bridge_node`:
	- `/visible_balls`, `/waypoint_status`, `/time`, `/front_camera`
	- `/current_position` only when `pose_estimation` is disabled
- Topics published by `pose_estimation`:
	- `/current_position` when `pose_estimation` is enabled
- Topics published by `decision_node`:
	- `/decisions`, `/decision_making_data`
